`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: alu
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
//   Revision 0.02 - 结果寄存一拍（乱序写回改造第 2 步）：写口/前送的口上值就是这一份。
//                    寄存时机 = 载荷推进那一拍（不能在冻结拍寄存，见下面注释）。
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module alu(
    input clk, rst,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input [2:0] flush_control_in,   // = flag_bus[7:5]：{异常, 中断, 改向}
    input write_enable_in, jal_flag, jalr_flag, cs_wr_en,
//判定那条自己那一笔的 ROB 索引（bju 的 idx_q）：跳转冲刷拍用它区分"本级挂着的是不是发起者自己"
    input [4:0] destination_in,
//ROB 索引：与结果同沿寄存（结果晚一拍，索引必须一起晚一拍，否则完成口会回填到错项）
    input [3:0] index_in,
//那一项的世代位：与 idx 全程同行，完成上报时带回 ROB 做身份校验（见 rob.v 文件头）
    input       generation_in,
    input [3:0] alu_function4,
    input [2:0] csr_function3,
    input [31:0] auxiliary_address_in,
    input [31:0] source_a_data, r2_data, cs_data,
//★ 2026-10-30：load 晚到数据。载入那条在【地址进 dcache 那一拍】就宣告"下一拍有数"（~miss 才宣告），
//  消费者因此能提前一拍发射；数据在下一拍才到，用这一档直接在 ALU 输入上顶进去。
//  ⇒ 两条门：bypass_load_* 是发射拍随操作数一起锁下的选择位；load_data_forward 是下一拍的数据。
    input bypass_load_a, bypass_load_b,
    input [31:0] load_data_forward,
    (* max_fanout = 8 *) output reg [4:0] destination_out,
    (* max_fanout = 8 *) output reg [3:0] index_out,
    output reg generation_out,
    (* max_fanout = 8 *) output reg [31:0] result, result_csr,
//本级组合结果（= 驱动 result 寄存器的 result_next，不加逻辑）：给"同 lane 背靠背"用。
//★ 它是一条**独立数据路径**（发射判据 → 本口 → 消费者的发射判据），接上之前必须单列时序。
    (* max_fanout = 8 *) output reg [31:0] result_now,
//结果口的【D 端】：下一沿 result 会装载的那个值（含跳转冲刷时的保持档）。
//前送表用它写表 ⇒ 表项与 result 同拍对齐；消费者只读表，不从本口拉 32 位数据线。
    (* max_fanout = 8 *) output reg [31:0] result_early,
//身份的 D 端（下一沿 write_enable/index_out/generation_out 会装载的值）：早唤醒用
    output reg write_enable_early,
    output reg [3:0] index_early,
    output reg generation_early,
    (* max_fanout = 8 *) output reg write_enable
    );

    localparam [3:0]
        ADD  = 4'b0000,
        SLL  = 4'b0001,
        SLT  = 4'b0010,
        SLTU = 4'b0011,
        XOR  = 4'b0100,
        SRL  = 4'b0101,
        OR   = 4'b0110,
        AND  = 4'b0111,
        SUB  = 4'b1000,
        SRA  = 4'b1101;

    localparam [2:0]
        CSRRW  = 3'b001,
        CSRRS  = 3'b010,
        CSRRC  = 3'b011,
        CSRRWI = 3'b101,
        CSRRSI = 3'b110,
        CSRSCI = 3'b111;

//本级组合算出的"待寄存结果"
    reg [4:0]  destination_next;
    reg [31:0] result_next;
    reg        write_enable_next;

//flag_bus 逐位翻译：本级只吃三条冲刷位 —— 数据相关由发射级（idu2 的 Q）原地等，
//结构相关由各单元自己的 ready 挡，这两类都不进本级。
    reg flush_control_exception, flush_control_interrupt, flush_control_jump;
    always @(*) begin
        flush_control_exception     = flush_control_in[2];
        flush_control_interrupt     = flush_control_in[1];
        flush_control_jump    = flush_control_in[0];
    end

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//---- 操作数级（2026-10-30）----
//★ ALU 的组合锥必须从【寄存器】出发。原来它直接挂在 idu2 的操作数选择锥上
//  （那条锥的起点是值表的 16:1 读），split-C 实测 12.7 ns，打不进 12.5 ns。
//★ 所有输入一起锁，不能只锁 a/b：控制位不锁的话，下一拍的控制已经属于下一条指令。
//★ 冲刷拍这一级【保持】—— 发起者自己那一笔（jalr 的 link / 分支）此刻正挂在输出上；
//  保持住 ⇒ result_next 不变 ⇒ 值表那一笔写是幂等的，原来的 D 端冲刷 mux 因此删掉。
//★ 前送是组合出去的（result_early / write_enable_early / index_early / generation_early），
//  但它只喂消费者的【操作数寄存器】，不回本级的组合锥。
    reg        op_write_enable, op_jal, op_jalr, op_cs_wr;
    reg        op_bypass_load_a, op_bypass_load_b;
    reg [4:0]  op_destination;
    reg [3:0]  op_index;
    reg        op_generation;
    reg [3:0]  op_alu_function4;
    reg [2:0]  op_csr_function3;
    reg [31:0] op_auxiliary_address, op_source_a, op_r2, op_cs;

    always @(posedge clk) begin
        if (reset_registered) begin
            op_write_enable <= 1'b0; op_jal <= 1'b0; op_jalr <= 1'b0; op_cs_wr <= 1'b0;
            op_bypass_load_a <= 1'b0; op_bypass_load_b <= 1'b0;
            op_destination <= 5'd0;  op_index <= 4'd0; op_generation <= 1'b0;
            op_alu_function4 <= 4'd0; op_csr_function3 <= 3'd0;
            op_auxiliary_address <= 32'd0; op_source_a <= 32'd0; op_r2 <= 32'd0; op_cs <= 32'd0;
        end
        else if (flush_control_jump) begin
            op_write_enable <= op_write_enable; op_jal <= op_jal; op_jalr <= op_jalr; op_cs_wr <= op_cs_wr;
            op_bypass_load_a <= op_bypass_load_a; op_bypass_load_b <= op_bypass_load_b;
            op_destination <= op_destination;  op_index <= op_index; op_generation <= op_generation;
            op_alu_function4 <= op_alu_function4; op_csr_function3 <= op_csr_function3;
            op_auxiliary_address <= op_auxiliary_address; op_source_a <= op_source_a; op_r2 <= op_r2; op_cs <= op_cs;
        end
        else begin
            op_write_enable <= write_enable_in; op_jal <= jal_flag; op_jalr <= jalr_flag; op_cs_wr <= cs_wr_en;
            op_bypass_load_a <= bypass_load_a; op_bypass_load_b <= bypass_load_b;
            op_destination <= destination_in;  op_index <= index_in; op_generation <= generation_in;
            op_alu_function4 <= alu_function4; op_csr_function3 <= csr_function3;
            op_auxiliary_address <= auxiliary_address_in; op_source_a <= source_a_data; op_r2 <= r2_data; op_cs <= cs_data;
        end
    end

//SYSTEM类指令写 csr 的那份数据：★ 必须走【不锁的】输入 —— csr 的写口在发射那一拍落地，
//相位一动就写到下一条头上去了。（写 rd 的那份在下面结果级里用锁后值。）
    always @(*) begin
        if (write_enable_in && cs_wr_en) begin
            case (csr_function3)
            CSRRW, CSRRWI: result_csr = source_a_data;
            CSRRS, CSRRSI: result_csr = source_a_data | cs_data;
            CSRRC, CSRSCI: result_csr = cs_data & ~source_a_data;
            default: result_csr = cs_data;
            endcase
        end
        else
            result_csr = r2_data;
    end

//---- 结果级（组合，锥只从操作数寄存器出发）----
//★ 唯一例外：load 晚到那一档。它不参与唤醒（唤醒在发射拍就做了），只是把数据在 ALU 输入上顶进去。
    wire [31:0] alu_source_a = op_bypass_load_a ? load_data_forward : op_source_a;
    wire [31:0] alu_source_b = op_bypass_load_b ? load_data_forward : op_r2;

    always @(*) begin
        if (op_write_enable && op_cs_wr) begin
            destination_next  = op_destination;
            write_enable_next = op_write_enable;
            result_next       = op_cs;
        end
//链接跳转类指令：将 pc 作为 result 输出存入 rd
        else if (op_write_enable && (op_jal | op_jalr)) begin
            result_next       = op_auxiliary_address;
            destination_next  = op_destination;
            write_enable_next = op_write_enable;
        end
//ALU/I 类指令：直接进行运算，result 存入 rd
        else if (op_write_enable) begin
            case (op_alu_function4)
                ADD: result_next = alu_source_a + alu_source_b;
                SLL: result_next = alu_source_a << alu_source_b[4:0];
                SLT: result_next = ($signed(alu_source_a) < $signed(alu_source_b)) ? 32'd1 : 32'd0;
                SLTU: result_next = (alu_source_a < alu_source_b) ? 32'd1 : 32'd0;
                XOR: result_next = alu_source_a ^ alu_source_b;
                SRL: result_next = alu_source_a >> alu_source_b[4:0];
                OR: result_next = alu_source_a | alu_source_b;
                AND: result_next = alu_source_a & alu_source_b;
                SUB: result_next = alu_source_a - alu_source_b;
                SRA: result_next = $signed(alu_source_a) >>> alu_source_b[4:0];
                default: result_next = 32'd0;
            endcase
            destination_next  = op_destination;
            write_enable_next = op_write_enable;
        end
        else begin
            destination_next  = 5'd0;
            result_next       = 32'd0;
            write_enable_next = 1'b0;
        end
//组合出口：就是驱动 result 寄存器的那个值，不加逻辑（给"同 lane 背靠背"用）
        result_now = result_next;
//前送口：冲刷由操作数级保持接住，这里不再需要 mux
        result_early       = result_next;
        write_enable_early = op_write_enable;
        index_early        = op_index;
        generation_early   = op_generation;
    end

//结果寄存一拍：ROB 完成口看到的口上值就是这一份（比前送晚一拍）。
//★ 登记判据是操作数级的 write_enable，不许再叠任何"单元侧 stall"门：被抬着那一拍送进来的
//  操作数还是旧的，寄存了下一拍就会把它当成这条的结果写进寄存器堆。
//★ 跳转冲刷拍【保持】：发起者自己那一笔此刻正挂在输出上。
    always @(posedge clk) begin
        if (reset_registered) begin
            destination_out   <= 5'd0;
            index_out  <= 4'd0;
            generation_out  <= 1'b0;
            result   <= 32'd0;
            write_enable       <= 1'b0;
        end
        else if (flush_control_jump) begin
            destination_out   <= destination_out;
            index_out  <= index_out;
            generation_out  <= generation_out;
            result   <= result;
            write_enable       <= write_enable;
        end
        else if (op_write_enable) begin
//收结果无条件：站式之后没有"单元侧 stall"这一档，多列一项就会把刚发出那条的结果丢掉
//（保持的是上一笔的 rd/idx/write_enable）⇒ 它那一项永远等不到完成口。
            destination_out   <= destination_next;
            index_out  <= op_index;
            generation_out  <= op_generation;
            result   <= result_next;
            write_enable       <= write_enable_next;
        end
        else begin
            destination_out   <= 5'd0;
            index_out  <= 4'd0;
            generation_out  <= 1'b0;
            result   <= 32'd0;
            write_enable       <= 1'b0;
        end
    end

endmodule
