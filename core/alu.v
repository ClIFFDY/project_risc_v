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
    input [7:0] flag_bus,
    input we_in, jal_flag, jalr_flag, cs_wr_en,
//判定那条自己那一笔的 ROB 索引（bju 的 idx_q）：跳转冲刷拍用它区分"本级挂着的是不是发起者自己"
    input [4:0] rd_in,
//ROB 索引：与结果同沿寄存（结果晚一拍，索引必须一起晚一拍，否则完成口会回填到错项）
    input [3:0] idx_in,
//那一项的世代位：与 idx 全程同行，完成上报时带回 ROB 做身份校验（见 rob.v 文件头）
    input       gen_in,
    input [3:0] alu_func4,
    input [2:0] csr_func3,
    input [31:0] aux_addr_in,
    input [31:0] r1_data, r2_data, cs_data,
    (* max_fanout = 8 *) output reg [4:0] rd_out,
    (* max_fanout = 8 *) output reg [3:0] idx_out,
    output reg gen_out,
    (* max_fanout = 8 *) output reg [31:0] result, result_csr,
//本级组合结果（= 驱动 result 寄存器的 result_nx，不加逻辑）：给"同 lane 背靠背"用。
//★ 它是一条**独立数据路径**（发射判据 → 本口 → 消费者的发射判据），接上之前必须单列时序。
    (* max_fanout = 8 *) output reg [31:0] result_now,
    (* max_fanout = 8 *) output reg we
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
    reg [4:0]  rd_nx;
    reg [31:0] result_nx;
    reg        we_nx;

//flag_bus 逐位翻译：本级只吃三条冲刷位 —— 数据相关由发射级（idu2 的 Q）原地等，
//结构相关由各单元自己的 ready 挡，这两类都不进本级。
    reg flush_con_exc, flush_con_irq, flush_con_jump;
    always @(*) begin
        flush_con_exc     = flag_bus[7];
        flush_con_irq     = flag_bus[6];
        flush_con_jump    = flag_bus[5];
    end

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//SYSTEM类指令：cs_data源于csr寄存器，result_csr写回csr寄存器，result返回rd
    always @(*) begin
        if (we_in && cs_wr_en) begin
            rd_nx = rd_in;
            we_nx = we_in;
            result_nx = cs_data;
            case (csr_func3)
            CSRRW, CSRRWI: result_csr = r1_data;
            CSRRS, CSRRSI: result_csr = r1_data | cs_data;
            CSRRC, CSRSCI: result_csr = cs_data & ~r1_data;
            default: result_csr = cs_data;
            endcase
        end
//链接跳转类指令：将pc作为result输出存入rd
        else if (we_in && (jal_flag | jalr_flag)) begin
            result_nx = aux_addr_in;
            rd_nx = rd_in;
            we_nx = we_in;
            result_csr = r2_data;
        end
//ALU/I类指令：直接进行运算，result存入rd
        else if (we_in) begin
            case (alu_func4)
                ADD: result_nx = r1_data + r2_data;
                SLL: result_nx = r1_data << r2_data[4:0];
                SLT: result_nx = ($signed(r1_data) < $signed(r2_data)) ? 32'd1 : 32'd0;
                SLTU: result_nx = (r1_data < r2_data) ? 32'd1 : 32'd0;
                XOR: result_nx = r1_data ^ r2_data;
                SRL: result_nx = r1_data >> r2_data[4:0];
                OR: result_nx = r1_data | r2_data;
                AND: result_nx = r1_data & r2_data;
                SUB: result_nx = r1_data - r2_data;
                SRA: result_nx = $signed(r1_data) >>> r2_data[4:0];
                default: result_nx = 32'd0;
            endcase
            rd_nx = rd_in;
            we_nx = we_in;
            result_csr = r2_data;
        end
        else begin
            rd_nx = 5'd0;
            result_nx = 32'd0;
            we_nx = 1'b0;
            result_csr = r2_data;
        end
//组合出口：就是驱动 result 寄存器的那个值，不加逻辑
        result_now = result_nx;
    end

//结果寄存一拍：写口级看到的口上值就是这一份（算完的下一拍）。
//★ 登记判据就是本级的 we_in，不许再叠任何"单元侧 stall"门：被抬着那一拍送进来的操作数
//  还是旧的，寄存了下一拍就会把它当成这条的结果写进寄存器堆。
//★ 跳转冲刷拍必须【保持】而不是清：发起者自己那一笔（jalr 的 link / 分支）此刻正挂在
//  本级输出上。异常/中断冲刷拍 issue_v 已经是 0（we_in=0）⇒ 自然走"清"那一档。
//★ 本级的操作数由发射级的 Q 寄存器直驱（整拍不变），不接受任何"停顿时冻住载荷"的写法。
    always @(posedge clk) begin
        if (rst_q) begin
            rd_out   <= 5'd0;
            idx_out  <= 3'd0;
            gen_out  <= 1'b0;
            result   <= 32'd0;
            we       <= 1'b0;
        end
        else if (flush_con_jump) begin
            rd_out   <= rd_out;
            idx_out  <= idx_out;
            gen_out  <= gen_out;
            result   <= result;
            we       <= we;
        end
        else if (we_in) begin
//收结果无条件：站式之后没有"单元侧 stall"这一档，多列一项就会把刚发出那条的结果丢掉
//（保持的是上一笔的 rd/idx/we）⇒ 它那一项永远等不到完成口。
            rd_out   <= rd_nx;
            idx_out  <= idx_in;
            gen_out  <= gen_in;
            result   <= result_nx;
            we       <= we_nx;
        end
        else begin
            rd_out   <= 5'd0;
            idx_out  <= 3'd0;
            gen_out  <= 1'b0;
            result   <= 32'd0;
            we       <= 1'b0;
        end
    end

endmodule
