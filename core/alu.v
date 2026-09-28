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
    input [13:0] flag_bus,
    input we_in, jal_flag, jalr_flag, cs_wr_en,
    input [4:0] rd_in,
//ROB 索引：与结果同沿寄存（结果晚一拍，索引必须一起晚一拍，否则完成口会回填到错项）
    input [2:0] idx_in,
    input [3:0] alu_func4,
    input [2:0] csr_func3,
    input [31:0] aux_addr_in,
    input [31:0] r1_data, r2_data, cs_data,
    output reg [4:0] rd_out,
    output reg [2:0] idx_out,
//这一笔永不落地（异常冲刷把"错路那条"标掉）：写口不发写、但照常回报，项才能退
    output reg kill_out,
    output reg [31:0] result, result_csr,
    output reg we
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

//flag_bus 逐位翻译成原名：本模块内不起新的组合名，判定处直接写或运算
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    reg stall_rob_full, stall_pc_redir;
    reg stall_lsu_haz, stall_lsu_unload, stall_lsu_full;
    reg stall_mulu_haz, stall_mulu_div;
    reg stall_dcache_miss, stall_icache_miss, stall_bus_hold;
    always @(*) begin
        flush_con_exc     = flag_bus[13];
        flush_con_irq     = flag_bus[12];
        flush_con_jump    = flag_bus[11];
        exec              = flag_bus[10];
        stall_rob_full    = flag_bus[9];
        stall_pc_redir    = flag_bus[8];
        stall_lsu_haz     = flag_bus[7];
        stall_lsu_unload  = flag_bus[6];
        stall_lsu_full    = flag_bus[5];
        stall_mulu_haz    = flag_bus[4];
        stall_mulu_div    = flag_bus[3];
        stall_dcache_miss = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        stall_bus_hold    = flag_bus[0];
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
    end

//结果寄存一拍：写口级看到的口上值就是这一份（算完的下一拍），而紧邻的消费者正好在那拍取用。
//★ 判据是本级的 we（指令站在 post_decoder 和 alu 之间时恒为 1）—— 不能用"载荷推进"当门：
//  重定向排队（等 ROB 排空）会把流水线冻住，冻结期间若把结果寄存器清掉/不寄存，
//  这条指令就永远不回报写口级 ⇒ ROB 排不空 ⇒ 等它排空的重定向永远等下去（自锁，实测四支程序全挂）。
//★ 但【单元侧的 stall】（lsu 的在途 load / 车厢满、mulu 的乘除）抬着的那一拍必须不寄存：
//  那一拍送进来的操作数还是旧的，寄存了下一拍写口级就会把它写进寄存器堆，
//  而写口级在下一拍看不出这个值是哪一拍算出来的 ⇒ 只能在本级挡。
//  单元侧那几条都是单元自己干完活就落、不看 ROB ⇒ 在这儿等它算完不成环。
//  抬着期间本级的 we/rd/idx 一起保持 ⇒ 写口级照上一条重复处置（同值、同广播，幂等），
//  回报也照旧每一拍都发 ⇒ ROB 退得动。
//★ 冲刷拍分两类，必须分开：
//  异常/中断冲刷（位 13/12）：这一拍刚进本级的是【年轻错路那条】（pd 要到边沿才清 we）
//    ⇒ 登记但标记"永不落地"（kill），错路的值不能写进寄存器堆。
//  跳转冲刷（位 11）：发起者自己那一笔（jalr 的 link / 分支）此刻正挂在本级输出上
//    ⇒ **保持**，不许清（清掉就把它抹了，旧 wbu 记的是同一个坑）。
    always @(posedge clk) begin
        if (rst_q) begin
            rd_out   <= 5'd0;
            idx_out  <= 3'd0;
            result   <= 32'd0;
            we       <= 1'b0;
            kill_out <= 1'b0;
        end
        else if (flush_con_jump) begin               // 跳转冲刷：错路那条刚要进本级 ⇒ 保持不寄存
            rd_out   <= rd_out;
            idx_out  <= idx_out;
            result   <= result;
            we       <= we;
            kill_out <= kill_out;
        end
        else if (we_in) begin
            if ((flush_con_exc | flush_con_irq) |
                ~(stall_lsu_haz | stall_lsu_unload | stall_lsu_full | stall_mulu_haz | stall_mulu_div)) begin
                rd_out   <= rd_nx;
                idx_out  <= idx_in;
                result   <= result_nx;
                we       <= we_nx;
                kill_out <= flush_con_exc;
            end
            else begin                               // 单元侧 stall 抬着：操作数还没定，结果连着 rd/idx/we 一起保持
                rd_out   <= rd_out;
                idx_out  <= idx_out;
                result   <= result;
                we       <= we;
                kill_out <= kill_out;
            end
        end
        else begin
            rd_out   <= 5'd0;
            idx_out  <= 3'd0;
            result   <= 32'd0;
            we       <= 1'b0;
            kill_out <= 1'b0;
        end
    end

endmodule
