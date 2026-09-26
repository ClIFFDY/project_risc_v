`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: wbu
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module wbu(
    input clk, rst,
    input [9:0] flag_bus,
//alu 路：s4 的组合结果，与本级原有的写回沿一致 ⇒ 受本级的 stall 门控
    input we_alu,
    input [4:0] rd_alu,
    input [31:0] result_alu,
    input exc_kill,
//mul 路 / ld 路：写沿由各自单元决定（与流水线偏移不绑定）⇒ 【不受本级 stall 门控】，直通
    input we_mul,
    input [4:0] rd_mul,
    input [31:0] result_mul,
    input we_ld,
    input [4:0] rd_ld,
    input [31:0] result_ld,
//两个物理写口的请求（仲裁结果，组合输出；写序由消费端 regfile 的语句顺序保证 A 在后）
    output reg we_a, we_b,
    output reg [4:0] rd_a, rd_b,
    output reg [31:0] data_a, data_b,
//前递源（back2）
    output reg [4:0] rd_back2,
    output reg [31:0] result_back2
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus = {exc, exec, flush_irq, flush_jump, dcache_hold, bus_hold_in, stall_m, stall_v, lsu_stall, icache_busy}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w, flush_jump_w;
    always @(*) begin
        flush_jump_w = flag_bus[6] | flag_bus[9];
        flush_w      = flag_bus[9] | flag_bus[7] | flag_bus[6];
        stall_w      = (flag_bus[5] | flag_bus[4] | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec         = flag_bus[8];
    end

//alu 路在本级寄存一拍（= 原来的 wb_reg 输出）；mul/ld 两路直通（各自自带输出保持）
    reg we_alu_q;
    reg [4:0] rd_alu_q;
    reg [31:0] result_alu_q;

//写回级缓冲寄存器
//跳转冲刷【只挂 flush_jump，绝不能带上 flush_irq】：判定搬去 decoder 的下一拍后，
//本级的输入在冲刷拍是错路的 B+1（原来那拍 decoder 已经清零了，现在还没），不capture 就漏写。
//但中断/mret 类冲刷的相位没变，那时本级里的正是 trap/mret 自己 —— 一起门掉就会把它的
//写回（含 jalr 的 link）抹掉。所以两条冲刷位必须分开处理：一条门本级，一条不门。
    always @(posedge clk) begin
        if (rst_q) begin
            we_alu_q <= 1'b0;
            rd_alu_q <= 5'd0;
            rd_back2 <= 5'd0;
            result_alu_q <= 32'd0;
            result_back2 <= 32'd0;
        end
        else if (exec) begin
//STALL 期间只保持数据，【写使能拉低】：
//若跟着保持，被冻住的这条 ALU 指令会每拍往 regfile 重复写一次，把期间更新的
//load/mulu 结果又盖回去（CoreMark rv32im 里 divu 冻住 wbu 34 拍、后面 sb 读到陈旧值）。
//写使能一拍已在进入本级那拍完成，故此处清零不会丢写。
            if (flush_jump_w) begin
                we_alu_q <= 1'b0;
                rd_alu_q <= 5'd0;
                rd_back2 <= 5'd0;
                result_alu_q <= 32'd0;
                result_back2 <= 32'd0;
            end
            else if (stall_w) begin
                we_alu_q <= 1'b0;
                rd_alu_q <= rd_alu_q;
                rd_back2 <= rd_back2;
                result_alu_q <= result_alu_q;
                result_back2 <= result_back2;
            end
            else begin
                we_alu_q <= we_alu;
                rd_alu_q <= rd_alu;
                rd_back2 <= rd_alu;
                result_alu_q <= result_alu;
                result_back2 <= result_alu;
            end
        end
        else begin
            we_alu_q <= 1'b0;
            rd_alu_q <= 5'd0;
            rd_back2 <= 5'd0;
            result_alu_q <= 32'd0;
            result_back2 <= 32'd0;
        end
    end

//两个物理写口的仲裁（原在 regfile.v 里，随写回源一起搬进来）：口 A = alu | mul，
//口 B = ld。优先级 alu > mul 与"老的一笔先落地"无关，它只是历史上口 A 的合并依据
//（alu 与 mul 的写沿严格同偏移、永不同拍）；ld 独占口 B 是因为它的写沿由总线事务长度定。
//★ mul/ld 两路【不经过上面那个寄存器、也不吃 stall_w】：它们的写沿与流水线偏移不绑定，
//  跟着本文级冻结就会丢写（今天就是直连 regfile 的，这里只把仲裁点搬过来）。
    always @(*) begin
        if (we_alu_q && rd_alu_q != 5'd0 && ~exc_kill) begin
            we_a = 1'b1;
            rd_a = rd_alu_q;
            data_a = result_alu_q;
        end
        else if (we_mul && rd_mul != 5'd0) begin
            we_a = 1'b1;
            rd_a = rd_mul;
            data_a = result_mul;
        end
        else begin
            we_a = 1'b0;
            rd_a = 5'd0;
            data_a = 32'd0;
        end
        we_b = we_ld && (rd_ld != 5'd0);
        rd_b = rd_ld;
        data_b = result_ld;
    end
endmodule
