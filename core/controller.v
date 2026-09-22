`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 19:04:53
// Design Name:
// Module Name: controller
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


module controller(
    input clk, rst, jalr_fail, br2, br3, irq_ret, trap, ebreak,
//d 类停顿的四个源（按"顶层不运算"从 cpu_top 下放至此，本模块内合成 stall_d）
    input lsu_stall, mul_stall, bus_hold_in, dcache_hold,
    input icache_busy,
    input jal, pre_jalr, btb_hit, br1,
    input csr_wr_en, exti, timi, softi,
    input [11:0] csr_addr,
//csr 读口地址（c2 级），比 csr_addr 早一拍；读值寄存一拍后由 csr_data_out 给出
    input [11:0] csr_addr_pre,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
    output reg [31:0] csr_data_out, isr_addr1, isr_addr2, mcause,
    output reg irq_act, irq_processing, irq,
    output reg [31:0] iret_addr1, iret_addr2,
    output reg [4:0] flag_bus,
    output reg req_valid,
    output reg [3:0] irq_bubble
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    reg [1:0] ird_tmr;
    reg jalr_pred;
    wire [31:0] csr_data_out_i, isr_addr1_i, isr_addr2_i, mcause_i;
    wire irq_act_i, irq_processing_i;
    wire [31:0] iret_addr1_i, iret_addr2_i;

//流水线控制位：五条 1bit，收束进 flag_bus，对外不再有统一的 stage / stall
    reg exec, flush_irq, flush_jump, stall_d, stall_i;
    reg flush_w;
    always @(*) flush_w = flush_irq | flush_jump;

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）：pre_jalr 是 pre_decoder 的译码输出，
//btb_hit 是 bra_predict 的命中输出，两者都是寄存器输出，此处相与不成环。
    always @(*) jalr_pred = pre_jalr & btb_hit;

//五条控制位各自成网：位与位之间不共享逻辑，综合时互不依赖
//前置冲刷 stallf 不进 flag_bus：它是 bju 判定块里的组合派生信号，绕 controller 一圈
//只是把同一根线进出一次，实测会让它挂上全片广播网、多花 0.45ns（见 path_cl10 vs path_jp10）。
    always @(*) begin
        flush_irq = irq || irq_ret || irq_act || trap;
        flush_jump = jalr_fail || br2 || br3;
        stall_d = lsu_stall | mul_stall | bus_hold_in | dcache_hold;
        stall_i = icache_busy;
        req_valid = !(stall_d || stall_i);
        flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i};
//复位期按原 stage = EXE / req_valid = 0 的口径给：只有 exec 抬、其余落下
        if (rst_q) begin
            flush_irq = 1'b0;
            flush_jump = 1'b0;
            stall_d = 1'b0;
            stall_i = 1'b0;
            req_valid = 1'b0;
            flag_bus = 5'b10000;
        end
    end

//csr异常/中断寄存器
    csr u_csr (
        .clk(clk),
        .rst(rst_q),
        .csr_wr_en(csr_wr_en),
        .stall_d(stall_d),
        .stall_i(stall_i),
        .flush(flush_w),
        .iret(irq_ret),
        .exti(exti),
        .timi(timi),
        .softi(softi),
        .trap(trap),
        .ebreak(ebreak),
        .csr_addr(csr_addr),
        .csr_addr_pre(csr_addr_pre),
        .csr_data_in(csr_data_in),
        .pc_addr_in(pc_addr_in),
        .irq_bubble(irq_bubble),
        .ird_tmr(ird_tmr),
        .jalr_fail(jalr_fail),
        .br2(br2),
        .br3(br3),
        .csr_data_out(csr_data_out_i),
        .isr_addr1(isr_addr1_i),
        .isr_addr2(isr_addr2_i),
        .mcause(mcause_i),
        .irq_act(irq_act_i),
        .irq_processing(irq_processing_i),
        .iret_addr1(iret_addr1_i),
        .iret_addr2(iret_addr2_i)
    );

    always @(*) begin
        csr_data_out = csr_data_out_i;
        isr_addr1 = isr_addr1_i;
        isr_addr2 = isr_addr2_i;
        mcause = mcause_i;
        irq_act = irq_act_i;
        irq_processing = irq_processing_i;
        irq = irq_act_i;
        iret_addr1 = iret_addr1_i;
        iret_addr2 = iret_addr2_i;
    end

//中断空窗计数器：作用为填充冲刷后流水线预取空窗
    always @(posedge clk) begin
        if (rst_q) irq_bubble <= 4'd12;
        else if (flush_w) irq_bubble <= 4'd4;
        else if (irq_bubble < 4'd12) irq_bubble <= irq_bubble + 4'd4;
    end

    always @(posedge clk) begin
        if (rst_q) ird_tmr <= 2'd0;
        else if ((jal | jalr_pred | br1) && !flush_w) ird_tmr <= 2'd3;
        else if (ird_tmr != 2'd0) ird_tmr <= ird_tmr - 2'd1;
    end

    always @(posedge clk) begin
        if (rst_q) exec <= 1'b1;
        else exec <= exec;
    end

endmodule
