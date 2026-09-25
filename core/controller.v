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
    input clk, rst, jalr_fail, br2, br3, irq_ret, trap, exc_bju,
//异常判据源：E4（post_decoder 的 ecall/ebreak/非法指令、lsu 的访存非对齐 + 出错地址 + 地址载荷）
//与 E5（bju 的指令地址非对齐 + 它的落点寄存器兼任 mtval）。
    input ebreak, illegal_e4, exc_ldst_misalign, exc_ldst_st,
    input [31:0] exc_ldst_addr, aux_addr_3,
    input [31:0] exc_pc_e5, jp_target,
//d 类停顿的四个源（按"顶层不运算"从 cpu_top 下放至此，本模块内合成 stall_d）
    input lsu_stall, stall_m, stall_v, bus_hold_in, dcache_hold,
    input icache_busy, lsu_inflight,
    input jal, pre_jalr, btb_hit, br1,
    input csr_wr_en, exti, timi, softi,
    input [11:0] csr_addr,
//csr 读口地址（c2 级），比 csr_addr 早一拍；读值寄存一拍后由 csr_data_out 给出
    input [11:0] csr_addr_pre,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
    output reg [31:0] csr_data_out, isr_addr2, mcause,
    output reg irq_act, irq_processing, irq,
    output reg [31:0] iret_addr2,
    output reg [9:0] flag_bus,
    output reg [3:0] irq_bubble
    );

    localparam CAUSE_MISALIGN_INST  = 4'd0;
    localparam CAUSE_ILLEGAL_INST   = 4'd2;
    localparam CAUSE_LOAD_MISALIGN  = 4'd4;
    localparam CAUSE_STORE_MISALIGN = 4'd6;
    localparam CAUSE_BREAKPOINT     = 4'd3;
    localparam CAUSE_ECALL_M        = 4'd11;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    reg [1:0] ird_tmr;
    reg jalr_pred;
    reg flush_older, exc_e4_gated;
    reg [3:0] exc_cause;
    reg [31:0] exc_pc, exc_tval, exc_pc_c;
    wire [31:0] csr_data_out_i, isr_addr2_i, mcause_i;
    wire irq_act_i, irq_processing_i;
    wire [31:0] iret_addr2_i;

//流水线控制位：三条冲刷/使能位自成本模块逻辑；停顿位按【逐源】原样进 flag_bus，
//由各消费端自己取位做或（本模块不再合成统一的 stall）
    reg exec, flush_irq, flush_jump, exc;
    reg flush_w;
    always @(*) flush_w = flush_irq | flush_jump | exc;

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）：pre_jalr 是 pre_decoder 的译码输出，
//btb_hit 是 bra_predict 的命中输出，两者都是寄存器输出，此处相与不成环。
    always @(*) jalr_pred = pre_jalr & btb_hit;

//各控制/停顿位各自成网：位与位之间不共享逻辑，综合时互不依赖
//前置冲刷 stallf 不进 flag_bus：它是 bju 判定块里的组合派生信号，绕 controller 一圈
//只是把同一根线进出一次，实测会让它挂上全片广播网、多花 0.45ns（见 path_cl10 vs path_jp10）。
    always @(*) begin
        flush_irq = irq || irq_ret || irq_act;
        flush_jump = jalr_fail || br2 || br3;
        flag_bus = {exc, exec, flush_irq, flush_jump, dcache_hold, bus_hold_in, stall_m, stall_v, lsu_stall, icache_busy};
//复位期按原 stage = EXE / 无停顿 的口径给：只有 exec 抬、其余落下
        if (rst_q) begin
            flush_irq = 1'b0;
            flush_jump = 1'b0;
            flag_bus = 10'b0100000000;
        end
    end



//异常仲裁（原 core/trap_unit.v 并入本模块 —— 它本来就是"第三类冲刷源"的产生处，与
//flush_irq / flush_jump 并列；并进来之后 exc / exc_cause / exc_pc / exc_tval 不必再穿顶层，
//csr 就在本模块里）。优先级 E5 > E4：同拍多源时程序序最老者胜；E4 那一组要被更老的冲刷挡掉。
    always @(*) flush_older = flag_bus[7] | flag_bus[6];

    always @(*) exc_e4_gated = (trap | illegal_e4 | exc_ldst_misalign) & ~flush_older;

    always @(*) begin
        exc       = exc_bju | exc_e4_gated;
        exc_cause = 4'd0;
        exc_pc_c  = 32'd0;
        exc_tval  = 32'd0;
        if (exc_bju) begin
            exc_cause = CAUSE_MISALIGN_INST;
            exc_pc_c  = exc_pc_e5;
//规范口径：指令地址非对齐的 mtval = 出错的【目标地址】。三路都从 bju 的落点寄存器取：
//br 的目标 / jalr 的目标本来就在那个寄存器里；jal 的目标不在流水里，由 post_decoder
//就地解 immJ 算好后单铺一路载荷送进来，同样落在这个寄存器。
            exc_tval  = jp_target;
        end
        else if (exc_e4_gated) begin
            if (illegal_e4) begin
                exc_cause = CAUSE_ILLEGAL_INST;
            end
            else if (exc_ldst_misalign) begin
                if (exc_ldst_st) begin
                    exc_cause = CAUSE_STORE_MISALIGN;
                end
                else begin
                    exc_cause = CAUSE_LOAD_MISALIGN;
                end
//规范口径：访存地址非对齐的 mtval = 出错的【地址】（不是指令字）
                exc_tval  = exc_ldst_addr;
            end
            else begin
                exc_cause = ebreak ? CAUSE_BREAKPOINT : CAUSE_ECALL_M;
            end
            exc_pc_c  = aux_addr_3;
        end
    end

    always @(*) exc_pc = exc_pc_c - 32'd4;

//csr异常/中断寄存器
    csr u_csr (
        .mem_inflight(lsu_inflight),
        .clk(clk),
        .rst(rst_q),
        .csr_wr_en(csr_wr_en),
        .flag_bus(flag_bus),
        .iret(irq_ret),
        .exti(exti),
        .timi(timi),
        .softi(softi),
        .trap(trap),
        .exc(exc),
        .exc_bju(exc_bju),
        .exc_cause(exc_cause),
        .exc_pc(exc_pc),
        .exc_tval(exc_tval),
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
        .isr_addr2(isr_addr2_i),
        .mcause(mcause_i),
        .irq_act(irq_act_i),
        .irq_processing(irq_processing_i),
        .iret_addr2(iret_addr2_i)
    );

    always @(*) begin
        csr_data_out = csr_data_out_i;
        isr_addr2 = isr_addr2_i;
        mcause = mcause_i;
        irq_act = irq_act_i;
        irq_processing = irq_processing_i;
        irq = irq_act_i;
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
