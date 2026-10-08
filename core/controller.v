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
    input clk, rst, jalr_fail, br2, br3, exc_irq_ret, exc_ecall, flush_bju_exc,
//异常仲裁源（原 trap_unit 并进本模块：它没有自己的流水级，按"模块按级划分"不该单独成文件）：
//★【过 bju】ecall/ebreak/illegal/mret 四路信号不再直接吃 decoder 载荷，而是跟操作数/控制位
//  一起进 bju 的【判定输入寄存级】、和 flush_bju_pre 同一拍出来（见 bju.v）。
//  这样它们天生比载荷晚一拍，正好与【更老那条的寄存版冲刷】同拍 —— 本拍这边的
//  flag_bus[9]/flush_bju_exc 描述的正是比它更老的那条 ⇒ 现成的 ~flush_older 就盖住了影子槽，
//  本模块因此不再需要组合的 flush_bju_pre（它只剩 lsu 一个消费者）。
//bju 名下那几种（分支/jalr/jal 非对齐）用【寄存版】：判定搬后一级之后，只有寄存版
//与冲刷拍（flush_bju_exc）同拍有效；组合版在冲刷拍已经翻到下一条 ⇒ 载荷/mtval 全错位。
    input [31:0] bju_pc_in,
    input [31:0] bju_tgt_in,
//判定输入拍采到的"更老指令正在冲刷"（bju 的 older_q）：门标记要用它，不能用当拍的 flush_older
    input bju_older_in,
    input exc_ecall_i, exc_ebreak_i, exc_illegal_i, exc_irq_ret_i,
    input [31:0] exc_pc_i,
    input [2:0]  bju_idx_i,
    input exc_ldst_misalign_in, exc_ldst_st_in,
    input [31:0] exc_ldst_addr_in,
    input [31:0] exc_pc_in,
//三条冲刷各自的 ROB 边界：故障项自己的索引 / E4 载荷（中断边界）/ bju 判定那条自己的索引
    input [2:0]  exc_ldst_idx_in,
    input [2:0]  issue_idx_in,
    input [2:0]  bju_idx_in,
//停顿源（按"顶层不运算"从 cpu_top 下放至此）：逐源各占 flag_bus 一位，
//本模块【不做任何或运算】—— 或/非一律下放到消费者模块内（见各家 stall_w/flush_w 的译码）。
    input stall_lsu_haz, stall_lsu_full,
    input stall_mulu_haz, stall_mulu_div,
    input stall_icache_miss, stall_bus_hold,
    input stall_rob_full, stall_pc_redir,
    input lsu_inflight,
//异常交付口（ROB 的 trap_fire + 载荷）：透传给 csr，由它在【队头那一拍】锁 mepc/mcause/mtval
    input rob_trap,
    input [3:0]  rob_trap_cause,
    input [31:0] rob_trap_pc, rob_trap_tval,
//ROB 分支冲刷脉冲：误预测那条在 wb 拍，它的索引由 cpu_top 用载荷传（顶层只连线）
    output reg flush_con_rob,
    output reg [2:0] flush_idx,
//写序号（乱序写回用，逻辑就放在本模块内）：发射级要不要发号 + 三个单元的滞留兜底请求
    input jal, pre_jalr, btb_hit, br1,
    input csr_wr_en, exti, timi, softi,
//lane1 压着更老那条：中断受理点要让开它
    input lane1_hold,
    input [11:0] csr_addr,
//csr 读口地址（c2 级），比 csr_addr 早一拍；读值寄存一拍后由 csr_data_out 给出
    input [11:0] csr_addr_pre,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
//取指队列头在册（透传给 csr 的中断受理门）
    input        fifo_h0_v,
//异常检测拍的载荷（cause/pc/tval）不再进 csr：csr 只在 ROB 交付那一拍锁上下文，
//载荷由 ROB 按"队头那条故障指令自己那份"给出（见 csr 的 exc_retire 口）
    output reg [31:0] csr_data_out, isr_addr2, mcause,
    output reg exc_irq_act, exc_irq_processing, exc_irq,
    output reg [31:0] iret_addr2,
    output reg [11:0] flag_bus,
//异常仲裁结果：flush_con_exc = 冲刷请求（寄存版判据）；exc_mark + cause/pc/tval = 给 ROB 的当拍标记
    output reg flush_con_exc,
    output reg exc_mark,
    output reg [3:0]  exc_cause,
    output reg [31:0] exc_pc, exc_tval,
//ROB 标记口的索引（三路 mux 原来在 cpu_top，按"顶层零逻辑"搬进来）；三路源各自带正确相位
    output reg [2:0]  exc_idx,
//mret 的资格版：同一根线既喂 pc 的排队武装、又喂 csr 的使能恢复（两者是同一次 mret 生效的两半）
    output reg exc_irq_ret_ok
    );

//异常 cause 号（规范）
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
    wire [31:0] csr_data_out_i, isr_addr2_i, mcause_i;
    wire exc_irq_act_i, exc_irq_processing_i;
    wire [31:0] iret_addr2_i;

//流水线控制位：本模块只产自己那三条冲刷源与 exec；八条停顿源逐源过路进 flag_bus，
//本模块内不做任何或运算（或/非是消费者模块的事）。
    reg exec, flush_con_irq, flush_con_jump;
    reg flush_w;
    always @(*) flush_w = flush_con_exc | flush_con_irq | flush_con_jump;

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）：pre_jalr 是 pre_decoder 的译码输出，
//btb_hit 是 bra_predict 的命中输出，两者都是寄存器输出，此处相与不成环。
    always @(*) jalr_pred = pre_jalr & btb_hit;

//各控制/停顿位各自成网：位与位之间不共享逻辑，综合时互不依赖
//★ 上一轮为"剥开 pre"临时加的 [13]/[12] 已整体删除：异常源现在统一过 bju 的判定输入寄存级，
//  exc_gated 里不再有任何与 pre 有关的项 ⇒ [11] 本身就是精确版 flush_con_exc，
//  消费端直接取位，不需要再重组。
//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//★ 每位只连一个源，本模块不在这里做或运算；消费端自己取自己要的位做或（冲刷优先于停顿）。
    always @(*) begin
//★ mret 那一项用【资格版】：错路 mret 的冲刷会把正确路径刚取进来的指令冲掉
//  （实测 wp_mret：分支落点后紧跟的 csrr/ sw 两条笔都不见了）。合法 mret 靠它的自冲刷清影子，必须留。
        flush_con_irq  = exc_irq || exc_irq_ret_ok || exc_irq_act;
        flush_con_jump = jalr_fail || br2 || br3;
        flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
                    stall_rob_full, stall_pc_redir,
                    stall_lsu_haz, stall_lsu_full,
                    stall_mulu_haz, stall_mulu_div,
                    stall_icache_miss, stall_bus_hold};
//ROB 冲刷口 + 边界：**三种冲刷都要作废比边界更年轻的项**。
//★ 异常/中断也必须在【检测那一拍】就冲 ROB（不能只等队头交付）：冲刷拍会把流水线里那些
//  年轻指令的载荷清掉（pd 清 we、lsu 门口挡住），它们已经不可能再写、也永远等不到完成回报
//  ⇒ 留着就是"永远不 wr 的项"卡在队头 ⇒ 而 pc 重定向正好在等 rob_empty ⇒ 自锁
//  （实测 exc_ldst_misalign / half_misalign / exc_irq_lsu / rv32i_cover 四支全卡死）。
//  比边界更老的项照旧留着（它们的写是架构要求的，必须落地后才交付）。
//  边界取法：异常 = 故障项自己那一项；中断 = 正进 E4 那条（它比边界老、要留着）；跳转 = 判定那条。
//★ 异常/中断也要作废 ROB 里【比边界更年轻的项】：冲刷拍会把流水线里那些年轻指令的载荷清掉
//  （pd 清 we、lsu 门口挡住），它们已经不可能再写、也永远等不到完成回报 ⇒ 留着就是"永远不 wr
//  的项"卡在队头 ⇒ 而 pc 重定向正好在等 rob_empty ⇒ 自锁（实测 half_misalign 卡在 pc 0x58）。
//  比边界更老的项照旧留着（它们的写是架构要求的，必须落地后才交付）。
        flush_con_rob = flush_con_jump | flush_con_irq | flush_con_exc;
//★ bju 名下那几种（分支/jalr/jal 非对齐）判定已搬后一级 ⇒ 边界必须用 bju 自己那一条的号
//  （idx_q），不能再用 issue_idx_in（E4 载荷那条 = 判定那条的下一条）。
//  判据用【寄存版】flush_bju_exc，与寄存后的 bju_idx_in 同拍。
//★ 解码级那一路（ecall/ebreak/illegal）现在跟着 bju 的判定输入级走 ⇒ 它的号是 bju_idx_i
//  （不再是当拍载荷的 issue_idx_in —— 那时载荷已经是它的后继了）。
        if (flush_con_exc)
            flush_idx = flush_bju_exc        ? bju_idx_in
                      : (exc_ldst_misalign_in ? exc_ldst_idx_in
                                              : bju_idx_i);
//★ 中断这一支也要分两种：真中断的边界 = 正进载荷那条自己（issue_idx_in）；
//  而 mret 的自冲刷现在也从 bju 输入级出 ⇒ 边界是 mret 自己（bju_idx_i）。
        else if (flush_con_irq)
            flush_idx = exc_irq_ret_ok ? bju_idx_i : issue_idx_in;
        else
            flush_idx = bju_idx_in;
//复位期按原 stage = EXE / 无停顿 的口径给：只有 exec 抬、其余落下
        if (rst_q) begin
            flush_con_irq  = 1'b0;
            flush_con_jump = 1'b0;
            flag_bus = {1'b0, 1'b0, 1'b0, 1'b1, 8'd0};
        end
    end



//===============================================================
// 异常仲裁（原 trap_unit）
//===============================================================
//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir, stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div, stall_icache_miss, stall_bus_hold}
    reg flush_older, flush_older_dec, exc_gated, exc_dec, exc_ldst_dec, flush_w_other;
//★ mret 与其它异常源走同一套：它现在也过 bju 的判定输入级 ⇒ 本拍它自己的标志（exc_irq_ret_i）
//  与"更老那条的寄存版冲刷"同拍。[11]（含 flush_bju_exc）与 [9]（br2/br3/jalr_fail）必来自更老的指令。
//  ⇒ 资格 = 我这条是 mret、且没有被更老的冲刷压住。
//  ★ 自冲刷项显式排掉：合法 mret 自己会拉 flush_con_irq（它靠这个冲掉影子），
//    那一项不能反过来把它自己挡掉 —— 原式是用 ~exc_irq_ret 做豁免的。
    always @(*) flush_w_other = flush_con_exc | flush_con_jump | exc_irq | exc_irq_act;
    always @(*) exc_irq_ret_ok = exc_irq_ret_i & ~flush_w_other;
    reg [31:0] exc_pc_c;
//★ 访存非对齐那条比载荷晚一拍到（它在 lsu 里寄存过），pc 载荷得跟着寄存一拍；
//  否则取到的是【下一条】指令的地址（实测 mepc 记成故障指令 +4：0x3c 而不是 0x38）。
//  （ecall/ebreak/illegal 那条现在走 bju 的 exc_pc_i，同拍有效，不用这一个。）
    reg [31:0] exc_pc_in_d1;
    always @(posedge clk) begin
        if (rst_q)
            exc_pc_in_d1 <= 32'd0;
        else
            exc_pc_in_d1 <= exc_pc_in;
    end
    always @(*) flush_older = flag_bus[10] | flag_bus[9];
//更老的指令在本拍冲刷 ⇒ 这条是错路，它的操作数是垃圾，不能拿它报异常
//★ 解码级那三条（ecall/ebreak/illegal）现在跟着 bju 的判定输入级走 ⇒ 本拍它们自己的标志
//  与"更老那条的寄存版冲刷"同拍。寄存版 bju 冲刷有【两路】：[9]（br2/br3/jalr_fail）与
//  flush_bju_exc（非对齐）—— "已预测跳、目标非对齐"那一支里 [9] 恰好为 0，
//  所以解码级的判据必须把 flush_bju_exc 一并进来，否则影子 ecall 会在那一支漏抑制。
    always @(*) flush_older_dec = flush_older | flush_bju_exc;
    always @(*) exc_dec      = (exc_ecall_i | exc_illegal_i) & ~flush_older_dec;
//★ 访存非对齐那条【保持不并 flush_bju_exc】：它是 lsu 自己寄存的一拍脉冲，与 bju 那条一般不同条；
//  并进去会在"分支非对齐与访存非对齐被 stall 挤到同拍"时误吞合法访存异常。
//  （本轮口径：先保持这处不对称落地，跑绿之后再单独合并。）
    always @(*) exc_ldst_dec = exc_ldst_misalign_in & ~flush_older;
    always @(*) exc_gated    = exc_dec | exc_ldst_dec;

//冲刷（寄存版判据）：bju 那条晚一拍报，与旧行为一致
    always @(*) flush_con_exc = flush_bju_exc | exc_gated;

//★ 标记（组合版判据）：ROB 靠它在【故障指令自己那一拍】把 (cause,pc,tval) 存进它的项。
//  必须是组合版：不写 rd 的指令（br/jal/jalr）在 ROB 里"出厂即 done"，只要排到队头且还没被
//  标记就立刻退掉；寄存版晚一拍，标记会落在已经退掉的槽上被丢弃（实测 exc_br_misalign 死循环）。
//★ 必需与 ~flush_older 相与（理由同上）；mtval 口径：指令地址非对齐用 bju 的【组合落点】
//  （br 目标 / jalr 目标 / jal 由 post_decoder 就地解出的目标），访存非对齐用出错地址，其余 0。
    always @(*) begin
        exc_mark  = (flush_bju_exc & ~bju_older_in) | exc_gated;
        exc_cause = 4'd0;
//三路源各自带正确相位：bju 延迟级 = 非对齐那条；bju 输入级 = ecall/ebreak/illegal 那条；lsu 寄存一拍 = 访存非对齐那条。
        exc_pc_c  = flush_bju_exc        ? bju_pc_in
                  : exc_dec              ? exc_pc_i
                  :                        exc_pc_in_d1;
//ROB 标记口的索引（三路 mux 原来在 cpu_top，按"顶层零逻辑"搬进来）：同样按相位选号
        exc_idx   = flush_bju_exc        ? bju_idx_in
                  : (exc_ldst_misalign_in ? exc_ldst_idx_in
                                          : bju_idx_i);
        exc_tval  = 32'd0;
        if (flush_bju_exc) begin
            exc_cause = CAUSE_MISALIGN_INST;
            exc_tval  = bju_tgt_in;
        end
        else if (exc_gated) begin
            if (exc_illegal_i) begin
                exc_cause = CAUSE_ILLEGAL_INST;
            end
            else if (exc_ldst_misalign_in) begin
                if (exc_ldst_st_in) begin
                    exc_cause = CAUSE_STORE_MISALIGN;
                end
                else begin
                    exc_cause = CAUSE_LOAD_MISALIGN;
                end
                exc_tval  = exc_ldst_addr_in;
            end
            else begin
                exc_cause = exc_ebreak_i ? CAUSE_BREAKPOINT : CAUSE_ECALL_M;
            end
        end
    end

//故障指令的 pc：exc_pc_c 已经是"该指令自己的地址+4"（三路源各按自己的相位取）⇒ 一律减 4
    always @(*) exc_pc = exc_pc_c - 32'd4;

//csr异常/中断寄存器
    csr u_csr (
        .mem_inflight(lsu_inflight),
        .clk(clk),
        .rst(rst_q),
        .csr_wr_en(csr_wr_en),
        .flag_bus(flag_bus),
        .exc_irq_ret(exc_irq_ret),
        .exc_irq_ret_ok(exc_irq_ret_ok),
        .exti(exti),
        .timi(timi),
        .softi(softi),
        .exc_ecall(exc_ecall),
        .flush_con_exc(flush_con_exc),
        .flush_bju_exc(flush_bju_exc),
        .exc_retire(rob_trap),
        .exc_retire_cause(rob_trap_cause),
        .exc_retire_pc(rob_trap_pc),
        .exc_retire_tval(rob_trap_tval),
        .csr_addr(csr_addr),
        .csr_addr_pre(csr_addr_pre),
        .csr_data_in(csr_data_in),
        .pc_addr_in(pc_addr_in),
        .h0_v(fifo_h0_v),
        .lane1_hold(lane1_hold),
        .ird_tmr(ird_tmr),
        .jalr_fail(jalr_fail),
        .br2(br2),
        .br3(br3),
        .csr_data_out(csr_data_out_i),
        .isr_addr2(isr_addr2_i),
        .mcause(mcause_i),
        .exc_irq_act(exc_irq_act_i),
        .exc_irq_processing(exc_irq_processing_i),
        .iret_addr2(iret_addr2_i)
    );

    always @(*) begin
        csr_data_out = csr_data_out_i;
        isr_addr2 = isr_addr2_i;
        mcause = mcause_i;
        exc_irq_act = exc_irq_act_i;
        exc_irq_processing = exc_irq_processing_i;
        exc_irq = exc_irq_act_i;
        iret_addr2 = iret_addr2_i;
    end

    always @(posedge clk) begin
        if (rst_q)
            ird_tmr <= 2'd0;
        else if ((jal | jalr_pred | br1) && !flush_w)
            ird_tmr <= 2'd3;
        else if (ird_tmr != 2'd0)
            ird_tmr <= ird_tmr - 2'd1;
    end

    always @(posedge clk) begin
        if (rst_q)
            exec <= 1'b1;
        else
            exec <= exec;
    end

endmodule
