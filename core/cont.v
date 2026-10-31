`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 19:04:53
// Design Name:
// Module Name: cont
// Project Name:
// Target Devices:
// Tool Versions:
// Description: 流水线控制与异常仲裁。
//              ① 两条 lane 的 bju 事件按 ROB 年龄二选一（判定 / 解码级异常各一套）；
//              ② 生成本拍的冲刷与停顿总线 flag_bus、ROB 冲刷边界 flush_index；
//              ③ 陷阱 / mret / 中断受理三种重定向的登记与落地；
//              ④ 异常仲裁：谁报、报什么 cause、pc/tval 取哪一份（原 trap_unit 并进本模块）。
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module cont(
    input clk, rst,
//异常仲裁源（原 trap_unit 并进本模块：它没有自己的流水级，按"模块按级划分"不该单独成文件）：
//★【过 bju】ecall/ebreak/illegal/mret 四路信号不再直接吃 decoder 载荷，而是跟操作数/控制位
//  一起进 bju 的【判定输入寄存级】、和 flush_bju_pre 同一拍出来（见 bju.v）。
//  这样它们天生比载荷晚一拍，正好与【更老那条的寄存版冲刷】同拍。
//★ 两条 lane 各一套（`_0`/`_1` 逐字同形），本模块按【ROB 年龄】仲裁：
//  同一拍两条都报事件时只有更老的那条生效（年轻那条本来就会被这次冲刷作废）。
//  idx 唯一 ⇒ 不会并列。哪条真的带事件，也只有它参与仲裁。
    input        exc_tgt_mis_0, exc_tgt_mis_1,
    input [31:0] bju_pc_in_0, bju_pc_in_1,
    input [31:0] bju_tgt_in_0, bju_tgt_in_1,
//判定延迟拍的三根结果（只为仲裁服务：本模块据此判"这一拍哪条 lane 真的判定了东西"）
    input        success_0, success_1, br_fail_0, br_fail_1, jalr_fail_0, jalr_fail_1,
//异常位：bju 已把输入级的 ecall/ebreak/illegal/mret 延迟到判定延迟级 ⇒ 两组同拍同一条指令
    input        exc_ecall_q_0, exc_ebreak_q_0, exc_illegal_q_0, exc_irq_ret_q_0,
    input        exc_ecall_q_1, exc_ebreak_q_1, exc_illegal_q_1, exc_irq_ret_q_1,
    input [3:0]  bju_index_in_0, bju_index_in_1,
//访存非对齐那条（lsu 自己寄存一拍出来，故比载荷晚一拍）
    input        exception_load_store_misalign_in, exc_ldst_st_in,
    input [31:0] exception_load_store_address_in,
//三条冲刷各自的 ROB 边界：故障项自己的索引 / E4 载荷（中断边界）/ bju 判定那条自己的索引
    input [31:0] exception_pc_in,
    input [3:0]  exception_load_store_index_in,
    input [3:0]  issue_index_in,
//站里最老有效项的号（中断边界与 csr 的 mepc 同源，见 idu2）
    input [3:0]  interrupt_head_index_in,
//站里至少有一个有效项：中断的 mepc 与冲刷边界都取"站里最老有效项"，站空时它给不出地址
    input        interrupt_head_valid,
//停顿源（按"顶层不运算"从 cpu_top 下放至此）：逐源各占 flag_bus 一位。
//★ 保留站的"满"【不在本总线上】：它只喂前端三级（cpu_top 直连），见 idu1 的端口说明。
//本模块【不做任何或运算】—— 或/非一律下放到消费者模块内（见各家 stall_w/flush_window 的译码）。
    input        stall_icache_miss, stall_bus_hold,
    input        stall_rob_full,
//队空（重定向落地条件）
    input [15:0] slot_above_head,
    input        rob_empty,
//异常交付口（ROB 的 trap_fire + 载荷）：透传给 csr，由它在【队头那一拍】锁 mepc/mcause/mtval
    input        rob_trap,
    input [1:0]  rob_ret_step,
    input [3:0]  rob_trap_cause,
    input [31:0] rob_trap_pc, rob_trap_tval,
//跳转判定与序号：np_take 是 pc 这一拍实际照做的落点（落点表命中），br2/br3 是判定延迟拍
    input        jump_link, np_take, br2, br3,
//★ S3：改向判定现在由 ROB 在【提交口】给出（队头那条判错了的分支），边界就是队头。
//  bju 的 br2/br3/jump_link_register_failure 不再直接引发冲刷 —— 它们只写进 ROB 项，等队头生效。
    input        redirect_in,
//csr 写口与中断源
    input        csr_write_enable,
//真写使能（idu2 产）：csr_write_enable 只管"是条 CSR 指令"，锁不锁看这根
    input        csr_write_action,
    input [11:0] csr_address,
    input [31:0] csr_data_in,
    input [31:0] pc_address_in,
    input        exti, timi, softi,
//流水线状态（寄存输出）
    output reg [7:0] flag_bus,
//★ 给【执行单元】的寄存版（v2）。当拍版仍然服务 pc 重定向与前端停手（那两条必须同拍），
//  mdu/alu/lsu 这些"错了也无副作用、由 ROB 的世代校验兜底"的单元改吃这一版：
//  它们的 D 引脚原来挂着「rob 的 trap_fire/head_ok → flag_bus → 单元作废」这条跨模块锥。
//  晚一拍无害：作废晚一拍只是多迭代/多算一拍，结果报告会被 ROB 的 ent_v/世代挡掉。
    output reg [7:0] flag_bus_registered,
    output reg flush_control_exception,
    output reg flush_control_rob,
    output reg [3:0] flush_index,
//杀集【含边界那条自己】：只给中断用（边界 = 站里最老那条还没执行的指令，要在 mret 之后重跑）
    output reg flush_inclusive,
//★ 冲刷记录的【寄存版】：作废晚一拍做。
//  为什么能晚：pc 改向、前端停手（flag_bus）仍然当拍；而 ROB 的作废、idu 的击杀、lsu 的丢弃
//  都只是在"那条指令真的写回/提交之前"完成即可。实测冲刷之后前端连续 4 拍不往 D 送新指令
//  （tb_reb：DNEW_AT +1..+4 全 0），这一拍本来就是空的。
//  为什么必须晚：当拍那条链是 cont 仲裁(6 级) → rob 边界判定(6 级) → idu2 击杀+操作数选择(11 级)，
//  跨三个模块，实测 23 级 / 20.559ns / 84% 是布线。寄存一刀把三跳劈成"cont 一跳"和"rob→idu2 一跳"。
//  ★ 只有【ROB 侧】的消费者改吃这一版；flag_bus、pc、bju 的 keep_judge 必须继续吃当拍版
//    （keep_judge 判的就是"冲刷这一拍我正在判的那条还算不算活着"，晚一拍就没有意义了）。
    output reg [3:0] flush_index_q,
    output reg flush_con_exc_q,
    output reg flush_con_rob_q,
    output reg flush_incl_q,
//"上一拍刚冲刷过"：这 1 拍里 Q 里的错路指令可能够到发射，必须挡住（尤其错路 csrrw 会在发射拍锁存 CSR）
    output reg flush_pending,
//异常报告口 ×3：lane0 / lane1 / lsu。各带各的槽号，直接进 ROB 的三个标记口，本模块不做仲裁。
    output reg        exc0_en,
    output reg [3:0]  exc0_index,
    output reg [3:0]  exc0_cause,
    output reg [31:0] exc0_pc, exc0_tval,
    output reg        exc1_en,
    output reg [3:0]  exc1_index,
    output reg [3:0]  exc1_cause,
    output reg [31:0] exc1_pc, exc1_tval,
    output reg        exception_selected_enable,
    output reg [3:0]  exception_selected_index,
    output reg [3:0]  exception_selected_cause,
    output reg [31:0] exception_selected_pc, exception_selected_trap_value,
//mret 的资格版：同一根线既喂 pc 的排队武装、又喂 csr 的使能恢复（两者是同一次 mret 生效的两半）
    output reg exception_interrupt_return_ok,
// mret 搬到提交口：判定拍只报"哪一项是 mret"，队头那一拍由 ROB 拉 mret_valid 回来。
    output reg        mret_mark,
    output reg [3:0]  mret_index,
    output reg [31:0] mret_target,
    input             mret_valid,
//重定向登记（原 pc.v 的 redirect_kind 状态机搬来）：陷阱 / mret / 中断受理三种，
//都要等 ROB 排空之后才落地；分支/jalr 的错路不走这里（pc 直接用 jp_tgt）。
    output reg        redirect_go,
    output reg [31:0] redirect_target,
//本拍 pc 落到重定向目标上（icache / iffu / bpu 的捕获窗口按它作废）
    output reg        flush_pc_redirect,
//判定延迟拍那一套（success/br_fail/jalr 落点/预测表索引/落点）该取哪条 lane：1 = `_1`
//csr 的对外口（例化输出在本模块内对齐后送出）
    output reg [31:0] interrupt_service_address_2, mcause,
//给 alu 的 csr 读值（组合、发那一拍）：站式后 csr 指令的发拍由站决定，提前一拍读不再可能
    output reg [31:0] csr_data_combinational,
    output reg        exception_interrupt_action, exception_interrupt_processing, exc_irq,
    output reg [31:0] interrupt_return_address_2
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
    reg reset_registered;

//流水线控制位：本模块只产自己那三条冲刷源与 exec；八条停顿源逐源过路进 flag_bus
    reg exec, flush_control_interrupt, flush_control_jump;
    reg flush_window;
    reg redirect_pending, stall_pc_redirect;

//重定向登记与异常窗口
    reg [1:0] redirect_kind;
    // redirect_index 已删：它只服务"中断受理记录的撤销判据"，而那条判据 = older(flush_index, 记录时的队头)。
    // 现在所有改向（分支 / 陷阱 / mret / 中断）都在 ROB 队头发起 ⇒ flush_index 恒 = 队头，
    // 而记录时的队头不可能比现在的队头更年轻 ⇒ older() 恒 0，这个寄存器与那支索引锥都是死的。
    // exc_hold 已删

//异常仲裁
    reg flush_older, flush_older_decode, exception_gated, exception_decode, exception_load_store_decode, exception_load_store_select;
    reg load_store_older_decode;
//（原来的 exception_pc_in_delayed 已删：访存非对齐的 pc 改由 lsu 与故障同拍锁存）

//csr 例化的输出（对齐后送端口）
    wire [31:0] isr_addr2_i, mcause_judged, csr_data_combinational_judged;
    wire exc_irq_act_i, exc_irq_processing_i;
    wire [31:0] iret_addr2_i;
//★★ 2026-10-31 修（存量 bug，从 10-28 的备份一路都在）：
//  csr 的 exc_irq_ret 口原来接的是 exc_irq_ret_i —— 这个名字在【本文件里从没声明过】，
//  Verilog 于是给了它一根隐式 1 位 wire【没人驱动 ⇒ 恒为 z】。
//  csr 里 `exception_interrupt_action = ... && !(exc_irq_ret | flag_bus[5]) && ...`，
//  z|0 = x ⇒ 只要 mstatus.MIE 一打开、且外部中断挂起，中断受理信号就是 X，
//  它经 flag_bus[6] 把 flush_window 打成 X ⇒ 整核冻死。
//  后果：本核的【机器外部中断从来就没工作过】（mt13/mt15 一直挂在这）。
//  它本来就该接"本拍有一条 mret 已经够资格"（exception_interrupt_return_ok）。
    wire exc_irq_ret_i;
    assign exc_irq_ret_i = exception_interrupt_return_ok;

//其它
    reg [1:0] interrupt_request_timer;

//===============================================================
// 控制位译码（本模块最前）
//===============================================================
//各控制/停顿位各自成网：位与位之间不共享逻辑，综合时互不依赖
//★ 异常源统一过 bju 的判定输入寄存级，exception_gated 里不再有任何与 pre 有关的项
//  ⇒ [7] 本身就是精确版 flush_control_exception，消费端直接取位，不需要再重组。
//flag_bus[7:0] = {flush_control_exception, flush_control_interrupt, flush_control_jump, exec,   // [7:4]
//                  stall_rob_full, stall_pc_redirect,                     // [3:2]
//                  stall_icache_miss, stall_bus_hold}                  // [1:0]
//★ 原来那三位 stall_lsu_full / stall_mulu_haz / stall_mulu_div 已【从总线上摘掉】：
//  保留站之后它们恒为 0（cpu_top 直接接 1'b0），挂在总线上只是死线；数据相关现在只在站内等。
//★ 每位只连一个源，本模块不在这里做或运算；消费端自己取自己要的位做或（冲刷优先于停顿）。
//★ mret 那一项用【资格版】：错路 mret 的冲刷会把正确路径刚取进来的指令冲掉
//  （实测 wp_mret：分支落点后紧跟的 csrr/ sw 两条笔都不见了）。合法 mret 靠它的自冲刷清影子，必须留。
//★ flush_inclusive 只有"真中断受理"这一路把边界那条也杀掉。mret 的自冲刷不在此列：它的边界是它自己、它要退。
//ROB 冲刷口 + 边界：**三种冲刷都要作废比边界更年轻的项**。
//★ 异常/中断也必须在【检测那一拍】就冲 ROB（不能只等队头交付）：冲刷拍会把流水线里那些
//  年轻指令的载荷清掉（pd 清 we、lsu 门口挡住），它们已经不可能再写、也永远等不到完成回报
//  ⇒ 留着就是"永远不 wr 的项"卡在队头 ⇒ 而 pc 重定向正好在等 rob_empty ⇒ 自锁
//  （实测 exc_ldst_misalign / half_misalign / exc_irq_lsu / rv32i_cover 四支全卡死）。
//  比边界更老的项照旧留着（它们的写是架构要求的，必须落地后才交付）。
//★ 边界取法：异常 = 故障项自己那一项；中断 = 正进 E4 那条（它比边界老、要留着）；跳转 = 判定那条。
//★ bju 名下那几种（分支/jalr/jump_link 非对齐）判定已搬后一级 ⇒ 边界必须用 bju 自己那一条的号
//  （idx_q），不能再用 issue_index_in（E4 载荷那条 = 判定那条的下一条）。
//  判据用【寄存版】exception_target_misalign，与寄存后的 branch_unit_index_in 同拍。
//★ 解码级那一路（ecall/ebreak/illegal）现在跟着 bju 的判定输入级走 ⇒ 它的号是 judged_index
//  （不再是当拍载荷的 issue_index_in —— 那时载荷已经是它的后继了）。
//★ 中断这一支也要分两种：真中断的边界 = 正进载荷那条自己（issue_index_in）；
//  而 mret 的自冲刷现在也从 bju 输入级出 ⇒ 边界是 mret 自己（judged_index）。
//★ 复位期按原 stage = EXE / 无停顿 的口径给：只有 exec 抬、其余落下。
    always @(*) begin
//★ mret 的自冲刷撤了：它现在走 bju 那条路（队头 redir_v → flush_control_jump），
//  改向那一拍自己就把比它年轻的冲干净，不需要判定拍先冲一次。
        flush_control_interrupt  = exc_irq || exception_interrupt_action;
        flush_control_jump = redirect_in;   // S3：提交口改向（原 bju 判定拍）
        flush_inclusive     = exception_interrupt_action;
        flag_bus = {flush_control_exception, flush_control_interrupt, flush_control_jump, exec,
                    stall_rob_full, stall_pc_redirect,
                    stall_icache_miss, stall_bus_hold};
//★ 2026-10-31：总线被按住的【消费者】已经挪进 lsu（blank_bus），所以 idu1 的 stall 不再取本位。
//  总线定义本身保持原样（八位一位不少、一位不换），只是第 0 位现在没有读者 —— 留着是为了
//  "总线 = 八个源"这件事可读，不是因为它还有用。
        flush_control_rob = flush_control_jump | flush_control_interrupt | flush_control_exception;
        if (flush_control_interrupt)
            flush_index = interrupt_head_index_in;
        else
// P4：改向边界是 ROB 自己的队头，由 ROB 内部解析（rob.flush_index_eff）。
// cont 只报"异常/中断候选"；flush_index 的下游消费者（idu1 / idu2 / rob）
// 只在 flush_inclusive=1（中断）时使用它，而中断永远不走改向这条路 ⇒ 改向这一档不必有值。
            flush_index = 4'd0;
        if (reset_registered) begin
            flush_control_interrupt  = 1'b0;
            flush_control_jump = 1'b0;
            flush_inclusive     = 1'b0;
            flag_bus = {1'b0, 1'b0, 1'b0, 1'b1, 4'd0};
        end
    end

    always @(posedge clk) begin
        if (reset_registered) flag_bus_registered <= 8'h10;
        else       flag_bus_registered <= flag_bus;
    end

//流水线冲刷位（三条冲刷源的或）
    always @(*) begin
        flush_window = flush_control_exception | flush_control_interrupt | flush_control_jump;
    end

//===============================================================
// 冲刷记录打一拍（ROB 侧消费者吃这一版，见端口注释）
//===============================================================
//★ 四个量必须【同一条边沿、同一组寄存器】：边界、包含边界、是否冲刷、是否异常。
//  只寄存 idx 不寄存 con_rob，ROB 就会拿旧边界配新冲刷；只寄存 con_rob 不寄存 idx，
//  就会拿新冲刷配旧边界 —— 两种都是"冲错人"。flush_pending 与它们同一组，保证"停手"和"作废"
//  永远相差恰好一拍，不会出现"作废了但没停手"或"停手了两拍还没作废"。
    always @(posedge clk) begin
        if (reset_registered) begin
            flush_index_q     <= 4'd0;
            flush_con_exc_q <= 1'b0;
            flush_con_rob_q <= 1'b0;
            flush_incl_q    <= 1'b0;
            flush_pending      <= 1'b0;
        end
        else begin
            flush_index_q     <= flush_index;
            flush_con_exc_q <= flush_control_exception;
            flush_con_rob_q <= flush_control_rob;
            flush_incl_q    <= flush_inclusive;
            flush_pending      <= flush_control_rob;
        end
    end

    always @(posedge clk) begin
        reset_registered <= rst;
    end

//===============================================================
// 两条 lane 的 bju 事件仲裁 —— 已整段删除（2026-10-30）
//===============================================================
//它产出的五个量 branch_unit_pc_in / branch_unit_target_in / branch_unit_index_in /
//jump_link_register_failure / exception_target_misalign 都没有读者：
//  · 改向早已搬到 ROB 提交口（本模块 flush_control_jump = redirect_in），判定拍不再直接改向；
//  · jump_link_register_failure 原送 csr，csr 在 S3 之后改用 flag_bus[5]（= 提交口改向）；
//  · branch_unit_index_in 原送 flush_index 的非中断分支，而 flush_index 的下游
//    （idu1 / idu2 / rob）只在 flush_inclusive=1（中断）时读它，中断那一路走 interrupt_head_index_in；
//  · branch_unit_pc_in / branch_unit_target_in / exception_target_misalign 从来没有读者。
//⇒ select_branch_event / branch_event_0/1 一并删掉，idu2 里那条 BJU 判定口互斥也随之失去防守对象。
//  两条 lane 的判定回报各走各的 ROB 口（bju_judged / branch_unit_lane1_judged），不需要仲裁。


//===============================================================
// 重定向登记（原 pc.v 的 redirect_kind 状态机搬来）
//===============================================================
//三个来源：① 陷阱交付（rob_trap）② mret（exception_interrupt_return_ok）③ 中断受理（exception_interrupt_action）。
//都必须在 ROB 排空之后才落地：比边界更老的项还要退、还要写寄存器堆，提前改 pc 会让它们
//重取一遍。rob_empty 就是那个条件（三种冲刷都会把边界之后的全杀掉 ⇒ 很快就空）。
//★ 分支/jalr 的错路**不走这里**：pc 自己在冲刷拍用 jp_tgt 落地（那一拍 pc 已经停住）。
//★ exc_hold 已删（部署 A）：异常载荷是【按项】存的（ent_exc/ent_data/ent_tgt），
//  从标记到交付的窗口里进来几条都不会互相覆盖 ⇒ 不再需要"窗口只有一条"。
//  前端只在交付拍之后停手（redirect_pending 一直拉到改向落地）。
//陷阱与中断跳 mtvec；mret 跳 mepc
//pc 落到重定向目标那一拍：跳转冲刷（flush_control_jump）也在同一拍改向
    always @(*) begin
        redirect_pending     = (redirect_kind != 2'd0);
        stall_pc_redirect = redirect_pending & ~rob_empty;
        redirect_go       = redirect_pending & rob_empty & ~reset_registered;
        redirect_target   = isr_addr2_i;   //只剩 1(陷阱) / 3(中断受理) 两档，落点都是 mtvec
        flush_pc_redirect = redirect_go | (flush_control_jump & ~flush_control_exception);
    end

//★ 撤回一条待交付的改向记录，判据只有一个：本方来了分支改向（redirect_in）。
//  mret 与陷阱现在都从 ROB 队头出（rob.v:517 mret_now / rob.v:519 redirect_now 都判 entry_valid[head]），
//  中断受理记录的 interrupt_head_index_in 也是队头（cpu_top.v:957 head_index_idu2）⇒
//  "有比我更老的冲刷"这一情形不存在（队头不会倒退）⇒ 原先那半判据 older(flush_index, redirect_index) 恒 0，已删。
    always @(posedge clk) begin
        if (reset_registered) begin
            redirect_kind <= 2'd0;
        end
        else if (redirect_go)
            redirect_kind <= 2'd0;
//★ 必须吃【组合版】trap_fire：吃寄存版 rob_trap 就比交付晚一拍，而交付那拍之后
//  既没有 flush_window 也没有 redirect_pending ⇒ 中间空出一拍，pc 会前进、取指会进队，
//  而它们的槽位已经不在"边界之后"，改向不会冲掉它们 ⇒ 它们在 handler 里退休。
        else if (rob_trap)
            redirect_kind <= 2'd1;
        else if (exception_interrupt_action)
            redirect_kind <= 2'd3;
// 撤销判据只剩【本方来了分支改向】：所有改向都在队头，队头不会倒退 ⇒ 不存在"比我更老的冲刷"，
// 原先那一半 older(flush_index, redirect_index) 恒 0，已删（连同 redirect_index 与那支 16:1 索引锥）。
        else if (flush_pc_redirect && (redirect_kind != 2'd0) && redirect_in)
            redirect_kind <= 2'd0;
    end

//===============================================================
// 异常仲裁（原 trap_unit）
//===============================================================
//flag_bus = {flush_control_exception, flush_control_interrupt, flush_control_jump, exec,
//            stall_rob_full, stall_pc_redirect,
//            stall_icache_miss, stall_bus_hold}
//★ mret 与其它异常源走同一套：它现在也过 bju 的判定输入级 ⇒ 本拍它自己的标志（exc_irq_ret_i）
//  与"更老那条的寄存版冲刷"同拍。[7]（含 exception_target_misalign）与 [5]（br2/br3/jump_link_register_failure）必来自更老的指令。
//  ⇒ 资格 = 我这条是 mret、且没有被更老的冲刷压住。
//  ★ 中断受理那一路（exception_interrupt_action → flush_control_interrupt）【不在本式里】：它由本模块的 flush_control_interrupt
//    经 flag_bus[6] 反喂回来，收进来就成零延时环（synth 8-295，工具靠 set_disable_timing 硬剪
//    ⇒ 硅上行为与 RTL 仿真不一致）。mret 的自冲刷项因此天然不在此列，无需再显式豁免。
    always @(*) begin
        exception_interrupt_return_ok = (exc_irq_ret_q_0 | exc_irq_ret_q_1)
                       & ~(exc_tgt_mis_0 | exc_tgt_mis_1 | flush_control_jump
                         | exc_ecall_q_0 | exc_ebreak_q_0 | exc_illegal_q_0
                         | exc_ecall_q_1 | exc_ebreak_q_1 | exc_illegal_q_1
                         | exception_load_store_misalign_in);
    end

//★ 2026-10-31：访存非对齐那条的 pc 不再在这里延迟。
//  原来拿前端的 auxiliary_address 延迟一拍再减 4，前提是"报故障那一拍 aux 正好是故障指令 +4"；
//  前端一停就完全不成立（mt11：故障在 0x30、aux 冻在 0x18 ⇒ mepc 记成 0x14 ⇒ 死循环）。
//  现在 pc 由 lsu 与故障同拍锁存（exception_load_store_pc_out），到这里已经是对的，
//  与 idx/address/cause 完全同相位，直接用。

//更老的指令在本拍冲刷 ⇒ 这条是错路，它的操作数是垃圾，不能拿它报异常
//★ 解码级那三条（ecall/ebreak/illegal）现在跟着 bju 的判定输入级走 ⇒ 本拍它们自己的标志
//  与"更老那条的寄存版冲刷"同拍。寄存版 bju 冲刷有【两路】：[5]（br2/br3/jump_link_register_failure）与
//  exception_target_misalign（非对齐）—— "已预测跳、目标非对齐"那一支里 [5] 恰好为 0，
//  所以解码级的判据必须把 exception_target_misalign 一并进来，否则影子 ecall 会在那一支漏抑制。
//★ 解码级那三条与"访存非对齐"撞在同一拍时，必须按【年龄】仲裁，取更老那一路。
//  判据与全核一致：age 小 = 老。取错就是拿更年轻那条当冲刷边界 ⇒ 边界自己也被冲掉，
//  陷阱永远交付不出来，而更老那条故障指令被静默 retire。
//  实测 WEIRD seed15：0x38 的非对齐 `sw`（idx7）与 0x3c 的非法指令（idx8）同拍报，
//  原式恒选 judged_index=8 ⇒ flush 杀掉 8 及其后，sw 静默通过，mcause 最后记成后面一条 load 的 4。
//★ 访存非对齐那条【保持不并 exception_target_misalign】：它是 lsu 自己寄存的一拍脉冲，与 bju 那条一般不同条；
//  并进去会在"分支非对齐与访存非对齐被 stall 挤到同拍"时误吞合法访存异常。
//★ 真正被采用的那一路（exception_load_store_select）供 cause/tval 用：exception_decode 已含年龄仲裁，
//  被它压住的访存那路不许再出 cause。
    always @(*) begin
        flush_older = flush_control_jump;
    end

//部署：异常/错误指令与 bju 同一套 —— 标记拍只把 (cause,pc,tval) 写进项，
//到队头那一刻（rob.trap_fire）才冲刷 + 改向，边界 = 队头。
//★ 标记拍【不冲刷、不冻前端】：比它年轻的项在交付拍被整表清空（rob.v 的 trap_fire 口），
//  精确性由"按项标记 + 队头交付"保证，不再靠"检出即冲刷"。
    always @(*) begin
//★ 广播必须从【触发器】出去，不能从队头读阵列出去：trap_fire 的组合版要穿
//  head_ptr → slot_above_head → older() → 异常仲裁 → ent_ex[head] 一整条链，直接把 flag_bus[7]
//  的 13 个消费者拖到 −8.410ns（实测最差路 rob/head_ptr → lsu/sqa_wdat，32 级/72% 布线）。
//  改吃寄存版 rob_trap（= dly 里的 trap_fire 寄存器）：广播从触发器出发，只剩纯网延迟。
//  相位仍然自洽：trap_fire 那一拍（T）负责清 ROB + 打 redirect_kind；
//  T+1 拍寄存版冲刷清流水线，而 redirect_go 也在 T+1 拍（rob 已空）把 pc 拉到 mtvec
//  ⇒ 两件事同拍，中间不留"pc 前进、取指进队"的空窗。
        flush_control_exception = rob_trap;
    end

//★ 标记（组合版判据）：ROB 靠它在【故障指令自己那一拍】把 (cause,pc,tval) 存进它的项。
//  必须是组合版：不写 rd 的指令（br/jump_link/jalr）在 ROB 里"出厂即 done"，只要排到队头且还没被
//  标记就立刻退掉；寄存版晚一拍，标记会落在已经退掉的槽上被丢弃（实测 exc_br_misalign 死循环）。
//★ 目标非对齐那一路不再加错路门（原来的 bju.older_q / bju_older_in 已整个删掉）：
//  判定拍上的 flag_bus[5] 就是【这条自己的】br2/jump_link_register_failure（同 lane、同一个 select_branch_event 选择器），
//  门了就是自己掐自己；而"更老的指令在冲我"那一半，ROB 在冲刷拍已按年龄把 ent_v 清掉
//  （rob.v:603-605），标记根本进不了项（rob.v:583）。裸位还缺年龄，两条 lane 相位错开时
//  会把活的指令错杀。
//mtval 口径：指令地址非对齐用 bju 的【组合落点】
//  （br 目标 / jalr 目标 / jump_link 由 post_decoder 就地解出的目标），访存非对齐用出错地址，其余 0。
//三路源各自带正确相位：bju 延迟级 = 非对齐那条；bju 输入级 = ecall/ebreak/illegal 那条；
//lsu 寄存一拍 = 访存非对齐那条。ROB 标记口的索引同理（三路 mux 原来在 cpu_top，按"顶层零逻辑"搬进来）。
//三个异常报告口：lane0 / lane1 / lsu。每条异常属于它自己那一项，各带各的槽号，互不仲裁。
//★ bju 已把输入级的 ecall/ebreak/illegal/mret 延迟到判定延迟级 ⇒ 与 exception_target_misalign 同拍同一条指令，
//  所以一个 lane 一拍最多报一条，lane 之间不需要比年龄。
//★ bju 那两路给的是"该指令地址 +4" ⇒ 这里减 4。
//  lsu 那一路给的就是 pc 本体（它自己锁的），不减。
    always @(*) begin
        exc0_pc  = bju_pc_in_0  - 32'd4;
        exc1_pc  = bju_pc_in_1  - 32'd4;
        exception_selected_pc = exception_pc_in;

        exc0_en    = (exc_tgt_mis_0 | exc_illegal_q_0 | exc_ebreak_q_0 | exc_ecall_q_0) & ~flush_older;
        exc0_index   = bju_index_in_0;
        exc0_cause = exc_tgt_mis_0   ? CAUSE_MISALIGN_INST :
                     exc_illegal_q_0 ? CAUSE_ILLEGAL_INST  :
                     exc_ebreak_q_0  ? CAUSE_BREAKPOINT    : CAUSE_ECALL_M;
        exc0_tval  = exc_tgt_mis_0  ? bju_tgt_in_0 :
                     exc_ebreak_q_0 ? exc0_pc : 32'd0;

        exc1_en    = (exc_tgt_mis_1 | exc_illegal_q_1 | exc_ebreak_q_1 | exc_ecall_q_1) & ~flush_older;
        exc1_index   = bju_index_in_1;
        exc1_cause = exc_tgt_mis_1   ? CAUSE_MISALIGN_INST :
                     exc_illegal_q_1 ? CAUSE_ILLEGAL_INST  :
                     exc_ebreak_q_1  ? CAUSE_BREAKPOINT    : CAUSE_ECALL_M;
        exc1_tval  = exc_tgt_mis_1  ? bju_tgt_in_1 :
                     exc_ebreak_q_1 ? exc1_pc : 32'd0;

        exception_selected_enable    = exception_load_store_misalign_in & ~flush_older;
        exception_selected_index   = exception_load_store_index_in;
        exception_selected_cause = exc_ldst_st_in ? CAUSE_STORE_MISALIGN : CAUSE_LOAD_MISALIGN;
        exception_selected_trap_value  = exception_load_store_address_in;
    end


//===============================================================
// csr 例化与输出对齐
//===============================================================
//csr异常/中断寄存器
    csr u_csr (
        .retire_step_in(rob_ret_step),
        .clk(clk),
        .rst(reset_registered),
        .csr_write_enable(csr_write_enable),
        .csr_write_action(csr_write_action),
        .flush_control_in(flag_bus[6:5]),
        .exc_irq_ret(exc_irq_ret_i),
        .exception_interrupt_return_ok(mret_valid),
        .exti(exti),
        .timi(timi),
        .softi(softi),
        .exc_ecall(exc_ecall_i),
        .flush_control_exception(flush_control_exception),
        .exception_mark_in(exc_mark),
        .interrupt_head_valid_in(interrupt_head_valid),
        .exception_retire(rob_trap),
        .exception_retire_cause(rob_trap_cause),
        .exception_retire_pc(rob_trap_pc),
        .exc_retire_tval(rob_trap_tval),
        .csr_address(csr_address),
        .csr_data_in(csr_data_in),
        .pc_address_in(pc_address_in),
        .interrupt_request_timer(interrupt_request_timer),
        .jump_link_register_failure(1'b0),   // csr 在 S3 之后改用 flag_bus[5]（提交口改向），此口已无读者
        .br2(br2),
        .br3(br3),
        .csr_data_combinational(csr_data_combinational_judged),
        .interrupt_service_address_2(isr_addr2_i),
        .mcause(mcause_judged),
        .exception_interrupt_action(exc_irq_act_i),
        .exception_interrupt_processing(exc_irq_processing_i),
        .interrupt_return_address_2(iret_addr2_i)
    );

    always @(*) begin
        csr_data_combinational = csr_data_combinational_judged;
        interrupt_service_address_2 = isr_addr2_i;
        mcause = mcause_judged;
        exception_interrupt_action = exc_irq_act_i;
        exception_interrupt_processing = exc_irq_processing_i;
        exc_irq = exc_irq_act_i;
        interrupt_return_address_2 = iret_addr2_i;
    end

//mret 标签出口：判定拍把"这一项是 mret"交给 ROB；落点由 mepc 在队头现读。
    always @(*) begin
        mret_mark = exception_interrupt_return_ok;
        mret_index  = exc_irq_ret_q_0 ? bju_index_in_0 : bju_index_in_1;
        mret_target  = iret_addr2_i;
    end

//===============================================================
// 序号与执行位
//===============================================================
//写序号（乱序写回用，逻辑就放在本模块内）：发射级要不要发号 + 三个单元的滞留兜底请求
    always @(posedge clk) begin
        if (reset_registered)
            interrupt_request_timer <= 2'd0;
        else if ((jump_link | np_take) && !flush_window)
            interrupt_request_timer <= 2'd3;
        else if (interrupt_request_timer != 2'd0)
            interrupt_request_timer <= interrupt_request_timer - 2'd1;
    end

    always @(posedge clk) begin
        if (reset_registered)
            exec <= 1'b1;
        else
            exec <= exec;
    end

endmodule
