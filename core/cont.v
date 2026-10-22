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
//              ② 生成本拍的冲刷与停顿总线 flag_bus、ROB 冲刷边界 flush_idx；
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
    input        exc_ecall_i_0, exc_ebreak_i_0, exc_illegal_i_0, exc_irq_ret_i_0,
    input        exc_ecall_i_1, exc_ebreak_i_1, exc_illegal_i_1, exc_irq_ret_i_1,
    input [31:0] exc_pc_i_0, exc_pc_i_1,
    input [3:0]  bju_idx_i_0, bju_idx_i_1,
    input [3:0]  bju_idx_in_0, bju_idx_in_1,
//访存非对齐那条（lsu 自己寄存一拍出来，故比载荷晚一拍）
    input        exc_ldst_misalign_in, exc_ldst_st_in,
    input [31:0] exc_ldst_addr_in,
//三条冲刷各自的 ROB 边界：故障项自己的索引 / E4 载荷（中断边界）/ bju 判定那条自己的索引
    input [31:0] exc_pc_in,
    input [3:0]  exc_ldst_idx_in,
    input [3:0]  issue_idx_in,
//站里最老有效项的号（中断边界与 csr 的 mepc 同源，见 idu2）
    input [3:0]  irq_head_idx_in,
//站里至少有一个有效项：中断的 mepc 与冲刷边界都取"站里最老有效项"，站空时它给不出地址
    input        irq_head_v,
//停顿源（按"顶层不运算"从 cpu_top 下放至此）：逐源各占 flag_bus 一位。
//★ 保留站的"满"【不在本总线上】：它只喂前端三级（cpu_top 直连），见 idu1 的端口说明。
//本模块【不做任何或运算】—— 或/非一律下放到消费者模块内（见各家 stall_w/flush_w 的译码）。
    input        stall_icache_miss, stall_bus_hold,
    input        stall_rob_full,
//ROB 队头指针（透传给 csr，作 minstret 的真退休口径）+ 队空（重定向落地条件）
    input [3:0]  rob_head_ptr,
    input        rob_empty,
//异常交付口（ROB 的 trap_fire + 载荷）：透传给 csr，由它在【队头那一拍】锁 mepc/mcause/mtval
    input        rob_trap,
    input [3:0]  rob_trap_cause,
    input [31:0] rob_trap_pc, rob_trap_tval,
//跳转判定与序号：br1/br2/br3 是判定延迟拍的那三根，jalr_pred 由 pre_jalr & btb_hit 在本模块合成
    input        jal, pre_jalr, btb_hit, br1, br2, br3,
//csr 写口与中断源
    input        csr_wr_en,
//真写使能（idu2 产）：csr_wr_en 只管"是条 CSR 指令"，锁不锁看这根
    input        csr_wr_act,
    input [11:0] csr_addr,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
    input        exti, timi, softi,
//流水线状态（寄存输出）
    output reg [7:0] flag_bus,
    output reg flush_con_exc,
    output reg flush_con_rob,
    output reg [3:0] flush_idx,
//杀集【含边界那条自己】：只给中断用（边界 = 站里最老那条还没执行的指令，要在 mret 之后重跑）
    output reg flush_incl,
//异常仲裁结果：flush_con_exc = 冲刷请求（寄存版判据）；exc_mark + cause/pc/tval = 给 ROB 的当拍标记
    output reg exc_mark,
    output reg [3:0]  exc_cause,
    output reg [31:0] exc_pc, exc_tval,
//ROB 标记口的索引（三路 mux 原来在 cpu_top，按"顶层零逻辑"搬进来）；三路源各自带正确相位
    output reg [3:0]  exc_idx,
//mret 的资格版：同一根线既喂 pc 的排队武装、又喂 csr 的使能恢复（两者是同一次 mret 生效的两半）
    output reg exc_irq_ret_ok,
//重定向登记（原 pc.v 的 redir_kind 状态机搬来）：陷阱 / mret / 中断受理三种，
//都要等 ROB 排空之后才落地；分支/jalr 的错路不走这里（pc 直接用 jp_tgt）。
    output reg        redir_go,
    output reg [31:0] redir_target,
//本拍 pc 落到重定向目标上（icache / iffu / bpu 的捕获窗口按它作废）
    output reg        flush_pc_redir,
//判定延迟拍那一套（success/br_fail/jalr 落点/预测表索引/落点）该取哪条 lane：1 = `_1`
    output reg        bju_fe_sel,
//csr 的对外口（例化输出在本模块内对齐后送出）
    output reg [31:0] isr_addr2, mcause,
//给 alu 的 csr 读值（组合、发那一拍）：站式后 csr 指令的发拍由站决定，提前一拍读不再可能
    output reg [31:0] csr_data_comb,
    output reg        exc_irq_act, exc_irq_processing, exc_irq,
    output reg [31:0] iret_addr2
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

//两条 lane 的 bju 事件仲裁（按 ROB 年龄；只有真带事件的那条参与）
    reg        exc_tgt_mis;
    reg [31:0] bju_pc_in, bju_tgt_in;
    reg        exc_ecall_i, exc_ebreak_i, exc_illegal_i, exc_irq_ret_i;
    reg [31:0] exc_pc_i;
    reg [3:0]  bju_idx_i, bju_idx_in;
    reg        jalr_fail;
    reg [4:0]  age_ev_0, age_ev_1, age_de_0, age_de_1;
    reg        ev_i_0, ev_i_1, ev_de_0, ev_de_1, sel_ev, sel_de;

//流水线控制位：本模块只产自己那三条冲刷源与 exec；八条停顿源逐源过路进 flag_bus
    reg exec, flush_con_irq, flush_con_jump;
    reg flush_w;
    reg redir_pend, stall_pc_redir;

//重定向登记与异常窗口
    reg [1:0] redir_kind;
    reg [3:0] redir_idx;
    reg exc_hold;

//异常仲裁
    reg flush_older, flush_older_dec, exc_gated, exc_dec, exc_ldst_dec, exc_ldst_sel;
    reg flush_w_other;
    reg ldst_older_dec;
    reg [31:0] exc_pc_c;
    reg [31:0] exc_pc_in_d1;

//csr 例化的输出（对齐后送端口）
    wire [31:0] isr_addr2_i, mcause_i, csr_data_comb_i;
    wire exc_irq_act_i, exc_irq_processing_i;
    wire [31:0] iret_addr2_i;

//其它
    reg [1:0] ird_tmr;
    reg jalr_pred;

//===============================================================
// 控制位译码（本模块最前）
//===============================================================
//各控制/停顿位各自成网：位与位之间不共享逻辑，综合时互不依赖
//★ 异常源统一过 bju 的判定输入寄存级，exc_gated 里不再有任何与 pre 有关的项
//  ⇒ [7] 本身就是精确版 flush_con_exc，消费端直接取位，不需要再重组。
//flag_bus[7:0] = {flush_con_exc, flush_con_irq, flush_con_jump, exec,   // [7:4]
//                  stall_rob_full, stall_pc_redir,                     // [3:2]
//                  stall_icache_miss, stall_bus_hold}                  // [1:0]
//★ 原来那三位 stall_lsu_full / stall_mulu_haz / stall_mulu_div 已【从总线上摘掉】：
//  保留站之后它们恒为 0（cpu_top 直接接 1'b0），挂在总线上只是死线；数据相关现在只在站内等。
//★ 每位只连一个源，本模块不在这里做或运算；消费端自己取自己要的位做或（冲刷优先于停顿）。
//★ mret 那一项用【资格版】：错路 mret 的冲刷会把正确路径刚取进来的指令冲掉
//  （实测 wp_mret：分支落点后紧跟的 csrr/ sw 两条笔都不见了）。合法 mret 靠它的自冲刷清影子，必须留。
//★ flush_incl 只有"真中断受理"这一路把边界那条也杀掉。mret 的自冲刷不在此列：它的边界是它自己、它要退。
//ROB 冲刷口 + 边界：**三种冲刷都要作废比边界更年轻的项**。
//★ 异常/中断也必须在【检测那一拍】就冲 ROB（不能只等队头交付）：冲刷拍会把流水线里那些
//  年轻指令的载荷清掉（pd 清 we、lsu 门口挡住），它们已经不可能再写、也永远等不到完成回报
//  ⇒ 留着就是"永远不 wr 的项"卡在队头 ⇒ 而 pc 重定向正好在等 rob_empty ⇒ 自锁
//  （实测 exc_ldst_misalign / half_misalign / exc_irq_lsu / rv32i_cover 四支全卡死）。
//  比边界更老的项照旧留着（它们的写是架构要求的，必须落地后才交付）。
//★ 边界取法：异常 = 故障项自己那一项；中断 = 正进 E4 那条（它比边界老、要留着）；跳转 = 判定那条。
//★ bju 名下那几种（分支/jalr/jal 非对齐）判定已搬后一级 ⇒ 边界必须用 bju 自己那一条的号
//  （idx_q），不能再用 issue_idx_in（E4 载荷那条 = 判定那条的下一条）。
//  判据用【寄存版】exc_tgt_mis，与寄存后的 bju_idx_in 同拍。
//★ 解码级那一路（ecall/ebreak/illegal）现在跟着 bju 的判定输入级走 ⇒ 它的号是 bju_idx_i
//  （不再是当拍载荷的 issue_idx_in —— 那时载荷已经是它的后继了）。
//★ 中断这一支也要分两种：真中断的边界 = 正进载荷那条自己（issue_idx_in）；
//  而 mret 的自冲刷现在也从 bju 输入级出 ⇒ 边界是 mret 自己（bju_idx_i）。
//★ 复位期按原 stage = EXE / 无停顿 的口径给：只有 exec 抬、其余落下。
    always @(*) begin
        flush_con_irq  = exc_irq || exc_irq_ret_ok || exc_irq_act;
        flush_con_jump = jalr_fail || br2 || br3;
        flush_incl     = exc_irq_act;
        flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
                    stall_rob_full, stall_pc_redir,
                    stall_icache_miss, stall_bus_hold};
        flush_con_rob = flush_con_jump | flush_con_irq | flush_con_exc;
        if (flush_con_exc)
            flush_idx = exc_tgt_mis        ? bju_idx_in
                      : (exc_ldst_misalign_in ? exc_ldst_idx_in
                                              : bju_idx_i);
        else if (flush_con_irq)
            flush_idx = exc_irq_ret_ok ? bju_idx_i : irq_head_idx_in;
        else
            flush_idx = bju_idx_in;
        if (rst_q) begin
            flush_con_irq  = 1'b0;
            flush_con_jump = 1'b0;
            flush_incl     = 1'b0;
            flag_bus = {1'b0, 1'b0, 1'b0, 1'b1, 4'd0};
        end
    end

//流水线冲刷位（三条冲刷源的或）
    always @(*) begin
        flush_w = flush_con_exc | flush_con_irq | flush_con_jump;
    end

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）：pre_jalr 是 idu1 的译码输出，
//btb_hit 是 bpu 的命中输出，两者都是寄存器输出，此处相与不成环。
    always @(*) begin
        jalr_pred = pre_jalr & btb_hit;
    end

    always @(posedge clk) begin
        rst_q <= rst;
    end

//===============================================================
// 两条 lane 的 bju 事件仲裁（按 ROB 年龄）
//===============================================================
//合并之后沿用原单套的名字 ⇒ 下面整段判定逻辑一个字都不用改。
//年龄口径与全核一致：{1'b0,(索引-head_ptr)}（4 位先截再扩）
//前端回报（success/br_fail/jalr 落点/预测表索引）也必须与选定那条同源：
//顶层按 bju_fe_sel 把整个判定延迟拍那一套从 _0/_1 里选出来。同一个 id 只可能被一条 lane 判定。
//★ 不能只按年龄选：没带事件那条的 idx 是陈旧值，让它按年龄赢过来会把真事件整条盖掉。
    always @(*) begin
        age_ev_0 = {1'b0, (bju_idx_i_0  - rob_head_ptr)};
        age_ev_1 = {1'b0, (bju_idx_i_1  - rob_head_ptr)};
        age_de_0 = {1'b0, (bju_idx_in_0 - rob_head_ptr)};
        age_de_1 = {1'b0, (bju_idx_in_1 - rob_head_ptr)};
        ev_i_0  = exc_ecall_i_0 | exc_ebreak_i_0 | exc_illegal_i_0 | exc_irq_ret_i_0;
        ev_i_1  = exc_ecall_i_1 | exc_ebreak_i_1 | exc_illegal_i_1 | exc_irq_ret_i_1;
        ev_de_0 = success_0 | br_fail_0 | jalr_fail_0 | exc_tgt_mis_0;
        ev_de_1 = success_1 | br_fail_1 | jalr_fail_1 | exc_tgt_mis_1;
        sel_ev = ev_i_1  & (~ev_i_0  | (age_ev_1 < age_ev_0));
        sel_de = ev_de_1 & (~ev_de_0 | (age_de_1 < age_de_0));
        exc_tgt_mis = sel_de ? exc_tgt_mis_1 : exc_tgt_mis_0;
        bju_pc_in     = sel_de ? bju_pc_in_1     : bju_pc_in_0;
        bju_tgt_in    = sel_de ? bju_tgt_in_1    : bju_tgt_in_0;
        bju_idx_in    = sel_de ? bju_idx_in_1    : bju_idx_in_0;
        jalr_fail     = sel_de ? jalr_fail_1     : jalr_fail_0;
        exc_ecall_i   = sel_ev ? exc_ecall_i_1   : exc_ecall_i_0;
        exc_ebreak_i  = sel_ev ? exc_ebreak_i_1  : exc_ebreak_i_0;
        exc_illegal_i = sel_ev ? exc_illegal_i_1 : exc_illegal_i_0;
        exc_irq_ret_i = sel_ev ? exc_irq_ret_i_1 : exc_irq_ret_i_0;
        exc_pc_i      = sel_ev ? exc_pc_i_1      : exc_pc_i_0;
        bju_idx_i     = sel_ev ? bju_idx_i_1     : bju_idx_i_0;
        bju_fe_sel    = sel_de;
    end

//===============================================================
// 重定向登记（原 pc.v 的 redir_kind 状态机搬来）
//===============================================================
//三个来源：① 陷阱交付（rob_trap）② mret（exc_irq_ret_ok）③ 中断受理（exc_irq_act）。
//都必须在 ROB 排空之后才落地：比边界更老的项还要退、还要写寄存器堆，提前改 pc 会让它们
//重取一遍。rob_empty 就是那个条件（三种冲刷都会把边界之后的全杀掉 ⇒ 很快就空）。
//★ 分支/jalr 的错路**不走这里**：pc 自己在冲刷拍用 jp_tgt 落地（那一拍 pc 已经停住）。
//★ 异常【标记】那一拍起前端就必须停手，一直到陷阱在队头交付、重定向落地。
//  只靠 rob_trap 拉 redir_pend 是不够的：从"标记"到"交付"之间 pc 还在推进、取指还在进队，
//  这一窗里取进来的指令注定要被冲掉；更要命的是它们里面若还有一条故障指令，它的
//  (cause,pc,tval) 会写进 ROB 的【侧寄存器】并把前一条的载荷盖掉 ⇒ 交付的陷阱报的是
//  【后来那条】的 mepc/mcause（实测 WEIRD seed1：0x40 的非法指令，mepc 却记成 0x7c 的）。
//  侧寄存器只存一份是刻意的（异常窗口里本来就只该有一条在飞），所以这里补的是"窗口只有一条"。
//  清条件：陷阱交付（rob_trap）或任何会把边界之后全冲掉的重定向冲刷。
    always @(posedge clk) begin
        if (rst_q)
            exc_hold <= 1'b0;
        else if (rob_trap | flush_con_jump | flush_con_irq)
            exc_hold <= 1'b0;
        else if (exc_mark)
            exc_hold <= 1'b1;
    end

//陷阱与中断跳 mtvec；mret 跳 mepc
//pc 落到重定向目标那一拍：跳转冲刷（flush_con_jump）也在同一拍改向
    always @(*) begin
        redir_pend     = (redir_kind != 2'd0);
        stall_pc_redir = (redir_pend | exc_hold) & ~rob_empty;
        redir_go       = redir_pend & rob_empty & ~rst_q;
        redir_target   = (redir_kind == 2'd2) ? iret_addr2_i : isr_addr2_i;
        flush_pc_redir = redir_go | (flush_con_jump & ~flush_con_exc);
    end

//★ mret 与 interrupt 是在【判定输入级】就登记的（它们不像陷阱那样等队头），
//  而两条 lane 各走各的 ⇒ 比它们【更老】的那条分支可能晚一两拍才判出"预测错"。
//  那一拍 flush_pc_redir 会把 pc 拉到分支落点，若我这条错路的 redir_kind 还留着，
//  下一拍它又会把 pc 拉回 mepc（实测 WEIRD seed1：bge 影子里的 mret 把 pc 拉回 0x40，
//  整段影子程序被当真路径跑完）。
//  撤判据 = 【冲刷边界比我更老】：age 小 = 老，与 ROB 内同一套口径。更年轻的分支冲刷不撤我。
    always @(posedge clk) begin
        if (rst_q) begin
            redir_kind <= 2'd0;
            redir_idx  <= 4'd0;
        end
        else if (redir_go)
            redir_kind <= 2'd0;
        else if (rob_trap)
            redir_kind <= 2'd1;
        else if (exc_irq_ret_ok) begin
            redir_kind <= 2'd2;
            redir_idx  <= bju_idx_i;
        end
        else if (exc_irq_act) begin
            redir_kind <= 2'd3;
            redir_idx  <= irq_head_idx_in;
        end
        else if (flush_pc_redir && (redir_kind != 2'd0) &&
                 ({1'b0, (flush_idx - rob_head_ptr)} < {1'b0, (redir_idx - rob_head_ptr)}))
            redir_kind <= 2'd0;
    end

//===============================================================
// 异常仲裁（原 trap_unit）
//===============================================================
//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//★ mret 与其它异常源走同一套：它现在也过 bju 的判定输入级 ⇒ 本拍它自己的标志（exc_irq_ret_i）
//  与"更老那条的寄存版冲刷"同拍。[7]（含 exc_tgt_mis）与 [5]（br2/br3/jalr_fail）必来自更老的指令。
//  ⇒ 资格 = 我这条是 mret、且没有被更老的冲刷压住。
//  ★ 中断受理那一路（exc_irq_act → flush_con_irq）【不在本式里】：它由本模块的 flush_con_irq
//    经 flag_bus[6] 反喂回来，收进来就成零延时环（synth 8-295，工具靠 set_disable_timing 硬剪
//    ⇒ 硅上行为与 RTL 仿真不一致）。mret 的自冲刷项因此天然不在此列，无需再显式豁免。
    always @(*) begin
        flush_w_other = exc_tgt_mis | flush_con_jump
                      | exc_ecall_i | exc_ebreak_i | exc_illegal_i | exc_ldst_misalign_in;
        exc_irq_ret_ok = exc_irq_ret_i & ~flush_w_other;
    end

//★ 访存非对齐那条比载荷晚一拍到（它在 lsu 里寄存过），pc 载荷得跟着寄存一拍；
//  否则取到的是【下一条】指令的地址（实测 mepc 记成故障指令 +4：0x3c 而不是 0x38）。
//  （ecall/ebreak/illegal 那条现在走 bju 的 exc_pc_i，同拍有效，不用这一个。）
    always @(posedge clk) begin
        if (rst_q)
            exc_pc_in_d1 <= 32'd0;
        else
            exc_pc_in_d1 <= exc_pc_in;
    end

//更老的指令在本拍冲刷 ⇒ 这条是错路，它的操作数是垃圾，不能拿它报异常
//★ 解码级那三条（ecall/ebreak/illegal）现在跟着 bju 的判定输入级走 ⇒ 本拍它们自己的标志
//  与"更老那条的寄存版冲刷"同拍。寄存版 bju 冲刷有【两路】：[5]（br2/br3/jalr_fail）与
//  exc_tgt_mis（非对齐）—— "已预测跳、目标非对齐"那一支里 [5] 恰好为 0，
//  所以解码级的判据必须把 exc_tgt_mis 一并进来，否则影子 ecall 会在那一支漏抑制。
//★ 解码级那三条与"访存非对齐"撞在同一拍时，必须按【年龄】仲裁，取更老那一路。
//  判据与全核一致：age 小 = 老。取错就是拿更年轻那条当冲刷边界 ⇒ 边界自己也被冲掉，
//  陷阱永远交付不出来，而更老那条故障指令被静默 retire。
//  实测 WEIRD seed15：0x38 的非对齐 `sw`（idx7）与 0x3c 的非法指令（idx8）同拍报，
//  原式恒选 bju_idx_i=8 ⇒ flush 杀掉 8 及其后，sw 静默通过，mcause 最后记成后面一条 load 的 4。
//★ 访存非对齐那条【保持不并 exc_tgt_mis】：它是 lsu 自己寄存的一拍脉冲，与 bju 那条一般不同条；
//  并进去会在"分支非对齐与访存非对齐被 stall 挤到同拍"时误吞合法访存异常。
//★ 真正被采用的那一路（exc_ldst_sel）供 cause/tval 用：exc_dec 已含年龄仲裁，
//  被它压住的访存那路不许再出 cause。
    always @(*) begin
        flush_older     = exc_irq_ret_ok | flush_con_jump;
        flush_older_dec = flush_older | exc_tgt_mis;
        ldst_older_dec  = ({1'b0, (exc_ldst_idx_in - rob_head_ptr)} <
                           {1'b0, (bju_idx_i       - rob_head_ptr)});
        exc_dec         = (exc_ecall_i | exc_ebreak_i | exc_illegal_i) & ~flush_older_dec
                        & ~(exc_ldst_misalign_in & ldst_older_dec);
        exc_ldst_dec = exc_ldst_misalign_in & ~flush_older;
        exc_gated    = exc_dec | exc_ldst_dec;
        exc_ldst_sel = exc_ldst_dec & ~exc_dec;
    end

//冲刷（寄存版判据）：bju 那条晚一拍报，与旧行为一致
    always @(*) begin
        flush_con_exc = exc_tgt_mis | exc_gated;
    end

//★ 标记（组合版判据）：ROB 靠它在【故障指令自己那一拍】把 (cause,pc,tval) 存进它的项。
//  必须是组合版：不写 rd 的指令（br/jal/jalr）在 ROB 里"出厂即 done"，只要排到队头且还没被
//  标记就立刻退掉；寄存版晚一拍，标记会落在已经退掉的槽上被丢弃（实测 exc_br_misalign 死循环）。
//★ 目标非对齐那一路不再加错路门（原来的 bju.older_q / bju_older_in 已整个删掉）：
//  判定拍上的 flag_bus[5] 就是【这条自己的】br2/jalr_fail（同 lane、同一个 sel_de 选择器），
//  门了就是自己掐自己；而"更老的指令在冲我"那一半，ROB 在冲刷拍已按年龄把 ent_v 清掉
//  （rob.v:603-605），标记根本进不了项（rob.v:583）。裸位还缺年龄，两条 lane 相位错开时
//  会把活的指令错杀。
//mtval 口径：指令地址非对齐用 bju 的【组合落点】
//  （br 目标 / jalr 目标 / jal 由 post_decoder 就地解出的目标），访存非对齐用出错地址，其余 0。
//三路源各自带正确相位：bju 延迟级 = 非对齐那条；bju 输入级 = ecall/ebreak/illegal 那条；
//lsu 寄存一拍 = 访存非对齐那条。ROB 标记口的索引同理（三路 mux 原来在 cpu_top，按"顶层零逻辑"搬进来）。
    always @(*) begin
        exc_mark  = exc_tgt_mis | exc_gated;
        exc_cause = 4'd0;
        exc_pc_c  = exc_tgt_mis        ? bju_pc_in
                  : exc_dec              ? exc_pc_i
                  :                        exc_pc_in_d1;
        exc_idx   = exc_tgt_mis        ? bju_idx_in
                  : exc_dec              ? bju_idx_i
                  :                        exc_ldst_idx_in;
        exc_tval  = 32'd0;
        if (exc_tgt_mis) begin
            exc_cause = CAUSE_MISALIGN_INST;
            exc_tval  = bju_tgt_in;
        end
        else if (exc_gated) begin
            if (exc_illegal_i & exc_dec) begin
                exc_cause = CAUSE_ILLEGAL_INST;
            end
            else if (exc_ldst_sel) begin
                if (exc_ldst_st_in) begin
                    exc_cause = CAUSE_STORE_MISALIGN;
                end
                else begin
                    exc_cause = CAUSE_LOAD_MISALIGN;
                end
                exc_tval  = exc_ldst_addr_in;
            end
            else if (exc_ebreak_i & exc_dec) begin
                exc_cause = CAUSE_BREAKPOINT;
                exc_tval  = exc_pc;
            end
            else begin
                exc_cause = CAUSE_ECALL_M;
            end
        end
    end

//故障指令的 pc：exc_pc_c 已经是"该指令自己的地址+4"（三路源各按自己的相位取）⇒ 一律减 4
    always @(*) begin
        exc_pc = exc_pc_c - 32'd4;
    end

//===============================================================
// csr 例化与输出对齐
//===============================================================
//csr异常/中断寄存器
    csr u_csr (
        .head_ptr_in(rob_head_ptr),
        .clk(clk),
        .rst(rst_q),
        .csr_wr_en(csr_wr_en),
        .csr_wr_act(csr_wr_act),
        .flag_bus(flag_bus),
        .exc_irq_ret(exc_irq_ret_i),
        .exc_irq_ret_ok(exc_irq_ret_ok),
        .exti(exti),
        .timi(timi),
        .softi(softi),
        .exc_ecall(exc_ecall_i),
        .flush_con_exc(flush_con_exc),
        .exc_mark_in(exc_mark),
        .irq_head_v_in(irq_head_v),
        .exc_retire(rob_trap),
        .exc_retire_cause(rob_trap_cause),
        .exc_retire_pc(rob_trap_pc),
        .exc_retire_tval(rob_trap_tval),
        .csr_addr(csr_addr),
        .csr_data_in(csr_data_in),
        .pc_addr_in(pc_addr_in),
        .ird_tmr(ird_tmr),
        .jalr_fail(jalr_fail),
        .br2(br2),
        .br3(br3),
        .csr_data_comb(csr_data_comb_i),
        .isr_addr2(isr_addr2_i),
        .mcause(mcause_i),
        .exc_irq_act(exc_irq_act_i),
        .exc_irq_processing(exc_irq_processing_i),
        .iret_addr2(iret_addr2_i)
    );

    always @(*) begin
        csr_data_comb = csr_data_comb_i;
        isr_addr2 = isr_addr2_i;
        mcause = mcause_i;
        exc_irq_act = exc_irq_act_i;
        exc_irq_processing = exc_irq_processing_i;
        exc_irq = exc_irq_act_i;
        iret_addr2 = iret_addr2_i;
    end

//===============================================================
// 序号与执行位
//===============================================================
//写序号（乱序写回用，逻辑就放在本模块内）：发射级要不要发号 + 三个单元的滞留兜底请求
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
