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


module clint(
    input clk, rst,
    input exc_tgt_mis_0, exc_tgt_mis_1,
    input [31:0] bju_pc_in_0, bju_pc_in_1,
    input [31:0] bju_tgt_in_0, bju_tgt_in_1,
//判定延迟拍的三根结果（只为仲裁服务：本模块据此判"这一拍哪条 lane 真的判定了东西"）
    input        success_0, success_1, br_fail_0, br_fail_1, jalr_fail_0, jalr_fail_1,
//异常位：bju 已把输入级的 ecall/ebreak/illegal/mret 延迟到判定延迟级 ⇒ 两组同拍同一条指令
    input        exc_ecall_q_0, exc_ebreak_q_0, exc_illegal_q_0, exc_irq_ret_q_0,
    input        exc_ecall_q_1, exc_ebreak_q_1, exc_illegal_q_1, exc_irq_ret_q_1,
    input [4:0]  bju_index_in_0, bju_index_in_1,
//访存非对齐那条（lsu 自己寄存一拍出来，故比载荷晚一拍）
    input        exception_load_store_misalign_in, exc_ldst_st_in,
    input [31:0] exception_load_store_address_in,
//三条冲刷各自的 ROB 边界：故障项自己的索引 / E4 载荷（中断边界）/ bju 判定那条自己的索引
    input [31:0] exception_pc_in,
    input [4:0]  exception_load_store_index_in,
    input [4:0]  issue_index_in,
//站里最老有效项的号（中断边界与 csr 的 mepc 同源，见 isq）
    input [4:0]  interrupt_head_index_in,
//站里至少有一个有效项：中断的 mepc 与冲刷边界都取"站里最老有效项"，站空时它给不出地址
    input        interrupt_head_valid,
//停顿源（按"顶层不运算"从 cpu_top 下放至此）：逐源各占 flag_bus 一位。
//★ 保留站的"满"【不在本总线上】：它只喂前端三级（cpu_top 直连），见 idu 的端口说明。
//本模块【不做任何或运算】—— 或/非一律下放到消费者模块内（见各家 stall_w/flush_window 的译码）。
    input        stall_icache_miss, stall_bus_hold,
    input        stall_rob_full,
//队空（重定向落地条件）
    input [31:0] slot_above_head,
//异常交付口（ROB 的 trap_fire + 载荷）：透传给 csr，由它在【队头那一拍】锁 mepc/mcause/mtval
    input        rob_trap,
    input [1:0]  rob_ret_step,
    input [3:0]  rob_trap_cause,
    input [31:0] rob_trap_pc, rob_trap_tval,
//跳转判定与序号：np_take 是 pc 这一拍实际照做的落点（落点表命中），br2/br3 是判定延迟拍
    input        jump_link, np_take, br2, br3,
//★ S3：改向判定现在由 ROB 在【提交口】给出（队头那条判错了的分支），边界就是队头。
//  bju 的 br2/br3/jump_link_register_failure 不再直接引发冲刷 —— 它们只写进 ROB 项，等队头生效。
    input        redir_backend_0,
//★ S5：A 域改向（判定拍 / mret）—— 只清前端 + 改 PC，不碰后端。B 域仍是上面的 redir_backend_0。
    input        redir_frontend,
//csr 读口（执行级）与提交口（写）与中断源
    input [11:0] csr_address,
//★ 2026-11-04（CSR 步骤 C）：读-改-写搬出执行级，改为提交拍落地。这四根由 ROB 提交口给（各是一级触发器）。
    input        csr_commit_valid,
    input [11:0] csr_commit_address,
    input [2:0]  csr_commit_function3,
    input [31:0] csr_commit_source,
    input [31:0] pc_address_in,
    input        exti, timi, softi,
//流水线状态（寄存输出）
    output reg   irq_kill,          // 中断受理（本模块自己判的）
//★ S5：前端域冲刷 {异常, 中断, 改PC}。只给 pc / icache / idu 的输入 lane。
//  A（判定拍改向）只在这一份里；B（提交口清表）只在上面的 flag_bus 里。
//  异常/中断两份都亮（它们本来就是 A+B 同拍）。mret 两份都亮（要改 PC，也要清表）。
//★ 给【执行单元】的寄存版（v2）。当拍版仍然服务 pc 重定向与前端停手（那两条必须同拍），
//  mdu/alu/lsu 这些"错了也无副作用、由 ROB 的世代校验兜底"的单元改吃这一版：
//  它们的 D 引脚原来挂着「rob 的 trap_fire/head_ok → flag_bus → 单元作废」这条跨模块锥。
//  晚一拍无害：作废晚一拍只是多迭代/多算一拍，结果报告会被 ROB 的 ent_v/世代挡掉。
    output reg exc_kill,
    output reg [4:0] flush_index,
//杀集【含边界那条自己】：只给中断用（边界 = 站里最老那条还没执行的指令，要在 mret 之后重跑）
//★ 冲刷记录的【寄存版】：作废晚一拍做。
//  为什么能晚：pc 改向、前端停手（flag_bus）仍然当拍；而 ROB 的作废、idu 的击杀、lsu 的丢弃
//  都只是在"那条指令真的写回/提交之前"完成即可。实测冲刷之后前端连续 4 拍不往 D 送新指令
//  （tb_reb：DNEW_AT +1..+4 全 0），这一拍本来就是空的。
//  为什么必须晚：当拍那条链是 cont 仲裁(6 级) → rob 边界判定(6 级) → isq 击杀+操作数选择(11 级)，
//  跨三个模块，实测 23 级 / 20.559ns / 84% 是布线。寄存一刀把三跳劈成"cont 一跳"和"rob→isq 一跳"。
//  ★ 只有【ROB 侧】的消费者改吃这一版；flag_bus、pc、bju 的 keep_judge 必须继续吃当拍版
//    （keep_judge 判的就是"冲刷这一拍我正在判的那条还算不算活着"，晚一拍就没有意义了）。
//"上一拍刚冲刷过"：这 1 拍里 Q 里的错路指令可能够到发射，必须挡住（尤其错路 csrrw 会在发射拍锁存 CSR）
//异常报告口 ×3：lane0 / lane1 / lsu。各带各的槽号，直接进 ROB 的三个标记口，本模块不做仲裁。
    output reg        exc0_en,
    output reg [4:0]  exc0_index,
    output reg [3:0]  exc0_cause,
    output reg [31:0] exc0_pc, exc0_tval,
    output reg        exc1_en,
    output reg [4:0]  exc1_index,
    output reg [3:0]  exc1_cause,
    output reg [31:0] exc1_pc, exc1_tval,
    output reg        exception_selected_enable,
    output reg [4:0]  exception_selected_index,
    output reg [3:0]  exception_selected_cause,
    output reg [31:0] exception_selected_pc, exception_selected_trap_value,
// mret 搬到提交口：判定拍不再报标签（标签随分配从 idu 的译码位进 ROB），
//  队头那一拍由 ROB 拉 mret_valid 回来。落点也不从这里出 —— pc 直接用 csr 的 mepc 锁存器。
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
    reg exec, flush_control_jump;
    reg flush_window;
    reg redirect_pending;

//重定向登记与异常窗口
    reg [1:0] redirect_kind;
//mret 的资格版：本拍有一条 mret 已经够资格。只在本模块内用（喂 csr 的 exc_irq_ret 闸门）。
    reg exception_interrupt_return_ok;
    // redirect_index 已删：它只服务"中断受理记录的撤销判据"，而那条判据 = older(flush_index, 记录时的队头)。
    // 现在所有改向（分支 / 陷阱 / mret / 中断）都在 ROB 队头发起 ⇒ flush_index 恒 = 队头，
    // 而记录时的队头不可能比现在的队头更年轻 ⇒ older() 恒 0，这个寄存器与那支索引锥都是死的。
    // exc_hold 已删

//异常仲裁
    reg flush_older, flush_older_decode, exception_gated, exception_decode, exception_load_store_decode, exception_load_store_select;
    reg load_store_older_decode;
//（原来的 exception_pc_in_delayed 已删：访存非对齐的 pc 改由 lsu 与故障同拍锁存）

//csr 例化的输出（对齐后送端口）
    reg [31:0] isr_addr2_i, mcause_judged, csr_data_combinational_judged;
    reg exc_irq_act_i, exc_irq_processing_i;
    reg [31:0] iret_addr2_i;
    
    wire exc_irq_ret_i;
    assign exc_irq_ret_i = exception_interrupt_return_ok;

//其它
    reg [1:0] interrupt_request_timer;

//===============================================================
// 控制位译码（本模块最前）
//===============================================================
    always @(*) begin
        irq_kill = exc_irq || exception_interrupt_action;
        flush_control_jump  = redir_backend_0;   // S3：提交口改向（原 bju 判定拍）
        if (irq_kill)
            flush_index = interrupt_head_index_in;
        else
// P4：改向边界是 ROB 自己的队头，由 ROB 内部解析（rob.flush_index_eff）。
// cont 只报"异常/中断候选"；flush_index 的下游消费者（idu / isq / rob）
// 只在 flush_inclusive=1（中断）时使用它，而中断永远不走改向这条路 ⇒ 改向这一档不必有值。
            flush_index = 4'd0;
        if (reset_registered) begin
            irq_kill  = 1'b0;
            flush_control_jump = 1'b0;
        end
    end

//流水线冲刷位（三条冲刷源的或）
    always @(*) begin
        flush_window = exc_kill | irq_kill | flush_control_jump;
    end

    always @(posedge clk) begin
        reset_registered <= rst;
    end

//===============================================================
// 重定向登记（原 pc.v 的 redirect_kind 状态机搬来）
//===============================================================

    always @(*) begin
        redirect_pending     = (redirect_kind != 2'd0);
        redirect_go       = redirect_pending & ~reset_registered;
        redirect_target   = isr_addr2_i;   //只剩 1(陷阱) / 3(中断受理) 两档，落点都是 mtvec
        flush_pc_redirect = redirect_go | (redir_frontend & ~exc_kill);
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            redirect_kind <= 2'd0;
        end
        else if (redirect_go)
            redirect_kind <= 2'd0;
        else if (rob_trap)
            redirect_kind <= 2'd1;
        else if (exception_interrupt_action)
            redirect_kind <= 2'd3;
        else if (flush_pc_redirect && (redirect_kind != 2'd0) && redir_backend_0)
            redirect_kind <= 2'd0;
    end

//===============================================================
// 异常仲裁（原 trap_unit）
//===============================================================

    always @(*) begin
        exception_interrupt_return_ok = (exc_irq_ret_q_0 | exc_irq_ret_q_1)
                       & ~(exc_tgt_mis_0 | exc_tgt_mis_1 | flush_control_jump
                         | exc_ecall_q_0 | exc_ebreak_q_0 | exc_illegal_q_0
                         | exc_ecall_q_1 | exc_ebreak_q_1 | exc_illegal_q_1
                         | exception_load_store_misalign_in);
    end

    always @(*) begin
        flush_older = flush_control_jump;
    end

    always @(*) begin
        exc_kill = rob_trap;
    end

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

    reg exception_interrupt_enable_registered, exception_interrupt_enable_post_registered, exception_external_interrupt_enable, exception_timer_interrupt_enable, exception_software_interrupt_enable;
    reg exception_external_interrupt_pending, exception_timer_interrupt_pending, exception_software_interrupt_pending, exception_interrupt_process, exception_global_pending;
    reg [31:0] isr_addr_reg1, isr_addr_reg2, mcause_registered, mcycle_registered, minstret_registered, mscratch_registered, mtval_registered;
    reg [31:0] interrupt_return_address_1;

//输入合流（按"上层不运算"下放至此）：中断闸门
    reg exception_interrupt_gate;
    always @(*) begin
        exception_interrupt_gate = (interrupt_request_timer != 2'd0);
    end

//CSR 读表写两份（执行级一份、提交口一份），不是偷懒：
//  ★ 本仓实测过，iverilog 的 @* 不把【函数体内读的信号】算进敏感表（见记忆
//    iverilog-function-body-not-in-sensitivity）。写成 function 再在两个 always @(*) 里调用，
//    那些 CSR 寄存器变化时两个读口都不会重算 ⇒ 仿真读到陈旧值。所以只能逐项展开。

//读-改-写（提交拍）：function3[1:0] = 01 写 / 10 置位 / 11 清位（[2] 是立即数形式，二选一已在源上做过）
    reg [31:0] csr_commit_read;
    reg [31:0] csr_write_intent;
    always @(*) begin
        case (csr_commit_address)
        12'h300: csr_commit_read = {19'd0, 2'b11, 3'd0, exception_interrupt_enable_post_registered, 3'd0, exception_interrupt_enable_registered, 3'd0};
        12'h301: csr_commit_read = 32'h40001100;
        12'h304: csr_commit_read = {20'd0, exception_external_interrupt_enable, 3'd0, exception_timer_interrupt_enable, 3'd0, exception_software_interrupt_enable, 3'd0};
        12'h305: csr_commit_read = isr_addr_reg1;
        12'h340: csr_commit_read = mscratch_registered;
        12'h341: csr_commit_read = interrupt_return_address_1;
        12'h342: csr_commit_read = mcause_registered;
        12'h343: csr_commit_read = mtval_registered;
        12'h344: csr_commit_read = {20'd0, exception_external_interrupt_pending, 3'd0, exception_timer_interrupt_pending, 3'd0, exception_software_interrupt_pending, 3'd0};
        12'hB00: csr_commit_read = mcycle_registered;
        12'hB02: csr_commit_read = minstret_registered;
        12'hF14: csr_commit_read = 32'd0;
        default: csr_commit_read = 32'd0;
        endcase
        case (csr_commit_function3[1:0])
        2'b01: csr_write_intent = csr_commit_source;
        2'b10: csr_write_intent = csr_commit_source | csr_commit_read;
        2'b11: csr_write_intent = csr_commit_read & ~csr_commit_source;
        default: csr_write_intent = csr_commit_read;
        endcase
    end

//即将锁存的写入值（与"意图写入值" csr_write_intent 区分开）：
//mtvec 的 MODE[1:0] 是 WARL，本核只实现 Direct ⇒ 写入时把低 2 位规范化为 0（BASE 保持 4 字节对齐）。
//★ 写后读旁路必须用这个值：旁路前递的是"将要写入的值"，一 masking 就会前递未屏蔽的值
//（与 misa 那处同类 —— 旁路前递的必须等于真正锁存的值）。
    reg [31:0] csr_write_value;
    always @(*) begin
        csr_write_value = csr_write_intent;
        if (csr_commit_address == 12'h305)
            csr_write_value = {csr_write_intent[31:2], 2'b00};
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            exception_interrupt_enable_registered <= 1'b0;
            exception_interrupt_enable_post_registered <= 1'b0;
            isr_addr_reg1 <= 32'd0;
            isr_addr_reg2 <= 32'd0;
            interrupt_return_address_1 <= 32'd0;
            iret_addr2_i <= 32'd0;
            mcause_registered <= 32'd0;
            mscratch_registered <= 32'd0;
            mtval_registered <= 32'd0;
            exception_external_interrupt_enable <= 1'b0;
            exception_timer_interrupt_enable <= 1'b0;
            exception_software_interrupt_enable <= 1'b0;
            exception_software_interrupt_pending <= 1'b0;
            exception_interrupt_process <= 1'b0;
        end
//默认情况中断地址/使能/状态寄存器保持
        else begin
            exception_interrupt_enable_registered <= exception_interrupt_enable_registered;
            exception_interrupt_enable_post_registered <= exception_interrupt_enable_post_registered;
            isr_addr_reg1 <= isr_addr_reg1;
            isr_addr_reg2 <= isr_addr_reg2;
            mcause_registered <= mcause_registered;
            mscratch_registered <= mscratch_registered;
            mtval_registered <= mtval_registered;
            exception_external_interrupt_enable <= exception_external_interrupt_enable;
            exception_timer_interrupt_enable <= exception_timer_interrupt_enable;
            exception_software_interrupt_enable <= exception_software_interrupt_enable;
            exception_interrupt_process <= exception_interrupt_process;
//根据不同的csr写地址写入不同的csr寄存器
//★ 提交口：valid / 地址 / 值都是 ROB 打完拍的那一组 ⇒ 不看停顿、也不看当拍冲刷 ——
//  这条指令在上一拍已经退休，它的写必须落地；同拍发生的任何冲刷都属于比它年轻的项。
            if (csr_commit_valid) begin
                case (csr_commit_address)
//全局使能设定
                12'h300: begin
                    exception_interrupt_enable_registered <= csr_write_value[3];
                    exception_interrupt_enable_post_registered <= csr_write_value[7];
                end
//三类型中断分别使能：规范位序 bit3=MSIE(软件)、bit7=MTIE(定时器)、bit11=MEIE(外部)
//★ 本核原来把外部/软件写反了（eirq@3、sirq@11），与自家 mcause_judged 的中断码（外部=11/定时器=7/
//  软件=3，规范）自相矛盾 ⇒ 按规范对调。定时器那条本来就对。
                12'h304: begin
                    exception_external_interrupt_enable <= csr_write_value[11];
                    exception_timer_interrupt_enable <= csr_write_value[7];
                    exception_software_interrupt_enable <= csr_write_value[3];
                end
//isr跳转目标设定
                12'h305: begin
                    isr_addr_reg1 <= csr_write_value;
                    isr_addr_reg2 <= csr_write_value;
                end
                12'h340: begin
                    mscratch_registered <= csr_write_value;
                end
//isr返回目标设定
                12'h341: begin
                    interrupt_return_address_1 <= {csr_write_value[31:2], 2'b00};
                    iret_addr2_i <= {csr_write_value[31:2], 2'b00};
                end
                12'h342: begin
//中断/异常原因寄存器
                    mcause_registered <= csr_write_value;
                end
                12'h343: begin
                    mtval_registered <= csr_write_value;
                end
                12'h344: begin
//软件中断挂起：按规范在 bit3（MSIP）；写 1 清挂起
                    if (csr_write_value[3])
                        exception_software_interrupt_pending <= 1'b0;
                end
                endcase
            end
//异常【检测】那一拍：只做"关中断 + 置受理态"。真正的锁存等 ROB 把故障项退到队头（下面那条）。
//★ 为什么不能在这里锁 mepc/mcause_judged/mtval：先报出来的异常未必是最老的异常 —— 更老的指令可能
//  还在 lsu 队列里没报（lsu 入口才判非对齐，队列里的那条要到下一拍才报）。检测拍就锁，会锁成
//  "年轻的那条"，而 ROB 是按老的那条交付的 ⇒ 两个侧面对不上。改为按 ROB 的交付口锁，才精确。
            if (exc_kill) begin
                exception_interrupt_enable_post_registered <= exception_interrupt_enable_registered;
                exception_interrupt_enable_registered <= 1'b0;
                exception_interrupt_process <= 1'b1;
            end
//非isr状态下触发中断，保存上下文并使能受理信号
//  mepc 取 pc_address_in - 4：cpu_top 把 pc_address_in 接的是 mid_decoder 的 aux_addr_2
//  （那条指令"地址 + 4"），也就是【正要进发射级、还没被发出去】的那一条 = 中断返回点。
//  旧写法是"取指 PC 减一个气泡计数"，ROB 之后前端在等 ROB 排空期间还在取指，那个计数已对不上。
            else if (exc_irq_act_i && !exception_interrupt_process) begin
                interrupt_return_address_1 <= pc_address_in - 32'd4;
                iret_addr2_i <= pc_address_in - 32'd4;
                if (exception_external_interrupt_enable && exception_external_interrupt_pending) begin
                    mcause_registered <= 32'h8000000B;
                end
                else if (exception_software_interrupt_enable && exception_software_interrupt_pending) begin
                    mcause_registered <= 32'h80000003;
                end
                else if (exception_timer_interrupt_enable && exception_timer_interrupt_pending) begin
                    mcause_registered <= 32'h80000007;
                end
                exception_interrupt_enable_post_registered <= exception_interrupt_enable_registered;
                exception_interrupt_enable_registered <= 1'b0;
                exception_interrupt_process <= 1'b1;
            end
//异常【交付】：ROB 的队头是那条故障指令（比它老的都退完了、比它年轻的已作废）⇒ 这一拍锁上下文。
//  与上面两条不冲突：受理态（irq_process）一置起，中断那条就不可能再进来。
            if (rob_trap) begin
                interrupt_return_address_1 <= rob_trap_pc;
                iret_addr2_i <= rob_trap_pc;
                mcause_registered <= {28'd0, rob_trap_cause};
                mtval_registered <= rob_trap_tval;
            end
//isr返回（目前只支持机器模式）
            if (mret_valid) begin
                exception_interrupt_enable_registered <= exception_interrupt_enable_post_registered;
                exception_interrupt_enable_post_registered <= 1'b1;
                exception_interrupt_process <= 1'b0;
            end
        end
    end

//指令退役逻辑，主要用于调试和性能测试
//★ minstret 数的必须是【真退休条数】：原来拿"本拍既未冲刷也未停顿"当退休、每拍最多 +1 ⇒
//  它数的是"没停顿的拍数"，与条数无关（实测 CoreMark：653614 拍读出 472257，看着像 IPC 0.722，
//  其实是非停顿拍占比；真退休只有 330916 条 ⇒ IPC 0.455）。ROB 一拍最多退 2 条，
//  唯一的真口径是【队头指针这一拍推进了几格】。
    reg [1:0] retire_count;
    always @(*) retire_count = rob_ret_step;
    always @(posedge clk) begin
        if (reset_registered) begin
            mcycle_registered <= 32'd0;
            minstret_registered <= 32'd0;
        end
        else begin
            mcycle_registered <= mcycle_registered + 32'd1;
            minstret_registered <= minstret_registered + {30'd0, retire_count};
        end
    end

//csr读操作逻辑
    always @(*) begin
        if (reset_registered) begin
            exception_global_pending = 1'b0;
            exc_irq_act_i = 1'b0;
            exc_irq_processing_i = 1'b0;
            mcause_judged = 32'd0;
            isr_addr2_i = 32'd0;
            exception_timer_interrupt_pending = 1'b0;
            exception_external_interrupt_pending = 1'b0;
        end
        else begin
            exception_timer_interrupt_pending = timi;
            exception_external_interrupt_pending = exti;
            exception_global_pending = (exception_external_interrupt_pending && exception_external_interrupt_enable) | (exception_timer_interrupt_pending && exception_timer_interrupt_enable) | (exception_software_interrupt_pending && exception_software_interrupt_enable);
//★ 同拍只要有任何异常在交付路径上，中断一律让位：exc_kill 已经把
//  ecall/ebreak/illegal（经 bju 输入级）与访存非对齐（经 lsu 寄存拍）全含进来，
//  所以不必在这里逐个列 —— 靠 else-if 优先级兜正确性是脆的。
//★ S3：这里原本用 bju 的判定拍三根（jump_link_register_failure|br2|br3）表达"本拍有一次改向"——
//  旧设计判定拍就是改向拍，成立；改向搬到提交拍后它们先到、改向后到 ⇒
//  判定拍白挡一次中断，真正改向那一拍反倒没人挡（中断与分支改向可能同拍打架）。
//  改成用【真正改向那一拍的事件】flush_control_in[0]（= 提交口改向）。
            exc_irq_act_i = exception_interrupt_enable_registered && exception_global_pending && !exception_interrupt_process && !exception_interrupt_gate &&
                          !(exc_irq_ret_i | redir_backend_0) && !exc_kill && interrupt_head_valid;
            exc_irq_processing_i = exception_interrupt_process;
            mcause_judged = mcause_registered;
            isr_addr2_i = isr_addr_reg2;
        end
    end

//给 alu 的组合读口：地址取本拍要发的 csr_address（= 发射拍那一条的地址）。不做写后读旁路 ——
//每条 csr 指令各占一个发射拍，前一条的写在前一沿就已落进寄存器，本条组合读到的就是它。
    always @(*) begin
        case (csr_address)
        12'h300: csr_data_combinational_judged = {19'd0, 2'b11, 3'd0, exception_interrupt_enable_post_registered, 3'd0, exception_interrupt_enable_registered, 3'd0};
        12'h301: csr_data_combinational_judged = 32'h40001100;
        12'h304: csr_data_combinational_judged = {20'd0, exception_external_interrupt_enable, 3'd0, exception_timer_interrupt_enable, 3'd0, exception_software_interrupt_enable, 3'd0};
        12'h305: csr_data_combinational_judged = isr_addr_reg1;
        12'h340: csr_data_combinational_judged = mscratch_registered;
        12'h341: csr_data_combinational_judged = interrupt_return_address_1;
        12'h342: csr_data_combinational_judged = mcause_registered;
        12'h343: csr_data_combinational_judged = mtval_registered;
        12'h344: csr_data_combinational_judged = {20'd0, exception_external_interrupt_pending, 3'd0, exception_timer_interrupt_pending, 3'd0, exception_software_interrupt_pending, 3'd0};
        12'hB00: csr_data_combinational_judged = mcycle_registered;
        12'hB02: csr_data_combinational_judged = minstret_registered;
        12'hF14: csr_data_combinational_judged = 32'd0;
        default: csr_data_combinational_judged = 32'd0;
        endcase
    end


    always @(*) begin
        csr_data_combinational = csr_data_combinational_judged;
        interrupt_service_address_2 = isr_addr2_i;
        mcause = mcause_judged;
        exception_interrupt_action = exc_irq_act_i;
        exception_interrupt_processing = exc_irq_processing_i;
        exc_irq = exc_irq_act_i;
        interrupt_return_address_2 = iret_addr2_i;
    end

//mret 标签不再从这里出：它是 idu 的译码位，随分配直接写进 ROB 项（rob: allocation_mret_*）。
//  原来这里报的是 bju 判定级的 1 拍脉冲 + 槽号，会被同一窗口里别人的异常标志压掉（seed36）。

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
