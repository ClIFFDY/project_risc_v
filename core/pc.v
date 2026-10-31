`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: pc
// Description:
//   ★★ 地址只有一个概念，就是 pc_registered。本模块只做两件事：
//     ① 选 next（改向 > 落点表 > 顺序 +8）；
//     ② 在【整个取指级前进】的那一拍把它装进 pc_registered。
//
//   约束（用户定的架构）：
//     · 被门控的寄存器只有四组：pc / 落点表输出 / tagram(R0) / 交付寄存器。
//       pc 是其中唯一【会被 redir 改值】的一组。
//     · redir 在其它模块里只做杀/控（作废、清、停一拍），绝不用它改同伴信号的值。
//     · "这一拍前进吗"全核只有一份，就是 icache 出的 advance —— 本模块不再自己解 flag_bus，
//       也不再合成第二份"走不走"。
//
//   为什么不再有组合改向旁路（原 `pc_address_0 = (redir|flush_jump_w) ? next : pc_registered`）：
//     旁路与 `pc_registered <= next` 同时存在时，改向目标会【连续两拍】出现在口上（C 拍旁路、
//     C+1 拍 pc_registered），R0 就装两次 ⇒ 同一对交付两次。把旁路去掉，代价是改向进 icache 晚一拍，
//     换来的是"呈现的地址 == pc_registered"这一个概念，以及 lp_ok / tk_drop / pc_pred_v / 影子寄存器整套删除。
//
//   为什么不再有 lp_ok / tk_drop：
//     落点表的输出寄存器和 R0 是【同一个沿、同一个使能、同一个 pc_addr】装进去的
//     ⇒ 「表项描述的正是 R0 里那一对」由结构保证，不由拍数推断（不变量 I1）。
//     "上一对是预测跳、这一对是顺延错路"由 R0 组携带的 r0_wrong_path 表示（跟着这一对走），
//     所以 take_ok = r0_valid & ~r0_wrong_path 两个位同属一组、同拍，不可能错相位。
//////////////////////////////////////////////////////////////////////////////////


module pc(
    input clk, rst,
//★★ 唯一前进判据（icache 出）：整个取指级这一拍前进一格。
    input       advance,
//★ 外部改向落地这一拍（cont 出）= redirect_go | (flush_con_jump & ~flush_con_exc)。
//  本拍 pc_registered 装改向目标；其它三组在 advance=0 那拍原地不动。
    input       flush_pc_redirect,
    input       redirect_go,
    input [31:0] redir_target,
    input [31:0] jump_target,
//★ R0 组的两个状态位，与 btb_take/btb_tgt 同一拍、同一对：
//  r0_valid  = 这一对的地址/表项有效；r0_wrong_path = 这一对是预测跳之后顺延取回来的错路。
    input       r0_valid, r0_wrong_path,
//落点表输出寄存器（与 R0 同一对）
    input        btb_take_0, btb_take_1,
    input [31:0] btb_target_0,  btb_target_1,
    output [31:0] pc_address_0, pc_address_1,
//本拍 pc 到底照做了哪一条 lane 的落点（随指令记进队列的"预测跳"必须就是这两位）
    output        pc_take0, pc_take1
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    reg  [31:0] pc_registered;

//==============================================================================
//  落点表资格
//==============================================================================
//★ take_ok 的两位都来自 R0 组（同沿、同门、同一对）：
//   r0_valid = 0：改向刚落地那一拍，R0 里压的还是改向前那一对，表项描述的不是它 ⇒ 不许用；
//   r0_wrong_path = 1：预测跳之后顺延取回来的错路对，它的表项更不许用（否则二次改向，落点整段被跳过）。
    wire take_ok   = r0_valid & ~r0_wrong_path;
    wire select_take_0 = take_ok &  btb_take_0;
    wire select_take_1 = take_ok & ~btb_take_0 & btb_take_1;

//==============================================================================
//  下一个地址（值的选择；装不装由下面的使能决定）
//==============================================================================
//改向优先于落点表：改向那一拍 pc 走的是改向，落点表就算命中也不算"照做"。
//  pc_take 必须说真话：它被 icache 记进队列当"这一对的预测"，记错了 bju 会拿别人的预测判。
    assign pc_take0 = select_take_0 & ~flush_pc_redirect;
    assign pc_take1 = select_take_1 & ~flush_pc_redirect;

    reg  [31:0] next;
    always @(*) begin
        if (redirect_go)            next = redir_target;
        else if (flush_pc_redirect) next = jump_target;
        else if (select_take_0)        next = btb_target_0;
        else if (select_take_1)        next = btb_target_1;
        else                     next = pc_registered + 32'd8;
    end

//==============================================================================
//  地址寄存器：唯一使能 = 取指级前进 | 改向落地
//==============================================================================
//★ 只有这两种原因：advance（整个前端前进一格）与 flush_pc_redirect（约束 3：redir 只改 pc 的值）。
//  落点表命中不是独立原因 —— 它只是替 next 选了一个值，走不走仍由 advance 决定。
    always @(posedge clk) begin
        if (rst_q)                     pc_registered <= 32'd0;
        else if (advance | flush_pc_redirect) pc_registered <= next;
    end

//★ pc_address_1 恒为 pc_address_0 + 4，由这两行赋值保证。icache 随指令把地址一起交付
//  （inst_addr0/1），所以任何时候都说得清口上是哪一条。
    assign pc_address_0 = pc_registered;
    assign pc_address_1 = pc_registered + 32'd4;

endmodule
