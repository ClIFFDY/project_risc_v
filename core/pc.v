`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: pc
// Description:
//   ★★ 地址只有一个概念，就是 pc_registered。本模块只做两件事：
//     ① 选 next（改向 > 落点表 > 顺序 +16）；
//     ② 在【整个取指级前进】的那一拍把它装进 pc_registered。
//
//   约束（用户定的架构）：
//     · 被门控的寄存器只有四组：pc / 落点表输出 / tagram(R0) / 交付寄存器。
//       pc 是其中唯一【会被 redir 改值】的一组。
//     · redir 在其它模块里只做杀/控（作废、清、停一拍），绝不用它改同伴信号的值。
//     · "这一拍前进吗"全核只有一份，就是 icache 出的 advance —— 本模块不再自己解 flag_bus，
//       也不再合成第二份"走不走"。
//
//   ★ 2026-11-08：取指改 16B 块。pc_registered 仍然是【下一条要执行的指令】的地址
//     （可以是块内第 1/2/3 条），但一拍把整块 4 条都取回来 ⇒ 顺序推进是 +16 并对齐到块首。
//     块基址与块内偏移都在本模块现算，分别送给 icache 与 bpu。
//
//   为什么不再有组合改向旁路（原 `pc_address_0 = (redir|flush_jump_w) ? next : pc_registered`）：
//     旁路与 `pc_registered <= next` 同时存在时，改向目标会【连续两拍】出现在口上
//     （C 拍旁路、C+1 拍 pc_registered），R0 就装两次 ⇒ 同一块交付两次。把旁路去掉，
//     代价是改向进 icache 晚一拍，换来的是"呈现的地址 == pc_registered"这一个概念。
//////////////////////////////////////////////////////////////////////////////////


module pc(
    input clk, rst,
//★★ 唯一前进判据（icache 出）：整个取指级这一拍前进一格。
    input       advance,
    input       redir_frontend, //前端改向
    input       exc_kill, //异常冲刷
    input       redirect_go, //
    input [31:0] redir_target,
    input [31:0] jump_target,
//★ R0 组的两个状态位，与 btb_take/btb_tgt 同一拍、同一块：
//  r0_valid  = 这一块的地址/表项有效；r0_wrong_path = 这一块是预测跳之后顺延取回来的错路。
    input       r0_valid, r0_wrong_path,
//落点表输出寄存器（与 R0 同一块）。btb_take_0..3 在 bpu 里已按块内序号仲裁过，恒至多一位为 1。
    input        btb_take_0, btb_take_1, btb_take_2, btb_take_3,
    input [31:0] btb_target_0,  btb_target_1, btb_target_2, btb_target_3,
//给 icache：下一条要执行的指令的地址（可能不是 16B 对齐）
    output [31:0] pc_address_0,
//给 bpu：这一块里四个取指地址（块基址 + 0/4/8/12）
    output [31:0] pc_block_0, pc_block_1, pc_block_2, pc_block_3,
//给 bpu：块内起始偏移（比它老的那几条不该参与仲裁）
    output [1:0]  pc_offset,
//本拍 pc 到底照做了哪一条 lane 的落点（随指令记进队列的"预测跳"必须就是这四位）
    output        pc_take0, pc_take1, pc_take2, pc_take3
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
//★ take_ok 的两位都来自 R0 组（同沿、同门、同一块）：
//   r0_valid = 0：改向刚落地那一拍，R0 里压的还是改向前那一块，表项描述的不是它 ⇒ 不许用；
//   r0_wrong_path = 1：预测跳之后顺延取回来的错路块，它的表项更不许用（否则二次改向，落点整段被跳过）。
    wire take_ok = r0_valid & ~r0_wrong_path;
    wire select_take_0 = take_ok & btb_take_0;
    wire select_take_1 = take_ok & ~btb_take_0 & btb_take_1;
    wire select_take_2 = take_ok & ~btb_take_0 & ~btb_take_1 & btb_take_2;
    wire select_take_3 = take_ok & ~btb_take_0 & ~btb_take_1 & ~btb_take_2 & btb_take_3;

//==============================================================================
//  下一个地址（值的选择；装不装由下面的使能决定）
//==============================================================================
//改向优先于落点表：改向那一拍 pc 走的是改向，落点表就算命中也不算"照做"。
//  pc_take 必须说真话：它被 icache 记进队列当"这一块的预测"，记错了 bju 会拿别人的预测判。
    assign pc_take0 = select_take_0 & ~(redirect_go | (redir_frontend & ~exc_kill));
    assign pc_take1 = select_take_1 & ~(redirect_go | (redir_frontend & ~exc_kill));
    assign pc_take2 = select_take_2 & ~(redirect_go | (redir_frontend & ~exc_kill));
    assign pc_take3 = select_take_3 & ~(redirect_go | (redir_frontend & ~exc_kill));

    reg  [31:0] next;
    always @(*) begin
        if (redirect_go)               next = redir_target;
        else if (redir_frontend & ~exc_kill) next = jump_target;
        else if (select_take_0)        next = btb_target_0;
        else if (select_take_1)        next = btb_target_1;
        else if (select_take_2)        next = btb_target_2;
        else if (select_take_3)        next = btb_target_3;
//★ 顺序推进 = 跳过整块：这一拍的 4 条（或从块内偏移起的那几条）都已经取回来了。
        else                           next = {pc_registered[31:4] + 1'b1, 4'd0};
    end

//==============================================================================
//  地址寄存器：唯一使能 = 取指级前进 | 改向落地
//==============================================================================
    always @(posedge clk) begin
        if (rst_q)                     pc_registered <= 32'd0;
        else if (advance | redirect_go | (redir_frontend & ~exc_kill)) pc_registered <= next;
    end

//★ 给 icache 的是【下一条要执行的指令】的地址（可能是块内第 1/2/3 条）。
    assign pc_address_0 = pc_registered;
//★ 给 bpu 的是这一块里四个地址 + 块内偏移（偏移用来把比它老的那几条挡在仲裁之外）。
    assign pc_block_0 = {pc_registered[31:4], 4'd0};
    assign pc_block_1 = {pc_registered[31:4], 4'd4};
    assign pc_block_2 = {pc_registered[31:4], 4'd8};
    assign pc_block_3 = {pc_registered[31:4], 4'd12};
    assign pc_offset  = pc_registered[3:2];

endmodule
