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
//★★ 背压：icache 缺失回放 / 取指队列满 —— 地址不动。
    input       icache_miss, fifo_full, busy, //前端背压
    input       redir_frontend, //前端改向（分支 / mret / 陷阱，落点由 cpu_top 按 cause 选好）
    input       irq_kill,       //中断受理那一拍：清前后端 + pc 当拍取 mtvec
    input [31:0] irq_target,
    input [31:0] jump_target,
    input        btb_take_0, btb_take_1,
    input [31:0] btb_target_0,  btb_target_1,
//给 icache：下一条要执行的指令的地址（可能不是 8B 对齐）
    output [31:0] pc_address_0,
//给 bpu：这一块里两个取指地址（块基址 + 0/4）
    output [31:0] pc_block_0, pc_block_1,
//给 bpu：块内起始偏移（比它老的那条不该参与仲裁）
    output        pc_offset
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;
    reg  [31:0] pc_registered;

//==============================================================================
//  块内选一条：序号小的优先（bpu 已仲裁过，这里按序号串一遍）
//==============================================================================
    reg  [31:0] next;
    always @(*) begin
        if (irq_kill)               next = irq_target;
        else if (redir_frontend)    next = jump_target;
        else if (btb_take_0)        next = {btb_target_0[31:3] + 1'b1, 3'd0};
        else if (btb_take_1)        next = {btb_target_1[31:3] + 1'b1, 3'd0};
//★ 顺序推进 = 跳过整块：这一拍的 2 条（或从块内偏移起的那条）都已经取回来了。
        else                        next = {pc_registered[31:3] + 1'b1, 3'd0};
    end

//==============================================================================
//  地址寄存器：重定向 > 背压。重定向那一拍换成落点，背压三根按住输出。
//==============================================================================
    always @(posedge clk) begin
        if (rst_q)
            pc_registered <= 32'd0;
        else if (irq_kill | redir_frontend)
            pc_registered <= next;
        else if (~(icache_miss | fifo_full | busy))
            pc_registered <= next;
        else
            pc_registered <= pc_registered;
    end

//★ 这一拍的取指地址：预测跳就吃落点，覆盖 pc_registered。
//  同一个式子写两份 —— _0 只喂 icache，_1 只喂 bpu。
    wire        take_any    = btb_take_0 | btb_take_1;
    wire [31:0] take_target = btb_take_0 ? btb_target_0 : btb_target_1;
    wire [31:0] pc_address_1;
    assign pc_address_0 = take_any ? take_target : pc_registered;
    assign pc_address_1 = take_any ? take_target : pc_registered;

//★ 给 bpu 的是同一块里四个地址 + 块内偏移（偏移用来把比它老的那几条挡在仲裁之外）。
//★ 2026-11-11 取指回 2 宽：块 = 8B，只给 bpu 两个地址。
    assign pc_block_0 = {pc_address_1[31:3], 3'd0};
    assign pc_block_1 = {pc_address_1[31:3], 3'd4};
    assign pc_offset  = pc_address_1[2];

endmodule
