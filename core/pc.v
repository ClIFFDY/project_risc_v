`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: pc
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


module pc(
    input clk, rst,
    input br1, exc_irq, exc_irq_ret, exc_mark, trap_fire,
    input jal, pre_jalr, pre_jalr_l1, btb_hit, btb_hit_l1,
    input br1_l1,
//bpu 那块服务 br1/jal 的 btb 命中：命中 ⇒ 目标 inst 当拍由 btb 交付给 idu1
//⇒ pc 落 T+4；未命中 ⇒ pc 落 T，让 icache 下一拍自己去取目标（差一拍，只有 jal 冷启动吃这一拍）。
    input bti_hit,
    input [7:0] flag_bus,
    input [31:0] jp_target, offset_jal2, offset_jalr2,
    input [31:0] offset_beq2, isr_addr2, isr_ret_addr2,
    input [31:0] offset_beq_l1, offset_jalr_l1,
    input rob_empty,
//取指队列满（iffu 的 full）：与 stall_pc_redir 同性质的前端停留 —— 队满时 pc 不许再走，
//否则已经交给 icache 的那条会被丢弃
    input fifo_full,
    input pair_ok,
    input fch_jal_l1,
    output reg [31:0] pc_addr, aux_addr,
//停顿源：重定向排队中（等 ROB 排空）/ 落点后多压一拍（stall_pc_redir）；
//冲刷源：本拍把 pc 落到新目标（flush_pc_redir，icache 用它挡掉陈旧交付）。
//ROB 满原来单独接端口，现在吃 flag_bus[3]（源名 stall_rob_full），见下面的译码块。
    output reg stall_pc_redir,
    output reg flush_pc_redir
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
//重定向排队的寄存器（声明必须在所有行为之前）
    reg [1:0] redir_kind;
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 cont 里已逐条分开）。ROB 满原来单独接端口，现在吃 flag_bus[3]。
    reg exec, flush_w, flush_jump_w, stall_icache_miss;
    reg exc_w;
    always @(*) begin
        exc_w        = flag_bus[7];
        flush_jump_w = flag_bus[5] & ~exc_w;
        flush_w      = exc_w | flag_bus[6] | flag_bus[5];
        stall_icache_miss = flag_bus[1];
        exec         = flag_bus[4];
    end


    reg [31:0] jal_base;
    always @(*) begin
        jal_base = fch_jal_l1 ? pc_addr : aux_addr;
    end

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）
    reg jalr, jalr_l1;
    always @(*) jalr = pre_jalr & btb_hit;
    always @(*) jalr_l1 = pre_jalr_l1 & btb_hit_l1;

    reg [31:0] br1_l1_base;
    always @(*) br1_l1_base = aux_addr + 32'd4;

//程序计数器，传递取指地址
//重定向排队：判定那一拍照旧【先冲刷更年轻的】，但真正的跳转要等 ROB 排空 —— 这样
//"比它老的全退完（可见）、比它年轻的已冲掉（无效果）"是结构保证，不靠推理。
    reg       redir_go;
    reg       redir_req_trap;
    reg       redir_req_ret;
    reg       redir_revoke;
    reg       redir_trap_seen;
//redir_hold：跳转已落地、再多压一拍。icache 是"这一拍读、下一拍出"，落点那条指令要到
//落地后第二拍才交得出来 —— 这一拍不压住，交付口就会把"跳转前那次取指"的陈旧值喂进流水线
//（实测 exc_ecall：凭空多执行一条 A+16 的 sw）。
    reg       redir_hold;
    always @(*) begin
//★ 武装资格必须与【交付资格】同源（这里曾经用 exc_ecall 裸载荷 + exc_w）：
//  exc_ecall 是 post_decoder 的裸寄存器，交付侧却要 & ~flush_older（经 exc_gated）；
//  exc_w（flush_con_exc）与标记资格 exc_mark 差一项 flush_bju_exc & bju_older_in —— 那一支永不交付。
//  拿它们武装 ⇒ 队列里会留下带陈旧落点（mtvec/mepc 或复位 0）的排队项，等下一次 rob_empty 无条件发射。
//  exc_irq 本身就是「同拍受理并锁 mepc」那根线，天然配对，保留。
//★ 标记装进来的排队项另外还要【等真交付】才许发射（redir_trap_seen）：错路指令的标记会被随后到达的
//  老分支在 ROB 里连项一起灭掉（那一支的 trap_fire 再也不会成立），而排队项没人撤 ⇒ rob_empty 一到
//  就跳 mtvec，处理器在【没有交付】的情况下跑起来，拿 mcause/mepc 的残值（实测：错路非法指令被分支
//  冲掉后返回点偏 4，整核再也出不来）。武装仍在标记拍不动（前端从标记拍起就压住，时序与改前逐拍一致）；
//  等不到交付又排空了的那一项直接【作废】（不带作废会把自己冻死：排队项不走，stall_pc_redir 就常驻）。
        redir_req_trap = exc_irq | exc_mark;
        redir_req_ret  = exc_irq_ret;
        redir_go        = (redir_kind != 2'd0) && rob_empty &&
                          ((redir_kind != 2'd1) || redir_trap_seen);
        redir_revoke    = (redir_kind == 2'd1) && !redir_trap_seen && rob_empty;
        flush_pc_redir  = redir_go;
        stall_pc_redir  = (redir_kind != 2'd0) | redir_hold;
    end
//本次排队项"够不够格发射"：中断装进来的当拍就算够（它那一拍已锁 mepc/mcause）；标记装进来的要等到
//ROB 真的交付过一次（trap_fire）。
    always @(posedge clk) begin
        if (rst_q)
            redir_trap_seen <= 1'b0;
        else if (redir_go || redir_revoke)
            redir_trap_seen <= 1'b0;
        else if (trap_fire)
            redir_trap_seen <= 1'b1;
        else if (redir_req_trap)
            redir_trap_seen <= exc_irq;
        else
            redir_trap_seen <= redir_trap_seen;
    end
    always @(posedge clk) begin
        if (rst_q)
            redir_hold <= 1'b0;
        else if (redir_go)
            redir_hold <= 1'b1;
        else
            redir_hold <= 1'b0;
    end
    always @(posedge clk) begin
        if (rst_q) begin
            redir_kind <= 2'd0;
        end
        else if (redir_go) begin
            redir_kind <= 2'd0;
        end
        else if (redir_req_trap) begin
            redir_kind <= 2'd1;
        end
        else if (redir_req_ret) begin
            redir_kind <= 2'd2;
        end
        else if (redir_revoke) begin
            redir_kind <= 2'd0;
        end
        else begin
            redir_kind <= redir_kind;
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            pc_addr <= 32'd4;
            aux_addr <= 32'd4;
        end
        else if (exec) begin
            if (redir_go && (redir_kind == 2'd1)) begin
                pc_addr <= isr_addr2;
                aux_addr <= isr_addr2;
            end
            else if (redir_go && (redir_kind == 2'd2)) begin
                pc_addr <= isr_ret_addr2;
                aux_addr <= isr_ret_addr2;
            end
            else if (!flush_w && !stall_pc_redir && !stall_icache_miss && !fifo_full) begin
//jal/br1 的改向分两档（"差一拍"就在这儿）：命中的那一拍目标 inst 已由 bpu 的 btb
//交付给 idu1，所以 pc 落**目标的下一个地址**（T+4）；未命中的那一拍没有 inst 可交付，
//pc 落**目标地址本身**（T），由 icache 下一拍去取目标 —— 比命中晚一拍。
//两个表达式都只落在 pc/aux 的 D 端（普通寄存器），不是 BRAM 地址脚 ⇒ 不进关键路径。
                if (br1) begin
                    if (bti_hit) begin
                        pc_addr <= aux_addr + offset_beq2;
                        aux_addr <= aux_addr + offset_beq2;
                    end
                    else begin
                        pc_addr <= aux_addr + offset_beq2 - 32'd4;
                        aux_addr <= aux_addr + offset_beq2;
                    end
                end
                else if (jal) begin
                    if (bti_hit) begin
                        pc_addr <= jal_base + offset_jal2;
                        aux_addr <= jal_base + offset_jal2;
                    end
                    else begin
                        pc_addr <= jal_base + offset_jal2 - 32'd4;
                        aux_addr <= jal_base + offset_jal2;
                    end
                end
                else if (jalr) begin
                    pc_addr <= offset_jalr2;
                    aux_addr <= offset_jalr2 + 32'd4;
                end
                else if (br1_l1) begin
                    pc_addr <= br1_l1_base + offset_beq_l1 - 32'd4;
                    aux_addr <= br1_l1_base + offset_beq_l1;
                end
                else if (jalr_l1) begin
                    pc_addr <= offset_jalr_l1;
                    aux_addr <= offset_jalr_l1 + 32'd4;
                end
                else begin
                    if (pair_ok) begin
                        pc_addr <= pc_addr + 32'd8;
                        aux_addr <= pc_addr + 32'd4;
                    end
                    else begin
                        pc_addr <= pc_addr + 4'd4;
                        aux_addr <= pc_addr + 4'd4;
                    end
                end
            end
            else if (flush_w) begin
//跳转类冲刷的落点已在 decoder 的判定延迟拍里算清（jp_target）：br2 = 分支地址+immB、
//br3 = 分支地址+4、jalr_fail = jalr 真目标。三种不再在这里区分，也不再各自做 32 位加法。
//icache 不再当拍跳 jalr 目标，改成跟着 pc 走：pc 落在目标上，icache 下一拍取目标指令，
//一拍后交付，所以落点一律不再 +4。
                if (flush_jump_w) begin
                    pc_addr <= jp_target;
                    aux_addr <= jp_target;
                end
//irq/trap/exc 的落点不再在这里 —— 它们改由上面的"排队"在 ROB 排空后才跳（精确交付）
            end
            else begin
                pc_addr <= pc_addr;
                aux_addr <= aux_addr;
            end
        end
    end

endmodule
