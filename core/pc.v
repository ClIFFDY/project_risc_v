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
    input br1, exc_irq, exc_irq_ret, exc_mark,
    input jal, pre_jalr, btb_hit,
//lane1 的 jal：lane1 的地址 = lane0 + 4 = pc − 4（pc 是"下一个待取地址"）⇒ 落点 T1 = pc + immJ1 − 4。
//★ 与 lane0 那一支不同：lane0 的 jal 命中有 bti 注入、pc 落 T+8（注入把 (T,T+4) 补上了）；
//  lane1 没有注入通路，pc 必须直接落 T1，让取指侧从 T1 起取。
//lane1 的改向（fifo 判完的三类合一）与落点（fifo 里算好，本级只落地址）
    input lane1_redir,
    input [31:0] lane1_target,
//bra_predict 那块服务 br1/jal 的 btb 命中：命中 ⇒ 目标 inst 当拍由 btb 交付给 pre_decoder
//⇒ pc 落 T+4；未命中 ⇒ pc 落 T，让 icache 下一拍自己去取目标（差一拍，只有 jal 冷启动吃这一拍）。
    input bti_hit,
    input [11:0] flag_bus,
    input [31:0] jp_target, offset_jal2, offset_jalr2,
    input [31:0] offset_beq2, isr_addr2, isr_ret_addr2,
    input rob_empty,
//取指队列满：本级与 icache 用【同一个门】冻住（icache 的 req_valid 里也含它）。
//★ 两边必须逐字一致：差一条就是"pc 被按住、交付寄存器却被换掉/被清掉"这类错位。
    input fifo_full,
    output reg [31:0] pc_addr, aux_addr,
//停顿源：重定向排队中（等 ROB 排空）/ 落点后多压一拍（stall_pc_redir）；
//冲刷源：本拍把 pc 落到新目标（flush_pc_redir，icache 用它挡掉陈旧交付）。
//ROB 满原来单独接端口，现在吃 flag_bus[7]（源名 stall_rob_full），见下面的译码块。
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
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 controller 里已逐条分开）。ROB 满原来单独接端口，现在吃 flag_bus[7]。
    reg exec, flush_w, stall_w, flush_jump_w;
    reg exc_w;
    always @(*) begin
        exc_w        = flag_bus[11];
        flush_jump_w = flag_bus[9] & ~exc_w;
        flush_w      = exc_w | flag_bus[10] | flag_bus[9];
//★ 本级在【取指侧】：只吃"重定向排队 + 取指自己 miss"；后端停顿由取指队列吸收。
        stall_w      = (flag_bus[6] | flag_bus[1]) & ~flush_w;
        exec         = flag_bus[8];
    end

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）
    reg jalr;
    always @(*) jalr = pre_jalr & btb_hit;

//程序计数器，传递取指地址
//重定向排队：判定那一拍照旧【先冲刷更年轻的】，但真正的跳转要等 ROB 排空 —— 这样
//"比它老的全退完（可见）、比它年轻的已冲掉（无效果）"是结构保证，不靠推理。
    reg       redir_go;
    reg       redir_req_trap;
    reg       redir_req_ret;
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
        redir_req_trap = exc_irq | exc_mark;
        redir_req_ret  = exc_irq_ret;
        redir_go        = (redir_kind != 2'd0) && rob_empty;
        flush_pc_redir  = redir_go;
        stall_pc_redir  = (redir_kind != 2'd0) | redir_hold;
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
        else begin
            redir_kind <= redir_kind;
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
//★ 复位值是 8 不是 4：pc 现在是"下一个待取地址"、恒定 +8 ⇒ 本拍交付的那个字是 instr(pc − 8)。
//  写 4 的话，自举交接（直接装 instr(0)/instr(4)）之后的第一拍会再取一次 instr(4) —— 重复一条。
            pc_addr <= 32'd8;
            aux_addr <= 32'd8;
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
            else if (!flush_w && !stall_w && !fifo_full) begin
//jal/br1 的改向分两档（"差一拍"就在这儿）：命中的那一拍目标 inst 已由 bra_predict 的 btb
//交付给 pre_decoder，所以 pc 落**目标的下一个地址**（T+4）；未命中的那一拍没有 inst 可交付，
//pc 落**目标地址本身**（T），由 icache 下一拍去取目标 —— 比命中晚一拍。
//两个表达式都只落在 pc/aux 的 D 端（普通寄存器），不是 BRAM 地址脚 ⇒ 不进关键路径。
                if (br1) begin
//命中：bti 直送目标那一对 {T, T+4}（一次占满两个字），所以 pc 落 T+8；
//未命中：交付一条 NOP，pc 落 T 本身，由 icache 下一拍去取目标（差一拍，只有冷启动吃这一拍）。
                    if (bti_hit) begin
                        pc_addr <= pc_addr + offset_beq2;
                        aux_addr <= aux_addr + offset_beq2;
                    end
                    else begin
                        pc_addr <= pc_addr + offset_beq2 - 32'd8;
                        aux_addr <= aux_addr + offset_beq2 - 32'd8;
                    end
                end
                else if (jal) begin
                    if (bti_hit) begin
                        pc_addr <= pc_addr + offset_jal2;
                        aux_addr <= aux_addr + offset_jal2;
                    end
                    else begin
                        pc_addr <= pc_addr + offset_jal2 - 32'd8;
                        aux_addr <= aux_addr + offset_jal2 - 32'd8;
                    end
                end
                else if (jalr) begin
                    pc_addr <= offset_jalr2;
                    aux_addr <= offset_jalr2;
                end
//★ lane1 是 jal：它不在取指侧的 lane0 译码里（那一份只看两个字里的第一个），而后端对普通
//  jal 没有改向出口 ⇒ 不在这儿补就整条漏掉（实测 fence 的 j 5c 落在 lane1：pc 顺着冲过
//  trap_handler、把 mret 当指令执行，整程序重跑）。
//  优先级放在 lane0 的四条之后：lane0 一旦改向，lane1 就是错路，不该再改 pc。
//★ 落点是 `− 4`（不是 +4）：lane1 的指令地址是 pc−4（lane0 是 pc−8），它的落点地址
//  T1 = pc + immJ1 − 4。而 icache 的读【滞后一拍】：交付口第 N 拍上是 pc(N−1) 对应的那一对
//  ⇒ 本拍改向、下一拍生效，于是**下下拍**交付的才是 pc(改向后) 对应的那一对。
//  要让下下拍交付 T1 那一对，pc 就该落 T1 本身 ⇒ `pc + immJ1 − 4`。
//  （改成 +4 实测把开机 bss 清零循环的循环体整个跳过 —— pc 卡在 0x20/0x28 不动。）
                else if (lane1_redir) begin
                    pc_addr <= lane1_target;
                    aux_addr <= lane1_target;
                end
                else begin
//★ 顺序推进【恒定 +8】：一次把两个字吃满，没有别的选项。
//  pc_addr 是"下一个待取地址"，fetch_addr = pc_addr>>2，eff = instr(pc_addr−4)、inst_next = instr(pc_addr)；
//  取指侧【不需要知道这两个字配不配】—— 配不配由队列读侧按条目的类位判，配不上就只弹一条。
//  pc 只被两种事搬走：停顿（stall / 队满）按住不动、改向（上面那几支）落到别处。
                    pc_addr <= pc_addr + 32'd8;
                    aux_addr <= pc_addr + 32'd8;
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
