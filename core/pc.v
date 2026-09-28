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
    input br1, exc_irq, exc_irq_ret, exc_ecall,
    input jal, pre_jalr, btb_hit,
    input [13:0] flag_bus,
    input [31:0] jp_target, offset_jal2, offset_jalr2,
    input [31:0] offset_beq2, isr_addr2, isr_ret_addr2,
    input rob_empty,
    output reg [31:0] pc_addr, aux_addr,
//停顿源：重定向排队中（等 ROB 排空）/ 落点后多压一拍（stall_pc_redir）；
//冲刷源：本拍把 pc 落到新目标（flush_pc_redir，icache 用它挡掉陈旧交付）。
//ROB 满原来单独接端口，现在吃 flag_bus[9]（源名 stall_rob_full），见下面的译码块。
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
//            stall_lsu_haz, stall_lsu_unload, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_dcache_miss, stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 controller 里已逐条分开）。ROB 满原来单独接端口，现在吃 flag_bus[9]。
    reg exec, flush_w, stall_w, flush_jump_w;
    reg exc_w;
    always @(*) begin
        flush_jump_w = flag_bus[11] & ~flag_bus[13];
        exc_w        = flag_bus[13];
        flush_w      = flag_bus[13] | flag_bus[12] | flag_bus[11];
        stall_w      = (flag_bus[9] | flag_bus[8] | flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4]
                      | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec         = flag_bus[10];
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
        redir_req_trap = exc_irq | exc_ecall | exc_w;
        redir_req_ret  = exc_irq_ret;
        redir_go        = (redir_kind != 2'd0) && rob_empty;
        flush_pc_redir  = redir_go;
        stall_pc_redir  = (redir_kind != 2'd0) | redir_hold;
    end
    always @(posedge clk) begin
        if (rst_q) redir_hold <= 1'b0;
        else if (redir_go) redir_hold <= 1'b1;
        else redir_hold <= 1'b0;
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
            else if (!flush_w && !stall_w) begin
                if (br1) begin
                    pc_addr <= pc_addr + offset_beq2 - 4'd4;
                    aux_addr <= aux_addr + offset_beq2 - 4'd4;
                end
                else if (jal) begin
                    pc_addr <= pc_addr + offset_jal2 - 4'd4;
                    aux_addr <= pc_addr + offset_jal2 - 4'd4;
                end
                else if (jalr) begin
                    pc_addr <= offset_jalr2;
                    aux_addr <= offset_jalr2;
                end
                else begin
                    pc_addr <= pc_addr + 4'd4;
                    aux_addr <= pc_addr + 4'd4;
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
