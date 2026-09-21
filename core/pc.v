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
    input br1, br2, br3,
    input jal, pre_jalr, btb_hit, jalr_fail, irq, irq_ret,
    input [4:0] flag_bus,
    input [31:0] offset_jal2, offset_jalr2,
    input [31:0] offset_beq2, isr_addr2, isr_ret_addr2,
    input [31:0] jalr_target_q, beq_off_q2,
    input [31:0] br_addr2,
    output reg [31:0] pc_addr, aux_addr
    );

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[3] | flag_bus[2];
        stall_w = (flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）
    reg jalr;
    always @(*) jalr = pre_jalr & btb_hit;

//程序计数器，传递取指地址
    always @(posedge clk) begin
        if (rst) begin
            pc_addr <= 32'd0;
            aux_addr <= 32'd0;
        end
        else if (exec) begin
            if (!flush_w && !stall_w) begin
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
                if (br2) begin
                    pc_addr <= (br_addr2 + beq_off_q2 - 4'd4);
                    aux_addr <= (br_addr2 + beq_off_q2 - 4'd4);
                end
                else if (br3) begin
                    pc_addr <= br_addr2 - 4'd4;
                    aux_addr <= br_addr2 - 4'd4;
                end
//icache 不再当拍跳 jalr 目标，改成跟着 pc 走：pc 落在目标上，icache 下一拍取目标指令，
//一拍后交付。所以这里不能再 +4（+4 是给"icache 当拍已读目标"那种时序补时差的）。
                else if (jalr_fail) begin
                    pc_addr <= jalr_target_q;
                    aux_addr <= jalr_target_q;
                end
//同上：icache 的 irq/irq_ret 支路也拿掉了，落点同样不能再 +4
                else if (irq) begin
                    pc_addr <= isr_addr2;
                    aux_addr <= isr_addr2;
                end
                else if (irq_ret) begin
                    pc_addr <= isr_ret_addr2;
                    aux_addr <= isr_ret_addr2;
                end
            end
            else begin
                pc_addr <= pc_addr;
                aux_addr <= aux_addr;
            end
        end
    end

endmodule
