`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: wb_reg
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


module wb_reg(
    input clk, rst,
    input [4:0] flag_bus,
    input we_in,
    input [4:0] rd_in,
    input [31:0] result_in,
    output reg we_out,
    output reg [4:0] rd_out, rd_back2,
    output reg [31:0] result_out, result_back2
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[3] | flag_bus[2];
        stall_w = (flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

//写回级缓冲寄存器
    always @(posedge clk) begin
        if (rst_q) begin
            we_out <= 1'b0;
            rd_out <= 5'd0;
            rd_back2 <= 5'd0;
            result_out <= 32'd0;
            result_back2 <= 32'd0;
        end
        else if (exec) begin
//STALL 期间只保持数据，【写使能拉低】：
//若跟着保持，被冻住的这条 ALU 指令会每拍往 regfile 重复写一次，把期间更新的
//load/mulu 结果又盖回去（CoreMark rv32im 里 divu 冻住 wb_reg 34 拍、后面 sb 读到陈旧值）。
//写使能一拍已在进入本级那拍完成，故此处清零不会丢写。
            if (stall_w) begin
                we_out <= 1'b0;
                rd_out <= rd_out;
                rd_back2 <= rd_back2;
                result_out <= result_out;
                result_back2 <= result_back2;
            end
            else begin
                we_out <= we_in;
                rd_out <= rd_in;
                rd_back2 <= rd_in;
                result_out <= result_in;
                result_back2 <= result_in;
            end
        end
        else begin
            we_out <= 1'b0;
            rd_out <= 5'd0;
            rd_back2 <= 5'd0;
            result_out <= 32'd0;
            result_back2 <= 32'd0;
        end
    end
endmodule
