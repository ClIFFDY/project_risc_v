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
    input [1:0] stage,
    input we_in,
    input [4:0] rd_in,
    input [31:0] result_in,
    output reg we_out,
    output reg [4:0] rd_out, rd_back2,
    output reg [31:0] result_out, result_back2
    );

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

//写回级缓冲寄存器
    always @(posedge clk) begin
        if (rst) begin
            we_out <= 1'd0;
            rd_out <= 5'd0;
            rd_back2 <= 5'd0;
            result_out <= 32'd0;
            result_back2 <= 32'd0;
        end
//STALL 期间只保持数据，【写使能拉低】：
//若跟着保持，被冻住的这条 ALU 指令会每拍往 regfile 重复写一次，把期间更新的
//load/mulu 结果又盖回去（CoreMark rv32im 里 divu 冻住 wb_reg 34 拍、后面 sb 读到陈旧值）。
//写使能一拍已在进入本级那拍完成，故此处清零不会丢写。
        else if (stage == STALL) begin
            we_out <= 1'd0;
            rd_out <= rd_out;
            rd_back2 <= rd_back2;
            result_out <= result_out;
            result_back2 <= result_back2;
        end
        else if (we_in) begin
            we_out <= we_in;
            rd_out <= rd_in;
            rd_back2 <= rd_in;
            result_out <= result_in;
            result_back2 <= result_in;
        end
        else begin
            we_out <= 1'd0;
            rd_out <= 5'd0;
            rd_back2 <= 5'd0;
            result_out <= 32'd0;
            result_back2 <= 32'd0;
        end
    end
endmodule
