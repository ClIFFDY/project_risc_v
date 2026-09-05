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
        else begin
            if (we_in && stage != STALL) begin
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
    end
endmodule
