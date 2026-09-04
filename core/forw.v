`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/28 14:41:46
// Design Name:
// Module Name: forw
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


module forw(
    input clk, rst,
    input [31:0] result_back1, result_back2,
    input [4:0] r1, r2,
    input [4:0] rd_back1, rd_back2,
    input [31:0] r1_data_in_dec, r2_data_in_dec, r1_data_in_lsu, r2_data_in_lsu, ld_data,
    input loaded, stall,
    output reg [31:0] r1_data_final_dec, r2_data_final_dec, r1_data_final_lsu, r2_data_final_lsu
    );
    reg [31:0] result_back2_final;
    reg stalled;
    always @(*) begin
        if (!stalled) begin
            result_back2_final = (loaded) ? ld_data : result_back2;
            r1_data_final_dec = ((rd_back1 != 5'd0 && r1 == rd_back1) ? result_back1 : (rd_back2 != 5'd0 && r1 == rd_back2) ? result_back2_final : r1_data_in_dec);
            r2_data_final_dec = ((rd_back1 != 5'd0 && r2 == rd_back1) ? result_back1 : (rd_back2 != 5'd0 && r2 == rd_back2) ? result_back2_final : r2_data_in_dec);
            r1_data_final_lsu = ((rd_back1 != 5'd0 && r1 == rd_back1) ? result_back1 : (rd_back2 != 5'd0 && r1 == rd_back2) ? result_back2_final : r1_data_in_lsu);
            r2_data_final_lsu = ((rd_back1 != 5'd0 && r2 == rd_back1) ? result_back1 : (rd_back2 != 5'd0 && r2 == rd_back2) ? result_back2_final : r2_data_in_lsu);
        end
        else begin
            result_back2_final = (loaded) ? ld_data : result_back2;
            r1_data_final_dec = ((rd_back2 != 5'd0 && r1 == rd_back2) ? result_back2_final : r1_data_in_dec);
            r2_data_final_dec = ((rd_back2 != 5'd0 && r2 == rd_back2) ? result_back2_final : r2_data_in_dec);
            r1_data_final_lsu = ((rd_back2 != 5'd0 && r1 == rd_back2) ? result_back2_final : r1_data_in_lsu);
            r2_data_final_lsu = ((rd_back2 != 5'd0 && r2 == rd_back2) ? result_back2_final : r2_data_in_lsu);
        end
    end

    always @(posedge clk) begin
        if (rst) stalled <= 1'd0;
        else stalled <= stall;
    end
endmodule
