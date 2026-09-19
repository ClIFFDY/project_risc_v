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
    input [4:0] rd_back1, rd_back2, rd_load,
    input [31:0] r1_data_in_dec, r2_data_in_dec, r1_data_in_lsu, r2_data_in_lsu, ld_data,
    input loaded, stall,
    output reg [31:0] r1_data_final_dec, r2_data_final_dec, r1_data_final_lsu, r2_data_final_lsu
    );
    reg stalled;

//对ld/st指令数据旁路进行延迟仲裁
    always @(*) begin
        if (!stalled && rd_back1 != 5'd0 && r1 == rd_back1) begin
            r1_data_final_dec = result_back1;
            r1_data_final_lsu = result_back1;
        end
        else if (loaded && rd_load != 5'd0 && r1 == rd_load) begin
            r1_data_final_dec = ld_data;
            r1_data_final_lsu = ld_data;
        end
        else if (rd_back2 != 5'd0 && r1 == rd_back2) begin
            r1_data_final_dec = result_back2;
            r1_data_final_lsu = result_back2;
        end
        else begin
            r1_data_final_dec = r1_data_in_dec;
            r1_data_final_lsu = r1_data_in_lsu;
        end
        if (!stalled && rd_back1 != 5'd0 && r2 == rd_back1) begin
            r2_data_final_dec = result_back1;
            r2_data_final_lsu = result_back1;
        end
        else if (loaded && rd_load != 5'd0 && r2 == rd_load) begin
            r2_data_final_dec = ld_data;
            r2_data_final_lsu = ld_data;
        end
        else if (rd_back2 != 5'd0 && r2 == rd_back2) begin
            r2_data_final_dec = result_back2;
            r2_data_final_lsu = result_back2;
        end
        else begin
            r2_data_final_dec = r2_data_in_dec;
            r2_data_final_lsu = r2_data_in_lsu;
        end
    end

    always @(posedge clk) begin
        if (rst) stalled <= 1'd0;
        else stalled <= stall;
    end
endmodule
