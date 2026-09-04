`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/28 15:27:58
// Design Name:
// Module Name: dtcm
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


module dtcm(
    input clk, rst,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    output reg [31:0] bus_data_out,
    output reg loaded
    );

    (* ram_style = "block" *) reg [31:0] dtcm [0:4095];
    integer i;
    initial begin
        for (i = 0; i < 4096; i = i + 1) dtcm[i] = 32'd0;
    end

    function [31:0] merge_word;
        input [31:0] oldw;
        input [31:0] din;
        input [3:0] be;
        begin
            merge_word = oldw;
            if (be[0]) merge_word[7:0] = din[7:0];
            if (be[1]) merge_word[15:8] = din[15:8];
            if (be[2]) merge_word[23:16] = din[23:16];
            if (be[3]) merge_word[31:24] = din[31:24];
        end
    endfunction

    always @(posedge clk) begin
        loaded <= 1'd0;
        bus_data_out <= 31'd0;
        if (bus_addr_in[31:24] == 8'd0 && bus_addr_in[23:20] == 4'd2) begin
            if (bus_we_in) begin
                dtcm[bus_addr_in[11:0]] <= merge_word(dtcm[bus_addr_in[11:0]], bus_data_in, bus_be_in);
            end
            else begin
                bus_data_out <= dtcm[bus_addr_in[11:0]];
                loaded <= 1'd1;
            end
        end
    end
endmodule
