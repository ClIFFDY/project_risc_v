`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/30 21:30:00
// Design Name:
// Module Name: bus_arb
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


module bus_arb(
    input clk, rst,
    input [31:0] bus_addr_f_cpu,
    input [31:0] bus_data_f_cpu,
    input [3:0] bus_be_f_cpu,
    input bus_we_f_cpu,
    output reg [31:0] bus_data_b_cpu,
    //
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out,
    output reg [31:0] bus_sel_out,
    //
    input [1023:0] bus_data_b,
    input [31:0] bus_ready,
    //
    output reg bus_loaded_out
    );

    reg [4:0] per_decode;
    reg [4:0] per_sel;

    always @(*) begin
        if (bus_addr_f_cpu[31:24] >= 8'd1 && bus_addr_f_cpu[31:24] <= 8'd31)
            per_decode = bus_addr_f_cpu[31:24];
        else
            per_decode = 5'd0;
    end

    always @(posedge clk) begin
        if (rst) per_sel <= 5'd0;
        else per_sel <= !bus_we_f_cpu ? per_decode : 5'd0;
    end

    always @(*) begin
        if (per_sel != 5'd0) bus_loaded_out = bus_ready[per_sel];
        else bus_loaded_out = 1'b0;
    end

    always @(*) begin
        bus_addr_out = bus_addr_f_cpu;
        bus_data_out = bus_data_f_cpu;
        bus_be_out = bus_be_f_cpu;
        bus_we_out = bus_we_f_cpu;
        bus_sel_out = (per_decode != 5'd0) ? (32'd1 << per_decode) : 32'd0;
    end

    always @(*) begin
        if (per_sel != 5'd0) bus_data_b_cpu = bus_data_b[per_sel * 32 +: 32];
        else bus_data_b_cpu = 32'd0;
    end

endmodule
