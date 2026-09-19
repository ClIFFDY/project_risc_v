`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: lsu_ahb_master
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


module lsu_ahb_master(
    input clk, rst,
    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    input             bus_valid_in,
    output reg [31:0] bus_data_out,
    output reg        bus_ready_out,
    output reg        bus_hold_out,
    output reg [31:0] haddr,
    output reg [1:0]  htrans,
    output reg        hwrite,
    output reg [2:0]  hsize,
    output reg [2:0]  hburst,
    output reg [31:0] hwdata,
    output reg [3:0]  hwstrb,
    input      [31:0] hrdata,
    input             hready,
    input             rd_valid
    );

    localparam [1:0] HTRANS_IDLE   = 2'b00;
    localparam [1:0] HTRANS_NONSEQ = 2'b10;

    always @(*) begin
        haddr  = bus_addr_in << 2;
        htrans = bus_valid_in ? HTRANS_NONSEQ : HTRANS_IDLE;
        hwrite = bus_we_in;
        hsize  = 3'b010;
        hburst = 3'b000;
        hwstrb = bus_be_in;
        hwdata = bus_data_in;
    end

    always @(*) begin
        bus_data_out  = hrdata;
        bus_ready_out = rd_valid;
        bus_hold_out  = 1'b0;
    end

endmodule
