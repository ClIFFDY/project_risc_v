`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: ahb_interconnect
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


module ahb_interconnect(
    input clk, rst,
    input      [31:0] m0_haddr,
    input      [1:0]  m0_htrans,
    input             m0_hwrite,
    input      [2:0]  m0_hsize,
    input      [2:0]  m0_hburst,
    input      [31:0] m0_hwdata,
    input      [3:0]  m0_hwstrb,
    output reg [31:0] m0_hrdata,
    output reg        m0_hready,
    output reg        m0_rd_valid,
    output reg [31:0] s_haddr,
    output reg [1:0]  s_htrans,
    output reg        s_hwrite,
    output reg [2:0]  s_hsize,
    output reg [2:0]  s_hburst,
    output reg [31:0] s_hwdata,
    output reg [3:0]  s_hwstrb,
    input      [31:0] s_hrdata,
    input             s_hready,
    input             s_rd_valid
    );

    always @(*) begin
        s_haddr  = m0_haddr;
        s_htrans = m0_htrans;
        s_hwrite = m0_hwrite;
        s_hsize  = m0_hsize;
        s_hburst = m0_hburst;
        s_hwdata = m0_hwdata;
        s_hwstrb = m0_hwstrb;
    end

    always @(*) begin
        m0_hrdata   = s_hrdata;
        m0_hready   = s_hready;
        m0_rd_valid = s_rd_valid;
    end

endmodule
