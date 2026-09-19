`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: bus_con
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


module bus_con_top(
    input clk, rst,
    input      [31:0] bus_addr_f_cpu,
    input      [31:0] bus_data_f_cpu,
    input      [3:0]  bus_be_f_cpu,
    input             bus_we_f_cpu,
    input             bus_valid_f_cpu,
    output            rst_cpu,
    output     [31:0] bus_data_b_cpu,
    output            bus_loaded_out,
    output            bus_hold_out,
    output     [31:0] bus_addr_out,
    output     [31:0] bus_data_out,
    output     [3:0]  bus_be_out,
    output            bus_we_out,
    output     [31:0] bus_sel_out,
    input      [31:0] uart_data,
    input             uart_ready,
    input      [31:0] plic_data,
    input             plic_ready,
    input      [31:0] gpio_data,
    input             gpio_ready,
    input      [31:0] i2c_data,
    input             i2c_ready
    );

    wire [31:0] m0_haddr, m0_hwdata, m0_hrdata;
    wire [1:0]  m0_htrans;
    wire        m0_hwrite, m0_hready, m0_rd_valid;
    wire [2:0]  m0_hsize, m0_hburst;
    wire [3:0]  m0_hwstrb;

    wire [31:0] s_haddr, s_hwdata, s_hrdata;
    wire [1:0]  s_htrans;
    wire        s_hwrite, s_hready, s_rd_valid;
    wire [2:0]  s_hsize, s_hburst;
    wire [3:0]  s_hwstrb;

    rst_buf u_rst_buf (
        .clk(clk),
        .rst_n(rst),
        .rst_stable(rst_cpu)
    );

    lsu_ahb_master u_lsu_ahb (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_f_cpu),
        .bus_data_in(bus_data_f_cpu),
        .bus_be_in(bus_be_f_cpu),
        .bus_we_in(bus_we_f_cpu),
        .bus_valid_in(bus_valid_f_cpu),
        .bus_data_out(bus_data_b_cpu),
        .bus_ready_out(bus_loaded_out),
        .bus_hold_out(bus_hold_out),
        .haddr(m0_haddr),
        .htrans(m0_htrans),
        .hwrite(m0_hwrite),
        .hsize(m0_hsize),
        .hburst(m0_hburst),
        .hwdata(m0_hwdata),
        .hwstrb(m0_hwstrb),
        .hrdata(m0_hrdata),
        .hready(m0_hready),
        .rd_valid(m0_rd_valid)
    );

    ahb_interconnect u_ahb_ic (
        .clk(clk),
        .rst(rst_cpu),
        .m0_haddr(m0_haddr),
        .m0_htrans(m0_htrans),
        .m0_hwrite(m0_hwrite),
        .m0_hsize(m0_hsize),
        .m0_hburst(m0_hburst),
        .m0_hwdata(m0_hwdata),
        .m0_hwstrb(m0_hwstrb),
        .m0_hrdata(m0_hrdata),
        .m0_hready(m0_hready),
        .m0_rd_valid(m0_rd_valid),
        .s_haddr(s_haddr),
        .s_htrans(s_htrans),
        .s_hwrite(s_hwrite),
        .s_hsize(s_hsize),
        .s_hburst(s_hburst),
        .s_hwdata(s_hwdata),
        .s_hwstrb(s_hwstrb),
        .s_hrdata(s_hrdata),
        .s_hready(s_hready),
        .s_rd_valid(s_rd_valid)
    );

    ahb_apb_bridge u_ahb_apb (
        .clk(clk),
        .rst(rst_cpu),
        .haddr(s_haddr),
        .htrans(s_htrans),
        .hwrite(s_hwrite),
        .hsize(s_hsize),
        .hburst(s_hburst),
        .hwdata(s_hwdata),
        .hwstrb(s_hwstrb),
        .hrdata(s_hrdata),
        .hready(s_hready),
        .hresp(),
        .rd_valid(s_rd_valid),
        .bus_addr_out(bus_addr_out),
        .bus_data_out(bus_data_out),
        .bus_be_out(bus_be_out),
        .bus_we_out(bus_we_out),
        .bus_sel_out(bus_sel_out),
        .uart_data(uart_data),
        .uart_ready(uart_ready),
        .plic_data(plic_data),
        .plic_ready(plic_ready),
        .gpio_data(gpio_data),
        .gpio_ready(gpio_ready),
        .i2c_data(i2c_data),
        .i2c_ready(i2c_ready)
    );

endmodule
