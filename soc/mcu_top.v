`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/29 02:38:18
// Design Name:
// Module Name: mcu_top
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


module mcu_top #(
//★★ 宏化旋钮：与 cpu_top 同名同义，只为让综合顶层能选档（synth_design -generic / xelab -generic_top）。
//  默认值 = 大核交付配置。
    parameter integer LANES             = 1,
    parameter integer ROB_DEPTH         = 8,
    parameter integer IFFU_DEPTH        = 4,
    parameter integer VALUE_TABLE_DEPTH = 8,
    parameter integer IC_INDEX_B        = 7,
    parameter integer BHT_B             = 9,
    parameter integer BTB_B             = 9,
    parameter integer DC_SETS_B         = 8,
    parameter integer ITCM_AW           = 14,
    parameter integer ADDER_AT_D        = 1
) (
    input clk, rst,
    inout wire [31:0] gpio_pin_bus
    );

    wire rst_cpu;

    wire [31:0] bus_addr_f_cpu, bus_data_f_cpu, bus_data_b_cpu;
    wire [3:0]  bus_be_f_cpu;
    wire        bus_we_f_cpu, bus_valid_f_cpu, bus_loaded_out, bus_hold_out;

    wire [31:0] bus_addr_out, bus_data_out, bus_sel_out;
    wire [3:0]  bus_be_out;
    wire        bus_we_out;

    wire [31:0] uart_data_w, plic_data_w, gpio_data_w, i2c_data_w;
    wire        uart_ready_w, plic_ready_w, gpio_ready_w, i2c_ready_w;
    wire        plic_exti;

    cpu_top #(.LANES(LANES), .ROB_DEPTH(ROB_DEPTH), .IFFU_DEPTH(IFFU_DEPTH), .VALUE_TABLE_DEPTH(VALUE_TABLE_DEPTH), .IC_INDEX_B(IC_INDEX_B), .BHT_B(BHT_B), .BTB_B(BTB_B), .ADDER_AT_D(ADDER_AT_D), .DC_SETS_B(DC_SETS_B), .ITCM_AW(ITCM_AW)) u_cpu_top (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_out(bus_addr_f_cpu),
        .bus_data_out(bus_data_f_cpu),
        .bus_be_out(bus_be_f_cpu),
        .bus_we_out(bus_we_f_cpu),
        .bus_valid_out(bus_valid_f_cpu),
        .bus_data_in_ext(bus_data_b_cpu),
        .bus_loaded_in(bus_loaded_out),
        .bus_hold_in(bus_hold_out),
        .exti(plic_exti)
    );

    bus_con_top u_bus_con (
        .clk(clk),
        .rst(rst),
        .bus_addr_f_cpu(bus_addr_f_cpu),
        .bus_data_f_cpu(bus_data_f_cpu),
        .bus_be_f_cpu(bus_be_f_cpu),
        .bus_we_f_cpu(bus_we_f_cpu),
        .bus_valid_f_cpu(bus_valid_f_cpu),
        .rst_cpu(rst_cpu),
        .bus_data_b_cpu(bus_data_b_cpu),
        .bus_loaded_out(bus_loaded_out),
        .bus_hold_out(bus_hold_out),
        .bus_addr_out(bus_addr_out),
        .bus_data_out(bus_data_out),
        .bus_be_out(bus_be_out),
        .bus_we_out(bus_we_out),
        .bus_sel_out(bus_sel_out),
        .uart_data(uart_data_w),
        .uart_ready(uart_ready_w),
        .plic_data(plic_data_w),
        .plic_ready(plic_ready_w),
        .gpio_data(gpio_data_w),
        .gpio_ready(gpio_ready_w),
        .i2c_data(i2c_data_w),
        .i2c_ready(i2c_ready_w)
    );

    io_block_top u_io_block (
        .clk(clk),
        .rst_cpu(rst_cpu),
        .gpio_pin_bus(gpio_pin_bus),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_out),
        .bus_sel_in(bus_sel_out),
        .uart_data(uart_data_w),
        .uart_ready(uart_ready_w),
        .plic_data(plic_data_w),
        .plic_ready(plic_ready_w),
        .gpio_data(gpio_data_w),
        .gpio_ready(gpio_ready_w),
        .i2c_data(i2c_data_w),
        .i2c_ready(i2c_ready_w),
        .plic_exti(plic_exti)
    );

endmodule
