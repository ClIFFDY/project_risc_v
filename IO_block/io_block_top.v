`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: io_block
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


module io_block_top(
    input clk, rst_cpu,
    inout wire [31:0] gpio_pin_bus,
    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    input      [31:0] bus_sel_in,
    output     [31:0] uart_data,
    output            uart_ready,
    output     [31:0] plic_data,
    output            plic_ready,
    output     [31:0] gpio_data,
    output            gpio_ready,
    output     [31:0] i2c_data,
    output            i2c_ready,
    output            plic_exti
    );

    localparam
    PER_UART = 1,
    PER_PLIC = 2,
    PER_GPIO = 4,
    PER_I2C  = 5;

    wire uart_rx_w, uart_tx_w;
    wire uart_rx_irq, gpio_irq, i2c_irq;
    wire i2c_scl_oe, i2c_sda_oe, i2c_scl_in, i2c_sda_in;

    reg bus_we_uart, bus_we_plic, bus_we_gpio, bus_we_i2c;
    always @(*) begin
        bus_we_uart = bus_we_in & bus_sel_in[PER_UART];
        bus_we_plic = bus_we_in & bus_sel_in[PER_PLIC];
        bus_we_gpio = bus_we_in & bus_sel_in[PER_GPIO];
        bus_we_i2c  = bus_we_in & bus_sel_in[PER_I2C];
    end

    uart_top u_uart (
        .clk(clk),
        .rst(rst_cpu),
        .rx(uart_rx_w),
        .tx(uart_tx_w),
        .rx_irq(uart_rx_irq),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_uart),
        .ld_ready(uart_ready),
        .bus_data_out(uart_data)
    );

    gpio_group u_gpio (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_gpio),
        .ld_ready(gpio_ready),
        .bus_data_out(gpio_data),
        .gpio_irq(gpio_irq),
        .gpio_pin_bus(gpio_pin_bus),
        .sda_oe(i2c_sda_oe),
        .scl_oe(i2c_scl_oe),
        .tx(uart_tx_w),
        .pwm1(1'b0),
        .pwm2(1'b0),
        .pwm3(1'b0),
        .pwm4(1'b0),
        .rx(uart_rx_w),
        .sda_in(i2c_sda_in),
        .scl_in(i2c_scl_in)
    );

    i2c u_i2c (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_i2c),
        .ld_ready(i2c_ready),
        .bus_data_out(i2c_data),
        .i2c_irq(i2c_irq),
        .scl_oe(i2c_scl_oe),
        .scl_in(i2c_scl_in),
        .sda_oe(i2c_sda_oe),
        .sda_in(i2c_sda_in),
        .busy()
    );

    plic u_plic (
        .clk(clk),
        .rst(rst_cpu),
        .irq_sources({26'd0, 1'b0, i2c_irq, gpio_irq, 1'b0, uart_rx_irq, 1'b0}),
        .exti(plic_exti),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_plic),
        .ld_ready(plic_ready),
        .bus_data_out(plic_data)
    );

endmodule
