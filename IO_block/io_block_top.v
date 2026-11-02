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
    output     [31:0] tim_data,
    output            tim_ready,
    output     [31:0] gpio_data,
    output            gpio_ready,
    output     [31:0] i2c_data,
    output            i2c_ready,
    output     [31:0] qspi_data,
    output            qspi_ready,
    output            plic_exti
    );

    localparam
    PER_UART = 1,
    PER_PLIC = 2,
    PER_TIM  = 3,
    PER_GPIO = 4,
    PER_I2C  = 5,
    PER_QSPI = 7;

// PLIC 中断源编号（32 路向量里的位号）——不要散着写常量
    localparam integer IRQ_UART = 1,
                       IRQ_GPIO = 3,
                       IRQ_I2C  = 4,
                       IRQ_TIM  = 5;

    wire uart_rx_w, uart_tx_w;
    wire uart_rx_irq, gpio_irq, i2c_irq, tim_irq, qspi_irq;
    wire i2c_scl_oe, i2c_sda_oe, i2c_scl_in, i2c_sda_in;
    wire tim_pwm1, tim_pwm2, tim_pwm3, tim_pwm4;
    wire qspi_sck, qspi_cs_n;
    wire [3:0] qspi_io_out, qspi_io_oe, qspi_io_in;

    reg bus_we_uart, bus_we_plic, bus_we_tim, bus_we_gpio, bus_we_i2c, bus_we_qspi;
    always @(*) begin
        bus_we_uart = bus_we_in & bus_sel_in[PER_UART];
        bus_we_plic = bus_we_in & bus_sel_in[PER_PLIC];
        bus_we_tim  = bus_we_in & bus_sel_in[PER_TIM];
        bus_we_gpio = bus_we_in & bus_sel_in[PER_GPIO];
        bus_we_i2c  = bus_we_in & bus_sel_in[PER_I2C];
        bus_we_qspi = bus_we_in & bus_sel_in[PER_QSPI];
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
        .pwm1(tim_pwm1),
        .pwm2(tim_pwm2),
        .pwm3(tim_pwm3),
        .pwm4(tim_pwm4),
        .spi_sck(qspi_sck),
        .spi_cs_n(qspi_cs_n),
        .spi_io_out(qspi_io_out),
        .spi_io_oe(qspi_io_oe),
        .spi_io_in(qspi_io_in),
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

    tim_p u_tim (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_tim),
        .ld_ready(tim_ready),
        .bus_data_out(tim_data),
        .tim_p_irq(tim_irq),
        .pwm1(tim_pwm1),
        .pwm2(tim_pwm2),
        .pwm3(tim_pwm3),
        .pwm4(tim_pwm4)
    );

    qspi u_qspi (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_qspi),
        .ld_ready(qspi_ready),
        .bus_data_out(qspi_data),
        .qspi_irq(qspi_irq),
        .sck(qspi_sck),
        .cs_n(qspi_cs_n),
        .io_out(qspi_io_out),
        .io_oe(qspi_io_oe),
        .io_in(qspi_io_in)
    );

    plic u_plic (
        .clk(clk),
        .rst(rst_cpu),
        .irq_sources({25'd0, qspi_irq, tim_irq, i2c_irq, gpio_irq, 1'b0, uart_rx_irq, 1'b0}),
        .exti(plic_exti),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_plic),
        .ld_ready(plic_ready),
        .bus_data_out(plic_data)
    );

endmodule
