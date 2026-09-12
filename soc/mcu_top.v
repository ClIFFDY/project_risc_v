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


module mcu_top(
    input clk, rst,
    inout wire [31:0] gpio_pin_bus
    );

    wire uart_rx_w;
    wire uart_tx_w;
    wire pwm1_w, pwm2_w;

    wire [31:0] bus_addr_f_cpu;
    wire [31:0] bus_data_f_cpu;
    wire [3:0] bus_be_f_cpu;
    wire bus_we_f_cpu;

    wire [31:0] bus_addr_out;
    wire [31:0] bus_data_out;
    wire [3:0] bus_be_out;
    wire bus_we_out;
    wire [31:0] bus_sel_out;

    wire [31:0] bus_data_b_cpu;
    wire [1023:0] bus_data_b;
    localparam
    PER_UART = 1,
    PER_PLIC = 2,
    PER_TIM  = 3,
    PER_GPIO = 4,
    PER_I2C  = 5;

    wire [31:0] bus_ready;
    wire uart_ld_ready, plic_ld_ready, tim_p_ld_ready, gpio_ld_ready, i2c_ld_ready;
    assign bus_ready[PER_UART] = uart_ld_ready;
    assign bus_ready[PER_PLIC] = plic_ld_ready;
    assign bus_ready[PER_TIM]  = tim_p_ld_ready;
    assign bus_ready[PER_GPIO] = gpio_ld_ready;
    assign bus_ready[PER_I2C]  = i2c_ld_ready;

    wire bus_loaded_out;
    wire [31:0] ibus_addr_w;
    wire ibus_re_w;
    wire rst_cpu;
    wire plic_exti;
    wire uart_rx_irq;
    wire [31:0] ibus_data_w;
    wire ibus_req_valid_w;
    wire ibus_busy_w;

    reg bus_we_uart;
    reg bus_we_plic;
    reg bus_we_tim_p;
    reg bus_we_gpio;
    reg bus_we_i2c;
    wire i2c_irq;
    wire i2c_scl_oe, i2c_sda_oe, i2c_scl_in, i2c_sda_in;
    always @(*) begin
        bus_we_uart = bus_we_out & bus_sel_out[PER_UART];
        bus_we_plic = bus_we_out & bus_sel_out[PER_PLIC];
        bus_we_tim_p = bus_we_out & bus_sel_out[PER_TIM];
        bus_we_gpio = bus_we_out & bus_sel_out[PER_GPIO];
        bus_we_i2c  = bus_we_out & bus_sel_out[PER_I2C];
    end

    rst_buf u_rst_buf (
        .clk(clk),
        .rst_n(rst),
        .rst_stable(rst_cpu)
    );

    cpu_top u_cpu_top (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_out(bus_addr_f_cpu),
        .bus_data_out(bus_data_f_cpu),
        .bus_be_out(bus_be_f_cpu),
        .bus_we_out(bus_we_f_cpu),
        .bus_data_in_ext(bus_data_b_cpu),
        .bus_loaded_in(bus_loaded_out),
        .exti(plic_exti),
        .ibus_addr_out(ibus_addr_w),
        .ibus_re_out(ibus_re_w),
        .ibus_req_valid(ibus_req_valid_w),
        .ibus_data_in(ibus_data_w),
        .ibus_addr_in(16'd0),
        .ibus_we_in(1'b0),
        .i_busy(ibus_busy_w)
    );

    icache u_icache (
        .clk(clk),
        .rst(rst_cpu),
        .ibus_addr_in(ibus_addr_w),
        .ibus_re_in(ibus_re_w),
        .req_valid(ibus_req_valid_w),
        .ibus_data_out(ibus_data_w),
        .busy(ibus_busy_w),
        .cache_miss(),
        .mem_req(),
        .mem_addr(),
        .mem_ready(1'b0),
        .mem_valid(1'b0),
        .mem_data(32'd0)
    );

    bus_arb u_bus_arb (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_f_cpu(bus_addr_f_cpu),
        .bus_data_f_cpu(bus_data_f_cpu),
        .bus_be_f_cpu(bus_be_f_cpu),
        .bus_we_f_cpu(bus_we_f_cpu),
        .bus_data_b_cpu(bus_data_b_cpu),
        .bus_addr_out(bus_addr_out),
        .bus_data_out(bus_data_out),
        .bus_be_out(bus_be_out),
        .bus_we_out(bus_we_out),
        .bus_sel_out(bus_sel_out),
        .bus_data_b(bus_data_b),
        .bus_ready(bus_ready),
        .bus_loaded_out(bus_loaded_out)
    );

    uart_top u_uart (
        .clk(clk),
        .rst(rst_cpu),
        .rx(uart_rx_w),
        .tx(uart_tx_w),
        .rx_irq(uart_rx_irq),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_uart),
        .ld_ready(uart_ld_ready),
        .bus_data_out(bus_data_b[PER_UART * 32 +: 32])
    );

    wire tim_p_irq;
    tim_p u_tim_p (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_tim_p),
        .ld_ready(tim_p_ld_ready),
        .bus_data_out(bus_data_b[PER_TIM * 32 +: 32]),
        .tim_p_irq(tim_p_irq),
        .pwm1(pwm1_w),
        .pwm2(pwm2_w)
    );

    wire gpio_irq;
    gpio_group u_gpio (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_gpio),
        .ld_ready(gpio_ld_ready),
        .bus_data_out(bus_data_b[PER_GPIO * 32 +: 32]),
        .gpio_irq(gpio_irq),
        .gpio_pin_bus(gpio_pin_bus),
        .sda_oe(i2c_sda_oe),
        .scl_oe(i2c_scl_oe),
        .tx(uart_tx_w),
        .pwm1(pwm1_w),
        .pwm2(pwm2_w),
        .rx(uart_rx_w),
        .sda_in(i2c_sda_in),
        .scl_in(i2c_scl_in)
    );

    i2c u_i2c (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_i2c),
        .ld_ready(i2c_ld_ready),
        .bus_data_out(bus_data_b[PER_I2C * 32 +: 32]),
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
        .irq_sources({27'd0, i2c_irq, gpio_irq, tim_p_irq, uart_rx_irq, 1'b0}),
        .exti(plic_exti),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_plic),
        .ld_ready(plic_ld_ready),
        .bus_data_out(bus_data_b[PER_PLIC * 32 +: 32])
    );
endmodule
