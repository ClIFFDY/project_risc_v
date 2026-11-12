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
    output     [31:0] tim_data,
    output            tim_ready,
//7 号槽：双通道 SPI 主机（flash + SD，共享 SCK/MOSI/MISO，各自 CS）
    output     [31:0] spi_data,
    output            spi_ready,
    output            spi_cs_n,
    output            spi_cs_sd,
    output            spi_sck,
    output            spi_mosi,
    input             spi_miso,
//boot 引擎的状态（只读展示在 pmem_spi 的 BOOT 寄存器里）
    input             boot_done,
    input             boot_err,
    input      [16:0] boot_count,
    output            plic_exti,
//DMA 的完成中断：挂到 PLIC 的 2 号源（3/4 号已被 GPIO/I2C 占）
    input             dma_irq,
//---- 总线错误日志（厂商扩展，寄存器在 plic 的 0x0900_0000 窗口）----
    input             err_valid,
    input             err_write,
    input      [1:0]  err_owner,
    input      [31:0] err_addr,
    output            err_clear
    );

    localparam
    PER_UART = 1,
    PER_PLIC = 2,
    PER_GPIO = 4,
    PER_I2C  = 5,
    PER_TIM  = 6,
    PER_SPI  = 7;

    wire uart_rx_w, uart_tx_w;
    wire uart_rx_irq, gpio_irq, i2c_irq, tim_irq_w;
    wire i2c_scl_oe, i2c_sda_oe, i2c_scl_in, i2c_sda_in;
    wire pwm1_w, pwm2_w, pwm3_w, pwm4_w;

    reg bus_we_uart, bus_we_plic, bus_we_gpio, bus_we_i2c, bus_we_tim, bus_we_spi;
    always @(*) begin
        bus_we_uart = bus_we_in & bus_sel_in[PER_UART];
        bus_we_plic = bus_we_in & bus_sel_in[PER_PLIC];
        bus_we_gpio = bus_we_in & bus_sel_in[PER_GPIO];
        bus_we_i2c  = bus_we_in & bus_sel_in[PER_I2C];
        bus_we_tim  = bus_we_in & bus_sel_in[PER_TIM];
        bus_we_spi  = bus_we_in & bus_sel_in[PER_SPI];
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
        .pwm1(pwm1_w),
        .pwm2(pwm2_w),
        .pwm3(pwm3_w),
        .pwm4(pwm4_w),
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

//定时器 / PWM（6 号槽）：FreeRTOS 的节拍源，四路 PWM 直接进 gpio_group 的 PWM1~4 模式
    tim_p u_tim (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_tim),
        .bus_data_out(tim_data),
        .ld_ready(tim_ready),
        .tim_p_irq(tim_irq_w),
        .pwm1(pwm1_w), .pwm2(pwm2_w), .pwm3(pwm3_w), .pwm4(pwm4_w)
    );

//双通道 SPI 主机（7 号槽）：flash 与 SD 共享三根线，CS 各一根
    pmem_spi u_pmem_spi (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_spi),
        .bus_data_out(spi_data),
        .ld_ready(spi_ready),
        .spi_cs_n(spi_cs_n),
        .spi_cs_sd(spi_cs_sd),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .boot_done(boot_done),
        .boot_err(boot_err),
        .boot_count(boot_count)
    );

    plic u_plic (
        .clk(clk),
        .rst(rst_cpu),
        //源号：1=uart_rx 2=dma 3=gpio 4=i2c 5=总线错误 6=定时器
        .irq_sources({25'd0, tim_irq_w, err_valid, i2c_irq, gpio_irq, dma_irq, uart_rx_irq, 1'b0}),
        .exti(plic_exti),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_plic),
        .ld_ready(plic_ready),
        .bus_data_out(plic_data),
        .err_valid(err_valid),
        .err_write(err_write),
        .err_owner(err_owner),
        .err_addr(err_addr),
        .err_clear(err_clear)
    );

endmodule
