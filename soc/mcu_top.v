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
    parameter integer LANES             = 2,
    parameter integer ROB_DEPTH         = 16,
    parameter integer IFFU_DEPTH        = 8,
    parameter integer VALUE_TABLE_DEPTH = 16,
    parameter integer IC_INDEX_B        = 8,
    parameter integer BHT_B             = 11,
    parameter integer BTB_B             = 11,
    parameter integer DC_SETS_B         = 9,
    parameter integer ITCM_AW           = 14,
    parameter integer ADDER_AT_D        = 0,
//★ 7 号槽 SPI 的分频由这两个频率算出来（见 IO_block/pmem_spi.v 头注释）：
//  换核时钟只改 CLK_MHZ，SCK 目标频率保持不变。
    parameter integer CLK_MHZ           = 120,
    parameter integer FLASH_SCK_KHZ     = 2500,
    parameter integer SD_SCK_KHZ        = 300
) (
    input clk, rst,
    inout wire [31:0] gpio_pin_bus,
//---- SPI 主机（7 号槽）：接板上的 SPI NOR flash，直出、不经 gpio ----
    output spi_cs_n,
    output spi_cs_sd,
    output spi_sck,
    output spi_mosi,
    input  spi_miso
    );

    wire rst_cpu;        // CPU / DMA 域（boot 搬运期间压着）
    wire rst_sys;        // 总线 / STCM / APB / SPI 域
    wire boot_done_w, boot_err_w;
    wire [16:0] boot_count_w;

    wire [31:0] bus_addr_f_cpu, bus_data_f_cpu, bus_data_b_cpu;
    wire [3:0]  bus_be_f_cpu;
    wire        bus_we_f_cpu, bus_valid_f_cpu, bus_loaded_out, bus_hold_out;

    wire [31:0] bus_addr_out, bus_data_out, bus_sel_out;
    wire [3:0]  bus_be_out;
    wire        bus_we_out;

    wire [31:0] uart_data_w, plic_data_w, gpio_data_w, i2c_data_w, tim_data_w, spi_data_w;
    wire        uart_ready_w, plic_ready_w, gpio_ready_w, i2c_ready_w, tim_ready_w, spi_ready_w;
    wire        plic_exti;
    wire        dma_irq_w;

//---- L2 的 AHB 主口（cpu_top ↔ bus_con_top 的 m1）----
    wire [31:0] l2_haddr_w, l2_hwdata_w, l2_hrdata_w;
    wire [1:0]  l2_htrans_w;
    wire        l2_hwrite_w, l2_hready_w, l2_rd_valid_w, l2_hgrant_w, l2_busy_w;
    wire        l2_hresp_w, l2_hmastlock_w;
    wire [2:0]  l2_hsize_w, l2_hburst_w;
    wire [3:0]  l2_hwstrb_w, l2_hprot_w;
    wire        l2_maint_done_w;

//---- 总线错误日志（厂商扩展，寄存器在 plic）----
    wire        err_valid_w, err_write_w, err_clear_w;
    wire [1:0]  err_owner_w;
    wire [31:0] err_addr_w;

//L2 维护与"非缓存窗口"（走总线的 DMA 缓冲区用）。
//★ 现在没有任何软件通路去写它们：maint 恒 0（不失效），非缓存窗口设成永不命中。
//  ⇒ DMA 直接改 STCM 之后，L2 里若有同一行的旧副本，CPU 会读到旧值。
//  这块要接上"某个能从软件写的寄存器"（PLIC 旁边加一个槽，或者拿一条 CSR），
//  在那之前 DMA 只能用于"搬完就不由 CPU 再读"的场景。
    wire [1:0]  l2_maint_w   = 2'b00;
    wire [31:0] l2_unc_base_w = 32'hFFFF_FFFF;
    wire [31:0] l2_unc_mask_w = 32'h0000_0000;

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
        .exti(plic_exti),
        .l2_haddr(l2_haddr_w), .l2_htrans(l2_htrans_w), .l2_hwrite(l2_hwrite_w),
        .l2_hsize(l2_hsize_w), .l2_hburst(l2_hburst_w),
        .l2_hprot(l2_hprot_w), .l2_hmastlock(l2_hmastlock_w),
        .l2_hwdata(l2_hwdata_w), .l2_hwstrb(l2_hwstrb_w),
        .l2_hrdata(l2_hrdata_w), .l2_hready(l2_hready_w), .l2_rd_valid(l2_rd_valid_w),
        .l2_hgrant(l2_hgrant_w), .l2_hresp(l2_hresp_w), .l2_busy(l2_busy_w),
        .l2_maint(l2_maint_w), .l2_maint_done(l2_maint_done_w),
        .l2_uncached_base(l2_unc_base_w), .l2_uncached_mask(l2_unc_mask_w)
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
        .rst_sys(rst_sys),
        .boot_done_out(boot_done_w),
        .boot_err_out(boot_err_w),
        .boot_count_out(boot_count_w),
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
        .i2c_ready(i2c_ready_w),
        .tim_data(tim_data_w),
        .tim_ready(tim_ready_w),
        .spi_data(spi_data_w),
        .spi_ready(spi_ready_w),
        .dma_irq(dma_irq_w),
        .m1_haddr(l2_haddr_w), .m1_htrans(l2_htrans_w), .m1_hwrite(l2_hwrite_w),
        .m1_hsize(l2_hsize_w), .m1_hburst(l2_hburst_w),
        .m1_hprot(l2_hprot_w), .m1_hmastlock(l2_hmastlock_w),
        .m1_hwdata(l2_hwdata_w), .m1_hwstrb(l2_hwstrb_w),
        .m1_hrdata(l2_hrdata_w), .m1_hready(l2_hready_w), .m1_hresp(l2_hresp_w),
        .m1_rd_valid(l2_rd_valid_w), .m1_hgrant(l2_hgrant_w),
        .err_valid(err_valid_w), .err_write(err_write_w), .err_owner(err_owner_w),
        .err_addr(err_addr_w), .err_clear(err_clear_w)
    );

    io_block_top #(
        .CLK_MHZ(CLK_MHZ),
        .FLASH_SCK_KHZ(FLASH_SCK_KHZ),
        .SD_SCK_KHZ(SD_SCK_KHZ)
    ) u_io_block (
        .clk(clk),
        .rst_cpu(rst_sys),
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
        .tim_data(tim_data_w),
        .tim_ready(tim_ready_w),
        .spi_data(spi_data_w),
        .spi_ready(spi_ready_w),
        .spi_cs_n(spi_cs_n),
        .spi_cs_sd(spi_cs_sd),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .boot_done(boot_done_w),
        .boot_err(boot_err_w),
        .boot_count(boot_count_w),
        .plic_exti(plic_exti),
        .dma_irq(dma_irq_w),
        .err_valid(err_valid_w), .err_write(err_write_w), .err_owner(err_owner_w),
        .err_addr(err_addr_w), .err_clear(err_clear_w)
    );

endmodule
