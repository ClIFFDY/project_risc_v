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
    input rx,
    output tx
    );

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
    PER_PLIC = 2;

    wire [31:0] bus_ready;
    wire uart_ld_ready, plic_ld_ready;
    assign bus_ready[PER_UART] = uart_ld_ready;
    assign bus_ready[PER_PLIC] = plic_ld_ready;

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
    always @(*) begin
        bus_we_uart = bus_we_out & bus_sel_out[PER_UART];
        bus_we_plic = bus_we_out & bus_sel_out[PER_PLIC];
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
        .rx(rx),
        .tx(tx),
        .rx_irq(uart_rx_irq),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_uart),
        .ld_ready(uart_ld_ready),
        .bus_data_out(bus_data_b[PER_UART * 32 +: 32])
    );

    plic u_plic (
        .clk(clk),
        .rst(rst_cpu),
        .irq_sources({30'd0, uart_rx_irq, 1'b0}),
        .exti(plic_exti),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_plic),
        .ld_ready(plic_ld_ready),
        .bus_data_out(bus_data_b[PER_PLIC * 32 +: 32])
    );
endmodule
