`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/30 21:30:00
// Design Name:
// Module Name: bus_arb
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


module bus_arb(
    input clk, rst,
    input [31:0] bus_addr_f_cpu,
    input [31:0] bus_data_f_cpu,
    input [3:0] bus_be_f_cpu,
    input bus_we_f_cpu,
    output reg [31:0] bus_data_b_cpu,
    //
    output reg [31:0] bus_addr_uart_f,
    output reg [31:0] bus_data_uart_f,
    output reg bus_we_uart_f,
    output reg [3:0] bus_be_uart_f,
    input [31:0] bus_data_uart_b,
    //
    output reg [31:0] bus_addr_plic_f,
    output reg [31:0] bus_data_plic_f,
    output reg bus_we_plic_f,
    output reg [3:0] bus_be_plic_f,
    input [31:0] bus_data_plic_b,
    //
    output reg bus_loaded_out
    );

    localparam [1:0]
    PER_NONE = 2'd0,
    PER_UART = 2'd1,
    PER_PLIC = 2'd2;

    reg [1:0] per_decode;
    reg [1:0] per_sel;

    always @(*) begin
        case (bus_addr_f_cpu[31:24])
        8'd1: per_decode = PER_UART;
        8'd2: per_decode = PER_PLIC;
        default: per_decode = PER_NONE;
        endcase
    end

    always @(posedge clk) begin
        if (rst) per_sel <= PER_NONE;
        else per_sel <= !bus_we_f_cpu ? per_decode : PER_NONE;
    end

    always @(*) bus_loaded_out = (per_sel != PER_NONE);

    always @(*) begin
        bus_addr_uart_f = 32'd0;
        bus_data_uart_f = 32'd0;
        bus_we_uart_f = 1'b0;
        bus_be_uart_f = 4'd0;
        bus_addr_plic_f = 32'd0;
        bus_data_plic_f = 32'd0;
        bus_we_plic_f = 1'b0;
        bus_be_plic_f = 4'd0;
        if (per_decode == PER_UART) begin
            bus_addr_uart_f = bus_addr_f_cpu;
            bus_data_uart_f = bus_data_f_cpu;
            bus_we_uart_f = bus_we_f_cpu;
            bus_be_uart_f = bus_be_f_cpu;
        end
        else if (per_decode == PER_PLIC) begin
            bus_addr_plic_f = bus_addr_f_cpu;
            bus_data_plic_f = bus_data_f_cpu;
            bus_we_plic_f = bus_we_f_cpu;
            bus_be_plic_f = bus_be_f_cpu;
        end
    end

    always @(*) begin
        case (per_sel)
        PER_UART: bus_data_b_cpu = bus_data_uart_b;
        PER_PLIC: bus_data_b_cpu = bus_data_plic_b;
        default: bus_data_b_cpu = 32'd0;
        endcase
    end
endmodule
