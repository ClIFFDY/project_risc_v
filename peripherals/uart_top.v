`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/30 22:06:59
// Design Name:
// Module Name: uart_top
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


module uart_top(
    input clk, rst,
    input rx,
    output tx,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    output reg [31:0] bus_data_out,
    output reg rx_irq
    );

    localparam CLK = 50_000_000;
    localparam BPSL = 9600;
    localparam BPSH = 115200;

    reg uart_bps;
    reg [13:0] mcnt, mcnt_half, mcnt_3q;

    always @(posedge clk) begin
        if (rst) begin
            mcnt <= CLK/BPSL;
            mcnt_half <= (CLK/BPSL)/2;
            mcnt_3q <= (CLK/BPSL)/2 + (CLK/BPSL)/4 - 1;
        end
        else if (bus_addr_in[23:20] == 4'b0011 && bus_we_in && bus_be_in[0]) begin
            if (bus_data_in[1:0] == 2'b01) begin
                mcnt <= CLK/BPSH;
                mcnt_half <= (CLK/BPSH)/2;
                mcnt_3q <= (CLK/BPSH)/2 + (CLK/BPSH)/4 - 1;
            end
            else begin
                mcnt <= CLK/BPSL;
                mcnt_half <= (CLK/BPSL)/2;
                mcnt_3q <= (CLK/BPSL)/2 + (CLK/BPSL)/4 - 1;
            end
        end
    end

    wire [7:0] rx_data;
    reg [7:0] tx_data;
    reg tx_en, rx_read, tx_busy, rx_cont;
    wire rx_done, busy;

    reg [7:0] rx_buf [0:63];
    reg [7:0] tx_buf [0:63];
    reg [5:0] wr_ptr_rx, rd_ptr_rx, wr_ptr_tx, rd_ptr_tx;
    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            rx_buf[i] = 8'b0;
            tx_buf[i] = 8'b0;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr_rx <= 6'd1;
            rd_ptr_rx <= 6'd0;
            wr_ptr_tx <= 6'd1;
            rd_ptr_tx <= 6'd0;
            rx_irq <= 1'd0;
            for (i = 0; i < 64; i = i + 1) begin
                rx_buf[i] = 8'b0;
                tx_buf[i] = 8'b0;
            end
        end
        else begin
            rx_irq <= 1'd0;
            if (rx_done && wr_ptr_rx != rd_ptr_rx) begin
                rx_buf[wr_ptr_rx] <= rx_data;
                wr_ptr_rx <= wr_ptr_rx + 1'b1;
            end
            if (bus_we_in && bus_addr_in[23:20] == 4'b0001 && bus_be_in[0] && wr_ptr_tx != rd_ptr_tx) begin
                tx_buf[wr_ptr_tx] <= bus_data_in[7:0];
                wr_ptr_tx <= wr_ptr_tx + 1'b1;
            end
            if (!busy && rd_ptr_tx != wr_ptr_tx - 1'b1) begin
                rd_ptr_tx <= rd_ptr_tx + 1'b1;
            end
            if (rx_read && rd_ptr_rx != wr_ptr_rx - 1'b1) begin
                rd_ptr_rx <= rd_ptr_rx + 1'b1;
            end
            if (rd_ptr_rx != wr_ptr_rx - 1'b1) begin
                rx_irq <= 1'd1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            bus_data_out <= 32'd0;
            rx_read <= 1'b0;
        end
        else begin
            rx_read <= 1'b0;
            if (bus_addr_in[23:20] == 4'b0001 && !bus_we_in) begin
                bus_data_out <= rx_buf[rd_ptr_rx + 1'b1];
                rx_read <= 1'b1;
            end
            else if (bus_addr_in[23:20] == 4'b0010 && !bus_we_in) begin
                bus_data_out <= {30'd0, tx_busy, rx_cont};
            end
            else if (bus_addr_in[23:20] == 4'b0011 && !bus_we_in) begin
                bus_data_out <= {18'd0, mcnt};
            end
        end
    end

    always @(*) begin
        tx_busy = 1'b0;
        rx_cont = 1'b0;
        tx_en = 1'b0;
        tx_data = 8'b0;
        if (wr_ptr_tx == rd_ptr_tx) tx_busy = 1'b1;
        if (rd_ptr_rx != wr_ptr_rx - 1'b1) rx_cont = 1'b1;
        if (!busy && rd_ptr_tx != wr_ptr_tx - 1'b1) begin
            tx_en = 1'b1;
            tx_data = tx_buf[rd_ptr_tx + 1'b1];
        end
    end

    uart_tx u_uart_tx(
        .clk(clk),
        .rst(rst),
        .tx_en(tx_en),
        .tx_data(tx_data),
        .mcnt(mcnt),
        //
        .tx(tx),
        .busy(busy)
    );

    uart_rx u_uart_rx(
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .mcnt(mcnt),
        .mcnt_half(mcnt_half),
        .mcnt_3q(mcnt_3q),
        //
        .rx_done(rx_done),
        .rx_data(rx_data)
    );
endmodule
