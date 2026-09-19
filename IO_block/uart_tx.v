`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/08/30 22:06:59
// Design Name: 
// Module Name: uart_tx
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


module uart_tx(
    input clk,
    input rst,
    input tx_en,
    input [7:0] tx_data,
    input [13:0] mcnt,
    output reg tx,
    output reg busy
    );

    reg [15:0] clk_cnt;
    reg [3:0] bit_cnt;
    reg [7:0] tx_data_buf;

    always @(posedge clk) begin
        if (rst) begin
            clk_cnt <= 16'b0;
            bit_cnt <= 4'b0;
            tx <= 1'b1;
            busy <= 1'b0;
            tx_data_buf <= 8'b0;
        end
        else begin
            if (!busy && tx_en) begin
                busy <= 1'b1;
                tx_data_buf <= tx_data;
                bit_cnt <= 4'b0;
                clk_cnt <= 16'b0;
                tx <= 1'b0;
            end
            else if (busy) begin
                if(clk_cnt < mcnt - 1'b1) begin
                    clk_cnt <= clk_cnt + 1;
                end
                else begin
                    clk_cnt <= 16'b0;
                    if (bit_cnt <= 4'd7) begin
                        tx <= tx_data_buf[0];
                        tx_data_buf <= {1'b0, tx_data_buf[7:1]};
                        bit_cnt <= bit_cnt + 1;
                    end
                    else if (bit_cnt == 4'd8) begin
                        tx <= 1'b1;
                        bit_cnt <= bit_cnt + 1;
                    end
                    else begin
                        busy <= 1'b0;
                    end
                end                
            end            
        end
    end
endmodule
