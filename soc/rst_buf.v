`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/08/31 18:27:26
// Design Name: 
// Module Name: rst_buf
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


module rst_buf(
    input wire clk, rst_n,
    output reg rst_stable = 1'b0
    );

    localparam 
        IDLE = 1'b0,
        PUSH = 1'b1;

    reg rst_s1 = 1'b1;
    reg rst_s2 = 1'b1;
    reg stage = IDLE;

    always @(posedge clk) begin
        rst_s1 <= rst_n;
        rst_s2 <= rst_s1;
    end

    reg [21:0] rst_cnt = 22'b0;

        always @(posedge clk) begin
            case (stage)
            IDLE: begin
                rst_cnt <= 22'b0;
                rst_stable <= 1'b0;
                if (rst_s2 && ~rst_s1) begin 
                    stage <= PUSH;
                    rst_stable <= 1'b1;
                end
            end
            PUSH: begin
                rst_stable <= 1'b1;
                if (~rst_s2 && rst_s1) begin       
                    rst_cnt <= 22'b0;
                end
                else if (rst_s1) begin            
                    if (rst_cnt[21]) begin          
                        stage <= IDLE;
                        rst_cnt <= 22'b0;
                        rst_stable <= 1'b0;
                    end
                    else begin
                        rst_cnt <= rst_cnt + 1;
                    end
                end
            end
            endcase
        end
endmodule
