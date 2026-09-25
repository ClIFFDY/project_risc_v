`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/30 23:44:18
// Design Name:
// Module Name: tim_in
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


module tim_in(
    input clk, rst,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    output reg [31:0] bus_data_out,
    output reg timi,
    output reg ready
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//内部时钟定时器，独占timi内部中断
    reg [31:0] cnt_set, cnt;

    always @(posedge clk) begin
        if (rst_q) begin
            cnt_set <= 32'd0;
            cnt <= 32'd0;
            timi <= 1'b0;
            bus_data_out <= 32'd0;
            ready <= 1'b0;
        end
        else begin
            bus_data_out <= 32'd0;
            ready <= 1'b0;
            if (cnt + 32'd1 == cnt_set) begin
                timi <= 1'b1;
                cnt <= 32'd0;
            end
            else begin
                cnt <= cnt + 32'd1;
            end
//字节使能写重装值设定
            if (bus_addr_in[31:24] == 8'd0 && bus_addr_in[23:20] == 4'd1) begin
                if (bus_addr_in[3:0] == 4'd1) begin
                    if (bus_we_in) begin
                        if (bus_be_in[0]) cnt_set[7:0] <= bus_data_in[7:0];
                        if (bus_be_in[1]) cnt_set[15:8] <= bus_data_in[15:8];
                        if (bus_be_in[2]) cnt_set[23:16] <= bus_data_in[23:16];
                        if (bus_be_in[3]) cnt_set[31:24] <= bus_data_in[31:24];
                    end
                    else begin
                        bus_data_out <= cnt_set;
                        ready <= 1'b1;
                    end
                end
                else if (bus_addr_in[3:0] == 4'd2) begin
                    if (!bus_we_in) begin
                        bus_data_out <= cnt;
                        ready <= 1'b1;
                    end
                end
//写指令清除中断挂起
                else if (bus_addr_in[3:0] == 4'd3) begin
                    if (bus_we_in) timi <= 1'b0;
                end
            end
        end
    end
endmodule
