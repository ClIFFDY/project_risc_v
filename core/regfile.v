`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: regfile
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


module regfile(
    input clk, rst,
    input [1:0] stage,
    input [4:0] r1, r2,
    input [4:0] rd_alu,
    input [31:0] rd_data_alu,
    input we_alu,
    input [4:0] rd_ld,
    input [31:0] ld_data_ld,
    input we_ld,
    input dec, lsu,
    output reg [31:0] r1_data_dec, r2_data_dec, r1_data_lsu, r2_data_lsu
    );

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

//同步读写型通用寄存器组，节省lut资源
    (* ram_style = "block" *) reg [31:0] regs [0:31];

    reg [4:0] r1_q, r2_q;
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
        end
    end

//读数据进行双写口(alu/ld)旁路仲裁并输出
    always @(posedge clk) begin
        if (rst) begin
            r1_data_dec <= 32'd0;
            r2_data_dec <= 32'd0;
            r1_data_lsu <= 32'd0;
            r2_data_lsu <= 32'd0;
            r1_q <= 5'd0;
            r2_q <= 5'd0;
        end
        else if (stage == STALL) begin
            r1_data_dec <= bypass(r1_q);
            r2_data_dec <= bypass(r2_q);
            r1_data_lsu <= bypass(r1_q);
            r2_data_lsu <= bypass(r2_q);
        end
        else begin
            r1_data_dec <= 32'd0;
            r2_data_dec <= 32'd0;
            r1_data_lsu <= 32'd0;
            r2_data_lsu <= 32'd0;
            r1_q <= r1;
            r2_q <= r2;
            if (dec) begin
                r1_data_dec <= bypass(r1);
                r2_data_dec <= bypass(r2);
            end
            else if (lsu) begin
                r1_data_lsu <= bypass(r1);
                r2_data_lsu <= bypass(r2);
            end
        end
    end

//双写口：ALU(rd_alu与load(rd_ld)各自写回，同拍互不影响
    always @(posedge clk) begin
        if (we_alu && rd_alu != 5'd0) regs[rd_alu] <= rd_data_alu;
        if (we_ld && rd_ld != 5'd0) regs[rd_ld] <= ld_data_ld;
    end

    function [31:0] bypass;
        input [4:0] rx;
        begin
            if (we_alu && rd_alu != 5'd0 && rx == rd_alu) bypass = rd_data_alu;
            else if (we_ld && rd_ld != 5'd0 && rx == rd_ld) bypass = ld_data_ld;
            else bypass = regs[rx];
        end
    endfunction
endmodule
