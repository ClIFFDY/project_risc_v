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
    input [4:0] r1, r2, rd,
    input [31:0] rd_data, ld_data,
    input we, loaded,
    input dec, lsu,
    output reg [31:0] r1_data_dec, r2_data_dec, r1_data_lsu, r2_data_lsu
    );

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

    reg [31:0] regs [0:31];
    reg [15:0] irq_buf [0:31];
    reg [4:0] irq_str;
    reg [31:0] wdata_final;

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
            irq_buf[i] = 16'd0;
        end
    end

    always @(*) begin
        if (loaded) wdata_final = ld_data;
        else wdata_final = rd_data;
    end

    always @(posedge clk) begin
        if (rst) begin
            r1_data_dec <= 32'd0;
            r2_data_dec <= 32'd0;
            r1_data_lsu <= 32'd0;
            r2_data_lsu <= 32'd0;
        end
        else begin
            r1_data_dec <= 32'd0;
            r2_data_dec <= 32'd0;
            r1_data_lsu <= 32'd0;
            r2_data_lsu <= 32'd0;
            if (stage != STALL) begin
                if (dec) begin
                    r1_data_dec <= (((loaded || we) && rd != 5'd0 && r1 == rd) ? wdata_final : regs[r1]);
                    r2_data_dec <= (((loaded || we) && rd != 5'd0 && r2 == rd) ? wdata_final : regs[r2]);
                end
                else if (lsu) begin
                    r1_data_lsu <= (((loaded || we) && rd != 5'd0 && r1 == rd) ? wdata_final : regs[r1]);
                    r2_data_lsu <= (((loaded || we) && rd != 5'd0 && r2 == rd) ? wdata_final : regs[r2]);
                end
            end
            else begin
                r1_data_dec <= r1_data_dec;
                r2_data_dec <= r2_data_dec;
                r1_data_lsu <= r1_data_lsu;
                r2_data_lsu <= r2_data_lsu;
            end
        end
    end

    always @(posedge clk) begin
        if ((loaded || we) && rd != 5'd0) regs[rd] <= wdata_final;
    end
endmodule
