`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/01 13:04:16
// Design Name:
// Module Name: mem_buf
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


module mem_buf(
    input clk, rst,
    input [1:0] stage,
    input [9:0] func10_in,
    input [31:0] imm_alu_in,
    input [11:0] imm12_csr_in,
    input [4:0] imm5_csr_in,
    input [4:0] rd_in,
    input [6:0] opcode_in,
    input [31:0] offset_jalr0_in,
    input [31:0] offset_beq0_aux_in,
    input [31:0] pc_operand_in,
    input [31:0] aux_addr_in,
    input [6:0] opcode_lsu_in,
    input [9:0] func10_lsu_in,
    input [31:0] offset_load0_in,
    input [31:0] offset_store0_in,
    input [4:0] r1_in,
    input [4:0] r2_in,
    output reg [9:0] func10_out,
    output reg [31:0] imm_alu_out,
    output reg [11:0] imm12_csr_out,
    output reg [4:0] imm5_csr_out,
    output reg [4:0] rd_out,
    output reg [6:0] opcode_out,
    output reg [31:0] offset_jalr0_out,
    output reg [31:0] offset_beq0_aux_out,
    output reg [31:0] pc_operand_out,
    output reg [31:0] aux_addr_out,
    output reg [6:0] opcode_lsu_out,
    output reg [9:0] func10_lsu_out,
    output reg [31:0] offset_load0_out,
    output reg [31:0] offset_store0_out,
    output reg [4:0] r1_out,
    output reg [4:0] r2_out
    );

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

//指令缓冲级，对齐寄存器组数据访问
    always @(posedge clk) begin
        if (rst) begin
            func10_out <= 10'd0;
            imm_alu_out <= 32'd0;
            imm12_csr_out <= 12'd0;
            imm5_csr_out <= 5'd0;
            rd_out <= 5'd0;
            opcode_out <= 7'd0;
            offset_jalr0_out <= 32'd0;
            offset_beq0_aux_out <= 32'd0;
            pc_operand_out <= 32'd0;
            aux_addr_out <= 32'd0;
            opcode_lsu_out <= 7'd0;
            func10_lsu_out <= 10'd0;
            offset_load0_out <= 32'd0;
            offset_store0_out <= 32'd0;
            r1_out <= 5'd0;
            r2_out <= 5'd0;
        end
        else if (stage == STALL) begin
            func10_out <= func10_out;
            imm_alu_out <= imm_alu_out;
            imm12_csr_out <= imm12_csr_out;
            imm5_csr_out <= imm5_csr_out;
            rd_out <= rd_out;
            opcode_out <= opcode_out;
            offset_jalr0_out <= offset_jalr0_out;
            offset_beq0_aux_out <= offset_beq0_aux_out;
            pc_operand_out <= pc_operand_out;
            aux_addr_out <= aux_addr_out;
            opcode_lsu_out <= opcode_lsu_out;
            func10_lsu_out <= func10_lsu_out;
            offset_load0_out <= offset_load0_out;
            offset_store0_out <= offset_store0_out;
            r1_out <= r1_out;
            r2_out <= r2_out;
        end
        else if (stage == FLUSH) begin
            func10_out <= 10'd0;
            imm_alu_out <= 32'd0;
            imm12_csr_out <= 12'd0;
            imm5_csr_out <= 5'd0;
            rd_out <= 5'd0;
            opcode_out <= 7'd0;
            offset_jalr0_out <= 32'd0;
            offset_beq0_aux_out <= 32'd0;
            pc_operand_out <= 32'd0;
            aux_addr_out <= 32'd0;
            opcode_lsu_out <= 7'd0;
            func10_lsu_out <= 10'd0;
            offset_load0_out <= 32'd0;
            offset_store0_out <= 32'd0;
            r1_out <= 5'd0;
            r2_out <= 5'd0;
        end
        else begin
            func10_out <= func10_in;
            imm_alu_out <= imm_alu_in;
            imm12_csr_out <= imm12_csr_in;
            imm5_csr_out <= imm5_csr_in;
            rd_out <= rd_in;
            opcode_out <= opcode_in;
            offset_jalr0_out <= offset_jalr0_in;
            offset_beq0_aux_out <= offset_beq0_aux_in;
            pc_operand_out <= pc_operand_in;
            aux_addr_out <= aux_addr_in;
            opcode_lsu_out <= opcode_lsu_in;
            func10_lsu_out <= func10_lsu_in;
            offset_load0_out <= offset_load0_in;
            offset_store0_out <= offset_store0_in;
            r1_out <= r1_in;
            r2_out <= r2_in;
        end
    end
endmodule
