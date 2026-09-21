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
    input [4:0] flag_bus,
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
    input br_pred_taken_in,
    input [31:0] jalr_pred_addr_in,
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
    output reg br_pred_taken_out,
    output reg [31:0] jalr_pred_addr_out,
    output reg [6:0] opcode_lsu_out,
    output reg [9:0] func10_lsu_out,
    output reg [31:0] offset_load0_out,
    output reg [31:0] offset_store0_out,
    output reg [4:0] r1_out,
    output reg [4:0] r2_out
    );

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[3] | flag_bus[2];
        stall_w = (flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

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
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
        end
        else if (exec) begin
            if (stall_w) begin
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
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
            end
            else if (flush_w) begin
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
                br_pred_taken_out <= 1'b0;
                jalr_pred_addr_out <= 32'd0;
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
                br_pred_taken_out <= br_pred_taken_in;
                jalr_pred_addr_out <= jalr_pred_addr_in;
            end
        end
    end
endmodule
