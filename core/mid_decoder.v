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


module mid_decoder(
    input clk, rst,
    input [4:0] flag_bus,
    input [31:0] inst_in,
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
    output reg [31:0] inst_out,
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

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_SYSTEM = 7'b1110011;

    function [31:0] immI;
        input [31:0] inst;
        immI = {{20{inst[31]}}, inst[31:20]};
    endfunction

    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immS;
        input [31:0] inst;
        immS = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    endfunction

    function [31:0] immU;
        input [31:0] inst;
        immU = {inst[31:12], 12'd0};
    endfunction

    reg [4:0] rd_c;
    reg [9:0] func10_c;
    reg [31:0] imm_c;
    always @(*) begin
        rd_c = 5'd0;
        func10_c = 10'd0;
        imm_c = 32'd0;
        case (inst_in[6:0])
            OPCODE_OP: begin
                rd_c = inst_in[11:7];
                func10_c = {inst_in[31:25], inst_in[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_c = immI(inst_in);
                rd_c = inst_in[11:7];
                func10_c = {(inst_in[14:12] == 3'b001 || inst_in[14:12] == 3'b101) ? inst_in[31:25] : 7'd0, inst_in[14:12]};
            end
            OPCODE_JAL: begin
                rd_c = inst_in[11:7];
            end
            OPCODE_JALR: begin
                rd_c = inst_in[11:7];
                imm_c = immI(inst_in);
            end
            OPCODE_BRANCH: begin
                imm_c = immB(inst_in);
                func10_c = {7'd0, inst_in[14:12]};
            end
            OPCODE_LOAD: begin
                imm_c = immI(inst_in);
                rd_c = inst_in[11:7];
            end
            OPCODE_STORE: begin
                imm_c = immS(inst_in);
            end
            OPCODE_LUI: begin
                imm_c = immU(inst_in);
                rd_c = inst_in[11:7];
            end
            OPCODE_AUIPC: begin
                imm_c = immU(inst_in) - 4'd4;
                rd_c = inst_in[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_c = inst_in[11:7];
                func10_c = {7'd0, inst_in[14:12]};
            end
        endcase
    end

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[3] | flag_bus[2];
        stall_w = (flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

//指令缓冲级，对齐寄存器组数据访问。
//寄存器只留跨拍真正要用的：指令字本身 + 一个 opcode 选择好的立即数 + PC 载荷/预测两件
//+ regfile 要用的 rd 与前递要用的 rs。其余字段（opcode/func10/offset_*/pc_operand/立即数各位）
//全部由 inst_out 就地切片/别名给出 —— 原来它们是各自寄存一遍的（18 个寄存器 323 位）。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out <= 32'd0;
            func10_out <= 10'd0;
            imm_alu_out <= 32'd0;
            rd_out <= 5'd0;
            aux_addr_out <= 32'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
            r1_out <= 5'd0;
            r2_out <= 5'd0;
        end
        else if (exec) begin
            if (stall_w) begin
                inst_out <= inst_out;
                func10_out <= func10_out;
                imm_alu_out <= imm_alu_out;
                rd_out <= rd_out;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
                r1_out <= r1_out;
                r2_out <= r2_out;
            end
            else if (flush_w) begin
                inst_out <= 32'd0;
                func10_out <= 10'd0;
                imm_alu_out <= 32'd0;
                rd_out <= 5'd0;
                aux_addr_out <= 32'd0;
                br_pred_taken_out <= 1'b0;
                jalr_pred_addr_out <= 32'd0;
                r1_out <= 5'd0;
                r2_out <= 5'd0;
            end
            else begin
                inst_out <= inst_in;
                func10_out <= func10_c;
                imm_alu_out <= imm_c;
                rd_out <= rd_c;
                aux_addr_out <= aux_addr_in;
                br_pred_taken_out <= br_pred_taken_in;
                jalr_pred_addr_out <= jalr_pred_addr_in;
                r1_out <= r1_in;
                r2_out <= r2_in;
            end
        end
    end

//派生输出：切片与别名，无逻辑。消费者都按 opcode 自门控，所以"每个 opcode 只看它那一份"
//（偏移族四个输出共用同一个 imm_alu_out：一条指令只有一个立即数 flavour 是活的）。
    always @(*) begin
        opcode_out = inst_out[6:0];
        opcode_lsu_out = inst_out[6:0];
        func10_lsu_out = {7'd0, inst_out[14:12]};
        imm12_csr_out = inst_out[31:20];
        imm5_csr_out = inst_out[19:15];
        pc_operand_out = aux_addr_out;
        offset_jalr0_out = imm_alu_out;
        offset_beq0_aux_out = imm_alu_out;
        offset_load0_out = imm_alu_out;
        offset_store0_out = imm_alu_out;
    end

endmodule
