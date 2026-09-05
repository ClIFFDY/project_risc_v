`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 16:02:13
// Design Name:
// Module Name: pre_decoder
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


module pre_decoder(
    input clk, rst,
    input [1:0] stage,
    input [31:0] inst_raw_in,
    input is_ibus_in,
    input [31:0] ibus_data_in,
    input [31:0] aux_addr_in,
    output reg [4:0] r1, r2, r1_mem, r2_mem, rd,
    output reg [9:0] func10_dec, func10_lsu,
    output reg [31:0] imm_alu_out,
    output reg [11:0] imm12_csr_out,
    output reg [31:0] offset_jal1, offset_jal2, offset_beq1, offset_beq2, offset_beq0_aux, offset_jalr0,
    output reg [31:0] offset_load0, offset_store0,
    output reg [31:0] pc_operand,
    output reg [31:0] aux_addr_out,
    output reg [4:0] imm5_csr_out,
    output reg [6:0] opcode_dec, opcode_lsu,
    output reg jal, dec, lsu, br_en, jalr
    );

    reg [31:0] inst_effective;
    always @(*) inst_effective = is_ibus_in ? ibus_data_in : inst_raw_in;

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

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

    function [31:0] immI;
        input [31:0] inst;
        immI = {{20{inst[31]}}, inst[31:20]};
    endfunction

    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

    function [31:0] immS;
        input [31:0] inst;
        immS = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    endfunction

    function [31:0] immU;
        input [31:0] inst;
        immU = {inst[31:12], 12'd0};
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            func10_dec <= 10'd0;
            func10_lsu <= 10'd0;
            r1 <= 4'd0;
            r2 <= 4'd0;
            r1_mem <= 4'd0;
            r2_mem <= 4'd0;
            rd <= 5'd0;
            dec <= 1'd0;
            lsu <= 1'd0;
            imm_alu_out <= 32'd0;
            offset_jalr0 <= 32'd0;
            offset_beq0_aux <= 32'd0;
            offset_load0 <= 32'd0;
            offset_store0 <= 32'd0;
            aux_addr_out <= 16'd0;
            pc_operand <= 32'd0;
            imm12_csr_out <= 12'd0;
            imm5_csr_out <= 4'd0;
            opcode_dec <= 7'd0;
            opcode_lsu <= 7'd0;
        end
        else if (stage == EXE) begin
            func10_dec <= 10'd0;
            func10_lsu <= 10'd0;
            r1 <= 4'd0;
            r2 <= 4'd0;
            r1_mem <= 4'd0;
            r2_mem <= 4'd0;
            rd <= 5'd0;
            dec <= 1'd0;
            lsu <= 1'd0;
            imm_alu_out <= 32'd0;
            offset_jalr0 <= 32'd0;
            offset_beq0_aux <= 32'd0;
            offset_load0 <= 32'd0;
            offset_store0 <= 32'd0;
            aux_addr_out <= 16'd0;
            pc_operand <= 32'd0;
            imm12_csr_out <= 12'd0;
            imm5_csr_out <= 4'd0;
            opcode_dec <= 7'd0;
            opcode_lsu <= 7'd0;
            case (inst_effective[6:0])
            OPCODE_OP: begin
                r2 <= inst_effective[24:20];
                r1 <= inst_effective[19:15];
                r2_mem <= inst_effective[24:20];
                r1_mem <= inst_effective[19:15];
                rd <= inst_effective[11:7];
                func10_dec <= {inst_effective[31:25], inst_effective[14:12]};
                opcode_dec <= OPCODE_OP;
                dec <= 1'd1;
            end
            OPCODE_OP_IMM: begin
                imm_alu_out <= immI(inst_effective);
                r1 <= inst_effective[19:15];
                r1_mem <= inst_effective[19:15];
                rd <= inst_effective[11:7];
                func10_dec <= {(inst_effective[14:12] == 3'b001 || inst_effective[14:12] == 3'b101) ? inst_effective[31:25] : 7'd0, inst_effective[14:12]};
                opcode_dec <= OPCODE_OP_IMM;
                dec <= 1'd1;
            end
            OPCODE_JAL: begin
                rd <= inst_effective[11:7];
                opcode_dec <= OPCODE_JAL;
                aux_addr_out <= aux_addr_in;
            end
            OPCODE_JALR: begin
                rd <= inst_effective[11:7];
                r1 <= inst_effective[19:15];
                r1_mem <= inst_effective[19:15];
                offset_jalr0 <= immI(inst_effective);
                opcode_dec <= OPCODE_JALR;
                aux_addr_out <= aux_addr_in;
                dec <= 1'd1;
            end
            OPCODE_BRANCH: begin
                opcode_dec <= OPCODE_BRANCH;
                offset_beq0_aux <= immB(inst_effective);
                r2 <= inst_effective[24:20];
                r1 <= inst_effective[19:15];
                r2_mem <= inst_effective[24:20];
                r1_mem <= inst_effective[19:15];
                func10_dec <= {7'd0, inst_effective[14:12]};
                dec <= 1'd1;
            end
            OPCODE_LOAD: begin
                opcode_dec <= OPCODE_LOAD;
                opcode_lsu <= OPCODE_LOAD;
                offset_load0 <= immI(inst_effective);
                rd <= inst_effective[11:7];
                r1 <= inst_effective[19:15];
                r1_mem <= inst_effective[19:15];
                func10_lsu <= {7'd0, inst_effective[14:12]};
                lsu <= 1'd1;
            end
            OPCODE_STORE: begin
                opcode_dec <= OPCODE_STORE;
                opcode_lsu <= OPCODE_STORE;
                offset_store0 <= immS(inst_effective);
                r2 <= inst_effective[24:20];
                r1 <= inst_effective[19:15];
                r2_mem <= inst_effective[24:20];
                r1_mem <= inst_effective[19:15];
                func10_lsu <= {7'd0, inst_effective[14:12]};
                lsu <= 1'd1;
            end
            OPCODE_LUI: begin
                opcode_dec <= OPCODE_LUI;
                opcode_lsu <= OPCODE_LUI;
                imm_alu_out <= immU(inst_effective);
                rd <= inst_effective[11:7];
            end
            OPCODE_AUIPC: begin
                opcode_dec <= OPCODE_AUIPC;
                imm_alu_out <= immU(inst_effective);
                pc_operand <= aux_addr_in - 4'd4;
                rd <= inst_effective[11:7];
            end
            OPCODE_SYSTEM: begin
                opcode_dec <= OPCODE_SYSTEM;
                imm12_csr_out <= inst_effective[31:20];
                r1 <= inst_effective[19:15];
                r1_mem <= inst_effective[19:15];
                imm5_csr_out <= inst_effective[19:15];
                rd <= inst_effective[11:7];
                func10_dec <= {7'd0, inst_effective[14:12]};
                dec <= 1'd1;
            end
            endcase
        end
        else if (stage == STALL) begin
            r1 <= r1;
            r2 <= r2;
            r1_mem <= r1_mem;
            r2_mem <= r2_mem;
            rd <= rd;
            dec <= dec;
            lsu <= lsu;
            func10_dec <= func10_dec;
            func10_lsu <= func10_lsu;
            imm_alu_out <= imm_alu_out;
            imm12_csr_out <= imm12_csr_out;
            imm5_csr_out <= imm5_csr_out;
            offset_jalr0 <= offset_jalr0;
            offset_beq0_aux <= offset_beq0_aux;
            offset_load0 <= offset_load0;
            offset_store0 <= offset_store0;
            aux_addr_out <= aux_addr_out;
            pc_operand <= pc_operand;
            opcode_dec <= opcode_dec;
            opcode_lsu <= opcode_lsu;
        end
        else begin
            r1 <= 4'd0;
            r2 <= 4'd0;
            r1_mem <= 4'd0;
            r2_mem <= 4'd0;
            rd <= 5'd0;
            dec <= 1'd0;
            lsu <= 1'd0;
            func10_dec <= 10'd0;
            func10_lsu <= 10'd0;
            imm_alu_out <= 32'd0;
            offset_jalr0 <= 32'd0;
            offset_beq0_aux <= 32'd0;
            offset_load0 <= 32'd0;
            offset_store0 <= 32'd0;
            aux_addr_out <= 16'd0;
            pc_operand <= 32'd0;
            imm12_csr_out <= 12'd0;
            imm5_csr_out <= 4'd0;
            opcode_dec <= 7'd0;
            opcode_lsu <= 7'd0;
        end
    end

    always @(*) begin
        if (rst) begin
            br_en = 1'd0;
            offset_beq1 = 32'd0;
            offset_beq2 = 32'd0;
            offset_jal1 = 32'd0;
            offset_jal2 = 32'd0;
            jal = 1'd0;
            jalr = 1'd0;
        end
        else begin
            br_en = 1'd0;
            offset_beq1 = 32'd0;
            offset_beq2 = 32'd0;
            offset_jal1 = 32'd0;
            offset_jal2 = 32'd0;
            jal = 1'd0;
            jalr = 1'd0;
            if (stage == EXE || stage == STALL) begin
                case (inst_effective[6:0])
                OPCODE_JAL: begin
                    offset_jal1 = $signed(immJ(inst_effective)) - 4'd4;
                    offset_jal2 = $signed(immJ(inst_effective));
                    jal = 1'd1;
                end
                OPCODE_JALR: begin
                    jalr = 1'd1;
                end
                OPCODE_BRANCH: begin
                    offset_beq1 = $signed(immB(inst_effective)) - 4'd4;
                    offset_beq2 = $signed(immB(inst_effective));
                    br_en = 1'd1;
                end
                endcase
            end
        end
    end

endmodule
