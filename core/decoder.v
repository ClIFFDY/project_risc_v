`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: decoder
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


module decoder(
    input clk, rst,
    input [1:0] stage,
    input [9:0] func10,
    input [4:0] rd_in,
    input [6:0] opcode,
    input [31:0] imm_alu_in, r1_data_final, r2_data_final,
    input [11:0] imm12_csr_in,
    input [4:0] imm5_csr_in,
    input [31:0] offset_jalr0, offset_beq0_aux, pc_operand_in,
    input [31:0] aux_addr_in,
    output reg [31:0] r1_data_out, r2_data_out,
    output reg [31:0] jalr_target_q, beq_off_q1, beq_off_q2,
    output reg [4:0] rd_out, rd_back1, csr_addr_back1,
    output reg [3:0] alu_func4,
    output reg [2:0] csr_func3,
    output reg we, jalr, br_fail, success, irq_ret, trap, ebreak, jal_flag, jalr_flag,
    output reg csr_wr_en,
    output reg [11:0] csr_addr,
    output reg [31:0] csr_data,
    output reg [31:0] aux_addr_out
    );
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

    always @(posedge clk) begin
        if (rst) begin
            r1_data_out <= 32'd0;
            r2_data_out <= 32'd0;
            rd_out <= 5'd0;
            rd_back1 <= 5'd0;
            alu_func4 <= 4'd0;
            we <= 1'd0;
            csr_wr_en <= 1'b0;
            csr_addr <= 12'd0;
            csr_data <= 32'd0;
            br_fail <=1'b0;
            success <= 1'b0;
            irq_ret <= 1'b0;
            trap <= 1'b0;
            ebreak <= 1'b0;
            jal_flag <= 1'b0;
            jalr_flag <= 1'd0;
            jalr_target_q <= 32'd0;
            beq_off_q1 <= 32'd0;
            beq_off_q2 <= 32'd0;
        end
        else begin
            if (stage == EXE) begin
                r1_data_out <= 32'd0;
                r2_data_out <= 32'd0;
                rd_out <= 5'd0;
                rd_back1 <= 5'd0;
                alu_func4 <= 4'd0;
                csr_func3 <= 3'd0;
                we <= 1'd0;
                csr_wr_en <= 1'b0;
                csr_addr <= 12'd0;
                csr_data <= 32'd0;
                jalr <= 1'd0;
                br_fail <=1'b0;
                success <= 1'b0;
                irq_ret <= 1'b0;
                trap <= 1'b0;
                ebreak <= 1'b0;
                jal_flag <= 1'b0;
                jalr_flag <= 1'b0;
                jalr_target_q <= 32'd0;
                beq_off_q1 <= 32'd0;
                beq_off_q2 <= 32'd0;
                aux_addr_out <= 16'd0;
                case (opcode)
                OPCODE_OP: begin
                    r1_data_out <= r1_data_final;
                    r2_data_out <= r2_data_final;
                    rd_out <= rd_in;
                    rd_back1 <= rd_in;
                    alu_func4 <= {func10[8], func10[2:0]};
                    we <= 1'd1;
                end
                OPCODE_OP_IMM: begin
                    r1_data_out <= r1_data_final;
                    r2_data_out <= imm_alu_in;
                    rd_out <= rd_in;
                    rd_back1 <= rd_in;
                    alu_func4 <= {func10[8], func10[2:0]};
                    we <= 1'd1;
                end
                OPCODE_JAL: begin
                    rd_out <= rd_in;
                    we <= 1'd1;
                    aux_addr_out <= aux_addr_in;
                    jal_flag <= 1'd1;
                end
                OPCODE_JALR: begin
                    jalr <= 1'd1;
                    rd_out <= rd_in;
                    we <= 1'd1;
                    jalr_flag <= 1'b1;
                    aux_addr_out <= aux_addr_in;
                    jalr_target_q <= r1_data_final + offset_jalr0;
                end
                OPCODE_BRANCH: begin
                    beq_off_q1 <= offset_beq0_aux;
                    beq_off_q2 <= offset_beq0_aux - 4'd4;
                    case (func10[2:0])
                        3'b000: begin
                            if (r1_data_final == r2_data_final) success <= 1'd1;
                            else br_fail <= 1'd1;
                        end
                        3'b001: begin
                            if (r1_data_final != r2_data_final) success <= 1'd1;
                            else br_fail <= 1'd1;
                        end
                        3'b100: begin
                            if ($signed(r1_data_final) < $signed(r2_data_final)) success <= 1'd1;
                            else br_fail <= 1'd1;
                        end
                        3'b101: begin
                            if ($signed(r1_data_final) >= $signed(r2_data_final)) success <= 1'd1;
                            else br_fail <= 1'd1;
                        end
                        3'b110: begin
                            if (r1_data_final < r2_data_final) success <= 1'd1;
                            else br_fail <= 1'd1;
                        end
                        3'b111: begin
                            if (r1_data_final >= r2_data_final) success <= 1'd1;
                            else br_fail <= 1'd1;
                        end
                        default: br_fail <= 1'b0;
                    endcase
                end
                OPCODE_LUI: begin
                    r1_data_out <= 32'd0;
                    r2_data_out <= imm_alu_in;
                    rd_out <= rd_in;
                    rd_back1 <= rd_in;
                    alu_func4 <= 4'd0;
                    we <= 1'd1;
                end
                OPCODE_AUIPC: begin
                    r1_data_out <= pc_operand_in;
                    r2_data_out <= imm_alu_in;
                    rd_out <= rd_in;
                    rd_back1 <= rd_in;
                    alu_func4 <= 4'd0;
                    we <= 1'd1;
                end
                OPCODE_SYSTEM: begin
                    rd_out <= rd_in;
                    rd_back1 <= rd_in;
                    csr_func3 <= func10[2:0];
                    irq_ret <= (func10[2:0] == 3'b000) && (imm12_csr_in == 12'h302);
                    csr_addr <= imm12_csr_in[11:0];
                    case (func10[2:0])
                    3'b000: begin
                        we <= 1'b0;
                        csr_wr_en <= 1'b0;
                        trap <= (imm12_csr_in == 12'h000) || (imm12_csr_in == 12'h001);
                        ebreak <= (imm12_csr_in == 12'h001);
                    end
                    3'b001: begin
                        we <= 1'b1;
                        csr_wr_en <= 1'b1;
                        r1_data_out <= r1_data_final;
                    end
                    3'b010: begin
                        we <= 1'b1;
                        csr_wr_en <= 1'b1;
                        r1_data_out <= r1_data_final;
                    end
                    3'b011: begin
                        we <= 1'b1;
                        csr_wr_en <= 1'b1;
                        r1_data_out <= r1_data_final;
                    end
                    3'b101: begin
                        we <= 1'b1;
                        csr_wr_en <= 1'b1;
                        r1_data_out <= {27'd0, imm5_csr_in};
                    end
                    3'b110: begin
                        we <= 1'b1;
                        csr_wr_en <= 1'b1;
                        r1_data_out <= {27'd0, imm5_csr_in};
                    end
                    3'b111: begin
                        we <= 1'b1;
                        csr_wr_en <= 1'b1;
                        r1_data_out <= {27'd0, imm5_csr_in};
                    end
                    default: begin
                        we <= 1'b0;
                        csr_wr_en <= 1'b0;
                    end
                    endcase
                end
                endcase
            end
            else if (stage == STALL) begin
                r1_data_out <= 32'd0;
                r2_data_out <= 32'd0;
                rd_out <= 5'd0;
                rd_back1 <= 5'd0;
                alu_func4 <= 4'd0;
                we <= 1'b0;
                csr_wr_en <= 1'b0;
                csr_addr <= 12'd0;
                csr_data <= 32'd0;
                jalr <= 1'b0;
                br_fail <= 1'b0;
                success <= 1'b0;
                irq_ret <= 1'b0;
                trap <= 1'b0;
                ebreak <= 1'b0;
                jal_flag <= 1'b0;
                jalr_flag <= 1'b0;
                jalr_target_q <= 32'd0;
                beq_off_q1 <= 32'd0;
                beq_off_q2 <= 32'd0;
                aux_addr_out <= 32'd0;
            end
            else begin
                r1_data_out <= 32'd0;
                r2_data_out <= 32'd0;
                rd_out <= 5'd0;
                rd_back1 <= 5'd0;
                alu_func4 <= 4'd0;
                we <= 1'd0;
                csr_wr_en <= 1'b0;
                csr_addr <= 12'd0;
                csr_data <= 32'd0;
                jalr <= 1'd0;
                br_fail <=1'b0;
                success <= 1'b0;
                irq_ret <= 1'b0;
                trap <= 1'b0;
                ebreak <= 1'b0;
                jal_flag <= 1'd0;
                jalr_flag <= 1'd0;
                jalr_target_q <= 32'd0;
                beq_off_q1 <= 32'd0;
                beq_off_q2 <= 32'd0;
            end
        end
    end
endmodule
