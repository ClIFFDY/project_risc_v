`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: pc
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


module pc(
    input clk, rst,
    input br1, br2, br3,
    input jal, jalr, jalr_fail, irq, irq_ret,
    input [1:0] stage,
    input [31:0] offset_jal2, offset_jalr2,
    input [31:0] offset_beq2, isr_addr2, isr_ret_addr2,
    input [31:0] jalr_target_q, beq_off_q1,
    input [31:0] br_addr2,
    output reg [31:0] pc_addr, aux_addr
    );

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

//程序计数器，传递取指地址
    always @(posedge clk) begin
        if (rst) begin
            pc_addr <= 16'd0;
            aux_addr <= 16'd0;
        end
        else begin
            case (stage)
            EXE: begin
                if (br1) begin
                    pc_addr <= pc_addr + offset_beq2;
                    aux_addr <= aux_addr + offset_beq2;
                end
                else if (jal) begin
                    pc_addr <= pc_addr + offset_jal2;
                    aux_addr <= pc_addr + offset_jal2;
                end
                else if (jalr) begin
                    pc_addr <= offset_jalr2;
                    aux_addr <= offset_jalr2;
                end
                else begin
                    pc_addr <= pc_addr + 4'd4;
                    aux_addr <= pc_addr + 4'd4;
                end
            end
            FLUSH: begin
                if (br2) begin
                    pc_addr <= br_addr2 + beq_off_q1 - 4'd4;
                    aux_addr <= br_addr2 + beq_off_q1 - 4'd4;
                end
                else if (br3) begin
                    pc_addr <= br_addr2;
                    aux_addr <= br_addr2;
                end
                else if (jalr_fail) begin
                    pc_addr <= jalr_target_q + 4'd4;
                    aux_addr <= jalr_target_q + 4'd4;
                end
                else if (irq) begin
                    pc_addr <= isr_addr2;
                    aux_addr <= isr_addr2;
                end
                else if (irq_ret) begin
                    pc_addr <= isr_ret_addr2;
                    aux_addr <= isr_ret_addr2;
                end
            end
            STALL: begin
                pc_addr <= pc_addr;
                aux_addr <= aux_addr;
            end
            default: begin
                pc_addr <= 4;
                aux_addr <= 4;
            end
            endcase
        end
    end

endmodule
