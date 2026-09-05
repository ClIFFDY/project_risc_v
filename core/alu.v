`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: alu
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


module alu(
    input we_in, jal_flag, jalr_flag, cs_wr_en,
    input [4:0] rd_in,
    input [3:0] alu_func4,
    input [2:0] csr_func3,
    input [31:0] aux_addr_in,
    input [31:0] r1_data, r2_data, cs_data,
    output reg [4:0] rd_out,
    output reg [31:0] result, result_csr, result_back1,
    output reg we
    );

    localparam [3:0]
        ADD  = 4'b0000,
        SLL  = 4'b0001,
        SLT  = 4'b0010,
        SLTU = 4'b0011,
        XOR  = 4'b0100,
        SRL  = 4'b0101,
        OR   = 4'b0110,
        AND  = 4'b0111,
        SUB  = 4'b1000,
        SRA  = 4'b1101;

    localparam [2:0]
        CSRRW  = 3'b001,
        CSRRS  = 3'b010,
        CSRRC  = 3'b011,
        CSRRWI = 3'b101,
        CSRRSI = 3'b110,
        CSRSCI = 3'b111;
        
//SYSTEM类指令：cs_data源于csr寄存器，result_csr写回csr寄存器，result返回rd
    always @(*) begin
        if (we_in && cs_wr_en) begin
            rd_out = rd_in;
            we = we_in;
            result = cs_data;
            case (csr_func3)
            CSRRW, CSRRWI: result_csr = r1_data;
            CSRRS, CSRRSI: result_csr = r1_data | cs_data;
            CSRRC, CSRSCI: result_csr = cs_data & ~r1_data;
            default: result_csr = cs_data;
            endcase
        end
//链接跳转类指令：将pc作为result输出存入rd
        else if (we_in && (jal_flag | jalr_flag)) begin
            result = aux_addr_in;
            rd_out = rd_in;
            we = we_in;
            result_csr = r2_data;
        end
//ALU/I类指令：直接进行运算，result存入rd
        else if (we_in) begin
            case (alu_func4)
                ADD: result = r1_data + r2_data;
                SLL: result = r1_data << r2_data[4:0];
                SLT: result = ($signed(r1_data) < $signed(r2_data)) ? 32'd1 : 32'd0;
                SLTU: result = (r1_data < r2_data) ? 32'd1 : 32'd0;
                XOR: result = r1_data ^ r2_data;
                SRL: result = r1_data >> r2_data[4:0];
                OR: result = r1_data | r2_data;
                AND: result = r1_data & r2_data;
                SUB: result = r1_data - r2_data;
                SRA: result = $signed(r1_data) >>> r2_data[4:0];
                default: result = 32'd0;
            endcase
            rd_out = rd_in;
            we = we_in;
            result_csr = r2_data;
        end
        else begin
            rd_out = 5'd0;
            result = 32'd0;
            we = 1'd0;
            result_csr = r2_data;
        end
        result_back1 = result;
    end

endmodule
