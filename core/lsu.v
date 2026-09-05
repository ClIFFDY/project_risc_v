`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/29 19:30:28
// Design Name:
// Module Name: lsu
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


module lsu(
    input clk, rst,
    input [1:0] stage,
    input [6:0] opcode,
    input [9:0] func10,
    input [4:0] rd_in, r1_fast, r2_fast,
    input [31:0] r1_data_final, r2_data_final,
    input [31:0] offset_load0, offset_store0,
    input [31:0] bus_data_in,
    input dtcm_loaded, bus_loaded_in,
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out,
    output reg we,
    output reg [4:0] rd_out,
    output reg [31:0] ld_data_out,
    output reg loaded,
    output reg stall
    );

    reg [2:0] ld_size_a, ld_size_q;
    reg [1:0] ld_off_a, ld_off_q;
    reg ld_valid_a, ld_valid_q;
    reg [4:0] rd_del, rd_del2;

    reg [31:0] st_addr;

    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STORE = 7'b0100011;

        localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

    always @(posedge clk) begin
        if (rst) begin
            bus_addr_out <= 30'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            we <= 1'b0;
            rd_out <= 5'd0;
        end
        else if (stage == STALL) begin
            if (we && !loaded) begin
                bus_addr_out <= bus_addr_out;
                bus_data_out <= bus_data_out;
                bus_be_out <= bus_be_out;
                bus_we_out <= bus_we_out;
                we <= we;
                rd_out <= rd_out;
            end
            else if (opcode == OPCODE_LOAD && rd_in != rd_out) begin
                we <= 1'b1;
                rd_out <= rd_in;
                bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                bus_we_out <= 1'b0;
                bus_be_out <= 4'd0;
                bus_data_out <= 32'd0;
            end
            else begin
                bus_addr_out <= bus_addr_out;
                bus_data_out <= bus_data_out;
                bus_be_out <= bus_be_out;
                bus_we_out <= bus_we_out;
                we <= we;
                rd_out <= rd_out;
            end
        end
        else if (stage == EXE) begin
            bus_addr_out <= 30'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            we <= 1'b0;
            rd_out <= 5'd0;
            case (opcode)
            OPCODE_LOAD: begin
                if (rd_in != rd_out) begin
                    we <= 1'b1;
                    rd_out <= rd_in;
                    case (func10[2:0])
                    3'b000: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                    3'b001: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                    3'b010: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                    3'b100: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                    3'b101: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                    default: bus_addr_out <= 30'd0;
                    endcase
                end
            end
            OPCODE_STORE: begin
                bus_we_out <= 1'b1;
                case (func10[2:0])
                3'b000: begin
                    bus_addr_out <= st_addr >> 2;
                    bus_be_out <= 4'b0001 << st_addr[1:0];
                    bus_data_out <= {24'd0, r2_data_final[7:0]} << (8 * st_addr[1:0]);
                end
                3'b001: begin
                    bus_addr_out <= st_addr >> 2;
                    bus_be_out <= 4'b0011 << (2 * st_addr[1]);
                    bus_data_out <= {16'd0, r2_data_final[15:0]} << (16 * st_addr[1]);
                end
                3'b010: begin
                    bus_addr_out <= (r1_data_final + offset_store0) >> 2;
                    bus_be_out <= 4'b1111;
                    bus_data_out <= r2_data_final;
                end
                default: bus_addr_out <= 30'd0;
                endcase
            end
            endcase
        end
        else begin
            bus_addr_out <= 30'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            we <= 1'b0;
            rd_out <= 5'd0;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            rd_del <= 5'd0;
            rd_del2 <= 5'd0;
        end
        else begin
            rd_del <= rd_out;
            rd_del2 <= rd_del;
        end
    end

    always @(*) begin
        st_addr = r1_data_final + offset_store0;
        stall = (opcode == OPCODE_LOAD) && ((rd_in == r1_fast) | (rd_in == r2_fast)) && !(rd_del2 == rd_in);
    end

    always @(posedge clk) begin
        if (rst) begin
            ld_valid_a <= 1'b0;
            ld_valid_q <= 1'b0;
            ld_size_a <= 3'd0;
            ld_off_a <= 2'd0;
            ld_size_q <= 3'd0;
            ld_off_q <= 2'd0;
        end
        else begin
            ld_valid_q <= ld_valid_a;
            ld_size_q <= ld_size_a;
            ld_off_q <= ld_off_a;
            if (opcode == OPCODE_LOAD) begin
                ld_valid_a <= 1'b1;
                ld_size_a <= func10[2:0];
                ld_off_a <= r1_data_final + offset_load0;
            end
            else begin
                ld_valid_a <= 1'b0;
                ld_size_a <= 3'd0;
                ld_off_a <= 2'd0;
            end
        end
    end

    always @(*) begin
        if (ld_valid_q) begin
            case (ld_size_q)
            3'b000: ld_data_out = {{24{bus_data_in[8*ld_off_q + 7]}}, bus_data_in[8*ld_off_q +: 8]};
            3'b001: ld_data_out = {{16{bus_data_in[16*ld_off_q[1] + 15]}}, bus_data_in[16*ld_off_q[1] +: 16]};
            3'b010: ld_data_out = bus_data_in;
            3'b100: ld_data_out = {24'd0, bus_data_in[8*ld_off_q +: 8]};
            3'b101: ld_data_out = {16'd0, bus_data_in[16*ld_off_q[1] +: 16]};
            default: ld_data_out = bus_data_in;
            endcase
        end
        else begin
            ld_data_out = bus_data_in;
        end
    end

    always @(*) begin
        loaded = dtcm_loaded | bus_loaded_in;
    end
endmodule
