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
    input [4:0] rd_in, r1_post, r2_post,
    input [31:0] r1_data_final, r2_data_final,
    input [31:0] offset_load0, offset_store0,
    input [31:0] bus_data_in,
    input ready_in,
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out,
    output reg we,
    output reg [31:0] ld_data_out,
    output reg loaded,
    output reg stall,
    output reg [4:0] rd_load
    );

    reg [4:0] rd_1, rd_2;
    reg [2:0] size_1, size_2;
    reg [1:0] off_1, off_2;
    reg [6:0] opcode_post;
    reg stalled;

    reg [31:0] st_addr;

    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STORE = 7'b0100011;

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

//ld/st读写类指令总线地址处理逻辑
    always @(posedge clk) begin
        if (rst) begin
            bus_addr_out <= 30'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            we <= 1'b0;
        end
        else begin
            if (stage == STALL) begin
                if (opcode == OPCODE_LOAD && rd_in != rd_1) begin
                    we <= 1'b1;
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
                end
            end
        else if (stage == EXE) begin
            bus_addr_out <= 30'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            we <= 1'b0;
            case (opcode)
            OPCODE_LOAD: begin
                we <= 1'b1;
                case (func10[2:0])
                3'b000: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                3'b001: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                3'b010: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                3'b100: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                3'b101: bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                default: bus_addr_out <= 30'd0;
                endcase
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
        end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            opcode_post <= 7'd0;
            stalled <= 1'b0;
        end
        else begin
            opcode_post <= opcode;
            if (stall) stalled <= 1'b1;
            else if (stalled && loaded) stalled <= 1'b0;
            else stalled <= stalled;
        end
    end

//stall信号拉起逻辑
    always @(*) begin
        st_addr = r1_data_final + offset_store0;
        if ((opcode_post == OPCODE_LOAD) &&
            ((rd_1 == r1_post) | (rd_1 == r2_post))) begin
            if (stalled && loaded)
                stall = 1'b0;
            else
                stall = 1'b1;
        end
        else begin
            stall = 1'b0;
        end
    end

//读数据写回处理逻辑，字节使能、访问宽度延迟处理（两级移位，loaded 取 _2）
    always @(posedge clk) begin
        if (rst) begin
            rd_1 <= 5'd0;
            rd_2 <= 5'd0;
            size_1 <= 3'd0;
            off_1 <= 2'd0;
            size_2 <= 3'd0;
            off_2 <= 2'd0;
        end
        else begin
            if (opcode == OPCODE_LOAD) begin
                rd_1 <= rd_in;
                size_1 <= func10[2:0];
                off_1 <= r1_data_final + offset_load0;
            end
            else begin
                rd_1 <= rd_1;
                size_1 <= size_1;
                off_1 <= off_1;
            end
            rd_2 <= rd_1;
            size_2 <= size_1;
            off_2 <= off_1;
        end
    end

//字节使能数据返回输出，loaded 拍 rd 与 data 同拍同源
    always @(*) begin
        loaded = ready_in;
        if (loaded) begin
            rd_load = rd_2;
            case (size_2)
            3'b000: ld_data_out = {{24{bus_data_in[8*off_2 + 7]}}, bus_data_in[8*off_2 +: 8]};
            3'b001: ld_data_out = {{16{bus_data_in[16*off_2[1] + 15]}}, bus_data_in[16*off_2[1] +: 16]};
            3'b010: ld_data_out = bus_data_in;
            3'b100: ld_data_out = {24'd0, bus_data_in[8*off_2 +: 8]};
            3'b101: ld_data_out = {16'd0, bus_data_in[16*off_2[1] +: 16]};
            default: ld_data_out = bus_data_in;
            endcase
        end
        else begin
            rd_load = 5'd0;
            ld_data_out = bus_data_in;
        end
    end
endmodule
