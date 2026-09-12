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
    output reg [31:0] ld_data_out,
    output reg loaded,
    output reg ld_we,
    output reg stall,
    output reg [4:0] rd_load
    );

    reg [4:0] rd_1;
    reg [6:0] opcode_post;
    reg stalled;

    reg [31:0] st_addr;

    reg [4:0] ld_rd_fifo [0:1];
    reg [2:0] ld_size_fifo [0:1];
    reg [1:0] ld_off_fifo [0:1];
    reg [1:0] ld_wr_ptr, ld_rd_ptr;
    reg [4:0] ld_hold_rd;
    reg [31:0] ld_hold_data;
    reg [31:0] ld_data_cur;
    reg ld_hold;

    reg ld_fifo_empty, ld_fifo_full, ld_pop, ld_enq, ld_push_eff;
    reg [4:0] ld_rd_cur;
    reg [2:0] ld_size_cur;
    reg [1:0] ld_off_cur;

    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STORE = 7'b0100011;

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

//在途 load 队列的空满、收发条件与队头载荷
    always @(*) begin
        ld_fifo_empty = (ld_wr_ptr == ld_rd_ptr);
        ld_fifo_full  = (ld_wr_ptr[1] != ld_rd_ptr[1]) && (ld_wr_ptr[0] == ld_rd_ptr[0]);
        ld_pop        = ready_in && !ld_fifo_empty;
        ld_enq        = (opcode == OPCODE_LOAD) && !stall && (stage != FLUSH);
        ld_push_eff   = ld_enq && !(ld_fifo_full && !ld_pop);
        ld_rd_cur     = ld_rd_fifo[ld_rd_ptr[0]];
        ld_size_cur   = ld_size_fifo[ld_rd_ptr[0]];
        ld_off_cur    = ld_off_fifo[ld_rd_ptr[0]];
    end

//ld/st读写类指令总线地址处理逻辑
    always @(posedge clk) begin
        if (rst) begin
            bus_addr_out <= 30'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
        end
        else begin
            if (stage == STALL) begin
                if (ld_push_eff) begin
                    bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                    bus_we_out <= 1'b0;
                    bus_be_out <= 4'd0;
                    bus_data_out <= 32'd0;
                end
//读地址只摆一拍：外设应答是寄存的，下一拍照常到达，重复摆地址只会让它连答
                else begin
                    bus_addr_out <= 30'd0;
                    bus_we_out <= 1'b0;
                    bus_be_out <= 4'd0;
                    bus_data_out <= 32'd0;
                end
            end
        else if (stage == EXE) begin
            bus_addr_out <= 30'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            case (opcode)
            OPCODE_LOAD: begin
                case (func10[2:0])
                3'b000: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 30'd0;
                3'b001: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 30'd0;
                3'b010: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 30'd0;
                3'b100: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 30'd0;
                3'b101: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 30'd0;
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

//在途 load 请求队列：load 发射入队，应答出队，rd/size/off 与应答同源出队
    always @(posedge clk) begin
        if (rst) begin
            ld_wr_ptr <= 2'd0;
            ld_rd_ptr <= 2'd0;
        end
        else begin
            if (ld_push_eff) begin
                ld_rd_fifo[ld_wr_ptr[0]]   <= rd_in;
                ld_size_fifo[ld_wr_ptr[0]] <= func10[2:0];
                ld_off_fifo[ld_wr_ptr[0]]  <= r1_data_final + offset_load0;
                ld_wr_ptr <= ld_wr_ptr + 2'd1;
            end
            if (ld_pop)
                ld_rd_ptr <= ld_rd_ptr + 2'd1;
        end
    end

//load-use 冒险判定用最近一次发射的 load 目的寄存器
    always @(posedge clk) begin
        if (rst) rd_1 <= 5'd0;
        else if (ld_push_eff) rd_1 <= rd_in;
        else rd_1 <= rd_1;
    end

//字节使能数据返回输出：出队拍按 off/size 取字节
    always @(*) begin
        case (ld_size_cur)
        3'b000: ld_data_cur = {{24{bus_data_in[8*ld_off_cur + 7]}}, bus_data_in[8*ld_off_cur +: 8]};
        3'b001: ld_data_cur = {{16{bus_data_in[16*ld_off_cur[1] + 15]}}, bus_data_in[16*ld_off_cur[1] +: 16]};
        3'b010: ld_data_cur = bus_data_in;
        3'b100: ld_data_cur = {24'd0, bus_data_in[8*ld_off_cur +: 8]};
        3'b101: ld_data_cur = {16'd0, bus_data_in[16*ld_off_cur[1] +: 16]};
        default: ld_data_cur = bus_data_in;
        endcase
    end

//应答只摆一拍的话，落在 back2 槽的消费者会取不到，故再保持一拍
    always @(posedge clk) begin
        if (rst) ld_hold <= 1'b0;
        else begin
            ld_hold <= ld_pop;
            if (ld_pop) begin
                ld_hold_rd <= ld_rd_cur;
                ld_hold_data <= ld_data_cur;
            end
        end
    end

//rd 与 data 同拍同源输出；ld_we 只在应答拍置起（避免保持拍重复写寄存器堆）
    always @(*) begin
        ld_we = ld_pop;
        if (ld_pop) begin
            loaded = 1'b1;
            rd_load = ld_rd_cur;
            ld_data_out = ld_data_cur;
        end
        else if (ld_hold) begin
            loaded = 1'b1;
            rd_load = ld_hold_rd;
            ld_data_out = ld_hold_data;
        end
        else begin
            loaded = 1'b0;
            rd_load = 5'd0;
            ld_data_out = bus_data_in;
        end
    end
endmodule
