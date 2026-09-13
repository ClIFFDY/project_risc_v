`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/12 20:19:40
// Design Name:
// Module Name: dcache
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


module dcache(
    input clk, rst,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    output reg [31:0] bus_data_out,
    output reg ld_ready,
    output reg busy,
    output reg mem_req, mem_we,
    output reg [31:0] mem_addr, mem_wdata,
    output reg [3:0] mem_be,
    input [31:0] mem_data,
    input mem_valid,
    input mem_ready
    );

    reg [22:0] tag;
    reg [6:0] idx;
    reg [1:0] word;
    reg [1:0] hit_way;
    reg cache_hit;

    reg [22:0] tag_ram [0:1][0:127];
    (* ram_style = "block" *) reg [31:0] data_ram [0:1][0:127][0:3];
    reg valid [0:1][0:127];
    reg lru [0:127];

    reg stage;
    reg fill_end;
    reg [1:0] miss_word;
    reg [1:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [22:0] fill_tag;
    reg [6:0] fill_idx;
    reg [1:0] fill_way;
    reg [31:0] fill_buf [0:3];

    reg rd_req, wr_req, wr_pend, wr_drive, rd_drive;
    reg [31:0] wr_addr, wr_data;
    reg [3:0] wr_be;

    integer i;

    always @(*) begin
        tag = bus_addr_in[31:9];
        idx = bus_addr_in[8:2];
        word = bus_addr_in[1:0];
        hit_way[0] = valid[0][idx] && (tag_ram[0][idx] == tag);
        hit_way[1] = valid[1][idx] && (tag_ram[1][idx] == tag);
        cache_hit = hit_way[0] || hit_way[1];

        rd_req = !rst && (stage == 1'd0) && (bus_addr_in[31:24] == 8'd6) && !bus_we_in;
        wr_req = !rst && (stage == 1'd0) && (bus_addr_in[31:24] == 8'd6) && bus_we_in && !wr_pend;

        wr_drive = wr_pend || wr_req;
        rd_drive = (stage == 1'd1) && !fill_end;

        busy = !rst && ((stage == 1'd1 && !fill_end) || (rd_req && !cache_hit) ||
                        (wr_pend && !mem_ready));
        mem_req = !rst && (wr_drive || rd_drive);
        mem_we = wr_drive;
        if (wr_pend) begin
            mem_addr = wr_addr;
            mem_wdata = wr_data;
            mem_be = wr_be;
        end
        else begin
            if (rd_drive) mem_addr = fill_addr + (fill_cnt << 2);
            else mem_addr = bus_addr_in << 2;
            mem_wdata = bus_data_in;
            mem_be = bus_be_in;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            stage <= 1'd0;
            fill_end <= 1'b0;
            fill_cnt <= 2'd0;
            wr_pend <= 1'b0;
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            for (i = 0; i < 128; i = i + 1) begin
                valid[0][i] <= 1'b0;
                valid[1][i] <= 1'b0;
                lru[i] <= 1'b0;
            end
        end
        else begin
            ld_ready <= 1'b0;
            bus_data_out <= 32'd0;

            if (rd_req) begin
                if (hit_way[0]) begin
                    bus_data_out <= data_ram[0][idx][word];
                    ld_ready <= 1'b1;
                    lru[idx] <= 1'b1;
                end
                else if (hit_way[1]) begin
                    bus_data_out <= data_ram[1][idx][word];
                    ld_ready <= 1'b1;
                    lru[idx] <= 1'b0;
                end
                else begin
                    stage <= 1'd1;
                    miss_word <= word;
                    fill_cnt <= 2'd0;
                    fill_addr <= (bus_addr_in & 32'hFFFF_FFFC) << 2;
                    fill_tag <= tag;
                    fill_idx <= idx;
                    fill_way <= (!valid[0][idx]) ? 1'b0 :
                                (!valid[1][idx]) ? 1'b1 : lru[idx];
                end
            end

            if (wr_req) begin
                if (hit_way[0]) begin
                    if (bus_be_in[0]) data_ram[0][idx][word][7:0]   <= bus_data_in[7:0];
                    if (bus_be_in[1]) data_ram[0][idx][word][15:8]  <= bus_data_in[15:8];
                    if (bus_be_in[2]) data_ram[0][idx][word][23:16] <= bus_data_in[23:16];
                    if (bus_be_in[3]) data_ram[0][idx][word][31:24] <= bus_data_in[31:24];
                    lru[idx] <= 1'b1;
                end
                else if (hit_way[1]) begin
                    if (bus_be_in[0]) data_ram[1][idx][word][7:0]   <= bus_data_in[7:0];
                    if (bus_be_in[1]) data_ram[1][idx][word][15:8]  <= bus_data_in[15:8];
                    if (bus_be_in[2]) data_ram[1][idx][word][23:16] <= bus_data_in[23:16];
                    if (bus_be_in[3]) data_ram[1][idx][word][31:24] <= bus_data_in[31:24];
                    lru[idx] <= 1'b0;
                end
                if (!mem_ready) begin
                    wr_pend <= 1'b1;
                    wr_addr <= bus_addr_in << 2;
                    wr_data <= bus_data_in;
                    wr_be <= bus_be_in;
                end
            end

            if (wr_pend && mem_ready) wr_pend <= 1'b0;

            if (stage == 1'd1 && !fill_end) begin
                if (mem_valid) begin
                    fill_buf[fill_cnt] <= mem_data;
                    if (fill_cnt == 2'd3) fill_end <= 1'b1;
                    else fill_cnt <= fill_cnt + 2'd1;
                end
            end

            if (fill_end) begin
                tag_ram[fill_way][fill_idx] <= fill_tag;
                valid[fill_way][fill_idx] <= 1'b1;
                for (i = 0; i < 4; i = i + 1)
                    data_ram[fill_way][fill_idx][i] <= fill_buf[i];
                lru[fill_idx] <= ~fill_way;
                bus_data_out <= fill_buf[miss_word];
                ld_ready <= 1'b1;
                stage <= 1'd0;
                fill_end <= 1'b0;
            end
        end
    end
endmodule
