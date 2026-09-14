`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/31 17:32:45
// Design Name:
// Module Name: icache
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


module icache(
    input clk, rst,
    input [31:0] ibus_addr_in,
    input ibus_re_in,
    input req_valid,
    output reg [31:0] ibus_data_out,
    output reg cache_miss,
    output reg busy,

    output reg mem_req, mem_we,
    output reg [31:0] mem_addr, mem_wdata,
    output reg [3:0] mem_be,
    input mem_ready,
    input mem_valid,
    input [31:0] mem_data
    );

    reg cache_hit;
    reg [19:0] tag;
    reg [6:0] idx;
    reg [4:0] word;
    reg [1:0] hit_way;
    reg fill_way;

    reg [19:0] tag0 [0:127];
    reg [19:0] tag1 [0:127];
    (* ram_style = "block" *) reg [31:0] iram [0:8191];
    reg [127:0] valid0, valid1, lru;

    reg stage;
    reg fill_end;
    reg [4:0] miss_word;
    reg [4:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [19:0] fill_tag;
    reg [6:0] fill_idx;

    reg [12:0] rd_addr;
    reg rd_en, line_match;

    integer i;

    always @(*) begin
        tag = ibus_addr_in[31:12];
        idx = ibus_addr_in[11:5];
        word = ibus_addr_in[4:0];
        hit_way[0] = valid0[idx] && (tag0[idx] == tag);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
        cache_hit = hit_way[0] || hit_way[1];
        cache_miss = (stage == 1'd0) && busy;
        busy = (stage == 1'd1 && (!fill_end || (!cache_hit && ibus_re_in &&
                                             !(fill_idx == idx && fill_tag == tag)))) ||
               (stage == 1'd0 && !cache_hit && ibus_re_in);
        mem_req = (stage == 1'd1) && !fill_end;
        mem_addr = fill_addr + (fill_cnt << 2);
        mem_we = 1'b0;
        mem_wdata = 32'd0;
        mem_be = 4'd0;

//读口：把 hit 与 fill_end 两条路合成一个地址/一个使能/一个数据选择，
//这样每个 iram 只有一个读口，才推得出 BRAM
        line_match = (ibus_addr_in & 32'hFFFFFFE0) == (fill_addr >> 2);
        rd_en  = (fill_end && line_match) || req_valid;
        rd_addr = (fill_end && line_match) ? {fill_way, fill_idx, miss_word}
                                           : {hit_way[1], idx, word};
    end

//回填：一拍收一个字直接写进 iram（BRAM 单写口，一拍只写一个地址）
    always @(posedge clk) begin
        if (stage == 1'd1 && !fill_end && mem_valid) begin
            iram[{fill_way, fill_idx, fill_cnt}] <= mem_data;
        end
    end

    always @(posedge clk) begin
        if (rst)
            ibus_data_out <= 32'd0;
        else if (rd_en)
            ibus_data_out <= iram[rd_addr];
    end

    always @(posedge clk) begin
        if (rst) begin
            stage <= 1'd0;
            fill_end <= 1'b0;
            fill_cnt <= 5'd0;
            fill_way <= 1'b0;
            valid0 <= 128'd0;
            valid1 <= 128'd0;
            lru <= 128'd0;
        end
        else begin
            if (stage == 1'd0) begin
                fill_end <= 1'b0;
                if (cache_hit)
                    lru[idx] <= ~hit_way[1];
                else if (ibus_re_in) begin
                    stage <= 1'd1;
                    miss_word <= word;
                    fill_cnt <= 5'd0;
                    fill_addr <= (ibus_addr_in & 32'hFFFFFFE0) << 2;
                    fill_tag <= tag;
                    fill_idx <= idx;
                    fill_way <= lru[idx];
                end
            end
            else begin
                if (mem_valid && !fill_end) begin
                    if (fill_cnt == 5'd31)
                        fill_end <= 1'b1;
                    else
                        fill_cnt <= fill_cnt + 1;
                end
                if (fill_end) begin
                    if (fill_way) begin
                        tag1[fill_idx] <= fill_tag;
                        valid1[fill_idx] <= 1'b1;
                    end
                    else begin
                        tag0[fill_idx] <= fill_tag;
                        valid0[fill_idx] <= 1'b1;
                    end
                    lru[fill_idx] <= ~lru[fill_idx];
                    if (!cache_hit && ibus_re_in && !(fill_idx == idx && fill_tag == tag)) begin
                        stage <= 1'd1;
                        miss_word <= word;
                        fill_cnt <= 5'd0;
                        fill_addr <= (ibus_addr_in & 32'hFFFFFFE0) << 2;
                        fill_tag <= tag;
                        fill_idx <= idx;
                        fill_way <= lru[idx];
                        fill_end <= 1'b0;
                    end
                    else begin
                        stage <= 1'd0;
                        fill_end <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
