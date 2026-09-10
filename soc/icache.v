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

    output reg mem_req,
    output reg [31:0] mem_addr,
    input mem_ready,
    input mem_valid,
    input [31:0] mem_data
    );

    reg cache_hit;
    reg [20:0] tag;
    reg [6:0] idx;
    reg [3:0] word;
    reg [1:0] hit_way;
    reg fill_way;

    (* ram_style = "block" *) reg [20:0] tag_ram [0:1][0:127];
    (* ram_style = "block" *) reg [31:0] iram [0:1][0:127][0:15];
    reg valid [0:1][0:127];
    reg lru [0:127];

    reg stage;
    reg fill_end;
    reg [3:0] miss_word;
    reg [3:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [20:0] fill_tag;
    reg [6:0] fill_idx;
    reg [31:0] fill_buf [0:15];

    integer i;
    reg [31:0] init_prog [0:15];
    integer p;

    initial begin
        $readmemh("e:/Vivado_Projects/project_risc_v/tools/hex/icache_prog.hex", init_prog);
        for (p = 0; p < 16; p = p + 1)
            iram[0][0][p] = init_prog[p];
        #1000;
        tag_ram[0][0] = 21'd2;
        valid[0][0] = 1'b1;
    end

    always @(*) begin
        tag = ibus_addr_in[31:11];
        idx = ibus_addr_in[10:4];
        word = ibus_addr_in[3:0];
        hit_way[0] = valid[0][idx] && (tag_ram[0][idx] == tag);
        hit_way[1] = valid[1][idx] && (tag_ram[1][idx] == tag); 
        cache_hit = hit_way[0] || hit_way[1];
        cache_miss = (stage == 1'd0) && busy;
        busy = (stage == 1'd1 && !fill_end) || (stage == 1'd0 && !cache_hit && ibus_re_in);
        mem_req = (stage == 1'd1) && !fill_end;
        mem_addr = fill_addr + (fill_cnt << 2);
    end

    always @(posedge clk) begin
        if (rst) 
            ibus_data_out <= 32'd0;
        else if (fill_end && ((ibus_addr_in & 32'hFFFFFFF0) == (fill_addr >> 2)))
            ibus_data_out <= fill_buf[miss_word];
        else if (req_valid) begin
            if (hit_way[0])
                ibus_data_out <= iram[0][idx][word]; 
            else if (hit_way[1])
                ibus_data_out <= iram[1][idx][word];
            else
                ibus_data_out <= 32'd0;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            stage <= 1'd0;
            fill_end <= 1'b0;
            fill_cnt <= 4'd0;
            for (i = 0; i < 128; i = i + 1) begin
                valid[0][i] <= 1'b0;
                valid[1][i] <= 1'b0;
                tag_ram[0][i] <= 21'd0;
                tag_ram[1][i] <= 21'd0;
                lru[i] <= 1'b0;
            end
        end
        else begin
            if (stage == 1'd0) begin
                fill_end <= 1'b0;
                if (cache_hit)
                    lru[idx] <= ~hit_way[1];
                else if (ibus_re_in) begin
                    stage <= 1'd1;
                    miss_word <= word;
                    fill_cnt <= 4'd0;
                    fill_addr <= (ibus_addr_in & 32'hFFFFFFF0) << 2;
                    fill_tag <= tag;
                    fill_idx <= idx;
                    fill_way <= lru[idx];
                end
            end
            else begin
                if (mem_valid && !fill_end) begin
                    fill_buf[fill_cnt] <= mem_data;
                    if (fill_cnt == 4'd15)
                        fill_end <= 1'b1;
                    else
                        fill_cnt <= fill_cnt + 1;
                end
                if (fill_end) begin
                    tag_ram[fill_way][fill_idx] <= fill_tag;
                    valid[fill_way][fill_idx] <= 1'b1;
                    for (i = 0; i < 16; i = i + 1)
                        iram[fill_way][fill_idx][i] <= fill_buf[i];
                    lru[fill_idx] <= ~lru[fill_idx];
                    if (!cache_hit && ibus_re_in && !(fill_idx == idx && fill_tag == tag)) begin
                        stage <= 1'd1;
                        miss_word <= word;
                        fill_cnt <= 4'd0;
                        fill_addr <= (ibus_addr_in & 32'hFFFFFFF0) << 2;
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
