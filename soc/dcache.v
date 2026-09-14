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

    reg [19:0] tag;
    reg [7:0] idx;
    reg [3:0] word;
    reg [1:0] hit_way;
    reg cache_hit;

    reg [19:0] tag0 [0:255];
    reg [19:0] tag1 [0:255];
    (* ram_style = "block" *) reg [31:0] data [0:8191];
    reg [255:0] valid0, valid1, lru;

    reg stage;
    reg fill_end;
    reg [3:0] miss_word;
    reg [3:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [19:0] fill_tag;
    reg [7:0] fill_idx;
    reg fill_way;

    reg rd_req, wr_req, wr_pend, wr_drive, rd_drive;
    reg [12:0] rd_addr;
    reg rd_en;
    reg [31:0] wr_addr, wr_data;
    reg [3:0] wr_be;

    integer i;

    always @(*) begin
        tag = bus_addr_in[31:12];
        idx = bus_addr_in[11:4];
        word = bus_addr_in[3:0];
        hit_way[0] = valid0[idx] && (tag0[idx] == tag);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
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

//读口：hit 与 fill_end 合成一个地址/一个使能/一个数据选择（每个 data 只有一个读口）
        rd_en  = fill_end || rd_req;
        rd_addr = fill_end ? {fill_way, fill_idx, miss_word} : {hit_way[1], idx, word};
    end

//写口：只有回填这一路整字写 —— 单写口，所以这个阵列能推成 BRAM
//字节粒度写会变成多个独立写操作（BRAM 只有 1 个写口），推不成，故不做缓存行更新
    always @(posedge clk) begin
        if (stage == 1'd1 && !fill_end && mem_valid)
            data[{fill_way, fill_idx, fill_cnt}] <= mem_data;
    end

    always @(posedge clk) begin
        if (rst)
            bus_data_out <= 32'd0;
        else if (rd_en)
            bus_data_out <= data[rd_addr];
    end

    always @(posedge clk) begin
        if (rst) begin
            stage <= 1'd0;
            fill_end <= 1'b0;
            fill_cnt <= 4'd0;
            fill_way <= 1'b0;
            wr_pend <= 1'b0;
            ld_ready <= 1'b0;
            valid0 <= 256'd0;
            valid1 <= 256'd0;
            lru <= 256'd0;
        end
        else begin
            ld_ready <= 1'b0;

            if (rd_req) begin
                if (hit_way[0]) begin
                    ld_ready <= 1'b1;
                    lru[idx] <= 1'b1;
                end
                else if (hit_way[1]) begin
                    ld_ready <= 1'b1;
                    lru[idx] <= 1'b0;
                end
                else begin
                    stage <= 1'd1;
                    miss_word <= word;
                    fill_cnt <= 4'd0;
                    fill_addr <= (bus_addr_in & 32'hFFFFFFF0) << 2;
                    fill_tag <= tag;
                    fill_idx <= idx;
                    fill_way <= (!valid0[idx]) ? 1'b0 :
                                (!valid1[idx]) ? 1'b1 : lru[idx];
                end
            end

            if (wr_req) begin
                if (hit_way[0])      valid0[idx] <= 1'b0;
                else if (hit_way[1]) valid1[idx] <= 1'b0;
                if (!mem_ready) begin
                    wr_pend <= 1'b1;
                    wr_addr <= bus_addr_in << 2;
                    wr_data <= bus_data_in;
                    wr_be <= bus_be_in;
                end
            end

            if (wr_pend && mem_ready) wr_pend <= 1'b0;

            if (stage == 1'd1 && !fill_end && mem_valid) begin
                if (fill_cnt == 4'd15) fill_end <= 1'b1;
                else fill_cnt <= fill_cnt + 4'd1;
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
                lru[fill_idx] <= ~fill_way;
                ld_ready <= 1'b1;
                stage <= 1'd0;
                fill_end <= 1'b0;
            end
        end
    end
endmodule
