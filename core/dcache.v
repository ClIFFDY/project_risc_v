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

    reg [20:0] tag;
    reg [7:0] idx;
    reg [2:0] word;
    reg [1:0] hit_way;
    reg cache_hit;

    reg [20:0] tag0 [0:255];
    reg [20:0] tag1 [0:255];
    (* ram_style = "block" *) reg [31:0] data [0:4095];
    reg [255:0] valid0, valid1, lru;

    reg stage;
    reg fill_end;
    reg [2:0] miss_word;
    reg [2:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [20:0] fill_tag;
    reg [7:0] fill_idx;
    reg fill_way;

    reg rd_req, wr_req, wr_pend, wr_drive, rd_drive;
    reg wr_full_hit;
    reg [11:0] rd_addr;
    reg rd_en;
    reg is_fill_wr, ram_we;
    reg [11:0] ram_waddr;
    reg [31:0] ram_wdata;
    reg [31:0] wr_addr, wr_data;
    reg [3:0] wr_be;

    integer i;

    initial begin
        for (i = 0; i < 4096; i = i + 1) data[i] = 32'd0;
    end

    always @(*) begin
        tag = bus_addr_in[31:11];
        idx = bus_addr_in[10:3];
        word = bus_addr_in[2:0];
        hit_way[0] = valid0[idx] && (tag0[idx] == tag);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
        cache_hit = hit_way[0] || hit_way[1];

        rd_req = !rst && (stage == 1'd0) && (bus_addr_in[31:24] == 8'd0) && (bus_addr_in[23:13] == 11'h100) && !bus_we_in;
        wr_req = !rst && (stage == 1'd0) && (bus_addr_in[31:24] == 8'd0) && (bus_addr_in[23:13] == 11'h100) && bus_we_in && !wr_pend;
//整字 store 命中：能整字覆盖行内那个字，故【不失效】，直接行内更新。
//写穿照做，缓存与后端仍一致。子字 store 覆盖不了整字，维持写失效。
        wr_full_hit = wr_req && cache_hit && (bus_be_in == 4'b1111);

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
        rd_en  = fill_end || (rd_req && cache_hit);
        rd_addr = fill_end ? {fill_way, fill_idx, miss_word} : {hit_way[1], idx, word};
    end

//写口：两路整字写共用【单写口】，仍能推成 BRAM ——
//  ① 回填（stage==1）
//  ② 整字 store 命中（stage==0）
//两者天然互斥：wr_req 要求 stage==0，填充时 stage==1。
//字节粒度写会变成多个独立写操作（BRAM 只有 1 个写口），推不成，故子字 store 不做行更新。
    always @(*) begin
        is_fill_wr = (stage == 1'd1) && !fill_end && mem_valid;
        ram_we     = is_fill_wr || wr_full_hit;
        if (is_fill_wr) begin
            ram_waddr = {fill_way, fill_idx, fill_cnt};
            ram_wdata = mem_data;
        end
        else begin
            ram_waddr = {hit_way[1], idx, word};
            ram_wdata = bus_data_in;
        end
    end

    always @(posedge clk) begin
        if (ram_we) data[ram_waddr] <= ram_wdata;
    end

//每拍先清零：cpu_top 把 dcache/tim_in/dtcm 的读数据按【位或】合流成一个 bus_data_in_final，
//必须靠"非本窗口时输出 0"来互斥。若只在 rd_en 时更新，残留值会被或进别人的读数据
//（实测：读 UART 状态口拿到上一次 dtcm 读的值 → ee_printf 的 while 轮询死循环）。
    always @(posedge clk) begin
        if (rst)
            bus_data_out <= 32'd0;
        else begin
            bus_data_out <= 32'd0;
            if (rd_en)
                bus_data_out <= data[rd_addr];
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            stage <= 1'd0;
            fill_end <= 1'b0;
            fill_cnt <= 3'd0;
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
                    fill_cnt <= 3'd0;
                    fill_addr <= (bus_addr_in & 32'hFFFFFFF8) << 2;
                    fill_tag <= tag;
                    fill_idx <= idx;
                    fill_way <= (!valid0[idx]) ? 1'b0 :
                                (!valid1[idx]) ? 1'b1 : lru[idx];
                end
            end

            if (wr_req) begin
                if (wr_full_hit) begin
                    if (hit_way[0]) lru[idx] <= 1'b1;
                    else            lru[idx] <= 1'b0;
                end
                else begin
                    if (hit_way[0])      valid0[idx] <= 1'b0;
                    else if (hit_way[1]) valid1[idx] <= 1'b0;
                end
                if (!mem_ready) begin
                    wr_pend <= 1'b1;
                    wr_addr <= bus_addr_in << 2;
                    wr_data <= bus_data_in;
                    wr_be <= bus_be_in;
                end
            end

            if (wr_pend && mem_ready) wr_pend <= 1'b0;

            if (stage == 1'd1 && !fill_end && mem_valid) begin
                if (fill_cnt == 4'd7) fill_end <= 1'b1;
                else fill_cnt <= fill_cnt + 3'd1;
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
