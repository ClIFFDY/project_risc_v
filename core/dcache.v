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
//总线接口
    input [31:0] bus_addr_in, bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
//后端存储接口
    input [31:0] mem_data,
    input mem_valid, mem_ready,

//总线接口
    output reg [31:0] bus_data_out,
    output reg ld_ready, busy, hold,
//后端存储接口
    output reg mem_req, mem_we,
    output reg [31:0] mem_addr, mem_wdata,
    output reg [3:0] mem_be
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    reg [20:0] tag;
    reg [7:0] idx;
    reg [2:0] word;
    reg [1:0] hit_way;
    reg cache_hit;

    reg [20:0] tag0 [0:255];
    reg [20:0] tag1 [0:255];
    (* ram_style = "block" *) reg [31:0] data [0:4095];
    reg [255:0] valid0, valid1;
//lru 必须是阵列，不能是宽向量：`reg [255:0] lru` 用运行时下标读写，会被综合成 256 个散
//触发器 + 一棵 256:1 mux 树（实测 256 FDRE + 353 LUT6 + 42 MUXF7/F8，还带一条 fo=260 的
//译码网），而阵列才推得成 LUTRAM。它和 valid0/valid1 的区别在于【复位后初值任意都安全】：
//只有两路都满时才参考它，选哪一路做牺牲品功能上都对。valid0/valid1 则必须留复位——
//tag 阵列本就无复位，全靠 valid 清零兜住脏 tag，去掉复位后 rst 再拉起会假命中读到陈旧数据。
    (* ram_style = "distributed" *) reg lru [0:255];

    reg stage;
    reg fill_end;
    reg [2:0] miss_word;
    reg [2:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [20:0] fill_tag;
    reg [7:0] fill_idx;
    reg fill_way;

    reg rd_req, wr_req, wr_pend, wr_drive, rd_drive;
    reg wr_upd, hit_upd;
    reg lru_we, lru_wval;
    reg [7:0] lru_widx;
    reg [11:0] rd_addr;
    reg rd_en;
    reg is_fill_wr, ram_we;
    reg [3:0] ram_be;
    reg [11:0] ram_waddr;
    reg [31:0] ram_wdata;
    reg [31:0] wr_addr, wr_data;
    reg [3:0] wr_be;
    reg busy_d1;

    integer i;

    initial begin
        for (i = 0; i < 4096; i = i + 1) data[i] = 32'd0;
        for (i = 0; i < 256;  i = i + 1) lru[i]  = 1'b0;
    end

    always @(*) begin
        tag = bus_addr_in[31:11];
        idx = bus_addr_in[10:3];
        word = bus_addr_in[2:0];
        hit_way[0] = valid0[idx] && (tag0[idx] == tag);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
        cache_hit = hit_way[0] || hit_way[1];

        rd_req = !rst_q && (stage == 1'b0) && (bus_addr_in[31:24] == 8'd0) && (bus_addr_in[23:13] == 11'h100) && !bus_we_in;
        wr_req = !rst_q && (stage == 1'b0) && (bus_addr_in[31:24] == 8'd0) && (bus_addr_in[23:13] == 11'h100) && bus_we_in && !wr_pend;
//store 命中即行内就地更新，一律【不失效】：整字写整字，子字写按 be 只改那几个字节。
//写穿照做，缓存与后端始终一致。原先子字 store 走失效——失效会连带丢掉同行其余 7 个字，
//下一个 lw/lb 立刻回填（实测 9983 次失效 / 10104 次缺失，1:1），这是缺失的全部来源。
        wr_upd = wr_req && cache_hit;
//lru 收成【单写口】：命中改 idx（stage==0），回填改 fill_idx（fill_end 拍 stage 仍为 1）。
//rd_req / wr_req / fill_end 三者两两互斥，所以一个写地址就能覆盖原来散在三处的写，
//索引与值逐字保持原语义（命中时 way0 优先 ⇒ 值取 hit_way[0]）。
        hit_upd  = (rd_req || wr_req) && cache_hit;
        lru_we   = hit_upd || fill_end;
        lru_widx = fill_end ? fill_idx : idx;
        lru_wval = fill_end ? ~fill_way : hit_way[0];

        wr_drive = wr_pend || wr_req;
        rd_drive = (stage == 1'b1) && !fill_end;

        busy = !rst_q && ((stage == 1'b1 && !fill_end) || (rd_req && !cache_hit) ||
                        (wr_pend && !mem_ready));
        mem_req = !rst_q && (wr_drive || rd_drive);
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

//写口：单写口 + 字节使能，三个写源合成【一个】写表达式，仍能推成 BRAM ——
//  ① 回填（stage==1，整字，be 恒 1111）
//  ② store 命中（stage==0，整字或子字，be 来自 lsu）
//互斥性：wr_req 要求 stage==0，is_fill_wr 要求 stage==1；rd_en 要求 fill_end 或
//  (rd_req && hit)，而 rd_req 要求 !bus_we_in —— 三者两两不同拍，不会同拍读写同址。
//整字写与字节写【必须在同一个 if 里按 be 展开成 WEBA[3:0]】：这才是单写口；
//拆成两个写表达式才会变成"多个写操作"、才推不成 BRAM。
    always @(*) begin
        is_fill_wr = (stage == 1'b1) && !fill_end && mem_valid;
        ram_we     = is_fill_wr || wr_upd;
        ram_be     = is_fill_wr ? 4'b1111 : bus_be_in;
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
        if (ram_we) begin
            if (ram_be[0]) data[ram_waddr][7:0]   <= ram_wdata[7:0];
            if (ram_be[1]) data[ram_waddr][15:8]  <= ram_wdata[15:8];
            if (ram_be[2]) data[ram_waddr][23:16] <= ram_wdata[23:16];
            if (ram_be[3]) data[ram_waddr][31:24] <= ram_wdata[31:24];
        end
    end

//每拍先清零：cpu_top 把 dcache/tim_in/dtcm 的读数据按【位或】合流成一个 bus_data_in_final，
//必须靠"非本窗口时输出 0"来互斥。若只在 rd_en 时更新，残留值会被或进别人的读数据
//（实测：读 UART 状态口拿到上一次 dtcm 读的值 → ee_printf 的 while 轮询死循环）。
    always @(posedge clk) begin
        if (rst_q)
            bus_data_out <= 32'd0;
        else begin
            bus_data_out <= 32'd0;
            if (rd_en)
                bus_data_out <= data[rd_addr];
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            stage <= 1'b0;
            fill_end <= 1'b0;
            fill_cnt <= 3'd0;
            fill_way <= 1'b0;
            wr_pend <= 1'b0;
            ld_ready <= 1'b0;
            valid0 <= 256'd0;
            valid1 <= 256'd0;
        end
        else begin
            ld_ready <= 1'b0;

            if (lru_we) lru[lru_widx] <= lru_wval;

            if (rd_req) begin
                if (hit_way[0]) begin
                    ld_ready <= 1'b1;
                end
                else if (hit_way[1]) begin
                    ld_ready <= 1'b1;
                end
                else begin
                    stage <= 1'b1;
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
                if (!mem_ready) begin
                    wr_pend <= 1'b1;
                    wr_addr <= bus_addr_in << 2;
                    wr_data <= bus_data_in;
                    wr_be <= bus_be_in;
                end
            end

            if (wr_pend && mem_ready) wr_pend <= 1'b0;

            if (stage == 1'b1 && !fill_end && mem_valid) begin
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
                ld_ready <= 1'b1;
                stage <= 1'b0;
                fill_end <= 1'b0;
            end
        end
    end

//busy 的 1 拍延拓（原 cpu_top 的 d_hold_int，按"顶层不运算"下放到此）：
//fill_end 拍 busy 就掉、ld_ready 要再等一拍，这 1 拍空窗必须兜住。
    always @(posedge clk) begin
        if (rst_q) busy_d1 <= 1'b0;
        else     busy_d1 <= busy;
    end

    always @(*) begin
        hold = busy | busy_d1;
    end
endmodule
