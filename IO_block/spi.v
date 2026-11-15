`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: spi
// Description: 7 号槽 sub1（0x1C01_0000）的通用 SPI 主机，给用户外设用。
//
//   ★ 四根线（CS / SCK / MOSI / MISO）全部走 GPIO 复用，不新增顶层端口：
//     gpio_group 的 mode 4 = SPI_CS、13 = SPI_SCK、14 = SPI_MOSI、15 = SPI_MISO。
//     软件自己决定用哪几个引脚，硬件不规定脚号。
//
//   ★ 2026-11-13 重写：本文件原来是 flash 的 XIP 后端（icache_mem_*/dcache_mem_*
//     三条上游口 + 60 位写 FIFO + 24 位 NOR 地址），全仓零例化。现在改成
//     寄存器式用户外设，NOR 命令由软件按字节自己拼。
//
//   寄存器（槽内【字】偏移 = bus_addr_in[4:0]）：
//     0  CTRL      W: bit0 start（自清） bit1 lsb_first bit2 auto
//                  R: {28'd0, auto, lsb_first, busy, done}
//     1  STATUS    R: bit0 busy bit1 done bit2 tx_empty bit3 tx_full
//                     bit4 rx_empty bit5 rx_full bit6 rx_ovf bit7 tx_ovf
//                  W: bit1 写 1 清 done
//     2  TXDATA    W: 压 TX FIFO（满时丢弃并置 tx_ovf）  R: 回读最后一次写进去的
//     3  RXDATA    R: 弹 RX FIFO（空时返回最后一次收到的字节）
//     4  CS        W: bit0 片选（0 = 选中）  R: 回读
//     5  DIV0      W/R: 分频，SCK = clk/(2*(div+1))；复位值由参数算
//     6  DIV1      W/R: 本模块只用一个分频，同图保留，读得回、不起作用
//     7  BOOT      R: 本模块没有 boot 引擎，读回 0
//     8  FIFO_CNT  R: {22'd0, rx_count[4:0], tx_count[4:0]}
//                  W: bit0 清 TX，bit1 清 RX（并清 rx_ovf）
//     9  IRQ       W: bit0 rx_ie bit1 tx_ie [7:4] tx_low
//                  R: {23'd0, irq, tx_evt, rx_evt, tx_low[3:0], tx_ie, rx_ie}
//
//   ★ 两种发射方式：
//       auto = 0（默认）一次 start 发一个字节 —— 与老 pmem_spi 的行为完全一致
//       auto = 1        一次 start 之后只要 TX FIFO 非空就接着发，字节之间 CS 不抬
//
//   ★ 中断是【电平】不是粘滞位：PLIC 每拍从电平重置 pending。rx_evt 跟 rx_empty
//     走，tx_evt 跟 tx_count <= tx_low 走；两个使能位复位的 0，所以复位后不报。
//
//   ★ 与 pmem_spi.v 共用同一张寄存器图。改一边必须同时改另一边。
//////////////////////////////////////////////////////////////////////////////////


module spi #(
//★ 分频的复位值由"核时钟 + 目标 SCK"算出来，不写死。向上取整是故意的：
//  宁可慢一点也不许超过目标频率（SD 卡初始化对频率有上限）。
    parameter integer CLK_MHZ = 120,
    parameter integer SCK_KHZ = 1000
)(
    input clk, rst,

    input        bus_sel_in,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0]  bus_be_in,
    input        bus_we_in,
    input       spi_miso,
    output reg [31:0] bus_data_out,
    output reg        ld_ready,

//---- SPI 四根线（由 GPIO 复用到引脚上）----
    output reg  spi_cs_n,
    output reg      spi_sck,
    output reg      spi_mosi,

    output reg  spi_irq
    );

    localparam [4:0]
    R_CTRL    = 5'd0,
    R_STATUS  = 5'd1,
    R_TXDATA  = 5'd2,
    R_RXDATA  = 5'd3,
    R_CS      = 5'd4,
    R_DIV0    = 5'd5,
    R_DIV1    = 5'd6,
    R_BOOT    = 5'd7,
    R_FIFOCNT = 5'd8,
    R_IRQ     = 5'd9;

//SCK = clk/(2*(div+1)) ⇒ 半周期 = ceil(clk/(2*SCK)) 个核时钟，div = 半周期 - 1
    localparam integer HALF = (CLK_MHZ * 1000 + 2 * SCK_KHZ - 1) / (2 * SCK_KHZ);
    localparam [7:0]   DIV_INIT = (HALF <= 1) ? 8'd0 :
                                  (HALF > 256) ? 8'd255 : HALF[7:0] - 8'd1;

//FIFO：16 深。★ 指针必须留 5 位（4 位索引 + 1 位区分空满），
//  只留 4 位的话满和空同值，满队会被读成空、静默覆盖未交付的数据。
    localparam integer FIFO_AW    = 4;
    localparam integer FIFO_DEPTH = 1 << FIFO_AW;
    localparam [FIFO_AW:0] FIFO_FULL = ({{FIFO_AW{1'b0}}, 1'b1} << FIFO_AW);

    reg [7:0] tx_mem [0:FIFO_DEPTH-1];
    reg [7:0] rx_mem [0:FIFO_DEPTH-1];
    reg [FIFO_AW:0] tx_wp, tx_rp, rx_wp, rx_rp;

//总线侧寄存器
    reg        lsb_first, auto_mode, cs_level;
    reg [7:0]  div_reg, tx_byte;
    reg        rx_ie, tx_ie;
    reg [3:0]  tx_low;
    reg        tx_ovf, rx_ovf;

//收发状态（只由状态机持有）
    reg        busy, done;
    reg [2:0]  bit_count;
    reg [7:0]  tx_shift, rx_shift, rx_byte;

//SCK 发生器
    reg [7:0]  div_count, div_active;
    reg        sck_level_d;

//------------------------------------------------------------------------------
//  FIFO 状态与译码
//------------------------------------------------------------------------------
    reg [FIFO_AW:0] tx_count, rx_count;
    reg tx_empty, tx_full, rx_empty, rx_full;
    always @(*) begin
        tx_count = tx_wp - tx_rp;
        rx_count = rx_wp - rx_rp;
        tx_empty = (tx_wp == tx_rp);
        rx_empty = (rx_wp == rx_rp);
        tx_full = (tx_count == FIFO_FULL);
        rx_full = (rx_count == FIFO_FULL);
    end

    reg [7:0] tx_head;
    always @(*) begin
        tx_head = tx_mem[tx_rp[FIFO_AW-1:0]];
    end

//SCK 边沿：全部锚在 spi_sck 的寄存器沿上
    reg sck_tick, sck_rise, sck_fall;
    always @(*) begin
        sck_tick = busy & (div_count == div_active);
        sck_rise = spi_sck & ~sck_level_d;
        sck_fall = ~spi_sck & sck_level_d;
    end

//中断的两个电平（跟状态位走，不粘滞）
    reg rx_evt, tx_evt;
    always @(*) begin
        rx_evt = ~rx_empty;
        tx_evt = (tx_count <= {1'b0, tx_low});
    end

    reg sel;
    reg [4:0] reg_sel;
    always @(*) begin
        sel = bus_sel_in;
        reg_sel = bus_addr_in[4:0];
    end

//CTRL 的 start 是一拍脉冲，从写选通直接译出来给状态机用
    reg start_request, done_clear, tx_push, rx_pop, tx_clear, rx_clear;
    always @(*) begin
        start_request = sel & bus_we_in & (reg_sel == R_CTRL)    & bus_be_in[0] & bus_data_in[0];
        done_clear = sel & bus_we_in & (reg_sel == R_STATUS)  & bus_be_in[0] & bus_data_in[1];
        tx_push = sel & bus_we_in & (reg_sel == R_TXDATA)  & bus_be_in[0];
        rx_pop = sel & ~bus_we_in & (reg_sel == R_RXDATA) & ~rx_empty;
        tx_clear = sel & bus_we_in & (reg_sel == R_FIFOCNT) & bus_be_in[0] & bus_data_in[0];
        rx_clear = sel & bus_we_in & (reg_sel == R_FIFOCNT) & bus_be_in[0] & bus_data_in[1];
    end

//引擎侧的取字节/换字节动作（组合量，由状态机块与 FIFO 块共用）
    // 一次 start 起一个字节
    reg tx_load;
    always @(*) begin
        tx_load = start_request & ~busy;
    end
    reg tx_pop, byte_end;
    always @(*) begin
        tx_pop = tx_load & ~tx_empty;
        byte_end = busy & sck_fall & (bit_count == 3'd7);
    end
    // 连发：接着来
    reg byte_auto;
    always @(*) begin
        byte_auto = byte_end & auto_mode & ~tx_empty;
    end
    reg byte_pop;
    always @(*) begin
        byte_pop = tx_pop | byte_auto;
    end

    reg [7:0] tx_load_byte;
    always @(*) begin
        tx_load_byte = tx_empty ? tx_byte : tx_head;
    end

//------------------------------------------------------------------------------
//  SCK 发生器：只在 busy 时走，半周期 = div_active+1 个时钟
//------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            div_count <= 8'd0;
            spi_sck <= 1'b0;
        end
        else if (!busy) begin
            div_count <= 8'd0;
            spi_sck <= 1'b0;
        end
        else if (sck_tick) begin
            div_count <= 8'd0;
            spi_sck <= ~spi_sck;
        end
        else
            div_count <= div_count + 8'd1;
    end

    always @(posedge clk) begin
        if (rst)
            sck_level_d <= 1'b0;
        else
            sck_level_d <= spi_sck;
    end

//------------------------------------------------------------------------------
//  FIFO + 寄存器堆
//  ★ 两个 FIFO 的数组和指针全部只在这一个块里写，避免多驱动。
//    状态机块只读 tx_head / tx_empty，并通过 byte_end 告诉这里"该压 RX 了"。
//  ★ 清 FIFO 一律把两个指针一起清 0。不能写成 "rp <= wp" 和 "wp <= rp"，
//    那样两边交换、不清空。
//------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            lsb_first <= 1'b0;
            auto_mode <= 1'b0;
            cs_level <= 1'b1;
            div_reg <= DIV_INIT;
            tx_byte <= 8'd0;
            rx_ie <= 1'b0;
            tx_ie <= 1'b0;
            tx_low <= 4'd4;
            tx_ovf <= 1'b0;
            rx_ovf <= 1'b0;
            tx_wp <= {(FIFO_AW+1){1'b0}};
            tx_rp <= {(FIFO_AW+1){1'b0}};
            rx_wp <= {(FIFO_AW+1){1'b0}};
            rx_rp <= {(FIFO_AW+1){1'b0}};
            spi_irq <= 1'b0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;

        //---- TX FIFO ----
            if (tx_clear) begin
                tx_wp <= {(FIFO_AW+1){1'b0}};
                tx_rp <= {(FIFO_AW+1){1'b0}};
                tx_ovf <= 1'b0;
            end
            else begin
                if (tx_push) begin
                    if (tx_full)
                        tx_ovf <= 1'b1;
                    else begin
                        tx_mem[tx_wp[FIFO_AW-1:0]] <= bus_data_in[7:0];
                        tx_wp <= tx_wp + 1'b1;
                    end
                end
                if (byte_pop)
                    tx_rp <= tx_rp + 1'b1;
            end

        //---- RX FIFO ----
            if (rx_clear) begin
                rx_wp <= {(FIFO_AW+1){1'b0}};
                rx_rp <= {(FIFO_AW+1){1'b0}};
                rx_ovf <= 1'b0;
            end
            else begin
                if (byte_end) begin
                    if (rx_full)
                        rx_ovf <= 1'b1;
                    else begin
                        rx_mem[rx_wp[FIFO_AW-1:0]] <= rx_shift;
                        rx_wp <= rx_wp + 1'b1;
                    end
                end
                if (rx_pop)
                    rx_rp <= rx_rp + 1'b1;
            end

        //---- 寄存器读 ----
            if (sel) begin
                case (reg_sel)
                R_CTRL: begin
                    bus_data_out <= {28'd0, auto_mode, lsb_first, busy, done};
                    ld_ready <= 1'b1;
                end
                R_STATUS: begin
                    bus_data_out <= {24'd0, tx_ovf, rx_ovf, rx_full, rx_empty,
                                     tx_full, tx_empty, done, busy};
                    ld_ready <= 1'b1;
                end
                R_TXDATA: begin
                    bus_data_out <= {24'd0, tx_byte};
                    ld_ready <= 1'b1;
                end
                R_RXDATA: begin
                    bus_data_out <= {24'd0, rx_empty ? rx_byte : rx_mem[rx_rp[FIFO_AW-1:0]]};
                    ld_ready <= 1'b1;
                end
                R_CS: begin
                    bus_data_out <= {31'd0, cs_level};
                    ld_ready <= 1'b1;
                end
                R_DIV0: begin
                    bus_data_out <= {24'd0, div_reg};
                    ld_ready <= 1'b1;
                end
                R_DIV1: begin
                    bus_data_out <= {24'd0, div_reg};
                    ld_ready <= 1'b1;
                end
                R_BOOT: begin
                    bus_data_out <= 32'd0;
                    ld_ready <= 1'b1;
                end
                R_FIFOCNT: begin
                    bus_data_out <= {22'd0, rx_count, tx_count};
                    ld_ready <= 1'b1;
                end
                R_IRQ: begin
                    bus_data_out <= {23'd0, spi_irq, tx_evt, rx_evt,
                                     tx_low, tx_ie, rx_ie};
                    ld_ready <= 1'b1;
                end
                default: begin
                    ld_ready <= 1'b1;
                end
                endcase
            end

        //---- 寄存器写 ----
            if (sel & bus_we_in) begin
                case (reg_sel)
                R_CTRL: begin
                    if (bus_be_in[0]) begin
//★ 位序只在"不是 start 的那次写"里改 —— 理由见 pmem_spi.v 同一处注释：
//  start 的写法 bit1 = 0，无条件写会把 LSb-first 清掉，那个功能就从来没生效过。
                        if (!bus_data_in[0])
                            lsb_first <= bus_data_in[1];
                        auto_mode <= bus_data_in[2];
                    end
                end
                R_TXDATA: begin
                    if (bus_be_in[0])
                        tx_byte <= bus_data_in[7:0];
                end
                R_CS: begin
                    if (bus_be_in[0])
                        cs_level <= bus_data_in[0];
                end
                R_DIV0: begin
                    if (bus_be_in[0])
                        div_reg <= bus_data_in[7:0];
                end
                R_IRQ: begin
                    if (bus_be_in[0]) begin
                        rx_ie  <= bus_data_in[0];
                        tx_ie  <= bus_data_in[1];
                        tx_low <= bus_data_in[7:4];
                    end
                end
                default: ;
                endcase
            end

            spi_irq <= (rx_ie & rx_evt) | (tx_ie & tx_evt);
        end
    end

//------------------------------------------------------------------------------
//  收发状态机：一个字节 = 8 个 SCK；上升沿之后采 MISO，下降沿之后换 MOSI。
//  ★ 采样/移位都锚在 spi_sck 的【寄存器沿】上（sck_rise/sck_fall），
//    不是在分频计数那一拍 —— 否则采到的是 SCK 还没翻上去的旧电平。
//  ★ 连发时字节之间零空拍：本字节结束那一拍 tx_rp 还指着【下一字节】，
//    直接把它装进移位寄存器即可。
//------------------------------------------------------------------------------
    reg [7:0] tx_next, rx_next;
    always @(*) begin
        tx_next = lsb_first ? {1'b1, tx_shift[7:1]} : {tx_shift[6:0], 1'b1};
        rx_next = lsb_first ? {spi_miso, rx_shift[7:1]} : {rx_shift[6:0], spi_miso};
    end

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            bit_count <= 3'd0;
            tx_shift <= 8'd0;
            rx_shift <= 8'd0;
            rx_byte <= 8'd0;
            div_active <= DIV_INIT;
        end
        else begin
            if (done_clear)
                done <= 1'b0;
            if (tx_load) begin
                busy <= 1'b1;
                done <= 1'b0;
                bit_count <= 3'd0;
                tx_shift <= tx_load_byte;
                rx_shift <= 8'd0;
                div_active <= div_reg;
            end
            else if (busy) begin
                if (sck_rise)
                    rx_shift <= rx_next;
                if (sck_fall) begin
                    if (bit_count == 3'd7) begin
                        rx_byte <= rx_shift;
                        bit_count <= 3'd0;
                        if (byte_auto) begin
                            tx_shift <= tx_head;
                            busy <= 1'b1;
                        end
                        else begin
                            tx_shift <= tx_next;
                            busy <= 1'b0;
                            done <= 1'b1;
                        end
                    end
                    else begin
                        tx_shift <= tx_next;
                        bit_count <= bit_count + 3'd1;
                    end
                end
            end
        end
    end

//------------------------------------------------------------------------------
//  输出：全部从寄存器出（模块 IO 必须有寄存器界）
//------------------------------------------------------------------------------
    always @(*) begin
        spi_mosi = lsb_first ? tx_shift[0] : tx_shift[7];
    end

    always @(*) begin
        spi_cs_n = cs_level;
    end

endmodule
