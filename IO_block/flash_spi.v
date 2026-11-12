`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: flash_spi
// Description: 7 号槽（0x1C00_0000）的 SPI 主机控制器，接板上的 SPI NOR flash。
//
//   槽号判据与 tim_p 同口径：字节地址 >> 2 之后 bus_addr_in[31:24] == 7。
//
//   寄存器（槽内字偏移 = bus_addr_in[2:0]）：
//     0  CTRL    W: bit0 cs_n（0 = 选中）bit1 start（写 1 启动，自清）bit2 lsb_first
//                  R: {29'd0, lsb_first, busy, cs_n}
//     1  STATUS  R: bit0 busy  bit1 done
//                  W: bit1 写 1 清 done
//     2  CONFIG  W/R: [7:0] clk_div，SCK = clk / (2*(clk_div+1))
//     3  TXDATA  W/R: [7:0] 待发字节
//     4  RXDATA  R: [7:0] 最近一次收齐的字节
//   对应字节地址（槽基址 0x1C00_0000）：00 / 04 / 08 / 0C / 10
//
//   ★ SPI Mode 0：SCK 空闲低；下降沿之后更新 MOSI，上升沿之后采 MISO。
//   ★ 一次 start 收发【一个字节】（8 个 SCK），完成置 done、清 busy。
//     字节攒成事务（命令 + 地址 + 数据）由软件按 CTRL.cs_n 自己拼：
//       CS拉低 → 发 0x03 → 发 3 字节地址 → 连发/连收数据 → CS拉高
//     flash 命令集不硬做进硬件，换器件/换命令不用改 RTL。
//   ★ 只做数据通路，不做页编程/擦除的状态轮询：那些靠软件读 RDSR(0x05)。
//   ★ 复位后 clk_div = 9 ⇒ SCK = 50MHz / 20 = 2.5 MHz，够慢，先保证能通。
//////////////////////////////////////////////////////////////////////////////////


module flash_spi(
    input clk, rst,

    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    output reg [31:0] bus_data_out,
    output reg        ld_ready,

    output reg spi_cs_n,
    output reg spi_sck,
    output reg spi_mosi,
    input      spi_miso
    );

    localparam [2:0]
    R_CTRL   = 3'd0,
    R_STATUS = 3'd1,
    R_CONFIG = 3'd2,
    R_TXDATA = 3'd3,
    R_RXDATA = 3'd4;

    wire sel = (bus_addr_in[31:24] == 8'd7);

//总线侧寄存器（只由总线块持有）
    reg        cs_level, lsb_first;
    reg [7:0]  clk_div, tx_byte;

//收发状态（只由状态机持有）
    reg        busy, done;
    reg [2:0]  bit_count;
    reg [7:0]  tx_shift, rx_shift, rx_byte;

//SCK 发生器
    reg [7:0]  div_count;
    reg        sck_level, sck_level_d;

    reg        start_request, done_clear;
    reg [7:0]  tx_next, rx_next;

    wire sck_tick = busy & (div_count == clk_div);
    wire sck_rise = sck_level & ~sck_level_d;
    wire sck_fall = ~sck_level & sck_level_d;

//CTRL 的写动作：start 是一拍脉冲，从写选通直接译出来给状态机用
    always @(*) begin
        start_request = sel & bus_we_in & (bus_addr_in[2:0] == R_CTRL) & bus_be_in[0] & bus_data_in[1];
        done_clear    = sel & bus_we_in & (bus_addr_in[2:0] == R_STATUS) & bus_be_in[0] & bus_data_in[1];
    end

//------------------------------------------------------------------------------
//  总线侧：CS 电平、位序、分频、待发字节。状态块只读它们，避免多驱动。
//------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            cs_level <= 1'b1;
            lsb_first <= 1'b0;
            clk_div <= 8'd9;
            tx_byte <= 8'd0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            if (sel) begin
                case (bus_addr_in[2:0])
                R_CTRL: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0]) begin
                            cs_level <= bus_data_in[0];
                            lsb_first <= bus_data_in[2];
                        end
                        else begin
                            cs_level <= cs_level;
                            lsb_first <= lsb_first;
                        end
                    end
                    else begin
                        bus_data_out <= {29'd0, lsb_first, busy, cs_level};
                        ld_ready <= 1'b1;
                    end
                end
                R_STATUS: begin
                    if (!bus_we_in) begin
                        bus_data_out <= {30'd0, done, busy};
                        ld_ready <= 1'b1;
                    end
                end
                R_CONFIG: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0])
                            clk_div <= bus_data_in[7:0];
                        else
                            clk_div <= clk_div;
                    end
                    else begin
                        bus_data_out <= {24'd0, clk_div};
                        ld_ready <= 1'b1;
                    end
                end
                R_TXDATA: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0])
                            tx_byte <= bus_data_in[7:0];
                        else
                            tx_byte <= tx_byte;
                    end
                    else begin
                        bus_data_out <= {24'd0, tx_byte};
                        ld_ready <= 1'b1;
                    end
                end
                R_RXDATA: begin
                    if (!bus_we_in) begin
                        bus_data_out <= {24'd0, rx_byte};
                        ld_ready <= 1'b1;
                    end
                end
                default: begin
                    if (!bus_we_in) begin
                        bus_data_out <= 32'd0;
                        ld_ready <= 1'b1;
                    end
                end
                endcase
            end
        end
    end

//------------------------------------------------------------------------------
//  SCK 发生器：只在 busy 时走，半周期 = clk_div+1 个时钟
//------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            div_count <= 8'd0;
            sck_level <= 1'b0;
        end
        else if (!busy) begin
            div_count <= 8'd0;
            sck_level <= 1'b0;
        end
        else if (sck_tick) begin
            div_count <= 8'd0;
            sck_level <= ~sck_level;
        end
        else
            div_count <= div_count + 8'd1;
    end

    always @(posedge clk) begin
        if (rst)
            sck_level_d <= 1'b0;
        else
            sck_level_d <= sck_level;
    end

//------------------------------------------------------------------------------
//  收发状态机：一个字节 = 8 个 SCK；上升沿之后采 MISO，下降沿之后换 MOSI。
//  ★ 采样/移位都锚在 sck_level 的【寄存器沿】上（sck_rise/sck_fall），
//    不是在分频计数那一拍 —— 否则采到的是 SCK 还没翻上去的旧电平。
//------------------------------------------------------------------------------
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
        end
        else begin
            if (done_clear)
                done <= 1'b0;
            if (start_request & ~busy) begin
                busy <= 1'b1;
                done <= 1'b0;
                bit_count <= 3'd0;
                tx_shift <= tx_byte;
                rx_shift <= 8'd0;
            end
            else if (busy) begin
                if (sck_rise)
                    rx_shift <= rx_next;
                if (sck_fall) begin
                    tx_shift <= tx_next;
                    if (bit_count == 3'd7) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        rx_byte <= rx_shift;
                        bit_count <= 3'd0;
                    end
                    else
                        bit_count <= bit_count + 3'd1;
                end
            end
        end
    end

//------------------------------------------------------------------------------
//  输出：三根线全部从寄存器出（模块 IO 必须有寄存器界）
//------------------------------------------------------------------------------
    always @(*) begin
        spi_cs_n = cs_level;
        spi_sck = sck_level;
        spi_mosi = lsb_first ? tx_shift[0] : tx_shift[7];
    end

endmodule
