`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: pmem_spi
// Description: 7 号槽（0x1C00_0000）的双通道 SPI 主机：一路给板上的 flash，一路给 SD 卡。
//
//   ★ 两个逻辑通道【共享 SCK/MOSI/MISO】，只有 CS 是各自一根。
//     所以只有一个收发引擎；"两个控制器"在软件眼里就是两根 CS + 两组分频。
//     两路同时拉低 CS 会一起收到 SCK，这是软件的事，硬件不拦（写进文档就够了）。
//
//   寄存器（槽内【字】偏移 = bus_addr_in[2:0]；外设拿到的是字节地址>>2）：
//     0  CTRL    W: bit0 start（自清） bit1 lsb_first
//                  R: {lsb_first, busy, done}
//     1  STATUS  R: bit0 busy  bit1 done
//                  W: bit1 写 1 清 done
//     2  TXDATA  W/R: 待发字节
//     3  RXDATA  R: 最近一次收齐的字节
//     4  CS      W: bit0 = flash 的 CS（0 = 选中） bit1 = SD 的 CS（0 = 选中）
//                  R: 回读
//     5  DIV0    W/R: flash 通道分频，SCK = clk/(2*(div+1))，复位值 9 ⇒ 2.5 MHz
//     6  DIV1    W/R: SD 通道分频，复位值 99 ⇒ 250 kHz（SD 初始化要求 100~400 kHz）
//     7  BOOT    R: {13'd0, boot_count[16:0], boot_err, boot_done}
//   对应字节地址（槽基址 0x1C00_0000）：00 / 04 / 08 / 0C / 10 / 14 / 18 / 1C
//
//   ★ SPI Mode 0：SCK 空闲低；下降沿之后更新 MOSI，上升沿之后采 MISO。
//   ★ 一次 start 收发【一个字节】（8 个 SCK），完成置 done、清 busy。
//     字节攒成事务（命令 + 地址 + 数据）由软件按 CS 自己拼。
//   ★ 分频在 start 那一拍【采样并锁存】：本拍哪一路的 CS 是低的就用哪一路的分频
//     （两路都低或都不低时用 flash 那组）。所以收发中途改 DIV 不影响进行中的这一字节。
//   ★ BOOT 那三个状态由 soc/boot_loader.v 驱动，本模块只负责摆出来给软件读。
//////////////////////////////////////////////////////////////////////////////////


module pmem_spi(
    input clk, rst,

    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    output reg [31:0] bus_data_out,
    output reg        ld_ready,

//---- 两路 CS + 共享的三根线 ----
    output reg spi_cs_n,
    output reg spi_cs_sd,
    output reg spi_sck,
    output reg spi_mosi,
    input      spi_miso,

//---- boot 引擎的状态（只读展示）----
    input      boot_done,
    input      boot_err,
    input [16:0] boot_count
    );

    localparam [2:0]
    R_CTRL   = 3'd0,
    R_STATUS = 3'd1,
    R_TXDATA = 3'd2,
    R_RXDATA = 3'd3,
    R_CS     = 3'd4,
    R_DIV0   = 3'd5,
    R_DIV1   = 3'd6,
    R_BOOT   = 3'd7;

    wire sel = (bus_addr_in[31:24] == 8'd7);

//总线侧寄存器（只由总线块持有）
    reg        cs_level, cs_sd, lsb_first;
    reg [7:0]  div_flash, div_sd, div_active;
    reg [7:0]  tx_byte;

//收发状态（只由状态机持有）
    reg        busy, done;
    reg [2:0]  bit_count;
    reg [7:0]  tx_shift, rx_shift, rx_byte;

//SCK 发生器
    reg [7:0]  div_count;
    reg        sck_level, sck_level_d;

    reg        start_request, done_clear;
    reg [7:0]  tx_next, rx_next;

//哪一路被选中就用哪一组的分频；两路都选中或都没选中时用 flash 那组
    wire sd_selected = ~cs_sd & cs_level;

    wire sck_tick = busy & (div_count == div_active);
    wire sck_rise = sck_level & ~sck_level_d;
    wire sck_fall = ~sck_level & sck_level_d;

//CTRL 的写动作：start 是一拍脉冲，从写选通直接译出来给状态机用
    always @(*) begin
        start_request = sel & bus_we_in & (bus_addr_in[2:0] == R_CTRL) & bus_be_in[0] & bus_data_in[0];
        done_clear    = sel & bus_we_in & (bus_addr_in[2:0] == R_STATUS) & bus_be_in[0] & bus_data_in[1];
    end

//------------------------------------------------------------------------------
//  总线侧：两路 CS、位序、两组分频、待发字节。状态块只读它们，避免多驱动。
//------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            cs_level <= 1'b1;
            cs_sd <= 1'b1;
            lsb_first <= 1'b0;
            div_flash <= 8'd9;
            div_sd <= 8'd99;
            tx_byte <= 8'd0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            if (sel) begin
                case (bus_addr_in[2:0])
                R_CTRL: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0])
                            lsb_first <= bus_data_in[1];
                        else
                            lsb_first <= lsb_first;
                    end
                    else begin
                        bus_data_out <= {29'd0, lsb_first, busy, done};
                        ld_ready <= 1'b1;
                    end
                end
                R_STATUS: begin
                    if (!bus_we_in) begin
                        bus_data_out <= {30'd0, done, busy};
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
                R_CS: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0]) begin
                            cs_level <= bus_data_in[0];
                            cs_sd <= bus_data_in[1];
                        end
                        else begin
                            cs_level <= cs_level;
                            cs_sd <= cs_sd;
                        end
                    end
                    else begin
                        bus_data_out <= {30'd0, cs_sd, cs_level};
                        ld_ready <= 1'b1;
                    end
                end
                R_DIV0: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0])
                            div_flash <= bus_data_in[7:0];
                        else
                            div_flash <= div_flash;
                    end
                    else begin
                        bus_data_out <= {24'd0, div_flash};
                        ld_ready <= 1'b1;
                    end
                end
                R_DIV1: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0])
                            div_sd <= bus_data_in[7:0];
                        else
                            div_sd <= div_sd;
                    end
                    else begin
                        bus_data_out <= {24'd0, div_sd};
                        ld_ready <= 1'b1;
                    end
                end
                R_BOOT: begin
                    if (!bus_we_in) begin
                        bus_data_out <= {13'd0, boot_count, boot_err, boot_done};
                        ld_ready <= 1'b1;
                    end
                end
                endcase
            end
        end
    end

//------------------------------------------------------------------------------
//  SCK 发生器：只在 busy 时走，半周期 = div_active+1 个时钟
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
            div_active <= 8'd99;
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
                div_active <= sd_selected ? div_sd : div_flash;
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
//  输出：全部从寄存器出（模块 IO 必须有寄存器界）
//------------------------------------------------------------------------------
    always @(*) begin
        spi_cs_n = cs_level;
        spi_cs_sd = cs_sd;
        spi_sck = sck_level;
        spi_mosi = lsb_first ? tx_shift[0] : tx_shift[7];
    end

endmodule
