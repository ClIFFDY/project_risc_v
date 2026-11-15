`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: pmem_spi
// Description: 7 号槽 sub0（0x1C00_0000）的双通道 SPI 主机：一路给板上的 flash，
//              一路给 SD 卡。
//
//   ★ 两个逻辑通道【共享 SCK/MOSI/MISO】，只有 CS 是各自一根。
//     所以只有一个收发引擎；"两个控制器"在软件眼里就是两根 CS + 两组分频。
//     两路同时拉低 CS 会一起收到 SCK，这是软件的事，硬件不拦（写进文档就够了）。
//
//   寄存器（槽内【字】偏移 = bus_addr_in[4:0]；外设拿到的是字节地址>>2）：
//     0  CTRL      W: bit0 start（自清） bit1 lsb_first bit2 auto
//                  R: {28'd0, auto, lsb_first, busy, done}
//     1  STATUS    R: bit0 busy bit1 done bit2 tx_empty bit3 tx_full
//                     bit4 rx_empty bit5 rx_full bit6 rx_ovf bit7 tx_ovf
//                  W: bit1 写 1 清 done
//     2  TXDATA    W: 压 TX FIFO（满时丢弃并置 tx_ovf）  R: 回读最后一次写进去的
//     3  RXDATA    R: 弹 RX FIFO（空时返回最后一次收到的字节）
//     4  CS        W: bit0 = flash 的 CS（0 = 选中） bit1 = SD 的 CS（0 = 选中）
//                  R: 回读
//     5  DIV0      W/R: flash 通道分频，SCK = clk/(2*(div+1))
//     6  DIV1      W/R: SD 通道分频，同理（SD 初始化要求 100~400 kHz）
//     7  BOOT      R: {13'd0, boot_count[16:0], boot_err, boot_done}
//     8  FIFO_CNT  R: {22'd0, rx_count[4:0], tx_count[4:0]}
//                  W: bit0 清 TX，bit1 清 RX（并清 rx_ovf）
//     9  IRQ       W: bit0 rx_ie bit1 tx_ie [7:4] tx_low
//                  R: {23'd0, irq, tx_evt, rx_evt, tx_low[3:0], tx_ie, rx_ie}
//   对应字节地址（槽基址 0x1C00_0000）：00 / 04 / 08 / 0C / 10 / 14 / 18 / 1C / 20 / 24
//
//   ★ 2026-11-13 加 FIFO / 连发 / 中断。老的一字节一 start 的行为完全保留
//     （auto = 0 时与改之前逐拍一致），新增位全在寄存器更高位。
//   ★ 与 spi.v（sub1，用户外设）共用同一张寄存器图。改一边必须同时改另一边。
//
//   ★ SPI Mode 0：SCK 空闲低；下降沿之后更新 MOSI，上升沿之后采 MISO。
//   ★ 分频在 start 那一拍【采样并锁存】：本拍哪一路的 CS 是低的就用哪一路的分频
//     （两路都低或都不低时用 flash 那组）。所以收发中途改 DIV 不影响进行中的这一字节。
//   ★ BOOT 那三个状态由 soc/boot_loader.v 驱动，本模块只负责摆出来给软件读。
//   ★ 中断是【电平】：rx_evt 跟 rx_empty 走，tx_evt 跟 tx_count <= tx_low 走。
//     两个使能位复位为 0 ⇒ 复位后不报。不要做成"置起等软件清"的粘滞位，
//     PLIC 每拍从电平重置 pending，粘滞位会造成中断风暴。
//////////////////////////////////////////////////////////////////////////////////


module pmem_spi #(
//★ 分频的复位值【不写死】：由"核时钟频率 + 目标 SCK"算出来。
//  换时钟（50 / 100 / 120 MHz）只改 CLK_MHZ，SCK 目标频率保持不变。
//  手算过的坑：原来按 50 MHz 写死 9 / 99，核改 120 MHz 之后 SCK 悄悄变成
//  6 MHz / 600 kHz —— 后者已经超出 SD 卡上电初始化 100~400 kHz 的硬要求。
    parameter integer CLK_MHZ       = 120,
    parameter integer FLASH_SCK_KHZ = 2500,
    parameter integer SD_SCK_KHZ    = 300
)(
    input clk, rst,
    input        bus_sel_in,
    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
//---- 两路 CS + 共享的三根线 ----
    input      spi_miso,
//---- boot 引擎的状态（只读展示）----
    input      boot_done,
    input      boot_err,
    input [16:0] boot_count,

    output reg [31:0] bus_data_out,
    output reg        ld_ready,
    output reg spi_cs_n,
    output reg spi_cs_sd,
    output reg spi_sck,
    output reg spi_mosi,
    output reg spi_irq
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
//  向上取整是故意的：宁可慢一点也不许超过目标频率（SD 卡对初始化频率有上限）。
    localparam integer FLASH_HALF = (CLK_MHZ * 1000 + 2 * FLASH_SCK_KHZ - 1) / (2 * FLASH_SCK_KHZ);
    localparam integer SD_HALF    = (CLK_MHZ * 1000 + 2 * SD_SCK_KHZ    - 1) / (2 * SD_SCK_KHZ);
    localparam [7:0]   DIV_FLASH_INIT = (FLASH_HALF <= 1) ? 8'd0 :
                                        (FLASH_HALF > 256) ? 8'd255 : FLASH_HALF[7:0] - 8'd1;
    localparam [7:0]   DIV_SD_INIT    = (SD_HALF    <= 1) ? 8'd0 :
                                        (SD_HALF    > 256) ? 8'd255 : SD_HALF[7:0]    - 8'd1;

//FIFO：16 深。★ 指针必须留 5 位（4 位索引 + 1 位区分空满），
//  只留 4 位的话满和空同值，满队会被读成空、静默覆盖未交付的数据。
    localparam integer FIFO_AW    = 4;
    localparam integer FIFO_DEPTH = 1 << FIFO_AW;
    localparam [FIFO_AW:0] FIFO_FULL = ({{FIFO_AW{1'b0}}, 1'b1} << FIFO_AW);

    reg [7:0] tx_mem [0:FIFO_DEPTH-1];
    reg [7:0] rx_mem [0:FIFO_DEPTH-1];
    reg [FIFO_AW:0] tx_wp, tx_rp, rx_wp, rx_rp;

//总线侧寄存器（只由总线块持有）
    reg        cs_level, cs_sd, lsb_first, auto_mode;
    reg [7:0]  div_flash, div_sd, tx_byte;
    reg        rx_ie, tx_ie;
    reg [3:0]  tx_low;
    reg        tx_ovf, rx_ovf;

//收发状态（只由状态机持有）
    reg        busy, done;
    reg [2:0]  bit_count;
    reg [7:0]  tx_shift, rx_shift, rx_byte;

//SCK 发生器
    reg [7:0]  div_count, div_active;
    reg        sck_level, sck_level_d;

//------------------------------------------------------------------------------
//  FIFO 状态与译码
//------------------------------------------------------------------------------
    reg [FIFO_AW:0] tx_count, rx_count;
    reg tx_empty, tx_full, rx_empty, rx_full;
    reg [7:0] tx_head;
    always @(*) begin
        tx_count = tx_wp - tx_rp;
        rx_count = rx_wp - rx_rp;
        tx_empty = (tx_wp == tx_rp);
        rx_empty = (rx_wp == rx_rp);
        tx_full = (tx_count == FIFO_FULL);
        rx_full = (rx_count == FIFO_FULL);
        tx_head = tx_mem[tx_rp[FIFO_AW-1:0]];
    end

//哪一路被选中就用哪一组的分频；两路都选中或都没选中时用 flash 那组
//SCK 边沿：全部锚在 sck_level 的寄存器沿上
//中断的两个电平（跟状态位走，不粘滞）
    reg sd_selected;
    reg sck_tick, sck_rise, sck_fall;
    reg rx_evt, tx_evt;
    always @(*) begin
        sd_selected = ~cs_sd & cs_level;
        sck_tick = busy & (div_count == div_active);
        sck_rise = sck_level & ~sck_level_d;
        sck_fall = ~sck_level & sck_level_d;
        rx_evt = ~rx_empty;
        tx_evt = (tx_count <= {1'b0, tx_low});
    end

    reg sel;
    reg [4:0] reg_sel;
    reg start_request, done_clear, tx_push, rx_pop, tx_clear, rx_clear;
//CTRL 的 start 是一拍脉冲，从写选通直接译出来给状态机用
    always @(*) begin
        sel = bus_sel_in;
        reg_sel = bus_addr_in[4:0];
        start_request = sel & bus_we_in & (reg_sel == R_CTRL)    & bus_be_in[0] & bus_data_in[0];
        done_clear = sel & bus_we_in & (reg_sel == R_STATUS)  & bus_be_in[0] & bus_data_in[1];
        tx_push = sel & bus_we_in & (reg_sel == R_TXDATA)  & bus_be_in[0];
        rx_pop = sel & ~bus_we_in & (reg_sel == R_RXDATA) & ~rx_empty;
        tx_clear = sel & bus_we_in & (reg_sel == R_FIFOCNT) & bus_be_in[0] & bus_data_in[0];
        rx_clear = sel & bus_we_in & (reg_sel == R_FIFOCNT) & bus_be_in[0] & bus_data_in[1];
    end

//引擎侧的取字节/换字节动作（组合量，由状态机块与 FIFO 块共用）
// 一次 start 起一个字节
// 连发：接着来
    reg tx_load;
    reg tx_pop, byte_end;
    reg byte_auto;
    reg byte_pop;
    always @(*) begin
        tx_load = start_request & ~busy;
        tx_pop = tx_load & ~tx_empty;
        byte_end = busy & sck_fall & (bit_count == 3'd7);
        byte_auto = byte_end & auto_mode & ~tx_empty;
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
            cs_level <= 1'b1;
            cs_sd <= 1'b1;
            lsb_first <= 1'b0;
            auto_mode <= 1'b0;
            div_flash <= DIV_FLASH_INIT;
            div_sd <= DIV_SD_INIT;
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
                    bus_data_out <= {30'd0, cs_sd, cs_level};
                    ld_ready <= 1'b1;
                end
                R_DIV0: begin
                    bus_data_out <= {24'd0, div_flash};
                    ld_ready <= 1'b1;
                end
                R_DIV1: begin
                    bus_data_out <= {24'd0, div_sd};
                    ld_ready <= 1'b1;
                end
                R_BOOT: begin
                    bus_data_out <= {13'd0, boot_count, boot_err, boot_done};
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
//★ 位序只在"不是 start 的那次写"里改。原来是无条件改，而 spi_transfer()
//  的写法是 "TXDATA = b; CTRL = START"，bit1 = 0 就把位序清掉了 ⇒ LSb-first
//  从来没生效过。现在 start 那次写不碰它，仍用 CTRL = 0 或 2 显式设置。
                    if (bus_be_in[0]) begin
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
                    if (bus_be_in[0]) begin
                        cs_level <= bus_data_in[0];
                        cs_sd <= bus_data_in[1];
                    end
                end
                R_DIV0: begin
                    if (bus_be_in[0])
                        div_flash <= bus_data_in[7:0];
                end
                R_DIV1: begin
                    if (bus_be_in[0])
                        div_sd <= bus_data_in[7:0];
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
//  ★ 采样/移位都锚在 sck_level 的【寄存器沿】上（sck_rise/sck_fall），
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
            div_active <= DIV_SD_INIT;
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
                div_active <= sd_selected ? div_sd : div_flash;
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
        spi_cs_n  = cs_level;
        spi_cs_sd = cs_sd;
        spi_sck   = sck_level;
        spi_mosi  = lsb_first ? tx_shift[0] : tx_shift[7];
    end

endmodule
