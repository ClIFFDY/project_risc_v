`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: cfg_spi
// Description: 配置 flash（Q64，板载 25Q064A 8 MB）专用 SPI 控制器
//
//   这块 flash 挂在 FPGA 的【专用配置引脚】上，和 pmem_spi 走的那组 GPIO 复用脚
//   是两回事：
//     CS   = C23 (FCS_B)
//     DI   = B24 (D00_MOSI)
//     DO   = A25 (D01_DIN)
//     CLK  = C8  (CCLK_0) —— 必须从 STARTUPE2 的 USRCCLKO 出去
//   WP# / HOLD# 没引出来 ⇒ 只能 x1 模式，本模块也只做 x1。
//
//   地址：APB 槽 7 的 sub2 = 0x1C02_0000（和 pmem_spi 同一个槽，靠
//   bus_addr_in[15:14] 分开：sub0 = pmem_spi，sub1 = 用户 SPI，sub2 = 本模块）。
//
//   寄存器（字节偏移 = 字偏移 × 4，字索引 = bus_addr_in[3:0]）
//     0  CTRL    W: [0] start  [1] 软复位（自清，16 拍后撤）
//                R: [0] busy  [1] done  [2] err
//     1  CMD     W: [7:0] 命令字节（0x03 读 / 0x02 写 / 0x06 写使能 /
//                            0x05 读状态 / 0x20 扇区擦 / 0xD8 块擦）
//     2  ADDR    W: [23:0] flash 内字节地址
//     3  LEN     W: [15:0] 数据字节数（读/写模式用；写使能/擦除填 0）
//     4  DATA    W: 写模式下一个输出字节；R: 读模式下一个收到的字节
//     5  STATUS  R: [0] busy  [1] done  [2] err  [3] rx_valid  [4] tx_wait
//     6  DIV     W/R: [7:0] SCK = clk / (2*(div+1))
//     7  GUARD   R: [23:0] 写/擦保护下限（< 这个地址一律拒）
//
//   ★ 地址护栏（这是本模块存在的主要理由）
//     比特流占 0x000000 ~ 0x6FFFFF，谁都不许碰。所以：
//       · 写（0x02）、扇区擦（0x20）、块擦（0xD8）：ADDR 或它的覆盖范围
//         低于 GUARD ⇒ 直接置 err 返回，SPI 总线一个电平都不动。
//       · 整片擦（0xC7 / 0x60）、写状态（0x01）⇒ 一律拒绝。
//
//   ★ 时序：SPI Mode 0（SCK 空闲低、上升沿采样）。SCK 由 clk 分频产生，
//     上板时 clk=50 MHz、DIV 默认给 9（2.5 MHz），和 pmem_spi 的 flash 档一致。
//
//   ★ 读是【流式】的：start 之后 FSM 把 CS 压住、按 LEN 连续收字节，每收一个
//     抬 rx_valid 等 CPU 读 DATA，读完再收下一个。bootloader 一个 ADDR + LEN
//     就能把整段镜像读下来。
//
//   ★ 写同理：CPU 每写一次 DATA 发一个字节，全部发完才抬 CS。擦/写之后的忙等
//     由软件自己读 flash 的 WIP（0x05），本模块不管。
//////////////////////////////////////////////////////////////////////////////////


module cfg_spi #(
    parameter integer CLK_MHZ   = 50,
    parameter integer SCK_KHZ   = 2500,
    parameter [23:0]  GUARD_LOW = 24'h700000
)(
    input             clk, rst,
    input             bus_sel_in,
    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    input             cfg_miso,

    output reg [31:0] bus_data_out,
    output reg        ld_ready,
    output reg        cfg_cs_n,
    output reg        cfg_sck,
    output reg        cfg_mosi,
    output reg        cfg_reset_out
    );

//===============================================================
// 寄存器索引
//===============================================================
    localparam [3:0]
    R_CTRL   = 4'd0,
    R_CMD    = 4'd1,
    R_ADDR   = 4'd2,
    R_LEN    = 4'd3,
    R_DATA   = 4'd4,
    R_STATUS = 4'd5,
    R_DIV    = 4'd6,
    R_GUARD  = 4'd7;

//===============================================================
// SPI 命令
//===============================================================
    localparam [7:0]
    C_READ_STATUS = 8'h05,
    C_WRITE_EN    = 8'h06,
    C_READ        = 8'h03,
    C_PROGRAM     = 8'h02,
    C_ERASE_4K    = 8'h20,
    C_ERASE_64K   = 8'hD8;

//===============================================================
// 主状态
//===============================================================
    localparam [2:0]
    S_IDLE  = 3'd0,
    S_CS    = 3'd1,     // CS 拉低后空一拍
    S_SEND  = 3'd2,     // 发一个字节（命令 / 地址 / 数据共用）
    S_RECV  = 3'd3,     // 收一个字节
    S_WDATA = 3'd4,     // 等 CPU 给下一个输出字节
    S_RDACK = 3'd5,     // 等 CPU 把收到的字节读走
    S_END   = 3'd6;     // 抬 CS 收尾

//===============================================================
// 分频：半个 SCK 周期 = div+1 拍
//===============================================================
    localparam integer HALF_RAW = (CLK_MHZ * 1000 + 2 * SCK_KHZ - 1) / (2 * SCK_KHZ);
    localparam [7:0]   DIV_INIT = (HALF_RAW <= 1) ? 8'd0 :
                                  (HALF_RAW > 256) ? 8'd255 : HALF_RAW[7:0] - 8'd1;

//===============================================================
// 内部信号
//===============================================================
    reg  [7:0]  div_reg;
    reg  [8:0]  div_cnt;
    reg         half;         // 0 = SCK 低（驱动 MOSI），1 = SCK 高（采 MISO）
    reg         miso_q;

    reg  [7:0]  cmd_reg;
    reg  [23:0] addr_reg;
    reg  [15:0] len_reg;

    reg  [2:0]  state;
    reg  [1:0]  phase;        // 0 = 命令，1..3 = 地址高→低，之后是数据
    reg         rw;           // 1 = 读（收），0 = 写（发）
    reg  [2:0]  bit_i;
    reg  [7:0]  shifter;
    reg  [15:0] data_left;
    reg  [7:0]  rx_byte;
    reg         rx_valid;
    reg  [7:0]  tx_byte;
    reg         tx_valid;
    reg         err_reg;
    reg         done_reg;
//收字节前先空一个位时间：flash 要到第一个 negedge 才把首字节摆上 MISO，
//  不空这一拍，第 0 个字节会采到 0x00（台子上实测过）
    reg         recv_prime;

    reg  [4:0]  reset_cnt;

    reg  [23:0] guard_end;
    reg         guard_ok;

//===============================================================
// 总线译码
//===============================================================
    reg sel, wr, rd, start_write, reset_write, rx_read, div_write;
    always @(*) begin
        sel         = bus_sel_in;
        wr          = sel & bus_we_in & (|bus_be_in);
        rd          = sel & ~bus_we_in;
        start_write = wr & (bus_addr_in[3:0] == R_CTRL) & bus_data_in[0];
        reset_write = wr & (bus_addr_in[3:0] == R_CTRL) & bus_data_in[1];
        rx_read     = rd & (bus_addr_in[3:0] == R_DATA);
        div_write   = wr & (bus_addr_in[3:0] == R_DIV);
    end

//===============================================================
// 护栏：命令 + 地址这一对能不能放行
//===============================================================
    always @(*) begin
        if (cmd_reg == C_ERASE_4K)
            guard_end = addr_reg + 24'h000FFF;
        else if (cmd_reg == C_ERASE_64K)
            guard_end = addr_reg + 24'h00FFFF;
        else
            guard_end = addr_reg;

        guard_ok = 1'b1;
        if ((cmd_reg == 8'hC7) | (cmd_reg == 8'h60) | (cmd_reg == 8'h01))
            guard_ok = 1'b0;
        else if ((cmd_reg == C_PROGRAM) | (cmd_reg == C_ERASE_4K) | (cmd_reg == C_ERASE_64K)) begin
            if ((addr_reg < GUARD_LOW) | (guard_end < GUARD_LOW))
                guard_ok = 1'b0;
        end
    end

//===============================================================
// 软复位：写 CTRL[1] ⇒ 拉 16 拍 cfg_reset_out
//===============================================================
    always @(posedge clk) begin
        if (rst) begin
            reset_cnt <= 5'd0;
        end
        else if (reset_write) begin
            reset_cnt <= 5'd16;
        end
        else if (reset_cnt != 5'd0) begin
            reset_cnt <= reset_cnt - 5'd1;
        end
    end

    always @(*) begin
        cfg_reset_out = (reset_cnt != 5'd0);
    end

//===============================================================
// 寄存器写口 + 命令 FSM
//===============================================================
    always @(posedge clk) begin
        if (rst) begin
            div_reg      <= DIV_INIT;
            cmd_reg      <= 8'd0;
            addr_reg     <= 24'd0;
            len_reg      <= 16'd0;
            state        <= S_IDLE;
            phase        <= 2'd0;
            rw           <= 1'b0;
            bit_i        <= 3'd7;
            shifter      <= 8'd0;
            data_left    <= 16'd0;
            rx_byte      <= 8'd0;
            rx_valid     <= 1'b0;
            tx_byte      <= 8'd0;
            tx_valid     <= 1'b0;
            err_reg      <= 1'b0;
            done_reg     <= 1'b0;
            recv_prime   <= 1'b0;
            div_cnt      <= 9'd0;
            half         <= 1'b0;
            miso_q       <= 1'b0;
            cfg_cs_n     <= 1'b1;
            cfg_sck      <= 1'b0;
            cfg_mosi     <= 1'b0;
            bus_data_out <= 32'd0;
            ld_ready     <= 1'b0;
        end
        else begin
            ld_ready <= 1'b0;
            miso_q   <= cfg_miso;

//---- 寄存器写口 ----
            if (div_write) begin
                div_reg <= bus_data_in[7:0];
            end
            if (wr & (bus_addr_in[3:0] == R_CMD)) begin
                cmd_reg <= bus_data_in[7:0];
            end
            if (wr & (bus_addr_in[3:0] == R_ADDR)) begin
                addr_reg <= bus_data_in[23:0];
            end
            if (wr & (bus_addr_in[3:0] == R_LEN)) begin
                len_reg <= bus_data_in[15:0];
            end
            if (wr & (bus_addr_in[3:0] == R_DATA)) begin
                tx_byte  <= bus_data_in[7:0];
                tx_valid <= 1'b1;
            end
            if (rx_read) begin
                rx_valid <= 1'b0;
            end
//忙的时候又来一次 start：不许静默丢掉，置 err 让软件看见
            if (start_write & (state != S_IDLE)) begin
                err_reg <= 1'b1;
            end

//---- 主状态机 ----
            case (state)
            S_IDLE: begin
                cfg_cs_n <= 1'b1;
                cfg_sck  <= 1'b0;
                cfg_mosi <= 1'b0;
                if (start_write) begin
                    err_reg <= 1'b0;
                    if (~guard_ok) begin
                        err_reg  <= 1'b1;
                        done_reg <= 1'b1;
                    end
                    else begin
                        done_reg  <= 1'b0;
                        tx_valid  <= 1'b0;
                        rx_valid  <= 1'b0;
                        phase     <= 2'd0;
                        bit_i     <= 3'd7;
                        shifter   <= cmd_reg;
                        div_cnt   <= 9'd0;
                        half      <= 1'b0;
                        cfg_cs_n  <= 1'b0;
                        state     <= S_CS;
                        if ((cmd_reg == C_READ) | (cmd_reg == C_READ_STATUS)) begin
                            rw        <= 1'b1;
                            if (len_reg == 16'd0)
                                data_left <= 16'd1;
                            else
                                data_left <= len_reg;
                        end
                        else begin
                            rw        <= 1'b0;
                            data_left <= len_reg;
                        end
                    end
                end
            end

            S_CS: begin
                div_cnt <= 9'd0;
                half    <= 1'b0;
                cfg_sck <= 1'b0;
                state   <= S_SEND;
            end

            S_SEND: begin
                if (half == 1'b0) begin
                    cfg_mosi <= shifter[7];
                    if (div_cnt == {1'b0, div_reg}) begin
                        div_cnt <= 9'd0;
                        half    <= 1'b1;
                        cfg_sck <= 1'b1;
                    end
                    else begin
                        div_cnt <= div_cnt + 9'd1;
                    end
                end
                else begin
                    if (div_cnt == {1'b0, div_reg}) begin
                        div_cnt <= 9'd0;
                        half    <= 1'b0;
                        cfg_sck <= 1'b0;
                        shifter <= {shifter[6:0], 1'b0};
                        if (bit_i != 3'd0) begin
                            bit_i <= bit_i - 3'd1;
                        end
                        else begin
                            bit_i <= 3'd7;
                            if (phase != 2'd3) begin
                                phase <= phase + 2'd1;
                                if (phase == 2'd0)
                                    shifter <= addr_reg[23:16];
                                else if (phase == 2'd1)
                                    shifter <= addr_reg[15:8];
                                else
                                    shifter <= addr_reg[7:0];
                                state <= S_SEND;
                            end
                            else if (data_left == 16'd0) begin
                                state <= S_END;
                            end
                            else if (rw) begin
                                state <= S_RECV;
                            end
                            else begin
                                state <= S_WDATA;
                            end
                        end
                    end
                    else begin
                        div_cnt <= div_cnt + 9'd1;
                    end
                end
            end

            S_RECV: begin
//Mode 0：从机在 SCK 下降沿换数据，主机在上升沿采样。
//  所以采样点在「拉高 SCK 那一拍」，不在拉低那一拍。
                if (half == 1'b0) begin
                    cfg_mosi <= 1'b0;
                    if (div_cnt == {1'b0, div_reg}) begin
                        div_cnt <= 9'd0;
                        half    <= 1'b1;
                        cfg_sck <= 1'b1;
                        begin
                            shifter <= {shifter[6:0], cfg_miso};
                            if (bit_i != 3'd0) begin
                                bit_i <= bit_i - 3'd1;
                            end
                            else begin
                                bit_i     <= 3'd7;
                                rx_byte   <= {shifter[6:0], cfg_miso};
                                rx_valid  <= 1'b1;
                                data_left <= data_left - 16'd1;
                                state     <= S_RDACK;
                            end
                        end
                    end
                    else begin
                        div_cnt <= div_cnt + 9'd1;
                    end
                end
                else begin
                    if (div_cnt == {1'b0, div_reg}) begin
                        div_cnt <= 9'd0;
                        half    <= 1'b0;
                        cfg_sck <= 1'b0;
                    end
                    else begin
                        div_cnt <= div_cnt + 9'd1;
                    end
                end
            end

            S_RDACK: begin
                if (~rx_valid) begin
                    if (data_left == 16'd0)
                        state <= S_END;
                    else
                        state <= S_RECV;
                end
            end

            S_WDATA: begin
                if (tx_valid) begin
                    tx_valid <= 1'b0;
                    shifter  <= tx_byte;
                    bit_i    <= 3'd7;
                    data_left <= data_left - 16'd1;
                    div_cnt  <= 9'd0;
                    half     <= 1'b0;
                    state    <= S_SEND;
                end
            end

            S_END: begin
                cfg_sck  <= 1'b0;
                cfg_cs_n <= 1'b1;
                done_reg <= 1'b1;
                state    <= S_IDLE;
            end

            default: begin
                state <= S_IDLE;
            end
            endcase

//---- 读口 ----
            bus_data_out <= 32'd0;
            if (rd) begin
                ld_ready <= 1'b1;
                case (bus_addr_in[3:0])
                R_CTRL:   bus_data_out <= {29'd0, (state != S_IDLE), done_reg, (state != S_IDLE)};
                R_STATUS: bus_data_out <= {27'd0, (state == S_WDATA), rx_valid, err_reg,
                                           done_reg, (state != S_IDLE)};
                R_DATA:   bus_data_out <= {24'd0, rx_byte};
                R_DIV:    bus_data_out <= {24'd0, div_reg};
                R_GUARD:  bus_data_out <= {8'd0, GUARD_LOW};
                default:  bus_data_out <= 32'd0;
                endcase
            end
        end
    end

endmodule
