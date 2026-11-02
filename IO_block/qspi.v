`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: qspi
// Description:
//   标准四线 QSPI 主机（1/2/4 线可配），字节 FIFO 接口，走总线寄存器访问。
//
//   帧结构（和标准 flash 命令一致）：
//     CS↓ → 命令字节（永远单线，从 TX FIFO 取）
//          → 地址/其它字节（同样来自 TX FIFO，按 lane 宽度发）
//          → dummy 拍（lane 宽度）
//          → 数据相：rw=1 收 ndata 字节进 RX FIFO；rw=0 从 TX FIFO 继续发
//     → CS↑
//
//   ★ 三态按 SPI 惯例：输出相位 io_oe=1 驱动，输入相位 io_oe=0 放开，
//     回读一律走独立输入 io_in（与 gpio_group 里 SDA/SCL 同一套写法）。
//   ★ 命令相固定单线（io0），地址/数据相按 ctrl.lane 切宽度。
//
//   寄存器（bus_addr_in[5:0]，槽位 7 -> bus_addr_in[31:24]==8'd7）：
//     0x00 R_TX    写：压一个字节进 TX FIFO
//     0x04 R_RX    读：从 RX FIFO 取一个字节（低 8 位）
//     0x08 R_CTRL  [0]start [1]rw(1=读) [3:2]lane(0=1线,1=2线,2=4线)
//                  [8:4]dummy(拍) [23:16]ndata(字节)
//     0x0c R_DIV   时钟分频（sck 半周期 = div+1 个 clk）
//     0x10 R_STATUS {…, done, busy, rx_full, rx_empty, tx_empty}
//     0x14 R_IRQ   {…, done}，写 1 清
//////////////////////////////////////////////////////////////////////////////////

module qspi(
    input clk, rst,

    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    output reg [31:0] bus_data_out,
    output reg        ld_ready,
    output reg        qspi_irq,

    output reg        sck,
    output reg        cs_n,
    output reg [3:0]  io_out,
    output reg [3:0]  io_oe,
    input      [3:0]  io_in
    );

    localparam [5:0]
    R_TX     = 6'h00,
    R_RX     = 6'h04,
    R_CTRL   = 6'h08,
    R_DIV    = 6'h0c,
    R_STATUS = 6'h10,
    R_IRQ    = 6'h14;

    localparam [1:0]
    P_CMD   = 2'd0,
    P_DUMMY = 2'd1,
    P_DATA  = 2'd2;

    localparam [1:0]
    S_IDLE = 2'd0,
    S_XFER = 2'd1,
    S_END  = 2'd2;

//====================== 寄存器 ======================
    reg        start_bit, rw;
    reg [1:0]  lane;          // 0=1线 1=2线 2=4线
    reg [4:0]  dummy_cfg;     // dummy 拍数
    reg [7:0]  ndata;         // 数据相字节数
    reg [3:0]  div_cfg;

//====================== FIFO ======================
    reg [7:0] tx_fifo [0:3];
    reg [1:0] tx_wp, tx_rp;
    reg [7:0] rx_fifo [0:3];
    reg [1:0] rx_wp, rx_rp;

    wire tx_empty = (tx_wp == tx_rp);
    wire tx_full  = (tx_wp[1:0] == tx_rp[1:0]) && (tx_wp[1] != tx_rp[1]);
    wire rx_empty = (rx_wp == rx_rp);
    wire rx_full  = (rx_wp[1:0] == rx_rp[1:0]) && (rx_wp[1] != rx_rp[1]);

//====================== 引擎 ======================
    reg [1:0]  state;
    reg [1:0]  phase;
    reg [2:0]  div_cnt;
    reg        sck_r;
    reg [2:0]  bits_left;     // 当前字节还剩几组（每组 = lane 宽度个 bit），3 位才能表到 8
    reg [7:0]  sh_out, sh_in;
    reg [7:0]  dummy_left, data_left;
    reg        done_r;

    wire [2:0] width  = (lane == 2'd2) ? 3'd4 : (lane == 2'd1) ? 3'd2 : 3'd1;

    wire tick  = (div_cnt == div_cfg);
    wire rise  = tick && !sck_r;
    wire fall  = tick &&  sck_r;

    always @(posedge clk) begin
        if (rst) begin
            div_cnt <= 3'd0;
            sck_r   <= 1'b0;
            sck     <= 1'b0;
        end
        else if (state != S_XFER) begin
            div_cnt <= 3'd0;
            sck_r   <= 1'b0;
            sck     <= 1'b0;
        end
        else if (tick) begin
            div_cnt <= 3'd0;
            sck_r   <= ~sck_r;
            sck     <= ~sck_r;
        end
        else div_cnt <= div_cnt + 3'd1;
    end

//发送数据：按 lane 宽度把 sh_out 的高位铺到 io 上
    always @(*) begin
        io_out = 4'hF;
        io_oe  = 4'h0;
        if (state == S_XFER) begin
            if (phase == P_DATA && rw) begin
//读相位：全部放开，由从设备驱动
                io_out = 4'hF;
                io_oe  = 4'h0;
            end
            else begin
                case (lane)
                2'd2: begin
                    io_out = sh_out[7:4];
                    io_oe  = 4'hF;
                end
                2'd1: begin
                    io_out = {2'b11, sh_out[7:6]};
                    io_oe  = 4'b0011;
                end
                default: begin
                    io_out = {3'b111, sh_out[7]};
                    io_oe  = 4'b0001;
                end
                endcase
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            phase      <= P_CMD;
            cs_n       <= 1'b1;
            bits_left  <= 2'd0;
            sh_out     <= 8'hFF;
            dummy_left <= 8'd0;
            data_left  <= 8'd0;
            done_r     <= 1'b0;
            start_bit  <= 1'b0;
            rw         <= 1'b0;
            lane       <= 2'd0;
            dummy_cfg  <= 5'd0;
            ndata      <= 8'd0;
            div_cfg    <= 4'd1;
            qspi_irq   <= 1'b0;
            tx_wp      <= 2'd0;
            tx_rp      <= 2'd0;
            rx_wp      <= 2'd0;
            rx_rp      <= 2'd0;
        end
        else begin
            done_r <= 1'b0;

            case (state)
            S_IDLE: begin
                cs_n <= 1'b1;
                if (start_bit) begin
                    start_bit <= 1'b0;
                    if (tx_empty) begin
//没有命令字节，直接报完成（避免空等）
                        done_r   <= 1'b1;
                        qspi_irq <= 1'b1;
                    end
                    else begin
                        cs_n       <= 1'b0;
                        phase      <= P_CMD;
                        sh_out     <= tx_fifo[tx_rp[1:0]];
                        tx_rp      <= tx_rp + 2'd1;
//★ 一个字节要发【满】width 组：1 线 8 组 → 初值 7（先发后减），
//  写成 0 会让第一个下降沿就判"字节结束"，只发出 1 个 bit 就切相位。
                        bits_left  <= (lane == 2'd2) ? 3'd1 : (lane == 2'd1) ? 3'd3 : 3'd7;
                        state      <= S_XFER;
                    end
                end
            end
            S_XFER: begin
                if (rise) begin
//采样沿：读相位把线上的值收进来
                    if ((phase == P_DATA) && rw) begin
                        case (width)
                        3'd4: sh_in = {sh_in[3:0], io_in[3:0]};
                        3'd2: sh_in = {sh_in[5:0], io_in[1:0]};
                        default: sh_in = {sh_in[6:0], io_in[0]};
                        endcase
                        if (bits_left == 2'd0) begin
                            rx_fifo[rx_wp[1:0]] <= sh_in;
                            rx_wp   <= rx_wp + 2'd1;
                            data_left <= data_left - 8'd1;
                            bits_left <= (width == 3'd4) ? 2'd1 : (width == 3'd2) ? 2'd3 : 3'd7;
                            if (data_left == 8'd1) state <= S_END;
                        end
                        else bits_left <= bits_left - 2'd1;
                    end
                end
                if (fall) begin
//移位沿：发送相位把下一组推出去
                    if (!((phase == P_DATA) && rw)) begin
                        case (width)
                        3'd4: sh_out = {sh_out[3:0], 4'hF};
                        3'd2: sh_out = {sh_out[5:0], 2'b11};
                        default: sh_out = {sh_out[6:0], 1'b1};
                        endcase
                        if (bits_left == 2'd0) begin
                            bits_left <= (width == 3'd4) ? 2'd1 : (width == 3'd2) ? 2'd3 : 3'd7;
                            case (phase)
                            P_CMD: begin
                                if (tx_empty) begin
                                    phase      <= (dummy_cfg == 5'd0) ? P_DATA : P_DUMMY;
                                    dummy_left <= {dummy_cfg, 1'b0};   // 拍数 → 沿数
                                    if (rw) begin
                                        data_left <= ndata;
                                        sh_in     <= 8'hFF;
                                        bits_left <= (width == 3'd4) ? 2'd1 :
                                                     (width == 3'd2) ? 2'd3 : 3'd7;
                                    end
                                    else begin
                                        data_left <= ndata;
                                        if (ndata == 8'd0) state <= S_END;
                                    end
                                end
                                else begin
                                    sh_out <= tx_fifo[tx_rp[1:0]];
                                    tx_rp  <= tx_rp + 2'd1;
                                end
                            end
                            P_DUMMY: begin
                                if (dummy_left <= 8'd1) phase <= P_DATA;
                                else dummy_left <= dummy_left - 8'd1;
                            end
                            default: begin
                                if (!rw) begin
                                    if ((data_left <= 8'd1) || tx_empty) state <= S_END;
                                    else begin
                                        sh_out    <= tx_fifo[tx_rp[1:0]];
                                        tx_rp     <= tx_rp + 2'd1;
                                        data_left <= data_left - 8'd1;
                                    end
                                end
                            end
                            endcase
                        end
                        else bits_left <= bits_left - 2'd1;
                    end
                end
            end
            S_END: begin
                cs_n     <= 1'b1;
                done_r   <= 1'b1;
                qspi_irq <= 1'b1;
                state    <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase

//====================== 总线寄存器 ======================
            bus_data_out <= 32'd0;
            ld_ready     <= 1'b0;
            if (bus_addr_in[31:24] == 8'd7) begin
                case (bus_addr_in[5:0])
                R_TX: begin
                    if (bus_we_in) begin
                        if (!tx_full) begin
                            tx_fifo[tx_wp[1:0]] <= bus_data_in[7:0];
                            tx_wp <= tx_wp + 2'd1;
                        end
                    end
                    else begin
                        bus_data_out <= {30'd0, tx_full, tx_empty};
                        ld_ready     <= 1'b1;
                    end
                end
                R_RX: begin
                    if (!bus_we_in) begin
                        bus_data_out <= {24'd0, rx_fifo[rx_rp[1:0]]};
                        if (!rx_empty) rx_rp <= rx_rp + 2'd1;
                        ld_ready     <= 1'b1;
                    end
                end
                R_CTRL: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0]) begin
                            start_bit <= bus_data_in[0];
                            rw        <= bus_data_in[1];
                            lane      <= bus_data_in[3:2];
                            dummy_cfg <= bus_data_in[8:4];
                        end
                        if (bus_be_in[2]) ndata <= bus_data_in[23:16];
                    end
                    else begin
                        bus_data_out <= {8'd0, ndata, 10'd0, dummy_cfg, 2'd0, lane, rw, start_bit};
                        ld_ready     <= 1'b1;
                    end
                end
                R_DIV: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0]) div_cfg <= bus_data_in[3:0];
                    end
                    else begin
                        bus_data_out <= {28'd0, div_cfg};
                        ld_ready     <= 1'b1;
                    end
                end
                R_STATUS: begin
                    if (!bus_we_in) begin
                        bus_data_out <= {27'd0, done_r, (state != S_IDLE),
                                         rx_full, rx_empty, tx_empty};
                        ld_ready     <= 1'b1;
                    end
                end
                R_IRQ: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0] && bus_data_in[0]) qspi_irq <= 1'b0;
                    end
                    else begin
                        bus_data_out <= {31'd0, qspi_irq};
                        ld_ready     <= 1'b1;
                    end
                end
                default: begin
                    if (!bus_we_in) ld_ready <= 1'b1;
                end
                endcase
            end
        end
    end

endmodule
