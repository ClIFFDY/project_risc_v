`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/12 08:58:46
// Design Name:
// Module Name: i2c
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


module i2c(
    input clk, rst,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    // ★ 2026-11-14（总线审计）：事务选通。读 DATA 会弹 RX FIFO，而 bus_addr_in 在
    //   事务结束后不撤（主设备 lat_addr 不清零）⇒ 只用"地址 + !we"判读会把 FIFO
    //   一路抽干。这里跟 uart_top / spi 一样，读副作用只在被选中那一拍发生。
    input bus_sel,
    input scl_in,
    input sda_in,

//★ 2026-11-13（D2）：给同层的 DPU（9 号槽）留一条侧带 —— 它不进总线，
//  直接把字节塞进 TX FIFO、直接拨 START/STOP/长度，等于替 CPU 干那三件事。
//  ★ 为什么不是"流式字节源 + 握手"：FIFO 本来就是字节队列，复用 8 深 + 计数
//    比另开一套握手省得多，而且 CPU 那条路径一个门都不动。
    input             dpu_push, // 一字节入队（DPU 负责不溢出）
    input      [7:0]  dpu_push_data,
    input             dpu_ctrl_we, // 一字节的"写 CTRL"，与 CPU 写 CTRL 同一条路
    input             dpu_ctrl_start,
    input             dpu_ctrl_stop,
    input      [7:0]  dpu_ctrl_len,
    output reg [31:0] bus_data_out,
    output reg ld_ready,
    output reg i2c_irq,

    output reg scl_oe,
    output reg sda_oe,
    output reg busy,
    output reg     [3:0]  dpu_tx_used, // 队列占用：DPU 靠它停手，留一格余量
    output reg        dpu_err            // NACK 中止脉冲（不是粘滞位）
    );

//★ 同上：SCL 分频由核时钟算。原来写死 50 MHz，核改 120 MHz 时
//  400 kHz 那档会变成 960 kHz，超出 fast-mode 器件上限。
    parameter integer CLK_MHZ = 100;
    localparam [31:0] CLK_FREQ = CLK_MHZ * 1_000_000;

    localparam [7:0]
        R_DATA     = 8'h00,
        R_CTRL     = 8'h04,
        R_CFG      = 8'h08,
        R_SLV_ADDR = 8'h0c,
        R_STATUS   = 8'h10,
        R_IRQ_STAT = 8'h14;

//★ 2026-11-13（D1/O5）：分频按 I2C 规范的【时间】直接算，不再写"13/25"这种魔法比例。
//  Fast-mode(400k)  t_LOW ≥ 1.3 µs、t_HIGH ≥ 0.6 µs（周期 2.5 µs）
//      ⇒ 低相固定 1.3 µs，剩下全给高相（@120 MHz：156 / 144）
//      ★ 原来给的 156/144 其实是对的，只是没人看得出来为什么；改成从时间推出来。
//  Standard(100k)   t_LOW ≥ 4.7 µs、t_HIGH ≥ 4.0 µs（周期 10 µs）
//      ⇒ 低 4.7 µs、高 5.3 µs（564 / 636）
//      ★ 原来给的是 480/720 = 4 µs / 6 µs —— **低相短于规范要求的 4.7 µs**，这一档顺手修正。
//  Ultra(1M)        非规范档，按 50% 给（60 / 60）。
//  ★ SIM_QUARTER > 0 时三档都用它 —— 只改时序、不改任何判据，台子靠它把
//    一帧从 2.7M 拍压到 70k 拍。
    parameter integer SIM_QUARTER = 0;
    localparam integer Q100_L = (SIM_QUARTER > 0) ? SIM_QUARTER : (CLK_FREQ * 47 / 10_000_000);
    localparam integer Q100_H = (SIM_QUARTER > 0) ? SIM_QUARTER : (CLK_FREQ * 53 / 10_000_000);
    localparam integer Q400_L = (SIM_QUARTER > 0) ? SIM_QUARTER : (CLK_FREQ * 13 / 10_000_000);
    localparam integer Q400_H = (SIM_QUARTER > 0) ? SIM_QUARTER : (CLK_FREQ * 12 / 10_000_000);
    localparam integer Q1M    = (SIM_QUARTER > 0) ? SIM_QUARTER : (CLK_FREQ / 2_000_000);

    localparam [2:0]
        M_IDLE  = 3'd0,
        M_START = 3'd1,
        M_SEND  = 3'd2,
        M_ACK1  = 3'd3,
        M_ACK2  = 3'd4,
        M_STOP  = 3'd5,
        M_STOP2 = 3'd6,
        M_BACK  = 3'd7;

    localparam [2:0]
        S_IDLE     = 3'd0,
        S_ADDR     = 3'd1,
        S_ADDR_ACK = 3'd2,
        S_RX       = 3'd3,
        S_RX_ACK   = 3'd4,
        S_TX       = 3'd5,
        S_TX_ACK   = 3'd6,
        S_DONE     = 3'd7;

    reg [3:0] cfg_reg;
    reg [15:0] slv_reg;
    reg [3:0] irq_stat;
    reg [3:0] irq_clr;
    reg mode, ctrl_start, ctrl_stop;
//★ 2026-11-13（D1/O3）：CTRL[15:8] = 本次传输的字节数（含地址字节），数完硬件自动 STOP。
//  0 = 老行为（软件自己拨 stop）。
    reg [7:0] ctrl_len;
    reg [7:0] m_len;
    reg len_en;
    always @(*) begin
        len_en = (ctrl_len != 8'd0);
    end
//★ 2026-11-13（D1/O4）：主机侧时钟拉伸的等待计数（超时保护用）。
    reg [15:0] stretch_cnt;
//★ 2026-11-13（D1/O2）：NACK 中止时把 TX FIFO 也倒掉。否则剩下那些没人要的字节
//  会在下一笔传输里被当成地址/数据发出去 —— 那比不中止还难查。
    reg        m_flush;

    reg [7:0] tx_buf [0:7];
    reg [3:0] tx_wr, tx_rd;
    reg [7:0] rx_buf [0:7];
    reg [3:0] rx_wr, rx_rd;

    reg [7:0] m_tx_data, s_tx_data;
    reg [7:0] rx_shift;
    reg [7:0] slv_recv;
    reg [7:0] s_rx_byte;
    reg [3:0] bit_cnt, s_bit_cnt;
    reg [15:0] cnt_l, cnt_h, cnt;
    reg phase_h;
    reg [2:0] m_state, s_state;
    reg scl_q1, scl_q2, sda_q1, sda_q2;
    reg scl_rise, scl_fall, start_det, stop_det;
    reg m_scl_low, m_sda_low, s_scl_low, s_sda_low;
    reg ack_ph;
    reg slv_match, rw_bit, stretch;
    reg m_pop, s_pop, s_rx_push;
    reg m_done, s_rx_done;
    reg m_start_clr, m_stop_clr;
    reg [7:0] reg_sel;
    reg [31:0] rd_data;
    reg [31:0] status;



//★ 2026-11-13（D1/O4）：主机侧时钟拉伸的等待条件。
//  ★ 判据只能看"我们自己有没有在拉低"，不能看 phase_h：phase_h 与 m_scl_low 在
//    M_ACK2→M_SEND 那条转移里本来就不一致（那边 m_scl_low<=1 而 phase_h<=0）。
//    第一版按 phase_h 判，每字节都要等满 4096 拍超时（实测 2861→39725 拍/字节）。
//    `!m_scl_low && !scl_q1` 的语义正是"我们放开了，线上却还是低" ⇒ 别人在拉伸。
//  ★ 超时保护：总线被拉死时不能把核挂住，超过 4096 拍就照常推进（并置 dpu_err 由上层看）。
    reg stretch_now, stretch_wait;
    always @(*) begin
        stretch_now = !m_scl_low && !scl_q1;
        stretch_wait = stretch_now && (stretch_cnt < 16'd4096);
    end

    reg tx_empty, tx_full, rx_empty, rx_full, slv_en;
    always @(*) begin
        tx_empty = (tx_wr == tx_rd);
        tx_full = (tx_wr[3] != tx_rd[3]) && (tx_wr[2:0] == tx_rd[2:0]);
        rx_empty = (rx_wr == rx_rd);
        rx_full = (rx_wr[3] != rx_rd[3]) && (rx_wr[2:0] == rx_rd[2:0]);
        slv_en = slv_reg[15];
    end

//★ 2026-11-13（D2）：DPU 侧带的三个观测口
    always @(*) begin
        dpu_tx_used = tx_wr - tx_rd;
    end

//分频：按 cfg[2:1] 给出 SCL 低/高相位拍数（见文件上方的 Q* 常量）
        // 100 kHz
        // 400 kHz
    always @(*) begin
        case (cfg_reg[2:1])
        2'd0: begin
            cnt_l = Q100_L;
            cnt_h = Q100_H;
        end
        2'd1: begin
            cnt_l = Q400_L;
            cnt_h = Q400_H;
        end
        2'd2: begin
            cnt_l = Q1M;
            cnt_h = Q1M;
        end
        // 1 MHz（非规范）
        default: begin
            cnt_l = Q100_L;
            cnt_h = Q100_H;
        end
        endcase
    end

//寄存器子地址
    always @(*) begin
        reg_sel = bus_addr_in[7:0];
    end

//SCL / SDA 两级同步
    always @(posedge clk) begin
        if (rst) begin
            scl_q1 <= 1'b1;
            scl_q2 <= 1'b1;
            sda_q1 <= 1'b1;
            sda_q2 <= 1'b1;
        end
        else begin
            scl_q1 <= scl_in;
            scl_q2 <= scl_q1;
            sda_q1 <= sda_in;
            sda_q2 <= sda_q1;
        end
    end

//边沿与起止条件；从机本次收到的字节
    always @(*) begin
        scl_rise  = scl_q1 && !scl_q2;
        scl_fall  = !scl_q1 && scl_q2;
        start_det = scl_q1 && !sda_q1 && sda_q2;
        stop_det  = scl_q1 && sda_q1 && !sda_q2;
        slv_recv  = {rx_shift[6:0], sda_q1};
    end

//输出选择、状态、中断聚合
    always @(*) begin
        scl_oe = mode ? s_scl_low : m_scl_low;
        sda_oe = mode ? s_sda_low : m_sda_low;
        status = 32'd0;
        if (mode)
            status[0] = slv_en && (s_state != S_IDLE) && (s_state != S_DONE);
        else
            status[0] = (m_state != M_IDLE) || !tx_empty;
        status[1] = irq_stat[0];
        status[2] = !rx_empty;
        status[3] = tx_full;
        status[4] = rx_full;
        status[5] = slv_match;
        busy = status[0];
        i2c_irq = |irq_stat;
    end

//中断状态单写口的写清掩码
    always @(*) begin
        irq_clr = 4'd0;
        if (bus_we_in && bus_addr_in[31:24] == 8'd5 && bus_be_in[0]) begin
            if (reg_sel == R_IRQ_STAT)
                irq_clr = bus_data_in[3:0];
            else if (reg_sel == R_CTRL && bus_data_in[2])
                irq_clr = 4'hf;
        end
    end

//寄存器读回
    always @(*) begin
        case (reg_sel)
        R_DATA:     rd_data = rx_empty ? 32'd0 : {24'd0, rx_buf[rx_rd[2:0]]};
        R_CTRL:     rd_data = {28'd0, mode, 1'b0, ctrl_stop, ctrl_start};
        R_CFG:      rd_data = {28'd0, cfg_reg[3:0]};
        R_SLV_ADDR: rd_data = {16'd0, slv_reg};
        R_STATUS:   rd_data = status;
        R_IRQ_STAT: rd_data = {28'd0, irq_stat};
        default:    rd_data = 32'd0;
        endcase
    end

//总线写
    always @(posedge clk) begin
        if (rst) begin
            tx_wr <= 4'd0;
            mode <= 1'b0;
            ctrl_start <= 1'b0;
            ctrl_stop <= 1'b0;
            cfg_reg <= 4'd0;
            slv_reg <= 16'd0;
        end
        else begin
//★ 2026-11-13（D1）：START/STOP 是**自清**位的 —— i2c.h 从第一天就这么写
//  （"启动一次主机事务（自清）"/"事务末尾产生 STOP（自清）"），但 RTL 里
//  m_start_clr / m_stop_clr 这两根线从来没人读，等于没接。后果：
//    · ctrl_stop 一旦置 1 永远是 1 ⇒ 第二笔事务在 FIFO 空过一次之后就截断；
//    · ctrl_start 一旦置 1 永远是 1 ⇒ NACK 中止之后 FIFO 里剩下的字节会自己
//      再起一笔事务（台子 T3 里那个凭空多出来的 0xA5 就是这么来的）。
//  总线写在后面，同一拍两种来源时以软件写为准。
            if (m_start_clr)
                ctrl_start <= 1'b0;
            if (m_stop_clr)
                ctrl_stop  <= 1'b0;
//★ 2026-11-13（D2）：DPU 的入队口。排在总线写前面 ⇒ 同一拍两边都想写时 DPU 赢
//  （软件在 DPU 跑着的时候别碰 I2C，这条写进 oled 的用法说明）。
            if (dpu_push && !tx_full) begin
                tx_buf[tx_wr[2:0]] <= dpu_push_data;
                tx_wr <= tx_wr + 4'd1;
            end
            else if (bus_addr_in[31:24] == 8'd5 && bus_we_in) begin
                case (reg_sel)
                R_DATA: begin
                    if (bus_be_in[0] && !tx_full) begin
                        tx_buf[tx_wr[2:0]] <= bus_data_in[7:0];
                        tx_wr <= tx_wr + 4'd1;
                    end
                end
                R_CTRL: begin
                    if (bus_be_in[0]) begin
                        if (bus_data_in[0])
                            ctrl_start <= 1'b1;
                        if (bus_data_in[1])
                            ctrl_stop <= 1'b1;
                        mode <= bus_data_in[3];
                        ctrl_len <= bus_data_in[15:8];     // D1/O3：本次传输的字节数（整字写）
                    end
                end
                R_CFG: begin
                    if (bus_be_in[0])
                        cfg_reg <= bus_data_in[3:0];
                end
                R_SLV_ADDR: begin
                    if (bus_be_in[1:0] == 2'b11)
                        slv_reg <= bus_data_in[15:0];
                end
                default: ;
                endcase
            end
//★ 2026-11-13（D2）：DPU 拨 CTRL 走的是同一条路（自清、长度、模式都一样），
//  只是入口从总线换成一根脉冲。
            if (dpu_ctrl_we) begin
                if (dpu_ctrl_start)
                    ctrl_start <= 1'b1;
                if (dpu_ctrl_stop)
                    ctrl_stop  <= 1'b1;
                ctrl_len <= dpu_ctrl_len;
            end
        end
    end

//总线读：读 DATA 弹 RX FIFO；data 与 ready 同沿
//★ 2026-11-14 晚（总线审计）：弹栈判据从"地址 + !we"改成"被选中 且 非写 且 本笔第一拍"。
//  原来一次读会把 8 深的 RX FIFO 一路抽干（和 uart 修之前一模一样），
//  而且写 DATA（主机发字节）之后地址还指着 DATA，也会跟着弹。
    reg bus_sel_q;

    always @(posedge clk) begin
        if (rst) begin
            rx_rd <= 4'd0;
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            bus_sel_q <= 1'b0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            bus_sel_q <= bus_sel;
            if (bus_sel && !bus_we_in && bus_addr_in[31:24] == 8'd5) begin
                bus_data_out <= rd_data;
                ld_ready <= 1'b1;
                if (reg_sel == R_DATA && !rx_empty && !bus_sel_q)
                    rx_rd <= rx_rd + 4'd1;
            end
        end
    end

//中断状态：单一写口，置位与写清同拍（写清赢）
    always @(posedge clk) begin
        if (rst)
            irq_stat <= 4'd0;
        else
            irq_stat <= (irq_stat | {stop_det, m_done, s_rx_done, dpu_err}) & ~irq_clr;
    end

//TX FIFO 读指针（主机/从机共用出口）
    always @(posedge clk) begin
        if (rst)
            tx_rd <= 4'd0;
        else if (m_flush)
            tx_rd <= tx_wr;
        else if (m_pop || s_pop)
            tx_rd <= tx_rd + 4'd1;
    end

//RX FIFO 写口
    always @(posedge clk) begin
        if (rst)
            rx_wr <= 4'd0;
        else if (s_rx_push)
            rx_wr <= rx_wr + 4'd1;
    end

    always @(posedge clk) begin
        if (s_rx_push)
            rx_buf[rx_wr[2:0]] <= s_rx_byte;
    end

//主机 FSM：自计时推进，SCL 由 scl_oe 开漏拉低
            // 一拍脉冲：上一拍置起、这一拍倒 FIFO
                            // 这笔事务到此为止，软件那次 STOP 请求一并吃掉
    always @(posedge clk) begin
        if (rst) begin
            m_state <= M_IDLE;
            m_scl_low <= 1'b0;
            m_sda_low <= 1'b0;
            m_tx_data <= 8'd0;
            bit_cnt <= 4'd0;
            phase_h <= 1'b0;
            cnt <= 16'd0;
            m_pop <= 1'b0;
            dpu_err <= 1'b0;
            m_done <= 1'b0;
            m_start_clr <= 1'b0;
            m_stop_clr <= 1'b0;
            m_len <= 8'd0;
            stretch_cnt <= 16'd0;
            m_flush <= 1'b0;
        end
        else begin
            m_pop <= 1'b0;
            dpu_err <= 1'b0;
            m_done <= 1'b0;
            m_flush <= 1'b0;
            m_start_clr <= 1'b0;
            m_stop_clr <= 1'b0;
//★ 2026-11-13（D1/O4）：该放开 SCL 的那一半里，如果 SCL 还没真的起来（别的从机
//  拉着），相位计数停住不动。原来主机完全不看 scl_in，SCL 全靠开环数拍 ——
//  别人拉时钟时会照拉照放，表现为偶发丢字节。
//  ★ 超时保护：总线被拉死时不能把核挂住，超过 4096 拍就照常推进并置 dpu_err。
            if (stretch_wait)
                cnt <= cnt;
            else if (cnt >= (phase_h ? cnt_h : cnt_l) - 1'b1)
                cnt <= 16'd0;
            else
                cnt <= cnt + 16'd1;
            if (stretch_now && (stretch_cnt != 16'hffff))
                stretch_cnt <= stretch_cnt + 16'd1;
            else if (!stretch_now)
                stretch_cnt <= 16'd0;
            if (mode == 1'b0) begin
                if ((cnt == (phase_h ? cnt_h : cnt_l) - 1'b1) && !stretch_wait) begin
                    case (m_state)
                    M_IDLE: begin
                        m_scl_low <= 1'b0;
                        m_sda_low <= 1'b0;
                        if (!tx_empty && ctrl_start) begin
                            m_tx_data <= tx_buf[tx_rd[2:0]];
                            m_pop <= 1'b1;
                            m_start_clr <= 1'b1;
                            bit_cnt <= 4'd8;
                            phase_h <= 1'b0;
                            m_len    <= ctrl_len;          // D1/O3
                            m_state <= M_START;
                        end
                    end
                    M_START: begin
                        m_sda_low <= 1'b1;
                        m_scl_low <= 1'b0;
                        phase_h <= 1'b0;
                        m_state <= M_SEND;
                    end
                    M_SEND: begin
                        if (!phase_h) begin
                            m_scl_low <= 1'b1;
                            m_sda_low <= ~m_tx_data[7];
                            m_tx_data <= {m_tx_data[6:0], 1'b0};
                            phase_h <= 1'b1;
                        end
                        else begin
                            m_scl_low <= 1'b0;
                            phase_h <= 1'b0;
                            if (bit_cnt > 4'd1)
                                bit_cnt <= bit_cnt - 4'd1;
                            else begin
                                bit_cnt <= 4'd0;
                                m_state <= M_ACK1;
                            end
                        end
                    end
                    M_ACK1: begin
                        if (!phase_h) begin
                            m_scl_low <= 1'b1;
                            m_sda_low <= 1'b0;
                            phase_h <= 1'b1;
                        end
                        else begin
                            m_scl_low <= 1'b0;
                            phase_h <= 1'b0;
                            m_state <= M_ACK2;
                        end
                    end
                    M_ACK2: begin
//★ 2026-11-13（D1/O2）：NACK **一律**中止。原来这门在 cfg_reg[0] 上，而 cfg 复位是 0
//  ⇒ 开箱状态从机不在也照打完全部字节（实测：错地址之后主机又空打了 9 个字节）。
                        if (sda_q1) begin
                            dpu_err   <= 1'b1;
                            m_flush <= 1'b1;
                            m_stop_clr <= 1'b1;
                            m_state <= M_STOP;
                        end
//★ 2026-11-13（D1/O3）：长度数完自动 STOP，软件不用自己数最后一条、也不用拨 stop。
                        else if (len_en && (m_len == 8'd1)) begin
                            m_stop_clr <= 1'b1;
                            m_state <= M_STOP;
                        end
                        else if (!tx_empty) begin
                            m_tx_data <= tx_buf[tx_rd[2:0]];
                            m_pop <= 1'b1;
                            bit_cnt <= 4'd8;
                            m_scl_low <= 1'b1;
                            phase_h <= 1'b0;
                            if (m_len != 8'd0)
                                m_len <= m_len - 8'd1;
                            m_state <= M_SEND;
                        end
                        else if (ctrl_stop) begin
                            m_stop_clr <= 1'b1;
                            m_state <= M_STOP;
                        end
                        else begin
                            m_done <= 1'b1;
                            m_state <= M_IDLE;
                        end
                    end
                    M_STOP: begin
                        m_scl_low <= 1'b1;
                        m_sda_low <= 1'b1;
                        phase_h <= 1'b0;
                        m_state <= M_STOP2;
                    end
                    M_STOP2: begin
                        m_scl_low <= 1'b0;
                        m_sda_low <= 1'b1;
                        phase_h <= 1'b1;
                        m_state <= M_BACK;
                    end
                    M_BACK: begin
                        m_scl_low <= 1'b0;
                        m_sda_low <= 1'b0;
                        phase_h <= 1'b1;
                        m_state <= M_IDLE;
                    end
                    default: m_state <= M_IDLE;
                    endcase
                end
            end
            else begin
                m_state <= M_IDLE;
                m_scl_low <= 1'b0;
                m_sda_low <= 1'b0;
            end
        end
    end

//从机 FSM：外部 SCL 边沿推进；ACK 在下降沿拉低、高电平期保持、下一个下降沿释放
    always @(posedge clk) begin
        if (rst) begin
            s_state <= S_IDLE;
            s_scl_low <= 1'b0;
            s_sda_low <= 1'b0;
            s_tx_data <= 8'hff;
            rx_shift <= 8'd0;
            s_rx_byte <= 8'd0;
            s_bit_cnt <= 4'd0;
            rw_bit <= 1'b0;
            slv_match <= 1'b0;
            stretch <= 1'b0;
            ack_ph <= 1'b0;
            s_pop <= 1'b0;
            s_rx_push <= 1'b0;
            s_rx_done <= 1'b0;
        end
        else begin
            s_pop <= 1'b0;
            s_rx_push <= 1'b0;
            s_rx_done <= 1'b0;
            if (mode && slv_en) begin
                if (stop_det) begin
                    s_state <= S_IDLE;
                    s_scl_low <= 1'b0;
                    s_sda_low <= 1'b0;
                    stretch <= 1'b0;
                    ack_ph <= 1'b0;
                end
                else begin
                    case (s_state)
                    S_IDLE: begin
                        s_scl_low <= 1'b0;
                        s_sda_low <= 1'b0;
                        if (start_det) begin
                            rx_shift <= 8'd0;
                            s_bit_cnt <= 4'd8;
                            s_state <= S_ADDR;
                        end
                    end
                    S_ADDR: begin
                        if (scl_rise) begin
                            rx_shift <= {rx_shift[6:0], sda_q1};
                            if (s_bit_cnt <= 4'd1) begin
                                slv_match <= ((slv_recv[7:1] ^ slv_reg[6:0]) & slv_reg[14:8]) == 7'd0;
                                rw_bit <= slv_recv[0];
                                s_bit_cnt <= 4'd0;
                                ack_ph <= 1'b0;
                                s_state <= S_ADDR_ACK;
                            end
                            else
                                s_bit_cnt <= s_bit_cnt - 4'd1;
                        end
                    end
                    S_ADDR_ACK: begin
                        if (scl_fall) begin
                            if (!ack_ph) begin
                                ack_ph <= 1'b1;
                                s_sda_low <= slv_match;
                            end
                            else begin
                                ack_ph <= 1'b0;
                                s_sda_low <= 1'b0;
                                if (slv_match) begin
                                    rx_shift <= 8'd0;
                                    s_bit_cnt <= 4'd8;
                                    if (rw_bit) begin
                                        if (!tx_empty) begin
                                            s_sda_low <= ~tx_buf[tx_rd[2:0]][7];
                                            s_tx_data <= {tx_buf[tx_rd[2:0]][6:0], 1'b0};
                                            s_pop <= 1'b1;
                                        end
                                        else begin
                                            s_sda_low <= 1'b0;
                                            s_tx_data <= 8'hff;
                                        end
                                        s_state <= S_TX;
                                    end
                                    else
                                        s_state <= S_RX;
                                end
                                else
                                    s_state <= S_DONE;
                            end
                        end
                    end
                    S_RX: begin
                        if (scl_rise) begin
                            rx_shift <= {rx_shift[6:0], sda_q1};
                            if (s_bit_cnt <= 4'd1) begin
                                s_rx_byte <= slv_recv;
                                s_bit_cnt <= 4'd0;
                                ack_ph <= 1'b0;
                                s_state <= S_RX_ACK;
                            end
                            else
                                s_bit_cnt <= s_bit_cnt - 4'd1;
                        end
                    end
                    S_RX_ACK: begin
                        if (stretch) begin
                            s_scl_low <= 1'b1;
                            if (!rx_full) begin
                                s_rx_push <= 1'b1;
                                s_rx_done <= 1'b1;
                                stretch <= 1'b0;
                                s_scl_low <= 1'b0;
                                s_sda_low <= 1'b1;
                            end
                        end
                        else if (scl_fall) begin
                            if (!ack_ph) begin
                                ack_ph <= 1'b1;
                                if (rx_full && cfg_reg[3]) begin
                                    stretch <= 1'b1;
                                    s_scl_low <= 1'b1;
                                end
                                else begin
                                    if (!rx_full) begin
                                        s_rx_push <= 1'b1;
                                        s_rx_done <= 1'b1;
                                    end
                                    s_sda_low <= !rx_full;
                                end
                            end
                            else begin
                                ack_ph <= 1'b0;
                                s_sda_low <= 1'b0;
                                rx_shift <= 8'd0;
                                s_bit_cnt <= 4'd8;
                                s_state <= S_RX;
                            end
                        end
                    end
                    S_TX: begin
                        if (scl_fall) begin
                            s_sda_low <= ~s_tx_data[7];
                            s_tx_data <= {s_tx_data[6:0], 1'b0};
                        end
                        if (scl_rise) begin
                            if (s_bit_cnt <= 4'd1) begin
                                s_bit_cnt <= 4'd0;
                                s_state <= S_TX_ACK;
                            end
                            else
                                s_bit_cnt <= s_bit_cnt - 4'd1;
                        end
                    end
                    S_TX_ACK: begin
                        if (scl_fall)
                            s_sda_low <= 1'b0;
                        if (scl_rise) begin
                            if (!sda_q1 && !tx_empty) begin
                                s_tx_data <= {tx_buf[tx_rd[2:0]][6:0], 1'b0};
                                s_pop <= 1'b1;
                                s_bit_cnt <= 4'd8;
                                s_state <= S_TX;
                            end
                            else
                                s_state <= S_DONE;
                        end
                    end
                    S_DONE: begin
                        s_scl_low <= 1'b0;
                        s_sda_low <= 1'b0;
                        if (start_det) begin
                            rx_shift <= 8'd0;
                            s_bit_cnt <= 4'd8;
                            s_state <= S_ADDR;
                        end
                    end
                    default: s_state <= S_IDLE;
                    endcase
                end
            end
            else begin
                s_state <= S_IDLE;
                s_scl_low <= 1'b0;
                s_sda_low <= 1'b0;
                stretch <= 1'b0;
                ack_ph <= 1'b0;
            end
        end
    end
endmodule
