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
    output reg [31:0] bus_data_out,
    output reg ld_ready,
    output reg i2c_irq,

    output reg scl_oe,
    input scl_in,
    output reg sda_oe,
    input sda_in,
    output reg busy
    );

    parameter CLK_FREQ = 32'd50_000_000;

    localparam [7:0]
        R_DATA     = 8'h00,
        R_CTRL     = 8'h04,
        R_CFG      = 8'h08,
        R_SLV_ADDR = 8'h0c,
        R_STATUS   = 8'h10,
        R_IRQ_STAT = 8'h14;

    localparam [31:0]
        LOW_FREQ   = 32'd100_000,
        HIGH_FREQ  = 32'd400_000,
        ULTRA_FREQ = 32'd1_000_000;

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

    reg [7:0] tx_buf [0:7];
    reg [3:0] tx_wr, tx_rd;
    reg [7:0] rx_buf [0:7];
    reg [3:0] rx_wr, rx_rd;

    reg [7:0] m_tx_data, s_tx_data;
    reg [7:0] rx_shift;
    reg [7:0] slv_recv;
    reg [7:0] s_rx_byte;
    reg [3:0] bit_cnt;
    reg [15:0] cnt_l, cnt_h, cnt;
    reg phase_h;
    reg [2:0] m_state, s_state;
    reg scl_q1, scl_q2, sda_q1, sda_q2;
    reg scl_rise, scl_fall, start_det, stop_det;
    reg m_scl_low, m_sda_low, s_scl_low, s_sda_low;
    reg ack_ph;
    reg slv_match, rw_bit, stretch;
    reg m_pop, s_pop, s_rx_push;
    reg m_err, m_done, s_rx_done;
    reg [7:0] reg_sel;
    reg [31:0] rd_data;
    reg [31:0] status;



    wire tx_empty = (tx_wr == tx_rd);
    wire tx_full  = (tx_wr[3] != tx_rd[3]) && (tx_wr[2:0] == tx_rd[2:0]);
    wire rx_empty = (rx_wr == rx_rd);
    wire rx_full  = (rx_wr[3] != rx_rd[3]) && (rx_wr[2:0] == rx_rd[2:0]);
    wire slv_en   = slv_reg[15];

//分频：按 cfg[2:1] 给出 SCL 低/高相位长度
    always @(*) begin
        case (cfg_reg[2:1])
        2'd0: begin
            cnt_l = CLK_FREQ * 2 / (LOW_FREQ * 5);
            cnt_h = CLK_FREQ * 3 / (LOW_FREQ * 5);
        end
        2'd1: begin
            cnt_l = CLK_FREQ / HIGH_FREQ * 13 / 25;
            cnt_h = CLK_FREQ / HIGH_FREQ * 12 / 25;
        end
        2'd2: begin
            cnt_l = CLK_FREQ * 2 / (ULTRA_FREQ * 5);
            cnt_h = CLK_FREQ * 3 / (ULTRA_FREQ * 5);
        end
        default: begin
            cnt_l = 25'd8;
            cnt_h = 25'd8;
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
            if (reg_sel == R_IRQ_STAT) irq_clr = bus_data_in[3:0];
            else if (reg_sel == R_CTRL && bus_data_in[2]) irq_clr = 4'hf;
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
            if (bus_addr_in[31:24] == 8'd5 && bus_we_in) begin
                case (reg_sel)
                R_DATA: begin
                    if (bus_be_in[0] && !tx_full) begin
                        tx_buf[tx_wr[2:0]] <= bus_data_in[7:0];
                        tx_wr <= tx_wr + 4'd1;
                    end
                end
                R_CTRL: begin
                    if (bus_be_in[0]) begin
                        if (bus_data_in[0]) ctrl_start <= 1'b1;
                        if (bus_data_in[1]) ctrl_stop <= 1'b1;
                        mode <= bus_data_in[3];
                    end
                end
                R_CFG: begin
                    if (bus_be_in[0]) cfg_reg <= bus_data_in[3:0];
                end
                R_SLV_ADDR: begin
                    if (bus_be_in[1:0] == 2'b11) slv_reg <= bus_data_in[15:0];
                end
                default: ;
                endcase
            end
        end
    end

//总线读：读 DATA 弹 RX FIFO；data 与 ready 同沿
    always @(posedge clk) begin
        if (rst) begin
            rx_rd <= 4'd0;
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            if (bus_addr_in[31:24] == 8'd5 && !bus_we_in) begin
                bus_data_out <= rd_data;
                ld_ready <= 1'b1;
                if (reg_sel == R_DATA && !rx_empty) rx_rd <= rx_rd + 4'd1;
            end
        end
    end

//中断状态：单一写口，置位与写清同拍（写清赢）
    always @(posedge clk) begin
        if (rst) irq_stat <= 4'd0;
        else irq_stat <= (irq_stat | {stop_det, m_done, s_rx_done, m_err}) & ~irq_clr;
    end

//TX FIFO 读指针（主机/从机共用出口）
    always @(posedge clk) begin
        if (rst) tx_rd <= 4'd0;
        else if (m_pop || s_pop) tx_rd <= tx_rd + 4'd1;
    end

//RX FIFO 写口
    always @(posedge clk) begin
        if (rst) rx_wr <= 4'd0;
        else if (s_rx_push) rx_wr <= rx_wr + 4'd1;
    end

    always @(posedge clk) begin
        if (s_rx_push) rx_buf[rx_wr[2:0]] <= s_rx_byte;
    end

//主机 FSM：自计时推进，SCL 由 scl_oe 开漏拉低
    always @(posedge clk) begin
        if (rst) begin
            m_state <= M_IDLE;
            m_scl_low <= 1'b0;
            m_sda_low <= 1'b0;
            m_tx_data <= 8'd0;
            bit_cnt <= 4'd0;
            phase_h <= 1'b0;
            cnt <= 25'd0;
            m_pop <= 1'b0;
            m_err <= 1'b0;
            m_done <= 1'b0;
        end
        else begin
            m_pop <= 1'b0;
            m_err <= 1'b0;
            m_done <= 1'b0;
            if (cnt >= (phase_h ? cnt_h : cnt_l) - 1'b1) cnt <= 16'd0;
            else cnt <= cnt + 16'd1;
            if (mode == 1'b0) begin
                if (cnt == (phase_h ? cnt_h : cnt_l) - 1'b1) begin
                    case (m_state)
                    M_IDLE: begin
                        m_scl_low <= 1'b0;
                        m_sda_low <= 1'b0;
                        if (!tx_empty && ctrl_start) begin
                            m_tx_data <= tx_buf[tx_rd[2:0]];
                            m_pop <= 1'b1;
                            ctrl_start <= 1'b0;
                            bit_cnt <= 4'd8;
                            phase_h <= 1'b0;
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
                            if (bit_cnt > 4'd1) bit_cnt <= bit_cnt - 4'd1;
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
                        if (cfg_reg[0] && sda_q1) begin
                            m_err <= 1'b1;
                            m_state <= M_STOP;
                        end
                        else if (!tx_empty) begin
                            m_tx_data <= tx_buf[tx_rd[2:0]];
                            m_pop <= 1'b1;
                            bit_cnt <= 4'd8;
                            m_scl_low <= 1'b1;
                            phase_h <= 1'b0;
                            m_state <= M_SEND;
                        end
                        else if (ctrl_stop) begin
                            ctrl_stop <= 1'b0;
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
            bit_cnt <= 4'd0;
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
                            bit_cnt <= 4'd8;
                            s_state <= S_ADDR;
                        end
                    end
                    S_ADDR: begin
                        if (scl_rise) begin
                            rx_shift <= {rx_shift[6:0], sda_q1};
                            if (bit_cnt <= 4'd1) begin
                                slv_match <= ((slv_recv[7:1] ^ slv_reg[6:0]) & slv_reg[14:8]) == 7'd0;
                                rw_bit <= slv_recv[0];
                                bit_cnt <= 4'd0;
                                ack_ph <= 1'b0;
                                s_state <= S_ADDR_ACK;
                            end
                            else bit_cnt <= bit_cnt - 4'd1;
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
                                    bit_cnt <= 4'd8;
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
                                    else s_state <= S_RX;
                                end
                                else s_state <= S_DONE;
                            end
                        end
                    end
                    S_RX: begin
                        if (scl_rise) begin
                            rx_shift <= {rx_shift[6:0], sda_q1};
                            if (bit_cnt <= 4'd1) begin
                                s_rx_byte <= slv_recv;
                                bit_cnt <= 4'd0;
                                ack_ph <= 1'b0;
                                s_state <= S_RX_ACK;
                            end
                            else bit_cnt <= bit_cnt - 4'd1;
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
                                bit_cnt <= 4'd8;
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
                            if (bit_cnt <= 4'd1) begin
                                bit_cnt <= 4'd0;
                                s_state <= S_TX_ACK;
                            end
                            else bit_cnt <= bit_cnt - 4'd1;
                        end
                    end
                    S_TX_ACK: begin
                        if (scl_fall) s_sda_low <= 1'b0;
                        if (scl_rise) begin
                            if (!sda_q1 && !tx_empty) begin
                                s_tx_data <= {tx_buf[tx_rd[2:0]][6:0], 1'b0};
                                s_pop <= 1'b1;
                                bit_cnt <= 4'd8;
                                s_state <= S_TX;
                            end
                            else s_state <= S_DONE;
                        end
                    end
                    S_DONE: begin
                        s_scl_low <= 1'b0;
                        s_sda_low <= 1'b0;
                        if (start_det) begin
                            rx_shift <= 8'd0;
                            bit_cnt <= 4'd8;
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
