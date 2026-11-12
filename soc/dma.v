`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: dma
// Description: AHB 主设备 DMA，4 通道，寄存器配置，支持突发（INCR）。
//
//   ── 传输模型 ───────────────────────────────────────────────────────────────
//   每次搬运切成若干"块"，每块 = min(剩余字数, 突发字数)，最大 16 字（64B）：
//      ① 从源发读突发（INCR，一个地址/拍）→ 数据落 16 字缓冲
//      ② 从缓冲向目的发写突发（INCR）
//      ③ 地址按 inc_src/inc_dst 前进、长度递减，回 ①
//   突发字数由 CTRL[4:3] 选：0=1 拍（单发）、1=4 拍、2=8 拍、3=16 拍。
//   ★ AHB 单次突发上限 16 拍，所以块最大 16 字。
//   ★ 本版只搬 32 位字，LEN 必须是 4 的倍数（8/16 位留给以后）。
//
//   ── 寄存器（APB 侧，通道基址 = 0x20 × n）──────────────────────────────────
//      0x00 SRC    源地址
//      0x04 DST    目的地址
//      0x08 LEN    字节长度
//      0x0C CTRL   [0]=start(写 1 启动) [1]=inc_src [2]=inc_dst
//                  [4:3]=burst 字数选择 [5]=irq_en
//      0x10 STAT   [0]=busy [1]=done(写 1 清) [7:0]=剩余字数（低 8 位）
//
//   ── 中断 ───────────────────────────────────────────────────────────────────
//   irq = 任一通道的 (done ∧ irq_en)，是组合电平，不是"置起后等软件清"的寄存器。
//   清中断的办法就是写 STAT 清该通道的 done。
//
//   ── 一致性 ─────────────────────────────────────────────────────────────────
//   DMA 是 AHB 主设备，看不见 L2/L1。缓冲区必须落在 L2 的非缓存窗口里
//   （见 l2cache.v 头注释），否则 CPU 会读到旧数据。
//////////////////////////////////////////////////////////////////////////////////


module dma(
    input clk, rst,

//---- 寄存器口（接 APB 桥）----
    input      [7:0]  reg_addr,
    input      [31:0] reg_wdata,
    input             reg_we,
    input             reg_re,
    output reg [31:0] reg_rdata,

//---- 中断 ----
    output            irq,

//---- AHB 主口 ----
    output reg [31:0] m_haddr,
    output reg [1:0]  m_htrans,
    output reg        m_hwrite,
    output reg [2:0]  m_hsize,
    output reg [2:0]  m_hburst,
    output reg [3:0]  m_hprot,
    output reg        m_hmastlock,
    output reg [31:0] m_hwdata,
    output reg [3:0]  m_hwstrb,
    input      [31:0] m_hrdata,
    input             m_hready,
    input             m_hgrant,
    input             m_hresp,
    input             m_rd_valid,
    output            m_busy
    );

    localparam [1:0] HTRANS_IDLE = 2'b00, HTRANS_NOSEQ = 2'b10, HTRANS_SEQ = 2'b11;

//突发字数：CTRL[4:3]
    function [4:0] burst_words;
        input [1:0] sel;
        begin
            case (sel)
            2'd0: burst_words = 5'd1;
            2'd1: burst_words = 5'd4;
            2'd2: burst_words = 5'd8;
            default: burst_words = 5'd16;
            endcase
        end
    endfunction

    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//==============================================================================
//  一、寄存器组
//==============================================================================
    reg [31:0] ch_src  [0:3];
    reg [31:0] ch_dst  [0:3];
    reg [31:0] ch_len  [0:3];
    reg [31:0] ch_ctrl [0:3];
    reg [31:0] ch_stat [0:3];

//---- 中断：各通道 done ∧ irq_en 直接组合出来，不为单独一位 ----
//   ★ 不能做成"收尾时置 1、等软件去清"的寄存器：PLIC 是电平输入，进位不清就等于
//     中断永远挂着，claim 清完下一拍又被置回来（中断风暴 + 别的源永远抢不到）。
//     这里让中断源跟着 done 位走，软件写 1 清 done 就自然清掉中断；
//     四个通道互不影响（原来单根 irq 分不出是哪个通道完成的）。
    assign irq = (ch_stat[0][1] & ch_ctrl[0][5]) | (ch_stat[1][1] & ch_ctrl[1][5])
               | (ch_stat[2][1] & ch_ctrl[2][5]) | (ch_stat[3][1] & ch_ctrl[3][5]);

    wire [1:0] sel_ch  = reg_addr[6:5];
    wire [4:0] sel_reg = reg_addr[4:0];

    integer c;
    initial begin
        for (c = 0; c < 4; c = c + 1) begin
            ch_src[c] = 32'd0; ch_dst[c] = 32'd0; ch_len[c] = 32'd0;
            ch_ctrl[c] = 32'd0; ch_stat[c] = 32'd0;
        end
    end

    always @(*) begin
        case (sel_reg)
        5'h00: reg_rdata = ch_src [sel_ch];
        5'h04: reg_rdata = ch_dst [sel_ch];
        5'h08: reg_rdata = ch_len [sel_ch];
        5'h0C: reg_rdata = ch_ctrl[sel_ch];
        5'h10: reg_rdata = ch_stat[sel_ch];
        default: reg_rdata = 32'd0;
        endcase
    end

//==============================================================================
//  二、通道仲裁：编号小的优先，同一时刻只有一个通道在跑
//==============================================================================
    reg [1:0]  act;

    reg [1:0]  pick;
    reg        pick_valid;
    integer i;
    always @(*) begin
        pick       = 2'd0;
        pick_valid = 1'b0;
        for (i = 3; i >= 0; i = i - 1)
            if (ch_ctrl[i][0] && !ch_stat[i][1]) begin
                pick       = i[1:0];
                pick_valid = 1'b1;
            end
    end

//==============================================================================
//  三、传送状态机
//==============================================================================
    localparam [1:0] S_IDLE = 2'd0, S_RD = 2'd1, S_WR = 2'd2, S_FIN = 2'd3;

    reg [1:0]  state;
    reg [31:0] src_addr, dst_addr, remain;      // remain = 剩余字数
    reg [4:0]  issue_cnt, recv_cnt, wr_cnt;
    reg [31:0] burst_buf [0:15];

//本块字数：剩余与突发字数取小（remain/chunk 在整块期间不变，直接组合用）
    wire [4:0] words_left = (remain >= 32'd16) ? 5'd16 : remain[4:0];
    wire [4:0] burst_now  = burst_words(ch_ctrl[act][4:3]);
    wire [4:0] chunk_now  = (words_left > burst_now) ? burst_now : words_left;

    always @(*) begin
        m_haddr  = 32'd0;
        m_htrans = HTRANS_IDLE;
        m_hwrite = 1'b0;
        m_hsize  = 3'b010;
        m_hburst = 3'b001;                       // INCR
//HPROT/HMASTLOCK：按 ARM IHI 0033A §3.7 的推荐值 4'b0011；DMA 不发锁传输 ⇒ 0。
        m_hprot  = 4'b0011;
        m_hmastlock = 1'b0;
        m_hwdata = 32'd0;
        m_hwstrb = 4'b0000;
        if (state == S_RD) begin
            m_haddr  = src_addr + {issue_cnt, 2'b00};
//★ 本块发完就必须把 HTRANS 拉成 IDLE：地址还挂在总线上时，从设备每拍都会把它
//  当成一次新传输接受（踩过：整块发完后连续多拍重复读同一个地址，读数据
//  被冲成 0/1 交替，块长直接算错）。
            m_htrans = (issue_cnt == 5'd0)        ? HTRANS_NOSEQ :
                       (issue_cnt >= chunk_now)   ? HTRANS_IDLE  : HTRANS_SEQ;
        end
        else if (state == S_WR) begin
            m_haddr  = dst_addr + {wr_cnt, 2'b00};
            m_htrans = (wr_cnt == 5'd0)      ? HTRANS_NOSEQ :
                       (wr_cnt >= chunk_now) ? HTRANS_IDLE  : HTRANS_SEQ;
            m_hwrite = (m_htrans != HTRANS_IDLE);
            m_hwdata = burst_buf[wr_cnt[3:0]];
            m_hwstrb = 4'b1111;
        end
//未获授权时把 HTRANS 拉成 IDLE（AHB 约定：没授权就不许发地址相位）
//★ htrans 不能按 hgrant 关掉：仲裁器把 htrans 当"我有请求"看（本总线没有独立的
//  HBUSREQ 线），关掉就永远等不到授权。真正保证"没授权不发"的是 hready。
    end

    always @(posedge clk) begin
        if (state == S_RD && m_rd_valid && recv_cnt != chunk_now)
            burst_buf[recv_cnt[3:0]] <= m_hrdata;
    end

    reg busy_flag;

    always @(posedge clk) begin
        if (reset_registered) begin
            state     <= S_IDLE;
            act       <= 2'd0;
            src_addr  <= 32'd0;
            dst_addr  <= 32'd0;
            remain    <= 32'd0;
            issue_cnt <= 5'd0;
            recv_cnt  <= 5'd0;
            wr_cnt    <= 5'd0;
            busy_flag <= 1'b0;
            for (c = 0; c < 4; c = c + 1) ch_stat[c] <= 32'd0;
        end
        else begin
//---- 寄存器写 ----
            if (reg_we) begin
                case (sel_reg)
                5'h00: ch_src [sel_ch] <= reg_wdata;
                5'h04: ch_dst [sel_ch] <= reg_wdata;
                5'h08: ch_len [sel_ch] <= reg_wdata;
                5'h0C: ch_ctrl[sel_ch] <= reg_wdata;
                5'h10: if (reg_wdata[1]) ch_stat[sel_ch][1] <= 1'b0;   // 清 done
                default: ;
                endcase
            end

            case (state)
//--------------------------------------------------------------- 等启动
            S_IDLE: begin
                busy_flag <= 1'b0;
                if (pick_valid) begin
                    act       <= pick;
                    src_addr  <= ch_src[pick];
                    dst_addr  <= ch_dst[pick];
                    remain    <= ch_len[pick] >> 2;
                    issue_cnt <= 5'd0;
                    recv_cnt  <= 5'd0;
                    wr_cnt    <= 5'd0;
                    ch_stat[pick][0] <= 1'b1;              // busy
                    state     <= S_RD;
                end
            end
//--------------------------------------------------------------- 读块
            S_RD: begin
                busy_flag <= 1'b1;
                if (issue_cnt != chunk_now && m_hready) issue_cnt <= issue_cnt + 5'd1;   // 未授权就停住
                if (m_rd_valid && recv_cnt != chunk_now) recv_cnt <= recv_cnt + 5'd1;
                if (recv_cnt == chunk_now) begin
                    issue_cnt <= 5'd0;
                    recv_cnt  <= 5'd0;
                    wr_cnt    <= 5'd0;
                    state     <= S_WR;
                end
            end
//--------------------------------------------------------------- 写块
            S_WR: begin
                busy_flag <= 1'b1;
                if (m_hready) begin                       // 拿到总线才推进写拍
                wr_cnt <= wr_cnt + 5'd1;
                if (wr_cnt == chunk_now - 5'd1) begin
                    src_addr <= ch_ctrl[act][1] ? (src_addr + {chunk_now, 2'b00}) : src_addr;
                    dst_addr <= ch_ctrl[act][2] ? (dst_addr + {chunk_now, 2'b00}) : dst_addr;
                    remain   <= remain - {27'd0, chunk_now};
                    ch_stat[act][7:0] <= ch_stat[act][7:0] - {3'd0, chunk_now};
                    if (remain == {27'd0, chunk_now}) begin
                        state <= S_FIN;
                    end
                    else begin
                        issue_cnt <= 5'd0;
                        recv_cnt  <= 5'd0;
                        wr_cnt    <= 5'd0;
                        state     <= S_RD;
                    end
                end
                end
            end
//--------------------------------------------------------------- 收尾
            S_FIN: begin
                busy_flag <= 1'b0;
                ch_stat[act][0] <= 1'b0;                  // busy 清
                ch_stat[act][1] <= 1'b1;                  // done
                ch_ctrl[act][0] <= 1'b0;                  // 清 start
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

    assign m_busy = busy_flag;

endmodule
