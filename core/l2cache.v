`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: l2cache
// Description: 64KB 统一二级缓存（4 路 × 128 组 × 128B 行），单 AHB 主口。
//
//   ── 一致性契约（设计前提，改之前先读）──────────────────────────────────────
//   1. 写策略 = 【写直达 + 写失效】：
//        · 每次 D 口写都无条件进 4 深写队列，由它单拍写到内存 ⇒ 内存永远最新；
//        · 同一条写若命中在册行，该行的 valid 立刻清掉（写失效）。
//          ⇒ 阵列里永远不会有"旧数据的干净行"，读缺失时从内存重填即可。
//          ⇒ 没有脏行、不需要写回、不需要 flush。
//   2. D 口读缺失、I 口缺失都整行（128B）分配。
//   3. DMA 往内存写：硬件不侦听。约定 DMA 缓冲区落在【非缓存窗口】
//      （unc_base / unc_mask 指定的区域），窗口内访问不进阵列，读走 8 拍突发、
//      写直接进写队列 ⇒ 与 CPU 天然一致，不需要任何软件维护。
//   4. 重装代码（自举把新程序写进内存）之后，由软件写 maint=10 失效整个 L2。
//
//   ── 端口 ──────────────────────────────────────────────────────────────────
//   I 口：与 itcm 同形。req 脉冲 → 连吐 32 个字（128B）。请求寄存 1 个。
//   D 口：与 dtcm 同形。req 脉冲 + we；
//          读 → 连吐 8 个字（32B 子块），d_ready 表示能接；
//          写 → 单拍，永远接（进写队列）。
//   maint：1 拍脉冲，10 = 失效（清 valid）。
//
//   ── 结构 ──────────────────────────────────────────────────────────────────
//   · tag：4 个独立阵列（每路一个），四路并行读，只写受害路。
//   · valid：128 组 × 4 路位向量；LRU：每组 2 位（记录上次替换的路）。
//   · 数据阵列复制两份（I 读口 / D 读口各一份），填充时两份同时写。
//   · 同一时刻只处理一个事务（I / D 读 / 写队列），AHB 单请求在飞。
//////////////////////////////////////////////////////////////////////////////////


module l2cache(
    input clk, rst,

//---- I 口 ----
    input             i_req,
    input      [31:0] i_addr,
    output     [31:0] i_data,
    output            i_valid,

//---- D 口 ----
    input             d_req,
    input             d_we,
    input      [31:0] d_addr,
    input      [31:0] d_wdata,
    input      [3:0]  d_be,
    output     [31:0] d_data,
    output            d_valid,
    output            d_ready,

//---- 维护 ----
    input      [1:0]  maint,
    output reg        maint_done,
//★ 2026-11-13（fence 排空判据）：4 深写队列里没有排队项 ⇒ 已收下的缓存写全部落到后端了。
//  只喂 ROB 提交口，不进访存通路（见 docs/SPEC_fence_i_and_cache_range_2026-11-13.txt §2.3）。
    output wire       write_queue_empty,

//---- 非缓存窗口 ----
    input      [31:0] unc_base,
    input      [31:0] unc_mask,

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

    localparam integer SETS  = 128;
    localparam integer SET_B = 7;
    localparam integer WRD_B = 5;
    localparam integer AWB   = 14;
    localparam integer TAG_B = 20;

    localparam [1:0] HTRANS_IDLE = 2'b00, HTRANS_NOSEQ = 2'b10, HTRANS_SEQ = 2'b11;

    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//==============================================================================
//  一、阵列
//==============================================================================
    (* ram_style = "distributed" *) reg [TAG_B-1:0] tag0 [0:SETS-1];
    (* ram_style = "distributed" *) reg [TAG_B-1:0] tag1 [0:SETS-1];
    (* ram_style = "distributed" *) reg [TAG_B-1:0] tag2 [0:SETS-1];
    (* ram_style = "distributed" *) reg [TAG_B-1:0] tag3 [0:SETS-1];
    reg [SETS-1:0]   valid0, valid1, valid2, valid3;
    reg [2*SETS-1:0] lru;

    (* ram_style = "block" *) reg [31:0] data_i [0:(1<<AWB)-1];
    (* ram_style = "block" *) reg [31:0] data_d [0:(1<<AWB)-1];

    integer n;
    initial begin
        for (n = 0; n < SETS; n = n + 1) begin
            tag0[n] = {TAG_B{1'b0}}; tag1[n] = {TAG_B{1'b0}};
            tag2[n] = {TAG_B{1'b0}}; tag3[n] = {TAG_B{1'b0}};
        end
        for (n = 0; n < (1<<AWB); n = n + 1) begin
            data_i[n] = 32'd0; data_d[n] = 32'd0;
        end
    end

//==============================================================================
//  二、端口请求寄存
//==============================================================================
    reg        i_pend;
    reg [31:0] i_pend_addr;
    reg        d_pend;                       // D 口读请求寄存
    reg [31:0] d_pend_addr;

//写队列：4 深。
//★★ 2026-11-12：指针必须 3 位（PTR_W = $clog2(DEPTH)+1）。原来是 2 位、再零扩展去算
//  `wq_count = {1'b0,wr} - {1'b0,rd}` —— 那个数只在【永不满】时才等于占用：wr 追上 rd
//  （真满 4 条）时差值是 0，和"空"一模一样 ⇒ **wq_full 永远不成立** ⇒ 队列照单全收、
//  每 4 笔覆盖一遍，被覆盖的写静默丢失。
//  实测：S_FILL 占 32 期，期间连写 6 笔，wr 转了一圈多，最早那笔（TCB 的 pvOwner）没了
//  ⇒ FreeRTOS 第一次上下文切换 mret 到地址 0。
    reg [2:0]  wq_wr, wq_rd;
    reg [31:0] wq_addr [0:3];
    reg [31:0] wq_data [0:3];
    reg [3:0]  wq_be   [0:3];
    wire [2:0] wq_count = wq_wr - wq_rd;
    wire       wq_empty = (wq_wr == wq_rd);
    wire       wq_full  = (wq_count == 3'd4);
//★ 2026-11-13（fence 排空判据）：与内部 l2_maint_w / rob 的排空握手用同一根。纯寄存器比较。
    assign write_queue_empty = wq_empty;

//★ 2026-11-12：写队列里还有没落到内存的写时，**同一条线**的填充不许开工。
//  填充是从内存整行读回来的，抢在排队写前面就会读成旧值；而 D 写是【写失效】的
//  （见下面 dw_hit 那一段），那一行被装进来之后就是永久旧值。
//  实测症状：FreeRTOS 建任务时写 TCB 的某个字段，之后读回 0，整条就绪表 pvOwner 为 0
//  ⇒ pxCurrentTCB 变成 0 ⇒ 第一次上下文切换 mret 到地址 0。
//  ★ 只按【线】判冲突，不是一律等队列排空 —— 否则每条指令缺失都要等一笔数据写出去。
    wire [3:0] wq_hit_i = { (wq_addr[3][31:7] == i_pend_addr[31:7]),
                            (wq_addr[2][31:7] == i_pend_addr[31:7]),
                            (wq_addr[1][31:7] == i_pend_addr[31:7]),
                            (wq_addr[0][31:7] == i_pend_addr[31:7]) };
    wire [3:0] wq_hit_d = { (wq_addr[3][31:7] == d_pend_addr[31:7]),
                            (wq_addr[2][31:7] == d_pend_addr[31:7]),
                            (wq_addr[1][31:7] == d_pend_addr[31:7]),
                            (wq_addr[0][31:7] == d_pend_addr[31:7]) };

//==============================================================================
//  三、查找
//==============================================================================
    wire [SET_B-1:0] i_set = i_pend_addr[SET_B+WRD_B+1 : WRD_B+2];
    wire [TAG_B-1:0] i_tag = i_pend_addr[31 : SET_B+WRD_B+2];
    wire [SET_B-1:0] d_set = d_pend_addr[SET_B+WRD_B+1 : WRD_B+2];
    wire [TAG_B-1:0] d_tag = d_pend_addr[31 : SET_B+WRD_B+2];

    wire [3:0] i_hit = {valid3[i_set] & (tag3[i_set] == i_tag),
                        valid2[i_set] & (tag2[i_set] == i_tag),
                        valid1[i_set] & (tag1[i_set] == i_tag),
                        valid0[i_set] & (tag0[i_set] == i_tag)};
    wire [3:0] d_hit = {valid3[d_set] & (tag3[d_set] == d_tag),
                        valid2[d_set] & (tag2[d_set] == d_tag),
                        valid1[d_set] & (tag1[d_set] == d_tag),
                        valid0[d_set] & (tag0[d_set] == d_tag)};

//受害路（四路全满用 LRU）；两份各算一个，避免依赖当前事务
    wire [1:0] i_victim = (!valid0[i_set]) ? 2'd0 : (!valid1[i_set]) ? 2'd1 :
                          (!valid2[i_set]) ? 2'd2 : (!valid3[i_set]) ? 2'd3 :
                          lru[{i_set, 1'b0} +: 2];
    wire [1:0] d_victim = (!valid0[d_set]) ? 2'd0 : (!valid1[d_set]) ? 2'd1 :
                          (!valid2[d_set]) ? 2'd2 : (!valid3[d_set]) ? 2'd3 :
                          lru[{d_set, 1'b0} +: 2];

//非缓存窗口
    wire d_uncached = ((d_addr & ~unc_mask) == (unc_base & ~unc_mask));

//==============================================================================
//  四、D 口写：无条件接收（进队列） + 写失效（清命中行的 valid）
//==============================================================================
    wire              d_write_now = d_req & d_we;
    wire [SET_B-1:0]  dw_set = d_addr[SET_B+WRD_B+1 : WRD_B+2];
    wire [TAG_B-1:0]  dw_tag = d_addr[31 : SET_B+WRD_B+2];
    wire [3:0]        dw_hit = {valid3[dw_set] & (tag3[dw_set] == dw_tag),
                                valid2[dw_set] & (tag2[dw_set] == dw_tag),
                                valid1[dw_set] & (tag1[dw_set] == dw_tag),
                                valid0[dw_set] & (tag0[dw_set] == dw_tag)};

//==============================================================================
//  五、流式输出（阵列 → 寄存器，一拍延迟）
//==============================================================================
    reg [4:0]      st_word;
    reg [5:0]      st_count;
    reg [1:0]      st_way;
    reg [SET_B-1:0] st_set;
    reg            st_is_i;
    reg            st_prime;                  // 第一拍只送地址、不置 valid

    wire [AWB-1:0] read_index = {st_way, st_set, st_word};
    reg [31:0]     data_i_q, data_d_q;
    reg [31:0]     byp_data_q;                // 旁路（非缓存窗口）读回的数据
    reg            byp_sel_q;                 // 那个条件延一拍，和 data_d_q 对齐
    reg            i_valid_q, d_valid_q;

//★ 2026-11-12：这里原先是 `if (S_BYP) data_d_q <= m_hrdata; else data_d_q <= data_d[...]`，
//  数组的读输出寄存器被旁路数据分了源 ⇒ 综合报 [Synth 8-6849] Infeasible
//  ram_style="block" for "u_l2/data_d_reg"，16Kx32 整个退回 LUTRAM（实测 11964 个 LUTRAM，
//  占器件 11.8%）。改成【阵列读走纯形状 + 出口显式 mux】，拍数一点没变。
    always @(posedge clk) begin
        data_i_q   <= data_i[read_index];
        data_d_q   <= data_d[read_index];     // 纯阵列读 ⇒ 才推得出 BRAM
        byp_data_q <= m_hrdata;
        byp_sel_q  <= (state == S_BYP) && m_rd_valid;
    end

//D 口出口：旁路值优先（与原来那条 if 同拍、同优先级）
    wire [31:0] data_d_sel = byp_sel_q ? byp_data_q : data_d_q;

//==============================================================================
//  六、状态机
//==============================================================================
    localparam [2:0] S_IDLE = 3'd0, S_IS = 3'd1, S_DS = 3'd2,
                     S_FILL = 3'd3, S_WR = 3'd4, S_BYP = 3'd5;

    reg [2:0]      state;
    reg [5:0]      fill_issue, fill_recv;
    reg [31:0]     fill_base;
    reg [3:0]      byp_issue, byp_recv;
    reg [31:0]     byp_base;
    reg            cur_is_i;
    reg [1:0]      cur_way;
    reg [SET_B-1:0] cur_set;
    reg [TAG_B-1:0] cur_tag;
    reg [5:0]      cur_words;                 // 本次要吐几个字（32 / 8）
    reg [4:0]      cur_start;                 // 行内起始字

    wire [31:0] d_block_addr = {d_pend_addr[31:5], 5'd0};

//AHB 地址相位
    always @(*) begin
        m_haddr  = 32'd0;
        m_htrans = HTRANS_IDLE;
        m_hwrite = 1'b0;
        m_hsize  = 3'b010;
        m_hburst = 3'b001;
//HPROT/HMASTLOCK：按 ARM IHI 0033A §3.7 的推荐值 4'b0011；L2 不发锁传输 ⇒ 0。
        m_hprot  = 4'b0011;
        m_hmastlock = 1'b0;
        m_hwdata = 32'd0;
        m_hwstrb = 4'b0000;
        if (state == S_FILL) begin
            m_haddr  = fill_base + {fill_issue, 2'b00};
//★ 整行发完必须拉 IDLE：地址挂在总线上时从设备每拍都会当成一次新传输接受，
//  会多读一个相邻字、并让互联的"事务未结束"永远不落。
            m_htrans = (fill_issue == 6'd0)       ? HTRANS_NOSEQ :
                       (fill_issue >= 6'd32)      ? HTRANS_IDLE  : HTRANS_SEQ;
        end
        else if (state == S_BYP) begin
            m_haddr  = byp_base + {byp_issue, 2'b00};
            m_htrans = (byp_issue == 4'd0)        ? HTRANS_NOSEQ :
                       (byp_issue >= 4'd8)        ? HTRANS_IDLE  : HTRANS_SEQ;
        end
        else if (state == S_WR) begin
            m_haddr  = wq_addr[wq_rd[1:0]];
            m_htrans = HTRANS_NOSEQ;
            m_hwrite = 1'b1;
            m_hwdata = wq_data[wq_rd[1:0]];
            m_hwstrb = wq_be[wq_rd[1:0]];
        end
//未获授权时把 HTRANS 拉成 IDLE（AHB 约定）
//★ htrans 不能按 hgrant 关掉：仲裁器把 htrans 当"我有请求"看（本总线没有独立的
//  HBUSREQ 线），关掉就永远等不到授权。真正保证"没授权不发"的是 hready
//  （互联里已经算成 从设备 hready & hgrant）。
    end

//填充数据落阵列
    always @(posedge clk) begin
        if (state == S_FILL && m_rd_valid && fill_recv < 6'd32) begin
            data_i[{cur_way, cur_set, fill_recv[4:0]}] <= m_hrdata;
            data_d[{cur_way, cur_set, fill_recv[4:0]}] <= m_hrdata;
        end
    end

//tag / valid / LRU 的写【必须】和主状态机在同一个 always 里：
//  写失效（visid0[dw_set] <= 0）与填充置位会撞同一个位，分两个 always 就是双驱动。
    reg busy_flag;
    assign m_busy  = busy_flag;
    assign i_data  = data_i_q;
    assign i_valid = i_valid_q & st_is_i;
    assign d_data  = data_d_sel;      // 旁路优先，见 §201-218 的说明
    assign d_valid = d_valid_q & ~st_is_i;
//★ 写队列满时 d_ready 必须低，上游（dcache 的写穿）才举着写使能等。
//  ★ 不能写成 `d_we ? ~wq_full : ~d_pend`：d_we 是本模块的输入、又是 dcache 写使能的
//    直接来源，那样会成 d_we → d_ready → mem_write_ready → mem_write_enable → d_we 的组合环
//    （队列满时来回翻）。这里只用水位和 d_pend 两个寄存器量。
//  读那一侧不看这根（L2 收读只看 d_req/d_we/d_pend），所以队列满不影响读。
    assign d_ready = ~d_pend & ~wq_full;

    always @(posedge clk) begin
        if (reset_registered) begin
            state        <= S_IDLE;
            i_pend       <= 1'b0;   i_pend_addr <= 32'd0;
            d_pend       <= 1'b0;   d_pend_addr <= 32'd0;
            wq_wr        <= 3'd0;   wq_rd       <= 3'd0;
            st_word      <= 5'd0;   st_count    <= 6'd0;
            st_way       <= 2'd0;   st_set      <= {SET_B{1'b0}};
            st_is_i      <= 1'b0;   st_prime    <= 1'b0;
            i_valid_q    <= 1'b0;   d_valid_q   <= 1'b0;
            fill_issue   <= 6'd0;   fill_recv   <= 6'd0;  fill_base <= 32'd0;
            byp_issue    <= 4'd0;   byp_recv    <= 4'd0;  byp_base  <= 32'd0;
            cur_is_i     <= 1'b0;   cur_way     <= 2'd0;
            cur_set      <= {SET_B{1'b0}};  cur_tag <= {TAG_B{1'b0}};
            cur_words    <= 6'd32;  cur_start   <= 5'd0;
            busy_flag    <= 1'b0;
            maint_done   <= 1'b0;
            valid0 <= {SETS{1'b0}}; valid1 <= {SETS{1'b0}};
            valid2 <= {SETS{1'b0}}; valid3 <= {SETS{1'b0}};
            lru    <= {(2*SETS){1'b0}};
        end
        else begin
            i_valid_q  <= 1'b0;
            d_valid_q  <= 1'b0;
            maint_done <= 1'b0;

//---- 请求登记（永远接）----
            if (i_req && !i_pend) begin
                i_pend      <= 1'b1;
                i_pend_addr <= i_addr;
            end
            if (d_req && !d_we && !d_pend) begin
                d_pend      <= 1'b1;
                d_pend_addr <= d_addr;
            end
            if (d_write_now && !wq_full) begin
                wq_addr[wq_wr[1:0]] <= d_addr;
                wq_data[wq_wr[1:0]] <= d_wdata;
                wq_be  [wq_wr[1:0]] <= d_be;
                wq_wr          <= wq_wr + 3'd1;
            end
//写失效
            if (d_write_now && !d_uncached) begin
                if (dw_hit[0]) valid0[dw_set] <= 1'b0;
                if (dw_hit[1]) valid1[dw_set] <= 1'b0;
                if (dw_hit[2]) valid2[dw_set] <= 1'b0;
                if (dw_hit[3]) valid3[dw_set] <= 1'b0;
            end

            case (state)
//--------------------------------------------------------------- 空闲
            S_IDLE: begin
                busy_flag <= 1'b0;
                if (maint != 2'd0) begin
                    valid0 <= {SETS{1'b0}}; valid1 <= {SETS{1'b0}};
                    valid2 <= {SETS{1'b0}}; valid3 <= {SETS{1'b0}};
                    maint_done <= 1'b1;
                end
                else if ( !wq_empty
                        && ( (i_pend && !(|i_hit)            && (|wq_hit_i))
                          || (d_pend && (d_uncached | !(|d_hit)) && (|wq_hit_d)) ) ) begin
//同线冲突：先把排队写落下去，再填充（i_pend / d_pend 不清，下一轮再来）
                    state <= S_WR;
                end
                else if (i_pend) begin
//I：命中直接流，缺失先整行填充
                    i_pend   <= 1'b0;
                    cur_is_i <= 1'b1;
                    cur_set  <= i_set;  cur_tag <= i_tag;
                    cur_way  <= (|i_hit) ? ((i_hit[0]) ? 2'd0 : (i_hit[1]) ? 2'd1 : (i_hit[2]) ? 2'd2 : 2'd3)
                                         : i_victim;
                    cur_words<= 6'd32;
                    cur_start<= 5'd0;
                    if (|i_hit) begin
//命中：直接把行内起始字送进阵列（下一拍起连吐 32 个字）
//★ 这里必须用命中表达式本身，不能写 cur_way —— 同一 always 里它是非阻塞赋值，
//  读到的还是上一笔的旧值（踩过：D 口命中读到 I 口那一路）。
                        st_way   <= (i_hit[0]) ? 2'd0 : (i_hit[1]) ? 2'd1 : (i_hit[2]) ? 2'd2 : 2'd3;
                        st_set   <= i_set;
                        st_is_i  <= 1'b1;
                        st_word  <= 5'd0;
                        st_count <= 6'd32;
                        st_prime <= 1'b1;
                        state    <= S_IS;
                    end
                    else begin
                        fill_base  <= {i_tag, i_set, {WRD_B{1'b0}}, 2'b00};
                        fill_issue <= 6'd0;  fill_recv <= 6'd0;
                        state      <= S_FILL;
                    end
                end
                else if (d_pend) begin
//D 读：非缓存窗口走旁路突发；否则查表
                    d_pend   <= 1'b0;
                    cur_is_i <= 1'b0;
                    cur_words<= 6'd8;
                    cur_start<= d_pend_addr[WRD_B+1:2];
                    if (d_uncached) begin
                        byp_base  <= d_block_addr;
                        byp_issue <= 4'd0;  byp_recv <= 4'd0;
                        state     <= S_BYP;
                    end
                    else begin
                        cur_set <= d_set;  cur_tag <= d_tag;
                        cur_way <= (|d_hit) ? ((d_hit[0]) ? 2'd0 : (d_hit[1]) ? 2'd1 : (d_hit[2]) ? 2'd2 : 2'd3)
                                            : d_victim;
                        if (|d_hit) begin
//★ 同上：用命中表达式，不读 cur_* 的非阻塞旧值
                            st_way   <= (d_hit[0]) ? 2'd0 : (d_hit[1]) ? 2'd1 : (d_hit[2]) ? 2'd2 : 2'd3;
                            st_set   <= d_set;
                            st_is_i  <= 1'b0;
                            st_word  <= d_pend_addr[WRD_B+1:2];
                            st_count <= 6'd8;
                            st_prime <= 1'b1;
                            state    <= S_DS;
                        end
                        else begin
                            fill_base  <= {d_tag, d_set, {WRD_B{1'b0}}, 2'b00};
                            fill_issue <= 6'd0;  fill_recv <= 6'd0;
                            state      <= S_FILL;
                        end
                    end
                end
                else if (!wq_empty) begin
                    state <= S_WR;
                end
            end
//--------------------------------------------------------------- I 流（32 字）
            S_IS: begin
                busy_flag <= 1'b1;
                if (st_prime) begin
//第一拍：地址已经送进阵列，本拍不出 valid（BRAM 读延迟）
                    st_prime <= 1'b0;
                end
                else begin
                    i_valid_q <= 1'b1;
                    st_word   <= st_word + 5'd1;
                    if (st_count == 6'd1) state <= S_IDLE;
                    else                  st_count <= st_count - 6'd1;
                end
            end
//--------------------------------------------------------------- D 流（8 字）
            S_DS: begin
                busy_flag <= 1'b1;
                if (st_prime) begin
                    st_prime <= 1'b0;
                end
                else begin
                    d_valid_q <= 1'b1;
                    st_word   <= st_word + 5'd1;
                    if (st_count == 6'd1) state <= S_IDLE;
                    else                  st_count <= st_count - 6'd1;
                end
            end
//--------------------------------------------------------------- 填充 32 拍
            S_FILL: begin
                busy_flag <= 1'b1;
                if (fill_issue < 6'd32 && m_hready) fill_issue <= fill_issue + 6'd1;   // 未获总线授权就停在原地
                if (m_rd_valid && fill_recv < 6'd32) fill_recv <= fill_recv + 6'd1;
                if (fill_recv == 6'd32) begin
//行已到手：先落 tag / valid / LRU，再进流式
                    case (cur_way)
                    2'd0: begin tag0[cur_set] <= cur_tag; valid0[cur_set] <= 1'b1; end
                    2'd1: begin tag1[cur_set] <= cur_tag; valid1[cur_set] <= 1'b1; end
                    2'd2: begin tag2[cur_set] <= cur_tag; valid2[cur_set] <= 1'b1; end
                    default: begin tag3[cur_set] <= cur_tag; valid3[cur_set] <= 1'b1; end
                    endcase
                    lru[{cur_set, 1'b0} +: 2] <= cur_way + 2'd1;
                    fill_issue <= 6'd0;  fill_recv <= 6'd0;
                    st_way     <= cur_way;
                    st_set     <= cur_set;
                    st_is_i    <= cur_is_i;
                    st_word    <= cur_start;
                    st_prime   <= 1'b1;
                    st_count   <= cur_words;
                    state      <= cur_is_i ? S_IS : S_DS;
                end
            end
//--------------------------------------------------------------- 旁路 8 拍（非缓存读）
            S_BYP: begin
                busy_flag <= 1'b1;
                if (byp_issue < 4'd8 && m_hready) byp_issue <= byp_issue + 4'd1;
                if (m_rd_valid && byp_recv < 4'd8) begin
                    byp_recv  <= byp_recv + 4'd1;
                    d_valid_q <= 1'b1;
                end
                if (byp_recv == 4'd8) begin
                    byp_issue <= 4'd0;  byp_recv <= 4'd0;
                    state     <= S_IDLE;
                end
            end
//--------------------------------------------------------------- 写队列单拍
            S_WR: begin
                busy_flag <= 1'b1;
                if (m_hready) begin                       // 拿到总线才真的发出去
                    wq_rd <= wq_rd + 3'd1;
                    state <= S_IDLE;
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
