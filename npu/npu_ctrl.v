`timescale 1ns / 1ps
// tile_controller —— Tile 调度、地址生成、搬运、C 写回（单模块版，2026-11-17）
//
// 原来分开的三个模块并进来了：systolic_fsm（主时序状态机）、c_tile_write_ctrl
// （C Tile 快照 + 量化 + 写回扫描）、result_quantizer（量化）。它们都只有本模块
// 一个使用者，端口就是内部连线。
//
// 本模块不保存矩阵数据，只负责把一次 GEMM 拆成 Tile 并把硬件推动起来：
//   参数使用 -> Tile 选择 -> A/BT 预取 -> 阵列喂数 -> 最后一个 K Tile 后排空一次
//   -> 结果快照与 C 写回（与下一个输出 Tile 的计算并行）
//
// 三条关键设计（决定了性能，改动前先读这里）：
//   1. K Tile 之间不清累加器、不排空阵列。阵列在预取期间整体冻结，整条 skew
//      管线一起保持偏移，所以跨 K Tile 的部分和留在 PE 累加器里连续累加。
//      TK 因此只是延迟/面积参数，不再是"每个 K Tile 付一次排空+搬回"的性能开销。
//   2. 预取与喂数重叠：一个输出 Tile 的 A/BT 数据在地址上是连续的一整段
//      （A 在 tile_i*K + [0,K)，BT 在 tile_j*K + [0,K)），所以换输出 Tile 时
//      一次发出整段 K 字的连续突发，喂数期间 FIFO 边填边被消费。
//   3. 结果不回阵列外累加：排空时把 16 个累加器整块快照给写回逻辑，
//      写回在后台完成，状态机直接进入下一个 Tile。快照是双 bank 的，状态机只等
//      "有空闲 bank"（snap_ready）而不等写回排空；core_done 等所有 C 真正落 RAM。
//
// 数据布局：A/BT 按 K 主序，第 (tile_i*K + tile_k*TK + k) 个字装第
// tile_i*4..tile_i*4+3 行（BT 为列）在第 k 步的 4 个 INT8。地址单位是 32 位字。
`include "npu_defines.vh"

module tile_controller(
    input        clk,
    input        rst,
    // 来自 npu_top：启动脉冲 + 锁存后的任务参数
    input        start_pulse,
    input [`NPU_DIM_W-1:0]  cfg_m, cfg_n, cfg_k,
    input [`NPU_TK_W-1:0]   cfg_tk,
    input [`NPU_QS_W-1:0]   cfg_qshift,
    // 预取 FIFO 接口
    input [31:0] a_fifo_rdata, bt_fifo_rdata,
    input [`NPU_FIFO_AW:0] a_fifo_count, bt_fifo_count,
    input [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_flat_in,
    input        drain_done,
    input        cwr_ready, cwr_empty, c_wr_pulse,
    // A/BT RAM 读请求（npu_buffer 内部同步读，数据下一次返回）
    output reg [`NPU_ABUF_AW-1:0] a_addr,
    output reg   a_re,
    output reg [`NPU_BBUF_AW-1:0] bt_addr,
    output reg   bt_re,
    output reg   fifo_pair_pop,
    // 到 npu_mac
    output reg   acc_clear,
    output reg   array_enable,
    output reg   drain_en,
    output reg   stream_valid,
    output reg [31:0] a_stream_data, bt_stream_data,
    output reg [3:0]  a_lane_en, bt_lane_en,
    // C 写回（到 npu_buffer 的写回 FIFO）
    output reg   cwr_valid,
    output reg [`NPU_CBUF_AW-1:0] cwr_addr,
    output reg [31:0] cwr_data,
    // 状态
    output reg   core_busy,
    output reg   core_done
);

    integer b;

    // ---- 任务参数（start_ctrl 已经锁存，这里直接复用同一份快照）----
    reg [`NPU_DIM_W-1:0] lb_m, lb_n, lb_k;
    reg [`NPU_TK_W-1:0] lb_tk;
    reg [`NPU_QS_W-1:0] lb_qs;
    reg [`NPU_TILE_W-1:0] lp_tm, lp_tn;
    always @(*) begin
        lb_m  = cfg_m;
        lb_n  = cfg_n;
        lb_k  = cfg_k;
        lb_tk = cfg_tk;
        lb_qs = cfg_qshift;
        lp_tm = `NPU_TM_TN;
        lp_tn = `NPU_TM_TN;
    end

    //====================================================================
    //  一、Tile 计数器
    //====================================================================
    reg [`NPU_TIDX_W-1:0] tile_i, tile_j, tile_k;
    reg sched_init, next_k_req, next_out_req;   // 由下面内联的状态机产生

    // ★ 上限与 K 余量改成【任务开始时算一次、之后每拍只做加减】。
    //   原来 nt_k = (lb_k + lb_tk - 1) / lb_tk 是变除数除法，综合成 63 级 CARRY4，
    //   它一路串进 last_k_tile -> fsm -> tile_j，是一条 29.474ns 的路径。
    reg [`NPU_TIDX_W-1:0] nt_i_l, nt_j_l;   // 各方向上界 - 1
    reg [`NPU_DIM_W-1:0]  rem_k_l;          // 剩余 K（含当前这个 K Tile）

    reg [`NPU_DIM_W-1:0] rem_m, rem_n;
    always @(*) begin
        rem_m = lb_m - tile_i * lp_tm;
        rem_n = lb_n - tile_j * lp_tn;
    end

    reg last_k_tile, last_out_tile;
    reg [`NPU_TILE_W-1:0] valid_tm, valid_tn;
    reg [`NPU_TK_W-1:0]   valid_tk;

    always @(posedge clk) begin
        if (rst) begin
            tile_i  <= {`NPU_TIDX_W{1'b0}};
            tile_j  <= {`NPU_TIDX_W{1'b0}};
            tile_k  <= {`NPU_TIDX_W{1'b0}};
            nt_i_l  <= {`NPU_TIDX_W{1'b0}};
            nt_j_l  <= {`NPU_TIDX_W{1'b0}};
            rem_k_l <= {`NPU_DIM_W{1'b0}};
        end
        else if (sched_init) begin
            tile_i  <= {`NPU_TIDX_W{1'b0}};
            tile_j  <= {`NPU_TIDX_W{1'b0}};
            tile_k  <= {`NPU_TIDX_W{1'b0}};
            nt_i_l  <= (lb_m + lp_tm - 6'd1) / lp_tm - 6'd1;
            nt_j_l  <= (lb_n + lp_tn - 6'd1) / lp_tn - 6'd1;
            rem_k_l <= lb_k;
        end
        else if (next_k_req) begin
            tile_k  <= tile_k + 6'd1;
            rem_k_l <= rem_k_l - {1'b0, lb_tk};
        end
        else if (next_out_req) begin
            tile_k  <= {`NPU_TIDX_W{1'b0}};
            rem_k_l <= lb_k;
            if (tile_j == nt_j_l) begin
                tile_j <= {`NPU_TIDX_W{1'b0}};
                tile_i <= tile_i + 6'd1;
            end
            else begin
                tile_j <= tile_j + 6'd1;
            end
        end
    end

    always @(*) begin
        last_k_tile   = (rem_k_l <= {1'b0, lb_tk});
        last_out_tile = (tile_i == nt_i_l) && (tile_j == nt_j_l);
        valid_tm      = (rem_m > {3'd0, lp_tm}) ? lp_tm : rem_m[`NPU_TILE_W-1:0];
        valid_tn      = (rem_n > {3'd0, lp_tn}) ? lp_tn : rem_n[`NPU_TILE_W-1:0];
        valid_tk      = (rem_k_l > {1'b0, lb_tk}) ? lb_tk : rem_k_l[`NPU_TK_W-1:0];
    end

    //====================================================================
    //  二、预取基地址（内部字地址，单位是 32 位字）
    //  第 tile_i 组 A 数据是 tile_i*K 起的连续 K 个字，第 tile_j 组 BT 数据是
    //  tile_j*K 起的连续 K 个字；一个输出 Tile 只需要这一整段。
    //  合法配置保证 tile_i*K + K <= ceil(M/4)*K <= 64，不会越过 Buffer 末尾。
    //====================================================================
    reg [`NPU_DIM_W+`NPU_DIM_W-1:0] a_tile_off, bt_tile_off;
    always @(*) begin
        a_tile_off  = tile_i * lb_k;
        bt_tile_off = tile_j * lb_k;
    end

    reg [`NPU_ABUF_AW-1:0] a_base;
    reg [`NPU_BBUF_AW-1:0] bt_base;
    always @(*) begin
        a_base  = a_tile_off[`NPU_ABUF_AW-1:0];
        bt_base = bt_tile_off[`NPU_BBUF_AW-1:0];
    end

    // ---- 下一个输出 Tile 的预测基址 ----
    // 排空期间状态机就启动下一块的预取（排空 6 拍正好遮住突发首字延迟），但那时
    // tile 计数器还没推进，所以这里组合预测：换输出 Tile 只是 tile_j 前进
    // （末列则回 0 且 tile_i 前进），而 A 只跟 tile_i 走、BT 只跟 tile_j 走。
    reg next_row;
    reg [`NPU_TIDX_W-1:0] next_i, next_j;
    reg [`NPU_DIM_W+`NPU_DIM_W-1:0] a_tile_off_next, bt_tile_off_next;
    reg [`NPU_ABUF_AW-1:0] a_base_next;
    reg [`NPU_BBUF_AW-1:0] bt_base_next;
    reg [`NPU_CBASE_W-1:0] c_base;
    always @(*) begin
        next_row         = (tile_j == nt_j_l);
        c_base           = (tile_i * lp_tm) * lb_n + tile_j * lp_tn;
        next_j           = next_row ? {`NPU_TIDX_W{1'b0}} : (tile_j + 6'd1);
        next_i           = next_row ? (tile_i + 6'd1) : tile_i;
        bt_tile_off_next = next_j * lb_k;
        a_tile_off_next  = next_i * lb_k;
        bt_base_next     = bt_tile_off_next[`NPU_BBUF_AW-1:0];
        a_base_next      = a_tile_off_next[`NPU_ABUF_AW-1:0];
    end

    //====================================================================
    //  三、C Tile 快照 -> 量化 -> 写回扫描（原 c_tile_write_ctrl + result_quantizer）
    //
    //  快照 bank 数量：2 组就够。扫描 16 拍，输出 Tile 的最短周期约 11 拍，
    //  两组 bank 能吸收"写回跟不上计算"时的积压；两组都忙时 snap_ready 拉低，
    //  状态机在 S_PREFETCH 等，不会丢结果。
    //  正确性依赖的接口假设：状态机必须在 S_PREFETCH 检查 snap_ready；
    //  bank 只会因扫描完成由忙变闲，所以"此刻有空闲 bank"⇒"下一次快照到达时仍有
    //  空闲 bank"，捕获不会丢。
    //  边界 Tile 中 r>=valid_tm 或 c>=valid_tn 的 lane 不入队，但 idx_q 照常前进，
    //  所以边界不影响扫描的时序长度，只影响入队项数。
    //====================================================================
    localparam BANK_NUM = 2;

    reg [`NPU_PE_NUM*`NPU_ACC_W-1:0] snap_q     [0:BANK_NUM-1];
    reg [`NPU_CBASE_W-1:0]           c_base_q   [0:BANK_NUM-1];
    reg [`NPU_DIM_W-1:0]             n_dim_q    [0:BANK_NUM-1];
    reg [`NPU_TILE_W-1:0]            valid_tm_q [0:BANK_NUM-1];
    reg [`NPU_TILE_W-1:0]            valid_tn_q [0:BANK_NUM-1];
    reg [BANK_NUM-1:0]               bank_busy_q;   // 已锁存结果、还没扫完

    reg [4:0]  idx_q;            // 0..15 扫描，16 表示扫描完成
    reg        active_q;         // 扫描进行中
    reg        snap_level_q;     // snap_valid_in 的上一拍值，用于上升沿检测
    reg        scan_bank_q;      // 正在扫描的 bank 号

    reg        cap_fire;         // 本拍锁存一个新快照
    reg        cap_bank;         // 新快照写入的 bank（优先 bank0）
    reg        any_free;         // 存在空闲 bank
    reg        scan_start;       // 本拍启动一次扫描
    reg        scan_bank_sel;    // 本次扫描选中的 bank
    reg        lane_ok;          // 当前扫描项落在有效边界内
    reg [1:0]  r_idx;
    reg [1:0]  c_idx;
    reg        advance;          // 本拍扫描前进一项
    reg        scan_done;        // 本拍扫描完最后一项
    reg [BANK_NUM-1:0] bank_set; // 本拍置忙的 bank
    reg [BANK_NUM-1:0] bank_clr; // 本拍置闲的 bank
    reg [BANK_NUM-1:0] bank_busy_d;

    reg [`NPU_ACC_W-1:0]  acc_sel;
    reg [`NPU_CBASE_W+1:0] c_addr;
    reg [31:0] quant_word;

    reg cwr_valid_w, cwr_idle;
    reg [`NPU_CBUF_AW-1:0] cwr_addr_w;
    reg [31:0] cwr_data_w;
    reg snap_ready;

    always @(*) begin
        // 赋值顺序 = 数据依赖顺序。bank_clr/bank_busy_d 依赖 scan_done，
        // 所以扫描数据通路必须写在它们前面：写反了会让 bank 清位晚一拍，扫描结束后
        // 出现"active_q=0 但 bank 仍然忙"的空档，scan_start 会再扫一遍同一块
        // （非累加模式下重复写同值是幂等的，测不出来；累加模式会多加一次结果）。
        r_idx   = idx_q[3:2];
        c_idx   = idx_q[1:0];
        lane_ok = (idx_q < 5'd16) &&
                  ({1'b0, r_idx} < valid_tm_q[scan_bank_q]) &&
                  ({1'b0, c_idx} < valid_tn_q[scan_bank_q]);
        advance   = active_q && (idx_q < 5'd16) && (lane_ok ? cwr_ready : 1'b1);
        scan_done = advance && (idx_q == 5'd15);

        any_free      = (bank_busy_q != {BANK_NUM{1'b1}});
        cap_bank      = bank_busy_q[0] ? 1'b1 : 1'b0;
        cap_fire      = drain_done && !snap_level_q && any_free;
        scan_bank_sel = bank_busy_q[0] ? 1'b0 : 1'b1;
        scan_start    = !active_q && (bank_busy_q != {BANK_NUM{1'b0}});

        bank_set    = cap_fire  ? ({{(BANK_NUM-1){1'b0}}, 1'b1} << cap_bank)  : {BANK_NUM{1'b0}};
        bank_clr    = scan_done ? ({{(BANK_NUM-1){1'b0}}, 1'b1} << scan_bank_q) : {BANK_NUM{1'b0}};
        bank_busy_d = (bank_busy_q | bank_set) & ~bank_clr;

        acc_sel = snap_q[scan_bank_q][idx_q[3:0]*`NPU_ACC_W +: `NPU_ACC_W];
        c_addr  = c_base_q[scan_bank_q] + r_idx * n_dim_q[scan_bank_q] + c_idx;
    end

    // ---- 量化（原 result_quantizer）----
    // qshift=0 时直通；qshift>0 时先加 2^(qshift-1) 做四舍五入，再算术右移，
    // 最后饱和到有符号 32 位范围。64 位中间量为舍入加法和负数右移提供符号位。
    reg signed [63:0] quant_t;
    always @(*) begin
        if (lb_qs == 5'd0) begin
            quant_word = acc_sel;
        end
        else begin
            quant_t = $signed(acc_sel);
            quant_t = (quant_t + (64'sd1 << (lb_qs - 1))) >>> lb_qs;
            if (quant_t > 64'sd2147483647)
                quant_word = 32'h7FFF_FFFF;
            else if (quant_t < -64'sd2147483648)
                quant_word = 32'h8000_0000;
            else
                quant_word = quant_t[31:0];
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            idx_q        <= 5'd0;
            active_q     <= 1'b0;
            snap_level_q <= 1'b0;
            scan_bank_q  <= 1'b0;
            bank_busy_q  <= {BANK_NUM{1'b0}};
            for (b = 0; b < BANK_NUM; b = b + 1) begin
                snap_q[b]     <= {`NPU_PE_NUM*`NPU_ACC_W{1'b0}};
                c_base_q[b]   <= {`NPU_CBASE_W{1'b0}};
                n_dim_q[b]    <= {`NPU_DIM_W{1'b0}};
                valid_tm_q[b] <= {`NPU_TILE_W{1'b0}};
                valid_tn_q[b] <= {`NPU_TILE_W{1'b0}};
            end
        end
        else begin
            snap_level_q <= drain_done;
            bank_busy_q  <= bank_busy_d;

            // 捕获：上升沿 + 有空闲 bank（接口假设保证此时一定有）
            if (cap_fire) begin
                snap_q[cap_bank]     <= acc_flat_in;
                c_base_q[cap_bank]   <= c_base;
                n_dim_q[cap_bank]    <= lb_n;
                valid_tm_q[cap_bank] <= valid_tm;
                valid_tn_q[cap_bank] <= valid_tn;
            end

            if (scan_start) begin
                active_q    <= 1'b1;
                scan_bank_q <= scan_bank_sel;
                idx_q       <= 5'd0;
            end
            else if (advance) begin
                idx_q <= idx_q + 5'd1;
                if (idx_q == 5'd15)
                    active_q <= 1'b0;   // 最后一项已送出，本 bank 在下拍置闲
            end
        end
    end

    always @(*) begin
        // cwr_valid 与 cwr_ready 相与，表示本拍一定能完成一次 FIFO 入队。
        cwr_valid_w = active_q && lane_ok && cwr_ready;
        cwr_addr_w  = c_addr[`NPU_CBUF_AW-1:0];
        cwr_data_w  = quant_word;
        // 只有"没有扫描在跑、没有 bank 待扫、写 FIFO 也空了"才算真正写完。
        cwr_idle    = !active_q && (bank_busy_q == {BANK_NUM{1'b0}}) && cwr_empty;
        snap_ready  = any_free;
    end

    //====================================================================
    //  四、主时序状态机（原 systolic_fsm）
    //
    //  状态与转移（每个输出 Tile 走一遍 K 循环，只在 Tile 结束时排空一次）：
    //    S_IDLE       start_pulse 到来 -> S_INIT
    //    S_INIT       复位 tile 计数器、启动第一个 tile 的整段预取 -> S_PREFETCH
    //    S_PREFETCH   等 A/BT FIFO 各出现至少 1 个字且有空闲快照 bank -> S_FEED
    //    S_FEED       成对喂 valid_tk 拍；喂完一个 K Tile 后
    //                 最后一个 K Tile -> S_DRAIN，否则留在 S_FEED 直接进下一个
    //    S_DRAIN      保持阵列推进 P+Q-2 拍让波前走完；在 drain_done 那一拍清 PE
    //                 累加器、推进 tile 并启动下一块的整段预取
    //                 -> 最后一个输出 Tile ? S_DONE : S_PREFETCH
    //    S_DONE       等 core_done（最后一块 C 已落入 RAM）-> S_IDLE
    //
    //  关键时序约定：
    //   1. K Tile 之间不清累加器、不排空阵列；喂数途中 FIFO 暂时没数就原地等
    //      （array_enable 保持 1，valid=0 的空拍只会在管线里平移，不会累加）。
    //   2. 第一个 tile 由 pf_start 启动；之后每个 tile 在**上一个 tile 的排空期间**
    //      由 pf_next 启动（那时 tile 计数器还没推进，所以用预测的下一块基址）。
    //   3. 状态机只等"有空闲 bank"，不等扫描结束；core_done 才代表全部 C 已落 RAM。
    //====================================================================
    localparam [2:0]
        S_IDLE     = 3'd0,
        S_INIT     = 3'd1,
        S_PREFETCH = 3'd2,
        S_FEED     = 3'd3,
        S_DRAIN    = 3'd4,
        S_DONE     = 3'd5;

    reg [2:0] state;
    reg [5:0] feed_cnt;      // 已送入的 A/BT 数据对数
    reg       pf_done_q;     // 本任务第一个 tile 的整段预取是否已发出

    reg pair_fire, stream_start;
    reg feed_go, pf_start, pf_next, fsm_done;
    reg fsm_acc_clear, fsm_drain_en, fsm_array_enable, fsm_core_busy;

    // 一对 A/BT 数据是否就绪：两个 FIFO 都非空就可以开喂。整段突发会让 FIFO
    // 边填边被消费，早开喂能把突发的首字延迟和喂数重叠，所以不等满一个 K Tile。
    reg pair_ready, feed_beat_done, is_beat_done;
    always @(*) begin
        pair_ready     = (a_fifo_count != 0) && (bt_fifo_count != 0);
        feed_beat_done = pair_fire && (feed_cnt + 6'd1 >= {1'b0, valid_tk});
        is_beat_done   = (state == S_FEED) && feed_beat_done;
    end

    // drain_en 由状态组合给出：寄存输出会晚一拍，实测每块 tile 多等 1 拍。
    // next_k_req 必须是组合脉冲（完成当拍推进 tile_k 计数器），不能改成寄存：
    // K Tile 背靠背喂数时（例如 TK=1），寄存脉冲会让 tile_k 晚一拍更新，于是
    // 下一拍读到的 last_k_tile 还是旧值，最后一个 K Tile 会被漏判、tile_k 冲过
    // 边界后卡在 S_FEED。
    always @(*) begin
        fsm_core_busy   = (state != S_IDLE);
        fsm_done        = (state == S_DONE);
        fsm_drain_en    = (state == S_DRAIN);
        fsm_array_enable = feed_go || fsm_drain_en;
        next_k_req      = is_beat_done && !last_k_tile;
    end

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            sched_init   <= 1'b0;
            next_out_req <= 1'b0;
            fsm_acc_clear <= 1'b0;
            pf_start     <= 1'b0;
            pf_next      <= 1'b0;
            feed_go      <= 1'b0;
            feed_cnt     <= 6'd0;
            pf_done_q    <= 1'b0;
        end
        else begin
            // 单拍脉冲默认清零；电平信号在各状态分支中显式拉高。
            // acc_clear 也在这里清零：它是"换输出 Tile 时的电平"，只在排空结束
            // 那一拍重新拉高，否则会一直保持高把累加器每拍清零。
            sched_init    <= 1'b0;
            next_out_req  <= 1'b0;
            fsm_acc_clear <= 1'b0;
            pf_start      <= 1'b0;
            pf_next       <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start_pulse)
                        state <= S_INIT;
                end

                S_INIT: begin
                    // 任务开始时必须清一次累加器：第一个输出 Tile 前面没有排空，
                    // 不在这里清就会带上一个任务的残留值。
                    sched_init    <= 1'b1;
                    feed_cnt      <= 6'd0;
                    pf_done_q     <= 1'b0;
                    fsm_acc_clear <= 1'b1;
                    state         <= S_PREFETCH;
                end

                // 整段预取在上一个 tile 的排空期间就发出了，这里通常一拍通过。
                // 第一个 tile 例外：它前面没有排空，预取要在这里发，而且必须等
                // sched_init 把 tile 计数器归零之后——预取控制器是在 start_in 那一拍
                // 的时钟沿采样基址的，同一拍清零会采到上一个任务残留的基址。
                S_PREFETCH: begin
                    feed_cnt <= 6'd0;
                    if (!pf_done_q) begin
                        pf_start  <= 1'b1;
                        pf_done_q <= 1'b1;
                    end
                    if (pair_ready && snap_ready) begin
                        state <= S_FEED;
                    end
                end

                S_FEED: begin
                    feed_go <= 1'b1;
                    if (pair_fire) begin
                        feed_cnt <= feed_cnt + 6'd1;
                        if (feed_beat_done) begin
                            if (last_k_tile) begin
                                // 最后一个 K Tile 喂完：立刻启动下一块的整段预取。
                                // 放在这里（排空第一拍）而不是排空结束那一拍，
                                // 是为了让排空的 7 拍把突发的首字延迟完全遮住。
                                if (!last_out_tile)
                                    pf_next <= 1'b1;
                                feed_go <= 1'b0;
                                state   <= S_DRAIN;
                            end
                            else begin
                                // 直接留在 S_FEED 进下一个 K Tile；万一数据没到齐，
                                // pair_fire 自然为 0，在这里等即可。
                                feed_cnt <= 6'd0;
                            end
                        end
                    end
                end

                // 只有在所有 K Tile 都喂完后才排空一次，结果才完整。
                // drain_done 那一拍：本块最后一批乘积已经落到最远端 PE，
                // 于是同一拍完成四件事——采数给写回逻辑、acc_clear 置起、
                // 推进 tile 计数器、启动下一段整段预取。
                S_DRAIN: begin
                    if (drain_done) begin
                        fsm_acc_clear <= 1'b1;
                        if (last_out_tile) begin
                            state <= S_DONE;
                        end
                        else begin
                            next_out_req <= 1'b1;
                            state        <= S_PREFETCH;
                        end
                    end
                end

                S_DONE: begin
                    if (core_done)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    //====================================================================
    //  五、A/BT 预取突发（npu_stream_ctrl 是共用小件，仍在 npu_common.v）
    //====================================================================
    reg [`NPU_ABUF_AW-1:0] a_pf_base;
    reg [`NPU_BBUF_AW-1:0] bt_pf_base;
    reg [`NPU_DIM_W-1:0]   pf_len;

    wire [`NPU_ABUF_AW-1:0] a_rd_addr_w;
    wire [`NPU_BBUF_AW-1:0] bt_rd_addr_w;
    wire a_rd_re_w, bt_rd_re_w;

    npu_stream_ctrl #(.AW(`NPU_ABUF_AW)) u_a_stream_ctrl(
        .clk(clk), .rst(rst),
        .base_in(a_pf_base), .len_in(pf_len), .start_in(stream_start),
        .rd_addr_out(a_rd_addr_w), .rd_re_out(a_rd_re_w)
    );

    npu_stream_ctrl #(.AW(`NPU_BBUF_AW)) u_bt_stream_ctrl(
        .clk(clk), .rst(rst),
        .base_in(bt_pf_base), .len_in(pf_len), .start_in(stream_start),
        .rd_addr_out(bt_rd_addr_w), .rd_re_out(bt_rd_re_w)
    );

    //====================================================================
    //  六、完成计数：只有真正写进 C RAM 的笔数才算完成
    //====================================================================
    localparam TOTAL_W = 12;
    reg [TOTAL_W-1:0] total_w;
    always @(*) begin
        total_w = lb_m * lb_n;      // 配置校验保证 <= NPU_CBUF_WORDS
    end
    reg [TOTAL_W-1:0] wr_cnt;
    reg done_seen;
    reg core_done_w;
    reg [11:0] wr_cnt_next;

    always @(posedge clk) begin
        if (rst) begin
            wr_cnt      <= {TOTAL_W{1'b0}};
            done_seen   <= 1'b0;
            core_done_w <= 1'b0;
        end
        else begin
            core_done_w <= 1'b0;
            if (start_pulse) begin
                wr_cnt    <= {TOTAL_W{1'b0}};
                done_seen <= 1'b0;
            end
            else begin
                wr_cnt <= wr_cnt_next;
                // 全部 C 元素都写进 RAM 且写回通路空闲，才算任务完成。
                if (!done_seen && fsm_done && (wr_cnt_next >= total_w) && cwr_idle) begin
                    core_done_w <= 1'b1;
                    done_seen   <= 1'b1;
                end
            end
        end
    end

    //====================================================================
    //  七、输出
    //====================================================================
    always @(*) begin
        pair_fire     = feed_go && (a_fifo_count != 0) && (bt_fifo_count != 0);
        fifo_pair_pop = pair_fire;

        // 第一个 tile 由 pf_start 启动（前面没有排空可遮），之后每个 tile
        // 由 pf_next 在上一块的排空期间启动，基址用上面预测的下一块值。
        stream_start = pf_start || pf_next;
        a_pf_base    = pf_next ? a_base_next  : a_base;
        bt_pf_base   = pf_next ? bt_base_next : bt_base;
        pf_len       = lb_k;

        wr_cnt_next = wr_cnt + {{(11){1'b0}}, c_wr_pulse};

        a_addr         = a_rd_addr_w;
        a_re           = a_rd_re_w;
        bt_addr        = bt_rd_addr_w;
        bt_re          = bt_rd_re_w;
        acc_clear      = fsm_acc_clear;
        drain_en       = fsm_drain_en;
        array_enable   = fsm_array_enable;
        core_busy      = fsm_core_busy;
        stream_valid   = pair_fire;
        a_stream_data  = a_fifo_rdata;
        bt_stream_data = bt_fifo_rdata;
        cwr_valid      = cwr_valid_w;
        cwr_addr       = cwr_addr_w;
        cwr_data       = cwr_data_w;
        core_done      = core_done_w;

        // 边界 Tile：valid_tm 行 / valid_tn 列以内的 lane 才有效，
        // 越界 lane 在 npu_mac 侧被补零，越界的 C 元素不会被写回。
        a_lane_en  = {4{1'b0}};
        bt_lane_en = {4{1'b0}};
        for (b = 0; b < `NPU_P; b = b + 1)
            a_lane_en[b] = (valid_tm > b);
        for (b = 0; b < `NPU_Q; b = b + 1)
            bt_lane_en[b] = (valid_tn > b);
    end

endmodule
