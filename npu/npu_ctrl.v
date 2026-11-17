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
    // per-channel 量化参数（来自 npu_top 的三张小表，2026-11-17）
    input        cfg_accumulate,      // 本任务是否跨任务累加
    input        cfg_final_seg,       // 本任务是不是 K 链最后一段
    input        qc_per_channel,
    input        qc_round,
    input [7:0]  qc_zp,
    input [7:0]  qc_lo,
    input [7:0]  qc_hi,
    input [31:0] qc_bias,
    input [31:0] qc_mult,
    input [5:0]  qc_shift,
    output reg [5:0] qc_chan,       // 当前扫描行的绝对输出通道号
    // 最后一段：扫描要向 C RAM 借一拍读口取旧值（K 链累加后的再量化）
    output reg        c_scan_re,
    output reg [`NPU_CBUF_AW-1:0] c_scan_addr,
    input      [31:0] c_scan_rdata,
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

    // ---- per-channel 量化参数的取数与对齐（2026-11-17）----
    // 参数表是同步读（地址给出去，下一拍数据才回来）。扫描是每拍一个元素，
    // 而 r_idx（=输出通道组）每 4 个元素才换一次，所以这里用"记住上一拍 r_idx"
    // 的办法判断参数是否已经跟上：换行那一拍参数还没回来 ⇒ 停一拍。
    // 每个输出 Tile 只有 TM(=4) 行，所以每个 C Tile 最多多花 4 拍。
    reg [5:0] chan_base_q [0:BANK_NUM-1];   // 本块第一行的绝对通道号
    reg [1:0]  param_idx_q;      // bias_q/mult_q/shift_q 属于哪一组行
    reg [31:0] bias_q, mult_q;
    reg [5:0]  shift_q;
    reg        param_ok;

    always @(posedge clk) begin
        if (rst) begin
            param_idx_q <= 2'd0;
            bias_q      <= 32'd0;
            mult_q      <= 32'd0;
            shift_q     <= 6'd0;
        end
        else begin
            // 本拍的 qc_chan 已经送到 RAM 上，下一拍回来的就是它的参数
            param_idx_q <= r_idx;
            bias_q      <= qc_bias;
            mult_q      <= qc_mult;
            shift_q     <= qc_shift;
        end
    end

    always @(*) begin
        qc_chan  = chan_base_q[scan_bank_q] + {4'd0, r_idx};
        param_ok = (param_idx_q == r_idx);
    end
    wire qc_wait = qc_per_channel && !param_ok;

    // ---- 最后一段的"读旧值再量化"（2026-11-17）----
    // K 分成多段跑时，前面几段把 raw int32 累加进 C RAM；最后一段要
    //   out8 = requant( C_RAM[addr] + 本段acc )
    // 所以扫描在这个模式下变成两拍一个元素：第 1 拍向 C RAM 发读，
    // 第 2 拍拿到旧值、加上本段、走同一套量化式、当普通写推给 FIFO。
    // 这样量化器全仓只有一份（就在下面），FIFO 也不用加宽。
    wire two_phase = cfg_accumulate && cfg_final_seg;
    reg  scan_phase;             // 0 = 发读，1 = 收值并写出
    reg  signed [31:0] acc_in;   // 累加器输入（最后一段要把 C RAM 旧值加进来）

    always @(posedge clk) begin
        if (rst)
            scan_phase <= 1'b0;
        else if (!active_q || advance)
            scan_phase <= 1'b0;      // 每个元素从发读开始
        else if (two_phase && lane_ok && (idx_q < 5'd16) && !qc_wait)
            // ★ 必须和 c_scan_re 用同一个条件：换行那一拍 qc_wait=1，读根本没发出去，
            //   这时候把相位推到 1，第 2 拍就会拿上一行的 C 旧值去加（实测 split 结果
            //   只有一趟的量，就是因为这个）。
            scan_phase <= 1'b1;
    end

    always @(*) begin
        acc_in = $signed(acc_sel) +
                 ((two_phase && scan_phase) ? $signed(c_scan_rdata) : 32'sd0);
        c_scan_re   = active_q && two_phase && lane_ok && (idx_q < 5'd16) && !scan_phase && !qc_wait;
        c_scan_addr = c_addr[`NPU_CBUF_AW-1:0];
    end

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
        advance   = active_q && (idx_q < 5'd16) &&
                    (lane_ok ? (cwr_ready && !qc_wait && (!two_phase || scan_phase))
                             : (!two_phase || !scan_phase));
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
    // 两种契约，由 qc_per_channel 选：
    //   0（默认，向后兼容）：qshift>0 时加 2^(q-1) 四舍五入、算术右移、饱和到 int32；
    //                        qshift=0 直通。
    //   1（per-channel，2026-11-17）：
    //        out8 = clamp( ((acc + bias[ch]) * mult[ch]) >> shift[ch] + zp, lo, hi )
    //        mult 是有符号 int32；round=1 时右移前加 2^(shift-1)；结果按 int8 符号扩展。
    //        这是业界 int8 推理的标准量化式（TFLite/CMSIS-NN 同构），
    //        差别只在于我们把"定点乘子+移位"直接写成一对 (mult, shift)，
    //        由软件用同一套算法算出来，所以两侧可以逐位对上。
    reg signed [63:0] quant_t;
    reg signed [63:0] qc_prod;
    reg signed [31:0] qc_sum, qc_val, qc_clamped;

    always @(*) begin
        if (cfg_accumulate && !cfg_final_seg) begin
            // ★ K 链的中间段：必须推**原始 int32**，让写回 FIFO 做 C_RAM += 本段。
            //   这一段不能量化 —— 量化了就是把每段单独截断再相加，结果全错。
            //   （踩过：两边都量化，split 的结果正好只有一趟的量。）
            quant_word = acc_sel;
        end
        else if (qc_per_channel) begin
            qc_sum  = acc_in + $signed(bias_q);
            qc_prod = $signed(qc_sum) * $signed(mult_q);
            if (qc_round && (shift_q != 6'd0))
                qc_prod = qc_prod + (64'sd1 << (shift_q - 6'd1));
            qc_val  = (qc_prod >>> shift_q);
            qc_val  = qc_val + $signed({{24{qc_zp[7]}}, qc_zp});
            if (qc_val > $signed({{24{qc_hi[7]}}, qc_hi}))
                qc_clamped = $signed({{24{qc_hi[7]}}, qc_hi});
            else if (qc_val < $signed({{24{qc_lo[7]}}, qc_lo}))
                qc_clamped = $signed({{24{qc_lo[7]}}, qc_lo});
            else
                qc_clamped = qc_val;
            quant_word = {{24{qc_clamped[7]}}, qc_clamped[7:0]};
        end
        else if (lb_qs == 5'd0) begin
            quant_word = acc_in;
        end
        else begin
            quant_t = $signed(acc_in);
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
                chan_base_q[b] <= 6'd0;
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
                // 本块第一个输出通道的绝对编号：tile_i*TM（此时 tile 计数器还没推进）
                chan_base_q[cap_bank] <= tile_i * lp_tm;
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
        // ★ 必须同时被 qc_wait 挡住，否则换行那一拍会带着上一行的量化参数发出去；
        //   vquant 最初每个通道参数都一样，测不出来 —— 改成真 per-channel 才会暴露。
        // ★ 两拍模式（最后一段）只在第 2 拍才发。
        cwr_valid_w = active_q && lane_ok && cwr_ready && !qc_wait &&
                      (!two_phase || scan_phase);
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
