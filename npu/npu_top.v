`timescale 1ns / 1ps
// npu_top —— CPU 控制平面（单模块版，2026-11-17）
//
// 原来分开的三个模块并进来了：control_regs（配置寄存器）、start_ctrl（任务生命周期
// 与启动校验）、buffer_access_ctrl（Buffer 访问仲裁与读数据复用）。它们都只有本模块
// 一个使用者，端口就是内部连线。
//
// 本模块把上游（ahb_lite_slave）给出的原生请求拆成四个方向：
//   1. 控制寄存器：保存 M/N/K、TK、量化参数和 accumulate；
//   2. 状态寄存器：返回 BUSY、DONE、BUFFER_READY 和 ERROR；
//   3. Buffer 端口：把 0x1000/0x2000/0x3000/0x3400/0x4000 地址窗口转换成 RAM 操作；
//   4. 搬运引擎 / bank 观测窗：0x0040-0x005F / 0x0060-0x007F。
// 地址译码结果（sel_*）和读数据回送给 ahb_lite_slave，由它组织 AHB 的数据阶段和
// 错误响应。
//
// 请求接口（req_addr/req_wdata/req_byte_en/req_we/req_valid）与已删除的 mmio_if
// 输出定义完全一致：req_valid 是数据阶段单拍脉冲，req_addr 与它同拍有效；
// 寄存器读数据同拍组合有效，Buffer 读数据在下一拍返回（见 npu_ram）。
//
// 控制寄存器（偏移 = addr[4:2]）：
//   0x00 CTRL    bit0=1 启动一次任务；bit1=1 清除 DONE/ERROR 状态
//   0x04 M       矩阵行数 1..63
//   0x08 N       矩阵列数 1..63
//   0x0C K       内积长度 1..63
//   0x10 ACC_CFG bit0 跨任务累加：1 时本次任务第一个输出 Tile 不清 PE 累加器
//   0x14 BANK    bit0 本次任务读哪一组 A/BT（双缓冲；另一组可同时被写）
//   0x18 TK      K 方向 Tile 大小 1..16
//   0x1C QUANT   量化右移位数
//
// CTRL 是命令寄存器，不保存 bit0/bit1；例如写 0x02 只产生 clear_done_req，
// 不会启动运算，回读 CTRL 时 bit0/bit1 始终为 0。
//
// 状态寄存器 0x20 的位定义：
//   bit0 BUSY          正在执行任务
//   bit1 DONE          任务完成，可读 C Buffer
//   bit2 保留，固定 0
//   bit3 BUFFER_READY  CPU 可以访问 Buffer（等价于 !BUSY）
//   bit4 ERROR         上一次 start 的配置非法，任务未启动
`include "npu_defines.vh"

module npu_top(
    input        clk,
    input        rst,
    // 来自 ahb_lite_slave 的原生请求（req_valid 在数据阶段单拍有效）
    input [31:0] req_addr,
    input [31:0] req_wdata,
    input [3:0]  req_byte_en,
    input        req_we,
    input        req_valid,
    // 来自 tile_controller
    input        core_busy,
    input        core_done,
    input [31:0] a_rdata_cpu, bt_rdata_cpu, c_rdata_cpu,
    // 来自搬运引擎
    input        dma_busy,
    input [31:0] dma_rdata,
    // 地址译码结果与读数据回给 ahb_lite_slave
    output reg        sel_ctrl,
    output reg        sel_status,
    output reg        sel_dma,     // 0x0040-0x005F 搬运引擎
    output reg        sel_bank,    // 0x0060-0x007F bank 观测
    output reg        sel_a_buf,
    output reg        sel_bt_buf,
    output reg        sel_c_buf,
    output reg        sel_wgt,     // 0x4000-0x7FFF 权重块
    output reg        sel_wgt_cfg, // 0x0800-0x080F 权重块配置
    output reg        sel_qparam,  // 0x0900-0x09FF 量化参数表
    output reg [2:0]  dma_reg_off,
    output reg [1:0]  bank_reg_off,
    output reg [31:0] control_rdata,
    output reg [31:0] status_rdata,
    output reg [31:0] bank_rdata,
    output reg [31:0] qparam_rdata,
    output reg [31:0] buffer_rdata,
    // 到 tile_controller（启动脉冲 + 锁存后的任务参数）
    output reg        start_pulse,
    output reg [`NPU_DIM_W-1:0]  cfg_m, cfg_n, cfg_k,
    output reg [`NPU_TK_W-1:0]   cfg_tk,
    output reg [`NPU_QS_W-1:0]   cfg_qshift,
    output reg        cfg_accumulate,
    output reg        core_bank, // 本任务读哪一组 A/BT（双缓冲）
    // CPU 侧 RAM 端口（到 npu_buffer）
    output reg        cpu_a_we,  cpu_a_re,
    output reg        cpu_bt_we, cpu_bt_re,
    output reg        cpu_c_we,  cpu_c_re,
    output reg [`NPU_ABUF_REG_AW-1:0] cpu_a_addr,
    output reg [`NPU_BBUF_REG_AW-1:0] cpu_bt_addr,
    output reg [`NPU_CBUF_REG_AW-1:0] cpu_c_addr,
    output reg [31:0] cpu_a_wdata, cpu_bt_wdata, cpu_c_wdata,
    output reg        cpu_wgt_we,
    output reg [`NPU_WGT_AW-1:0] cpu_wgt_addr,
    output reg [31:0] cpu_wgt_wdata,
    // 权重块配置直通（只在任务之间变化，不必像 m/n/k 那样锁存）
    output reg [`NPU_WGT_AW-1:0] wgt_base,
    output reg                   wgt_en,
    // ---- 量化参数表（per-channel requant，2026-11-17）----
    // 读口给 tile_controller 的写回扫描用；写口是下面 CPU 的那条路。
    input      [5:0]  qc_ram_addr,
    output     [31:0] qc_ram_bias,
    output     [31:0] qc_ram_mult,
    output     [5:0]  qc_ram_shift,
    output reg        qc_per_channel,   // 0 = 走旧的 qshift 路径（向后兼容）
    output reg        qc_round,         // 1 = 右移前加 2^(s-1) 做四舍五入
    output reg  [7:0] qc_zp,            // 输出 zero-point（按有符号解释）
    output reg  [7:0] qc_lo,            // 激活下界
    output reg  [7:0] qc_hi             // 激活上界
);

    // ---- 译码出的本地字地址与寄存器偏移 ----
    reg [`NPU_ABUF_REG_AW-1:0] a_local_addr;
    reg [`NPU_BBUF_REG_AW-1:0] bt_local_addr;
    reg [`NPU_CBUF_REG_AW-1:0] c_local_addr;
    reg [`NPU_WGT_AW-1:0]  wgt_local_addr;
    reg [1:0]  wgt_reg_off;
    reg [2:0]  ctrl_reg_off;

    // 锁存后的任务参数：任务开始后由这些寄存器保持到任务结束。
    reg [`NPU_DIM_W-1:0]  m_l, n_l, k_l;
    reg [`NPU_TK_W-1:0]   tk_l;
    reg [`NPU_QS_W-1:0]   qs_l;
    reg        accumulate_l;
    reg        done_status, error_status;
    reg        start_req, clear_done_req;
    reg        start_pulse_w;

    // Buffer 访问仲裁的输出直接落到端口上（见第五节），这里不再中转。

    //====================================================================
    //  一、MMIO 地址译码：req_addr 由 ahb_lite_slave 在数据阶段给出
    //  0x0000-0x001F 控制寄存器
    //  0x0020-0x002F 状态寄存器
    //  0x0040-0x005F 搬运引擎寄存器
    //  0x0060-0x007F bank 观测
    //  ★ 双缓冲：bank0 在 0x1000/0x2000，bank1 在 0x1800/0x2800，
    //    页内偏移仍是 [7:2]，bit[11] 选 bank（0x1400 这种中间页不认，保持原有空洞语义）。
    //  ★ C 也双缓冲：0x3000-0x33FF 是 bank0，0x3400-0x37FF 是 bank1，
    //    页内偏移 [9:2]，bit[10] 选 bank。
    //  0x4000-0x7FFF 权重块
    //  0x0800-0x080F 权重块配置
    //====================================================================
    always @(*) begin
        sel_ctrl    = (req_addr[31:5] == 27'd0);
        sel_status  = (req_addr[31:4] == 28'h0000002);
        sel_dma     = (req_addr[31:5] == 27'd2);
        sel_bank    = (req_addr[31:5] == 27'd3);
        sel_a_buf   = (req_addr[31:12] == 20'h00001) && (req_addr[10:8] == 3'd0);
        sel_bt_buf  = (req_addr[31:12] == 20'h00002) && (req_addr[10:8] == 3'd0);
        sel_c_buf   = (req_addr[31:11] == 21'd6);
        sel_wgt     = (req_addr[31:15] == 17'd0) && req_addr[14];
        sel_wgt_cfg = (req_addr[31:4] == 28'h00000080);
        // 0x0900-0x0FFF 量化参数表：bank=addr[9:8]（0=BIAS 1=MULT 2=SHIFT 3=QCFG）
        //            idx=addr[7:2]（按字索引，不是按字节）
        sel_qparam  = (req_addr[31:12] == 20'd0) && (req_addr[11:8] >= 4'd9);
        a_local_addr   = {req_addr[11], req_addr[7:2]};
        bt_local_addr  = {req_addr[11], req_addr[7:2]};
        c_local_addr   = {req_addr[10], req_addr[9:2]};
        wgt_local_addr = req_addr[13:2];
        wgt_reg_off    = req_addr[3:2];
        ctrl_reg_off   = req_addr[4:2];
        dma_reg_off    = req_addr[4:2];
        bank_reg_off   = req_addr[3:2];
    end

    //====================================================================
    //  二、配置寄存器（原 control_regs）
    //  req_valid 只在 ahb_lite_slave 发布一笔已锁存的请求时有效一个周期。
    //  配置寄存器均位于一个 32 位字的低字节，只接受 byte_en[0] 有效的写入，
    //  这样字节写和整字写行为一致，也不会误改高位保留字段。
    //====================================================================
    reg [`NPU_DIM_W-1:0]  cr_m, cr_n, cr_k;
    reg [`NPU_TK_W-1:0]   cr_tk;
    reg [`NPU_QS_W-1:0]   cr_qs;
    reg                   cr_accumulate;
    reg                   cr_bank;

    reg wr, byte0_wr, wgt_wr;
    reg [31:0] control_rdata_w;
    always @(*) begin
        wr        = req_valid && sel_ctrl && req_we;
        wgt_wr    = req_valid && sel_wgt_cfg && req_we && req_byte_en[0];
        byte0_wr  = wr && req_byte_en[0];
    end

    // 两个命令脉冲：只有写 0x00 CTRL 且低字节有效时才产生。
    always @(*) begin
        start_req      = byte0_wr && (ctrl_reg_off == 3'd0) && req_wdata[0];
        clear_done_req = byte0_wr && (ctrl_reg_off == 3'd0) && req_wdata[1];
    end

    always @(posedge clk) begin
        if (rst) begin
            cr_m           <= 6'd4;
            cr_n           <= 6'd4;
            cr_k           <= 6'd4;
            cr_tk          <= 5'd4;
            cr_qs          <= 5'd0;
            cr_accumulate  <= 1'b0;
            cr_bank        <= 1'b0;
            wgt_base       <= {`NPU_WGT_AW{1'b0}};
            wgt_en         <= 1'b0;
        end
        else if (wgt_wr) begin
            case (wgt_reg_off)
                2'd0: wgt_base <= req_wdata[`NPU_WGT_AW-1:0];
                2'd1: wgt_en   <= req_wdata[0];
                default: ;
            endcase
        end
        else if (byte0_wr) begin
            case (ctrl_reg_off)
                3'd1: cr_m          <= req_wdata[`NPU_DIM_W-1:0];
                3'd2: cr_n          <= req_wdata[`NPU_DIM_W-1:0];
                3'd3: cr_k          <= req_wdata[`NPU_DIM_W-1:0];
                3'd4: cr_accumulate <= req_wdata[0];
                3'd5: cr_bank       <= req_wdata[0];
                3'd6: cr_tk         <= req_wdata[`NPU_TK_W-1:0];
                3'd7: cr_qs         <= req_wdata[`NPU_QS_W-1:0];
                default: ;          // 0x00 CTRL 只有命令位，不保存
            endcase
        end
    end

    // 寄存器回读（字读）。CTRL 本身是命令寄存器，所以读 0x00 得到 0。
    always @(*) begin
        control_rdata_w =
            (ctrl_reg_off == 3'd0) ? {30'd0, 2'b00}          :
            (ctrl_reg_off == 3'd1) ? {26'd0, cr_m}           :
            (ctrl_reg_off == 3'd2) ? {26'd0, cr_n}           :
            (ctrl_reg_off == 3'd3) ? {26'd0, cr_k}           :
            (ctrl_reg_off == 3'd4) ? {31'd0, cr_accumulate}  :
            (ctrl_reg_off == 3'd5) ? {31'd0, cr_bank}        :
            (ctrl_reg_off == 3'd6) ? {27'd0, cr_tk}          :
            (ctrl_reg_off == 3'd7) ? {27'd0, cr_qs}          :
                                     32'd0;
    end

    //====================================================================
    //  三、状态寄存器
    //====================================================================
    always @(*) begin
        status_rdata = (ctrl_reg_off == 3'd0) ?
            {27'd0, error_status, !core_busy, 1'b0, done_status, core_busy} :
            32'd0;
    end

    // ---- bank 观测窗（0x0060-0x007F）----
    // 这里没有"host_bank"寄存器：本仓的 bank 是**地址里的位**（A/BT 用 bit[11]、
    // C 用 bit[10]），主机要哪一组直接按地址写就行，不需要再记一份影子选择。
    // 留下来的两个东西是双缓冲真正用得上的：当前算哪一组，以及"搬运和计算同时忙"
    // 的累计拍数 —— 后者是判断重叠有没有真的发生（而不是排队）的唯一硬指标。
    reg [31:0] overlap_cycles;
    always @(posedge clk) begin
        if (rst)
            overlap_cycles <= 32'd0;
        else if (core_busy && dma_busy)
            overlap_cycles <= overlap_cycles + 32'd1;
    end
    always @(*) begin
        case (bank_reg_off)
            2'd0:    bank_rdata = {28'd0, dma_busy, core_busy, 1'b0, core_bank};
            2'd1:    bank_rdata = overlap_cycles;
            2'd2:    bank_rdata = {27'd0, error_status, !core_busy, 1'b0, done_status, core_busy};
            default: bank_rdata = 32'd0;
        endcase
    end

    //====================================================================
    //  三之一、量化参数表（per-channel requant，2026-11-17）
    //
    //  写回扫描每拍处理一个输出元素，需要"它属于哪个输出通道"的 bias / 乘子 / 移位。
    //  这里给三张 64 深的小表，读口交给 tile_controller（地址由它给，见 npu_ctrl 的
    //  行切换停顿说明）。写口就是 CPU 那条路。
    //
    //  地址：0x0900-0x093F BIAS[0..63] / 0x0940-0x097F MULT[0..63]
    //        0x0980-0x09BF SHIFT[0..63] / 0x09C0 QCFG / 0x09C4 QOUT
    //====================================================================
    wire        qparam_wr   = req_valid && req_we && sel_qparam;
    // 表基址 0x0900，用"减去基址后的第 [9:8] 位"分表，第 [7:2] 位当字索引
    wire [11:0] qparam_off  = req_addr[11:0] - 12'h900;
    wire [1:0]  qparam_bank = qparam_off[9:8];
    wire [5:0]  qparam_idx  = qparam_off[7:2];

    wire [31:0] qc_bias_raw, qc_mult_raw, qc_shift_raw;

    npu_ram #(.AW(6)) u_qc_bias(
        .clk(clk), .we(qparam_wr && (qparam_bank == 2'd0)), .waddr(qparam_idx),
        .wdata(req_wdata), .re(1'b1), .raddr(qc_ram_addr), .rdata(qc_bias_raw)
    );
    npu_ram #(.AW(6)) u_qc_mult(
        .clk(clk), .we(qparam_wr && (qparam_bank == 2'd1)), .waddr(qparam_idx),
        .wdata(req_wdata), .re(1'b1), .raddr(qc_ram_addr), .rdata(qc_mult_raw)
    );
    npu_ram #(.AW(6)) u_qc_shift(
        .clk(clk), .we(qparam_wr && (qparam_bank == 2'd2)), .waddr(qparam_idx),
        .wdata(req_wdata), .re(1'b1), .raddr(qc_ram_addr), .rdata(qc_shift_raw)
    );

    always @(*) begin
        qc_ram_bias  = qc_bias_raw;
        qc_ram_mult  = qc_mult_raw;
        qc_ram_shift = qc_shift_raw[5:0];
    end

    wire qcfg_wr = qparam_wr && (qparam_bank == 2'd3) && (qparam_idx == 6'd0);
    wire qout_wr = qparam_wr && (qparam_bank == 2'd3) && (qparam_idx == 6'd1);

    always @(posedge clk) begin
        if (rst) begin
            qc_per_channel <= 1'b0;      // 默认走旧的 qshift 路径，旧软件不受影响
            qc_round       <= 1'b0;
            qc_zp          <= 8'd0;
            qc_lo          <= 8'h80;     // -128
            qc_hi          <= 8'h7F;     //  127
        end
        else begin
            if (qcfg_wr) begin
                qc_per_channel <= req_wdata[0];
                qc_round       <= req_wdata[1];
            end
            if (qout_wr) begin
                qc_zp <= req_wdata[7:0];
                qc_lo <= req_wdata[15:8];
                qc_hi <= req_wdata[23:16];
            end
        end
    end

    // 回读只保证 QCFG/QOUT（RAM 那三张表是只写，读回 0 —— 它们的内容本来就由软件掌握）
    always @(*) begin
        if (qparam_bank == 2'd3 && qparam_idx == 6'd0)
            qparam_rdata = {30'd0, qc_round, qc_per_channel};
        else if (qparam_bank == 2'd3 && qparam_idx == 6'd1)
            qparam_rdata = {8'd0, qc_hi, qc_lo, qc_zp};
        else
            qparam_rdata = 32'd0;
    end

    //====================================================================
    //  四、任务生命周期与启动校验（原 start_ctrl）
    //
    //  状态：ST_IDLE / ST_DONE 都可以接受新任务（启动即自动清除 DONE）
    //        ST_RUN -> (core_done) ST_DONE
    //        ST_ERROR -> (clear_done) ST_IDLE
    //
    //  · 只有配置合法才进入 ST_RUN 并锁存任务参数，运行期间软件改动不影响当前任务；
    //  · 配置非法时不进入 BUSY，只置 error_status，软件用 CTRL.clear_done 清除，
    //    核心不会被配置错误锁死（旧版本写 TK=0 会让 BUSY 永久卡住，只能硬件复位）；
    //  · ST_DONE 也接受 start_req：DONE 只是状态标志，此时核心已经空闲。早期版本
    //    只在 ST_IDLE 接受，DONE=1 时新的 start 被静默丢弃，软件会一直轮询到上一次
    //    的旧结果；跨任务累加（把 K 分段）也会因此失效。
    //
    //  配置校验依据 npu_defines.vh 里的容量宏：
    //    M/N/K 均在 1..63，TK 在 1..NPU_TK_MAX；
    //    A/BT 各需要 ceil(M/4)*K、ceil(N/4)*K 个 32 位字，不得超过 Buffer 容量；
    //    C 需要 M*N 个元素，不得超过 C Buffer 容量。
    //  注意：M/N/K 各 1..63 只是位宽上界，真正的约束是 Buffer 容量，
    //  ceil(M/4)*K <= 64 才是软件必须遵守的条件（例如 K=63 时 M 最多 4）。
    //====================================================================
    localparam [1:0]
        ST_IDLE  = 2'd0,
        ST_RUN   = 2'd1,
        ST_DONE  = 2'd2,
        ST_ERROR = 2'd3;

    reg [1:0] start_state;

    reg dim_ok, tk_ok;
    always @(*) begin
        dim_ok = (cr_m != 0) && (cr_n != 0) && (cr_k != 0);
        tk_ok  = (cr_tk != 0) && (cr_tk <= `NPU_TK_MAX);
    end

    // ceil(x/4) = (x+3)>>2，先除后乘避免中间量位宽爆炸。
    reg [`NPU_WORD_W-1:0] a_words, bt_words;
    reg buf_ok, cbuf_ok;
    always @(*) begin
        a_words  = ((cr_m + 6'd3) >> 2) * cr_k;
        bt_words = ((cr_n + 6'd3) >> 2) * cr_k;
        cbuf_ok  = (cr_m * cr_n) <= `NPU_CBUF_WORDS;
        buf_ok   = (a_words <= `NPU_ABUF_WORDS) && (bt_words <= `NPU_BBUF_WORDS);
    end
    // 跨任务累加在 C 写回路径上做"原值 + 本段结果"，所以每段结果必须是未经量化
    // 的原始部分和，否则多次分段会累积舍入误差。qshift!=0 时直接拒绝，而不是
    // 静默忽略量化配置。
    reg acc_ok, cfg_ok;
    always @(*) begin
        acc_ok = !(cr_accumulate && (cr_qs != 0));
        cfg_ok = dim_ok && tk_ok && buf_ok && cbuf_ok && acc_ok;
    end

    always @(*) begin
        done_status  = (start_state == ST_DONE);
        error_status = (start_state == ST_ERROR);
    end

    always @(posedge clk) begin
        if (rst) begin
            start_state  <= ST_IDLE;
            start_pulse_w <= 1'b0;
            m_l          <= 6'd0;
            n_l          <= 6'd0;
            k_l          <= 6'd0;
            tk_l         <= 5'd0;
            qs_l         <= 5'd0;
            accumulate_l <= 1'b0;
        end
        else begin
            start_pulse_w <= 1'b0;
            case (start_state)
                ST_IDLE, ST_DONE: begin
                    if (start_req) begin
                        if (cfg_ok) begin
                            // 锁存本次任务的参数：运行期间软件再改配置不生效。
                            m_l          <= cr_m;
                            n_l          <= cr_n;
                            k_l          <= cr_k;
                            tk_l         <= cr_tk;
                            qs_l         <= cr_qs;
                            accumulate_l <= cr_accumulate;
                            start_pulse_w <= 1'b1;
                            start_state  <= ST_RUN;
                        end
                        else begin
                            start_state <= ST_ERROR;
                        end
                    end
                    else if (clear_done_req) begin
                        start_state <= ST_IDLE;
                    end
                end

                ST_RUN: begin
                    if (core_done)
                        start_state <= ST_DONE;
                end

                ST_ERROR: begin
                    if (clear_done_req)
                        start_state <= ST_IDLE;
                end

                default: start_state <= ST_IDLE;
            endcase
        end
    end

    //====================================================================
    //  五、Buffer 访问仲裁（原 buffer_access_ctrl）
    //
    //  core_busy=0 时 CPU 拥有 A/BT/C 的访问端口；core_busy=1 时 NPU 正在使用
    //  Buffer，所有 CPU 读写使能都被屏蔽。本段屏蔽的是 RAM 端口，属于最后一道
    //  防线；对外的错误应答由 ahb_lite_slave 负责（core_busy=1 时访问 Buffer 会回
    //  HRESP=ERROR，而不是像旧 mmio_if 那样静默丢弃却应答成功）。
    //  同时完成 CPU 写数据的字节屏蔽，直接输出三组 RAM 端口。
    //====================================================================
    reg cpu_ok, cpu_wr_ok;
    reg [31:0] byte_mask;

    always @(*) begin
        // Buffer_READY 与 cpu_ok 同源；状态寄存器 bit3 用它告诉软件当前能否装载/读取数据。
        cpu_ok    = !core_busy;
        // 写：空闲随便写；忙的时候只能写另一组（地址里的 bank 位 != 核正在用的 bank）。
        // 这一条对 A/BT 成立；C 的写口忙时归写回 FIFO，所以仍按 cpu_ok 一刀切。
        cpu_wr_ok = cpu_ok | (a_local_addr[`NPU_ABUF_REG_AW-1] != core_bank);
        cpu_a_we  = req_valid && req_we  && sel_a_buf  && cpu_wr_ok;
        cpu_a_re  = req_valid && !req_we && sel_a_buf  && cpu_ok;
        cpu_bt_we = req_valid && req_we  && sel_bt_buf && (cpu_ok | (bt_local_addr[`NPU_BBUF_REG_AW-1] != core_bank));
        cpu_bt_re = req_valid && !req_we && sel_bt_buf && cpu_ok;
        cpu_c_we  = req_valid && req_we  && sel_c_buf  && cpu_ok;
        cpu_c_re  = req_valid && !req_we && sel_c_buf  && cpu_ok;
        cpu_a_addr  = a_local_addr;
        cpu_bt_addr = bt_local_addr;
        cpu_c_addr  = c_local_addr;
        byte_mask = {{8{req_byte_en[3]}}, {8{req_byte_en[2]}},
                     {8{req_byte_en[1]}}, {8{req_byte_en[0]}}};
        cpu_a_wdata  = req_wdata & byte_mask;
        cpu_bt_wdata = req_wdata & byte_mask;
        cpu_c_wdata  = req_wdata & byte_mask;
        cpu_wgt_we    = req_valid && req_we && sel_wgt && cpu_ok;
        cpu_wgt_addr  = wgt_local_addr;
        cpu_wgt_wdata = req_wdata & byte_mask;

        buffer_rdata = sel_a_buf  ? a_rdata_cpu  :
                       sel_bt_buf ? bt_rdata_cpu : c_rdata_cpu;
    end

    //====================================================================
    //  六、输出转发
    //====================================================================
    // 双缓冲：bank 在 start_pulse 那一拍锁存，运行期间软件改 BANK 不影响当前任务；
    // 这样软件可以在核忙着算 bank b 的时候，往 bank ~b 里预装下一块的数据。
    always @(posedge clk) begin
        if (rst)
            core_bank <= 1'b0;
        else if (start_pulse_w)
            core_bank <= cr_bank;
    end

    always @(*) begin
        control_rdata  = control_rdata_w;
        start_pulse    = start_pulse_w;
        cfg_m          = m_l;
        cfg_n          = n_l;
        cfg_k          = k_l;
        cfg_tk         = tk_l;
        cfg_qshift     = qs_l;
        cfg_accumulate = accumulate_l;
    end

endmodule
