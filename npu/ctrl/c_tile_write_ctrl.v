`timescale 1ns / 1ps
// c_tile_write_ctrl — C Tile 结果快照与写回扫描（双 bank 快照）
//
// 1) 模块位置和职责
//    tile_controller 的子模块。npu_mac 排空完成那一拍，把 16 个 PE 累加器整块
//    锁存成"快照"，之后逐拍扫描：选累加器 -> 量化 -> 推入 C 写 FIFO。结果一旦
//    搬进快照，阵列就能立刻开始下一个输出 Tile，写回与下一块计算并行。
//
// 2) 数据布局、位宽和单位
//    acc_flat_in 的第 (r*Q+c)*NPU_ACC_W +: NPU_ACC_W 位是 PE[r][c] 的累加器；
//    扫描序号 idx_q 取 0..15，其中 r=idx_q[3:2]、c=idx_q[1:0]；
//    cwr_addr_out 是 C Buffer 的 32 位字地址，c_base_* 是与它同精度的基地址；
//    valid_tm/valid_tn 是边界 Tile 的有效行数和列数。
//    快照必须把"结果 + 写回所需的全部上下文"一起锁存：状态机在排空完成同一拍
//    就发出 next_out_req 推进 tile 计数器，扫描期间 tile_controller 给出的
//    c_base/valid_tm/valid_tn 已经是"下一块"的值，直接用会把整块结果写错地址。
//
// 3) 时序延迟
//    snap_valid_in（= 排空完成）是电平，用上一拍寄存值做上升沿检测，只捕获一次。
//    捕获后下一拍开始扫描：每个时钟沿推入一项，cwr_ready_in 为低（FIFO 满）时
//    扫描暂停并保持 bank 忙，FIFO 一有空间就从原位置继续。
//    量化由 result_quantizer 组合完成，不额外增加延迟。
//
// 4) 复位后的状态和主要转移
//    复位后 active_q=0、bank_busy_q=2'b00，两块快照 bank 都空闲。转移：
//      空闲 + snap_valid_in 上升沿 + 有空闲 bank -> 锁存进空闲 bank 并置该 bank 忙；
//      空闲 + 有 bank 忙              -> 启动扫描（一次只扫一块，扫描顺序无关：
//                                       每块结果写的是不同地址，跨任务累加模式下
//                                       是对同一地址做加法，可交换）；
//      扫描 + idx_q 走到 15 且本拍推进 -> 该 bank 置闲，回空闲。
//
// 5) 边界条件、错误处理和不能修改的接口假设
//    · 双 bank 的目的：旧实现只有一组快照寄存器，状态机必须等到"扫描结束 +
//      C 写 FIFO 排空"才能开下一个输出 Tile，每块 tile 白等 15 拍（16x16x16
//      实测共 240 拍，占全部周期的 25%）。改成两组 bank 后状态机只等
//      "有空闲 bank"，而扫描的 16 拍能被下一块 tile 的计算时间遮住。
//    · 正确性依赖的接口假设：**状态机必须在 S_CLEAR_ACC 用 snap_ready_out
//      确认有空闲 bank 之后才推进 tile**。bank 只会在扫描完成时由忙变闲，两次
//      快照之间不会被别的逻辑占用，所以"此刻有空闲 bank"⇒"下一次快照到达时
//      仍有空闲 bank"，捕获不会丢。删掉 tile_controller 里这个检查会丢结果。
//    · idle_out 的语义保持"扫描结束且 C 写 FIFO 已排空"，tile_controller 用它
//      判定整个任务真正写完。不要用 snap_ready_out 替代它：后者只表示"能接受
//      下一块快照"，不代表结果已经落进 C RAM。
//    · 边界 Tile 中 r>=valid_tm 或 c>=valid_tn 的 lane 不入队，但 idx_q 照常前进，
//      所以边界不影响扫描的时序长度，只影响入队项数。
`include "npu_defines.vh"

module c_tile_write_ctrl(
    input        clk,
    input        rst,
    // 来自 npu_mac
    input [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_flat_in,
    input        snap_valid_in,
    // 来自 tile_controller
    input [`NPU_CBASE_W-1:0] c_base_in,
    input [`NPU_DIM_W-1:0]   n_dim_in,
    input [`NPU_TILE_W-1:0]  valid_tm_in,
    input [`NPU_TILE_W-1:0]  valid_tn_in,
    input [`NPU_QS_W-1:0]    qshift_in,
    // 到 c_write_fifo
    input        cwr_ready_in,          // FIFO 未满
    input        cwr_empty_in,          // FIFO 已排空
    output reg   cwr_valid_out,
    output reg [`NPU_CBUF_AW-1:0] cwr_addr_out,
    output reg [31:0] cwr_data_out,
    // 状态
    output reg   idle_out,              // 扫描结束且写 FIFO 已排空（任务真正写完）
    output reg   snap_ready_out         // 有空闲快照 bank（可以接受下一块结果）
);

    // 快照 bank 数量：2 组就够。扫描 16 拍，输出 Tile 的最短周期约 11 拍，
    // 两组 bank 能吸收"写回跟不上计算"时的积压；积压到两组都忙时，
    // snap_ready_out 拉低，由状态机在 S_CLEAR_ACC 等待，不会丢结果。
    localparam BANK_NUM = 2;

    // 每个 bank 一份快照，以及写回所需的全部上下文。
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

    integer b;

    // ---- 组合中间结果：全部在 always @(*) 中赋值，无隐含锁存器 ----
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

    reg [`NPU_ACC_W-1:0] acc_sel;
    reg [`NPU_CBASE_W+1:0] c_addr;
    wire [31:0] quant_word;

    always @(*) begin
        // 赋值顺序 = 数据依赖顺序。组合块用阻塞赋值，后面的语句能读到前面刚算出的
        // 结果，但反过来会读到"上一拍的值"。bank_clr/bank_busy_d 依赖 scan_done，
        // 所以扫描数据通路必须写在它们前面：写反了会让 bank 清位晚一拍，扫描结束后
        // 出现"active_q=0 但 bank 仍然忙"的空档，scan_start 会再扫一遍同一块
        // （非累加模式下重复写同值是幂等的，测不出来；累加模式会多加一次结果）。
        // ---- 扫描数据通路（active_q=1 时有效，bank 号取 scan_bank_q）----
        r_idx   = idx_q[3:2];
        c_idx   = idx_q[1:0];
        lane_ok = (idx_q < 5'd16) &&
                  ({1'b0, r_idx} < valid_tm_q[scan_bank_q]) &&
                  ({1'b0, c_idx} < valid_tn_q[scan_bank_q]);
        // 有效项在 FIFO 有空间时入队并前进；无效项直接跳过。
        advance    = active_q && (idx_q < 5'd16) && (lane_ok ? cwr_ready_in : 1'b1);
        scan_done  = advance && (idx_q == 5'd15);

        // ---- bank 管理 ----
        // 优先用 bank0 作为新快照的落点，扫描优先取 bank0。
        // 捕获受 any_free 门控，因此 cap_bank 只在有空闲 bank 时被使用。
        any_free      = (bank_busy_q != {BANK_NUM{1'b1}});
        cap_bank      = bank_busy_q[0] ? 1'b1 : 1'b0;
        cap_fire      = snap_valid_in && !snap_level_q && any_free;
        scan_bank_sel = bank_busy_q[0] ? 1'b0 : 1'b1;
        scan_start    = !active_q && (bank_busy_q != {BANK_NUM{1'b0}});

        // bank_busy 的下一拍值：置忙与置闲在这里合并成一次赋值，避免同一信号在
        // 时序块内被多次赋值（编码规范 4.2）。同一拍捕获与扫完必然不是同一个
        // bank：捕获只能落在空闲 bank，而扫完的 bank 本拍仍然是忙的。
        bank_set    = cap_fire ? ({{(BANK_NUM-1){1'b0}}, 1'b1} << cap_bank) : {BANK_NUM{1'b0}};
        bank_clr    = scan_done ? ({{(BANK_NUM-1){1'b0}}, 1'b1} << scan_bank_q) : {BANK_NUM{1'b0}};
        bank_busy_d = (bank_busy_q | bank_set) & ~bank_clr;

        acc_sel = snap_q[scan_bank_q][idx_q[3:0]*`NPU_ACC_W +: `NPU_ACC_W];
        c_addr  = c_base_q[scan_bank_q] + r_idx * n_dim_q[scan_bank_q] + c_idx;
    end

    result_quantizer u_result_quantizer(
        .acc_data(acc_sel), .qshift(qshift_in), .c_word(quant_word)
    );

    always @(posedge clk) begin
        if (rst) begin
            idx_q       <= 5'd0;
            active_q    <= 1'b0;
            snap_level_q <= 1'b0;
            scan_bank_q <= 1'b0;
            bank_busy_q <= {BANK_NUM{1'b0}};
            for (b = 0; b < BANK_NUM; b = b + 1) begin
                snap_q[b]     <= {`NPU_PE_NUM*`NPU_ACC_W{1'b0}};
                c_base_q[b]   <= {`NPU_CBASE_W{1'b0}};
                n_dim_q[b]    <= {`NPU_DIM_W{1'b0}};
                valid_tm_q[b] <= {`NPU_TILE_W{1'b0}};
                valid_tn_q[b] <= {`NPU_TILE_W{1'b0}};
            end
        end
        else begin
            snap_level_q <= snap_valid_in;
            bank_busy_q  <= bank_busy_d;

            // 捕获：上升沿 + 有空闲 bank（接口假设保证此时一定有，见文件头第 5 条）
            if (cap_fire) begin
                snap_q[cap_bank]     <= acc_flat_in;
                c_base_q[cap_bank]   <= c_base_in;
                n_dim_q[cap_bank]    <= n_dim_in;
                valid_tm_q[cap_bank] <= valid_tm_in;
                valid_tn_q[cap_bank] <= valid_tn_in;
            end

            if (scan_start) begin
                active_q    <= 1'b1;
                scan_bank_q <= scan_bank_sel;
                idx_q       <= 5'd0;
            end
            else if (advance) begin
                idx_q <= idx_q + 5'd1;
                if (idx_q == 5'd15)
                    active_q <= 1'b0;      // 最后一项已送出，本 bank 在下拍置闲
            end
        end
    end

    always @(*) begin
        // cwr_valid_out 与 cwr_ready_in 相与，表示本拍一定能完成一次 FIFO 入队。
        cwr_valid_out = active_q && lane_ok && cwr_ready_in;
        cwr_addr_out  = c_addr[`NPU_CBUF_AW-1:0];
        cwr_data_out  = quant_word;
        // 只有"没有扫描在跑、没有 bank 待扫、写 FIFO 也空了"才算真正写完。
        idle_out      = !active_q && (bank_busy_q == {BANK_NUM{1'b0}}) && cwr_empty_in;
        snap_ready_out = any_free;
    end

endmodule
