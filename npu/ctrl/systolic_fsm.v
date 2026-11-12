`timescale 1ns / 1ps
// systolic_fsm — 一次任务的主时序状态机
//
// 状态与转移（每个输出 Tile 走一遍 K 循环，只在 Tile 结束时排空一次）：
//   S_IDLE       start_pulse 到来 -> S_INIT
//   S_INIT       sched_init 复位 tile 计数器、启动第一个 tile 的整段预取 -> S_PREFETCH
//   S_PREFETCH   等 A/BT FIFO 各出现至少 1 个字（pair_ready）且有空闲快照 bank
//                （snap_ready，本块结果有地方落）-> S_FEED
//   S_FEED       成对喂 valid_tk 拍；喂完一个 K Tile 后
//                最后一个 K Tile -> S_DRAIN，否则留在 S_FEED 直接进下一个 K Tile
//                （next_k_req 只用来推进 tile_k 计数器，不再回 S_PREFETCH 绕一圈）
//   S_DRAIN      保持阵列推进 P+Q-2 拍让波前走完；同时在 drain_done 那一拍清 PE
//                累加器、锁存本块结果、推进 tile 并启动下一块的整段预取
//                -> 最后一个输出 Tile ? S_DONE : S_PREFETCH
//   S_DONE       等 core_done（最后一块 C 已落入 RAM）-> S_IDLE
//
// 关键时序约定：
//   1. K Tile 之间不清累加器、不排空阵列，跨 K Tile 的部分和留在 PE 里连续累加，
//      TK 只是延迟/面积参数，不再是每个 K Tile 都要付一次的排空开销。
//      喂数途中若 FIFO 暂时没数，S_FEED 原地等（array_enable 保持 1，valid=0 的
//      空拍只会在管线里平移，不会累加），波前对齐不受影响。
//   2. 预取与喂数重叠：一个输出 Tile 的 A/BT 数据在地址上是连续的一段
//      （A: tile_i*K + [0,K)，BT: tile_j*K + [0,K)），一次发出整段 K 字的连续突发。
//      第一个 tile 由 pf_first 启动；之后每个 tile 在**上一个 tile 的排空期间**
//      就由 pf_next 启动（那时 tile 计数器还没推进，所以用 tile_controller 预测的
//      下一块基址）。排空 6 拍正好把突发的首字延迟遮住，S_PREFETCH 通常一拍通过。
//   3. 排空后结果由 c_tile_write_ctrl 锁存成快照并在后台写回。快照有双 bank，
//      状态机只等"有空闲 bank"（snap_ready），不等扫描结束、也不等写 FIFO 排空，
//      所以写回的 16 拍扫描能被下一块 tile 的计算遮住；core_done 才代表全部
//      C 已真正落入 RAM，它仍然等 cwr_idle（扫描结束且写 FIFO 排空）。
`include "npu_defines.vh"

module systolic_fsm(
    input        clk,
    input        rst,
    input        start_pulse,
    input        core_done,          // 全部 C 元素已真正写入 C RAM
    input        last_k_tile,
    input        last_out_tile,
    input [`NPU_TK_W-1:0]  valid_tk,
    // 预取 FIFO 计数
    input [`NPU_FIFO_AW:0] a_fifo_count,
    input [`NPU_FIFO_AW:0] bt_fifo_count,
    // 喂数节拍(tile_controller 内部成对出队)
    input        pair_fire,
    // MAC 排空完成
    input        drain_done,
    // 有空闲快照 bank：可以接受本块结果而不覆盖还没扫完的上一块。
    // 注意这不是"写回已完成"，完成判定用 core_done。
    input        snap_ready,
    // 输出
    output reg   sched_init,
    output reg   next_k_req,
    output reg   next_out_req,
    output reg   acc_clear,
    output reg   pf_start,         // 单拍：启动本输出 Tile 的整段预取
    output reg   pf_next,          // 单拍：排空期间启动下一个输出 Tile 的整段预取
    output reg   feed_go,
    output reg   drain_en,         // 电平：排空期(由 state 组合给出，不少等一拍)
    output reg   array_enable,
    output reg   fsm_done,
    output reg   core_busy
);

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

    // 一对 A/BT 数据是否就绪：两个 FIFO 都非空就可以开喂。整段突发会让 FIFO
    // 边填边被消费，早开喂能把突发的首字延迟和喂数重叠，所以不等满一个 K Tile。
    // 每拍消费 1 对、突发每拍补 1 个字，稳态下 FIFO 深度只留几个字的余量。
    wire pair_ready     = (a_fifo_count != 0) && (bt_fifo_count != 0);
    wire feed_beat_done = pair_fire && (feed_cnt + 6'd1 >= {1'b0, valid_tk});
    wire is_beat_done   = (state == S_FEED) && feed_beat_done;

    always @(*) begin
        core_busy    = (state != S_IDLE);
        fsm_done     = (state == S_DONE);
        // drain_en 由状态组合给出：寄存输出会晚一拍，实测每块 tile 多等 1 拍。
        drain_en     = (state == S_DRAIN);
        array_enable = feed_go || drain_en;
        // next_k_req 必须是组合脉冲（完成当拍推进 tile_k 计数器），不能改成寄存：
        // K Tile 背靠背喂数时（例如 TK=1，每拍喂完一个 K Tile），寄存脉冲会让
        // tile_k 晚一拍更新，于是本模块在下一拍读到的 last_k_tile 还是旧值，
        // 最后一个 K Tile 会被漏判、tile_k 冲过边界后卡在 S_FEED。
        // 旧实现每个 K Tile 都回 S_PREFETCH 绕一圈，正好把这一拍让给计数器，
        // 所以当时用寄存脉冲看不出问题。
        next_k_req   = is_beat_done && !last_k_tile;
    end

    always @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            sched_init    <= 1'b0;
            next_out_req  <= 1'b0;
            acc_clear     <= 1'b0;
            pf_start      <= 1'b0;
            pf_next       <= 1'b0;
            feed_go       <= 1'b0;
            feed_cnt      <= 6'd0;
            pf_done_q     <= 1'b0;
        end else begin
            // 单拍脉冲默认清零；电平信号在各状态分支中显式拉高。
            // acc_clear 也在这里清零：它是"换输出 Tile 时的电平"，只在排空结束那一拍
            // 重新拉高，否则会一直保持高把累加器每拍清零。
            sched_init    <= 1'b0;
            next_out_req  <= 1'b0;
            acc_clear     <= 1'b0;
            pf_start      <= 1'b0;
            pf_next       <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start_pulse)
                        state <= S_INIT;
                end

                S_INIT: begin
                    sched_init <= 1'b1;
                    feed_cnt   <= 6'd0;
                    pf_done_q  <= 1'b0;
                    // 任务开始时必须清一次累加器：合并掉 S_CLEAR_ACC 之后，第一个
                    // 输出 Tile 前面没有排空，不在这里清就会带上一个任务的残留值。
                    acc_clear  <= 1'b1;
                    state      <= S_PREFETCH;
                end

                // 整段预取在上一个 tile 的排空期间就发出了，这里通常一拍通过；
                // 同时确认有空闲快照 bank（本块结果必须有地方落）。
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
                                // 本输出 Tile 最后一个 K Tile 喂完：立刻启动下一块的
                                // 整段预取。放在这里（排空第一拍）而不是排空结束那一拍，
                                // 是为了让排空的 7 拍把突发的首字延迟完全遮住。
                                // 基址用 tile_controller 预测的下一块值，因为此刻
                                // tile 计数器还没推进。
                                if (!last_out_tile)
                                    pf_next <= 1'b1;
                                feed_go <= 1'b0;
                                state   <= S_DRAIN;
                            end else begin
                                // 直接留在 S_FEED 进下一个 K Tile：预取是整块 tile 的
                                // 连续突发，下一个 K Tile 的数据已经在 FIFO 里或正在
                                // 路上；万一没到齐，pair_fire 自然为 0，在这里等即可。
                                // tile_k 计数器由组合的 next_k_req 在当拍推进。
                                feed_cnt <= 6'd0;
                            end
                        end
                    end
                end

                // 只有在所有 K Tile 都喂完后才排空一次，结果才完整。
                // drain_done 那一拍：本块最后一批乘积已经落到最远端 PE，
                // 于是同一拍完成四件事——
                //   采数给 c_tile_write_ctrl（它自己在 drain_done 上取沿）；
                //   acc_clear 置起来（下一拍生效，先读走再清零）；
                //   推进 tile 计数器；
                //   用预测的下一块基址启动下一段整段预取（排空这几拍正好遮住突发首字延迟）。
                S_DRAIN: begin
                    if (drain_done) begin
                        acc_clear <= 1'b1;
                        if (last_out_tile) begin
                            state <= S_DONE;
                        end else begin
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

endmodule
