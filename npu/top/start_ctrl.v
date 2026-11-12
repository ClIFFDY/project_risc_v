`timescale 1ns / 1ps
// start_ctrl — 任务生命周期与启动前配置校验
//
// 状态：ST_IDLE / ST_DONE 都可以接受新任务（启动即自动清除 DONE）
//       ST_RUN -> (core_done) ST_DONE
//       ST_ERROR -> (clear_done) ST_IDLE
//
// 责任：
//   1. 只有配置合法才进入 ST_RUN 并锁存任务参数，运行期间软件改动不影响当前任务；
//   2. 配置非法时不进入 BUSY，只置 error_status，软件用 CTRL.clear_done 清除，
//      核心不会被配置错误锁死（旧版本写 TK=0 会让 BUSY 永久卡住，只能硬件复位）；
//   3. accumulate=1 时锁存"跨任务累加"，第一个输出 Tile 不清 PE 累加器。
//
// 为什么 ST_DONE 也接受 start_req：DONE 只是状态标志，此时核心已经空闲。
// 早期版本只在 ST_IDLE 接受 start_req，DONE=1 时新的 start 被静默丢弃，
// 软件会一直轮询到上一次的旧结果；跨任务累加（把 K 分段）也会因此失效。
// 现在启动会直接把 DONE 清掉，软件不必"先写 clear_done 再写 start"。
//
// 配置校验依据 npu_defines.vh 里的容量宏：
//   M/N/K 均在 1..63，TK 在 1..NPU_TK_MAX；
//   A/BT 各需要 ceil(M/4)*K、ceil(N/4)*K 个 32 位字，不得超过 Buffer 容量；
//   C 需要 M*N 个元素，不得超过 C Buffer 容量。
// 注意：规格里 M/N/K 各 1..63 只是位宽上界，真正的约束是 Buffer 容量，
// ceil(M/4)*K <= 64 才是软件必须遵守的条件（例如 K=63 时 M 最多 4）。
`include "npu_defines.vh"

module start_ctrl(
    input        clk,
    input        rst,
    input        start_req,
    input        clear_done_req,
    input [`NPU_DIM_W-1:0]  cfg_m, cfg_n, cfg_k,
    input [`NPU_TK_W-1:0]   cfg_tk,
    input [`NPU_QS_W-1:0]   cfg_qshift,
    input                   cfg_accumulate,
    input        core_done_i,
    output reg         start_pulse,       // 单拍
    output reg  [`NPU_DIM_W-1:0]  m_l, n_l, k_l,
    output reg  [`NPU_TK_W-1:0]   tk_l,
    output reg  [`NPU_QS_W-1:0]   qs_l,
    output reg         accumulate_l,
    output reg         done_status,
    output reg         error_status
);

    localparam [1:0]
        ST_IDLE  = 2'd0,
        ST_RUN   = 2'd1,
        ST_DONE  = 2'd2,
        ST_ERROR = 2'd3;

    reg [1:0] state;

    // ---- 配置校验（组合）----
    wire dim_ok = (cfg_m != 0) && (cfg_n != 0) && (cfg_k != 0);
    wire tk_ok  = (cfg_tk != 0) && (cfg_tk <= `NPU_TK_MAX);

    // ceil(x/4) = (x+3)>>2，先除后乘避免中间量位宽爆炸。
    wire [`NPU_WORD_W-1:0] a_words  = ((cfg_m + 6'd3) >> 2) * cfg_k;
    wire [`NPU_WORD_W-1:0] bt_words = ((cfg_n + 6'd3) >> 2) * cfg_k;
    wire buf_ok  = (a_words <= `NPU_ABUF_WORDS) && (bt_words <= `NPU_BBUF_WORDS);
    wire cbuf_ok = (cfg_m * cfg_n) <= `NPU_CBUF_WORDS;
    // 跨任务累加在 C 写回路径上做"原值 + 本段结果"，所以每段结果必须是未经量化
    // 的原始部分和，否则多次分段会累积舍入误差。qshift!=0 时直接拒绝，而不是
    // 静默忽略量化配置。
    wire acc_ok  = !(cfg_accumulate && (cfg_qshift != 0));
    wire cfg_ok  = dim_ok && tk_ok && buf_ok && cbuf_ok && acc_ok;

    always @(*) begin
        done_status  = (state == ST_DONE);
        error_status = (state == ST_ERROR);
    end

    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            start_pulse  <= 1'b0;
            m_l          <= 6'd0;
            n_l          <= 6'd0;
            k_l          <= 6'd0;
            tk_l         <= 5'd0;
            qs_l         <= 5'd0;
            accumulate_l <= 1'b0;
        end else begin
            start_pulse <= 1'b0;
            case (state)
                // ST_DONE 也接受新任务：启动即进入 ST_RUN，DONE 随之清除。
                ST_IDLE, ST_DONE: begin
                    if (start_req) begin
                        if (cfg_ok) begin
                            // 锁存本次任务的参数：运行期间软件再改配置不生效。
                            m_l          <= cfg_m;
                            n_l          <= cfg_n;
                            k_l          <= cfg_k;
                            tk_l         <= cfg_tk;
                            qs_l         <= cfg_qshift;
                            accumulate_l <= cfg_accumulate;
                            start_pulse  <= 1'b1;
                            state        <= ST_RUN;
                        end else begin
                            state <= ST_ERROR;
                        end
                    end else if (clear_done_req) begin
                        state <= ST_IDLE;
                    end
                end

                ST_RUN: begin
                    if (core_done_i)
                        state <= ST_DONE;
                end

                ST_ERROR: begin
                    if (clear_done_req)
                        state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
