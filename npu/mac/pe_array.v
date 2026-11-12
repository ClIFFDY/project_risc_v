`timescale 1ns / 1ps
// pe_array — P x Q PE 阵列（数据流：A 从左向右传播，BT 从上向下传播）
//
// PE[r][c] 累加 C[r][c] = sum_k A[r][k] * BT[c][k]（输出驻留）。
// 端口布局：
//   a_lanes_in 的第 r*8 +: 8 位是第 r 行的最左输入；
//   bt_lanes_in 的第 c*8 +: 8 位是第 c 列的最上输入；
//   a_lane_v_in / bt_lane_v_in 是每个 lane 的 valid，与数据同拍；
//   acc_flat_out 的第 (r*Q+c)*NPU_ACC_W +: NPU_ACC_W 位是 PE[r][c] 的累加器。
// 时序：a_out/bt_out 每级 1 拍寄存转发，所以从最左/最上输入到 PE[r][c]
// 的传播延迟分别是 c 拍和 r 拍（波前对齐由 a_bt_skew_pipeline 完成）。
// clear_acc 优先级高于 enable：清累加器会同时冲刷转发寄存器里的残留波前，
// 只在换输出 Tile 时使用，不能在 K Tile 之间使用（那会丢掉部分和）。
`include "npu_defines.vh"

module pe_array(
    input        clk,
    input        rst,
    input        enable,
    input        clear_acc,
    input [`NPU_P*8-1:0] a_lanes_in,
    input [`NPU_P-1:0]   a_lane_v_in,
    input [`NPU_Q*8-1:0] bt_lanes_in,
    input [`NPU_Q-1:0]   bt_lane_v_in,
    output reg [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_flat_out
);

    // PE 之间的转发网络：下标都是 r*Q+c，与 acc_flat_out 的 PE 编号一致。
    wire [`NPU_PE_NUM*8-1:0]  a_fwd_w;
    wire [`NPU_PE_NUM*8-1:0]  bt_fwd_w;
    wire [`NPU_PE_NUM-1:0]    a_v_fwd_w;
    wire [`NPU_PE_NUM-1:0]    bt_v_fwd_w;
    wire [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_w;

    genvar r, c;

    generate
        for (r = 0; r < `NPU_P; r = r + 1) begin : g_pe_row
            for (c = 0; c < `NPU_Q; c = c + 1) begin : g_pe_col
                // 上游 PE 的下标；边界取 0 是为了避免生成负下标（该分支不会被选到）。
                localparam integer R_UP = (r == 0) ? 0 : (r - 1);
                localparam integer C_UP = (c == 0) ? 0 : (c - 1);

                pe_cell u_pe_cell(
                    .clk(clk), .rst(rst),
                    .enable(enable), .clear_acc(clear_acc),
                    .a_in    ((c == 0) ? a_lanes_in[r*8 +: 8]
                                       : a_fwd_w[(r*`NPU_Q + C_UP)*8 +: 8]),
                    .a_v_in  ((c == 0) ? a_lane_v_in[r]
                                       : a_v_fwd_w[r*`NPU_Q + C_UP]),
                    .bt_in   ((r == 0) ? bt_lanes_in[c*8 +: 8]
                                       : bt_fwd_w[(R_UP*`NPU_Q + c)*8 +: 8]),
                    .bt_v_in ((r == 0) ? bt_lane_v_in[c]
                                       : bt_v_fwd_w[R_UP*`NPU_Q + c]),
                    .a_out   (a_fwd_w[(r*`NPU_Q + c)*8 +: 8]),
                    .a_v_out (a_v_fwd_w[r*`NPU_Q + c]),
                    .bt_out  (bt_fwd_w[(r*`NPU_Q + c)*8 +: 8]),
                    .bt_v_out(bt_v_fwd_w[r*`NPU_Q + c]),
                    .acc     (acc_w[(r*`NPU_Q + c)*`NPU_ACC_W +: `NPU_ACC_W])
                );
            end
        end
    endgenerate

    always @(*) begin
        acc_flat_out = acc_w;
    end

endmodule
