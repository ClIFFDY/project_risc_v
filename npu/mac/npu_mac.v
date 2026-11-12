`timescale 1ns / 1ps
// npu_mac — INT8 MAC 计算平面
//
// 输入路径把一个 32 位字拆成 4 个带边界屏蔽的 INT8 lane，再用 skew 管线
// 让不同的行/列在正确的周期相遇。PE 阵列只在 array_enable_in 时推进；
// 排空阶段没有新输入但仍保持 enable，让最后一个波前到达远端 PE。
//
// 本模块不做 Tile 调度，也不访问 RAM：它消费成对的 A/BT 流，直接输出
// 16 个 PE 累加器（acc_flat_out）。累加器跨 K Tile 一直保持，只在
// acc_clear_in 时清零，因此部分和不需要搬到阵列外面再搬回来。
//
// 边界补零：lane_en_in 为 0 的 lane 数据被强制为 0，越界的行/列参与乘法
// 但不影响结果（且越界 C 元素不会被写回）。
// 端口布局：
//   a_stream_in/a_lane_en_in 的第 r 位是第 r 行；bt_* 的第 c 位是第 c 列；
//   acc_flat_out 的第 (r*Q+c)*NPU_ACC_W +: NPU_ACC_W 位是 PE[r][c] 的累加器。
`include "npu_defines.vh"

module npu_mac(
    input        clk,
    input        rst,
    // 阵列控制(来自 tile_controller)
    input        acc_clear_in,     // 换输出 Tile：清累加器并冲刷残留波前
    input        array_enable_in,  // 喂数或排空期间为 1，其余时间阵列整体冻结
    input        drain_en_in,      // 排空阶段电平
    // A/BT 成对输入流(来自 tile_controller 的 FIFO 出队)
    input        stream_valid_in,
    input [31:0] a_stream_in,
    input [31:0] bt_stream_in,
    input [3:0]  a_lane_en_in,
    input [3:0]  bt_lane_en_in,
    // 结果与状态
    output reg [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_flat_out,
    output reg   drain_done_out
);

    integer i;

    // ---- 输入拆包：一个 32 位字 -> 四个带边界屏蔽的 INT8 lane ----
    reg [7:0] a_lane  [0:3];
    reg [7:0] bt_lane [0:3];

    always @(*) begin
        for (i = 0; i < 4; i = i + 1) begin
            a_lane[i]  = a_lane_en_in[i]  ? a_stream_in[i*8 +: 8]  : 8'h00;
            bt_lane[i] = bt_lane_en_in[i] ? bt_stream_in[i*8 +: 8] : 8'h00;
        end
    end

    // ---- 波前对齐：第 r 行 / 第 c 列分别延迟 r / c 拍 ----
    wire [`NPU_P*8-1:0] a_sk;
    wire [`NPU_P-1:0]   a_sk_v;
    wire [`NPU_Q*8-1:0] bt_sk;
    wire [`NPU_Q-1:0]   bt_sk_v;

    a_bt_skew_pipeline u_skew(
        .clk(clk), .rst(rst), .enable_in(array_enable_in),
        .a_in({a_lane[3], a_lane[2], a_lane[1], a_lane[0]}),
        .a_in_valid(stream_valid_in),
        .bt_in({bt_lane[3], bt_lane[2], bt_lane[1], bt_lane[0]}),
        .bt_in_valid(stream_valid_in),
        .a_out(a_sk), .a_out_valid(a_sk_v),
        .bt_out(bt_sk), .bt_out_valid(bt_sk_v)
    );

    // ---- PE 阵列 ----
    wire [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_w;
    wire drain_done_w;

    pe_array u_pe_array(
        .clk(clk), .rst(rst),
        .enable(array_enable_in), .clear_acc(acc_clear_in),
        .a_lanes_in(a_sk), .a_lane_v_in(a_sk_v),
        .bt_lanes_in(bt_sk), .bt_lane_v_in(bt_sk_v),
        .acc_flat_out(acc_w)
    );

    // ---- 排空计数 ----
    drain_controller u_drain_controller(
        .clk(clk), .rst(rst),
        .drain_en_in(drain_en_in), .drain_done_out(drain_done_w)
    );

    always @(*) begin
        acc_flat_out   = acc_w;
        drain_done_out = drain_done_w;
    end

endmodule
