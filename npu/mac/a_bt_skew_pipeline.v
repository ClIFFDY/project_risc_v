`timescale 1ns / 1ps
// a_bt_skew_pipeline — 脉动阵列输入波前对齐
//
// A 的第 r 个 lane（行）延迟 r 拍，BT 的第 c 个 lane（列）延迟 c 拍，
// 且数据和 valid 使用完全相同的寄存器级数。这样 A[r][k] 与 BT[c][k]
// 才会在 PE[r][c] 的输入端同一拍到达。
// enable_in=0 时所有级保持，暂停不会丢失也不会错位波前（整条管线一起冻结）。
// lane 0 直通不占寄存器；只有 lane 1..P-1 / 1..Q-1 需要 1..N-1 级寄存器。
`include "npu_defines.vh"

module a_bt_skew_pipeline(
    input        clk,
    input        rst,
    input        enable_in,
    // A 侧输入：lane r 在 [r*8 +: 8]，a_in_valid 对整个 32 位字有效
    input [`NPU_P*8-1:0] a_in,
    input        a_in_valid,
    // BT 侧输入：lane c 在 [c*8 +: 8]
    input [`NPU_Q*8-1:0] bt_in,
    input        bt_in_valid,
    // A 侧输出：lane r 在 [r*8 +: 8]，valid 每 lane 一位
    output reg [`NPU_P*8-1:0] a_out,
    output reg [`NPU_P-1:0]   a_out_valid,
    // BT 侧输出
    output reg [`NPU_Q*8-1:0] bt_out,
    output reg [`NPU_Q-1:0]   bt_out_valid
);

    // [lane][stage]：lane l 只用 stage 0..l-1，输出取 stage l-1，即 l 拍延迟。
    reg [7:0] a_sh  [0:`NPU_P-1][0:`NPU_P-1];
    reg       a_shv [0:`NPU_P-1][0:`NPU_P-1];
    reg [7:0] b_sh  [0:`NPU_Q-1][0:`NPU_Q-1];
    reg       b_shv [0:`NPU_Q-1][0:`NPU_Q-1];

    integer l, s;

    always @(posedge clk) begin
        if (rst) begin
            for (l = 1; l < `NPU_P; l = l + 1)
                for (s = 0; s < l; s = s + 1) begin
                    a_sh[l][s]  <= 8'd0;
                    a_shv[l][s] <= 1'b0;
                end
            for (l = 1; l < `NPU_Q; l = l + 1)
                for (s = 0; s < l; s = s + 1) begin
                    b_sh[l][s]  <= 8'd0;
                    b_shv[l][s] <= 1'b0;
                end
        end else if (enable_in) begin
            // 数据与 valid 必须一起延迟，否则无效数据会被 PE 累加。
            for (l = 1; l < `NPU_P; l = l + 1) begin
                a_sh[l][0]  <= a_in[l*8 +: 8];
                a_shv[l][0] <= a_in_valid;
                for (s = 1; s < l; s = s + 1) begin
                    a_sh[l][s]  <= a_sh[l][s-1];
                    a_shv[l][s] <= a_shv[l][s-1];
                end
            end
            for (l = 1; l < `NPU_Q; l = l + 1) begin
                b_sh[l][0]  <= bt_in[l*8 +: 8];
                b_shv[l][0] <= bt_in_valid;
                for (s = 1; s < l; s = s + 1) begin
                    b_sh[l][s]  <= b_sh[l][s-1];
                    b_shv[l][s] <= b_shv[l][s-1];
                end
            end
        end
    end

    always @(*) begin
        a_out       = {`NPU_P*8{1'b0}};
        a_out_valid = {`NPU_P{1'b0}};
        bt_out       = {`NPU_Q*8{1'b0}};
        bt_out_valid = {`NPU_Q{1'b0}};

        // lane 0 旁路，lane l 取自己的第 l 级。
        a_out[7:0]       = a_in[7:0];
        a_out_valid[0]   = a_in_valid;
        bt_out[7:0]      = bt_in[7:0];
        bt_out_valid[0]  = bt_in_valid;

        for (l = 1; l < `NPU_P; l = l + 1) begin
            a_out[l*8 +: 8]  = a_sh[l][l-1];
            a_out_valid[l]   = a_shv[l][l-1];
        end
        for (l = 1; l < `NPU_Q; l = l + 1) begin
            bt_out[l*8 +: 8] = b_sh[l][l-1];
            bt_out_valid[l]  = b_shv[l][l-1];
        end
    end

endmodule
