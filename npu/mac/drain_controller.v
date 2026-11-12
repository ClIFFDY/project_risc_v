`timescale 1ns / 1ps
// drain_controller — 阵列波前排空计时器
//
// 1) 模块位置和职责
//    npu_mac 的子模块。最后一对输入送进阵列后，波前还要走 P+Q-2 个 enable 拍才能
//    到达最远端的 PE；这段时间里不能再进新数据，否则尾部乘积会落到下一块结果里。
//    本模块给上层"什么时候可以换输出 Tile"的计时依据。
//
// 2) 接口与单位
//    drain_en_in 是电平，保持几拍就数几拍；drain_done_out 是电平，在第 DRAIN_BEATS
//    拍拉高并保持到 drain_en_in 撤销。计数单位是时钟拍。
//
// 3) 时序延迟
//    drain_en_in 拉高的第 1 拍起计数，第 DRAIN_BEATS+1 拍给出 drain_done_out
//    （多出的那一拍用于让最远端 PE 的最后一个乘积先落进累加器，见第 5 条）。
//    drain_en_in=0 时计数器和输出立即复位，所以每个输出 Tile 都是一次全新的排空计时。
//
// 4) 复位后的状态
//    复位后 cnt_q=0、drain_done_out=0。
//
// 5) 边界条件、错误处理和不能修改的接口假设
//    · 为什么要在第 DRAIN_BEATS+1 拍才拉高：cnt_q 在第 1 个 enable 拍是 0，之后
//      每拍加一，比较 cnt_q == DRAIN_BEATS-1 让 drain_done_out 落在第
//      DRAIN_BEATS+1 拍。这一拍不是冗余——上层正是在 drain_done 那一拍把累加器
//      采进快照，而最远端 PE[P-1][Q-1] 的最后一个乘积恰好在这一拍的前一个时钟沿
//      才写进累加器；提前一拍采数会丢掉远角的那一个乘积（实测只有 C[3][3] 一类
//      的远角元素变小的错法）。
//    · drain_done_out 是电平不是单拍脉冲，上层必须用"只消费一次"的方式使用它。
`include "npu_defines.vh"

module drain_controller(
    input        clk,
    input        rst,
    input        drain_en_in,
    output reg   drain_done_out
);

    localparam [3:0] DRAIN_BEATS = `NPU_P + `NPU_Q - 2;

    reg [3:0] cnt_q;

    always @(posedge clk) begin
        if (rst) begin
            cnt_q          <= 4'd0;
            drain_done_out <= 1'b0;
        end else if (!drain_en_in) begin
            cnt_q          <= 4'd0;
            drain_done_out <= 1'b0;
        end else if (!drain_done_out) begin
            if (cnt_q == DRAIN_BEATS - 4'd1)
                drain_done_out <= 1'b1;
            else
                cnt_q <= cnt_q + 4'd1;
        end
    end

endmodule
