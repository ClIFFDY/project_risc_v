`timescale 1ns / 1ps
// pe_cell — 单个处理单元(Processing Element, PE)
//
// A 向右传播、BT 向下传播；acc 是输出驻留累加器。
// enable=0 时所有状态保持，clear_acc 优先级高于 enable，用于切换 Tile 时冲刷波前。
//   有效周期：acc += signed(a_in) * signed(bt_in)
//   a_out/bt_out 为寄存转发(各 1 拍传播延迟)
// 输入数据和 valid 必须保持同拍；两个 valid 同时为 1 才允许累加。
//
// ★ P2（2026-11-12）：为了把乘法推成 DSP48，改了两处【写法】，行为逐拍不变：
//   1. 乘法不再单独放一个组合块（prod_ext），直接写在累加语句里；
//   2. 原来的 rst / clear_acc 两个优先级的清零合并成一根 acc_clear_now。
//   DSP48 只有一个复位端，两个不同优先级的清零会让综合器放弃推断（实测 DSP=0，
//   16 个乘法全在 LUT 里）。改完以综合报告的 DSPs 数是否 = 16 为准。
`include "npu_defines.vh"

module pe_cell(
    input        clk,
    input        rst,
    input        enable,
    input        clear_acc,
    input [7:0]  a_in,
    input        a_v_in,
    input [7:0]  bt_in,
    input        bt_v_in,
    output reg  [7:0]  a_out,
    output reg         a_v_out,
    output reg  [7:0]  bt_out,
    output reg         bt_v_out,
    (* use_dsp = "yes" *) output reg [`NPU_ACC_W-1:0] acc
);

    // 两路清零合成一根：rst 与 clear_acc 对累加器的效果完全一样，只是优先级不同，
    // 而"或"起来以后不存在优先级问题。
    wire acc_clear_now = rst | clear_acc;

    // ★ 乘积必须是 32 位上下文！直接写 acc + $signed(a_in)*$signed(bt_in) 时，
    //   iverilog 按"乘法结果是最大操作数宽度"取 8 位再扩展 ⇒ 结果全错（实测 17/27 挂）。
    //   用一根 32 位 wire 承接，两个仿真器与综合器的语义就一致了。
    wire signed [`NPU_ACC_W-1:0] prod_ext = $signed(a_in) * $signed(bt_in);

    always @(posedge clk) begin
        if (rst) begin
            a_out <= 8'd0; a_v_out <= 1'b0;
            bt_out <= 8'd0; bt_v_out <= 1'b0;
        end else if (clear_acc) begin
            a_v_out <= 1'b0; bt_v_out <= 1'b0;   // 冲刷残留波前
            a_out <= 8'd0;  bt_out <= 8'd0;
        end else if (enable) begin
            a_out <= a_in;   a_v_out <= a_v_in;
            bt_out <= bt_in; bt_v_out <= bt_v_in;
        end
    end

    // 累加器单独一个块、只有一个清零条件；乘加写在同一句里才能折成 MACC。
    always @(posedge clk) begin
        if (acc_clear_now)
            acc <= {`NPU_ACC_W{1'b0}};
        else if (enable && a_v_in && bt_v_in)
            acc <= acc + prod_ext;
        else
            acc <= acc;
    end

endmodule
