`timescale 1ns / 1ps
// result_quantizer — ACC_W 位有符号累加结果到 32 位 C 字
//
// qshift=0 时直接输出；qshift>0 时先加 2^(qshift-1) 做四舍五入，
// 再执行算术右移，最后饱和到有符号 32 位范围 [-2^31, 2^31-1]。
`include "npu_defines.vh"

module result_quantizer(
    input [`NPU_ACC_W-1:0] acc_data,    // 解释为有符号
    input [`NPU_QS_W-1:0]  qshift,
    output reg  [31:0] c_word
);

    // 64 位中间量为舍入加法和负数算术右移提供足够符号位。
    reg signed [63:0] t;

    // 中间量显式扩展到 64 位并按有符号数右移，避免负数被零扩展。
    // 最终结果饱和到有符号 32 位范围，再以 bit pattern 写入 C Buffer。
    always @(*) begin
        if (qshift == 5'd0) begin
            c_word = acc_data;                          // 直通(数值范围已保证不溢出)
        end else begin
            // 64 位中间量做舍入右移(acc_data 按有符号解释,显式符号扩展)
            t = $signed(acc_data);
            t = (t + (64'sd1 << (qshift - 1))) >>> qshift;
            if (t > 64'sd2147483647)        c_word = 32'h7FFF_FFFF;   // 饱和
            else if (t < -64'sd2147483648)  c_word = 32'h8000_0000;
            else                            c_word = t[31:0];
        end
    end

endmodule
