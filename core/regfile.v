`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: regfile
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module regfile(
    input clk, rst,
    input [9:0] flag_bus,
    input [4:0] r1, r2,
//两个物理写口的请求（仲裁已在 wbu 内完成）：口 A = alu | mul，口 B = ld
    input we_a,
    input [4:0] rd_a,
    input [31:0] data_a,
    input we_b,
    input [4:0] rd_b,
    input [31:0] data_b,
//读数据：合并成一对（原来是 dec/lsu/mul 三份按限定分开填）。按消费者复制交给
//max_fanout 在布局阶段做 —— 比手工拆三份更省逻辑，复制点也更贴实际负载。
    output reg [31:0] r1_data, r2_data
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    reg [31:0] regs [0:31];

    reg [4:0] r1_q, r2_q;

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
        end
    end

//bypass 与写口用同一优先级（口 A > 口 B，即"后面那条 if 写的赢"）：真撞上时
//"写进去的"与"前递出去的"是同一个值。
    function [31:0] bypass;
        input [4:0] rx;
        begin
            if (we_a && rx == rd_a) bypass = data_a;
            else if (we_b && rx == rd_b) bypass = data_b;
            else bypass = regs[rx];
        end
    endfunction

//flag_bus = {exc, exec, flush_irq, flush_jump, dcache_hold, bus_hold_in, stall_m, stall_v, lsu_stall, icache_busy}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[9] | flag_bus[7] | flag_bus[6];
        stall_w = (flag_bus[5] | flag_bus[4] | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//读数据进行双写口(alu/ld)旁路仲裁并输出。
//原来的 dec/lsu/mul 三个限定只用来决定"填哪一份"，合并成一对后不再需要：
//没有限定置位时算出来的值无人消费（JAL 之类），驱出去无害。
    always @(posedge clk) begin
        if (rst_q) begin
            r1_data <= 32'd0;
            r2_data <= 32'd0;
            r1_q <= 5'd0;
            r2_q <= 5'd0;
        end
        else if (exec) begin
            if (stall_w) begin
                r1_data <= bypass(r1_q);
                r2_data <= bypass(r2_q);
            end
            else begin
                r1_data <= bypass(r1);
                r2_data <= bypass(r2);
                r1_q <= r1;
                r2_q <= r2;
            end
        end
    end

//两个物理写口（原来是三个，见 逻辑说明 §27）。仲裁【已搬到 wbu】：本模块只按"准备好的两口"
//写入，语句顺序即优先级（口 A 在后 ⇒ 同 rd 撞上时口 A 赢 = 原来 alu>mul>ld 的口径）。
    always @(posedge clk) begin
        if (we_b) regs[rd_b] <= data_b;
        if (we_a) regs[rd_a] <= data_a;
    end

endmodule
