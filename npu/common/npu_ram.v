`timescale 1ns / 1ps
// npu_ram — 通用单时钟 RAM 原语
//
// 接口约定：
//   we=1 时，在当前 posedge 将 wdata 写入 waddr；
//   re=1 时，raddr 在当前 posedge 被采样，rdata 在该沿之后更新。
// 当前实现每拍都执行读端口赋值，因此即使 re=0，rdata 也会反映 raddr
// 对应的存储单元。上层用 re 产生访问请求，用 rdata 的一拍延迟匹配同步 RAM。
`include "npu_defines.vh"

module npu_ram #(parameter AW = 6) (
    input        clk,
    input        we,
    input [AW-1:0] waddr,
    input [31:0] wdata,
    input        re,
    input [AW-1:0] raddr,
    output reg  [31:0] rdata
);

    // 深度为 2^AW，每个地址保存一个 32 位字。
    reg [31:0] mem [0:(1<<AW)-1];
    integer i;

    // re 由各上层模块保留为端口协议信号；本行为模型采用同步异步混合读法，
    // 每拍都把 raddr 对应的内容送入 rdata，真正的请求门控由上层完成。

    initial begin
        for (i = 0; i < (1<<AW); i = i + 1) mem[i] = 32'd0;
        rdata = 32'd0;
    end

    // 非阻塞赋值保证写入和读出都在同一个时钟沿更新，符合同步 RAM 行为。
    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

endmodule
