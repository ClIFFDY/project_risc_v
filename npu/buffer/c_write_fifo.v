`timescale 1ns / 1ps
// c_write_fifo — 解耦 C tile 写回控制与 C RAM 写时序，并实现跨任务累加
//
// FIFO 每个条目为 {C 地址, C 数据}。c_tile_write_ctrl 只负责把结果入队；
// 本模块在 C RAM 可用且 FIFO 非空时处理队头：
//   accumulate=0：直接写 C RAM 并出队（每拍一项）；
//   accumulate=1：先读 C RAM 的同地址原值，下一拍把 (原值 + 入队值) 写回并出队。
//                 C RAM 读数据在读请求后一拍返回（见 npu_ram），所以累加模式下
//                 每项占两拍。
// 累加放在这里而不是 PE 里：16 个 PE 累加器被所有输出 Tile 复用，任务结束时它们
// 保存的是最后一块 Tile 的部分和，不能作为跨任务累加的载体；而 C RAM 天然按
// (i,j) 保存每块输出，在写回路径上累加与 Tile 顺序无关。
// c_wr_pulse 表示"已经真正写 RAM"，tile_controller 用它统计完成，
// 不能统计 FIFO 入队数。累加模式下读那一拍不计入。
`include "npu_defines.vh"

module c_write_fifo(
    input        clk,
    input        rst,
    input        core_busy,
    input        accumulate,          // 1：把入队值加到 C RAM 原值上
    // 来自 c_tile_write_ctrl
    input        push,
    input [`NPU_CBUF_AW-1:0] waddr,
    input [31:0] wdata,
    output reg        ready,          // !full
    output reg        empty,
    // 到 npu_buffer 内部的 C RAM
    output reg        ram_we,
    output reg [`NPU_CBUF_AW-1:0] ram_addr,
    output reg [31:0] ram_wdata,
    output reg        ram_re,         // 累加模式下读原值
    output reg [`NPU_CBUF_AW-1:0] ram_raddr,
    input [31:0] ram_rdata,           // C RAM 读数据（读请求后一拍）
    // 到 tile_controller：真正写入 C RAM 的脉冲
    output reg        c_wr_pulse
);

    wire [39:0] rdata;
    wire        full;
    wire        empty_w;
    reg         rmw_q;      // 累加模式的读-改-写相位：0=发读，1=写回

    wire [`NPU_CBUF_AW-1:0] head_addr = rdata[39:32];
    wire [31:0]             head_data = rdata[31:0];

    // 非累加模式只走写相位；累加模式先读原值再写和。队头在写回那一拍才出队，
    // 读相位期间队头保持不变，地址和数据都稳定。
    wire do_read  = !empty_w && core_busy && accumulate && !rmw_q;
    wire do_write = !empty_w && core_busy && (!accumulate || rmw_q);

    npu_sync_fifo #(.AW(`NPU_CWF_AW), .DW(40)) u_fifo(
        .clk(clk), .rst(rst),
        .push(push && !full), .wdata({waddr, wdata}), .pop(do_write),
        .rdata(rdata), .valid(), .empty(empty_w), .full(full), .count()
    );

    always @(posedge clk) begin
        if (rst)
            rmw_q <= 1'b0;
        else if (!core_busy || empty_w)
            rmw_q <= 1'b0;             // 空闲或队头处理完，下一个队头从读相位开始
        else if (accumulate)
            rmw_q <= !rmw_q;
    end

    always @(*) begin
        ready      = !full;
        empty      = empty_w;
        ram_re     = do_read;
        ram_raddr  = head_addr;
        ram_we     = do_write;
        ram_addr   = head_addr;
        ram_wdata  = (accumulate && rmw_q) ? (ram_rdata + head_data) : head_data;
        c_wr_pulse = do_write;
    end

endmodule
