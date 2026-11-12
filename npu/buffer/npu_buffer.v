`timescale 1ns / 1ps
// npu_buffer — 数据存储与搬运平面
//
// 组成：A/BT/C 三块同步 RAM、A/BT 预取 FIFO、C 结果写回 FIFO。
// 端口所有权：
//   core_busy=0：CPU 可以读写三块 Buffer；
//   core_busy=1：A/BT 读端口交给 NPU，C 写端口交给写回 FIFO，
//               上游 buffer_access_ctrl 会屏蔽 CPU 的 Buffer 访问。
`include "npu_defines.vh"

module npu_buffer(
    input        clk,
    input        rst,
    input        core_busy,
    input        core_bank,          // 核正在读的那一组 A/BT（双缓冲）
    input        accumulate,        // 跨任务累加：C 写回做读-改-写
    // CPU 侧端口(来自 npu_top.buffer_access_ctrl)
    input        cpu_a_we,  cpu_a_re,
    input [`NPU_ABUF_REG_AW-1:0] cpu_a_addr,
    input [31:0] cpu_a_wdata,
    input        cpu_bt_we, cpu_bt_re,
    input [`NPU_BBUF_REG_AW-1:0] cpu_bt_addr,
    input [31:0] cpu_bt_wdata,
    input        cpu_c_we,  cpu_c_re,
    input [`NPU_CBUF_AW-1:0] cpu_c_addr,
    input [31:0] cpu_c_wdata,
    // NPU 读请求(来自 tile_controller)
    input [`NPU_ABUF_AW-1:0] npu_a_addr,
    input        npu_a_re,
    input [`NPU_BBUF_AW-1:0] npu_bt_addr,
    input        npu_bt_re,
    // CPU 读数据返回
    output reg [31:0] a_rdata_cpu, bt_rdata_cpu, c_rdata_cpu,
    // 预取 FIFO 接口(到 tile_controller)
    output reg [31:0] a_fifo_rdata,
    output reg [`NPU_FIFO_AW:0] a_fifo_count,
    input        fifo_pair_pop,
    output reg [31:0] bt_fifo_rdata,
    output reg [`NPU_FIFO_AW:0] bt_fifo_count,
    // C 写回(来自 tile_controller.c_tile_write_ctrl)
    input        cwr_valid,
    input [`NPU_CBUF_AW-1:0] cwr_addr,
    input [31:0] cwr_data,
    output reg        cwr_ready, cwr_empty, c_wr_pulse
);

    // A/BT 在运行期间由 NPU 读，空闲期间由 CPU 读写。
    // 写端口始终来自 CPU；core_busy=1 时 CPU 写请求在上游被禁止，
    // 因此不会出现 CPU 写端口与 NPU 读端口争用同一 RAM 的情况。
    // ---- A RAM 端口归属 ----
    // 双缓冲：写口地址直接带 bank 位（CPU 能往任意一组写，包括核没在读的那组）；
    // 读口在运行时锁死在 core_bank 那一组。
    wire [`NPU_ABUF_REG_AW-1:0] a_ram_waddr = cpu_a_addr;
    wire                        a_ram_we    = cpu_a_we;
    wire [`NPU_ABUF_REG_AW-1:0] a_ram_raddr = core_busy ? {core_bank, npu_a_addr} : cpu_a_addr;
    wire                     a_ram_re    = core_busy ? npu_a_re    : cpu_a_re;
    wire [31:0]              a_ram_rdata;

    npu_ram #(.AW(`NPU_ABUF_REG_AW)) u_a_buffer(
        .clk(clk),
        .we(a_ram_we), .waddr(a_ram_waddr), .wdata(cpu_a_wdata),
        .re(a_ram_re), .raddr(a_ram_raddr), .rdata(a_ram_rdata)
    );
    // ---- BT RAM 端口归属：与 A RAM 同构 ----
    wire [`NPU_BBUF_REG_AW-1:0] bt_ram_waddr = cpu_bt_addr;
    wire                        bt_ram_we    = cpu_bt_we;
    wire [`NPU_BBUF_REG_AW-1:0] bt_ram_raddr = core_busy ? {core_bank, npu_bt_addr} : cpu_bt_addr;
    wire                     bt_ram_re    = core_busy ? npu_bt_re    : cpu_bt_re;
    wire [31:0]              bt_ram_rdata;

    npu_ram #(.AW(`NPU_BBUF_REG_AW)) u_bt_buffer(
        .clk(clk),
        .we(bt_ram_we), .waddr(bt_ram_waddr), .wdata(cpu_bt_wdata),
        .re(bt_ram_re), .raddr(bt_ram_raddr), .rdata(bt_ram_rdata)
    );
    // ---- C RAM 端口归属 ----
    // C RAM 写端在空闲时来自 CPU，运行时来自 c_write_fifo；读端在空闲时归 CPU，
    // 运行时归 c_write_fifo（跨任务累加要读原值做读-改-写）。两者通过 core_busy 互斥，
    // 因为 buffer_access_ctrl 在 core_busy=1 时已经屏蔽了 CPU 的 Buffer 访问。
    wire [`NPU_CBUF_AW-1:0] cwf_addr;   // c_write_fifo -> C RAM
    wire                     cwf_we;
    wire [31:0]              cwf_data;
    wire                     cwf_re;
    wire [`NPU_CBUF_AW-1:0] cwf_raddr;

    wire [`NPU_CBUF_AW-1:0] c_ram_waddr = core_busy ? cwf_addr : cpu_c_addr;
    wire                     c_ram_we    = core_busy ? cwf_we    : cpu_c_we;
    wire [31:0]              c_ram_wdata = core_busy ? cwf_data  : cpu_c_wdata;
    wire [`NPU_CBUF_AW-1:0] c_ram_raddr = core_busy ? cwf_raddr : cpu_c_addr;
    wire                     c_ram_re    = core_busy ? cwf_re    : cpu_c_re;
    wire [31:0]              c_ram_rdata;

    npu_ram #(.AW(`NPU_CBUF_AW)) u_c_buffer(
        .clk(clk),
        .we(c_ram_we), .waddr(c_ram_waddr), .wdata(c_ram_wdata),
        .re(c_ram_re), .raddr(c_ram_raddr), .rdata(c_ram_rdata)
    );
    // 同步 RAM 的读数据比 read enable 晚一拍返回；在这里延迟读使能，
    // 让 FIFO push 与 RAM 返回数据保持同拍。
    // ---- NPU 读端口控制:同步读返回 -> FIFO ----
    reg  a_fifo_push, bt_fifo_push;
    wire [31:0] a_fifo_rdata_w, bt_fifo_rdata_w;
    wire [`NPU_FIFO_AW:0] a_fifo_count_w, bt_fifo_count_w;
    wire cwr_ready_w, cwr_empty_w, c_wr_pulse_w;

    reg a_re_d, bt_re_d;

    always @(posedge clk) begin
        if (rst) begin
            a_re_d  <= 1'b0;
            bt_re_d <= 1'b0;
        end else begin
            a_re_d  <= npu_a_re  && core_busy;
            bt_re_d <= npu_bt_re && core_busy;
        end
    end

    npu_sync_fifo #(.AW(`NPU_FIFO_AW), .DW(32)) u_a_prefetch_fifo(
        .clk(clk), .rst(rst),
        .push(a_fifo_push), .wdata(a_ram_rdata), .pop(fifo_pair_pop),
        .rdata(a_fifo_rdata_w), .valid(), .empty(), .full(),
        .count(a_fifo_count_w)
    );

    npu_sync_fifo #(.AW(`NPU_FIFO_AW), .DW(32)) u_bt_prefetch_fifo(
        .clk(clk), .rst(rst),
        .push(bt_fifo_push), .wdata(bt_ram_rdata), .pop(fifo_pair_pop),
        .rdata(bt_fifo_rdata_w), .valid(), .empty(), .full(),
        .count(bt_fifo_count_w)
    );

    // C 写 FIFO 将结果写回和 C RAM 的实际写脉冲解耦；tile_controller 统计 c_wr_pulse。
    // ---- C 写 FIFO ----
    c_write_fifo u_c_write_fifo(
        .clk(clk), .rst(rst), .core_busy(core_busy),
        .accumulate(accumulate),
        .push(cwr_valid), .waddr(cwr_addr), .wdata(cwr_data),
        .ready(cwr_ready_w), .empty(cwr_empty_w),
        .ram_we(cwf_we), .ram_addr(cwf_addr), .ram_wdata(cwf_data),
        .ram_re(cwf_re), .ram_raddr(cwf_raddr), .ram_rdata(c_ram_rdata),
        .c_wr_pulse(c_wr_pulse_w)
    );

    always @(*) begin
        // 同步 RAM 读数据比读使能晚一拍返回，这里用延迟后的读使能作为 FIFO push，
        // 保证 push 的数据就是当拍返回的字。
        a_fifo_push  = a_re_d;
        bt_fifo_push = bt_re_d;
        a_rdata_cpu = a_ram_rdata;
        bt_rdata_cpu = bt_ram_rdata;
        c_rdata_cpu = c_ram_rdata;
        a_fifo_rdata = a_fifo_rdata_w;
        a_fifo_count = a_fifo_count_w;
        bt_fifo_rdata = bt_fifo_rdata_w;
        bt_fifo_count = bt_fifo_count_w;
        cwr_ready = cwr_ready_w;
        cwr_empty = cwr_empty_w;
        c_wr_pulse = c_wr_pulse_w;
    end

endmodule
