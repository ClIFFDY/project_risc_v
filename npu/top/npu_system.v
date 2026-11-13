`timescale 1ns / 1ps
// npu_system — NPU 唯一系统顶层
//
// 对外只有一条总线：ARM AHB-Lite 从机。ahb_lite_slave 把 AHB 传输翻译成
// 内部原生请求（req_*），npu_top 负责译码和控制平面，其余三个平面照旧。
//
// 四个功能平面的边界固定在这里，顶层只负责端口连接，不执行算法：
//   ahb_lite_slave   AHB-Lite 协议 -> 原生请求，错误响应与等待态；
//   npu_top          CPU 请求译码、配置寄存器、启动校验和 CPU/Buffer 访问仲裁；
//   tile_controller  Tile 循环、地址生成、预取/喂数时序和 C 写回；
//   npu_buffer       A/BT/C RAM、A/BT 预取 FIFO 和 C 写 FIFO；
//   npu_mac          INT8 拆包、波前对齐、PE 阵列和排空计数。
// 控制路径：AHB -> ahb_lite_slave -> npu_top -> tile_controller -> npu_mac。
// 数据路径：A/BT Buffer -> 预取 FIFO -> MAC 累加器 -> 结果快照 -> C 写 FIFO -> C Buffer。
//
// 单从机系统里 HREADY 就是 HREADYOUT，由上层互连回接；多从机时接互连的
// 全局 HREADY。HRESETn 是低有效异步复位，内部统一转成高有效 rst（与项目
// 其余模块的复位约定一致）；上板时 HRESETn 的释放沿应与 HCLK 同步。
`include "npu_defines.vh"

module npu_system #(
    parameter [31:0] ADDR_BASE = `NPU_ADDR_BASE
)(
    input              HCLK,
    input              HRESETn,        // AHB 复位，低有效
    // ---- AHB-Lite 从机接口 ----
    input              HSEL,
    input       [31:0] HADDR,
    input       [1:0]  HTRANS,
    input              HWRITE,
    input       [2:0]  HSIZE,
    input       [31:0] HWDATA,
    input              HREADY,         // 互连回送的全局 HREADY
    output reg  [31:0] HRDATA,
    output reg         HREADYOUT,
    output reg  [1:0]  HRESP,
    output             rd_valid        // 读完成脉冲（本仓互联约定）
);

    // 内部高有效复位：全项目统一使用 if (rst) 的同步高有效复位（编码规范 4.1，
    // 不允许模块内部出现异步低有效 rst_n 形式）。AHB 的 HRESETn 只在本模块的
    // 端口上出现，进入内部即转换为 rst。上板时 HRESETn 的释放沿应与 HCLK 同步。
    reg rst;
    always @(*) begin
        rst = !HRESETn;
    end

    // ahb_lite_slave 的输出先接内部 wire，再在 always @(*) 里转发到端口：
    // Verilog-2001 不允许实例输出直接驱动 reg 端口(见编码规范 2.1/5)。
    wire [31:0] hrdata_w;
    wire        hreadyout_w;
    wire [1:0]  hresp_w;

    // ahb_lite_slave 与 npu_top 之间的原生请求接口。
    wire [31:0] req_addr, req_wdata;
    wire [3:0]  req_byte_en;
    wire        req_we, req_valid;
    wire        sel_ctrl, sel_status, sel_a_buf, sel_bt_buf, sel_c_buf;
    wire [31:0] control_rdata, status_rdata, buffer_rdata;

    // npu_top 与 tile_controller 之间的任务级控制接口。
    wire        start_pulse;
    wire [`NPU_DIM_W-1:0]  cfg_m, cfg_n, cfg_k;
    wire [`NPU_TK_W-1:0]   cfg_tk;
    wire [`NPU_QS_W-1:0]   cfg_qshift;
    wire                   cfg_accumulate;
    wire        core_busy, core_done;

    // npu_top 与 npu_buffer 之间的 CPU 端口。
    wire        cpu_a_we, cpu_a_re, cpu_bt_we, cpu_bt_re, cpu_c_we, cpu_c_re;
    wire [`NPU_ABUF_REG_AW-1:0] cpu_a_addr;
    wire [`NPU_BBUF_REG_AW-1:0] cpu_bt_addr;
    wire        core_bank;
    wire        cpu_wgt_we;
    wire [`NPU_WGT_AW-1:0] cpu_wgt_addr;
    wire [31:0] cpu_wgt_wdata;
    wire [`NPU_WGT_AW-1:0] wgt_base;
    wire        wgt_en;
    wire        sel_wgt, sel_wgt_cfg;
    wire [`NPU_CBUF_AW-1:0] cpu_c_addr;
    wire [31:0] cpu_a_wdata, cpu_bt_wdata, cpu_c_wdata;
    wire [31:0] a_rdata_cpu, bt_rdata_cpu, c_rdata_cpu;

    // tile_controller 发起 A/BT 读请求，npu_buffer 返回同步 RAM 数据和 FIFO 状态。
    wire [`NPU_ABUF_AW-1:0] npu_a_addr;
    wire        npu_a_re;
    wire [`NPU_BBUF_AW-1:0] npu_bt_addr;
    wire        npu_bt_re;
    wire        fifo_pair_pop;
    wire [31:0] a_fifo_rdata, bt_fifo_rdata;
    wire [`NPU_FIFO_AW:0] a_fifo_count, bt_fifo_count;

    // tile_controller 到 npu_mac 的阵列控制、输入流，以及 npu_mac 回到
    // tile_controller 的累加器快照和排空完成。
    wire        acc_clear, array_enable, drain_en;
    wire        stream_valid;
    wire [31:0] a_stream_data, bt_stream_data;
    wire [3:0]  a_lane_en, bt_lane_en;
    wire [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_flat;
    wire        drain_done;

    // tile_controller 到 npu_buffer 的 C 写回 FIFO 接口。
    wire        cwr_valid, cwr_ready, cwr_empty, c_wr_pulse;
    wire [`NPU_CBUF_AW-1:0] cwr_addr;
    wire [31:0] cwr_data;

    // ============ ahb_lite_slave ============
    ahb_lite_slave #(.ADDR_BASE(ADDR_BASE)) u_ahb_lite_slave(
        .clk(HCLK), .rst(rst),
        .HSEL(HSEL), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE),
        .HSIZE(HSIZE), .HWDATA(HWDATA), .HREADY_in(HREADY),
        .HRDATA(hrdata_w), .HREADYOUT(hreadyout_w), .HRESP(hresp_w), .rd_valid(rd_valid),
        .core_busy(core_busy), .core_bank(core_bank),
        .req_addr(req_addr), .req_wdata(req_wdata),
        .req_byte_en(req_byte_en), .req_we(req_we), .req_valid(req_valid),
        .sel_ctrl(sel_ctrl), .sel_status(sel_status),
        .sel_a_buf(sel_a_buf), .sel_bt_buf(sel_bt_buf), .sel_c_buf(sel_c_buf),
        .sel_wgt(sel_wgt), .sel_wgt_cfg(sel_wgt_cfg),
        .control_rdata(control_rdata), .status_rdata(status_rdata),
        .buffer_rdata(buffer_rdata)
    );

    always @(*) begin
        HRDATA    = hrdata_w;
        HREADYOUT = hreadyout_w;
        HRESP     = hresp_w;
    end

    // ============ npu_top ============
    npu_top u_npu_top(
        .clk(HCLK), .rst(rst),
        .req_addr(req_addr), .req_wdata(req_wdata),
        .req_byte_en(req_byte_en), .req_we(req_we), .req_valid(req_valid),
        .sel_ctrl(sel_ctrl), .sel_status(sel_status),
        .sel_a_buf(sel_a_buf), .sel_bt_buf(sel_bt_buf), .sel_c_buf(sel_c_buf),
        .control_rdata(control_rdata), .status_rdata(status_rdata),
        .buffer_rdata(buffer_rdata),
        .start_pulse(start_pulse),
        .cfg_m(cfg_m), .cfg_n(cfg_n), .cfg_k(cfg_k),
        .cfg_tk(cfg_tk), .cfg_qshift(cfg_qshift), .cfg_accumulate(cfg_accumulate),
        .core_busy(core_busy), .core_done(core_done),
        .cpu_a_we(cpu_a_we), .cpu_a_re(cpu_a_re),
        .cpu_bt_we(cpu_bt_we), .cpu_bt_re(cpu_bt_re),
        .cpu_c_we(cpu_c_we), .cpu_c_re(cpu_c_re),
        .cpu_a_addr(cpu_a_addr), .cpu_bt_addr(cpu_bt_addr), .cpu_c_addr(cpu_c_addr),
        .cpu_a_wdata(cpu_a_wdata), .cpu_bt_wdata(cpu_bt_wdata), .cpu_c_wdata(cpu_c_wdata),
        .a_rdata_cpu(a_rdata_cpu), .bt_rdata_cpu(bt_rdata_cpu), .c_rdata_cpu(c_rdata_cpu),
        .core_bank(core_bank),
        .cpu_wgt_we(cpu_wgt_we), .cpu_wgt_addr(cpu_wgt_addr), .cpu_wgt_wdata(cpu_wgt_wdata),
        .wgt_base(wgt_base), .wgt_en(wgt_en),
        .sel_wgt(sel_wgt), .sel_wgt_cfg(sel_wgt_cfg)
    );

    // ============ tile_controller ============
    tile_controller u_tile_controller(
        .clk(HCLK), .rst(rst),
        .start_pulse(start_pulse),
        .cfg_m(cfg_m), .cfg_n(cfg_n), .cfg_k(cfg_k),
        .cfg_tk(cfg_tk), .cfg_qshift(cfg_qshift),
        .a_addr(npu_a_addr), .a_re(npu_a_re),
        .bt_addr(npu_bt_addr), .bt_re(npu_bt_re),
        .a_fifo_rdata(a_fifo_rdata),
        .a_fifo_count(a_fifo_count), .fifo_pair_pop(fifo_pair_pop),
        .bt_fifo_rdata(bt_fifo_rdata),
        .bt_fifo_count(bt_fifo_count),
        .acc_clear(acc_clear), .array_enable(array_enable), .drain_en(drain_en),
        .stream_valid(stream_valid),
        .a_stream_data(a_stream_data), .bt_stream_data(bt_stream_data),
        .a_lane_en(a_lane_en), .bt_lane_en(bt_lane_en),
        .acc_flat_in(acc_flat), .drain_done(drain_done),
        .cwr_valid(cwr_valid), .cwr_addr(cwr_addr), .cwr_data(cwr_data),
        .cwr_ready(cwr_ready), .cwr_empty(cwr_empty), .c_wr_pulse(c_wr_pulse),
        .core_busy(core_busy), .core_done(core_done)
    );

    // ============ npu_buffer ============
    npu_buffer u_npu_buffer(
        .clk(HCLK), .rst(rst), .core_busy(core_busy), .core_bank(core_bank),
        .cpu_wgt_we(cpu_wgt_we), .cpu_wgt_addr(cpu_wgt_addr), .cpu_wgt_wdata(cpu_wgt_wdata),
        .wgt_base(wgt_base), .wgt_en(wgt_en),
        .accumulate(cfg_accumulate),
        .cpu_a_we(cpu_a_we), .cpu_a_re(cpu_a_re), .cpu_a_addr(cpu_a_addr),
        .cpu_a_wdata(cpu_a_wdata),
        .cpu_bt_we(cpu_bt_we), .cpu_bt_re(cpu_bt_re), .cpu_bt_addr(cpu_bt_addr),
        .cpu_bt_wdata(cpu_bt_wdata),
        .cpu_c_we(cpu_c_we), .cpu_c_re(cpu_c_re), .cpu_c_addr(cpu_c_addr),
        .cpu_c_wdata(cpu_c_wdata),
        .npu_a_addr(npu_a_addr), .npu_a_re(npu_a_re),
        .npu_bt_addr(npu_bt_addr), .npu_bt_re(npu_bt_re),
        .a_rdata_cpu(a_rdata_cpu), .bt_rdata_cpu(bt_rdata_cpu), .c_rdata_cpu(c_rdata_cpu),
        .a_fifo_rdata(a_fifo_rdata),
        .a_fifo_count(a_fifo_count), .fifo_pair_pop(fifo_pair_pop),
        .bt_fifo_rdata(bt_fifo_rdata),
        .bt_fifo_count(bt_fifo_count),
        .cwr_valid(cwr_valid), .cwr_addr(cwr_addr), .cwr_data(cwr_data),
        .cwr_ready(cwr_ready), .cwr_empty(cwr_empty), .c_wr_pulse(c_wr_pulse)
    );

    // ============ npu_mac ============
    npu_mac u_npu_mac(
        .clk(HCLK), .rst(rst),
        .acc_clear_in(acc_clear),
        .array_enable_in(array_enable),
        .drain_en_in(drain_en),
        .stream_valid_in(stream_valid),
        .a_stream_in(a_stream_data), .bt_stream_in(bt_stream_data),
        .a_lane_en_in(a_lane_en), .bt_lane_en_in(bt_lane_en),
        .acc_flat_out(acc_flat), .drain_done_out(drain_done)
    );

endmodule
