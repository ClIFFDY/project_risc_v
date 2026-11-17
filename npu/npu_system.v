`timescale 1ns / 1ps
// npu_system —— NPU 唯一系统顶层（单模块版，2026-11-17）
//
// 只做端口连接与复位转换，不执行算法。子模块各管一个平面：
//   ahb_lite_slave   上游 AHB-Lite 从口 -> 原生请求（npu_bus.v）
//   npu_top          控制平面：译码、配置、启动校验、Buffer 访问仲裁（npu_top.v）
//   tile_controller  Tile 循环、地址生成、预取/喂数时序、C 写回（npu_ctrl.v）
//   npu_buffer       A/BT/C/权重 RAM、预取 FIFO、写回 FIFO（npu_buffer.v）
//   npu_mac          INT8 拆包、波前对齐、PE 阵列、排空计数（npu_mac.v）
//   npu_dma          搬运引擎：本地口 + AHB-Lite 主口（npu_dma.v）
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
    output             rd_valid,       // 读完成脉冲（本仓互联约定）
    // ---- AHB-Lite 主机接口（搬运引擎的主口，接互联的 m4）----
    // 口径和其余主设备一样：本仓的"地址与数据同拍 + rd_valid 带出读数据"。
    output reg  [31:0] M_HADDR,
    output reg  [1:0]  M_HTRANS,
    output reg         M_HWRITE,
    output reg  [2:0]  M_HSIZE,
    output reg  [2:0]  M_HBURST,
    output reg  [3:0]  M_HPROT,
    output reg  [31:0] M_HWDATA,
    output reg  [3:0]  M_HWSTRB,
    input       [31:0] M_HRDATA,
    input              M_HREADY,
    input              M_HRESP,
    input              M_RD_VALID
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
    wire        sel_dma, sel_bank;
    wire [2:0]  dma_reg_off;
    wire [1:0]  bank_reg_off;
    wire [31:0] control_rdata, status_rdata, buffer_rdata, dma_rdata, bank_rdata;

    // ---- 搬运引擎 ----
    wire        dma_busy, dma_done, dma_error;
    wire        dma_local_valid, dma_local_we;
    wire [31:0] dma_local_addr, dma_local_wdata;
    reg  [31:0] dma_local_rdata;
    // 主机侧 Buffer 端口的最终归属：引擎忙的时候归引擎，否则归 CPU 路径。
    reg         host_a_we, host_a_re, host_bt_we, host_bt_re, host_c_we, host_c_re;
    reg         host_wgt_we;
    reg [`NPU_ABUF_REG_AW-1:0] host_a_addr;
    reg [`NPU_BBUF_REG_AW-1:0] host_bt_addr;
    reg [`NPU_CBUF_REG_AW-1:0] host_c_addr;
    reg [`NPU_WGT_AW-1:0]      host_wgt_addr;
    reg [31:0] host_a_wdata, host_bt_wdata, host_c_wdata, host_wgt_wdata;
    // 本地字节偏移落到哪一块：和 npu_top 的译码同一套窗口（相对 NPU 基址）
    wire dma_sel_a   = (dma_local_addr[31:12] == 20'h00001) && (dma_local_addr[10:8] == 3'd0);
    wire dma_sel_bt  = (dma_local_addr[31:12] == 20'h00002) && (dma_local_addr[10:8] == 3'd0);
    wire dma_sel_c   = (dma_local_addr[31:11] == 21'd6);
    wire dma_sel_wgt = (dma_local_addr[31:15] == 17'd0) && dma_local_addr[14];

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
    wire [`NPU_CBUF_REG_AW-1:0] cpu_c_addr;
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
        .core_busy(core_busy), .core_bank(core_bank), .dma_busy(dma_busy),
        .req_addr(req_addr), .req_wdata(req_wdata),
        .req_byte_en(req_byte_en), .req_we(req_we), .req_valid(req_valid),
        .sel_ctrl(sel_ctrl), .sel_status(sel_status),
        .sel_a_buf(sel_a_buf), .sel_bt_buf(sel_bt_buf), .sel_c_buf(sel_c_buf),
        .sel_wgt(sel_wgt), .sel_wgt_cfg(sel_wgt_cfg),
        .sel_dma(sel_dma), .sel_bank(sel_bank),
        .control_rdata(control_rdata), .status_rdata(status_rdata),
        .buffer_rdata(buffer_rdata), .dma_rdata(dma_rdata), .bank_rdata(bank_rdata)
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
        .sel_dma(sel_dma), .sel_bank(sel_bank),
        .dma_reg_off(dma_reg_off), .bank_reg_off(bank_reg_off),
        .dma_busy(dma_busy), .dma_rdata(dma_rdata),
        .control_rdata(control_rdata), .status_rdata(status_rdata),
        .bank_rdata(bank_rdata),
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

    // ============ npu_dma（搬运引擎）============
    // 它只有两个出口：寄存器口（CPU 经 AHB 从口写）和 AHB 主口（出到互联）。
    // 本地口在下面和 CPU 那条路做二选一。
    npu_dma #(.ADDR_BASE(ADDR_BASE)) u_npu_dma(
        .clk(HCLK), .rst(rst),
        .reg_valid(req_valid), .reg_we(req_we),
        .reg_off(dma_reg_off), .reg_wdata(req_wdata), .reg_rdata(dma_rdata),
        .core_busy(core_busy), .core_bank(core_bank), .core_accumulate(cfg_accumulate),
        .local_valid(dma_local_valid), .local_we(dma_local_we),
        .local_addr(dma_local_addr), .local_wdata(dma_local_wdata),
        .local_rdata(dma_local_rdata),
        .m_haddr(M_HADDR), .m_htrans(M_HTRANS), .m_hwrite(M_HWRITE),
        .m_hsize(M_HSIZE), .m_hburst(M_HBURST), .m_hprot(M_HPROT),
        .m_hwdata(M_HWDATA), .m_hwstrb(M_HWSTRB),
        .m_hrdata(M_HRDATA), .m_hready(M_HREADY), .m_hresp(M_HRESP), .m_rd_valid(M_RD_VALID),
        .busy(dma_busy), .done(dma_done), .error(dma_error)
    );

    // 本地读数据要按**上一拍**的地址选：同步 RAM 的读数据比读使能晚一拍回来。
    // 用当拍地址选会把 A 的字当成 C 的送给引擎（同一类坑在本仓踩过多次）。
    reg  [31:0] dma_read_addr_q;
    always @(posedge HCLK) begin
        if (rst)
            dma_read_addr_q <= 32'd0;
        else if (dma_local_valid && !dma_local_we)
            dma_read_addr_q <= dma_local_addr;
    end
    reg dma_rd_c, dma_rd_bt;
    always @(*) begin
        dma_rd_c   = (dma_read_addr_q[31:11] == 21'd6);
        dma_rd_bt  = (dma_read_addr_q[31:12] == 20'h00002);
        // 权重块没有主机读口（只写），真读到它按 0 处理；描述符校验会把这种请求挡掉。
        dma_local_rdata = dma_rd_c ? c_rdata_cpu : dma_rd_bt ? bt_rdata_cpu : a_rdata_cpu;
    end

    // 主机侧 Buffer 端口：引擎在发本地请求的这一拍整组归它，其余时间归 CPU 路径。
    // 两边不会同时有效 —— 引擎忙的时候 ahb_lite_slave 对 Buffer 访问一律回 ERROR，
    // CPU 那条路上本来就没有有效使能。
    always @(*) begin
        host_a_we     = dma_local_valid ? (dma_local_we && dma_sel_a)   : cpu_a_we;
        host_a_re     = dma_local_valid ? (!dma_local_we && dma_sel_a)  : cpu_a_re;
        host_bt_we    = dma_local_valid ? (dma_local_we && dma_sel_bt)  : cpu_bt_we;
        host_bt_re    = dma_local_valid ? (!dma_local_we && dma_sel_bt) : cpu_bt_re;
        host_c_we     = dma_local_valid ? (dma_local_we && dma_sel_c)   : cpu_c_we;
        host_c_re     = dma_local_valid ? (!dma_local_we && dma_sel_c)  : cpu_c_re;
        host_wgt_we   = dma_local_valid ? (dma_local_we && dma_sel_wgt) : cpu_wgt_we;
        host_a_addr   = dma_local_valid ? {dma_local_addr[11], dma_local_addr[7:2]} : cpu_a_addr;
        host_bt_addr  = dma_local_valid ? {dma_local_addr[11], dma_local_addr[7:2]} : cpu_bt_addr;
        host_c_addr   = dma_local_valid ? {dma_local_addr[10], dma_local_addr[9:2]} : cpu_c_addr;
        host_wgt_addr = dma_local_valid ? dma_local_addr[13:2] : cpu_wgt_addr;
        host_a_wdata  = dma_local_valid ? dma_local_wdata : cpu_a_wdata;
        host_bt_wdata = dma_local_valid ? dma_local_wdata : cpu_bt_wdata;
        host_c_wdata  = dma_local_valid ? dma_local_wdata : cpu_c_wdata;
        host_wgt_wdata= dma_local_valid ? dma_local_wdata : cpu_wgt_wdata;
    end

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
    // 主机侧端口接的是 host_*（CPU 路径和搬运引擎二选一的结果），不是 cpu_*。
    npu_buffer u_npu_buffer(
        .clk(HCLK), .rst(rst), .core_busy(core_busy), .core_bank(core_bank),
        .cpu_wgt_we(host_wgt_we), .cpu_wgt_addr(host_wgt_addr), .cpu_wgt_wdata(host_wgt_wdata),
        .wgt_base(wgt_base), .wgt_en(wgt_en),
        .accumulate(cfg_accumulate),
        .cpu_a_we(host_a_we), .cpu_a_re(host_a_re), .cpu_a_addr(host_a_addr),
        .cpu_a_wdata(host_a_wdata),
        .cpu_bt_we(host_bt_we), .cpu_bt_re(host_bt_re), .cpu_bt_addr(host_bt_addr),
        .cpu_bt_wdata(host_bt_wdata),
        .cpu_c_we(host_c_we), .cpu_c_re(host_c_re), .cpu_c_addr(host_c_addr),
        .cpu_c_wdata(host_c_wdata),
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
