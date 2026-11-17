`timescale 1ns / 1ps
// npu_buffer —— 数据存储与搬运平面（单模块版，2026-11-17）
//
// 组成：A/BT/C 三块同步 RAM、A/BT 预取 FIFO、C 结果写回 FIFO，以及权重块。
// 原来单独成模块的 c_write_fifo 已经并进来（它只有这一个使用者，端口就是本模块
// 内部连线，留着只能多一层名字）。
//
// 端口所有权：
//   core_busy=0：主机（CPU 或搬运引擎）可以读写三块 Buffer；
//   core_busy=1：A/BT 读端口交给核，C 写口交给写回 FIFO，
//                上游 buffer_access_ctrl / ahb_lite_slave 会挡住主机的 Buffer 访问。
`include "npu_defines.vh"

module npu_buffer(
    input        clk,
    input        rst,
    input        core_busy,
    input        core_bank, // 核正在读的那一组 A/BT，同时也是核正在写的那一组 C
    // 权重块（16KB）：WGT_EN=1 时 A 侧读的是它，地址 = WGT_BASE + a_addr
    input        cpu_wgt_we,
    input [`NPU_WGT_AW-1:0] cpu_wgt_addr,
    input [31:0] cpu_wgt_wdata,
    input [`NPU_WGT_AW-1:0] wgt_base,
    input        wgt_en,
    input        accumulate, // 跨任务累加：C 写回做读-改-写
    // 主机侧端口(来自 npu_top.buffer_access_ctrl 与搬运引擎的二选一)
    input        cpu_a_we,  cpu_a_re,
    input [`NPU_ABUF_REG_AW-1:0] cpu_a_addr,
    input [31:0] cpu_a_wdata,
    input        cpu_bt_we, cpu_bt_re,
    input [`NPU_BBUF_REG_AW-1:0] cpu_bt_addr,
    input [31:0] cpu_bt_wdata,
    input        cpu_c_we,  cpu_c_re,
    input [`NPU_CBUF_REG_AW-1:0] cpu_c_addr,
    input [31:0] cpu_c_wdata,
    // NPU 读请求(来自 tile_controller)
    input [`NPU_ABUF_AW-1:0] npu_a_addr,
    input        npu_a_re,
    input [`NPU_BBUF_AW-1:0] npu_bt_addr,
    input        npu_bt_re,
    input        fifo_pair_pop,
    // C 写回(来自 tile_controller 的 C Tile 写回扫描)
    input        cwr_valid,
    input [`NPU_CBUF_AW-1:0] cwr_addr,
    input [31:0] cwr_data,
    // 主机读数据返回
    output reg [31:0] a_rdata_cpu, bt_rdata_cpu, c_rdata_cpu,
    // 预取 FIFO 接口(到 tile_controller)
    output reg [31:0] a_fifo_rdata,
    output reg [`NPU_FIFO_AW:0] a_fifo_count,
    output reg [31:0] bt_fifo_rdata,
    output reg [`NPU_FIFO_AW:0] bt_fifo_count,
    output reg        cwr_ready, cwr_empty, c_wr_pulse
);

    //====================================================================
    //  A/BT RAM 端口归属
    //  双缓冲：写口地址直接带 bank 位（主机能往任意一组写，包括核没在读的那组）；
    //  读口在运行时锁死在 core_bank 那一组。
    //====================================================================
    reg [`NPU_ABUF_REG_AW-1:0] a_ram_waddr;
    reg a_ram_we;
    reg [`NPU_ABUF_REG_AW-1:0] a_ram_raddr;
    reg a_ram_re;
    always @(*) begin
        a_ram_waddr = cpu_a_addr;
        a_ram_we    = cpu_a_we;
        a_ram_raddr = core_busy ? {core_bank, npu_a_addr} : cpu_a_addr;
        a_ram_re    = core_busy ? npu_a_re                : cpu_a_re;
    end
    wire [31:0] a_ram_rdata;

    // ---- 权重块 RAM：和 A Buffer 同构的同步读（一拍延迟），所以 FIFO 时序不变 ----
    reg [`NPU_WGT_AW-1:0] wgt_raddr;
    always @(*) begin
        wgt_raddr = wgt_base + npu_a_addr;
    end
    wire [31:0] wgt_rdata;
    npu_ram #(.AW(`NPU_WGT_AW)) u_wgt_ram(
        .clk(clk),
        .we(cpu_wgt_we), .waddr(cpu_wgt_addr), .wdata(cpu_wgt_wdata),
        .re(core_busy && wgt_en), .raddr(wgt_raddr), .rdata(wgt_rdata)
    );
    // A 侧数据来源：驻留模式读权重块，否则读 A Buffer
    reg [31:0] a_src_rdata;
    always @(*) begin
        a_src_rdata = (core_busy && wgt_en) ? wgt_rdata : a_ram_rdata;
    end

    npu_ram #(.AW(`NPU_ABUF_REG_AW)) u_a_buffer(
        .clk(clk),
        .we(a_ram_we), .waddr(a_ram_waddr), .wdata(cpu_a_wdata),
        .re(a_ram_re), .raddr(a_ram_raddr), .rdata(a_ram_rdata)
    );

    // ---- BT RAM 端口归属：与 A RAM 同构 ----
    reg [`NPU_BBUF_REG_AW-1:0] bt_ram_waddr;
    reg bt_ram_we;
    reg [`NPU_BBUF_REG_AW-1:0] bt_ram_raddr;
    reg bt_ram_re;
    always @(*) begin
        bt_ram_waddr = cpu_bt_addr;
        bt_ram_we    = cpu_bt_we;
        bt_ram_raddr = core_busy ? {core_bank, npu_bt_addr} : cpu_bt_addr;
        bt_ram_re    = core_busy ? npu_bt_re                 : cpu_bt_re;
    end
    wire [31:0] bt_ram_rdata;

    npu_ram #(.AW(`NPU_BBUF_REG_AW)) u_bt_buffer(
        .clk(clk),
        .we(bt_ram_we), .waddr(bt_ram_waddr), .wdata(cpu_bt_wdata),
        .re(bt_ram_re), .raddr(bt_ram_raddr), .rdata(bt_ram_rdata)
    );

    //====================================================================
    //  C 写回 FIFO（原 c_write_fifo，并进来的）
    //
    //  解耦 C Tile 写回控制与 C RAM 写时序，并实现跨任务累加。
    //  每个条目为 {C 地址, C 数据}；tile_controller 的写回扫描只负责入队，
    //  本段在 C RAM 可用且 FIFO 非空时处理队头：
    //    accumulate=0：直接写 C RAM 并出队（每拍一项）；
    //    accumulate=1：先读 C RAM 的同地址原值，下一拍把 (原值 + 入队值) 写回并出队。
    //                  C RAM 读数据在读请求后一拍返回，所以累加模式每项占两拍。
    //  累加放在这里而不是 PE 里：16 个 PE 累加器被所有输出 Tile 复用，任务结束时它们
    //  保存的是最后一块 Tile 的部分和，不能作为跨任务累加的载体；而 C RAM 天然按
    //  (i,j) 保存每块输出，在写回路径上累加与 Tile 顺序无关。
    //  c_wr_pulse 表示"已经真正写 RAM"，tile_controller 用它统计完成，不能统计入队数。
    //====================================================================
    reg         cwf_rmw_q;   // 累加模式的读-改-写相位：0=发读，1=写回
    reg [`NPU_CBUF_AW-1:0] cwf_head_addr;
    reg [31:0]  cwf_head_data;
    wire [39:0] cwf_rdata;
    wire        cwf_full;
    wire        cwf_empty;
    // C RAM 的读数据在下面才产生，这里先声明（累加模式要用它做原值加）
    wire [31:0] c_ram_rdata;

    always @(*) begin
        cwf_head_addr = cwf_rdata[39:32];
        cwf_head_data = cwf_rdata[31:0];
    end

    // 非累加模式只走写相位；累加模式先读原值再写和。队头在写回那一拍才出队，
    // 读相位期间队头保持不变，地址和数据都稳定。
    reg cwf_do_read, cwf_do_write;
    always @(*) begin
        cwf_do_read  = !cwf_empty && core_busy && accumulate && !cwf_rmw_q;
        cwf_do_write = !cwf_empty && core_busy && (!accumulate || cwf_rmw_q);
    end

    npu_sync_fifo #(.AW(`NPU_CWF_AW), .DW(40)) u_c_write_fifo(
        .clk(clk), .rst(rst),
        .push(cwr_valid && !cwf_full), .wdata({cwr_addr, cwr_data}), .pop(cwf_do_write),
        .rdata(cwf_rdata), .valid(), .empty(cwf_empty), .full(cwf_full), .count()
    );

    always @(posedge clk) begin
        if (rst)
            cwf_rmw_q <= 1'b0;
        else if (!core_busy || cwf_empty)
            cwf_rmw_q <= 1'b0;      // 空闲或队头处理完，下一个队头从读相位开始
        else if (accumulate)
            cwf_rmw_q <= !cwf_rmw_q;
    end

    reg        cwf_we;
    reg [`NPU_CBUF_AW-1:0] cwf_addr;
    reg [31:0] cwf_wdata;
    reg        cwf_re;
    reg [`NPU_CBUF_AW-1:0] cwf_raddr;
    always @(*) begin
        cwf_re    = cwf_do_read;
        cwf_raddr = cwf_head_addr;
        cwf_we    = cwf_do_write;
        cwf_addr  = cwf_head_addr;
        // 累加模式下累加的是**未量化**的原值；量化在写回扫描那一侧已经做过。
        cwf_wdata = (accumulate && cwf_rmw_q) ? (c_ram_rdata + cwf_head_data) : cwf_head_data;
    end

    //====================================================================
    //  C RAM 端口归属
    //  写口在 core_busy 期间归写回 FIFO，否则归主机；
    //  读口平时归主机，只有跨任务累加的读-改-写才借给写回 FIFO。
    //  ★ 这一条是"搬 C 和算下一块重叠"能成立的前提：核在写 bank b 时，主机读 bank ~b，
    //    两个端口不打架。accumulate=1 时要读原值，读口必须还回去，那种任务就不重叠。
    //====================================================================
    reg  [`NPU_CBUF_REG_AW-1:0] c_ram_waddr;
    reg  c_ram_we;
    reg  [31:0] c_ram_wdata;
    reg  [`NPU_CBUF_REG_AW-1:0] c_ram_raddr;
    reg  c_ram_re;
    wire c_rmw = core_busy && accumulate;
    always @(*) begin
        c_ram_waddr = core_busy ? {core_bank, cwf_addr}  : cpu_c_addr;
        c_ram_we    = core_busy ? cwf_we                 : cpu_c_we;
        c_ram_wdata = core_busy ? cwf_wdata              : cpu_c_wdata;
        c_ram_raddr = c_rmw     ? {core_bank, cwf_raddr} : cpu_c_addr;
        c_ram_re    = c_rmw     ? cwf_re                 : cpu_c_re;
    end

    npu_ram #(.AW(`NPU_CBUF_REG_AW)) u_c_buffer(
        .clk(clk),
        .we(c_ram_we), .waddr(c_ram_waddr), .wdata(c_ram_wdata),
        .re(c_ram_re), .raddr(c_ram_raddr), .rdata(c_ram_rdata)
    );

    //====================================================================
    //  A/BT 预取 FIFO
    //  同步 RAM 的读数据比 read enable 晚一拍返回，这里用延迟后的读使能作为 push，
    //  保证 push 的就是当拍返回的字。
    //====================================================================
    reg a_re_d, bt_re_d;
    always @(posedge clk) begin
        if (rst) begin
            a_re_d  <= 1'b0;
            bt_re_d <= 1'b0;
        end
        else begin
            a_re_d  <= npu_a_re  && core_busy;
            bt_re_d <= npu_bt_re && core_busy;
        end
    end

    reg  a_fifo_push, bt_fifo_push;
    wire [31:0] a_fifo_rdata_w, bt_fifo_rdata_w;
    wire [`NPU_FIFO_AW:0] a_fifo_count_w, bt_fifo_count_w;

    always @(*) begin
        a_fifo_push  = a_re_d;
        bt_fifo_push = bt_re_d;
    end

    npu_sync_fifo #(.AW(`NPU_FIFO_AW), .DW(32)) u_a_prefetch_fifo(
        .clk(clk), .rst(rst),
        .push(a_fifo_push), .wdata(a_src_rdata), .pop(fifo_pair_pop),
        .rdata(a_fifo_rdata_w), .valid(), .empty(), .full(),
        .count(a_fifo_count_w)
    );

    npu_sync_fifo #(.AW(`NPU_FIFO_AW), .DW(32)) u_bt_prefetch_fifo(
        .clk(clk), .rst(rst),
        .push(bt_fifo_push), .wdata(bt_ram_rdata), .pop(fifo_pair_pop),
        .rdata(bt_fifo_rdata_w), .valid(), .empty(), .full(),
        .count(bt_fifo_count_w)
    );

    //====================================================================
    //  端口输出
    //====================================================================
    always @(*) begin
        a_rdata_cpu   = a_ram_rdata;
        bt_rdata_cpu  = bt_ram_rdata;
        c_rdata_cpu   = c_ram_rdata;
        a_fifo_rdata  = a_fifo_rdata_w;
        a_fifo_count  = a_fifo_count_w;
        bt_fifo_rdata = bt_fifo_rdata_w;
        bt_fifo_count = bt_fifo_count_w;
        cwr_ready     = !cwf_full;
        cwr_empty     = cwf_empty;
        c_wr_pulse    = cwf_do_write;
    end

endmodule
