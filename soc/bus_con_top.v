`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: bus_con_top
// Description: 总线汇聚层（mcu_top 里 cpu_top 之外的那一半）。
//
//   2026-11-12 多主改造：
//     · 互联从【单主直通】换成四主四从（旧的单主版已删，ahb_interconnect 现在就是四主的）；
//     · 新增 DDR/AXI 桥（AHB 从设备 s3，0x8000_0000 起 1GB），AXI4 主口出到模块外面，
//       下游 MIG 7 系列还没接（2026-11-12）；
//     · 新增 STCM（64KB 模拟外存，AHB 从设备 s1）作为 DMA 的搬运对象；
//     · 新增 DMA（AHB 主设备 m2），寄存器口挂 3 号槽 0x0C00_0000。
//     · m1 是 L2 的 AHB 主口（cpu_top 里的 l2cache，从 mcu_top 引过来）。
//
//   地址口径（全树只有这一种）：
//     CPU 字节地址 >> 2 = 本模块 bus_addr_f_cpu（字地址）
//     lsu_ahb_master 再 << 2 还原成 AHB 字节地址 haddr
//     互联按地址 slot 选从设备：STCM(0x0000_xxxx) / NPU(0x2000_xxxx) / DDR(0x8000_0000-0xBFFF_FFFF) / 其余 APB 桥
//     桥再 >> 2 吐给外设，外设按 bus_addr_in[31:26] 认槽位
//       1 UART / 2 PLIC / 3 DMA / 4 GPIO / 5 I2C / 6 TIM(PWM)
//////////////////////////////////////////////////////////////////////////////////


module bus_con_top #(
//DDR_PRESENT=0：下游没有存储器，桥对任何访问直接回 ERROR（AXI 口闭嘴，留着给 MIG）
    parameter integer DDR_PRESENT = 0
) (
    input clk, rst,
    input      [31:0] bus_addr_f_cpu,
    input      [31:0] bus_data_f_cpu,
    input      [3:0]  bus_be_f_cpu,
    input             bus_we_f_cpu,
    input             bus_valid_f_cpu,
    output reg        rst_cpu,        // CPU / DMA 域：上电搬运期间一直压着
    output reg        rst_sys,        // 总线 / STCM / APB 桥 / SPI 域：复位一放开就活
//---- boot 引擎状态（给 pmem_spi 的 BOOT 寄存器展示）----
    output reg        boot_done_out,
    output reg        boot_err_out,
    output reg [16:0] boot_count_out,
    output     [31:0] bus_data_b_cpu,
    output            bus_loaded_out,
    output            bus_hold_out,
    output     [31:0] bus_addr_out,
    output     [31:0] bus_data_out,
    output     [3:0]  bus_be_out,
    output            bus_we_out,
    output     [31:0] bus_sel_out,
    input      [31:0] uart_data,
    input             uart_ready,
    input      [31:0] plic_data,
    input             plic_ready,
    input      [31:0] gpio_data,
    input             gpio_ready,
    input      [31:0] i2c_data,
    input             i2c_ready,
    input      [31:0] tim_data,
    input             tim_ready,
    input      [31:0] spi_data,
    input             spi_ready,
//---- m1：L2（cpu_top 里的 64KB 统一二级缓存）----
    input      [31:0] m1_haddr,
    input      [1:0]  m1_htrans,
    input             m1_hwrite,
    input      [2:0]  m1_hsize,
    input      [2:0]  m1_hburst,
    input      [3:0]  m1_hprot,
    input             m1_hmastlock,
    input      [31:0] m1_hwdata,
    input      [3:0]  m1_hwstrb,
    output     [31:0] m1_hrdata,
    output            m1_hready,
    output            m1_hresp,
    output            m1_rd_valid,
    output            m1_hgrant,
    output            dma_irq,
//---- 错误日志（寄存器实体在 plic.v，这里只做汇总）----
    output            err_valid,
    output            err_write,
    output     [1:0]  err_owner,
    output     [31:0] err_addr,
    input             err_clear,
//---- s3：DDR/AXI 主设备口（下游就是 MIG 7 系列的 AXI4 从口）----
//  ★ 2026-11-12：桥已经挂上（s3，0x8000_0000-0xBFFF_FFFF），MIG 还没进来。
//    现在这组口在 mcu_top 外面空着：任何访问 DDR 的总线操作都会一直等，
//    ddr_mem_ready 拉低可以把它挡在 S_WAIT_MEM 里（同样是等）。
    input             ddr_mem_ready,
    output     [3:0]  ddr_axi_awid,
    output     [31:0] ddr_axi_awaddr,
    output     [7:0]  ddr_axi_awlen,
    output     [2:0]  ddr_axi_awsize,
    output     [1:0]  ddr_axi_awburst,
    output            ddr_axi_awlock,
    output     [3:0]  ddr_axi_awcache,
    output     [2:0]  ddr_axi_awprot,
    output     [3:0]  ddr_axi_awqos,
    output            ddr_axi_awvalid,
    input             ddr_axi_awready,
    output     [127:0] ddr_axi_wdata,
    output     [15:0] ddr_axi_wstrb,
    output            ddr_axi_wlast,
    output            ddr_axi_wvalid,
    input             ddr_axi_wready,
    input      [3:0]  ddr_axi_bid,
    input      [1:0]  ddr_axi_bresp,
    input             ddr_axi_bvalid,
    output            ddr_axi_bready,
    output     [3:0]  ddr_axi_arid,
    output     [31:0] ddr_axi_araddr,
    output     [7:0]  ddr_axi_arlen,
    output     [2:0]  ddr_axi_arsize,
    output     [1:0]  ddr_axi_arburst,
    output            ddr_axi_arlock,
    output     [3:0]  ddr_axi_arcache,
    output     [2:0]  ddr_axi_arprot,
    output     [3:0]  ddr_axi_arqos,
    output            ddr_axi_arvalid,
    input             ddr_axi_arready,
    input      [3:0]  ddr_axi_rid,
    input      [127:0] ddr_axi_rdata,
    input      [1:0]  ddr_axi_rresp,
    input             ddr_axi_rlast,
    input             ddr_axi_rvalid,
    output            ddr_axi_rready
    );

//---- m0：LSU ----
    wire [31:0] m0_haddr, m0_hwdata, m0_hrdata;
    wire [1:0]  m0_htrans;
    wire        m0_hwrite, m0_hready, m0_rd_valid, m0_hgrant, m0_hresp, m0_hmastlock;
    wire [2:0]  m0_hsize, m0_hburst;
    wire [3:0]  m0_hwstrb, m0_hprot;

//---- m1：L2（端口，来自 cpu_top）----

//---- m2：DMA ----
    wire [31:0] m2_haddr, m2_hwdata, m2_hrdata;
    wire [1:0]  m2_htrans;
    wire        m2_hwrite, m2_hready, m2_rd_valid, m2_hgrant, m2_hresp, m2_hmastlock;
    wire [2:0]  m2_hsize, m2_hburst;
    wire [3:0]  m2_hwstrb, m2_hprot;
    wire        dma_busy;

//---- m3：boot 搬运引擎 ----
    wire [31:0] m3_haddr, m3_hwdata, m3_hrdata;
    wire [1:0]  m3_htrans;
    wire        m3_hwrite, m3_hready, m3_rd_valid, m3_hgrant, m3_hresp, m3_hmastlock;
    wire [2:0]  m3_hsize, m3_hburst;
    wire [3:0]  m3_hwstrb, m3_hprot;
    wire        boot_done_w, boot_err_w;
    wire [16:0] boot_count_w;

//---- s0：APB 外设桥 ----
    wire [31:0] s0_haddr, s0_hwdata, s0_hrdata;
    wire [1:0]  s0_htrans;
    wire        s0_hwrite, s0_hreadyout, s0_rd_valid, s0_hresp, s0_hmastlock;
    wire [2:0]  s0_hsize, s0_hburst;
    wire [3:0]  s0_hwstrb, s0_hprot;

//---- s2：NPU（slot 8 = 0x2000_0000）----
    wire [31:0] s2_haddr, s2_hwdata, s2_hrdata;
    wire [1:0]  s2_htrans;
    wire        s2_hwrite, s2_hreadyout, s2_hresp, s2_rd_valid, s2_hsel;
    wire [2:0]  s2_hsize;
    wire [1:0]  npu_hresp;

//---- s1：STCM ----
    wire [31:0] s1_haddr, s1_hwdata, s1_hrdata;
    wire [1:0]  s1_htrans;
    wire        s1_hwrite, s1_hreadyout, s1_rd_valid, s1_hresp, s1_hmastlock;
    wire [2:0]  s1_hsize, s1_hburst;
    wire [3:0]  s1_hwstrb, s1_hprot;

//---- s3：DDR/AXI 桥（slot 32 = 0x8000_0000）----
    wire [31:0] s3_haddr, s3_hwdata, s3_hrdata;
    wire [1:0]  s3_htrans;
    wire        s3_hwrite, s3_hreadyout, s3_hresp, s3_rd_valid, s3_hsel;
    wire [2:0]  s3_hsize;
    wire [3:0]  s3_hprot, s3_hwstrb;

//==============================================================================
//  一、错误日志：只留第一次，软件写 1 清
//    锁存放在这里（只有本模块看得见三个主设备的 hresp）；寄存器窗口开在
//    plic 的 0x0900_0000（厂商扩展），那边只负责摆出来 + 产生 err_clear。
//==============================================================================
    wire        err_now     = m0_hresp | m1_hresp | m2_hresp;
    wire [1:0]  err_owner_now = m0_hresp ? 2'd0 : m1_hresp ? 2'd1 : 2'd2;
    wire [31:0] err_addr_now  = m0_hresp ? m0_haddr : (m1_hresp ? m1_haddr : m2_haddr);
    wire        err_write_now = m0_hresp ? m0_hwrite : (m1_hresp ? m1_hwrite : m2_hwrite);

    reg        err_valid_q;
    reg        err_write_q;
    reg [1:0]  err_owner_q;
    reg [31:0] err_addr_q;

    always @(posedge clk) begin
        if (rst_cpu) begin
            err_valid_q <= 1'b0;
            err_write_q <= 1'b0;
            err_owner_q <= 2'd0;
            err_addr_q  <= 32'd0;
        end
        else if (err_clear) begin
            err_valid_q <= 1'b0;
        end
        else if (err_now && !err_valid_q) begin
            err_valid_q <= 1'b1;
            err_write_q <= err_write_now;
            err_owner_q <= err_owner_now;
            err_addr_q  <= err_addr_now;
        end
    end

    assign err_valid = err_valid_q;
    assign err_write = err_write_q;
    assign err_owner = err_owner_q;
    assign err_addr  = err_addr_q;

    rst_buf u_rst_buf (
        .clk(clk),
        .rst_n(rst),
        .rst_stable(rst_stable)
    );
    wire rst_stable;

//==============================================================================
//  一之一、复位分域：boot 搬运期间 CPU / DMA 压着，总线与 SPI 先活
//    rst_sys = rst_buf 拉长后的复位（一放开就活）
//    rst_cpu = rst_sys 或 "还没搬完"      （搬完才放开）
//  ★ boot_done 只在 rst_sys 为高时清 —— 也就是下一次复位才允许重新搬运。
//==============================================================================
    reg boot_done_q;
    always @(posedge clk) begin
        if (rst_stable)
            boot_done_q <= 1'b0;
        else if (boot_done_w)
            boot_done_q <= 1'b1;
        else
            boot_done_q <= boot_done_q;
    end

    always @(*) begin
        rst_sys = rst_stable;
        rst_cpu = rst_stable | ~boot_done_q;
    end

//==============================================================================
//  一、CPU 侧主设备
//==============================================================================
    lsu_ahb_master u_lsu_ahb (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_f_cpu),
        .bus_data_in(bus_data_f_cpu),
        .bus_be_in(bus_be_f_cpu),
        .bus_we_in(bus_we_f_cpu),
        .bus_valid_in(bus_valid_f_cpu),
        .bus_data_out(bus_data_b_cpu),
        .bus_ready_out(bus_loaded_out),
        .bus_hold_out(bus_hold_out),
        .haddr(m0_haddr),
        .htrans(m0_htrans),
        .hwrite(m0_hwrite),
        .hsize(m0_hsize),
        .hburst(m0_hburst),
        .hprot(m0_hprot),
        .hmastlock(m0_hmastlock),
        .hwdata(m0_hwdata),
        .hwstrb(m0_hwstrb),
        .hrdata(m0_hrdata),
        .hready(m0_hready),
        .hresp(m0_hresp),
        .rd_valid(m0_rd_valid),
        .hgrant(m0_hgrant)
    );

//==============================================================================
//  二、DMA：AHB 主设备 + 寄存器口（3 号槽，0x0C00_0000）
//==============================================================================
//寄存器字节地址 = bus_addr_out[4:0] << 2（haddr[6:2] = 字偏移，haddr[6:0] 里通道号在 [6:5]）
    wire        dma_slot      = bus_sel_out[3];
    wire [7:0]  dma_reg_addr  = {1'b0, bus_addr_out[4:0], 2'b00};
    wire        dma_reg_we    = dma_slot & bus_we_out & bus_be_out[0];
    wire        dma_reg_re    = dma_slot & ~bus_we_out;
    wire [31:0] dma_reg_rdata;

//读数据必须在这里锁一拍：桥的应答比地址相位晚一拍，而 dma.v 的 reg_rdata 是
//纯组合量（地址一撤就变了）。其余外设自己锁，DMA 不回写就只在这里锁。
    reg [31:0] dma_data_q;
    always @(posedge clk) begin
        if (rst_cpu)                     dma_data_q <= 32'd0;
        else if (dma_slot && !bus_we_out) dma_data_q <= dma_reg_rdata;
    end

    dma u_dma (
        .clk(clk), .rst(rst_cpu),
        .reg_addr(dma_reg_addr), .reg_wdata(bus_data_out), .reg_we(dma_reg_we), .reg_re(dma_reg_re),
        .reg_rdata(dma_reg_rdata), .irq(dma_irq),
        .m_haddr(m2_haddr), .m_htrans(m2_htrans), .m_hwrite(m2_hwrite), .m_hsize(m2_hsize),
        .m_hburst(m2_hburst), .m_hprot(m2_hprot), .m_hmastlock(m2_hmastlock),
        .m_hwdata(m2_hwdata), .m_hwstrb(m2_hwstrb),
        .m_hrdata(m2_hrdata), .m_hready(m2_hready), .m_rd_valid(m2_rd_valid),
        .m_busy(dma_busy), .m_hgrant(m2_hgrant), .m_hresp(m2_hresp)
    );

//==============================================================================
//  三、互联（三主两从）
//==============================================================================
    ahb_interconnect u_ahb_ic (
        .clk(clk), .rst(rst_sys),
        .m0_haddr(m0_haddr), .m0_htrans(m0_htrans), .m0_hwrite(m0_hwrite), .m0_hsize(m0_hsize),
        .m0_hburst(m0_hburst), .m0_hprot(m0_hprot), .m0_hmastlock(m0_hmastlock),
        .m0_hwdata(m0_hwdata), .m0_hwstrb(m0_hwstrb),
        .m0_hrdata(m0_hrdata), .m0_hready(m0_hready), .m0_hresp(m0_hresp),
        .m0_rd_valid(m0_rd_valid), .m0_hgrant(m0_hgrant),
        .m1_haddr(m1_haddr), .m1_htrans(m1_htrans), .m1_hwrite(m1_hwrite), .m1_hsize(m1_hsize),
        .m1_hburst(m1_hburst), .m1_hprot(m1_hprot), .m1_hmastlock(m1_hmastlock),
        .m1_hwdata(m1_hwdata), .m1_hwstrb(m1_hwstrb),
        .m1_hrdata(m1_hrdata), .m1_hready(m1_hready), .m1_hresp(m1_hresp),
        .m1_rd_valid(m1_rd_valid), .m1_hgrant(m1_hgrant),
        .m2_haddr(m2_haddr), .m2_htrans(m2_htrans), .m2_hwrite(m2_hwrite), .m2_hsize(m2_hsize),
        .m2_hburst(m2_hburst), .m2_hprot(m2_hprot), .m2_hmastlock(m2_hmastlock),
        .m2_hwdata(m2_hwdata), .m2_hwstrb(m2_hwstrb),
        .m2_hrdata(m2_hrdata), .m2_hready(m2_hready), .m2_hresp(m2_hresp),
        .m2_rd_valid(m2_rd_valid), .m2_hgrant(m2_hgrant),
        .s0_haddr(s0_haddr), .s0_htrans(s0_htrans), .s0_hwrite(s0_hwrite), .s0_hsize(s0_hsize),
        .s0_hburst(s0_hburst), .s0_hprot(s0_hprot), .s0_hmastlock(s0_hmastlock),
        .s0_hwdata(s0_hwdata), .s0_hwstrb(s0_hwstrb),
        .s0_hrdata(s0_hrdata), .s0_hreadyout(s0_hreadyout), .s0_hresp(s0_hresp),
        .s0_rd_valid(s0_rd_valid),
        .s1_haddr(s1_haddr), .s1_htrans(s1_htrans), .s1_hwrite(s1_hwrite), .s1_hsize(s1_hsize),
        .s1_hburst(s1_hburst), .s1_hprot(s1_hprot), .s1_hmastlock(s1_hmastlock),
        .s1_hwdata(s1_hwdata), .s1_hwstrb(s1_hwstrb),
        .s1_hrdata(s1_hrdata), .s1_hreadyout(s1_hreadyout), .s1_hresp(s1_hresp),
        .s1_rd_valid(s1_rd_valid),
        .m3_haddr(m3_haddr), .m3_htrans(m3_htrans), .m3_hwrite(m3_hwrite), .m3_hsize(m3_hsize),
        .m3_hburst(m3_hburst), .m3_hprot(m3_hprot), .m3_hmastlock(m3_hmastlock),
        .m3_hwdata(m3_hwdata), .m3_hwstrb(m3_hwstrb),
        .m3_hrdata(m3_hrdata), .m3_hready(m3_hready), .m3_hresp(m3_hresp),
        .m3_rd_valid(m3_rd_valid), .m3_hgrant(m3_hgrant),
        .s2_haddr(s2_haddr), .s2_htrans(s2_htrans), .s2_hwrite(s2_hwrite),
        .s2_hsize(s2_hsize), .s2_hwdata(s2_hwdata), .s2_hsel(s2_hsel),
        .s2_hrdata(s2_hrdata), .s2_hreadyout(s2_hreadyout),
        .s2_hresp(s2_hresp), .s2_rd_valid(s2_rd_valid),
        .s3_haddr(s3_haddr), .s3_htrans(s3_htrans), .s3_hwrite(s3_hwrite),
        .s3_hsize(s3_hsize), .s3_hprot(s3_hprot), .s3_hwdata(s3_hwdata),
        .s3_hwstrb(s3_hwstrb), .s3_hsel(s3_hsel),
        .s3_hrdata(s3_hrdata), .s3_hreadyout(s3_hreadyout),
        .s3_hresp(s3_hresp), .s3_rd_valid(s3_rd_valid)
    );

//==============================================================================
//  三之一、NPU（AHB-Lite 从设备，4x4 INT8 GEMM）
//    · 它是变延迟从设备（Buffer 读要等同步 RAM、忙时回两拍 ERROR）⇒ 应答走互联的
//      直达通路，不压 owner FIFO。
//    · HRESP 是本从机的两拍结构，互联只认一位 ⇒ 取低一位给互联。
//    · 复位跟总线域（rst_sys）：boot 期间 CPU 摸不到它，但总线活着就能答。
//==============================================================================
    npu_system u_npu (
        .HCLK(clk), .HRESETn(~rst_sys),
        .HSEL(s2_hsel), .HADDR(s2_haddr), .HTRANS(s2_htrans), .HWRITE(s2_hwrite),
        .HSIZE(s2_hsize), .HWDATA(s2_hwdata), .HREADY(s2_hreadyout),
        .HRDATA(s2_hrdata), .HREADYOUT(s2_hreadyout), .HRESP(npu_hresp),
        .rd_valid(s2_rd_valid)
    );
    assign s2_hresp = npu_hresp[0];

//==============================================================================
//  三之三、DDR/AXI 桥（AHB 从设备 s3 → AXI4 主设备）
//    · 和 NPU 一样是变延迟从设备，而且更硬：整笔传输期间压着 HREADYOUT，
//      主设备的请求根本没被接受 ⇒ 授权与地址不会换手，应答按当拍 sel 直达。
//    · 地址窗口：0x8000_0000 起 1 GB（互联只判 slot，精确窗口在桥里判）。
//    · AXI 侧：128 位、ID=0、LEN=0、INCR、满宽；下游留给 MIG 7 系列的 AXI4 从口。
//    · 复位跟总线域（rst_sys）。接 MIG 时 HCLK/RESET 要跟 c0_ui_clk / c0_aresetn
//      同一个域（桥头注释有说明）。
//==============================================================================
    ahb_axi_bridge #(
        .DDR_PRESENT    (DDR_PRESENT),
        .AXI_ADDR_WIDTH (32),
        .AXI_DATA_WIDTH (128),
        .AXI_ID_WIDTH   (4),
        .DDR_BASE       (32'h8000_0000),
        .DDR_BYTES      (33'h0_4000_0000),
        .AXI_BASE_ADDR  (32'h0000_0000)
    ) u_ddr_bridge (
        .HCLK       (clk),
        .HRESETn    (~rst_sys),
        .MEM_READY  (ddr_mem_ready),
        .HSEL       (s3_hsel),
        .HADDR      (s3_haddr),
        .HTRANS     (s3_htrans),
        .HWRITE     (s3_hwrite),
        .HSIZE      (s3_hsize),
        .HPROT      (s3_hprot),
        .HWSTRB     (s3_hwstrb),
        .HWDATA     (s3_hwdata),
        .HRDATA     (s3_hrdata),
        .HREADYOUT  (s3_hreadyout),
        .HRESP      (s3_hresp),
        .rd_valid   (s3_rd_valid),
        .M_AXI_AWID    (ddr_axi_awid),
        .M_AXI_AWADDR  (ddr_axi_awaddr),
        .M_AXI_AWLEN   (ddr_axi_awlen),
        .M_AXI_AWSIZE  (ddr_axi_awsize),
        .M_AXI_AWBURST (ddr_axi_awburst),
        .M_AXI_AWLOCK  (ddr_axi_awlock),
        .M_AXI_AWCACHE (ddr_axi_awcache),
        .M_AXI_AWPROT  (ddr_axi_awprot),
        .M_AXI_AWQOS   (ddr_axi_awqos),
        .M_AXI_AWVALID (ddr_axi_awvalid),
        .M_AXI_AWREADY (ddr_axi_awready),
        .M_AXI_WDATA   (ddr_axi_wdata),
        .M_AXI_WSTRB   (ddr_axi_wstrb),
        .M_AXI_WLAST   (ddr_axi_wlast),
        .M_AXI_WVALID  (ddr_axi_wvalid),
        .M_AXI_WREADY  (ddr_axi_wready),
        .M_AXI_BID     (ddr_axi_bid),
        .M_AXI_BRESP   (ddr_axi_bresp),
        .M_AXI_BVALID  (ddr_axi_bvalid),
        .M_AXI_BREADY  (ddr_axi_bready),
        .M_AXI_ARID    (ddr_axi_arid),
        .M_AXI_ARADDR  (ddr_axi_araddr),
        .M_AXI_ARLEN   (ddr_axi_arlen),
        .M_AXI_ARSIZE  (ddr_axi_arsize),
        .M_AXI_ARBURST (ddr_axi_arburst),
        .M_AXI_ARLOCK  (ddr_axi_arlock),
        .M_AXI_ARCACHE (ddr_axi_arcache),
        .M_AXI_ARPROT  (ddr_axi_arprot),
        .M_AXI_ARQOS   (ddr_axi_arqos),
        .M_AXI_ARVALID (ddr_axi_arvalid),
        .M_AXI_ARREADY (ddr_axi_arready),
        .M_AXI_RID     (ddr_axi_rid),
        .M_AXI_RDATA   (ddr_axi_rdata),
        .M_AXI_RRESP   (ddr_axi_rresp),
        .M_AXI_RLAST   (ddr_axi_rlast),
        .M_AXI_RVALID  (ddr_axi_rvalid),
        .M_AXI_RREADY  (ddr_axi_rready)
    );

//==============================================================================
//  三之二、boot 搬运引擎：上电把镜像从 SPI flash 搬进 STCM，搬完放开 CPU
//==============================================================================
    boot_loader #(
        .SPI_BASE(32'h1C00_0000),
        .LOAD_BASE(32'h0000_0000)
    ) u_boot_loader (
        .clk(clk),
        .rst(rst_sys),
        .haddr(m3_haddr), .htrans(m3_htrans), .hwrite(m3_hwrite), .hsize(m3_hsize),
        .hburst(m3_hburst), .hprot(m3_hprot), .hmastlock(m3_hmastlock),
        .hwdata(m3_hwdata), .hwstrb(m3_hwstrb),
        .hrdata(m3_hrdata), .hready(m3_hready), .hresp(m3_hresp),
        .hgrant(m3_hgrant), .rd_valid(m3_rd_valid),
        .boot_done(boot_done_w), .boot_err(boot_err_w), .boot_count(boot_count_w)
    );

    always @(*) begin
        boot_done_out = boot_done_q;
        boot_err_out = boot_err_w;
        boot_count_out = boot_count_w;
    end

//==============================================================================
//  四、从设备
//==============================================================================
    ahb_apb_bridge u_ahb_apb (
        .clk(clk),
        .rst(rst_sys),
        .haddr(s0_haddr),
        .htrans(s0_htrans),
        .hwrite(s0_hwrite),
        .hsize(s0_hsize),
        .hburst(s0_hburst),
        .hprot(s0_hprot),
        .hmastlock(s0_hmastlock),
        .hwdata(s0_hwdata),
        .hwstrb(s0_hwstrb),
        .hrdata(s0_hrdata),
        .hreadyout(s0_hreadyout),
        .hresp(s0_hresp),
        .rd_valid(s0_rd_valid),
        .bus_addr_out(bus_addr_out),
        .bus_data_out(bus_data_out),
        .bus_be_out(bus_be_out),
        .bus_we_out(bus_we_out),
        .bus_sel_out(bus_sel_out),
        .uart_data(uart_data),
        .uart_ready(uart_ready),
        .plic_data(plic_data),
        .plic_ready(plic_ready),
        .gpio_data(gpio_data),
        .gpio_ready(gpio_ready),
        .i2c_data(i2c_data),
        .i2c_ready(i2c_ready),
        .tim_data(tim_data),
        .tim_ready(tim_ready),
        .spi_data(spi_data),
        .spi_ready(spi_ready),
        .dma_data(dma_data_q),
        .dma_ready(1'b1)
    );

//64KB 模拟外存：DMA 的搬运对象，也是将来 L2 的后备存储
    stcm #(.AW(16)) u_stcm (
        .clk(clk),
        .rst(rst_sys),
        .haddr(s1_haddr),
        .htrans(s1_htrans),
        .hwrite(s1_hwrite),
        .hsize(s1_hsize),
        .hburst(s1_hburst),
        .hprot(s1_hprot),
        .hmastlock(s1_hmastlock),
        .hwdata(s1_hwdata),
        .hwstrb(s1_hwstrb),
        .hrdata(s1_hrdata),
        .hreadyout(s1_hreadyout),
        .hresp(s1_hresp),
        .rd_valid(s1_rd_valid)
    );

endmodule
