`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/29 02:38:18
// Design Name:
// Module Name: mcu_top
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


module mcu_top #(
//★★ 宏化旋钮：与 cpu_top 同名同义，只为让综合顶层能选档（synth_design -generic / xelab -generic_top）。
//  默认值 = 大核交付配置。
    parameter integer LANES             = 2,
    parameter integer ROB_DEPTH         = 16,
    parameter integer IFFU_DEPTH        = 8,
    parameter integer VALUE_TABLE_DEPTH = 16,
    parameter integer IC_INDEX_B        = 6,    // 16KB（8=64KB / 7=32KB）；与 cpu_top 默认一致
    parameter integer BHT_B             = 11,
    parameter integer BTB_B             = 11,
    parameter integer DC_SETS_B         = 8,    // 16KB（9=32KB / 8=16KB / 7=8KB）；与 cpu_top 默认一致
    parameter integer ITCM_AW           = 14,
    parameter integer ADDER_AT_D        = 0,
//★ 系统镜像从哪块 flash 搬：0 = 数据盘 Q128（现状），1 = 配置盘 Q64 的 0x70_0000。
//  两条硬件通路都已验证过（台子 tb_boot / tb_bootcfg），默认保持能开机的那条。
    parameter integer BOOT_FROM_CFG     = 0,
//★ 7 号槽 SPI 的分频由这两个频率算出来（见 IO_block/pmem_spi.v 头注释）：
//  换核时钟只改 CLK_MHZ，SCK 目标频率保持不变。
    parameter integer CLK_MHZ           = 100,
    parameter integer FLASH_SCK_KHZ     = 2500,
    parameter integer SD_SCK_KHZ        = 300,
//DDR 下游在不在。0（默认）= 没有存储器，访问 DDR 窗口一律回 ERROR；接 MIG 时改 1。
    parameter integer DDR_PRESENT       = 0
) (
    input clk, rst,
    input  spi_miso,
//---- 配置 flash（Q64）那组专用脚：只做直通，引脚约束在顶层 ----
    input  cfg_miso,
//---- SPI 主机（7 号槽）：接板上的 SPI NOR flash，直出、不经 gpio ----
    output spi_cs_n,
    output spi_cs_sd,
    output spi_sck,
    output spi_mosi,
    output cfg_cs_n,
    output cfg_sck,
    output cfg_mosi,
    inout wire [31:0] gpio_pin_bus,
//---- 板载 DDR3（MIG 7 系列）：只有 D:\ 上板综合时这套才存在，仿真里是空壳 ----
//  ★ 为什么用 `ifdef SYNTHESIS 而不是参数：仿真台（vmenu/vsys/vbus ...）不该为了
//    一个没人访问的 DDR 去编译 100 多份 MIG RTL。仿真分支里这些口全部钉常数，
//    ★ 注意：注释行别让仿真器的名字紧跟在行首的双斜杠后面 —— 那是它的元注释
//      语法，会被当成指令解析并报 "Unknown verilator comment"（踩过）。
//      仿真器里看不到 mig_ddr3，文件清单不用改。
    input  wire        clk_200,        // MIG 系统时钟 / IDELAYCTRL 参考时钟（50→200 MMCM）
    inout  wire [31:0] ddr3_dq,
    inout  wire [3:0]  ddr3_dqs_p,
    inout  wire [3:0]  ddr3_dqs_n,
    output wire [13:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n,
    output wire        ddr3_cas_n,
    output wire        ddr3_we_n,
    output wire        ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p,
    output wire [0:0]  ddr3_ck_n,
    output wire [0:0]  ddr3_cke,
    output wire [0:0]  ddr3_cs_n,
    output wire [3:0]  ddr3_dm,
    output wire [0:0]  ddr3_odt
    );

//配置 flash 控制器的软复位：不出顶层，直接进 bus_con_top 的复位或树
    wire cfg_reset_w;

//L2 维护与"非缓存窗口"（走总线的 DMA 缓冲区用）。
//★ 现在没有任何软件通路去写它们：maint 恒 0（不失效），非缓存窗口设成永不命中。
//  ⇒ DMA 直接改 STCM 之后，L2 里若有同一行的旧副本，CPU 会读到旧值。
//  这块要接上"某个能从软件写的寄存器"（PLIC 旁边加一个槽，或者拿一条 CSR），
//  在那之前 DMA 只能用于"搬完就不由 CPU 再读"的场景。
    reg [1:0] l2_maint_w;
    reg [31:0] l2_unc_base_w, l2_unc_mask_w;

    wire rst_cpu;        // CPU / DMA 域（boot 搬运期间压着）
    wire rst_sys;        // 总线 / STCM / APB / SPI 域
    wire rst_wdt;        // 看门狗自己的复位（不含它自己的输出，见 bus_con_top）
    wire [31:0] wdt_data_w;
    wire        wdt_ready_w, wdt_reset_w;
    wire boot_done_w, boot_err_w;
    wire [16:0] boot_count_w;

    wire [31:0] bus_addr_f_cpu, bus_data_f_cpu, bus_data_b_cpu;
    wire [3:0]  bus_be_f_cpu;
    wire        bus_we_f_cpu, bus_valid_f_cpu, bus_loaded_out, bus_hold_out;

    wire [31:0] bus_addr_out, bus_data_out, bus_sel_out;
    wire [3:0]  bus_be_out;
    wire        bus_we_out;

    wire [31:0] uart_data_w, plic_data_w, gpio_data_w, i2c_data_w, tim_data_w, spi_data_w;
    wire        uart_ready_w, plic_ready_w, gpio_ready_w, i2c_ready_w, tim_ready_w, spi_ready_w;
//9 号槽：DPU（显示引擎）。它只是 io_block 里的一个从设备，和别的一样经 APB 桥。
    wire [31:0] dpu_data_w;
    wire        dpu_ready_w;
    wire        plic_exti;
    wire        dma_irq_w;

//---- L2 的 AHB 主口（cpu_top ↔ bus_con_top 的 m1）----
    wire [31:0] l2_haddr_w, l2_hwdata_w, l2_hrdata_w;
    wire [1:0]  l2_htrans_w;
    wire        l2_hwrite_w, l2_hready_w, l2_rd_valid_w, l2_hgrant_w, l2_busy_w;
    wire        l2_hresp_w, l2_hmastlock_w;
    wire [2:0]  l2_hsize_w, l2_hburst_w;
    wire [3:0]  l2_hwstrb_w, l2_hprot_w;
    wire        l2_maint_done_w;

//---- 总线错误日志（厂商扩展，寄存器在 plic）----
    wire        err_valid_w, err_write_w, err_clear_w;
    wire [1:0]  err_owner_w;
    wire [31:0] err_addr_w;

//L2 维护与"非缓存窗口"恒为安全值（见上方声明处的说明）
    always @(*) begin
        l2_maint_w = 2'b00;
        l2_unc_base_w = 32'hFFFF_FFFF;
        l2_unc_mask_w = 32'h0000_0000;
    end

    cpu_top #(.LANES(LANES), .ROB_DEPTH(ROB_DEPTH), .IFFU_DEPTH(IFFU_DEPTH), .VALUE_TABLE_DEPTH(VALUE_TABLE_DEPTH), .IC_INDEX_B(IC_INDEX_B), .BHT_B(BHT_B), .BTB_B(BTB_B), .ADDER_AT_D(ADDER_AT_D), .DC_SETS_B(DC_SETS_B), .ITCM_AW(ITCM_AW)) u_cpu_top (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_out(bus_addr_f_cpu),
        .bus_data_out(bus_data_f_cpu),
        .bus_be_out(bus_be_f_cpu),
        .bus_we_out(bus_we_f_cpu),
        .bus_valid_out(bus_valid_f_cpu),
        .bus_data_in_ext(bus_data_b_cpu),
        .bus_loaded_in(bus_loaded_out),
        .bus_hold_in(bus_hold_out),
        .exti(plic_exti),
        .l2_haddr(l2_haddr_w), .l2_htrans(l2_htrans_w), .l2_hwrite(l2_hwrite_w),
        .l2_hsize(l2_hsize_w), .l2_hburst(l2_hburst_w),
        .l2_hprot(l2_hprot_w), .l2_hmastlock(l2_hmastlock_w),
        .l2_hwdata(l2_hwdata_w), .l2_hwstrb(l2_hwstrb_w),
        .l2_hrdata(l2_hrdata_w), .l2_hready(l2_hready_w), .l2_rd_valid(l2_rd_valid_w),
        .l2_hgrant(l2_hgrant_w), .l2_hresp(l2_hresp_w), .l2_busy(l2_busy_w),
        .l2_maint(l2_maint_w), .l2_maint_done(l2_maint_done_w),
        .l2_uncached_base(l2_unc_base_w), .l2_uncached_mask(l2_unc_mask_w)
    );

//--------------------------------------------------------------- DDR 通路
//  桥 →（异步 AXI 时钟转换）→ MIG → DDR3 颗粒
//  ★ 两个时钟域：桥在核时钟（120 MHz），MIG 的 AXI 口在 c0_ui_clk（100 MHz = 400/4）。
//    桥是单笔在飞、对延迟不敏感，插一层异步 FIFO 最省事，也不用把核时钟改成 100。
//  ★ 地址位宽：桥按 1 GB 窗口判（DDR_BYTES 默认），MIG 的 s_axi_awaddr 只有 29 位
//    = 512 MB ⇒ 过 CDC 时取低 29 位。软件不要碰 0xA000_0000 以上：那里会回卷到
//    低半区（不是没存下，是窗口没跟着改；要改得动 bridge 的 DDR_BYTES）。
    wire [3:0]   ddr_axi_awid, ddr_axi_bid, ddr_axi_arid, ddr_axi_rid;
    wire [31:0]  ddr_axi_awaddr, ddr_axi_araddr;
    wire [7:0]   ddr_axi_awlen, ddr_axi_arlen;
    wire [2:0]   ddr_axi_awsize, ddr_axi_awprot, ddr_axi_arsize, ddr_axi_arprot;
    wire [1:0]   ddr_axi_awburst, ddr_axi_arburst, ddr_axi_bresp, ddr_axi_rresp;
    wire [3:0]   ddr_axi_awcache, ddr_axi_awqos, ddr_axi_arcache, ddr_axi_arqos;
    wire         ddr_axi_awlock, ddr_axi_arlock;
    wire [127:0] ddr_axi_wdata, ddr_axi_rdata;
    wire [15:0]  ddr_axi_wstrb;
    wire         ddr_axi_awvalid, ddr_axi_awready, ddr_axi_wvalid, ddr_axi_wready;
    wire         ddr_axi_wlast, ddr_axi_bvalid, ddr_axi_bready;
    wire         ddr_axi_arvalid, ddr_axi_arready, ddr_axi_rvalid, ddr_axi_rlast, ddr_axi_rready;
    wire         ddr_mem_ready;

    bus_con_top #(
        .DDR_PRESENT(DDR_PRESENT),
        .BOOT_FROM_CFG(BOOT_FROM_CFG)
    ) u_bus_con (
        .clk(clk),
        .rst(rst),
        .soft_reset(cfg_reset_w),
        .bus_addr_f_cpu(bus_addr_f_cpu),
        .bus_data_f_cpu(bus_data_f_cpu),
        .bus_be_f_cpu(bus_be_f_cpu),
        .bus_we_f_cpu(bus_we_f_cpu),
        .bus_valid_f_cpu(bus_valid_f_cpu),
        .rst_cpu(rst_cpu),
        .rst_sys(rst_sys),
        .rst_wdt(rst_wdt),
        .wdt_data(wdt_data_w),
        .wdt_ready(wdt_ready_w),
        .wdt_reset(wdt_reset_w),
        .boot_done_out(boot_done_w),
        .boot_err_out(boot_err_w),
        .boot_count_out(boot_count_w),
        .bus_data_b_cpu(bus_data_b_cpu),
        .bus_loaded_out(bus_loaded_out),
        .bus_hold_out(bus_hold_out),
        .bus_addr_out(bus_addr_out),
        .bus_data_out(bus_data_out),
        .bus_be_out(bus_be_out),
        .bus_we_out(bus_we_out),
        .bus_sel_out(bus_sel_out),
        .uart_data(uart_data_w),
        .uart_ready(uart_ready_w),
        .plic_data(plic_data_w),
        .plic_ready(plic_ready_w),
        .gpio_data(gpio_data_w),
        .gpio_ready(gpio_ready_w),
        .i2c_data(i2c_data_w),
        .i2c_ready(i2c_ready_w),
        .tim_data(tim_data_w),
        .tim_ready(tim_ready_w),
        .spi_data(spi_data_w),
        .spi_ready(spi_ready_w),
        .dpu_data(dpu_data_w),
        .dpu_ready(dpu_ready_w),
        .dma_irq(dma_irq_w),
        .m1_haddr(l2_haddr_w), .m1_htrans(l2_htrans_w), .m1_hwrite(l2_hwrite_w),
        .m1_hsize(l2_hsize_w), .m1_hburst(l2_hburst_w),
        .m1_hprot(l2_hprot_w), .m1_hmastlock(l2_hmastlock_w),
        .m1_hwdata(l2_hwdata_w), .m1_hwstrb(l2_hwstrb_w),
        .m1_hrdata(l2_hrdata_w), .m1_hready(l2_hready_w), .m1_hresp(l2_hresp_w),
        .m1_rd_valid(l2_rd_valid_w), .m1_hgrant(l2_hgrant_w),
        .err_valid(err_valid_w), .err_write(err_write_w), .err_owner(err_owner_w),
        .err_addr(err_addr_w), .err_clear(err_clear_w),
// ★ DDR/AXI 下游还没有东西（MIG 未接），桥的 DDR_PRESENT=0 ⇒ 访问 0x8000_0000 起的窗口
//   一律回总线 ERROR。AXI 侧输入钉 0、输出不接。
//   ★ 不要把 AXI 主口引成 mcu_top 的端口：顶层每根端口都要占一个 IO 缓冲，128 位数据
//     + 32 位地址 + ID 一类一共 200 多根，没有引脚约束时布局器直接
//     "[Place 30-99] IO Clock Placer failed"（实测 103 个 ERROR、OBUF 全部未布局）。
//     接 MIG 时把 MIG 例化在【这个文件里】，AXI 链路保持内部；DDR 引脚才是顶层端口。
        .ddr_mem_ready(ddr_mem_ready),
        .ddr_axi_awready(ddr_axi_awready), .ddr_axi_wready(ddr_axi_wready),
        .ddr_axi_bid(ddr_axi_bid), .ddr_axi_bresp(ddr_axi_bresp), .ddr_axi_bvalid(ddr_axi_bvalid),
        .ddr_axi_arready(ddr_axi_arready),
        .ddr_axi_rid(ddr_axi_rid), .ddr_axi_rdata(ddr_axi_rdata), .ddr_axi_rresp(ddr_axi_rresp),
        .ddr_axi_rlast(ddr_axi_rlast), .ddr_axi_rvalid(ddr_axi_rvalid),
        .ddr_axi_awid(ddr_axi_awid), .ddr_axi_awaddr(ddr_axi_awaddr),
        .ddr_axi_awlen(ddr_axi_awlen), .ddr_axi_awsize(ddr_axi_awsize),
        .ddr_axi_awburst(ddr_axi_awburst), .ddr_axi_awlock(ddr_axi_awlock),
        .ddr_axi_awcache(ddr_axi_awcache), .ddr_axi_awprot(ddr_axi_awprot),
        .ddr_axi_awqos(ddr_axi_awqos), .ddr_axi_awvalid(ddr_axi_awvalid),
        .ddr_axi_wdata(ddr_axi_wdata), .ddr_axi_wstrb(ddr_axi_wstrb),
        .ddr_axi_wlast(ddr_axi_wlast), .ddr_axi_wvalid(ddr_axi_wvalid),
        .ddr_axi_bready(ddr_axi_bready),
        .ddr_axi_arid(ddr_axi_arid), .ddr_axi_araddr(ddr_axi_araddr),
        .ddr_axi_arlen(ddr_axi_arlen), .ddr_axi_arsize(ddr_axi_arsize),
        .ddr_axi_arburst(ddr_axi_arburst), .ddr_axi_arlock(ddr_axi_arlock),
        .ddr_axi_arcache(ddr_axi_arcache), .ddr_axi_arprot(ddr_axi_arprot),
        .ddr_axi_arqos(ddr_axi_arqos), .ddr_axi_arvalid(ddr_axi_arvalid),
        .ddr_axi_rready(ddr_axi_rready)
    );

//--------------------------------------------------------------- DDR 下游：MIG + CDC
`ifdef SYNTHESIS
    generate
    if (DDR_PRESENT != 0) begin : g_ddr_on
        wire        ui_clk, ui_clk_sync_rst, init_calib_complete;

        //CDC 的 M 口（ui_clk 域）→ MIG 的 AXI4 从口
        wire [3:0]   m_awid, m_bid, m_arid, m_rid;
        wire [28:0]  m_awaddr, m_araddr;
        wire [7:0]   m_awlen, m_arlen;
        wire [2:0]   m_awsize, m_awprot, m_arsize, m_arprot;
        wire [1:0]   m_awburst, m_arburst, m_bresp, m_rresp;
        wire [3:0]   m_awcache, m_awqos, m_arcache, m_arqos;
        wire         m_awlock, m_arlock;
        wire [127:0] m_wdata, m_rdata;
        wire [15:0]  m_wstrb;
        wire         m_awvalid, m_awready, m_wvalid, m_wready, m_wlast;
        wire         m_bvalid, m_bready, m_arvalid, m_arready, m_rvalid, m_rlast, m_rready;

        //★ init_calib_complete 是 ui_clk 域的电平，进核时钟域要两级同步再当 MEM_READY。
        //  低的时候桥会停在 S_WAIT_MEM（HREADYOUT 一直压着），所以"标定没完就别碰 DDR"
        //  这条是硬件自己保证的，软件不用轮询。
        reg [1:0] calib_sync;
        always @(posedge clk) begin
            if (rst_sys) calib_sync <= 2'b00;
            else         calib_sync <= {calib_sync[0], init_calib_complete};
        end
        assign ddr_mem_ready = calib_sync[1];

        axi_cc_ddr u_axi_cc (
            .s_axi_aclk    (clk),
            .s_axi_aresetn (~rst_sys),
            .s_axi_awid    (ddr_axi_awid),
            .s_axi_awaddr  (ddr_axi_awaddr[28:0]),
            .s_axi_awlen   (ddr_axi_awlen),
            .s_axi_awsize  (ddr_axi_awsize),
            .s_axi_awburst (ddr_axi_awburst),
            .s_axi_awlock  (ddr_axi_awlock),
            .s_axi_awcache (ddr_axi_awcache),
            .s_axi_awprot  (ddr_axi_awprot),
            .s_axi_awregion(4'd0),
            .s_axi_awqos   (ddr_axi_awqos),
            .s_axi_awvalid (ddr_axi_awvalid),
            .s_axi_awready (ddr_axi_awready),
            .s_axi_wdata   (ddr_axi_wdata),
            .s_axi_wstrb   (ddr_axi_wstrb),
            .s_axi_wlast   (ddr_axi_wlast),
            .s_axi_wvalid  (ddr_axi_wvalid),
            .s_axi_wready  (ddr_axi_wready),
            .s_axi_bid     (ddr_axi_bid),
            .s_axi_bresp   (ddr_axi_bresp),
            .s_axi_bvalid  (ddr_axi_bvalid),
            .s_axi_bready  (ddr_axi_bready),
            .s_axi_arid    (ddr_axi_arid),
            .s_axi_araddr  (ddr_axi_araddr[28:0]),
            .s_axi_arlen   (ddr_axi_arlen),
            .s_axi_arsize  (ddr_axi_arsize),
            .s_axi_arburst (ddr_axi_arburst),
            .s_axi_arlock  (ddr_axi_arlock),
            .s_axi_arcache (ddr_axi_arcache),
            .s_axi_arprot  (ddr_axi_arprot),
            .s_axi_arregion(4'd0),
            .s_axi_arqos   (ddr_axi_arqos),
            .s_axi_arvalid (ddr_axi_arvalid),
            .s_axi_arready (ddr_axi_arready),
            .s_axi_rid     (ddr_axi_rid),
            .s_axi_rdata   (ddr_axi_rdata),
            .s_axi_rresp   (ddr_axi_rresp),
            .s_axi_rlast   (ddr_axi_rlast),
            .s_axi_rvalid  (ddr_axi_rvalid),
            .s_axi_rready  (ddr_axi_rready),

            .m_axi_aclk    (ui_clk),
            .m_axi_aresetn (~ui_clk_sync_rst),
            .m_axi_awid    (m_awid),
            .m_axi_awaddr  (m_awaddr),
            .m_axi_awlen   (m_awlen),
            .m_axi_awsize  (m_awsize),
            .m_axi_awburst (m_awburst),
            .m_axi_awlock  (m_awlock),
            .m_axi_awcache (m_awcache),
            .m_axi_awprot  (m_awprot),
            .m_axi_awqos   (m_awqos),
            .m_axi_awvalid (m_awvalid),
            .m_axi_awready (m_awready),
            .m_axi_wdata   (m_wdata),
            .m_axi_wstrb   (m_wstrb),
            .m_axi_wlast   (m_wlast),
            .m_axi_wvalid  (m_wvalid),
            .m_axi_wready  (m_wready),
            .m_axi_bid     (m_bid),
            .m_axi_bresp   (m_bresp),
            .m_axi_bvalid  (m_bvalid),
            .m_axi_bready  (m_bready),
            .m_axi_arid    (m_arid),
            .m_axi_araddr  (m_araddr),
            .m_axi_arlen   (m_arlen),
            .m_axi_arsize  (m_arsize),
            .m_axi_arburst (m_arburst),
            .m_axi_arlock  (m_arlock),
            .m_axi_arcache (m_arcache),
            .m_axi_arprot  (m_arprot),
            .m_axi_arqos   (m_arqos),
            .m_axi_arvalid (m_arvalid),
            .m_axi_arready (m_arready),
            .m_axi_rid     (m_rid),
            .m_axi_rdata   (m_rdata),
            .m_axi_rresp   (m_rresp),
            .m_axi_rlast   (m_rlast),
            .m_axi_rvalid  (m_rvalid),
            .m_axi_rready  (m_rready)
        );

        mig_ddr3 u_ddr3 (
            .ddr3_dq      (ddr3_dq),
            .ddr3_dqs_n   (ddr3_dqs_n),
            .ddr3_dqs_p   (ddr3_dqs_p),
            .ddr3_addr    (ddr3_addr),
            .ddr3_ba      (ddr3_ba),
            .ddr3_ras_n   (ddr3_ras_n),
            .ddr3_cas_n   (ddr3_cas_n),
            .ddr3_we_n    (ddr3_we_n),
            .ddr3_reset_n (ddr3_reset_n),
            .ddr3_ck_p    (ddr3_ck_p),
            .ddr3_ck_n    (ddr3_ck_n),
            .ddr3_cke     (ddr3_cke),
            .ddr3_cs_n    (ddr3_cs_n),
            .ddr3_dm      (ddr3_dm),
            .ddr3_odt     (ddr3_odt),
            //★ sys_clk_i / clk_ref_i 都是 No Buffer 模式（MIG 里 SYSCLK_TYPE/REFCLK_TYPE
            //  = NOBUF），直接从 fabric 进，所以这里喂的是 MMCM 出来的 200 MHz。
            //  IDELAYCTRL 只认 200 MHz，系统时钟和参考时钟共用一个，板子上那颗
            //  200 MHz 晶振的球号一直没定位到 —— 这样就不需要它。
            .sys_clk_i    (clk_200),
            .clk_ref_i    (clk_200),
            .ui_clk       (ui_clk),
            .ui_clk_sync_rst (ui_clk_sync_rst),
            .mmcm_locked  (),
            .aresetn      (~ui_clk_sync_rst),
            .app_sr_req   (1'b0),
            .app_ref_req  (1'b0),
            .app_zq_req   (1'b0),
            .app_sr_active(),
            .app_ref_ack  (),
            .app_zq_ack   (),
            .s_axi_awid   (m_awid),
            .s_axi_awaddr (m_awaddr),
            .s_axi_awlen  (m_awlen),
            .s_axi_awsize (m_awsize),
            .s_axi_awburst(m_awburst),
            .s_axi_awlock (m_awlock),
            .s_axi_awcache(m_awcache),
            .s_axi_awprot (m_awprot),
            .s_axi_awqos  (m_awqos),
            .s_axi_awvalid(m_awvalid),
            .s_axi_awready(m_awready),
            .s_axi_wdata  (m_wdata),
            .s_axi_wstrb  (m_wstrb),
            .s_axi_wlast  (m_wlast),
            .s_axi_wvalid (m_wvalid),
            .s_axi_wready (m_wready),
            .s_axi_bid    (m_bid),
            .s_axi_bresp  (m_bresp),
            .s_axi_bvalid (m_bvalid),
            .s_axi_bready (m_bready),
            .s_axi_arid   (m_arid),
            .s_axi_araddr (m_araddr),
            .s_axi_arlen  (m_arlen),
            .s_axi_arsize (m_arsize),
            .s_axi_arburst(m_arburst),
            .s_axi_arlock (m_arlock),
            .s_axi_arcache(m_arcache),
            .s_axi_arprot (m_arprot),
            .s_axi_arqos  (m_arqos),
            .s_axi_arvalid(m_arvalid),
            .s_axi_arready(m_arready),
            .s_axi_rid    (m_rid),
            .s_axi_rdata  (m_rdata),
            .s_axi_rresp  (m_rresp),
            .s_axi_rlast  (m_rlast),
            .s_axi_rvalid (m_rvalid),
            .s_axi_rready (m_rready),
            .init_calib_complete(init_calib_complete),
            .device_temp  (),
            //MIG 的 sys_rst 是【低有效】（GUI 里选的是 ACTIVE LOW），我们内部复位是高有效
            .sys_rst      (~rst_sys)
        );
    end else begin : g_ddr_off
        assign ddr_mem_ready   = 1'b0;
        assign ddr_axi_awready = 1'b0;
        assign ddr_axi_wready  = 1'b0;
        assign ddr_axi_bid     = 4'd0;
        assign ddr_axi_bresp   = 2'd0;
        assign ddr_axi_bvalid  = 1'b0;
        assign ddr_axi_arready = 1'b0;
        assign ddr_axi_rid     = 4'd0;
        assign ddr_axi_rdata   = 128'd0;
        assign ddr_axi_rresp   = 2'd0;
        assign ddr_axi_rlast   = 1'b0;
        assign ddr_axi_rvalid  = 1'b0;
        assign ddr3_addr    = 14'd0;
        assign ddr3_ba      = 3'd0;
        assign ddr3_ras_n   = 1'b1;
        assign ddr3_cas_n   = 1'b1;
        assign ddr3_we_n    = 1'b1;
        assign ddr3_reset_n = 1'b0;
        assign ddr3_ck_p    = 1'b0;
        assign ddr3_ck_n    = 1'b1;
        assign ddr3_cke     = 1'b0;
        assign ddr3_cs_n    = 1'b1;
        assign ddr3_dm      = 4'd0;
        assign ddr3_odt     = 1'b0;
        assign ddr3_dq      = 32'bz;
        assign ddr3_dqs_p   = 4'bz;
        assign ddr3_dqs_n   = 4'bz;
    end
    endgenerate
`else
//------------------------------------------------------------------ 仿真分支
//  MIG 不进仿真：bvmenu/vsys/vbus 这些台子不该为一个没人访问的 DDR 编译 100 多份
//  MIG RTL。这里把 AXI 从口钉死，DDR_PRESENT 在仿真里保持 0（访问 DDR 窗口回 ERROR）。
    assign ddr_mem_ready   = 1'b0;
    assign ddr_axi_awready = 1'b0;
    assign ddr_axi_wready  = 1'b0;
    assign ddr_axi_bid     = 4'd0;
    assign ddr_axi_bresp   = 2'd0;
    assign ddr_axi_bvalid  = 1'b0;
    assign ddr_axi_arready = 1'b0;
    assign ddr_axi_rid     = 4'd0;
    assign ddr_axi_rdata   = 128'd0;
    assign ddr_axi_rresp   = 2'd0;
    assign ddr_axi_rlast   = 1'b0;
    assign ddr_axi_rvalid  = 1'b0;
    assign ddr3_addr    = 14'd0;
    assign ddr3_ba      = 3'd0;
    assign ddr3_ras_n   = 1'b1;
    assign ddr3_cas_n   = 1'b1;
    assign ddr3_we_n    = 1'b1;
    assign ddr3_reset_n = 1'b0;
    assign ddr3_ck_p    = 1'b0;
    assign ddr3_ck_n    = 1'b1;
    assign ddr3_cke     = 1'b0;
    assign ddr3_cs_n    = 1'b1;
    assign ddr3_dm      = 4'd0;
    assign ddr3_odt     = 1'b0;
    assign ddr3_dq      = 32'bz;
    assign ddr3_dqs_p   = 4'bz;
    assign ddr3_dqs_n   = 4'bz;
`endif

    io_block_top #(
        .CLK_MHZ(CLK_MHZ),
        .FLASH_SCK_KHZ(FLASH_SCK_KHZ),
        .SD_SCK_KHZ(SD_SCK_KHZ)
    ) u_io_block (
        .clk(clk),
        .rst_cpu(rst_sys),
        //★ 看门狗单独一根复位：它自己的 wdt_reset 已经进了 rst_sys，
        //  再吃一口就会变成"自己复位自己"，脉冲只剩一两拍。
        .rst_wdt(rst_wdt),
        .gpio_pin_bus(gpio_pin_bus),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_out),
        .bus_sel_in(bus_sel_out),
        .uart_data(uart_data_w),
        .uart_ready(uart_ready_w),
        .plic_data(plic_data_w),
        .plic_ready(plic_ready_w),
        .gpio_data(gpio_data_w),
        .gpio_ready(gpio_ready_w),
        .i2c_data(i2c_data_w),
        .i2c_ready(i2c_ready_w),
        .tim_data(tim_data_w),
        .tim_ready(tim_ready_w),
        .spi_data(spi_data_w),
        .spi_ready(spi_ready_w),
        .spi_cs_n(spi_cs_n),        .spi_cs_sd(spi_cs_sd),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .cfg_miso(cfg_miso),
        .cfg_cs_n(cfg_cs_n),
        .cfg_sck(cfg_sck),
        .cfg_mosi(cfg_mosi),
        .cfg_reset_out(cfg_reset_w),
        .boot_done(boot_done_w),
        .boot_err(boot_err_w),
        .boot_count(boot_count_w),
        .dpu_data(dpu_data_w),
        .dpu_ready(dpu_ready_w),
        .wdt_data(wdt_data_w),
        .wdt_ready(wdt_ready_w),
        .wdt_reset(wdt_reset_w),
        .plic_exti(plic_exti),
        .dma_irq(dma_irq_w),
        .err_valid(err_valid_w), .err_write(err_write_w), .err_owner(err_owner_w),
        .err_addr(err_addr_w), .err_clear(err_clear_w)
    );

endmodule
