`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: bus_con_top
// Description: 总线汇聚层（mcu_top 里 cpu_top 之外的那一半）。
//
//   2026-11-12 多主改造：
//     · 互联从【单主直通】换成三主两从（旧的单主版已删，ahb_interconnect 现在就是三主的）；
//     · 新增 STCM（64KB 模拟外存，AHB 从设备 s1）作为 DMA 的搬运对象；
//     · 新增 DMA（AHB 主设备 m2），寄存器口挂 3 号槽 0x0C00_0000。
//     · m1 是 L2 的 AHB 主口（cpu_top 里的 l2cache，从 mcu_top 引过来）。
//
//   地址口径（全树只有这一种）：
//     CPU 字节地址 >> 2 = 本模块 bus_addr_f_cpu（字地址）
//     lsu_ahb_master 再 << 2 还原成 AHB 字节地址 haddr
//     互联按 haddr[31:16]==0 选 STCM，否则选 APB 桥
//     桥再 >> 2 吐给外设，外设按 bus_addr_in[31:26] 认槽位
//       1 UART / 2 PLIC / 3 DMA / 4 GPIO / 5 I2C / 6 TIM(PWM)
//////////////////////////////////////////////////////////////////////////////////


module bus_con_top(
    input clk, rst,
    input      [31:0] bus_addr_f_cpu,
    input      [31:0] bus_data_f_cpu,
    input      [3:0]  bus_be_f_cpu,
    input             bus_we_f_cpu,
    input             bus_valid_f_cpu,
    output            rst_cpu,
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
    input             err_clear
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

//---- s0：APB 外设桥 ----
    wire [31:0] s0_haddr, s0_hwdata, s0_hrdata;
    wire [1:0]  s0_htrans;
    wire        s0_hwrite, s0_hreadyout, s0_rd_valid, s0_hresp, s0_hmastlock;
    wire [2:0]  s0_hsize, s0_hburst;
    wire [3:0]  s0_hwstrb, s0_hprot;

//---- s1：STCM ----
    wire [31:0] s1_haddr, s1_hwdata, s1_hrdata;
    wire [1:0]  s1_htrans;
    wire        s1_hwrite, s1_hreadyout, s1_rd_valid, s1_hresp, s1_hmastlock;
    wire [2:0]  s1_hsize, s1_hburst;
    wire [3:0]  s1_hwstrb, s1_hprot;

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
        .rst_stable(rst_cpu)
    );

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
        .clk(clk), .rst(rst_cpu),
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
        .s1_rd_valid(s1_rd_valid)
    );

//==============================================================================
//  四、从设备
//==============================================================================
    ahb_apb_bridge u_ahb_apb (
        .clk(clk),
        .rst(rst_cpu),
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
        .dma_data(dma_data_q),
        .dma_ready(1'b1)
    );

//64KB 模拟外存：DMA 的搬运对象，也是将来 L2 的后备存储
    stcm #(.AW(14)) u_stcm (
        .clk(clk),
        .rst(rst_cpu),
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
