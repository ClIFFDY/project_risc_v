`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: io_block
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


module io_block_top #(
//★ CLK_MHZ = 核时钟频率（MHz）：UART 波特率、I2C SCL、SPI 分频全都由它算出来，
//  换核时钟只改这一处（子模块各自按目标频率反解自己的分频）。
    parameter integer CLK_MHZ       = 100,
    parameter integer FLASH_SCK_KHZ = 2500,
    parameter integer SD_SCK_KHZ    = 300,
    parameter integer USER_SCK_KHZ  = 1000   // 7 号槽 sub1 用户 SPI 的复位分频目标
)(
    input clk, rst_cpu,
    //★ 看门狗专用的复位：不含它自己的 wdt_reset 输出（见 soc/bus_con_top.v）。
    //  别的外设吃 rst_cpu（= rst_sys，已经含看门狗复位）。
    input             rst_wdt,
    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    input      [31:0] bus_sel_in,
    input             spi_miso,
//配置 flash（Q64）专用那组：CS/DI/DO/CLK 都走专用配置脚，和上面那组 GPIO 复用脚无关
    input             cfg_miso,
//boot 引擎的状态（只读展示在 pmem_spi 的 BOOT 寄存器里）
    input             boot_done,
    input             boot_err,
    input      [16:0] boot_count,
//DMA 的完成中断：挂到 PLIC 的 2 号源（3/4 号已被 GPIO/I2C 占）
    input             dma_irq,
//---- 总线错误日志（厂商扩展，寄存器在 plic 的 0x0900_0000 窗口）----
    input             err_valid,
    input             err_write,
    input      [1:0]  err_owner,
    input      [31:0] err_addr,
    output     [31:0] uart_data,
    output            uart_ready,
    output     [31:0] plic_data,
    output            plic_ready,
    output     [31:0] gpio_data,
    output            gpio_ready,
    output     [31:0] i2c_data,
    output            i2c_ready,
    output     [31:0] tim_data,
    output            tim_ready,
//7 号槽：双通道 SPI 主机（flash + SD，共享 SCK/MOSI/MISO，各自 CS）
    output     [31:0] spi_data,
    output            spi_ready,
    output            spi_cs_n,
    output            spi_cs_sd,
    output            spi_sck,
    output            spi_mosi,
//配置 flash 的四根线 + 软复位（cfg_reset_out 要一路拉到 rst_buf 的复位或树里）
    output            cfg_cs_n,
    output            cfg_sck,
    output            cfg_mosi,
    output            cfg_reset_out,
//9 号槽：DPU（显示引擎，0x2400_0000）。它和 5 号槽的 i2c 有一条侧带直连，
//  不经过总线；中断在块内部直接进 PLIC 的 9 号源，不用穿端口。
    output     [31:0] dpu_data,
    output            dpu_ready,
//10 号槽：看门狗（0x2800_0000）。它有一根复位输出要走 soc 的复位树，
//  所以这一级只把数据口和中断交出去，wdt_reset 由 mcu_top 单独接。
    output     [31:0] wdt_data,
    output            wdt_ready,
    output            wdt_reset,        // 直接进 soc 的复位树（见 mcu_top / bus_con_top）
    output            plic_exti,
    output            err_clear,
    inout wire [31:0] gpio_pin_bus
    );

    localparam
    PER_UART = 1,
    PER_PLIC = 2,
    PER_GPIO = 4,
    PER_I2C  = 5,
    PER_TIM  = 6,
    PER_SPI  = 7,
    PER_DPU  = 9,      // 桥的译码从 1..7 放开到 1..9（8 号槽是 NPU，不走 APB）
    PER_WDT  = 10;     // 10 号槽：看门狗（桥的译码已放开到 1..10）

    reg bus_we_uart, bus_we_plic, bus_we_gpio, bus_we_i2c, bus_we_tim;
    reg bus_we_spi_pmem, bus_we_spi_user, bus_we_dpu, bus_we_wdt;

//★ 2026-11-13：7 号槽切成 4 个 64 KB 窗口，用 bus_addr_in[17:16] 选子外设。
//  槽号只有 1..7 可用（桥的译码写死），已经占掉 1/2/4/5/6/7，再开新槽没地方。
//  槽内分窗口不动物理译码，现有外设地址一个都不用改。
//      sub0  0x1C00_0000  pmem_spi（flash + SD）
//      sub1  0x1C01_0000  spi（用户外设，四根线走 GPIO 复用）
//      sub2  0x1C02_0000  预留
//      sub3  0x1C03_0000  预留
//   ★ 位选是 [15:14] 不是 [17:16]：bus_addr_in 是【字】地址（haddr>>2），
//     64 KB 的字节偏移 0x1_0000 落到字地址上就是 0x4000 = bit14。
    reg [1:0] spi_sub;
    reg spi_sel_pmem, spi_sel_user, spi_sel_cfg;

    wire uart_rx_w, uart_tx_w;
    wire uart_rx_irq, gpio_irq, i2c_irq, tim_irq_w, dpu_irq, wdt_irq_w;
    wire i2c_scl_oe, i2c_sda_oe, i2c_scl_in, i2c_sda_in;
    wire pwm1_w, pwm2_w, pwm3_w, pwm4_w;

    wire [31:0] spi_pmem_data, spi_user_data, spi_cfg_data;
    wire        spi_pmem_ready, spi_user_ready, spi_cfg_ready;
    wire        spi_pmem_irq, spi_user_irq;
    wire        spi_user_cs_w, spi_user_sck_w, spi_user_mosi_w, spi_user_miso_w;

    always @(*) begin
        spi_sub = bus_addr_in[15:14];
        spi_sel_pmem = bus_sel_in[PER_SPI] & (spi_sub == 2'd0);
        spi_sel_user = bus_sel_in[PER_SPI] & (spi_sub == 2'd1);
        spi_sel_cfg  = bus_sel_in[PER_SPI] & (spi_sub == 2'd2);
    end

//spi_data / spi_ready 是【两个生产者合流】到同一个输出口：sub0 的 pmem_spi 与 sub1 的
//  spi 各出一份，按 spi_sub 选。合流必须留在本层（下层各自只认识自己的口），所以保留 assign。
    assign spi_data  = (spi_sub == 2'd0) ? spi_pmem_data :
                       (spi_sub == 2'd1) ? spi_user_data :
                       (spi_sub == 2'd2) ? spi_cfg_data : 32'd0;
    // ★ 2026-11-14（总线审计）：sub2/sub3 是保留窗口，原来 ready 恒 0 ⇒ 读这两个窗口
    //   桥永远不出 rd_valid ⇒ 一条 load 不完成 ⇒ 整核挂死（还会把 owner FIFO 顶死）。
    //   保留窗口一律"应答 0"。
    assign spi_ready = (spi_sub == 2'd0) ? spi_pmem_ready :
                       (spi_sub == 2'd1) ? spi_user_ready :
                       (spi_sub == 2'd2) ? spi_cfg_ready : 1'b1;

    always @(*) begin
        bus_we_uart = bus_we_in & bus_sel_in[PER_UART];
        bus_we_plic = bus_we_in & bus_sel_in[PER_PLIC];
        bus_we_gpio = bus_we_in & bus_sel_in[PER_GPIO];
        bus_we_i2c  = bus_we_in & bus_sel_in[PER_I2C];
        bus_we_tim  = bus_we_in & bus_sel_in[PER_TIM];
        bus_we_spi_pmem = bus_we_in & spi_sel_pmem;
        bus_we_spi_user = bus_we_in & spi_sel_user;
        bus_we_dpu      = bus_we_in & bus_sel_in[PER_DPU];
        bus_we_wdt      = bus_we_in & bus_sel_in[PER_WDT];
    end

    uart_top #(.CLK_MHZ(CLK_MHZ)) u_uart (
        .clk(clk),
        .rst(rst_cpu),
        .rx(uart_rx_w),
        .tx(uart_tx_w),
        .rx_irq(uart_rx_irq),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_uart),
        .bus_sel(bus_sel_in[PER_UART]),
        .ld_ready(uart_ready),
        .bus_data_out(uart_data)
    );

    gpio_group u_gpio (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_gpio),
        .ld_ready(gpio_ready),
        .bus_data_out(gpio_data),
        .gpio_irq(gpio_irq),
        .gpio_pin_bus(gpio_pin_bus),
        .sda_oe(i2c_sda_oe),
        .scl_oe(i2c_scl_oe),
        .tx(uart_tx_w),
        .pwm1(pwm1_w),
        .pwm2(pwm2_w),
        .pwm3(pwm3_w),
        .pwm4(pwm4_w),
        .rx(uart_rx_w),
        .sda_in(i2c_sda_in),
        .scl_in(i2c_scl_in),
        .spi_user_cs(spi_user_cs_w),
        .spi_user_sck(spi_user_sck_w),
        .spi_user_mosi(spi_user_mosi_w),
        .spi_user_miso(spi_user_miso_w)
    );

//5 号槽：I2C 主机/从机。9 号槽的 DPU 通过一条侧带直接往它的 TX FIFO 塞字节
    wire        i2c_busy_w, i2c_err_w;
    wire [3:0]  i2c_tx_used_w;
    wire        dpu_push_w, dpu_ctrl_we_w, dpu_ctrl_start_w, dpu_ctrl_stop_w;
    wire [7:0]  dpu_push_data_w, dpu_ctrl_len_w;

    i2c #(.CLK_MHZ(CLK_MHZ)) u_i2c (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_i2c),
        .bus_sel(bus_sel_in[PER_I2C]),
        .ld_ready(i2c_ready),
        .bus_data_out(i2c_data),
        .i2c_irq(i2c_irq),
        .scl_oe(i2c_scl_oe),
        .scl_in(i2c_scl_in),
        .sda_oe(i2c_sda_oe),
        .sda_in(i2c_sda_in),
        .busy(i2c_busy_w),
        .dpu_push(dpu_push_w),
        .dpu_push_data(dpu_push_data_w),
        .dpu_ctrl_we(dpu_ctrl_we_w),
        .dpu_ctrl_start(dpu_ctrl_start_w),
        .dpu_ctrl_stop(dpu_ctrl_stop_w),
        .dpu_ctrl_len(dpu_ctrl_len_w),
        .dpu_tx_used(i2c_tx_used_w),
        .dpu_err(i2c_err_w)
    );

//9 号槽：显示引擎（帧缓冲 + 脏页 + 帧状态机）。侧带直连上面的 i2c，不经总线。
    dpu u_dpu (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_dpu),
        .bus_data_out(dpu_data),
        .ld_ready(dpu_ready),
        .i2c_push(dpu_push_w),
        .i2c_push_data(dpu_push_data_w),
        .i2c_ctrl_we(dpu_ctrl_we_w),
        .i2c_ctrl_start(dpu_ctrl_start_w),
        .i2c_ctrl_stop(dpu_ctrl_stop_w),
        .i2c_ctrl_len(dpu_ctrl_len_w),
        .i2c_tx_used(i2c_tx_used_w),
        .i2c_busy(i2c_busy_w),
        .i2c_err(i2c_err_w),
        .dpu_irq(dpu_irq)
    );

//定时器 / PWM（6 号槽）：FreeRTOS 的节拍源，四路 PWM 直接进 gpio_group 的 PWM1~4 模式
    tim_p u_tim (
        .clk(clk),
        .rst(rst_cpu),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_tim),
        .bus_data_out(tim_data),
        .ld_ready(tim_ready),
        .tim_p_irq(tim_irq_w),
        .pwm1(pwm1_w), .pwm2(pwm2_w), .pwm3(pwm3_w), .pwm4(pwm4_w)
    );

//7 号槽 sub0：双通道 SPI 主机（flash 与 SD 共享三根线，CS 各一根）
    pmem_spi #(
        .CLK_MHZ(CLK_MHZ),
        .FLASH_SCK_KHZ(FLASH_SCK_KHZ),
        .SD_SCK_KHZ(SD_SCK_KHZ)
    ) u_pmem_spi (
        .clk(clk),
        .rst(rst_cpu),
        .bus_sel_in(spi_sel_pmem),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_spi_pmem),
        .bus_data_out(spi_pmem_data),
        .ld_ready(spi_pmem_ready),
        .spi_cs_n(spi_cs_n),
        .spi_cs_sd(spi_cs_sd),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_irq(spi_pmem_irq),
        .boot_done(boot_done),
        .boot_err(boot_err),
        .boot_count(boot_count)
    );

//配置 flash 的 MISO：台子上这几根脚没接时是 z/x，按空闲高处理，别把 z 传进状态机
    wire cfg_miso_safe;
    assign cfg_miso_safe = (cfg_miso === 1'b0) ? 1'b0 : 1'b1;

//7 号槽 sub2：配置 flash（Q64）专用控制器。引脚是 FPGA 的专用配置脚，
//  和上面那组 GPIO 复用脚无关；CLK 要从顶层 STARTUPE2 的 USRCCLKO 出去。
    cfg_spi #(
        .CLK_MHZ(CLK_MHZ)
    ) u_cfg_spi (
        .clk(clk),
        .rst(rst_cpu),
        .bus_sel_in(spi_sel_cfg),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_in),
        .cfg_miso(cfg_miso_safe),
        .bus_data_out(spi_cfg_data),
        .ld_ready(spi_cfg_ready),
        .cfg_cs_n(cfg_cs_n),
        .cfg_sck(cfg_sck),
        .cfg_mosi(cfg_mosi),
        .cfg_reset_out(cfg_reset_out)
    );

    spi #(
        .CLK_MHZ(CLK_MHZ),
        .SCK_KHZ(USER_SCK_KHZ)
    ) u_spi_user (
        .clk(clk),
        .rst(rst_cpu),
        .bus_sel_in(spi_sel_user),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_spi_user),
        .bus_data_out(spi_user_data),
        .ld_ready(spi_user_ready),
        .spi_cs_n(spi_user_cs_w),
        .spi_sck(spi_user_sck_w),
        .spi_mosi(spi_user_mosi_w),
        .spi_miso(spi_user_miso_w),
        .spi_irq(spi_user_irq)
    );

//10 号槽：看门狗。★ 复位用 rst_wdt（不含它自己的输出），不是 rst_cpu。
    wdt #(.CLK_MHZ(CLK_MHZ)) u_wdt (
        .clk(clk),
        .rst(rst_wdt),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_wdt),
        .bus_sel(bus_sel_in[PER_WDT]),
        .ld_ready(wdt_ready),
        .bus_data_out(wdt_data),
        .wdt_irq(wdt_irq_w),
        .wdt_reset(wdt_reset)
    );

    plic u_plic (
        .clk(clk),
        .rst(rst_cpu),
        //源号：1=uart_rx 2=dma 3=gpio 4=i2c 5=总线错误 6=定时器 7=spi(flash/SD)
        //      8=spi(用户) 9=DPU 10=看门狗
        .irq_sources({21'd0, wdt_irq_w, dpu_irq, spi_user_irq, spi_pmem_irq,
                      tim_irq_w, err_valid, i2c_irq, gpio_irq, dma_irq, uart_rx_irq, 1'b0}),
        .exti(plic_exti),
        .bus_addr_in(bus_addr_in),
        .bus_data_in(bus_data_in),
        .bus_be_in(bus_be_in),
        .bus_we_in(bus_we_plic),
        .bus_sel(bus_sel_in[PER_PLIC]),
        .ld_ready(plic_ready),
        .bus_data_out(plic_data),
        .err_valid(err_valid),
        .err_write(err_write),
        .err_owner(err_owner),
        .err_addr(err_addr),
        .err_clear(err_clear)
    );

endmodule
