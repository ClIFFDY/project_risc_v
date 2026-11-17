`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: k7_board_top
// Description:
//   xc7k160t-ffg676-2 板级顶层。只做三件事：
//     1. 板载 50 MHz 差分时钟（R21/P21，LVDS）→ IBUFDS → 全核单时钟 clk_50
//        （UART 的 CLK 常数就是 50 MHz，全程不倍频）
//     2. 上电复位脉冲（见下）
//     3. J1 排针 ↔ mcu_top 的 gpio_pin_bus：用 tran 把 15 根已解出球号的脚
//        双向接到对应 bit（gpio_8 = J1-8 球号未解出，跳过）
//
//   ★ 哪根脚当 UART：由 gpio_group 的 MODE 寄存器配（TX = 4'b0101 / RX = 4'b0110），
//     不是在这里硬连。软件把 pin0 配 TX、pin1 配 RX；I2C 同理（SCL = 4'b1001 /
//     SDA = 4'b1010），本仓的上板实验用 pin2 = J1-3、pin3 = J1-4。
//     ★ I2C 是开漏，模块只会拉低 ⇒ SCL/SDA 到 3.3V 必须有上拉电阻（4.7k）。
//   ★ bit 映射按排针号：J1-N → gpio_pin_bus[N-1]（N = 1..7, 11..18）。
//     UART 用的就是 J1-1（AD21，bit0 = TX）与 J1-2（AB21，bit1 = RX）。
//////////////////////////////////////////////////////////////////////////////////

module k7_board_top #(
//★ 系统镜像从哪块 flash 搬：0 = 数据盘 Q128（现状，上板在用的那条），
//  1 = 配置盘 Q64 的 0x70_0000。切 1 之前必须先把镜像烧进 Q64，否则不开机。
    parameter integer BOOT_FROM_CFG = 0,
//★ 核时钟频率（MHz）。板载晶振固定 50，经 soc/clk_gen.v 的 MMCM 变成这个频率。
//  这一个参数同时决定：MMCM 的 M/D、UART 波特率分频、I2C SCL、SPI 分频。
//  ★ 2026-11-15 定档 120，和上游 mcu_top / io_block_top 的默认值一致。
//    改它必须同时改 Tools/freertos/FreeRTOSConfig.h 与 Tools/sys 的 BOARD_CLK_MHZ，
//    以及固件里所有按拍数写的时间常数（两边差 2.4 倍时，表现是"串口乱码 + 息屏时间不对"）。
//  ★ 不要在这里配时序约束：晶振输入永远是 50 MHz（20 ns，在 XDC 里），
//    输出时钟由 Vivado 从 MMCM 的 M/D 自动派生，改这个参数它会跟着变。
    parameter integer CLK_MHZ = 100
)(
    input  sys_clk_p,
    input  sys_clk_n,
//---- 板载配置 flash（25Q064A）的专用配置脚 ----
//   CS/DI/DO 是普通 IO，走约束；CLK 是 CCLK_0 专用脚，必须从 STARTUPE2 出，
//   所以没有 cfg_sck 这个顶层端口。
    input  cfg_miso,
    output cfg_cs_n,
    output cfg_mosi,
    inout  wire [14:0] j1,
//---- 板载 DDR3（2 × K4B2G1646 = 2Gb x16 两颗、32 位、单 rank、512 MB）----
//  引脚约束在 constrs_1/new/ddr3_pins.xdc（从 docs/k7160Tpin.csv 转出来的）。
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

//--------------------------------------------------------------- 上电复位
// ★ 复位脉冲由 soc/rst_buf.v 生成，它的时序是：
//     rst 的【下降沿】开始复位 → rst 回到高之后计数 2^21 拍（50 MHz 下 42 ms）→ 放开。
//   rst 一直为低就卡在复位里（计数器只在 rst 为高时走）。
//   所以这里造"高 → 低 → 高"的脉冲：高 1.3 ms、低 1.3 ms、之后恒高
//   ⇒ 内核大约在上电后 45 ms 开始跑。
    reg [15:0] power_on_count = 16'd0;
    reg [1:0]  power_on_state = 2'd0;
    reg        rst_por        = 1'b1;
//--------------------------------------------------------------- 时钟
    wire clk_50;
    wire [31:0] gpio_bus;
    wire cfg_sck_int;

    wire clk_50_ibuf;
    IBUFDS #(
        .DIFF_TERM   ("TRUE"),
        .IOSTANDARD  ("LVDS_25")
    ) u_sys_clk_ibuf (
        .I  (sys_clk_p),
        .IB (sys_clk_n),
        .O  (clk_50_ibuf)
    );

//★ 2026-11-16：IBUFDS 输出先过一级 BUFG 再去两个 MMCM。
//  7 系列里一处 IBUFDS 只能直连一个 MMCM 的 CLKIN，第二个要么换 IO 要么全局走线；
//  显式加 BUFG 之后 120 MHz（核）和 200 MHz（MIG）两路都能从同一根晶振长出来。
    BUFG u_bufg_sys_clk (
        .I (clk_50_ibuf),
        .O (clk_50)
    );

//--------------------------------------------------------------- 核时钟（MMCM）
//★ 2026-11-15：挂上 PLL。晶振 50 MHz 进 MMCM，出来的 clk_core 才是核时钟，
//  频率由顶部参数 CLK_MHZ 决定（默认 50，和挂 PLL 之前实测行为一致）。
//  上电复位脉冲仍在 clk_50 域产生（晶振一起来就有，不依赖 PLL）；
//  进核域时再和 locked 相与 ⇒ PLL 锁不住就一直复位，不会带着抖动的时钟跑。
    wire clk_core;
    wire pll_locked;

    clk_gen #(
        .IN_MHZ (50),
        .OUT_MHZ(CLK_MHZ)
    ) u_clk_gen (
        .clk_in (clk_50),
        //★ rst_por 的【稳态是高】（高 1.3 ms → 低 1.3 ms → 之后恒高），而 MMCME2 的 RST
        //  是高有效。直接接过去就等于把 MMCM 永久按在复位里 ⇒ LOCKED 永不来 ⇒ 核不启动，
        //  表现就是屏幕全黑。必须取反：低脉冲那段当复位，之后放开。
        .rst_in (~rst_por),
        .clk_out(clk_core),
        .locked (pll_locked)
    );

//--------------------------------------------------------------- MIG 的 200 MHz
//★ 2026-11-16：MIG 的系统时钟（也是 IDELAYCTRL 的参考时钟）要 200 MHz，而板子上那颗
//  200 MHz 晶振的球号一直没定位到（docs/k7160Tpin.csv 里 osc_200mhz 那行是 UNKNOWN）。
//  所以自己长一根：50 → 200（clk_gen 反解出来 DIV=4、MULT=16，VCO 800，正好整数）。
//  MIG 里系统时钟和参考时钟都选的是 No Buffer，所以这根是直接进 fabric 的，不占引脚。
//  ★ 它和核时钟是两个独立的 MMCM，互不影响：核跑 120，MIG 这边 200（ui_clk 100）。
    wire clk_200;
    wire pll200_locked;

    clk_gen #(
        .IN_MHZ (50),
        .OUT_MHZ(200)
    ) u_clk_gen_200 (
        .clk_in (clk_50),
        .rst_in (~rst_por),
        .clk_out(clk_200),
        .locked (pll200_locked)
    );

    reg [2:0]  por_sync   = 3'b000;    reg [16:0] rst_hold   = 17'd0;
    reg        rst_core_r = 1'b1;

    always @(posedge clk_core) begin
        por_sync <= {por_sync[1:0], (rst_por & pll_locked)};
        if (!(&por_sync)) begin
            // 等 PLL 锁定 + 复位脉冲结束：这时候还不能给 rst_buf 任何边沿
            rst_hold   <= 17'd0;
            rst_core_r <= 1'b1;
        end
        else if (!rst_hold[16]) begin
            //★ rst_buf 要的是【高 → 低 → 高】这个形状：下降沿开始复位，
            //  重新拉高之后再数 2^21 拍才放开。所以这里必须真的造一个低脉冲，
            //  不能只把 ~locked 直接接过去（那样只有一次下降沿，永远等不到上升沿，
            //  整核卡在复位里 —— 屏全黑就是这个症状）。
            rst_hold   <= rst_hold + 17'd1;
            rst_core_r <= 1'b0;
        end
        else begin
            rst_core_r <= 1'b1;
        end
    end

    wire rst_core;
    assign rst_core = rst_core_r;


    always @(posedge clk_50) begin
        case (power_on_state)
        2'd0: begin // 高 1.3 ms
            rst_por <= 1'b1;
            if (power_on_count == 16'hFFFF) begin
                power_on_count <= 16'd0;
                power_on_state <= 2'd1;
            end
            else begin
                power_on_count <= power_on_count + 16'd1;
            end
        end
        2'd1: begin // 低 1.3 ms：下降沿把复位压住
            rst_por <= 1'b0;
            if (power_on_count == 16'hFFFF) begin
                power_on_count <= 16'd0;
                power_on_state <= 2'd2;
            end
            else begin
                power_on_count <= power_on_count + 16'd1;
            end
        end
        default: begin // 之后恒高：rst_buf 数 42 ms 才放开
            rst_por <= 1'b1;
        end
        endcase
    end

//--------------------------------------------------------------- J1 排针 ↔ gpio_pin_bus
// j1 的第 g 根（g = 0..14）对应排针 J1-(g+1)（g < 7）或 J1-(g+4)（g >= 7），
// 即 gpio_pin_bus 的 bit 号 = 排针号 - 1。
    function integer j1_bit;
        input integer g;
        begin
            j1_bit = (g < 7) ? g : (g + 3);
        end
    endfunction

    genvar g;
    generate
        for (g = 0; g < 15; g = g + 1) begin
            : j1_pad
            tran (j1[g], gpio_bus[j1_bit(g)]);
        end
    endgenerate

//--------------------------------------------------------------- MCU
// ★ 2026-11-16：DDR 已经接上（MIG 在 mcu_top 里面，AXI 链路不出 mcu_top 的边界）。
//   这里的 ddr3_* 是真正的颗粒引脚，约束在 ddr3_pins.xdc；clk_200 给 MIG 当
//   系统时钟 + IDELAYCTRL 参考时钟。
//   AXI 那 200 多根脚不引到顶层：没有引脚约束的 IO 会让布局器直接死
//   （实测 103 个 [Place 30-99] ERROR）。
    mcu_top #(
// ★ 核时钟来自 clk_gen（见上），所以 CLK_MHZ 要和 clk_gen 的 OUT_MHZ 是同一个参数。
//   这个参数是 UART 波特率 / I2C SCL / SPI 分频的唯一来源，漏掉它会全偏。
        .CLK_MHZ(CLK_MHZ),
        .BOOT_FROM_CFG(BOOT_FROM_CFG),
        .DDR_PRESENT(1)
    ) u_mcu (
        .clk           (clk_core),
        .rst           (rst_core),
        .clk_200       (clk_200),
        .cfg_miso      (cfg_miso),
        .cfg_cs_n      (cfg_cs_n),
        .cfg_sck       (cfg_sck_int),
        .cfg_mosi      (cfg_mosi),
        .gpio_pin_bus  (gpio_bus),
        .ddr3_dq       (ddr3_dq),
        .ddr3_dqs_p    (ddr3_dqs_p),
        .ddr3_dqs_n    (ddr3_dqs_n),
        .ddr3_addr     (ddr3_addr),
        .ddr3_ba       (ddr3_ba),
        .ddr3_ras_n    (ddr3_ras_n),
        .ddr3_cas_n    (ddr3_cas_n),
        .ddr3_we_n     (ddr3_we_n),
        .ddr3_reset_n  (ddr3_reset_n),
        .ddr3_ck_p     (ddr3_ck_p),
        .ddr3_ck_n     (ddr3_ck_n),
        .ddr3_cke      (ddr3_cke),
        .ddr3_cs_n     (ddr3_cs_n),
        .ddr3_dm       (ddr3_dm),
        .ddr3_odt      (ddr3_odt)
    );

//--------------------------------------------------------------- 配置 flash 的时钟脚
//CCLK_0 是专用配置脚，普通 ODDR/OBUF 驱动不了，只能从 STARTUPE2 的 USRCCLKO 出去。
//  KEYCLEARB 拉高、USRCCLKTS 拉低 = 把 CCLK 的控制权彻底交给用户逻辑。
//仿真里没有这个原语，用 SYNTHESIS 包起来；cfg_sck_int 在仿真里就悬着，不影响功能。
`ifdef SYNTHESIS
    STARTUPE2 #(
        .PROG_USR     ("FALSE"),
        .SIM_CCLK_FREQ(0.0)
    ) u_startup (
        .CFGCLK    (),
        .CFGMCLK   (),
        .EOS       (),
        .PREQ      (),
        .CLK       (1'b0),
        .GSR       (1'b0),
        .GTS       (1'b0),
        .KEYCLEARB (1'b1),
        .PACK      (1'b0),
        .USRCCLKO  (cfg_sck_int),
        .USRCCLKTS (1'b0),
        .USRDONEO  (1'b1),
        .USRDONETS (1'b1)
    );
`endif

endmodule
