`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: clk_gen
// Description: 板载 50 MHz 晶振 → 核时钟（MMCME2_ADV）
//
//   为什么要有这个模块：板子原来不挂 PLL，IBUFDS 出来就是 50 MHz，核直接跑 50。
//   要扫频率、要知道这颗硅片到底能跑多快，就必须有一个可以设的时钟源。
//
//   ★ 只改 OUT_MHZ 一个参数。VCO 与分频由它反解：
//       选分频 DIV ∈ {1,2,4,8,16,32,64}，让 OUT×DIV 落进 MMCM 的 VCO 区间 600~1200 MHz，
//       然后 MULT = VCO / IN。这样不用手算 M/D，换频率只改一个数。
//     例（IN=50）：OUT=50 → DIV=16, VCO=800, MULT=16.0
//                  OUT=100→ DIV=8,  VCO=800, MULT=16.0
//                  OUT=120→ DIV=8,  VCO=960, MULT=19.2
//                  OUT=130→ DIV=8,  VCO=1040,MULT=20.8
//
//   ★ OUT_MHZ 必须同时传给 mcu_top 的 CLK_MHZ（UART 波特率 / I2C SCL / SPI 分频
//     全靠它算），两个不一致 → 串口波特率偏，看起来像"板子坏了"。
//
//   ★ 复位的用法见 k7_board_top：RST 接上电复位脉冲，输出 locked 参与核复位，
//     锁不住就一直复位（而不是带着一个抖的时钟跑起来）。
//////////////////////////////////////////////////////////////////////////////////

module clk_gen #(
    parameter integer IN_MHZ  = 50,
    parameter integer OUT_MHZ = 50
)(
    input  clk_in,
    input  rst_in,
    output clk_out,
    output locked
);

//VCO 目标区间（MMCME2，-1/-2 档都是 600~1200 MHz）
    localparam integer VCO_MIN = 600;
    localparam integer VCO_MAX = 1200;

    function integer pick_div;
        input integer out_mhz;
        integer d;
        begin
            pick_div = 0;
            //★ 取【第一个】满足区间的 d（也就是 VCO 尽量低），不要取最后一个。
            //  150 MHz 时 VCO 会落在 600 和 1200 两个边界上，取低的那个更稳。
            for (d = 1; d <= 64; d = d * 2)
                if ((pick_div == 0) && (out_mhz * d >= VCO_MIN) && (out_mhz * d <= VCO_MAX))
                    pick_div = d;
            if (pick_div == 0)
                pick_div = 8;
        end
    endfunction

    localparam integer DIV    = pick_div(OUT_MHZ);
//★ CLKFBOUT_MULT_F 的合法步进是 0.125，不是任意实数。所以先把 MULT 量化到 1/8，
//  否则像 19.2 这种写进去 Vivado 会四舍五入或直接报错。
    localparam integer MUL_X8 = ((OUT_MHZ * DIV * 8) + (IN_MHZ / 2)) / IN_MHZ;
    localparam real    MULT_R = MUL_X8 / 8.0;
    localparam real    DIV_R  = DIV * 1.0;
    localparam real    INPER  = 1000.0 / (IN_MHZ * 1.0);
//实际出来的频率（可能和 OUT_MHZ 差一点点，因为 MULT 只能取 1/8 的整数倍）
    localparam integer ACT_KHZ = (IN_MHZ * MUL_X8 * 125) / DIV;
//例：IN=50, OUT=120 → DIV=8, MUL=19.125, VCO=956.25 MHz, 实际 119.53 MHz（偏 0.4%，波特率吃得下）

    wire clkfb_unbuf;
    wire clkfb;
    wire clk_out_raw;

    MMCME2_ADV #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKOUT4_CASCADE    ("FALSE"),
        .COMPENSATION       ("ZHOLD"),
        .STARTUP_WAIT       ("FALSE"),
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (MULT_R),
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (DIV_R),
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT0_PHASE      (0.000),
        .CLKIN1_PERIOD      (INPER),
        .REF_JITTER1        (0.010)
    ) u_mmcm (
        .CLKOUT0   (clk_out_raw),
        .CLKFBOUT  (clkfb_unbuf),
        .CLKFBIN   (clkfb),
        .CLKIN1    (clk_in),
        .CLKIN2    (1'b0),
        .CLKINSEL  (1'b1),
        .DADDR     (7'd0),
        .DCLK      (1'b0),
        .DEN       (1'b0),
        .DI        (16'd0),
        .DO        (),
        .DRDY      (),
        .DWE       (1'b0),
        .PSCLK     (1'b0),
        .PSEN      (1'b0),
        .PSINCDEC  (1'b0),
        .PSDONE    (),
        .RST       (rst_in),
        .PWRDWN    (1'b0),
        .LOCKED    (locked)
    );

    BUFG u_bufg_fb (
        .I (clkfb_unbuf),
        .O (clkfb)
    );

    BUFG u_bufg_out (
        .I (clk_out_raw),
        .O (clk_out)
    );

endmodule
