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
//     不是在这里硬连。软件把 pin0 配 TX、pin1 配 RX。
//   ★ bit 映射按排针号：J1-N → gpio_pin_bus[N-1]（N = 1..7, 11..18）。
//     UART 用的就是 J1-1（AD21，bit0 = TX）与 J1-2（AB21，bit1 = RX）。
//////////////////////////////////////////////////////////////////////////////////

module k7_board_top(
    input  sys_clk_p,
    input  sys_clk_n,
    inout  wire [14:0] j1
    );

//--------------------------------------------------------------- 时钟
    wire clk_50;

    IBUFDS #(
        .DIFF_TERM   ("TRUE"),
        .IOSTANDARD  ("LVDS_25")
    ) u_sys_clk_ibuf (
        .I  (sys_clk_p),
        .IB (sys_clk_n),
        .O  (clk_50)
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

    always @(posedge clk_50) begin
        case (power_on_state)
        2'd0: begin                       // 高 1.3 ms
            rst_por <= 1'b1;
            if (power_on_count == 16'hFFFF) begin
                power_on_count <= 16'd0;
                power_on_state <= 2'd1;
            end
            else begin
                power_on_count <= power_on_count + 16'd1;
            end
        end
        2'd1: begin                       // 低 1.3 ms：下降沿把复位压住
            rst_por <= 1'b0;
            if (power_on_count == 16'hFFFF) begin
                power_on_count <= 16'd0;
                power_on_state <= 2'd2;
            end
            else begin
                power_on_count <= power_on_count + 16'd1;
            end
        end
        default: begin                    // 之后恒高：rst_buf 数 42 ms 才放开
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

    wire [31:0] gpio_bus;

    genvar g;
    generate
        for (g = 0; g < 15; g = g + 1) begin : j1_pad
            tran (j1[g], gpio_bus[j1_bit(g)]);
        end
    endgenerate

//--------------------------------------------------------------- MCU
// ★ DDR 下游还没接（MIG 未进）。桥的 DDR_PRESENT=0 ⇒ 任何访问 0x8000_0000-0xBFFF_FFFF
//   的传输都会被回成总线 ERROR（错误日志 plic 0x0900_0000 能看见），不挂死、不静默回 0。
//   MIG 将来例化在 mcu_top 里面（AXI 链路不出 mcu_top 的边界）—— 顶层的 200 多根
//   AXI 脚没有引脚约束时布局器会直接死，别再引出来。
    mcu_top u_mcu (
        .clk           (clk_50),
        .rst           (rst_por),
        .gpio_pin_bus  (gpio_bus)
    );

endmodule
