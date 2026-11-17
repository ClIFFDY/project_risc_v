`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/30 22:06:59
// Design Name:
// Module Name: uart_top
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


module uart_top #(
//★ 分频由核时钟算出来。原来写死 localparam CLK = 50 MHz ⇒ 核改 120 MHz 时
//  波特率会整体乘 2.4（9600 变 23040），对端必然收错。改核频率只改 CLK_MHZ。
//  mcnt 是 14 位：CLK/BPS_LOW 最大 16383 ⇒ 9600 只支持到 CLK_MHZ ≤ 157
//  （120 MHz 时 12500，余量够；再往上要么加宽 mcnt、要么降 BPS_LOW）。
    parameter integer CLK_MHZ  = 100,
    parameter integer BPS_LOW  = 9600,
    parameter integer BPS_HIGH = 115200
) (
    input clk, rst,
    input rx,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    // ★ 2026-11-14：事务选通。`bus_we_in` 是"本槽被选中 且 是写"，读这一路原来
    //   没有任何选通可用 —— 而 bus_addr_in（haddr>>2）在事务结束后**不撤**
    //   （主设备的 lat_addr 只在收到新请求时更新），于是"地址对上就当在读"的判据
    //   会一直成立。RX 弹出就是被这个反复触发的。这里把选择线单独引进来，
    //   读副作用只在被选中的那一拍（或一笔事务的第一拍）发生。
    input bus_sel,
    output tx,
    output reg [31:0] bus_data_out,
    output reg ld_ready,
    output reg rx_irq
    );

    localparam integer CLK = CLK_MHZ * 1_000_000;
    localparam integer PRESET_LOW  = CLK / BPS_LOW;
    /* 0 号预设 */
    localparam integer PRESET_HIGH = CLK / BPS_HIGH;
    /* 1 号预设 */

  reg [13:0] mcnt, mcnt_half, mcnt_3q;

//★ 2026-11-14：BAUD 寄存器从"两档选择"改成"任意分频"。
//  写入值的含义（向后兼容，老软件写 0/1 行为不变）：
//      0        → BPS_LOW 预设（9600）
//      1        → BPS_HIGH 预设（115200）
//      16..16383→ 直接当分频值用：mcnt = 核心时钟 / 目标波特率
//      2..15    → 保留，不改当前分频（那几档折合 3 MHz 以上，收发都不可靠）
//  读回口给的仍然是 {18'd0, mcnt}（uart.h 的 uart_baud_divider() 已经按这个读）。
//  ★ 分频必须是整数：50 MHz 下能整除的档位才没有误差 ——
//      25 → 2 Mbps、20 → 2.5 Mbps、16 → 3.125 Mbps 都是精确的；
//      921600 只能取 54（+0.47%），能用但不是零误差。
//  ★ 改分频会立刻改变移位节拍，只能在收发空闲时改（软件先 uart_set_baud_* 再收发）。
    always @(posedge clk) begin
        if (rst) begin
            mcnt <= PRESET_LOW;
            mcnt_half <= PRESET_LOW/2;
            mcnt_3q <= PRESET_LOW/2 + PRESET_LOW/4 - 1;
        end
        else if (bus_addr_in[23:20] == 4'b0011 && bus_we_in && bus_be_in[0]) begin
            if (bus_data_in[13:0] == 14'd0) begin
                mcnt <= PRESET_LOW;
                mcnt_half <= PRESET_LOW/2;
                mcnt_3q <= PRESET_LOW/2 + PRESET_LOW/4 - 1;
            end
            else if (bus_data_in[13:0] == 14'd1) begin
                mcnt <= PRESET_HIGH;
                mcnt_half <= PRESET_HIGH/2;
                mcnt_3q <= PRESET_HIGH/2 + PRESET_HIGH/4 - 1;
            end
            else if (bus_data_in[13:0] >= 14'd16) begin
                mcnt <= bus_data_in[13:0];
                mcnt_half <= {1'b0, bus_data_in[13:1]};
                mcnt_3q <= {1'b0, bus_data_in[13:1]} + {2'b0, bus_data_in[13:2]} - 1'b1;
            end
        end
    end

  wire [7:0] rx_data;
  reg [7:0] tx_data;
  reg  tx_en, rx_read, tx_busy, rx_cont;
  wire  rx_done, busy;

//★ 2026-11-15：RX 环 32 → 256 格。
//  原因：IDE 的二进制帧一帧动辄几十上百字节，32 格（可用 31）连一个 INFO 请求
//  都勉强，WRITE 更是一帧都收不全 —— 核是按 10 ms 节拍轮询的，1 Mbps 下 10 ms
//  就是 100 个字节，31 格根本兜不住。
//  256 格 ⇒ 一帧总长封顶 252 字节（留 4 格给"任务还没醒"的窗口）。
//  TX 环不动：往外发的节奏是设备自己排的，没有这个问题。
  reg [7:0] rx_buf [0:255];
  reg [7:0] tx_buf [0:31];
  reg [7:0] wr_ptr_rx, rd_ptr_rx;
  reg [4:0] wr_ptr_tx, rd_ptr_tx;
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1)
            rx_buf[i] = 8'b0;
        for (i = 0; i < 32; i = i + 1)
            tx_buf[i] = 8'b0;
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr_rx <= 8'd1;
            rd_ptr_rx <= 8'd0;
            wr_ptr_tx <= 5'd1;
            rd_ptr_tx <= 5'd0;
            rx_irq <= 1'd0;
            //★ 2026-11-16：tx_buf 的复位清零也删了，理由和下面 rx_buf 那段一样。
            //  原来这里有一句 `for (i...) tx_buf[i] = 8'b0;`，于是 32×8 = 256 位被
            //  推成 256 个带同步复位的触发器，复位网扇出 256 —— 接上 MIG 之后这条
            //  复位路径成了整颗核最差的一条（tx_buf_reg[6][4]/R，WNS -0.897）。
            //  tx_buf 读的是 rd_ptr_tx、写的是 wr_ptr_tx，复位后两者都回 1，
            //  缓冲是空的 ⇒ 旧数据永远读不到，清它没有意义。
            //  (rx_buf 在复位里【不清】：指针已经复位，旧数据永远读不到。
            //  清一遍是 2048 位同步复位，会把整块从 LUTRAM（32 个 LUT6）变成
            //  2048 个触发器 —— 为了一个没人读的状态付这个价不值。
        end
        else begin
            rx_irq <= 1'd0;
            if (rx_done && wr_ptr_rx != rd_ptr_rx) begin
                rx_buf[wr_ptr_rx] <= rx_data;
                wr_ptr_rx <= wr_ptr_rx + 1'b1;
            end
            if (bus_we_in && bus_addr_in[23:20] == 4'b0001 && bus_be_in[0] && wr_ptr_tx != rd_ptr_tx) begin
                tx_buf[wr_ptr_tx] <= bus_data_in[7:0];
                wr_ptr_tx <= wr_ptr_tx + 1'b1;
            end
            if (!busy && rd_ptr_tx != wr_ptr_tx - 1'b1) begin
                rd_ptr_tx <= rd_ptr_tx + 1'b1;
            end
            if (rx_read && rd_ptr_rx != wr_ptr_rx - 1'b1) begin
                rd_ptr_rx <= rd_ptr_rx + 1'b1;
            end
            if (rd_ptr_rx != wr_ptr_rx - 1'b1) begin
                rx_irq <= 1'd1;
            end
        end
    end

//★ 2026-11-14：一次 CPU 读事务会把地址连续保持好几拍（整核实测 25 拍）。
//  读侧原来是电平判据 ⇒ 每个保持拍都弹一格 FIFO ⇒ 一次 load 把整个 RX 环抽干，
//  只把第一个字节交给 CPU。通道口的表现是"每读一个字符就顺手丢掉后面那个"，
//  回车永远读不到，整行输入永远提交不了（tb_menu 抓到：发 "6"+CR，rr 从 0 走到 2，
//  固件只看到 '6'）。
//  所以弹栈只在【这次事务的第一拍】发生。
//  ★ 2026-11-14 晚（总线审计）：上面那个判据还留了一个洞 —— 写数据口那一拍
//  `!bus_we_in` 为假会把标志清 0，下一拍地址还指着 DATA、we 已撤，就变成"第一拍"
//  又弹一格（边发边收时每发一字节吞一个收字节）。现在改用**事务选通** `bus_sel`：
//  它由桥从 htrans 产生，真正只有一拍。读副作用 = 被选中 且 非写 且 本笔第一拍。
//  这是全仓统一的写法（spi.v/pmem_spi.v 一直这么做），别的读口照这个抄。
  reg  bus_sel_q;  // 上一拍是否选中本槽

    always @(posedge clk) begin
        if (rst) begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            rx_read <= 1'b0;
            bus_sel_q <= 1'b0;
        end
        else begin
            ld_ready <= 1'b0;
            rx_read <= 1'b0;
            bus_sel_q <= bus_sel;
//读口：**本槽里任何子地址都必须应答**。漏一个子地址，桥那边 rd_valid 永远不来，
//  LSU 的 load 永不完成 ⇒ 整核挂死，还会把 owner FIFO 永久顶死（见审计文档）。
            if (bus_sel && !bus_we_in && bus_addr_in[31:24] == 8'd1) begin
                ld_ready <= 1'b1;
                case (bus_addr_in[23:20])
                4'b0001: begin
                    bus_data_out <= rx_buf[rd_ptr_rx + 1'b1];
                    if (!bus_sel_q)
                        rx_read <= 1'b1;
                    // 一笔事务只弹一格
                end
                4'b0010: bus_data_out <= {30'd0, tx_busy, rx_cont};
                4'b0011: bus_data_out <= {18'd0, mcnt};
                default: bus_data_out <= 32'd0;
                // 未定义子地址：答 0，不挂死
                endcase
            end
        end
    end

    always @(*) begin
        tx_busy = 1'b0;
        rx_cont = 1'b0;
        tx_en = 1'b0;
        tx_data = 8'b0;
//★ 2026-11-14：tx_busy 的判据必须按【超前一格】的指针约定写。
//  这个 FIFO 读的是 tx_buf[rd_ptr_tx + 1]，所以
//      空   ⇔ rd_ptr_tx == wr_ptr_tx - 1
//      满   ⇔ wr_ptr_tx == rd_ptr_tx
//  历史上两次都写错了：
//    · 原来 `wr_ptr_tx == rd_ptr_tx` —— 那是"满"，不是"空"；
//    · 2026-11-13 改成 `wr_ptr_tx != rd_ptr_tx` —— 那是"不满"，复位后
//      wr=1/rd=0 就已经成立 ⇒ **常 1**（tb_uart_baud 的"空闲时 tx_busy=0"抓到）。
//  正确判据与 rx_cont（下面那行）同源：环里还有没发的字节，或者移位器在移。
//  位号不变（状态口仍是 {30'd0, tx_busy, rx_cont} ⇒ bit1=tx_busy、bit0=rx_cont），
//  因为 CoreMark 的 core_portme.c 已经按"bit0 = 有数据待取"写死了，别动位号。
        if ((rd_ptr_tx != wr_ptr_tx - 1'b1) || busy)
            tx_busy = 1'b1;
        if (rd_ptr_rx != wr_ptr_rx - 1'b1)
            rx_cont = 1'b1;
        if (!busy && rd_ptr_tx != wr_ptr_tx - 1'b1) begin
            tx_en = 1'b1;
            tx_data = tx_buf[rd_ptr_tx + 1'b1];
        end
    end

    uart_tx u_uart_tx(
        .clk(clk),
        .rst(rst),
        .tx_en(tx_en),
        .tx_data(tx_data),
        .mcnt(mcnt),
        //
        .tx(tx),
        .busy(busy)
    );

    uart_rx u_uart_rx(
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .mcnt(mcnt),
        .mcnt_half(mcnt_half),
        .mcnt_3q(mcnt_3q),
        //
        .rx_done(rx_done),
        .rx_data(rx_data)
    );
endmodule
