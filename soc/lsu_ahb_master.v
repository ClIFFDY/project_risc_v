`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: lsu_ahb_master
// Description: LSU 非缓存访问的 AHB 主设备。
//
//   ★ 2026-11-12 多主改造：LSU 给出的 bus_valid 是【一拍脉冲】，而 AHB 要求
//     地址保持到 HREADY & HGRANT 才撤。单主时代"发出去就一定被收"的隐含前提
//     在多主下不成立 ⇒ 这里把整个请求锁一份，见到 hgrant & hready 那一拍才撤。
//     没被授权时 htrans 保持 NONSEQ（这就是"我有一笔要发"的请求信号）。
//
//   完成判定仍是 rd_valid（读应答），写的应答不存在 —— 与改造前一致。
//////////////////////////////////////////////////////////////////////////////////


module lsu_ahb_master(
    input clk, rst,
    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    input             bus_valid_in,
    output reg [31:0] bus_data_out,
    output reg        bus_ready_out,
    output reg        bus_hold_out,
    output reg [31:0] haddr,
    output reg [1:0]  htrans,
    output reg        hwrite,
    output reg [2:0]  hsize,
    output reg [2:0]  hburst,
    output reg [3:0]  hprot,
    output reg        hmastlock,
    output reg [31:0] hwdata,
    output reg [3:0]  hwstrb,
    input      [31:0] hrdata,
    input             hready,
    input             hresp,
    input             rd_valid,
    input             hgrant
    );

    localparam [1:0] HTRANS_IDLE   = 2'b00;
    localparam [1:0] HTRANS_NONSEQ = 2'b10;

    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//请求锁存
    reg        pend;
    reg [31:0] lat_addr, lat_data;
    reg [3:0]  lat_be;
    reg        lat_we;

    wire accepted = pend & hgrant & hready;

    always @(posedge clk) begin
        if (reset_registered) begin
            pend     <= 1'b0;
            lat_addr <= 32'd0;
            lat_data <= 32'd0;
            lat_be   <= 4'd0;
            lat_we   <= 1'b0;
        end
        else begin
//★ 2026-11-12：接受那一拍正好来新请求时，新请求【直接顶上】（原地替换，不加存储）。
//  原写法 `bus_valid_in && !pend` 会把这一拍的新请求丢掉：LSU 的 stage1_valid_out 是
//  一拍脉冲，而本模块从收到到被接受要 2 拍（脉冲拍 + 发出拍），于是背靠背的非缓存
//  store 每隔一笔丢一笔。实测：FreeRTOS 配定时器时 `TIM_CNT_SET` 就是这么丢的
//  （TIM_MODE / TIM_CNT_SET / PLIC 三条连着写，只有后两条出去了）。
            if (bus_valid_in && (!pend || accepted)) begin
                pend     <= 1'b1;
                lat_addr <= bus_addr_in << 2;
                lat_data <= bus_data_in;
                lat_be   <= bus_be_in;
                lat_we   <= bus_we_in;
            end
            else if (accepted)
                pend <= 1'b0;
        end
    end

    always @(*) begin
        haddr  = lat_addr;
        htrans = pend ? HTRANS_NONSEQ : HTRANS_IDLE;
        hwrite = lat_we;
        hsize  = 3'b010;
        hburst = 3'b000;
//HPROT 按 ARM IHI 0033A §3.7 的推荐值：非缓存、非缓冲、特权、数据访问。
//本核没有用户态，也不产生取指（取指走 L2 的 I 口），所以恒这个值。
        hprot  = 4'b0011;
//HMASTLOCK：RV32I 没有 SWP、也没有 A 扩展，本核从不发锁传输 ⇒ 恒 0。
        hmastlock = 1'b0;
        hwstrb = lat_be;
        hwdata = lat_data;
        bus_data_out  = hrdata;
        bus_ready_out = rd_valid;
//★ 2026-11-12：在飞期间反压上游。LSU 的请求是【一拍脉冲】，本模块却要 2 拍才把它
//  送上总线；总线被 L2/DMA 占住时 pend 会挂很久，这期间来的脉冲本来会被直接丢掉
//  （实测：FreeRTOS 配 PLIC 时 UART 的 'S' 就这么没了 —— 总线被 L2 回填占了 25 拍）。
//  这一根就是那个"别发"信号：LSU 拿它挡 store 排空（见 lsu.v 的 store_queue_drain）。
        bus_hold_out  = pend;
    end

endmodule
