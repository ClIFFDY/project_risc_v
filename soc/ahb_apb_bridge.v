`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: ahb_apb_bridge
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


module ahb_apb_bridge(
    input clk, rst,
    input      [31:0] haddr,
    input      [1:0]  htrans,
    input             hwrite,
    input      [2:0]  hsize,
    input      [2:0]  hburst,
    input      [3:0]  hprot,
    input             hmastlock,
    input      [31:0] hwdata,
    input      [3:0]  hwstrb,
    output reg [31:0] hrdata,
//标准 AHB-Lite 从设备口。rd_valid 是本仓自己的完成脉冲（非标准），
//第一期先和 HREADYOUT 并存，等三个主设备都改认 HREADY/HRESP 之后再删。
    output reg            hreadyout,
    output reg            hresp,
    output reg        rd_valid,
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0]  bus_be_out,
    output reg        bus_we_out,
    output reg [31:0] bus_sel_out,
    input      [31:0] uart_data,
    input             uart_ready,
    input      [31:0] plic_data,
    input             plic_ready,
    input      [31:0] gpio_data,
    input             gpio_ready,
    input      [31:0] i2c_data,
    input             i2c_ready,
//★ 2026-11-12：DMA 挂 3 号槽（0x0C00_0000）。它不是 APB 外设，但在本仓的约定里
//  所有从设备都共用这一组"字节地址 >> 2"的广播总线，各自按槽号认领 ⇒ 接法一致。
    input      [31:0] dma_data,
    input             dma_ready,
//★ 2026-11-12：定时器/PWM 挂 6 号槽（0x1800_0000）。FreeRTOS 的节拍源，gpio_group
//  的 PWM1~4 模式也吃它的四路输出。接法与其它外设相同（字节地址 >> 2 广播 + 槽号认领）。
    input      [31:0] tim_data,
    input             tim_ready,
//★ 2026-11-12：SPI 主机挂 7 号槽（0x1C00_0000），接板上的 flash。
    input      [31:0] spi_data,
    input             spi_ready,
//★ 2026-11-13：DPU（显示引擎）挂 9 号槽（0x2400_0000）。8 号槽是 NPU（直连互联，
//  不走 APB），所以槽位译码从 1..7 放开到 1..9，中间那格空着不占。
    input      [31:0] dpu_data,
    input             dpu_ready
    );

    localparam [5:0]
    SLOT_NONE = 6'd0,
    SLOT_UART = 6'd1,
    SLOT_PLIC = 6'd2,
    SLOT_DMA  = 6'd3,
    SLOT_GPIO = 6'd4,
    SLOT_I2C  = 6'd5,
    SLOT_TIM  = 6'd6,
    SLOT_SPI  = 6'd7,
    SLOT_DPU  = 6'd9;

    reg [5:0] slot;
    always @(*) begin
        if (haddr[31:26] >= 6'd1 && haddr[31:26] <= 6'd9)
            slot = haddr[31:26];
        else
            slot = SLOT_NONE;
    end

    reg [5:0] sel_pend;
    reg pend_v, pend_we;
    always @(posedge clk) begin
        if (rst) begin
            sel_pend <= SLOT_NONE;
            pend_v   <= 1'b0;
            pend_we  <= 1'b0;
        end
        else begin
            pend_v  <= (htrans != 2'b00);
            pend_we <= hwrite;
            if (htrans != 2'b00)
                sel_pend <= slot;
        end
    end

    always @(*) begin
        bus_addr_out = haddr >> 2;
        bus_data_out = hwdata;
        bus_be_out   = hwstrb;
        bus_we_out   = hwrite && (htrans != 2'b00);
        bus_sel_out  = 32'd0;
        bus_sel_out[slot] = (htrans != 2'b00);
    end

    reg sel_ld_ready;
    always @(*) begin
        case (sel_pend)
        SLOT_UART: sel_ld_ready = uart_ready;
        SLOT_PLIC: sel_ld_ready = plic_ready;
        SLOT_DMA:  sel_ld_ready = dma_ready;
        SLOT_GPIO: sel_ld_ready = gpio_ready;
        SLOT_I2C:  sel_ld_ready = i2c_ready;
        SLOT_TIM:  sel_ld_ready = tim_ready;
        SLOT_SPI:  sel_ld_ready = spi_ready;
        SLOT_DPU:  sel_ld_ready = dpu_ready;
        default:   sel_ld_ready = 1'b1;
        endcase
    end

    reg [31:0] hrdata_c;
    reg rd_valid_c;

//==============================================================================
//  未映射地址 → 两拍 ERROR（ARM IHI 0033A §5.1.3）
//    第一拍 HRESP=ERROR & HREADYOUT=0（把主设备按住）
//    第二拍 HRESP=ERROR & HREADYOUT=1（结束传输）
//  ★ 必须是【状态机】，不能把 err_first 做成纯组合：
//      hreadyout=0 ⇒ 主设备不撤 ⇒ htrans 一直高 ⇒ err_first 一直高 ⇒ 死锁（踩过）。
//  ★ 本仓的总线没有独立的地址/数据相位寄存器，这两拍锚在【地址相位】上，
//    比严格 AHB 从设备早一拍。功能等价：HREADYOUT=0 一样把主设备按住一拍。
//  ★ 槽位 0 与 7~63 没有外设认领；这个洞还包含 0x0001_0000~0x03FF_FFFF
//    这一整片（互联只把 haddr[31:16]==0 判给 STCM）。以前这些地址的读
//    根本不产生 rd_valid ⇒ 外设 load 卡在 ELSQ 队头 ⇒ 整核死锁。
//==============================================================================
    localparam [1:0] ERR_IDLE = 2'd0, ERR_FIRST = 2'd1, ERR_SECOND = 2'd2;

    reg [1:0] err_state;
    reg       err_done;
    reg       err_we_q;              // 出错那一笔是不是写（完成脉冲只给读）

    always @(posedge clk) begin
        if (rst) begin
            err_state <= ERR_IDLE;
            err_done  <= 1'b0;
            err_we_q  <= 1'b0;
        end
        else begin
//★ 2026-11-14（总线审计）：完成脉冲必须和正常读一样落在【接受后第 2 拍】。
//  互联的 owner FIFO 是单指针顺序出队，代码里写明的隐含假设是"两个从设备的读延迟相同"。
//  原来 err_done 要等 ERR_SECOND 之后才寄存（晚 2 拍 ⇒ 总共 +4），
//  错误读后面紧跟的那一笔读的应答会落在"队头还指着错误项"的那几拍里被吞掉，
//  之后也不会再来 ⇒ 那个主设备永久等应答。改成地址拍之后一拍出（= +2，与正常读对齐）。
//  两拍 ERROR 的 hresp/hreadyout 形状不动（那是给将来会看 HRESP 的主设备留的）。
            err_done <= (err_state == ERR_IDLE) && (htrans != 2'b00) && (slot == SLOT_NONE);
            if ((err_state == ERR_IDLE) && (htrans != 2'b00) && (slot == SLOT_NONE))
                err_we_q <= hwrite;
            case (err_state)
            ERR_IDLE:   if ((htrans != 2'b00) && (slot == SLOT_NONE)) err_state <= ERR_FIRST;
            ERR_FIRST:  err_state <= ERR_SECOND;
            ERR_SECOND: err_state <= ERR_IDLE;
            default:    err_state <= ERR_IDLE;
            endcase
        end
    end

    always @(*) begin
        hreadyout = (err_state != ERR_FIRST);
        hresp = (err_state == ERR_FIRST) || (err_state == ERR_SECOND);
    end

    always @(*) begin
        case (sel_pend)
        SLOT_UART: hrdata_c = uart_data;
        SLOT_PLIC: hrdata_c = plic_data;
        SLOT_DMA:  hrdata_c = dma_data;
        SLOT_GPIO: hrdata_c = gpio_data;
        SLOT_I2C:  hrdata_c = i2c_data;
        SLOT_TIM:  hrdata_c = tim_data;
        SLOT_SPI:  hrdata_c = spi_data;
        SLOT_DPU:  hrdata_c = dpu_data;
        default:   hrdata_c = 32'd0;
        endcase
        rd_valid_c = (pend_v && !pend_we && (sel_pend != SLOT_NONE) && sel_ld_ready)
                   | (err_done && !err_we_q);
//出错那一笔**读**照样给一个确定的完成（数据 0）：第一期 LSU 还不认 HRESP，
//不给完成就会挂死（见 docs/SPEC_bus_error_2026-11-12.txt 第七节）。第二期删。
//★ 2026-11-14（总线审计）：写错误不能给 —— 互联是无条件出队，一次写空洞会弹掉
//  队头一条**读**项，把 0 当数据送给那条读的主人。所以用 err_we_q 区分。
        if (err_done)
            hrdata_c = 32'd0;
    end

//读回再打一拍才交出去。四家外设的 ld_ready 与 bus_data_out 本来就都是寄存器，中间只隔
//sel_pend 的这一个 4:1 mux —— 这一级是纯组合的，原先一路穿过 u_io_block → u_bus_con →
//u_cpu_top/u_lsu 的前递选择端，是全部失败路径的共同头部。打一拍把它断在寄存器 D 端，
//下游从寄存器 Q 重新起算一条短链。代价：外设读应答晚一拍（lsu 侧由 ld_fifo 的深度接住）。
    always @(posedge clk) begin
        if (rst) begin
            hrdata   <= 32'd0;
            rd_valid <= 1'b0;
        end
        else begin
            hrdata   <= hrdata_c;
            rd_valid <= rd_valid_c;
        end
    end

endmodule
