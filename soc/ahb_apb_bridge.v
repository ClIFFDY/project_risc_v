`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ahb_apb_bridge
// Description:
//   AHB-Lite 从机 → 扁平外设总线（本仓外设不是 APB，只有
//   addr/data/be/we/sel → data/ld_ready 这几个信号，语义上等价 APB 的
//   SETUP/ACCESS + PREADY，所以按 APB 的做法生成握手）。
//
//   协议（AHB-Lite）：
//     地址相：HTRANS≠IDLE 且 HREADYIN=1 被接收 → 锁存地址/写/字节使能
//     数据相：给外设【一拍】sel 脉冲（外设在这一拍干活），然后等它 ld_ready
//             最后一拍 HREADYOUT=1，HRDATA 必须同拍有效
//     错误：  未映射槽位 → HRESP=1，按规范两拍（先 HREADYOUT=0 再 =1）
//
//   ★ 2026-11-02 改（原实现的三处不合规）：
//     ① HREADY 恒 1：外设要下一拍才回数据，桥只能靠 rd_valid 旁路把数据"事后"
//        塞给主机 —— 不是 AHB。现在数据在 HREADYOUT=1 的那一拍出。
//     ② 没有 HRESP：未映射地址静默返回 0。现在回 ERROR。
//     ③ 没有 HREADYIN：多从机场景下无法接续。现在按规范接。
//////////////////////////////////////////////////////////////////////////////////

module ahb_apb_bridge(
    input clk, rst,
    input      [31:0] haddr,
    input      [1:0]  htrans,
    input             hwrite,
    input      [2:0]  hsize,
    input      [2:0]  hburst,
    input      [31:0] hwdata,
    input      [3:0]  hwstrb,
    input             hreadyin,
    output reg [31:0] hrdata,
    output reg        hreadyout,
    output reg        hresp,
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0]  bus_be_out,
    output reg        bus_we_out,
    output reg [31:0] bus_sel_out,
    input      [31:0] uart_data,
    input             uart_ready,
    input      [31:0] plic_data,
    input             plic_ready,
    input      [31:0] tim_data,
    input             tim_ready,
    input      [31:0] gpio_data,
    input             gpio_ready,
    input      [31:0] i2c_data,
    input             i2c_ready,
    input      [31:0] qspi_data,
    input             qspi_ready
    );

//槽位号 = haddr[31:26]，也就是 CPU 字节地址的 [29:24]
    localparam [5:0]
    SLOT_NONE = 6'd0,
    SLOT_UART = 6'd1,
    SLOT_PLIC = 6'd2,
    SLOT_TIM  = 6'd3,
    SLOT_GPIO = 6'd4,
    SLOT_I2C  = 6'd5,
    SLOT_QSPI = 6'd7;

    reg [5:0] slot;
    always @(*) begin
        case (haddr[31:26])
        6'd1:    slot = SLOT_UART;
        6'd2:    slot = SLOT_PLIC;
        6'd3:    slot = SLOT_TIM;
        6'd4:    slot = SLOT_GPIO;
        6'd5:    slot = SLOT_I2C;
        6'd7:    slot = SLOT_QSPI;
        default: slot = SLOT_NONE;
        endcase
    end

    localparam [2:0]
        B_IDLE = 3'd0,   // 等地址相
        B_ACC  = 3'd1,   // 给外设一拍脉冲
        B_WAIT = 3'd2,   // 等外设 ld_ready（= PREADY）
        B_ERR1 = 3'd3,   // 错误响应第一拍：HRESP=1, HREADYOUT=0
        B_ERR2 = 3'd4;   // 错误响应第二拍：HRESP=1, HREADYOUT=1

    reg [2:0]  state;
    reg [31:0] lat_addr, lat_wdata;
    reg [3:0]  lat_be;
    reg        lat_we;
    reg [5:0]  lat_slot;

    wire accept = (htrans != 2'b00) && hreadyin;

//外设应答多路（按【锁存后】的槽位选，不能用当拍组合的 slot）
    reg        sel_ld_ready;
    reg [31:0] sel_rdata;
    always @(*) begin
        case (lat_slot)
        SLOT_UART: begin sel_ld_ready = uart_ready; sel_rdata = uart_data; end
        SLOT_PLIC: begin sel_ld_ready = plic_ready; sel_rdata = plic_data; end
        SLOT_TIM:  begin sel_ld_ready = tim_ready;  sel_rdata = tim_data;  end
        SLOT_GPIO: begin sel_ld_ready = gpio_ready; sel_rdata = gpio_data; end
        SLOT_I2C:  begin sel_ld_ready = i2c_ready;  sel_rdata = i2c_data;  end
        SLOT_QSPI: begin sel_ld_ready = qspi_ready; sel_rdata = qspi_data; end
        default:   begin sel_ld_ready = 1'b1;       sel_rdata = 32'd0;     end
        endcase
    end

//★ 外设侧必须用【组合】脉冲：B_ACC 那一拍 sel/we 就是 1。
//  写成寄存器会在进入 B_WAIT 之后才拉高（实测外设晚一拍收到，白等 2 拍，
//  并且读数据整体错位到下一笔事务）。地址/数据一直挂着，由外设用 sel 限定。
//★ 写数据必须取【当拍的 HWDATA】：AHB-Lite 规定 HWDATA 只在数据相有效，
//  地址相那一沿它还没有值。原来在地址相就锁进 lat_wdata，锁到的是上一笔的旧数据
//  （实测所有写都落了个 0 或者上一笔的值）。
    always @(*) begin
        bus_sel_out  = (state == B_ACC) ? (32'd1 << lat_slot) : 32'd0;
        bus_we_out   = (state == B_ACC) ? lat_we : 1'b0;
        bus_addr_out = lat_addr;
        bus_data_out = hwdata;
        bus_be_out   = lat_be;
    end

    always @(posedge clk) begin
        if (rst) begin
            state       <= B_IDLE;
            hrdata      <= 32'd0;
            hreadyout   <= 1'b1;
            hresp       <= 1'b0;
            lat_addr    <= 32'd0;
            lat_wdata   <= 32'd0;
            lat_be      <= 4'd0;
            lat_we      <= 1'b0;
            lat_slot    <= SLOT_NONE;
        end
        else begin
            hresp       <= 1'b0;

            case (state)
            B_IDLE: begin
                hreadyout <= 1'b1;
                if (accept) begin
                    lat_addr  <= haddr >> 2;
                    lat_wdata <= hwdata;
                    lat_be    <= hwstrb;
                    lat_we    <= hwrite;
                    lat_slot  <= slot;
//★ 外设的写【没有应答】（它们的 ld_ready 只在读时拉起），所以写的数据相
//  固定一拍就完成，不能等 ld_ready —— 等了就是死等。读才插等待态。
                    hreadyout <= hwrite ? 1'b1 : 1'b0;
                    state     <= (slot == SLOT_NONE) ? B_ERR1 : B_ACC;
                end
            end
            B_ACC: begin
                if (lat_we) begin
                    hreadyout <= 1'b1;      // 写：外设在这一拍动作，数据相同拍收口
                    state     <= B_IDLE;
                end
                else begin
                    hreadyout <= 1'b0;
                    state     <= B_WAIT;
                end
            end
            B_WAIT: begin
                hreadyout <= 1'b0;
                if (sel_ld_ready) begin
                    hrdata    <= lat_we ? 32'd0 : sel_rdata;
                    hreadyout <= 1'b1;
                    state     <= B_IDLE;
                end
            end
            B_ERR1: begin
                hresp     <= 1'b1;
                hreadyout <= 1'b0;
                state     <= B_ERR2;
            end
            B_ERR2: begin
                hresp     <= 1'b1;
                hreadyout <= 1'b1;
                hrdata    <= 32'd0;
                state     <= B_IDLE;
            end
            default: state <= B_IDLE;
            endcase
        end
    end

endmodule
