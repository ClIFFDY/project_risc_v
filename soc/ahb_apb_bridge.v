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
    input      [31:0] hwdata,
    input      [3:0]  hwstrb,
    output reg [31:0] hrdata,
    output reg        hready,
    output reg [1:0]  hresp,
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
    input             i2c_ready
    );

    localparam [5:0]
    SLOT_NONE = 6'd0,
    SLOT_UART = 6'd1,
    SLOT_PLIC = 6'd2,
    SLOT_GPIO = 6'd4,
    SLOT_I2C  = 6'd5;

    reg [5:0] slot;
    always @(*) begin
        if (haddr[31:26] >= 6'd1 && haddr[31:26] <= 6'd5) slot = haddr[31:26];
        else slot = SLOT_NONE;
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
            if (htrans != 2'b00) sel_pend <= slot;
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
        SLOT_GPIO: sel_ld_ready = gpio_ready;
        SLOT_I2C:  sel_ld_ready = i2c_ready;
        default:   sel_ld_ready = 1'b1;
        endcase
    end

    reg [31:0] hrdata_c;
    reg rd_valid_c;
    always @(*) begin
        case (sel_pend)
        SLOT_UART: hrdata_c = uart_data;
        SLOT_PLIC: hrdata_c = plic_data;
        SLOT_GPIO: hrdata_c = gpio_data;
        SLOT_I2C:  hrdata_c = i2c_data;
        default:   hrdata_c = 32'd0;
        endcase
        rd_valid_c = pend_v && !pend_we && (sel_pend != SLOT_NONE) && sel_ld_ready;
        hready = 1'b1;
        hresp  = 2'b00;
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
