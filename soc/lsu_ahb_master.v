`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: lsu_ahb_master
// Description:
//   CPU 侧 stage1 总线口 → AHB-Lite 主机（单笔、非流水、SINGLE burst）。
//
//   本模块是【两种协议之间的适配器】，两边语义不同，不能直接连根线了事：
//     CPU 侧：bus_valid_in 一拍脉冲 + bus_addr_in（字节地址>>2）等；
//             读完成由 bus_ready_out 出一拍脉冲（和原来的 rd_valid 同义）。
//     AHB 侧：地址相（HTRANS=NONSEQ 一拍）→ 数据相（等 HREADY）；
//             读数据在 HREADY=1 的那一拍有效。
//
//   ★ 2026-11-02 改：原来是组合直通 + 拿桥的 rd_valid 当应答，
//     等效于"HREADY 恒 1、数据走旁路"，不是 AHB-Lite。现在改成三态 FSM：
//     等待期 HADDR/HTRANS/HWDATA 全部保持不动（AHB-Lite 明文要求），
//     完成判据只看 HREADY。
//
//   写不需要完成脉冲（与改造前一致：CPU 侧只对读要 ready）。
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
    output reg [31:0] hwdata,
    output reg [3:0]  hwstrb,
    input      [31:0] hrdata,
    input             hready,
    input             hresp
    );

    localparam [1:0]
        HTRANS_IDLE   = 2'b00,
        HTRANS_NONSEQ = 2'b10;

    localparam [2:0]
        M_IDLE  = 3'd0,  // 等 CPU 的 valid
        M_DRIVE = 3'd3,  // 请求已锁存，等 HREADY 摆地址相
        M_ADDR  = 3'd1,  // 地址相：HTRANS=NONSEQ
        M_DATA  = 3'd2;  // 数据相：等 HREADY

    reg [2:0] state;
    reg       req_write;

//★ CPU 侧的 valid 是【一拍脉冲】，必须锁存。原来在 M_IDLE 里要求 valid 和
//  hready 同拍，前面有在途传输时这一笔请求会被静默丢掉（= 读永远等不到应答）。
    reg [31:0] lat_addr;
    reg [3:0]  lat_be;
    reg        lat_we;
//HSIZE 由字节使能反推：4 个字节全写 = word，连续两个 = half，单个 = byte。
//  地址本身可能不对齐，所以按"最低被使能的字节"定 size 的上界即可。
    reg [2:0] size_from_be;
    always @(*) begin
        case (lat_be)
        4'b0001, 4'b0010, 4'b0100, 4'b1000: size_from_be = 3'b000;  // byte
        4'b0011, 4'b0110, 4'b1100:          size_from_be = 3'b001;  // half word
        default:                            size_from_be = 3'b010;  // word
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state         <= M_IDLE;
            haddr         <= 32'd0;
            htrans        <= HTRANS_IDLE;
            hwrite        <= 1'b0;
            hsize         <= 3'b010;
            hburst        <= 3'b000;
            hwdata        <= 32'd0;
            hwstrb        <= 4'd0;
            req_write     <= 1'b0;
            bus_data_out  <= 32'd0;
            bus_ready_out <= 1'b0;
            bus_hold_out  <= 1'b0;
        end
        else begin
            bus_ready_out <= 1'b0;
            case (state)
            M_IDLE: begin
                if (bus_valid_in) begin
                    lat_addr  <= bus_addr_in;

                    lat_be    <= bus_be_in;
                    lat_we    <= bus_we_in;
                    state     <= M_DRIVE;
                end
            end
            M_DRIVE: begin
                if (hready) begin
//地址相：这里就把 haddr/htrans 摆好，下一拍生效
                    haddr     <= lat_addr << 2;
                    htrans    <= HTRANS_NONSEQ;
                    hwrite    <= lat_we;
                    hsize     <= size_from_be;
                    hburst    <= 3'b000;
                    hwstrb    <= lat_be;
                    req_write <= lat_we;
                    state     <= M_ADDR;
                end
            end
            M_ADDR: begin
//地址相已经摆了一拍；数据相开始，地址相结束（HTRANS 回 IDLE）
                htrans <= HTRANS_IDLE;
                hwdata <= bus_data_in;      // 写数据在数据相才驱动
                state  <= M_DATA;
            end
            M_DATA: begin
//等 HREADY。等待期 haddr/hwdata/hwrite/hwstrb 一律不动。
                if (hready) begin
                    if (!req_write && !hresp) begin
                        bus_data_out  <= hrdata;
                        bus_ready_out <= 1'b1;
                    end
                    else if (!req_write && hresp) begin
//错误响应：数据无效，但也要把这一笔收掉，否则 CPU 侧会一直等
                        bus_data_out  <= 32'd0;
                        bus_ready_out <= 1'b1;
                    end
                    state <= M_IDLE;
                end
            end
            default: state <= M_IDLE;
            endcase
        end
    end


endmodule
