`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ahb_axi_bridge
// Description: 本仓总线（AHB-Lite 口径）→ AXI4 主设备的桥，单笔在飞。
//
//   来源：ahb_axi_bridge/ 交付的四份 RTL（ahb_lite_axi4_bridge / ahb_npu_ddr_decode
//   / project_ahb_adapter / project_npu_ddr_subsystem）合并成这一个模块。合并时：
//     · 丢掉 project_ahb_adapter 的四主合流 —— 本仓 ahb_interconnect 已经在仲裁；
//     · 丢掉 ahb_npu_ddr_decode —— 只剩一个下游，地址窗口判据并进来（bad_addr）；
//     · 留下 AXI4 引擎本体，把外圈换成本仓的从设备握手。
//
//   本仓从设备握手（与 ahb_apb_bridge / stcm / NPU 同一条口径）：
//     · 地址相位：HADDR/HTRANS/HWRITE/HSIZE/HPROT/HWSTRB/HWDATA 同拍给出，主设备保持；
//     · 本模块在【整笔传输期间】把 HREADYOUT 压低 —— 主设备因此不会撤请求，
//       互联的 grant_busy 一直成立、授权与地址都不换手 ⇒ 应答可以按当拍 sel/grant
//       直达回送（和 NPU 同一条通路），不必压 owner FIFO。
//       ★ 不要改成"地址相位先给 HREADY=1、数据相位再拉低"：本仓主设备把 hready=1
//         当"已接受"，那一拍就会撤掉请求，后面 AXI 回来时当拍 sel 早就不是 DDR 了。
//     · 完成拍：HREADYOUT=1；读同时给 rd_valid=1（本仓的完成脉冲），HRDATA 有效。
//       写要等 AXI 回 BRESP 才完结 ⇒ DMA / boot 的 store 真正落盘后才放行。
//     · 错误：标准两拍（第 1 拍 HRESP=1 且 HREADYOUT=0，第 2 拍 HRESP=1 且 HREADYOUT=1）。
//       读也照给 rd_valid=1、HRDATA=0 —— 本仓主设备没有异常出口，不给就永远等。
//
//   AXI4 侧只用最保守的用法（MIG 7 系列 AXI4 从口全部接受）：
//     LEN=0、BURST=INCR、SIZE=满宽、ID=0、AWVALID/WVALID 同拍、单笔在飞、不发锁传输。
//     ★ 满宽 + LEN=0 是刻意的：不碰 MIG 的窄传输/上变换路径，地址不用对齐到拍边界
//       （读整拍回来再按 word_shift 把目标字捞出来，写靠 WSTRB 保住无关字节）。
//
//   地址口径：AHB 字节地址 DDR_BASE..DDR_BASE+DDR_BYTES-1
//            → AXI 字节地址 AXI_BASE_ADDR..（MIG 侧就是 DDR 器件 0 起的字节地址）。
//
//   MEM_READY：HCLK 域电平（MIG 标定完拉高）。为低时收到请求先进 S_WAIT_MEM 等，
//              期间 HREADYOUT 一直压着 ⇒ 主设备不丢请求，代价是整条总线被按住。
//              未接 MIG 时可以恒 1，也可以当"DDR 没就绪就别碰"的闸门。
//
//   DDR_PRESENT（2026-11-12 合并）：下游在不在，由本模块自己表达，不再外面挂一个
//   "空从设备"模块。
//     DDR_PRESENT=1：正常走 AXI（下面这套）。
//     DDR_PRESENT=0：AXI 侧整个闭嘴（VALID 恒 0），任何命中本窗口的访问直接进
//                    错误响应 —— 两拍 HRESP=1，读照给 rd_valid=1、HRDATA=0。
//                    也就是原来 axi4_absent_slave 的语义，但少一次 AXI 往返、
//                    少一个模块；参数为 0 时综合把整个 AXI 引擎裁掉。
//                    选 0 时【不要把 AXI 口接到任何地方】（端口留着是给 MIG 用的）。
//
//   AXI_DATA_WIDTH 只能取 32/64/128/256（MIG 的 ddr_axi_upsizer 参数注释给的范围），
//   AXI_ID_WIDTH 必须等于 MIG 的 C0_C_S_AXI_ID_WIDTH。
//////////////////////////////////////////////////////////////////////////////////


module ahb_axi_bridge #(
    parameter integer AXI_ADDR_WIDTH = 32,
    parameter integer AXI_DATA_WIDTH = 128,
    parameter integer AXI_ID_WIDTH   = 4,
    parameter [31:0]  DDR_BASE       = 32'h8000_0000,
    parameter [32:0]  DDR_BYTES      = 33'h0_4000_0000,   // 1 GB
    parameter [31:0]  AXI_BASE_ADDR  = 32'h0000_0000,
    parameter integer DDR_PRESENT    = 0    // 0 = 下游没有存储器（访问一律 ERROR）
) (
    //---- 本仓总线从设备口 ----
    input               HCLK,
    input               HRESETn,
    input               MEM_READY,
    input               HSEL,
    input      [31:0]   HADDR,
    input      [1:0]    HTRANS,
    input               HWRITE,
    input      [2:0]    HSIZE,
    input      [3:0]    HPROT,
    input      [3:0]    HWSTRB,
    input      [31:0]   HWDATA,
    output     [31:0]   HRDATA,
    output              HREADYOUT,
    output              HRESP,
    output              rd_valid,

    //---- AXI4 主设备口 ----
    output [AXI_ID_WIDTH-1:0]     M_AXI_AWID,
    output [AXI_ADDR_WIDTH-1:0]   M_AXI_AWADDR,
    output [7:0]                  M_AXI_AWLEN,
    output [2:0]                  M_AXI_AWSIZE,
    output [1:0]                  M_AXI_AWBURST,
    output                        M_AXI_AWLOCK,
    output [3:0]                  M_AXI_AWCACHE,
    output [2:0]                  M_AXI_AWPROT,
    output [3:0]                  M_AXI_AWQOS,
    output                        M_AXI_AWVALID,
    input                         M_AXI_AWREADY,
    output [AXI_DATA_WIDTH-1:0]   M_AXI_WDATA,
    output [AXI_DATA_WIDTH/8-1:0] M_AXI_WSTRB,
    output                        M_AXI_WLAST,
    output                        M_AXI_WVALID,
    input                         M_AXI_WREADY,
    input  [AXI_ID_WIDTH-1:0]     M_AXI_BID,
    input  [1:0]                  M_AXI_BRESP,
    input                         M_AXI_BVALID,
    output                        M_AXI_BREADY,
    output [AXI_ID_WIDTH-1:0]     M_AXI_ARID,
    output [AXI_ADDR_WIDTH-1:0]   M_AXI_ARADDR,
    output [7:0]                  M_AXI_ARLEN,
    output [2:0]                  M_AXI_ARSIZE,
    output [1:0]                  M_AXI_ARBURST,
    output                        M_AXI_ARLOCK,
    output [3:0]                  M_AXI_ARCACHE,
    output [2:0]                  M_AXI_ARPROT,
    output [3:0]                  M_AXI_ARQOS,
    output                        M_AXI_ARVALID,
    input                         M_AXI_ARREADY,
    input  [AXI_ID_WIDTH-1:0]     M_AXI_RID,
    input  [AXI_DATA_WIDTH-1:0]   M_AXI_RDATA,
    input  [1:0]                  M_AXI_RRESP,
    input                         M_AXI_RLAST,
    input                         M_AXI_RVALID,
    output                        M_AXI_RREADY
);

    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            clog2 = 0;
            while (v > 0) begin
                clog2 = clog2 + 1;
                v = v >> 1;
            end
        end
    endfunction

    localparam        DDR_ON    = (DDR_PRESENT != 0);
    localparam integer AXI_BYTES = AXI_DATA_WIDTH / 8;
    localparam integer LANE_BITS = clog2(AXI_BYTES);
    localparam [2:0]   BEAT_SIZE = LANE_BITS[2:0];

    localparam [3:0]
        S_IDLE     = 4'd0,   // HREADYOUT 只在"总线上没有有效地址相位"时为 1
        S_WAIT_MEM = 4'd1,   // 等 MEM_READY（MIG 标定）
        S_ARADDR   = 4'd2,
        S_RDATA    = 4'd3,
        S_WRITE    = 4'd4,
        S_BRESP    = 4'd5,
        S_DONE     = 4'd6,   // 完成拍：HREADYOUT=1，读给 rd_valid
        S_TAIL     = 4'd7,   // 主设备撤请求的那一拍（不许就地再接受一笔）
        S_ERR1     = 4'd8,   // 错误响应第 1 拍：HRESP=1, HREADYOUT=0
        S_ERR2     = 4'd9;   // 错误响应第 2 拍：HRESP=1, HREADYOUT=1

    reg [3:0]  state;
    reg [31:0] offset_q;      // 相对 DDR_BASE 的字节偏移（AHB 侧算一次）
    reg [31:0] read_q;        // 读回的目标字
    reg [3:0]  prot_q;
    reg        write_q;
    reg [31:0] wdata_q;       // 本仓主设备在地址相位就给写数据 ⇒ 接受那一拍锁走
    reg [3:0]  strb_q;
    reg        aw_done, w_done;
    reg        read_bad;

    wire       selected_phase = HSEL && (HTRANS != 2'b00);
    wire [32:0] offset        = {1'b0, HADDR} - {1'b0, DDR_BASE};
    wire       bad_addr       = ({1'b0, HADDR} < {1'b0, DDR_BASE}) || (offset >= DDR_BYTES);
    wire       bad_size       = (HSIZE > 3'd2) ||
                                ((HSIZE == 3'd1) && HADDR[0]) ||
                                ((HSIZE == 3'd2) && (HADDR[1:0] != 2'b00));
    wire       accept         = (state == S_IDLE) && selected_phase;

    //每拍一发，地址对齐到拍边界；拍内偏移只取 4 字节对齐的那一档
    wire [31:0] beat_addr  = AXI_BASE_ADDR + ((offset_q >> LANE_BITS) << LANE_BITS);
    wire [31:0] word_shift = ((offset_q & (AXI_BYTES - 1)) & 32'hFFFF_FFFC) * 8;
    //整拍读回来，把目标那个 32 位字挪到低位（截到 32 位是故意的）
    wire [31:0] rdata_word = M_AXI_RDATA[word_shift +: 32];

    //4 个字节使能摆到拍内正确的那一段
    reg  [AXI_BYTES-1:0] wstrb_build;
    always @(*) begin
        wstrb_build = {AXI_BYTES{1'b0}};
        wstrb_build[(word_shift / 8) +: 4] = strb_q;
    end

    assign HREADYOUT = (state == S_DONE) || (state == S_ERR2) || !selected_phase;
    assign HRESP     = (state == S_ERR1) || (state == S_ERR2);
    assign HRDATA    = read_q;
    assign rd_valid  = ((state == S_DONE) || (state == S_ERR2)) && !write_q;

    assign M_AXI_AWID    = {AXI_ID_WIDTH{1'b0}};
    assign M_AXI_ARID    = {AXI_ID_WIDTH{1'b0}};
    assign M_AXI_AWADDR  = beat_addr;
    assign M_AXI_ARADDR  = beat_addr;
    assign M_AXI_AWLEN   = 8'd0;
    assign M_AXI_ARLEN   = 8'd0;
    assign M_AXI_AWSIZE  = BEAT_SIZE;
    assign M_AXI_ARSIZE  = BEAT_SIZE;
    assign M_AXI_AWBURST = 2'b01;
    assign M_AXI_ARBURST = 2'b01;
    assign M_AXI_AWLOCK  = 1'b0;
    assign M_AXI_ARLOCK  = 1'b0;
    assign M_AXI_AWCACHE = 4'b0000;
    assign M_AXI_ARCACHE = 4'b0000;
    assign M_AXI_AWPROT  = {~prot_q[0], 1'b0, prot_q[1]};
    assign M_AXI_ARPROT  = {~prot_q[0], 1'b0, prot_q[1]};
    assign M_AXI_AWQOS   = 4'd0;
    assign M_AXI_ARQOS   = 4'd0;

    //DDR_PRESENT=0 时 AXI 侧必须闭嘴（参数为 0 时这几根综合成常 0）
    assign M_AXI_AWVALID = DDR_ON && (state == S_WRITE) && !aw_done;
    assign M_AXI_WVALID  = DDR_ON && (state == S_WRITE) && !w_done;
    assign M_AXI_BREADY  = DDR_ON && (state == S_BRESP);
    assign M_AXI_ARVALID = DDR_ON && (state == S_ARADDR);
    assign M_AXI_RREADY  = DDR_ON && (state == S_RDATA);
    assign M_AXI_WLAST   = 1'b1;
    //整字复制到每一段，WSTRB 只留目标那一段 ⇒ 不需要宽带移位器
    assign M_AXI_WDATA   = {(AXI_DATA_WIDTH/32){wdata_q}};
    assign M_AXI_WSTRB   = wstrb_build;

    always @(posedge HCLK) begin
        if (!HRESETn) begin
            state     <= S_IDLE;
            offset_q  <= 32'd0;
            read_q    <= 32'd0;
            prot_q    <= 4'd0;
            write_q   <= 1'b0;
            wdata_q   <= 32'd0;
            strb_q    <= 4'd0;
            aw_done   <= 1'b0;
            w_done    <= 1'b0;
            read_bad  <= 1'b0;
        end
        else begin
            case (state)
//------------------------------------------------------------------ 地址相位
            S_IDLE: begin
                if (accept) begin
                    offset_q <= offset[31:0];
                    prot_q   <= HPROT;
                    write_q  <= HWRITE;
                    wdata_q  <= HWDATA;      // 本仓口径：写数据与地址同拍
                    strb_q   <= HWSTRB;
                    read_q   <= 32'd0;
                    read_bad <= 1'b0;
                    aw_done  <= 1'b0;
                    w_done   <= 1'b0;
                    //下游不在 ⇒ 窗口判不判都是错，直接走错误响应
                    if (!DDR_ON || bad_addr || bad_size)
                        state <= S_ERR1;
                    else if (MEM_READY)
                        state <= HWRITE ? S_WRITE : S_ARADDR;
                    else
                        state <= S_WAIT_MEM;
                end
            end
//------------------------------------------------------------------ 等下游
            S_WAIT_MEM: if (MEM_READY) state <= write_q ? S_WRITE : S_ARADDR;
//------------------------------------------------------------------ 写
            S_WRITE: begin
                if (M_AXI_AWREADY) aw_done <= 1'b1;
                if (M_AXI_WREADY)  w_done  <= 1'b1;
                if ((aw_done || M_AXI_AWREADY) && (w_done || M_AXI_WREADY))
                    state <= S_BRESP;
            end
            S_BRESP: if (M_AXI_BVALID) begin
                if ((M_AXI_BRESP != 2'b00) || (M_AXI_BID != {AXI_ID_WIDTH{1'b0}}))
                    state <= S_ERR1;
                else
                    state <= S_DONE;
            end
//------------------------------------------------------------------ 读
            S_ARADDR: if (M_AXI_ARREADY) state <= S_RDATA;
            S_RDATA: if (M_AXI_RVALID) begin
                //多拍回来的异常应答要排空到 RLAST 再报错
                read_bad <= read_bad || (M_AXI_RRESP != 2'b00) ||
                            (M_AXI_RID != {AXI_ID_WIDTH{1'b0}}) || !M_AXI_RLAST;
                if (M_AXI_RLAST) begin
                    if (read_bad || (M_AXI_RRESP != 2'b00) || (M_AXI_RID != {AXI_ID_WIDTH{1'b0}})) begin
                        read_q <= 32'd0;
                        state  <= S_ERR1;
                    end
                    else begin
                        read_q <= rdata_word;
                        state  <= S_DONE;
                    end
                end
            end
//------------------------------------------------------------------ 收尾
            S_DONE: state <= S_TAIL;
            S_TAIL: state <= S_IDLE;
            S_ERR1: state <= S_ERR2;
            S_ERR2: state <= S_TAIL;
            default: state <= S_IDLE;
            endcase
        end
    end

    // elaboration 期检查，不进综合
    // synthesis translate_off
    initial begin
        if ((AXI_DATA_WIDTH != 32) && (AXI_DATA_WIDTH != 64) &&
            (AXI_DATA_WIDTH != 128) && (AXI_DATA_WIDTH != 256))
            $fatal(1, "AXI_DATA_WIDTH must be 32/64/128/256 for MIG 7-series");
        if ((AXI_ADDR_WIDTH < 1) || (AXI_ADDR_WIDTH > 32) || (AXI_ID_WIDTH < 1))
            $fatal(1, "Invalid AXI address or ID width");
        if (((DDR_BYTES & (DDR_BYTES - 1)) != 0) || (DDR_BASE[LANE_BITS-1:0] != 0) ||
            (AXI_BASE_ADDR[LANE_BITS-1:0] != 0))
            $fatal(1, "DDR window / AXI base must be aligned to the AXI beat");
    end
    // synthesis translate_on

endmodule
