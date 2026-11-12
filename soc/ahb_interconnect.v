`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ahb_interconnect
// Description: 四主四从 AHB 互联（单时钟）。按 AHB 的握手口径实现：
//              s0=APB 桥 / s1=STCM / s2=NPU（变延迟，直达）/ s3=DDR-AXI 桥（变延迟，直达）
//
//   ── 握手约定（这次是标准做法，别再退回"脉冲 valid"）────────────────────────
//   1. 主设备把要发的节拍放在 HADDR/HTRANS 上，并【保持】；
//   2. 仲裁器给出 HGRANT（同一时刻只有一位为 1）；
//   3. 送给主设备的 HREADY = 从设备 HREADY & HGRANT —— 只有被授权的主设备
//      才看到 HREADY=1，那一拍它的地址相位才算被接受；
//   4. 主设备看到 HREADY=1 才推进到下一拍（突发）或撤掉请求（单拍）；
//      没被授权时它必须把 HTRANS 拉成 IDLE（否则从设备会把它当成新请求）。
//   5. 读应答比地址晚两拍，且可能在换授权之后才回 —— 用一条 FIFO 记
//      "这一条应答是谁的、发给哪个从设备"，不靠当前 grant、也不靠当前地址猜。
//      ★ 从设备号必须一起记：应答回来时地址早就换人了，按当拍地址选数据会把
//        STCM 的读数据送到正在读 PLIC 的主设备上（踩过）。
//
//   仲裁：固定优先级 m0(LSU) > m1(L2) > m2(DMA)，只在当前主设备的事务
//        结束（HTRANS 回 IDLE）时换手；DMA 连续等 64 拍插队一次防饿死。
//////////////////////////////////////////////////////////////////////////////////


module ahb_interconnect(
    input clk, rst,

//---- m0：LSU 非缓存口 ----
    input      [31:0] m0_haddr,
    input      [1:0]  m0_htrans,
    input             m0_hwrite,
    input      [2:0]  m0_hsize,
    input      [2:0]  m0_hburst,
    input      [3:0]  m0_hprot,
    input             m0_hmastlock,
    input      [31:0] m0_hwdata,
    input      [3:0]  m0_hwstrb,
    output reg [31:0] m0_hrdata,
    output reg        m0_hready,
    output reg        m0_hresp,
    output reg        m0_rd_valid,
    output reg        m0_hgrant,

//---- m1：L2 ----
    input      [31:0] m1_haddr,
    input      [1:0]  m1_htrans,
    input             m1_hwrite,
    input      [2:0]  m1_hsize,
    input      [2:0]  m1_hburst,
    input      [3:0]  m1_hprot,
    input             m1_hmastlock,
    input      [31:0] m1_hwdata,
    input      [3:0]  m1_hwstrb,
    output reg [31:0] m1_hrdata,
    output reg        m1_hready,
    output reg        m1_hresp,
    output reg        m1_rd_valid,
    output reg        m1_hgrant,

//---- m2：DMA ----
    input      [31:0] m2_haddr,
    input      [1:0]  m2_htrans,
    input             m2_hwrite,
    input      [2:0]  m2_hsize,
    input      [2:0]  m2_hburst,
    input      [3:0]  m2_hprot,
    input             m2_hmastlock,
    input      [31:0] m2_hwdata,
    input      [3:0]  m2_hwstrb,
    output reg [31:0] m2_hrdata,
    output reg        m2_hready,
    output reg        m2_hresp,
    output reg        m2_rd_valid,
    output reg        m2_hgrant,

//---- s0：APB 外设桥 ----
    output reg [31:0] s0_haddr,
    output reg [1:0]  s0_htrans,
    output reg        s0_hwrite,
    output reg [2:0]  s0_hsize,
    output reg [2:0]  s0_hburst,
    output reg [3:0]  s0_hprot,
    output reg        s0_hmastlock,
    output reg [31:0] s0_hwdata,
    output reg [3:0]  s0_hwstrb,
    input      [31:0] s0_hrdata,
    input             s0_hreadyout,
    input             s0_hresp,
    input             s0_rd_valid,

//---- s1：STCM ----
    output reg [31:0] s1_haddr,
    output reg [1:0]  s1_htrans,
    output reg        s1_hwrite,
    output reg [2:0]  s1_hsize,
    output reg [2:0]  s1_hburst,
    output reg [3:0]  s1_hprot,
    output reg        s1_hmastlock,
    output reg [31:0] s1_hwdata,
    output reg [3:0]  s1_hwstrb,
    input      [31:0] s1_hrdata,
    input             s1_hreadyout,
    input             s1_hresp,
    input             s1_rd_valid,

//---- m3：boot 搬运引擎（只在上电搬运时说话）----
    input      [31:0] m3_haddr,
    input      [1:0]  m3_htrans,
    input             m3_hwrite,
    input      [2:0]  m3_hsize,
    input      [2:0]  m3_hburst,
    input      [3:0]  m3_hprot,
    input             m3_hmastlock,
    input      [31:0] m3_hwdata,
    input      [3:0]  m3_hwstrb,
    output reg [31:0] m3_hrdata,
    output reg        m3_hready,
    output reg        m3_hresp,
    output reg        m3_rd_valid,
    output reg        m3_hgrant,

//---- s2：NPU（AHB-Lite 从设备，带等待态 ⇒ 应答走直达通路）----
    output reg [31:0] s2_haddr,
    output reg [1:0]  s2_htrans,
    output reg        s2_hwrite,
    output reg [2:0]  s2_hsize,
    output reg [31:0] s2_hwdata,
    output reg        s2_hsel,
    input      [31:0] s2_hrdata,
    input             s2_hreadyout,
    input             s2_hresp,
    input             s2_rd_valid,

//---- s3：DDR/AXI 桥（slot 32 = 0x8000_0000，AXI 往返几十拍 ⇒ 同样走直达通路）----
    output reg [31:0] s3_haddr,
    output reg [1:0]  s3_htrans,
    output reg        s3_hwrite,
    output reg [2:0]  s3_hsize,
    output reg [3:0]  s3_hprot,
    output reg [31:0] s3_hwdata,
    output reg [3:0]  s3_hwstrb,
    output reg        s3_hsel,
    input      [31:0] s3_hrdata,
    input             s3_hreadyout,
    input             s3_hresp,
    input             s3_rd_valid
    );

    localparam [1:0] M_LSU = 2'd0, M_L2 = 2'd1, M_DMA = 2'd2, M_BOOT = 2'd3;

    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//==============================================================================
//  一、授权
//==============================================================================
    reg [1:0] grant;
    reg [6:0] dma_wait;

    wire m0_req = (m0_htrans != 2'b00);
    wire m1_req = (m1_htrans != 2'b00);
    wire m2_req = (m2_htrans != 2'b00);
    wire m3_req = (m3_htrans != 2'b00);

//当前授权主设备还在事务中（它还有节拍要发）⇒ 不换手
    wire grant_busy = (grant == M_LSU)  ? m0_req :
                      (grant == M_L2)   ? m1_req :
                      (grant == M_DMA)  ? m2_req : m3_req;

    always @(posedge clk) begin
        if (reset_registered) begin
            grant    <= M_LSU;
            dma_wait <= 7'd0;
        end
        else begin
            if (!grant_busy) begin
//★ boot 引擎优先级最高：它只在"其他主设备全在复位里"的那段时间说话，
//  给它最高优先级是为了不被 DMA 的 64 拍插队拖慢上电。
                if (m3_req)                             grant <= M_BOOT;
                else if (m2_req && (dma_wait >= 7'd64)) grant <= M_DMA;
                else if (m0_req)                        grant <= M_LSU;
                else if (m1_req)                        grant <= M_L2;
                else if (m2_req)                        grant <= M_DMA;
            end
            if (m2_req && (grant != M_DMA) && grant_busy) begin
                if (dma_wait != 7'd127) dma_wait <= dma_wait + 7'd1;
            end
            else if (grant == M_DMA)
                dma_wait <= 7'd0;
        end
    end

    wire g0 = (grant == M_LSU);
    wire g1 = (grant == M_L2);
    wire g2 = (grant == M_DMA);
    wire g3 = (grant == M_BOOT);

//==============================================================================
//  二、请求信号选择
//==============================================================================
    reg [31:0] haddr;
    reg [1:0]  htrans;
    reg        hwrite;
    reg [2:0]  hsize, hburst;
    reg [3:0]  hprot;
    reg        hmastlock;
    reg [31:0] hwdata;
    reg [3:0]  hwstrb;

    always @(*) begin
        haddr = 32'd0; htrans = 2'b00; hwrite = 1'b0; hsize = 3'b010;
        hburst = 3'b000; hprot = 4'b0011; hmastlock = 1'b0;
        hwdata = 32'd0; hwstrb = 4'd0;
        if (g0)      begin haddr = m0_haddr; htrans = m0_htrans; hwrite = m0_hwrite;
                           hsize = m0_hsize; hburst = m0_hburst; hprot = m0_hprot;
                           hmastlock = m0_hmastlock; hwdata = m0_hwdata; hwstrb = m0_hwstrb; end
        else if (g1) begin haddr = m1_haddr; htrans = m1_htrans; hwrite = m1_hwrite;
                           hsize = m1_hsize; hburst = m1_hburst; hprot = m1_hprot;
                           hmastlock = m1_hmastlock; hwdata = m1_hwdata; hwstrb = m1_hwstrb; end
        else if (g2) begin haddr = m2_haddr; htrans = m2_htrans; hwrite = m2_hwrite;
                           hsize = m2_hsize; hburst = m2_hburst; hprot = m2_hprot;
                           hmastlock = m2_hmastlock; hwdata = m2_hwdata; hwstrb = m2_hwstrb; end
        else if (g3) begin haddr = m3_haddr; htrans = m3_htrans; hwrite = m3_hwrite;
                           hsize = m3_hsize; hburst = m3_hburst; hprot = m3_hprot;
                           hmastlock = m3_hmastlock; hwdata = m3_hwdata; hwstrb = m3_hwstrb; end
    end

//==============================================================================
//  三、地址译码
//==============================================================================
    // STCM 扩到 256KB（AW=16）：haddr[31:18]==0
    wire sel_stcm = (haddr[31:18] == 14'd0);
    // NPU 占 slot 8（0x2000_0000）
    wire sel_npu  = (haddr[31:26] == 6'd8);
    // DDR/AXI 桥占 0x8000_0000-0xBFFF_FFFF（slot 32..47）；精确窗口由桥自己再判
    wire sel_ddr  = (haddr[31:30] == 2'b10);

    always @(*) begin
        s0_haddr = haddr; s0_htrans = htrans; s0_hwrite = hwrite;
        s0_hsize = hsize; s0_hburst = hburst; s0_hprot = hprot; s0_hmastlock = hmastlock;
        s0_hwdata = hwdata; s0_hwstrb = hwstrb;
        s1_haddr = haddr; s1_htrans = htrans; s1_hwrite = hwrite;
        s1_hsize = hsize; s1_hburst = hburst; s1_hprot = hprot; s1_hmastlock = hmastlock;
        s1_hwdata = hwdata; s1_hwstrb = hwstrb;
//没被选中的从设备看不到地址 —— 否则它的读状态机会被无端触发
        s2_haddr = haddr; s2_htrans = htrans; s2_hwrite = hwrite;
        s2_hsize = hsize; s2_hwdata = hwdata; s2_hsel = sel_npu;
        s3_haddr = haddr; s3_htrans = htrans; s3_hwrite = hwrite;
        s3_hsize = hsize; s3_hprot = hprot; s3_hwdata = hwdata; s3_hwstrb = hwstrb;
        s3_hsel = sel_ddr;
//没被选中的从设备看不到地址 —— 否则它的读状态机会被无端触发
        if (sel_npu) begin
            s0_htrans = 2'b00; s0_hwrite = 1'b0; s0_hwstrb = 4'd0;
            s1_htrans = 2'b00; s1_hwrite = 1'b0; s1_hwstrb = 4'd0;
            s3_htrans = 2'b00; s3_hwrite = 1'b0; s3_hwstrb = 4'd0;
        end
        else if (sel_stcm) begin
            s0_htrans = 2'b00; s0_hwrite = 1'b0; s0_hwstrb = 4'd0;
            s2_htrans = 2'b00; s2_hwrite = 1'b0;
            s3_htrans = 2'b00; s3_hwrite = 1'b0; s3_hwstrb = 4'd0;
        end
        else if (sel_ddr) begin
            s0_htrans = 2'b00; s0_hwrite = 1'b0; s0_hwstrb = 4'd0;
            s1_htrans = 2'b00; s1_hwrite = 1'b0; s1_hwstrb = 4'd0;
            s2_htrans = 2'b00; s2_hwrite = 1'b0;
        end
        else begin
            s1_htrans = 2'b00; s1_hwrite = 1'b0; s1_hwstrb = 4'd0;
            s2_htrans = 2'b00; s2_hwrite = 1'b0;
            s3_htrans = 2'b00; s3_hwrite = 1'b0; s3_hwstrb = 4'd0;
        end
    end

    wire        s_hreadyout = sel_npu ? s2_hreadyout : sel_ddr ? s3_hreadyout : sel_stcm ? s1_hreadyout : s0_hreadyout;
    wire        s_hresp     = sel_npu ? s2_hresp     : sel_ddr ? s3_hresp     : sel_stcm ? s1_hresp     : s0_hresp;

//==============================================================================
//  四、owner FIFO：读地址被接受时把"谁发的 + 发给哪个从设备"一起记下来
//
//  ★ 必须记【从设备号】：应答比地址相位晚两拍，而 sel_stcm 是拿【当拍地址】算的。
//    中间仲裁换手、另一个主设备去访问另一个从设备时，sel_stcm 早变了 ——
//    只按当前 sel 选数据，就会把 STCM 的读数据送给正在读 PLIC 的主设备（踩过：
//    并发时 CPU 读 PLIC 拿到 DMA 的搬运数据，DMA 搬完的内容整体错位）。
//
//  ★ 出队按接受顺序还原 ⇒ 隐含假设两个从设备的读延迟相同。现在 STCM 与
//    ahb_apb_bridge 的应答都是"接受后第 2 拍"，成立。以后谁加了等待态，
//    这里要么拆成每从设备一条队列，要么改成带标签的乱序回收。
//==============================================================================
    reg [3:0] owner_mem [0:7];                // {sel[1:0], owner[1:0]}：0=APB 1=STCM（2=NPU、3=DDR 走直达不进队）
    reg [2:0] owner_wr, owner_rd;
    wire      owner_empty = (owner_wr == owner_rd);
    wire [2:0] owner_count = owner_wr - owner_rd;

//★ 必须带 s_hready：地址相位"被接受"的定义是 HREADY 为 1 的那一拍。
//  只按 htrans 计会在从设备拉等待时把同一拍记成两次（现在的 STCM hready 恒 1
//  看不出来，换了带等待的从设备就是致命的）。
    wire accept_read = (htrans != 2'b00) && !hwrite && s_hreadyout;

//★ 2026-11-13：NPU 的直达应答必须按【接受时的记录】回送，不能拿当拍地址判。
//  原因：LSU 主设备的地址锁存（lsu_ahb_master.v 的 lat_addr）在请求被接受后
//  不会清零，所以 CPU 只要碰过一次 NPU，haddr 就一直指着 NPU、sel_npu 恒 1。
//  这时别的从设备（尤其 L2 的 32 拍行填充）的读应答会被 resp_sel 选成 NPU
//  然后丢掉：L2 的 fill_recv 停在 31/32，owner FIFO 里那一项永远不弹，
//  icache 等不到数据，整核冻死。实测现象就是"写过一次 NPU 之后核卡死"。
//  NPU 的 rd_valid 至少比地址被接受晚一拍（npu/bus/ahb_lite_slave.v 在 S_DATA
//  才回），所以一深寄存器够用，不存在"接受与应答同拍"的冲突。
//  ★ DDR 桥不走这条：它在 S_DONE 同拍给 HREADYOUT 和 rd_valid，而且整笔传输
//    期间压着 HREADYOUT ⇒ 授权与地址都不换手，按当拍 sel_ddr 判是对的。
    reg        npu_read_pending;
    reg [1:0]  npu_read_owner;

    always @(posedge clk) begin
        if (reset_registered) begin
            npu_read_pending <= 1'b0;
            npu_read_owner   <= 2'd0;
        end
        else if (accept_read && sel_npu) begin
            npu_read_pending <= 1'b1;
            npu_read_owner   <= grant;
        end
        else if (resp_rd_valid && npu_read_pending)
            npu_read_pending <= 1'b0;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            owner_wr <= 3'd0;
            owner_rd <= 3'd0;
        end
        else begin
            if (accept_read && !sel_npu && !sel_ddr && (owner_count != 3'd7)) begin
                owner_mem[owner_wr] <= {sel_stcm ? 2'd1 : 2'd0, grant};
                owner_wr <= owner_wr + 3'd1;
            end
            if (resp_rd_valid && !npu_read_pending && !owner_empty)
                owner_rd <= owner_rd + 3'd1;
        end
    end

    wire [3:0]  resp_tag   = owner_mem[owner_rd];
    wire [1:0]  resp_sel   = npu_read_pending ? 2'd2 : sel_ddr ? 2'd3 : resp_tag[3:2];
    wire [1:0]  resp_owner = npu_read_pending ? npu_read_owner
                                             : (sel_ddr ? grant : resp_tag[1:0]);
    wire [31:0] resp_hrdata   = (resp_sel == 2'd3) ? s3_hrdata :
                                (resp_sel == 2'd2) ? s2_hrdata :
                                (resp_sel == 2'd1) ? s1_hrdata : s0_hrdata;
    wire        resp_rd_valid = (resp_sel == 2'd3) ? s3_rd_valid :
                                (resp_sel == 2'd2) ? s2_rd_valid :
                                (resp_sel == 2'd1) ? s1_rd_valid : s0_rd_valid;

//==============================================================================
//  五、回送
//==============================================================================
    always @(*) begin
        m0_hrdata = 32'd0; m0_hready = 1'b0; m0_rd_valid = 1'b0; m0_hgrant = g0; m0_hresp = 1'b0;
        m1_hrdata = 32'd0; m1_hready = 1'b0; m1_rd_valid = 1'b0; m1_hgrant = g1; m1_hresp = 1'b0;
        m2_hrdata = 32'd0; m2_hready = 1'b0; m2_rd_valid = 1'b0; m2_hgrant = g2; m2_hresp = 1'b0;
        m3_hrdata = 32'd0; m3_hready = 1'b0; m3_rd_valid = 1'b0; m3_hgrant = g3; m3_hresp = 1'b0;
//HREADY = 从设备 HREADYOUT & 被授权（标准做法）
        if (g0) m0_hready = s_hreadyout;
        if (g1) m1_hready = s_hreadyout;
        if (g2) m2_hready = s_hreadyout;
        if (g3) m3_hready = s_hreadyout;
//HRESP 送给【当前被授权】的主设备，不走 FIFO：错误是在被按住的那笔传输里
//  举两拍，那两拍授权不会换手（grant_busy 看的就是它的 htrans），所以按 grant 选
//  是准的。数据/应答才必须按 FIFO 里记的从设备号选（见上面那段注释）。
        if (g0) m0_hresp = s_hresp;
        if (g1) m1_hresp = s_hresp;
        if (g2) m2_hresp = s_hresp;
        if (g3) m3_hresp = s_hresp;
        case (resp_owner)
        M_LSU: begin m0_hrdata = resp_hrdata; m0_rd_valid = resp_rd_valid; end
        M_L2:  begin m1_hrdata = resp_hrdata; m1_rd_valid = resp_rd_valid; end
        M_DMA: begin m2_hrdata = resp_hrdata; m2_rd_valid = resp_rd_valid; end
        default: begin m3_hrdata = resp_hrdata; m3_rd_valid = resp_rd_valid; end
        endcase
    end

endmodule
