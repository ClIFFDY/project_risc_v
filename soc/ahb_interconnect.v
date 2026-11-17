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
    input      [31:0] s0_hrdata,
    input             s0_hreadyout,
    input             s0_hresp,
    input             s0_rd_valid,
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

//---- m4：NPU 自带的搬运引擎（2026-11-17）----
//  ★ 它和 m2（3 号槽那块通用 DMA）是两块完全不同的东西：m4 只搬 NPU 的
//    A/BT/C/权重块，主口出在 npu_system 上。仲裁里排在最后，并且自带防饿死计数。
    input      [31:0] m4_haddr,
    input      [1:0]  m4_htrans,
    input             m4_hwrite,
    input      [2:0]  m4_hsize,
    input      [2:0]  m4_hburst,
    input      [3:0]  m4_hprot,
    input             m4_hmastlock,
    input      [31:0] m4_hwdata,
    input      [3:0]  m4_hwstrb,
    input      [31:0] s2_hrdata,
    input             s2_hreadyout,
    input             s2_hresp,
    input             s2_rd_valid,
    input      [31:0] s3_hrdata,
    input             s3_hreadyout,
    input             s3_hresp,
    input             s3_rd_valid,
    output reg [31:0] m0_hrdata,
    output reg        m0_hready,
    output reg        m0_hresp,
    output reg        m0_rd_valid,
    output reg        m0_hgrant,
    output reg [31:0] m1_hrdata,
    output reg        m1_hready,
    output reg        m1_hresp,
    output reg        m1_rd_valid,
    output reg        m1_hgrant,
    output reg [31:0] m2_hrdata,
    output reg        m2_hready,
    output reg        m2_hresp,
    output reg        m2_rd_valid,
    output reg        m2_hgrant,
    output reg [31:0] m4_hrdata,
    output reg        m4_hready,
    output reg        m4_hresp,
    output reg        m4_rd_valid,
    output reg        m4_hgrant,

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

//---- s3：DDR/AXI 桥（slot 32 = 0x8000_0000，AXI 往返几十拍 ⇒ 同样走直达通路）----
    output reg [31:0] s3_haddr,
    output reg [1:0]  s3_htrans,
    output reg        s3_hwrite,
    output reg [2:0]  s3_hsize,
    output reg [3:0]  s3_hprot,
    output reg [31:0] s3_hwdata,
    output reg [3:0]  s3_hwstrb,
    output reg        s3_hsel
    );

    localparam [2:0] M_LSU = 3'd0, M_L2 = 3'd1, M_DMA = 3'd2, M_BOOT = 3'd3, M_CDMA = 3'd4;

    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//==============================================================================
//  一、授权
//==============================================================================
    reg [2:0] grant;
    reg [6:0] dma_wait;      // m2（通用 DMA）的防饿死计数
    reg [6:0] cdma_wait;     // m4（NPU 搬运引擎）的防饿死计数

    reg m0_req, m1_req, m2_req, m3_req, m4_req;
    always @(*) begin
        m0_req = (m0_htrans != 2'b00);
        m1_req = (m1_htrans != 2'b00);
        m2_req = (m2_htrans != 2'b00);
        m3_req = (m3_htrans != 2'b00);
        m4_req = (m4_htrans != 2'b00);
    end

//当前授权主设备还在事务中（它还有节拍要发）⇒ 不换手
    reg grant_busy;
    always @(*) begin
        grant_busy = (grant == M_LSU)  ? m0_req :
                     (grant == M_L2)   ? m1_req :
                     (grant == M_DMA)  ? m2_req :
                     (grant == M_BOOT) ? m3_req : m4_req;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            grant     <= M_LSU;
            dma_wait  <= 7'd0;
            cdma_wait <= 7'd0;
        end
        else begin
            if (!grant_busy) begin
//★ boot 引擎优先级最高：它只在"其他主设备全在复位里"的那段时间说话，
//  给它最高优先级是为了不被 DMA 的 64 拍插队拖慢上电。
//★ m4（NPU 搬运引擎，2026-11-17）排在最后，但和 m2 一样带防饿死：
//  连续等够 64 拍插一次队。它自己每 8 笔插一拍 IDLE，正常情况下能周期性放开，
//  这个计数只是兜底。
                if (m3_req)
                    grant <= M_BOOT;
                else if (m4_req && (cdma_wait >= 7'd64))
                    grant <= M_CDMA;
                else if (m2_req && (dma_wait >= 7'd64))
                    grant <= M_DMA;
                else if (m0_req)
                    grant <= M_LSU;
                else if (m1_req)
                    grant <= M_L2;
                else if (m2_req)
                    grant <= M_DMA;
                else if (m4_req)
                    grant <= M_CDMA;
            end
            if (m2_req && (grant != M_DMA) && grant_busy) begin
                if (dma_wait != 7'd127)
                    dma_wait <= dma_wait + 7'd1;
            end
            else if (grant == M_DMA)
                dma_wait <= 7'd0;

            if (m4_req && (grant != M_CDMA) && grant_busy) begin
                if (cdma_wait != 7'd127)
                    cdma_wait <= cdma_wait + 7'd1;
            end
            else if (grant == M_CDMA)
                cdma_wait <= 7'd0;
        end
    end

    reg g0, g1, g2, g3, g4;
    always @(*) begin
        g0 = (grant == M_LSU);
        g1 = (grant == M_L2);
        g2 = (grant == M_DMA);
        g3 = (grant == M_BOOT);
        g4 = (grant == M_CDMA);
    end

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
        haddr = 32'd0;
        htrans = 2'b00;
        hwrite = 1'b0;
        hsize = 3'b010;
        hburst = 3'b000;
        hprot = 4'b0011;
        hmastlock = 1'b0;
        hwdata = 32'd0;
        hwstrb = 4'd0;
        if (g0) begin
            haddr = m0_haddr;
            htrans = m0_htrans;
            hwrite = m0_hwrite;
                           hsize = m0_hsize;
                           hburst = m0_hburst;
                           hprot = m0_hprot;
                           hmastlock = m0_hmastlock;
                           hwdata = m0_hwdata;
                           hwstrb = m0_hwstrb;
                           end
        else if (g1) begin
            haddr = m1_haddr;
            htrans = m1_htrans;
            hwrite = m1_hwrite;
                           hsize = m1_hsize;
                           hburst = m1_hburst;
                           hprot = m1_hprot;
                           hmastlock = m1_hmastlock;
                           hwdata = m1_hwdata;
                           hwstrb = m1_hwstrb;
                           end
        else if (g2) begin
            haddr = m2_haddr;
            htrans = m2_htrans;
            hwrite = m2_hwrite;
                           hsize = m2_hsize;
                           hburst = m2_hburst;
                           hprot = m2_hprot;
                           hmastlock = m2_hmastlock;
                           hwdata = m2_hwdata;
                           hwstrb = m2_hwstrb;
                           end
        else if (g3) begin
            haddr = m3_haddr;
            htrans = m3_htrans;
            hwrite = m3_hwrite;
                           hsize = m3_hsize;
                           hburst = m3_hburst;
                           hprot = m3_hprot;
                           hmastlock = m3_hmastlock;
                           hwdata = m3_hwdata;
                           hwstrb = m3_hwstrb;
                           end
        else if (g4) begin
            haddr = m4_haddr;
            htrans = m4_htrans;
            hwrite = m4_hwrite;
                           hsize = m4_hsize;
                           hburst = m4_hburst;
                           hprot = m4_hprot;
                           hmastlock = m4_hmastlock;
                           hwdata = m4_hwdata;
                           hwstrb = m4_hwstrb;
                           end
    end

//==============================================================================
//  三、地址译码
//==============================================================================
    // STCM 扩到 256KB（AW=16）：haddr[31:18]==0
    reg sel_stcm;
    always @(*) begin
        sel_stcm = (haddr[31:18] == 14'd0);
    end
    // NPU 占 slot 8（0x2000_0000）
    reg sel_npu;
    always @(*) begin
        sel_npu = (haddr[31:26] == 6'd8);
    end
    // DDR/AXI 桥占 0x8000_0000-0xBFFF_FFFF（slot 32..47）；精确窗口由桥自己再判
    reg sel_ddr;
    always @(*) begin
        sel_ddr = (haddr[31:30] == 2'b10);
    end

//★ 2026-11-15：owner FIFO 快满时【反压主设备】，不许"照收但丢项"。
//  原来那句 `(owner_count != 3'd7)` 只是不记账，HREADY 照样放行 ⇒ 主设备以为这笔读
//  被接受了，可 FIFO 里没有它的项 ⇒ 应答到了以后被当成队头那项的应答消化掉，
//  真正的主人永远等不到。实测：L2 的 32 拍整行填充和 LSU 的外设读并发时，
//  fill_issue=32 / fill_recv=16 卡死，accept 与 pop 差 16，整核冻死。
//  修法：FIFO 满就把这一笔读【压住】—— 既不发给从设备（htrans 拉回 IDLE，
//  否则从设备照样执行、多吐一个没人认领的应答），也不回 HREADY。
//  ★ 只压"要进 FIFO 的读"：写不记账、NPU/DDR 走直达，都不压。
//  ★ 指针在这里提前声明（原来在下面 owner FIFO 那一节），否则压不住从设备侧。
    reg [4:0] owner_mem [0:7];                // {sel[1:0], owner[2:0]}
    reg  [2:0] owner_wr, owner_rd;
    reg owner_empty;
    reg [2:0] owner_count;
    always @(*) begin
        owner_empty = (owner_wr == owner_rd);
        owner_count = owner_wr - owner_rd;
    end
//★ 不变式：FIFO 里【同时只装一个从设备】的读项。
//  出队时按队头 tag 选从设备、按接受顺序还原 —— 这条只有在"应答顺序 = 接受顺序"
//  时才成立。同一个从设备的应答天然保序；跨从设备就不一定（APB 侧可能被外设的
//  ld_ready / 错误两拍拖后）。所以这里干脆不允许两个从设备的读同时在飞：
//  新读的目标从设备跟队头不一致时，压住它。
    reg [1:0] new_sel;
    reg fifo_same;
    always @(*) begin
        new_sel = sel_stcm ? 2'd1 : 2'd0;
        fifo_same = owner_empty || (owner_mem[owner_rd][4:3] == new_sel);
    end
    reg read_hold;
    always @(*) begin
        read_hold = (htrans != 2'b00) && !hwrite && !sel_npu && !sel_ddr && ((owner_count == 3'd7) || !fifo_same);
    end
    reg [1:0] htrans_gated;
    always @(*) begin
        htrans_gated = read_hold ? 2'b00 : htrans;
    end

//没被选中的从设备看不到地址 —— 否则它的读状态机会被无端触发
    always @(*) begin
        s0_haddr = haddr;
        s0_htrans = htrans_gated;
        s0_hwrite = hwrite;
        s0_hsize = hsize;
        s0_hburst = hburst;
        s0_hprot = hprot;
        s0_hmastlock = hmastlock;
        s0_hwdata = hwdata;
        s0_hwstrb = hwstrb;
        s1_haddr = haddr;
        s1_htrans = htrans_gated;
        s1_hwrite = hwrite;
        s1_hsize = hsize;
        s1_hburst = hburst;
        s1_hprot = hprot;
        s1_hmastlock = hmastlock;
        s1_hwdata = hwdata;
        s1_hwstrb = hwstrb;
        s2_haddr = haddr;
        s2_htrans = htrans_gated;
        s2_hwrite = hwrite;
        s2_hsize = hsize;
        s2_hwdata = hwdata;
        s2_hsel = sel_npu;
        s3_haddr = haddr;
        s3_htrans = htrans_gated;
        s3_hwrite = hwrite;
        s3_hsize = hsize;
        s3_hprot = hprot;
        s3_hwdata = hwdata;
        s3_hwstrb = hwstrb;
        s3_hsel = sel_ddr;
//没被选中的从设备看不到地址 —— 否则它的读状态机会被无端触发
        if (sel_npu) begin
            s0_htrans = 2'b00;
            s0_hwrite = 1'b0;
            s0_hwstrb = 4'd0;
            s1_htrans = 2'b00;
            s1_hwrite = 1'b0;
            s1_hwstrb = 4'd0;
            s3_htrans = 2'b00;
            s3_hwrite = 1'b0;
            s3_hwstrb = 4'd0;
        end
        else if (sel_stcm) begin
            s0_htrans = 2'b00;
            s0_hwrite = 1'b0;
            s0_hwstrb = 4'd0;
            s2_htrans = 2'b00;
            s2_hwrite = 1'b0;
            s3_htrans = 2'b00;
            s3_hwrite = 1'b0;
            s3_hwstrb = 4'd0;
        end
        else if (sel_ddr) begin
            s0_htrans = 2'b00;
            s0_hwrite = 1'b0;
            s0_hwstrb = 4'd0;
            s1_htrans = 2'b00;
            s1_hwrite = 1'b0;
            s1_hwstrb = 4'd0;
            s2_htrans = 2'b00;
            s2_hwrite = 1'b0;
        end
        else begin
            s1_htrans = 2'b00;
            s1_hwrite = 1'b0;
            s1_hwstrb = 4'd0;
            s2_htrans = 2'b00;
            s2_hwrite = 1'b0;
            s3_htrans = 2'b00;
            s3_hwrite = 1'b0;
            s3_hwstrb = 4'd0;
        end
    end

    reg s_hreadyout, s_hresp;
    always @(*) begin
        s_hreadyout = sel_npu ? s2_hreadyout : sel_ddr ? s3_hreadyout : sel_stcm ? s1_hreadyout : s0_hreadyout;
        s_hresp = sel_npu ? s2_hresp     : sel_ddr ? s3_hresp     : sel_stcm ? s1_hresp     : s0_hresp;
    end

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

//★ 必须带 s_hready：地址相位"被接受"的定义是 HREADY 为 1 的那一拍。
//  只按 htrans 计会在从设备拉等待时把同一拍记成两次（现在的 STCM hready 恒 1
//  看不出来，换了带等待的从设备就是致命的）。
//  ★ 2026-11-15：判据用 htrans_gated，不是 htrans —— 被反压压住的那一拍，
//    从设备根本没看到这笔读，就不能记进 FIFO（否则记了一项永远等不到应答）。
    reg accept_read;
    always @(*) begin
        accept_read = (htrans_gated != 2'b00) && !hwrite && s_hreadyout;
    end

//★ 2026-11-13：NPU 的直达应答必须按【接受时的记录】回送，不能拿当拍地址判。
//  原因：LSU 主设备的地址锁存（lsu_ahb_master.v 的 lat_addr）在请求被接受后
//  不会清零，所以 CPU 只要碰过一次 NPU，haddr 就一直指着 NPU、sel_npu 恒 1。
//  这时别的从设备（尤其 L2 的 32 拍行填充）的读应答会被 resp_sel 选成 NPU
//  然后丢掉：L2 的 fill_recv 停在 31/32，owner FIFO 里那一项永远不弹，
//  icache 等不到数据，整核冻死。实测现象就是"写过一次 NPU 之后核卡死"。
//  NPU 的 rd_valid 至少比地址被接受晚一拍（npu/bus/ahb_lite_slave.v 在 S_DATA
//  才回），所以一深寄存器够用，不存在"接受与应答同拍"的冲突。
//  ★ DDR 桥同理，但判据要换成 s3_rd_valid，不能是 sel_ddr（2026-11-16 实测踩到）：
//    sel_ddr 是拿【当拍地址】算的，而 LSU 主设备的 lat_addr 在请求被接受后会一直
//    挂着旧值。CPU 的上一笔 APB 读（例如轮询 DMA 的 STAT）应答要等 2 拍，这 2 拍里
//    CPU 已经发出下一笔 DDR 读、lat_addr 指向 0x8000_xxxx ⇒ 应答回来那一拍 sel_ddr=1，
//    于是这一笔 APB 读的应答被当成 DDR 的、resp_rd_valid 取成 s3_rd_valid=0 ⇒
//    数据丢了、owner FIFO 里那一项还留着。之后队头一直是 APB 的 tag，STCM 的读
//    全被 fifo_same 挡住 ⇒ 核永久冻死；APB 读也整体错位一格（拿到上一笔的数）。
//    实测轨迹：DMA 读 DDR 的同时 CPU 轮询 STAT，fifo wr=1 rd=0 卡到超时。
//    ★ 用 s3_rd_valid 是安全的：DDR 的应答只有在桥上真的回来那一拍才算，其余时刻
//      一律按 owner FIFO 走 ⇒ APB/STCM 的应答不会在 DDR 传输期间被吞掉。
//    ★ 反过来也不会撞车：owner FIFO 的应答是"接受后第 2 拍"，而 DDR 应答最早也得
//      在它被接受后第 5 拍；DDR 一生效就压着 HREADYOUT，别的读根本进不来。
    reg        npu_read_pending;
    reg [2:0]  npu_read_owner;

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
            //★ 2026-11-16：判据是 s3_rd_valid，不是 sel_ddr。DDR 的读从不进 owner FIFO，
            //  它自己的应答由 resp_sel 的 s3_rd_valid 分支送出去；其余任何一拍回来的
            //  应答都属于队头那一项，必须弹掉。见上面那段"为什么不能用 sel_ddr"。
            if (resp_rd_valid && !npu_read_pending && !s3_rd_valid && !owner_empty)
                owner_rd <= owner_rd + 3'd1;
        end
    end

    reg [4:0] resp_tag;
    reg [1:0] resp_sel;
    always @(*) begin
        resp_tag = owner_mem[owner_rd];
        resp_sel = npu_read_pending ? 2'd2 : s3_rd_valid ? 2'd3 : resp_tag[4:3];
    end
    reg [2:0] resp_owner;
    always @(*) begin
        resp_owner = npu_read_pending ? npu_read_owner : (s3_rd_valid ? grant : resp_tag[2:0]);
    end
    reg [31:0] resp_hrdata;
    always @(*) begin
        resp_hrdata = (resp_sel == 2'd3) ? s3_hrdata : (resp_sel == 2'd2) ? s2_hrdata : (resp_sel == 2'd1) ? s1_hrdata : s0_hrdata;
    end
    reg resp_rd_valid;
    always @(*) begin
        resp_rd_valid = (resp_sel == 2'd3) ? s3_rd_valid : (resp_sel == 2'd2) ? s2_rd_valid : (resp_sel == 2'd1) ? s1_rd_valid : s0_rd_valid;
    end

//==============================================================================
//  五、回送
//==============================================================================
    always @(*) begin
        m0_hrdata = 32'd0;
        m0_hready = 1'b0;
        m0_rd_valid = 1'b0;
        m0_hgrant = g0;
        m0_hresp = 1'b0;
        m1_hrdata = 32'd0;
        m1_hready = 1'b0;
        m1_rd_valid = 1'b0;
        m1_hgrant = g1;
        m1_hresp = 1'b0;
        m2_hrdata = 32'd0;
        m2_hready = 1'b0;
        m2_rd_valid = 1'b0;
        m2_hgrant = g2;
        m2_hresp = 1'b0;
        m3_hrdata = 32'd0;
        m3_hready = 1'b0;
        m3_rd_valid = 1'b0;
        m3_hgrant = g3;
        m3_hresp = 1'b0;
        m4_hrdata = 32'd0;
        m4_hready = 1'b0;
        m4_rd_valid = 1'b0;
        m4_hgrant = g4;
        m4_hresp = 1'b0;
//HREADY = 从设备 HREADYOUT & 被授权（标准做法）
//★ 2026-11-15：owner FIFO 快满时【反压主设备】，不许"照收但丢项"。
//  原来那句 `(owner_count != 3'd7)` 只是不记账，HREADY 照样放行 ⇒ 主设备以为这笔读
//  被接受了，可 FIFO 里没有它的项 ⇒ 应答到了以后被当成队头那项的应答消化掉，
//  真正的主人永远等不到。实测：L2 的 32 拍整行填充和 LSU 的外设读并发时，
//  fill_issue=32 / fill_recv=16 卡死，acc 与 pop 差 16，整核冻死。
//  修法：FIFO 满时把这一笔读压住（htrans_gated 已经在从设备侧拉成 IDLE，
//  这里再压一拍 HREADY，让主设备知道"这笔没被接受"）。
//  两处必须成对：只压 HREADY 的话从设备照样执行、多吐一个没人认领的应答。
        if (g0)
            m0_hready = s_hreadyout && !read_hold;
        if (g1)
            m1_hready = s_hreadyout && !read_hold;
        if (g2)
            m2_hready = s_hreadyout && !read_hold;
        if (g3)
            m3_hready = s_hreadyout && !read_hold;
        if (g4)
            m4_hready = s_hreadyout && !read_hold;
//HRESP 送给【当前被授权】的主设备，不走 FIFO：错误是在被按住的那笔传输里
//  举两拍，那两拍授权不会换手（grant_busy 看的就是它的 htrans），所以按 grant 选
//  是准的。数据/应答才必须按 FIFO 里记的从设备号选（见上面那段注释）。
        if (g0)
            m0_hresp = s_hresp;
        if (g1)
            m1_hresp = s_hresp;
        if (g2)
            m2_hresp = s_hresp;
        if (g3)
            m3_hresp = s_hresp;
        if (g4)
            m4_hresp = s_hresp;
        case (resp_owner)
        M_LSU: begin
            m0_hrdata = resp_hrdata;
            m0_rd_valid = resp_rd_valid;
        end
        M_L2: begin
            m1_hrdata = resp_hrdata;
            m1_rd_valid = resp_rd_valid;
        end
        M_DMA: begin
            m2_hrdata = resp_hrdata;
            m2_rd_valid = resp_rd_valid;
        end
        M_BOOT: begin
            m3_hrdata = resp_hrdata;
            m3_rd_valid = resp_rd_valid;
        end
        default: begin
            m4_hrdata = resp_hrdata;
            m4_rd_valid = resp_rd_valid;
        end
        endcase
    end

endmodule
