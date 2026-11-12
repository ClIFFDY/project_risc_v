`timescale 1ns / 1ps
// ahb_lite_slave — ARM AHB-Lite 从机接口（NPU 唯一的 CPU 总线入口）
//
// 1) 模块位置和职责
//    位于 npu_system 顶层与 npu_top 之间，是 NPU 对外的唯一总线接口。它只做
//    协议翻译：把 AHB-Lite 的一次传输翻译成 NPU 内部的原生请求
//    (req_addr/req_wdata/req_byte_en/req_we/req_valid)，不包含任何 NPU 语义。
//    下游的地址译码、control_regs、start_ctrl、buffer_access_ctrl 都不需要
//    知道 AHB 的存在。原 mmio_if 已被本模块取代。
//
// 2) 数据布局、位宽和单位
//    HADDR   ：字节地址，本从机只接受 32 位字访问，因此 HADDR[1:0] 必须为 0。
//    HWDATA  ：32 位写数据，只在数据阶段有效（地址阶段不必驱动）。
//    HRDATA  ：32 位读数据，只要求在 HREADYOUT=1 那一拍有效，其余时间不关心。
//    req_addr：相对 ADDR_BASE 的字节地址（= HADDR - ADDR_BASE），字索引为
//              req_addr[NPU_*BUF_AW+1:2]；req_wdata/req_byte_en 与它同拍有效。
//    core_busy：来自 tile_controller，为 1 表示 NPU 正在使用 A/BT/C Buffer。
//
// 3) 时序延迟
//    一次传输分"地址阶段"和"数据阶段"，数据阶段紧跟在地址阶段之后：
//      地址阶段：HSEL=1 且 HTRANS=NONSEQ/SEQ，本模块在结束沿锁存地址；
//      数据阶段：S_DATA 状态，req_valid 单拍有效，并把内部请求发给下游。
//    寄存器访问和 Buffer 写是 0 等待，数据阶段当拍完成（HREADYOUT=1）；
//    Buffer 读因为 npu_ram 是同步读（请求后一拍返回），要多等 1 拍。
//    完成拍同时也是下一笔传输的地址阶段，所以背靠背访问可以每拍接受一笔。
//
// 4) 复位后的状态和状态机主要转移
//    内部使用项目统一的 clk/rst（高有效同步复位，见编码规范 4.1）；AHB 的
//    HCLK/HRESETn 由 npu_system 转换（rst = !HRESETn）。复位后 state=S_IDLE。
//      S_IDLE  等有效地址阶段，HREADYOUT=1；
//      S_DATA  发内部请求：0 等待访问当拍完成、Buffer 读转 S_RDD、
//              错误条件转 S_ERR1；
//      S_RDD   Buffer 读数据返回拍；
//      S_ERR1  错误响应第 1 拍（HRESP=ERROR 且 HREADYOUT=0）；
//      S_ERR2  错误响应第 2 拍（HRESP=ERROR 且 HREADYOUT=1）。
//    每个状态的入口、保持、退出条件见文件下方的 localparam 说明。
//
// 5) 边界条件、错误处理和不能修改的接口假设
//    · 错误响应（HRESP=ERROR，必须两拍，一拍 master 采不到）：
//      a) 未映射地址：译码后五个 sel 全为 0。旧设计"读回 0、写忽略"是私有
//         总线的将就做法，在 AHB 上等于协议违规，软件写错地址永远发现不了；
//      b) 非 32 位字访问：HSIZE != 3'b010 或 HADDR[1:0] != 0；
//      c) core_busy=1 期间访问 A/BT/C Buffer：旧 buffer_access_ctrl 会把请求
//         丢掉却仍然应答成功，软件会以为数据写进去了；该屏蔽保留作为最后一道
//         防线，本模块负责把错误如实报给 CPU。
//      错误路径比正常路径多 1 拍等待（译码结果在数据阶段才有效），不影响正常通路。
//    · 接口假设：req_* / sel_* / *_rdata 的定义与旧 mmio_if 输出完全一致，
//      npu_top 的译码和寄存器逻辑依赖这个定义，不要改动它们的位宽和语义。
//      内部译码仍由 npu_top 完成，本模块不重复定义地址映射，避免两处常量漂移。
//
// 端口命名说明：AHB-Lite 是 ARM 公开协议，HSEL/HADDR/HTRANS/HWDATA/HRDATA/
// HREADYOUT/HRESP 等端口沿用协议规范名（编码规范 2.2 的 _in/_out 后缀建议在此
// 让位于与协议文档的逐字对照性）；只有 HREADY_in 需要与本模块输出的 HREADYOUT
// 区分，故按约定加 _in 后缀。
module ahb_lite_slave #(
    parameter [31:0] ADDR_BASE = 32'h4000_0000
)(
    input              clk,
    input              rst,
    // ---- AHB-Lite 从机接口 ----
    input              HSEL,
    input       [31:0] HADDR,
    input       [1:0]  HTRANS,
    input              HWRITE,
    input       [2:0]  HSIZE,
    input       [31:0] HWDATA,
    input              HREADY_in,
    output reg  [31:0] HRDATA,
    output reg         HREADYOUT,
    output reg  [1:0]  HRESP,
    // ★ 本仓互联约定的完成脉冲：读数据有效的这一拍为 1（与 stcm/ahb_apb_bridge 同口径）
    output             rd_valid,      // 连续赋值（本仓互联约定的读完成脉冲）
    // ---- NPU 运行状态 ----
    input              core_busy,
    input              core_bank,      // 核正在读的那一组 A/BT
    // ---- 下游原生请求(定义与旧 mmio_if 输出一致) ----
    output reg  [31:0] req_addr,
    output reg  [31:0] req_wdata,
    output reg  [3:0]  req_byte_en,
    output reg         req_we,
    output reg         req_valid,
    // ---- 来自 npu_top 的译码与读数据 ----
    input              sel_ctrl,
    input              sel_status,
    input              sel_a_buf,
    input              sel_bt_buf,
    input              sel_c_buf,
    input       [31:0] control_rdata,
    input       [31:0] status_rdata,
    input       [31:0] buffer_rdata
);

    // HTRANS 与 HRESP 的协议编码(ARM AHB-Lite)。HTRANS_IDLE/HTRANS_BUSY 都不算
    // 传输，判定时只需识别 NONSEQ/SEQ，这里列全是为了对照协议编码表。
    localparam [1:0] HTRANS_IDLE   = 2'b00;
    localparam [1:0] HTRANS_BUSY   = 2'b01;
    localparam [1:0] HTRANS_NONSEQ = 2'b10;
    localparam [1:0] HTRANS_SEQ    = 2'b11;
    localparam [1:0] HRESP_OKAY    = 2'b00;
    localparam [1:0] HRESP_ERROR   = 2'b01;

    // 状态编码。每个状态的入口/保持/退出条件：
    //   S_IDLE 入口：复位或上一笔传输结束。保持：没有有效地址阶段。
    //          退出：cap_now 成立时锁存地址并转 S_DATA。
    //   S_DATA 入口：刚接受一笔传输。保持：错误条件或 Buffer 读等待期间由
    //          HREADYOUT=0 延长数据阶段。退出：0 等待访问当拍完成（直接接
    //          下一笔或回 S_IDLE）；Buffer 读转 S_RDD；错误转 S_ERR1。
    //   S_RDD  入口：Buffer 读发出了请求。保持：1 拍等同步 RAM 数据。
    //          退出：无条件，按 cap_now 接下一笔或回 S_IDLE。
    //   S_ERR1 入口：数据阶段判定为错误。保持：1 拍(HRESP=ERROR,HREADYOUT=0)。
    //          退出：无条件转 S_ERR2。
    //   S_ERR2 入口：错误响应第 2 拍。保持：1 拍(HRESP=ERROR,HREADYOUT=1)。
    //          退出：按 cap_now 接下一笔或回 S_IDLE。
    localparam [2:0]
        S_IDLE = 3'd0,
        S_DATA = 3'd1,
        S_RDD  = 3'd2,
        S_ERR1 = 3'd3,
        S_ERR2 = 3'd4;

    reg [2:0]  state;
    reg [31:0] addr_q;      // 本笔传输的内部字节地址(已减基址)
    reg        write_q;     // 本笔传输的方向
    reg        size_ok_q;   // 地址阶段采到的 HSIZE / 对齐检查结果

    // 组合中间结果。全部在 always @(*) 中赋值，无隐含锁存器。
    reg [31:0] addr_off;    // HADDR 减基址后的内部字节地址
    reg        bus_phase;   // 本拍总线上是有效地址阶段
    reg        cap_now;     // 本拍接受下一笔地址
    reg        is_buf;      // 本笔访问 A/BT/C Buffer
    reg        mapped;      // 本笔访问落在已映射窗口内
    reg        err_now;     // 本笔访问需要回 ERROR
    reg        idle_bank_wr; // 本笔是往另一组 A/BT 预装（忙时也放行）

    always @(*) begin
        addr_off  = HADDR - ADDR_BASE;
        bus_phase = HSEL && ((HTRANS == HTRANS_NONSEQ) || (HTRANS == HTRANS_SEQ))
                    && HREADY_in;
        // 接受下一笔地址的条件：本拍自己的传输已经完成(HREADYOUT=1)且总线上是
        // 有效地址阶段。S_IDLE 下没有在途传输，同样成立。漏掉"完成拍就地锁存"
        // 会让背靠背访问只执行第 1、3、5… 笔，这是自己写 AHB 从机最常见的错。
        cap_now   = bus_phase && HREADYOUT;

        // ★ 双缓冲：忙的时候【仍然允许】往核没在读的那一组 A/BT 写（预装下一块）。
        //   核正在读的那一组、以及 C，忙时一律 ERROR —— 这是数据一致性的最后一道防线。
        is_buf  = sel_a_buf || sel_bt_buf || sel_c_buf;
        mapped  = sel_ctrl || sel_status || is_buf;
        idle_bank_wr = write_q && (sel_a_buf || sel_bt_buf) && (addr_q[11] != core_bank);
        err_now = !mapped || !size_ok_q || (core_busy && is_buf && !idle_bank_wr);

        // ---- 下游原生请求 ----
        // Only legal DATA transactions may reach the core. An ERROR response
        // must suppress register, RAM, and CTRL.START side effects.
        req_addr    = addr_q;
        req_wdata   = HWDATA;
        req_we      = write_q;
        req_byte_en = write_q ? 4'hF : 4'h0;
        // An ERROR response must never issue a register write, RAM access or START.
        req_valid   = (state == S_DATA) && !err_now;

        // ---- AHB 应答 ----
        HRDATA    = sel_ctrl   ? control_rdata :
                    sel_status ? status_rdata  : buffer_rdata;
        // HREADYOUT=0 只在两种场合：Buffer 读等同步 RAM 数据、错误响应的第 1 拍。
        // 空闲(S_IDLE)必须是 1，否则总线上永远不出现有效地址阶段。
        HREADYOUT = ((state == S_DATA) && (err_now || (is_buf && !write_q))) ? 1'b0 :
                    (state == S_ERR1) ? 1'b0 : 1'b1;
        HRESP     = ((state == S_ERR1) || (state == S_ERR2)) ? HRESP_ERROR : HRESP_OKAY;
    end

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            addr_q    <= 32'd0;
            write_q   <= 1'b0;
            size_ok_q <= 1'b0;
        end
        else begin
            // 完成拍就地锁存下一笔地址(见 cap_now 的说明)
            if (cap_now) begin
                addr_q    <= addr_off;
                write_q   <= HWRITE;
                size_ok_q <= (HSIZE == 3'b010) && (HADDR[1:0] == 2'd0);
            end
            case (state)
                S_IDLE: begin
                    state <= cap_now ? S_DATA : S_IDLE;
                end
                S_DATA: begin
                    if (err_now) begin
                        state <= S_ERR1;
                    end
                    else if (is_buf && !write_q) begin
                        state <= S_RDD;      // 同步 RAM 读，数据下一拍回来
                    end
                    else begin
                        state <= cap_now ? S_DATA : S_IDLE;
                    end
                end
                S_RDD: begin
                    state <= cap_now ? S_DATA : S_IDLE;
                end
                S_ERR1: begin
                    state <= S_ERR2;         // 错误响应第 1 拍结束
                end
                S_ERR2: begin
                    state <= cap_now ? S_DATA : S_IDLE;
                end
                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end
    // 读完成的定义：处于数据阶段（含 Buffer 读等待后的 S_RDD）且 HREADYOUT 已放开、
    // 方向是读、且不是错误响应。放在模块末尾是因为 iverilog 不允许先声明后用。
    assign rd_valid = ((state == S_DATA) || (state == S_RDD)) && HREADYOUT
                      && !write_q && !err_now;

endmodule
