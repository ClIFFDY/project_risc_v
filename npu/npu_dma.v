// ============================================================================
//  npu_dma.v —— NPU 自带的搬运引擎（2026-11-17，新写）
//
// 1) 位置和职责
//    长在 NPU 里面，不和 soc/dma.v（3 号槽那块通用 DMA）共用。上游是 CPU：
//    通过 NPU 现有的 AHB 从口写 0x0040..0x005F 这组寄存器下发描述符；下游有两组端口：
//      · 本地口 —— 直接挂 npu_buffer 的 A/BT/C/权重写读口，和 CPU 侧二选一；
//      · 主口   —— AHB-Lite 主设备，出到 ahb_interconnect 的 m4，访存 STCM/DDR。
//    它自己不做任何算法，只搬字；格式变换不在本模块里（见 SPEC 的第三批）。
//
// 2) 传输模型
//    描述符 = {MODE, SRC, DST, WORDS}，一次搬 WORDS 个 32 位字。
//      MODE=0  外部 → 本地：按地址从主口读，回来的数据写进本地缓冲；
//      MODE=1  本地 → 外部：从本地缓冲顺序读，经 8 字 FIFO 往主口写。
//    地址口径：SRC 是**外部字节地址**（绝对地址，STCM/DDR 随便），
//              DST 是**相对 NPU 基址的字节偏移**（和 CPU 访问 Buffer 用同一套偏移）。
//    LOAD 允许 8 拍在飞（地址先走、数据后到），因为本仓互联对同一从设备保序；
//    STORE 用 8 字 FIFO 把本地同步 RAM 的读延迟和总线停顿解耦。
//
// 3) 为什么每 8 笔插一拍 IDLE
//    ahb_interconnect 的换手条件是 grant_busy=0，而 grant_busy 就是"当前主设备
//    的 HTRANS 还非 0"。一个一直拉着 HTRANS 的主设备会把总线独占到底，
//    别的设备（CPU 的 LSU、L2 填充）一拍都拿不到。所以这里每发 8 笔地址就插一拍
//    IDLE，给仲裁器一个换手点；代价约 12% 带宽，换来 CPU 不被饿死。
//
// 4) 读数据的拍位
//    ★ 本仓的约定是"数据在被 HREADY 接受后，由 rd_valid 那一拍带出来"
//      （soc/stcm.v 的 hreadyout 恒 1、数据晚一拍）。所以 LOAD 必须等 m_rd_valid，
//      不能像"标准的"主设备那样在 HREADY 那拍就去采 HRDATA。
//
// 5) 错误与中断
//    描述符在启动瞬间整段校验（对齐、上界、本地窗口、跨 bank、自环）。
//    非法描述符、所选 bank 正在计算、总线错误三种情况置 error + error_code，
//    引擎停在 IDLE，不再发拍；已经搬完的字不回滚，COMPLETED 记的是成功字数。
//    不产生中断，软件轮询 STATUS（和原来的 NPU 一样，PLIC 不新增源）。
`include "npu_defines.vh"

module npu_dma #(
    // NPU 自己的 AHB 从口基址：用来判"源地址打回自己"这个自环
    parameter [31:0] ADDR_BASE = `NPU_ADDR_BASE
)(
    input        clk,
    input        rst,
    // ---- 寄存器口（由 npu_top 译码，npu_system 转接）----
    input        reg_valid,
    input        reg_we,
    input [2:0]  reg_off,       // 0x40..0x5F -> 0..7（字偏移）
    input [31:0] reg_wdata,
    output reg [31:0] reg_rdata,
    // ---- 运行环境 ----
    input        core_busy,     // 核正在算
    input        core_bank,     // 核正在读/写的那一组 bank
    input        core_accumulate, // 核这一次任务带跨任务累加（C 读口会被写回 FIFO 占用）
    // ---- 本地缓冲口（npu_system 里与 CPU 侧二选一）----
    output reg        local_valid,
    output reg        local_we,
    output reg [31:0] local_addr,   // NPU 相对字节偏移
    output reg [31:0] local_wdata,
    input      [31:0] local_rdata,
    // ---- AHB-Lite 主口 ----
    output reg [31:0] m_haddr,
    output reg [1:0]  m_htrans,
    output reg        m_hwrite,
    output reg [2:0]  m_hsize,
    output reg [2:0]  m_hburst,
    output reg [3:0]  m_hprot,
    output reg [31:0] m_hwdata,
    output reg [3:0]  m_hwstrb,
    input      [31:0] m_hrdata,
    input             m_hready,
    input             m_hresp,
    input             m_rd_valid,
    // ---- 状态 ----
    output reg        busy,
    output reg        done,
    output reg        error
);

    localparam [1:0] HTRANS_IDLE   = 2'b00;
    localparam [1:0] HTRANS_NONSEQ = 2'b10;

    // 错误码（写进 ERROR_CODE，同时 STATUS[7:4] 也镜像一份，少读一次）
    localparam [3:0] ERR_NONE       = 4'd0;
    localparam [3:0] ERR_DESCRIPTOR = 4'd1;
    localparam [3:0] ERR_BANK_BUSY  = 4'd2;
    localparam [3:0] ERR_BUS        = 4'd3;

    // 寄存器字偏移
    localparam [2:0] R_CTRL = 3'd0, R_STATUS = 3'd1, R_SRC = 3'd2, R_DST = 3'd3,
                     R_WORDS = 3'd4, R_MODE = 3'd5, R_COMPLETED = 3'd6, R_ERR = 3'd7;

    localparam [1:0] S_IDLE = 2'd0, S_LOAD = 2'd1, S_STORE = 2'd2;

    // LOAD 在飞上限 / STORE 预取 FIFO 深度
    localparam [3:0] INFLIGHT_MAX = 4'd8;
    localparam [3:0] FIFO_DEPTH   = 4'd8;

    reg [1:0]  state;
    reg [31:0] cfg_src, cfg_dst, cfg_words, cfg_mode;   // 软件可写
    reg [31:0] run_src, run_dst, run_words;             // 启动瞬间锁存
    reg        run_load;                                // 1=外部→本地
    reg [31:0] issue_n;                                 // 已发出的总线地址数
    reg [31:0] done_n;                                  // 已完成的字数
    reg [31:0] prefetch_n;                              // STORE：已发出的本地读

    reg [3:0]  error_code;
    reg [2:0]  gap_cnt;
    reg        gap_q;

    // STORE 的预取 FIFO（单时钟、同步读）
    reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [2:0]  fifo_wr, fifo_rd;
    reg [3:0]  fifo_count;
    reg        read_pending;      // 上一拍发了本地读，这一拍数据回来

    //---------------------------------------------------------------- 描述符校验
    // 本地窗口：A bank0/bank1、BT bank0/bank1、C bank0/bank1、权重块。
    // 一次描述符必须整段落在同一个窗口里 —— 跨 bank、越界、回绕一律拒绝。
    function local_window_ok;
        input [31:0] first;
        input [32:0] last;
        begin
            local_window_ok = !last[32] &&
                ((first >= 32'h1000 && last <= 33'h10FF) ||
                 (first >= 32'h1800 && last <= 33'h18FF) ||
                 (first >= 32'h2000 && last <= 33'h20FF) ||
                 (first >= 32'h2800 && last <= 33'h28FF) ||
                 (first >= 32'h3000 && last <= 33'h33FF) ||
                 (first >= 32'h3400 && last <= 33'h37FF) ||
                 (first >= 32'h4000 && last <= 33'h7FFF));
        end
    endfunction

    reg [32:0] dst_end, src_end;
    reg        desc_ok, self_loop;
    always @(*) begin
        dst_end  = {1'b0, cfg_dst} + ({1'b0, cfg_words} << 2) - 1'b1;
        src_end  = {1'b0, cfg_src} + ({1'b0, cfg_words} << 2) - 1'b1;
        // 外部地址不能落回 NPU 自己（那会把总线打成一个等不到应答的环）
        self_loop = (cfg_src[31:26] == ADDR_BASE[31:26]);
        desc_ok = (cfg_words != 0) && (cfg_words <= `NPU_DMA_WORDS_MAX) &&
                  (cfg_mode <= 1) &&
                  (cfg_src[1:0] == 2'd0) && (cfg_dst[1:0] == 2'd0) &&
                  !src_end[32] && local_window_ok(cfg_dst, dst_end) &&
                  !self_loop &&
                  // 权重块只有写口，没有主机读口 ⇒ 从它往外搬的描述符直接判非法
                  !((cfg_mode == 32'd1) && is_wgt_win);
    end

    // 描述符落到的 bank。A/BT 用 bit[11]，C 用 bit[10]，权重块没有 bank。
    reg is_c_win, is_wgt_win, is_ab_win, dst_bank;
    always @(*) begin
        is_c_win   = (cfg_dst[31:11] == 21'd6);
        is_wgt_win = (cfg_dst[31:15] == 17'd0) && cfg_dst[14];
        is_ab_win  = !is_c_win && !is_wgt_win;
        dst_bank   = is_c_win ? cfg_dst[10] : cfg_dst[11];
    end
    // 和核抢端口的情况，按 RAM 的**端口归属**判，不是按"bank 一样不一样"一刀切：
    //   · A/BT 写口始终归主机 ⇒ 外部→A/BT 只要不是核正在读的那一组就放行（双缓冲的本意）；
    //   · A/BT 读口在 core_busy 期间归核 ⇒ 从 A/BT 往外搬一律等它算完；
    //   · C 写口在 core_busy 期间归写回 FIFO ⇒ 往 C 搬要等；
    //   · C 读口平时空着，只有跨任务累加(accumulate=1)的读-改-写要用 ⇒ 那时要等。
    reg bank_conflict;
    always @(*) begin
        if (cfg_mode == 32'd0)   // 外部 → 本地（要写本地）
            bank_conflict = core_busy && (is_c_win || is_wgt_win || (dst_bank == core_bank));
        else                     // 本地 → 外部（要读本地）
            bank_conflict = core_busy && (is_ab_win || is_wgt_win || (dst_bank == core_bank) ||
                                          (is_c_win && core_accumulate));
    end

    //---------------------------------------------------------------- 组合控制
    // LOAD：在飞量 = 已发地址 - 已完成
    reg [31:0] inflight;
    reg        load_issue, load_accept, load_data, load_data_ok;
    // STORE
    reg        store_read, store_push, store_pop, store_write, store_accept;
    reg [4:0]  fifo_used;      // 5 位：fifo_count 满 8 时再加 1 不会回绕

    always @(*) begin
        inflight   = issue_n - done_n;
        fifo_used  = {1'b0, fifo_count} + {4'd0, read_pending};
        // 地址相位：只在窗口没满、且不在换手空隙里的时候拉高
        load_issue  = (state == S_LOAD) && (issue_n < run_words) &&
                      (inflight < {28'd0, INFLIGHT_MAX}) && !gap_q;
        load_accept = load_issue && m_hready;
        load_data    = (state == S_LOAD) && m_rd_valid;
        load_data_ok = load_data && !m_hresp;

        store_read  = (state == S_STORE) && (prefetch_n < run_words) &&
                      (fifo_used < {1'b0, FIFO_DEPTH});
        store_push  = (state == S_STORE) && read_pending;
        store_write = (state == S_STORE) && (fifo_count != 4'd0) && !gap_q;
        store_pop   = store_write && m_hready;
        store_accept = store_write && m_hready;
    end

    //---------------------------------------------------------------- 寄存器读
    always @(*) begin
        case (reg_off)
            R_CTRL:      reg_rdata = 32'd0;                       // 命令寄存器，读回 0
            R_STATUS:    reg_rdata = {24'd0, error_code, 1'b0, error, done, busy};
            R_SRC:       reg_rdata = cfg_src;
            R_DST:       reg_rdata = cfg_dst;
            R_WORDS:     reg_rdata = cfg_words;
            R_MODE:      reg_rdata = cfg_mode;
            R_COMPLETED: reg_rdata = done_n;
            R_ERR:       reg_rdata = {28'd0, error_code};
            default:     reg_rdata = 32'd0;
        endcase
    end

    //---------------------------------------------------------------- 总线输出
    // HTRANS 在没拿到授权 / 没数据可发的时候一律 IDLE：本总线没有 HBUSREQ，
    // 仲裁器把 htrans!=0 当成"我有请求"，所以不能靠关 htrans 来等授权。
    always @(*) begin
        m_haddr   = 32'd0;
        m_htrans  = HTRANS_IDLE;
        m_hwrite  = 1'b0;
        m_hsize   = 3'b010;
        m_hburst  = 3'b000;      // SINGLE
        m_hprot   = 4'b0011;
        m_hwdata  = 32'd0;
        m_hwstrb  = 4'd0;
        if (load_issue) begin
            m_haddr  = run_src + {issue_n, 2'b00};
            m_htrans = HTRANS_NONSEQ;
            m_hwrite = 1'b0;
        end
        else if (store_write) begin
            m_haddr  = run_src + {done_n, 2'b00};
            m_htrans = HTRANS_NONSEQ;
            m_hwrite = 1'b1;
            m_hwdata = fifo_mem[fifo_rd];
            m_hwstrb = 4'hF;
        end
    end

    //---------------------------------------------------------------- 本地口
    always @(*) begin
        local_valid = 1'b0;
        local_we    = 1'b0;
        local_addr  = 32'd0;
        local_wdata = 32'd0;
        if (load_data_ok) begin
            // 读回来的数据直接写本地：地址按已完成数走，天然和读顺序对齐
            local_valid = 1'b1;
            local_we    = 1'b1;
            local_addr  = run_dst + {done_n, 2'b00};
            local_wdata = m_hrdata;
        end
        else if (store_read) begin
            local_valid = 1'b1;
            local_we    = 1'b0;
            local_addr  = run_dst + {prefetch_n, 2'b00};
        end
    end

    //---------------------------------------------------------------- 时序
    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            cfg_src     <= 32'd0;
            cfg_dst     <= 32'd0;
            cfg_words   <= 32'd0;
            cfg_mode    <= 32'd0;
            run_src     <= 32'd0;
            run_dst     <= 32'd0;
            run_words   <= 32'd0;
            run_load    <= 1'b0;
            issue_n     <= 32'd0;
            done_n      <= 32'd0;
            prefetch_n  <= 32'd0;
            error_code  <= ERR_NONE;
            gap_cnt     <= 3'd0;
            gap_q       <= 1'b0;
            fifo_wr     <= 3'd0;
            fifo_rd     <= 3'd0;
            fifo_count  <= 4'd0;
            read_pending<= 1'b0;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
        end
        else begin
            // 换手空隙：每接受 8 笔地址插一拍 IDLE（见文件头第 3 条）
            if ((load_accept || store_pop) && (gap_cnt == 3'd7)) begin
                gap_cnt <= 3'd0;
                gap_q   <= 1'b1;
            end
            else if (gap_q) begin
                gap_q   <= 1'b0;
                gap_cnt <= 3'd0;
            end
            else if (load_accept || store_pop) begin
                gap_cnt <= gap_cnt + 3'd1;
            end

            // 本地同步 RAM：上一拍发的读，这一拍数据回来
            read_pending <= store_read;
            // ★ prefetch_n 必须【发读那一拍】就加，不能等入 FIFO 再加。
            //   等入队再加的话，同一笔读会在"发了但还没入队"的那一拍被重发一次，
            //   FIFO 里于是出现{0,0,1,2,...}这种重复项（实测写回整体错位一格）。
            if (store_read)
                prefetch_n <= prefetch_n + 32'd1;
            if (store_push) begin
                fifo_mem[fifo_wr] <= local_rdata;
                fifo_wr <= fifo_wr + 3'd1;
            end
            if (store_pop)
                fifo_rd <= fifo_rd + 3'd1;
            case ({store_push, store_pop})
                2'b10:   fifo_count <= fifo_count + 4'd1;
                2'b01:   fifo_count <= fifo_count - 4'd1;
                default: ;
            endcase

            if (load_accept)
                issue_n <= issue_n + 32'd1;

            case (state)
                //-------------------------------------------------------- 等启动
                S_IDLE: begin
                    busy <= 1'b0;
                    if (reg_valid && reg_we && (reg_off == R_CTRL)) begin
                        if (reg_wdata[1]) begin
                            done       <= 1'b0;
                            error      <= 1'b0;
                            error_code <= ERR_NONE;
                        end
                        if (reg_wdata[0]) begin
                            done       <= 1'b0;
                            error      <= 1'b0;
                            error_code <= ERR_NONE;
                            if (!desc_ok) begin
                                error      <= 1'b1;
                                error_code <= ERR_DESCRIPTOR;
                            end
                            else if (bank_conflict) begin
                                error      <= 1'b1;
                                error_code <= ERR_BANK_BUSY;
                            end
                            else begin
                                run_src    <= cfg_src;
                                run_dst    <= cfg_dst;
                                run_words  <= cfg_words;
                                run_load   <= (cfg_mode == 32'd0);
                                issue_n    <= 32'd0;
                                done_n     <= 32'd0;
                                prefetch_n <= 32'd0;
                                fifo_wr    <= 3'd0;
                                fifo_rd    <= 3'd0;
                                fifo_count <= 4'd0;
                                gap_cnt    <= 3'd0;
                                gap_q      <= 1'b0;
                                busy       <= 1'b1;
                                state      <= (cfg_mode == 32'd0) ? S_LOAD : S_STORE;
                            end
                        end
                    end
                    else if (reg_valid && reg_we) begin
                        case (reg_off)
                            R_SRC:   cfg_src   <= reg_wdata;
                            R_DST:   cfg_dst   <= reg_wdata;
                            R_WORDS: cfg_words <= reg_wdata;
                            R_MODE:  cfg_mode  <= reg_wdata;
                            default: ;
                        endcase
                    end
                end
                //-------------------------------------------------------- 外部 → 本地
                S_LOAD: begin
                    busy <= 1'b1;
                    if (load_data) begin
                        if (!load_data_ok) begin
                            error      <= 1'b1;
                            error_code <= ERR_BUS;
                            busy       <= 1'b0;
                            state      <= S_IDLE;
                        end
                        else if (done_n + 32'd1 == run_words) begin
                            done_n <= done_n + 32'd1;
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            state  <= S_IDLE;
                        end
                        else
                            done_n <= done_n + 32'd1;
                    end
                end
                //-------------------------------------------------------- 本地 → 外部
                S_STORE: begin
                    busy <= 1'b1;
                    if (store_accept) begin
                        if (m_hresp) begin
                            error      <= 1'b1;
                            error_code <= ERR_BUS;
                            busy       <= 1'b0;
                            state      <= S_IDLE;
                        end
                        else if (done_n + 32'd1 == run_words) begin
                            done_n <= done_n + 32'd1;
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            state  <= S_IDLE;
                        end
                        else
                            done_n <= done_n + 32'd1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
