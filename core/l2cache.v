`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: l2cache
// Description:
//   二级缓存（128KB / 128 字节行 / 直接映射），取代原来的 itcm(64KB) + dtcm(32KB)。
//
//   存储层次：
//       icache(32KB) / dcache(32KB) ──► l2cache(128KB) ──► AHB-Lite 主机口（下一级）
//
//   ★ BRAM 推断是硬要求，所以阵列形状只有一种合法写法：
//       一个写口（ram_we/ram_waddr/ram_wdata/ram_wbe 由同一组合块产生）
//     + 一个读口（地址是寄存器、输出再打一拍）
//     tag 阵列同样走【寄存读】——组合读多个地方会被 Vivado 拆成触发器（1024×22 = 22K FF）。
//     写 data_mem 的地方必须合并成一处，否则报
//     "RAM has multiple writes via different ports in same process" 直接综合失败。
//
//   ★ 两级一致性
//     · L1D 写穿到 L2：命中改行并置 dirty；
//     · L2 写脉冲回灌 L1I（icache 按行作废）；
//     · 行被替换时 dirty 行整行写回下一级。
//
//   ★ 上电预装：ins.hex → 代码窗口；data.hex → 数据窗口（"hex 统一烧进二级缓存"）。
//
//   ★ 地址窗口必须和 lsu.v 的 stage1_non_cacheable 逐位一致：
//     lsu 判的是【字地址】word[23:13]==11'h100，等价到字节地址是
//     byte[25:15]==11'h100 ⇒ 窗口 = 0x0080_0000..0x0080_1FFF（8KB）。
//////////////////////////////////////////////////////////////////////////////////

module l2cache #(
    parameter integer L2_AW    = 15,   // 15 = 32768 字 = 128KB
    parameter integer L2_ILINE = 32,   // icache 一次回填字数（= 一整行）
    parameter integer L2_DLINE = 8     // dcache 一次回填字数
)(
    input clk, rst,

    input         i_req,
    input  [31:0] i_addr,
    output [31:0] i_data,
    output        i_valid,

    input         d_req,
    input         d_we,
    input  [31:0] d_addr,
    input  [31:0] d_wdata,
    input  [3:0]  d_be,
    output [31:0] d_data,
    output        d_valid,
    output        d_ready,

    output        w_pulse,
    output [31:0] w_addr,

    output reg [31:0] m_haddr,
    output reg        m_hwrite,
    output reg [1:0]  m_htrans,
    output reg [2:0]  m_hsize,
    output reg [31:0] m_hwdata,
    output reg [3:0]  m_hwstrb,
    input      [31:0] m_hrdata,
    input             m_hready,
    output            m_busy
    );

//============================== 常量 ==============================
    localparam integer L2_WORDS = (1 << L2_AW);
    localparam integer LINE_W   = 5;
    localparam integer L2_LINES = (1 << (L2_AW - LINE_W));
    localparam integer IDX_B    = L2_AW - LINE_W;
    localparam integer TAG_B    = 20;
    localparam integer TM_W     = TAG_B + 2;      // {dirty, valid, tag}

    localparam [IDX_B-1:0] DATA_LINE_BASE = 10'd512;

    localparam [1:0] HTRANS_IDLE   = 2'b00;
    localparam [1:0] HTRANS_NONSEQ = 2'b10;

    localparam [3:0]
        M_IDLE  = 4'd0,
        M_TAG   = 4'd1,
        M_CHECK = 4'd2,
        M_WB    = 4'd3,
        M_FILL  = 4'd4,
        M_SERVE = 4'd5,
        M_WT    = 4'd6;

    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//============================== 地址窗口 ==============================
    function [L2_AW-1:0] arr_word;
        input [31:0] a;
        begin
            if ((a[31:26] == 6'd0) && (a[25:15] == 11'h100))
                arr_word = 15'h4000 + a[12:2];
            else
                arr_word = {5'd0, a[16:2]};
        end
    endfunction

    function [IDX_B-1:0] line_index;
        input [31:0] a;
        begin
            if ((a[31:26] == 6'd0) && (a[25:15] == 11'h100))
                line_index = DATA_LINE_BASE + a[12:7];
            else
                line_index = a[16:7];
        end
    endfunction

    function [TAG_B-1:0] line_tag;
        input [31:0] a;
        begin
            line_tag = {((a[31:26] == 6'd0) && (a[25:15] == 11'h100)), a[31:13]};
        end
    endfunction

//============================== 阵列 ==============================
    (* ram_style = "block" *) reg [31:0] data_mem [0:L2_WORDS-1];
    (* ram_style = "block" *) reg [TM_W-1:0] tag_mem [0:L2_LINES-1];

//★ 上电镜像：一幅文件同时含代码(@0)与数据(@0x4000)，单个 $readmemh 进阵列。
//  旧写法用 data_init 中转 + 循环拷贝，Vivado 报 Synth 8-311
//  「ignoring non-constant assignment in initial block」⇒ data.hex 根本没进 BRAM（已验证：
//  0x4000 处 INIT 全 0）。合并成一幅镜像后综合能直接把 @ 指令译成 INIT。
//  镜像由 Tools/mk_l2image.sh 从 ins.hex + data.hex 生成。
    integer k;
    initial begin
        for (k = 0; k < L2_WORDS; k = k + 1) data_mem[k] = 32'd0;
        for (k = 0; k < L2_LINES; k = k + 1) tag_mem[k]  = {TM_W{1'b0}};
        $readmemh("e:/Vivado_Projects/project_risc_v/tools/hex/l2image.hex", data_mem);
        for (k = 0; k < 512; k = k + 1) tag_mem[k]       = {1'b0, 1'b1, 1'b0, 19'h00000};
        for (k = 0; k < 64;  k = k + 1) tag_mem[512 + k] = {1'b0, 1'b1, 1'b1, 19'h00400};
    end

    function [31:0] rebuild_addr;
        input [IDX_B-1:0] idx;
        input [TM_W-1:0]  tm;
        begin
            if (tm[TAG_B-1])
                rebuild_addr = {tm[TAG_B-2:0], ((idx - DATA_LINE_BASE) & 6'h3F), 7'b0};
            else
                rebuild_addr = {tm[TAG_B-2:4], idx, 7'b0};
        end
    endfunction

//============================== 状态寄存器 ==============================
    reg [3:0]       mstate;

    reg [L2_AW-1:0] serve_ptr;
    reg [5:0]       serve_left;
    reg             serve_is_i;
    reg [1:0]       rd_src_q;
    reg [31:0]      rd_data_q;

    reg [L2_AW-1:0] rd_wb_addr;
    wire [L2_AW-1:0] rd_addr_sel = (mstate == M_WB) ? rd_wb_addr : serve_ptr;

    reg [IDX_B-1:0] tag_index;
    reg [TM_W-1:0]  tag_q;

    reg             ram_we;
    reg [L2_AW-1:0] ram_waddr;
    reg [31:0]      ram_wdata;
    reg [3:0]       ram_wbe;

    reg             tag_we;
    reg [IDX_B-1:0] tag_waddr;
    reg [TM_W-1:0]  tag_wdata;

    reg [31:0]      chk_addr;
    reg [IDX_B-1:0] chk_index;
    reg [TAG_B-1:0] chk_tag;
    reg             chk_is_i;
    reg             chk_is_w;

    reg [5:0]       beat;
    reg [31:0]      victim_addr;
    reg             wb_phase;

    reg [31:0]      st_addr, st_data;
    reg [3:0]       st_be;

    reg             req_i_d, req_d_d;
    reg             i_active, d_active;
    reg             i_edge_pend, d_edge_pend;

//store 队列（4 深）：L2 忙的时候到达的写不能丢
    reg [67:0] sq [0:3];
    reg [2:0]  sq_wp, sq_rp;
    wire       sq_empty = (sq_wp == sq_rp);
    wire       sq_full  = (sq_wp[1:0] == sq_rp[1:0]) && (sq_wp[2] != sq_rp[2]);
    wire       sq_push  = d_req & d_we;
    wire [31:0] sq_addr = sq[sq_rp[1:0]][67:36];
    wire [31:0] sq_data = sq[sq_rp[1:0]][35:4];
    wire [3:0]  sq_be   = sq[sq_rp[1:0]][3:0];

    assign m_busy  = (mstate != M_IDLE);
    assign w_pulse = d_req & d_we;
    assign w_addr  = d_addr;

//============================== BRAM 模板 ==============================
    always @(posedge clk) begin
        if (ram_we) begin
            if (ram_wbe[0]) data_mem[ram_waddr][7:0]   <= ram_wdata[7:0];
            if (ram_wbe[1]) data_mem[ram_waddr][15:8]  <= ram_wdata[15:8];
            if (ram_wbe[2]) data_mem[ram_waddr][23:16] <= ram_wdata[23:16];
            if (ram_wbe[3]) data_mem[ram_waddr][31:24] <= ram_wdata[31:24];
        end
//★ 读地址直接取 serve_ptr：它和 rd_src_q 同沿推进，"下一拍的数据"与"下一拍的有效位"才对齐
        rd_data_q <= data_mem[rd_addr_sel];
    end

    always @(posedge clk) begin
        if (tag_we) tag_mem[tag_waddr] <= tag_wdata;
        tag_q <= tag_mem[tag_index];
    end

//============================== 主状态机 ==============================
    always @(posedge clk) begin
        if (reset_registered) begin
            mstate     <= M_IDLE;
            beat       <= 6'd0;
            serve_ptr  <= {L2_AW{1'b0}};
            serve_left <= 6'd0;
            serve_is_i <= 1'b0;
            rd_src_q   <= 2'd0;
            rd_wb_addr <= {L2_AW{1'b0}};
            tag_index  <= {IDX_B{1'b0}};
            ram_we     <= 1'b0;
            ram_waddr  <= {L2_AW{1'b0}};
            ram_wdata  <= 32'd0;
            ram_wbe    <= 4'd0;
            tag_we     <= 1'b0;
            tag_waddr  <= {IDX_B{1'b0}};
            tag_wdata  <= {TM_W{1'b0}};
            chk_addr   <= 32'd0;
            chk_index  <= {IDX_B{1'b0}};
            chk_tag    <= {TAG_B{1'b0}};
            chk_is_i   <= 1'b0;
            chk_is_w   <= 1'b0;
            victim_addr<= 32'd0;
            wb_phase   <= 1'b0;
            st_addr    <= 32'd0;
            st_data    <= 32'd0;
            st_be      <= 4'd0;
            req_i_d    <= 1'b0;
            req_d_d    <= 1'b0;
            i_active   <= 1'b0;
            d_active   <= 1'b0;
            i_edge_pend<= 1'b0;
            d_edge_pend<= 1'b0;
            sq_wp      <= 3'd0;
            sq_rp      <= 3'd0;
            m_haddr    <= 32'd0;
            m_hwrite   <= 1'b0;
            m_htrans   <= HTRANS_IDLE;
            m_hsize    <= 3'b010;
            m_hwdata   <= 32'd0;
            m_hwstrb   <= 4'd0;
        end
        else begin
            req_i_d    <= i_req;
            req_d_d    <= d_req;
            if (i_req & ~req_i_d) i_edge_pend <= 1'b1;
            if (d_req & ~d_we & ~req_d_d) d_edge_pend <= 1'b1;

            if (sq_push && !sq_full) begin
                sq[sq_wp[1:0]] <= {d_addr, d_wdata, d_be};
                sq_wp <= sq_wp + 3'd1;
            end

//默认：不写
            ram_we   <= 1'b0;
            tag_we   <= 1'b0;
            m_htrans <= HTRANS_IDLE;
            m_hwrite <= 1'b0;
            m_hwstrb <= 4'd0;

            if (serve_left != 6'd0) begin
                rd_src_q   <= serve_is_i ? 2'd1 : 2'd2;
                serve_ptr  <= serve_ptr + 1'b1;
                serve_left <= serve_left - 6'd1;
            end
            else rd_src_q <= 2'd0;

            case (mstate)
            M_IDLE: begin
                if (!sq_empty) begin
                    st_addr  <= sq_addr; st_data <= sq_data; st_be <= sq_be;
                    chk_addr <= sq_addr; chk_is_i <= 1'b0; chk_is_w <= 1'b1;
                    chk_index<= line_index(sq_addr); chk_tag <= line_tag(sq_addr);
                    tag_index<= line_index(sq_addr);
                    sq_rp    <= sq_rp + 3'd1;
                    mstate   <= M_TAG;
                end
                else if (i_edge_pend & ~i_active) begin
                    i_edge_pend <= 1'b0;
                    chk_addr <= i_addr; chk_is_i <= 1'b1; chk_is_w <= 1'b0;
                    chk_index<= line_index(i_addr); chk_tag <= line_tag(i_addr);
                    tag_index<= line_index(i_addr);
                    mstate   <= M_TAG;
                end
                else if (d_edge_pend & ~d_active) begin
                    d_edge_pend <= 1'b0;
                    chk_addr <= d_addr; chk_is_i <= 1'b0; chk_is_w <= 1'b0;
                    chk_index<= line_index(d_addr); chk_tag <= line_tag(d_addr);
                    tag_index<= line_index(d_addr);
                    mstate   <= M_TAG;
                end
            end

            M_TAG: mstate <= M_CHECK;

            M_CHECK: begin
                if (tag_q[TAG_B] && (tag_q[TAG_B-1:0] == chk_tag)) begin
                    if (chk_is_w) begin
                        ram_we    <= 1'b1;
                        ram_waddr <= arr_word(chk_addr);
                        ram_wdata <= st_data;
                        ram_wbe   <= st_be;
                        tag_we    <= 1'b1;
                        tag_waddr <= chk_index;
                        tag_wdata <= {1'b1, 1'b1, chk_tag};
                        mstate    <= M_IDLE;
                    end
                    else begin
                        serve_ptr  <= arr_word(chk_addr);
                        serve_left <= chk_is_i ? L2_ILINE : L2_DLINE;
                        serve_is_i <= chk_is_i;
                        if (chk_is_i) i_active <= 1'b1; else d_active <= 1'b1;
                        mstate     <= M_SERVE;
                    end
                end
                else begin
                    victim_addr <= rebuild_addr(chk_index, tag_q);
                    beat  <= 6'd0;
                    if (chk_is_w)            mstate <= M_WT;
                    else if (tag_q[TAG_B+1]) mstate <= M_WB;
                    else                     mstate <= M_FILL;
                end
            end

            M_WB: begin
//两拍一个 beat：先读阵列，下一拍发总线
                if (!wb_phase) begin
                    rd_wb_addr <= {chk_index, beat[4:0]};
                    wb_phase   <= 1'b1;
                end
                else begin
                    m_haddr  <= {victim_addr[31:7], beat[4:0], 2'b00};
                    m_hwrite <= 1'b1;
                    m_htrans <= HTRANS_NONSEQ;
                    m_hsize  <= 3'b010;
                    m_hwstrb <= 4'hF;
                    m_hwdata <= rd_data_q;
                    if (m_hready && (m_htrans == HTRANS_NONSEQ)) begin
                        wb_phase <= 1'b0;
                        if (beat == 6'd31) begin beat <= 6'd0; mstate <= M_FILL; end
                        else beat <= beat + 6'd1;
                    end
                end
            end

            M_FILL: begin
                m_haddr  <= {chk_addr[31:7], beat[4:0], 2'b00};
                m_hwrite <= 1'b0;
                m_htrans <= HTRANS_NONSEQ;
                m_hsize  <= 3'b010;
                if (m_hready && (m_htrans == HTRANS_NONSEQ)) begin
                    ram_we    <= 1'b1;
                    ram_waddr <= {chk_index, beat[4:0]};
                    ram_wdata <= m_hrdata;
                    ram_wbe   <= 4'hF;
                    if (beat == 6'd31) begin
                        tag_we    <= 1'b1;
                        tag_waddr <= chk_index;
                        tag_wdata <= {1'b0, 1'b1, chk_tag};
                        beat      <= 6'd0;
                        serve_ptr <= {chk_index, 5'd0};
                        serve_left<= chk_is_i ? L2_ILINE : L2_DLINE;
                        serve_is_i<= chk_is_i;
                        if (chk_is_i) i_active <= 1'b1; else d_active <= 1'b1;
                        mstate    <= M_SERVE;
                    end
                    else beat <= beat + 6'd1;
                end
            end

            M_WT: begin
                if (m_hready && (m_htrans == HTRANS_NONSEQ)) mstate <= M_IDLE;
                else begin
                    m_haddr  <= {chk_addr[31:2], 2'b00};
                    m_hwrite <= 1'b1;
                    m_htrans <= HTRANS_NONSEQ;
                    m_hsize  <= 3'b010;
                    m_hwdata <= st_data;
                    m_hwstrb <= st_be;
                end
            end

            M_SERVE: begin
                if (serve_left == 6'd1) begin
                    i_active <= 1'b0;
                    d_active <= 1'b0;
                    mstate   <= M_IDLE;
                end
            end
            default: mstate <= M_IDLE;
            endcase
        end
    end

    assign i_data  = rd_data_q;
    assign i_valid = (rd_src_q == 2'd1);
    assign d_data  = rd_data_q;
    assign d_valid = (rd_src_q == 2'd2);
    assign d_ready = 1'b1;

endmodule
