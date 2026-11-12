`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: boot_loader
// Description: 上电搬运引擎 —— AHB 主设备（m3），从 SPI flash 把镜像搬进 STCM。
//
//   它不跑代码、不改复位向量：CPU 还在复位里的时候，这条状态机独占总线，
//   用 7 号槽的 pmem_spi 一个字节一个字节地把镜像读出来、攒成 32 位字、写进 STCM，
//   搬完把 boot_done 拉起来，由 bus_con_top 放开 CPU（CPU 从地址 0 开始取指）。
//
//   镜像格式（flash 偏移，SPI 先出的字节在前 = 小端）：
//     0x00  magic  4 字节   'R','V','S','1'
//     0x04  length 4 字节   payload 字节数（1..65536，超了夹到 65536）
//     0x08  entry  4 字节   保留（本期不用；CPU 固定从 0 开始）
//     0x0C  payload ...     原样搬到 STCM 的 0 地址
//
//   三种收场：
//     boot_done=1, boot_err=0  搬完 length 字节
//     boot_done=1, boot_err=1  魔数不符 / 长度非法 / SPI 一直不应答 ⇒ 一个字节都不写 STCM
//     复位期间 boot_done=0
//
//   ★ 总线每次只发一笔单拍传输，发出去按住地址直到 hgrant&hready，被接受之后
//     立刻把 HTRANS 拉回 IDLE —— 这是 AHB 的硬要求（地址挂着会被从设备当成连续传输）。
//     读数据走本仓的 rd_valid 完成脉冲。
//   ★ 错误应答（hresp）本期不区分：本仓的从设备在错误时仍给一个完成脉冲，引擎照走。
//////////////////////////////////////////////////////////////////////////////////


module boot_loader #(
    parameter [31:0] SPI_BASE  = 32'h1C00_0000,   // 7 号槽
    parameter [31:0] LOAD_BASE = 32'h0000_0000    // 镜像落到 STCM 的哪里
)(
    input clk, rst,

//---- AHB-Lite 主设备口（m3）----
    output reg [31:0] haddr,
    output reg [1:0]  htrans,
    output reg        hwrite,
    output reg [2:0]  hsize,
    output reg [2:0]  hburst,
    output reg [3:0]  hprot,
    output reg        hmastlock,
    output reg [31:0] hwdata,
    output reg [3:0]  hwstrb,
    input      [31:0] hrdata,
    input             hready,
    input             hresp,
    input             hgrant,
    input             rd_valid,

//---- 状态（给 pmem_spi 的 BOOT 寄存器展示）----
    output reg        boot_done,
    output reg        boot_err,
    output reg [16:0] boot_count
    );

    localparam [1:0] HTRANS_IDLE   = 2'b00;
    localparam [1:0] HTRANS_NONSEQ = 2'b10;

//7 号槽寄存器（字偏移 0..7，字节地址 +4）
    localparam [31:0] R_CTRL   = SPI_BASE + 32'h00;
    localparam [31:0] R_STATUS = SPI_BASE + 32'h04;
    localparam [31:0] R_TXDATA = SPI_BASE + 32'h08;
    localparam [31:0] R_RXDATA = SPI_BASE + 32'h0c;
    localparam [31:0] R_CS     = SPI_BASE + 32'h10;

    localparam [31:0] MAGIC    = 32'h3153_5652;   // 'R','V','S','1' 小端
    localparam [31:0] MAX_LEN  = 32'd65536;
    localparam [31:0] POLL_MAX = 32'd65536;       // 一个字节最多轮询这么多拍

//==============================================================================
//  一、单拍 AHB 主设备
//==============================================================================
    reg reset_registered;
    always @(posedge clk)
        reset_registered <= rst;

    reg        pend;
    reg [31:0] lat_addr, lat_wdata;
    reg [3:0]  lat_strb;
    reg        lat_we;

    wire accepted = pend & hgrant & hready;

//发起：bus_go 一拍脉冲；完成：写=被接受那一拍，读=rd_valid 那一拍
    reg        bus_go, bus_we, op_busy;
    reg [31:0] bus_addr, bus_wdata;
    reg [3:0]  bus_strb;
    reg [31:0] bus_rdata;
    reg        bus_done;

    always @(posedge clk) begin
        if (reset_registered) begin
            pend <= 1'b0;
            lat_addr <= 32'd0;
            lat_wdata <= 32'd0;
            lat_strb <= 4'd0;
            lat_we <= 1'b0;
            bus_rdata <= 32'd0;
            bus_done <= 1'b0;
        end
        else begin
            bus_done <= 1'b0;
            if (bus_go && (!pend || accepted)) begin
                pend <= 1'b1;
                lat_addr <= bus_addr;
                lat_wdata <= bus_wdata;
                lat_strb <= bus_strb;
                lat_we <= bus_we;
            end
            else if (accepted)
                pend <= 1'b0;

            if (rd_valid) begin
                bus_rdata <= hrdata;
                bus_done <= 1'b1;
            end
            else if (accepted && lat_we)
                bus_done <= 1'b1;
        end
    end

    always @(*) begin
        haddr  = lat_addr;
        htrans = pend ? HTRANS_NONSEQ : HTRANS_IDLE;
        hwrite = lat_we;
        hsize  = 3'b010;
        hburst = 3'b000;
        hprot  = 4'b0011;
        hmastlock = 1'b0;
        hwstrb = lat_strb;
        hwdata = lat_wdata;
    end

//==============================================================================
//  二、搬运序列
//     S_IDLE → 选 flash → [命令前缀 0x03+3 字节地址] → [收 N 字节] → 收尾
//     每个 SPI 字节四次总线动作：写 TXDATA / 写 CTRL(start) / 轮询 STATUS / 读 RXDATA
//==============================================================================
    localparam [3:0]
    S_IDLE   = 4'd0,
    S_SEL    = 4'd1,    // 写 CS
    S_PUT    = 4'd2,    // 写 TXDATA
    S_GO     = 4'd3,    // 写 CTRL.start
    S_POLL   = 4'd4,    // 读 STATUS
    S_TAKE   = 4'd5,    // 读 RXDATA
    S_STORE  = 4'd6,    // 攒字节 / 写 STCM / 写头部
    S_NEXT   = 4'd7,    // 决定下一个字节
    S_FIN    = 4'd8,
    S_DESEL  = 4'd9;    // 一段结束：先抬 CS，下一段再拉低（flash 靠 CS 边沿分事务）

    reg [3:0]  st;
    reg [2:0]  cmd_idx;        // 已装入 tx_byte 的命令前缀字节数（0..4）
    reg [1:0]  phase;          // 0=读头部 1=读 payload
    reg [23:0] flash_addr;
    reg [16:0] len_left;       // 还没发出的数据字节数（17 位：整块 STCM 64KB = 65536）
    reg [15:0] word_idx;
    reg [31:0] acc;
    reg [1:0]  byte_idx;       // 当前字里已收到的字节数
    reg [31:0] hdr0, hdr1;
    reg [31:0] tx_byte;
    reg [31:0] poll_cnt;
//★ 命令前缀那 4 个字节也会走完 S_TAKE/S_STORE，必须和真正的数据字节分开，
//  否则 hdr0 会先吃到 0x03/地址那几拍的空回读（实测 hdr0 变成 FEFFFFFF）。
    reg        data_byte;

    wire header_phase = (phase == 2'd0);

    // 累加：SPI 先出的字节落在最低位（小端，与内存布局一致）
    reg [31:0] acc_next;
    always @(*) begin
        case (byte_idx)
        2'd0:    acc_next = {24'd0, bus_rdata[7:0]};
        2'd1:    acc_next = {16'd0, bus_rdata[7:0], acc[7:0]};
        2'd2:    acc_next = {8'd0,  bus_rdata[7:0], acc[15:0]};
        default: acc_next = {bus_rdata[7:0], acc[23:0]};
        endcase
    end

    wire block_full = (byte_idx == 2'd3);
    wire block_end  = block_full | (len_left == 17'd0);   // 凑满 4 字节，或这是最后一个字节

    // 数据字节的字节使能：最后一块可能只有 1~3 个有效字节
    reg [3:0] data_strb;
    always @(*) begin
        case (byte_idx)
        2'd0:    data_strb = 4'b0001;
        2'd1:    data_strb = 4'b0011;
        2'd2:    data_strb = 4'b0111;
        default: data_strb = 4'b1111;
        endcase
    end

    wire [31:0] hdr_len = hdr1;
    wire hdr_ok = (hdr0 == MAGIC) && (hdr_len != 32'd0);

    // 本拍的总线动作（由 st 挑）
    reg [31:0] op_addr, op_wdata;
    reg [3:0]  op_strb;
    reg        op_we, op_go;

    always @(*) begin
        op_addr  = 32'd0;
        op_wdata = 32'd0;
        op_strb  = 4'b1111;
        op_we    = 1'b0;
        op_go    = 1'b0;
        case (st)
        S_SEL: begin
            op_addr  = R_CS;
            op_wdata = 32'h0000_0002;    // bit0=0：flash 选中；bit1=1：SD 不选
            op_we    = 1'b1;
            op_go    = 1'b1;
        end
        S_PUT: begin
            op_addr  = R_TXDATA;
            op_wdata = tx_byte;
            op_we    = 1'b1;
            op_go    = 1'b1;
        end
        S_GO: begin
            op_addr  = R_CTRL;
            op_wdata = 32'h0000_0001;    // bit0=start
            op_we    = 1'b1;
            op_go    = 1'b1;
        end
        S_POLL: begin
            op_addr = R_STATUS;
            op_go   = 1'b1;
        end
        S_TAKE: begin
            op_addr = R_RXDATA;
            op_go   = 1'b1;
        end
        S_STORE: begin
            if (!header_phase && block_end) begin
                op_addr  = LOAD_BASE + {word_idx, 2'b00};
                op_wdata = acc_next;
                op_strb  = data_strb;
                op_we    = 1'b1;
                op_go    = 1'b1;
            end
        end
        S_DESEL: begin
            op_addr  = R_CS;
            op_wdata = 32'h0000_0003;    // 两路 CS 都拉高
            op_we    = 1'b1;
            op_go    = 1'b1;
        end
        S_FIN: begin
            op_addr  = R_CS;
            op_wdata = 32'h0000_0003;    // 两路 CS 都拉高
            op_we    = 1'b1;
            op_go    = ~boot_done;       // ★ 抬起 CS 只发一次：done 之后就闭嘴，
                                         //   否则状态机停在 S_FIN 会一直重发这笔写，
                                         //   而 m3 是最高优先级 ⇒ 搬完还在堵总线
        end
        default: begin
        end
        endcase
    end

//★ 一次只允许一笔在飞：S_POLL 连着好几拍都要 op，不加这道门会一笔还没回来就再发一笔
    reg issue;
    always @(*) begin
        issue = 1'b0;
        if (op_go & ~op_busy & ~pend)
            issue = 1'b1;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            st <= S_IDLE;
            cmd_idx <= 3'd0;
            phase <= 2'd0;
            flash_addr <= 24'd0;
            len_left <= 17'd0;
            word_idx <= 16'd0;
            acc <= 32'd0;
            byte_idx <= 2'd0;
            hdr0 <= 32'd0;
            hdr1 <= 32'd0;
            tx_byte <= 32'd0;
            poll_cnt <= 32'd0;
            data_byte <= 1'b0;
            boot_done <= 1'b0;
            boot_err <= 1'b0;
            boot_count <= 16'd0;
            bus_go <= 1'b0;
            bus_we <= 1'b0;
            bus_addr <= 32'd0;
            bus_wdata <= 32'd0;
            bus_strb <= 4'd0;
            op_busy <= 1'b0;
        end
        else begin
            bus_go <= 1'b0;
            bus_addr <= op_addr;
            bus_wdata <= op_wdata;
            bus_strb <= op_strb;
            bus_we <= op_we;
            if (issue)
                bus_go <= 1'b1;
            if (bus_done)
                op_busy <= 1'b0;
            else if (issue)
                op_busy <= 1'b1;

            case (st)
            S_IDLE: begin
                phase <= 2'd0;
                cmd_idx <= 3'd1;
                flash_addr <= 24'd0;
                len_left <= 17'd12;
                word_idx <= 16'd0;
                acc <= 32'd0;
                byte_idx <= 2'd0;
                tx_byte <= 32'h0000_0003;    // READ 命令
                poll_cnt <= 32'd0;
                data_byte <= 1'b0;
                st <= S_SEL;
            end

            S_SEL: begin
                if (bus_done)
                    st <= S_PUT;
            end

            S_PUT: begin
                if (bus_done)
                    st <= S_GO;
            end

            S_GO: begin
                if (bus_done) begin
                    poll_cnt <= 32'd0;      // ★ 超时是"这一个字节"的超时，不能跨字节累加
                    st <= S_POLL;
                end
            end

            S_POLL: begin
                if (bus_done) begin
                    if (bus_rdata[1])
                        st <= S_TAKE;
                    else if (poll_cnt >= POLL_MAX) begin
                        boot_err <= 1'b1;
                        st <= S_FIN;
                    end
                    else begin
                        poll_cnt <= poll_cnt + 32'd1;
                        st <= S_POLL;
                    end
                end
            end

            S_TAKE: begin
                if (bus_done)
                    st <= S_STORE;
            end

            S_STORE: begin
                if (!data_byte) begin
                    st <= S_NEXT;
                end
                else if (!block_end) begin
                    acc <= acc_next;
                    byte_idx <= byte_idx + 2'd1;
                    st <= S_NEXT;
                end
                else if (header_phase) begin
                    if (word_idx == 16'd0)
                        hdr0 <= acc_next;
                    else if (word_idx == 16'd1)
                        hdr1 <= acc_next;
                    else
                        hdr1 <= hdr1;
                    word_idx <= word_idx + 16'd1;
                    acc <= 32'd0;
                    byte_idx <= 2'd0;
                    st <= S_NEXT;
                end
                else if (bus_done) begin
                    word_idx <= word_idx + 16'd1;
                    boot_count <= boot_count + {15'd0, byte_idx} + 17'd1;
                    acc <= 32'd0;
                    byte_idx <= 2'd0;
                    st <= S_NEXT;
                end
            end

            S_DESEL: begin
                if (bus_done)
                    st <= S_SEL;
            end

            S_NEXT: begin
                if (cmd_idx < 3'd4) begin
                    case (cmd_idx)
                    3'd1: tx_byte <= {8'd0, flash_addr[23:16]};
                    3'd2: tx_byte <= {8'd0, flash_addr[15:8]};
                    default: tx_byte <= {8'd0, flash_addr[7:0]};
                    endcase
                    cmd_idx <= cmd_idx + 3'd1;
                    data_byte <= 1'b0;
                    st <= S_PUT;
                end
                else if (len_left != 17'd0) begin
                    len_left <= len_left - 17'd1;
                    tx_byte <= 32'h0000_00ff;
                    data_byte <= 1'b1;
                    st <= S_PUT;
                end
                else if (header_phase) begin
                    if (!hdr_ok) begin
                        boot_err <= 1'b1;
                        st <= S_FIN;
                    end
                    else begin
                        phase <= 2'd1;
                        cmd_idx <= 3'd1;
                        flash_addr <= 24'd12;
                        len_left <= (hdr_len > MAX_LEN) ? MAX_LEN[16:0] : hdr_len[16:0];
                        word_idx <= 16'd0;
                        acc <= 32'd0;
                        byte_idx <= 2'd0;
                        tx_byte <= 32'h0000_0003;
                        data_byte <= 1'b0;      // ★ 换段后第一个字节是命令，不是数据
                        st <= S_DESEL;
                    end
                end
                else
                    st <= S_FIN;
            end

            S_FIN: begin
                if (!boot_done && bus_done)
                    boot_done <= 1'b1;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
