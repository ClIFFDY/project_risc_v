`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: boot_loader_cfg
// Description: 上电搬运 —— 从【配置 flash（Q64）】把系统镜像搬进 STCM
//
//   为什么换成本模块：系统镜像必须放在 Q64 里（配置盘），Q128 是数据盘，
//   格式化/擦干净了也不能把系统弄没。Q64 由 IO_block/cfg_spi.v 管，
//   寄存器协议和 pmem_spi 那套 FIFO 完全不一样，所以单独一个搬运器。
//
//   镜像格式（和原来一样，只是落点从 flash 偏移 0 改成 IMAGE_BASE）：
//     0x00  magic  4 字节   'R','V','S','1'（小端 0x3153_5652）
//     0x04  length 4 字节   payload 字节数（1..65535；更大 ⇒ boot_err，不夹）
//     0x08  entry  4 字节   保留，CPU 固定从 0 开始
//     0x0C  payload ...     原样搬到 STCM 的 LOAD_BASE
//
//   流程（两段传输，中间抬一次 CS）：
//     ① CMD=0x03 ADDR=IMAGE_BASE    LEN=12  → 收 12 字节头部
//     ② CMD=0x03 ADDR=IMAGE_BASE+12 LEN=len → 收 payload，每 4 字节凑一个字写进 STCM
//
//   ★ 读是流式的：cfg_spi 每收到一个字节就抬 rx_valid 等我们把 DATA 读走，
//     读完它才收下一个。所以这里不用管 SCK 节拍，只管 STATUS → DATA。
//
//   ★ 本模块只读，不写 flash：bios mode 的烧写通路在别处（cfg_spi 的写/擦）。
//////////////////////////////////////////////////////////////////////////////////


module boot_loader_cfg #(
    parameter [31:0] SPI_BASE   = 32'h1C02_0000,   // 7 号槽 sub2
    parameter [31:0] IMAGE_BASE = 32'h0070_0000,   // Q64 里系统镜像的落点
    parameter [31:0] LOAD_BASE  = 32'h0000_0000    // 搬进 STCM 的哪个地址
)(
    input clk, rst,
    input      [31:0] hrdata,
    input             hready,
    input             hresp,
    input             hgrant,
    input             rd_valid,

    output reg [31:0] haddr,
    output reg [1:0]  htrans,
    output reg        hwrite,
    output reg [2:0]  hsize,
    output reg [2:0]  hburst,
    output reg [2:0]  hprot,
    output reg        hmastlock,
    output reg [31:0] hwdata,
    output reg [3:0]  hwstrb,

    output reg        boot_done,
    output reg        boot_err,
    output reg [16:0] boot_count
    );

    localparam [1:0] HTRANS_IDLE   = 2'b00;
    localparam [1:0] HTRANS_NONSEQ = 2'b10;

//cfg_spi 的寄存器（字节偏移，字索引 = 偏移/4）
    localparam [31:0] W_CTRL   = SPI_BASE + 32'h0000;
    localparam [31:0] W_CMD    = SPI_BASE + 32'h0004;
    localparam [31:0] W_ADDR   = SPI_BASE + 32'h0008;
    localparam [31:0] W_LEN    = SPI_BASE + 32'h000C;
    localparam [31:0] W_DATA   = SPI_BASE + 32'h0010;
    localparam [31:0] W_STATUS = SPI_BASE + 32'h0014;

    localparam [7:0]  C_READ   = 8'h03;
    localparam [31:0] MAGIC    = 32'h3153_5652;   // 'R','V','S','1'
//★ 上限是 65535 不是 65536：cfg_spi 的 LEN 寄存器是 16 位（len_reg[15:0]），
//  写 65536 会被截成 0 ⇒ 一个字节都不读。超过 65535 的镜像直接判错，
//  不做"夹到上限"——夹一下就是悄悄少搬一截，开机后花在别处（栈/数据）上更难查。
    localparam [31:0] MAX_LEN  = 32'd65535;
    localparam [31:0] POLL_MAX = 32'd100000;

//==============================================================================
//  一、单拍 AHB 主设备（和 boot_loader 同一套壳）
//==============================================================================
    reg reset_registered;
    always @(posedge clk) begin
        reset_registered <= rst;
    end

    reg        pend;
    reg [31:0] lat_addr, lat_wdata;
    reg [3:0]  lat_strb;
    reg        lat_we;

    reg accepted;
    always @(*) begin
        accepted = pend & hgrant & hready;
    end

    reg        bus_go, bus_we, bus_done;
    reg [31:0] bus_addr, bus_wdata, bus_rdata;
    reg [3:0]  bus_strb;

    always @(posedge clk) begin
        if (reset_registered) begin
            pend      <= 1'b0;
            lat_addr  <= 32'd0;
            lat_wdata <= 32'd0;
            lat_strb  <= 4'd0;
            lat_we    <= 1'b0;
            bus_rdata <= 32'd0;
            bus_done  <= 1'b0;
        end
        else begin
            bus_done <= 1'b0;
            if (bus_go && (!pend || accepted)) begin
                pend      <= 1'b1;
                lat_addr  <= bus_addr;
                lat_wdata <= bus_wdata;
                lat_strb  <= bus_strb;
                lat_we    <= bus_we;
            end
            else if (accepted) begin
                pend <= 1'b0;
            end

            if (rd_valid) begin
                bus_rdata <= hrdata;
                bus_done  <= 1'b1;
            end
            else if (accepted && lat_we) begin
                bus_done <= 1'b1;
            end
        end
    end

    always @(*) begin
        haddr     = lat_addr;
        htrans    = pend ? HTRANS_NONSEQ : HTRANS_IDLE;
        hwrite    = lat_we;
        hsize     = 3'b010;
        hburst    = 3'b000;
        hprot     = 4'b0011;
        hmastlock = 1'b0;
        hwstrb    = lat_strb;
        hwdata    = lat_wdata;
    end

//==============================================================================
//  二、搬运序列
//==============================================================================
    localparam [3:0]
    S_HDR_GO   = 4'd0,   // 发起头部那段读
    S_HDR_POLL = 4'd1,   // 等 rx_valid
    S_HDR_TAKE = 4'd2,   // 读 DATA
    S_BODY_GO  = 4'd3,   // 发起 payload 那段读
    S_BODY_POLL= 4'd4,
    S_BODY_TAKE= 4'd5,
    S_FIN      = 4'd6,
    S_IDLEW    = 4'd7,   // 等上一段传输真的收尾（busy=0）再发下一段，否则 start 会撞上忙
    S_WRWAIT   = 4'd8;   // 写 STCM 那一笔等 bus_done

    reg [3:0]  state;
    reg [3:0]  sub;              // 序列内的子步
    reg [3:0]  hdr_i;            // 头部已收字节数
    reg [95:0] hdr_sh;           // 12 字节头部：新字节从 [95:88] 进，最老的在 [7:0]
    reg [31:0] acc;              // payload 攒字
    reg [1:0]  acc_i;
    reg [16:0] left;             // 还没收的 payload 字节数
    reg [16:0] widx;             // 已经写进 STCM 的字数
    reg [31:0] poll_cnt;
    reg [31:0] img_len;
    reg        hdr_bad;
    reg        issued;           // 当前 sub 的事务已经发出去了，等 bus_done
    reg [4:0]  wait_cnt;         // 发出后等了多少拍；等太久就重发（APB 单拍事务 16 拍足够）

    wire [31:0] shift_byte;
    assign shift_byte = {24'd0, bus_rdata[7:0]};

//★ 收满 12 字节的那一拍，hdr_sh 还停在 11 字节的旧值上（NBA 要到下拍才更新），
//  所以判 magic / length 必须拿"接上本字节之后"的值，不能拿 hdr_sh 本身。
    wire [95:0] hdr_next;
    assign hdr_next = {shift_byte[7:0], hdr_sh[95:8]};

//payload 攒字的同理：新字节从高位进，攒满时用这个值直接写 STCM
    wire [31:0] acc_next;
    assign acc_next = {bus_rdata[7:0], acc[31:8]};

    always @(posedge clk) begin
        if (reset_registered) begin
            state      <= S_HDR_GO;
            sub        <= 4'd0;
            hdr_i      <= 4'd0;
            hdr_sh     <= 96'd0;
            acc        <= 32'd0;
            acc_i      <= 2'd0;
            left       <= 17'd0;
            widx       <= 17'd0;
            poll_cnt   <= 32'd0;
            img_len    <= 32'd0;
            hdr_bad    <= 1'b0;
            issued     <= 1'b0;
            wait_cnt   <= 5'd0;
            boot_done  <= 1'b0;
            boot_err   <= 1'b0;
            boot_count <= 17'd0;
            bus_go     <= 1'b0;
            bus_we     <= 1'b0;
            bus_addr   <= 32'd0;
            bus_wdata  <= 32'd0;
            bus_strb   <= 4'd0;
        end
        else begin
            bus_go <= 1'b0;

//握手看门狗：issued 抬着但迟迟等不到 bus_done，就重发这一笔
            if (issued)
                wait_cnt <= wait_cnt + 5'd1;
            else
                wait_cnt <= 5'd0;
            if (issued && (wait_cnt >= 5'd16))
                issued <= 1'b0;

            case (state)
//--------------------------------------------------------------- 头部
            S_HDR_GO: begin
                if (~issued) begin
                    issued <= 1'b1;
                    case (sub)
                    4'd0: begin bus_addr <= W_CMD;  bus_wdata <= {24'd0, C_READ}; bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    4'd1: begin bus_addr <= W_ADDR; bus_wdata <= IMAGE_BASE;      bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    4'd2: begin bus_addr <= W_LEN;  bus_wdata <= 32'd12;          bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    4'd3: begin bus_addr <= W_CTRL; bus_wdata <= 32'd1;           bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    default: begin
                        poll_cnt <= 32'd0;
                        state    <= S_HDR_POLL;
                        sub      <= 4'd0;
                        issued   <= 1'b0;
                    end
                    endcase
                end
                else if (bus_done) begin
                    issued <= 1'b0;
                    if (sub != 4'd4)
                        sub <= sub + 4'd1;
                end
            end

            S_HDR_POLL: begin
                if (~issued) begin
                    issued   <= 1'b1;
                    bus_addr <= W_STATUS;
                    bus_we   <= 1'b0;
                    bus_strb <= 4'hF;
                    bus_go   <= 1'b1;
                end
                else if (bus_done) begin
                    issued <= 1'b0;
                    if (bus_rdata[2]) begin
                        boot_err  <= 1'b1;
                        boot_done <= 1'b1;
                        state     <= S_FIN;
                    end
                    else if (poll_cnt > POLL_MAX) begin
                        boot_err  <= 1'b1;
                        boot_done <= 1'b1;
                        state     <= S_FIN;
                    end
                    else if (bus_rdata[3]) begin
                        bus_addr <= W_DATA;
                        bus_we   <= 1'b0;
                        bus_strb <= 4'hF;
                        bus_go   <= 1'b1;
                        issued   <= 1'b1;
                        state    <= S_HDR_TAKE;
                    end
                    else begin
                        poll_cnt <= poll_cnt + 32'd1;
                    end
                end
            end

            S_HDR_TAKE: begin
                if (bus_done) begin
                    issued <= 1'b0;
                    hdr_sh <= hdr_next;
                    if (hdr_i == 4'd11) begin
                        state   <= S_IDLEW;
                        sub     <= 4'd0;
                        hdr_i   <= 4'd0;
                        poll_cnt<= 32'd0;
                        widx    <= 17'd0;
                        acc_i   <= 2'd0;
                        acc     <= 32'd0;
                        // 头部 12 字节：magic(4) length(4) entry(4)，小端
                        if (hdr_next[31:0] !== MAGIC) begin
                            hdr_bad <= 1'b1;
                            left    <= 17'd0;
                        end
                        else if (hdr_next[63:32] > MAX_LEN) begin
                            hdr_bad <= 1'b1;          // 超过一次能搬的量：判错，不夹
                            left    <= 17'd0;
                        end
                        else begin
                            //★ 不许写 hdr_next[63:32][16:0]：Vivado 报 Synth 8-2599
                            //  "range is not allowed in a prefix"。Verilator 吃，Vivado 不吃。
                            //  要的是 length 域的低 17 位 = hdr_next[48:32]。
                            left <= hdr_next[48:32];
                        end
                    end
                    else begin
                        hdr_i    <= hdr_i + 4'd1;
                        poll_cnt <= 32'd0;
                        state    <= S_HDR_POLL;
                    end
                end
            end

            S_BODY_GO: begin
                /* ★ 先判"要不要发"，再判"发到哪一步"：原来这两条 else-if 排在
                   if(~issued) 后面，永远轮不到 —— 魔数坏了照样去读正文。 */
                if (hdr_bad) begin
                    boot_err  <= 1'b1;
                    boot_done <= 1'b1;
                    state     <= S_FIN;
                end
                else if (left == 17'd0) begin
                    boot_done <= 1'b1;
                    state     <= S_FIN;
                end
                else if (~issued) begin
                    issued <= 1'b1;
                    case (sub)
                    4'd0: begin bus_addr <= W_CMD;  bus_wdata <= {24'd0, C_READ};        bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    4'd1: begin bus_addr <= W_ADDR; bus_wdata <= IMAGE_BASE + 32'd12;    bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    4'd2: begin bus_addr <= W_LEN;  bus_wdata <= {15'd0, left};          bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    4'd3: begin bus_addr <= W_CTRL; bus_wdata <= 32'd1;                  bus_we <= 1'b1; bus_strb <= 4'hF; bus_go <= 1'b1; end
                    default: begin
                        poll_cnt <= 32'd0;
                        state    <= S_BODY_POLL;
                        issued   <= 1'b0;
                    end
                    endcase
                end
                else if (bus_done) begin
                    issued <= 1'b0;
                    if (sub != 4'd4)
                        sub <= sub + 4'd1;
                end
            end

            S_BODY_POLL: begin
                if (~issued) begin
                    issued   <= 1'b1;
                    bus_addr <= W_STATUS;
                    bus_we   <= 1'b0;
                    bus_strb <= 4'hF;
                    bus_go   <= 1'b1;
                end
                else if (bus_done) begin
                    issued <= 1'b0;
                    if (bus_rdata[2]) begin
                        boot_err  <= 1'b1;
                        boot_done <= 1'b1;
                        state     <= S_FIN;
                    end
                    else if (poll_cnt > POLL_MAX) begin
                        boot_err  <= 1'b1;
                        boot_done <= 1'b1;
                        state     <= S_FIN;
                    end
                    else if (bus_rdata[3]) begin
                        bus_addr <= W_DATA;
                        bus_we   <= 1'b0;
                        bus_strb <= 4'hF;
                        bus_go   <= 1'b1;
                        issued   <= 1'b1;
                        state    <= S_BODY_TAKE;
                    end
                    else begin
                        poll_cnt <= poll_cnt + 32'd1;
                    end
                end
            end

            S_BODY_TAKE: begin
                if (bus_done) begin
                    issued <= 1'b0;
                    acc      <= acc_next;
                    acc_i    <= acc_i + 2'd1;
                    left     <= left - 17'd1;
                    boot_count <= boot_count + 17'd1;
                    poll_cnt <= 32'd0;
                    if ((acc_i == 2'd3) || (left == 17'd1)) begin
                        bus_addr <= LOAD_BASE + {widx, 2'b00};
                        bus_wdata <= acc_next;
                        bus_we   <= 1'b1;
                        bus_strb <= 4'hF;
                        bus_go   <= 1'b1;
                        acc      <= 32'd0;
                        acc_i    <= 2'd0;
                        widx     <= widx + 17'd1;
                        state    <= S_WRWAIT;
                    end
                    else begin
                        state <= S_BODY_POLL;
                    end
                end
            end

//★ 写 STCM 那一笔也要等到 bus_done 再走：不等就去发下一条 STATUS 读，
//  总线主设备那一拍 pend 还抬着，bus_go 会被这条 if 直接丢掉。
//  顺带在这里收尾：left 减到 0 就说明刚才写的是最后一个字。
            S_WRWAIT: begin
                if (bus_done) begin
                    if (left == 17'd0) begin
                        boot_done <= 1'b1;
                        state     <= S_FIN;
                    end
                    else
                        state <= S_BODY_POLL;
                end
            end

//--------------------------------------------------------------- 收尾
            S_IDLEW: begin
                if (~issued) begin
                    issued   <= 1'b1;
                    bus_addr <= W_STATUS;
                    bus_we   <= 1'b0;
                    bus_strb <= 4'hF;
                    bus_go   <= 1'b1;
                end
                else if (bus_done) begin
                    issued <= 1'b0;
                    if (~bus_rdata[0])
                        state <= S_BODY_GO;
                end
            end

            S_FIN: begin
                bus_go <= 1'b0;
            end

            default: begin
                state <= S_HDR_GO;
            end
            endcase
        end
    end

endmodule
