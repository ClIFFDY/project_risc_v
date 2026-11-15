`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: dpu —— Display Processing Unit（9 号槽 0x2400_0000）
//
// 干什么：CPU 只往帧缓冲写"改了的字节"+ 一次 commit，剩下"拼命令、把字节搬上
//        I2C、算到哪儿停"全在这里做。它买的是"CPU 不用在场"，不是帧率。
//
// ★ 它和 I2C 的关系是【同层的兄弟 + 一条侧带】，不是串在 APB 和 I2C 中间：
//      APB 桥 ─┬─ 5 号槽 → i2c.v ──► SCL/SDA
//              └─ 9 号槽 → dpu.v ──(push/start/stop/len)──┘
//   CPU→I2C 那条总线路径一个门都没变。
//
// 地址（bus_addr_in 是【字】地址，= haddr>>2；帧缓冲只写不读，读回一律 0）：
//   0x000..0x0FF   FB0   屏0 帧缓冲 256 字 = 1 KB（页格式：mem[page*128+col]）
//   0x100..0x1FF   FB1   屏1
//   0x200  CTRL    [0] commit  [1] abort  [2] 整屏刷（忽略脏页）  [3] 只清脏页
//   0x201  STATUS  [0] busy  [1] done(W1C)  [2] nack_err(W1C)
//                  [15:8] dirty0  [23:16] dirty1
//   0x202  IRQ     [0] done_en  [1] err_en（整字写，写什么就是什么）
//   0x203  ADDR0   [6:0] 屏0 从机地址（复位 0x3C）
//   0x204  ADDR1   [6:0] 屏1 从机地址（复位 0x3D）
//
// ★ 一个字节 = 该列 8 个纵向像素，bit0 在最上 —— 就是 SSD1306 的原生页格式，
//   流出时零转置，软件侧的字体表本来就是这么排的。
// ★ 速率档不在这里：软件照旧写 i2c 的 CFG（i2c_master_configure）。面板的初始化
//   命令（0xAE/0xD5/…/0xAF）也由软件在启动时走 i2c 的 CPU 路径发一次。
//
// 每个脏页两笔事务（命令和数据必须分开 —— 控制字节只支持 Co=0）：
//   [S][addr+W][0x00][0x21 0 127][0x22 p p][P]        8 字节
//   [S][addr+W][0x40][ 128 字节取自 FB ][P]         130 字节
//   138 字节 × 2870 拍/字节（400 kHz @120 MHz）≈ 3.3 ms/页
//////////////////////////////////////////////////////////////////////////////////

module dpu(
    input clk, rst,
    // 总线侧（与 io_block 其它外设同形状；bus_we_in 已与槽选通与过）
    input      [31:0] bus_addr_in,
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    output reg [31:0] bus_data_out,
    output reg        ld_ready,
    // 到 i2c 的侧带
    output reg        i2c_push,
    output reg [7:0]  i2c_push_data,
    output reg        i2c_ctrl_we,
    output reg        i2c_ctrl_start,
    output reg        i2c_ctrl_stop,
    output reg [7:0]  i2c_ctrl_len,
    input      [3:0]  i2c_tx_used,
    input             i2c_busy,
    input             i2c_err,
    output reg            dpu_irq
    );

    localparam [2:0]
        R_CTRL   = 3'd0,
        R_STATUS = 3'd1,
        R_IRQ    = 3'd2,
        R_ADDR0  = 3'd3,
        R_ADDR1  = 3'd4;

    localparam [3:0]
        DPU_IDLE   = 4'd0,
        DPU_PICK   = 4'd1,
        DPU_CMD    = 4'd2,
        DPU_CMD_W  = 4'd3,
        DPU_DATA   = 4'd4,
        DPU_DATA_R = 4'd5,
        DPU_DATA_P = 4'd6,
        DPU_DATA_W = 4'd7,
        DPU_NEXT   = 4'd8;

    // ---------------- 地址划分 ----------------
    reg sel_frame;
    always @(*) begin
        sel_frame = (bus_addr_in[10:9] == 2'b00);
    end
    // 字 0x000..0x1FF
    reg sel_reg;
    always @(*) begin
        sel_reg = (bus_addr_in[10:9] == 2'b01);
    end
    // 字 0x200..0x3FF
    reg [8:0] frame_word;
    reg [2:0] reg_sel;
    always @(*) begin
        frame_word = bus_addr_in[8:0];
        reg_sel = bus_addr_in[2:0];
    end
    // 0 = 屏0，1 = 屏1
    // 一页 32 字 = 128 字节
    reg        write_screen;
    reg [2:0]  write_page;
    always @(*) begin
        write_screen = frame_word[8];
        write_page = frame_word[7:5];
    end

    // ---------------- 帧缓冲：4 个字节道 ----------------
    // ★ 用 4 个单写口、单读口的 8 位阵列，而不是一个 32 位阵列做字节使能：
    //   同一个 process 里对同一个阵列多条写语句、或写语句带位选，Vivado 推不出
    //   BRAM（PRF/ROB 那次踩过）。分道之后每个阵列只有一条写语句，干净。
    reg [7:0] fb_lane0 [0:511];
    reg [7:0] fb_lane1 [0:511];
    reg [7:0] fb_lane2 [0:511];
    reg [7:0] fb_lane3 [0:511];

    reg frame_write;
    always @(*) begin
        frame_write = bus_we_in && sel_frame;
    end

    always @(posedge clk) begin
        if (frame_write) begin
            if (bus_be_in[0])
                fb_lane0[frame_word] <= bus_data_in[7:0];
            if (bus_be_in[1])
                fb_lane1[frame_word] <= bus_data_in[15:8];
            if (bus_be_in[2])
                fb_lane2[frame_word] <= bus_data_in[23:16];
            if (bus_be_in[3])
                fb_lane3[frame_word] <= bus_data_in[31:24];
        end
    end

    // ---------------- 脏页位图 ----------------
    reg  [7:0] dirty0, dirty1;
    reg        dirty_clear;            // 一拍脉冲
    reg        dirty_clear_screen;
    reg  [2:0] dirty_clear_page;

    reg [7:0] dirty0_set, dirty1_set, dirty0_clr, dirty1_clr;
    always @(*) begin
        dirty0_set = (frame_write && !write_screen) ? (8'd1 << write_page) : 8'd0;
        dirty1_set = (frame_write &&  write_screen) ? (8'd1 << write_page) : 8'd0;
        dirty0_clr = (dirty_clear && !dirty_clear_screen) ? (8'd1 << dirty_clear_page) : 8'd0;
        dirty1_clr = (dirty_clear &&  dirty_clear_screen) ? (8'd1 << dirty_clear_page) : 8'd0;
    end

    // ---------------- 控制 / 状态 ----------------
    reg        start_request, abort_request, full_request, clear_request;
    reg        full_mode;
    reg        frame_busy;
    reg        frame_error;
    reg        done_set, error_set;
    reg        status_done, status_error;
    reg        irq_done_enable, irq_error_enable;
    reg [6:0]  slave_address0, slave_address1;

    // ---------------- 引擎状态 ----------------
    reg [3:0]  state;
    reg        scan_screen;
    reg [2:0]  scan_page;
    reg [3:0]  scan_index;          // 0..15：这一帧已经看过几个槽（2 屏 × 8 页）
    reg [3:0]  command_index;       // CMD 事务的字节号 0..7
    reg [6:0]  data_index;          // DATA 事务里 FB 的字节号 0..127
    reg [23:0] wait_timeout;

    // 帧缓冲读出（每拍都读，地址由 data_index 决定；一个 4 字节组内地址不变）
    reg [8:0] read_word;
    always @(*) begin
        read_word = {scan_screen, scan_page, data_index[6:2]};
    end
    reg  [7:0]  read_byte0, read_byte1, read_byte2, read_byte3;

    always @(posedge clk) begin
        read_byte0 <= fb_lane0[read_word];
        read_byte1 <= fb_lane1[read_word];
        read_byte2 <= fb_lane2[read_word];
        read_byte3 <= fb_lane3[read_word];
    end

    reg  [7:0]  data_byte;
    always @(*) begin
        case (data_index[1:0])
        2'd0:    data_byte = read_byte0;
        2'd1:    data_byte = read_byte1;
        2'd2:    data_byte = read_byte2;
        default: data_byte = read_byte3;
        endcase
    end

    reg [6:0] slave_address_now;
    always @(*) begin
        slave_address_now = scan_screen ? slave_address1 : slave_address0;
    end
    reg slot_dirty;
    always @(*) begin
        slot_dirty = full_mode ||
                     (scan_screen ? dirty1[scan_page] : dirty0[scan_page]);
    end

    // 命令字节表：4'd1 = 0x00（控制字节，后面全是命令）；4'd2 = 0x21（设置列窗口 0..127）；
    // 4'd5 = 0x22（设置页窗口 p..p）。
    reg [7:0]  command_byte;
    always @(*) begin
        case (command_index)
        4'd0: command_byte = {slave_address_now, 1'b0};
        4'd1: command_byte = 8'h00;
        4'd2: command_byte = 8'h21;
        4'd3: command_byte = 8'h00;
        4'd4: command_byte = 8'h7F;
        4'd5: command_byte = 8'h22;
        default: command_byte = {5'd0, scan_page};
        endcase
    end

    // 队列里留一格余量：tx_used 是上一拍的值，留一格才保证这一拍推的一定被收下
    reg push_room;
    always @(*) begin
        push_room = (i2c_tx_used <= 4'd6);
    end

    // ---------------- 主状态机 ----------------
    always @(posedge clk) begin
        if (rst) begin
            state              <= DPU_IDLE;
            frame_busy         <= 1'b0;
            frame_error        <= 1'b0;
            full_mode          <= 1'b0;
            scan_screen        <= 1'b0;
            scan_page          <= 3'd0;
            scan_index         <= 4'd0;
            command_index      <= 4'd0;
            data_index         <= 7'd0;
            wait_timeout       <= 24'd0;
            i2c_push           <= 1'b0;
            i2c_push_data      <= 8'd0;
            i2c_ctrl_we        <= 1'b0;
            i2c_ctrl_start     <= 1'b0;
            i2c_ctrl_stop      <= 1'b0;
            i2c_ctrl_len       <= 8'd0;
            done_set           <= 1'b0;
            error_set          <= 1'b0;
            dirty_clear        <= 1'b0;
            dirty_clear_screen <= 1'b0;
            dirty_clear_page   <= 3'd0;
        end
        else begin
            // 默认全是一拍脉冲
            i2c_push       <= 1'b0;
            i2c_ctrl_we    <= 1'b0;
            i2c_ctrl_start <= 1'b0;
            i2c_ctrl_stop  <= 1'b0;
            i2c_ctrl_len   <= 8'd0;
            done_set       <= 1'b0;
            error_set      <= 1'b0;
            dirty_clear    <= 1'b0;

            // NACK 中止（i2c 那边已经把队列倒了、发了 STOP）：这一帧到 T 就废了
            if (i2c_err)
                frame_error <= 1'b1;

            if (abort_request && (state != DPU_IDLE)) begin
                // ★ 让 i2c 把它手上那笔用 STOP 收干净：不补这一下，长度数不满
                //   会在队列空掉的那一刻静默结束、不发 STOP，屏就卡在半帧上。
                i2c_ctrl_we    <= 1'b1;
                i2c_ctrl_stop  <= 1'b1;
                i2c_ctrl_len   <= 8'd0;
                frame_busy     <= 1'b0;
                full_mode      <= 1'b0;
                done_set       <= 1'b1;
                state          <= DPU_IDLE;
            end
            else
                case (state)
            //------------------------------------------------ 等 commit
            DPU_IDLE: begin
                if (start_request) begin
                    frame_busy  <= 1'b1;
                    frame_error <= 1'b0;
                    full_mode   <= full_request;
                    scan_screen <= 1'b0;
                    scan_page   <= 3'd0;
                    scan_index  <= 4'd0;
                    state       <= DPU_PICK;
                end
            end
            //------------------------------------------------ 挑下一个脏页
            DPU_PICK: begin
                if (slot_dirty) begin
                    command_index <= 4'd0;
                    state         <= DPU_CMD;
                end
                else
                    state <= DPU_NEXT;
            end
            //------------------------------------------------ 推 8 个命令字节
            DPU_CMD: begin
                if (push_room) begin
                    i2c_push      <= 1'b1;
                    i2c_push_data <= command_byte;
                    command_index <= command_index + 4'd1;
                    if (command_index == 4'd7) begin
                        i2c_ctrl_we    <= 1'b1;
                        i2c_ctrl_start <= 1'b1;
                        i2c_ctrl_stop  <= 1'b1;
                        i2c_ctrl_len   <= 8'd8;
                        wait_timeout   <= 24'd0;
                        state          <= DPU_CMD_W;
                    end
                end
            end
            //------------------------------------------------ 等命令事务收尾
            DPU_CMD_W: begin
                wait_timeout <= wait_timeout + 24'd1;
                if (!i2c_busy) begin
                    if (frame_error) begin
                        frame_busy <= 1'b0;
                        done_set <= 1'b1;
                        error_set <= 1'b1;
                        state <= DPU_IDLE;
                    end
                    else begin
                        command_index <= 4'd0;
                        state         <= DPU_DATA;
                    end
                end
                else if (wait_timeout == 24'hFFFFFF) begin
                    frame_error <= 1'b1;
                    frame_busy  <= 1'b0;
                    done_set    <= 1'b1;
                    error_set   <= 1'b1;
                    state       <= DPU_IDLE;
                end
            end
            //------------------------------------------------ 数据流的头 2 字节
            DPU_DATA: begin
                if (push_room) begin
                    i2c_push      <= 1'b1;
                    i2c_push_data <= (command_index == 4'd0) ? {slave_address_now, 1'b0}
                                                             : 8'h40;
                    command_index <= command_index + 4'd1;
                    if (command_index == 4'd1) begin
                        // ★ START/长度必须在【开头】就拨：130 个字节装不进 8 深 FIFO，
                        //   要是等推完再拨 start，队列满了、主机又在 IDLE 等 start，
                        //   两边互相等死（第一版就是这么卡在 DATA_P 上的）。
                        i2c_ctrl_we    <= 1'b1;
                        i2c_ctrl_start <= 1'b1;
                        i2c_ctrl_stop  <= 1'b1;
                        i2c_ctrl_len   <= 8'd130;
                        wait_timeout   <= 24'd0;
                        data_index     <= 7'd0;
                        state          <= DPU_DATA_R;
                    end
                end
            end
            //------------------------------------------------ 读一个字（4 字节）
            DPU_DATA_R: begin
                state <= DPU_DATA_P;
            end
            //------------------------------------------------ 连着推 4 个字节
            DPU_DATA_P: begin
                if (push_room) begin
                    i2c_push      <= 1'b1;
                    i2c_push_data <= data_byte;
                    data_index    <= data_index + 7'd1;
                    if (data_index == 7'd127) begin
                        state          <= DPU_DATA_W;
                    end
                    else if (data_index[1:0] == 2'b11)
                        state <= DPU_DATA_R;
                end
            end
            //------------------------------------------------ 等数据事务收尾
            DPU_DATA_W: begin
                wait_timeout <= wait_timeout + 24'd1;
                if (!i2c_busy) begin
                    state <= DPU_NEXT;
                end
                else if (wait_timeout == 24'hFFFFFF) begin
                    frame_error <= 1'b1;
                    frame_busy  <= 1'b0;
                    done_set    <= 1'b1;
                    error_set   <= 1'b1;
                    state       <= DPU_IDLE;
                end
            end
            //------------------------------------------------ 下一个槽 / 收尾
            DPU_NEXT: begin
                dirty_clear        <= 1'b1;
                // 这一页已经在线上刷过了
                dirty_clear_screen <= scan_screen;
                dirty_clear_page   <= scan_page;
                if (scan_index == 4'd15) begin
                    frame_busy <= 1'b0;
                    full_mode  <= 1'b0;
                    done_set   <= 1'b1;
                    if (frame_error)
                        error_set <= 1'b1;
                    state      <= DPU_IDLE;
                end
                else begin
                    scan_index <= scan_index + 4'd1;
                    if (scan_page == 3'd7) begin
                        scan_page   <= 3'd0;
                        scan_screen <= ~scan_screen;
                    end
                    else
                        scan_page <= scan_page + 3'd1;
                    state <= DPU_PICK;
                end
            end
            default: state <= DPU_IDLE;
            endcase
        end
    end

    // ---------------- 寄存器写 ----------------
    // 状态位的写清条件（W1C）：先声明，写清与引擎置位在同一个 always 里合流
    reg done_clear;
    reg error_clear;
    always @(*) begin
        done_clear = bus_we_in && sel_reg && bus_be_in[0] &&
                     (reg_sel == R_STATUS) && bus_data_in[1];
        error_clear = bus_we_in && sel_reg && bus_be_in[0] &&
                      (reg_sel == R_STATUS) && bus_data_in[2];
    end

    always @(posedge clk) begin
        if (rst) begin
            start_request      <= 1'b0;
            abort_request      <= 1'b0;
            full_request       <= 1'b0;
            clear_request      <= 1'b0;
            status_done        <= 1'b0;
            status_error       <= 1'b0;
            irq_done_enable    <= 1'b0;
            irq_error_enable   <= 1'b0;
            slave_address0     <= 7'h3c;
            slave_address1     <= 7'h3d;
        end
        else begin
            start_request <= 1'b0;
            abort_request <= 1'b0;
            full_request  <= 1'b0;
            clear_request <= 1'b0;

            // 状态位：引擎置、总线写清（同拍写清赢）
            status_done  <= (status_done  | done_set)  & ~done_clear;
            status_error <= (status_error | error_set) & ~error_clear;

            if (bus_we_in && sel_reg) begin
                case (reg_sel)
                R_CTRL: if (bus_be_in[0]) begin
                            if (bus_data_in[0]) begin
                                start_request <= 1'b1;
                            end
                            if (bus_data_in[2]) begin
                                start_request <= 1'b1;
                                full_request <= 1'b1;
                            end
                            if (bus_data_in[1])
                                abort_request <= 1'b1;
                            if (bus_data_in[3])
                                clear_request <= 1'b1;
                        end
                R_IRQ:  if (bus_be_in[0]) begin
                            irq_done_enable  <= bus_data_in[0];
                            irq_error_enable <= bus_data_in[1];
                        end
                R_ADDR0: if (bus_be_in[0]) slave_address0 <= bus_data_in[6:0];
                R_ADDR1: if (bus_be_in[0]) slave_address1 <= bus_data_in[6:0];
                default: ;
                endcase
            end
        end
    end

    // 脏页：引擎清（一页刷完）+ 软件整片清；写帧缓冲置位，置位赢
    reg [7:0] clear_mask0;
    reg [7:0] clear_mask1;
    always @(*) begin
        clear_mask0 = (clear_request || (dirty_clear && !dirty_clear_screen))
                      ? (clear_request ? 8'hff : (8'd1 << dirty_clear_page)) : 8'd0;
        clear_mask1 = (clear_request || (dirty_clear &&  dirty_clear_screen))
                      ? (clear_request ? 8'hff : (8'd1 << dirty_clear_page)) : 8'd0;
    end

    always @(posedge clk) begin
        if (rst) begin
            dirty0 <= 8'd0;
            dirty1 <= 8'd0;
        end
        else begin
            dirty0 <= (dirty0 & ~clear_mask0) | dirty0_set;
            dirty1 <= (dirty1 & ~clear_mask1) | dirty1_set;
        end
    end

    // ---------------- 寄存器读 ----------------
    reg [31:0] read_data;
    always @(*) begin
        case (reg_sel)
        R_STATUS: read_data = {8'd0, dirty1, dirty0, 5'd0, status_error, status_done, frame_busy};
        R_IRQ:    read_data = {30'd0, irq_error_enable, irq_done_enable};
        R_ADDR0:  read_data = {25'd0, slave_address0};
        R_ADDR1:  read_data = {25'd0, slave_address1};
        default:  read_data = 32'd0;
        // CTRL 只写；帧缓冲只写不读
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            bus_data_out <= 32'd0;
            ld_ready     <= 1'b0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready     <= 1'b0;
            if (bus_addr_in[31:24] == 8'd9 && !bus_we_in) begin
                bus_data_out <= read_data;
                ld_ready     <= 1'b1;
            end
        end
    end

    // 中断是**电平跟着状态位走**（PLIC 每拍从电平重置 pending，做成粘滞位会打风暴）
    always @(*) begin
        dpu_irq = (status_done  && irq_done_enable) | (status_error && irq_error_enable);
    end

endmodule
