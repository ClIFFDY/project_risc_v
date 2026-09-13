`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/12 20:17:54
// Design Name:
// Module Name: flash_spi
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module flash_spi(
    input clk, rst,

    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0]  bus_be_in,
    input        bus_we_in,
    output reg [31:0] bus_data_out,
    output reg        ld_ready,

    input        icache_mem_req,
    input        icache_mem_we,
    input [31:0] icache_mem_addr,
    input [31:0] icache_mem_wdata,
    input [3:0]  icache_mem_be,
    output reg [31:0] icache_mem_data,
    output reg        icache_mem_valid,
    output reg        icache_mem_ready,

    input        dcache_mem_req,
    input        dcache_mem_we,
    input [31:0] dcache_mem_addr,
    input [31:0] dcache_mem_wdata,
    input [3:0]  dcache_mem_be,
    output reg [31:0] dcache_mem_data,
    output reg        dcache_mem_valid,
    output reg        dcache_mem_ready,

    output reg spi_cs_n, spi_sck, spi_mosi,
    input      spi_miso,
    output reg flash_spi_irq
    );

    localparam [7:0] CMD_READ  = 8'h03;
    localparam [7:0] CMD_PROG  = 8'h02;
    localparam [7:0] CMD_WREN  = 8'h06;
    localparam [7:0] CMD_RDSR  = 8'h05;
    localparam [7:0] CMD_ERASE = 8'hD8;

    localparam [5:0]
    R_ERASE   = 6'h00,
    R_STATUS  = 6'h04,
    R_CONFIG  = 6'h08,
    R_TIMEOUT = 6'h0c,
    R_IRQ     = 6'h10;

    localparam [2:0] DEPTH = 3'd4;

    localparam [23:0] ICACHE_REGION = 24'h008000;
    localparam [23:0] DCACHE_REGION = 24'h000000;
    localparam [23:0] ICACHE_BASE   = 24'h000000;
    localparam [23:0] DCACHE_BASE   = 24'h400000;

    localparam [2:0]
    S_IDLE = 3'd0, S_CMD = 3'd1, S_ADDR = 3'd2, S_DATA = 3'd3, S_END = 3'd4,
    S_GAP  = 3'd5;

    localparam [2:0]
    X_READ = 3'd0, X_PROG = 3'd1, X_ERASE = 3'd2, X_POLL = 3'd3;

    localparam [1:0] P_WREN = 2'd0, P_MAIN = 2'd1;

    reg [59:0] wr_fifo [0:3];
    reg [2:0]  wr_wp, wr_rp;
    reg        wr_full, wr_empty, rd_pick, wr_pick, poll_pick, era_pick;

    reg [2:0]  state, xact;
    reg [1:0]  pstep, wphase, div_cnt, spi_div;
    reg        sck_r;
    reg        sel_ic;
    reg        prog_busy, era_busy, era_pend, flash_wip;
    reg [23:0] era_addr;
    reg [25:0] tout_reg, tout_cnt;
    reg        irq_err;
    reg [7:0]  poll_dly;
    reg [5:0]  reg_sel;
    reg [2:0]  bit_cnt, addr_cnt, byte_idx, w_len, w_first, st_bit;
    reg [5:0]  word_bit;
    reg [7:0]  tx_sh, st_sh, status_rx;
    reg [31:0] rx_sh;
    reg [23:0] a_reg, ic_flash_addr, dc_flash_addr, pick_addr;
    reg [31:0] wd_reg;
    reg [3:0]  be_reg;
    reg [31:0] rd_word, rd_data;
    reg        rd_valid;

    reg [31:0] wsh;
    reg [7:0]  wby0, wby1, wby2, wby3, tx_next, sa_hi, sa_mid, sa_lo;
    reg [23:0] sa_reg;
    reg        master_req, fsm_busy, wr_xact, flash_op_busy;

    wire sck_tick = (div_cnt == spi_div);
    wire sck_rise = sck_tick && !sck_r;
    wire sck_fall = sck_tick &&  sck_r;

    wire [23:0] fifo_flash_addr = wr_fifo[wr_rp[1:0]][59:36];
    wire [31:0] fifo_wdata      = wr_fifo[wr_rp[1:0]][35:4];
    wire [3:0]  fifo_be         = wr_fifo[wr_rp[1:0]][3:0];

    wire wr_push = dcache_mem_req && dcache_mem_we && !wr_full;
    wire rd_req  = (icache_mem_req && !icache_mem_we) || (dcache_mem_req && !dcache_mem_we);

    always @(*) begin
        wr_empty   = (wr_wp == wr_rp);
        wr_full    = (wr_wp[1:0] == wr_rp[1:0]) && (wr_wp[2] != wr_rp[2]);
        master_req = sel_ic ? icache_mem_req : dcache_mem_req;
        flash_op_busy = prog_busy || era_busy;
        fsm_busy   = (state != S_IDLE) || flash_op_busy || era_pend;
        flash_spi_irq = irq_err;

        rd_pick   = (state == S_IDLE) && rd_req;
        poll_pick = (state == S_IDLE) && !rd_pick && (prog_busy || era_busy) &&
                    (poll_dly == 8'd0);
        era_pick  = (state == S_IDLE) && !rd_pick && !poll_pick && era_pend &&
                    !prog_busy && !era_busy;
        wr_pick   = (state == S_IDLE) && !rd_pick && !poll_pick && !era_pick &&
                    !wr_empty && !prog_busy && !era_busy;

        w_first = 3'd3;
        if (be_reg[0]) w_first = 3'd0;
        else if (be_reg[1]) w_first = 3'd1;
        else if (be_reg[2]) w_first = 3'd2;

        w_len = 3'd4;
        if (!be_reg[3]) begin
            if (be_reg[2])      w_len = 3'd3;
            else if (be_reg[1]) w_len = 3'd2;
            else                w_len = 3'd1;
        end

        wsh = wd_reg >> (w_first * 8);
        wby0 = wsh[7:0];
        wby1 = wsh[15:8];
        wby2 = wsh[23:16];
        wby3 = wsh[31:24];

        case (byte_idx + 3'd1)
        3'd0:    tx_next = wby0;
        3'd1:    tx_next = wby1;
        3'd2:    tx_next = wby2;
        default: tx_next = wby3;
        endcase

        ic_flash_addr = icache_mem_addr[23:0] - ICACHE_REGION + ICACHE_BASE;
        dc_flash_addr = dcache_mem_addr[23:0] - DCACHE_REGION + DCACHE_BASE;
        pick_addr = (icache_mem_req && !icache_mem_we) ? ic_flash_addr : dc_flash_addr;

        sa_reg = (xact == X_READ) ? a_reg :
                 (xact == X_ERASE) ? era_addr : (a_reg + w_first);
        sa_hi  = sa_reg[23:16];
        sa_mid = sa_reg[15:8];
        sa_lo  = sa_reg[7:0];

        wr_xact = (xact != X_READ);

        dcache_mem_ready = !rst && !wr_full;
        icache_mem_ready = !rst;

        icache_mem_data  = (rd_valid &&  sel_ic) ? rd_data : 32'd0;
        dcache_mem_data  = (rd_valid && !sel_ic) ? rd_data : 32'd0;
        icache_mem_valid = rd_valid &&  sel_ic;
        dcache_mem_valid = rd_valid && !sel_ic;

        spi_sck  = sck_r;
        spi_mosi = tx_sh[7];
    end

    always @(posedge clk) begin
        if (rst) begin
            div_cnt <= 2'd0;
            sck_r <= 1'b0;
        end
        else if (spi_cs_n) begin
            div_cnt <= 2'd0;
            sck_r <= 1'b0;
        end
        else if (sck_tick) begin
            div_cnt <= 2'd0;
            sck_r <= ~sck_r;
        end
        else div_cnt <= div_cnt + 2'd1;
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_wp <= 3'd0;
            wr_rp <= 3'd0;
        end
        else begin
            if (wr_push) begin
                wr_fifo[wr_wp[1:0]] <= {dc_flash_addr, dcache_mem_wdata, dcache_mem_be};
                wr_wp <= wr_wp + 3'd1;
            end
            if (wr_pick) wr_rp <= wr_rp + 3'd1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            era_pend <= 1'b0;
            era_addr <= 24'd0;
            spi_div <= 2'd1;
            tout_reg <= 26'h3FF_FFFF;
            irq_err <= 1'b0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            reg_sel = bus_addr_in[5:0];
            if (bus_addr_in[31:24] == 8'd7) begin
                case (reg_sel)
                R_ERASE: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0] && !era_pend && !era_busy) begin
                            era_pend <= 1'b1;
                            era_addr <= (bus_data_in[23:0] - DCACHE_REGION + DCACHE_BASE) & 24'hFFF000;
                        end
                    end
                    else begin
                        bus_data_out <= {8'd0, era_addr};
                        ld_ready <= 1'b1;
                    end
                end
                R_STATUS: begin
                    if (!bus_we_in) begin
                        bus_data_out <= {25'd0, irq_err, wr_empty, wr_full, era_busy,
                                         prog_busy, flash_wip, fsm_busy};
                        ld_ready <= 1'b1;
                    end
                end
                R_CONFIG: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0]) spi_div <= bus_data_in[1:0];
                    end
                    else begin
                        bus_data_out <= {30'd0, spi_div};
                        ld_ready <= 1'b1;
                    end
                end
                R_TIMEOUT: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0]) tout_reg[7:0]   <= bus_data_in[7:0];
                        if (bus_be_in[1]) tout_reg[15:8]  <= bus_data_in[15:8];
                        if (bus_be_in[2]) tout_reg[23:16] <= bus_data_in[23:16];
                        if (bus_be_in[3]) tout_reg[25:24] <= bus_data_in[25:24];
                    end
                    else begin
                        bus_data_out <= {6'd0, tout_reg};
                        ld_ready <= 1'b1;
                    end
                end
                R_IRQ: begin
                    if (bus_we_in) begin
                        if (bus_be_in[0] && bus_data_in[0]) irq_err <= 1'b0;
                    end
                    else begin
                        bus_data_out <= {31'd0, irq_err};
                        ld_ready <= 1'b1;
                    end
                end
                default: begin
                    if (!bus_we_in) ld_ready <= 1'b1;
                end
                endcase
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            spi_cs_n <= 1'b1;
            xact <= X_READ;
            pstep <= P_WREN;
            wphase <= P_WREN;
            sel_ic <= 1'b1;
            prog_busy <= 1'b0;
            era_busy <= 1'b0;
            flash_wip <= 1'b0;
            poll_dly <= 8'd0;
            bit_cnt <= 3'd0;
            addr_cnt <= 3'd0;
            byte_idx <= 3'd0;
            st_bit <= 3'd0;
            word_bit <= 6'd0;
            tx_sh <= 8'hFF;
            st_sh <= 8'd0;
            status_rx <= 8'hFF;
            rx_sh <= 32'd0;
            a_reg <= 24'd0;
            wd_reg <= 32'd0;
            be_reg <= 4'd0;
            rd_word <= 32'd0;
            rd_data <= 32'd0;
            rd_valid <= 1'b0;
        end
        else begin
            rd_valid <= 1'b0;
            if (poll_dly != 8'd0) poll_dly <= poll_dly - 8'd1;

            if (flash_op_busy && tout_reg != 26'd0) begin
                if (tout_cnt == 26'd0) begin
                    prog_busy <= 1'b0;
                    era_busy  <= 1'b0;
                    era_pend  <= 1'b0;
                    irq_err   <= 1'b1;
                end
                else tout_cnt <= tout_cnt - 26'd1;
            end
            else tout_cnt <= tout_reg;

            if (sck_rise && state == S_DATA && xact == X_READ) begin
                rx_sh <= {rx_sh[30:0], spi_miso};
                if (word_bit == 6'd31) begin
                    word_bit <= 6'd0;
                    rd_word <= 32'd0;
                    rd_data <= {rx_sh[6:0], spi_miso, rd_word[31:8]};
                    rd_valid <= 1'b1;
                end
                else begin
                    word_bit <= word_bit + 6'd1;
                    if (word_bit[2:0] == 3'd7)
                        rd_word <= {rx_sh[6:0], spi_miso, rd_word[31:8]};
                end
            end

            if (sck_rise && state == S_DATA && xact == X_POLL) begin
                st_sh <= {st_sh[6:0], spi_miso};
                if (st_bit == 3'd7) status_rx <= {st_sh[6:0], spi_miso};
                else                st_bit <= st_bit + 3'd1;
            end

            if (sck_fall) begin
                if (bit_cnt == 3'd7) begin
                    bit_cnt <= 3'd0;
                    case (state)
                    S_CMD: begin
                        addr_cnt <= 3'd0;
                        if (xact == X_READ) begin
                            tx_sh <= sa_hi;
                            state <= S_ADDR;
                        end
                        else if (pstep == P_WREN) begin
                            state <= S_END;
                        end
                        else if (xact == X_POLL) begin
                            st_bit <= 3'd0;
                            state <= S_DATA;
                        end
                        else begin
                            tx_sh <= sa_hi;
                            state <= S_ADDR;
                        end
                    end
                    S_ADDR: begin
                        if (addr_cnt == 3'd2) begin
                            addr_cnt <= 3'd0;
                            byte_idx <= 3'd0;
                            word_bit <= 6'd0;
                            if (xact == X_ERASE) state <= S_END;
                            else begin
                                tx_sh <= wby0;
                                state <= S_DATA;
                            end
                        end
                        else begin
                            addr_cnt <= addr_cnt + 3'd1;
                            if (addr_cnt == 3'd0) tx_sh <= sa_mid;
                            else                  tx_sh <= sa_lo;
                        end
                    end
                    S_DATA: begin
                        if (xact == X_READ) begin
                            if (!master_req) state <= S_END;
                            else tx_sh <= 8'hFF;
                        end
                        else if (xact == X_POLL) state <= S_END;
                        else if (byte_idx + 3'd1 >= w_len) state <= S_END;
                        else begin
                            byte_idx <= byte_idx + 3'd1;
                            tx_sh <= tx_next;
                        end
                    end
                    default: ;
                    endcase
                end
                else begin
                    bit_cnt <= bit_cnt + 3'd1;
                    tx_sh <= {tx_sh[6:0], 1'b1};
                end
            end

            case (state)
            S_IDLE: begin
                spi_cs_n <= 1'b1;
                bit_cnt <= 3'd0;
                addr_cnt <= 3'd0;
                byte_idx <= 3'd0;
                word_bit <= 6'd0;
                if (rd_pick) begin
                    xact <= X_READ;
                    pstep <= P_WREN;
                    sel_ic <= (icache_mem_req && !icache_mem_we);
                    a_reg <= pick_addr;
                    tx_sh <= CMD_READ;
                    spi_cs_n <= 1'b0;
                    state <= S_CMD;
                end
                else if (poll_pick) begin
                    xact <= X_POLL;
                    pstep <= P_MAIN;
                    tx_sh <= CMD_RDSR;
                    spi_cs_n <= 1'b0;
                    state <= S_CMD;
                end
                else if (era_pick) begin
                    xact <= X_ERASE;
                    pstep <= P_WREN;
                    sel_ic <= 1'b0;
                    tx_sh <= CMD_WREN;
                    spi_cs_n <= 1'b0;
                    state <= S_CMD;
                end
                else if (wr_pick) begin
                    xact <= X_PROG;
                    pstep <= P_WREN;
                    sel_ic <= 1'b0;
                    a_reg <= fifo_flash_addr;
                    wd_reg <= fifo_wdata;
                    be_reg <= fifo_be;
                    tx_sh <= CMD_WREN;
                    spi_cs_n <= 1'b0;
                    state <= S_CMD;
                end
            end
            S_CMD: ;
            S_ADDR: ;
            S_DATA: ;
            S_GAP: begin
                spi_cs_n <= 1'b0;
                state <= S_CMD;
            end
            S_END: begin
                spi_cs_n <= 1'b1;
                bit_cnt <= 3'd0;
                if (xact == X_READ) state <= S_IDLE;
                else if (pstep == P_WREN) begin
                    pstep <= P_MAIN;
                    tx_sh <= (xact == X_ERASE) ? CMD_ERASE : CMD_PROG;
                    addr_cnt <= 3'd0;
                    state <= S_GAP;
                end
                else if (xact == X_POLL) begin
                    flash_wip <= status_rx[0];
                    if (status_rx[0]) begin
                        tx_sh <= CMD_RDSR;
                        st_bit <= 3'd0;
                        state <= S_GAP;
                    end
                    else begin
                        prog_busy <= 1'b0;
                        era_busy <= 1'b0;
                        state <= S_IDLE;
                    end
                end
                else begin
                    flash_wip <= 1'b1;
                    poll_dly <= 8'hFF;
                    tout_cnt <= tout_reg;
                    if (xact == X_ERASE) begin
                        era_busy <= 1'b1;
                        era_pend <= 1'b0;
                    end
                    else prog_busy <= 1'b1;
                    state <= S_IDLE;
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
