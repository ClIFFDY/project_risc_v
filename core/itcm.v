`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/28 15:28:51
// Design Name:
// Module Name: itcm
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


module itcm(
    input clk, rst,
    input br1, br2, br3,
    input jal, jalr, jalr_fail, irq, irq_ret,
    input [1:0] stage,
    input [31:0] pc_addr,
    input [31:0] offset_jal1, offset_jalr1,
    input [31:0] offset_beq1, isr_addr1, isr_ret_addr1,
    input [31:0] jalr_target_q, beq_off_q2,
    input [31:0] br_addr1,
    output reg [31:0] inst_raw_out,
    output reg is_ibus_q,
    //
    output reg [31:0] fetch_addr,
    output reg ibus_re_out,
    input [15:0] ibus_addr_in,
    input [31:0] ibus_data_in,
    input ibus_we_in
    );

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

//指令TCM紧耦合内存，1clk读延迟，1clk写延迟，无命中判定
    (* ram_style = "block" *) reg [31:0] itcm [0:4095];
    initial begin
        $readmemh("e:/Vivado_Projects/project_risc_v/tools/hex/ins.hex", itcm);
    end

//pc地址itcm/icache仲裁
    reg is_ibus;
    always @(*) is_ibus = |fetch_addr[31:12];

//读写逻辑处理（写逻辑目前无意义）
    always @(posedge clk) begin
        if (rst) begin
            inst_raw_out <= 32'd0;
            is_ibus_q <= 1'd0;
        end
        else if (stage == IDLE && ibus_we_in) begin
            itcm[ibus_addr_in[13:2]] <= ibus_data_in;
        end
        else if (stage == STALL) begin
            inst_raw_out <= inst_raw_out;
            is_ibus_q <= is_ibus_q;
        end
        else begin
            inst_raw_out <= itcm[fetch_addr[11:0]];
            is_ibus_q <= is_ibus;
        end
    end

//跳转类地址透传处理，减少取值冲刷空窗
    always @(*) begin
        if (rst) begin
            fetch_addr = 32'd0;
            ibus_re_out = 1'd0;
        end
        else begin
            ibus_re_out = is_ibus;
            fetch_addr = 32'd0;
            case (stage)
            EXE, STALL: begin
                if (br1) fetch_addr = (pc_addr + offset_beq1) >>> 2;
                else if (jal) fetch_addr = (pc_addr + offset_jal1) >>> 2;
                else if (jalr) fetch_addr = offset_jalr1 >>> 2;
                else fetch_addr = pc_addr >>> 2;
            end
            FLUSH: begin
                if (br2) fetch_addr = (br_addr1 + beq_off_q2) >>> 2;
                else if (br3) fetch_addr = br_addr1 >>> 2;
                else if (jalr_fail) fetch_addr = jalr_target_q >>> 2;
                else fetch_addr = pc_addr >>> 2;
            end
            default: fetch_addr = 16'd0;
            endcase
            if (stage == FLUSH && irq) fetch_addr = isr_addr1 >>> 2;
            else if (stage == FLUSH && irq_ret) fetch_addr = isr_ret_addr1 >>> 2;
        end
    end
endmodule
