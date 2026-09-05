`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 19:04:53
// Design Name:
// Module Name: controller
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


module controller(
    input clk, rst, jalr_fail, br2, br3, irq_ret, trap, ebreak, stall,
    input jal, jalr_pred, br1,
    input csr_wr_en, exti, timi, softi,
    input retire,
    input [11:0] csr_addr,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
    output reg [31:0] csr_data_out, isr_addr1, isr_addr2, mcause,
    output reg irq_act, irq_processing, irq,
    output reg [31:0] iret_addr1, iret_addr2,
    output reg [1:0] stage,
    output reg req_valid,
    output reg [3:0] irq_bubble
    );

    reg [1:0] ird_tmr;
    wire [31:0] csr_data_out_i, isr_addr1_i, isr_addr2_i, mcause_i;
    wire irq_act_i, irq_processing_i;
    wire [31:0] iret_addr1_i, iret_addr2_i;

    localparam [1:0]
    IDLE = 2'd0,
    EXE = 2'd1,
    FLUSH = 2'd2,
    STALL = 2'd3;

    csr u_csr (
        .clk(clk),
        .rst(rst),
        .csr_wr_en(csr_wr_en),
        .iret(irq_ret),
        .exti(exti),
        .timi(timi),
        .softi(softi),
        .trap(trap),
        .ebreak(ebreak),
        .csr_addr(csr_addr),
        .csr_data_in(csr_data_in),
        .pc_addr_in(pc_addr_in),
        .retire(retire),
        .irq_bubble(irq_bubble),
        .irq_gate(ird_tmr != 2'd0),
        .jalr_fail(jalr_fail),
        .br2(br2),
        .br3(br3),
        .csr_data_out(csr_data_out_i),
        .isr_addr1(isr_addr1_i),
        .isr_addr2(isr_addr2_i),
        .mcause(mcause_i),
        .irq_act(irq_act_i),
        .irq_processing(irq_processing_i),
        .iret_addr1(iret_addr1_i),
        .iret_addr2(iret_addr2_i)
    );

    always @(*) begin
        csr_data_out = csr_data_out_i;
        isr_addr1 = isr_addr1_i;
        isr_addr2 = isr_addr2_i;
        mcause = mcause_i;
        irq_act = irq_act_i;
        irq_processing = irq_processing_i;
        irq = irq_act_i;
        iret_addr1 = iret_addr1_i;
        iret_addr2 = iret_addr2_i;
    end

    always @(posedge clk) begin
        if (rst) irq_bubble <= 4'd12;
        else if (stage == FLUSH) irq_bubble <= 4'd4;
        else if (irq_bubble < 4'd12) irq_bubble <= irq_bubble + 4'd4;
    end

    always @(posedge clk) begin
        if (rst) ird_tmr <= 2'd0;
        else if (jal | jalr_pred | br1) ird_tmr <= 2'd3;
        else if (ird_tmr != 2'd0) ird_tmr <= ird_tmr - 2'd1;
    end

    always @(*) begin
        if (rst) begin
            stage = EXE;
            req_valid = 1'b0;
        end
        else begin
            if (irq || irq_ret || irq_act || trap || jalr_fail || br2 || br3) stage = FLUSH;
            else if (stall) stage = STALL;
            else stage = EXE;
            req_valid = (stage != 2'd3);
        end
    end
endmodule
