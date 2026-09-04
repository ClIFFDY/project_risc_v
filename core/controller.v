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
    input csr_wr_en, exti, timi, softi,
    input retire,
    input [11:0] csr_addr,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
    output [31:0] csr_data_out, isr_addr1, isr_addr2, mcause,
    output irq_act, irq_processing, irq,
    output [31:0] iret_addr1, iret_addr2,
    output reg [1:0] stage,
    output reg req_valid,
    output reg [3:0] irq_bubble
    );

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
        .csr_data_out(csr_data_out),
        .isr_addr1(isr_addr1),
        .isr_addr2(isr_addr2),
        .mcause(mcause),
        .irq_act(irq_act),
        .irq_processing(irq_processing),
        .iret_addr1(iret_addr1),
        .iret_addr2(iret_addr2)
    );

    assign irq = (!(irq_ret | trap | jalr_fail | br2 | br3) && irq_act);

    always @(posedge clk) begin
        if (rst) irq_bubble <= 4'd0;
        else if (!irq_act && !irq_processing) begin
            case (stage)
            FLUSH: irq_bubble <= 4'd8;
            default: begin 
                if (irq_act != 4'd0) irq_bubble <= irq_bubble - 4'd4;
            end
            endcase
        end
        else begin
            irq_bubble <= irq_bubble;
        end
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
