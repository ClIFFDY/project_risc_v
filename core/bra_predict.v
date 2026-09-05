`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/29 20:12:36
// Design Name:
// Module Name: bra_predict
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


module bra_predict(
    input clk, rst,
    input [1:0] stage,
    input [31:0] pc_addr_in,
    input [31:0] jalr_target_q,
    input success, br_fail, br_en, pre_jalr,
    output reg [31:0] br_addr1, br_addr2,
    output reg [31:0] jalr_predict_offset,
    output reg br1, br2, br3, jalr, jalr_fail
    );

    reg [31:0] jalr_real_target;
    always @(*) jalr_real_target = jalr_target_q;

    reg [1:0] bht [0:63];
    reg [31:0] btb [0:63], br_pc_q [0:1], jalr_pc_q, jt_q [0:1];
    reg pv_q [0:1], wr_ptr, rd_ptr, jwp, jrp, predict_en;
    integer i;

    always @(*) begin
        if (rst) predict_en = 1'b0;
        else predict_en = (bht[pc_addr_in[8:3]] > 2'd1);
    end

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 64; i = i + 1) bht[i] <= 2'd1;
            for (i = 0; i < 64; i = i + 1) btb[i] <= 32'd0;
            jalr_pc_q <= 16'd0;
            jt_q[0] <= 32'd0;
            jt_q[1] <= 32'd0;
            br_pc_q[0] <= 16'd0;
            br_pc_q[1] <= 16'd0;
            pv_q[0] <= 1'b0;
            pv_q[1] <= 1'b0;
            wr_ptr <= 1'b0;
            rd_ptr <= 1'b0;
            jwp <= 1'b0;
            jrp <= 1'b0;
        end
        else begin
            jalr_pc_q <= pc_addr_in;
            if (jalr_real_target != 32'd0)
                btb[jalr_pc_q[8:3]] <= jalr_real_target;
            if (success) begin
                if (bht[br_pc_q[rd_ptr][8:3]] < 2'd3)
                    bht[br_pc_q[rd_ptr][8:3]] <= bht[br_pc_q[rd_ptr][8:3]] + 1'd1;
                rd_ptr <= ~rd_ptr;
            end
            else if (br_fail) begin
                if (bht[br_pc_q[rd_ptr][8:3]] > 2'd0)
                    bht[br_pc_q[rd_ptr][8:3]] <= bht[br_pc_q[rd_ptr][8:3]] - 1'd1;
                rd_ptr <= ~rd_ptr;
            end
            if (stage == 2'd2) begin
                wr_ptr <= 1'b0;
                rd_ptr <= 1'b0;
                jwp <= 1'b0;
                jrp <= 1'b0;
            end
            else begin
                if (br_en) begin
                    br_pc_q[wr_ptr] <= pc_addr_in;
                    pv_q[wr_ptr] <= predict_en;
                    wr_ptr <= ~wr_ptr;
                end
                if (pre_jalr) begin
                    jt_q[jwp] <= btb[pc_addr_in[8:3]];
                    jwp <= ~jwp;
                end
                if (jalr_real_target != 32'd0)
                    jrp <= ~jrp;
            end
        end
    end

    always @(*) begin
        if (rst) begin
            br_addr1 = 16'd0;
            br_addr2 = 16'd0;
            br1 = 1'b0;
            br2 = 1'b0;
            br3 = 1'b0;
            jalr = 1'b0;
            jalr_fail = 1'b0;
            jalr_predict_offset = 32'd0;
        end
        else begin
            br_addr1 = br_pc_q[rd_ptr];
            br_addr2 = br_pc_q[rd_ptr] + 4'd4;
            br1 = br_en & predict_en;
            br2 = success & !pv_q[rd_ptr];
            br3 = br_fail & pv_q[rd_ptr];
            jalr_predict_offset = btb[pc_addr_in[8:3]];
            jalr = (btb[pc_addr_in[8:3]] != 32'd0);
            jalr_fail = (jalr_real_target != 32'd0) & (jalr_real_target != jt_q[jrp]);
        end
    end

endmodule
