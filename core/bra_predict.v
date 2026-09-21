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
    input [31:0] pc_addr_in, jalr_target_q, br_pc_in, jalr_pred_addr_in,
    input success, br_fail, br_en, jalr_flag, br_pred_taken_in,
    output reg [31:0] br_addr1, br_addr2,
    output reg [31:0] jalr_predict_offset,
    output reg br1, br2, br3, jalr, jalr_fail
    );

    reg [1:0] bht [0:63];
    reg [31:0] btb [0:63];
//BTB 有效表：把命中判定从"目标 != 0"（32 位或归约）换成 1 位查表。
//写口与 btb 同拍按 (jalr_target_q != 0) 更新，复位和非命中项都是 0，
//所以 btb_v[i] 恒等于原来的 (btb[i] != 0)，逐位等价。
    reg btb_v [0:63];
    reg predict_en;
    integer i;

//BHT查当前取指PC，饱和计数>1则预测跳转
    always @(*) begin
        if (rst) predict_en = 1'b0;
        else predict_en = (bht[pc_addr_in[8:3]] > 2'd1);
    end

    always @(posedge clk) begin
        if (rst) begin
//初始BHT回到弱不跳转
            for (i = 0; i < 64; i = i + 1) bht[i] <= 2'd1;
            for (i = 0; i < 64; i = i + 1) btb[i] <= 32'd0;
            for (i = 0; i < 64; i = i + 1) btb_v[i] <= 1'b0;
        end
        else begin
//分支在EX期判定：按随指令流水的载荷PC回写BHT
            if (success) begin
                if (bht[br_pc_in[8:3]] < 2'd3)
                    bht[br_pc_in[8:3]] <= bht[br_pc_in[8:3]] + 2'd1;
            end
            else if (br_fail) begin
                if (bht[br_pc_in[8:3]] > 2'd0)
                    bht[br_pc_in[8:3]] <= bht[br_pc_in[8:3]] - 2'd1;
            end
//jalr实际目标在EX期解析：按随指令流水的载荷PC回写BTB
            if (jalr_flag) begin
                btb[br_pc_in[8:3]] <= jalr_target_q;
                btb_v[br_pc_in[8:3]] <= (jalr_target_q != 32'd0);
            end
        end
    end

//组合输出：当前取指PC的预测结果 + 载荷的预测判定
    always @(*) begin
        if (rst) begin
            br_addr1 = 32'd0;
            br_addr2 = 32'd0;
            br1 = 1'b0;
            br2 = 1'b0;
            br3 = 1'b0;
            jalr = 1'b0;
            jalr_fail = 1'b0;
            jalr_predict_offset = 32'd0;
        end
        else begin
            br_addr1 = br_pc_in;
            br_addr2 = br_pc_in + 4'd4;
            br1 = br_en & predict_en;
            br2 = success & !br_pred_taken_in;
            br3 = br_fail & br_pred_taken_in;
            jalr_predict_offset = btb[pc_addr_in[8:3]];
            jalr = btb_v[pc_addr_in[8:3]];
            jalr_fail = (jalr_target_q != 32'd0) & (jalr_target_q != jalr_pred_addr_in);
        end
    end

endmodule
