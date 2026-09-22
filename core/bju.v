`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/22
// Design Name:
// Module Name: bju
// Project Name:
// Target Devices:
// Tool Versions:
// Description: 分支/跳转判定单元（branch-jump unit）。
//              判定源全部来自 decoder 本级的寄存器输出（与 alu 的输入同源），
//              比较与加法落在这一拍，结果、落点、以及预测表回写要用的限定信号
//              在下一拍一并生效；另给出一条组合版的前置冲刷 stallf，只有 lsu/mulu 消费。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module bju(
    input clk, rst,
    input [4:0] flag_bus,
//判定源：与 alu 的输入同源（decoder 本级的寄存器输出）
    input [31:0] r1_data_in, r2_data_in,
    input [3:0] alu_func4_in,
    input br_flag_in, jalr_flag_in,
    input [31:0] aux_addr_in, beq_off_in, jalr_pred_addr_in,
    input br_pred_taken_in,
//判定结果与跳转落点（寄存一拍后有效）
    output reg success, br_fail, jalr_fail,
    output reg [31:0] jp_target, jalr_target_q2,
    output reg [5:0] br_pc_idx,
    output reg jalr_flag_q, br_pred_taken_q,
//前置冲刷：同一判定的组合版本，早一拍，只喂 lsu/mulu
    output reg stallf
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：判定延迟拍只关心"本拍是不是冲刷拍"。
    reg flush_w;
    always @(*) flush_w = flag_bus[3] | flag_bus[2];

//判定组合逻辑用的寄存器
    reg success_now, br_fail_now, br2_now, br3_now, jalr_fail_now;
    reg [31:0] jalr_target_now, jp_target_now;

//跳转判定（组合）：br 的两个操作数与 jalr 的基址/偏移都由 decoder 的寄存器给出
//（r1_data_in/r2_data_in 就是 alu 的输入），故比较与加法落在这一拍，结果在下一拍生效。
//原设计把这两件事串在【decoder 进本级那一拍】的 D 端上，而那条 D 端的入口是 forw 的输出
//（经 csr 读口 → alu 直通），是全片最差路径；搬到这里后入口变成触发器，链头整段消失。
    always @(*) begin
        success_now = 1'b0;
        br_fail_now = 1'b0;
        jalr_fail_now = 1'b0;
        jalr_target_now = 32'd0;
        if (br_flag_in) begin
            case (alu_func4_in[2:0])
                3'b000: begin
                    if (r1_data_in == r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b001: begin
                    if (r1_data_in != r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b100: begin
                    if ($signed(r1_data_in) < $signed(r2_data_in)) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b101: begin
                    if ($signed(r1_data_in) >= $signed(r2_data_in)) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b110: begin
                    if (r1_data_in < r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b111: begin
                    if (r1_data_in >= r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                default: br_fail_now = 1'b0;
            endcase
        end
        else if (jalr_flag_in) begin
            jalr_target_now = r1_data_in + r2_data_in;
            if (jalr_target_now != 32'd0) begin
                if (jalr_target_now != jalr_pred_addr_in) jalr_fail_now = 1'b1;
            end
        end
    end

//落点与前置冲刷：br2/br3 沿用原定义（成功且预测不跳 / 失败且预测跳）。
//落点按"分支自身地址 + 4"的口径就地算清：beq_off_in 已含 -4、aux_addr_in 已含 +4，
//两者相加正好是 B+immB；br3 的落点就是 aux_addr_in（落空的后继）。
//stallf 是同一判定的组合版本、早一拍：判定结果寄存后只能覆盖 c1..c4 与 wb，而错路指令
//在 c2 上会停留两拍（前一条落前置拍、后一条落寄存拍），那两拍里它已经会去动 FIFO 指针、
//拉总线、推乘法流水，等寄存器清已经收不回来 —— lsu/mulu 因此在入口多挡一拍。
    always @(*) begin
        br2_now = success_now & ~br_pred_taken_in;
        br3_now = br_fail_now & br_pred_taken_in;
        stallf = br2_now | br3_now | jalr_fail_now;
        if (br2_now) jp_target_now = aux_addr_in + beq_off_in;
        else if (br3_now) jp_target_now = aux_addr_in;
        else if (jalr_fail_now) jp_target_now = jalr_target_now;
        else jp_target_now = 32'd0;
    end

//判定延迟拍：判定结果、跳转落点、以及预测表回写要用的索引与限定信号统一寄存一拍。
//冲刷拍必须清（否则同一笔判定会连拉两拍）；停顿拍不清：操作数被冻住时判定值不变，
//与旧设计把 success/br_fail 放在 decoder 主块里自保持的行为一致。
    always @(posedge clk) begin
        if (rst_q) begin
            success <= 1'b0;
            br_fail <= 1'b0;
            jalr_fail <= 1'b0;
            jp_target <= 32'd0;
            jalr_target_q2 <= 32'd0;
            br_pc_idx <= 6'd0;
            jalr_flag_q <= 1'b0;
            br_pred_taken_q <= 1'b0;
        end
        else if (flush_w) begin
            success <= 1'b0;
            br_fail <= 1'b0;
            jalr_fail <= 1'b0;
            jp_target <= 32'd0;
            jalr_target_q2 <= 32'd0;
            br_pc_idx <= 6'd0;
            jalr_flag_q <= 1'b0;
            br_pred_taken_q <= 1'b0;
        end
        else begin
            success <= success_now;
            br_fail <= br_fail_now;
            jalr_fail <= jalr_fail_now;
            jp_target <= jp_target_now;
            jalr_target_q2 <= jalr_target_now;
            br_pc_idx <= aux_addr_in[8:3];
            jalr_flag_q <= jalr_flag_in;
            br_pred_taken_q <= br_pred_taken_in;
        end
    end
endmodule
