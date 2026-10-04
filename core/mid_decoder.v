`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/10/04
// Design Name:
// Module Name: mid_decoder
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   取指侧的中间寄存器级：把 pre_decoder 的输出整体重新寄存一拍。
//
//   ★ 与 862a90e 之前那版不同：那时本级还要【就地解一份】rd/func10/imm（因为 pre 是纯
//     寄存器级、不存译码结果）。现在 pre_decoder 已经解好了，本级只做纯寄存 —— 不重复一份译码。
//
//   ★ 为什么要有这一级：regfile 的读口地址推到本级（r1_out/r2_out），于是 rob 的槽扫描
//     可以吃 pre_decoder 的输出触发器（普通触发器），而不是 icache 的 BRAM 输出寄存器。
//     BRAM 输出寄存器的 clock-to-out（≈2.45ns）是扫描那一拍白吃的开销。
//
// Dependencies:
//
// Revision:
//   Revision 0.01 - 重新引入（纯寄存版）
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module mid_decoder(
    input clk, rst,
    input [11:0] flag_bus,
    input [31:0] inst_in,
    input [4:0]  rd_in,
    input [9:0]  func10_in,
    input [31:0] imm_alu_in,
    input [31:0] aux_addr_in,
    input        br_pred_taken_in,
    input [31:0] jalr_pred_addr_in,
    input [4:0]  r1_in, r2_in,
    output reg [31:0] inst_out,
    output reg [4:0]  rd_out,
    output reg [9:0]  func10_out,
    output reg [31:0] imm_alu_out,
    output reg [31:0] aux_addr_out,
    output reg        br_pred_taken_out,
    output reg [31:0] jalr_pred_addr_out,
    output reg [4:0]  r1_out, r2_out
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，工具只能在布局阶段自己复制。每个模块各自打一拍，
//寄存器就落在本模块旁边；全核都只打一拍，彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[11] | flag_bus[10] | flag_bus[9];
        stall_w = (flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4] | flag_bus[3]
                 | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//推进/保持/清：与 pre_decoder 的载荷锁存【逐字同形】。
//★ 条件必须与上一级一致：本级的"保持"要跟上一级的"保持"落在同一拍，差一拍就是整条流水线错位。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out <= 32'd0;
            rd_out <= 5'd0;
            func10_out <= 10'd0;
            imm_alu_out <= 32'd0;
            aux_addr_out <= 32'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
            r1_out <= 5'd0;
            r2_out <= 5'd0;
        end
        else if (exec) begin
            if (!flush_w && !stall_w) begin
                inst_out <= inst_in;
                rd_out <= rd_in;
                func10_out <= func10_in;
                imm_alu_out <= imm_alu_in;
                aux_addr_out <= aux_addr_in;
                br_pred_taken_out <= br_pred_taken_in;
                jalr_pred_addr_out <= jalr_pred_addr_in;
                r1_out <= r1_in;
                r2_out <= r2_in;
            end
            else if (stall_w) begin
                inst_out <= inst_out;
                rd_out <= rd_out;
                func10_out <= func10_out;
                imm_alu_out <= imm_alu_out;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
                r1_out <= r1_out;
                r2_out <= r2_out;
            end
            else begin
                inst_out <= 32'd0;
                rd_out <= 5'd0;
                func10_out <= 10'd0;
                imm_alu_out <= 32'd0;
                aux_addr_out <= 32'd0;
                br_pred_taken_out <= 1'b0;
                jalr_pred_addr_out <= 32'd0;
                r1_out <= 5'd0;
                r2_out <= 5'd0;
            end
        end
    end

endmodule
