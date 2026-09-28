`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/28
// Design Name:
// Module Name: wport —— 写口级（乱序写回：完成即写 + 同 rd 同拍年轻者胜）
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   三个结果口（alu / mulu / lsu）→ 寄存器堆的两条物理写口。**纯组合**：三个单元各自负责
//   自己那笔的时序（alu 在 alu.v 里寄存一拍、mulu 保持到 taken_mul、lsu 一拍脉冲），
//   本级只做三件事：
//
//   ① 选口：口 A = alu → mul → 空；口 B = ld →（mul，若 A 没占）→ 空。
//      流式源（alu 每拍一笔、ld 一拍脉冲）必须保住自己的口 —— 它们等不了；
//      被挤的只能是 mul（它真能保持）。⇒ 这个优先序不能改成"三源平等"。
//   ② 同 rd 撞两个口：只放行年轻那笔，老的那笔**当拍丢弃**（不是延后 —— 延后落地会把年轻的盖掉）。
//      年龄用 head 相对值 {1'b0,(idx - rob_head)}，先截 4 位再比（3 位减法回绕会判错）。
//   ③ 唯一广播点：只广播**真落地**的那笔 (rd, idx)，各单元据此杀自己更老的同 rd 在途记录。
//      被杀的笔（kill_mul/kill_ld）不写、不广播，但照常"放行 + 回报" —— 否则单元死等、队头也不动。
//
// Dependencies:
//
// Revision:
//   Revision 0.01 - 新建（写口级 + 寄存版 alu 结果）
//   Revision 0.02 - alu 那一笔的寄存器搬进 alu.v ⇒ 本级退化成纯组合；处置回报改三条通道
//                   （fin_alu/fin_mul/fin_ld，与 ROB 现有三个完成口同形，只是不再带数据）
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module wport(
//ROB 队头（年龄基准）
    input [2:0]  rob_head,
//alu 结果口（alu.v 内寄存一拍，一拍宽）
    input        we_alu,
    input [4:0]  rd_alu,
    input [31:0] data_alu,
    input [2:0]  idx_alu,
//mulu 结果口（保持到 taken_mul）
    input        we_mul,
    input [4:0]  rd_mul,
    input [31:0] data_mul,
    input [2:0]  idx_mul,
//lsu 结果口（一拍脉冲）
    input        we_ld,
    input [4:0]  rd_ld,
    input [31:0] data_ld,
    input [2:0]  idx_ld,
//已被判"永不落地"的笔：不写、不广播，但照常放行并回报
    input        kill_mul,
    input        kill_ld,
    input        kill_alu,
//写口（到寄存器堆）
    output reg        we_a,
    output reg [4:0]  rd_a,
    output reg [31:0] data_a,
    output reg        we_b,
    output reg [4:0]  rd_b,
    output reg [31:0] data_b,
//放行 mulu 的那一笔（可以撤值了）
    output reg        taken_mul,
//处置回报（给 ROB：落地或被丢弃都算这一项完了）
    output reg        fin_alu,
    output reg [2:0]  fin_alu_idx,
    output reg        fin_mul,
    output reg [2:0]  fin_mul_idx,
    output reg        fin_ld,
    output reg [2:0]  fin_ld_idx,
//落地广播（给 lsu/mulu 杀老写：只广播真落地的那笔）
    output reg        b0_we,
    output reg [4:0]  b0_rd,
    output reg [2:0]  b0_idx,
    output reg        b1_we,
    output reg [4:0]  b1_rd,
    output reg [2:0]  b1_idx
    );

//三个候选：有效性与年龄
    reg        c0_v, c1_v, c2_v;
    reg [3:0]  c0_age, c1_age, c2_age;
//端口归属与同 rd 压制
    reg [1:0]  pa, pb;
    reg [4:0]  pa_rd, pb_rd;
    reg [2:0]  pa_idx, pb_idx;
    reg [3:0]  pa_age, pb_age;
    reg        sup_a, sup_b;

//候选有效性：不写 rd=0 的那笔不占口；已判死的笔也不占口（但下面照常放行/回报）
    always @(*) begin
        c0_v   = we_alu && (rd_alu != 5'd0) && ~kill_alu;
        c1_v   = we_mul && (rd_mul != 5'd0) && ~kill_mul;
        c2_v   = we_ld && (rd_ld != 5'd0) && ~kill_ld;
        c0_age = {1'b0, (idx_alu - rob_head)};
        c1_age = {1'b0, (idx_mul - rob_head)};
        c2_age = {1'b0, (idx_ld - rob_head)};
    end

//端口归属：口 A 只给 alu/mul（ld 走 B），口 B 给 ld，mul 在 A 没占上时捡 B
    always @(*) begin
        pa     = 2'd3;
        pb     = 2'd3;
        pa_rd  = 5'd0;
        pb_rd  = 5'd0;
        pa_idx = 3'd0;
        pb_idx = 3'd0;
        pa_age = 4'd0;
        pb_age = 4'd0;
        if (c0_v) begin
            pa     = 2'd0;
            pa_rd  = rd_alu;
            pa_idx = idx_alu;
            pa_age = c0_age;
        end
        else if (c1_v) begin
            pa     = 2'd1;
            pa_rd  = rd_mul;
            pa_idx = idx_mul;
            pa_age = c1_age;
        end
        if (c2_v) begin
            pb     = 2'd2;
            pb_rd  = rd_ld;
            pb_idx = idx_ld;
            pb_age = c2_age;
        end
        else if (c1_v && (pa != 2'd1)) begin
            pb     = 2'd1;
            pb_rd  = rd_mul;
            pb_idx = idx_mul;
            pb_age = c1_age;
        end
//同 rd 撞两个口：只放行更年轻的那笔，老的那笔当拍丢弃（置 sup，写使能不发）
        sup_a = 1'b0;
        sup_b = 1'b0;
        if ((pa != 2'd3) && (pb != 2'd3) && (pa_rd == pb_rd)) begin
            if (pa_age > pb_age) begin
                sup_b = 1'b1;
            end
            else begin
                sup_a = 1'b1;
            end
        end
    end

//写口 / 放行 / 回报 / 广播
    always @(*) begin
        we_a      = 1'b0;
        rd_a      = 5'd0;
        data_a    = 32'd0;
        we_b      = 1'b0;
        rd_b      = 5'd0;
        data_b    = 32'd0;
        taken_mul = 1'b0;
        fin_alu   = we_alu;
        fin_alu_idx = idx_alu;
        fin_mul   = we_mul && (kill_mul || (rd_mul == 5'd0) || (pa == 2'd1) || (pb == 2'd1));
        fin_mul_idx = idx_mul;
        fin_ld    = we_ld;
        fin_ld_idx = idx_ld;
        b0_we     = 1'b0;
        b0_rd     = 5'd0;
        b0_idx    = 3'd0;
        b1_we     = 1'b0;
        b1_rd     = 5'd0;
        b1_idx    = 3'd0;
        if (pa == 2'd0) begin
            data_a = data_alu;
        end
        else if (pa == 2'd1) begin
            data_a = data_mul;
        end
        if (pb == 2'd1) begin
            data_b = data_mul;
        end
        else if (pb == 2'd2) begin
            data_b = data_ld;
        end
        if ((pa != 2'd3) && ~sup_a) begin
            we_a   = 1'b1;
            rd_a   = pa_rd;
            b0_we  = 1'b1;
            b0_rd  = pa_rd;
            b0_idx = pa_idx;
        end
        if ((pb != 2'd3) && ~sup_b) begin
            we_b   = 1'b1;
            rd_b   = pb_rd;
            b1_we  = 1'b1;
            b1_rd  = pb_rd;
            b1_idx = pb_idx;
        end
//放行口径 = "你这笔已经被处置掉了"，不是"你落地了"：rd=0 的写不占口也不落地，
//同样要放行 —— 否则 mulu 会永远举着一笔没人取的值（死等）。
        if (fin_mul)
            taken_mul = 1'b1;
    end

endmodule
