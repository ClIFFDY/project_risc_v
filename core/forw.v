`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/28 14:41:46
// Design Name:
// Module Name: forw
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   前送级：**两个前送点，共用同一组前送源 —— 三个执行单元的【结果口】**。
//
//   点①：寄存器堆数据出来 → post_decoder 之间（消费者是 mem_buf 那条，寄存器号用 _2）。
//        它保证"进载荷之前"的值是最新的；I 型的 r2 随后会被 post_decoder 用立即数覆盖，
//        所以这一点上即使按 rs2 字段（其实是立即数）误命中，也影响不到最终操作数。
//   点②：post_decoder → 三个执行单元之间（消费者是 decoder 载荷那条，寄存器号用 _3）。
//        它盖住"载荷寄存器采样之后、单元取用之前"这一拍里才就绪的那一笔。
//
//   源（三条，各自带 ROB 索引当年龄）：
//     alu  结果口：alu.v 里寄存一拍（算完的下一拍），正好是紧邻消费者取用那一拍；
//     mulu 结果口：乘法/除法的写回窗口（m_pv / d_pend / 保持），单元自己举到被取走；
//     lsu  结果口：load 数据回来那一拍 + 保持拍（ld_hold）。
//   ★ 为什么不是"写口"：写口是这三条的 mux 出口（多一级 mux 落在操作数路上），而且
//     被杀的笔不上写口、却仍然要能给"夹在被杀者与杀它者之间"的消费者前送。
//
//   优先级 = **按年龄取最年轻**（年龄 = {1'b0,(idx - rob_head)}，先截 4 位再比）：
//     乱序写回之后三条源的先后【不由源的身份决定】，同 rd 多命中时必须挑更年轻的那笔。
//     用语句顺序定优先级是错的（旧的"口 B 更年轻"口径只对按序退成立）。
//
// Dependencies:
//
// Revision:
//   Revision 0.03 - 改成"两个前送点 + 写回口作源"
//   Revision 0.04 - 判据从 function 改回【块内直接比较】（iverilog 敏感表）
//   Revision 0.05 - 乱序写回：源改接三个结果口，优先级改按 ROB 索引年龄取最年轻
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module forw(
    input clk, rst,
//年龄基准 + 两个消费者各自的 ROB 索引（"只取比我更老的"要用）
    input [2:0]  rob_head,
    input [2:0]  idx_mid,
    input [2:0]  idx_post,
//前送源①：alu 结果口（寄存一拍，一拍宽）
    input        we_alu,
    input [4:0]  rd_alu,
    input [31:0] data_alu,
    input [2:0]  idx_alu,
//前送源②：mulu 结果口（保持到被写口取走）
    input        we_mul,
    input [4:0]  rd_mul,
    input [31:0] data_mul,
    input [2:0]  idx_mul,
//前送源③：lsu 结果口（数据回来拍 + 保持拍）
    input        we_ld,
    input [4:0]  rd_ld,
    input [31:0] data_ld,
    input [2:0]  idx_ld,
//点①：消费者 = mem_buf 那条
    input [4:0]  r1_mid, r2_mid,
    input [31:0] r1_data_mid_in, r2_data_mid_in,
    output reg [31:0] r1_data_mid, r2_data_mid,
//点② 的命中标志（给 post_decoder 在 stall 期间回灌用）：只在命中"比我更老"的那一档时置起
    output reg fb_hit1, fb_hit2,
//点②：消费者 = decoder 载荷那条（r1_en/r2_en = 该操作数是不是寄存器操作数）
    input [4:0]  r1_post, r2_post,
    input        r1_en, r2_en,
    input [31:0] r1_data_post_in, r2_data_post_in,
    (* max_fanout = 32 *) output reg [31:0] r1_data_final, r2_data_final
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//三条源的年龄（head 相对值，先截 4 位再比）+ 两个消费者自己的年龄
    reg [3:0] age_alu, age_mul, age_ld, age_mid, age_post;
//两个前送点各自的"已命中/当前最优年龄"（不能共用：两个块都会写，等于竞争）
    reg        h1m, h2m;
    reg [3:0]  g1m, g2m;
    reg [3:0]  g1f, g2f;

//★ 必须算"消费者自己"的年龄：结果口是前送源之后，【消费者自己那一笔也在源上】——
//  比如 `addi x28,x28,14` 紧跟在一条写 x28 的 load 后面时，它自己的结果口就在 x28 上，
//  不加这条限制，"取最年轻"会把【它自己的结果】喂回给它当操作数（实测 exc_irq_lsu：
//  算出 0+14，写口又把 load 那笔正确值压掉）。
    always @(*) begin
        age_alu  = {1'b0, (idx_alu  - rob_head)};
        age_mul  = {1'b0, (idx_mul  - rob_head)};
        age_ld   = {1'b0, (idx_ld   - rob_head)};
        age_mid  = {1'b0, (idx_mid  - rob_head)};
        age_post = {1'b0, (idx_post - rob_head)};
    end

//点①：源三条，同 rd 命中时按年龄取最年轻的
//★ 判据必须【直接写在本块里】，不许再塞进 function：
//  iverilog 的 `always @(*)` 只把直接出现在表达式里的信号收进隐含敏感表，函数体里读的
//  模块级信号不收 ⇒ 那些信号单独变化时本块不重新求值，输出停在旧值（单模块微测实测过）。
//★ x0 保护：x0 恒为 0，不许被任何源覆盖（`lw x0`/`mul x0` 会让 rd=0 的结果举在结果口上）。
    always @(*) begin
        r1_data_mid = r1_data_mid_in;
        r2_data_mid = r2_data_mid_in;
        if (r1_mid != 5'd0) begin
            h1m = 1'b0;
            g1m = 4'd0;
            if (we_alu && (r1_mid == rd_alu) && (age_alu < age_mid)) begin
                r1_data_mid = data_alu;
                g1m = age_alu;
                h1m = 1'b1;
            end
            if (we_mul && (r1_mid == rd_mul) && (age_mul < age_mid) && (!h1m || (age_mul > g1m))) begin
                r1_data_mid = data_mul;
                g1m = age_mul;
                h1m = 1'b1;
            end
            if (we_ld && (r1_mid == rd_ld) && (age_ld < age_mid) && (!h1m || (age_ld > g1m))) begin
                r1_data_mid = data_ld;
                g1m = age_ld;
                h1m = 1'b1;
            end
        end
        if (r2_mid != 5'd0) begin
            h2m = 1'b0;
            g2m = 4'd0;
            if (we_alu && (r2_mid == rd_alu) && (age_alu < age_mid)) begin
                r2_data_mid = data_alu;
                g2m = age_alu;
                h2m = 1'b1;
            end
            if (we_mul && (r2_mid == rd_mul) && (age_mul < age_mid) && (!h2m || (age_mul > g2m))) begin
                r2_data_mid = data_mul;
                g2m = age_mul;
                h2m = 1'b1;
            end
            if (we_ld && (r2_mid == rd_ld) && (age_ld < age_mid) && (!h2m || (age_ld > g2m))) begin
                r2_data_mid = data_ld;
                g2m = age_ld;
                h2m = 1'b1;
            end
        end
    end

//点②：同上，消费者是 decoder 载荷那条（只有真寄存器操作数才前送）
    always @(*) begin
        r1_data_final = r1_data_post_in;
        r2_data_final = r2_data_post_in;
        if (r1_en && (r1_post != 5'd0)) begin
            fb_hit1 = 1'b0;
            g1f = 4'd0;
            if (we_alu && (r1_post == rd_alu) && (age_alu < age_post)) begin
                r1_data_final = data_alu;
                g1f = age_alu;
                fb_hit1 = 1'b1;
            end
            if (we_mul && (r1_post == rd_mul) && (age_mul < age_post) && (!fb_hit1 || (age_mul > g1f))) begin
                r1_data_final = data_mul;
                g1f = age_mul;
                fb_hit1 = 1'b1;
            end
            if (we_ld && (r1_post == rd_ld) && (age_ld < age_post) && (!fb_hit1 || (age_ld > g1f))) begin
                r1_data_final = data_ld;
                g1f = age_ld;
                fb_hit1 = 1'b1;
            end
        end
        if (r2_en && (r2_post != 5'd0)) begin
            fb_hit2 = 1'b0;
            g2f = 4'd0;
            if (we_alu && (r2_post == rd_alu) && (age_alu < age_post)) begin
                r2_data_final = data_alu;
                g2f = age_alu;
                fb_hit2 = 1'b1;
            end
            if (we_mul && (r2_post == rd_mul) && (age_mul < age_post) && (!fb_hit2 || (age_mul > g2f))) begin
                r2_data_final = data_mul;
                g2f = age_mul;
                fb_hit2 = 1'b1;
            end
            if (we_ld && (r2_post == rd_ld) && (age_ld < age_post) && (!fb_hit2 || (age_ld > g2f))) begin
                r2_data_final = data_ld;
                g2f = age_ld;
                fb_hit2 = 1'b1;
            end
        end
    end
endmodule
