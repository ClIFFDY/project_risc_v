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
//   前送级：**只剩一个数据 mux**。裁决**全部提前到生产单元里做完**：
//
//   | 源 | 命中位怎么来 | 有效窗口 |
//   | --- | --- | --- |
//   | alu  | `alu.v` 在【结果寄存那一沿】用当拍槽扫描比好、与 result 同沿寄存（一整拍余量） | 一拍 |
//   | mulu | `mulu.v` 在三条结果路（m_pv / d_pend / hold）里与 `mul_idx` 逐支同步给出 | 保持到被取走 |
//   | lsu  | `lsu.v` 在 `ld_we` / `ld_hold` 两支里与 `ld_idx` 同步给出 | 两拍 |
//
//   ★ 为什么能这样：单元寄存结果那一沿，消费者正好在 pre 级，槽扫描给出的就是它的期望槽号 ⇒ 单元自己
//     （拿自己的 idx）就能判定"我这一拍是不是在给它供值"。alu 的结果口只一拍宽，所以那一沿寄存即准；
//     mulu/lsu 的数据是当拍组合给出的（寄一份会与数据错开一拍），故用【载荷里锁存的】槽号比。
//   ★ 本模块不再有 `idx_* == slot` 比较器、也没有 alu→mul→ld 优先链：收到的是三根已算好的命中位
//     （槽号唯一 ⇒ 至多一个为 1，`if-else` 只是防御性 tie-break）。
//   ★ 命中位全 0 时落回 `r1_data`（regfile 的寄存读，含它的 5 源读侧旁路）：那一拍生产者还没出结果
//     （消费它的指令被单元侧 hazard 顶着），或者它的值已经落进阵列 —— 两种情况下落回值要么正确、要么不被消费。
//
// Dependencies:
//
// Revision:
//   Revision 0.03 - 改成"两个前送点 + 写回口作源"
//   Revision 0.04 - 判据从 function 改回【块内直接比较】（iverilog 敏感表）
//   Revision 0.05 - 乱序写回：源改接三个结果口，优先级改按 ROB 索引年龄取最年轻
//   Revision 0.06 - 点① 扩成 5 源（加 wport 出口寄存的两条写口），点② 保持 3 源
//   Revision 0.07 - 点② 改"槽序"：裁决搬到 rob 的扫描（上一拍），本级只剩一位比较 + 一级 mux
//   Revision 0.08 - 删 mid_decoder：点① 整体删除，5 个源下移到 regfile 读侧旁路；本级只剩点②
//   Revision 0.09 - 命中位搬进三个单元（各在结果那一沿/同一拍比好），本级退化成"一级 mux + 一个编码"；
//                   装配（imm/pc/uimm 覆盖）也从 post_decoder 并进来，操作数只穿这一级
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module forw(
    input clk, rst,
//前送源①：alu 结果口
    input [31:0] data_alu,
//前送源②：mulu 结果口
    input [31:0] data_mul,
//前送源③：lsu 结果口
    input [31:0] data_ld,
//三个单元算好的命中位（各 2 位：r1/r2 各一位）——本模块不再做任何比较
    input        hit1_alu, hit1_mul, hit1_ld,
    input        hit2_alu, hit2_mul, hit2_ld,
//操作数默认源（原来在 post_decoder 里装配的两组，并到这里同一次选择）
    input [31:0] r1_data,    r2_data,
    input [31:0] r1_imm_val, r2_imm_val,
    input        r1_imm_sel, r2_imm_sel,
    (* max_fanout = 32 *) output reg [31:0] r1_data_final, r2_data_final
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就放在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//点②唯一的数据 mux（源值 → 消费者）：选择端全部早到，晚到的只有数据。
//★ 选择码与 mux 必须写在【同一个】always 块里：拆成两块时 case 的表达式由另一块驱动，
//  块间求值次序不定（iverilog 实测首拍就选空、输出留 X）—— 合成无所谓，仿真会错。
//★ 选择码：0=reg 1=imm 2=alu 3=mul 4=ld。"命中优先于 imm"在下面落定：先按 imm_sel 落默认，
//  命中再覆盖（与旧 forw 的合成语义一致；槽号唯一 ⇒ 至多一个命中，命中间的顺序只是防御性 tie-break）。
    reg [2:0] src1, src2;
    always @(*) begin
        if (r1_imm_sel)
            src1 = 3'd1;
        else
            src1 = 3'd0;
        if (hit1_alu)
            src1 = 3'd2;
        else if (hit1_mul)
            src1 = 3'd3;
        else if (hit1_ld)
            src1 = 3'd4;
        if (r2_imm_sel)
            src2 = 3'd1;
        else
            src2 = 3'd0;
        if (hit2_alu)
            src2 = 3'd2;
        else if (hit2_mul)
            src2 = 3'd3;
        else if (hit2_ld)
            src2 = 3'd4;
        case (src1)
            3'd0: r1_data_final = r1_data;
            3'd1: r1_data_final = r1_imm_val;
            3'd2: r1_data_final = data_alu;
            3'd3: r1_data_final = data_mul;
            3'd4: r1_data_final = data_ld;
            default: r1_data_final = r1_data;
        endcase
        case (src2)
            3'd0: r2_data_final = r2_data;
            3'd1: r2_data_final = r2_imm_val;
            3'd2: r2_data_final = data_alu;
            3'd3: r2_data_final = data_mul;
            3'd4: r2_data_final = data_ld;
            default: r2_data_final = r2_data;
        endcase
    end

endmodule
