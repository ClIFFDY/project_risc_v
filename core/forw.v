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
//   前送级：**只剩一个前送点**（消费者 = decoder 载荷那条）。裁决不在本级 ——
//   选择位 `sel_slot1/sel_slot2/sel_v1/sel_v2` 是**载荷**：由 `rob` 在**上一拍**
//   （本条指令还在 pre 级、`rs*_1` 有效时）用**槽序扫描**算好、随载荷一起锁存寄下来
//   ⇒ 本级只剩"源口的 `idx_*` == 目标槽号"的**一位比较 + 一级 mux**。
//   `rs==rd` 匹配 / 年龄相减 / 取最年轻的优先裁决**全部不在这里**。
//
//   默认值 `r1_data_post_in/r2_data_post_in` = `post_decoder` 的**组合装配**输出
//   （底值 = regfile 寄存读的值、已含它的 5 源读侧旁路；非寄存器操作数在那里被立即数/pc/uimm 覆盖）。
//
//   ★ 为什么能这样：ROB 按程序序、一拍一条分配（`alloc_en = payload_go`，与进载荷同沿）
//     ⇒ 扫描那一拍在册的槽**全是比消费者老的**，"比我老"不需要任何比较。选中的槽必然满足
//     `rd == rs`，所以 `lsu`/`mulu` **原样的** rd 键 hazard 同一条件必然命中，值就绪由它们兜
//     （这两处检查点按设计约定一字不动）。
//
//   | 源 | 产生者 | 有效窗口 |
//   | --- | --- | --- |
//   | we_alu / rd_alu / data_alu / idx_alu | alu.v（算完的下一拍，紧邻消费者取用那一拍） | 一拍 |
//   | we_mul / rd_mul / data_mul / idx_mul | mulu（m_pv / d_pend / hold） | 保持到 taken_mul |
//   | we_ld / rd_ld / data_ld / idx_ld | lsu（数据回来拍 + ld_hold） | 两拍 |
//
//   ★【前送点① 已随 mid_decoder 一起删除】：原来"mid 级那一拍"的前送点（5 源 = 3 结果口 +
//     2 写口出口寄存）在删掉 mid_decoder 之后没有消费者了 —— 它那 5 个源整体下移到
//     `regfile` 的**读侧旁路**（读锁存沿 + 停顿重读都在那里），本级只剩点②、仍是 3 源。
//     `rob_head` / `idx_mid` / 8 个 `wp_*` 端口随之整体退出。
//
// Dependencies:
//
// Revision:
//   Revision 0.03 - 改成"两个前送点 + 写回口作源"
//   Revision 0.04 - 判据从 function 改回【块内直接比较】（iverilog 敏感表）
//   Revision 0.05 - 乱序写回：源改接三个结果口，优先级改按 ROB 索引年龄取最年轻
//   Revision 0.06 - 点① 扩成 5 源（加 wport 出口寄存的两条写口），点② 保持 3 源
//   Revision 0.07 - 点② 改"槽序"：裁决搬到 rob 的扫描（上一拍），本级只剩一位比较 + 一级 mux
//   Revision 0.08 - 删 mid_decoder：点① 整体删除，它的 5 个源下移到 regfile 读侧旁路；
//                   本级只剩点②、仍是 3 源
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module forw(
    input clk, rst,
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
//点②：消费者 = decoder 载荷那条。★ 它的"该选谁"已经由 `rob` 的槽扫描在**上一拍**算完、
//  随载荷一起寄下来了（sel_slot/sel_v）⇒ 本拍只剩"源口的 ROB 索引 == 目标槽号"的**一位比较 + 一级 mux**；
//  `rs==rd` 匹配、年龄相减、取最年轻的优先裁决**全部不在这里**（也不再需要 r1_post/r2_post/r1_en/r2_en）。
    input [2:0]  sel_slot1, sel_slot2,
    input        sel_v1,    sel_v2,
    input [31:0] r1_data_post_in, r2_data_post_in,
    (* max_fanout = 32 *) output reg [31:0] r1_data_final, r2_data_final
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//点②：消费者是 decoder 载荷那条。选择位是【载荷】（上一拍由 rob 的槽扫描算好）⇒
//本拍只剩"源口的 ROB 索引 == 目标槽号"的一位比较 + 一级 mux（槽号唯一 ⇒ 至多一个口命中）。
//★ 顺序仍按 alu→mul→ld 书写：槽号唯一时不会并列，这是防御性的 tie-break。
//★ 选中的槽若这一拍没有任何口命中，就落回载荷值 —— 那一拍必然是单元侧 stall 抬着
//  （选中的槽必然满足 rd == rs，`lsu`/`mulu` 的旧 hazard 同一条件必然命中），而各单元
//  在 stall 拍不锁结果 ⇒ 落回值不会被消费。
    always @(*) begin
        r1_data_final = r1_data_post_in;
        if (sel_v1) begin
            if (we_alu && (idx_alu == sel_slot1))
                r1_data_final = data_alu;
            else if (we_mul && (idx_mul == sel_slot1))
                r1_data_final = data_mul;
            else if (we_ld && (idx_ld == sel_slot1))
                r1_data_final = data_ld;
        end
        r2_data_final = r2_data_post_in;
        if (sel_v2) begin
            if (we_alu && (idx_alu == sel_slot2))
                r2_data_final = data_alu;
            else if (we_mul && (idx_mul == sel_slot2))
                r2_data_final = data_mul;
            else if (we_ld && (idx_ld == sel_slot2))
                r2_data_final = data_ld;
        end
    end
endmodule
