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
//   前送级：**两个前送点，选法同形** —— 都是"槽号的一位比较 + 一级 mux"，裁决都不在本级。
//   本文件内 always 块按【前送点（= 消费者所在流水级）】排列：点①（mid 级，寄存器号 _2）→ 点②（post 级，_3）。
//
//   点①：寄存器堆数据出来 → post_decoder 之间（消费者是 mid 级那条，用 rs1_2/rs2_2）。
//        选择位 = rob 的槽扫描**当拍组合输出**（`fwd_slot*/fwd_hit*`）——扫描口读的就是
//        mid 那条的 `rs1_2/rs2_2`，与点② 是同一对线，只是点① 用当拍版、点② 用锁存版。
//        它保证"进载荷之前"的值是最新的；I 型的 r2 随后会被 post_decoder 用立即数覆盖，
//        所以这一点上即使按 rs2 字段（其实是立即数）误命中，也影响不到最终操作数。
//   点②：post_decoder → 三个执行单元之间（消费者是 decoder 载荷那条）。
//        ★ 它**不在这一级做裁决**：选择位 `sel_slot1/sel_slot2/sel_v1/sel_v2` 是**载荷**，
//          由 `rob` 在**上一拍**（本条指令还在 mid 级、`rs1_2/rs2_2` 有效时）用**槽序扫描**算好，
//          与 `issue_idx` 同一个 `payload_go` 沿锁存寄下来。本级只剩
//          "源口的 `idx_*` == 目标槽号"的**一位比较 + 一级 mux**，
//          `rs==rd` 匹配 / 年龄相减 / 取最年轻的优先裁决**全部不在这里**。
//        ★ 为什么能这样：ROB 按程序序、一拍一条分配（`alloc_en = payload_go`，与进载荷同沿）
//          ⇒ 扫描那一拍在册的槽**全是比消费者老的**，"比我老"不需要任何比较；"最年轻"= 槽序里
//          离 tail 最近那个。选中的槽必然满足 `rd == rs`，所以 `lsu`/`mulu` **原样的** rd 键
//          hazard 同一条件必然命中，值就绪由它们兜（这两处检查点按设计约定一字不动）。
//
//   | 源 | 产生者 | 有效窗口 | 点① | 点② |
//   | --- | --- | --- | --- | --- |
//   | we_alu / rd_alu / data_alu / idx_alu | alu.v（算完的下一拍，紧邻消费者取用那一拍） | 一拍 | ✓ | ✓（按 idx 选） |
//   | we_mul / rd_mul / data_mul / idx_mul | mulu（m_pv / d_pend / hold） | 保持到 taken_mul | ✓ | ✓（按 idx 选） |
//   | we_ld / rd_ld / data_ld / idx_ld | lsu（数据回来拍 + ld_hold） | 两拍 | ✓ | ✓（按 idx 选） |
//   | wp_we_a·rd_a·data_a·idx_a（口 A） | wport 出口寄存（与阵列写同拍） | 一拍 | ✓ | — |
//   | wp_we_b·rd_b·data_b·idx_b（口 B） | 同上 | 一拍 | ✓ | — |
//
//   ★ 点① 为什么多两条：写口整体打一拍之后，值在出口寄存器里多待一拍 —— 这一拍它既不在
//     寄存器堆阵列里（阵列到下一沿才写），也不在单元结果口上（下一拍已换人）。regfile 的读
//     本身又是【寄存读】⇒ 这个窗口只有点① 补得住。
//   ★ 点① 一度不能改成槽序，理由是"消费者是 mid 级那条，上一拍还没被分配、拿不到槽号"。
//     **那个理由不成立**：前送要的是【生产者】的槽号，而扫描给的正是它 —— 扫描口
//     `scan_rs1/scan_rs2` 读的就是 `rs1_2/rs2_2`（点① 自己的消费者号），且扫描是组合输出、
//     当拍就能用。消费者自己的槽号从来不是前送需要的东西。
//   ★ 两个前送点的选择位来源同一处（rob 的槽序扫描）⇒ "rd 匹配 / 年龄相减 / 取最年轻"
//     这一整棵裁决树在两个点上都不存在了；`rob_head` 也从本模块整体退出。
//
// Dependencies:
//
// Revision:
//   Revision 0.03 - 改成"两个前送点 + 写回口作源"
//   Revision 0.04 - 判据从 function 改回【块内直接比较】（iverilog 敏感表）
//   Revision 0.05 - 乱序写回：源改接三个结果口，优先级改按 ROB 索引年龄取最年轻
//   Revision 0.06 - 点① 扩成 5 源（加 wport 出口寄存的两条写口），点② 保持 3 源
//   Revision 0.07 - 点② 改"槽序"：裁决搬到 rob 的扫描（上一拍），本级只剩一位比较 + 一级 mux
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module forw(
    input clk, rst,
//点① 用的槽扫描结果（rob 的组合输出）：扫的就是【点① 自己的消费者】——mid 那条的 rs1_2/rs2_2，
//与点② 那条是同一对线，只是相位早一拍（点① 当拍用、点② 用锁存版）。
//⇒ 点① 不再需要"年龄基准 rob_head + idx_mid + 取最年轻"这一整套。
    input [2:0]  fwd_slot1, fwd_slot2,
    input        fwd_hit1,  fwd_hit2,
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
//前送源④⑤：写口级出口寄存的两条物理写口（**寄存版**）—— 只给【点①】用。
//  写口打一拍之后值在出口寄存器里多待一拍，而寄存器堆当拍采的读值看不到它、单元结果口
//  下一拍又已换人 ⇒ 这个窗口只有这里补得住（点②不加：它的下游是消费者的寄生逻辑，
//  加源会让 lsu 的加法器/移位器那条路变深）。
    input        wp_we_a,
    input [4:0]  wp_rd_a,
    input [31:0] wp_data_a,
    input [2:0]  wp_idx_a,
    input        wp_we_b,
    input [4:0]  wp_rd_b,
    input [31:0] wp_data_b,
    input [2:0]  wp_idx_b,
//点①：消费者 = mid 级那条
    input [31:0] r1_data_mid_in, r2_data_mid_in,
    output reg [31:0] r1_data_mid, r2_data_mid,
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

//点①：与点②【同形】—— "该选谁"由 rob 的槽扫描给出（它扫的就是 mid 这条的 rs1_2/rs2_2，
//  当拍组合有效）⇒ 这里不再有"rd 匹配 + 年龄相减 + 取最年轻"的裁决，只剩
//  "源口的 ROB 索引 == 扫描出的槽号"的**一位比较 + 一级 mux**。
//★ 槽号唯一（8 个槽互不相同）⇒ 至多一个口命中；下面的优先序只是与点② 同序的防御性 tie-break。
//★ x0 保护：扫描的命中项本身带 `ent_rd != 0` 与 `scan_rs != 0`（见 rob.v 的 scan_m）⇒
//  读 x0 时 fwd_hit=0、落回寄存器堆读值（x0 恒 0），与旧式的 `r1_mid != 0` 门等价。
//★ 扫到的槽这一拍若没有任何口命中，就落回寄存器堆读值 —— 那一拍生产者在执行单元里还没出结果，
//  消费者必然被 lsu/mulu 的 hazard 顶住（与点② 那条注释同一个理由）。
//★ 判据必须【直接写在本块里】，不许塞进 function：iverilog 的 `always @(*)` 只把直接出现在
//  表达式里的信号收进隐含敏感表，函数体里读的模块级信号不收 ⇒ 本块不会重新求值（踩过）。
    always @(*) begin
        r1_data_mid = r1_data_mid_in;
        if (fwd_hit1) begin
            if (wp_we_b && (wp_idx_b == fwd_slot1))
                r1_data_mid = wp_data_b;
            else if (wp_we_a && (wp_idx_a == fwd_slot1))
                r1_data_mid = wp_data_a;
            else if (we_alu && (idx_alu == fwd_slot1))
                r1_data_mid = data_alu;
            else if (we_mul && (idx_mul == fwd_slot1))
                r1_data_mid = data_mul;
            else if (we_ld && (idx_ld == fwd_slot1))
                r1_data_mid = data_ld;
        end
        r2_data_mid = r2_data_mid_in;
        if (fwd_hit2) begin
            if (wp_we_b && (wp_idx_b == fwd_slot2))
                r2_data_mid = wp_data_b;
            else if (wp_we_a && (wp_idx_a == fwd_slot2))
                r2_data_mid = wp_data_a;
            else if (we_alu && (idx_alu == fwd_slot2))
                r2_data_mid = data_alu;
            else if (we_mul && (idx_mul == fwd_slot2))
                r2_data_mid = data_mul;
            else if (we_ld && (idx_ld == fwd_slot2))
                r2_data_mid = data_ld;
        end
    end

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
