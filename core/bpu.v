`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/29 20:12:36
// Design Name:
// Module Name: bpu
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
//   Revision 0.01 - File Created
//   Revision 0.02 - 新增服务 br1/jal 的独立 bti（"br/jal 目标 inst 专用 itcm"）：
//                   jal/br1 的落点地址只送 pc，目标那条指令由本表直送 idu1，
//                   于是 icache 的取指地址不再吃"当拍译码 + 加法器"那条 14.4ns 的链。
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module bpu(
    input clk, rst,
//jalr 真目标（EX 期解析，落 BTB 用）
    input [31:0] jalr_tgt_q,
//交付口上【这条指令自己的地址】：本模块所有表的读口都用它，内部不做任何"取指地址 ± 拍差"
//的换算 —— 那个换算属于顶层接线（fe_top 的 cur_addr），不属于功能模块。
    input [31:0] cur_addr_0, cur_addr_1,
    input [8:0] br_pc_idx,
    input success, br_fail, jalr_tgt_vld, br_pred_taken_in,
//两条 lane 对称的预测源：iffu 的 br_en / jal 译码，各自独立、本模块不仲裁
    input br_en_0, br_en_1,
    input fch_jal_0, fch_jal_1,
    input fch_jalr_0,
//icache 当拍交付的一对（冷启动捕获用）与 flag_bus
    input [31:0] inst_in_0, inst_in_1,
    input inst_valid_0, inst_valid_1,
    input [7:0] flag_bus,
//pc 落到重定向目标那一拍（与 icache / iffu 的同一根）：捕获窗口必须在这拍作废
    input        flush_pc_redir,
//保留站的"满"【只喂前端取指推进】
    input stall_rs_full,
//队列的满：它的三态和 inst 一起被冻住，也一起重判
    input        fifo_full,
//两条 lane 对称的预测结果（br1 / jalr / 落点 / btic 命中）
    output reg [31:0] jalr_pred_offset_0, jalr_pred_offset_1,
    output reg br1_0, br1_1, br2, br3, jalr_0, jalr_1,
    output reg bti_hit_0, bti_hit_1,
//btic 注入：两条 lane 各一套完整独立（各自的欠交付、各自的一对目标 inst），本模块不做任何仲裁；
//iffu 按"哪一套欠着"自己选。bti_inst_x 是目标地址上的一对：低 32 = 目标那条、高 32 = 目标+4
    output reg [63:0] bti_inst_0, bti_inst_1,
    output reg [1:0]  bti_sel_0, bti_sel_1
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus 译码（行为块，放本模块最前）：**先把各位还原成原名，再按名字做逻辑**（不用位号）。
//adv 与 pc.v 那个推进门逐项同门：两边必须一致，否则"该改向却没交付"或"目标 inst 丢了"。
//这里的四条 = 前端真正会停的四条（后端停顿由取指队列吸收，不进这一位）。
    reg flush_con_exc, flush_con_irq, flush_con_jump;
    reg stall_rob_full, stall_pc_redir;
    reg stall_icache_miss, stall_bus_hold;
    reg exec_b, flush_w, stall_w, adv;
    always @(*) begin
        flush_con_exc     = flag_bus[7];
        flush_con_irq     = flag_bus[6];
        flush_con_jump    = flag_bus[5];
        exec_b            = flag_bus[4];
        stall_rob_full    = flag_bus[3];
        stall_pc_redir    = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        stall_bus_hold    = flag_bus[0];
        flush_w = flush_con_exc | flush_con_irq | flush_con_jump;
        stall_w = (stall_rob_full | stall_pc_redir | stall_rs_full
                 | stall_icache_miss | stall_bus_hold) & ~flush_w;
        adv     = ~flush_w & ~stall_pc_redir & ~stall_icache_miss & ~fifo_full;
    end

//==================================================================================
// 老的一组：BHT（方向）+ 服务 jalr 的 btb（预测落点地址）。本次一行不改。
//==================================================================================
    reg [1:0] bht [0:511];
    reg [31:0] btb [0:63];
//BTB 有效表：把命中判定从"目标 != 0"（32 位或归约）换成 1 位查表。
//写口与 btb 同拍按 (jalr_tgt_q != 0) 更新，复位和非命中项都是 0，
//所以 btb_v[i] 恒等于原来的 (btb[i] != 0)，逐位等价。
    reg btb_v [0:63];
    reg predict_en_0, predict_en_1;
//★ 两张表的读写索引必须都落在"指令自己的地址"上：写口吃的是后端带回来的载荷 PC
//（br_pc_idx，分支那条指令的地址），读口也吃交付口上那条指令自己的地址（cur_addr）。
//两宽取指下 pc_addr 比交付口上的指令地址大 8，直接拿 pc_addr 索引会把 [11:3] / [8:3]
//整个挪一格，训进去的项下一次永远读不到（实测：jalr 循环四遍，BTB 一次都没命中）。
    integer i;

//BHT查这条指令的地址，饱和计数>1则预测跳转（两条 lane 各查各的地址）
    always @(*) begin
        predict_en_0 = (bht[cur_addr_0[11:3]] > 2'd1);
        predict_en_1 = (bht[cur_addr_1[11:3]] > 2'd1);
    end

//==================================================================================
// 新的一组：服务 br1/jal 的独立 bti —— 存的是**跳转目标那条指令**（不是落点地址）。
//
// 为什么存 inst：落点地址还是要给 pc（pc 得跳到目标才取得下去）；而存了"目标 inst"之后，
// 跳转那一拍 icache 就**不必被寻址** ⇒ 取指地址 = pc>>>2 纯线选，"预测器出口 → iram 地址口"
// 这条路根本不存在（那正是当前 14.4ns 关键路径的终点）。
//
// 索引/口径：读口用交付口上这条指令自己的地址 cur_addr；写口在 take 那一拍锁同一批位。预测发生在"这条指令被
// idu1 译码那一拍"（那时 pc 已是 指令地址+4），读写口看到同一个 pc 值，自洽。
// tag 22 位（pc[31:10]）与 8 位索引合起来覆盖 pc[31:2] 全部位 ⇒ 两条不同指令不可能同 tag
// 同 index ⇒ **命中即条目属于这条指令**；jal/br 落点每地址恒定、无自修改代码 ⇒ 存下的 inst
// 恒真，不需要校验（jalr 那套预测+校验是另一张表，别混）。
// 复位只清 bti_v：bti_inst/bti_tag 陈旧无害，带复位会撑爆 slice 打包（icache 的 tag1/tag2 同理）。
//==================================================================================
    (* ram_style = "distributed" *) reg [63:0] bti_inst [0:255];
    (* ram_style = "distributed" *) reg [21:0] bti_tag  [0:255];
    (* ram_style = "distributed" *) reg        bti_v    [0:255];

    reg [7:0]  bti_rd_idx0, bti_rd_idx1;
    reg [21:0] bti_rd_tag0, bti_rd_tag1;
    always @(*) begin
        bti_rd_idx0 = cur_addr_0[9:2];
        bti_rd_tag0 = cur_addr_0[31:10];
        bti_rd_idx1 = cur_addr_1[9:2];
        bti_rd_tag1 = cur_addr_1[31:10];
    end
//交付：take_x 那一拍记下"欠一次交付"。交付位必须能粘住，不能是单拍脉冲。
//撤销只看 full：full 那拍 icache 保持输出、本模块也保持三态；没 full 就说明这一拍的三态
//已经被队列收下，下一拍重新判（该跳的会再 take 一次）。
    reg        bti_pend_0, bti_pend_1;
    reg        bti_kind_0, bti_kind_1;
//★ 冲刷必须**作废**欠着的这次交付：欠交付的那条分支一定比冲刷源更年轻 ⇒ 是错路。
//  只靠 take 上的 ~flush_w 挡不住"已经欠下、随后才来冲刷"的情形（实测：分支预测跳、
//  下一拍来了 br3 冲刷，欠下的目标 inst 在冲刷后仍被交付 ⇒ 多执行一条）
//捕获（冷启动学习）：take 且没命中时开闸，两拍后正好是"icache 交付目标 inst"那一拍。
//任何 flush/stall 都中止这一次（自愈：下一次再学），绝不让垃圾写进表。
    reg [1:0]  cap_q;
    reg [7:0]  cap_idx_q1, cap_idx_q2;
    reg [21:0] cap_tag_q1, cap_tag_q2;

//新 bti 的查表/命中判定 + take（与 pc.v 同一个门）：
//take=1 且命中 ⇒ 这一拍记下交付目标 inst；take=1 且未命中 ⇒ 这一拍记下交付 NOP，
//由 pc 落 T 让 icache 自己去取目标（差一拍，只有 jal 冷启动吃这一拍）。
    reg bti_hit0, bti_hit1;
    reg take_0, take_1, lane0_redir;
    always @(*) begin
        bti_hit0 = bti_v[bti_rd_idx0] & (bti_tag[bti_rd_idx0] == bti_rd_tag0);
        bti_hit1 = bti_v[bti_rd_idx1] & (bti_tag[bti_rd_idx1] == bti_rd_tag1);
        bti_hit_0 = bti_hit0;
        bti_hit_1 = bti_hit1;
//这一对里 lane0 会不会改向（口径与 iffu 的 lane0_redir、pc 的 if 链顺序一致：lane0 先）。
//lane0 改了向，lane1 就是错路，它的欠交付绝不能再武装 —— 否则 iffu 会把 lane1 那一套
//注入摆到口上，把 pc 实际去的落点（lane0 的）整个换掉。
//BTB（jalr 落点）和 BTIC（目标指令）是两件东西，这里只是把"lane0 改不改向"拼出来，
//两张表各管各的，仍然互不干涉。
        lane0_redir = br1_0 | fch_jal_0 | (fch_jalr_0 & jalr_0);
        take_0   = (fch_jal_0 | br1_0) & adv;
        take_1   = (fch_jal_1 | br1_1) & adv & ~lane0_redir;
    end

//欠交付状态机：两条 lane 各一套，互不相干
    always @(posedge clk) begin
        if (rst_q) begin
            bti_pend_0 <= 1'b0;
            bti_pend_1 <= 1'b0;
            bti_kind_0 <= 1'b0;
            bti_kind_1 <= 1'b0;
            bti_inst_0 <= 64'd0;
            bti_inst_1 <= 64'd0;
        end
        else begin
//★ 重定向优先级最高：欠交付属于被改向掉的那条跳转，必须作废。只靠 `~fifo_full` 撤不掉
//  —— 重定向那拍如果恰好队列是满的，三态会被"保持"，下一拍它带着新的 pc 再注一次
//  （实测 seed=8：注入被标成 0x3ff8 = 新 pc 0x4000 − 8）。
            if (flush_w | flush_pc_redir) begin
                bti_pend_0 <= 1'b0;
                bti_pend_1 <= 1'b0;
            end
            else begin
                if (take_0) begin
                    bti_pend_0 <= 1'b1;
                    bti_kind_0 <= bti_hit0;
                    bti_inst_0 <= bti_inst[bti_rd_idx0];
                end
                else if (~fifo_full) begin
                    bti_pend_0 <= 1'b0;
                end
                if (take_1) begin
                    bti_pend_1 <= 1'b1;
                    bti_kind_1 <= bti_hit1;
                    bti_inst_1 <= bti_inst[bti_rd_idx1];
                end
                else if (~fifo_full) begin
                    bti_pend_1 <= 1'b0;
                end
            end
        end
    end

//交付三态：0 = 用 icache 正常交付；1 = 用本 lane 的那对目标 inst；2 = 交付 NOP
    always @(*) begin
        bti_sel_0 = bti_pend_0 ? (bti_kind_0 ? 2'd1 : 2'd2) : 2'd0;
        bti_sel_1 = bti_pend_1 ? (bti_kind_1 ? 2'd1 : 2'd2) : 2'd0;
    end

//捕获通路的使能与 index/tag 跟着走两拍。
//★ 停顿只能【冻结】窗口，不能作废：pc_addr 的推进条件就是上面 adv 那四条
//  ⇒ 停顿期间取指侧（含 icache 的"这一拍读、下一拍出"）和这条窗口是【一起冻住】的，两拍的
//  对齐关系不变。原来把 stall 也当"作废"清呢，一旦停顿相位相对取指移了一拍（删 mid_decoder
//  就是），窗口几乎每次都被清掉 —— 实测 40 万拍里开闸 16521 次、只有 257 次走到待写（94% 是
//  stall 清掉的），bti 表学不进去 ⇒ 每次跳转吃冷启动 NOP ⇒ CoreMark +1.7%。
//  冲刷（flush_w）仍作废：那是错路，pc 会跳走，写进去就是垃圾。
//  ★ pc 改向（flush_pc_redir）同样必须作废：窗口记的是"两拍后口上那一对就是目标"，
//  中间 pc 被重定向拉走，口上就不是目标了。实测（自环 jal 冷启动，窗口开着时来一次重定向）：
//  index 10 被写进 {instr(0x18), instr(0x14)}，下一次命中就在目标地址上注入了这两条。
    always @(posedge clk) begin
        if (rst_q) begin
            cap_q      <= 2'd0;
            cap_idx_q1 <= 8'd0;
            cap_idx_q2 <= 8'd0;
            cap_tag_q1 <= 22'd0;
            cap_tag_q2 <= 22'd0;
        end
        else if (flush_w | flush_pc_redir) begin
            cap_q      <= 2'd0;
            cap_idx_q1 <= cap_idx_q1;
            cap_tag_q1 <= cap_tag_q1;
            cap_idx_q2 <= cap_idx_q2;
            cap_tag_q2 <= cap_tag_q2;
        end
        else if (!adv) begin
            cap_q      <= cap_q;
            cap_idx_q1 <= cap_idx_q1;
            cap_tag_q1 <= cap_tag_q1;
            cap_idx_q2 <= cap_idx_q2;
            cap_tag_q2 <= cap_tag_q2;
        end
        else begin
//捕获口只有一个（一拍只能学一条目标行）：两条 lane 同时没命中时学 lane0 那条，
//lane1 的下次再学。这只是学习率，不影响正确性。
            cap_q      <= {cap_q[0], (take_0 & ~bti_hit0) | (take_1 & ~bti_hit1)};
            cap_idx_q1 <= take_0 ? bti_rd_idx0 : bti_rd_idx1;
            cap_tag_q1 <= take_0 ? bti_rd_tag0 : bti_rd_tag1;
            cap_idx_q2 <= cap_idx_q1;
            cap_tag_q2 <= cap_tag_q1;
        end
    end

//回写：方向表按载荷 PC 加减；jalr 落点按载荷 PC 回写；新 bti 按捕获通路写目标 inst。
//三张新表各自只有这一条写语句（写成两条不同地址的语句会被推成多端口 —— icache 的 iram 就是这么翻车的）。
    always @(posedge clk) begin
        if (rst_q) begin
//初始BHT回到弱不跳转
            for (i = 0; i < 512; i = i + 1)
                bht[i] <= 2'd1;
            for (i = 0; i < 64; i = i + 1)
                btb[i] <= 32'd0;
            for (i = 0; i < 64; i = i + 1)
                btb_v[i] <= 1'b0;
            for (i = 0; i < 256; i = i + 1)
                bti_v[i] <= 1'b0;
        end
        else begin
//分支在EX期判定：按随指令流水的载荷PC回写BHT
            if (success) begin
                if (bht[br_pc_idx] < 2'd3)
                    bht[br_pc_idx] <= bht[br_pc_idx] + 2'd1;
            end
            else if (br_fail) begin
                if (bht[br_pc_idx] > 2'd0)
                    bht[br_pc_idx] <= bht[br_pc_idx] - 2'd1;
            end
//jalr实际目标在EX期解析：按随指令流水的载荷PC回写BTB
            if (jalr_tgt_vld) begin
                btb[br_pc_idx[5:0]] <= jalr_tgt_q;
                btb_v[br_pc_idx[5:0]] <= (jalr_tgt_q != 32'd0);
            end
//冷启动捕获：这一拍 icache 交付的就是目标 inst（pc 已于 take 那拍落 T）
//★ 写口必须自己再带 ~flush_w：cap_q 是"到沿才清"，冲刷那一拍 cap_q[1] 还是 1，
//  而 inst_valid 也还是上一拍的（电平信号、到沿才被冲刷压掉）⇒ 会把冲掉的错路交付
//  （实测是 0）当目标 inst 连同 bti_v=1 一起写进表 ⇒ 该分支此后命中到一条空指令、
//  "pc 落 T+4 + bti 交付 T"这一对里 T 永远没人交付（静默丢一条）。
//  表里 0 只可能是空泡（真指令不会是 0）⇒ 这一条不改学习能力、也不改命中率。
            if (cap_q[1] & inst_valid_0 & inst_valid_1 & ~flush_w) begin
                bti_inst[cap_idx_q2] <= {inst_in_1, inst_in_0};
                bti_tag[cap_idx_q2]  <= cap_tag_q2;
                bti_v[cap_idx_q2]    <= 1'b1;
            end
        end
    end

//组合输出：两条 lane 各自对称的预测结果（本模块不仲裁谁先）
    always @(*) begin
        br1_0 = br_en_0 & predict_en_0;
        br1_1 = br_en_1 & predict_en_1;
        br2 = success & !br_pred_taken_in;
        br3 = br_fail & br_pred_taken_in;
        jalr_0 = btb_v[cur_addr_0[8:3]];
        jalr_1 = btb_v[cur_addr_1[8:3]];
        jalr_pred_offset_0 = btb[cur_addr_0[8:3]];
        jalr_pred_offset_1 = btb[cur_addr_1[8:3]];
    end

endmodule
