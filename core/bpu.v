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
    input [31:0] pc_addr_in, jalr_target_q,
    input [5:0] br_pc_idx,
    input success, br_fail, br_en, jalr_flag, br_pred_taken_in,
//新增：jal 译码信号（idu1 当拍组合输出）、i-cache 当拍交付的指令与有效位（冷启动捕获用）、
//flag_bus（本模块自己译出 flush/stall，按"模块内先还原原名再做逻辑"的规矩）
    input jal,
    input [31:0] inst_in,
    input inst_valid,
    input [7:0] flag_bus,
//保留站的"满"【只喂前端取指推进】
    input stall_rs_full,
    input        fifo_full,
//队列把本拍这条交付收下了：欠交付的目标 inst 只有被收下才算交掉（空泡也是收下）
    input        take_en,
    output reg [31:0] jalr_predict_offset,
    output reg br1, br2, br3, jalr,
//新增：命中标志（给 pc 选"落 T+4 / 落 T"）、交付给 idu1 的目标指令与三态选择
    output reg bti_hit,
    output reg [31:0] bti_inst_q,
    output reg [1:0] bti_sel_q
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
        if (rst_q)
            predict_en = 1'b0;
        else
            predict_en = (bht[pc_addr_in[8:3]] > 2'd1);
    end

//==================================================================================
// 新的一组：服务 br1/jal 的独立 bti —— 存的是**跳转目标那条指令**（不是落点地址）。
//
// 为什么存 inst：落点地址还是要给 pc（pc 得跳到目标才取得下去）；而存了"目标 inst"之后，
// 跳转那一拍 icache 就**不必被寻址** ⇒ 取指地址 = pc>>>2 纯线选，"预测器出口 → iram 地址口"
// 这条路根本不存在（那正是当前 14.4ns 关键路径的终点）。
//
// 索引/口径：读口用当拍 pc_addr_in；写口在 take 那一拍锁同一批位。预测发生在"这条指令被
// idu1 译码那一拍"（那时 pc 已是 指令地址+4），读写口看到同一个 pc 值，自洽。
// tag 22 位（pc[31:10]）与 8 位索引合起来覆盖 pc[31:2] 全部位 ⇒ 两条不同指令不可能同 tag
// 同 index ⇒ **命中即条目属于这条指令**；jal/br 落点每地址恒定、无自修改代码 ⇒ 存下的 inst
// 恒真，不需要校验（jalr 那套预测+校验是另一张表，别混）。
// 复位只清 bti_v：bti_inst/bti_tag 陈旧无害，带复位会撑爆 slice 打包（icache 的 tag1/tag2 同理）。
//==================================================================================
    (* ram_style = "distributed" *) reg [31:0] bti_inst [0:255];
    (* ram_style = "distributed" *) reg [21:0] bti_tag  [0:255];
    (* ram_style = "distributed" *) reg        bti_v    [0:255];

    reg [7:0]  bti_rd_idx;
    reg [21:0] bti_rd_tag;
    reg        take;
//交付：take 那一拍记下"欠一次交付"，一直保持到 idu1 真的把这条锁进流水线
//（exec_b & adv），中途 stall 也不会丢 —— 交付位必须能粘住，不能是单拍脉冲。
    reg        bti_pend;
    reg        bti_kind_q;
//捕获（冷启动学习）：take 且没命中时开闸，两拍后正好是"icache 交付目标 inst"那一拍。
//任何 flush/stall 都中止这一次（自愈：下一次再学），绝不让垃圾写进表。
    reg [1:0]  cap_q;
    reg [7:0]  cap_idx_q1, cap_idx_q2;
    reg [21:0] cap_tag_q1, cap_tag_q2;

//新 bti 的查表/命中判定 + take（与 pc.v 同一个门）：
//take=1 且命中 ⇒ 这一拍记下交付目标 inst；take=1 且未命中 ⇒ 这一拍记下交付 NOP，
//由 pc 落 T 让 icache 自己去取目标（差一拍，只有 jal 冷启动吃这一拍）。
    always @(*) begin
        if (rst_q) begin
            bti_rd_idx = 8'd0;
            bti_rd_tag = 22'd0;
            bti_hit    = 1'b0;
            take       = 1'b0;
        end
        else begin
            bti_rd_idx = pc_addr_in[9:2];
            bti_rd_tag = pc_addr_in[31:10];
            bti_hit    = bti_v[bti_rd_idx] & (bti_tag[bti_rd_idx] == bti_rd_tag);
            take       = (jal | br1) & adv;
        end
    end

//欠交付状态机
    always @(posedge clk) begin
        if (rst_q) begin
            bti_pend    <= 1'b0;
            bti_kind_q  <= 1'b0;
            bti_inst_q  <= 32'd0;
        end
        else if (take) begin
            bti_pend    <= 1'b1;
            bti_kind_q  <= bti_hit;
            bti_inst_q  <= bti_inst[bti_rd_idx];
        end
//★ 冲刷必须**作废**欠着的这次交付：欠交付的那条分支一定比冲刷源更年轻 ⇒ 是错路。
//  只靠 take 上的 ~flush_w 挡不住"已经欠下、随后才来冲刷"的情形（实测：分支预测跳、
//  下一拍来了 br3 冲刷，欠下的目标 inst 在冲刷后仍被交付 ⇒ 多执行一条）。
//  交付位必须能粘住（stall 不丢），但必须能被冲刷清掉。
        else if (flush_w | (bti_pend && take_en)) begin
            bti_pend    <= 1'b0;
        end
    end

//交付三态：0 = 用 icache 正常交付；1 = 用 bti 的目标 inst；2 = 交付 NOP
    always @(*) begin
        if (rst_q)
            bti_sel_q = 2'd0;
        else if (bti_pend)
            bti_sel_q = bti_kind_q ? 2'd1 : 2'd2;
        else
            bti_sel_q = 2'd0;
    end

//捕获通路的使能与 index/tag 跟着走两拍。
//★ 停顿只能【冻结】窗口，不能作废：pc_addr 的推进条件就是上面 adv 那四条
//  ⇒ 停顿期间取指侧（含 icache 的"这一拍读、下一拍出"）和这条窗口是【一起冻住】的，两拍的
//  对齐关系不变。原来把 stall 也当"作废"清呢，一旦停顿相位相对取指移了一拍（删 mid_decoder
//  就是），窗口几乎每次都被清掉 —— 实测 40 万拍里开闸 16521 次、只有 257 次走到待写（94% 是
//  stall 清掉的），bti 表学不进去 ⇒ 每次跳转吃冷启动 NOP ⇒ CoreMark +1.7%。
//  冲刷（flush_w）仍作废：那是错路，pc 会跳走，写进去就是垃圾。
    always @(posedge clk) begin
        if (rst_q) begin
            cap_q      <= 2'd0;
            cap_idx_q1 <= 8'd0;
            cap_idx_q2 <= 8'd0;
            cap_tag_q1 <= 22'd0;
            cap_tag_q2 <= 22'd0;
        end
        else if (flush_w) begin
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
            cap_q      <= {cap_q[0], take & ~bti_hit};
            cap_idx_q1 <= pc_addr_in[9:2];
            cap_tag_q1 <= pc_addr_in[31:10];
            cap_idx_q2 <= cap_idx_q1;
            cap_tag_q2 <= cap_tag_q1;
        end
    end

//回写：方向表按载荷 PC 加减；jalr 落点按载荷 PC 回写；新 bti 按捕获通路写目标 inst。
//三张新表各自只有这一条写语句（写成两条不同地址的语句会被推成多端口 —— icache 的 iram 就是这么翻车的）。
    always @(posedge clk) begin
        if (rst_q) begin
//初始BHT回到弱不跳转
            for (i = 0; i < 64; i = i + 1)
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
            if (jalr_flag) begin
                btb[br_pc_idx] <= jalr_target_q;
                btb_v[br_pc_idx] <= (jalr_target_q != 32'd0);
            end
//冷启动捕获：这一拍 icache 交付的就是目标 inst（pc 已于 take 那拍落 T）
            if (cap_q[1] & inst_valid) begin
                bti_inst[cap_idx_q2] <= inst_in;
                bti_tag[cap_idx_q2]  <= cap_tag_q2;
                bti_v[cap_idx_q2]    <= 1'b1;
            end
        end
    end

//组合输出：当前取指PC的预测结果 + 载荷的预测判定
    always @(*) begin
        if (rst_q) begin
            br1 = 1'b0;
            br2 = 1'b0;
            br3 = 1'b0;
            jalr = 1'b0;
            jalr_predict_offset = 32'd0;
        end
        else begin
            br1 = br_en & predict_en;
            br2 = success & !br_pred_taken_in;
            br3 = br_fail & br_pred_taken_in;
            jalr_predict_offset = btb[pc_addr_in[8:3]];
            jalr = btb_v[pc_addr_in[8:3]];
        end
    end

endmodule
