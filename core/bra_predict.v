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
//   Revision 0.01 - File Created
//   Revision 0.02 - 新增服务 br1/jal 的独立 bti（"br/jal 目标 inst 专用 itcm"）：
//                   jal/br1 的落点地址只送 pc，目标那条指令由本表直送 pre_decoder，
//                   于是 icache 的取指地址不再吃"当拍译码 + 加法器"那条 14.4ns 的链。
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module bra_predict(
    input clk, rst,
    input [31:0] pc_addr_in, jalr_target_q,
    input [5:0] br_pc_idx,
    input success, br_fail, br_en, jalr_flag, br_pred_taken_in,
//新增：jal 译码信号（pre_decoder 当拍组合输出）、i-cache 当拍交付的指令与有效位（冷启动捕获用）、
//flag_bus（本模块自己译出 flush/stall，按"模块内先还原原名再做逻辑"的规矩）
    input jal,
//本拍交付的 lane0 是不是控制转移（fetch_fifo 在 inst_eff 上译的）：决定三张表这一拍服务哪条
    input lane0_ct,
//★ 这一拍【有没有改向】（两个 lane 的三类合一，fetch_fifo 出）：有 ⇒ 欠 BTIC 一次交付 ——
//  命中就送目标对、未命中就让"冲刷那拍出 NOP"顶掉 icache 的字（三态里的 state 2）。jalr 也算，
//  它那拍必须出 NOP，否则改向后那对错路会进队。
    input ct_redir,
//★ BTIC 的键（fetch_fifo 给）：正在改向那条指令自己的键 —— 与 rd_key 不同源
    input [31:0] bti_key,
//★ 这一拍改向的是不是【jalr】：是的话**不许捕获**进表 —— 本表存的是目标那一对 inst，
//  只对落点恒定的指令成立（jalr 落点随寄存器变，缓存下来就是脏的）。
    input ct_jalr,
    input [31:0] inst_in, inst_next_in,
    input inst_valid,
    input [11:0] flag_bus,
//取指队列满：与本级的推进门同源（本级也在取指侧）
    input fifo_full,
    output reg [31:0] jalr_predict_offset,
    output reg br1, br2, br3, jalr,
//lane1 的方向预测（三张表服务 lane1 时才有意义）
    output reg br1_1,
//新增：命中标志（给 pc 选"落 T+4 / 落 T"）、交付给 pre_decoder 的目标指令与三态选择
    output reg bti_hit,
    output reg [63:0] bti_inst_q,
    output reg [1:0] bti_sel_q
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//索引口径 = "指令地址 + 4"：pc 是"下一个待取地址"、一次吃两个字，本拍交付的 lane0 是
//instr(pc − 8)，它的"指令地址 + 4"就是 pc − 4。★ 读口必须与写口 br_pc_idx 同口径，
//否则 bht/btb/bti 永远命中不了（详见下面那段注释）。

//flag_bus 译码（行为块，放本模块最前）：**先把各位还原成原名，再按名字做逻辑**（不用位号）。
//adv 与 pc.v 那个 `!flush_w && !stall_w` 是同一个门，两边必须一致，否则"该改向却没交付"。
    reg flush_con_exc, flush_con_irq, flush_con_jump;
    reg stall_rob_full, stall_pc_redir;
    reg stall_lsu_haz, stall_lsu_full;
    reg stall_mulu_haz, stall_mulu_div;
    reg stall_icache_miss, stall_bus_hold;
    reg exec_b, flush_w, stall_w, adv;
    always @(*) begin
        flush_con_exc     = flag_bus[11];
        flush_con_irq     = flag_bus[10];
        flush_con_jump    = flag_bus[9];
        exec_b            = flag_bus[8];
        stall_rob_full    = flag_bus[7];
        stall_pc_redir    = flag_bus[6];
        stall_lsu_haz     = flag_bus[5];
        stall_lsu_full    = flag_bus[4];
        stall_mulu_haz    = flag_bus[3];
        stall_mulu_div    = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        stall_bus_hold    = flag_bus[0];
        flush_w = flush_con_exc | flush_con_irq | flush_con_jump;
//★ 本级在【取指侧】：只吃"重定向排队 + 取指自己 miss"；后端停顿由取指队列吸收。
        stall_w = (stall_pc_redir | stall_icache_miss) & ~flush_w;
        adv     = ~flush_w & ~stall_w & ~fifo_full;
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
//三张表的读 key：**2:1 选择，不加载第二个读口**（同一拍只有一条指令要用预测器）。
//  lane0 是控制转移 ⇒ 查 lane0（key = 当拍 pc）；否则查 lane1（key = pc + 4）。
//★ 行为逐位不变：三张表的输出全都被 lane0 自己的译码门控（br1/jalr/bti 都配 lane0 的类位）。
    reg [31:0] rd_key;
    always @(*) begin
        if (lane0_ct)
            rd_key = pc_addr_in;
        else
            rd_key = pc_addr_in + 32'd4;
    end
    integer i;


//BHT查当前取指PC，饱和计数>1则预测跳转
    always @(*) begin
        if (rst_q)
            predict_en = 1'b0;
        else
            predict_en = (bht[rd_key[8:3]] > 2'd1);
    end

//==================================================================================
// 新的一组：服务 br1/jal 的独立 bti —— 存的是**跳转目标那条指令**（不是落点地址）。
//
// 为什么存 inst：落点地址还是要给 pc（pc 得跳到目标才取得下去）；而存了"目标 inst"之后，
// 跳转那一拍 icache 就**不必被寻址** ⇒ 取指地址 = pc>>>2 纯线选，"预测器出口 → iram 地址口"
// 这条路根本不存在（那正是当前 14.4ns 关键路径的终点）。
//
// 索引/口径：读口用当拍 pc_addr_in；写口在 take 那一拍锁同一批位。预测发生在"这条指令被
// pre_decoder 译码那一拍"（那时 pc 已是 指令地址+4），读写口看到同一个 pc 值，自洽。
// tag 22 位（pc[31:10]）与 8 位索引合起来覆盖 pc[31:2] 全部位 ⇒ 两条不同指令不可能同 tag
// 同 index ⇒ **命中即条目属于这条指令**；jal/br 落点每地址恒定、无自修改代码 ⇒ 存下的 inst
// 恒真，不需要校验（jalr 那套预测+校验是另一张表，别混）。
// 复位只清 bti_v：bti_inst/bti_tag 陈旧无害，带复位会撑爆 slice 打包（icache 的 tag1/tag2 同理）。
//==================================================================================
    (* ram_style = "distributed" *) reg [63:0] bti_inst [0:255];
    (* ram_style = "distributed" *) reg [21:0] bti_tag  [0:255];
    (* ram_style = "distributed" *) reg        bti_v    [0:255];

//★ 索引口径 = "指令地址 + 4"（= 条目的 addr 口径，也 = 写口 br_pc_idx 的口径）。
//  pc 现在是"下一个待取地址"、一次吃两个字 ⇒ 本拍交付的 lane0 是 instr(pc − 8)、它的
//  "指令地址 + 4" 就是 pc − 4。★ 读口与写口必须同口径：老设计里交付那拍 pc 恰为
//  "指令地址+4"，直接拿 pc_addr_in 索引两边就是同值；pc 改 +8 之后若还拿 pc 索引，
//  读比写整整偏一格 ⇒ bht/btb/bti 永远命中不了（实测：pre_jalr=1 而 btb_hit=0，
//  ret 无从改向、一路冲进零区，非法指令跳回 0 把整个程序重跑）。
    reg [7:0]  bti_rd_idx;
    reg [21:0] bti_rd_tag;
    reg        take;
//交付：take 那一拍记下"欠一次交付"，一直保持到 pre_decoder 真的把这条锁进流水线
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
            bti_rd_idx = bti_key[9:2];
            bti_rd_tag = bti_key[31:10];
            bti_hit    = bti_v[bti_rd_idx] & (bti_tag[bti_rd_idx] == bti_rd_tag);
            take       = ct_redir & adv;
        end
    end

//欠交付状态机
    always @(posedge clk) begin
        if (rst_q) begin
            bti_pend    <= 1'b0;
            bti_kind_q  <= 1'b0;
            bti_inst_q  <= 64'd0;
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
        else if (flush_w | (bti_pend && exec_b && adv)) begin
            bti_pend    <= 1'b0;
        end
    end

//交付三态：0 = 用 icache 正常交付；1 = 用 bti 的目标 inst；2 = 交付 NOP
    always @(*) begin
        if (rst_q)
            bti_sel_q = 2'd0;
        else if (bti_pend)
            if (bti_kind_q)
                bti_sel_q = 2'd1;
            else
                bti_sel_q = 2'd2;
        else
            bti_sel_q = 2'd0;
    end

//捕获通路的使能与 index/tag 跟着走两拍。
//★ 停顿（stall_w）只能【冻结】窗口，不能作废：pc_addr 的推进条件就是 `!flush_w && !stall_w`
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
//★ 冻结门必须与取指侧那道门【一致】：队列满时前端与 icache 交付都冻住了，
//  捕获窗口若照常推进，两拍后取到的就不是"目标 inst 交付那一拍" ⇒ 表里存进错的一条
//  （实测 ret_raw：bti 里存成目标的下一条 ⇒ 自环指令的目标指令被替换错 ⇒ pc 跑飞重来）。
        else if (stall_w | fifo_full) begin
            cap_q      <= cap_q;
            cap_idx_q1 <= cap_idx_q1;
            cap_tag_q1 <= cap_tag_q1;
            cap_idx_q2 <= cap_idx_q2;
            cap_tag_q2 <= cap_tag_q2;
        end
        else begin
            cap_q      <= {cap_q[0], take & ~bti_hit & ~ct_jalr};
//★ 捕获键值必须与读口同口径（rd_key），否则 lane1 学不进去（读 pc+4、写 pc ⇒ 钥匙差 4）
            cap_idx_q1 <= bti_key[9:2];
            cap_tag_q1 <= bti_key[31:10];
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
//★ 存【一对连续指令】：[63:32] 是目标那条（lane0）、[31:0] 是它后面那条（lane1）。
//  注入那一拍两条一起进队列、pc 走 +8 —— 注入拍与常态拍的口径这才完全一致，pc（也就是本表的
//  索引）才稳定。单字注入会让 pc 在 T+8/T+4 之间来回跳 ⇒ 自环指令 hit/miss 交替、控制流转圈
//  （实测 smoke 在 0x58→0x60→0x5c 死转）。
                bti_inst[cap_idx_q2] <= {inst_in, inst_next_in};
                bti_tag[cap_idx_q2]  <= cap_tag_q2;
                bti_v[cap_idx_q2]    <= 1'b1;
            end
        end
    end

//组合输出：当前取指PC的预测结果 + 载荷的预测判定
    always @(*) begin
        if (rst_q) begin
            br1 = 1'b0;
            br1_1 = 1'b0;
            br2 = 1'b0;
            br3 = 1'b0;
            jalr = 1'b0;
            jalr_predict_offset = 32'd0;
        end
        else begin
            br1 = br_en & predict_en;
            br1_1 = ~lane0_ct & predict_en;
            br2 = success & !br_pred_taken_in;
            br3 = br_fail & br_pred_taken_in;
            jalr_predict_offset = btb[rd_key[8:3]];
            jalr = btb_v[rd_key[8:3]];
        end
    end

endmodule
