`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: bpu
// Description:
//   前端预测器。两张表，一份学习源，两个读口。
//
//     BHT  : 方向 —— 这条条件分支跳不跳（2 位饱和）。只服务 btb_take 的判据。
//     BTB  : 落点 —— 一条【指令】一项，给出它跳到的绝对地址。
//
//   ★ 学习源只有一个：bju 的判定延迟级（branch_unit_kind / branch_unit_taken / branch_unit_pc / bju_tgt）。
//     - 方向表吃 kind==分支 的 taken；
//     - 落点表吃 taken 的一切跳转（条件分支 / jal / jalr），存的是【事实】不是预测；
//     前端不译 br/jal/jalr：iffu 一旦自己译，它译的就是"会被自己改内容"的那个交付口，成环。
//
//   ★ 读口吃 pc 的两个取指地址，**addr0 的跳转优先级 > addr1**，仲裁在本模块内做完
//     （btb_take_0/1 恒互斥）⇒ pc 只看到"跳不跳 + 跳到哪"，不必再拼一次仲裁。
//
//   ★ 本模块与 icache 的【组合数据通路】零交互。
//
//   ★ 两张阵列都要能例化成 BRAM：
//     - 每张表【只有一条写语句】（两条不同地址的语句会被推成多端口 ⇒ 掉进 LUTRAM）；
//     - 读全部寄存一拍（同步读）；
//     - 落点表备两份阵列（一个写口 + 两条 lane 的读口，一个 BRAM 只有两个口）；
//     - 复位只清输出寄存器，阵列走 initial（Xilinx BRAM 没有复位脚）。
//
//   ★★ 2026-10-29 索引重做（三条一起改，缺一条都留着老错）：
//     ① 两张表都按【一条指令】索引（pc[B+1:2]），不再按 8 字节 pair。老的方向表吃 pc[11:3]，
//        同一个 8 字节块里的两条分支共用一个计数器 —— CoreMark 里这种成对分支正好是热点：
//        0x3b0 beqz(s4)/0x3b4 bnez(s2)、0x1290 beq/0x1294 bltu、0x12f0 beqz/0x12f4 beq，
//        一条常年不跳、一条常年跳，共用一格 ⇒ 后者的 334/432/312 次"漏跳"全是这一格造成的。
//     ② 索引再吃一路【高地址位异或】：index = pc[B+1:2] ^ {pc[B+H+1:B+2], 0}。
//        老的纯低位索引只覆盖 4KB（BHT）/ 4KB（BTB），而正文 0x716c≈29KB，
//        0x4fc 与 0x14fc（只差 bit12）、0x130c 与 0xb08 这类远距离同名格互相踩。
//        异或之后覆盖 2^(B+H+2) 字节；tag 位宽按"索引+tag 能唯一还原 pc"取，
//        两张表都仍是【精确】的（不会凭空假命中）。
//     ③ 学习门放开：错路这条限制本来是想拦住"错路操作数算出来的垃圾"，
//        但落点对 branch/jal 是 pc+imm（译码出来的，与错路无关），方向表的 taken 才是数据算的。
//        实测被 redir_pending 挡掉的训练只占 1~3%，而它把 0x1508 这类"每次都判错的 jal"
//        永久锁在门外（自己判错 ⇒ 自己的改向一直挂着 ⇒ 自己永远训不上）。放开，用实测背书。
//////////////////////////////////////////////////////////////////////////////////


module bpu(
    input clk, rst,
//★★ 唯一前进判据（icache 出）。输出寄存器（落点表那一级）只在这一拍换内容。
//  它和 icache 的 R0 是同一个沿、同一个使能、同一个 pc_addr ⇒ 两边描述的永远是同一对（不变量 I1）。
    input        advance,

//──────────────────────────────────────────────────────────────────────────────
// 学习源：bju 判定延迟级的寄存器输出（同一条指令、同一沿）
//──────────────────────────────────────────────────────────────────────────────
//这条被判定的指令是哪种：0=不训（非跳转 / 被冲刷 / 已吞掉），1=条件分支，2=jal，3=jalr
    input [1:0]  branch_unit_kind,
    input        branch_unit_taken,
//branch_unit_pc  = 那条指令【自己】的地址（不是 pc+4）；bju_tgt = 它真正要去的下一条 pc。
//  条件分支没跳时 bju_tgt 是顺延地址 —— 本模块只在 taken 那一档用它，不会拿它覆盖落点。
    input [31:0] branch_unit_pc, bju_tgt,
//它进流水线时随指令记下的预测（用来出 branch_stage2/branch_stage3）
    input        branch_prediction_taken_in,
//★ 老口 redir_pending（ROB 的在册改向）2026-10-29 整个删掉：训练门不再看它。
//  理由见文件头 ③：它把"自己判错 ⇒ 自己永远训不上"锁死了，而实测它只挡掉 1~3% 的训练。

//──────────────────────────────────────────────────────────────────────────────
// 读口：pc 这一拍要取的两个地址（addr0 优先级 > addr1）
//──────────────────────────────────────────────────────────────────────────────
    input [31:0] pc_addr_0, pc_addr_1,

//──────────────────────────────────────────────────────────────────────────────
// 给 pc：落点（btb_take_0/1 恒互斥）
//──────────────────────────────────────────────────────────────────────────────
    output reg        btb_take_0, btb_take_1,
    output reg [31:0] btb_target_0,  btb_target_1,

//──────────────────────────────────────────────────────────────────────────────
// 给 cont / csr：判定口（"预测错没错"）
//──────────────────────────────────────────────────────────────────────────────
    output reg branch_stage2, branch_stage3
    );

//复位就地打一拍（与其它模块同款）
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//==============================================================================
//  训练源拆解
//==============================================================================
    reg is_branch, is_jump_link, is_jump_link_register, training_target;
    always @(*) begin
        is_branch        = (branch_unit_kind == 2'd1);
        is_jump_link       = (branch_unit_kind == 2'd2);
        is_jump_link_register      = (branch_unit_kind == 2'd3);
//落点表：taken 的一切跳转都训（条件分支没跳时没有落点可存 ⇒ 不许进来）
        training_target    = (branch_unit_kind != 2'd0) & branch_unit_taken;
    end

//★ 2026-10-29：两条写门都不再看 redir_pending（老门只在注释里留个来由）。
//  实测：被它挡掉的训练 226~1018 次/窗口（占训练总量 1~3%），而它挡住的 0x1508（312 次）
//  是"100% 判错的 jal 永远学不会"—— 自己判错 ⇒ 自己挂着改向 ⇒ 自己永远被挡 ⇒ 永远判错。
    wire training_ok = training_target;
    wire bht_ok      =  is_branch;

    always @(*) begin
        branch_stage2 = is_branch &  branch_unit_taken & ~branch_prediction_taken_in;
        branch_stage3 = is_branch & ~branch_unit_taken &  branch_prediction_taken_in;
    end

//==============================================================================
//  索引（两张表同一套形状：一条指令一项 + 高地址位异或）
//    index = pc[B+1:2] ^ {pc[B+H+1:B+2], 0…0}
//  写口用 branch_unit_pc、读口用 pc_addr，两边【必须逐位同式】，否则训练落到别的格
//  （老的全长注释批判过的那类"自己那格永远停在弱不跳、每执行一次错一次"）。
//==============================================================================
    localparam BHT_B = 9;              // 方向表索引位宽 ⇒ 2048 项
    localparam BHT_H = 4;               // 折进索引的高位个数（pc[16:13]）
    localparam BHT_D = (1 << BHT_B);

    localparam BTB_B  = 9;             // 落点表索引位宽 ⇒ 2048 项
    localparam BTB_H  = 4;              // 折进索引的高位个数（pc[16:13]）
    localparam BTB_D  = (1 << BTB_B);
    localparam BTB_TW = 11;              // tag 位宽 pc[21:13]（索引 11 位 + tag 9 位 = pc[21:2]，精确）

    wire [BHT_B-1:0] bht_index_0 = pc_addr_0[BHT_B+1:2]
                                 ^ {pc_addr_0[BHT_B+BHT_H+1:BHT_B+2], {(BHT_B-BHT_H){1'b0}}};
    wire [BHT_B-1:0] bht_index_1 = pc_addr_1[BHT_B+1:2]
                                 ^ {pc_addr_1[BHT_B+BHT_H+1:BHT_B+2], {(BHT_B-BHT_H){1'b0}}};
    wire [BHT_B-1:0] bht_index_w = branch_unit_pc[BHT_B+1:2]
                                 ^ {branch_unit_pc[BHT_B+BHT_H+1:BHT_B+2], {(BHT_B-BHT_H){1'b0}}};

//==============================================================================
//  方向表 BHT：只回答"这条分支跳不跳"。
//  读写同口径：都用【指令自己的地址】（不是 pc+4 —— 加 4 会给 4 mod 8 的分支向 bit3 进位，
//  训练落到下一格，自己那格永远停在弱不跳，这类分支每执行一次错一次）。
//==============================================================================
//★ 组合读（分布式 RAM）：方向位必须和【当拍】那个地址对上。寄存读晚一拍，
//  而晚的这一拍正好是"pred 要随指令进流水线"的那一拍 —— 相位从这一级就开始错。
    (* ram_style = "distributed" *) reg [1:0] bht [0:BHT_D-1];
    wire [1:0] bht_registered_0 = bht[bht_index_0];
    wire [1:0] bht_registered_1 = bht[bht_index_1];
    integer i;

    wire bht_direction_0 = (bht_registered_0 > 2'd1);
    wire bht_direction_1 = (bht_registered_1 > 2'd1);

//==============================================================================
//  落点表 BTB：一条【指令】一项。两张阵列分开，各自进自己能进的存储：
//    ① tag 表 {is_jump, tag[8:0], valid} → 【分布式 RAM（LUTRAM）】，组合读。
//       命中判据在【D 端】用当拍 pc_addr 的索引/tag 比完，与地址同拍出结果。
//    ② 落点表 tgt[31:0]                  → BRAM，寄存读。
//       Q 端出【判好的 pred + 寄存好的落点】，两者同一沿一起寄存 ⇒ 永远配对。
//  ★★ 为什么 tag 不能塞进落点表里：{v,is_jump,tag,tgt} 是 52 位/项的宽表，只能进 BRAM，
//     命中就得到 Q 端才比 —— 整个预测比地址晚一拍，前端所有相位（记录/改向/错路标记）
//     都得靠寄存器去追这一拍。tag 单独拿出来只有 11 位宽，2048 项 = 22Kb，LUTRAM 装得下，
//     命中在地址那一拍就算完。
//  ★ 按指令、不按 pair：改向落点常是 4 mod 8，取指对会跨两条 pair（实测 seed111：
//    交付对 (0x66c,0x670)，按 [13:3] 查到的是 0x668 那条 pair 的落点 ⇒ jal 被拉回它自己的
//    地址、原地打转）。按指令索引就没有"对齐"这回事。
//  ★ 必须带 valid：低地址的 tag 恰好是 0，和阵列复位值撞上 ⇒ 不加这一位就"处处命中、落点 0"。
//  ★ 必须带 is_jump：方向表只吃条件分支，jal/jalr 在里面没有训练项；
//    不单独标出来，无条件跳转就会被读成"方向不跳"。
//  ★ tag 只留 9 位（pc[21:13]）：全 tag 要 20 位，LUTRAM 代价翻倍。索引+tag 覆盖 pc[21:2]，
//    代码 <4MB 时仍然精确、不会有假命中；再大的地址空间会截断出假命中，
//    分支的假落点由 bju 的 br_tgt_fail 兜住（jalr 那头本来就有 jalr_fail）。
//==============================================================================
    wire [BTB_B-1:0]  btb_index_0 = pc_addr_0[BTB_B+1:2]
                                  ^ {pc_addr_0[BTB_B+BTB_H+1:BTB_B+2], {(BTB_B-BTB_H){1'b0}}};
    wire [BTB_B-1:0]  btb_index_1 = pc_addr_1[BTB_B+1:2]
                                  ^ {pc_addr_1[BTB_B+BTB_H+1:BTB_B+2], {(BTB_B-BTB_H){1'b0}}};
    wire [BTB_B-1:0]  btb_index_w = branch_unit_pc[BTB_B+1:2]
                                  ^ {branch_unit_pc[BTB_B+BTB_H+1:BTB_B+2], {(BTB_B-BTB_H){1'b0}}};
    wire [BTB_TW-1:0] btb_tag_0 = pc_addr_0[BTB_B+BTB_TW+1:BTB_B+2];
    wire [BTB_TW-1:0] btb_tag_1 = pc_addr_1[BTB_B+BTB_TW+1:BTB_B+2];
    wire [BTB_TW-1:0] btb_tag_w = branch_unit_pc[BTB_B+BTB_TW+1:BTB_B+2];

//tag 表：{is_jump, tag[8:0], valid}（valid 在 bit0 —— 与写端口逐位对齐，改一侧必须改两侧）
    (* ram_style = "distributed" *) reg [BTB_TW+1:0] btb_target_registered [0:BTB_D-1];
    wire [BTB_TW+1:0] btb_tag_queue_0 = btb_target_registered[btb_index_0];
    wire [BTB_TW+1:0] btb_tag_queue_1 = btb_target_registered[btb_index_1];
    wire btb_hit_0 = btb_tag_queue_0[0] & (btb_tag_queue_0[BTB_TW:1] == btb_tag_0);
    wire btb_hit_1 = btb_tag_queue_1[0] & (btb_tag_queue_1[BTB_TW:1] == btb_tag_1);

//落点表：一个写口 + 两个读口 ⇒ 两份阵列（BRAM 只有两个口）
    (* ram_style = "block" *) reg [31:0] btb_target_ram_0 [0:BTB_D-1];
    (* ram_style = "block" *) reg [31:0] btb_target_ram_1 [0:BTB_D-1];

//==============================================================================
//  ★ 循环预测器 v2（2026-10-29）：只治计数循环的【出口】。
//  2 位饱和计数器对"跑 N 趟都跳、第 N+1 趟不跳"这种结构，每次出口必错一次；这不是别名、也不是容量。
//
//  ★★ v1（计数放在【判定】侧）实测是负的：窗口 310,642 → 323,569（+4.2%）。
//    两条原因都写在这里，别再踩：
//    ① **判定比取指晚 6~8 拍**，而 matrix_mul 的内层循环 body 只有 4 条 ⇒ 取到"出口那一趟"时
//       count 还差一两趟没数上 ⇒ 324 次出口一次都没治到（0xc08/0xcbc 的 325 假跳纹丝不动）。
//    ② 6 位计数在 core_bench_list 的 ~200 趟链上**饱和**，出口时 trip=63，此后每一趟
//       count 都停在 63 == trip ⇒ 趟趟判出口，把本来只错 14 次的 0x60c 弄成 931 次。
//  ⇒ v2 把计数搬到【取指侧】：谁取指谁数，天然没有滞后；判定侧只写"这条循环上次跑了几趟"。
//
//  两张小表、各一个写口：
//    loop_meta_tab  {valid, conf[1:0], tag[7:0], trip[5:0]}   —— 判定侧写
//    loop_count_tab {valid, exit_pred, tag[7:0], count[5:0]} —— 取指侧写；判定侧（出口复位）优先
//  ★ 取指侧只在【新的一次取指】上计数（pc_addr 与上一拍不同）。停顿/背压时同一地址会被问很多拍，
//    不判这个就会一拍拍地重复加。
//  ★ 只有【真正驱动这次取指】的那条 lane 计数：lane0 预测跳了，lane1 那条根本不在取指流里。
//  ★ 出口预测错了（记了 exit_pred，判定却是 taken）⇒ 直接撤掉整条（trip=0、conf=0）。
//    没有这一条，一个错 trip 会每隔 trip 趟再错一次，越错越多。
//  ★ 只有条件分支进这张表：jal/jalr 没有"跳 N 趟然后不跳"这回事。
//==============================================================================
    localparam LOOP_B  = 6;             // 64 项
    localparam LOOP_D  = (1 << LOOP_B);
    localparam LOOP_TW = 8;             // tag = pc[15:8]（索引 pc[7:2] + tag ⇒ 覆盖 pc[15:2]）
    localparam LOOP_CN = 6;             // trip / count 位宽
    localparam META_W  = 1 + 2 + LOOP_TW + LOOP_CN;      // 17
    localparam CNT_W   = 1 + 1 + LOOP_TW + LOOP_CN;      // 16

    wire [LOOP_B-1:0]  lindex_0 = pc_addr_0[LOOP_B+1:2];
    wire [LOOP_B-1:0]  lindex_1 = pc_addr_1[LOOP_B+1:2];
    wire [LOOP_B-1:0]  lindex_w = branch_unit_pc[LOOP_B+1:2];
    wire [LOOP_TW-1:0] ltag_0 = pc_addr_0[LOOP_B+LOOP_TW+1:LOOP_B+2];
    wire [LOOP_TW-1:0] ltag_1 = pc_addr_1[LOOP_B+LOOP_TW+1:LOOP_B+2];
    wire [LOOP_TW-1:0] ltag_w = branch_unit_pc[LOOP_B+LOOP_TW+1:LOOP_B+2];

    (* ram_style = "distributed" *) reg [META_W-1:0] loop_meta_tab  [0:LOOP_D-1];
    (* ram_style = "distributed" *) reg [CNT_W-1:0]  loop_count_tab [0:LOOP_D-1];

//────────────────────────── 取指侧：判出口 ──────────────────────────
    wire [META_W-1:0] lmeta_0 = loop_meta_tab[lindex_0];
    wire [META_W-1:0] lmeta_1 = loop_meta_tab[lindex_1];
    wire [CNT_W-1:0]  lcnt_0  = loop_count_tab[lindex_0];
    wire [CNT_W-1:0]  lcnt_1  = loop_count_tab[lindex_1];

    wire meta_hit_0 = lmeta_0[META_W-1] & (lmeta_0[META_W-4 -: LOOP_TW] == ltag_0);
    wire meta_hit_1 = lmeta_1[META_W-1] & (lmeta_1[META_W-4 -: LOOP_TW] == ltag_1);
    wire cnt_hit_0  = lcnt_0[CNT_W-1] & (lcnt_0[CNT_W-3 -: LOOP_TW] == ltag_0);
    wire cnt_hit_1  = lcnt_1[CNT_W-1] & (lcnt_1[CNT_W-3 -: LOOP_TW] == ltag_1);

    wire [LOOP_CN-1:0] trip_0 = lmeta_0[LOOP_CN-1:0];
    wire [LOOP_CN-1:0] trip_1 = lmeta_1[LOOP_CN-1:0];
    wire [LOOP_CN-1:0] lcnt_val_0 = lcnt_0[LOOP_CN-1:0];
    wire [LOOP_CN-1:0] lcnt_val_1 = lcnt_1[LOOP_CN-1:0];

    wire loop_exit_0 = meta_hit_0 & (lmeta_0[META_W-2:META_W-3] >= 2'd2)
                     & (trip_0 > {{(LOOP_CN-2){1'b0}}, 2'd1})
                     & cnt_hit_0 & (lcnt_val_0 == trip_0);
    wire loop_exit_1 = meta_hit_1 & (lmeta_1[META_W-2:META_W-3] >= 2'd2)
                     & (trip_1 > {{(LOOP_CN-2){1'b0}}, 2'd1})
                     & cnt_hit_1 & (lcnt_val_1 == trip_1);

//────────────────── 取指侧：数这一趟（一条写口，lane0 优先）──────────────────
//"新的一次取指"：地址变了才算一次。停顿期间同一地址会被问很多拍。
    reg [31:0] fetch_pc_prev_0, fetch_pc_prev_1;
    wire fetch_new_0 = (pc_addr_0 != fetch_pc_prev_0);
    wire fetch_new_1 = (pc_addr_1 != fetch_pc_prev_1);

    wire lane0_take_now = btb_hit_0 & (btb_tag_queue_0[BTB_TW+1] | (bht_direction_0 & ~loop_exit_0));
    wire lane0_count = fetch_new_0 & meta_hit_0;
    wire lane1_count = fetch_new_1 & meta_hit_1 & ~lane0_take_now & ~lane0_count;

    wire [LOOP_B-1:0]  count_wr_index = lane0_count ? lindex_0 : lindex_1;
    wire [LOOP_TW-1:0] count_wr_tag   = lane0_count ? ltag_0   : ltag_1;
    wire               count_wr_exit  = lane0_count ? loop_exit_0 : loop_exit_1;
    wire [LOOP_CN-1:0] count_wr_value = lane0_count
                                      ? ((loop_exit_0 | (lcnt_val_0 == {LOOP_CN{1'b1}})) ? {LOOP_CN{1'b0}}
                                                                                        : lcnt_val_0 + 1'b1)
                                      : ((loop_exit_1 | (lcnt_val_1 == {LOOP_CN{1'b1}})) ? {LOOP_CN{1'b0}}
                                                                                        : lcnt_val_1 + 1'b1);

//────────────────────────── 判定侧：记趟数 ──────────────────────────
    wire [META_W-1:0] lmeta_w = loop_meta_tab[lindex_w];
    wire meta_hit_w = lmeta_w[META_W-1] & (lmeta_w[META_W-4 -: LOOP_TW] == ltag_w);
    wire [LOOP_CN-1:0] trip_w = meta_hit_w ? lmeta_w[LOOP_CN-1:0] : {LOOP_CN{1'b0}};
    wire [1:0]         conf_w = meta_hit_w ? lmeta_w[META_W-2:META_W-3] : 2'd0;

    wire [CNT_W-1:0]  lcnt_w  = loop_count_tab[lindex_w];
    wire cnt_hit_w = lcnt_w[CNT_W-1] & (lcnt_w[CNT_W-3 -: LOOP_TW] == ltag_w);
    wire [LOOP_CN-1:0] cnt_w  = cnt_hit_w ? lcnt_w[LOOP_CN-1:0] : {LOOP_CN{1'b0}};
    wire exit_pred_w = cnt_hit_w & lcnt_w[CNT_W-2];

    wire loop_judge = (branch_unit_kind == 2'd1);
    wire loop_exit_now = loop_judge & ~branch_unit_taken;      // 真的走到出口
    wire loop_wrong_exit = loop_judge & branch_unit_taken & exit_pred_w;  // 猜"出口"猜错了

    reg [LOOP_CN-1:0] meta_trip_next;
    reg [1:0]         meta_conf_next;
    always @(*) begin
        meta_trip_next = trip_w;
        meta_conf_next = conf_w;
        if (loop_wrong_exit) begin
//猜错的出口：这条循环的趟数根本不是 trip ⇒ 整条撤掉，等重新攒够两次一致的出口再信。
            meta_trip_next = {LOOP_CN{1'b0}};
            meta_conf_next = 2'd0;
        end
        else if (loop_exit_now) begin
            if (meta_hit_w & (cnt_w == trip_w)) begin
                if (conf_w != 2'd3) meta_conf_next = conf_w + 1'b1;
            end
            else begin
                meta_trip_next = (cnt_w > {{(LOOP_CN-2){1'b0}}, 2'd1}) ? cnt_w : {LOOP_CN{1'b0}};
                meta_conf_next = 2'd0;
            end
        end
    end

    integer loop_bi;
    initial begin
        for (loop_bi = 0; loop_bi < LOOP_D; loop_bi = loop_bi + 1) begin
            loop_meta_tab[loop_bi]  = {META_W{1'b0}};
            loop_count_tab[loop_bi] = {CNT_W{1'b0}};
        end
    end

//★ addr0 优先级 > addr1：lane0 预测跳了，lane1 就在错路上，它的预测一律作废。
//  仲裁在这里做完（不再让 pc 拼第二遍）—— pc 只看到"跳不跳 + 跳到哪"。
//★ 循环预测器只压在【条件分支的方向】上：jal/jalr 照旧只看 BTB 的 is_jump；
//  条件分支在"这一趟是出口"时把 taken 压成 not-taken。
    wire take_candidate_0 = btb_hit_0 & (btb_tag_queue_0[BTB_TW+1] | (bht_direction_0 & ~loop_exit_0));
    wire take_candidate_1 = btb_hit_1 & (btb_tag_queue_1[BTB_TW+1] | (bht_direction_1 & ~loop_exit_1)) & ~take_candidate_0;



//Q 端：判好的 pred 与寄存好的落点【同一沿、同一个地址】⇒ 天然配对
    always @(posedge clk) begin
        if (reset_registered) begin
            btb_take_0 <= 1'b0; btb_take_1 <= 1'b0;
            btb_target_0  <= 32'd0; btb_target_1 <= 32'd0;
        end
        else if (advance) begin
            btb_take_0 <= take_candidate_0;
            btb_take_1 <= take_candidate_1;
            btb_target_0  <= btb_target_ram_0[btb_index_0];
            btb_target_1  <= btb_target_ram_1[btb_index_1];
        end
    end

    integer bi;
    initial begin
        for (bi = 0; bi < BTB_D; bi = bi + 1) begin
            btb_target_registered[bi] = {(BTB_TW+2){1'b0}};
            btb_target_ram_0[bi] = 32'd0;
            btb_target_ram_1[bi] = 32'd0;
        end
    end

//==============================================================================
//  两张表各自的写语句（一条一个，阵列才推得成存储器）
//==============================================================================
//★ 阵列不参与复位：在 always 里整表回到初值 = 每个触发器带同步复位，LUTRAM 就推不出来
//  （模块头那一条"复位只清输出寄存器、阵列走 initial"说的就是这个）。初值放到 initial 里。
    initial begin
        for (i = 0; i < BHT_D; i = i + 1)
            bht[i] = 2'd1;                       // 弱不跳转
    end

    always @(posedge clk) begin
        if (!reset_registered) begin
//方向表：按指令自己的地址加减（晚一拍更新不改变正确性）
            if (bht_ok & branch_unit_taken) begin
                if (bht[bht_index_w] < 2'd3)
                    bht[bht_index_w] <= bht[bht_index_w] + 2'd1;
            end
            else if (bht_ok & ~branch_unit_taken) begin
                if (bht[bht_index_w] > 2'd0)
                    bht[bht_index_w] <= bht[bht_index_w] - 2'd1;
            end
//tag 表：{is_jump, tag[8:0], valid} —— 布局与读端逐位对齐（valid 在 bit0）
            if (training_ok) begin
                btb_target_registered[btb_index_w] <= {(is_jump_link | is_jump_link_register), btb_tag_w, 1'b1};
                btb_target_ram_0[btb_index_w] <= bju_tgt;
                btb_target_ram_1[btb_index_w] <= bju_tgt;
            end
//循环表：一条写语句、一个写口（读改写都在这一条里 ⇒ 分布式 RAM 推得出来）
//循环表 v2：两张表各一条写语句（各一个写口），否则推不成分布式 RAM。
//  meta 只在【条件分支判定】那一拍写；count 在取指侧写，判定侧的"出口复位"优先。
            if (loop_wrong_exit | loop_exit_now)
                loop_meta_tab[lindex_w] <= {1'b1, meta_conf_next, ltag_w, meta_trip_next};
            if (loop_exit_now)
                loop_count_tab[lindex_w] <= {1'b1, 1'b0, ltag_w, {LOOP_CN{1'b0}}};
            else if (lane0_count | lane1_count)
                loop_count_tab[count_wr_index] <= {1'b1, count_wr_exit, count_wr_tag, count_wr_value};
//"上一拍被问的地址"，用来判"这是不是新的一次取指"
            fetch_pc_prev_0 <= pc_addr_0;
            fetch_pc_prev_1 <= pc_addr_1;
        end
    end

endmodule
