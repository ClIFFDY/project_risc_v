`timescale 1ns / 1ps
//==============================================================================
// rename.v —— 重命名：架构号 x0..x31 → 物理号
//
// 三样东西：
//   rat[]        推测态映射。idu 那一拍（指令被分配物理号那一拍）就改。
//   arch_map[]   架构态映射。只有【提交口】改，冲刷不碰它。
//   free_vector  物理号空闲位向量，1 = 空闲。
//
// 为什么要有两份映射：冲刷时 `rat <= arch_map` 整片抄回去，**一拍恢复、不需要快照、
//   也不需要倒着走表**。arch_map 只被已退休的指令改过，所以它天然就是"冲刷边界那一刻"的样子。
//
// bank：物理号最低位 = bank 位。lane0 只从偶数号取、lane1 只从奇数号取
//   ⇒ 两条 lane 的结果口天然落在不同的 bank 上，每 bank 一个写口。
//   访存条也都从 lane0 走，所以 LD 结果也在 bank0（这一点在 PRF 那边再处理写口合并）。
//
// 物理 0..ARCH_COUNT-1 是复位的架构映射（x0..x31），**不参与分配**；
//   上面那些才是空闲池。x0 钉死在物理 0：写 x0 的指令不发号，rat[0] 永远不动。
//==============================================================================
module rename #(
    parameter integer ARCH_COUNT = 32,
    parameter integer PHYS_COUNT = 64,
    parameter integer PHYS_BITS  = 6
)(
    input clk, rst,

//---- 重命名（idu 那一拍），两条 lane ----
    input                 alloc_en_0,
    input                 alloc_we_0,          // 这一条有写回（jal/load/csr 都算）
    input  [4:0]          alloc_rd_0,
    input  [4:0]          alloc_rs1_0, alloc_rs2_0,
    output [PHYS_BITS-1:0] phys_rd_0, phys_rs1_0, phys_rs2_0,
    input                 alloc_en_1,
    input                 alloc_we_1,
    input  [4:0]          alloc_rd_1,
    input  [4:0]          alloc_rs1_1, alloc_rs2_1,
    output [PHYS_BITS-1:0] phys_rd_1, phys_rs1_1, phys_rs2_1,
//★ 2026-11-08：口 2/3 加宽（保留站每拍每 lane 收两条）。age 次序仍是 0 最老、3 最年轻；
//  bank 归属不变 —— 偶数口 0/2 取偶数 bank，奇数口 1/3 取奇数 bank。
    input                 alloc_en_2,
    input                 alloc_we_2,
    input  [4:0]          alloc_rd_2,
    input  [4:0]          alloc_rs1_2, alloc_rs2_2,
    output [PHYS_BITS-1:0] phys_rd_2, phys_rs1_2, phys_rs2_2,
    input                 alloc_en_3,
    input                 alloc_we_3,
    input  [4:0]          alloc_rd_3,
    input  [4:0]          alloc_rs1_3, alloc_rs2_3,
    output [PHYS_BITS-1:0] phys_rd_3, phys_rs1_3, phys_rs2_3,
//本 lane 所属 bank 已经没号了：这一条不许再往队列里放
    output                alloc_stall_0, alloc_stall_1, alloc_stall_2, alloc_stall_3,
    output                bank_empty_0, bank_empty_1, bank_empty_2, bank_empty_3,

//---- 通用查找口：按架构号给出【当前的】物理号（消费者就是靠这个拿到生产者的号）----
//  用被推入队列那条指令【自己的】rs 字段驱动，不要用别的相位上凑出来的 rs。
    input  [4:0]          lookup_a, lookup_b, lookup_c, lookup_d, lookup_e, lookup_f,
    output [PHYS_BITS-1:0] lookup_a_phys, lookup_b_phys, lookup_c_phys, lookup_d_phys,
                           lookup_e_phys, lookup_f_phys,


//---- 提交口（rob 队头，口 A 更老；物理号跟着 ROB 项走）----
    input                 commit_we_a, input [4:0] commit_rd_a, input [PHYS_BITS-1:0] commit_phys_a,
    input                 commit_we_b, input [4:0] commit_rd_b, input [PHYS_BITS-1:0] commit_phys_b,

//---- 冲刷 / 恢复：都在提交口发 ----
    input                 flush,
//★ 2026-11-09：squash_mask 端口删。回收一直走下面的 flush_free_mask = ~arch_next_mask，
//  这个口只剩一句"保留作交叉检查"的注释、代码里一次没读过（ROB 侧为它白造 DEPTH×64 译码器）。
//★ 2026-11-05 NaxRiscv 口径：改向一经判定就把分配冻住，rat 在冻住期间没人读
//  ⇒ 回滚退化成"整片抄回 arch_map"，不需要按架构号挑最老被杀项。
//  restore = 那条分支自己提交的那一拍：flush 和 restore 对这里做的事完全相同。
    input                 restore,

//---- 给 PRF 的就绪位用 ----
    output [PHYS_COUNT-1:0] alloc_onehot,        // 本拍分配出去的号
    output [PHYS_COUNT-1:0] arch_next_onehot     // 冲刷后仍在用的号（= 落完提交的架构映射）
    );

//复位就地打一拍（全核统一的做法）
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

    reg [PHYS_BITS-1:0] rat      [0:ARCH_COUNT-1];
    reg [PHYS_BITS-1:0] arch_map [0:ARCH_COUNT-1];
    reg [PHYS_COUNT-1:0] free_vector;

//偶数位掩码 / 奇数位掩码 = 两个 bank
    localparam [PHYS_COUNT-1:0] EVEN_MASK = {PHYS_COUNT/2{2'b01}};
    localparam [PHYS_COUNT-1:0] ODD_MASK  = {PHYS_COUNT/2{2'b10}};

    wire [PHYS_COUNT-1:0] free_even = free_vector & EVEN_MASK;
    wire [PHYS_COUNT-1:0] free_odd  = free_vector & ODD_MASK;

//这一条要不要发号：有写回、且不是 x0
    wire need_0 = alloc_en_0 & alloc_we_0 & (alloc_rd_0 != 5'd0);
    wire need_1 = alloc_en_1 & alloc_we_1 & (alloc_rd_1 != 5'd0);
    wire need_2 = alloc_en_2 & alloc_we_2 & (alloc_rd_2 != 5'd0);
    wire need_3 = alloc_en_3 & alloc_we_3 & (alloc_rd_3 != 5'd0);

//每个 bank 每拍要发【两个】号（口 0/2 取偶数 bank，口 1/3 取奇数 bank）。
//  第一轮照旧取最低空闲位；把它从空闲向量里抠掉再取一次，就是第二个号。
//  倒序 for 让最低位最后赋值 ⇒ 低位优先（与原写法同一个技巧）。
    reg [PHYS_BITS-1:0]   pick_e0, pick_e1, pick_o0, pick_o1;
    reg [PHYS_COUNT-1:0]  pick_e0_onehot, pick_e1_onehot, pick_o0_onehot, pick_o1_onehot;
    reg                   found_e0, found_e1, found_o0, found_o1;
    reg [PHYS_COUNT-1:0]  free_even_2nd, free_odd_2nd;
    integer k;
    always @(*) begin
        pick_e0 = {PHYS_BITS{1'b0}}; pick_e0_onehot = {PHYS_COUNT{1'b0}}; found_e0 = 1'b0;
        pick_o0 = {PHYS_BITS{1'b0}}; pick_o0_onehot = {PHYS_COUNT{1'b0}}; found_o0 = 1'b0;
        for (k = PHYS_COUNT - 1; k >= 0; k = k - 1) begin
            if (free_even[k]) begin
                pick_e0        = k;
                pick_e0_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_e0       = 1'b1;
            end
            if (free_odd[k]) begin
                pick_o0        = k;
                pick_o0_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_o0       = 1'b1;
            end
        end
        free_even_2nd = free_even & ~pick_e0_onehot;
        free_odd_2nd  = free_odd  & ~pick_o0_onehot;
        pick_e1 = {PHYS_BITS{1'b0}}; pick_e1_onehot = {PHYS_COUNT{1'b0}}; found_e1 = 1'b0;
        pick_o1 = {PHYS_BITS{1'b0}}; pick_o1_onehot = {PHYS_COUNT{1'b0}}; found_o1 = 1'b0;
        for (k = PHYS_COUNT - 1; k >= 0; k = k - 1) begin
            if (free_even_2nd[k]) begin
                pick_e1        = k;
                pick_e1_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_e1       = 1'b1;
            end
            if (free_odd_2nd[k]) begin
                pick_o1        = k;
                pick_o1_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_o1       = 1'b1;
            end
        end
    end

//★ 口 0 没要号时口 2 直接顶上【第一个】号：不让它空等"第二个"。
    wire alloc_ok_0 = need_0 & found_e0;
    wire alloc_ok_1 = need_1 & found_o0;
    wire [PHYS_BITS-1:0]  pick_b0_1 = alloc_ok_0 ? pick_e1 : pick_e0;
    wire [PHYS_BITS-1:0]  pick_b1_1 = alloc_ok_1 ? pick_o1 : pick_o0;
    wire [PHYS_COUNT-1:0] pick_b0_1_onehot = alloc_ok_0 ? pick_e1_onehot : pick_e0_onehot;
    wire [PHYS_COUNT-1:0] pick_b1_1_onehot = alloc_ok_1 ? pick_o1_onehot : pick_o0_onehot;
    wire                  found_b0_1 = alloc_ok_0 ? found_e1 : found_e0;
    wire                  found_b1_1 = alloc_ok_1 ? found_o1 : found_o0;
    wire alloc_ok_2 = need_2 & found_b0_1;
    wire alloc_ok_3 = need_3 & found_b1_1;

    assign alloc_stall_0 = need_0 & ~found_e0;
    assign alloc_stall_1 = need_1 & ~found_o0;
    assign alloc_stall_2 = need_2 & ~found_b0_1;
    assign alloc_stall_3 = need_3 & ~found_b1_1;
//★ 纯状态：本 bank 已经没有空号。给前端做保守门（与 alloc_en 无组合环），
//  防止"发了号但 ROB 那边没接"⇒ 号漏光 ⇒ 输出物理 0 ⇒ 架构映射被写成 x0 那一格。
    assign bank_empty_0 = ~found_e0;
    assign bank_empty_1 = ~found_o0;
//口 2/3 的"空"是"只剩一个号了"，因为一条进来还要给它留一个。
    assign bank_empty_2 = ~found_b0_1;
    assign bank_empty_3 = ~found_b1_1;

//---- 源操作数：查 rat，带【只对更老的写】旁路 ----
//  ★ 自己这一条的写不能旁路到自己的读：`add x5, x5, x1` 的源必须是旧 x5。
//    lane0 最老 ⇒ 看不到任何本拍写；lane1 只看到 lane0 的写。
    assign phys_rs1_0 = rat[alloc_rs1_0];
    assign phys_rs2_0 = rat[alloc_rs2_0];
    assign phys_rs1_1 = (need_0 & (alloc_rd_0 == alloc_rs1_1)) ? pick_e0 : rat[alloc_rs1_1];
    assign phys_rs2_1 = (need_0 & (alloc_rd_0 == alloc_rs2_1)) ? pick_e0 : rat[alloc_rs2_1];
//★ 口 2 看得见口 0/1 的写，口 3 看得见口 0/1/2 的写；同一个 rs 上更年轻的那个赢。
//  （门用 alloc_ok：没抢到号的那条根本不会写 rat，拿它的号去旁路是错的。）
    assign phys_rs1_2 = (alloc_ok_1 & (alloc_rd_1 == alloc_rs1_2)) ? pick_o0
                      : (alloc_ok_0 & (alloc_rd_0 == alloc_rs1_2)) ? pick_e0
                      : rat[alloc_rs1_2];
    assign phys_rs2_2 = (alloc_ok_1 & (alloc_rd_1 == alloc_rs2_2)) ? pick_o0
                      : (alloc_ok_0 & (alloc_rd_0 == alloc_rs2_2)) ? pick_e0
                      : rat[alloc_rs2_2];
    assign phys_rs1_3 = (alloc_ok_2 & (alloc_rd_2 == alloc_rs1_3)) ? pick_b0_1
                      : (alloc_ok_1 & (alloc_rd_1 == alloc_rs1_3)) ? pick_o0
                      : (alloc_ok_0 & (alloc_rd_0 == alloc_rs1_3)) ? pick_e0
                      : rat[alloc_rs1_3];
    assign phys_rs2_3 = (alloc_ok_2 & (alloc_rd_2 == alloc_rs2_3)) ? pick_b0_1
                      : (alloc_ok_1 & (alloc_rd_1 == alloc_rs2_3)) ? pick_o0
                      : (alloc_ok_0 & (alloc_rd_0 == alloc_rs2_3)) ? pick_e0
                      : rat[alloc_rs2_3];
    assign phys_rd_0  = pick_e0;
    assign phys_rd_1  = pick_o0;
    assign phys_rd_2  = pick_b0_1;
    assign phys_rd_3  = pick_b1_1;

//---- 通用查找口 ----
//  ★ 查找口看到的必须是【本拍分配之前】的映射：一条 `add x5,x5,x1` 的 rs 读的是旧 x5。
//    所以这里只查 rat，不做任何本拍写的旁路。
//  a/b 那一组挂的是 lane0 自己（最老），没有更老的写要旁路；
//  c/d 那一组可能落在 lane1 上，必须看见同拍 lane0 的分配，否则 `add x5,..` / `add ..,x5` 连发会拿到旧号。
    wire bypass_0 = alloc_ok_0 & (alloc_rd_0 != 5'd0);
    assign lookup_a_phys = rat[lookup_a];
    assign lookup_b_phys = rat[lookup_b];
    assign lookup_c_phys = (bypass_0 & (alloc_rd_0 == lookup_c)) ? pick_e0 : rat[lookup_c];
    assign lookup_d_phys = (bypass_0 & (alloc_rd_0 == lookup_d)) ? pick_e0 : rat[lookup_d];
//★ 2026-11-09 三宽分配：第三条的查找口。它看得见口 0/1 的写（口 1 更新，赢）。
    assign lookup_e_phys = (alloc_ok_1 & (alloc_rd_1 == lookup_e)) ? pick_o0
                         : (alloc_ok_0 & (alloc_rd_0 == lookup_e)) ? pick_e0
                         : rat[lookup_e];
    assign lookup_f_phys = (alloc_ok_1 & (alloc_rd_1 == lookup_f)) ? pick_o0
                         : (alloc_ok_0 & (alloc_rd_0 == lookup_f)) ? pick_e0
                         : rat[lookup_f];


//---- 提交口要释放的旧号 ----
//  `old = arch_map[rd]` 现读现用，不在 ROB 里另存 old_phys。
//  同拍两条写同一个 rd：后一条（B）释放的是 A 刚定格进去的号，不是原来的那个。
    wire old_valid_a = commit_we_a & (commit_rd_a != 5'd0);
    wire old_valid_b = commit_we_b & (commit_rd_b != 5'd0);
    wire same_rd     = old_valid_a & old_valid_b & (commit_rd_a == commit_rd_b);

    wire [PHYS_BITS-1:0] old_a = arch_map[commit_rd_a];
    wire [PHYS_BITS-1:0] old_b = same_rd ? commit_phys_a : arch_map[commit_rd_b];

    wire [PHYS_COUNT-1:0] freed_a_onehot = old_valid_a ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << old_a) : {PHYS_COUNT{1'b0}};
    wire [PHYS_COUNT-1:0] freed_b_onehot = old_valid_b ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << old_b) : {PHYS_COUNT{1'b0}};
    wire [PHYS_COUNT-1:0] freed_mask     = freed_a_onehot | freed_b_onehot;

    wire [PHYS_COUNT-1:0] alloc_mask = (alloc_ok_0 ? pick_e0_onehot   : {PHYS_COUNT{1'b0}})
                                     | (alloc_ok_1 ? pick_o0_onehot   : {PHYS_COUNT{1'b0}})
                                     | (alloc_ok_2 ? pick_b0_1_onehot : {PHYS_COUNT{1'b0}})
                                     | (alloc_ok_3 ? pick_b1_1_onehot : {PHYS_COUNT{1'b0}});
    assign alloc_onehot = alloc_mask;

//★ 冲刷后【唯一还在用】的物理号就是 arch_map 里那 32 个（在飞的项全被作废了）。
//  所以空闲表直接取补集 —— 按构造正确，不会漏号。
    reg [PHYS_COUNT-1:0] arch_next_mask;
    integer m;
    always @(*) begin
        arch_next_mask = {PHYS_COUNT{1'b0}};
        for (m = 0; m < ARCH_COUNT; m = m + 1) begin
            if      (old_valid_b && (commit_rd_b == m)) arch_next_mask = arch_next_mask | ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << commit_phys_b);
            else if (old_valid_a && (commit_rd_a == m)) arch_next_mask = arch_next_mask | ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << commit_phys_a);
            else                                        arch_next_mask = arch_next_mask | ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << arch_map[m]);
        end
    end
    wire [PHYS_COUNT-1:0] flush_free_mask = ~arch_next_mask;
    assign arch_next_onehot = arch_next_mask;

    integer i;
    always @(posedge clk) begin
        if (reset_registered) begin
            for (i = 0; i < ARCH_COUNT; i = i + 1) begin
                rat[i]      <= i;
                arch_map[i] <= i;
            end
//0..ARCH_COUNT-1 被架构映射占着，上面那些才是空闲池
            free_vector <= ~({{(PHYS_COUNT-ARCH_COUNT){1'b0}}, {ARCH_COUNT{1'b1}}});
        end
        else if (flush | restore) begin
//一拍恢复：整片抄回 + 把在册的号收回来。
//★ 冲刷拍【常常同时压着一笔打拍后的提交】：rob.v 明确写了"打拍里那笔在边界之前，清了会丢一笔合法写"。
//★ 2026-11-05：改向判定后分配一直冻到那条分支提交（rob 的 redirect_stop_alloc）

            for (i = 0; i < ARCH_COUNT; i = i + 1) begin
                if      (old_valid_b && (commit_rd_b == i)) rat[i] <= commit_phys_b;
                else if (old_valid_a && (commit_rd_a == i)) rat[i] <= commit_phys_a;
                else                                        rat[i] <= arch_map[i];
            end
            if (old_valid_a) arch_map[commit_rd_a] <= commit_phys_a;
            if (old_valid_b) arch_map[commit_rd_b] <= commit_phys_b;
//空闲表按构造写：架构映射占着的之外全可用。
            free_vector <= flush_free_mask;
        end
        else begin
//分配：同一个 rd 被同拍多条写时，最年轻的那条赢（按 age 次序赋值，后写覆盖先写）
            if (alloc_ok_0) rat[alloc_rd_0] <= pick_e0;
            if (alloc_ok_1) rat[alloc_rd_1] <= pick_o0;
            if (alloc_ok_2) rat[alloc_rd_2] <= pick_b0_1;
            if (alloc_ok_3) rat[alloc_rd_3] <= pick_b1_1;

//提交：定格架构映射
            if (old_valid_a) arch_map[commit_rd_a] <= commit_phys_a;
            if (old_valid_b) arch_map[commit_rd_b] <= commit_phys_b;

            free_vector <= (free_vector & ~alloc_mask) | freed_mask;
        end
    end

endmodule
