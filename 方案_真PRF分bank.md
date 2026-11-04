# 真 PRF 分 bank 方案

日期：2026-11-04
前置：exu 已落地，每 lane 只剩一个值表写口 ⇒ 值表写口 5 → 3（lane0 / lane1 / LD）。

## 1. 现在的三套值存储

| 存储 | 尺寸 | 写口 | 读口 | 索引 |
|---|---|---|---|---|
| `regfile` | 32 × 32 FF | 2（提交口 A/B） | 6（组合） | 架构号 0..31 |
| `value_table`（idu2） | 16 × 32 FF | 3（lane0/lane1/LD 结果口） | 6（组合） | **ROB 槽号** |
| `queue_base_a/b_*`（idu2） | 每条目 2 × 32，lane0/lane1/mem 三条队列各 8 条 | 提交口广播 | — | — |

操作数选择：`producer_valid ? value_table[producer_slot] : queue_base`。

`queue_base` 是"不重命名"的补丁：每条站内条目自己存一份基准值，靠提交口
`(destination_register == rs) & (slot == commit_slot) & (gen == commit_gen)` 的 32 位比较广播刷新。
1000 条唯一端点里 `queue_base_*` 占了约 137 条。

## 2. 真 PRF 的目标形态

- 物理寄存器数 `N = ROB_DEPTH + 32 = 48`（ROB_DEPTH=16）。
- 重命名在 idu1 入队时做：有写回的指令从空闲表 pop 一个物理号；重命名表 `RAT[rd]` 指向它。
- 消费者记物理号，不记 (slot, gen)。就绪用一张 `prf_ready[N]` 位表。
- 写口 3 个，写的是物理号；读口 6 个，按物理号读。
- 提交：`RAT` 定格，该物理号的原持有者进空闲表。
- 冲刷：`RAT` 回滚 + 物理号回收（按分配序号，不按位置）。

`queue_base_*` 与提交口广播整套删掉。

## 3. FPGA 的硬约束（决定 bank 怎么分）

- **分布式 RAM（LUTRAM）每个阵列只有 1 个写口。** 多写口推不出来，会退回 FF。
- **块 RAM（RAMB36，真双口）有 2 个写口，但总共只有 2 个端口，读写共用。** 6 个组合读口放不下 ⇒ 块 RAM 排除。
- **FF 阵列写口不限**，但每个读口要一棵 `N:1` 的 32 位 mux。N=48 时 6 棵 48:1，面积和布线都很难看（现在 16 深就已经是"512 FF + 6 棵 16:1x32，route 79%"）。

⇒ 想靠 LUTRAM 落地，**必须把每个 bank 压到 1 个写口**。

## 4. 三条可行的 bank 划分

### A. 按产生者分 3 份（每个写源一份）
- 空闲表分三份：lane0 / lane1 / LD 各 16 个物理号。
- 每份 1 个写口 ⇒ 3 块 LUTRAM，各 16 × 32 = 512 位。
- 读：物理号高的 2 位直接指出 bank，3:1 mux（一级 LUT）。
- **代价**：空闲表三分 ⇒ 某一份用光而别的份还有剩，就得停发那条 lane 的写回类指令。偏斜会咬人。

### B. 按产生者分 2 份（lane0 / lane1），LD 结果借道
- LD 结果本来就从 lsu 单口回来；把它并到**发射它的那条 lane** 的写口上（ld 从哪条 lane 发就写哪份），
  两条 lane 同拍都有 LD 结果时挂一拍（LD 吞吐本来就 ≤1/拍，实际不会撞）。
- 每份 1 个写口 ⇒ 2 块 LUTRAM。
- **代价**：lane 侧要知道"这笔 LD 属于哪条 lane"，要随 lsu 条目带一位。

### C. 统一收成 2 个写口
- 在 exu 之后再加一级全局合并，把 3 口压成 2 口，冲突时进影子。
- 每份 2 个写口 ⇒ LUTRAM 推不出来，只能 FF 阵列 ⇒ 没解决读 mux 的问题。
- **不推荐。**

## 5. 结论（2026-11-04 已定）

**bank 位 = lane 位。空闲表按 lane 分成两份，lane0 只从 bank0 取号、lane1 只从 bank1 取号。**

这样写源自然落回两条：
- bank0 的写口 = {lane0 的 alu/mul/div 结果，**bank 位为 0 的 LD 结果**}，2 选 1 合并后 1 个写口；
- bank1 的写口 = {lane1 的 alu/mul/div 结果，**bank 位为 1 的 LD 结果**}，同上。

LD 结果不需要额外带 lane 位 —— **它的物理号本身就带 bank 位**，直接按那一位送到对应 bank 的端口。
两条 lane 同拍都有 LD 结果不会发生（lsu 是共享单口）。撞拍时 LD 结果挂住（它本来就等 stage3）。

读口：6 个读口各读一个物理号，bank 位选出 2 个 LUTRAM 输出里的一个，一级 2:1 mux。

两块阵列各 16 × 32 位（N=48 取 48，每 bank 24；按 2 的幂取 32 也行），**每块 1 个写口 ⇒ 推得成 LUTRAM**。

**代价**：空闲表两份会有偏斜 —— 某条 lane 的池空了而另一条还有剩。本核每拍最多 2 条在飞、
每条 lane 在飞数远小于 24，偏斜可控；真咬人再开"跨 bank 借用"。

## 6. 新出来的那块硬骨头：RAT 回滚

现在没有重命名表，冲刷靠 `(slot, gen)` 的世代位，**没有状态要回滚**。
上了 RAT 之后，冲刷必须把 `RAT[rd]` 恢复到冲刷边界那一刻的映射，并把边界之上分出去的物理号还回空闲表。

### 三条路，账算下来只有第三条对

| 方案 | 存储 | 回滚拍数 | 备注 |
|---|---|---|---|
| 每分支一份整表快照 | 192 FF × 在飞分支数，最多 16 份 = **3072 FF** | 0 | ROB 16 深就是这个上限 |
| 每条 ROB 项存 `old_phys`，冲刷走表 | 16 × 6 = 96 FF，但 ROB 要加任意下标读口（rd / old_phys 各一棵 16:1） | **最长 16 拍** | 前端这 16 拍不能重新分配 |
| **架构映射表 + 向量空闲表** | 192 + 48 = **240 FF** | **0** | 见下 |

走表那 16 拍的代价比省下的 96 FF 大得多（CoreMark 约 5,000 次误预测，每次十几拍）。

### 定案：架构映射表 + 向量空闲表

- `rat[32]`（推测态，分配拍改）+ `arch_map[32]`（架构态，提交拍改，2 个写口）。
  冲刷时 `rat <= arch_map`，**一拍恢复**。
- 空闲表用 **48 位向量**，不用链表：
  - 分配 = 向量里找第一个 0。按 8 位一组拆 + 汇总向量（每组的"有没有空"再一棵 6 位优先级编码），
    避免 48 位的大优先级编码器。
  - 冲刷回收 = 把边界之上那些项的物理号送一个 48 位 one-hot，整列 **OR 归约**，同样一拍。
    不需要走表，也不需要 `old_phys`。
- 这样 `old_phys` 那 16×6 FF 和整套回滚状态机都不用写。

## 7. 落地顺序

1. `rename.v`：`rat` / `arch_map` / 向量空闲表 + 分配 + 提交定格/释放 + 冲刷一拍恢复。先不接进数据通路。
2. `value_table` 按 bank 拆两块、写口各 1 个，读者改成"物理号 + bank 位选"。
3. `queue_base_*` 与提交口广播整套删掉，操作数选择改成 `prf_ready[phys] ? prf[phys] : …`。
4. 冲刷/回滚接上（第 1 步已写好）。
5. 用 `USE_PRF` 宏切，旧路径留着可回退。


## 8. 已落地：`core/rename.v`（2026-11-04）

`PHYS_COUNT=64 / PHYS_BITS=6 / ARCH_COUNT=32`。物理 0..31 是复位的架构映射（不参与分配），
32..63 是空闲池，每 bank 16 个。

**独立体检台 `Tools/test/rename/tb_rename.v`**：16 深 ROB 影子模型，随机 分配/提交/冲刷，查七条
（分到的号当时必空闲且 bank 位对得上 / 没号必须拉 stall / 排空后空闲数守恒 /
冲刷后 `rat` 逐项等于 `arch_map` / `arch_map` 一一映射 / 空闲表不含 `arch_map` 在用的号 / 冲刷掩码必回收）。
**60/60 个种子通过。**

`rename.v` 单独 synth（`xc7a100tfgg676-1`，out_of_context）：**LUT 1244 / FF 437**。
FF 账对得上：`rat` 192 + `arch_map` 192 + `free_vector` 64 − x0 的两位 = 437。

★ 1244 LUT 比预估大，主要吃在两个"找最低空闲号"的 64 位优先级编码器上。所以
**"比走表方案省"目前只有拍数成立（回滚 0 拍 vs 最长 16 拍），LUT 账没赢**，没做过同口径对比。
要压就把优先级编码换成进位链式的"找第一个 1"（约 64 LUT 量级），不是现在的 mux 树。

### 落地时踩到的两个坑（都在 TB 里）

1. `phys_rd_*` 必须在**上升沿之前**采样：沿之后 `free_vector` 已更新，再读拿到的是下一拍的号。
2. `reset_registered` 比 `rst` 晚一拍，TB 必须多等几拍再发分配，否则 DUT 还在复位。

## 9. 第二步已落地：RAT 进核 + 精确冲刷（2026-11-04）

- `core/rename.v` 实例化在 **idu1 的解码/分配拍**（和 rob 的槽分配是同一拍、同一条指令）。
- `core/rob.v`：每项多存 6 位 `entry_phys`；新增 `commit_phys0/1` 与 `squash_phys_mask` 输出。
  `allocation_phys_0/1` 从 idu1 进来，与 rd 同条件同沿写。
- `cpu_top.v`：`alloc_phys_0/1` → rob，`commit_phys0/1` 与 `squash_phys_mask` 回 idu1，`rob_full` 进 idu1。

**踩到的三个坑（都是真的，不是理论）**

1. **冲刷拍通常还压着一笔打拍后的提交**（rob.v 写明"打拍里那笔在边界之前，清了会丢一笔合法写"）。
   最初 flush 分支把提交整个忽略 ⇒ 那笔的物理号既不进 `arch_map` 也不在冲刷集里 ⇒ 漏号，
   真核里 `arch_map` 变成非一一映射。现在 flush 拍照样定格提交，`rat` 恢复用的是**落完之后**的映射。
2. **rename 的分配条件必须和 rob 的分配口逐字同门**（`!full`）。原来 `alloc_lane0` 在冲刷窗口里
   照样为 1（只是不落盘），rename 却照着发号 ⇒ 漏号 ⇒ 池耗尽 ⇒ 输出物理 0 ⇒ 架构映射被写成 x0 那一格。
3. 冲刷时**不用 rob 送掩码**：本核 `kill_arc` 已删，每次冲刷整表作废 ⇒ 冲刷后唯一在用的就是
   `arch_map` 那 32 个 ⇒ `free_vector <= ~arch_next_mask`，**按构造正确**，不依赖任何掩码。
   `squash_phys_mask` 保留为一致性交叉检查口。

**验证**

- `rename` 单元台（整表作废语义）：**40/40 种子通过**。
- 真核：随机差分 **200/200**；WEIRD 200 只挂 `w36`（基线同挂）；CoreMark **`mcycle 1,144,532 / IPC 1.007 / crc 0x9f95`**，
  与接 PRF 之前**逐字相同** ⇒ 这一步对数据通路完全透明。
- CoreMark 全程跑不变量探针（`arch_map` 一一映射 + 空闲表与它不重叠）：**0 违例**，
  两个 bank 从未同时空（`both_bank_empty_cycles=0`）。

## 10. 第二步的收尾（已被第 11 节取代）

上一版这里写着"还差最后一步"：操作数还在走老路（值表按 ROB 槽索引 + `queue_base_*`）。
第 11 节已经把它做完了。

## 11. 第三步已落地：操作数真正走 PRF（2026-11-04）

### 结构

- **新增 `core/prf.v`**：64 个物理号 × 32 位。6 个组合读口（lane0 a/b、lane1 a/b、访存流 a/b），
  4 个写源。bank = 物理号最低位（lane0 只取偶数号、lane1 只取奇数号）。
- **写口分配**（FPGA 上每阵列只有 1 个写口，这是全部约束的来源）：

  | 写源 | 落哪个 bank | 说明 |
  |---|---|---|
  | lane0 结果口（exu 合并口） | bank0 口0 | 物理号恒偶 |
  | lane1 结果口 | bank1 口1 | 物理号恒奇 |
  | LOAD 返回级（`return_valid`） | bank0 口2 | **权威**：外设 load 只走这条路 |
  | LOAD s2 早口（`stage2_valid & ready_in`） | bank0 口3 | 只为省一拍 |

  ⇒ bank0 三个写口，bank1 一个。bank0 用"三份阵列 + 每项 2 位版本号"，读的时候按版本选。
- **`ready[64]`**：分配清、写回置、冲刷时置成 `arch_next_mask`（在飞的全作废，剩下的值都在家）。
  老值表的 `valid + generation` 二元组整个退场 —— 物理号本身唯一，不需要世代位。
- **idu1**：`rename` 的通用查找口按【译码后的 rs】驱动（`lsu_push_source_a` / `push1_source_a`），
  物理号随载荷走 —— lane 队列 4 个 + LSU FIFO 条目 2 个（**复用原来 tag 那 12 位，宽度不变**）。
  lane1 的查找口带 lane0 同拍旁路（`alloc_ok_0 & rd 相同 ⇒ pick_0`）。
  LSU FIFO 的"提交清 tag"整段删除：物理号不因提交而失效。
- **idu2**：`producer_table`(pstat) / `queue_producer_*` / `queue_base_*` / `value_table_*` /
  提交口广播（`commit_hit_*`、`commit_port_*`、`rdata*_in`）全部删除。
  就绪 = `prf_ready[queue_phys]`，值 = `prf[queue_phys]`。
- **前送按物理号比**：ALU 两条 + LOAD，物理号由 rob 按槽查（`entry_phys[index_*_early]`）。
  `mdu` 仍然绝不进旁路网。

### 落地时踩到的四个坑（都是真的，且都是死锁或静默错值）

1. **写口 `live` 门不能漏**。冲刷之后在飞的结果还会落完，那些项的物理号可能已被回收再分配。
   `live = entry_valid[slot] & (entry_generation[slot] == 世代)`，来自 rob；晚到的死写一律不进阵列。
2. **LOAD 只写早口会漏外设 load**。`load_write_enable_early = stage2_valid & ready_in` 只覆盖 dcache 快路，
   外设 load 只亮 `return_valid`。老设计靠"提交口广播 + `queue_base`"兜这一档，PRF 里没有这条路
   ⇒ 随机差分第一版就挂在 `lw x27,0(x26)`：它分到的物理号永远不写，三个消费者等一个不存在的值，
   ROB 满在 15、lane1 被 lane0 的顺序判据挡死。**两个口都要写。**
3. **只有晚口会掉 9.3% IPC**（CoreMark `mcycle` 288353 → 315175）。`return_valid` 比 s2 晚两拍，
   每个 load 的消费者都要多等。加回早口后回到 +2.04%（294240）。两个口都要，缺一个要么错要么慢。
4. **前送必须排掉物理 0**。写 x0 的指令结果照样挂到总线上（它在 ROB 里的物理号是 0），
   按物理号比会让所有读 x0 的消费者吃到它的值。实测 9 个种子错值：
   `lui x0,0xe31bf` 的 0xe31bf000 被当成 x26 的源（x26 差 `e71ca6dc` vs `0400b6dc`）。
   修法：`alu_fwd_ok_* = write_enable_alu_*_early & (|phys_alu_early_*)`，LOAD 同理。

### 验收

- 随机差分 **200/200**；WEIRD 200 只挂 `w36`（基线同挂）。
- CoreMark：`minstret 288215` 与基线逐字相同，`mcycle 294240` 对基线 `288353` ⇒ **+2.04%**。
- **这 2% 是"每次冲刷约 1.12 拍"**，不是每个 load 一拍：CoreMark 全程冲刷 **5258 次**，
  多出来 **5887 拍**。所以早口加回来之后 load 那条路已经追平了，剩下的是冲刷后的第一拍
  有什么东西多等了一拍。下一步要收就查这个（候选：`prf_ready` 在冲刷拍用 `& ~alloc_mask`
  清掉了"同一拍被 rename 分配、但 rob 因冲刷没接"的那个号；这个号其实是空闲的）。
- 保留 `squash_phys_mask` 与 `arch_next_onehot` 作一致性交叉检查口。

### 时序：**没通过**（2026-11-04，`a100t_impl_e.tcl`，12.5 ns，xc7a100tfgg676-1）

`WNS −0.902 / TNS −95.236 / 失败端点 479`（PRF 之前是 `−0.371 / −0.467 / 2`）。
资源：Slice LUT 32401(51.1%)、LUT as Memory **2852**、Slice FF 15667(12.4%)、BRAM 51.5、DSP 8。

失败端点按终点分族（`-unique_pins`）：

| 端点族 | 条数 | 起点 | 备注 |
|---|---|---|---|
| `u_cont/u_csr/{mscratch,mcause,mtval,…}` | ~165 | `u_lsu/stage2_index` | 异常/陷阱路，不是 PRF 引出来的，但布局变差后一起掉 |
| `u_iffu/half1_{instruction,address,…}` | ~96 | `u_idu1/u_rename/free_vector` | 分配→iffu 那族，同样是被布局挤坏 |
| `u_prf/bank1_port1[*] / bank0_port0[*]` | ~90+ | `u_exu_*/u_mdu/multiply_stage_operation` | **新族**：写口。mdu→exu 仲裁 mux→`entry_phys` 查表→LUTRAM 的 D 脚 |
| `u_exu_0/u_mdu/divide_state_quotient_partial` | 32 | — | |

结论：**功能全绿，时序红了**。读侧不是问题，问题是**写口** —— 和当初值表一样的位置，
只是这次从"触发器阵列的写译码"换成了"LUTRAM 的 D 脚"，而且前面还串了一级 exu 仲裁 mux
和 rob 的 `entry_phys` 查表。下一步要么给写口加一级寄存器（代价：load/结果写回再晚一拍），
要么把 `phys` 挪到 exu 的早口上（和 ALU 前送同一套身份），让写地址不再等仲裁结果。
`u_csr` / `u_iffu` 那两族**不是**布局噪声：同脚本重跑两次，`WNS/TNS/端点 三个数逐位相同`
（−0.902 / −95.236 / 479），说明这份 RTL 是确定性变差，不是随机摆歪。

## 12. bank0 三块压回两块（2026-11-04）

LOAD 的两个时相共用一个写口，删掉 `bank0_port3`。现在阵列是
`bank0_result`（lane0 结果口）、`bank0_load`（LOAD 早晚两个时相）、`bank1_result`（lane1 结果口），
`bank0_version` 从每项 2 位降到 1 位，6 个读口的 mux 从 3:1 降到 2:1。

**这一步是免费的，因为第三块阵列本来就在撞拍时被白买。** `wp3_active = write_ok_3 & ~write_ok_2`
这个门只在"背靠背 load"（前一笔在返回级、后一笔在 s2）时生效，而它一生效，
早写就被门掉了 —— 三块阵列的时候也一样写不进去。合并之后行为逐拍相同：

- 撞拍时晚的赢（权威值），早的那笔到自己的返回级还会再写一次；
- 被门掉的那个号 `ready` 也不置（`write_mask` 用的就是 `wp3_active`），消费者等返回级；
- 不撞拍时早、晚各写一次同一项，值相同，只是 `bank0_version` 都置 1。

**实测**：随机差分 200/200、WEIRD 199/200（只挂基线同挂的 `w36`）、
CoreMark `minstret 288215 / mcycle 294240` 与合并前**逐字相同**。

★ 代价是**关掉了"拆掉 `wp3_active` 门"这条路**：一块阵列一个写口，撞拍必须仲裁。
要拿到那个早写，就得留两块阵列（或者给写口加一级寄存器）。这一条是取舍，不是白捡。

**实现（同脚本同口径，12.5ns / xc7a100tfgg676-1）**

| | WNS | TNS | 端点 | LUT | FF | LUT as Memory |
|---|---|---|---|---|---|---|
| 四块阵列 | −0.902 | −95.236 | 479 | 32401 | 15667 | 2852 |
| 三块阵列 | **−0.744** | **−24.499** | **242** | **30410** | **14519** | **2852** |

省下来的是 1024 个触发器 + 约 2000 LUT —— 后者是 6 个读口各短一截 3:1 mux、外加版本号的译码。

**发现：PRF 的阵列根本没进 LUTRAM。** 报告里的名字是 `bank0_result_reg[*]` / `bank1_result_reg[*]`，
`LUT as Memory` 两个版本**一模一样 2852**（那份是 BTB/BHT/LRU 的）。所以"每阵列 1 个写口"
这条 FPGA 约束在这份 RTL 上**没有生效**，综合器选了触发器 + 读 mux。
分阵列真正买到的东西不是写口数，而是**每个阵列只有一个写源 ⇒ D 脚上没有 4 选 1 的数据 mux**，
只剩索引比较；这正好是当年值表那堵墙的位置。要进一步缩就只能往写地址/写数据的前面去动，见下。

最差那条仍是 dcache 那一条（`u_lsu/stage1_base → u_dcache/load_ready_reg/D`，−0.744），
**不是** PRF。但 1000 条 `-unique_pins` 里 PRF 写口占 786 条：
`bank0_result` 411 + `bank1_result` 375，起点清一色 `u_exu_*/u_mdu/multiply_stage_operation`。
路径：`mdu 结果 → exu 仲裁 mux → rob 的 entry_phys 查表 → D 脚`。



