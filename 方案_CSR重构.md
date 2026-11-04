# CSR 交互重构（spec，未落地）

日期：2026-11-04
起因：12.5ns 下最差路径是 `alu/op_index_reg → cont/u_csr/mscratch_registered_reg/D`（−0.186，14 级，
route 占 78%），链上串着 `idu2/issue_valid_1`（扇出 315）和 `csr_data_combinational`。
这是从值表时代就存在的一族，真 PRF 落地后它的链头换成了 PRF 前送的那次 `entry_phys` 查表。

## 1. 现在是怎么走的（问题所在）

```
idu2 发射级：csr_address/function3 → csr.v 的 12:1 case → csr_data_combinational
          → 作为 ALU 的一个操作数（op_cs / cs_data）
          → ALU 出 csr_result_alu
          → 同一拍给 csr.v 的写口（csr_write_enable_0/1 + csr_address_0/1）
```

三点特征：

1. **CSR 读值经过 ALU**。`csr.v:255-267` 是一个 12:1 的大 case，输出直接进 ALU 的 `cs_data`。
2. **写使能是发射级的组合量**（`idu2:629` `csr_write_enable_0 = issue_valid_0 & queue_csr_write_enable_0`），
   所以写口挂在发射判据上。
3. **乱序洞**：`csr_held`（`idu2:543`）只在"这条不是队头"时挡写，不挡读
   ⇒ `csrr` 可能读到还没提交的 CSR 写之前的值。这条 2026-10-29 记过账（arch 44→55）。

开源核（Rocket/BOOM、Ibex、CVA6、VexRiscv）共同点：CSR 的值**从不经过 ALU**；
写使能随流水线走；Rocket/BOOM 干脆把 CSR 写口放在**提交级**，因为 CSR 读必须看到精确状态。

## 2. 目标形态

| | 形状 |
|---|---|
| CSR 值 | 不经过 ALU。`cs_data` 输入、`result_csr` 输出整条删 |
| 写口 | **只有一个，在提交口**。逐条读-改-写 |
| 读 | WB 级一级 mux：`rd_data = is_csr ? csr_rdata : result`，再进 PRF 写口 |
| 串行化 | CSR 指令只能从**队头**发（更老的必须已提交）⇒ 读到的就是精确值 |

## 3. 各模块改什么

### idu1 / 载荷
- 译码出 `csr_op[1:0]`（00 = 不写、10 = 写、11 = 置位/清位由 function3 区分）与 `csr_addr[11:0]`，
  随指令进 ROB 项；`zimm` 那一位也带上（决定源是 rs1 还是 5 位立即数）。
- idu2 不再收到 `decoded_csr_*` 的发射级副本。

### idu2
- 删：`queue_csr_write_enable_0/1`、`queue_csr_address_0/1`、`queue_csr_function3_*`、
  `csr_write_enable_0/1`、`csr_write_action_0/1`、`csr_address_0/1`、`op_cs` 整族。
- 发射门：删 `csr_held_0/1`，换成一条"这条是 CSR 且不是队头 ⇒ 不发"（等价于队头限制）。
  ★ 注意 `order_ok` 已经有一条"队头优先"的判据，这条要挂在同一处，别变成两个相位不同的门。

### exu / alu
- 删 `cs_data` 输入与 `result_csr` 输出；ALU 只剩真正的 ALU 运算。

### rob
- 每项加 `csr_op[1:0]` + `csr_addr[11:0]` + `csr_src_phys[5:0]` + `csr_imm_flag`（共约 20 位）。
  16 项 = 320 个触发器。
- 提交口把这一组连同 `commit_write_enable` 一起打拍送出（现成的提交打拍那一组）。

### prf
- 多一个读口：提交口按 `csr_src_phys` 读源值。地址 6 位、数据 32 位，纯组合。

### csr.v
- 写口输入改成提交版：`commit_csr_write`、`commit_csr_addr`、`commit_csr_data`（读-改-写算好的值）、
  `commit_csr_op`。
- 读口照旧组合（`csr_data_combinational`），但输出不再进 ALU，而是进 WB 级那个 mux。
- 陷阱 / `mret` / 中断那几组更新**不动**（`exception_retire`、`exception_interrupt_return_ok`），
  它们本来就在提交口相位上，和指令写口天然同拍可仲裁。

### cont
- 不需要动。`flush_control_exception` 那套仍然只挡中断。

## 4. 必须成立的不变量

1. **CSR 读必须看到精确状态**：一条 CSR 指令发出时，所有比它老的指令都已提交。靠第 3 节的队头门保证。
2. **提交口读 PRF 一定命中**：`csr_src_phys` 在提交时必已写回（更老的写全部完成），
   所以这一路不需要 ready 判断，直接读。
3. **写口仲裁**：陷阱 > 指令写。两者同拍发生时，指令那条是故障项，本来就不该写（它是 `exception` 项，
   提交时已按 trap 处理），所以显式给 trap 优先即可。
4. **自增类 CSR**（`mcycle` / `minstret`）不走指令写口，仍由现有逻辑推进。
5. **`mstatus` 的 MIE/MPIE 恢复**仍走 `exception_interrupt_return_ok`（现有那根，不能与 pc 侧脱钩）。
6. **`csr_write_action`**（rs1 字段为 0 时 csrrs/csrrc 不写）在提交口实现，判据不变。

## 5. 代价

| 项 | 量 |
|---|---|
| ROB | +20 位 × 16 = 320 FF |
| PRF | +1 个组合读口（6 位地址 / 32 位数据） |
| 发射门 | +1 位（CSR 队头限制） |
| IPC | CSR 指令串行化。CoreMark 全程约 150 条 CSR ⇒ 可忽略 |
| 收益 | 最差路径那 14 级消失；`csr_data_combinational` 从发射锥里退出；乱序洞根治 |

## 6. 落地顺序（三步，每步独立验收）

**A. 串行化门**（只改 idu2 发射门）
把 `csr_held` 的语义从"挡写"改成"不是队头就不发"。不改任何结构。
判据：差分 200/200；CoreMark `minstret` 不变，`mcycle` 允许 +0.1% 量级。

**B. ALU 摘掉 CSR**
CSR 读值改为在 PRF 写口前一级 mux 选，写口仍在发射级（`csr_write_enable_0/1` 保留）。
这一步单独就能把 `csr_data_combinational → alu` 那段从关键路径上摘掉。
判据：同上；远端 12.5ns 端点族里 `mscratch`/`mtval` 那两条应消失。

**C. 写口搬提交口**
ROB 载荷 + PRF 提交读口 + csr.v 写口改接。A 已经保证了精确性，C 只是把写从发射级挪到提交级，
顺带把 `idu2:629` 那条发射判据到写口的链整段删掉。

## 7. 验证

1. 随机差分 200/200（生成器里有 CSR 指令）；WEIRD 199/200（只许挂既有 `w36`）。
2. CoreMark：`minstret` 逐字 288215（CSR 条数不变），`mcycle` 记录前后值。
3. 不变量探针（CoreMark 全程，0 违例）：
   - 每条 CSR 指令发出时，ROB 里没有更老的在册项；
   - 提交口读 PRF 的 `csr_src_phys` 的 `prf_ready` 恒为 1；
   - 陷阱拍 CSR 不被指令写覆盖；
   - `csrs`/`csrc` 对只读位（如 `misa`）不产生写。
4. 时序：远端 12.5ns，看 `u_csr/*` 那一族端点是否清零，最差路径换成谁。

## 8. 回退

三步三个 commit，任一步可单独回退。B 和 C 之间系统必须都是功能全绿的中间态
（B 只改数据源、不改写口相位；C 只改写口相位、不改读）。
