`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/27
// Design Name:
// Module Name: rob —— 重排序缓冲（reorder buffer）
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   执行可以乱序完成，但写寄存器堆【只走按序退这一条路】⇒ 同 rd 的先后由结构保证。
//   分配：每条指令占一项（包括不写 rd 的和 store —— "谁比谁老"必须覆盖每一条指令，
//         否则陷阱不知道自己在程序序里的位置）。
//   完成：执行单元把（值 + 索引 + 世代）报到完成口，本模块按索引把 wr/data 同条件同一沿
//         写进那项；非 valid 的槽、世代对不上的（冲刷/复用后的迟到上报）一律忽略。
//   退：  head（和 head+1）valid 且 wr 才退，2 宽、严格按序；退口就是寄存器堆的两条写口。
//   值读口（拉式）：发射级按它自己锁下的 {slot,gen} 来读 entry_data + 世代校验。
//   冲刷：作废比 flush_index 更年轻的项；flush_incl = 边界那条也一起作废（只有中断拉高）。
//   陷阱：故障项的 (cause,pc,tval) 随自己的项存下，退到 head 时【不写寄存器堆】地退掉，
//         比它年轻的项全部作废，同时拉 exception_valid_delayed 把载荷交给上级 —— 精确异常。
//
// ★ 本文件按三个区组织（寄存器声明统一放在最前，Verilog 要求先声明后使用）：
//     分配区 —— tail 端：指针与占用、槽号与世代、入项时写进去的静态信息
//     提交区 —— head 端：退役判据、冲刷、陷阱、提交口（写寄存器堆）
//   项状态阵列（v / wr / ex / data / jdg）是唯一被三块共写的对象，只能有一个 always 块，
//   放在提交区末尾：分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
//
// ★ 与流水级的对应（本模块没有自己的流水级，是"全核的状态机"；每个口挂错一拍就是
//   "回填打到错项 / 标记落在已退的槽上"这类静默错）：
//   allocation_enable           —— 分配拍（iffu 出队格换新 / 该条进 idu1 输出级那一拍）
//   alu/alu_lane1/mul/multiply_lane1/ld 完成口 —— 结果被写口级处置那一拍，比分配晚一拍起步，mul/ld 不定长
//   bju_judged/branch_unit_lane1_judged —— bju 判定寄存一拍后的 wb 拍（两条 lane 各一套）
//   exc_en             —— 该条在载荷里的那一拍（bju 的组合异常标记）
//   flush_control_rob      —— bju 判定寄存一拍后的 wb 拍；flush_index = 那条分支/跳转自己的项
//   exception_valid_delayed          —— 故障项排到队头那一拍，上级同拍锁 mepc/mcause/mtval 并重定向
//   full/empty/occupancy —— 组合状态；full 喂 cont 的发射级压制
//
//   ★ 年龄判据一律用 `age = {1'b0,(索引-head_pointer)}`：拼接的操作数是自决定的，保证减法
//     在 4 位里做完再扩成 5 位；直接写 `(a-b) < entry_count` 会按 5 位求值，回绕时判错。
//
// ★ 2026-10-30：拉式值读口（query_slot/query_data/query_ok 那 4 组）已整条删除。
//   发射级的操作数选值现在只有三档（立即数 / 本地值表 / queue_base），
//   那 4 组答案在 idu2 里从来没有被读过 ⇒ ROB 这边的 16:1 entry_data 选择、
//   世代比较、以及 16 个答案/身份寄存器全部是无人消费的计算。
//
// Dependencies:
//
// Revision:
//   Revision 0.01 - 新建
//   Revision 0.02 - 值读口改成拉式（删掉入队扫描 / 1 深载荷冻结 / 前送槽号 / rdy）；
//                   完成口按 lane 扩成 alu/alu_lane1/mul/multiply_lane1/ld 五条
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module rob(
    input clk, rst,
//分配口。相位 = iffu 出队格换新那一拍（本模块无自有流水级）；两条 lane 各一套、独立推进
    input        allocation_en_0,
    input        allocation_en_1,
    input [4:0]  allocation_rd_0,
    input [4:0]  allocation_rd_1,
    input        allocation_we_0,
    input        allocation_we_1,
//★ 2026-11-04 真 PRF：这一条被分到的物理号，跟着 ROB 项走。
//  提交时它定格进 arch_map；冲刷时把在册项的物理号一起还给空闲表。
    input [5:0]  allocation_phys_0,
    input [5:0]  allocation_phys_1,
//★ 2026-11-04（精确冲刷第 1 步）：分配时被顶掉的旧映射 = 分配那一拍 rat[rd] 的原值。
//  精确冲刷时按「击杀集合里同架构号最老那条」的 old_phys 回滚 rat。本步只存不用，对通路透明。
    input [5:0]  allocation_old_phys_0,
    input [5:0]  allocation_old_phys_1,
//★ 2026-11-04（CSR 步骤 C）：CSR 指令与它的载荷 {写使能 1, 地址 12, funct3 3, 源 6}。
//  ★ 载荷**单独一组阵列**，不借 entry_target：借用的第一版实测把 entry_target 的 D 脚变成一个
//    跨 bju/lsu/idu1 三处的大 mux（它本来有判定口和异常口两个写源，再加分配拍这一个），
//    12.5ns 下直接成了最差路径（−0.852/432，比不借用的 −0.588/159 还差）。
//    单独一组只有【分配】一个写源，D 脚就是载荷本身。代价 22×16 = 352 FF。
    input        allocation_csr_0,
    input [21:0] allocation_csr_payload_0,
    input        allocation_csr_1,
    input [21:0] allocation_csr_payload_1,
    input        allocation_jmp_0,
    input        allocation_jmp_1,
//M 类 / LOAD：即使 rd==0 也会回报完成口，不许在"发出即置可退"里退休
    input        allocation_pend_0,
    input        allocation_pend_1,
//SYSTEM 类（ecall / ebreak / illegal / mret）：异常或改向标记在 bju 判定延迟级才出，
//这几类的退休必须等那一级过去；其余项不受影响。四根保持分开，只在判据里或。
    input        allocation_ecall_0, allocation_ebreak_0, allocation_illegal_0, allocation_mret_0,
    input        allocation_ecall_1, allocation_ebreak_1, allocation_illegal_1, allocation_mret_1,
//分到的槽号与世代（随载荷走到执行单元，完成上报时带回校验）
    output reg [3:0]  allocation_index_0,
    output reg [3:0]  allocation_index_1,
    output reg        allocation_generation_0,
    output reg        allocation_generation_1,

//完成口：写口级处置完一笔就回报（落地 / 被杀 / 不写），data / gen 与 done / idx 同源同拍
    input        bju_judged,
    input [3:0]  bju_index,
    input        bju_generation,
    input        branch_unit_lane1_judged,
    input [3:0]  branch_unit_lane1_index,
    input        branch_unit_lane1_generation,
//★ S3：判定要不要改向 + 落点。跟着判定口一起进来，写进项里；队头到了才真正改 PC。
    input        bju_redir,
    input [31:0] bju_tgt,
    input        branch_unit_lane1_redir,
    input [31:0] branch_unit_lane1_tgt,
    input        alu_done,
    input [3:0]  alu_index,
    input [31:0] alu_data,
    input        alu_generation,
    input        alu_lane1_done,
    input [3:0]  alu_lane1_index,
    input [31:0] alu_lane1_data,
    input        alu_lane1_generation,
    input        mul_done,
    input [3:0]  mul_index,
    input [31:0] mul_data,
    input        mul_generation,
    input        multiply_lane1_done,
    input [3:0]  multiply_lane1_index,
    input [31:0] multiply_lane1_data,
    input        multiply_lane1_generation,
    input        div_done,
    input [3:0]  div_index,
    input [31:0] div_data,
    input        div_generation,
    input        divide_lane1_done,
    input [3:0]  divide_lane1_index,
    input [31:0] divide_lane1_data,
    input        divide_lane1_generation,
    input        st_done,
    input [3:0]  st_index,
    input        st_generation,
    input        ld_done,
    input [3:0]  ld_index,
    input [31:0] ld_data,
    input        ld_generation,
//结果口的接受条件（与完成回填 entry_data 的判据逐字同门）
    output reg        acc_alu_o,
    output reg        acc_alu2_o,
    output reg        acc_mul_o,
    output reg        acc_mul2_o,
    output reg        acc_div_o,
    output reg        acc_div2_o,
    output reg        acc_ld_o,
    output reg        acc_st_o,
//发出口：不写 rd 的项靠这一拍置"可退"（见完成段）
    input        issue_enable_0,
    input [3:0]  issue_index_0,
    input        issue_generation_0,
    input        issue_enable_1,
    input [3:0]  issue_index_1,
    input        issue_generation_1,
//冲刷口：作废比边界更年轻的项；flush_incl = 边界那条也一起作废。
//★ 本模块自己照旧吃【当拍】的冲刷记录：边界是否还在窗口里（flush_ok）必须拿当拍的 ROB 状态判，
//  晚一拍窗口已经动了，会把"部分冲刷"误判成"全冲" ⇒ 把比边界老的活项一起抹掉 ⇒ 死锁（实测）。
//  要打拍的是【本模块 → idu2】那一跳：flush_partial_registered 是本模块当拍算好的判定，寄存一拍给 idu2 用。
    input        flush_control_rob,
//★ S5：冲刷记录的寄存版（cont 的 flush_pending，晚一拍）。只喂"停分配"这一根 ——
//  idu1 的载荷清零吃的是它，分配不停就会把同拍刚推进去的正确路径指令擦掉（实测 seed1）。
    input        flush_pend_in,
//异常标记口 ×3：lane0 / lane1 / lsu。
//每条异常属于它自己那一项，各带各的槽号 ⇒ 三路同时到达也互不相干，本模块不做任何仲裁。
//(cause,pc,tval) 随故障指令自己的项存下，退到队头才交付。
    input        exc0_en,
    input [3:0]  exc0_index,
    input [3:0]  exc0_cause,
    input [31:0] exc0_pc,
    input [31:0] exc0_tval,
    input        exc1_en,
    input [3:0]  exc1_index,
    input [3:0]  exc1_cause,
    input [31:0] exc1_pc,
    input [31:0] exc1_tval,
    input        exception_selected_enable,
    input [3:0]  exception_selected_index,
    input [3:0]  exception_selected_cause,
    input [31:0] exception_selected_pc,
    input [31:0] exception_selected_trap_value,
//退口：寄存器堆的两条写口（口 A = head、口 B = head+1）
    output reg        commit_write_enable_0,
    output reg [4:0]  commit_destination_0,
    output reg        commit_write_enable_1,
    output reg [4:0]  commit_destination_1,
//★ 2026-11-04 真 PRF：这两条提交项各自的物理号（arch_map 定格用）
    output reg [5:0]  commit_phys0,
    output reg [5:0]  commit_phys1,
//★ CSR 提交口（与 commit_write_enable_* 同相，打一拍）：csr.v 在这一拍做读-改-写并落地。
//  载荷 = {写使能, 地址, funct3, 源}，原样吐出去，不在这里解释。
    output reg        commit_csr_valid,
    output reg [21:0] commit_csr_payload,
//★ 冲刷掩码：本拍在册的全部项物理号各展成 one-hot 后 OR 起来，直接给 rename 回收。
//  kill_arc 已删 ⇒ 本设计每次冲刷都是整表作废，不需要按边界挑。
    output reg [63:0] squash_phys_mask,
//★ 精确冲刷：这一拍【没被杀】的在飞项所占的物理号。
//  rename 的空闲表照旧按构造写：free = ~(arch_next_mask | 幸存掩码)；
//  prf 的就绪位照旧按构造写：ready = arch_next_mask | (ready & 幸存掩码) | 本拍写回。
//  整窗那一档幸存掩码恒 0 ⇒ 两处都与改动前逐字等价。
    output reg [63:0] squash_survive_mask,
//★ 精确冲刷：击杀掩码打一拍广播给 idu2 / idu1 / lsu —— 消费端的击杀都比冲刷晚一拍。
//  第 i 位 = 槽 i 这一拍要被作废。整窗那一档全亮（与改动前"全杀"逐字等价）。
    output [15:0]     squash_kill_mask_registered,
//  当拍版：lsu 的击杀与冲刷同拍（flush_commit 是组合的），吃这一份。
    output [15:0]     squash_kill_mask,
//★ 精确冲刷：每个架构号在【击杀集合】里最老那条的 old_phys（回滚 rat 用）。
//  hit 位 = 该架构号有被杀项写过；phys = 那条项的 old_phys。没被杀到就保持 rat 现值。
    output reg [31:0]  squash_arch_hit,
    output reg [191:0] squash_arch_phys,
//★ 精确冲刷：这个架构号有没有【幸存】在飞项写过。有 ⇒ rat 保持现值（它正指着那个幸存项）；
//  没有 ⇒ rat 必须退回 arch_map（判定期同拍新分配出去的那个号没有 ROB 项，留着就是个死引用）。
    output reg [31:0]  squash_survive_hit,
//★ 判定拍改向【接住】的那一拍不许分配：本拍的 kill 掩码是用【拍前】的 entry_valid 算的，
//  同拍新进来的项不在掩码里，会被当成幸存项、把尾指针算歪 ⇒ 指令变孤儿。
//  与 flush_control_rob 那一拍的口径一致，只是早了一拍。
//★ 2026-11-05 改成"改向一经判定就冻住分配，直到那条分支自己提交"（NaxRiscv 口径）：
//  冻结期间不分配新项 ⇒ 重命名状态不会被任何新项读到，回滚就退化成"提交拍整片抄回 arch_map"，
//  不需要按架构号挑最老被杀项的 32×16 扫描。
    output             redirect_stop_alloc,
//★ 冻结的收尾那一拍：队头这条带 pending 标记的项正在退休 ⇒ rename / prf 一拍恢复。
//  ★ 必须和提交口【同相位】打一拍：提交口（commit_write_enable_0/…）是寄存的，
//    恢复要用"落完这笔提交"的 arch_map，组合版会早一拍、漏掉分支自己那笔写（实测 seed1：x5 停在旧号）。
    output             restore_valid,
//★ 2026-11-04 真 PRF：三个写回口的物理号 + 存活判据。
//  live = 本项还有效、且世代对得上 —— 冲刷之后在飞的结果还会落完，
//  那些项的物理号可能已经被回收再分配，晚到的死写不许进阵列。
    input        prf_wb_en_0, input [3:0] prf_wb_index_0, input prf_wb_generation_0,
    input        prf_wb_en_1, input [3:0] prf_wb_index_1, input prf_wb_generation_1,
    input [3:0]  prf_ld_index, input prf_ld_generation,
    input [3:0]  prf_alu_early_index_0, prf_alu_early_index_1,
    input [3:0]  prf_ld_early_index, input prf_ld_early_generation,
    output [5:0] prf_ld_early_phys, output prf_ld_early_live,
    output [5:0] prf_alu_early_phys_0, prf_alu_early_phys_1,
    output [5:0] prf_wb_phys_0, prf_wb_phys_1, prf_ld_phys,
    output       prf_wb_live_0, prf_wb_live_1, prf_ld_live,
//状态
//★ 提交口改向：队头这一项是"判错过了"的分支 ⇒ 本拍改 PC、并把它之后的全杀掉。
//  队头即边界 ⇒ "所有在途项都比边界年轻"是定理，消费端不再需要年龄判据。
//★ "已判定、还没排到队头"的改向是否还在册。给 idu2 挡【新的访存发射】：
//  判定已经知道错路了，但要等它排到队头才杀；这个窗口里放出去的访存是错路的，
//  而错路访存可能打到没有从设备的地址 → LSU 永远等应答（实测 CoreMark：错路 `lbu 1(x0)` 卡死整核）。
//  挡住"新发"即可；已经在 s2/s3 的那几条会在提交口改向那一拍被 kill_arc 杀干净。
// mret：与 bju 同一套（项内标签 + 队头改向）。标签随分配落地（allocation_mret_*），
//  落点由 pc 直接取 csr 的 mepc。本模块只出 [这一笔是 mret 改向]。
    output reg        mret_valid_delayed,
    output reg        redirect_any,
//★ S5：A 域事件（判定拍 / mret）—— 只改 PC + 清前端，不碰后端。
    output reg        redirect_front_valid,
//★ S5：B 域事件（边界走到提交口）—— 只清整表 + 后端，不碰前端。
    (* max_fanout = 32 *) output reg redirect_valid_delayed,
    output reg [31:0] redirect_target_delayed,
    output reg [3:0]  head_pointer,
//本拍退休的两条（给 lsu：退休过的项不许再按年龄冲刷作废）
    output reg        ret0_v, ret1_v,
    output reg [3:0]  ret0_index, ret1_index,
    output reg        ret0_generation, ret1_generation,
    output reg        full,
//★ 击杀弧掩码：第 i 位 = "槽 i 比冲刷边界年轻"。寄存版，给 lsu/idu1/idu2 的作废判据用。
//  为什么它可以寄存而 flush_ok 不行：这是一段【绝对槽号的弧】—— (边界, 队尾]。队头前进不改变
//  哪些槽比边界年轻，所以晚一拍用同一个掩码是【精确等价】。而 flush_ok 判的是"边界还在不在
//  占用窗口里"，那个窗口是跟着队头走的，晚一拍就变。
//  为什么值得寄存：消费端原来是 `{1'b0,(idx-head_pointer)} > {1'b0,(flush_index-head_pointer)}`，
//  两次减法 + 一次比较，而且 head_pointer 要从本模块扇到 lsu/idu1/idu2/cont/iffu/alu 六个地方。
//  换成 `arc_q[idx]` 之后消费端只剩一次 16:1 查表，head_pointer 不再出门。
//★ "槽 i 还没绕过队头"（i >= head_pointer）的位向量。给消费端做【lane 对 lane】的年龄排序用。
//  为什么需要它：age(X) < age(Y) 不能化成 X < Y —— 年龄在 16 的环上回绕，跨过 0 时排序会翻。
//  但有它就能拆成三段：同侧（都 >= head 或都 < head）时排序等于比槽号，跨侧时恒有定论。
//  `older = (slot_above_head[X] & ~slot_above_head[Y]) | (same & (X < Y))`，其中 same = ~(slot_above_head[X]^slot_above_head[Y])。
//  这样这些模块一个减法都不用做，head_pointer 也不必再扇过去。
    output reg [15:0] slot_above_head,
    output reg [15:0] slot_above_head_registered,
// P2: head one-hot (registered). For "am I the head" qualification checks,
// instead of shipping the pointer out and letting every module compare.
// D side is only a 4->16 decode, so replicating it is cheap.
    output reg [15:0] head_one_hot_registered,
//---- 中断边界 / mepc：必须取【ROB 队头 = 最老未退休】----
//  原来这两项取自 idu2 的队列头（= 最老【未发射】）。两者不等价：中断切割之后，比它老但已发射
//  未退休的项仍在 ROB 里照常退休，而它们的控制流改向被这次冲刷抹掉 ⇒ 落空路径被架构执行。
//  ★ 这里必须【组合】给出（不能寄存一拍）：队头同一拍可能刚退休，寄存版会把已退休那条当成
//    返回点，导致它被执行两次。
    input [31:0]  allocation_pc_0, allocation_pc_1,
    output reg [31:0] head_pc_out,
    output reg        head_valid_out,
// P1: how many slots the head advances this cycle (0/1/2). Decided once here;
// minstret consumes this delta instead of subtracting pointers.
    output reg [1:0]  retire_step,
    output reg        empty,
    output reg [4:0]  occupancy,
    output reg        exception_valid_delayed,
//陷阱载荷：与 exception_valid_delayed 同拍，都是 dly 里的寄存器。
    output reg [3:0]  exception_cause_delayed,
    output reg [31:0] exception_pc_delayed,
    output reg [31:0] exception_trap_value_delayed
    );

//==============================================================================
//  参数与声明区：本模块全部 localparam/parameter + reg/wire。
//  Verilog 要求先声明后用，所以全放这里；正文只留逻辑。
//==============================================================================
parameter integer DEPTH = 16;   // ★宏化旋钮：16=大核 / 8=小核（必须是 2 的幂）
localparam [3:0] DEPTH_MASK = DEPTH - 1;
localparam integer ARCH_NUMBER = 32;   // RV32I 架构寄存器个数（rat 回滚按这个维度展开）

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，各模块各自打一拍，彼此没有相位差。
reg reset_registered;
//==============================================================================
//  信号声明（Verilog 要求先声明后使用，三区的寄存器统一放在这里，按区归类）
//==============================================================================
//分配区
reg [3:0]  tail_pointer;
reg [4:0]  entry_count;
reg [3:0]  tail_pointer_next;
reg [4:0]  entry_count_next;
//每槽的目的寄存器号（写 x0 与"不写 rd"一律存 0）：值读口不按 rd 比较，这里只给提交口与
//"发出即置可退"用；退项与冲刷不必清（判据里 entry_valid 与它相与）。异常载荷不逐槽存：见 exc_side_*。
//每槽的分配指针标签 {wrap, slot}（年龄的唯一货币）
reg [4:0]  entry_tag [0:DEPTH-1];
reg [4:0]  tail_tag, tail_tag_next;
reg [4:0]  entry_destination_register [0:DEPTH-1];
//★ 2026-11-04：每槽的物理号（真 PRF）。只有【复位 + 分配】两个写源，与 rd 同条件同沿。
reg [5:0]  entry_phys [0:DEPTH-1];
//★ 2026-11-04：这一项分配时顶掉的那个映射（rat[rd] 的旧值），精确冲刷回滚用。
//  第 1 步（透明中间态）已经接线并每项存下来；回滚逻辑是下一步，现在没有任何读者。
reg [5:0]  entry_old_phys [0:DEPTH-1];
//每槽的世代位：每次分配翻转一次。完成上报必须世代一致才被接受 —— 这是"迟到上报
//打进新住户"的唯一根治手段（entry_valid 挡不住"新住户也 valid"）。
reg        entry_generation  [0:DEPTH-1];
reg        entry_jump  [0:DEPTH-1];
reg        entry_pending [0:DEPTH-1];
reg        entry_system  [0:DEPTH-1];
//★ 2026-11-04（CSR 步骤 C）：这一项是不是 CSR 指令，以及它的载荷（单独一组，理由见端口注释）。
//  ★ 载荷原来是单独一组阵列。曾经试过把它做成分布式 RAM（`ram_style="distributed"` + 踢出复位循环），
//    结果一位没省：本文件的 entry_* 全都是触发器，原因不是复位也不是 ram_style，而是**同一拍有两条写语句**
//    （lane0 写 tail_pointer、lane1 写 allocation_index_1）—— 7 系列 LUTRAM 原语只有 1 个写口，
//    综合器直接判 `8-4767 ... multiple writes via different ports` 后落回触发器。别再往这个方向试。
reg        entry_csr     [0:DEPTH-1];
reg [21:0] entry_csr_payload [0:DEPTH-1];
//提交区
reg [3:0]  head_next_pointer;
// P4: cont only supplies the EXCEPTION boundary candidate. For a commit-port redirect
// the boundary is this module's own head, so the effective boundary is resolved here
// and head_pointer no longer has to leave the ROB just to be muxed into flush_index.
// 边界归属：改向（队头）与 trap 交付（队头）都在本模块内解析；
// cont 只送"改向/中断的边界候选"，异常那一档已经不需要边界了。
reg [3:0]  head_pointer_next;
reg [1:0]  retire_step_next;
reg        flush_any;
reg        head_ok;
reg        head_next_ok;
//==============================================================================
//★ 年龄判据 = 【分配指针标签】：每条指令在分配那一拍存下当时 tail 指针的 `{wrap, slot}`。
//  这才是开源核（BOOM/香山 RobPtr{flag,value}）用的东西，冲刷时指针【连 wrap 位一起回卷】，
//  所以标签始终是"按分配顺序递增 1"的 mod-32 序号，"谁更老"= 两个标签相减取符号位。
//★ 不能用 entry_generation 当环绕位：gen 是"每个槽各自的分配奇偶"，冲刷回卷后同一位会在同一圈里
//  被分配两次，各槽奇偶错开（实测：队头 5、活项 5..12，entry_generation 却把 6..11 判成"比队头老"）。
//★ 也不能用"每次分配 seq_cnt+1"：冲刷释放掉的项已经消耗过号，活窗口的跨度会超过 16
//  （实测跨度 21），5 位比较就失效了。指针回卷正好把号收回来，跨度恒等于在册条数 ≤16。
//提交口的组合版（打拍前）：head_ok / entry_destination_register[head_pointer] / entry_data[head_pointer]
reg        commit_we0_comb, commit_we1_comb;
reg [4:0]  commit_rd0_comb, commit_rd1_comb;
reg [5:0]  commit_phys0_comb, commit_phys1_comb;
//CSR 提交口（打拍前）：队头这一项是 CSR 就吐载荷
reg        commit_csr_comb;
reg [21:0] commit_csr_payload_comb;
//★ 2026-11-05（S5）：改向拆成两个域，边界只在【在册】的这一份寄存器里，不按项挂标记。
//  A = 判定拍：清前端 + 改 PC + 冻分配；B = 提交口：清整张表和它后面的后端。
//  两张按项的阵列（"这条要改向" + 落点）都删了 —— 落点 A 那一拍就锁进 pending_target，
//  提交口只需要知道"哪一项是边界"，一次 4 位比较就够。
reg        flush_boundary_valid;
reg [3:0]  flush_boundary_index;
//★ 判定口的候选（组合）：两条 lane 各算一次年龄，取更老的；再和【在册边界】比一次。
reg        judge_valid, judge_accept, judge_pick1;
reg [3:0]  judge_index, judge_age;
reg [31:0] judge_target;
//项状态阵列（v / wr / ex / data / jdg）：唯一被三个区共写的对象，只能有一个 always 块
//（在本区末尾）：分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
reg        entry_valid  [0:DEPTH-1];
reg        entry_written [0:DEPTH-1];
reg        entry_exception [0:DEPTH-1];
//每槽的结果数据：与 entry_written 同条件同一沿写，值读口/提交口读；退项/冲刷/陷阱不必清。
reg [31:0] entry_data [0:DEPTH-1];
//每槽的指令地址（= 该条指令地址 + 4，与 csr.mepec 的减 4 约定一致）。
//只服务"中断返回点"，所以只有队头那一项被读。
reg [31:0] entry_pc   [0:DEPTH-1];
reg        entry_judged  [0:DEPTH-1];
//mret 标签：第二种【队头改向】，落点不由项携带（取 mepc）
reg        entry_mret [0:DEPTH-1];
//head 拍判定出来的事件（组合）。下一沿变成对外的重定向事件 —— 冲刷从这排触发器出去。
reg        mret_now, exc_now;
//B 事件的边界随事件一起打一拍。
//dly 里现在是否有事件（= 本拍正在冲刷）。它挡住队头写回和新的装载。
wire       delayed_busy = redirect_valid_delayed | exception_valid_delayed;
wire [15:0] flush_kill_mask_d1, flush_kill_mask_d2;
reg [31:0] entry_target  [0:DEPTH-1];
//★ S4：异常载荷【逐槽存】，不再是一份侧寄存器。
//  侧寄存器只能记一条 ⇒ 窗口里第二条故障会把第一条盖掉（实测 seed1：mepc 记成后一条的），
//  而"从标记到交付"之间本来就允许多条故障同时在册。
//  cause 用新增的 4 位阵列（16×4=64 FF）；pc / tval 直接复用【本槽已经闲置的两个 32 位域】：
//   · entry_data：故障项永远等不到完成口（它不会再有 acc_*），这个域对它恒空 ⇒ 存故障 PC；
//   · entry_target ：只有判定口会写它，而判定口只写 entry_jump 的项；访存类故障项 entry_jump=0 ⇒ 存 tval；
//               分支/跳转类故障（目标非对齐）entry_jump=1，但那一拍的 bju_tgt 就是那个坏目标 ⇒ 同值。
reg [3:0]  entry_exc [0:DEPTH-1];
//==============================================================================
//  幸存者表 rb_* —— idu2 用它重建 pstat（冲刷后第 2 拍取用）
//==============================================================================
//★ 时相：本表是每拍都在的"当前在册项"，**不再是"冲刷那一拍的组合结果"**。
//  冲刷在 F 沿已把 entry_valid/entry_written/entry_exception 按年龄清干净（见下面 entry_valid 写口那一段），
//  所以 F 之后任何一拍读到的 entry_valid 就是幸存者本身 ⇒ 冲刷锥与这张表彻底脱钩。
//  取用时刻由 idu2 的 reb_d2 决定（冲刷后第 2 拍），这里只管"报当前状态"。
//★ 原来那段 flush_ok / flush_age / 16 项年龄比较的 kill 判据【整段删除】：
//  它计算的事情时钟沿上已经做过了，留着只是把冲刷锥再拖一遍。
//★ 只保留【窗口上界】age < entry_count 一项：它是"在册"的判据，兼管两件事 ——
//  ① 陷阱交付（exception_valid_delayed）只回卷指针、不清 entry_valid；② 已退项在 head 之前，age 回绕成
//  大数，必须靠它挡掉（否则会把已退休的写者当成活写者登记回去）。
//==============================================================================
//  提交区（head 端）：退役判据、冲刷、陷阱、提交口（写寄存器堆）
//==============================================================================
//退口判据 / 冲刷判据 / 状态输出
//  可退 = 在册且已完成。exc 标记项不算可退：它不写寄存器堆，由 exception_valid_delayed 交上级；
//  同拍标记（exc_en 正落在 head/head+1 上）必须【组合】就压掉退，否则会在 entry_exception 落盘前退掉。
//★ S5：改向分两个域 —— A 在判定拍（清前端 + 改 PC + 冻分配），B 在提交口（清整表 + 后端）。
//  判定口给的东西（bju_judged / bju_index / bju_redir / bju_tgt）都是 exu 的 *_registered，
//  同拍同源 ⇒ 不需要再为边界打拍。
//★ 异常/中断优先：队头是故障项时，本拍不放误预测改向（异常那条在程序序上更老，
//  它的 mepc 必须取故障指令自己的 pc；被改向一掺就错）。分支的边界留在在册寄存器里，
//  异常交付那一拍整表清空、边界一起撤。
    wire head_is_exception = (entry_count != 4'd0) && entry_valid[head_pointer] && entry_exception[head_pointer];
    wire [3:0] judge0_age = bju_index             - head_pointer;
    wire [3:0] judge1_age = branch_unit_lane1_index - head_pointer;
    reg        boundary_now;

always @(posedge clk) reset_registered <= rst;

integer ai;

//重建口：物理槽序，静态下标（零 mux）

integer ri;
integer si;
integer rp;

//==============================================================================
//  分配区（tail 端）：这一拍进几条、进到哪个槽、槽里写什么
//==============================================================================
//槽号与世代是组合输出的环位，作为载荷沿流水线传下去。
//★ lane1 的落位取决于 lane0 这一拍分不分配（两条 lane 独立推进，可能只有 lane1 来）。

always @(*) begin
        allocation_index_0 = tail_pointer;
        allocation_index_1 = (tail_pointer + {3'b000, allocation_en_0}) & DEPTH_MASK;
        allocation_generation_0 = ~entry_generation[tail_pointer];
        allocation_generation_1 = ~entry_generation[allocation_index_1];
        occupancy  = entry_count;
        empty      = (entry_count == 4'd0);
        full       = (entry_count >= DEPTH - 4'd1);
    end

//下一拍的指针与占用数：先算退掉的（0/1/2 笔），再算分配的，冲刷最后覆盖。
//★ 判据必须与提交区的项状态块【逐字一致】：边界项失效时走"全冲"，这里也要把 entry_count/tail
//  归零，否则"项全清、计数还留着" ⇒ head 指向无效项 ⇒ 队头永不动。
    always @(*) begin
        entry_count_next  = entry_count;
        tail_pointer_next = tail_pointer;
        tail_tag_next = tail_tag;
        if (head_ok) begin
            entry_count_next = entry_count_next - 4'd1;
        end
        if (head_next_ok) begin
            entry_count_next = entry_count_next - 4'd1;
        end
        if (allocation_en_0 && !full && !flush_control_rob) begin
            tail_pointer_next = (tail_pointer_next + 4'd1) & DEPTH_MASK;
            tail_tag_next = tail_tag_next + 5'd1;
            entry_count_next  = entry_count_next + 4'd1;
        end
        if (allocation_en_1 && !full && !flush_control_rob) begin
            tail_pointer_next = (tail_pointer_next + 4'd1) & DEPTH_MASK;
            tail_tag_next = tail_tag_next + 5'd1;
            entry_count_next  = entry_count_next + 4'd1;
        end
        if (flush_control_rob) begin
//★ 2026-11-05：改向 / 异常 / 中断都在提交口发，前端和整张 ROB 同一拍一起作废 ⇒ 恒为整窗。
            entry_count_next  = 4'd0;
            tail_pointer_next = head_pointer;
            tail_tag_next = entry_tag[head_pointer];
        end
//陷阱交付：队头那条故障指令【不写寄存器堆】地退掉，比它年轻的项全部作废。
//★ 必须放在【冲刷赋值之后】：交付那一拍冲刷口可能同时有效，不能被 flush_keep 盖掉。
        if (exception_valid_delayed) begin
            tail_pointer_next = (head_pointer + 4'd1) & DEPTH_MASK;
            entry_count_next  = 4'd0;
            tail_tag_next = entry_tag[head_pointer] + 5'd1;
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            tail_pointer <= 3'd0;
            entry_count  <= 5'd0;
            tail_tag <= 5'd0;
        end
        else begin
            tail_pointer <= tail_pointer_next;
            entry_count  <= entry_count_next;
            tail_tag <= tail_tag_next;
        end
    end

//入项：rd / 世代 / 是否会跳 / 会不会报完成口 —— 这四张表只有【复位 + 分配】两个写源。
    always @(posedge clk) begin
        if (reset_registered) begin
            for (ai = 0; ai < DEPTH; ai = ai + 1) begin
                entry_tag[ai]  <= 5'd0;
                entry_destination_register[ai]   <= 5'd0;
                entry_phys[ai]   <= 6'd0;
                entry_old_phys[ai] <= 6'd0;
                entry_generation[ai]  <= 1'b0;
                entry_jump[ai]  <= 1'b0;
                entry_pending[ai] <= 1'b0;
                entry_system[ai]  <= 1'b0;
                entry_csr[ai]     <= 1'b0;
                entry_csr_payload[ai] <= 22'd0;
            end
        end
        else begin
            if (allocation_en_0 && !full && !flush_control_rob) begin
                entry_tag[tail_pointer]  <= tail_tag;
//前送表的目的寄存器号（保守侧：SYSTEM 也算写，带上 rd != 0 后只剩非法编码会"报了写其实不写"）。
                entry_destination_register[tail_pointer]   <= (allocation_we_0 && (allocation_rd_0 != 5'd0)) ? allocation_rd_0 : 5'd0;
                entry_phys[tail_pointer]                   <= (allocation_we_0 && (allocation_rd_0 != 5'd0)) ? allocation_phys_0 : 6'd0;
                entry_old_phys[tail_pointer]               <= (allocation_we_0 && (allocation_rd_0 != 5'd0)) ? allocation_old_phys_0 : 6'd0;
                entry_generation[tail_pointer]  <= ~entry_generation[tail_pointer];
                entry_jump[tail_pointer]  <= allocation_jmp_0;
                entry_pending[tail_pointer] <= allocation_pend_0;
                entry_system[tail_pointer]  <= allocation_ecall_0 | allocation_ebreak_0 | allocation_illegal_0 | allocation_mret_0;
                entry_csr[tail_pointer]     <= allocation_csr_0;
                entry_csr_payload[tail_pointer] <= allocation_csr_payload_0;
                entry_pc[tail_pointer]      <= allocation_pc_0;
            end
            if (allocation_en_1 && !full && !flush_control_rob) begin
                entry_tag[allocation_index_1] <= tail_tag + {4'd0, allocation_en_0};
                entry_destination_register[allocation_index_1]   <= (allocation_we_1 && (allocation_rd_1 != 5'd0)) ? allocation_rd_1 : 5'd0;
                entry_phys[allocation_index_1]                   <= (allocation_we_1 && (allocation_rd_1 != 5'd0)) ? allocation_phys_1 : 6'd0;
                entry_old_phys[allocation_index_1]               <= (allocation_we_1 && (allocation_rd_1 != 5'd0)) ? allocation_old_phys_1 : 6'd0;
                entry_generation[allocation_index_1]  <= ~entry_generation[allocation_index_1];
                entry_jump[allocation_index_1]  <= allocation_jmp_1;
                entry_pending[allocation_index_1] <= allocation_pend_1;
                entry_system[allocation_index_1]  <= allocation_ecall_1 | allocation_ebreak_1 | allocation_illegal_1 | allocation_mret_1;
                entry_csr[allocation_index_1]     <= allocation_csr_1;
                entry_csr_payload[allocation_index_1] <= allocation_csr_payload_1;
                entry_pc[allocation_index_1]      <= allocation_pc_1;
            end
        end
    end

//S5 判定口的两条 lane（组合）：这一拍判定出来、真要改向的项。
//  判据与旧结构逐项同门，只是把"判定过 && 要改向"两项换成当拍判定口给出的值。
    wire judge0_ok = bju_judged & entry_jump[bju_index] & bju_redir &
                     entry_valid[bju_index] & ~entry_exception[bju_index] &
                     (entry_generation[bju_index] == bju_generation);
    wire judge1_ok = branch_unit_lane1_judged & entry_jump[branch_unit_lane1_index] &
                     branch_unit_lane1_redir &
                     entry_valid[branch_unit_lane1_index] & ~entry_exception[branch_unit_lane1_index] &
                     (entry_generation[branch_unit_lane1_index] == branch_unit_lane1_generation);

    always @(*) begin
        head_next_pointer   = (head_pointer + 4'd1) & DEPTH_MASK;
//★ 冲刷那一拍一律不退：退是组合的，而冲刷的清位要到边沿才生效 ⇒ 错路那条若正好在 head 上
//  且已 done，会在冲刷拍就写进寄存器堆。
//★ 两拍都要挡住提交：当拍是冲刷正在发生，下一拍是 ROB 还没清完。
//★ S3 提交口改向：队头这一项是"判错过了"的分支 ⇒ 本拍改 PC + 杀掉它之后的全部。
//  队头即边界 ⇒ "在途项都比边界年轻"成立，消费端不必再做年龄比较。
//  ★ 判定在 head 拍、事件在下一拍：本拍只允许"改向那条自己退休"，事件由下一拍的触发器出。
//    冲刷那一拍（事件拍）一律不退 —— flush_any = flush_control_rob。
//★ mret 与 bju 挤在同一个出口：两者都是"队头这一项自己要走、且要走的路不是顺序下一条"。
//  落点不同 —— bju 的落点在项里（entry_target，判定拍就算完）；mret 的落点是 csr 的 mepc 锁存器，
//  本模块不碰：它只报"这一笔改向是 mret"，pc 在事件那一拍直接取 mepc。
//  ★ 相位刚好够：事件在队头+1 出，而 `csrw mepc` 的写也在那一沿落盘（提交口影子晚一拍）⇒ pc 采到的是新值。
        // ★ 判定直接读队头那一项（阵列）。写回在队头这一拍完成，冲刷事件只进 dly、下一拍才出。
        // ★ dly 非空时既不出新事件，也不许退休 —— 这一拍的队头是更年轻的错路项。
        flush_any = flush_control_rob | delayed_busy;
        mret_now  = (entry_count != 4'd0) && entry_valid[head_pointer] && entry_mret[head_pointer] &&
                    !entry_exception[head_pointer] && !flush_any;
        exc_now  = (entry_count != 4'd0) && entry_valid[head_pointer] && entry_exception[head_pointer] && !flush_any;
//★ S5：判定拍只出 A（清前端 + 改 PC + 冻分配），**不改队头退休** —— 边界那条要走到队头
//  自己退休才发 B。所以这里不再有 redirect_now 这种东西。
//★ 改向那一拍【要退】：队头就是那条分支、它是正确路径，改向/退休/杀后续是同一个事件。
//  吃 flush_any 的话队头永远退不掉（这次冲刷是它自己发的）⇒ 自锁。
//★ exc_gone 随部署删除：异常改成【按项标记】之后，标记落在 entry_exception[exc_index] 上，
//  项在交付前不会消失（head_ok 的 !entry_exception 把它挡在队头）⇒ 不存在"边界项已退"这一档。
//★ 三条标记口各自成项、不合成一根：命中队头就在 head_ok 里逐条取反，同拍标记必须走标记口。
        head_ok   = (entry_count != 4'd0) && entry_valid[head_pointer] && entry_written[head_pointer] &&
                    (entry_judged[head_pointer] || (!entry_jump[head_pointer] && !entry_system[head_pointer])) &&
                    !entry_exception[head_pointer] &&
                    !(exc0_en  && (exc0_index  == head_pointer))  &&
                    !(exc1_en  && (exc1_index  == head_pointer))  &&
                    !(exception_selected_enable && (exception_selected_index == head_pointer))  &&
                    !flush_any;
//★ 边界那条【不许同拍从 head+1 一起退】：它当上队头、单独退休，下一拍才发 B。
//  不然错路的后继会跟着它一起退掉、写进寄存器堆（实测 seed5：jal 后面那条 lui 提交了）。
        head_next_ok  = head_ok && !mret_now && !exc_now &&
                    !(flush_boundary_valid && (head_pointer == flush_boundary_index)) &&
                    !entry_mret[head_next_pointer] &&
                    (entry_count >= 4'd2) && entry_valid[head_next_pointer] && entry_written[head_next_pointer] &&
                    (entry_judged[head_next_pointer] || (!entry_jump[head_next_pointer] && !entry_system[head_next_pointer])) &&
                    !entry_exception[head_next_pointer] &&
                    !(exc0_en  && (exc0_index  == head_next_pointer))  &&
                    !(exc1_en  && (exc1_index  == head_next_pointer))  &&
                    !(exception_selected_enable && (exception_selected_index == head_next_pointer));
//★ 退休回报必须写在 head_ok/head_next_ok 之后：同一个 always @(*) 块里【先读后写】同一个信号，
//  @* 不保证收敛（实测 iverilog 下 ret0_v 恒定慢一拍）⇒ lsu 的 SQ 记不上"已退休"⇒ 死锁。
        ret0_v   = head_ok;
        ret0_index = head_pointer;
        ret0_generation = entry_generation[head_pointer];
        ret1_v   = head_next_ok;
        ret1_index = head_next_pointer;
        ret1_generation = entry_generation[head_next_pointer];
//------------------------------------------------------------------------------
// S5 判定拍（A 域）：候选 = 这一拍判定出来、真要改向的分支项
//  · 两条 lane 同拍判定取更老的：年轻的那条在错路上，会被这次改向带走（一次 4 位比较）。
//  · 只认在册且世代对得上的项 —— 迟到判定不许改向。
//  · 年龄 = 槽号 - 队头。在册窗口是连续的一段、跨度 ≤ DEPTH ⇒ 这个差就是名次，4 位减法够。
//  · 和在册边界比一次：候选更老才受理 ⇒ 同时满足"新的不许压老的"和"老的可以顶掉新的"。
//------------------------------------------------------------------------------
        judge_pick1  = judge1_ok & (!judge0_ok | (judge1_age < judge0_age));
        judge_index  = judge_pick1 ? branch_unit_lane1_index : bju_index;
        judge_target = judge_pick1 ? branch_unit_lane1_tgt : bju_tgt;
        judge_age    = judge_pick1 ? judge1_age : judge0_age;
        judge_valid  = (judge0_ok | judge1_ok) & !head_is_exception & !flush_any;
        judge_accept = judge_valid &
                       (!flush_boundary_valid | (judge_age < (flush_boundary_index - head_pointer)));
//B 事件：在册边界走到提交口。两条 lane 都认 —— 它从 head+1 退掉也照样发。
        boundary_now = flush_boundary_valid &
                       ((head_ok      & (head_pointer      == flush_boundary_index)) |
                        (head_next_ok & (head_next_pointer == flush_boundary_index)));
//★ 交付【不能】要求 entry_written：被 lsu 拒绝的访存非对齐故障指令永远等不到完成口，
//  要 done 就既不退也不交付 ⇒ 整核死等。它排到队头就说明比它老的都退完了。

//陷阱载荷取【故障项自己那一份】（不是"当前哪条又出错了"）：队头就是程序序里最老的故障指令。
//★ 同拍命中（exc_hit_head）必须走标记口那一路 —— 侧寄存器此刻还是上一条异常留下的旧值。
//★ S4：队头取【它自己那一槽】的载荷。同拍标记（exc_hit_head）以及"边界项已退"（exc_gone）
//  这两条路还走组合口 —— 那一沿 entry_exception/entry_exc 还没落盘，只能吃当拍的 exc_*。

//★ 边界必须在【占用窗口内】才算"部分冲刷"：age(flush_index) < entry_count。只查 entry_valid 不够 ——
//  已退槽的 valid 位可能是脏的，边界落在窗口外时 age 的回绕会算出怪值，ROB 状态直接烂掉。
//  窗口外一律按"全冲"处理（活着的项本来就都比它年轻）。
//★ 全杀：边界这个概念不存在了（改向那一条在上一拍已经退掉，ROB 里剩下的全是更年轻的项）。
//提交口：只有队头两项可退，且【写 rd 的项】才写寄存器堆。
//★ entry_destination_register != 0 与 head_ok 里的 !entry_exception 是两道独立的门：故障项的 entry_destination_register 可能非 0。
        commit_we0_comb   = head_ok && (entry_destination_register[head_pointer] != 5'd0);
        commit_rd0_comb   = entry_destination_register[head_pointer];
        commit_phys0_comb = entry_phys[head_pointer];
        commit_we1_comb   = head_next_ok && (entry_destination_register[head_next_pointer] != 5'd0);
        commit_rd1_comb   = entry_destination_register[head_next_pointer];
        commit_phys1_comb = entry_phys[head_next_pointer];
//CSR：队头这一项是 CSR ⇒ 载荷随提交口出去。1 宽就够 —— 同时至多一条 CSR 在飞
//（idu2 的 csr_held 要求它发射时就是队头），同拍不可能有第二条在 head+1 上等。
        commit_csr_comb         = head_ok && entry_csr[head_pointer];
        commit_csr_payload_comb = entry_csr_payload[head_pointer];

//★ 2026-11-04 真 PRF：冲刷时把【在册全部项】的物理号各展成 one-hot 再 OR，直接交给 rename 回收。
//  kill_arc 已删（每次冲刷整表作废、没有"部分冲刷"），所以这里不需要按边界挑。
//  没写回的项 entry_phys 恒 0，贡献的是 bit0（物理 0 = x0，rename 那边会掩掉）。
        squash_phys_mask = 64'd0;
        squash_survive_mask = 64'd0;
        for (si = 0; si < DEPTH; si = si + 1)
            if (entry_valid[si]) begin
                if (flush_kill_mask_d1[si])
                    squash_phys_mask = squash_phys_mask | (64'd1 << entry_phys[si]);
                else
                    squash_survive_mask = squash_survive_mask | (64'd1 << entry_phys[si]);
            end

//★ 2026-11-05 NaxRiscv 口径：rat 的回滚不再靠"挑最老被杀项的 old_phys"。
//  改向一经判定就冻住分配（见 redirect_stop_alloc），被杀项的号直接回收，
//  rat 在冻住期间不可能被任何新项读到 ⇒ 提交拍整片抄回 arch_map 就够。
//  端口保留（rename 已经不接），恒 0。
        squash_arch_hit  = 32'd0;
        squash_arch_phys = 192'd0;
//★ 幸存掩码照旧：prf 的 ready 要靠它保住没被杀的在飞项。
//  survive_hit 是给 rat 用的，现在归零（同上）。
        squash_survive_hit = 32'd0;
    end

//队头字段副本的装载：每个沿装【下一拍的队头】那一项（head_pointer_next 正是 head_pointer 在下一沿的值，
//两者必然同指一项）。纯读阵列 → 寄存器，锥止于触发器。
//★ 改向/陷阱事件打一拍：head 那一拍只做判定，重定向信号从触发器出去。
//  这一拍也是改向那条自己退休的那一拍；下一拍全核冲刷（清空整表）。
//★ mret 不带落点：mepc 是 csr 里的架构状态，不是指令自带的立即数 —— 由 pc 在事件那一拍
//  直接取 csr 的 mepc 锁存器（cpu_top: fetch_jump_target）。本模块只报"这一笔是 mret 改向"
//  （mret_valid_delayed），不再把它搬进 redirect_target_delayed。
    always @(posedge clk) begin
        if (reset_registered) begin
            redirect_valid_delayed <= 1'b0; mret_valid_delayed <= 1'b0; redirect_target_delayed <= 32'd0;
            redirect_front_valid <= 1'b0;
            exception_valid_delayed <= 1'b0; exception_cause_delayed <= 4'd0; exception_pc_delayed <= 32'd0; exception_trap_value_delayed <= 32'd0;
        end
        else begin
//A 域（判定拍 / mret）：改 PC + 清前端。**不吃 delayed_busy**：
//  judge_accept 自己已经吃过 flush_any，再门一次会把刚受理的边界吞掉。
            redirect_front_valid <= judge_accept | mret_now;
            if (judge_accept)  redirect_target_delayed <= judge_target;
            if (!delayed_busy) begin
//B 域（提交口）：在册边界走到队列头 ⇒ 整表 + 后端一起作废。
//★ mret 的 B 跟 A 同拍 —— 两把刀必须同拍，不然清表和改 PC 错开。
                redirect_valid_delayed   <= boundary_now | mret_now;
                mret_valid_delayed    <= mret_now;
                exception_valid_delayed <= exc_now;
                if (exc_now) begin
                    exception_cause_delayed <= entry_exc [head_pointer];
                    exception_pc_delayed    <= entry_data[head_pointer];
                    exception_trap_value_delayed  <= entry_target [head_pointer];
                end
            end
            else begin
                redirect_valid_delayed   <= 1'b0;
                mret_valid_delayed    <= 1'b0;
                exception_valid_delayed <= 1'b0;
            end
        end
    end

//在册边界：判定拍受理时锁下，提交口（以及异常/中断交付）那一拍撤掉。
//  ★ 撤在 flush_control_rob：B / 异常 / 中断 三种都把整张表清了，边界自然不存在。
//    同一拍来的新候选一律不受理（它比边界年轻，本来就在被杀的那一段里）。
    always @(posedge clk) begin
        if (reset_registered) flush_boundary_valid <= 1'b0;
        else if (flush_control_rob) flush_boundary_valid <= 1'b0;
        else if (judge_accept) begin
            flush_boundary_valid <= 1'b1;
            flush_boundary_index <= judge_index;
        end
    end

//★ 这两个都打一拍给 idu2：判定用【当拍】的 ROB 状态算（边界还在不在窗口里 / 队头是不是故障项），
//  算完才寄存。idu2 拿它们去杀 D/Q，晚一拍做没有副作用 —— 那几拍前端本来就不送新指令。
//  ★ 为什么要寄存：exception_valid_delayed 是组合出口、在 idu2 里扇出 ~128，它和 flush_partial 一起构成
//    "cont 仲裁 → rob 边界判定 → idu2 击杀+操作数选择" 那条跨三模块的长链。
//击杀弧：第 i 位 = 槽 i 的年龄 > 边界年龄。16 个比较器全在本模块里，布线是短的；
//寄存出去之后消费端只做一次 16:1 查表。
    integer ka;
//★ kill_arc 已删（2026-10-31）：它每一位都是 1'b1（本设计每次冲刷都整表作废），
//  在消费端早被综合折掉。这里直接不再生成，`slot_above_head` 保留（它是真信号）。
    always @(*) begin
        slot_above_head    = 16'd0;
        for (ka = 0; ka < DEPTH; ka = ka + 1) begin
//★ 击杀弧恒为「全杀」：本设计里每一种冲刷都把 ROB 整表清空 ——
//  改向/陷阱边界恒等于队头；中断的边界是 irq_head_index（也是队头）且 flush_incl=1（连边界一起杀）。
//  所以「比边界年轻」对任何在册项都成立，16 个 older() 比较器 + entry_tag[flush_index_eff] 那次
//  16:1 读全是恒真逻辑，只剩一条长锥挂在 idu1/mdu/lsu/bju 的击杀门上。
            slot_above_head[ka]    = (ka[3:0] >= head_pointer);
        end
    end

//击杀掩码：第 i 位 = 槽 i 这一拍要被作废。
//★ 2026-11-05：改向只在提交口发一次、前端和整张 ROB 一起作废 ⇒ 每一次冲刷都是整窗的，
//  掩码恒全亮。原来那套「按边界挑严格更年轻的项」（16 个环上年龄比较 + alloc_slot_hot
//  补同拍新分配的项）整块删掉 —— 它存在的唯一理由就是"判定拍提前杀"，而那一刀已经放弃。
//  端口留着（lsu / mdu / exu / idu1 / idu2 / prf 的端口没动），综合折成常数。
    assign flush_kill_mask_d1 = 16'hFFFF;
    assign flush_kill_mask_d2 = 16'hFFFF;
//★ S5：判定拍发 A 之后分配就冻住 —— 前端会一直取到正确路径、停在分配口。
//  解冻在提交口那一拍之后：那一拍 flush_control_rob 本来就挡着分配，所以不会漏进错路项。
    assign redirect_stop_alloc = flush_boundary_valid | flush_pend_in;
    assign squash_kill_mask            = flush_kill_mask_d1;
    assign squash_kill_mask_registered = flush_kill_mask_d2;

//==============================================================================
//  中断返回点：队头的指令地址（组合读）
//  ★ 逐项列成 case，不写 entry_pc[head_pointer]：数组元素在 always @(*) 里进不了敏感表
//    （本项目实测过，见 idu2 值表读口的同款注释）。
//  ★ 组合而非寄存：队头同一拍可能刚退休，寄存版会把已退休那条当成返回点 ⇒ 它被执行两次。
//==============================================================================
    always @(*) begin
        head_pc_out = entry_pc[head_pointer];
        head_valid_out = entry_valid[head_pointer];
    end

//★ S5：改向在册（寄存一拍）给 idu2 挡新的访存发射。
//  就是"边界在册"本身 + 事件那片 dly —— 16 项扫描整块删掉。
//  挡"新发"就够：已在 s2/s3 的那几条会在提交口那一拍被击杀掩码杀干净。
    always @(posedge clk) begin
        if (reset_registered) redirect_any <= 1'b0;
        else                  redirect_any <= flush_boundary_valid | redirect_valid_delayed;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            slot_above_head_registered      <= 16'd0;
            head_one_hot_registered    <= 16'd0;
            retire_step     <= 2'd0;
        end
        else begin
            slot_above_head_registered      <= slot_above_head;
            head_one_hot_registered    <= 16'd1 << head_pointer_next;
            retire_step     <= retire_step_next;

        end
    end

//队头指针：退 0/1/2 笔；陷阱交付也前进一格（盖过上面）。
// P1: compute the advance as retire_step once, then add. The decision lives here only;
// head_pointer's D side is now a 4-bit increment, and csr counts with retire_step.
    always @(*) begin
        if (head_next_ok)      retire_step_next = 2'd2;
        else if (head_ok)  retire_step_next = 2'd1;
        else               retire_step_next = 2'd0;
        if (exception_valid_delayed)     retire_step_next = 2'd1;
        head_pointer_next = (head_pointer + {3'b000, retire_step_next}) & DEPTH_MASK;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            head_pointer <= 3'd0;
        end
        else begin
            head_pointer <= head_pointer_next;
        end
    end

//提交口打一拍：把"要不要提交 / 提交哪个架构号 / 对应哪个物理号"在提交拍算好、寄存，
//  下一拍再交给 rename 的提交口（arch_map 定格 + 物理号回收）。
//★ 为什么打：arch_map 是变址写阵列，综合器表达不出 CE，只能合成 D_i = (we&&rd==i)?phys:Q_i
//  ⇒ head_ok 里的中断/异常/分支判定锥会整条压在阵列的 D 引脚上。打一拍后长锥止于此。
//★ 绝不能在 exception_valid_delayed / 冲刷拍清：head_ok 与 exception_valid_delayed 天然互斥（一个要 !entry_exception、一个要 entry_exception）；
//  冲刷只作废比边界年轻的项，而打拍里那笔在边界之前 ⇒ 清了会丢一笔合法写。
    always @(posedge clk) begin
        if (reset_registered) begin
            commit_write_enable_0   <= 1'b0;
            commit_destination_0   <= 5'd0;
            commit_write_enable_1   <= 1'b0;
            commit_destination_1   <= 5'd0;
            commit_phys0 <= 6'd0;
            commit_phys1 <= 6'd0;
            commit_csr_valid <= 1'b0;
            commit_csr_payload <= 22'd0;
        end
        else begin
            commit_write_enable_0   <= commit_we0_comb;
            commit_destination_0   <= commit_rd0_comb;
            commit_phys0 <= commit_phys0_comb;
            commit_write_enable_1   <= commit_we1_comb;
            commit_destination_1   <= commit_rd1_comb;
            commit_phys1 <= commit_phys1_comb;
            commit_csr_valid   <= commit_csr_comb;
            commit_csr_payload <= commit_csr_payload_comb;
        end
    end

//★ 2026-11-05：恢复脉冲已删。rename / prf 的状态回滚由冲刷那一拍自己做
//  （rat <= arch_map、ready <= arch_next），和提交口同相位，不需要第二条通路。
    assign restore_valid = 1'b0;

//结果口的"接受条件"（与下面完成回填 entry_data 的判据逐字同门）
    always @(*) begin
        acc_alu_o  = alu_done  && entry_valid[alu_index]  && (entry_generation[alu_index]  == alu_generation);
        acc_alu2_o = alu_lane1_done && entry_valid[alu_lane1_index] && (entry_generation[alu_lane1_index] == alu_lane1_generation);
        acc_mul_o  = mul_done  && entry_valid[mul_index]  && (entry_generation[mul_index]  == mul_generation);
        acc_mul2_o = multiply_lane1_done && entry_valid[multiply_lane1_index] && (entry_generation[multiply_lane1_index] == multiply_lane1_generation);
        acc_div_o  = div_done  && entry_valid[div_index]  && (entry_generation[div_index]  == div_generation);
        acc_div2_o = divide_lane1_done && entry_valid[divide_lane1_index] && (entry_generation[divide_lane1_index] == divide_lane1_generation);
        acc_ld_o   = ld_done   && entry_valid[ld_index]   && (entry_generation[ld_index]   == ld_generation);
        acc_st_o   = st_done   && entry_valid[st_index]   && (entry_generation[st_index]   == st_generation);
    end

//------------------------------------------------------------------------------
//项状态阵列（v / wr / ex / data / jdg）：唯一被三个区共写的对象，只能有一个 always 块。
//分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
//------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset_registered) begin
            for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                entry_valid[ri]  <= 1'b0;
                entry_written[ri] <= 1'b0;
                entry_exception[ri] <= 1'b0;
                entry_exc[ri] <= 4'd0;
                entry_data[ri] <= 32'd0;
                entry_judged[ri]  <= 1'b0;
                entry_mret[ri]  <= 1'b0;
            end
        end
        else begin
//退掉的槽立刻失效；完成回填只认 valid 的槽。
            if (head_ok) begin
                entry_valid[head_pointer]  <= 1'b0;
                entry_written[head_pointer] <= 1'b0;
                entry_exception[head_pointer] <= 1'b0;
            end
            if (head_next_ok) begin
                entry_valid[head_next_pointer]  <= 1'b0;
                entry_written[head_next_pointer] <= 1'b0;
                entry_exception[head_next_pointer] <= 1'b0;
            end
            if (allocation_en_0 && !full && !flush_control_rob) begin
                entry_valid[tail_pointer]  <= 1'b1;
//★ 不写 rd 的项不由"出厂"给 wr，由 issue_enable_0 那一拍置位（见下）：发射级是 1 深 Q 之后
//  "执行一定早于退役"不再成立 —— 没发出去的分支会被队头直接退掉，跳转就永远不发生。
                entry_written[tail_pointer] <= 1'b0;
                entry_exception[tail_pointer] <= 1'b0;
                entry_judged[tail_pointer]  <= 1'b0;
//★ mret 标签【随分配落地】：和 entry_jump/entry_system 同源，都是 idu1 对指令字的译码位，
//  跟指令一起进项，不靠后面的边带脉冲补写。
//  原来那条路是 cont 的 mret_mark/mret_index（bju 判定级 1 拍脉冲 + 槽号），实测会被同一窗口里
//  别人的异常标志压掉 —— seed36 第二遍处理器：非法指令的 exc_illegal_q 正好盖住 mret 的唯一那拍
//  ⇒ 标签丢了 ⇒ 那条 mret 当普通指令退休、PC 顺延到下一条，整段 body 重跑。
                entry_mret[tail_pointer]  <= allocation_mret_0;
            end
            if (allocation_en_1 && !full && !flush_control_rob) begin
                entry_valid[allocation_index_1]  <= 1'b1;
                entry_written[allocation_index_1] <= 1'b0;
                entry_exception[allocation_index_1] <= 1'b0;
                entry_judged[allocation_index_1]  <= 1'b0;
                entry_mret[allocation_index_1]  <= allocation_mret_1;
            end
//完成回填：wr 与 data 必须【同条件、同一沿】写入（拆开会出现"wr 已置、data 还是旧值"的一拍窗口）。
//★ 世代校验必需：被冲刷/已释放的槽在新住户身上的迟到上报只有它能挡（entry_valid 挡不住"新住户也 valid"）。
//★ "发出即置可退"只给【根本不会有完成口】的项：写 rd 的等完成口；M 类与 LOAD 即使 rd==0
//  也照样回报 ⇒ 必须排掉，否则提前退休后那个晚到的完成口会砸到槽的新住户头上。
            if (issue_enable_0 && entry_valid[issue_index_0] && (entry_generation[issue_index_0] == issue_generation_0) &&
                (entry_destination_register[issue_index_0] == 5'd0) && !entry_pending[issue_index_0]) begin
                entry_written[issue_index_0] <= 1'b1;
            end
            if (issue_enable_1 && entry_valid[issue_index_1] && (entry_generation[issue_index_1] == issue_generation_1) &&
                (entry_destination_register[issue_index_1] == 5'd0) && !entry_pending[issue_index_1]) begin
                entry_written[issue_index_1] <= 1'b1;
            end
            if (acc_alu_o) begin
                entry_data[alu_index] <= alu_data;
                entry_written[alu_index] <= 1'b1;
            end
            if (acc_alu2_o) begin
                entry_data[alu_lane1_index] <= alu_lane1_data;
                entry_written[alu_lane1_index] <= 1'b1;
            end
//两条 lane 的判定口各一套：同拍两条都判定时，一条都不能丢（丢了 entry_judged 永远不置 ⇒ 队头卡死）。
//★ entry_judged 对【所有】项都置（只记"判定延迟级已过"），但只有跳转类与 SYSTEM 类在退休判据
//  里要求它：这两类的改向/异常标记正是在那一级出的，早了就会被丢（实测 WEIRD w68）。
//  ★ 其余项不许被它压住 —— store 一压，SQ 排空资格就晚一拍，紧跟的 load 会抢到总线读到旧值
//    （实测 CoreMark：0x80766c 读回 0x807d24，crclist/crcstate 崩）。
//  跳转类额外记 redir / tgt。
            if (bju_judged && entry_valid[bju_index] && (entry_generation[bju_index] == bju_generation)) begin
                entry_judged[bju_index] <= 1'b1;
//★ S5：判定拍只写"落点"（提交口的 tval 还要它）；"这条要改向"不再按项记，
//  边界只在 flush_boundary_* 一份里，年龄比较在判定口一次做完。
                if (entry_jump[bju_index]) begin
                    entry_target  [bju_index] <= bju_tgt;
                end
            end
            if (branch_unit_lane1_judged && entry_valid[branch_unit_lane1_index] && (entry_generation[branch_unit_lane1_index] == branch_unit_lane1_generation)) begin
                entry_judged[branch_unit_lane1_index] <= 1'b1;
                if (entry_jump[branch_unit_lane1_index]) begin
                    entry_target  [branch_unit_lane1_index] <= branch_unit_lane1_tgt;
                end
            end
            if (acc_mul_o) begin
                entry_written[mul_index]   <= 1'b1;
                entry_data[mul_index] <= mul_data;
            end
            if (acc_mul2_o) begin
                entry_written[multiply_lane1_index]   <= 1'b1;
                entry_data[multiply_lane1_index] <= multiply_lane1_data;
            end
            if (acc_div_o) begin
                entry_written[div_index]   <= 1'b1;
                entry_data[div_index] <= div_data;
            end
            if (acc_div2_o) begin
                entry_written[divide_lane1_index]   <= 1'b1;
                entry_data[divide_lane1_index] <= divide_lane1_data;
            end
            if (acc_ld_o) begin
                entry_written[ld_index]   <= 1'b1;
                entry_data[ld_index] <= ld_data;
            end
//store：进 SQ 那一拍回报"已落地"，ROB 这时才置可退（不再走"发出即可退"的捷径）。
//★ 没有这个完成口，store 会在还没进 LSU 的时候就退休 ⇒ SQ 项永远等不到排空资格（实测死锁）。
            if (acc_st_o) begin
                entry_written[st_index] <= 1'b1;
            end
//★ mret 必须是一发：不清的话事件还拉 ⇒ 冲刷永远不退、队头永远退不掉。
//★ 清在【它自己那一槽】上（mret_now 那一拍 head_pointer 就是它）。吃寄存版的事件信号会清到
//  邻居的槽上 —— 事件多压了一拍之后会清到后面两格，撞上一条真的 mret 就把它的改向吞掉。
//  标签本身在【分配】那一沿就由 allocation_mret_* 写好了，这里只管一发清零。
            if (mret_now) entry_mret [head_pointer] <= 1'b0;
//★ 标记陷阱时必须【同时置 wr】：故障指令的写被 pd 挡掉、也不会再有任何单元回报它，
//  只置 exc 不置 wr 它就退不掉 ⇒ ROB 排不空 ⇒ redir 等不到 rob_empty ⇒ 整核挂死。
//★ S4：标记即写项。必须排在所有 acc_*/判定口之后（同一槽同一沿，后写者赢）。
            if (exc0_en && entry_valid[exc0_index]) begin
                entry_exception  [exc0_index] <= 1'b1;
                entry_written  [exc0_index] <= 1'b1;
                entry_exc [exc0_index] <= exc0_cause;
                entry_data[exc0_index] <= exc0_pc;
                entry_target [exc0_index] <= exc0_tval;
            end
            if (exc1_en && entry_valid[exc1_index]) begin
                entry_exception  [exc1_index] <= 1'b1;
                entry_written  [exc1_index] <= 1'b1;
                entry_exc [exc1_index] <= exc1_cause;
                entry_data[exc1_index] <= exc1_pc;
                entry_target [exc1_index] <= exc1_tval;
            end
            if (exception_selected_enable && entry_valid[exception_selected_index]) begin
                entry_exception  [exception_selected_index] <= 1'b1;
                entry_written  [exception_selected_index] <= 1'b1;
                entry_exc [exception_selected_index] <= exception_selected_cause;
                entry_data[exception_selected_index] <= exception_selected_pc;
                entry_target [exception_selected_index] <= exception_selected_trap_value;
            end
            if (allocation_en_0 && !full && !flush_control_rob) entry_written[tail_pointer] <= 1'b0;
            if (allocation_en_1 && !full && !flush_control_rob) entry_written[allocation_index_1] <= 1'b0;
//★ 冲刷 = 整窗作废（改向 / 异常 / 中断都在提交口发的同一笔，边界恒为队头）。
//  ★ 冲刷拍分配口被 flush_control_rob 挡住，所以这里不会误清同拍新进来的项。
            if (flush_control_rob) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    entry_valid[ri]     <= 1'b0;
                    entry_written[ri]    <= 1'b0;
                    entry_exception[ri]    <= 1'b0;
                    entry_mret[ri]  <= 1'b0;
                end
            end
//陷阱交付拍：整条 ROB 作废。放在最后 ⇒ 它盖过同拍的一切（含"刚分配的那一项"）。
            if (exception_valid_delayed) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    entry_valid[ri]  <= 1'b0;
                    entry_written[ri] <= 1'b0;
                    entry_exception[ri] <= 1'b0;
                    entry_mret[ri] <= 1'b0;
                end
            end
        end
    end

//==============================================================================
// 真 PRF 写口的物理号 / 存活判据
//   连续赋值（不进 always @(*)）—— 数组元素在 always @(*) 里进不了敏感表，本仓踩过。
//==============================================================================
    assign prf_wb_phys_0 = entry_phys[prf_wb_index_0];
    assign prf_wb_phys_1 = entry_phys[prf_wb_index_1];
    assign prf_ld_phys   = entry_phys[prf_ld_index];
    assign prf_alu_early_phys_0 = entry_phys[prf_alu_early_index_0];
    assign prf_ld_early_phys    = entry_phys[prf_ld_early_index];
    assign prf_ld_early_live    = entry_valid[prf_ld_early_index] & (entry_generation[prf_ld_early_index] == prf_ld_early_generation);
    assign prf_alu_early_phys_1 = entry_phys[prf_alu_early_index_1];
    assign prf_wb_live_0 = prf_wb_en_0 & entry_valid[prf_wb_index_0] & (entry_generation[prf_wb_index_0] == prf_wb_generation_0);
    assign prf_wb_live_1 = prf_wb_en_1 & entry_valid[prf_wb_index_1] & (entry_generation[prf_wb_index_1] == prf_wb_generation_1);
    assign prf_ld_live   =                 entry_valid[prf_ld_index]   & (entry_generation[prf_ld_index]   == prf_ld_generation);

endmodule
