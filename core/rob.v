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
//         比它年轻的项全部作废，同时拉 kill_trap 把载荷交给上级 —— 精确异常。
//
// ★ 本文件按三个区组织（寄存器声明统一放在最前，Verilog 要求先声明后使用）：
//     分配区 —— tail 端：指针与占用、槽号与世代、入项时写进去的静态信息
//     提交区 —— head 端：退役判据、冲刷、陷阱、提交口（写寄存器堆）
//   项状态阵列（v / wr / ex / data / jdg）是唯一被三块共写的对象，只能有一个 always 块，
//   放在提交区末尾：分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
//
// ★ 与流水级的对应（本模块没有自己的流水级，是"全核的状态机"；每个口挂错一拍就是
//   "回填打到错项 / 标记落在已退的槽上"这类静默错）：
//   allocation_enable           —— 分配拍（iffu 出队格换新 / 该条进 idu 输出级那一拍）
//   alu/alu_lane1/mul/multiply_lane1/ld 完成口 —— 结果被写口级处置那一拍，比分配晚一拍起步，mul/ld 不定长
//   bju_judged/branch_unit_lane1_judged —— bju 判定寄存一拍后的 wb 拍（两条 lane 各一套）
//   exc_en             —— 该条在载荷里的那一拍（bju 的组合异常标记）
//   后端击杀      —— bju 判定寄存一拍后的 wb 拍；flush_index = 那条分支/跳转自己的项
//   kill_trap          —— 故障项排到队头那一拍，上级同拍锁 mepc/mcause/mtval 并重定向
//   full/empty/occupancy —— 组合状态；full 喂 cont 的发射级压制
//
//   ★ 年龄判据一律用 `age = {1'b0,(索引-head_pointer)}`：拼接的操作数是自决定的，保证减法
//     在 5 位（槽号宽度）里做完再扩成 6 位；直接写 `(a-b) < entry_count` 会按 6 位求值，回绕时判错。
//
// ★ 2026-10-30：拉式值读口（query_slot/query_data/query_ok 那 4 组）已整条删除。
//   发射级的操作数选值现在只有三档（立即数 / 本地值表 / queue_base），
//   那 4 组答案在 isq 里从来没有被读过 ⇒ ROB 这边的 16:1 entry_data 选择、
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
//★ 2026-11-09：加第 3 组（分配口 2 宽 → 3 宽，见 docs/SPEC_alloc3.md）。口 0 最老、口 2 最年轻。
    input        allocation_en_0,
    input        allocation_en_1,
    input        allocation_en_2,
    input [4:0]  allocation_rd_0,
    input [4:0]  allocation_rd_1,
    input [4:0]  allocation_rd_2,
    input        allocation_we_0,
    input        allocation_we_1,
    input        allocation_we_2,
//★ 2026-11-04 真 PRF：这一条被分到的物理号，跟着 ROB 项走。
//  提交时它定格进 arch_map；冲刷时把在册项的物理号一起还给空闲表。
    input [5:0]  allocation_phys_0,
    input [5:0]  allocation_phys_1,
    input [5:0]  allocation_phys_2,
//★ 2026-11-04（CSR 步骤 C）：CSR 指令（1 位/项）+ 它的载荷 {写使能 1, 地址 12, funct3 3, 源 6}。
//★ 2026-11-06 载荷瘦身：CSR 必须排到 ROB 队头才准发（isq 的 q_csr_block），所以同一时刻
//  最多一条 CSR 处在"已发射未退休"。载荷不必逐项存，【发射那一拍】锁一深就够；
//  发射之前的载荷一直带在 isq 的站项里（地址/funct3/写位/源物理号那边都有）。
//  352 FF → 27 FF。
    input        allocation_csr_0,
    input        allocation_csr_1,
    input        allocation_csr_2,
    input        allocation_jmp_0,
    input        allocation_jmp_1,
    input        allocation_jmp_2,
//M 类 / LOAD：即使 rd==0 也会回报完成口，不许在"发出即置可退"里退休
    input        allocation_pend_0,
    input        allocation_pend_1,
    input        allocation_pend_2,
//SYSTEM 类（ecall / ebreak / illegal / mret）：异常或改向标记在 bju 判定延迟级才出，
//这几类的退休必须等那一级过去；其余项不受影响。四根保持分开，只在判据里或。
    input        allocation_ecall_0, allocation_ebreak_0, allocation_illegal_0, allocation_mret_0,
    input        allocation_ecall_1, allocation_ebreak_1, allocation_illegal_1, allocation_mret_1,
    input        allocation_ecall_2, allocation_ebreak_2, allocation_illegal_2, allocation_mret_2,
//分到的槽号与世代（随载荷走到执行单元，完成上报时带回校验）
    output reg [4:0]  allocation_index_0,
    output reg [4:0]  allocation_index_1,
    output reg [4:0]  allocation_index_2,
    output reg        allocation_generation_0,
    output reg        allocation_generation_1,
    output reg        allocation_generation_2,

//完成口：写口级处置完一笔就回报（落地 / 被杀 / 不写），data / gen 与 done / idx 同源同拍
    input        bju_judged,
    input [4:0]  bju_index,
    input        bju_generation,
    input        branch_unit_lane1_judged,
    input [4:0]  branch_unit_lane1_index,
    input        branch_unit_lane1_generation,
//★ S3：判定要不要改向 + 落点。跟着判定口一起进来，写进项里；队头到了才真正改 PC。
    input        bju_redir,
    input [31:0] bju_tgt,
    input        branch_unit_lane1_redir,
    input [31:0] branch_unit_lane1_tgt,
    input        alu_done,
    input [4:0]  alu_index,
    input [31:0] alu_data,
    input        alu_generation,
    input        alu_lane1_done,
    input [4:0]  alu_lane1_index,
    input [31:0] alu_lane1_data,
    input        alu_lane1_generation,
    input        mul_done,
    input [4:0]  mul_index,
    input [31:0] mul_data,
    input        mul_generation,
    input        multiply_lane1_done,
    input [4:0]  multiply_lane1_index,
    input [31:0] multiply_lane1_data,
    input        multiply_lane1_generation,
    input        div_done,
    input [4:0]  div_index,
    input [31:0] div_data,
    input        div_generation,
    input        divide_lane1_done,
    input [4:0]  divide_lane1_index,
    input [31:0] divide_lane1_data,
    input        divide_lane1_generation,
    input        st_done,
    input [4:0]  st_index,
    input        st_generation,
    input        ld_done,
    input [4:0]  ld_index,
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
    input [4:0]  issue_index_0,
    input        issue_generation_0,
    input        issue_enable_1,
    input [4:0]  issue_index_1,
    input        issue_generation_1,
//★ 2026-11-06：CSR 载荷随【发射】那一拍过来（CSR 必须排到队头才准发 ⇒ 同拍最多一条）
    input [21:0] issue_csr_payload_0,
    input [21:0] issue_csr_payload_1,
//冲刷口：作废比边界更年轻的项；flush_incl = 边界那条也一起作废。
//★ 本模块自己照旧吃【当拍】的冲刷记录：边界是否还在窗口里（flush_ok）必须拿当拍的 ROB 状态判，
//  晚一拍窗口已经动了，会把"部分冲刷"误判成"全冲" ⇒ 把比边界老的活项一起抹掉 ⇒ 死锁（实测）。
//  要打拍的是【本模块 → isq】那一跳：flush_partial_registered 是本模块当拍算好的判定，寄存一拍给 isq 用。
    input        interrupt_kill,     // 中断受理那一拍同样整窗作废（cont 给）
//★ S5：冲刷记录的寄存版（cont 的 flush_pending，晚一拍）。只喂"停分配"这一根 ——
//  idu 的载荷清零吃的是它，分配不停就会把同拍刚推进去的正确路径指令擦掉（实测 seed1）。

//异常标记口 ×3：lane0 / lane1 / lsu。
//每条异常属于它自己那一项，各带各的槽号 ⇒ 三路同时到达也互不相干，本模块不做任何仲裁。
//(cause,pc,tval) 随故障指令自己的项存下，退到队头才交付。
    input        exc0_en,
    input [4:0]  exc0_index,
    input [3:0]  exc0_cause,
    input [31:0] exc0_pc,
    input [31:0] exc0_tval,
    input        exc1_en,
    input [4:0]  exc1_index,
    input [3:0]  exc1_cause,
    input [31:0] exc1_pc,
    input [31:0] exc1_tval,
    input        exception_selected_enable,
    input [4:0]  exception_selected_index,
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
//★ 2026-11-09：冲刷掩码 squash_phys_mask 整条删。
//  它原来把在册全部项的物理号各展 one-hot 再 OR，交给 rename 回收；但 rename 的回收
//  早改成 flush_free_mask = ~arch_next_mask（rename.v），掩码口在那边只剩一句注释、
//  一次都没被读 ⇒ 这里是纯白算的 64 个译码器 × DEPTH 项（单独综合实测 899 LUT）。
//★ 2026-11-08：击杀掩码那两根输出已摘。改向只在提交口发一次 ⇒ 每次冲刷都是整窗的，
//  掩码恒为 MASK_ALL 常量；各消费端改成判"这一拍有没有冲刷"，等价且不用查表。
//★ 精确冲刷：每个架构号在【击杀集合】里最老那条的 old_phys（回滚 rat 用）。
//  hit 位 = 该架构号有被杀项写过；phys = 那条项的 old_phys。没被杀到就保持 rat 现值。
    output reg [31:0]  squash_arch_hit,
    output reg [191:0] squash_arch_phys,
//★ 精确冲刷：这个架构号有没有【幸存】在飞项写过。有 ⇒ rat 保持现值（它正指着那个幸存项）；
//  没有 ⇒ rat 必须退回 arch_map（判定期同拍新分配出去的那个号没有 ROB 项，留着就是个死引用）。
    output reg [31:0]  squash_survive_hit,
//★ 判定拍改向【接住】的那一拍不许分配：本拍的 kill 掩码是用【拍前】的 entry_valid 算的，
//  同拍新进来的项不在掩码里，会被当成幸存项、把尾指针算歪 ⇒ 指令变孤儿。
//  与 后端击杀 那一拍的口径一致，只是早了一拍。
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
    input        prf_wb_en_0, input [4:0] prf_wb_index_0, input prf_wb_generation_0,
    input        prf_wb_en_1, input [4:0] prf_wb_index_1, input prf_wb_generation_1,
    input [4:0]  prf_ld_index, input prf_ld_generation,
    input [4:0]  prf_alu_early_index_0, prf_alu_early_index_1,
    input        prf_alu_early_generation_0, prf_alu_early_generation_1,
    input [4:0]  prf_ld_early_index, input prf_ld_early_generation,
    input [4:0]  prf_div_wake_index_0, prf_div_wake_index_1,
    output [5:0] prf_ld_early_phys, output prf_ld_early_live,
    output [5:0] prf_alu_early_phys_0, prf_alu_early_phys_1,
    output       prf_alu_early_live_0, prf_alu_early_live_1,
    output [5:0] prf_div_wake_phys_0, prf_div_wake_phys_1,
    output [5:0] prf_wb_phys_0, prf_wb_phys_1, prf_ld_phys,
    output       prf_wb_live_0, prf_wb_live_1, prf_ld_live,
//状态
//★ 提交口改向：队头这一项是"判错过了"的分支 ⇒ 本拍改 PC、并把它之后的全杀掉。
//  队头即边界 ⇒ "所有在途项都比边界年轻"是定理，消费端不再需要年龄判据。
//★ "已判定、还没排到队头"的改向是否还在册。给 isq 挡【新的访存发射】：
//  判定已经知道错路了，但要等它排到队头才杀；这个窗口里放出去的访存是错路的，
//  而错路访存可能打到没有从设备的地址 → LSU 永远等应答（实测 CoreMark：错路 `lbu 1(x0)` 卡死整核）。
//  挡住"新发"即可；已经在 s2/s3 的那几条会在提交口改向那一拍被 kill_arc 杀干净。
// mret：与 bju 同一套（项内标签 + 队头改向）。标签随分配落地（allocation_mret_*），
//  落点由 pc 直接取 csr 的 mepc。本模块只出 [这一笔是 mret 改向]。
    output reg        kill_redirect_mret,
    output reg        redirect_any,
//★ S5：A 域事件（判定拍 / mret）—— 只改 PC + 清前端，不碰后端。
    output reg        redir_frontend,
//★ S5：B 域事件（边界走到提交口）—— 只清整表 + 后端，不碰前端。
    (* max_fanout = 32 *) output reg redir_backend_0,
    output reg        redir_backend_1,   // 上面那根打一拍：清 isq / idu 等后端
    output reg [31:0] kill_redirect_target,
    output reg [4:0]  head_pointer,
//本拍退休的两条（给 lsu：退休过的项不许再按年龄冲刷作废）
    output reg        ret0_v, ret1_v,
    output reg [4:0]  ret0_index, ret1_index,
    output reg        ret0_generation, ret1_generation,
    output reg        full,
//★ 击杀弧掩码：第 i 位 = "槽 i 比冲刷边界年轻"。寄存版，给 lsu/idu/isq 的作废判据用。
//  为什么它可以寄存而 flush_ok 不行：这是一段【绝对槽号的弧】—— (边界, 队尾]。队头前进不改变
//  哪些槽比边界年轻，所以晚一拍用同一个掩码是【精确等价】。而 flush_ok 判的是"边界还在不在
//  占用窗口里"，那个窗口是跟着队头走的，晚一拍就变。
//  为什么值得寄存：消费端原来是 `{1'b0,(idx-head_pointer)} > {1'b0,(flush_index-head_pointer)}`，
//  两次减法 + 一次比较，而且 head_pointer 要从本模块扇到 lsu/idu/isq/cont/iffu/alu 六个地方。
//  换成 `arc_q[idx]` 之后消费端只剩一次 16:1 查表，head_pointer 不再出门。
//★ "槽 i 还没绕过队头"（i >= head_pointer）的位向量，给 cont 的异常仲裁做年龄比较。
//  age(X) < age(Y) 不能化成 X < Y —— 年龄在 16 的环上回绕，跨过 0 时排序会翻；
//  有这一位就能拆成三段：同侧时排序等于比槽号，跨侧时恒有定论。
    output reg [31:0] slot_above_head,
// P2: head one-hot (registered). For "am I the head" qualification checks,
// instead of shipping the pointer out and letting every module compare.
// D side is only a 4->16 decode, so replicating it is cheap.
    output reg [31:0] head_one_hot_registered,
//---- 中断边界 / mepc：必须取【ROB 队头 = 最老未退休】----
//  原来这两项取自 isq 的队列头（= 最老【未发射】）。两者不等价：中断切割之后，比它老但已发射
//  未退休的项仍在 ROB 里照常退休，而它们的控制流改向被这次冲刷抹掉 ⇒ 落空路径被架构执行。
//  ★ 这里必须【组合】给出（不能寄存一拍）：队头同一拍可能刚退休，寄存版会把已退休那条当成
//    返回点，导致它被执行两次。
    input [31:0]  allocation_pc_0, allocation_pc_1, allocation_pc_2,
    output reg [31:0] head_pc_out,
    output reg        head_valid_out,
// P1: how many slots the head advances this cycle (0/1/2). Decided once here;
// minstret consumes this delta instead of subtracting pointers.
    output reg [1:0]  retire_step,
    output reg        empty,
    output reg [5:0]  occupancy,
    output reg        kill_trap,
//陷阱载荷：与 kill_trap 同拍，都是 dly 里的寄存器。
    output reg [3:0]  kill_trap_cause,
    output reg [31:0] kill_trap_pc,
    output reg [31:0] kill_trap_value
    );

//==============================================================================
//  参数与声明区：本模块全部 localparam/parameter + reg/wire。
//  Verilog 要求先声明后用，所以全放这里；正文只留逻辑。
//==============================================================================
parameter integer DEPTH = 32;   // ★宏化旋钮：32=大核 / 16=小核（必须是 2 的幂）
localparam [4:0] DEPTH_MASK = DEPTH - 1;
localparam integer ARCH_NUMBER = 32;   // RV32I 架构寄存器个数（rat 回滚按这个维度展开）

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，各模块各自打一拍，彼此没有相位差。
reg reset_registered;
//==============================================================================
//  信号声明（Verilog 要求先声明后使用，三区的寄存器统一放在这里，按区归类）
//==============================================================================
//分配区
reg [4:0]  tail_pointer;
reg [5:0]  entry_count;
reg [4:0]  tail_pointer_next;
reg [5:0]  entry_count_next;
reg        alloc_take_0, alloc_take_1, alloc_take_2;
//每槽的目的寄存器号（写 x0 与"不写 rd"一律存 0）：值读口不按 rd 比较，这里只给提交口与
//"发出即置可退"用；退项与冲刷不必清（判据里 entry_valid 与它相与）。异常载荷不逐槽存：见 exc_side_*。
//★ 2026-11-04：每槽的物理号（真 PRF）。只有【复位 + 分配】两个写源，与 rd 同条件同沿。
//每槽的世代位：每次分配翻转一次。完成上报必须世代一致才被接受 —— 这是"迟到上报
//打进新住户"的唯一根治手段（entry_valid 挡不住"新住户也 valid"）。
reg        entry_generation  [0:DEPTH-1];
reg        entry_jump  [0:DEPTH-1];
//★ 2026-11-04（CSR 步骤 C）：这一项是不是 CSR 指令，以及它的载荷（单独一组，理由见端口注释）。
//  ★ 载荷原来是单独一组阵列。曾经试过把它做成分布式 RAM（`ram_style="distributed"` + 踢出复位循环），
//    结果一位没省：本文件的 entry_* 全都是触发器，原因不是复位也不是 ram_style，而是**同一拍有两条写语句**
//    （lane0 写 tail_pointer、lane1 写 allocation_index_1）—— 7 系列 LUTRAM 原语只有 1 个写口，
//    综合器直接判 `8-4767 ... multiple writes via different ports` 后落回触发器。别再往这个方向试。
//CSR 载荷一深旁表：只有发射那一拍写，提交那一拍按槽号对上才认
reg [21:0] csr_side_payload;
reg [4:0]  csr_side_index;
reg        csr_side_valid;
//提交区
reg [4:0]  head_next_pointer;
// P4: cont only supplies the EXCEPTION boundary candidate. For a commit-port redirect
// the boundary is this module's own head, so the effective boundary is resolved here
// and head_pointer no longer has to leave the ROB just to be muxed into flush_index.
// 边界归属：改向（队头）与 trap 交付（队头）都在本模块内解析；
// cont 只送"改向/中断的边界候选"，异常那一档已经不需要边界了。
reg [4:0]  head_pointer_next;
reg [1:0]  retire_step_next;
reg        kill_any;
reg        head_ok;
reg        head_next_ok;
//==============================================================================
//★ 年龄 = 槽号 - 队头。指针只按 mod-DEPTH 回卷，在册窗口是连续的一段、跨度 ≤ DEPTH
//  ⇒ 这个差不回绕，就是名次，5 位减法够。全部年龄比较只有两处：判定口两条 lane 取更老、
//  候选与在册边界比一次。
//★ 不能用 entry_generation 当环绕位：gen 是"每个槽各自的分配奇偶"，冲刷回卷后同一位会在同一圈里
//  被分配两次，各槽奇偶错开（实测：队头 5、活项 5..12，entry_generation 却把 6..11 判成"比队头老"）。
//提交口的组合版（打拍前）：head_ok / entry_destination_register_rd_head_pointer / entry_data[head_pointer]
reg        commit_we0_comb, commit_we1_comb;
reg [4:0]  commit_rd0_comb, commit_rd1_comb;
reg [5:0]  commit_phys0_comb, commit_phys1_comb;
//CSR 提交口（打拍前）：队头这一项是 CSR 就吐载荷
reg        commit_csr_comb;
reg [21:0] commit_csr_payload_comb;
//★ 2026-11-05（S5）：改向拆成两个域，边界只在【在册】的这一份寄存器里，不按项挂标记。
//  A = 判定拍：清前端 + 改 PC + 冻分配；B = 提交口：清整张表和它后面的后端。
//  两张按项的阵列（"这条要改向" + 落点）都删了 —— 落点 A 那一拍就锁进 pending_target，
//  提交口只需要知道"哪一项是边界"，一次 5 位比较就够。
reg        flush_boundary_valid;
reg [4:0]  flush_boundary_index;
//★ 判定口的候选（组合）：两条 lane 各算一次年龄，取更老的；再和【在册边界】比一次。
reg        judge_valid, judge_accept, judge_pick1;
reg [4:0]  judge_index, judge_age;
reg [31:0] judge_target;
//项状态阵列（v / wr / ex / data / jdg）：唯一被三个区共写的对象，只能有一个 always 块
//（在本区末尾）：分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
reg        entry_valid  [0:DEPTH-1];
reg        entry_written [0:DEPTH-1];
reg        entry_exception [0:DEPTH-1];
//每槽的结果数据：与 entry_written 同条件同一沿写，值读口/提交口读；退项/冲刷/陷阱不必清。
//每槽的指令地址（= 该条指令地址 + 4，与 csr.mepec 的减 4 约定一致）。
//只服务"中断返回点"，所以只有队头那一项被读。
reg        entry_judged  [0:DEPTH-1];
//mret 标签：第二种【队头改向】，落点不由项携带（取 mepc）
reg        entry_mret [0:DEPTH-1];
//head 拍判定出来的事件（组合）。下一沿变成对外的重定向事件 —— 冲刷从这排触发器出去。
reg        mret_now, exc_now;
//B 事件的边界随事件一起打一拍。
//dly 里现在是否有事件（= 本拍正在冲刷）。它挡住队头写回和新的装载。
wire       kill_busy = redir_backend_0 | kill_trap;
//★ 2026-11-10：entry_target 按【写者】分阵列（它有 5 个任意槽的写者，没法按槽分银行）。
//  配一张 entry_target_source（每槽记「最后是哪个写者写的」），读口按它选。
//  写者优先级与原代码的先后一致：exception_selected > exc1 > exc0 > 判定lane1 > 判定lane0。
localparam [2:0] TARGET_FROM_JUMP_LANE0  = 3'd0;
localparam [2:0] TARGET_FROM_JUMP_LANE1  = 3'd1;
localparam [2:0] TARGET_FROM_EXC_LANE0   = 3'd2;
localparam [2:0] TARGET_FROM_EXC_LANE1   = 3'd3;
localparam [2:0] TARGET_FROM_EXC_SELECTED= 3'd4;
reg [31:0] entry_target_jump_lane0        [0:DEPTH-1];
reg [31:0] entry_target_jump_lane1        [0:DEPTH-1];
reg [31:0] entry_target_exception_lane0   [0:DEPTH-1];
reg [31:0] entry_target_exception_lane1   [0:DEPTH-1];
reg [31:0] entry_target_exception_selected[0:DEPTH-1];
reg [2:0]  entry_target_source            [0:DEPTH-1];
//★ S4：异常载荷【逐槽存】，不再是一份侧寄存器。
//  侧寄存器只能记一条 ⇒ 窗口里第二条故障会把第一条盖掉（实测 seed1：mepc 记成后一条的），
//  而"从标记到交付"之间本来就允许多条故障同时在册。
//  cause 用新增的 4 位阵列（32×4=128 FF）；pc / tval 直接复用【本槽已经闲置的两个 32 位域】：
//   · entry_data：故障项永远等不到完成口（它不会再有 acc_*），这个域对它恒空 ⇒ 存故障 PC；
//   · entry_target ：只有判定口会写它，而判定口只写 entry_jump 的项；访存类故障项 entry_jump=0 ⇒ 存 tval；
//               分支/跳转类故障（目标非对齐）entry_jump=1，但那一拍的 bju_tgt 就是那个坏目标 ⇒ 同值。
reg [3:0]  entry_exc [0:DEPTH-1];
//==============================================================================
//  幸存者表 rb_* —— isq 用它重建 pstat（冲刷后第 2 拍取用）
//==============================================================================
//★ 时相：本表是每拍都在的"当前在册项"，**不再是"冲刷那一拍的组合结果"**。
//  冲刷在 F 沿已把 entry_valid/entry_written/entry_exception 按年龄清干净（见下面 entry_valid 写口那一段），
//  所以 F 之后任何一拍读到的 entry_valid 就是幸存者本身 ⇒ 冲刷锥与这张表彻底脱钩。
//  取用时刻由 isq 的 reb_d2 决定（冲刷后第 2 拍），这里只管"报当前状态"。
//★ 原来那段 flush_ok / flush_age / 16 项年龄比较的 kill 判据【整段删除】：
//  它计算的事情时钟沿上已经做过了，留着只是把冲刷锥再拖一遍。
//★ 只保留【窗口上界】age < entry_count 一项：它是"在册"的判据，兼管两件事 ——
//  ① 陷阱交付（kill_trap）只回卷指针、不清 entry_valid；② 已退项在 head 之前，age 回绕成
//  大数，必须靠它挡掉（否则会把已退休的写者当成活写者登记回去）。
//==============================================================================
//  提交区（head 端）：退役判据、冲刷、陷阱、提交口（写寄存器堆）
//==============================================================================
//退口判据 / 冲刷判据 / 状态输出
//  可退 = 在册且已完成。exc 标记项不算可退：它不写寄存器堆，由 kill_trap 交上级；
//  同拍标记（exc_en 正落在 head/head+1 上）必须【组合】就压掉退，否则会在 entry_exception 落盘前退掉。
//★ S5：改向分两个域 —— A 在判定拍（清前端 + 改 PC + 冻分配），B 在提交口（清整表 + 后端）。
//  判定口给的东西（bju_judged / bju_index / bju_redir / bju_tgt）都是 exu 的 *_registered，
//  同拍同源 ⇒ 不需要再为边界打拍。
//★ 异常/中断优先：队头是故障项时，本拍不放误预测改向（异常那条在程序序上更老，
//  它的 mepc 必须取故障指令自己的 pc；被改向一掺就错）。分支的边界留在在册寄存器里，
//  异常交付那一拍整表清空、边界一起撤。
    wire head_is_exception = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_exception[head_pointer];
    wire [4:0] judge0_age = bju_index             - head_pointer;
    wire [4:0] judge1_age = branch_unit_lane1_index - head_pointer;
    reg        boundary_now;

always @(posedge clk) reset_registered <= rst;

integer ai;

//重建口：物理槽序，静态下标（零 mux）

integer ri;
integer rp;

//==============================================================================
//  分配区（tail 端）：这一拍进几条、进到哪个槽、槽里写什么
//==============================================================================
//槽号与世代是组合输出的环位，作为载荷沿流水线传下去。
//★ lane1 的落位取决于 lane0 这一拍分不分配（两条 lane 独立推进，可能只有 lane1 来）。

always @(*) begin
        allocation_index_0 = tail_pointer;
        allocation_index_1 = (tail_pointer + {4'b0000, allocation_en_0}) & DEPTH_MASK;
        allocation_index_2 = (tail_pointer + {4'b0000, allocation_en_0}
                                          + {4'b0000, allocation_en_1}) & DEPTH_MASK;
        allocation_generation_0 = ~entry_generation[tail_pointer];
        allocation_generation_1 = ~entry_generation[allocation_index_1];
        allocation_generation_2 = ~entry_generation[allocation_index_2];
        occupancy  = entry_count;
        empty      = (entry_count == 5'd0);
        full       = (entry_count >= DEPTH - 5'd1);
//★ 2026-11-09 三宽分配：逐口判"还放得下第 k 条"。判据 = occupancy + k < DEPTH，
//  与 idu 的分配门逐字同门 —— 两边差一条就是"发了号 ROB 没接 ⇒ 指令蒸发"。
        alloc_take_0 = allocation_en_0 & ~(redir_backend_0 | interrupt_kill | kill_trap) & ((entry_count + 5'd0) < DEPTH);
        alloc_take_1 = allocation_en_1 & ~(redir_backend_0 | interrupt_kill | kill_trap) & ((entry_count + 5'd1) < DEPTH);
        alloc_take_2 = allocation_en_2 & ~(redir_backend_0 | interrupt_kill | kill_trap) & ((entry_count + 5'd2) < DEPTH);
    end

//下一拍的指针与占用数：先算退掉的（0/1/2 笔），再算分配的，冲刷最后覆盖。
//★ 判据必须与提交区的项状态块【逐字一致】：边界项失效时走"全冲"，这里也要把 entry_count/tail
//  归零，否则"项全清、计数还留着" ⇒ head 指向无效项 ⇒ 队头永不动。
    always @(*) begin
        entry_count_next  = entry_count;
        tail_pointer_next = tail_pointer;
        if (head_ok) begin
            entry_count_next = entry_count_next - 5'd1;
        end
        if (head_next_ok) begin
            entry_count_next = entry_count_next - 5'd1;
        end
        if (alloc_take_0) begin
            tail_pointer_next = (tail_pointer_next + 5'd1) & DEPTH_MASK;
            entry_count_next  = entry_count_next + 5'd1;
        end
        if (alloc_take_1) begin
            tail_pointer_next = (tail_pointer_next + 5'd1) & DEPTH_MASK;
            entry_count_next  = entry_count_next + 5'd1;
        end
        if (alloc_take_2) begin
            tail_pointer_next = (tail_pointer_next + 5'd1) & DEPTH_MASK;
            entry_count_next  = entry_count_next + 5'd1;
        end
        if ((redir_backend_0 | interrupt_kill | kill_trap)) begin
//★ 2026-11-05：改向 / 异常 / 中断都在提交口发，前端和整张 ROB 同一拍一起作废 ⇒ 恒为整窗。
            entry_count_next  = 5'd0;
            tail_pointer_next = head_pointer;
        end
//陷阱交付：队头那条故障指令【不写寄存器堆】地退掉，比它年轻的项全部作废。
//★ 必须放在【冲刷赋值之后】：交付那一拍冲刷口可能同时有效，不能被 flush_keep 盖掉。
        if (kill_trap) begin
            tail_pointer_next = (head_pointer + 5'd1) & DEPTH_MASK;
            entry_count_next  = 5'd0;
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            tail_pointer <= 5'd0;
            entry_count  <= 5'd0;
        end
        else begin
            tail_pointer <= tail_pointer_next;
            entry_count  <= entry_count_next;
        end
    end

//=============================================================================
//  入项六张表：按槽号[1:0] 分成 4 个银行
//
//  为什么要分：分布式 RAM 每块只有一个写口，而 3 宽分配每拍有三条写语句
//  （Vivado 直接放弃推断，报 8-4767），阵列只能落触发器 —— ROB 一直是 cpu_top 里最大的一块。
//  分配的三槽恒连续（tail / +1 / +2）=> 三个槽 mod 4 必然互异
//  => 每个银行每拍至多一条写语句。写时刻与读写语义和不分银行时逐字相同。
//
//  能这么分的只有「分配时写、之后不再改」的六张。entry_valid / entry_written /
//  entry_exception / entry_judged / entry_mret / entry_generation 是按位清置的位向量，
//  语义本身就是位操作，仍旧留在触发器里。
//
//  不复位：槽内容在入项之前没有意义，读侧由 entry_valid 挡着。
//  读口写成「一位热码 + 三目或」而不是三目链：Verilator 5.034 对四级以上的
//  「? mem[i]」链会误报语法错。
//=============================================================================
localparam integer DEPTH_AW = (DEPTH > 16) ? 5 : 4;   // 槽号位宽
localparam integer ROW_AW   = DEPTH_AW - 2;           // 行内地址位宽

//---- 分配口三份数据（rd=x0 或本来就不写回的，目的号与物理号都记 0）----
    wire [31:0] alloc_pc_0       = allocation_pc_0;
    wire        alloc_we_valid_0 = allocation_we_0 && (allocation_rd_0 != 5'd0);
    wire [5:0]  alloc_phys_0     = alloc_we_valid_0 ? allocation_phys_0 : 6'd0;
    wire [4:0]  alloc_dest_0     = alloc_we_valid_0 ? allocation_rd_0   : 5'd0;
    wire        alloc_sys_0      = allocation_ecall_0 | allocation_ebreak_0 | allocation_illegal_0 | allocation_mret_0;

    wire [31:0] alloc_pc_1       = allocation_pc_1;
    wire        alloc_we_valid_1 = allocation_we_1 && (allocation_rd_1 != 5'd0);
    wire [5:0]  alloc_phys_1     = alloc_we_valid_1 ? allocation_phys_1 : 6'd0;
    wire [4:0]  alloc_dest_1     = alloc_we_valid_1 ? allocation_rd_1   : 5'd0;
    wire        alloc_sys_1      = allocation_ecall_1 | allocation_ebreak_1 | allocation_illegal_1 | allocation_mret_1;

    wire [31:0] alloc_pc_2       = allocation_pc_2;
    wire        alloc_we_valid_2 = allocation_we_2 && (allocation_rd_2 != 5'd0);
    wire [5:0]  alloc_phys_2     = alloc_we_valid_2 ? allocation_phys_2 : 6'd0;
    wire [4:0]  alloc_dest_2     = alloc_we_valid_2 ? allocation_rd_2   : 5'd0;
    wire        alloc_sys_2      = allocation_ecall_2 | allocation_ebreak_2 | allocation_illegal_2 | allocation_mret_2;
//---- entry_target 的读选择：源表 -> 一位热码 -> 掩码或 ----
    wire [2:0]  entry_target_source_of_head = entry_target_source[head_pointer];
    wire [4:0]  entry_target_pick           = 5'd1 << entry_target_source_of_head;
    wire [31:0] entry_target_of_head =
        ({32{entry_target_pick[0]}} & entry_target_jump_lane0        [head_pointer]) |
        ({32{entry_target_pick[1]}} & entry_target_jump_lane1        [head_pointer]) |
        ({32{entry_target_pick[2]}} & entry_target_exception_lane0   [head_pointer]) |
        ({32{entry_target_pick[3]}} & entry_target_exception_lane1   [head_pointer]) |
        ({32{entry_target_pick[4]}} & entry_target_exception_selected[head_pointer]);

    wire [DEPTH_AW-1:0] alloc_slot_0 = tail_pointer;
    wire [DEPTH_AW-1:0] alloc_slot_1 = allocation_index_1;
    wire [DEPTH_AW-1:0] alloc_slot_2 = allocation_index_2;

    reg [31:0] entry_pc_bank0 [0:DEPTH/4-1];
    reg [31:0] entry_pc_bank1 [0:DEPTH/4-1];
    reg [31:0] entry_pc_bank2 [0:DEPTH/4-1];
    reg [31:0] entry_pc_bank3 [0:DEPTH/4-1];
    reg [5:0] entry_phys_bank0 [0:DEPTH/4-1];
    reg [5:0] entry_phys_bank1 [0:DEPTH/4-1];
    reg [5:0] entry_phys_bank2 [0:DEPTH/4-1];
    reg [5:0] entry_phys_bank3 [0:DEPTH/4-1];
    reg [4:0] entry_destination_register_bank0 [0:DEPTH/4-1];
    reg [4:0] entry_destination_register_bank1 [0:DEPTH/4-1];
    reg [4:0] entry_destination_register_bank2 [0:DEPTH/4-1];
    reg [4:0] entry_destination_register_bank3 [0:DEPTH/4-1];
    reg  entry_csr_bank0 [0:DEPTH/4-1];
    reg  entry_csr_bank1 [0:DEPTH/4-1];
    reg  entry_csr_bank2 [0:DEPTH/4-1];
    reg  entry_csr_bank3 [0:DEPTH/4-1];
    reg  entry_pending_bank0 [0:DEPTH/4-1];
    reg  entry_pending_bank1 [0:DEPTH/4-1];
    reg  entry_pending_bank2 [0:DEPTH/4-1];
    reg  entry_pending_bank3 [0:DEPTH/4-1];
    reg  entry_system_bank0 [0:DEPTH/4-1];
    reg  entry_system_bank1 [0:DEPTH/4-1];
    reg  entry_system_bank2 [0:DEPTH/4-1];
    reg  entry_system_bank3 [0:DEPTH/4-1];

//---- 银行 0 ----
    wire alloc_hit_bank0_port0 = alloc_take_0 && (alloc_slot_0[1:0] == 2'd0);
    wire alloc_hit_bank0_port1 = alloc_take_1 && (alloc_slot_1[1:0] == 2'd0);
    wire alloc_hit_bank0_port2 = alloc_take_2 && (alloc_slot_2[1:0] == 2'd0);
    wire [ROW_AW-1:0] alloc_row_bank0 = alloc_hit_bank0_port2 ? alloc_slot_2[DEPTH_AW-1:2] :
                                        alloc_hit_bank0_port1 ? alloc_slot_1[DEPTH_AW-1:2] :
                                                                  alloc_slot_0[DEPTH_AW-1:2];
    wire [31:0] alloc_pc_bank0   = alloc_hit_bank0_port2 ? alloc_pc_2   :
                                    alloc_hit_bank0_port1 ? alloc_pc_1 : alloc_pc_0;
    wire [5:0]  alloc_phys_bank0 = alloc_hit_bank0_port2 ? alloc_phys_2 :
                                    alloc_hit_bank0_port1 ? alloc_phys_1 : alloc_phys_0;
    wire [4:0]  alloc_dest_bank0 = alloc_hit_bank0_port2 ? alloc_dest_2 :
                                    alloc_hit_bank0_port1 ? alloc_dest_1 : alloc_dest_0;
    wire        alloc_csr_bank0  = alloc_hit_bank0_port2 ? allocation_csr_2 :
                                    alloc_hit_bank0_port1 ? allocation_csr_1 : allocation_csr_0;
    wire        alloc_pend_bank0 = alloc_hit_bank0_port2 ? allocation_pend_2 :
                                    alloc_hit_bank0_port1 ? allocation_pend_1 : allocation_pend_0;
    wire        alloc_sys_bank0  = alloc_hit_bank0_port2 ? alloc_sys_2 :
                                    alloc_hit_bank0_port1 ? alloc_sys_1 : alloc_sys_0;

    always @(posedge clk) begin
        if (alloc_hit_bank0_port0 | alloc_hit_bank0_port1 | alloc_hit_bank0_port2) begin
            entry_pc_bank0                  [alloc_row_bank0] <= alloc_pc_bank0;
            entry_phys_bank0                [alloc_row_bank0] <= alloc_phys_bank0;
            entry_destination_register_bank0[alloc_row_bank0] <= alloc_dest_bank0;
            entry_csr_bank0                 [alloc_row_bank0] <= alloc_csr_bank0;
            entry_pending_bank0             [alloc_row_bank0] <= alloc_pend_bank0;
            entry_system_bank0              [alloc_row_bank0] <= alloc_sys_bank0;
        end
    end

//---- 银行 1 ----
    wire alloc_hit_bank1_port0 = alloc_take_0 && (alloc_slot_0[1:0] == 2'd1);
    wire alloc_hit_bank1_port1 = alloc_take_1 && (alloc_slot_1[1:0] == 2'd1);
    wire alloc_hit_bank1_port2 = alloc_take_2 && (alloc_slot_2[1:0] == 2'd1);
    wire [ROW_AW-1:0] alloc_row_bank1 = alloc_hit_bank1_port2 ? alloc_slot_2[DEPTH_AW-1:2] :
                                        alloc_hit_bank1_port1 ? alloc_slot_1[DEPTH_AW-1:2] :
                                                                  alloc_slot_0[DEPTH_AW-1:2];
    wire [31:0] alloc_pc_bank1   = alloc_hit_bank1_port2 ? alloc_pc_2   :
                                    alloc_hit_bank1_port1 ? alloc_pc_1 : alloc_pc_0;
    wire [5:0]  alloc_phys_bank1 = alloc_hit_bank1_port2 ? alloc_phys_2 :
                                    alloc_hit_bank1_port1 ? alloc_phys_1 : alloc_phys_0;
    wire [4:0]  alloc_dest_bank1 = alloc_hit_bank1_port2 ? alloc_dest_2 :
                                    alloc_hit_bank1_port1 ? alloc_dest_1 : alloc_dest_0;
    wire        alloc_csr_bank1  = alloc_hit_bank1_port2 ? allocation_csr_2 :
                                    alloc_hit_bank1_port1 ? allocation_csr_1 : allocation_csr_0;
    wire        alloc_pend_bank1 = alloc_hit_bank1_port2 ? allocation_pend_2 :
                                    alloc_hit_bank1_port1 ? allocation_pend_1 : allocation_pend_0;
    wire        alloc_sys_bank1  = alloc_hit_bank1_port2 ? alloc_sys_2 :
                                    alloc_hit_bank1_port1 ? alloc_sys_1 : alloc_sys_0;

    always @(posedge clk) begin
        if (alloc_hit_bank1_port0 | alloc_hit_bank1_port1 | alloc_hit_bank1_port2) begin
            entry_pc_bank1                  [alloc_row_bank1] <= alloc_pc_bank1;
            entry_phys_bank1                [alloc_row_bank1] <= alloc_phys_bank1;
            entry_destination_register_bank1[alloc_row_bank1] <= alloc_dest_bank1;
            entry_csr_bank1                 [alloc_row_bank1] <= alloc_csr_bank1;
            entry_pending_bank1             [alloc_row_bank1] <= alloc_pend_bank1;
            entry_system_bank1              [alloc_row_bank1] <= alloc_sys_bank1;
        end
    end

//---- 银行 2 ----
    wire alloc_hit_bank2_port0 = alloc_take_0 && (alloc_slot_0[1:0] == 2'd2);
    wire alloc_hit_bank2_port1 = alloc_take_1 && (alloc_slot_1[1:0] == 2'd2);
    wire alloc_hit_bank2_port2 = alloc_take_2 && (alloc_slot_2[1:0] == 2'd2);
    wire [ROW_AW-1:0] alloc_row_bank2 = alloc_hit_bank2_port2 ? alloc_slot_2[DEPTH_AW-1:2] :
                                        alloc_hit_bank2_port1 ? alloc_slot_1[DEPTH_AW-1:2] :
                                                                  alloc_slot_0[DEPTH_AW-1:2];
    wire [31:0] alloc_pc_bank2   = alloc_hit_bank2_port2 ? alloc_pc_2   :
                                    alloc_hit_bank2_port1 ? alloc_pc_1 : alloc_pc_0;
    wire [5:0]  alloc_phys_bank2 = alloc_hit_bank2_port2 ? alloc_phys_2 :
                                    alloc_hit_bank2_port1 ? alloc_phys_1 : alloc_phys_0;
    wire [4:0]  alloc_dest_bank2 = alloc_hit_bank2_port2 ? alloc_dest_2 :
                                    alloc_hit_bank2_port1 ? alloc_dest_1 : alloc_dest_0;
    wire        alloc_csr_bank2  = alloc_hit_bank2_port2 ? allocation_csr_2 :
                                    alloc_hit_bank2_port1 ? allocation_csr_1 : allocation_csr_0;
    wire        alloc_pend_bank2 = alloc_hit_bank2_port2 ? allocation_pend_2 :
                                    alloc_hit_bank2_port1 ? allocation_pend_1 : allocation_pend_0;
    wire        alloc_sys_bank2  = alloc_hit_bank2_port2 ? alloc_sys_2 :
                                    alloc_hit_bank2_port1 ? alloc_sys_1 : alloc_sys_0;

    always @(posedge clk) begin
        if (alloc_hit_bank2_port0 | alloc_hit_bank2_port1 | alloc_hit_bank2_port2) begin
            entry_pc_bank2                  [alloc_row_bank2] <= alloc_pc_bank2;
            entry_phys_bank2                [alloc_row_bank2] <= alloc_phys_bank2;
            entry_destination_register_bank2[alloc_row_bank2] <= alloc_dest_bank2;
            entry_csr_bank2                 [alloc_row_bank2] <= alloc_csr_bank2;
            entry_pending_bank2             [alloc_row_bank2] <= alloc_pend_bank2;
            entry_system_bank2              [alloc_row_bank2] <= alloc_sys_bank2;
        end
    end

//---- 银行 3 ----
    wire alloc_hit_bank3_port0 = alloc_take_0 && (alloc_slot_0[1:0] == 2'd3);
    wire alloc_hit_bank3_port1 = alloc_take_1 && (alloc_slot_1[1:0] == 2'd3);
    wire alloc_hit_bank3_port2 = alloc_take_2 && (alloc_slot_2[1:0] == 2'd3);
    wire [ROW_AW-1:0] alloc_row_bank3 = alloc_hit_bank3_port2 ? alloc_slot_2[DEPTH_AW-1:2] :
                                        alloc_hit_bank3_port1 ? alloc_slot_1[DEPTH_AW-1:2] :
                                                                  alloc_slot_0[DEPTH_AW-1:2];
    wire [31:0] alloc_pc_bank3   = alloc_hit_bank3_port2 ? alloc_pc_2   :
                                    alloc_hit_bank3_port1 ? alloc_pc_1 : alloc_pc_0;
    wire [5:0]  alloc_phys_bank3 = alloc_hit_bank3_port2 ? alloc_phys_2 :
                                    alloc_hit_bank3_port1 ? alloc_phys_1 : alloc_phys_0;
    wire [4:0]  alloc_dest_bank3 = alloc_hit_bank3_port2 ? alloc_dest_2 :
                                    alloc_hit_bank3_port1 ? alloc_dest_1 : alloc_dest_0;
    wire        alloc_csr_bank3  = alloc_hit_bank3_port2 ? allocation_csr_2 :
                                    alloc_hit_bank3_port1 ? allocation_csr_1 : allocation_csr_0;
    wire        alloc_pend_bank3 = alloc_hit_bank3_port2 ? allocation_pend_2 :
                                    alloc_hit_bank3_port1 ? allocation_pend_1 : allocation_pend_0;
    wire        alloc_sys_bank3  = alloc_hit_bank3_port2 ? alloc_sys_2 :
                                    alloc_hit_bank3_port1 ? alloc_sys_1 : alloc_sys_0;

    always @(posedge clk) begin
        if (alloc_hit_bank3_port0 | alloc_hit_bank3_port1 | alloc_hit_bank3_port2) begin
            entry_pc_bank3                  [alloc_row_bank3] <= alloc_pc_bank3;
            entry_phys_bank3                [alloc_row_bank3] <= alloc_phys_bank3;
            entry_destination_register_bank3[alloc_row_bank3] <= alloc_dest_bank3;
            entry_csr_bank3                 [alloc_row_bank3] <= alloc_csr_bank3;
            entry_pending_bank3             [alloc_row_bank3] <= alloc_pend_bank3;
            entry_system_bank3              [alloc_row_bank3] <= alloc_sys_bank3;
        end
    end

//---- 读口：一位热码 + 三目或 ----
    wire [ROW_AW-1:0] row_of_head_next_pointer  = head_next_pointer[DEPTH_AW-1:2];
    wire [3:0]        bank_of_head_next_pointer = 4'd1 << head_next_pointer[1:0];
    wire [ROW_AW-1:0] row_of_head_pointer  = head_pointer[DEPTH_AW-1:2];
    wire [3:0]        bank_of_head_pointer = 4'd1 << head_pointer[1:0];
    wire [ROW_AW-1:0] row_of_issue_index_0  = issue_index_0[DEPTH_AW-1:2];
    wire [3:0]        bank_of_issue_index_0 = 4'd1 << issue_index_0[1:0];
    wire [ROW_AW-1:0] row_of_issue_index_1  = issue_index_1[DEPTH_AW-1:2];
    wire [3:0]        bank_of_issue_index_1 = 4'd1 << issue_index_1[1:0];
    wire [ROW_AW-1:0] row_of_prf_alu_early_index_0  = prf_alu_early_index_0[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_alu_early_index_0 = 4'd1 << prf_alu_early_index_0[1:0];
    wire [ROW_AW-1:0] row_of_prf_alu_early_index_1  = prf_alu_early_index_1[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_alu_early_index_1 = 4'd1 << prf_alu_early_index_1[1:0];
    wire [ROW_AW-1:0] row_of_prf_div_wake_index_0  = prf_div_wake_index_0[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_div_wake_index_0 = 4'd1 << prf_div_wake_index_0[1:0];
    wire [ROW_AW-1:0] row_of_prf_div_wake_index_1  = prf_div_wake_index_1[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_div_wake_index_1 = 4'd1 << prf_div_wake_index_1[1:0];
    wire [ROW_AW-1:0] row_of_prf_ld_early_index  = prf_ld_early_index[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_ld_early_index = 4'd1 << prf_ld_early_index[1:0];
    wire [ROW_AW-1:0] row_of_prf_ld_index  = prf_ld_index[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_ld_index = 4'd1 << prf_ld_index[1:0];
    wire [ROW_AW-1:0] row_of_prf_wb_index_0  = prf_wb_index_0[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_wb_index_0 = 4'd1 << prf_wb_index_0[1:0];
    wire [ROW_AW-1:0] row_of_prf_wb_index_1  = prf_wb_index_1[DEPTH_AW-1:2];
    wire [3:0]        bank_of_prf_wb_index_1 = 4'd1 << prf_wb_index_1[1:0];

    wire [31:0] entry_pc_rd_head_pointer =
        (bank_of_head_pointer[0] ? entry_pc_bank0[row_of_head_pointer] : 32'd0) |
        (bank_of_head_pointer[1] ? entry_pc_bank1[row_of_head_pointer] : 32'd0) |
        (bank_of_head_pointer[2] ? entry_pc_bank2[row_of_head_pointer] : 32'd0) |
        (bank_of_head_pointer[3] ? entry_pc_bank3[row_of_head_pointer] : 32'd0);

    wire [5:0] entry_phys_rd_head_next_pointer =
        (bank_of_head_next_pointer[0] ? entry_phys_bank0[row_of_head_next_pointer] : 6'd0) |
        (bank_of_head_next_pointer[1] ? entry_phys_bank1[row_of_head_next_pointer] : 6'd0) |
        (bank_of_head_next_pointer[2] ? entry_phys_bank2[row_of_head_next_pointer] : 6'd0) |
        (bank_of_head_next_pointer[3] ? entry_phys_bank3[row_of_head_next_pointer] : 6'd0);
    wire [5:0] entry_phys_rd_head_pointer =
        (bank_of_head_pointer[0] ? entry_phys_bank0[row_of_head_pointer] : 6'd0) |
        (bank_of_head_pointer[1] ? entry_phys_bank1[row_of_head_pointer] : 6'd0) |
        (bank_of_head_pointer[2] ? entry_phys_bank2[row_of_head_pointer] : 6'd0) |
        (bank_of_head_pointer[3] ? entry_phys_bank3[row_of_head_pointer] : 6'd0);
    wire [5:0] entry_phys_rd_prf_alu_early_index_0 =
        (bank_of_prf_alu_early_index_0[0] ? entry_phys_bank0[row_of_prf_alu_early_index_0] : 6'd0) |
        (bank_of_prf_alu_early_index_0[1] ? entry_phys_bank1[row_of_prf_alu_early_index_0] : 6'd0) |
        (bank_of_prf_alu_early_index_0[2] ? entry_phys_bank2[row_of_prf_alu_early_index_0] : 6'd0) |
        (bank_of_prf_alu_early_index_0[3] ? entry_phys_bank3[row_of_prf_alu_early_index_0] : 6'd0);
    wire [5:0] entry_phys_rd_prf_alu_early_index_1 =
        (bank_of_prf_alu_early_index_1[0] ? entry_phys_bank0[row_of_prf_alu_early_index_1] : 6'd0) |
        (bank_of_prf_alu_early_index_1[1] ? entry_phys_bank1[row_of_prf_alu_early_index_1] : 6'd0) |
        (bank_of_prf_alu_early_index_1[2] ? entry_phys_bank2[row_of_prf_alu_early_index_1] : 6'd0) |
        (bank_of_prf_alu_early_index_1[3] ? entry_phys_bank3[row_of_prf_alu_early_index_1] : 6'd0);
    wire [5:0] entry_phys_rd_prf_div_wake_index_0 =
        (bank_of_prf_div_wake_index_0[0] ? entry_phys_bank0[row_of_prf_div_wake_index_0] : 6'd0) |
        (bank_of_prf_div_wake_index_0[1] ? entry_phys_bank1[row_of_prf_div_wake_index_0] : 6'd0) |
        (bank_of_prf_div_wake_index_0[2] ? entry_phys_bank2[row_of_prf_div_wake_index_0] : 6'd0) |
        (bank_of_prf_div_wake_index_0[3] ? entry_phys_bank3[row_of_prf_div_wake_index_0] : 6'd0);
    wire [5:0] entry_phys_rd_prf_div_wake_index_1 =
        (bank_of_prf_div_wake_index_1[0] ? entry_phys_bank0[row_of_prf_div_wake_index_1] : 6'd0) |
        (bank_of_prf_div_wake_index_1[1] ? entry_phys_bank1[row_of_prf_div_wake_index_1] : 6'd0) |
        (bank_of_prf_div_wake_index_1[2] ? entry_phys_bank2[row_of_prf_div_wake_index_1] : 6'd0) |
        (bank_of_prf_div_wake_index_1[3] ? entry_phys_bank3[row_of_prf_div_wake_index_1] : 6'd0);
    wire [5:0] entry_phys_rd_prf_ld_early_index =
        (bank_of_prf_ld_early_index[0] ? entry_phys_bank0[row_of_prf_ld_early_index] : 6'd0) |
        (bank_of_prf_ld_early_index[1] ? entry_phys_bank1[row_of_prf_ld_early_index] : 6'd0) |
        (bank_of_prf_ld_early_index[2] ? entry_phys_bank2[row_of_prf_ld_early_index] : 6'd0) |
        (bank_of_prf_ld_early_index[3] ? entry_phys_bank3[row_of_prf_ld_early_index] : 6'd0);
    wire [5:0] entry_phys_rd_prf_ld_index =
        (bank_of_prf_ld_index[0] ? entry_phys_bank0[row_of_prf_ld_index] : 6'd0) |
        (bank_of_prf_ld_index[1] ? entry_phys_bank1[row_of_prf_ld_index] : 6'd0) |
        (bank_of_prf_ld_index[2] ? entry_phys_bank2[row_of_prf_ld_index] : 6'd0) |
        (bank_of_prf_ld_index[3] ? entry_phys_bank3[row_of_prf_ld_index] : 6'd0);
    wire [5:0] entry_phys_rd_prf_wb_index_0 =
        (bank_of_prf_wb_index_0[0] ? entry_phys_bank0[row_of_prf_wb_index_0] : 6'd0) |
        (bank_of_prf_wb_index_0[1] ? entry_phys_bank1[row_of_prf_wb_index_0] : 6'd0) |
        (bank_of_prf_wb_index_0[2] ? entry_phys_bank2[row_of_prf_wb_index_0] : 6'd0) |
        (bank_of_prf_wb_index_0[3] ? entry_phys_bank3[row_of_prf_wb_index_0] : 6'd0);
    wire [5:0] entry_phys_rd_prf_wb_index_1 =
        (bank_of_prf_wb_index_1[0] ? entry_phys_bank0[row_of_prf_wb_index_1] : 6'd0) |
        (bank_of_prf_wb_index_1[1] ? entry_phys_bank1[row_of_prf_wb_index_1] : 6'd0) |
        (bank_of_prf_wb_index_1[2] ? entry_phys_bank2[row_of_prf_wb_index_1] : 6'd0) |
        (bank_of_prf_wb_index_1[3] ? entry_phys_bank3[row_of_prf_wb_index_1] : 6'd0);

    wire [4:0] entry_destination_register_rd_head_next_pointer =
        (bank_of_head_next_pointer[0] ? entry_destination_register_bank0[row_of_head_next_pointer] : 5'd0) |
        (bank_of_head_next_pointer[1] ? entry_destination_register_bank1[row_of_head_next_pointer] : 5'd0) |
        (bank_of_head_next_pointer[2] ? entry_destination_register_bank2[row_of_head_next_pointer] : 5'd0) |
        (bank_of_head_next_pointer[3] ? entry_destination_register_bank3[row_of_head_next_pointer] : 5'd0);
    wire [4:0] entry_destination_register_rd_head_pointer =
        (bank_of_head_pointer[0] ? entry_destination_register_bank0[row_of_head_pointer] : 5'd0) |
        (bank_of_head_pointer[1] ? entry_destination_register_bank1[row_of_head_pointer] : 5'd0) |
        (bank_of_head_pointer[2] ? entry_destination_register_bank2[row_of_head_pointer] : 5'd0) |
        (bank_of_head_pointer[3] ? entry_destination_register_bank3[row_of_head_pointer] : 5'd0);
    wire [4:0] entry_destination_register_rd_issue_index_0 =
        (bank_of_issue_index_0[0] ? entry_destination_register_bank0[row_of_issue_index_0] : 5'd0) |
        (bank_of_issue_index_0[1] ? entry_destination_register_bank1[row_of_issue_index_0] : 5'd0) |
        (bank_of_issue_index_0[2] ? entry_destination_register_bank2[row_of_issue_index_0] : 5'd0) |
        (bank_of_issue_index_0[3] ? entry_destination_register_bank3[row_of_issue_index_0] : 5'd0);
    wire [4:0] entry_destination_register_rd_issue_index_1 =
        (bank_of_issue_index_1[0] ? entry_destination_register_bank0[row_of_issue_index_1] : 5'd0) |
        (bank_of_issue_index_1[1] ? entry_destination_register_bank1[row_of_issue_index_1] : 5'd0) |
        (bank_of_issue_index_1[2] ? entry_destination_register_bank2[row_of_issue_index_1] : 5'd0) |
        (bank_of_issue_index_1[3] ? entry_destination_register_bank3[row_of_issue_index_1] : 5'd0);

    wire  entry_csr_rd_head_pointer =
        (bank_of_head_pointer[0] ? entry_csr_bank0[row_of_head_pointer] : 1'd0) |
        (bank_of_head_pointer[1] ? entry_csr_bank1[row_of_head_pointer] : 1'd0) |
        (bank_of_head_pointer[2] ? entry_csr_bank2[row_of_head_pointer] : 1'd0) |
        (bank_of_head_pointer[3] ? entry_csr_bank3[row_of_head_pointer] : 1'd0);
    wire  entry_csr_rd_issue_index_0 =
        (bank_of_issue_index_0[0] ? entry_csr_bank0[row_of_issue_index_0] : 1'd0) |
        (bank_of_issue_index_0[1] ? entry_csr_bank1[row_of_issue_index_0] : 1'd0) |
        (bank_of_issue_index_0[2] ? entry_csr_bank2[row_of_issue_index_0] : 1'd0) |
        (bank_of_issue_index_0[3] ? entry_csr_bank3[row_of_issue_index_0] : 1'd0);
    wire  entry_csr_rd_issue_index_1 =
        (bank_of_issue_index_1[0] ? entry_csr_bank0[row_of_issue_index_1] : 1'd0) |
        (bank_of_issue_index_1[1] ? entry_csr_bank1[row_of_issue_index_1] : 1'd0) |
        (bank_of_issue_index_1[2] ? entry_csr_bank2[row_of_issue_index_1] : 1'd0) |
        (bank_of_issue_index_1[3] ? entry_csr_bank3[row_of_issue_index_1] : 1'd0);

    wire  entry_pending_rd_issue_index_0 =
        (bank_of_issue_index_0[0] ? entry_pending_bank0[row_of_issue_index_0] : 1'd0) |
        (bank_of_issue_index_0[1] ? entry_pending_bank1[row_of_issue_index_0] : 1'd0) |
        (bank_of_issue_index_0[2] ? entry_pending_bank2[row_of_issue_index_0] : 1'd0) |
        (bank_of_issue_index_0[3] ? entry_pending_bank3[row_of_issue_index_0] : 1'd0);
    wire  entry_pending_rd_issue_index_1 =
        (bank_of_issue_index_1[0] ? entry_pending_bank0[row_of_issue_index_1] : 1'd0) |
        (bank_of_issue_index_1[1] ? entry_pending_bank1[row_of_issue_index_1] : 1'd0) |
        (bank_of_issue_index_1[2] ? entry_pending_bank2[row_of_issue_index_1] : 1'd0) |
        (bank_of_issue_index_1[3] ? entry_pending_bank3[row_of_issue_index_1] : 1'd0);

    wire  entry_system_rd_head_next_pointer =
        (bank_of_head_next_pointer[0] ? entry_system_bank0[row_of_head_next_pointer] : 1'd0) |
        (bank_of_head_next_pointer[1] ? entry_system_bank1[row_of_head_next_pointer] : 1'd0) |
        (bank_of_head_next_pointer[2] ? entry_system_bank2[row_of_head_next_pointer] : 1'd0) |
        (bank_of_head_next_pointer[3] ? entry_system_bank3[row_of_head_next_pointer] : 1'd0);
    wire  entry_system_rd_head_pointer =
        (bank_of_head_pointer[0] ? entry_system_bank0[row_of_head_pointer] : 1'd0) |
        (bank_of_head_pointer[1] ? entry_system_bank1[row_of_head_pointer] : 1'd0) |
        (bank_of_head_pointer[2] ? entry_system_bank2[row_of_head_pointer] : 1'd0) |
        (bank_of_head_pointer[3] ? entry_system_bank3[row_of_head_pointer] : 1'd0);
//入项：rd / 世代 / 是否会跳 / 会不会报完成口 —— 这四张表只有【复位 + 分配】两个写源。
    always @(posedge clk) begin
        if (reset_registered) begin
            for (ai = 0; ai < DEPTH; ai = ai + 1) begin
                entry_generation[ai]  <= 1'b0;
                entry_jump[ai]  <= 1'b0;
            end
        end
        else begin
            if (alloc_take_0) begin
//前送表的目的寄存器号（保守侧：SYSTEM 也算写，带上 rd != 0 后只剩非法编码会"报了写其实不写"）。
                entry_generation[tail_pointer]  <= ~entry_generation[tail_pointer];
                entry_jump[tail_pointer]  <= allocation_jmp_0;
            end
            if (alloc_take_1) begin
                entry_generation[allocation_index_1]  <= ~entry_generation[allocation_index_1];
                entry_jump[allocation_index_1]  <= allocation_jmp_1;
            end
            if (alloc_take_2) begin
                entry_generation[allocation_index_2]  <= ~entry_generation[allocation_index_2];
                entry_jump[allocation_index_2]  <= allocation_jmp_2;
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
        head_next_pointer   = (head_pointer + 5'd1) & DEPTH_MASK;
//★ 冲刷那一拍一律不退：退是组合的，而冲刷的清位要到边沿才生效 ⇒ 错路那条若正好在 head 上
//  且已 done，会在冲刷拍就写进寄存器堆。
//★ 两拍都要挡住提交：当拍是冲刷正在发生，下一拍是 ROB 还没清完。
//★ S3 提交口改向：队头这一项是"判错过了"的分支 ⇒ 本拍改 PC + 杀掉它之后的全部。
//  队头即边界 ⇒ "在途项都比边界年轻"成立，消费端不必再做年龄比较。
//  ★ 判定在 head 拍、事件在下一拍：本拍只允许"改向那条自己退休"，事件由下一拍的触发器出。
//    冲刷那一拍（事件拍）一律不退 —— kill_any = 后端击杀。
//★ mret 与 bju 挤在同一个出口：两者都是"队头这一项自己要走、且要走的路不是顺序下一条"。
//  落点不同 —— bju 的落点在项里（entry_target，判定拍就算完）；mret 的落点是 csr 的 mepc 锁存器，
//  本模块不碰：它只报"这一笔改向是 mret"，pc 在事件那一拍直接取 mepc。
//  ★ 相位刚好够：事件在队头+1 出，而 `csrw mepc` 的写也在那一沿落盘（提交口影子晚一拍）⇒ pc 采到的是新值。
        // ★ 判定直接读队头那一项（阵列）。写回在队头这一拍完成，冲刷事件只进 dly、下一拍才出。
        // ★ dly 非空时既不出新事件，也不许退休 —— 这一拍的队头是更年轻的错路项。
        kill_any = (redir_backend_0 | interrupt_kill | kill_trap) | kill_busy;
        mret_now  = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_mret[head_pointer] &&
                    !entry_exception[head_pointer] && !kill_any;
        exc_now  = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_exception[head_pointer] && !kill_any;
//★ S5：判定拍只出 A（清前端 + 改 PC + 冻分配），**不改队头退休** —— 边界那条要走到队头
//  自己退休才发 B。所以这里不再有 redirect_now 这种东西。
//★ 改向那一拍【要退】：队头就是那条分支、它是正确路径，改向/退休/杀后续是同一个事件。
//  吃 kill_any 的话队头永远退不掉（这次冲刷是它自己发的）⇒ 自锁。
//★ exc_gone 随部署删除：异常改成【按项标记】之后，标记落在 entry_exception[exc_index] 上，
//  项在交付前不会消失（head_ok 的 !entry_exception 把它挡在队头）⇒ 不存在"边界项已退"这一档。
//★ 三条标记口各自成项、不合成一根：命中队头就在 head_ok 里逐条取反，同拍标记必须走标记口。
        head_ok   = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_written[head_pointer] &&
                    (entry_judged[head_pointer] || (!entry_jump[head_pointer] && !entry_system_rd_head_pointer)) &&
                    !entry_exception[head_pointer] &&
                    !(exc0_en  && (exc0_index  == head_pointer))  &&
                    !(exc1_en  && (exc1_index  == head_pointer))  &&
                    !(exception_selected_enable && (exception_selected_index == head_pointer))  &&
                    !kill_any;
//★ 边界那条【不许同拍从 head+1 一起退】：它当上队头、单独退休，下一拍才发 B。
//  不然错路的后继会跟着它一起退掉、写进寄存器堆（实测 seed5：jal 后面那条 lui 提交了）。
        head_next_ok  = head_ok && !mret_now && !exc_now &&
                    !(flush_boundary_valid && (head_pointer == flush_boundary_index)) &&
                    !entry_mret[head_next_pointer] &&
                    (entry_count >= 5'd2) && entry_valid[head_next_pointer] && entry_written[head_next_pointer] &&
                    (entry_judged[head_next_pointer] || (!entry_jump[head_next_pointer] && !entry_system_rd_head_next_pointer)) &&
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
//  · 两条 lane 同拍判定取更老的：年轻的那条在错路上，会被这次改向带走（一次 5 位比较）。
//  · 只认在册且世代对得上的项 —— 迟到判定不许改向。
//  · 年龄 = 槽号 - 队头。在册窗口是连续的一段、跨度 ≤ DEPTH ⇒ 这个差就是名次，5 位减法够。
//  · 和在册边界比一次：候选更老才受理 ⇒ 同时满足"新的不许压老的"和"老的可以顶掉新的"。
//------------------------------------------------------------------------------
        judge_pick1  = judge1_ok & (!judge0_ok | (judge1_age < judge0_age));
        judge_index  = judge_pick1 ? branch_unit_lane1_index : bju_index;
        judge_target = judge_pick1 ? branch_unit_lane1_tgt : bju_tgt;
        judge_age    = judge_pick1 ? judge1_age : judge0_age;
        judge_valid  = (judge0_ok | judge1_ok) & !head_is_exception & !kill_any;
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
        commit_we0_comb   = head_ok && (entry_destination_register_rd_head_pointer != 5'd0);
        commit_rd0_comb   = entry_destination_register_rd_head_pointer;
        commit_phys0_comb = entry_phys_rd_head_pointer;
        commit_we1_comb   = head_next_ok && (entry_destination_register_rd_head_next_pointer != 5'd0);
        commit_rd1_comb   = entry_destination_register_rd_head_next_pointer;
        commit_phys1_comb = entry_phys_rd_head_next_pointer;
//CSR：队头这一项是 CSR ⇒ 载荷随提交口出去。1 宽就够 —— 同时至多一条 CSR 在飞
//（isq 的 csr_held 要求它发射时就是队头），同拍不可能有第二条在 head+1 上等。
        commit_csr_comb         = head_ok && entry_csr_rd_head_pointer;
        commit_csr_payload_comb = (csr_side_valid && (csr_side_index == head_pointer)) ? csr_side_payload : 22'd0;

//★ 2026-11-09：原来这里把在册各项的 entry_phys 展成 one-hot 再 OR（squash_phys_mask）。
//  rename 的回收走 flush_free_mask = ~arch_next_mask，那个口从没被读过 ⇒ 整段删除。

//★ 2026-11-05 NaxRiscv 口径：rat 的回滚不再靠"挑最老被杀项的 old_phys"。
//  改向一经判定就冻住分配（见 redirect_stop_alloc），被杀项的号直接回收，
//  rat 在冻住期间不可能被任何新项读到 ⇒ 提交拍整片抄回 arch_map 就够。
//  端口保留（rename 已经不接），恒 0。
        squash_arch_hit  = 32'd0;
        squash_arch_phys = 192'd0;
//  survive_hit 是给 rat 用的，现在归零（同上）。
        squash_survive_hit = 32'd0;
    end

//★ 改向/陷阱事件打一拍：head 那一拍只做判定，重定向信号从触发器出去。
//★ mret 不带落点：mepc 是 csr 里的架构状态，不是指令自带的立即数 —— 由 pc 在事件那一拍

    always @(posedge clk) begin
        if (reset_registered) begin
            redir_backend_0 <= 1'b0; kill_redirect_mret <= 1'b0; kill_redirect_target <= 32'd0;
            redir_frontend <= 1'b0;
            kill_trap <= 1'b0; kill_trap_cause <= 4'd0; kill_trap_pc <= 32'd0; kill_trap_value <= 32'd0;
        end
        else begin
//A 域（判定拍 / mret）：改 PC + 清前端。**不吃 kill_busy**：
//  judge_accept 自己已经吃过 kill_any，再门一次会把刚受理的边界吞掉。
            redir_frontend <= judge_accept | mret_now;
            if (judge_accept)  kill_redirect_target <= judge_target;
            if (!kill_busy) begin
//B 域（提交口）：在册边界走到队列头 ⇒ 整表 + 后端一起作废。
//★ mret 的 B 跟 A 同拍 —— 两把刀必须同拍，不然清表和改 PC 错开。
                redir_backend_0   <= boundary_now | mret_now;
                kill_redirect_mret    <= mret_now;
                kill_trap <= exc_now;
                if (exc_now) begin
                    kill_trap_cause <= entry_exc [head_pointer];
                    kill_trap_pc    <= entry_pc_rd_head_pointer;
                    kill_trap_value  <= entry_target_of_head;
                end
            end
            else begin
                redir_backend_0   <= 1'b0;
                kill_redirect_mret    <= 1'b0;
                kill_trap <= 1'b0;
            end
        end
    end

//影子那根：上面那一笔打一拍。给 isq / idu 这些晚一拍才清的模块，以及自己的 redirect_stop_alloc。
    always @(posedge clk) begin
        if (reset_registered) redir_backend_1 <= 1'b0;
        else                  redir_backend_1 <= (redir_backend_0 | interrupt_kill | kill_trap);
    end

//在册边界：判定拍受理时锁下，提交口（以及异常/中断交付）那一拍撤掉。
//  ★ 撤在 后端击杀：B / 异常 / 中断 三种都把整张表清了，边界自然不存在。

    always @(posedge clk) begin
        if (reset_registered) flush_boundary_valid <= 1'b0;
        else if ((redir_backend_0 | interrupt_kill | kill_trap)) flush_boundary_valid <= 1'b0;
        else if (judge_accept) begin
            flush_boundary_valid <= 1'b1;
            flush_boundary_index <= judge_index;
        end
    end

//★ 这两个都打一拍给 isq：判定用【当拍】的 ROB 状态算（边界还在不在窗口里 / 队头是不是故障项），
//  算完才寄存。isq 拿它们去杀 D/Q，晚一拍做没有副作用 —— 那几拍前端本来就不送新指令。
//  ★ 为什么要寄存：kill_trap 是组合出口、在 isq 里扇出 ~128，它和 flush_partial 一起构成
//    "cont 仲裁 → rob 边界判定 → isq 击杀+操作数选择" 那条跨三模块的长链。
//年龄基准位向量：第 i 位 = 槽 i 在队头之上（含队头）。cont 的异常仲裁用它做环上年龄比较。
    integer ka;
    always @(*) begin
        slot_above_head    = {DEPTH{1'b0}};
        for (ka = 0; ka < DEPTH; ka = ka + 1) begin
            slot_above_head[ka]    = (ka[4:0] >= head_pointer);
        end
    end

//击杀：改向只在提交口发一次、前端和整张 ROB 一起作废 ⇒ 每一次冲刷都是整窗的。
//★ 2026-11-08：原来那两根常数掩码线（MASK_ALL）连同端口一起摘掉，消费端一律判
//  "这一拍有没有冲刷"（isq 的 kill_all / lsu 的 flush_commit / mdu 的 flush_window）。
//★ S5：判定拍发 A 之后分配就冻住 —— 前端会一直取到正确路径、停在分配口。
//  解冻在提交口那一拍之后：那一拍 后端击杀 本来就挡着分配，所以不会漏进错路项。
    assign redirect_stop_alloc = flush_boundary_valid | redir_backend_1;

//==============================================================================
//  中断返回点：队头的指令地址（组合读）
//  ★ 逐项列成 case，不写 entry_pc_rd_head_pointer：数组元素在 always @(*) 里进不了敏感表
//    （本项目实测过，见 isq 值表读口的同款注释）。
//  ★ 组合而非寄存：队头同一拍可能刚退休，寄存版会把已退休那条当成返回点 ⇒ 它被执行两次。
//==============================================================================
    always @(*) begin
        head_pc_out = entry_pc_rd_head_pointer;
        head_valid_out = entry_valid[head_pointer];
    end

//★ S5：改向在册（寄存一拍）给 isq 挡新的访存发射。
//  就是"边界在册"本身 + 事件那片 dly —— 16 项扫描整块删掉。
//  挡"新发"就够：已在 s2/s3 的那几条会在提交口那一拍被击杀掩码杀干净。
    always @(posedge clk) begin
        if (reset_registered) redirect_any <= 1'b0;
        else                  redirect_any <= flush_boundary_valid | redir_backend_0;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            head_one_hot_registered    <= {DEPTH{1'b0}};
            retire_step     <= 2'd0;
        end
        else begin
            head_one_hot_registered    <= {{(DEPTH-1){1'b0}}, 1'b1} << head_pointer_next;
            retire_step     <= retire_step_next;

        end
    end

//队头指针：退 0/1/2 笔；陷阱交付也前进一格（盖过上面）。
// P1: compute the advance as retire_step once, then add. The decision lives here only;
// head_pointer's D side is now a 5-bit increment, and csr counts with retire_step.
    always @(*) begin
        if (head_next_ok)      retire_step_next = 2'd2;
        else if (head_ok)  retire_step_next = 2'd1;
        else               retire_step_next = 2'd0;
        if (kill_trap)     retire_step_next = 2'd1;
        head_pointer_next = (head_pointer + {4'b0000, retire_step_next}) & DEPTH_MASK;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            head_pointer <= 5'd0;
        end
        else begin
            head_pointer <= head_pointer_next;
        end
    end

//提交口打一拍：把"要不要提交 / 提交哪个架构号 / 对应哪个物理号"在提交拍算好、寄存，
//  下一拍再交给 rename 的提交口（arch_map 定格 + 物理号回收）。
//★ 为什么打：arch_map 是变址写阵列，综合器表达不出 CE，只能合成 D_i = (we&&rd==i)?phys:Q_i
//  ⇒ head_ok 里的中断/异常/分支判定锥会整条压在阵列的 D 引脚上。打一拍后长锥止于此。
//★ 绝不能在 kill_trap / 冲刷拍清：head_ok 与 kill_trap 天然互斥（一个要 !entry_exception、一个要 entry_exception）；
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
                entry_judged[ri]  <= 1'b0;
                entry_mret[ri]  <= 1'b0;
            end
            csr_side_valid <= 1'b0;
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
            if (alloc_take_0) begin
                entry_valid[tail_pointer]  <= 1'b1;
//★ 不写 rd 的项不由"出厂"给 wr，由 issue_enable_0 那一拍置位（见下）：发射级是 1 深 Q 之后
//  "执行一定早于退役"不再成立 —— 没发出去的分支会被队头直接退掉，跳转就永远不发生。
                entry_written[tail_pointer] <= 1'b0;
                entry_exception[tail_pointer] <= 1'b0;
                entry_judged[tail_pointer]  <= 1'b0;
//★ mret 标签【随分配落地】：和 entry_jump/entry_system 同源，都是 idu 对指令字的译码位，
//  跟指令一起进项，不靠后面的边带脉冲补写。
//  原来那条路是 cont 的 mret_mark/mret_index（bju 判定级 1 拍脉冲 + 槽号），实测会被同一窗口里
//  别人的异常标志压掉 —— seed36 第二遍处理器：非法指令的 exc_illegal_q 正好盖住 mret 的唯一那拍
//  ⇒ 标签丢了 ⇒ 那条 mret 当普通指令退休、PC 顺延到下一条，整段 body 重跑。
                entry_mret[tail_pointer]  <= allocation_mret_0;
            end
            if (alloc_take_1) begin
                entry_valid[allocation_index_1]  <= 1'b1;
                entry_written[allocation_index_1] <= 1'b0;
                entry_exception[allocation_index_1] <= 1'b0;
                entry_judged[allocation_index_1]  <= 1'b0;
                entry_mret[allocation_index_1]  <= allocation_mret_1;
            end
            if (alloc_take_2) begin
                entry_valid[allocation_index_2]  <= 1'b1;
                entry_written[allocation_index_2] <= 1'b0;
                entry_exception[allocation_index_2] <= 1'b0;
                entry_judged[allocation_index_2]  <= 1'b0;
                entry_mret[allocation_index_2]  <= allocation_mret_2;
            end
//完成回填：wr 与 data 必须【同条件、同一沿】写入（拆开会出现"wr 已置、data 还是旧值"的一拍窗口）。
//★ 世代校验必需：被冲刷/已释放的槽在新住户身上的迟到上报只有它能挡（entry_valid 挡不住"新住户也 valid"）。
//★ "发出即置可退"只给【根本不会有完成口】的项：写 rd 的等完成口；M 类与 LOAD 即使 rd==0
//  也照样回报 ⇒ 必须排掉，否则提前退休后那个晚到的完成口会砸到槽的新住户头上。
            if (issue_enable_0 && entry_valid[issue_index_0] && (entry_generation[issue_index_0] == issue_generation_0) &&
                (entry_destination_register_rd_issue_index_0 == 5'd0) && !entry_pending_rd_issue_index_0) begin
                entry_written[issue_index_0] <= 1'b1;
            end
            if (issue_enable_1 && entry_valid[issue_index_1] && (entry_generation[issue_index_1] == issue_generation_1) &&
                (entry_destination_register_rd_issue_index_1 == 5'd0) && !entry_pending_rd_issue_index_1) begin
                entry_written[issue_index_1] <= 1'b1;
            end
//CSR 载荷在【发射】那一拍锁一深。CSR 必须排到队头才准发 ⇒ 同拍最多一条，一深够用。
            if (issue_enable_0 && entry_valid[issue_index_0] && entry_csr_rd_issue_index_0) begin
                csr_side_valid   <= 1'b1;
                csr_side_index   <= issue_index_0;
                csr_side_payload <= issue_csr_payload_0;
            end
            else if (issue_enable_1 && entry_valid[issue_index_1] && entry_csr_rd_issue_index_1) begin
                csr_side_valid   <= 1'b1;
                csr_side_index   <= issue_index_1;
                csr_side_payload <= issue_csr_payload_1;
            end
            if (acc_alu_o) begin
                entry_written[alu_index] <= 1'b1;
            end
            if (acc_alu2_o) begin
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
                    entry_target_jump_lane0[bju_index] <= bju_tgt;
                    entry_target_source   [bju_index] <= TARGET_FROM_JUMP_LANE0;
                end
            end
            if (branch_unit_lane1_judged && entry_valid[branch_unit_lane1_index] && (entry_generation[branch_unit_lane1_index] == branch_unit_lane1_generation)) begin
                entry_judged[branch_unit_lane1_index] <= 1'b1;
                if (entry_jump[branch_unit_lane1_index]) begin
                    entry_target_jump_lane1[branch_unit_lane1_index] <= branch_unit_lane1_tgt;
                    entry_target_source   [branch_unit_lane1_index] <= TARGET_FROM_JUMP_LANE1;
                end
            end
            if (acc_mul_o) begin
                entry_written[mul_index]   <= 1'b1;
            end
            if (acc_mul2_o) begin
                entry_written[multiply_lane1_index]   <= 1'b1;
            end
            if (acc_div_o) begin
                entry_written[div_index]   <= 1'b1;
            end
            if (acc_div2_o) begin
                entry_written[divide_lane1_index]   <= 1'b1;
            end
            if (acc_ld_o) begin
                entry_written[ld_index]   <= 1'b1;
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
                entry_target_exception_lane0[exc0_index] <= exc0_tval;
                entry_target_source         [exc0_index] <= TARGET_FROM_EXC_LANE0;
            end
            if (exc1_en && entry_valid[exc1_index]) begin
                entry_exception  [exc1_index] <= 1'b1;
                entry_written  [exc1_index] <= 1'b1;
                entry_exc [exc1_index] <= exc1_cause;
                entry_target_exception_lane1[exc1_index] <= exc1_tval;
                entry_target_source         [exc1_index] <= TARGET_FROM_EXC_LANE1;
            end
            if (exception_selected_enable && entry_valid[exception_selected_index]) begin
                entry_exception  [exception_selected_index] <= 1'b1;
                entry_written  [exception_selected_index] <= 1'b1;
                entry_exc [exception_selected_index] <= exception_selected_cause;
                entry_target_exception_selected[exception_selected_index] <= exception_selected_trap_value;
                entry_target_source            [exception_selected_index] <= TARGET_FROM_EXC_SELECTED;
            end
            if (alloc_take_0) entry_written[tail_pointer] <= 1'b0;
            if (alloc_take_1) entry_written[allocation_index_1] <= 1'b0;
            if (alloc_take_2) entry_written[allocation_index_2] <= 1'b0;
//★ 冲刷 = 整窗作废（改向 / 异常 / 中断都在提交口发的同一笔，边界恒为队头）。
//  ★ 冲刷拍分配口被 后端击杀 挡住，所以这里不会误清同拍新进来的项。
            if ((redir_backend_0 | interrupt_kill | kill_trap)) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    entry_valid[ri]     <= 1'b0;
                    entry_written[ri]    <= 1'b0;
                    entry_exception[ri]    <= 1'b0;
                    entry_mret[ri]  <= 1'b0;
                end
                csr_side_valid <= 1'b0;
            end
//陷阱交付拍：整条 ROB 作废。放在最后 ⇒ 它盖过同拍的一切（含"刚分配的那一项"）。
            if (kill_trap) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    entry_valid[ri]  <= 1'b0;
                    entry_written[ri] <= 1'b0;
                    entry_exception[ri] <= 1'b0;
                    entry_mret[ri] <= 1'b0;
                end
                csr_side_valid <= 1'b0;
            end
        end
    end

//==============================================================================
// 真 PRF 写口的物理号 / 存活判据
//   连续赋值（不进 always @(*)）—— 数组元素在 always @(*) 里进不了敏感表，本仓踩过。
//==============================================================================
    assign prf_wb_phys_0 = entry_phys_rd_prf_wb_index_0;
    assign prf_wb_phys_1 = entry_phys_rd_prf_wb_index_1;
    assign prf_ld_phys   = entry_phys_rd_prf_ld_index;
    assign prf_alu_early_phys_0 = entry_phys_rd_prf_alu_early_index_0;
    assign prf_ld_early_phys    = entry_phys_rd_prf_ld_early_index;
    assign prf_ld_early_live    = entry_valid[prf_ld_early_index] & (entry_generation[prf_ld_early_index] == prf_ld_early_generation);
    assign prf_alu_early_phys_1 = entry_phys_rd_prf_alu_early_index_1;
//★ 2026-11-07：ALU 有自己值表写口了 ⇒ 也该有存活判据（与 load 早口逐字同形）
    assign prf_alu_early_live_0 = entry_valid[prf_alu_early_index_0] & (entry_generation[prf_alu_early_index_0] == prf_alu_early_generation_0);
    assign prf_alu_early_live_1 = entry_valid[prf_alu_early_index_1] & (entry_generation[prf_alu_early_index_1] == prf_alu_early_generation_1);
    assign prf_div_wake_phys_0  = entry_phys_rd_prf_div_wake_index_0;
    assign prf_div_wake_phys_1  = entry_phys_rd_prf_div_wake_index_1;
    assign prf_wb_live_0 = prf_wb_en_0 & entry_valid[prf_wb_index_0] & (entry_generation[prf_wb_index_0] == prf_wb_generation_0);
    assign prf_wb_live_1 = prf_wb_en_1 & entry_valid[prf_wb_index_1] & (entry_generation[prf_wb_index_1] == prf_wb_generation_1);
    assign prf_ld_live   =                 entry_valid[prf_ld_index]   & (entry_generation[prf_ld_index]   == prf_ld_generation);

endmodule
