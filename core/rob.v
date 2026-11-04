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
    output reg [31:0] commit_data0,
    output reg [3:0]  commit_slot0,
    output reg        commit_gen0,
    output reg        commit_write_enable_1,
    output reg [4:0]  commit_destination_1,
    output reg [31:0] commit_data1,
    output reg [3:0]  commit_slot1,
    output reg        commit_gen1,
//★ 2026-11-04 真 PRF：这两条提交项各自的物理号（arch_map 定格用）
    output reg [5:0]  commit_phys0,
    output reg [5:0]  commit_phys1,
//★ 冲刷掩码：本拍在册的全部项物理号各展成 one-hot 后 OR 起来，直接给 rename 回收。
//  kill_arc 已删 ⇒ 本设计每次冲刷都是整表作废，不需要按边界挑。
    output reg [63:0] squash_phys_mask,
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
// mret：与 bju 同一套（项内标签 + 队头改向），落点取 csr 的 mepc。
    input        mret_mark,
    input [3:0]  mret_index,
    input [31:0] mret_tgt,
    output reg        mret_valid_delayed,
    output reg        redirect_any,
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

parameter integer DEPTH = 16;   // ★宏化旋钮：16=大核 / 8=小核（必须是 2 的幂）
localparam [3:0] DEPTH_MASK = DEPTH - 1;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，各模块各自打一拍，彼此没有相位差。
reg reset_registered;
always @(posedge clk) reset_registered <= rst;


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
//每槽的世代位：每次分配翻转一次。完成上报必须世代一致才被接受 —— 这是"迟到上报
//打进新住户"的唯一根治手段（entry_valid 挡不住"新住户也 valid"）。
reg        entry_generation  [0:DEPTH-1];
reg        entry_jump  [0:DEPTH-1];
reg        entry_pending [0:DEPTH-1];
reg        entry_system  [0:DEPTH-1];
integer ai;

//重建口：物理槽序，静态下标（零 mux）

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
reg [31:0] commit_data0_comb, commit_data1_comb;
reg [3:0]  commit_slot0_comb, commit_slot1_comb;
reg        commit_gen0_comb, commit_gen1_comb;
reg [5:0]  commit_phys0_comb, commit_phys1_comb;
integer ri;
integer si;

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
reg        entry_redirect[0:DEPTH-1];
//mret 标签：与 entry_redirect 并列的第二种【队头改向】，落点不由项携带（取 mepc）
reg        entry_mret [0:DEPTH-1];
//head 拍判定出来的事件（组合）。下一沿变成对外的重定向事件 —— 冲刷从这排触发器出去。
reg        mret_now, redirect_now, exc_now;
reg [31:0] redirect_target_now;
//dly 里现在是否有事件（= 本拍正在冲刷）。它挡住队头写回和新的装载。
wire       delayed_busy = redirect_valid_delayed | exception_valid_delayed;
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
                entry_generation[ai]  <= 1'b0;
                entry_jump[ai]  <= 1'b0;
                entry_pending[ai] <= 1'b0;
                entry_system[ai]  <= 1'b0;
            end
        end
        else begin
            if (allocation_en_0 && !full && !flush_control_rob) begin
                entry_tag[tail_pointer]  <= tail_tag;
//前送表的目的寄存器号（保守侧：SYSTEM 也算写，带上 rd != 0 后只剩非法编码会"报了写其实不写"）。
                entry_destination_register[tail_pointer]   <= (allocation_we_0 && (allocation_rd_0 != 5'd0)) ? allocation_rd_0 : 5'd0;
                entry_phys[tail_pointer]                   <= (allocation_we_0 && (allocation_rd_0 != 5'd0)) ? allocation_phys_0 : 6'd0;
                entry_generation[tail_pointer]  <= ~entry_generation[tail_pointer];
                entry_jump[tail_pointer]  <= allocation_jmp_0;
                entry_pending[tail_pointer] <= allocation_pend_0;
                entry_system[tail_pointer]  <= allocation_ecall_0 | allocation_ebreak_0 | allocation_illegal_0 | allocation_mret_0;
                entry_pc[tail_pointer]      <= allocation_pc_0;
            end
            if (allocation_en_1 && !full && !flush_control_rob) begin
                entry_tag[allocation_index_1] <= tail_tag + {4'd0, allocation_en_0};
                entry_destination_register[allocation_index_1]   <= (allocation_we_1 && (allocation_rd_1 != 5'd0)) ? allocation_rd_1 : 5'd0;
                entry_phys[allocation_index_1]                   <= (allocation_we_1 && (allocation_rd_1 != 5'd0)) ? allocation_phys_1 : 6'd0;
                entry_generation[allocation_index_1]  <= ~entry_generation[allocation_index_1];
                entry_jump[allocation_index_1]  <= allocation_jmp_1;
                entry_pending[allocation_index_1] <= allocation_pend_1;
                entry_system[allocation_index_1]  <= allocation_ecall_1 | allocation_ebreak_1 | allocation_illegal_1 | allocation_mret_1;
                entry_pc[allocation_index_1]      <= allocation_pc_1;
            end
        end
    end



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
//  落点不同 —— bju 的落点在项里（entry_target，判定拍就算完），mret 的落点在 csr（mepc），
//  必须【队头现读】：更老的 csrw mepc 是头门控的，它在 mret 之前提交，所以队头这一拍读到的
//  才是架构值；在判定拍锁进项就会读到旧值。
        // ★ 判定直接读队头那一项（阵列）。写回在队头这一拍完成，冲刷事件只进 dly、下一拍才出。
        // ★ dly 非空时既不出新事件，也不许退休 —— 这一拍的队头是更年轻的错路项。
        flush_any = flush_control_rob | delayed_busy;
        mret_now  = (entry_count != 4'd0) && entry_valid[head_pointer] && entry_mret[head_pointer] &&
                    !entry_exception[head_pointer] && !flush_any;
        redirect_now = mret_now | ((entry_count != 4'd0) && entry_valid[head_pointer] && entry_jump[head_pointer] &&
                    entry_judged[head_pointer] && entry_redirect[head_pointer] && !entry_exception[head_pointer] && !flush_any);
        exc_now  = (entry_count != 4'd0) && entry_valid[head_pointer] && entry_exception[head_pointer] && !flush_any;
        redirect_target_now = mret_now ? mret_tgt : entry_target[head_pointer];
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
//★ head+1 要改向的项【不许同拍退休】：改向只看 head_pointer，2 宽退休会让它从 head+1 直接退掉、
//  永远当不上队头 ⇒ 改向被静默吞掉（实测 CoreMark：slot12 的 ret 和第 11 条同拍退休，
//  ret 的落点再也没生效，整核顺着 0x2794 的 epilogue 顺序跑进死路）。
//  让它多等一拍、当上队头再改向；代价是每次错预测多一拍。
        head_next_ok  = head_ok && !redirect_now && !exc_now &&
                    !entry_redirect[head_next_pointer] && !entry_mret[head_next_pointer] &&
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
        commit_data0_comb = entry_data[head_pointer];
        commit_we1_comb   = head_next_ok && (entry_destination_register[head_next_pointer] != 5'd0);
        commit_rd1_comb   = entry_destination_register[head_next_pointer];
        commit_phys1_comb = entry_phys[head_next_pointer];
        commit_data1_comb = entry_data[head_next_pointer];

//★ 2026-11-04 真 PRF：冲刷时把【在册全部项】的物理号各展成 one-hot 再 OR，直接交给 rename 回收。
//  kill_arc 已删（每次冲刷整表作废、没有"部分冲刷"），所以这里不需要按边界挑。
//  没写回的项 entry_phys 恒 0，贡献的是 bit0（物理 0 = x0，rename 那边会掩掉）。
        squash_phys_mask = 64'd0;
        for (si = 0; si < DEPTH; si = si + 1)
            if (entry_valid[si])
                squash_phys_mask = squash_phys_mask | (64'd1 << entry_phys[si]);
        commit_slot0_comb = head_pointer;
        commit_gen0_comb  = entry_generation[head_pointer];
        commit_slot1_comb = head_next_pointer;
        commit_gen1_comb  = entry_generation[head_next_pointer];
    end

//队头字段副本的装载：每个沿装【下一拍的队头】那一项（head_pointer_next 正是 head_pointer 在下一沿的值，
//两者必然同指一项）。纯读阵列 → 寄存器，锥止于触发器。
//★ 改向/陷阱事件打一拍：head 那一拍只做判定，重定向信号和 PC 落点都从触发器出去。
//  这一拍也是改向那条自己退休的那一拍；下一拍全核冲刷（清空整表）。
    always @(posedge clk) begin
        if (reset_registered) begin
            redirect_valid_delayed <= 1'b0; mret_valid_delayed <= 1'b0; redirect_target_delayed <= 32'd0;
            exception_valid_delayed <= 1'b0; exception_cause_delayed <= 4'd0; exception_pc_delayed <= 32'd0; exception_trap_value_delayed <= 32'd0;
        end
        else if (!delayed_busy) begin
            redirect_valid_delayed   <= redirect_now | mret_now;
            mret_valid_delayed    <= mret_now;
            exception_valid_delayed <= exc_now;
            if (redirect_now | mret_now) redirect_target_delayed <= redirect_target_now;
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

//★ 改向在册（寄存一拍）：窗口内任一在册项带 entry_redirect ⇒ 给 idu2 挡新的访存发射。
//  挡"新发"就够：已在 s2/s3 的那几条会在提交口改向那一拍被 kill_arc 杀干净。
    integer rq;
    always @(posedge clk) begin
        if (reset_registered) redirect_any <= 1'b0;
        else begin
            redirect_any <= 1'b0;
            for (rq = 0; rq < DEPTH; rq = rq + 1)
                if (entry_valid[rq] && (entry_redirect[rq] || entry_mret[rq]) && ({1'b0, (rq[3:0] - head_pointer)} < entry_count))
                    redirect_any <= 1'b1;
        end
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

//提交口打一拍：把"要不要写 / 写哪个 / 写什么"在提交拍算好、寄存，下一拍再写寄存器堆。
//★ 为什么打：regfile 是变址写阵列，综合器表达不出 CE，只能合成 D_i = (we&&rd==i)?data:Q_i
//  ⇒ head_ok 里的中断/异常/分支判定锥会整条压在阵列的 D 引脚上。打一拍后长锥止于此。
//★ 绝不能在 exception_valid_delayed / 冲刷拍清：head_ok 与 exception_valid_delayed 天然互斥（一个要 !entry_exception、一个要 entry_exception）；
//  冲刷只作废比边界年轻的项，而打拍里那笔在边界之前 ⇒ 清了会丢一笔合法写。
    always @(posedge clk) begin
        if (reset_registered) begin
            commit_write_enable_0   <= 1'b0;
            commit_destination_0   <= 5'd0;
            commit_data0 <= 32'd0;
            commit_write_enable_1   <= 1'b0;
            commit_destination_1   <= 5'd0;
            commit_data1 <= 32'd0;
            commit_slot0 <= 4'd0;
            commit_slot1 <= 4'd0;
            commit_gen0  <= 1'b0;
            commit_gen1  <= 1'b0;
            commit_phys0 <= 6'd0;
            commit_phys1 <= 6'd0;
        end
        else begin
            commit_write_enable_0   <= commit_we0_comb;
            commit_destination_0   <= commit_rd0_comb;
            commit_phys0 <= commit_phys0_comb;
            commit_data0 <= commit_data0_comb;
            commit_write_enable_1   <= commit_we1_comb;
            commit_destination_1   <= commit_rd1_comb;
            commit_phys1 <= commit_phys1_comb;
            commit_data1 <= commit_data1_comb;
            commit_slot0 <= commit_slot0_comb;
            commit_slot1 <= commit_slot1_comb;
            commit_gen0  <= commit_gen0_comb;
            commit_gen1  <= commit_gen1_comb;
        end
    end

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
                entry_redirect[ri] <= 1'b0;
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
                entry_redirect[tail_pointer] <= 1'b0;
                entry_mret[tail_pointer]  <= 1'b0;
            end
            if (allocation_en_1 && !full && !flush_control_rob) begin
                entry_valid[allocation_index_1]  <= 1'b1;
                entry_written[allocation_index_1] <= 1'b0;
                entry_exception[allocation_index_1] <= 1'b0;
                entry_judged[allocation_index_1]  <= 1'b0;
                entry_redirect[allocation_index_1] <= 1'b0;
                entry_mret[allocation_index_1]  <= 1'b0;
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
                if (entry_jump[bju_index]) begin
                    entry_redirect[bju_index] <= bju_redir;
                    entry_target  [bju_index] <= bju_tgt;
                end
            end
            if (branch_unit_lane1_judged && entry_valid[branch_unit_lane1_index] && (entry_generation[branch_unit_lane1_index] == branch_unit_lane1_generation)) begin
                entry_judged[branch_unit_lane1_index] <= 1'b1;
                if (entry_jump[branch_unit_lane1_index]) begin
                    entry_redirect[branch_unit_lane1_index] <= branch_unit_lane1_redir;
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
//★ 改向必须是一发：不清的话下一拍队头还是它、redirect_valid_delayed 还拉 ⇒ 冲刷永远不退、队头永远退不掉。
//  清掉之后下一拍 flush 已撤、head_ok 抬起 ⇒ 它正常退休（改向与退休分两拍做）。
            if (redirect_valid_delayed) entry_redirect[head_pointer] <= 1'b0;
            if (mret_valid_delayed)  entry_mret [head_pointer] <= 1'b0;
//★ 标记陷阱时必须【同时置 wr】：故障指令的写被 pd 挡掉、也不会再有任何单元回报它，
//  只置 exc 不置 wr 它就退不掉 ⇒ ROB 排不空 ⇒ redir 等不到 rob_empty ⇒ 整核挂死。
//★ S4：标记即写项。必须排在所有 acc_*/判定口之后（同一槽同一沿，后写者赢）。
//★ mret 标签：与异常同一套（判定拍写项、队头生效）。表里那一项一定还在册 ——
//  mret 不写 rd，但它的 entry_written 由发射拍置，且 entry_mret 只影响"队头要不要改向"，不影响可退。
            if (mret_mark && entry_valid[mret_index]) entry_mret[mret_index] <= 1'b1;
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
//★ 冲刷 = 清空整表。改向那一条在【上一拍】已经退掉，此刻在册的项全都是更年轻的。
            if (flush_control_rob) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    entry_valid[ri]     <= 1'b0;
                    entry_written[ri]    <= 1'b0;
                    entry_exception[ri]    <= 1'b0;
                    entry_mret[ri]  <= 1'b0;
                    entry_redirect[ri] <= 1'b0;
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
