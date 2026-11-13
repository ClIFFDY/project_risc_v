`timescale 1ns / 1ps
//==============================================================================
// rob.v —— 重排序缓冲（reorder buffer）
//
//   执行可以乱序完成，写寄存器堆只走按序退这一条路 ⇒ 同 rd 的先后由结构保证。
//   每条指令占一项（含不写 rd 的与 store —— "谁比谁老"必须覆盖每一条）。
//   三区：分配（tail 端）/ 提交（head 端）/ 项状态阵列（唯一被三区共写，只能一个 always 块）。
//
//   各口相位（本模块没有自己的流水级，口挂错一拍就是"回填打到错项"这类静默错）：
//     allocation_enable —— 分配拍；完成口 —— 结果被写口级处置那一拍（mul/ld 不定长）；
//     bju 判定口 —— 判定寄存一拍后的 wb 拍，后端击杀同一拍；
//     陷阱交付 —— 故障项排到队头那一拍，A+B 一起发，上级同拍锁 mepc/mcause/mtval。
//==============================================================================

module rob(
    input clk, rst,
//分配口：相位 = iffu 出队格换新那一拍；三条 lane 各一套、独立推进（口 0 最老）
    input        allocation_en_0,
    input        allocation_en_1,
    input        allocation_en_2,
    input [4:0]  allocation_rd_0,
    input [4:0]  allocation_rd_1,
    input [4:0]  allocation_rd_2,
    input        allocation_we_0,
    input        allocation_we_1,
    input        allocation_we_2,
//★ 每槽物理号（真 PRF）：跟着 ROB 项走，提交时定格进 arch_map
    input [5:0]  allocation_phys_0,
    input [5:0]  allocation_phys_1,
    input [5:0]  allocation_phys_2,
//★ CSR：1 位/项 + 一深旁表载荷。CSR 必须排到队头才准发 ⇒ 同拍最多一条"已发射未退休"的，
//  发射那一拍锁一深就够（352 FF → 27 FF）。
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
//SYSTEM 类（ecall/ebreak/illegal/mret）：异常/改向标记在 bju 判定延迟级才出，退休必须等那一级过去
    input        allocation_ecall_0, allocation_ebreak_0, allocation_illegal_0, allocation_mret_0,
    input        allocation_ecall_1, allocation_ebreak_1, allocation_illegal_1, allocation_mret_1,
    input        allocation_ecall_2, allocation_ebreak_2, allocation_illegal_2, allocation_mret_2,
//★ 2026-11-13（fence）：MISC-MEM 的两条。随分配写进项里，队头就是"这道栅栏"。
//  fence 只要求排空（写队列可见），fence.i 还要求把 icache 整表作废并重取。
    input        allocation_fence_0, allocation_fence_i_0,
    input        allocation_fence_1, allocation_fence_i_1,
    input        allocation_fence_2, allocation_fence_i_2,
//★ 2026-11-13（P4）：写 CSR 0x7C0 = 三层缓存全失效（软件一致性原语）。
//  提交口的时序与 fence.i 同构，只是多走一步"等 L2 的 maint_done"。
    input        allocation_cache_flush_0,
    input        allocation_cache_flush_1,
    input        allocation_cache_flush_2,
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
//★ S3：判定要不要改向 + 落点，随判定口进来写进项；队头到了才真正改 PC
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
    input        irq_kill,     // 中断受理那一拍同样整窗作废（cont 给）
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
//★ CSR 提交口（与 commit_write_enable_* 同相，打一拍）：csr.v 在这一拍读-改-写并落地
    output reg        commit_csr_valid,
    output reg [21:0] commit_csr_payload,
//★ 精确冲刷：每个架构号在【击杀集合】里最老那条的 old_phys（回滚 rat 用），hit 位 = 该号有被杀项写过。
//★ 2026-11-08/09：kill 掩码恒为 MASK_ALL、squash_phys_mask 无人读，两根输出连同端口一起摘掉。
    output reg [31:0]  squash_arch_hit,
    output reg [191:0] squash_arch_phys,
//★ 精确冲刷：这个架构号有没有【幸存】在飞项写过；没有 ⇒ rat 必须退回 arch_map（判定期新分配的号没有 ROB 项）。
    output reg [31:0]  squash_survive_hit,
//★ 判定拍与后端击杀那两拍都不许分配：本拍新进来的项不在 kill 掩码里，会被当成幸存项把尾指针算歪。
//★ 2026-11-05：改向一经判定就冻住分配，直到那条分支自己提交 ⇒ rat 回滚退化成提交拍整片抄回 arch_map。
    output             redirect_stop_alloc,
//★ 真 PRF：三个写回口的物理号 + 存活判据。live = 本项还有效且世代对得上 ——
//  冲刷后在飞的结果还会落完，那些号可能已被回收再分配，晚到的死写不许进阵列。
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
//★ 提交口改向：队头这一项判错过了 ⇒ 本拍改 PC、杀掉它之后的全部（队头即边界，消费端不再需要年龄判据）。
//★ "已判定、还没排到队头"的改向在册：给 isq 挡【新的访存发射】—— 错路访存可能打到没有从设备的
//  地址，LSU 永远等应答（实测 CoreMark：错路 lbu 1(x0) 卡死整核）。挡"新发"就够。
//mret：与 bju 同一套（项内标签 + 队头改向），落点由 pc 直接取 csr 的 mepc。
    output reg        kill_redirect_mret,
    output reg        redirect_any,
//★ S5：A 域事件（判定拍 / mret）—— 只改 PC + 清前端，不碰后端。
    output reg        redir_frontend,
//★ S5：B 域事件（边界走到提交口）—— 只清整表 + 后端，不碰前端。
    (* max_fanout = 32 *) output reg redir_backend_0,
    output reg        redir_backend_1,   // 上面那根打一拍：清 isq / idu 等后端
    output reg [31:0] kill_redirect_target,
//★ 2026-11-13（fence.i）：icache 整表作废脉冲，与改向同拍（1 拍）。
    output reg        icache_invalidate,
//★ 2026-11-13（P4，软件的一致性原语 CSR 0x7C0）：另外两层的失效。
//  dcache 是 1 拍脉冲（翻世代位）；L2 是【电平】—— 一直举到 maint_done 为止。
//  ★ L2 必须是电平而不是脉冲：l2cache 只在 S_IDLE 里看 maint，回填/写穿期间
//    一个单拍脉冲会被整条丢掉 ⇒ 等 maint_done 就永远等不到（死锁）。
    output reg        dcache_invalidate,
    output reg        cache_flush_maint,
    input             maint_done,
//★ 2026-11-13（fence）：排空判据，由 cpu_top 组合给出（ELSQ 无已退休项 / 总线无在飞 /
//  L2 写队列空 / L2 空闲）。本模块【先打一拍再用】—— 它反过来参与 head_ok，
//  不寄存就是一条组合环（SQ 的 _ret 里含 ret0_valid = head_ok 的一部分）。
    input             drain_ok,
    output reg [4:0]  head_pointer,
//本拍退休的两条（给 lsu：退休过的项不许再按年龄冲刷作废）
    output reg        ret0_v, ret1_v,
    output reg [4:0]  ret0_index, ret1_index,
    output reg        ret0_generation, ret1_generation,
    output reg        full,
//★ 击杀弧掩码：第 i 位 = "槽 i 比冲刷边界年轻"。寄存版 —— 这是绝对槽号的弧 (边界, 队尾]，
//  队头前进不改变它，晚一拍用完全等价（flush_ok 判的是窗口，那个会变，所以只能当拍算）。
//★ 队头之上位向量（i >= head_pointer）：给 cont 的异常仲裁做环上年龄比较（跨 0 时排序会翻）。
    output reg [31:0] slot_above_head,
// P2: head one-hot (registered)，省掉把指针送出去让每个模块自己比。
    output reg [31:0] head_one_hot_registered,
//---- 中断边界 / mepc：必须取【ROB 队头 = 最老未退休】，不是 isq 队列头（最老未发射）----
//★ 必须组合给出：队头同一拍可能刚退休，寄存版会把已退休那条当成返回点 ⇒ 执行两次。
    input [31:0]  allocation_pc_0, allocation_pc_1, allocation_pc_2,
    output reg [31:0] head_pc_out,
    output reg        head_valid_out,
// P1: head 这一拍前进几格（0/1/2）在这里算一次，csr 的 minstret 直接吃这个增量。
    output reg [1:0]  retire_step,
    output reg        empty,
    output reg [5:0]  occupancy,
    output reg [1:0]  redir_cause,        // 与 redir_frontend / redir_backend_0 同拍：0=分支 1=陷阱 2=mret
//陷阱载荷：与陷阱那一拍同拍，都是 dly 里的寄存器。
    output reg [3:0]  trap_cause,
    output reg [31:0] trap_pc,
    output reg [31:0] trap_value
    );

//---- 参数与声明区（Verilog 先声明后使用）----
parameter integer DEPTH = 32;   // ★宏化旋钮：32=大核 / 16=小核（必须是 2 的幂）
localparam [4:0] DEPTH_MASK = DEPTH - 1;
//改向原因：跟 redir_frontend / redir_backend_0 同拍走的数据位（不是冲刷信号）
localparam [1:0] REDIR_CAUSE_BRANCH = 2'd0,
                 REDIR_CAUSE_TRAP   = 2'd1,
                 REDIR_CAUSE_MRET   = 2'd2;
localparam integer ARCH_NUMBER = 32;   // RV32I 架构寄存器个数（rat 回滚按这个维度展开）

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，各模块各自打一拍，彼此没有相位差。
reg reset_registered;
//---- 信号声明（三区的寄存器统一放这里，按区归类）----
//分配区
reg [4:0]  tail_pointer;
reg [5:0]  entry_count;
reg [4:0]  tail_pointer_next;
reg [5:0]  entry_count_next;
reg        alloc_take_0, alloc_take_1, alloc_take_2;
//每槽目的寄存器号（写 x0 / 不写 rd 一律存 0）：只给提交口和"发出即置可退"用，退项与冲刷不必清
//★ 每槽物理号：只有【复位 + 分配】两个写源，与 rd 同条件同沿。
//每槽世代位：每次分配翻转一次，完成上报必须世代一致 —— 挡"迟到上报打进新住户"。
reg        entry_generation  [0:DEPTH-1];
reg        entry_jump  [0:DEPTH-1];
//★ CSR 位 + 一深旁表载荷（理由见端口注释）：只有发射那一拍写，提交拍按槽号对上才认。
//★ 别试分布式 RAM：本文件 entry_* 同一拍有两条写语句，7 系列 LUTRAM 只有 1 个写口，
//  综合器直接判 8-4767 后落回触发器。
reg [21:0] csr_side_payload;
reg [4:0]  csr_side_index;
reg        csr_side_valid;
//提交区
reg [4:0]  head_next_pointer;
// P4: cont 只送"改向/中断的边界候选"；异常那一档不需要边界。
// 边界归属：改向与 trap 交付都在本模块内解析，head_pointer 不必再送出本模块。
reg [4:0]  head_pointer_next;
reg [1:0]  retire_step_next;
reg        kill_any;
reg        head_ok;
reg        head_next_ok;
//★ 2026-11-13（fence）：队头是不是一道栅栏
reg        head_is_fence, head_is_fence_i, fence_head;
//★ 年龄 = 槽号 - 队头。在册窗口连续且跨度 ≤ DEPTH ⇒ 差值就是名次，5 位减法够。
//★ 不能用 entry_generation 当环绕位：它是每槽各自的分配奇偶，回绕后同一圈会被分配两次（实测）。
//提交口的组合版（打拍前）：head_ok / entry_destination_register_rd_head_pointer / entry_data[head_pointer]
reg        commit_we0_comb, commit_we1_comb;
reg [4:0]  commit_rd0_comb, commit_rd1_comb;
reg [5:0]  commit_phys0_comb, commit_phys1_comb;
//CSR 提交口（打拍前）：队头这一项是 CSR 就吐载荷
reg        commit_csr_comb;
reg [21:0] commit_csr_payload_comb;
//★ S5：改向分两个域 —— A = 判定拍（清前端 + 改 PC + 冻分配），B = 提交口（清整表 + 后端）。
//  边界只在 flush_boundary 那一份寄存器里，一次 5 位比较就够，不按项挂标记。
reg        flush_boundary_valid;
reg [4:0]  flush_boundary_index;
//★ 判定口的候选（组合）：两条 lane 各算一次年龄，取更老的；再和【在册边界】比一次。
reg        judge_valid, judge_accept, judge_pick1;
reg [4:0]  judge_index, judge_age;
reg [31:0] judge_target;
//项状态阵列（v / wr / ex / data / jdg）：唯一被三区共写的对象，只能有一个 always 块（在本区末尾）。
reg        entry_valid  [0:DEPTH-1];
reg        entry_written [0:DEPTH-1];
reg        entry_exception [0:DEPTH-1];
//每槽结果数据：与 entry_written 同条件同一沿写；退项/冲刷/陷阱不必清。
//每槽指令地址（= 指令地址 + 4，csr.mepc 那边减 4 对齐）：只服务"中断返回点"，只读队头那一项。
reg        entry_judged  [0:DEPTH-1];
//mret 标签：第二种【队头改向】，落点不由项携带（取 mepc）
reg        entry_mret [0:DEPTH-1];
//★ 2026-11-13（fence）：与 entry_mret 同型 —— 随分配落地，队头读到就是"这一项是栅栏"。
//  fence_i 是"栅栏 + 作废 icache"；两者的区别只在退役那一拍做什么，排队与年龄语义完全一样。
reg        entry_fence    [0:DEPTH-1];
reg        entry_fence_i  [0:DEPTH-1];
//★ 2026-11-13（P4）：写 CSR 0x7C0 ⇒ 三层全失效。队头读到它就走那条两拍的状态机。
reg        entry_cache_flush [0:DEPTH-1];
//排空判据的打拍版（见端口注释：不打拍就是组合环）
reg        drain_ok_registered;
//"队头这道 fence.i 这一拍真的走"：在组合提交口里算，下一沿变成 icache 作废 + 改向。
reg        fence_i_now;
//★ 2026-11-13（P4）：CSR 0x7C0 的提交口状态机。
//   IDLE → 队头是它且已排空 ⇒ 拉 maint（电平）→ MAINT → maint_done ⇒ 三层失效 + 退休 → IDLE。
localparam [1:0] CACHE_FLUSH_IDLE  = 2'd0,
                 CACHE_FLUSH_MAINT = 2'd1;
reg [1:0]  cache_flush_state;
reg        cache_flush_ready;       // 置起后，队头这一项才许退休
reg        cache_flush_invalidate;  // 1 拍：dcache + icache 一起作废
reg        cache_flush_start;       // 组合：这一拍该起手
reg        cache_flush_now;         // 组合：L2 办完了，这一拍失效 + 改向
reg        head_is_cache_flush;     // 组合：队头这一项就是它
//head 拍判定出来的事件（组合）。下一沿变成对外的重定向事件 —— 冲刷从这排触发器出去。
reg        mret_now, exc_now;
//★ entry_target 按【写者】分阵列（5 个任意槽的写者，没法按槽分银行），配一张 entry_target_source 记最后写者。
//  写者优先级：exception_selected > exc1 > exc0 > 判定lane1 > 判定lane0。
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
//★ S4：异常载荷【逐槽存】。侧寄存器只能记一条，窗口里第二条故障会盖掉第一条（实测 seed1）。
//  cause 用 4 位阵列；pc / tval 复用本槽两个闲置域：entry_data（故障项等不到完成口）与
//  entry_target（判定口只写 entry_jump 的项；跳转类故障那一拍 bju_tgt 就是坏目标，同值）。
reg [3:0]  entry_exc [0:DEPTH-1];
//---- 幸存者表 rb_*（isq 用它重建 pstat，冲刷后第 2 拍取用）----
//★ 本表每拍都在报"当前在册项"，不是冲刷那一拍的组合结果：冲刷在边沿已按年龄清干净 entry_valid。
//★ 只保留窗口上界 age < entry_count 一项：它是"在册"判据，兼管陷阱交付（只回卷指针不清 valid）
//  与已退项（age 回绕成大数）。原来那段 16 项年龄比较的 kill 判据整段删除。
//---- 提交区（head 端）：退役判据、冲刷、陷阱、提交口 ----
//可退 = 在册且已完成。exc 标记项不算可退（它不写寄存器堆，由陷阱交付交上级）；
//★ 同拍标记（exc_en 正落在 head 上）必须【组合】压掉退，否则会在 entry_exception 落盘前退掉。
//★ 异常/中断优先于误预测改向：队头是故障项时，它的 mepc 必须取故障指令自己的 pc，被改向一掺就错。
    wire head_is_exception = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_exception[head_pointer];
    wire [4:0] judge0_age = bju_index             - head_pointer;
    wire [4:0] judge1_age = branch_unit_lane1_index - head_pointer;
    reg        boundary_now;

always @(posedge clk) reset_registered <= rst;

integer ai;

//重建口：物理槽序，静态下标（零 mux）

integer ri;
integer rp;

//---- 分配区（tail 端）：这一拍进几条、进到哪个槽、槽里写什么 ----
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
//★ 三宽分配：逐口判"还放得下第 k 条"（occupancy + k < DEPTH），与 idu 的分配门逐字同门。
        alloc_take_0 = allocation_en_0 & ~(redir_backend_0 | irq_kill) & ((entry_count + 5'd0) < DEPTH);
        alloc_take_1 = allocation_en_1 & ~(redir_backend_0 | irq_kill) & ((entry_count + 5'd1) < DEPTH);
        alloc_take_2 = allocation_en_2 & ~(redir_backend_0 | irq_kill) & ((entry_count + 5'd2) < DEPTH);
    end

//下一拍的指针与占用数：先算退掉的（0/1/2 笔），再算分配的，冲刷最后覆盖。
//★ 判据必须与提交区的项状态块逐字一致，否则"项全清、计数还留着" ⇒ head 指向无效项 ⇒ 队头永不动。
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
        if ((redir_backend_0 | irq_kill)) begin
//★ 2026-11-05：改向 / 异常 / 中断都在提交口发，前端和整张 ROB 同一拍一起作废 ⇒ 恒为整窗。
            entry_count_next  = 5'd0;
            tail_pointer_next = head_pointer;
        end
//陷阱交付：队头那条故障指令【不写寄存器堆】地退掉，比它年轻的项全部作废。
//★ 必须放在冲刷赋值之后：中断的队头是重跑，陷阱的队头是走掉。
        if (redir_backend_0 && (redir_cause == REDIR_CAUSE_TRAP)) begin
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

// 入项六张表：按槽号[1:0] 分成 4 个银行。
//   分布式 RAM 每块只有一个写口，而 3 宽分配每拍有三条写语句（Vivado 直接放弃推断，8-4767）
//   ⇒ 阵列只能落触发器。三个槽恒连续（tail / +1 / +2）⇒ mod 4 必然互异 ⇒ 每银行至多一条写语句。
//   能这么分的只有「分配时写、之后不再改」的六张；valid/written/exception/... 按位清置，留在触发器。
//   不复位（读侧由 entry_valid 挡）；读口写成「一位热码 + 三目或」—— Verilator 5.034 不吃四级以上 ? : 链。
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
//★ 冲刷那一拍一律不退：退是组合的、冲刷的清位到边沿才生效 ⇒ 错路那条若在 head 上会在冲刷拍写进寄存器堆。
//★ 两拍都要挡提交：当拍是冲刷正在发生，下一拍是 ROB 还没清完。
//★ mret 与 bju 挤在同一个出口：都是"队头这一项自己要走、且不是顺序下一条"。bju 的落点在项里，
//  mret 的落点是 csr 的 mepc —— 事件在队头+1 出，csrw mepc 同沿落盘 ⇒ pc 采到的是新值。
        // ★ 判定直接读队头那一项；dly 非空时既不出新事件，也不许退休（这时队头是更年轻的错路项）。
        kill_any = redir_backend_0 | irq_kill;
        mret_now  = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_mret[head_pointer] &&
                    !entry_exception[head_pointer] && !kill_any;
        exc_now  = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_exception[head_pointer] && !kill_any;
//★ 2026-11-13（fence）：栅栏的语义 = "比它老的访存都可见了才许退休"。判据是 drain_ok_registered，
//  由 cpu_top 组合给出、在这里打了拍（见端口注释）。★ 它【不进 kill_any】：栅栏是一道要走的指令，
//  不是一次作废；没等到排空就只是"这一拍还不许退"。
        head_is_fence   = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_fence  [head_pointer] &&
                          !entry_exception[head_pointer];
        head_is_fence_i = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_fence_i[head_pointer] &&
                          !entry_exception[head_pointer];
        fence_head      = head_is_fence | head_is_fence_i;
//★ 2026-11-13（P4）：CSR 0x7C0 的那一项。它也要等排空（语义与 fence 相同），
//  而且要等 L2 把 maint 办完才许退。
        head_is_cache_flush = (entry_count != 5'd0) && entry_valid[head_pointer] &&
                              entry_cache_flush[head_pointer] && !entry_exception[head_pointer];
//★ S5：判定拍只出 A；边界那条要走到队头自己退休才发 B。
//★ 改向那一拍【要退】：队头就是那条分支、它是正确路径，改向/退休/杀后续是同一个事件；吃 kill_any 会自锁。
//★ 三条标记口各自成项、不合成一根：命中队头就在 head_ok 里逐条取反。
        head_ok   = (entry_count != 5'd0) && entry_valid[head_pointer] && entry_written[head_pointer] &&
                    (entry_judged[head_pointer] || (!entry_jump[head_pointer] && !entry_system_rd_head_pointer)) &&
                    !entry_exception[head_pointer] &&
                    !(exc0_en  && (exc0_index  == head_pointer))  &&
                    !(exc1_en  && (exc1_index  == head_pointer))  &&
                    !(exception_selected_enable && (exception_selected_index == head_pointer))  &&
                    (!fence_head || drain_ok_registered) &&
                    (!head_is_cache_flush || cache_flush_ready) &&
                    !kill_any;
//★ 边界那条不许同拍从 head+1 一起退（实测 seed5：jal 后面那条 lui 提交了）。
        head_next_ok  = head_ok && !mret_now && !exc_now &&
                    !(flush_boundary_valid && (head_pointer == flush_boundary_index)) &&
                    !entry_mret[head_next_pointer] &&
//★ 2026-11-13（fence）：栅栏一拍一道。同拍退第二道会把 fence.i 的作废/改向相位搞乱，
//  代价只是多一拍（栅栏本来就极少）。
                    !entry_fence[head_next_pointer] && !entry_fence_i[head_next_pointer] &&
                    !entry_cache_flush[head_next_pointer] &&
                    (entry_count >= 5'd2) && entry_valid[head_next_pointer] && entry_written[head_next_pointer] &&
                    (entry_judged[head_next_pointer] || (!entry_jump[head_next_pointer] && !entry_system_rd_head_next_pointer)) &&
                    !entry_exception[head_next_pointer] &&
                    !(exc0_en  && (exc0_index  == head_next_pointer))  &&
                    !(exc1_en  && (exc1_index  == head_next_pointer))  &&
                    !(exception_selected_enable && (exception_selected_index == head_next_pointer));
//★ 退休回报必须写在 head_ok 之后：同一 always @(*) 里先读后写同一信号，@* 不保证收敛（实测 lsu 死锁）。
        ret0_v   = head_ok;
        ret0_index = head_pointer;
        ret0_generation = entry_generation[head_pointer];
        ret1_v   = head_next_ok;
        ret1_index = head_next_pointer;
        ret1_generation = entry_generation[head_next_pointer];
//---- S5 判定拍（A 域）：候选 = 这一拍判定出来、真要改向的分支项 ----
//  两条 lane 同拍判定取更老的；只认在册且世代对得上的项（迟到判定不许改向）。
//  和在册边界比一次：候选更老才受理 ⇒ 新的不许压老的、老的可以顶掉新的。
        judge_pick1  = judge1_ok & (!judge0_ok | (judge1_age < judge0_age));
        judge_index  = judge_pick1 ? branch_unit_lane1_index : bju_index;
        judge_target = judge_pick1 ? branch_unit_lane1_tgt : bju_tgt;
        judge_age    = judge_pick1 ? judge1_age : judge0_age;
        judge_valid  = (judge0_ok | judge1_ok) & !head_is_exception & !kill_any;
        judge_accept = judge_valid &
                       (!flush_boundary_valid | (judge_age < (flush_boundary_index - head_pointer)));
//★ 2026-11-13（fence.i）：队头这道栅栏够资格走 ⇒ 本拍退休，下一沿作废 icache + 把前端改向到 pc+4。
//  head_ok 里已经含了 drain_ok_registered / !kill_any / entry_written。
//  与判定拍抢同一拍时让给判定（它更急，而栅栏下一拍还稳稳在队头）。
        fence_i_now = head_ok && head_is_fence_i && !judge_accept && !mret_now && !exc_now;
//★ 2026-11-13（P4）：CSR 0x7C0 起手 —— 只在 IDLE 且排空已就绪时，去叫 L2 干活。
//  ★ 与 head_ok 无关（这时它还退不了），也【不进 kill_any】：它是"要办一件事"，
//    不是一次作废。被打断就回 IDLE，等它再走到队头重来一遍（幂等）。
        cache_flush_start = head_is_cache_flush && (cache_flush_state == CACHE_FLUSH_IDLE) &&
                            drain_ok_registered && entry_written[head_pointer] && !kill_any &&
                            !cache_flush_ready;
//★ 2026-11-13（P4）：ready 置起之后那一拍 —— 失效已经做完（上一沿），
//  这一拍【退休 + 改向 pc+4】。落点仍取队头这项的 entry_pc（= 指令地址 + 4），与 fence.i 同一口径。
//  ★ 改向是必须的，不是锦上添花：更年轻的 load 可能早就乱序执行过、把旧值读进 PRF 了，
//    只作废缓存救不回来。改向把整条年轻流冲掉重取 —— 这正是 fence.i 那条路。
        cache_flush_now = head_is_cache_flush && cache_flush_ready && !kill_any;
//B 事件：在册边界走到提交口。两条 lane 都认 —— 它从 head+1 退掉也照样发。
        boundary_now = flush_boundary_valid &
                       ((head_ok      & (head_pointer      == flush_boundary_index)) |
                        (head_next_ok & (head_next_pointer == flush_boundary_index)));
//★ 交付【不能】要求 entry_written：被 lsu 拒绝的访存非对齐故障永远等不到完成口，要 done 就死等。

//陷阱载荷取【故障项自己那一份】：队头就是程序序里最老的故障指令。
//★ 同拍命中（exc_hit_head）必须走标记口那一路 —— 侧寄存器此刻还是上一条异常留下的旧值。

//★ 边界必须在【占用窗口内】才算部分冲刷（age(flush_index) < entry_count）。只查 entry_valid 不够：
//  已退槽的 valid 可能是脏的，窗口外 age 回绕会算出怪值。窗口外一律按全冲处理。
//★ entry_destination_register != 0 与 head_ok 里的 !entry_exception 是两道独立的门。
        commit_we0_comb   = head_ok && (entry_destination_register_rd_head_pointer != 5'd0);
        commit_rd0_comb   = entry_destination_register_rd_head_pointer;
        commit_phys0_comb = entry_phys_rd_head_pointer;
        commit_we1_comb   = head_next_ok && (entry_destination_register_rd_head_next_pointer != 5'd0);
        commit_rd1_comb   = entry_destination_register_rd_head_next_pointer;
        commit_phys1_comb = entry_phys_rd_head_next_pointer;
//CSR：队头这一项是 CSR ⇒ 载荷随提交口出去。1 宽够用（同拍至多一条在飞）。
        commit_csr_comb         = head_ok && entry_csr_rd_head_pointer;
        commit_csr_payload_comb = (csr_side_valid && (csr_side_index == head_pointer)) ? csr_side_payload : 22'd0;

//★ squash_phys_mask 整段删除：rename 的回收走 flush_free_mask = ~arch_next_mask，那个口从没被读过。

//★ rat 回滚不再靠"挑最老被杀项的 old_phys"：改向一判定就冻住分配 ⇒ 提交拍整片抄回 arch_map 就够。
//  端口保留（rename 已经不接），恒 0。
        squash_arch_hit  = 32'd0;
        squash_arch_phys = 192'd0;
//  survive_hit 同理归零。
        squash_survive_hit = 32'd0;
    end

//★ 改向/陷阱事件打一拍：head 那一拍只做判定，重定向信号从触发器出去。
//★ mret 不带落点：mepc 是 csr 的架构状态，由 pc 在事件那一拍直接取。

    always @(posedge clk) begin
        if (reset_registered) begin
            redir_backend_0 <= 1'b0; kill_redirect_mret <= 1'b0; kill_redirect_target <= 32'd0;
            redir_frontend <= 1'b0; redir_cause <= REDIR_CAUSE_BRANCH;
            icache_invalidate <= 1'b0;
            trap_cause <= 4'd0; trap_pc <= 32'd0; trap_value <= 32'd0;
        end
        else begin
//A 域（判定拍 / mret / 陷阱）：改 PC + 清前端。不门 redir_backend_0（judge_accept 已吃过 kill_any，再门会吞掉刚受理的边界）。
//★ 2026-11-13（fence.i）：与 mret 同构 —— A 域改 PC（落点 = 本项 pc+4）、B 域清整表，
//  ★ 额外一根 icache_invalidate 【同一沿】：世代位翻在改向那拍，重取的第一条指令就按新世代查。
            redir_frontend <= judge_accept | mret_now | exc_now | fence_i_now | cache_flush_now;
            icache_invalidate <= fence_i_now | cache_flush_invalidate;
            if (judge_accept)  kill_redirect_target <= judge_target;
//fence.i 的落点就是顺序下一条（项里存的是"指令地址 + 4"，与 mepc 同一个口径）
            if (fence_i_now)   kill_redirect_target <= entry_pc_rd_head_pointer;
//CSR 0x7C0 也一样：落在本项 pc+4，把更年轻的指令全部冲掉重取（它们可能读过旧数据）
            if (cache_flush_now) kill_redirect_target <= entry_pc_rd_head_pointer;
//改向原因跟事件同沿寄存（pc 按它选落点，本模块按它选"陷阱要不要跳过队头"）
            if (exc_now)             redir_cause <= REDIR_CAUSE_TRAP;
            else if (mret_now)       redir_cause <= REDIR_CAUSE_MRET;
            else if (judge_accept | boundary_now | fence_i_now | cache_flush_now) redir_cause <= REDIR_CAUSE_BRANCH;
            if (!redir_backend_0) begin
//B 域（提交口）：在册边界走到队列头 ⇒ 整表 + 后端一起作废。
//★ mret 的 B 跟 A 同拍；陷阱也同构 —— 清表和改 PC 不许错开。
                redir_backend_0   <= boundary_now | mret_now | exc_now | fence_i_now | cache_flush_now;
                kill_redirect_mret    <= mret_now;
                if (exc_now) begin
                    trap_cause <= entry_exc [head_pointer];
                    trap_pc    <= entry_pc_rd_head_pointer;
                    trap_value  <= entry_target_of_head;
                end
            end
            else begin
                redir_backend_0   <= 1'b0;
                kill_redirect_mret    <= 1'b0;
            end
        end
    end

//影子那根：上面那一笔打一拍。给 isq / idu 这些晚一拍才清的模块，以及自己的 redirect_stop_alloc。
    always @(posedge clk) begin
        if (reset_registered) redir_backend_1 <= 1'b0;
        else                  redir_backend_1 <= (redir_backend_0 | irq_kill);
    end

//在册边界：判定拍受理时锁下，提交口（以及异常/中断交付）那一拍撤掉

    always @(posedge clk) begin
        if (reset_registered) flush_boundary_valid <= 1'b0;
        else if ((redir_backend_0 | irq_kill)) flush_boundary_valid <= 1'b0;
        else if (judge_accept) begin
            flush_boundary_valid <= 1'b1;
            flush_boundary_index <= judge_index;
        end
    end

//★ 这两个都打一拍给 isq：判定用【当拍】的 ROB 状态算，算完才寄存 —— 那几拍前端本来就不送新指令。
//  为什么寄存：redir_backend_0 在 isq 里扇出 ~128，和 flush_partial 一起构成跨三模块的长链。
//年龄基准位向量：第 i 位 = 槽 i 在队头之上（含队头），cont 的异常仲裁用它做环上年龄比较。
    integer ka;
    always @(*) begin
        slot_above_head    = {DEPTH{1'b0}};
        for (ka = 0; ka < DEPTH; ka = ka + 1) begin
            slot_above_head[ka]    = (ka[4:0] >= head_pointer);
        end
    end

//击杀：改向只在提交口发一次 ⇒ 每次冲刷都是整窗的；消费端一律判"这一拍有没有冲刷"。
//★ S5：判定拍发 A 之后分配就冻住，解冻在提交口那一拍之后（那拍本来就被 后端击杀 挡着）。
    assign redirect_stop_alloc = flush_boundary_valid | redir_backend_1;

//---- 中断返回点：队头的指令地址（组合读）----
//★ 逐项列成 case，不写 entry_pc_rd_head_pointer：数组元素在 always @(*) 里进不了敏感表。
//★ 组合而非寄存：队头同拍可能刚退休，寄存版会把已退休那条当成返回点 ⇒ 执行两次。
    always @(*) begin
        head_pc_out = entry_pc_rd_head_pointer;
        head_valid_out = entry_valid[head_pointer];
    end

//★ S5：改向在册（寄存一拍）给 isq 挡新的访存发射，就是"边界在册"本身 + 事件那片 dly。
//  挡"新发"就够：已在 s2/s3 的会在提交口那一拍被击杀掩码杀干净。
    always @(posedge clk) begin
        if (reset_registered) redirect_any <= 1'b0;
        else                  redirect_any <= flush_boundary_valid | redir_backend_0;
    end

//★ 2026-11-13（fence）：排空判据就地打一拍。它只喂提交口的 head_ok / fence_i_now，
//  打进来的目的是断环（SQ 的 _ret 里含 ret0_valid，而 ret0_valid 就是 head_ok）。
//  代价 = 每道栅栏多等一拍；常规 load/store 一根线都不碰。
    always @(posedge clk) begin
        if (reset_registered) drain_ok_registered <= 1'b0;
        else                  drain_ok_registered <= drain_ok;
    end

//★ 2026-11-13（P4）：CSR 0x7C0 的两拍状态机（软件的一致性原语，给 DMA 用）。
//   IDLE  ──队头是它、已排空──→ 举 cache_flush_maint（电平）→ MAINT
//   MAINT ──maint_done───────→ dcache / icache 失效脉冲 + 置 cache_flush_ready → IDLE
//   · maint 用【电平】不用脉冲：l2cache 只在 S_IDLE 看它，回填期间的单拍会被整个丢掉，
//     那样 maint_done 永远不来 ⇒ 队头卡死。
//   · 被打断（改向 / 中断）一律回 IDLE：作废是幂等的，它下次走到队头再做一遍即可。
//   · 退休与失效同拍（ready 与两个脉冲是同一个沿置起的），所以"失效完成之后才退休"成立。
    always @(posedge clk) begin
        if (reset_registered) begin
            cache_flush_state      <= CACHE_FLUSH_IDLE;
            cache_flush_ready      <= 1'b0;
            cache_flush_maint      <= 1'b0;
            cache_flush_invalidate <= 1'b0;
            dcache_invalidate      <= 1'b0;
        end
        else begin
            cache_flush_invalidate <= 1'b0;
            dcache_invalidate      <= 1'b0;
            if (redir_backend_0 | irq_kill) begin
                cache_flush_state <= CACHE_FLUSH_IDLE;
                cache_flush_ready <= 1'b0;
                cache_flush_maint <= 1'b0;
            end
            else if (cache_flush_start) begin
                cache_flush_maint <= 1'b1;
                cache_flush_state <= CACHE_FLUSH_MAINT;
            end
            else if ((cache_flush_state == CACHE_FLUSH_MAINT) && maint_done) begin
                cache_flush_maint      <= 1'b0;
                cache_flush_invalidate <= 1'b1;
                dcache_invalidate      <= 1'b1;
                cache_flush_ready      <= 1'b1;
                cache_flush_state      <= CACHE_FLUSH_IDLE;
            end
//退了就把"可以退"撤掉（下一道同类的项要重新走一遍）
            if (head_ok && head_is_cache_flush)
                cache_flush_ready <= 1'b0;
        end
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
// P1: 前进格数算一次存进 retire_step，csr 的 minstret 直接吃它。
    always @(*) begin
        if (head_next_ok)      retire_step_next = 2'd2;
        else if (head_ok)  retire_step_next = 2'd1;
        else               retire_step_next = 2'd0;
//陷阱交付：队头那条自己算走（tail 也指向 head+1），所以这条分支必须盖过 kill_any 的 0。
        if (redir_backend_0 && (redir_cause == REDIR_CAUSE_TRAP)) retire_step_next = 2'd1;
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

//提交口打一拍：把"要不要提交 / 哪个架构号 / 对应哪个物理号"在提交拍算好、寄存，下一拍交给 rename。
//★ 为什么打：arch_map 是变址写阵列，综合器表达不出 CE，head_ok 那条锥会整条压在 D 引脚上。
//★ 绝不能在陷阱交付 / 冲刷拍清：冲刷只作废比边界年轻的项，而打拍里那笔在边界之前 ⇒ 清了丢一笔合法写。
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

//★ 恢复脉冲已删（2026-11-05，2026-11-11 连出口一起清掉）：rename / prf 的回滚由冲刷那一拍自己做。

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

//---- 项状态阵列（唯一被三区共写的对象，只能一个 always 块）----
    always @(posedge clk) begin
        if (reset_registered) begin
            for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                entry_valid[ri]  <= 1'b0;
                entry_written[ri] <= 1'b0;
                entry_exception[ri] <= 1'b0;
                entry_exc[ri] <= 4'd0;
                entry_judged[ri]  <= 1'b0;
                entry_mret[ri]  <= 1'b0;
                entry_fence[ri]     <= 1'b0;
                entry_fence_i[ri]   <= 1'b0;
                entry_cache_flush[ri] <= 1'b0;
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
//★ 不写 rd 的项由 issue_enable_0 那一拍置 wr：发射级是 1 深 Q 之后，"执行一定早于退役"不再成立。
                entry_written[tail_pointer] <= 1'b0;
                entry_exception[tail_pointer] <= 1'b0;
                entry_judged[tail_pointer]  <= 1'b0;
//★ mret 标签【随分配落地】：和 entry_jump/entry_system 同源，都是 idu 的译码位，不靠边带脉冲补写。
//  原来那条路（cont 的 mret_mark/mret_index）实测会被同窗口别人的异常标志压掉（seed36：标签丢了，
//  那条 mret 当普通指令退休、PC 顺延到下一条，整段 body 重跑）。
                entry_mret[tail_pointer]  <= allocation_mret_0;
                entry_fence[tail_pointer]   <= allocation_fence_0;
                entry_fence_i[tail_pointer] <= allocation_fence_i_0;
                entry_cache_flush[tail_pointer] <= allocation_cache_flush_0;
            end
            if (alloc_take_1) begin
                entry_valid[allocation_index_1]  <= 1'b1;
                entry_written[allocation_index_1] <= 1'b0;
                entry_exception[allocation_index_1] <= 1'b0;
                entry_judged[allocation_index_1]  <= 1'b0;
                entry_mret[allocation_index_1]  <= allocation_mret_1;
                entry_fence[allocation_index_1]   <= allocation_fence_1;
                entry_fence_i[allocation_index_1] <= allocation_fence_i_1;
                entry_cache_flush[allocation_index_1] <= allocation_cache_flush_1;
            end
            if (alloc_take_2) begin
                entry_valid[allocation_index_2]  <= 1'b1;
                entry_written[allocation_index_2] <= 1'b0;
                entry_exception[allocation_index_2] <= 1'b0;
                entry_judged[allocation_index_2]  <= 1'b0;
                entry_mret[allocation_index_2]  <= allocation_mret_2;
                entry_fence[allocation_index_2]   <= allocation_fence_2;
                entry_fence_i[allocation_index_2] <= allocation_fence_i_2;
                entry_cache_flush[allocation_index_2] <= allocation_cache_flush_2;
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
//两条 lane 的判定口各一套：同拍两条都判定时一条都不能丢（丢了 entry_judged 永远不置 ⇒ 队头卡死）。
//★ entry_judged 对所有项都置，但只有跳转类与 SYSTEM 类在退休判据里要求它 —— 这两类的改向/异常
//  标记正是那一级出的（实测 WEIRD w68）；其余项不许被它压住（store 一压，紧跟的 load 会读旧值）。
            if (bju_judged && entry_valid[bju_index] && (entry_generation[bju_index] == bju_generation)) begin
                entry_judged[bju_index] <= 1'b1;
//★ S5：判定拍只写"落点"；"这条要改向"不按项记，边界只在 flush_boundary_* 一份里。
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
//store：进 SQ 那一拍回报"已落地"，ROB 这时才置可退。
//★ 没有这个完成口，store 会在还没进 LSU 时就退休 ⇒ SQ 项永远等不到排空资格（实测死锁）。
            if (acc_st_o) begin
                entry_written[st_index] <= 1'b1;
            end
//★ mret 必须是一发：不清的话事件还拉 ⇒ 冲刷永远不退、队头永远退不掉。
//★ 清在【它自己那一槽】上：吃寄存版的事件信号会清到邻居的槽，撞上一条真的 mret 就把改向吞掉。
            if (mret_now) entry_mret [head_pointer] <= 1'b0;
//★ 标记陷阱时必须【同时置 wr】：故障指令不会再有单元回报它，只置 exc 不置 wr 就退不掉 ⇒ 整核挂死。
//★ S4：标记即写项，必须排在所有 acc_*/判定口之后（同一槽同一沿，后写者赢）。
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
//★ 冲刷 = 整窗作废（改向 / 异常 / 中断都在提交口发，边界恒为队头）。
//  冲刷拍分配口被 后端击杀 挡住，不会误清同拍新进来的项。
            if ((redir_backend_0 | irq_kill)) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    entry_valid[ri]     <= 1'b0;
                    entry_written[ri]    <= 1'b0;
                    entry_exception[ri]    <= 1'b0;
                    entry_mret[ri]  <= 1'b0;
                    entry_fence[ri]     <= 1'b0;
                    entry_fence_i[ri]   <= 1'b0;
                    entry_cache_flush[ri] <= 1'b0;
                end
                csr_side_valid <= 1'b0;
            end
        end
    end

//---- 真 PRF 写口的物理号 / 存活判据（连续赋值，数组元素在 always @(*) 里进不了敏感表）----
    assign prf_wb_phys_0 = entry_phys_rd_prf_wb_index_0;
    assign prf_wb_phys_1 = entry_phys_rd_prf_wb_index_1;
    assign prf_ld_phys   = entry_phys_rd_prf_ld_index;
    assign prf_alu_early_phys_0 = entry_phys_rd_prf_alu_early_index_0;
    assign prf_ld_early_phys    = entry_phys_rd_prf_ld_early_index;
    assign prf_ld_early_live    = entry_valid[prf_ld_early_index] & (entry_generation[prf_ld_early_index] == prf_ld_early_generation);
    assign prf_alu_early_phys_1 = entry_phys_rd_prf_alu_early_index_1;
//★ ALU 有自己值表写口 ⇒ 也有存活判据（与 load 早口逐字同形）
    assign prf_alu_early_live_0 = entry_valid[prf_alu_early_index_0] & (entry_generation[prf_alu_early_index_0] == prf_alu_early_generation_0);
    assign prf_alu_early_live_1 = entry_valid[prf_alu_early_index_1] & (entry_generation[prf_alu_early_index_1] == prf_alu_early_generation_1);
    assign prf_div_wake_phys_0  = entry_phys_rd_prf_div_wake_index_0;
    assign prf_div_wake_phys_1  = entry_phys_rd_prf_div_wake_index_1;
    assign prf_wb_live_0 = prf_wb_en_0 & entry_valid[prf_wb_index_0] & (entry_generation[prf_wb_index_0] == prf_wb_generation_0);
    assign prf_wb_live_1 = prf_wb_en_1 & entry_valid[prf_wb_index_1] & (entry_generation[prf_wb_index_1] == prf_wb_generation_1);
    assign prf_ld_live   = entry_valid[prf_ld_index]   & (entry_generation[prf_ld_index]   == prf_ld_generation);

endmodule
