`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu2 —— 发射级：两条 lane 各一级 D→Q 直通发射
//
//   由"四类统一保留站"退化而来。站阵列 / 入站两槽 / 入队扫描 / 唤醒比较器阵列 /
//   年龄与前缀树 / 选优（sel_slot）/ 站内搬迁 / free_slot / occupancy **全部删除**：
//   顺序双发射不需要调度器，"谁可以发"就是两条 lane 各自的本地判据。
//
//   ★ 结构：每条 lane 一级 D→Q 寄存器。D = idu1 的 S 级输出，Q = 发射寄存器。
//     Q 端组合算 issue_v 与全部字段，单元当拍收；D→Q 只在 Q 空 / Q 本拍发出时发生。
//
//   ★ 操作数在 Q 端组合选出：立即数 → ALU0 前送 → ALU1 前送 → LOAD 前送 → prf。
//     就绪 = queue_immediate | prf_ready[物理号] | 三条前送任一命中。物理号在进队那一拍就定死，
//     不会因为 ROB 槽被复用而漂。
//
//   ★ 结果口的相位：生产者 T 拍发、口上 T+1 有效，prf 在同一沿写、ready 下一拍才置起
//     ⇒ 发出那一拍靠前送、之后靠 prf_ready，两段正好接上。本条自己的结果最早下一拍才回来。
//
//   ★ 前送只认已经发出去的指令的产物。now_* 是可选的"同拍 alu 结果"路径（独立数据路径），
//     不接（恒 0）功能完全正确。
//
//   ★ 冲刷：按 idx 与边界比年龄，年轻清掉；trap / 全冲两条都清。已发出的 lane 不回收
//     （它已经不在 Q 里，完成口还要回 rob）。冲刷拍不给 load，但**照样给 acc**：
//     idu1 的 S 级在冲刷后仍躺着那条错路指令，不给 credit 会把 idu1 永久按死。
//
//   ★ ROB 的 fwd 口是【拉式】：每 lane 每源一份 scoreboard {hit,slot,gen} + wait，
//     slot/gen 交给 rob 换值（qry_ok = 在册 & wr & 世代匹配）。
//     tag 的来源是 32 项生产者表 pstat（每寄存器一个"最年轻的在生产写者"）。
//////////////////////////////////////////////////////////////////////////////////

module idu2(
    input clk, rst,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input [2:0] flush_control_in,   // = flag_bus[7:5]：{异常, 中断, 改向}
    input       exec_in,            // = flag_bus[4]
//前端：本 lane 的 D 有指令、且还没被收
    input        stage_advance,
    input        stage_advance_lane1,

//---- D 级 lane0（idu1 的 S 级寄存器）----
    input [4:0]  rs1_in, rs2_in,
    input [31:0] instruction_in, pc_operand_in, aux_addr_in,
    input        branch_prediction_taken_in,
    input [3:0]  index_in,
    input        alloc_generation_in,
    input [1:0]  decoded_class,
    input        decoded_write_enable,
    input [4:0]  decoded_destination_register,
    input        decoded_is_st,
    input [2:0]  decoded_multiply_operation,
    input [2:0]  decoded_load_store_function3,
    input [3:0]  decoded_function4,
    input [2:0]  decoded_csr_function3,
    input        decoded_csr_write_enable,
    input        decoded_csr_write_commit,
    input [11:0] decoded_csr_address,
    input [31:0] decoded_jump_value,
    input        decoded_branch_flag,
    input        decoded_jump_link_flag,
    input        decoded_jump_link_register_flag,
    input [3:0]  decoded_branch_jump_function4,
    input        decoded_r1_immediate,
    input        decoded_r2_immediate,
    input [31:0] decoded_r1_immediate_value,
    input [31:0] decoded_r2_immediate_value,
//---- D 级 lane1 ----
    input [4:0]  rs1_in_lane1, rs2_in1,
    input [31:0] inst_in_lane1, pc_operand_in1, aux_addr_in1,
    input        branch_prediction_taken_in_lane1,
    input [3:0]  index_in_lane1,
    input        alloc_generation_in_lane1,
    input [1:0]  decoded_class_lane1,
    input        decoded_write_enable_lane1,
    input [4:0]  decoded_destination_register_lane1,
    input        decoded_is_st_lane1,
    input [2:0]  decoded_multiply_operation_lane1,
    input [2:0]  decoded_load_store_function3_lane1,
    input [3:0]  decoded_function4_lane1,
    input [2:0]  decoded_csr_function3_lane1,
    input        decoded_csr_write_enable_lane1,
    input        decoded_csr_write_commit_lane1,
    input [11:0] decoded_csr_address_lane1,
    input [31:0] decoded_jump_value_lane1,
    input        decoded_branch_flag_lane1,
    input        decoded_jump_link_flag_lane1,
    input        decoded_jump_link_register_flag_lane1,
    input [3:0]  decoded_branch_jump_function4_lane1,
    input        decoded_r1_immediate_lane1,
    input        decoded_r2_immediate_lane1,
    input [31:0] decoded_r1_immediate_value_lane1,
    input [31:0] decoded_r2_immediate_value_lane1,
//==============================================================================
// 访存专用流（idu1 的 LSU FIFO）
//   ★ 两个相位分开：pstat 的【登记 + 查 tag】发生在【进队那一拍】（下面带 push 的那组），
//     tag 随条目在 FIFO 里等，出队时（下面带 _in 的那组）直接拿来用。
//     道理：访存条在 FIFO 里可能躺好几拍，pstat 只保留最年轻的写者 ⇒ 等出队再查，
//     查到的是那些【比它年轻】、在它排队期间已登记的写者（实测 seed8 的地址被算成 0x468）。
//==============================================================================
//---- 出队侧：FIFO 队头 ----
    input [4:0]  mem_rs1_in, mem_rs2_in, mem_destination_register_in,
    input [31:0] mem_auxiliary_address_in, mem_immediate_in,
    input [3:0]  mem_index_in,
    input        mem_generation_in,
    input        mem_is_store_in,
    input [2:0]  mem_load_store_function3_in,
    input        mem_valid_in,
//★ 2026-11-04 真 PRF：两个源的物理号（进队那一拍就定死，随条目走）
    input [5:0]  mem_phys_a_in, mem_phys_b_in,
    output reg   mem_pop_out,

//---- 结果口（寄存值）----
//★ 2026-10-28 清理：每路只留 32 位数据 + 身份 D 端。原来还有一套"身份 Q 端"
//  （write_enable_alu / index_alu / generation_alu 共 21 个端口）从来没有被读过 ——
//  消费端的判据只用 *_early 那一组（结果口 D 端），Q 端那一组是纯冗余输入。
    input [31:0] data_alu_early,
    input [31:0] data_alu_lane1_early,
    input [31:0] data_ld_early,
//---- 前送身份（只认 ALU 与 LOAD；mdu 的结果绝不进旁路网）----
    input        write_enable_alu_early,       input [5:0] phys_alu_early_0,
    input        write_enable_alu_lane1_early, input [5:0] phys_alu_early_1,
    input        write_enable_ld_early,        input [5:0] ld_phys_early,
//---- D 级那一格的两个源物理号（rename 在 idu1 那一拍给出的）----
    input [5:0]  rs1_phys_in, rs2_phys_in, rs1_phys_in_lane1, rs2_phys_in_lane1,
//---- 真 PRF：读口地址出去，数据与就绪回来 ----
    output [5:0] prf_read_a_0_phys, prf_read_b_0_phys,
    output [5:0] prf_read_a_1_phys, prf_read_b_1_phys,
    output [5:0] prf_read_a_m_phys, prf_read_b_m_phys,
    input [31:0] prf_data_a_0, prf_data_b_0, prf_data_a_1, prf_data_b_1, prf_data_a_mem, prf_data_b_mem,
    input [63:0] prf_ready_in,


//---- 冲刷 / 陷阱 / 年龄基准 ----
//★ 冲刷记录吃 cont 的寄存版（比判定拍晚一拍，见 cont 端口注释）；flush_pending 是同一组寄存器的
//  "上一拍刚冲刷过"，用来补上这一拍的空档：错路指令还躺在 Q 里，若不管它，它会照常发射 ——
//  最坏是错路 csrrw 在发射拍就把 CSR 锁存住（影子里的写污染 mscratch）。
    input        flush_control_rob_registered,
    input        flush_pending,
    input        flush_inclusive_registered,
    input [3:0]  flush_index_registered,
    input        trap_fire_registered,
// P2/P3: no head pointer here. Qualification uses the ROB's head_one_hot_registered;
// whether a memory op issues is decided by the LSU's own send gate.
    input [15:0] head_one_hot_registered,
    input [15:0] above,
//★ 2026-11-04（CSR 步骤 C）：CSR 的写口搬到【提交拍】了 ⇒ 上一拍的写正在这一拍落地。
//  本拍要是也放一条 CSR 出去，它的组合读口读到的还是落地前的旧值（实测：csrr 读回上一条写之前的值）。
//  所以这一拍挡所有 CSR 的发射；CSR 本来就串行在队头，多等一拍对 IPC 没有可测影响。
    input        csr_write_landing,
//---- 单元就绪 ----
    input        lsu_full,
//★ 在途 load 的目的寄存器（两个槽）：判"本条的操作数是不是正被 load 写"。
//  这条判据本来就存在（lsu 入口的 load_store_use_hit），但它只在本模块入口生效 —— 发射级不知道，
//  于是提前把指令放出来：lsu 拒收、idu2 已经出队 ⇒ 指令静默蒸发，ROB 那项永远等不到完成
//  （实测 seed311 整核自锁；这是预存 bug，与保留站无关）。
    input        load_hazard_valid_0, input [4:0] load_hazard_rd_0,
    input        load_hazard_valid_1, input [4:0] load_hazard_rd_1,
//---- load 早唤醒（2026-10-30）----
//★ 载入那条在【地址进 dcache 那一拍】（load_wake_valid，已带 ~miss）就宣告"下一拍有数"。
//  消费者因此能提前一拍发射；数据下一拍由 alu.v 的 load_data_forward 顶进 ALU 输入。
//★ 它只给两条 lane 用：访存条的地址加法要的是【发射拍】的值，晚到的顶不进去。
    output reg   bypass_load_a_0, bypass_load_b_0, bypass_load_a_1, bypass_load_b_1,
//★ "已判定、还没排到队头"的改向是否在册。在册期间不许发【非队头】的访存：
//  那些条一定比改向点年轻、是错路，而错路访存可能打到没有从设备的地址 ⇒ LSU 永远等应答 ⇒ 整核自锁
//  （实测 CoreMark：错路 `lbu 1(x0)`）。
//  ★ 但【队头】必须放行：队头一定比改向点老（退休是顺序的）、是活路，而且只有它能推进队列
//  —— 连它一起挡就是死锁：队头等完成 → 不发 → 队头退不了 → 改向永远走不到队头交付（实测卡死）。
    input        mdu_mul_ok_0, mdu_div_ok_0, mdu_mul_ok_1, mdu_div_ok_1,
    input        bju_ok_0, bju_ok_1,

//---- 读口在下面 PRF 那一组：地址 = Q 里的物理号，数据与就绪组合回来 ----

//---- 出站 lane0 ----
    output reg        issue_valid_0,
    output reg [4:0]  destination_register_0,
    output reg [3:0]  issue_index_0,
    output reg        issue_generation_0,
    output reg        write_enable_0,
    output reg [1:0]  sel_class_0,
    output reg [6:0]  opcode_0,
    output reg [9:0]  function10_0, function10_load_store_0,
    output reg [3:0]  alu_function4_0,
    output reg [2:0]  csr_func3_0,
    output reg        csr_write_enable_0, csr_write_action_0,
    output reg [11:0] csr_address_0,
    output reg [31:0] auxiliary_address_0, off_mem_0, beq_off_0, jump_link_register_prediction_address_0, jump_link_target_0,
    output reg [31:0] operand_data_a_0, operand_data_b_0,
    output reg [4:0]  source_register_a_0, source_register_b_0,
    output reg        branch_flag_0, jump_link_flag_0, jump_link_register_flag_0, branch_prediction_taken_0,
    output reg [3:0]  bju_function4_0,
    output reg        mul_go_0, div_go_0,
//---- 出站 lane1 ----
    output reg        issue_valid_1,
    output reg [4:0]  destination_register_1,
    output reg [3:0]  issue_index_1,
    output reg        issue_generation_1,
    output reg        write_enable_1,
    output reg [1:0]  sel_class_1,
    output reg [6:0]  opcode_1,
    output reg [9:0]  function10_1, function10_load_store_1,
    output reg [3:0]  alu_function4_1,
    output reg [2:0]  csr_func3_1,
    output reg        csr_write_enable_1, csr_write_action_1,
    output reg [11:0] csr_address_1,
    output reg [31:0] auxiliary_address_1, off_mem_1, beq_off_1, jump_link_register_prediction_address_1, jump_link_target_1,
    output reg [31:0] operand_data_a_1, operand_data_b_1,
    output reg [4:0]  source_register_a_1, source_register_b_1,
    output reg        branch_flag_1, jump_link_flag_1, jump_link_register_flag_1, branch_prediction_taken_1,
    output reg [3:0]  bju_function4_1,
    output reg        mul_go_1, div_go_1,

//---- 共享 LSU 口（被选中 lane 的字段 + lsu_lane）----
    output reg        lsu_valid,
    output reg        lsu_lane,
    output reg [6:0]  lsu_opcode,
    output reg [9:0]  lsu_function10,
    output reg [4:0]  lsu_destination_register,
//访存地址的两个加数：基址 + 立即数，由 lsu 自己相加
    output reg [31:0] lsu_imm,
    output reg [3:0]  lsu_index,
    output reg        lsu_generation,
//访存条【自己的】pc（随 LSU FIFO 一起排队）。
//★ 2026-10-31：不能用 lane0 的 auxiliary_address 顶替 —— 访存条走的是 idu1 的专用 FIFO，
//  它与 lane0 的 aux 早就脱钩了。mt11 里故障条在 0x30，而那一刻 aux0 冻在 0x18 ⇒ mepc 记错。
    output reg [31:0] lsu_pc,
    output reg [31:0] lsu_a, lsu_b,
    output reg [4:0]  lsu_source_register_a, lsu_source_register_b,

//---- 中断边界 / 状态 / credit ----
    output reg        accept_0, accept_1,
    output reg        issue_pop_0, issue_pop_1
    );

//==============================================================================
// 零、控制位 / 复位 / 常量
//==============================================================================
    reg flush_control_exception, flush_control_interrupt, flush_control_jump, exec, flush_window;
    always @(*) begin
        flush_control_exception   = flush_control_in[2];
        flush_control_interrupt   = flush_control_in[1];
        flush_control_jump  = flush_control_in[0];
        exec            = exec_in;
        flush_window         = flush_control_exception | flush_control_interrupt | flush_control_jump;
    end

//D 侧推进门：与 idu1 的 `go = exec & ~flush_window & ~stall` 逐字同门。
//★ 少了它，idu1 被停住那一拍我们照样给 acc ⇒ idu1 把 l0_con 置起、lane_v 落下，
//  而 S 级寄存器并没有换新 ⇒ 那条指令凭空消失（静默）。
//★ pipe_go 已删（2026-10-31）：只赋值、全仓无人读，纯死逻辑。

    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;

    localparam CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;

//==============================================================================
// 一、D 级组合量：JAL 目标 / 三态合一的跳转值 / 冲刷边界
//==============================================================================
    function [31:0] immJ;
        input [31:0] instruction;
        immJ = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
    endfunction

    reg [31:0] jump_link_target_now, jump_link_target_now_1, jump_value_0, jump_value_1;
    always @(*) begin
        jump_link_target_now   = pc_operand_in  + immJ(instruction_in)  - 32'd4;
        jump_link_target_now_1 = pc_operand_in1 + immJ(inst_in_lane1) - 32'd4;
        jump_value_0 = decoded_jump_link_flag  ? jump_link_target_now   : decoded_jump_value;
        jump_value_1 = decoded_jump_link_flag_lane1 ? jump_link_target_now_1 : decoded_jump_value_lane1;
    end

// KILL-ARC / ABOVE：冲刷击杀改成查 rob 的槽位掩码，lane 对 lane 的年龄排序改成
// 查 above + 比槽号（见 rob 端口注释）。head_ptr 已经完全不进本模块：
// 幸存者重建删掉之后，它在 本模块的最后一个用途也没了。
    function older;                       // 返回 1 表示 age(x) < age(y)，即 x 更老
        input [3:0] x, y;
        reg ax, ay;
        begin
            ax = above[x];
            ay = above[y];
            older = (ax & ~ay) | (~(ax ^ ay) & (x < y));
        end
    endfunction

    reg kill_all;
//kill_pending："上一拍刚发生过整条管子作废"（冲刷 / 陷阱）。这一拍不许发射、不许消费、不许登记。
//  ★ 必须与击杀同源同时相：击杀改成晚一拍做，冻结也要跟着晚一拍 —— 否则 go 被挡而 acc 照发，
//    D 里那条会被静默吞掉（实测：CoreMark 退休 843 条后 ROB 灌满死锁）。
    reg kill_pending;
    always @(*) kill_pending = flush_pending | trap_fire_registered;
//★ 2026-10-31：rob_flush_part 恒 0（rob.v: flush_partial = 1'b0）⇒ `& ~rob_flush_part` 恒真，
//  化简成下面这一行。本设计每一次冲刷都是整表作废，不存在"只杀比边界年轻的"那一档。
    always @(*) kill_all = trap_fire_registered | flush_control_rob_registered;

//==============================================================================

//★ d_dead 是"这一条还被冲刷杀着"的标记。它必须是**定时器的形状**，不能靠认身份：
//  {idx, gen} 只有 5 位，冲刷回卷 + 反复重分配之后**会重复**（实测 seed3：槽5的 gen 转了一圈又回来，
//  新指令被当成老的那条杀掉 ⇒ 队头永不完成）。身份变化既不是"新的"的充分条件，也不是必要条件。
//  正确的生命周期只有两个事件：冲刷那一拍按【年龄】置位；这条被消费（acc）那一拍清掉。
//  acc 不看 d_dead（acc = stage_advance & 空位 & ~flush），所以置位/清零不成环。
    reg        dead_registered_0, dead_registered_1;
    reg        decode_dead_0, decode_dead_1;
    reg        decode_kill_0, decode_kill_1;
    reg        decode_dead_mem;
    always @(*) begin
        decode_dead_0 = dead_registered_0;
        decode_dead_1 = dead_registered_1;
//★ 冲刷那一拍【本拍就死】的那条：dk_v 要到下一拍才生效，可 pstat 的登记就在这一拍发生。
//  漏掉它会留下一个指向已死项的 tag —— 那一项的提交口永远不会出现（它已经被杀了），
//  消费者于是永久等待（实测 seed16：D 里 idx12 已被杀，仍登记成 x15 的生产者，Q 永久等 x15）。
//★ 年龄判据改成查 rob 送来的击杀弧掩码：`arc[idx]` = "本槽比边界年轻"。
//  掩码在 rob 里用当拍 head_ptr 算好、寄存一拍；本模块晚一拍用它是精确等价的
//  （弧是绝对槽号的集合，队头前进不改变它）。这样 head_ptr 不再扇进本模块。
//★ 2026-10-31：击杀弧与 rob_flush_part 都是【常数】（rob.v：kill_arc[ka] 恒 1、flush_partial 恒 0），
//  这条 16:1 查表在综合时早就被折掉了。这里直接在 RTL 上写死，读的人不用再追两层。
        decode_kill_0 = kill_all;
        decode_kill_1 = kill_all;
//访存那一路的"进队"事件：go 里有 ~flush_window，所以进队拍 kill_all 与冲刷位都是 0 ⇒ 这两项恒 0，
//留着是为了判据与两条 lane 逐条同形（将来若放开进队门也不会漏）。
        decode_dead_mem = 1'b0;
    end


//==============================================================================
// 三、Q：两条 lane 各一份发射寄存器
//==============================================================================
    reg        queue_valid_0, queue_valid_1;
    reg        issue_ok_0, issue_ok_1;   // 「这条本身能不能发」，不含 Q 是否为空
    reg        advance_ok_0, advance_ok_1;   // 只认操作数 + 单元，不含顺序/冲刷/杀
    reg [4:0]  queue_source_register_a_0, queue_source_register_b_0, queue_source_register_a_1, queue_source_register_b_1;
    reg [3:0]  queue_index_0, queue_index_1;
    reg        queue_generation_0, queue_generation_1;
    reg        queue_real_0, queue_real_1;
    reg [1:0]  queue_class_0, queue_class_1;
    reg        queue_write_enable_0, queue_write_enable_1;
    reg [4:0]  queue_destination_register_0, queue_destination_register_1;
    reg        queue_is_st_0, queue_is_st_1;
    reg [2:0]  queue_multiply_operation_0, queue_multiply_operation_1;
    reg [2:0]  queue_load_store_function3_0, queue_load_store_function3_1;
    reg [3:0]  queue_function4_0, queue_function4_1;
    reg [2:0]  queue_csr_function3_0, queue_csr_function3_1;
    reg        queue_csr_write_enable_0, queue_csr_write_enable_1;
    reg        queue_csr_write_write_0, queue_csr_write_write_1;
    reg [11:0] queue_csr_address_0, queue_csr_address_1;
    reg [31:0] queue_jump_value_0, queue_jump_value_1;
    reg [31:0] queue_auxiliary_0, queue_auxiliary_1;
    reg        queue_branch_flag_0, queue_branch_flag_1, queue_jump_link_flag_0, queue_jump_link_flag_1, queue_jump_link_register_flag_0, queue_jump_link_register_flag_1;
    reg  [3:0] queue_branch_jump_function4_0, queue_branch_jump_function4_1;
    reg        queue_immediate_a_0, queue_immediate_b_0, queue_immediate_a_1, queue_immediate_b_1;
    reg [31:0] queue_immediate_value_a_0, queue_immediate_value_b_0, queue_immediate_value_a_1, queue_immediate_value_b_1;
    reg        queue_branch_prediction_0, queue_branch_prediction_1;
//scoreboard：每个源只记【物理号】—— 就绪看 prf_ready，值直接读 PRF（不再有 base 快照）
    reg [5:0]  queue_phys_a_0, queue_phys_b_0, queue_phys_a_1, queue_phys_b_1;
//---- 访存专用流的 Q（字段只留访存用得着的那些）----
    reg        queue_valid_mem;
    reg [4:0]  queue_source_register_a_mem, queue_source_register_b_mem;
    reg [3:0]  queue_index_mem;
    reg        queue_generation_mem;
    reg [4:0]  queue_destination_register_mem;
    reg        queue_is_st_mem;
    reg [2:0]  queue_load_store_function3_mem;
    reg [31:0] queue_auxiliary_mem, queue_jump_value_mem;
//scoreboard：每个源只记【物理号】—— 就绪看 prf_ready，值直接读 PRF（不再有 base 快照）
    reg [5:0]  queue_phys_a_mem, queue_phys_b_mem;

//六个读口直接拿队列里的物理号（PRF 在 cpu_top 里，纯组合读）
    assign prf_read_a_0_phys = queue_phys_a_0;
    assign prf_read_b_0_phys = queue_phys_b_0;
    assign prf_read_a_1_phys = queue_phys_a_1;
    assign prf_read_b_1_phys = queue_phys_b_1;
    assign prf_read_a_m_phys = queue_phys_a_mem;
    assign prf_read_b_m_phys = queue_phys_b_mem;
    reg kill_0, kill_1, kill_mem;
    always @(*) begin
        kill_0 = queue_valid_0 & kill_all;
        kill_1 = queue_valid_1 & kill_all;
        kill_mem = queue_valid_mem & kill_all;
    end


//==============================================================================
// 四、就绪 / 取值：直接读 PRF
//   源操作数在【推入队列那一拍】就已经换成物理号（rename 给的），此后与 ROB 槽无关。
//   · 就绪 = prf_ready[物理号]：分配时清、写回时置、冲刷时回到架构映射那一组。
//   · 值   = prf[物理号]：写回那一拍就落进去了，不再有 base 快照、不再有提交广播。
//   · 前送是"值还在路上、结果口这一拍就挂在总线上"的那一档，按【物理号】比。
//==============================================================================
    reg        operand_ready_a_0, operand_ready_b_0, operand_ready_a_1, operand_ready_b_1;
    reg [31:0] operand_a_0, operand_b_0, operand_a_1, operand_b_1;
    reg        operand_ready_a_mem, operand_ready_b_mem;
    reg [31:0] operand_a_mem, operand_b_mem;
//---- ALU 前送（2026-10-30）----
//★ ALU 的组合锥从它自己的操作数寄存器出发之后，结果比写进 PRF 早一拍。
//  「刚算出来、还没进 PRF」的这一拍必须有人顶住，否则消费者会多等一拍 ⇒ 这就是那一档。
//★ 前送是组合的（结果口上的身份 + 结果直接出去），但它只喂操作数选择，
//  不回 ALU 自己的组合锥 —— 所以不会串成 ALU→ALU 的长锥。
    reg alu_lane0_forward_a_0, alu_lane0_forward_b_0, alu_lane1_forward_a_0, alu_lane1_forward_b_0;
    reg alu_lane0_forward_a_1, alu_lane0_forward_b_1, alu_lane1_forward_a_1, alu_lane1_forward_b_1;
//---- load 前送（2026-10-30）----
//★ 它和 load 早唤醒是【两个不同的控制来源】，必须分开看（impl 实测的结论）：
//  · load_forward_*：身份全部来自触发器（write_enable_ld_early = stage3_valid & ready_in，
//    ready_in 里的 ready_dcache 是 dcache 的 load_ready，寄存器）。只有【数据】是组合的，
//    而它的源是 dcache 的 bus_data_out，也是寄存器。⇒ 这条旁路不碰 dcache 阵列。
//  · load_wakeup_*：使能来自 load_ready_next，那是 dcache 里 tag 阵列读 + 21 位比较的组合结果。
//    把它挂进 operand_ready_* 等于让 issue_valid 等一次查表。
//  ⇒ 2026-10-30 只保留前送，早唤醒整族撤回。
    reg load_forward_a_0, load_forward_b_0, load_forward_a_1, load_forward_b_1;
    reg load_forward_a_mem, load_forward_b_mem;
    reg alu_lane0_forward_a_mem, alu_lane1_forward_a_mem;
    reg alu_lane0_forward_b_mem, alu_lane1_forward_b_mem;
//★ 前送按【物理号】比，但物理 0 是 x0：写 x0 的那条也会把结果挂到总线上（它的 rd 在 ROB 里记成 phys 0），
//  不排掉就会让所有读 x0 的消费者吃到它的值（实测：`lui x0` 的值被当成 x26 的源）。
    wire alu_fwd_ok_0 = write_enable_alu_early        & (|phys_alu_early_0);
    wire alu_fwd_ok_1 = write_enable_alu_lane1_early & (|phys_alu_early_1);
    wire ld_fwd_ok    = write_enable_ld_early        & (|ld_phys_early);

    always @(*) begin
//---- 访存专用流（第三组读口）----
//访存的两个源恒是寄存器（load/store 没有立即数源）⇒ 没有 immediate 这一档。
//★ 2026-10-31：访存条也吃 ALU 前送（加数已在 lsu.v 的 stage1_base 锁住，这一档只剩「比较 + mux」）。
//  档位顺序与 lane 逐条同形：ALU(两条) → load 前送 → PRF（load 数据到得最晚，贴着最后一级 mux）。
        load_forward_a_mem = ld_fwd_ok & (ld_phys_early == queue_phys_a_mem);
        load_forward_b_mem = ld_fwd_ok & (ld_phys_early == queue_phys_b_mem);
        alu_lane0_forward_a_mem = alu_fwd_ok_0 & (phys_alu_early_0 == queue_phys_a_mem);
        alu_lane1_forward_a_mem = alu_fwd_ok_1 & (phys_alu_early_1 == queue_phys_a_mem);
        alu_lane0_forward_b_mem = alu_fwd_ok_0 & (phys_alu_early_0 == queue_phys_b_mem);
        alu_lane1_forward_b_mem = alu_fwd_ok_1 & (phys_alu_early_1 == queue_phys_b_mem);
        operand_ready_a_mem = prf_ready_in[queue_phys_a_mem] | alu_lane0_forward_a_mem | alu_lane1_forward_a_mem | load_forward_a_mem;
        operand_ready_b_mem = prf_ready_in[queue_phys_b_mem] | alu_lane0_forward_b_mem | alu_lane1_forward_b_mem | load_forward_b_mem;
        operand_a_mem = alu_lane0_forward_a_mem ? data_alu_early :
                        alu_lane1_forward_a_mem ? data_alu_lane1_early :
                        load_forward_a_mem ? data_ld_early :
                        prf_data_a_mem;
        operand_b_mem = alu_lane0_forward_b_mem ? data_alu_early :
                        alu_lane1_forward_b_mem ? data_alu_lane1_early :
                        load_forward_b_mem ? data_ld_early :
                        prf_data_b_mem;

        load_forward_a_0 = ld_fwd_ok & (ld_phys_early == queue_phys_a_0);
        load_forward_b_0 = ld_fwd_ok & (ld_phys_early == queue_phys_b_0);
        load_forward_a_1 = ld_fwd_ok & (ld_phys_early == queue_phys_a_1);
        load_forward_b_1 = ld_fwd_ok & (ld_phys_early == queue_phys_b_1);
        alu_lane0_forward_a_0 = alu_fwd_ok_0 & (phys_alu_early_0 == queue_phys_a_0);
        alu_lane1_forward_a_0 = alu_fwd_ok_1 & (phys_alu_early_1 == queue_phys_a_0);
        alu_lane0_forward_b_0 = alu_fwd_ok_0 & (phys_alu_early_0 == queue_phys_b_0);
        alu_lane1_forward_b_0 = alu_fwd_ok_1 & (phys_alu_early_1 == queue_phys_b_0);
        alu_lane0_forward_a_1 = alu_fwd_ok_0 & (phys_alu_early_0 == queue_phys_a_1);
        alu_lane1_forward_a_1 = alu_fwd_ok_1 & (phys_alu_early_1 == queue_phys_a_1);
        alu_lane0_forward_b_1 = alu_fwd_ok_0 & (phys_alu_early_0 == queue_phys_b_1);
        alu_lane1_forward_b_1 = alu_fwd_ok_1 & (phys_alu_early_1 == queue_phys_b_1);
        operand_ready_a_0 = queue_immediate_a_0 | prf_ready_in[queue_phys_a_0] | alu_lane0_forward_a_0 | alu_lane1_forward_a_0 | load_forward_a_0;
        operand_ready_b_0 = queue_immediate_b_0 | prf_ready_in[queue_phys_b_0] | alu_lane0_forward_b_0 | alu_lane1_forward_b_0 | load_forward_b_0;
        operand_ready_a_1 = queue_immediate_a_1 | prf_ready_in[queue_phys_a_1] | alu_lane0_forward_a_1 | alu_lane1_forward_a_1 | load_forward_a_1;
        operand_ready_b_1 = queue_immediate_b_1 | prf_ready_in[queue_phys_b_1] | alu_lane0_forward_b_1 | alu_lane1_forward_b_1 | load_forward_b_1;
//★ 档位顺序：load 前送放在【倒数第二档】，紧贴 PRF。各档比较的是"这个物理号的结果在哪条线上"，
//  一个物理号只属于一条指令、结果只从一条口出来 ⇒ 各档互斥，换顺序不改行为。
//  而 load 的数据到得最晚（dcache 的 Q 端），放最后意味着它后面只剩一级 mux。
        operand_a_0 = queue_immediate_a_0 ? queue_immediate_value_a_0 :
                      alu_lane0_forward_a_0 ? data_alu_early :
                      alu_lane1_forward_a_0 ? data_alu_lane1_early :
                      load_forward_a_0 ? data_ld_early :
                      prf_data_a_0;
        operand_b_0 = queue_immediate_b_0 ? queue_immediate_value_b_0 :
                      alu_lane0_forward_b_0 ? data_alu_early :
                      alu_lane1_forward_b_0 ? data_alu_lane1_early :
                      load_forward_b_0 ? data_ld_early :
                      prf_data_b_0;
        operand_a_1 = queue_immediate_a_1 ? queue_immediate_value_a_1 :
                      alu_lane0_forward_a_1 ? data_alu_early :
                      alu_lane1_forward_a_1 ? data_alu_lane1_early :
                      load_forward_a_1 ? data_ld_early :
                      prf_data_a_1;
        operand_b_1 = queue_immediate_b_1 ? queue_immediate_value_b_1 :
                      alu_lane0_forward_b_1 ? data_alu_early :
                      alu_lane1_forward_b_1 ? data_alu_lane1_early :
                      load_forward_b_1 ? data_ld_early :
                      prf_data_b_1;
    end

//==============================================================================
// 五、发射判据（每 lane 独立）
//==============================================================================
    reg is_divide_0, is_divide_1;
    reg unit_ok_0, unit_ok_1, csr_held_0, csr_held_1;
    reg lsu_hazard_0, lsu_hazard_1, lsu_hazard_mem;
//★ 实验开关（2026-10-29）：1 = 严格按程序序发射（年轻的站项不许越过更老的站项）
//  ⇒ 这就是"纯静态（in-order）"在发射侧的等价物。默认 0 = 本核的半乱序，行为一字不改。
//  用途：量化半乱序到底在 CoreMark 上买到了多少 IPC（测量报告见 SPEC/会话记录）。
    localparam STRICT_IN_ORDER_ISSUE = 1'b0;
    reg order_ok_0, order_ok_1;
    reg mem_issue_valid, mem_load;
    reg load_0, load_1;
//---- 访存口的【程序序】仲裁 ----
//★ lsu 只有一条口，它会按【进它的顺序】去读写内存 ⇒ 访存类必须按 ROB 年龄排队。
//  两条 lane 独立发射时这个顺序会被打破：实测 seed16 的尾声，`sw x21,76(x1)`（0x80024C）
//  比 tohost 那条 store 还晚一拍落到 dtcm，签名被提前读走（tb 一看到 dtcm[0]!=0 就收）。
//  判据：本 lane 这条访存是否【最老的未发射访存条】。候选 = 两条 Q + 两条 D
//  （D 是 idu1 输出级，还没进 Q，但它更老时也要挡住年轻的先走）。
//  不会死锁：数据相关一律指向更老的指令，所以"最老的那条"永远等得到它要的东西。
//★ 2026-10-29：访存的【程序序仲裁】整段删除。
//  原来访存要从两条 lane 抢一个 LSU 口，所以必须按 ROB 年龄判"我是不是最老的未发射访存条"
//  （6 处 older()：16 位弧掩码选择 + 比较，全串在发射锥上）。
//  现在访存走自己的队列：iffu 队头按 opcode 就直接分流进 idu1 的 LSU FIFO，出队严格先进先出，
//  「进 LSU 的顺序 = 程序序」由队列本身保证 —— lane 上已经没有访存条，比较的对象不存在了。
    always @(*) begin
        if (STRICT_IN_ORDER_ISSUE) begin
            order_ok_0 = ~(queue_valid_1 & older(queue_index_1, queue_index_0));
            order_ok_1 = ~(queue_valid_0 & older(queue_index_0, queue_index_1));
        end
        else begin
            order_ok_0 = 1'b1;
            order_ok_1 = 1'b1;
        end
        is_divide_0 = queue_multiply_operation_0[2];
        is_divide_1 = queue_multiply_operation_1[2];
//★ 2026-10-30：BJU 的【判定口互斥】已删（原 lane0_wants_bju）。
//  它当初防守的是"两条 BJU 同拍判定 ⇒ cont 的判定回报口一拍只报一条，年轻那条的改向被吞"。
//  那条前提现在不成立了：改向早已搬到 ROB 提交口（cont 里 flush_control_jump = redirect_in），
//  而 ROB 本来就是两条独立的判定口（bju_judged / branch_unit_lane1_judged）。
//  cont 里那套"两条 lane 的 bju 事件仲裁"（select_branch_event）的五个输出
//  —— branch_unit_pc_in / branch_unit_target_in / jump_link_register_failure /
//  exception_target_misalign / branch_unit_index_in —— 现在没有任何读者，已一并删除。
//  ⇒ 这条互斥只是白挡 lane1 的 BJU，不再防守任何东西。
//★ CSR 写【不可撤回】：它在发射那一拍就把值锁进 csr 寄存器组（csr.v 只挡"同拍被冲刷"），
//  错路执行会永久污染。⇒ 真写 CSR 的指令必须等到【自己是 ROB 队头】才许发：
//  那时比它老的全退完了，它一定在正确路径上；发射是顺序的，它一停后面也就跟着停，不会乱序。
//  实测 WEIRD seed36：0xcc8 的 `bne` 预测不跳 ⇒ 影子里的 `csrrwi x2,mscratch,24` 先发了、
//  把 mscratch 写成 24；随后 bne 判"跳"的冲刷只能撤销它的 rd 写，撤销不了 CSR 写 ⇒
//  重取后再读 mscratch 得到 24（应为旧值 0），逐条写流在第 24 笔分岔。
//★ 在途 load 的 rd 命中本条的操作数 ⇒ 这一拍不许发（lsu 入口会拒收，而 idu2 已经出队 ⇒ 指令蒸发）。
//  比较对象是【站项自己的源寄存器】，不经过 r1_post/r2_post ⇒ 不成组合环。
        lsu_hazard_0 = (load_hazard_valid_0 & ((load_hazard_rd_0 == queue_source_register_a_0) | (load_hazard_rd_0 == queue_source_register_b_0)))
                     | (load_hazard_valid_1 & ((load_hazard_rd_1 == queue_source_register_a_0) | (load_hazard_rd_1 == queue_source_register_b_0)));
        lsu_hazard_1 = (load_hazard_valid_0 & ((load_hazard_rd_0 == queue_source_register_a_1) | (load_hazard_rd_0 == queue_source_register_b_1)))
                     | (load_hazard_valid_1 & ((load_hazard_rd_1 == queue_source_register_a_1) | (load_hazard_rd_1 == queue_source_register_b_1)));
//★ 2026-10-29：门从"CSR 写"扩大到"CSR 访问"（读也算）。
//  原来只挡写：写要等到自己是 ROB 队头（不可撤回），可**读没人挡** —— 一条更年轻的 `csrr`
//  可以抢在更老的 `csrrw` 前面发，读到写之前的旧值。两条 lane 独立发射时这不是理论问题：
//  实测 arch 套件 andi-01 的复位 trampoline，lane0 的 `csrr t3,mtvec`(0x21b4) 在 141 拍发、
//  lane1 的 `csrrw t2,mtvec,t1`(0x21ac) 到 143 拍才发 ⇒ t3 读到 mtvec=0，`bne t3,t1` 判成跳，
//  整条测试拐进 init_tramp 错路，后面每一条都挂死（本树 49/63 超时，基线 19/63）。
//  CSR 指令本来就少，加到队头才发不影响 IPC；同拍只可能有一条到队头 ⇒ CSR 之间天然有序。
        csr_held_0 = (queue_csr_address_0 != 12'd0) & (queue_csr_function3_0 != 3'd0) & (~head_one_hot_registered[queue_index_0] | csr_write_landing);
        csr_held_1 = (queue_csr_address_1 != 12'd0) & (queue_csr_function3_1 != 3'd0) & (~head_one_hot_registered[queue_index_1] | csr_write_landing);
//---- 访存专用流：唯一的入口门 ----
//  它与 lsu.v 的 new_in_pre 逐项同门（~lsu_full = stage_free、~load_store_use_hit = ~lsu_hazard_mem、
//  ~flush_window），少了任何一项就会"这边弹出、那边没收" ⇒ 载荷静默蒸发。
        lsu_hazard_mem = (load_hazard_valid_0 & ((load_hazard_rd_0 == queue_source_register_a_mem) | (load_hazard_rd_0 == queue_source_register_b_mem)))
                       | (load_hazard_valid_1 & ((load_hazard_rd_1 == queue_source_register_a_mem) | (load_hazard_rd_1 == queue_source_register_b_mem)));
        mem_issue_valid = queue_valid_mem & ~flush_window & ~kill_pending & ~kill_mem
                        & operand_ready_a_mem & operand_ready_b_mem
                        & ~lsu_full & ~lsu_hazard_mem;
//★ 出队 = D 这一格被消费（Q 空着就只是"装进 Q"，Q 有货就要求真发出去）。与两条 lane 的 accept 同形。
        mem_pop_out = mem_valid_in & (~queue_valid_mem | mem_issue_valid) & ~flush_window & ~kill_pending;
        mem_load = mem_pop_out & ~decode_dead_mem;
        unit_ok_0 = 1'b1;
        case (queue_class_0)
            CLS_MUL: unit_ok_0 = is_divide_0 ? mdu_div_ok_0 : mdu_mul_ok_0;
//★ CLS_LSU 那一支已删（2026-10-31）：访存条在 iffu 队头就分流进 LSU FIFO，结构上进不了 lane。
//★ CLS_BJU 那一支已删（2026-10-31）：cpu_top 把 bju_ok_0 接成常数 1'b1，等价于 default。
            default: unit_ok_0 = 1'b1;
        endcase
//★ 2026-10-31：`~csr_held_0` 从 unit_ok 里摘出来 —— 它不是"单元收不收"，
//  是"CSR 必须按顺序发"的纪律，属于发射判据，不属于推进判据（见下面的 advance_ok/issue_ok）。
//lane1：LSU 口让 lane0 先（资源互斥，访存已不走 lane）。BJU 不再互斥：每 lane 一套 bju，
//判定回报各走各的 ROB 口，改向在 ROB 提交口生效 —— 同拍两条 BJU 没有任何共享口。
//MUL 口按两条独立流水各算各的，不互斥。
        unit_ok_1 = 1'b1;
        case (queue_class_1)
            CLS_MUL: unit_ok_1 = is_divide_1 ? mdu_div_ok_1 : mdu_mul_ok_1;

            default: unit_ok_1 = 1'b1;
        endcase
//同上：lane1 的 csr_held 也不进 unit_ok。
//★ 2026-10-31：把「这条本身能不能发」从「Q 里有没有货」剥出来（issue_ok_*）。
//  用途只有一个：`accept = 空位 | 能发`，而 `~queue_valid | (queue_valid & X) == ~queue_valid | X`
//  ⇒ accept 不必再经 queue_valid 那一层 AND，也不必再把 flush/kill_pending 与一遍。
//★★ 2026-10-31：把三个判据彻底分开，各归各位。
//  ① advance_ok = 【能不能往下推】—— 只认"操作数到没到"和"单元收不收"（实际只有 mdu 会忙）。
//     没有别的模块需要背压：alu 是组合的、bju 不拒收、访存不走 lane。冲刷/杀都不该让上游停手 ——
//     被杀的条目本来就该消失，让上游停一拍只是白等。
//  ② issue_ok   = 【这条能不能真发出去】—— 在 ① 之上再加顺序纪律（跨 lane 年龄 + CSR 必须队头）
//     和杀/冲刷门。错路的 CSR/MRET 一旦发出去是有副作用的，这一层绝不能松。
//  ③ 落盘门（load_*）= 推进了但【不一定要收】：冲刷/杀的那条装进来也没用，直接丢掉。
//     ★ 这一点与 idu1 的 lane0_store/lsu_push 完全同形（判据在推进上、kill 在落盘上）。
        advance_ok_0 = operand_ready_a_0 & operand_ready_b_0 & unit_ok_0;
        advance_ok_1 = operand_ready_a_1 & operand_ready_b_1 & unit_ok_1;
        issue_ok_0 = advance_ok_0 & ~kill_0 & order_ok_0 & ~csr_held_0;
        issue_ok_1 = advance_ok_1 & ~kill_1 & order_ok_1 & ~csr_held_1;
        issue_valid_0 = queue_valid_0 & ~flush_window & ~kill_pending & issue_ok_0;
        issue_valid_1 = queue_valid_1 & ~flush_window & ~kill_pending & issue_ok_1;
//★ S2：同拍"BJU + store"仲裁已删。它当初防守的是 store 在【发射拍写穿内存】——
//  错路 store 与更老分支的冲刷同拍落地，lsu 出口按年龄作废晚一拍（实测 CoreMark list/state CRC 错）。
//  现在 store 只在【退休拍】才写（lsu 的 SQ 排空），被冲刷的 store 永远退不了 ⇒ 永远不会写，
//  这条仲裁失去防守对象；留着反而把 BJU 的年龄比较挂在发射锥上。
//D→Q：Q 空 / Q 本拍发出才装；冲刷拍一律不装。
//★ 错路 D（d_dead）照样给 acc：不给 credit 就把 idu1 永久按死；
//  它被消费掉但不进 Q（load 带 ~d_dead）。
//★★ 2026-10-31：**accept 必须用 issue_ok**，不能用 advance_ok。
//  原因不是策略，是结构：Q 只有 1 格，`load_*` 会直接覆盖 Q 里那条 ⇒
//  "D 能被消费" 严格等价于 "Q 里那条这一拍一定走掉"。用 advance_ok 会在
//  order_ok/csr_held/kill 挡住发射的那几拍把还没发的 Q 条目覆盖掉（实测 mt 5/15）。
//  ⇒ 要让"推进只认操作数"成立，前提是 Q 至少 2 深（或者加一格交接寄存器）。
//★ 2026-10-31：`stage_advance` 从 accept 挪到 load —— accept 只表达"Q 这拍会空出来"，
//  "D 里到底有没有东西"是落盘的事（idu1 的 lane0_free 里 `~payload_valid_out_0` 已经覆盖）。
        accept_0 = ~queue_valid_0 | issue_ok_0;
        accept_1 = ~queue_valid_1 | issue_ok_1;
//★ 落盘门只认【进来的这一条】自己死没死（decode_dead_* = dead_registered_*，冲刷那一沿
//  由 decode_kill_* 置位）。accept 已经不带 flush/kill_pending 了，错路 D 就靠这一项丢掉拿。
        load_0 = stage_advance       & accept_0 & ~decode_dead_0;
        load_1 = stage_advance_lane1 & accept_1 & ~decode_dead_1;
    end

//==============================================================================
// 六、出站字段（无论哪一类都驱动；消费端只在自己那一类里看）
//==============================================================================
    always @(*) begin
        sel_class_0 = queue_class_0;
        destination_register_0        = queue_destination_register_0;
        issue_index_0 = queue_index_0;
        issue_generation_0 = queue_generation_0;
//★ we 必须被 issue_v 门控：alu 的"登记判据"就是 we_in（它没有单独的 valid 输入），
//  Q 空着或没发时把 q_we 直接送出去，alu 会把它当成一条真指令、把上一笔的操作数再寄存一次
//  ⇒ 结果口冒出一笔假的完成，ROB 被打进错误的槽，唤醒阵列还会拿它去命中别人的 tag
//  （实测：lane0 在真生产者还没算完时就"醒"了，RAW 链整条算错）。
        write_enable_0        = issue_valid_0 & queue_write_enable_0;
        alu_function4_0 = queue_function4_0;
        csr_func3_0 = queue_csr_function3_0;
        csr_write_enable_0 = issue_valid_0 & queue_csr_write_enable_0;
        csr_write_action_0= issue_valid_0 & queue_csr_write_write_0;
//★ 2026-11-04：地址不再乘 issue_valid。它是 csr.v 那个 12:1 case 的输入，乘进发射判据
//  等于把 issue_valid 那条扇出 300+ 的网拖进 CSR 读写两侧的锥里。
//  非 CSR 指令拿到的是"上一条 CSR 指令的地址"，读值没人用（写口由 csr_write_action 门控）。
        csr_address_0  = queue_csr_address_0;
        auxiliary_address_0  = queue_auxiliary_0;
        off_mem_0   = queue_jump_value_0;
        beq_off_0   = queue_branch_flag_0   ? queue_jump_value_0 : 32'd0;
        jump_link_register_prediction_address_0 = queue_jump_link_register_flag_0 ? queue_jump_value_0 : 32'd0;
        jump_link_target_0     = queue_jump_link_flag_0  ? queue_jump_value_0 : 32'd0;
        operand_data_a_0 = operand_a_0;
        operand_data_b_0 = operand_b_0;
        source_register_a_0      = queue_source_register_a_0;
        source_register_b_0      = queue_source_register_b_0;
        branch_flag_0   = queue_branch_flag_0;
        jump_link_flag_0  = queue_jump_link_flag_0;
        jump_link_register_flag_0 = queue_jump_link_register_flag_0;
        branch_prediction_taken_0 = queue_branch_prediction_0;
        bju_function4_0         = queue_branch_jump_function4_0;
        bypass_load_a_0 = 1'b0;
        bypass_load_b_0 = 1'b0;
        mul_go_0 = issue_valid_0 & (queue_class_0 == CLS_MUL) & ~is_divide_0;
        div_go_0 = issue_valid_0 & (queue_class_0 == CLS_MUL) &  is_divide_0;
        opcode_0 = 7'd0;
        function10_0 = 10'd0;
        function10_load_store_0 = 10'd0;
        case (queue_class_0)
            CLS_LSU: begin
                opcode_0      = queue_is_st_0 ? OPCODE_STORE : OPCODE_LOAD;
                function10_load_store_0  = {7'd0, queue_load_store_function3_0};
            end
            CLS_MUL: begin
                opcode_0      = OPCODE_OP;
                function10_0     = {7'b0000001, queue_multiply_operation_0};
            end
            default: ;
        endcase

        sel_class_1 = queue_class_1;
        destination_register_1        = queue_destination_register_1;
        issue_index_1 = queue_index_1;
        issue_generation_1 = queue_generation_1;
        write_enable_1        = issue_valid_1 & queue_write_enable_1;
        alu_function4_1 = queue_function4_1;
        csr_func3_1 = queue_csr_function3_1;
        csr_write_enable_1 = issue_valid_1 & queue_csr_write_enable_1;
        csr_write_action_1= issue_valid_1 & queue_csr_write_write_1;
        csr_address_1  = queue_csr_address_1;   // 同 lane0：不乘 issue_valid，理由见上
        auxiliary_address_1  = queue_auxiliary_1;
        off_mem_1   = queue_jump_value_1;
        beq_off_1   = queue_branch_flag_1   ? queue_jump_value_1 : 32'd0;
        jump_link_register_prediction_address_1 = queue_jump_link_register_flag_1 ? queue_jump_value_1 : 32'd0;
        jump_link_target_1     = queue_jump_link_flag_1  ? queue_jump_value_1 : 32'd0;
        operand_data_a_1 = operand_a_1;
        operand_data_b_1 = operand_b_1;
        source_register_a_1      = queue_source_register_a_1;
        source_register_b_1      = queue_source_register_b_1;
        branch_flag_1   = queue_branch_flag_1;
        jump_link_flag_1  = queue_jump_link_flag_1;
        jump_link_register_flag_1 = queue_jump_link_register_flag_1;
        branch_prediction_taken_1 = queue_branch_prediction_1;
        bju_function4_1         = queue_branch_jump_function4_1;
        bypass_load_a_1 = 1'b0;
        bypass_load_b_1 = 1'b0;
        mul_go_1 = issue_valid_1 & (queue_class_1 == CLS_MUL) & ~is_divide_1;
        div_go_1 = issue_valid_1 & (queue_class_1 == CLS_MUL) &  is_divide_1;
        opcode_1 = 7'd0;
        function10_1 = 10'd0;
        function10_load_store_1 = 10'd0;
        case (queue_class_1)
            CLS_LSU: begin
                opcode_1      = queue_is_st_1 ? OPCODE_STORE : OPCODE_LOAD;
                function10_load_store_1  = {7'd0, queue_load_store_function3_1};
            end
            CLS_MUL: begin
                opcode_1      = OPCODE_OP;
                function10_1     = {7'b0000001, queue_multiply_operation_1};
            end
            default: ;
        endcase

//---- LSU 口：只由访存专用流驱动（2026-10-29）----
//★ 这里原来是"两条 lane 抢一个口"的仲裁：lane0 优先、lane1 不丢。改成专用流之后仲裁整个消失 ——
//  访存条在 iffu 队头就被分流进 idu1 的 LSU FIFO，出队严格先进先出，进 LSU 的顺序 = 程序序。
        lsu_a = operand_a_mem;
        lsu_imm = queue_jump_value_mem;
        lsu_b = operand_b_mem;

        lsu_valid = mem_issue_valid;
        lsu_lane = 1'b0;
        lsu_opcode = queue_is_st_mem ? OPCODE_STORE : OPCODE_LOAD;
        lsu_function10 = {7'd0, queue_load_store_function3_mem};
        lsu_destination_register = queue_destination_register_mem;
        lsu_index = queue_index_mem;
        lsu_generation = queue_generation_mem;
//★ 与 lsu_index 同源：都是随 LSU FIFO 排队的那条指令自己的字段。
//  队列里存的是 auxiliary 约定（= 指令地址 + 4），这里减回真 pc，LSU 那边直接当 pc 用。
        lsu_pc = queue_auxiliary_mem - 32'd4;
        lsu_source_register_a = queue_source_register_a_mem;
        lsu_source_register_b = queue_source_register_b_mem;
    end


//==============================================================================
// 八、bju 在途（lsu 的早一拍遮挡）
//==============================================================================
    reg       bju_issue_now;
    always @(*) begin
        bju_issue_now = (issue_valid_0 & (queue_class_0 == CLS_BJU))
                      | (issue_valid_1 & (queue_class_1 == CLS_BJU));
        issue_pop_0   = issue_valid_0;
        issue_pop_1   = issue_valid_1;
    end

//==============================================================================
// 九、寄存器堆读口
//==============================================================================
//★ 2026-10-30：rob 的【拉式值读口】整条删除。操作数选值现在只有三档
//  （立即数 / 本地值表 / queue_base），ROB 不在选值路上；那 4 组的答案
//  （query_data / query_ok / query_slot_registered / query_gen_registered）
//  在本模块从来没有被读过，请求侧（query_slot / query_generation）也只是喂给 rob 做无人消费的计算。

//==============================================================================
// 十、时序：Q / pstat / D 身份
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            queue_valid_0 <= 1'b0;
            queue_source_register_a_0 <= 5'd0;
            queue_source_register_b_0 <= 5'd0;
            queue_index_0 <= 4'd0;
            queue_generation_0 <= 1'b0;
            queue_real_0 <= 1'b0;
            queue_class_0 <= 2'd0;
            queue_write_enable_0 <= 1'b0;
            queue_destination_register_0 <= 5'd0;
            queue_is_st_0 <= 1'b0;
            queue_multiply_operation_0 <= 3'd0;
            queue_load_store_function3_0 <= 3'd0;
            queue_function4_0 <= 4'd0;
            queue_csr_function3_0 <= 3'd0;
            queue_csr_write_enable_0 <= 1'b0;
            queue_csr_write_write_0 <= 1'b0;
            queue_csr_address_0 <= 12'd0;
            queue_jump_value_0 <= 32'd0;
            queue_auxiliary_0 <= 32'd0;
            queue_branch_flag_0 <= 1'b0;
            queue_jump_link_flag_0 <= 1'b0;
            queue_jump_link_register_flag_0 <= 1'b0;
            queue_branch_jump_function4_0 <= 4'd0;
            queue_immediate_a_0 <= 1'b0;
            queue_immediate_b_0 <= 1'b0;
            queue_immediate_value_a_0 <= 32'd0;
            queue_immediate_value_b_0 <= 32'd0;
            queue_branch_prediction_0 <= 1'b0;
            queue_phys_a_0 <= 6'd0;
            queue_phys_b_0 <= 6'd0;
        end
        else if (load_0) begin
            queue_valid_0 <= 1'b1;
            queue_source_register_a_0 <= rs1_in;
            queue_source_register_b_0 <= rs2_in;
            queue_index_0 <= index_in;
            queue_generation_0 <= alloc_generation_in;
            queue_real_0 <= (aux_addr_in != 32'd0);
            queue_class_0 <= decoded_class;
            queue_write_enable_0 <= decoded_write_enable;
            queue_destination_register_0 <= decoded_destination_register;
            queue_is_st_0 <= decoded_is_st;
            queue_multiply_operation_0 <= decoded_multiply_operation;
            queue_load_store_function3_0 <= decoded_load_store_function3;
            queue_function4_0 <= decoded_function4;
            queue_csr_function3_0 <= decoded_csr_function3;
            queue_csr_write_enable_0 <= decoded_csr_write_enable;
            queue_csr_write_write_0 <= decoded_csr_write_commit;
            queue_csr_address_0 <= decoded_csr_address;
            queue_jump_value_0 <= jump_value_0;
            queue_auxiliary_0 <= aux_addr_in;
            queue_immediate_a_0 <= decoded_r1_immediate;
            queue_immediate_b_0 <= decoded_r2_immediate;
            queue_immediate_value_a_0 <= decoded_r1_immediate_value;
            queue_immediate_value_b_0 <= decoded_r2_immediate_value;
            queue_branch_flag_0 <= decoded_branch_flag;
            queue_jump_link_flag_0 <= decoded_jump_link_flag;
            queue_jump_link_register_flag_0 <= decoded_jump_link_register_flag;
            queue_branch_jump_function4_0 <= decoded_branch_jump_function4;
            queue_branch_prediction_0 <= branch_prediction_taken_in;
            queue_phys_a_0 <= rs1_phys_in;
            queue_phys_b_0 <= rs2_phys_in;
        end
        else if (issue_valid_0 | kill_0) begin
            queue_valid_0 <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            queue_valid_1 <= 1'b0;
            queue_source_register_a_1 <= 5'd0;
            queue_source_register_b_1 <= 5'd0;
            queue_index_1 <= 4'd0;
            queue_generation_1 <= 1'b0;
            queue_real_1 <= 1'b0;
            queue_class_1 <= 2'd0;
            queue_write_enable_1 <= 1'b0;
            queue_destination_register_1 <= 5'd0;
            queue_is_st_1 <= 1'b0;
            queue_multiply_operation_1 <= 3'd0;
            queue_load_store_function3_1 <= 3'd0;
            queue_function4_1 <= 4'd0;
            queue_csr_function3_1 <= 3'd0;
            queue_csr_write_enable_1 <= 1'b0;
            queue_csr_write_write_1 <= 1'b0;
            queue_csr_address_1 <= 12'd0;
            queue_jump_value_1 <= 32'd0;
            queue_auxiliary_1 <= 32'd0;
            queue_branch_flag_1 <= 1'b0;
            queue_jump_link_flag_1 <= 1'b0;
            queue_jump_link_register_flag_1 <= 1'b0;
            queue_branch_jump_function4_1 <= 4'd0;
            queue_immediate_a_1 <= 1'b0;
            queue_immediate_b_1 <= 1'b0;
            queue_immediate_value_a_1 <= 32'd0;
            queue_immediate_value_b_1 <= 32'd0;
            queue_branch_prediction_1 <= 1'b0;
            queue_phys_a_1 <= 6'd0;
            queue_phys_b_1 <= 6'd0;
        end
        else if (load_1) begin
            queue_valid_1 <= 1'b1;
            queue_source_register_a_1 <= rs1_in_lane1;
            queue_source_register_b_1 <= rs2_in1;
            queue_index_1 <= index_in_lane1;
            queue_generation_1 <= alloc_generation_in_lane1;
            queue_real_1 <= (aux_addr_in1 != 32'd0);
            queue_class_1 <= decoded_class_lane1;
            queue_write_enable_1 <= decoded_write_enable_lane1;
            queue_destination_register_1 <= decoded_destination_register_lane1;
            queue_is_st_1 <= decoded_is_st_lane1;
            queue_multiply_operation_1 <= decoded_multiply_operation_lane1;
            queue_load_store_function3_1 <= decoded_load_store_function3_lane1;
            queue_function4_1 <= decoded_function4_lane1;
            queue_csr_function3_1 <= decoded_csr_function3_lane1;
            queue_csr_write_enable_1 <= decoded_csr_write_enable_lane1;
            queue_csr_write_write_1 <= decoded_csr_write_commit_lane1;
            queue_csr_address_1 <= decoded_csr_address_lane1;
            queue_jump_value_1 <= jump_value_1;
            queue_auxiliary_1 <= aux_addr_in1;
            queue_immediate_a_1 <= decoded_r1_immediate_lane1;
            queue_immediate_b_1 <= decoded_r2_immediate_lane1;
            queue_immediate_value_a_1 <= decoded_r1_immediate_value_lane1;
            queue_immediate_value_b_1 <= decoded_r2_immediate_value_lane1;
            queue_branch_flag_1 <= decoded_branch_flag_lane1;
            queue_jump_link_flag_1 <= decoded_jump_link_flag_lane1;
            queue_jump_link_register_flag_1 <= decoded_jump_link_register_flag_lane1;
            queue_phys_a_1 <= rs1_phys_in_lane1;
            queue_phys_b_1 <= rs2_phys_in_lane1;
            queue_branch_jump_function4_1 <= decoded_branch_jump_function4_lane1;
            queue_branch_prediction_1 <= branch_prediction_taken_in_lane1;
        end
        else if (issue_valid_1 | kill_1) begin
            queue_valid_1 <= 1'b0;
        end
    end

//==============================================================================
// 访存专用流的 Q：装 / 保持 / 清，与两条 lane 逐字同形
//★ 它没有"另一条"可以抢口，所以没有 unit_ok 里那些互斥项；入口门只有 lsu 的 stage_free
//  与"源寄存器正是在飞那笔的 rd"（lsu_hazard_mem）—— 两项都必须与 lsu 的 new_in_pre 同门，
//  少一项就是"这边弹出了、那边没收" ⇒ 指令静默蒸发。
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            queue_valid_mem <= 1'b0;
            queue_source_register_a_mem <= 5'd0;
            queue_source_register_b_mem <= 5'd0;
            queue_index_mem <= 4'd0;
            queue_generation_mem <= 1'b0;
            queue_destination_register_mem <= 5'd0;
            queue_is_st_mem <= 1'b0;
            queue_load_store_function3_mem <= 3'd0;
            queue_auxiliary_mem <= 32'd0;
            queue_jump_value_mem <= 32'd0;
            queue_phys_a_mem <= 6'd0;
            queue_phys_b_mem <= 6'd0;
        end
        else if (mem_load) begin
            queue_valid_mem <= 1'b1;
            queue_source_register_a_mem <= mem_rs1_in;
            queue_source_register_b_mem <= mem_rs2_in;
            queue_index_mem <= mem_index_in;
            queue_generation_mem <= mem_generation_in;
            queue_destination_register_mem <= mem_destination_register_in;
            queue_is_st_mem <= mem_is_store_in;
            queue_load_store_function3_mem <= mem_load_store_function3_in;
            queue_auxiliary_mem <= mem_auxiliary_address_in;
            queue_jump_value_mem <= mem_immediate_in;
//phys 随条目走过来（在 idu1 进队那一拍就定下了），直接用。
            queue_phys_a_mem <= mem_phys_a_in;
            queue_phys_b_mem <= mem_phys_b_in;
        end
        else if (mem_issue_valid | kill_mem) begin
            queue_valid_mem <= 1'b0;
        end
    end

//==============================================================================
// D 级"这条被冲刷杀着"的标记
//★ 它必须是**定时器的形状**，不能靠认身份：{idx, gen} 只有 5 位，冲刷回卷 + 反复重分配
//  之后会重复（实测 seed3：槽5的 gen 转了一圈又回来，新指令被当成老的那条杀掉 ⇒ 队头永不完成）。
//  正确的生命周期只有两个事件：冲刷那一拍按【年龄】置位；这条被消费（acc）那一拍清掉。
//  acc 不看 d_dead（acc = stage_advance & 空位 & ~flush），所以置位/清零不成环。
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            dead_registered_0 <= 1'b0;
            dead_registered_1 <= 1'b0;
        end
        else begin
            if (kill_all | flush_control_rob_registered) begin
                dead_registered_0 <= decode_kill_0;
                dead_registered_1 <= decode_kill_1;
            end
            else begin
                if (~stage_advance        | accept_0) dead_registered_0 <= 1'b0;
                if (~stage_advance_lane1 | accept_1) dead_registered_1 <= 1'b0;
            end
        end
    end

endmodule

