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
//   ★ 操作数在 Q 端组合选出，四档：立即数 → 结果口(idx+gen 命中) → rob 拉式 fwd 值 → rdata。
//     rdata 这一档是"装 Q 那一拍由上一拍的 D.rs 预取"的值；装进 Q 之后地址改跟 Q 的 rs，
//     于是它每拍被 regfile 刷新 —— 生产者在消费者等待期间提交时，只有这一档兜得住。
//
//   ★ 结果口是寄存值：生产者 T 拍发、口上 T+1 有效；rob 的 ent_wr 要到 T+2 才置起
//     ⇒ 结果口这一档比 qry_ok 早一拍，两档都要。本条自己的结果最早下一拍才回来。
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
    input [31:0] mem_rdata1_in, mem_rdata2_in,
    input [3:0]  mem_index_in,
    input        mem_generation_in,
    input        mem_is_store_in,
    input [2:0]  mem_load_store_function3_in,
    input        mem_valid_in,
    input        mem_producer_a_valid_in,
    input [3:0]  mem_producer_a_slot_in,
    input        mem_producer_a_generation_in,
    input        mem_producer_b_valid_in,
    input [3:0]  mem_producer_b_slot_in,
    input        mem_producer_b_generation_in,
    output reg   mem_pop_out,
//---- 进队侧：前端队头那两格（每拍最多两条）----
//★ 2026-10-31：pstat 的取号点从本模块的 D 阶段整体挪到【进队这一拍】。
//  读者从三个（lane0 D / lane1 D / 访存）变成两个（h0 / h1）—— 访存条并进了 h0。
//  取号之后 tag 随条目走（idu1 的载荷和 lsu fifo 条目各带一份），D 阶段不再查表。
    input        lane0_fresh_in, lane1_fresh_in,
    input        mem_push_valid,          // h0 这一条要被收（lane0 或访存，二者互斥）
    input        mem_push_is_mem,         // 上面那一条是不是访存（只用来清值表槽）
    input [4:0]  mem_push_rs1_in, mem_push_rs2_in, mem_push_destination_register_in,
    input [3:0]  mem_push_index_in,
    input        mem_push_generation_in,
    input        push1_valid_in,          // h1 这一条要被收（= alloc_lane0 & alloc_lane1）
    input [4:0]  push1_rs1_in, push1_rs2_in, push1_destination_register_in,
    input [3:0]  push1_index_in,
    input        push1_generation_in,
    output reg [5:0] mem_push_prod_a_out, mem_push_prod_b_out,
    output reg [5:0] push1_prod_a_out, push1_prod_b_out,
//---- 载荷随条目带过来的生产者号（进队那拍查好的，打包 {valid, slot[3:0], generation}）----
    input [5:0]  lane0_prod_a_in, lane0_prod_b_in,
    input [5:0]  lane1_prod_a_in, lane1_prod_b_in,

//---- 结果口（寄存值）----
//★ 2026-10-28 清理：每路只留 32 位数据 + 身份 D 端。原来还有一套"身份 Q 端"
//  （write_enable_alu / index_alu / generation_alu 共 21 个端口）从来没有被读过 ——
//  消费端的判据只用 *_early 那一组（结果口 D 端），Q 端那一组是纯冗余输入。
    input [31:0] data_alu_early,
    input [31:0] data_alu_lane1_early,
    input [31:0] data_mul_early,
    input [31:0] data_div_early,
    input [31:0] data_multiply_lane1_early,
    input [31:0] data_divide_lane1_early,
    input [31:0] data_ld_early,
//结果口的【身份 D 端】—— 早唤醒在这一组上判
    input        write_enable_alu_early,  input [3:0] index_alu_early,  input generation_alu_early,
    input        write_enable_alu_lane1_early, input [3:0] index_alu_lane1_early, input generation_alu_lane1_early,
    input        write_enable_mul_early,  input [3:0] index_mul_early,  input generation_mul_early,
    input        write_enable_multiply_lane1_early, input [3:0] index_multiply_lane1_early, input generation_multiply_lane1_early,
    input        write_enable_div_early,  input [3:0] index_div_early,  input generation_div_early,
    input        write_enable_divide_lane1_early, input [3:0] index_divide_lane1_early, input generation_divide_lane1_early,
    input        write_enable_ld_early,   input [3:0] index_ld_early,   input generation_ld_early,

//---- 提交口（清 wait / 清生产者表；必须同时比 rd 与 slot+gen）----
    input        write_enable_a, input [4:0] destination_register_a,
    input        write_enable_b, input [4:0] destination_register_b,
    input [3:0]  commit_slot0, commit_slot1,
    input        commit_gen0,  commit_gen1,
//提交口的数据：Q 在等的那条生产者若在这一拍提交，base 直接吃这个值（同拍精确旁路）
    input [31:0] commit_data0, commit_data1,

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
    input        load_wake_valid, input [3:0] load_wake_index, input load_wake_generation,
    output reg   bypass_load_a_0, bypass_load_b_0, bypass_load_a_1, bypass_load_b_1,
//★ "已判定、还没排到队头"的改向是否在册。在册期间不许发【非队头】的访存：
//  那些条一定比改向点年轻、是错路，而错路访存可能打到没有从设备的地址 ⇒ LSU 永远等应答 ⇒ 整核自锁
//  （实测 CoreMark：错路 `lbu 1(x0)`）。
//  ★ 但【队头】必须放行：队头一定比改向点老（退休是顺序的）、是活路，而且只有它能推进队列
//  —— 连它一起挡就是死锁：队头等完成 → 不发 → 队头退不了 → 改向永远走不到队头交付（实测卡死）。
    input        mdu_mul_ok_0, mdu_div_ok_0, mdu_mul_ok_1, mdu_div_ok_1,
    input        bju_ok_0, bju_ok_1,

//---- 寄存器堆读口（寄存读：地址本拍给、数据下一拍到）----
//---- 寄存器堆读值 ----
//★ 地址不在这里出：读口地址由 iffu 的出队格（F 级）直接给 regfile，寄存一拍，
//  正好与"这一条进 D 级"同拍到达 ⇒ rdata 与 D 里的指令对齐。本模块只吃不驱动。
    input [31:0] rdata1_in, rdata2_in, rdata3_in, rdata4_in,

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
// 二、生产者表 pstat：每个架构寄存器"最年轻的在生产写者"
//   它是"我要等谁"的唯一来源。按【D 级登记】而不是装 Q 时登记：D 就是程序序的流，
//   被按住的 D 会连续多拍出现，所以只认身份变化（d_new），重复登记会把更年轻的
//   写者覆盖成更老的写者（静默错值）。
//   两条 lane 同拍进 D 时 lane0 恒比 lane1 老（idu1 的装载规则保证）⇒ 登记顺序固定。
//
// ---- 命名规范（本模块生产者这一族）----
//   旧名一律废掉（`q_t_*` 的 `t` 是 tag，谁也看不出；`_v/_s/_g` 也看不出是哪个字段）。
//   现在一律写成：<所处阶段>_prod_<端口>_<lane>_<字段>
//     · 字段三个，不再用缩写：valid（有没有生产者）/ slot（ROB 槽号）/ gen（世代）
//     · 端口两个：a = rs1 侧，b = rs2 侧；lane 两个：0 / 1
//     · 阶段前缀：
//         prod_tbl_*      表本身（上一沿落表的值）
//         prod_tbl_c_*    表 + 本拍清（提交清 / 冲刷清）
//         prod_tbl_cw_*   再加本拍【较老】那条的登记     ← 年轻那条读它
//         prod_nx_*       再加本拍【较年轻】那条的登记   ← 下一沿落表的就是它
//         d_prod_*        D 级组合读出的"我的生产者"     ← 老那条读表、年轻那条读 cw
//         dt_prod_*       D 级快照（只在 d_new 那一沿锁，被按住时不许重算）
//         dq_prod_*       D → Q 那一沿要装进 Q 的那一份（新出现取 d_prod_*、否则取 dt_prod_*）
//         q_prod_*        Q 级随身携带的"我的生产者"，此后与 pstat 无关
//==============================================================================
    reg        producer_table_valid [0:31];
    reg [3:0]  producer_table_slot [0:31];
    reg        producer_table_generation [0:31];
//提交/冲刷清理后的值（= tag 的 lane0 来源）
    reg        producer_table_cleared_valid [0:31];
    reg [3:0]  producer_table_cleared_slot [0:31];
    reg        producer_table_cleared_generation [0:31];
//再叠 lane0 本拍登记（= tag 的 lane1 来源）
    reg        producer_table_with_older_valid [0:31];
    reg [3:0]  producer_table_with_older_slot [0:31];
    reg        producer_table_with_older_generation [0:31];
//下一个寄存器值（再叠 h1）
    reg        producer_next_valid   [0:31];
    reg [3:0]  producer_next_slot   [0:31];
    reg        producer_next_generation   [0:31];

//★ d_dead 是"这一条还被冲刷杀着"的标记。它必须是**定时器的形状**，不能靠认身份：
//  {idx, gen} 只有 5 位，冲刷回卷 + 反复重分配之后**会重复**（实测 seed3：槽5的 gen 转了一圈又回来，
//  新指令被当成老的那条杀掉 ⇒ 队头永不完成）。身份变化既不是"新的"的充分条件，也不是必要条件。
//  正确的生命周期只有两个事件：冲刷那一拍按【年龄】置位；这条被消费（acc）那一拍清掉。
//  acc 不看 d_dead（acc = stage_advance & 空位 & ~flush），所以置位/清零不成环。
    reg        dead_registered_0, dead_registered_1;
    reg        decode_dead_0, decode_dead_1;
    reg        decode_kill_0, decode_kill_1;
    reg        decode_dead_mem, decode_kill_mem, decode_new_mem;
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
        decode_kill_mem = kill_all;
        decode_new_mem = mem_push_is_mem;
    end
    reg        decode_seen_valid_0, decode_seen_valid_1;
    reg [3:0]  decode_seen_index_0, decode_seen_index_1;
    reg        decode_seen_generation_0, decode_seen_generation_1;
    reg        decode_new_0, decode_new_1;

    always @(*) begin
//★ d_new 不能吃冲刷门：它把 D 里这条【登记成 pstat 生产者】。冲刷那一拍进来的也可能是
//  比边界老的活指令（两条 lane 各走各的），门掉它 ⇒ 它名下的生产者不登记 ⇒ 消费者掉回 rdata
//  读到架构旧值。实测 WEIRD seed44：`addi x26,x26,0x6ac` 拿到 LUI 之前的旧 x26。
//  会死的那条照样登记没关系：它的条目由 d_kill（晚一拍）+ 冲刷重建兜底。
//★ "新出现"用 idu1 送的【装载脉冲】判，不再比 (rob 索引, 世代)（见 idu1 那段注释）。
        decode_new_0 = stage_advance & lane0_fresh_in;
        decode_new_1 = stage_advance_lane1 & lane1_fresh_in;
    end

//---- 冲刷后清表 ----
//★ 本树【没有部分冲刷】：rob 的 flush_part 恒 0，任何一次冲刷都在同一沿把整张 ROB 清空
//  （rob.v 的 `if (flush_con_rob) ... ent_v[ri] <= 1'b0`），陷阱交付拍（exc_v_dly）同样整表清。
//  ⇒ 冲刷之后不存在"比边界老、还在飞"的写者，pstat 没有幸存者要重建，整表清零即可。
//  实测（CoreMark 全跑）：reb_d2 拉高 5977 次（= 冲刷次数），每次幸存者表都全零。
//  原来的 rba_*/reb_*（16→32 重编）是死逻辑，已删 —— 它同时是 rob→idu2 那族 14 级路的起点。
//  时相不变：仍摊到冲刷后第 2 拍落表，D 里那几条此刻查到的就是"没有生产者"。
    reg        reb_d1, reb_d2;
    reg decode_write_0, decode_write_1, flush_any;
//★ 两条 lane 的 D 级【不再按 lane 分先后】：各自独立推进之后，两条 D 里躺着的身份会错开，
//  lane0 的 D 完全可能比 lane1 的 D 更年轻（lane1 那条被按住、lane0 已经换了下一条）。
//  登记顺序必须按 ROB 年龄：老的那条先写、年轻的覆盖。否则老写者会把年轻写者盖掉
//  ⇒ 后来者拿到一个比自己还年轻的生产者 tag，永远等不到值（实测 RAW 链第三拍就锁死）。
//★ 2026-10-29：D 里的参与者从 2 个变成 3 个（多了访存专用流【进队】的那一条）。
//  登记顺序仍是"先老后年轻"三档：with_older（叠了较老那条 lane）、middle（再叠较年轻那条 lane）、
//  最后才叠访存那一条。
//★ 访存那一条【恒为最年轻】，所以不参与 older() 比较 —— 这一条不能靠推：
//  它是在【本拍】从 iffu 队头译出来的，而 idu2 的 D 里那两条是【上一拍】装进 S 级、本拍才到 D 的
//  （S 级差一拍）⇒ 程序序上一定更老。
//  ★ 反过来用 older() 去排它就会踩 `above` 的滞后：`above` 是 rob 头指针的【寄存版】，
//    队尾绕回、rob 快满的那一拍，刚分配的槽会被算成"比谁都老"，于是进队条被排成最老、
//    读到一份不含本拍登记的旧表 ⇒ 拿到上一代生产者的值。
//    实测 seed20：`lw x28,0(x26)` 排在 `lui/addi x26` 之前 ⇒ 地址用成 0x806228，
//    签名第 26 个字读到 0（应为 0xda0e0b10）。
//  这样改还顺带省掉两处 older()（16 位弧掩码选择 + 比较），进队侧的写锥也短了。
//★ 2026-10-31：登记/取号点整体挪到【进队那一拍】之后，参与者只剩两个：h0 与 h1。
//  pstat 的内容从此恒等于"全部还在飞的写者"，跟本拍谁在译码无关；
//  h0 恒比 h1 老（同拍同序消费），所以 older() 那套排序整个删掉。
//  三档并成两档：with_older = 表 + h0（h1 读它），next = with_older + h1（落表）。
    integer pi;
    always @(*) begin
//★ 登记判据 = "这条会不会写 rd"，**不是** decoded_write_enable。decoded_write_enable 只在"结果从 ALU 口写回"时为 1，
//  LOAD 与 MUL 的 decoded_write_enable 都是 0（它们的结果从 lsu / mdu 的口回填）⇒ 用它登记会让
//  load/mul **永远不当生产者**：消费者不等它们、直接读架构旧值。
//  （实测 seed12：`lb x20` 之后紧接 `andi x23,x20,imm`，x23 拿到 0。）
//  rd 字段在 branch/store/系统非写类里已被译码器清成 0，所以 (rd != 0) 就是"写 rd"。
        decode_write_0 = mem_push_valid & (mem_push_destination_register_in != 5'd0);
        decode_write_1 = push1_valid_in & (push1_destination_register_in  != 5'd0);
        flush_any = kill_all;
        for (pi = 0; pi < 32; pi = pi + 1) begin
//---- 基础值：平时 = 上一拍的 pstat；重编那一拍（冲刷后第 2 拍）= 从幸存者表重建 ----
            if (reb_d2) begin
                producer_table_cleared_valid[pi] = 1'b0;
                producer_table_cleared_slot[pi] = 4'd0;
                producer_table_cleared_generation[pi] = 1'b0;
            end
            else begin
                producer_table_cleared_valid[pi] = producer_table_valid[pi];
                producer_table_cleared_slot[pi] = producer_table_slot[pi];
                producer_table_cleared_generation[pi] = producer_table_generation[pi];
            end
//---- 提交口清（按 slot+gen，只按 rd 会把老写者清掉）----
            if (write_enable_a & (producer_table_cleared_slot[pi] == commit_slot0) & (producer_table_cleared_generation[pi] == commit_gen0))
                producer_table_cleared_valid[pi] = 1'b0;
            if (write_enable_b & (producer_table_cleared_slot[pi] == commit_slot1) & (producer_table_cleared_generation[pi] == commit_gen1))
                producer_table_cleared_valid[pi] = 1'b0;
//---- 登记第一档：h0（较老）。h1 读的就是这一份 ----
            producer_table_with_older_valid[pi] = producer_table_cleared_valid[pi];
            producer_table_with_older_slot[pi] = producer_table_cleared_slot[pi];
            producer_table_with_older_generation[pi] = producer_table_cleared_generation[pi];
            if (decode_write_0 & (mem_push_destination_register_in == pi)) begin
                producer_table_with_older_valid[pi] = 1'b1;
                producer_table_with_older_slot[pi] = mem_push_index_in;
                producer_table_with_older_generation[pi] = mem_push_generation_in;
            end
//---- 登记第二档：h1（较年轻）。这一份就是下一沿落表的值 ----
            producer_next_valid[pi] = producer_table_with_older_valid[pi];
            producer_next_slot[pi] = producer_table_with_older_slot[pi];
            producer_next_generation[pi] = producer_table_with_older_generation[pi];
            if (decode_write_1 & (push1_destination_register_in == pi)) begin
                producer_next_valid[pi] = 1'b1;
                producer_next_slot[pi] = push1_index_in;
                producer_next_generation[pi] = push1_generation_in;
            end
        end
    end

//重建时相计数：冲刷那一拍起算，两拍后重编并落表。
//★ 冲刷电平可能连拉几拍，这时按最后一拍重起算（ping 一拍再排一次，代价只是多一次重编，不影响正确性）。
    always @(posedge clk) begin
        if (reset_registered) begin
            reb_d1 <= 1'b0;
            reb_d2 <= 1'b0;
        end
        else begin
            reb_d1 <= flush_any;
            reb_d2 <= reb_d1;
        end
    end

//D 级的两源 tag（装 Q 那一沿锁进 Q）
//★ 老的那条看 clr（本拍谁都不算它的生产者）；年轻的那条看 wr0（含老那条本拍的登记）。
//进队那拍的两组 tag（idu1 把它们塞进条目随条目走，出队时不再查表）
//★ push0（h0）读【上一沿落表的触发器】：它读的时候本拍的 h0 还没落表 ⇒ 天然不含自己、也不含更年轻的。
//★ push1（h1）读 with_older（已叠过 h0 那一份）—— h0 与 h1 同拍被消费，h0 更老。
    reg        push0_prod_a_valid, push0_prod_b_valid;
    reg [3:0]  push0_prod_a_slot, push0_prod_b_slot;
    reg        push0_prod_a_generation, push0_prod_b_generation;
    reg        push1_prod_a_valid, push1_prod_b_valid;
    reg [3:0]  push1_prod_a_slot, push1_prod_b_slot;
    reg        push1_prod_a_generation, push1_prod_b_generation;
    reg        decode_producer_a_0_valid, decode_producer_b_0_valid, decode_producer_a_1_valid, decode_producer_b_1_valid;
    reg [3:0]  decode_producer_a_0_slot, decode_producer_b_0_slot, decode_producer_a_1_slot, decode_producer_b_1_slot;
    reg        decode_producer_a_0_generation, decode_producer_b_0_generation, decode_producer_a_1_generation, decode_producer_b_1_generation;
//★ 名次 2 读的是【触发器】（上一沿落表），不含"本拍提交清" ⇒ 生产者恰好在进队这一拍提交时，
//  会登记出一个已经死掉的 tag。两条 lane 有 commit_same_cycle_* 兜这一拍，这里也必须兜：
//  漏了就是永久等待（实测 seed170：`sw x12,...` 的 tag 指向一个已经提交掉的槽 6/gen1，
//  值阵列永远等不到 ⇒ 访存口冻结 ⇒ ROB 队头永远是它 ⇒ 整核 60000 拍不动）。
    reg        commit_same_cycle_a_push0, commit_same_cycle_b_push0;
    reg        commit_same_cycle_a_push1, commit_same_cycle_b_push1;
//★ 读哪一份表：两条 lane 保持原来的二分 —— 老那条读 prod_tbl_* 【触发器】（不许读它的 D 端，
//  否则"落表那一拍"整段被旁路、生产者表的写锥一路穿到 q_prod_*_valid），年轻那条读 with_older。
//  访存那一条【恒为最年轻】⇒ 恒读 middle（= 已叠过两条 lane 的那一份）。
    always @(*) begin
        push0_prod_a_valid      = producer_table_valid[mem_push_rs1_in];
        push0_prod_a_slot       = producer_table_slot[mem_push_rs1_in];
        push0_prod_a_generation = producer_table_generation[mem_push_rs1_in];
        push0_prod_b_valid      = producer_table_valid[mem_push_rs2_in];
        push0_prod_b_slot       = producer_table_slot[mem_push_rs2_in];
        push0_prod_b_generation = producer_table_generation[mem_push_rs2_in];
        push1_prod_a_valid      = producer_table_with_older_valid[push1_rs1_in];
        push1_prod_a_slot       = producer_table_with_older_slot[push1_rs1_in];
        push1_prod_a_generation = producer_table_with_older_generation[push1_rs1_in];
        push1_prod_b_valid      = producer_table_with_older_valid[push1_rs2_in];
        push1_prod_b_slot       = producer_table_with_older_slot[push1_rs2_in];
        push1_prod_b_generation = producer_table_with_older_generation[push1_rs2_in];
        commit_same_cycle_a_push0 = (write_enable_a & (destination_register_a == mem_push_rs1_in) & (push0_prod_a_slot == commit_slot0) & (push0_prod_a_generation == commit_gen0))
                                  | (write_enable_b & (destination_register_b == mem_push_rs1_in) & (push0_prod_a_slot == commit_slot1) & (push0_prod_a_generation == commit_gen1));
        commit_same_cycle_b_push0 = (write_enable_a & (destination_register_a == mem_push_rs2_in) & (push0_prod_b_slot == commit_slot0) & (push0_prod_b_generation == commit_gen0))
                                  | (write_enable_b & (destination_register_b == mem_push_rs2_in) & (push0_prod_b_slot == commit_slot1) & (push0_prod_b_generation == commit_gen1));
        commit_same_cycle_a_push1 = (write_enable_a & (destination_register_a == push1_rs1_in) & (push1_prod_a_slot == commit_slot0) & (push1_prod_a_generation == commit_gen0))
                                  | (write_enable_b & (destination_register_b == push1_rs1_in) & (push1_prod_a_slot == commit_slot1) & (push1_prod_a_generation == commit_gen1));
        commit_same_cycle_b_push1 = (write_enable_a & (destination_register_a == push1_rs2_in) & (push1_prod_b_slot == commit_slot0) & (push1_prod_b_generation == commit_gen0))
                                  | (write_enable_b & (destination_register_b == push1_rs2_in) & (push1_prod_b_slot == commit_slot1) & (push1_prod_b_generation == commit_gen1));
//访存那一条用的是【进队】时的 rs1/rs2，不是队头的（队头可能已经是另一条了）。
//★ tag 打包成 {valid, slot[3:0], generation} = 6 位，与 lsu fifo 条目里的 [11:0] 同构。
//★ 这就是进队条目随身穿走的那份 tag：只在这里给出，出队时不再查表。
//  名次 2 读的是触发器（不含本拍提交清），所以本拍提交必须在这里再兜一次。
        mem_push_prod_a_out = {push0_prod_a_valid & ~commit_same_cycle_a_push0 & ~(reb_d1 | reb_d2),
                               push0_prod_a_slot, push0_prod_a_generation};
        mem_push_prod_b_out = {push0_prod_b_valid & ~commit_same_cycle_b_push0 & ~(reb_d1 | reb_d2),
                               push0_prod_b_slot, push0_prod_b_generation};
        push1_prod_a_out = {push1_prod_a_valid & ~commit_same_cycle_a_push1 & ~(reb_d1 | reb_d2),
                            push1_prod_a_slot, push1_prod_a_generation};
        push1_prod_b_out = {push1_prod_b_valid & ~commit_same_cycle_b_push1 & ~(reb_d1 | reb_d2),
                            push1_prod_b_slot, push1_prod_b_generation};
//★ 冲刷后【重建窗口】里不许取 tag：整表是冲刷后第 2 拍才清的（reb_d2），
//  而冲刷后第 1、2 拍就可能已经进队了 —— 那两拍查到的表里全是**已经被冲掉**的生产者 tag，
//  它们再也不会提交 ⇒ 值阵列永远等不到 ⇒ 访存口冻结 ⇒ ROB 队头卡死。
//  实测 andi-01（arch 套件）：`lw x31,0(x7)` 在前一条错路实例里留下 (slot9,gen0)，
//  冲刷重取后那条 store 还在窗口里进队，抓到这个死 tag。
//★ D 级【不再查表】：tag 是进队那拍就取好、随载荷条目带过来的。
//  这就是"边带随数据穿队列"—— 队列多深都不影响 tag 的正确性。
        decode_producer_a_0_valid      = lane0_prod_a_in[5];
        decode_producer_a_0_slot       = lane0_prod_a_in[4:1];
        decode_producer_a_0_generation = lane0_prod_a_in[0];
        decode_producer_b_0_valid      = lane0_prod_b_in[5];
        decode_producer_b_0_slot       = lane0_prod_b_in[4:1];
        decode_producer_b_0_generation = lane0_prod_b_in[0];
        decode_producer_a_1_valid      = lane1_prod_a_in[5];
        decode_producer_a_1_slot       = lane1_prod_a_in[4:1];
        decode_producer_a_1_generation = lane1_prod_a_in[0];
        decode_producer_b_1_valid      = lane1_prod_b_in[5];
        decode_producer_b_1_slot       = lane1_prod_b_in[4:1];
        decode_producer_b_1_generation = lane1_prod_b_in[0];
    end

//★ 到达那一拍就要认【提交旁路】：生产者恰好在同一拍提交时，pstat 同一拍被清掉、tag 也就
//  指不到活人了。若只在 else 分支里认 cmt，这一拍进来的那条会带着 wait=1，而之后既没有
//  wake（tag 已指不到人）也没有 cmt（提交那一拍已经过去）⇒ 永久等待
    reg        commit_same_cycle_a_0, commit_same_cycle_b_0, commit_same_cycle_a_1, commit_same_cycle_b_1;
//  （实测 seed16 死锁：Q 里 idx14 等 x15，tag 指 slot12/gen1，那条正在同一拍提交）。
    always @(*) begin
        commit_same_cycle_a_0 = (write_enable_a & (destination_register_a == rs1_in ) & (decode_producer_a_0_slot == commit_slot0) & (decode_producer_a_0_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs1_in ) & (decode_producer_a_0_slot == commit_slot1) & (decode_producer_a_0_generation == commit_gen1));
        commit_same_cycle_b_0 = (write_enable_a & (destination_register_a == rs2_in ) & (decode_producer_b_0_slot == commit_slot0) & (decode_producer_b_0_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs2_in ) & (decode_producer_b_0_slot == commit_slot1) & (decode_producer_b_0_generation == commit_gen1));
        commit_same_cycle_a_1 = (write_enable_a & (destination_register_a == rs1_in_lane1) & (decode_producer_a_1_slot == commit_slot0) & (decode_producer_a_1_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs1_in_lane1) & (decode_producer_a_1_slot == commit_slot1) & (decode_producer_a_1_generation == commit_gen1));
        commit_same_cycle_b_1 = (write_enable_a & (destination_register_a == rs2_in1) & (decode_producer_b_1_slot == commit_slot0) & (decode_producer_b_1_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs2_in1) & (decode_producer_b_1_slot == commit_slot1) & (decode_producer_b_1_generation == commit_gen1));
    end

//★ D 侧的 tag 快照：只在 d_new 那一沿锁。同一身份被按住、在 D 里连躺几拍时**不能重算** ——
//  那时 pstat 里已经有它自己那一项，重算会把"自己"读成自己的生产者（消费者等自己 ⇒ 自锁）。
//  锁下之后再遇到生产者提交（槽+世代匹配），就把 hit 清掉：值已经在寄存器堆里，改吃架构值。
    reg        decode_snapshot_producer_a_0_valid, decode_snapshot_producer_b_0_valid, decode_snapshot_producer_a_1_valid, decode_snapshot_producer_b_1_valid;
    reg [3:0]  decode_snapshot_producer_a_0_slot, decode_snapshot_producer_b_0_slot, decode_snapshot_producer_a_1_slot, decode_snapshot_producer_b_1_slot;
    reg        decode_snapshot_producer_a_0_generation, decode_snapshot_producer_b_0_generation, decode_snapshot_producer_a_1_generation, decode_snapshot_producer_b_1_generation;
    reg [31:0] db_a_0, db_b_0, db_a_1, db_b_1;
//★ base 的取值必须补【提交口旁路】：rdata 是寄存读（地址由 F 级给、值下一拍到 D），
//  所以"刚提交这一拍"的那个值在 rdata 里还是**旧的**。而 pstat 在同一拍按提交口把 tag 清掉
//  （因为值已进寄存器堆） ⇒ 消费者就会拿着旧 rdata 当架构值直接发出去。
//  实测 seed9：`addi x1,x1,0x200` 与 `lh x11,1008(x1)` 的 D 拍重合 ⇒ 地址算成 0x00800000。
//  口径与 regfile 的 bypass() 一致，只是这里的拍数是对的。
    reg [31:0] decode_base_next_a_0, decode_base_next_b_0, decode_base_next_a_1, decode_base_next_b_1;
    reg [31:0] decode_base_next_a_mem, decode_base_next_b_mem;
    reg        commit_at_queue_load_a_mem, commit_at_queue_load_b_mem;
    reg [31:0] commit_data_at_queue_load_a_mem, commit_data_at_queue_load_b_mem;
//★ 两口同 rd 时【口 B 赢】：B 是 head+1，比 A 年轻；regfile 的 bypass()/写口也是这个优先级。
//  反过来（A 先查）会在"同一拍退掉两条都写同一个 rd"时拿到**老那条**的值
//  （实测 seed301：`lhu x28` 与后一条写 x28 同拍退，`blt x28,x14` 的 A 口拿到 0）。
    always @(*) begin
        decode_base_next_a_0 = (write_enable_b & (destination_register_b == rs1_in )) ? commit_data1 :
                   (write_enable_a & (destination_register_a == rs1_in )) ? commit_data0 : rdata1_in;
        decode_base_next_b_0 = (write_enable_b & (destination_register_b == rs2_in )) ? commit_data1 :
                   (write_enable_a & (destination_register_a == rs2_in )) ? commit_data0 : rdata2_in;
        decode_base_next_a_1 = (write_enable_b & (destination_register_b == rs1_in_lane1)) ? commit_data1 :
                   (write_enable_a & (destination_register_a == rs1_in_lane1)) ? commit_data0 : rdata3_in;
        decode_base_next_b_1 = (write_enable_b & (destination_register_b == rs2_in1)) ? commit_data1 :
                   (write_enable_a & (destination_register_a == rs2_in1)) ? commit_data0 : rdata4_in;
        decode_base_next_a_mem = (write_enable_b & (destination_register_b == mem_rs1_in)) ? commit_data1 :
                   (write_enable_a & (destination_register_a == mem_rs1_in)) ? commit_data0 : mem_rdata1_in;
        decode_base_next_b_mem = (write_enable_b & (destination_register_b == mem_rs2_in)) ? commit_data1 :
                   (write_enable_a & (destination_register_a == mem_rs2_in)) ? commit_data0 : mem_rdata2_in;
//出队那一拍就撞上生产者提交：tag 直接作废、base 直接吃提交口的数据（与两条 lane 逐字同形）。
        commit_at_queue_load_a_mem = (write_enable_a & (destination_register_a == mem_rs1_in) & (mem_producer_a_slot_in == commit_slot0) & (mem_producer_a_generation_in == commit_gen0))
                 | (write_enable_b & (destination_register_b == mem_rs1_in) & (mem_producer_a_slot_in == commit_slot1) & (mem_producer_a_generation_in == commit_gen1));
        commit_at_queue_load_b_mem = (write_enable_a & (destination_register_a == mem_rs2_in) & (mem_producer_b_slot_in == commit_slot0) & (mem_producer_b_generation_in == commit_gen0))
                 | (write_enable_b & (destination_register_b == mem_rs2_in) & (mem_producer_b_slot_in == commit_slot1) & (mem_producer_b_generation_in == commit_gen1));
        commit_data_at_queue_load_a_mem = (write_enable_b & (destination_register_b == mem_rs1_in)) ? commit_data1 : commit_data0;
        commit_data_at_queue_load_b_mem = (write_enable_b & (destination_register_b == mem_rs2_in)) ? commit_data1 : commit_data0;
    end
//==============================================================================
// 值阵列（PRF）：按【槽号】索引的结果值表
//   取代原来的"前送表 fw0..fw10（按端口寻址，11 项）+ q_val 锁存 + 拉式 query"三条路。
//   相位与旧的前送表完全一致（都是结果口 D 端打一拍）—— 只是寻址方式从"端口号"换成"槽号"。
//   这样一拍最多两条 lane 各一个结果也不用合并：每个结果口各配一套 16 路 one-hot 写译码。
//   ★ 用 packed vector + 变址部分选择，不用 unpacked 数组：数组元素在消费它的 always @(*)
//     里进不了敏感表（实测 CoreMark 65 条后 load 地址算成 0、死锁）。
//   ★ 写口【身份随值走】：data / valid / generation 三样一起落。
//     `generation[slot]` 唯一的赋值来源就是这里 —— 不写它这个字段就没有值。
//   ★ 分配拍清本槽 valid（防 ABA）：`gen` 是跟着结果写留下的，同一槽偶数次分配后 gen 会"返祖"，
//     一个【还没写】的新生产者会和上上个占用者留下的值对上号。idu2 的 D 见到一条新指令
//     （decode_new_*）就是 rob 在给它分配槽，用它来清本槽，不需要新端口。
//   ★ 冲刷整表清 valid：这一条允许比冲刷【晚若干拍】—— 冲刷是整核的，改向目标要重新取指，
//     到 idu2 的 Q 至少 5 拍；而错路结果最晚在冲刷后 2 拍就落完（mul 二级 / lsu 三级）。
//     所以"槽已被重新分配 + 消费者已在 Q"和"错路写正在落地"这两个事件在时间上不可能重叠。
//==============================================================================
//★ 2026-10-28：数据阵列拆成 16 个【标量】而不是一个 512 位向量。
//  向量写法让每一位的 D 都挂一棵 16 槽 × 7 口的 mux，Vivado 把整块摆在一起再长线扇回 ⇒
//  impl 实测 375/500 条最差端点的终点都是 value_table_data_reg，布线占 79%。
//  拆成标量后每个槽是独立的 32 位寄存器，D 端是一棵 7:1（带保持）的小 mux，布局能就近收敛。
//  ★ 读口也必须逐项列成 case：数组元素在 always @(*) 里进不了敏感表（本项目实测过）。
//★宏化旋钮：值表深度（必须 = ROB 深度）。16 = 大核 / 8 = 小核。
//  ★ 改成数组是为了让深度真的是参数：原来 16 个显式寄存器 + case 树把分支上限写死了，
//    深度一改就引用越界元素（iverilog 报 out-of-bounds、返回 x）。
    parameter integer VALUE_TABLE_DEPTH = 16;
    localparam [3:0] VT_IDX_MASK = VALUE_TABLE_DEPTH - 1;
    reg [31:0] value_table_data   [0:VALUE_TABLE_DEPTH-1];
//★ 2026-10-30：值表的【第二份】。访存专用流的两个读口只读这一份。
//  为什么：值表是触发器阵列（写口多、进不了 RAM），6 个读口 = 6 棵 16:1 mux 树，
//  全部得贴着同一份 512 个 FF 放，而消费者分散在 alu_0/alu_1/mdu/lsu ⇒ 布局只能摊开，route 占 79%。
//  拆两份后每份只服务自己那几个消费者，树可以跟着消费者放。两份【内容恒等】，只是多一份寄存器。
//  ★ 写点必须两处都写。case 分支里用 begin/end 包住两条赋值 —— 直接并列会被当成重复分支标签，
//    只有第一条生效（实测：mem 那一份永远收不到写，访存流读到 0，整核在 pc=0x68 挂死）。
    reg [VALUE_TABLE_DEPTH-1:0] value_table_valid;
    reg [VALUE_TABLE_DEPTH-1:0] value_table_generation;
//★ 2026-10-28：值阵列只接【5 个】写口 —— alu / alu_lane1 / mul / multiply_lane1 / ld。
//  div / divide_lane1 不进阵列：除法在 CoreMark 里只有 ~150 次，占的等待不到 1%，
//  但它要占两套 16 路写译码，而写口正是 impl 实测的最差路径（105 条失败端点全在这族）。
//  除法结果走【提交前送】：div 提交那一拍 commit_hit 清掉 tag、base 换成提交值，消费者照常发出。
//  活锁证明：生产者一定比消费者老 ⇒ div 的提交一定早于消费者，那条路必然走得到。
//  ⇒ 对应的 idu2 输入口 write_enable_div_early / write_enable_divide_lane1_early 现在是【未使用】。

    always @(posedge clk) begin
        if (reset_registered) begin
            for (pi = 0; pi < VALUE_TABLE_DEPTH; pi = pi + 1) begin
                value_table_data[pi]   <= 32'd0;
            end
            value_table_valid      <= {VALUE_TABLE_DEPTH{1'b0}};
            value_table_generation <= {VALUE_TABLE_DEPTH{1'b0}};
        end
        else begin
            if (write_enable_alu_early) begin
                value_table_data  [index_alu_early & VT_IDX_MASK] <= data_alu_early;
            end
            if (write_enable_alu_lane1_early) begin
                value_table_data  [index_alu_lane1_early & VT_IDX_MASK] <= data_alu_lane1_early;
            end
            if (write_enable_mul_early) begin
                value_table_data  [index_mul_early & VT_IDX_MASK] <= data_mul_early;
            end
            if (write_enable_multiply_lane1_early) begin
                value_table_data  [index_multiply_lane1_early & VT_IDX_MASK] <= data_multiply_lane1_early;
            end
            if (write_enable_ld_early) begin
                value_table_data  [index_ld_early & VT_IDX_MASK] <= data_ld_early;
            end
//valid / generation 仍是 16 位向量（只有 1 位宽，译码便宜）
            if (write_enable_alu_early) begin
                value_table_valid[index_alu_early] <= 1'b1;
                value_table_generation[index_alu_early] <= generation_alu_early;
            end
            if (write_enable_alu_lane1_early) begin
                value_table_valid[index_alu_lane1_early] <= 1'b1;
                value_table_generation[index_alu_lane1_early] <= generation_alu_lane1_early;
            end
            if (write_enable_mul_early) begin
                value_table_valid[index_mul_early] <= 1'b1;
                value_table_generation[index_mul_early] <= generation_mul_early;
            end
            if (write_enable_multiply_lane1_early) begin
                value_table_valid[index_multiply_lane1_early] <= 1'b1;
                value_table_generation[index_multiply_lane1_early] <= generation_multiply_lane1_early;
            end
            if (write_enable_ld_early) begin
                value_table_valid[index_ld_early] <= 1'b1;
                value_table_generation[index_ld_early] <= generation_ld_early;
            end
//分配拍清本槽（ABA 的唯一防线）
            if (decode_new_0) value_table_valid[index_in]        <= 1'b0;
            if (decode_new_1) value_table_valid[index_in_lane1]  <= 1'b0;
//访存条的清也提到【进队】那一拍（与 pstat 登记同拍），不再等到出队。
            if (decode_new_mem) value_table_valid[mem_push_index_in] <= 1'b0;
//冲刷：整表作废（允许晚若干拍，见上面注释）。放在写之后 ⇒ 冲刷赢。
            if (kill_all | flush_control_rob_registered) value_table_valid <= 16'd0;
        end
    end

//★ D 侧的 base 快照：rdata 与 D 里的指令【同拍对齐】只在"这条刚进 D 那一拍"成立
//  （读口地址由 F 级给、寄存一拍到 D 拍）。D 一旦被按住，F 就走过去了、rdata 也跟着变，
//  所以在 d_new 那一沿把 base 锁下来，装 Q 的时候直接用。
//★ 它必须和 dt_prod_* 在【同一个】always 块里更新：生产者提交时 tag 清掉、base 要同时换成提交值，
//  两块分开写就会漏掉后者（实测自锁）。
    always @(posedge clk) begin
        if (reset_registered) begin
            decode_snapshot_producer_a_0_valid <= 1'b0;
            decode_snapshot_producer_b_0_valid <= 1'b0;
            decode_snapshot_producer_a_1_valid <= 1'b0;
            decode_snapshot_producer_b_1_valid <= 1'b0;
            decode_snapshot_producer_a_0_slot <= 4'd0;
            decode_snapshot_producer_b_0_slot <= 4'd0;
            decode_snapshot_producer_a_1_slot <= 4'd0;
            decode_snapshot_producer_b_1_slot <= 4'd0;
            decode_snapshot_producer_a_0_generation <= 1'b0;
            decode_snapshot_producer_b_0_generation <= 1'b0;
            decode_snapshot_producer_a_1_generation <= 1'b0;
            decode_snapshot_producer_b_1_generation <= 1'b0;
            db_a_0 <= 32'd0;
            db_b_0 <= 32'd0;
            db_a_1 <= 32'd0;
            db_b_1 <= 32'd0;
        end
        else begin
            if (decode_new_0) begin
                decode_snapshot_producer_a_0_valid <= decode_producer_a_0_valid & ~commit_same_cycle_a_0;
                decode_snapshot_producer_a_0_slot <= decode_producer_a_0_slot;
                decode_snapshot_producer_a_0_generation <= decode_producer_a_0_generation;
                decode_snapshot_producer_b_0_valid <= decode_producer_b_0_valid & ~commit_same_cycle_b_0;
                decode_snapshot_producer_b_0_slot <= decode_producer_b_0_slot;
                decode_snapshot_producer_b_0_generation <= decode_producer_b_0_generation;
                db_a_0 <= decode_base_next_a_0;
                db_b_0 <= decode_base_next_b_0;
            end
            else begin
//★ 生产者在这条还躺在 D 里的时候提交了：清 hit（值已在寄存器堆）**并且**把 base 换成提交值。
//  只清 hit 不改 base 是错的一半 —— base 快照是"生产者还没写回"时拍的，留着它就是拿着旧值
//  直接发出去（实测：`lw xN,0(x31)` 的 A 口拿到 0，访存地址算成 0，dcache 永不应答 ⇒ 整核自锁）。
                if (write_enable_a & (destination_register_a == rs1_in) & (decode_snapshot_producer_a_0_slot == commit_slot0) & (decode_snapshot_producer_a_0_generation == commit_gen0)) begin
                    decode_snapshot_producer_a_0_valid <= 1'b0;
                    db_a_0 <= commit_data0;
                end
                if (write_enable_b & (destination_register_b == rs1_in) & (decode_snapshot_producer_a_0_slot == commit_slot1) & (decode_snapshot_producer_a_0_generation == commit_gen1)) begin
                    decode_snapshot_producer_a_0_valid <= 1'b0;
                    db_a_0 <= commit_data1;
                end
                if (write_enable_a & (destination_register_a == rs2_in) & (decode_snapshot_producer_b_0_slot == commit_slot0) & (decode_snapshot_producer_b_0_generation == commit_gen0)) begin
                    decode_snapshot_producer_b_0_valid <= 1'b0;
                    db_b_0 <= commit_data0;
                end
                if (write_enable_b & (destination_register_b == rs2_in) & (decode_snapshot_producer_b_0_slot == commit_slot1) & (decode_snapshot_producer_b_0_generation == commit_gen1)) begin
                    decode_snapshot_producer_b_0_valid <= 1'b0;
                    db_b_0 <= commit_data1;
                end
            end
            if (decode_new_1) begin
                decode_snapshot_producer_a_1_valid <= decode_producer_a_1_valid & ~commit_same_cycle_a_1;
                decode_snapshot_producer_a_1_slot <= decode_producer_a_1_slot;
                decode_snapshot_producer_a_1_generation <= decode_producer_a_1_generation;
                decode_snapshot_producer_b_1_valid <= decode_producer_b_1_valid & ~commit_same_cycle_b_1;
                decode_snapshot_producer_b_1_slot <= decode_producer_b_1_slot;
                decode_snapshot_producer_b_1_generation <= decode_producer_b_1_generation;
                db_a_1 <= decode_base_next_a_1;
                db_b_1 <= decode_base_next_b_1;
            end
            else begin
//★ 同 lane0：清 hit 的同时把 base 换成提交值（base 快照是生产者还没写回时拍的）。
                if (write_enable_a & (destination_register_a == rs1_in_lane1) & (decode_snapshot_producer_a_1_slot == commit_slot0) & (decode_snapshot_producer_a_1_generation == commit_gen0)) begin
                    decode_snapshot_producer_a_1_valid <= 1'b0;
                    db_a_1 <= commit_data0;
                end
                if (write_enable_b & (destination_register_b == rs1_in_lane1) & (decode_snapshot_producer_a_1_slot == commit_slot1) & (decode_snapshot_producer_a_1_generation == commit_gen1)) begin
                    decode_snapshot_producer_a_1_valid <= 1'b0;
                    db_a_1 <= commit_data1;
                end
                if (write_enable_a & (destination_register_a == rs2_in1) & (decode_snapshot_producer_b_1_slot == commit_slot0) & (decode_snapshot_producer_b_1_generation == commit_gen0)) begin
                    decode_snapshot_producer_b_1_valid <= 1'b0;
                    db_b_1 <= commit_data0;
                end
                if (write_enable_b & (destination_register_b == rs2_in1) & (decode_snapshot_producer_b_1_slot == commit_slot1) & (decode_snapshot_producer_b_1_generation == commit_gen1)) begin
                    decode_snapshot_producer_b_1_valid <= 1'b0;
                    db_b_1 <= commit_data1;
                end
            end
        end
    end

//装 Q 那一拍取哪一份 tag：本拍正是这条指令【新出现】那一拍（d_new）⇒ 用组合那份；
//它已经在 D 里躺过（被按住过）⇒ 用锁下来的快照。两者不能合成一份：
//组合那份在"自己已经登记进 pstat"之后会把自分读成生产者，快照那份又拿不到刚出现那一拍的值。
    reg        to_queue_producer_a_0_valid, to_queue_producer_b_0_valid, to_queue_producer_a_1_valid, to_queue_producer_b_1_valid;
    reg [3:0]  to_queue_producer_a_0_slot, to_queue_producer_b_0_slot, to_queue_producer_a_1_slot, to_queue_producer_b_1_slot;
    reg        to_queue_producer_a_0_generation, to_queue_producer_b_0_generation, to_queue_producer_a_1_generation, to_queue_producer_b_1_generation;
    always @(*) begin
        to_queue_producer_a_0_valid = decode_new_0 ? decode_producer_a_0_valid : decode_snapshot_producer_a_0_valid;
        to_queue_producer_b_0_valid = decode_new_0 ? decode_producer_b_0_valid : decode_snapshot_producer_b_0_valid;
        to_queue_producer_a_1_valid = decode_new_1 ? decode_producer_a_1_valid : decode_snapshot_producer_a_1_valid;
        to_queue_producer_b_1_valid = decode_new_1 ? decode_producer_b_1_valid : decode_snapshot_producer_b_1_valid;
        to_queue_producer_a_0_slot = decode_new_0 ? decode_producer_a_0_slot : decode_snapshot_producer_a_0_slot;
        to_queue_producer_b_0_slot = decode_new_0 ? decode_producer_b_0_slot : decode_snapshot_producer_b_0_slot;
        to_queue_producer_a_1_slot = decode_new_1 ? decode_producer_a_1_slot : decode_snapshot_producer_a_1_slot;
        to_queue_producer_b_1_slot = decode_new_1 ? decode_producer_b_1_slot : decode_snapshot_producer_b_1_slot;
        to_queue_producer_a_0_generation = decode_new_0 ? decode_producer_a_0_generation : decode_snapshot_producer_a_0_generation;
        to_queue_producer_b_0_generation = decode_new_0 ? decode_producer_b_0_generation : decode_snapshot_producer_b_0_generation;
        to_queue_producer_a_1_generation = decode_new_1 ? decode_producer_a_1_generation : decode_snapshot_producer_a_1_generation;
        to_queue_producer_b_1_generation = decode_new_1 ? decode_producer_b_1_generation : decode_snapshot_producer_b_1_generation;
    end


//（db_* 的锁存与提交旁路已并入上面的 dt_prod_* 那个块 —— 两者必须同沿同门更新）

//★ 装 Q／锁 dt 那一拍就要认【提交旁路】：生产者恰好在同一拍提交时，pstat 同一拍被清掉、
//  tag 也已经指不到活人。如果只在 else 分支里认 cmt，这一拍进来的那条就带着 wait=1，
//  而之后既没有 wake（tag 已指不到人）也没有 cmt（提交那一拍已经过去）⇒ 永久等待
//  （实测 seed16 死锁：Q 里 idx14 等 x15，tag 指 slot12/gen1，那条正在同一拍提交）。
    reg        commit_at_queue_load_a_0, commit_at_queue_load_b_0, commit_at_queue_load_a_1, commit_at_queue_load_b_1;
    always @(*) begin
        commit_at_queue_load_a_0 = (write_enable_a & (destination_register_a == rs1_in ) & (to_queue_producer_a_0_slot == commit_slot0) & (to_queue_producer_a_0_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs1_in ) & (to_queue_producer_a_0_slot == commit_slot1) & (to_queue_producer_a_0_generation == commit_gen1));
        commit_at_queue_load_b_0 = (write_enable_a & (destination_register_a == rs2_in ) & (to_queue_producer_b_0_slot == commit_slot0) & (to_queue_producer_b_0_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs2_in ) & (to_queue_producer_b_0_slot == commit_slot1) & (to_queue_producer_b_0_generation == commit_gen1));
        commit_at_queue_load_a_1 = (write_enable_a & (destination_register_a == rs1_in_lane1) & (to_queue_producer_a_1_slot == commit_slot0) & (to_queue_producer_a_1_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs1_in_lane1) & (to_queue_producer_a_1_slot == commit_slot1) & (to_queue_producer_a_1_generation == commit_gen1));
        commit_at_queue_load_b_1 = (write_enable_a & (destination_register_a == rs2_in1) & (to_queue_producer_b_1_slot == commit_slot0) & (to_queue_producer_b_1_generation == commit_gen0))
                 | (write_enable_b & (destination_register_b == rs2_in1) & (to_queue_producer_b_1_slot == commit_slot1) & (to_queue_producer_b_1_generation == commit_gen1));
    end
//装 Q 那一拍如果正好撞上提交，base 直接吃提交口的数据（与 regfile 的 bypass 同口径）
    reg [31:0] commit_data_at_queue_load_a_0, commit_data_at_queue_load_b_0, commit_data_at_queue_load_a_1, commit_data_at_queue_load_b_1;
    always @(*) begin
//装 Q 那一拍如果正好撞上提交，base 直接吃提交口的数据（与 regfile 的 bypass 同口径）。
//★ 同样【口 B 赢】：同一拍退掉的两条都写这个 rd 时，架构值是年轻的那条（B）。
        commit_data_at_queue_load_a_0 = (write_enable_b & (destination_register_b == rs1_in )) ? commit_data1 : commit_data0;
        commit_data_at_queue_load_b_0 = (write_enable_b & (destination_register_b == rs2_in )) ? commit_data1 : commit_data0;
        commit_data_at_queue_load_a_1 = (write_enable_b & (destination_register_b == rs1_in_lane1)) ? commit_data1 : commit_data0;
        commit_data_at_queue_load_b_1 = (write_enable_b & (destination_register_b == rs2_in1)) ? commit_data1 : commit_data0;
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
//scoreboard：每源 {hit, slot, gen} + wait
    reg        queue_producer_a_0_valid, queue_producer_b_0_valid, queue_producer_a_1_valid, queue_producer_b_1_valid;
    reg [3:0]  queue_producer_a_0_slot, queue_producer_b_0_slot, queue_producer_a_1_slot, queue_producer_b_1_slot;
    reg        queue_producer_a_0_generation, queue_producer_b_0_generation, queue_producer_a_1_generation, queue_producer_b_1_generation;
//唤醒时锁下来的值：带值唤醒之后操作数不再依赖活线
//rdata 兜底档
    reg [31:0] queue_base_a_0, queue_base_b_0, queue_base_a_1, queue_base_b_1;
//---- 访存专用流的 Q（字段只留访存用得着的那些）----
    reg        queue_valid_mem;
    reg [4:0]  queue_source_register_a_mem, queue_source_register_b_mem;
    reg [3:0]  queue_index_mem;
    reg        queue_generation_mem;
    reg [4:0]  queue_destination_register_mem;
    reg        queue_is_st_mem;
    reg [2:0]  queue_load_store_function3_mem;
    reg [31:0] queue_auxiliary_mem, queue_jump_value_mem;
    reg        queue_producer_a_mem_valid, queue_producer_b_mem_valid;
    reg [3:0]  queue_producer_a_mem_slot, queue_producer_b_mem_slot;
    reg        queue_producer_a_mem_generation, queue_producer_b_mem_generation;
    reg [31:0] queue_base_a_mem, queue_base_b_mem;

    reg kill_0, kill_1, kill_mem;
    always @(*) begin
        kill_0 = queue_valid_0 & kill_all;
        kill_1 = queue_valid_1 & kill_all;
        kill_mem = queue_valid_mem & kill_all;
    end


//==============================================================================
// 四、唤醒 / 就绪 / 操作数四档
//==============================================================================
    reg        commit_hit_a_0, commit_hit_b_0, commit_hit_a_1, commit_hit_b_1;
    reg        commit_hit_a_mem, commit_hit_b_mem;
//按端口分的提交命中：base 直接吃命中那个口的数据（不再靠读口每拍刷新）
    reg        commit_port_a_source_a_lane0, commit_port_b_source_a_lane0, commit_port_a_source_b_lane0, commit_port_b_source_b_lane0;
    reg        commit_port_a_source_a_lane1, commit_port_b_source_a_lane1, commit_port_a_source_b_lane1, commit_port_b_source_b_lane1;
    reg        commit_port_a_source_a_mem, commit_port_b_source_a_mem, commit_port_a_source_b_mem, commit_port_b_source_b_mem;
    reg        operand_ready_a_0, operand_ready_b_0, operand_ready_a_1, operand_ready_b_1;
    reg [31:0] operand_a_0, operand_b_0, operand_a_1, operand_b_1;
//值阵列读口：按槽号一次电平读（取代 q_val 锁存 / 前送 mux / 拉式 query 三条路）
    reg        value_table_hit_a_0, value_table_hit_b_0, value_table_hit_a_1, value_table_hit_b_1;
//---- ALU 前送（2026-10-30）----
//★ ALU 的组合锥改成从它自己的操作数寄存器出发以后，结果比原来晚一拍才进值表。
//  「刚算出来、还没进表」的这一拍必须有人顶住，否则消费者会多等一拍 ⇒ 这就是那一档。
//★ 前送是组合的（ALU 口上的身份 + 结果直接出去），但它只喂操作数选择，
//  不回 ALU 自己的组合锥 —— 所以不会串成 ALU→ALU 的长锥。
    reg alu_lane0_forward_a_0, alu_lane0_forward_b_0, alu_lane1_forward_a_0, alu_lane1_forward_b_0;
    reg alu_lane0_forward_a_1, alu_lane0_forward_b_1, alu_lane1_forward_a_1, alu_lane1_forward_b_1;
//---- load 前送（2026-10-30）----
//★ 它和 load 早唤醒是【两个不同的控制来源】，必须分开看（impl 实测的结论）：
//  · load_forward_*：身份全部来自触发器（write_enable_ld_early = stage3_valid & ready_in，
//    ready_in 里的 ready_dcache 是 dcache 的 load_ready，寄存器）。只有【数据】是组合的，
//    而它的源是 dcache 的 bus_data_out，也是寄存器。⇒ 这条旁路不碰 dcache 阵列。
//  · load_wakeup_*：使能来自 load_ready_next，那是 dcache 里 tag 阵列读 + 21 位比较的组合结果
//    （dcache.v:151 在 always @(*) 里）。把它挂进 operand_ready_* 等于让 issue_valid 等一次查表。
//  ⇒ 2026-10-30 只保留前送，早唤醒整族撤回。代价 ≤ 2.56% ticks，换发射判定回到纯触发器。
    reg load_forward_a_0, load_forward_b_0, load_forward_a_1, load_forward_b_1;
    reg load_forward_a_mem, load_forward_b_mem;
    reg alu_lane0_forward_a_mem, alu_lane1_forward_a_mem;
    reg alu_lane0_forward_b_mem, alu_lane1_forward_b_mem;
    reg [31:0] value_table_read_a_0, value_table_read_b_0, value_table_read_a_1, value_table_read_b_1;
    reg        value_table_hit_a_mem, value_table_hit_b_mem;
    reg [31:0] value_table_read_a_mem, value_table_read_b_mem;
    reg        operand_ready_a_mem, operand_ready_b_mem;
    reg [31:0] operand_a_mem, operand_b_mem;

    always @(*) begin
//---- 提交口命中（rd + slot + gen 三者全比）----
//按端口分开留：base 的刷新要取【命中那个口的数据】（口 B 更年轻，同 rd 时它赢）。
        commit_port_a_source_a_lane0 = write_enable_a & (destination_register_a == queue_source_register_a_0) & (commit_slot0 == queue_producer_a_0_slot) & (commit_gen0 == queue_producer_a_0_generation);
        commit_port_b_source_a_lane0 = write_enable_b & (destination_register_b == queue_source_register_a_0) & (commit_slot1 == queue_producer_a_0_slot) & (commit_gen1 == queue_producer_a_0_generation);
        commit_port_a_source_b_lane0 = write_enable_a & (destination_register_a == queue_source_register_b_0) & (commit_slot0 == queue_producer_b_0_slot) & (commit_gen0 == queue_producer_b_0_generation);
        commit_port_b_source_b_lane0 = write_enable_b & (destination_register_b == queue_source_register_b_0) & (commit_slot1 == queue_producer_b_0_slot) & (commit_gen1 == queue_producer_b_0_generation);
        commit_port_a_source_a_lane1 = write_enable_a & (destination_register_a == queue_source_register_a_1) & (commit_slot0 == queue_producer_a_1_slot) & (commit_gen0 == queue_producer_a_1_generation);
        commit_port_b_source_a_lane1 = write_enable_b & (destination_register_b == queue_source_register_a_1) & (commit_slot1 == queue_producer_a_1_slot) & (commit_gen1 == queue_producer_a_1_generation);
        commit_port_a_source_b_lane1 = write_enable_a & (destination_register_a == queue_source_register_b_1) & (commit_slot0 == queue_producer_b_1_slot) & (commit_gen0 == queue_producer_b_1_generation);
        commit_port_b_source_b_lane1 = write_enable_b & (destination_register_b == queue_source_register_b_1) & (commit_slot1 == queue_producer_b_1_slot) & (commit_gen1 == queue_producer_b_1_generation);
        commit_hit_a_0 = commit_port_a_source_a_lane0 | commit_port_b_source_a_lane0;
        commit_hit_b_0 = commit_port_a_source_b_lane0 | commit_port_b_source_b_lane0;
        commit_hit_a_1 = commit_port_a_source_a_lane1 | commit_port_b_source_a_lane1;
        commit_hit_b_1 = commit_port_a_source_b_lane1 | commit_port_b_source_b_lane1;
        commit_port_a_source_a_mem = write_enable_a & (destination_register_a == queue_source_register_a_mem) & (commit_slot0 == queue_producer_a_mem_slot) & (commit_gen0 == queue_producer_a_mem_generation);
        commit_port_b_source_a_mem = write_enable_b & (destination_register_b == queue_source_register_a_mem) & (commit_slot1 == queue_producer_a_mem_slot) & (commit_gen1 == queue_producer_a_mem_generation);
        commit_port_a_source_b_mem = write_enable_a & (destination_register_a == queue_source_register_b_mem) & (commit_slot0 == queue_producer_b_mem_slot) & (commit_gen0 == queue_producer_b_mem_generation);
        commit_port_b_source_b_mem = write_enable_b & (destination_register_b == queue_source_register_b_mem) & (commit_slot1 == queue_producer_b_mem_slot) & (commit_gen1 == queue_producer_b_mem_generation);
        commit_hit_a_mem = commit_port_a_source_a_mem | commit_port_b_source_a_mem;
        commit_hit_b_mem = commit_port_a_source_b_mem | commit_port_b_source_b_mem;
//---- 就绪 + 取值：一次电平读 ----
//★ 老的四档（immediate / value_ok / pending 前送 / 拉式 query / base）全部退场：
//  值有了固定的家（按槽号索引的 value_table），消费者只需要知道槽号。
//  `queue_producer_*_valid` = "有一个还没提交的在飞写者"；为 0 ⇒ 值已经进寄存器堆 ⇒ 用 base 快照。
        value_table_hit_a_0 = value_table_valid[queue_producer_a_0_slot] & (value_table_generation[queue_producer_a_0_slot] == queue_producer_a_0_generation);
        value_table_hit_b_0 = value_table_valid[queue_producer_b_0_slot] & (value_table_generation[queue_producer_b_0_slot] == queue_producer_b_0_generation);
        value_table_hit_a_1 = value_table_valid[queue_producer_a_1_slot] & (value_table_generation[queue_producer_a_1_slot] == queue_producer_a_1_generation);
        value_table_hit_b_1 = value_table_valid[queue_producer_b_1_slot] & (value_table_generation[queue_producer_b_1_slot] == queue_producer_b_1_generation);
        value_table_read_a_0 = value_table_data[queue_producer_a_0_slot & VT_IDX_MASK];
        value_table_read_b_0 = value_table_data[queue_producer_b_0_slot & VT_IDX_MASK];
        value_table_read_a_1 = value_table_data[queue_producer_a_1_slot & VT_IDX_MASK];
        value_table_read_b_1 = value_table_data[queue_producer_b_1_slot & VT_IDX_MASK];
//---- 访存专用流（第三个读口）----
        value_table_hit_a_mem = value_table_valid[queue_producer_a_mem_slot] & (value_table_generation[queue_producer_a_mem_slot] == queue_producer_a_mem_generation);
        value_table_hit_b_mem = value_table_valid[queue_producer_b_mem_slot] & (value_table_generation[queue_producer_b_mem_slot] == queue_producer_b_mem_generation);
        value_table_read_a_mem = value_table_data[queue_producer_a_mem_slot & VT_IDX_MASK];
        value_table_read_b_mem = value_table_data[queue_producer_b_mem_slot & VT_IDX_MASK];
//访存的两个源恒是寄存器（load/store 没有立即数源）⇒ 没有 immediate 这一档。
//★ 2026-10-30：load 早唤醒已撤（见上方 load 前送那段），这里只剩前送这一档。
        load_forward_a_mem = queue_producer_a_mem_valid & write_enable_ld_early & (index_ld_early == queue_producer_a_mem_slot) & (generation_ld_early == queue_producer_a_mem_generation);
        load_forward_b_mem = queue_producer_b_mem_valid & write_enable_ld_early & (index_ld_early == queue_producer_b_mem_slot) & (generation_ld_early == queue_producer_b_mem_generation);
//★ 2026-10-31：访存条也吃 ALU 前送。
//  以前不给：LSU 的加法在 D 端，操作数要早一拍到，「比较 + mux + 加法」串不进去。
//  现在加数在 Q 端锁（lsu.v 的 stage1_base），这一档只剩「比较 + mux → 寄存器」。
//  档位顺序与 lane 逐条同形：ALU(两条) → load 前送 → 值表/base（load 数据到得最晚，贴着最后一级 mux）。
        alu_lane0_forward_a_mem = queue_producer_a_mem_valid & write_enable_alu_early & (index_alu_early == queue_producer_a_mem_slot) & (generation_alu_early == queue_producer_a_mem_generation);
        alu_lane1_forward_a_mem = queue_producer_a_mem_valid & write_enable_alu_lane1_early & (index_alu_lane1_early == queue_producer_a_mem_slot) & (generation_alu_lane1_early == queue_producer_a_mem_generation);
        alu_lane0_forward_b_mem = queue_producer_b_mem_valid & write_enable_alu_early & (index_alu_early == queue_producer_b_mem_slot) & (generation_alu_early == queue_producer_b_mem_generation);
        alu_lane1_forward_b_mem = queue_producer_b_mem_valid & write_enable_alu_lane1_early & (index_alu_lane1_early == queue_producer_b_mem_slot) & (generation_alu_lane1_early == queue_producer_b_mem_generation);
        operand_ready_a_mem = ~queue_producer_a_mem_valid | value_table_hit_a_mem | alu_lane0_forward_a_mem | alu_lane1_forward_a_mem | load_forward_a_mem;
        operand_ready_b_mem = ~queue_producer_b_mem_valid | value_table_hit_b_mem | alu_lane0_forward_b_mem | alu_lane1_forward_b_mem | load_forward_b_mem;
        operand_a_mem = alu_lane0_forward_a_mem ? data_alu_early :
                        alu_lane1_forward_a_mem ? data_alu_lane1_early :
                        load_forward_a_mem ? data_ld_early :
                        queue_producer_a_mem_valid ? value_table_read_a_mem : queue_base_a_mem;
        operand_b_mem = alu_lane0_forward_b_mem ? data_alu_early :
                        alu_lane1_forward_b_mem ? data_alu_lane1_early :
                        load_forward_b_mem ? data_ld_early :
                        queue_producer_b_mem_valid ? value_table_read_b_mem : queue_base_b_mem;

        load_forward_a_0 = queue_producer_a_0_valid & write_enable_ld_early & (index_ld_early == queue_producer_a_0_slot) & (generation_ld_early == queue_producer_a_0_generation);
        load_forward_b_0 = queue_producer_b_0_valid & write_enable_ld_early & (index_ld_early == queue_producer_b_0_slot) & (generation_ld_early == queue_producer_b_0_generation);
        load_forward_a_1 = queue_producer_a_1_valid & write_enable_ld_early & (index_ld_early == queue_producer_a_1_slot) & (generation_ld_early == queue_producer_a_1_generation);
        load_forward_b_1 = queue_producer_b_1_valid & write_enable_ld_early & (index_ld_early == queue_producer_b_1_slot) & (generation_ld_early == queue_producer_b_1_generation);
        alu_lane0_forward_a_0 = queue_producer_a_0_valid & write_enable_alu_early      & (index_alu_early      == queue_producer_a_0_slot) & (generation_alu_early      == queue_producer_a_0_generation);
        alu_lane1_forward_a_0 = queue_producer_a_0_valid & write_enable_alu_lane1_early & (index_alu_lane1_early == queue_producer_a_0_slot) & (generation_alu_lane1_early == queue_producer_a_0_generation);
        alu_lane0_forward_b_0 = queue_producer_b_0_valid & write_enable_alu_early      & (index_alu_early      == queue_producer_b_0_slot) & (generation_alu_early      == queue_producer_b_0_generation);
        alu_lane1_forward_b_0 = queue_producer_b_0_valid & write_enable_alu_lane1_early & (index_alu_lane1_early == queue_producer_b_0_slot) & (generation_alu_lane1_early == queue_producer_b_0_generation);
        alu_lane0_forward_a_1 = queue_producer_a_1_valid & write_enable_alu_early      & (index_alu_early      == queue_producer_a_1_slot) & (generation_alu_early      == queue_producer_a_1_generation);
        alu_lane1_forward_a_1 = queue_producer_a_1_valid & write_enable_alu_lane1_early & (index_alu_lane1_early == queue_producer_a_1_slot) & (generation_alu_lane1_early == queue_producer_a_1_generation);
        alu_lane0_forward_b_1 = queue_producer_b_1_valid & write_enable_alu_early      & (index_alu_early      == queue_producer_b_1_slot) & (generation_alu_early      == queue_producer_b_1_generation);
        alu_lane1_forward_b_1 = queue_producer_b_1_valid & write_enable_alu_lane1_early & (index_alu_lane1_early == queue_producer_b_1_slot) & (generation_alu_lane1_early == queue_producer_b_1_generation);
        operand_ready_a_0 = queue_immediate_a_0 | ~queue_producer_a_0_valid | value_table_hit_a_0 | alu_lane0_forward_a_0 | alu_lane1_forward_a_0 | load_forward_a_0;
        operand_ready_b_0 = queue_immediate_b_0 | ~queue_producer_b_0_valid | value_table_hit_b_0 | alu_lane0_forward_b_0 | alu_lane1_forward_b_0 | load_forward_b_0;
        operand_ready_a_1 = queue_immediate_a_1 | ~queue_producer_a_1_valid | value_table_hit_a_1 | alu_lane0_forward_a_1 | alu_lane1_forward_a_1 | load_forward_a_1;
        operand_ready_b_1 = queue_immediate_b_1 | ~queue_producer_b_1_valid | value_table_hit_b_1 | alu_lane0_forward_b_1 | alu_lane1_forward_b_1 | load_forward_b_1;
//★ 档位顺序：load 前送放在【倒数第二档】，紧贴 table/base。各档比较的是"这个槽是谁写的"，
//  一个槽只属于一条队列项、只有一个单元会写它 ⇒ 各档互斥，换顺序不改行为。
//  而 load 的数据到得最晚（dcache 的 Q 端），放最后意味着它后面只剩一级 mux。
        operand_a_0 = queue_immediate_a_0 ? queue_immediate_value_a_0 :
                      alu_lane0_forward_a_0 ? data_alu_early :
                      alu_lane1_forward_a_0 ? data_alu_lane1_early :
                      load_forward_a_0 ? data_ld_early :
                      queue_producer_a_0_valid ? value_table_read_a_0 : queue_base_a_0;
        operand_b_0 = queue_immediate_b_0 ? queue_immediate_value_b_0 :
                      alu_lane0_forward_b_0 ? data_alu_early :
                      alu_lane1_forward_b_0 ? data_alu_lane1_early :
                      load_forward_b_0 ? data_ld_early :
                      queue_producer_b_0_valid ? value_table_read_b_0 : queue_base_b_0;
        operand_a_1 = queue_immediate_a_1 ? queue_immediate_value_a_1 :
                      alu_lane0_forward_a_1 ? data_alu_early :
                      alu_lane1_forward_a_1 ? data_alu_lane1_early :
                      load_forward_a_1 ? data_ld_early :
                      queue_producer_a_1_valid ? value_table_read_a_1 : queue_base_a_1;
        operand_b_1 = queue_immediate_b_1 ? queue_immediate_value_b_1 :
                      alu_lane0_forward_b_1 ? data_alu_early :
                      alu_lane1_forward_b_1 ? data_alu_lane1_early :
                      load_forward_b_1 ? data_ld_early :
                      queue_producer_b_1_valid ? value_table_read_b_1 : queue_base_b_1;
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
        csr_held_0 = (queue_csr_address_0 != 12'd0) & (queue_csr_function3_0 != 3'd0) & ~head_one_hot_registered[queue_index_0];
        csr_held_1 = (queue_csr_address_1 != 12'd0) & (queue_csr_function3_1 != 3'd0) & ~head_one_hot_registered[queue_index_1];
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
        csr_address_0  = issue_valid_0 ? queue_csr_address_0 : 12'd0;
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
        csr_address_1  = issue_valid_1 ? queue_csr_address_1 : 12'd0;
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
            queue_producer_a_0_valid <= 1'b0;
            queue_producer_b_0_valid <= 1'b0;
            queue_producer_a_0_slot <= 4'd0;
            queue_producer_b_0_slot <= 4'd0;
            queue_producer_a_0_generation <= 1'b0;
            queue_producer_b_0_generation <= 1'b0;
            queue_base_a_0 <= 32'd0;
            queue_base_b_0 <= 32'd0;
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
            queue_base_a_0 <= commit_at_queue_load_a_0 ? commit_data_at_queue_load_a_0 : (decode_new_0 ? decode_base_next_a_0 : db_a_0);
            queue_base_b_0 <= commit_at_queue_load_b_0 ? commit_data_at_queue_load_b_0 : (decode_new_0 ? decode_base_next_b_0 : db_b_0);
            queue_branch_prediction_0 <= branch_prediction_taken_in;
            queue_producer_a_0_valid <= to_queue_producer_a_0_valid & ~commit_at_queue_load_a_0;
            queue_producer_b_0_valid <= to_queue_producer_b_0_valid & ~commit_at_queue_load_b_0;
            queue_producer_a_0_slot <= to_queue_producer_a_0_slot;
            queue_producer_b_0_slot <= to_queue_producer_b_0_slot;
            queue_producer_a_0_generation <= to_queue_producer_a_0_generation;
            queue_producer_b_0_generation <= to_queue_producer_b_0_generation;
        end
        else if (issue_valid_0 | kill_0) begin
            queue_valid_0 <= 1'b0;
        end
        else begin
            queue_producer_a_0_valid <= queue_producer_a_0_valid & ~commit_hit_a_0;
            queue_producer_b_0_valid <= queue_producer_b_0_valid & ~commit_hit_b_0;
//★ base 只被"生产者提交"改写，且直接吃提交口的数据（口 B 更年轻，同 rd 它赢）。
//  读口这一拍跟的是 D 的 rs（为下一条预取），拿这里去刷新 base 会读到别的寄存器的值。
            if (commit_hit_a_0) queue_base_a_0 <= commit_port_b_source_a_lane0 ? commit_data1 : commit_data0;
            if (commit_hit_b_0) queue_base_b_0 <= commit_port_b_source_b_lane0 ? commit_data1 : commit_data0;
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
            queue_producer_a_1_valid <= 1'b0;
            queue_producer_b_1_valid <= 1'b0;
            queue_producer_a_1_slot <= 4'd0;
            queue_producer_b_1_slot <= 4'd0;
            queue_producer_a_1_generation <= 1'b0;
            queue_producer_b_1_generation <= 1'b0;
            queue_base_a_1 <= 32'd0;
            queue_base_b_1 <= 32'd0;
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
            queue_branch_jump_function4_1 <= decoded_branch_jump_function4_lane1;
            queue_base_a_1 <= commit_at_queue_load_a_1 ? commit_data_at_queue_load_a_1 : (decode_new_1 ? decode_base_next_a_1 : db_a_1);
            queue_base_b_1 <= commit_at_queue_load_b_1 ? commit_data_at_queue_load_b_1 : (decode_new_1 ? decode_base_next_b_1 : db_b_1);
            queue_branch_prediction_1 <= branch_prediction_taken_in_lane1;
            queue_producer_a_1_valid <= to_queue_producer_a_1_valid & ~commit_at_queue_load_a_1;
            queue_producer_b_1_valid <= to_queue_producer_b_1_valid & ~commit_at_queue_load_b_1;
            queue_producer_a_1_slot <= to_queue_producer_a_1_slot;
            queue_producer_b_1_slot <= to_queue_producer_b_1_slot;
            queue_producer_a_1_generation <= to_queue_producer_a_1_generation;
            queue_producer_b_1_generation <= to_queue_producer_b_1_generation;
        end
        else if (issue_valid_1 | kill_1) begin
            queue_valid_1 <= 1'b0;
        end
        else begin
            queue_producer_a_1_valid <= queue_producer_a_1_valid & ~commit_hit_a_1;
            queue_producer_b_1_valid <= queue_producer_b_1_valid & ~commit_hit_b_1;
            if (commit_hit_a_1) queue_base_a_1 <= commit_port_b_source_a_lane1 ? commit_data1 : commit_data0;
            if (commit_hit_b_1) queue_base_b_1 <= commit_port_b_source_b_lane1 ? commit_data1 : commit_data0;
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
            queue_producer_a_mem_valid <= 1'b0;
            queue_producer_b_mem_valid <= 1'b0;
            queue_producer_a_mem_slot <= 4'd0;
            queue_producer_b_mem_slot <= 4'd0;
            queue_producer_a_mem_generation <= 1'b0;
            queue_producer_b_mem_generation <= 1'b0;
            queue_base_a_mem <= 32'd0;
            queue_base_b_mem <= 32'd0;
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
//base 就在这里读（regfile 第 5/6 口）—— 访存条还没退休 ⇒ 架构值里只可能有"比它老"的写者，
//所以任何时候读都等于"它该看到的值"，不需要随指令走快照。
            queue_base_a_mem <= commit_at_queue_load_a_mem ? commit_data_at_queue_load_a_mem : decode_base_next_a_mem;
            queue_base_b_mem <= commit_at_queue_load_b_mem ? commit_data_at_queue_load_b_mem : decode_base_next_b_mem;
//tag 随条目走过来的，直接用。
            queue_producer_a_mem_valid <= mem_producer_a_valid_in & ~commit_at_queue_load_a_mem;
            queue_producer_b_mem_valid <= mem_producer_b_valid_in & ~commit_at_queue_load_b_mem;
            queue_producer_a_mem_slot <= mem_producer_a_slot_in;
            queue_producer_b_mem_slot <= mem_producer_b_slot_in;
            queue_producer_a_mem_generation <= mem_producer_a_generation_in;
            queue_producer_b_mem_generation <= mem_producer_b_generation_in;
        end
        else if (mem_issue_valid | kill_mem) begin
            queue_valid_mem <= 1'b0;
        end
        else begin
            queue_producer_a_mem_valid <= queue_producer_a_mem_valid & ~commit_hit_a_mem;
            queue_producer_b_mem_valid <= queue_producer_b_mem_valid & ~commit_hit_b_mem;
            if (commit_hit_a_mem) queue_base_a_mem <= commit_port_b_source_a_mem ? commit_data1 : commit_data0;
            if (commit_hit_b_mem) queue_base_b_mem <= commit_port_b_source_b_mem ? commit_data1 : commit_data0;
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            for (pi = 0; pi < 32; pi = pi + 1) begin
                producer_table_valid[pi] <= 1'b0;
                producer_table_slot[pi] <= 4'd0;
                producer_table_generation[pi] <= 1'b0;
            end
        end
        else begin
            for (pi = 0; pi < 32; pi = pi + 1) begin
                producer_table_valid[pi] <= producer_next_valid[pi];
                producer_table_slot[pi] <= producer_next_slot[pi];
                producer_table_generation[pi] <= producer_next_generation[pi];
            end
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            decode_seen_valid_0 <= 1'b0;
            decode_seen_valid_1 <= 1'b0;
            decode_seen_index_0 <= 4'd0;
            decode_seen_index_1 <= 4'd0;
            decode_seen_generation_0 <= 1'b0;
            decode_seen_generation_1 <= 1'b0;
            dead_registered_0 <= 1'b0;
            dead_registered_1 <= 1'b0;
        end
        else begin
            if (decode_new_0) begin
                decode_seen_valid_0 <= 1'b1;
                decode_seen_index_0 <= index_in;
                decode_seen_generation_0 <= alloc_generation_in;
            end
            if (decode_new_1) begin
                decode_seen_valid_1 <= 1'b1;
                decode_seen_index_1 <= index_in_lane1;
                decode_seen_generation_1 <= alloc_generation_in_lane1;
            end
            if (kill_all | flush_control_rob_registered) begin
//★ 冲刷那一拍，D 里这条**不一定死**：两条 lane 独立推进 ⇒ 它可以比冲刷点老
//  （lane0 跑到跳转并判定，lane1 那条还卡在 D 后面等它的生产者）。只有比冲刷点年轻的才死。
//  死/活与"是不是同一条"必须分开记：死是年龄判据，身份只用来把标志挂到那一条身上。
//  ★ 判据与 d_kill 逐字同一个式子（都吃 cont 的寄存版冲刷记录）。挂在 flush_control_rob_registered 上而不是
//  flush_window 上：记录晚一拍才到，挂在当拍会拿一个还没到的边界去判 ⇒ 只置成 kill_all。
                dead_registered_0 <= decode_kill_0;
                dead_registered_1 <= decode_kill_1;
            end
            else begin
//这条被消费掉就把它身上的"死"标记清掉；这一 lane 手上【根本没有活指令】（stage_advance=0）时也清 ——
//  那时 D 里没有东西可以被杀，标志留着只会误杀下一个住户（实测 seed3：lane0 空了两拍，
//  标志没清，新分配的 idx3 一进来就被当死的丢掉，ROB 满在 16、队头 e3 永不完成）。
                if (~stage_advance  | accept_0) dead_registered_0 <= 1'b0;
                if (~stage_advance_lane1 | accept_1) dead_registered_1 <= 1'b0;
            end
        end
    end

endmodule
