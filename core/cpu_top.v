`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: cpu_top
// Description:
//   顺序双发射核的顶层：前端（pc / bpu / icache / iffu / itcm）+ 译码（idu1）
//   + 发射（idu2，两条 lane 各一级 D→Q）+ 重排序缓冲（rob）+ 每 lane 一套
//   alu / mdu / bju + 共享 lsu + 控制（cont，内含 csr）+ 寄存器堆 + 数据侧存储。
//
//   本层不做运算，只连线。允许的两类例外：
//     ① 拍差换算（cur_addr 的 bti 偏置、dcache 的握手直连），它们都是"哪一根线"的问题；
//     ② 两条 lane 的合流（前端回报 / 乘法结果口 / CSR 写口）—— 选择码由 cont 给（bju_fe_sel），
//        其余按"有事件的那条优先"，逐条写明。
//
//   流水线（一拍一级）：
//     F  iffu 出队格 h0/h1 ──► idu1 的 S 级（D）
//     D  idu1 的 S 级寄存器  ──► idu2 的 D 端（译码、rob 分配、生产者表登记）
//     Q  idu2 的 Q 寄存器    ──► 单元（发射判据在 Q 端组合，发出那一拍锁字段）
//     W  单元结果口（寄存）  ──► rob 的完成回填；rob 按队头提交
//   相位契约见 docs/spec_idu2_v2.md 第三节。
//////////////////////////////////////////////////////////////////////////////////

module cpu_top #(
//★★ 宏化旋钮：一份源码切档。默认值 = 大核（交付配置，行为与原树逐位相同）。
    parameter integer LANES             = 1,    // 2 = 双发 / 1 = 单发
    parameter integer ROB_DEPTH         = 8,   // 16 / 8（必须是 2 的幂）
    parameter integer IFFU_DEPTH        = 4,    // 8 / 4
    parameter integer VALUE_TABLE_DEPTH = 8,   // 必须 = ROB_DEPTH
    parameter integer IC_INDEX_B        = 7,    // 8 = 64KB / 7 = 32KB
    parameter integer BHT_B             = 9,   // 11 / 9
    parameter integer BTB_B             = 9,   // 11 / 9
    parameter integer DC_SETS_B         = 9,   // 8 = 16KB / 7 = 8KB
    parameter integer ITCM_AW           = 14,  // 14 = 64KB / 13 = 32KB
    parameter integer ADDER_AT_D        = 1     // 1 = 访存地址加法搬到 D 端
) (
    input clk, rst,
    input [31:0] bus_data_in_ext,
    input bus_loaded_in, bus_hold_in, exti,
    output reg [31:0] bus_addr_out, bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out, bus_valid_out,

//---------------- L2 的下一级存储口（AHB-Lite 主机，接到 SoC 总线上）----------------
    output     [31:0] l2_haddr, l2_hwdata,
    output            l2_hwrite,
    output     [1:0]  l2_htrans,
    output     [2:0]  l2_hsize,
    output     [3:0]  l2_hwstrb,
    input      [31:0] l2_hrdata,
    input             l2_hready,
    output            l2_busy
    );

//==============================================================================
// 前端
//==============================================================================
    wire [31:0] pc_address_0, pc_address_1;
//测试台接口：几乎每个 tb 都用 dut.address_pc 当"取指 PC 活动窗口"的判据（这是原 pc 模块的 pc_addr 口，
//  双 lane 改造后改成了 pc_address_0/pc_address_1）。这里只做一次别名，不进任何逻辑；综合时无人读会被自动裁掉。
    wire [31:0] address_pc = pc_address_0;
    reg  [31:0] current_address_0, current_address_1;
    wire [31:0] instruction_out_icache, instruction_next_icache, instruction_addr0_icache, instruction_addr1_icache;
    wire        instruction_valid_icache, instruction_valid_n_icache, icache_miss_icache, busy_icache;
//★ 取指级的四根公共量（全部由 icache 一处产生 / 持有）：
//  adv        = 这一拍整个取指级前进一格（pc / R0 / 交付 / 落点表输出 全吃它）
//  r0_v/r0_wp = R0 那一对有效 / 是错路（pc 的 tk_ok = r0_v & ~r0_wp）
//  dwp        = 交付口这一对是错路（iffu 的 slot_rep 直接吃）
    wire        advance_icache, r0_valid_icache, r0_wrong_path_icache, deliver_wrong_path_icache;
    wire [31:0] mem_address_icache, mem_wdata_icache, mem_data_icache;
    wire [3:0]  mem_byte_enable_icache;
    wire        mem_req_icache, mem_write_enable_icache, mem_valid_icache;

    wire        br2, br3;
//★ 前端不再译 br/jal/jalr：判定与译码统一在 bju，落点表由它写。
//  前端只剩"取指 + 交付"两件事。
//★ BTB（落点表）输出：与【交付口】同拍到齐（都吃同一根 pc_addr、都寄存一拍）。
//  btb_take_0/1 已在 bpu 内部按 addr0 优先级仲裁过，恒互斥；btb_tgt 同时就是这条 jalr 的
//  "预测落点"（跟指令进队列、交给 bju 比对）。
    wire        btb_take_0, btb_take_1;
    wire [31:0] btb_target_0, btb_target_1;
//pc 这一拍实际照做的那一位 = 落点表命中 且 上一拍真读了。记录进队列的"预测跳"就是它，
//由 pc 自己出，保证"照做的"和"记账的"结构上是同一根线。
    wire        pc_take0, pc_take1;

//前端"这一拍有冲刷发生"（交付/边带都要按它作废）。★ 提前声明：边带两级都要清。
//  （flag_bus / flush_pc_redir 的声明也一并提前到这里 —— 它们由下面的 cont 例化驱动，
//    提前的只是【名字】，不影响谁驱动。）
    wire [7:0]  flag_bus, flag_bus_registered;
    wire        flush_pc_redir;
//bju 判定延迟级的训练载荷（btb 的唯一学习源），bju_fe_sel 做年龄二选一
    reg  [1:0]  fetch_kind;
    reg         fetch_taken;
    reg  [31:0] fetch_pc, fetch_target;

//★ 随指令记进队列的"预测跳 / 预测落点 / 补位"必须跟着【这一对指令】走。三条相位口径：
//  ① 这些量的 D 端（pc_take / btb_tgt）描述的是【这一拍压在第 1 级里的那一对】；
//  ② 交付寄存器在【同一沿】把它读出去 ⇒ 这里的装载使能必须和 icache 的 deliv_load 逐字相同
//     （fetch_ok & ~fifo_full）；
//  ③ 队满把这一对按住时使能是 0 ⇒ 这些寄存器一起按住，放行之后描述的仍是同一对。
//  ★ 曾经用 deliv_new_ic 当使能：deliv_new 比 deliv_load 晚一拍，于是记下的是【下一对】的
//    预测 —— 对 pc_take 就变成"把刚跳走的那一对当错路扔掉"（实测 seed8：0x2fc 的 store
//    被整对丢弃，签名第 15 个字写不进去）；对 push_br_pred 就变成"记录的不是 pc 照做的那一份"。
//  ★★ 实测口径（按地址对拍，见下面 pr0 那段）：真正与交付口逐对对齐的是 pr0_* ——
//    所以 iffu 吃的就是 pr0/jp0，本文件不再保留第二级 pr_*。
//★ "这一对是错路" 同样是【跟着这一对走】的边带，而且必须晚一对对齐：
//  上一对被预测改向（且那一拍真的读了顺延地址）⇒ 这一对是顺延取回来的错路。
//  做法与 pr_l0 同款：只在 deliv_new 那一拍（= 真的来了一对新指令）推进，
//  队满把这一对按住时它们不动 —— 放行之后描述的仍是同一对。
//  ★ 为什么不能普通打一拍：队满时 icache 会把同一对按住好几拍，打一拍的标记会错位到下一对上去
//    （实测：CoreMark 在 _start 处取指停摆 4488 拍、第 53 个字符就错）。
//  ★★ 冲刷必须把它们清掉：冲刷之后 pc 是【组合旁路】到落点的，下一对交付就是从落点取回来的
//    真指令 —— 而"上一对是错路"的标记还挂在寄存器里的话，会把这一对好指令整对扔掉。
//  （这两个寄存器放在下面 cont 的 flag_bus / flush_pc_redir 声明之后，见「前端：取指地址」段。）

    wire [31:0] h0_instruction, h0_address, h0_jump_link_register_prediction, h1_instruction, h1_address, h1_jump_link_register_prediction;
    wire        h0_valid, h1_valid;
    wire        h0_branch_prediction, h1_branch_prediction, full_iffu;

//★★ 预测不再是另打一拍的旁路，它就是这一对指令的延长位：
//  icache 在【交付寄存器装载的那一个沿】把 pc_take/btb_tgt 一起装进同一个寄存器组
//  （见 icache 的 inst_out 那一块），所以这里直接吃交付口的输出即可 ——
//  本文件不再需要 pr0/pr_l/jp0/jp_l/fz_d/tk_en 这一整套相位补偿。
//  原来为什么错：pr0 打一拍、pr_l 再打一拍，而 iffu 吃的 pr_l 在同一个沿上采的是
//  pr0 的【旧值】⇒ 记录的是【上一对】的预测（实测 CoreMark：pc 按落点表跳了，
//  队列条目里 br_pred 却是 0 ⇒ bju 的 br3 永不发 ⇒ bne 该退出不退出，死循环）。
    wire        dpred0_icache, dpred1_icache;
    wire [31:0] djpred0_icache, djpred1_icache;
//==============================================================================
// 译码级（idu1）→ 分配
//==============================================================================
    wire [1:0]  advance_idu1;
    wire        lane_valid_0, lane_valid_1, accept_0, accept_1;
    wire        credit_stall_idu1;

    wire        alloc_enable_0, alloc_enable_1, alloc_write_enable_0, alloc_write_enable_1;
    wire        alloc_jmp_0, alloc_jmp_1, alloc_pending_0, alloc_pending_1;
//分配拍的 SYSTEM 类译码（与 alloc_jmp 同相）—— 给 ROB 判"退休要不要等判定延迟级"
    wire        alloc_ecall_0, alloc_ebreak_0, alloc_illegal_0, alloc_mret_0;
    wire        alloc_ecall_1, alloc_ebreak_1, alloc_illegal_1, alloc_mret_1;
    wire [4:0]  alloc_destination_register_0, alloc_destination_register_1;
    wire [3:0]  alloc_index_0, alloc_index_1;
    wire        alloc_generation_0, alloc_generation_1;

    wire [31:0] instruction_out_0, instruction_out_1, auxiliary_address_out_0, auxiliary_address_out_1;
    wire [4:0]  rs1_idu1_0, rs2_idu1_0, rs1_idu1_1, rs2_idu1_1;
    wire [3:0]  index_out_0, index_out_1;
    wire        generation_out_0, generation_out_1, branch_prediction_taken_out_0, branch_prediction_taken_out_1;

    wire [1:0]  decoded_class_0, decoded_class_1;
    wire        decoded_write_enable_0, decoded_write_enable_1, decoded_is_store_0, decoded_is_store_1;
    wire [4:0]  decoded_destination_register_0, decoded_destination_register_1;
    wire [2:0]  decoded_mulop_0, decoded_mulop_1, decoded_load_store_function3_0, decoded_load_store_function3_1;
    wire [3:0]  decoded_func4_0, decoded_func4_1;
    wire [2:0]  decoded_csr_function3_0, decoded_csr_function3_1;
    wire        decoded_csr_write_enable_0, decoded_csr_write_enable_1;
    wire        decoded_csr_write_write_0, decoded_csr_write_write_1;
    wire [11:0] decoded_csr_address_0, decoded_csr_address_1;
    wire [31:0] decoded_jump_value_0, decoded_jump_value_1;
    wire        decoded_branch_flag_0, decoded_branch_flag_1, decoded_jump_link_flag_0, decoded_jump_link_flag_1, decoded_jump_link_register_flag_0, decoded_jump_link_register_flag_1;
    wire [3:0]  decoded_bjuf4_0, decoded_bjuf4_1;
    wire        decoded_r1_immediate_0, decoded_r1_immediate_1, decoded_r2_immediate_0, decoded_r2_immediate_1;
    wire [31:0] decoded_r1_immediate_value_0, decoded_r1_immediate_value_1, decoded_r2_immediate_value_0, decoded_r2_immediate_value_1;

//==============================================================================
// 发射级（idu2）↔ 单元
//==============================================================================

    wire        bypass_load_a_0, bypass_load_b_0, bypass_load_a_1, bypass_load_b_1;
    wire        issue_valid_0, issue_valid_1, write_enable_0, write_enable_1;
    wire [4:0]  destination_register_0, destination_register_1;
    wire [3:0]  issue_index_0, issue_index_1;
    wire        issue_generation_0, issue_generation_1;
    wire [3:0]  alu_func4_0, alu_func4_1;
    wire [2:0]  csr_func3_0, csr_func3_1;
    wire        csr_write_enable_0, csr_write_enable_1, csr_write_action_0, csr_write_action_1;
    wire [11:0] csr_address_0, csr_address_1;
    wire [31:0] auxiliary_address_0, auxiliary_address_1, beq_offset_0, beq_offset_1;
    wire [31:0] jump_link_register_prediction_address_0, jump_link_register_prediction_address_1, jump_link_target_0, jump_link_target_1;
    wire [31:0] operand_data_a_0, operand_data_b_0, operand_data_a_1, operand_data_b_1;
    wire        branch_flag_0, branch_flag_1, jump_link_flag_0, jump_link_flag_1, jump_link_register_flag_0, jump_link_register_flag_1;
    wire        branch_prediction_taken_0, branch_prediction_taken_1;
    wire [3:0]  bju_func4_0, bju_func4_1;
    wire        mul_go_0, mul_go_1, div_go_0, div_go_1;
    wire [9:0]  function10_0, function10_1, function10_load_store_0, function10_load_store_1;

    wire        lsu_valid, lsu_lane, lsu_generation;
    wire [6:0]  lsu_opcode;
    wire [9:0]  lsu_function10;
    wire [4:0]  lsu_destination_register, lsu_rs_a, lsu_rs_b;
    wire [3:0]  lsu_index;
    wire [31:0] lsu_pc;
    wire [31:0] lsu_a, lsu_b, lsu_imm;
//在途 load 的 rd（LSU → idu2）：让发射级挡住"操作数正被在途 load 写"的访存
    wire        load_hazard_valid_0, load_hazard_valid_1;
    wire [4:0]  load_hazard_rd_0, load_hazard_rd_1;

    wire [31:0] head_pc_rob;
    wire        head_valid_rob, issue_pop_0, issue_pop_1;
    wire [31:0] rdata1, rdata2, rdata3, rdata4, rdata5, rdata6;
//---- 访存专用流：idu1 的 LSU FIFO <-> idu2 ----
    wire [31:0] lsu_instruction_idu1, lsu_auxiliary_address_idu1, lsu_immediate_idu1;
    wire [4:0]  lsu_source_a_idu1, lsu_source_b_idu1, lsu_destination_register_idu1;
    wire [4:0]  lsu_push_source_a_idu1, lsu_push_source_b_idu1, lsu_push_destination_idu1;
    wire [2:0]  lsu_load_store_function3_idu1;
    wire [3:0]  lsu_index_idu1, lsu_push_index_idu1;
    wire        lsu_is_store_idu1, lsu_write_enable_idu1, lsu_generation_idu1, lsu_valid_idu1;
    wire        lsu_producer_a_valid_idu1, lsu_producer_a_generation_idu1;
    wire [3:0]  lsu_producer_a_slot_idu1;
    wire        lsu_producer_b_valid_idu1, lsu_producer_b_generation_idu1;
    wire [3:0]  lsu_producer_b_slot_idu1;
    wire        lsu_pop_idu2, lsu_push_valid_idu1, lsu_push_generation_idu1;
    wire        payload_fresh_0, payload_fresh_1;
//进队那拍取好的生产者号，打包 {valid, slot[3:0], generation}
//  push0 = h0（lane0 与访存条共用同一份），push1 = h1（双发时才有）
    wire [5:0]  push0_prod_a_idu2, push0_prod_b_idu2;
    wire [5:0]  push1_prod_a_idu2, push1_prod_b_idu2;
//载荷随条目带进 idu2 的 tag
    wire [5:0]  prod_a_idu1_0, prod_b_idu1_0, prod_a_idu1_1, prod_b_idu1_1;
//h1 那一格的进队描述（rs1/rs2/rd/index/gen）
    wire        push1_valid_idu1, push1_generation_idu1, lsu_push_is_mem_idu1;
    wire [4:0]  push1_source_a_idu1, push1_source_b_idu1, push1_destination_idu1;
    wire [3:0]  push1_index_idu1;
//idu1 本拍 lane0 装没装：决定 lane1 的读口地址取队头 h0 还是第二格 h1
    wire        lane0_load;

//==============================================================================
// 单元结果口
//==============================================================================
    wire [31:0] source_index_alu_0, source_index_alu_1, csr_result_alu_0, csr_result_alu_1;
//前送表的写表用：7 个结果口的 D 端
    wire [31:0] source_index_alu_early_0, source_index_alu_early_1;
    wire [31:0] mul_data_early_0, mul_data_early_1;
    wire [31:0] div_data_early_0, div_data_early_1;
    wire [31:0] load_data_early_lsu;
//早唤醒用：7 个结果口的身份 D 端
    wire        write_enable_alu_early_0, write_enable_alu_early_1;
    wire [3:0]  index_alu_early_0, index_alu_early_1;
    wire        generation_alu_early_0, generation_alu_early_1;
    wire        mul_write_enable_early_0, mul_write_enable_early_1, div_write_enable_early_0, div_write_enable_early_1;
    wire [3:0]  mul_index_early_0, mul_index_early_1, div_index_early_0, div_index_early_1;
    wire        mul_generation_early_0, mul_generation_early_1, div_generation_early_0, div_generation_early_1;
    wire        load_write_enable_early_lsu, load_generation_early_lsu;
    wire [3:0]  load_index_early_lsu;
    wire [3:0]  index_alu_0, index_alu_1;
    wire        generation_alu_0, generation_alu_1, write_enable_alu_0, write_enable_alu_1;

    wire [31:0] mul_data_0, mul_data_1;
    wire [3:0]  mul_index_0, mul_index_1;
    wire        mul_generation_0, mul_generation_1, mul_write_enable_0, mul_write_enable_1;
//除法独立口
    wire [31:0] div_data_0, div_data_1;
    wire [3:0]  div_index_0, div_index_1;
    wire        div_generation_0, div_generation_1, div_write_enable_0, div_write_enable_1;
    wire        mul_ready_0, div_ready_0, mul_ready_1, div_ready_1;

//rob 本拍退休的两条 → lsu（退休过的项不许再按年龄作废）
    wire        ret0_valid_rob, ret1_valid_rob, ret0_generation_rob, ret1_generation_rob;
    wire [3:0]  ret0_index_rob, ret1_index_rob;

    wire [31:0] load_data_lsu;
    wire [3:0]  load_index_lsu;
    wire        load_generation_lsu, load_write_enable_lsu, carriage_free_lsu;
    wire        store_done_lsu; wire [3:0] store_index_lsu; wire store_generation_lsu;

// 两条 mdu 的结果口在 idu2 侧只有一条（早唤醒用；rob 侧两条都有）

//==============================================================================
// bju ×2
//==============================================================================
    wire        success_0, branch_fail_0, jump_link_register_fail_0, success_1, branch_fail_1, jump_link_register_fail_1;
    wire [31:0] jump_target_0, jump_target_1, jump_link_register_target_0, jump_link_register_target_1;
    wire [3:0]  bju_index_registered_0, bju_index_registered_1, bju_index_i_0, bju_index_i_1;
    wire        judged_0, judged_1, generation_registered_0, generation_registered_1;
    wire        exception_target_misalign_0, exception_target_misalign_1, jump_link_register_valid_0, jump_link_register_valid_1, branch_prediction_taken_registered_0, branch_prediction_taken_registered_1;
    wire [8:0]  branch_pc_index_0, branch_pc_index_1;
//分支自身完整 PC（jalr 落点回写落点表用），与 br_pc_idx 同源同拍
    wire [31:0] branch_pc_0, branch_pc_1;
//bju 的训练载荷（BTB 的唯一来源），与 br_pc_q 同沿寄存
    wire [1:0]  bju_kind_0, bju_kind_1;
    wire        bju_taken_0, bju_taken_1;
    wire [31:0] bju_pc_0, bju_pc_1, bju_target_0, bju_target_1;
    wire [31:0] exception_pc_registered_0, exception_pc_registered_1, exception_pc_i_0, exception_pc_i_1;
//bju 判定延迟级的异常位（输入级的延迟版）
    wire        exception_ecall_registered_0, exception_ebreak_registered_0, exception_illegal_registered_0, exception_interrupt_ret_registered_0;
    wire        exception_ecall_registered_1, exception_ebreak_registered_1, exception_illegal_registered_1, exception_interrupt_ret_registered_1;
//异常标记口 ×3：lane0 / lane1 / lsu
    wire        exc0_enable, exc1_enable, excls_enable;
    wire [3:0]  exc0_index, exc1_index, excls_index;
    wire [3:0]  exc0_cause, exc1_cause, excls_cause;
    wire [31:0] exc0_pc, exc0_trap_value, exc1_pc, exc1_trap_value, excls_pc, excls_trap_value;
    wire        exception_ecall_0, exception_ecall_1, exception_ebreak_0, exception_ebreak_1, exception_illegal_0, exception_illegal_1, exception_interrupt_ret_0, exception_interrupt_ret_1;

//==============================================================================
// rob
//==============================================================================
    wire        commit_we0, commit_we1, commit_gen0, commit_gen1;
    wire [4:0]  commit_rd0, commit_rd1;
    wire [3:0]  commit_slot0, commit_slot1;
    wire [31:0] commit_data0, commit_data1;
    wire [3:0]  head_pointer;
    wire        rob_redir_valid, redir_registered_0, redir_registered_1, rob_redir_any;
    wire [31:0] rob_redir_pc;
    wire        rob_full, rob_empty, trap_fire;
    wire [15:0] above, above_registered, head_one_hot_registered;
    wire [1:0]  rob_ret_step;
    wire [4:0]  occupancy_rob;
    wire [3:0]  trap_cause;
    wire [31:0] trap_pc, trap_trap_value;

//==============================================================================
// cont / csr
//==============================================================================
    wire        flush_con_rob, flush_inclusive, flush_con_exception;
    wire [3:0]  flush_index;
//冲刷记录的寄存版（cont 出口打一拍）：ROB / idu1 / idu2 吃这一版，pc / flag_bus / bju / lsu / mdu
//仍吃当拍版 —— 前三个是"当拍必须改向/必须停手"，bju 的 keep_judge 判的就是冲刷这一拍，
//lsu 的丢弃必须比 ROB 更早（宁严勿松）。
    wire        flush_con_rob_registered, flush_inclusive_registered, flush_con_exception_registered, flush_pending;
    wire [3:0]  flush_index_registered;
    wire [3:0]  exception_cause;
    wire [31:0] exception_pc, exception_trap_value;
    wire [3:0]  exception_index;
    wire        redir_go, exception_interrupt_ret_ok;
    wire        mret_mark, mret_valid;
    wire [3:0]  mret_index;
    wire [31:0] mret_target;
    wire [31:0] redir_target;
    wire [31:0] isr_addr2, mcause, iret_addr2, csr_data_combinational;
    wire        exception_interrupt_action, exception_interrupt_processing, exception_interrupt;
    wire [3:0]  exception_ldst_index_lsu;
    wire [31:0] exception_ldst_address_lsu, exception_ldst_pc_lsu;
    wire        exception_ldst_misalign_lsu, exception_ldst_store_lsu;

//==============================================================================
// 数据侧存储
//==============================================================================
//★ 2026-10-31：LSU 的总线出口一分为二 —— dcache 独占一组（dcache_*），
//  AHB master / tim_in 用另一组（bus_*）。两组由 LSU 同时驱动，谁都不必为对方轮空。
    wire [31:0] bus_address_lsu, bus_data_lsu, data_dcache, data_tim;
    wire [3:0]  bus_byte_enable_lsu;
    wire        bus_write_enable_lsu, bus_valid_lsu;
    wire [31:0] dcache_address_lsu, dcache_data_lsu;
    wire [3:0]  dcache_byte_enable_lsu;
    wire        dcache_write_enable_lsu, dcache_valid_lsu;
    wire [31:0] mem_address_dcache, mem_wdata_dcache, mem_data_dtcm;
    wire [3:0]  mem_byte_enable_dcache;
    wire        mem_req_dcache, mem_write_enable_dcache, mem_valid_dtcm;
//L2 写穿通知 → icache 按行作废（一二级一致性）
    wire        l2_write_pulse;
    wire [31:0] l2_write_addr;
    wire        ready_tim, timer_interrupt;
    wire        hold_dcache, load_ready_dcache, load_ready_next_dcache, busy_dcache, request_blocked_dcache;
//load 早唤醒：载入那条在【地址进 dcache 那一拍】就宣告下一拍有数（load_ready_next 的当拍版）
    wire        load_wake_registered_lsu, load_wake_generation_lsu;
    wire [3:0]  load_wake_index_lsu;
    wire        load_wake_valid;
    assign load_wake_valid = load_wake_registered_lsu & load_ready_next_dcache;

//==============================================================================
// 两条 lane 的合流（前端回报：由 cont 给的选择码选；乘法结果口：谁回来用谁）
//==============================================================================
// 判定延迟拍那一套：success/br_fail/jalr 落点/预测表索引/落点 —— 必须是【同一条指令】的
//★ 2026-10-30：这里原来还有 fetch_success / fetch_branch_fail / fetch_jump_link_register_fail /
//  _valid / _target / fetch_brpc / fetch_brpc32 —— 全部【只有声明和赋值、没有任何读者】（grep 可验），
//  已删。bju_fetch_select 也随之无读者，连同 cont 的那个输出口一起删掉。
    reg         fetch_bpred;
    reg  [31:0] fetch_jump_link_register_target, fetch_jump_target;
    reg         csr_write_enable_m, csr_write_action_m;
    reg  [11:0] csr_address_m;
    reg  [31:0] csr_data_m;
//训练包：1 深缓冲（2026-10-30）
//★ 实测（tb_pred5，CoreMark 窗口 288,352 拍）：两 lane 同拍都有跳转载荷的拍 = 4,799，
//  占判定总数 61,731 的 7.8%。单写口每拍只收一条 ⇒ 这 4,799 次训练整条丢掉，
//  而那两条里 lane1 常常正是短循环体的条件分支。
//  这里加一个 1 深缓冲：撞拍时本拍送 lane0，lane1 那条压进缓冲、下一拍补送。
//★ 缓冲【优先】于 live（它更老）。只有在"缓冲里还压着一条、本拍又有两条 live"时才会再丢一条，
//  那要连着两拍撞车，概率比 1.66% 低一个数量级。
    reg        train_buf_valid;
    reg [1:0]  train_buf_kind;
    reg        train_buf_taken, train_buf_bpred;
    reg [31:0] train_buf_pc, train_buf_target;

    wire train_live_0 = (bju_kind_0 != 2'd0);
    wire train_live_1 = (bju_kind_1 != 2'd0);

//2'd0 = 缓冲, 2'd1 = lane0, 2'd2 = lane1, 2'd3 = 本拍无训练包
    reg [1:0] train_sel;
    always @(*) begin
        if      (train_buf_valid) train_sel = 2'd0;
        else if (train_live_0)    train_sel = 2'd1;
        else if (train_live_1)    train_sel = 2'd2;
        else                      train_sel = 2'd3;
    end

//本拍没被送出去的那条 live —— 压进缓冲（优先 lane1，与原来"lane0 赢"的口径一致）
    wire train_push       = ((train_sel != 2'd2) & train_live_1) | ((train_sel != 2'd1) & train_live_0);
    wire train_push_lane1 =  (train_sel != 2'd2) & train_live_1;

    always @(posedge clk) begin
        if (rst) begin
            train_buf_valid  <= 1'b0;
            train_buf_kind   <= 2'd0;
            train_buf_taken  <= 1'b0;
            train_buf_bpred  <= 1'b0;
            train_buf_pc     <= 32'd0;
            train_buf_target <= 32'd0;
        end
        else begin
            train_buf_valid <= train_push;
            if (train_push) begin
                train_buf_kind   <= train_push_lane1 ? bju_kind_1   : bju_kind_0;
                train_buf_taken  <= train_push_lane1 ? bju_taken_1  : bju_taken_0;
                train_buf_pc     <= train_push_lane1 ? bju_pc_1     : bju_pc_0;
                train_buf_target <= train_push_lane1 ? bju_target_1 : bju_target_0;
                train_buf_bpred  <= train_push_lane1 ? branch_prediction_taken_registered_1
                                                     : branch_prediction_taken_registered_0;
            end
        end
    end

    always @(*) begin
//★ 必须跟训练包用【同一个】选择器：bpu 里 branch_stage2/3 是
//     is_branch & branch_unit_taken & ~branch_prediction_taken_in
//  —— 一边是"训练包里的 taken"、另一边是"随指令记下的预测"。两边来自不同 lane 时这个比较是垃圾。
        fetch_bpred      = (train_sel == 2'd0) ? train_buf_bpred :
                           (train_sel == 2'd1) ? branch_prediction_taken_registered_0
                                               : branch_prediction_taken_registered_1;
        fetch_jump_target     = rob_redir_pc;   // S3：改向落点来自 ROB 队头那一项
//训练载荷同一条指令的年龄选择：必须在同一拍整体走一侧，不能一个吃 lane0、一个吃 lane1
//★★ 2026-10-29：训练包的选法与"前端回报包"分开。回报包按 branch_event（success/br_fail/jalr_fail/
//   异常）选，训练包按【谁真的带训练载荷】选（bju_kind != 0）。
//   为什么必须分开（实测）：0x14fc `j 12c4`、0x63c `j 53c` 这两条无条件跳转，判定侧数得出 312/272 次
//   （在 lane1 上判），但 `bpu.branch_unit_pc` 一次都没对上过它们 —— 因为 lane0 那拍只要有个事件
//   （哪怕 kind=0、根本不可训练），按 branch_event 的旧选法就把整包换成 lane0 的，lane1 的训练整条丢掉
//   ⇒ 这两条 `j` 的落点表永远写不进去 ⇒ 恒判 not-taken ⇒ 100% 漏跳（各 ~300 次判错）。
//   改成按 kind 选之后，lane0 没有训练载荷时 lane1 的训练才拿得到那一个写口。
//   ★ 2026-10-30：训练口仍然是【一个】，但撞拍时 lane1 那条不再丢 —— 见上面的 1 深训练缓冲。
        fetch_kind       = (train_sel == 2'd0) ? train_buf_kind :
                           (train_sel == 2'd1) ? bju_kind_0     : bju_kind_1;
        fetch_taken      = (train_sel == 2'd0) ? train_buf_taken :
                           (train_sel == 2'd1) ? bju_taken_0     : bju_taken_1;
        fetch_pc         = (train_sel == 2'd0) ? train_buf_pc :
                           (train_sel == 2'd1) ? bju_pc_0     : bju_pc_1;
        fetch_target     = (train_sel == 2'd0) ? train_buf_target :
                           (train_sel == 2'd1) ? bju_target_0     : bju_target_1;
//CSR 写口同理：两条同拍都是 CSR 指令时只落 lane0（并发 CSR 写不在本核口径内）
        csr_write_enable_m  = csr_write_enable_0 | csr_write_enable_1;
        csr_write_action_m = csr_write_action_0 | csr_write_action_1;
        csr_address_m   = csr_write_enable_1 ? csr_address_1 : csr_address_0;
        csr_data_m   = csr_write_enable_1 ? csr_result_alu_1 : csr_result_alu_0;
    end

//★ 2026-10-31：原来这里有个 exception_pc_in_mux（lsu_lane ? aux1 : aux0）给访存非对齐当 pc。
//  访存条走 idu1 的专用 FIFO，与 lane 的 auxiliary_address 早已脱钩（mt11：故障条在 0x30，
//  那一刻 aux0 冻在 0x18）⇒ 已删。pc 改由 idu2 随 LSU FIFO 排队送出（lsu_pc）。

//总线出口：本层只是把 lsu 的请求摆到芯片口上
    always @(*) begin
        bus_addr_out  = bus_address_lsu;
        bus_data_out  = bus_data_lsu;
        bus_be_out    = bus_byte_enable_lsu;
        bus_we_out    = bus_write_enable_lsu;
        bus_valid_out = bus_valid_lsu;
    end

//==============================================================================
// 前端：取指地址

    pc u_pc (
        .clk(clk), .rst(rst),
//★ pc 只吃：一根 adv + 两根改向量 + R0 的两个状态位。
//  不再自己解 flag_bus，也不再合成第二份"走不走"（全核只有 icache 的 adv 一份）。
        .advance(advance_icache),
        .flush_pc_redirect(flush_pc_redir),
        .r0_valid(r0_valid_icache), .r0_wrong_path(r0_wrong_path_icache),
        .btb_take_0(btb_take_0), .btb_target_0(btb_target_0),
        .btb_take_1(btb_take_1), .btb_target_1(btb_target_1),
        .jump_target(fetch_jump_target),
        .redirect_go(redir_go),
        .redir_target(redir_target),
        .pc_address_0(pc_address_0), .pc_address_1(pc_address_1),
        .pc_take0(pc_take0), .pc_take1(pc_take1)
    );

//==============================================================================
// 前端：预测器
//==============================================================================
    bpu #(.BHT_B(BHT_B), .BTB_B(BTB_B)) u_bpu (
        .clk(clk), .rst(rst),
        .advance(advance_icache),
//学习源只有 bju 一家：它同时知道"这是哪种跳转""它自己去哪""它自己的地址"
        .branch_unit_kind(fetch_kind), .branch_unit_taken(fetch_taken),
        .branch_unit_pc(fetch_pc), .bju_tgt(fetch_target),
        .branch_prediction_taken_in(fetch_bpred),
//训练门不再看 ROB 的在册改向（见 bpu.v 文件头 ③）
//读口：pc 的两个取指地址（addr0 优先级 > addr1，仲裁在 bpu 内做完）
        .pc_addr_0(pc_address_0), .pc_addr_1(pc_address_1),
//给 pc：落点
        .btb_take_0(btb_take_0), .btb_take_1(btb_take_1),
        .btb_target_0(btb_target_0),   .btb_target_1(btb_target_1),
        .branch_stage2(br2), .branch_stage3(br3)
    );

//==============================================================================
// 前端：指令缓存 + 指令紧耦合存储
//==============================================================================
    icache #(.IC_INDEX_B(IC_INDEX_B)) u_icache (
        .clk(clk), .rst(rst),
        .pc_address_0(pc_address_0), .pc_address_1(pc_address_1),
        .exception_interrupt(exception_interrupt), .exc_irq_ret(exception_interrupt_ret_ok),
        .exc_ecall(exception_ecall_0 | exception_ecall_1),
        .flush_control_in(flag_bus[7:5]), .stall_pc_redirect_in(flag_bus[2]), .stall_icache_miss_in(flag_bus[1]),
        .mem_valid(mem_valid_icache), .mem_data(mem_data_icache),
        .flush_pc_redirect(flush_pc_redir),
        .fifo_full(full_iffu),
        .instruction_out(instruction_out_icache), .instruction_next(instruction_next_icache),
        .instruction_address_0(instruction_addr0_icache), .instruction_address_1(instruction_addr1_icache),
//★ 预测随地址进 icache、随指令出 icache（就是这一对指令的延长位，不是另打一拍的旁路）
        .pc_take_0(pc_take0), .pc_take_1(pc_take1),
        .btb_target_0(btb_target_0), .btb_target_1(btb_target_1),
        .prediction_taken_0(dpred0_icache), .prediction_taken_1(dpred1_icache),
        .prediction_jump_target_0(djpred0_icache), .prediction_jump_target_1(djpred1_icache),
        .instruction_valid(instruction_valid_icache), .instruction_valid_next(instruction_valid_n_icache),
        .busy(busy_icache),
        .icache_miss(icache_miss_icache),
        .advance(advance_icache),
        .r0_valid(r0_valid_icache), .r0_wrong_path(r0_wrong_path_icache), .deliver_wrong_path(deliver_wrong_path_icache),
        .mem_req(mem_req_icache), .mem_write_enable(mem_write_enable_icache),
        .mem_address(mem_address_icache), .mem_write_data(mem_wdata_icache), .mem_byte_enable(mem_byte_enable_icache),
//一二级一致性：L2 被写穿 ⇒ 把命中同一行的两路作废
        .invalidate(l2_write_pulse), .invalidate_address(l2_write_addr)
    );

// itcm/dtcm 已由统一的二级存储 l2mem 取代（128KB，代码 0x0000 起 / 数据 0x4000 起）


//交付口上这条指令自己的地址：直接取 icache 交付时随数据出的 inst_addr。
//★ 取指地址与交付口之间的拍差只在这里换算一次。
    always @(*) begin
        current_address_0 = instruction_addr0_icache;
        current_address_1 = instruction_addr1_icache;
    end

//==============================================================================
// 前端：取指队列
//==============================================================================
    iffu #(.DEPTH(IFFU_DEPTH)) u_iffu (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]), .exec_in(flag_bus[4]),
        .flush_pc_redirect(flush_pc_redir),
        .instruction_in_0(instruction_out_icache), .inst_in_1(instruction_next_icache),
        .instruction_valid_0(instruction_valid_icache), .inst_valid_1(instruction_valid_n_icache),
//★ 必须和 push_br_pred_0 用同一个值：队满被按住的那几拍 pc_take0 是 0（那拍没有改向动作），
//  但那条分支【当时】确实跳了 —— 用现采的值会让 lane1（错路那条）被放进来（实测 +400 条/千轮）。
//  pr_l* 就是"随这一对指令走的那一份"，按住时它跟着一起按住。
        .lane0_take(dpred0_icache),
//★ 错路作废：icache 交付组里的 dwp 直接给 —— 它与指令同一个寄存器、同一个门，天然带地址。
//  顶层那套 pair_drop + pair_waddr（另建寄存器 + 地址判据）整段删掉了。
        .slot_replay(deliver_wrong_path_icache),
//★ 队列里存的地址是**"指令自己地址 + 4"**（全核 aux_addr 口径：jal/link/异常 pc 都靠它，
//  分支立即数在 idu1 里已经预减过 4）。icache 交付的是指令自己的地址 ⇒ 进队前在这里 +4。
        .push_address_0(current_address_0 + 32'd4), .push_addr_1(current_address_1 + 32'd4),
//★ 随指令记进队列的"预测跳"必须就是 pc 这一拍【实际照做】的那一个。
//  两份预测不一致的后果：表里没条目时 pc 顺下去、指令却被标成"预测跳"，真跳过时
//  br2 判"预测对了" ⇒ 永不改向 ⇒ 整核一路顺序跑（实测：3000 拍里一次改向都没有）。
        .push_branch_prediction_0(dpred0_icache),
        .push_br_pred_1(dpred1_icache),
        .push_jump_link_register_prediction_0(djpred0_icache),
        .push_jalr_pred_1(djpred1_icache),
        .advance(advance_idu1),
        .half0_instruction(h0_instruction), .half0_address(h0_address), .half0_jump_link_register_prediction(h0_jump_link_register_prediction),
        .half0_branch_prediction(h0_branch_prediction),
        .half1_instruction(h1_instruction), .half1_address(h1_address), .half1_jump_link_register_prediction(h1_jump_link_register_prediction),
        .half1_branch_prediction(h1_branch_prediction),
        .half0_valid(h0_valid), .half1_valid(h1_valid),
        .full(full_iffu)
    );

//==============================================================================
// 译码级
//==============================================================================
    idu1 #(.LANES(LANES)) u_idu1 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]), .exec_in(flag_bus[4]), .stall_rob_full_in(flag_bus[3]),
        .flush_pend(flush_pending),
//冲刷的年龄边界：输出级里挂着的那条比冲刷点老时不许清（见 idu1 里 keep_* 的注释）
//本树没有保留站：弹出量由发射级的 issue_pop 计（credit 走 acc_in）
        .store_pop({1'b0, issue_pop_0} + {1'b0, issue_pop_1}),
        .store_flush_pop(4'd0),
        .credit_stall_out(credit_stall_idu1),
        .advance(advance_idu1),
        .lane0_load_o(lane0_load),

        .instruction_in_0(h0_instruction), .address_in_0(h0_address), .instruction_valid_0(h0_valid),
        .branch_prediction_in_0(h0_branch_prediction), .jump_link_register_prediction_in_0(h0_jump_link_register_prediction),
        .index_in_0(alloc_index_0), .generation_in_0(alloc_generation_0),
//顺序发射：发射级在 Q 端等操作数，D 侧永远"就绪"
        .ready1_in_0(1'b1), .ready2_in_0(1'b1),
        .accept_in_0(accept_0),
        .lane_valid_0(lane_valid_0),
        .instruction_out_0(instruction_out_0), .auxiliary_address_out_0(auxiliary_address_out_0),
        .branch_prediction_taken_out_0(branch_prediction_taken_out_0),
        .source_a_out_0(rs1_idu1_0), .source_b_out_0(rs2_idu1_0),
        .index_out_0(index_out_0), .generation_out_0(generation_out_0),
        .payload_valid_out_0(),
        .alloc_en_0(alloc_enable_0), .alloc_we_0(alloc_write_enable_0),
        .alloc_jmp_0(alloc_jmp_0), .alloc_pend_0(alloc_pending_0),
        .alloc_ecall_0(alloc_ecall_0), .alloc_ebreak_0(alloc_ebreak_0),
        .alloc_illegal_0(alloc_illegal_0), .alloc_mret_0(alloc_mret_0),
        .alloc_rd_0(alloc_destination_register_0),
        .dec_class_0(decoded_class_0), .dec_we_0(decoded_write_enable_0), .dec_rd_0(decoded_destination_register_0),
        .dec_is_st_0(decoded_is_store_0), .dec_mulop_0(decoded_mulop_0), .dec_lsf3_0(decoded_load_store_function3_0),
        .dec_func4_0(decoded_func4_0), .dec_csrf3_0(decoded_csr_function3_0),
        .dec_csrw_en_0(decoded_csr_write_enable_0),
        .dec_csrw_wr_0(decoded_csr_write_write_0), .dec_csra_0(decoded_csr_address_0),
        .dec_jval_0(decoded_jump_value_0), .dec_brf_0(decoded_branch_flag_0),
        .dec_jalf_0(decoded_jump_link_flag_0), .dec_jalfr_0(decoded_jump_link_register_flag_0),
        .dec_bjuf4_0(decoded_bjuf4_0),
        .dec_r1_imm_0(decoded_r1_immediate_0), .dec_r2_imm_0(decoded_r2_immediate_0),
        .dec_r1_immv_0(decoded_r1_immediate_value_0), .dec_r2_immv_0(decoded_r2_immediate_value_0),

        .instruction_in_1(h1_instruction), .address_in_1(h1_address), .instruction_valid_1(h1_valid),
        .branch_prediction_in_1(h1_branch_prediction), .jump_link_register_prediction_in_1(h1_jump_link_register_prediction),
        .index_in_1(alloc_index_1), .generation_in_1(alloc_generation_1),
        .ready1_in_1(1'b1), .ready2_in_1(1'b1),
        .accept_in_1(accept_1),
        .lane_valid_1(lane_valid_1),
        .instruction_out_1(instruction_out_1), .auxiliary_address_out_1(auxiliary_address_out_1),
        .branch_prediction_taken_out_1(branch_prediction_taken_out_1),
        .source_a_out_1(rs1_idu1_1), .source_b_out_1(rs2_idu1_1),
        .index_out_1(index_out_1), .generation_out_1(generation_out_1),
        .payload_valid_out_1(),
        .alloc_en_1(alloc_enable_1), .alloc_we_1(alloc_write_enable_1),
        .alloc_jmp_1(alloc_jmp_1), .alloc_pend_1(alloc_pending_1),
        .alloc_ecall_1(alloc_ecall_1), .alloc_ebreak_1(alloc_ebreak_1),
        .alloc_illegal_1(alloc_illegal_1), .alloc_mret_1(alloc_mret_1),
        .alloc_rd_1(alloc_destination_register_1),
        .dec_class_1(decoded_class_1), .dec_we_1(decoded_write_enable_1), .dec_rd_1(decoded_destination_register_1),
        .dec_is_st_1(decoded_is_store_1), .dec_mulop_1(decoded_mulop_1), .dec_lsf3_1(decoded_load_store_function3_1),
        .dec_func4_1(decoded_func4_1), .dec_csrf3_1(decoded_csr_function3_1),
        .dec_csrw_en_1(decoded_csr_write_enable_1),
        .dec_csrw_wr_1(decoded_csr_write_write_1), .dec_csra_1(decoded_csr_address_1),
        .dec_jval_1(decoded_jump_value_1), .dec_brf_1(decoded_branch_flag_1),
        .dec_jalf_1(decoded_jump_link_flag_1), .dec_jalfr_1(decoded_jump_link_register_flag_1),
        .dec_bjuf4_1(decoded_bjuf4_1),
        .dec_r1_imm_1(decoded_r1_immediate_1), .dec_r2_imm_1(decoded_r2_immediate_1),
        .dec_r1_immv_1(decoded_r1_immediate_value_1), .dec_r2_immv_1(decoded_r2_immediate_value_1),
        .lsu_pop_in(lsu_pop_idu2),
        .payload_fresh_out_0(payload_fresh_0), .payload_fresh_out_1(payload_fresh_1),
        .write_enable_a_in(commit_we0), .write_enable_b_in(commit_we1),
        .commit_slot0_in(commit_slot0), .commit_slot1_in(commit_slot1),
        .commit_gen0_in(commit_gen0), .commit_gen1_in(commit_gen1),
        .commit_rd0_in(commit_rd0), .commit_rd1_in(commit_rd1),
        .lsu_push_prod_a(push0_prod_a_idu2), .lsu_push_prod_b(push0_prod_b_idu2),
        .push1_prod_a(push1_prod_a_idu2), .push1_prod_b(push1_prod_b_idu2),
        .lsu_push_valid(lsu_push_valid_idu1), .lsu_push_is_mem(lsu_push_is_mem_idu1),
        .lsu_push_source_a(lsu_push_source_a_idu1), .lsu_push_source_b(lsu_push_source_b_idu1),
        .lsu_push_destination(lsu_push_destination_idu1),
        .lsu_push_index(lsu_push_index_idu1),
        .lsu_push_generation(lsu_push_generation_idu1),
        .push1_valid(push1_valid_idu1),
        .push1_source_a(push1_source_a_idu1), .push1_source_b(push1_source_b_idu1),
        .push1_destination(push1_destination_idu1),
        .push1_index(push1_index_idu1),
        .push1_generation(push1_generation_idu1),
        .prod_a_out_0(prod_a_idu1_0), .prod_b_out_0(prod_b_idu1_0),
        .prod_a_out_1(prod_a_idu1_1), .prod_b_out_1(prod_b_idu1_1),
        .lsu_instruction_out(lsu_instruction_idu1),
        .lsu_auxiliary_address_out(lsu_auxiliary_address_idu1),
        .lsu_immediate_out(lsu_immediate_idu1),
        .lsu_source_a_out(lsu_source_a_idu1), .lsu_source_b_out(lsu_source_b_idu1),
        .lsu_destination_register_out(lsu_destination_register_idu1),
        .lsu_load_store_function3_out(lsu_load_store_function3_idu1),
        .lsu_is_store_out(lsu_is_store_idu1),
        .lsu_write_enable_out(lsu_write_enable_idu1),
        .lsu_index_out(lsu_index_idu1),
        .lsu_generation_out(lsu_generation_idu1),
        .lsu_valid_out(lsu_valid_idu1),
        .lsu_producer_a_valid_out(lsu_producer_a_valid_idu1),
        .lsu_producer_a_slot_out(lsu_producer_a_slot_idu1),
        .lsu_producer_a_generation_out(lsu_producer_a_generation_idu1),
        .lsu_producer_b_valid_out(lsu_producer_b_valid_idu1),
        .lsu_producer_b_slot_out(lsu_producer_b_slot_idu1),
        .lsu_producer_b_generation_out(lsu_producer_b_generation_idu1)
    );

//==============================================================================
// 发射级
//==============================================================================
    idu2 #(.VALUE_TABLE_DEPTH(VALUE_TABLE_DEPTH)) u_idu2 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]), .exec_in(flag_bus[4]),
        .flush_control_rob_registered(flush_con_rob_registered),
        .stage_advance(lane_valid_0), .stage_advance_lane1(lane_valid_1),

        .rs1_in(rs1_idu1_0), .rs2_in(rs2_idu1_0),
        .instruction_in(instruction_out_0),
        .pc_operand_in(auxiliary_address_out_0), .aux_addr_in(auxiliary_address_out_0),
        .branch_prediction_taken_in(branch_prediction_taken_out_0),
        .index_in(index_out_0), .alloc_generation_in(generation_out_0),
        .decoded_class(decoded_class_0), .decoded_write_enable(decoded_write_enable_0), .decoded_destination_register(decoded_destination_register_0),
        .decoded_is_st(decoded_is_store_0), .decoded_multiply_operation(decoded_mulop_0), .decoded_load_store_function3(decoded_load_store_function3_0),
        .decoded_function4(decoded_func4_0), .decoded_csr_function3(decoded_csr_function3_0),
        .decoded_csr_write_enable(decoded_csr_write_enable_0),
        .decoded_csr_write_commit(decoded_csr_write_write_0), .decoded_csr_address(decoded_csr_address_0),
        .decoded_jump_value(decoded_jump_value_0), .decoded_branch_flag(decoded_branch_flag_0),
        .decoded_jump_link_flag(decoded_jump_link_flag_0), .decoded_jump_link_register_flag(decoded_jump_link_register_flag_0),
        .decoded_branch_jump_function4(decoded_bjuf4_0),
        .decoded_r1_immediate(decoded_r1_immediate_0), .decoded_r2_immediate(decoded_r2_immediate_0),
        .decoded_r1_immediate_value(decoded_r1_immediate_value_0), .decoded_r2_immediate_value(decoded_r2_immediate_value_0),

        .rs1_in_lane1(rs1_idu1_1), .rs2_in1(rs2_idu1_1),
        .inst_in_lane1(instruction_out_1),
        .pc_operand_in1(auxiliary_address_out_1), .aux_addr_in1(auxiliary_address_out_1),
        .branch_prediction_taken_in_lane1(branch_prediction_taken_out_1),
        .index_in_lane1(index_out_1), .alloc_generation_in_lane1(generation_out_1),
        .decoded_class_lane1(decoded_class_1), .decoded_write_enable_lane1(decoded_write_enable_1), .decoded_destination_register_lane1(decoded_destination_register_1),
        .decoded_is_st_lane1(decoded_is_store_1), .decoded_multiply_operation_lane1(decoded_mulop_1), .decoded_load_store_function3_lane1(decoded_load_store_function3_1),
        .decoded_function4_lane1(decoded_func4_1), .decoded_csr_function3_lane1(decoded_csr_function3_1),
        .decoded_csr_write_enable_lane1(decoded_csr_write_enable_1),
        .decoded_csr_write_commit_lane1(decoded_csr_write_write_1), .decoded_csr_address_lane1(decoded_csr_address_1),
        .decoded_jump_value_lane1(decoded_jump_value_1), .decoded_branch_flag_lane1(decoded_branch_flag_1),
        .decoded_jump_link_flag_lane1(decoded_jump_link_flag_1), .decoded_jump_link_register_flag_lane1(decoded_jump_link_register_flag_1),
        .decoded_branch_jump_function4_lane1(decoded_bjuf4_1),
        .decoded_r1_immediate_lane1(decoded_r1_immediate_1), .decoded_r2_immediate_lane1(decoded_r2_immediate_1),
        .decoded_r1_immediate_value_lane1(decoded_r1_immediate_value_1), .decoded_r2_immediate_value_lane1(decoded_r2_immediate_value_1),

        .mem_rs1_in(lsu_source_a_idu1), .mem_rs2_in(lsu_source_b_idu1),
        .mem_destination_register_in(lsu_destination_register_idu1),
        .mem_auxiliary_address_in(lsu_auxiliary_address_idu1),
        .mem_immediate_in(lsu_immediate_idu1),
        .mem_rdata1_in(rdata5), .mem_rdata2_in(rdata6),
        .mem_index_in(lsu_index_idu1), .mem_generation_in(lsu_generation_idu1),
        .mem_is_store_in(lsu_is_store_idu1),
        .mem_load_store_function3_in(lsu_load_store_function3_idu1),
        .mem_valid_in(lsu_valid_idu1),
        .mem_producer_a_valid_in(lsu_producer_a_valid_idu1),
        .mem_producer_a_slot_in(lsu_producer_a_slot_idu1),
        .mem_producer_a_generation_in(lsu_producer_a_generation_idu1),
        .mem_producer_b_valid_in(lsu_producer_b_valid_idu1),
        .mem_producer_b_slot_in(lsu_producer_b_slot_idu1),
        .mem_producer_b_generation_in(lsu_producer_b_generation_idu1),
        .mem_pop_out(lsu_pop_idu2),
        .lane0_fresh_in(payload_fresh_0), .lane1_fresh_in(payload_fresh_1),
        .mem_push_valid(lsu_push_valid_idu1), .mem_push_is_mem(lsu_push_is_mem_idu1),
        .mem_push_rs1_in(lsu_push_source_a_idu1), .mem_push_rs2_in(lsu_push_source_b_idu1),
        .mem_push_destination_register_in(lsu_push_destination_idu1),
        .mem_push_index_in(lsu_push_index_idu1),
        .mem_push_generation_in(lsu_push_generation_idu1),
        .push1_valid_in(push1_valid_idu1),
        .push1_rs1_in(push1_source_a_idu1), .push1_rs2_in(push1_source_b_idu1),
        .push1_destination_register_in(push1_destination_idu1),
        .push1_index_in(push1_index_idu1),
        .push1_generation_in(push1_generation_idu1),
        .mem_push_prod_a_out(push0_prod_a_idu2), .mem_push_prod_b_out(push0_prod_b_idu2),
        .push1_prod_a_out(push1_prod_a_idu2), .push1_prod_b_out(push1_prod_b_idu2),
        .lane0_prod_a_in(prod_a_idu1_0), .lane0_prod_b_in(prod_b_idu1_0),
        .lane1_prod_a_in(prod_a_idu1_1), .lane1_prod_b_in(prod_b_idu1_1),

//结果口：只接 32 位数据 + 身份 D 端（原来的"身份 Q 端"一组 idu2 从没读过，已删）
        .data_alu_early(source_index_alu_early_0),
        .data_alu_lane1_early(source_index_alu_early_1),
        .data_mul_early(mul_data_early_0),
        .data_multiply_lane1_early(mul_data_early_1),
        .data_div_early(div_data_early_0),
        .data_divide_lane1_early(div_data_early_1),
        .data_ld_early(load_data_early_lsu),
        .write_enable_alu_early(write_enable_alu_early_0),   .index_alu_early(index_alu_early_0),   .generation_alu_early(generation_alu_early_0),
        .write_enable_alu_lane1_early(write_enable_alu_early_1),  .index_alu_lane1_early(index_alu_early_1),  .generation_alu_lane1_early(generation_alu_early_1),
        .write_enable_mul_early(mul_write_enable_early_0),   .index_mul_early(mul_index_early_0),   .generation_mul_early(mul_generation_early_0),
        .write_enable_multiply_lane1_early(mul_write_enable_early_1),  .index_multiply_lane1_early(mul_index_early_1),  .generation_multiply_lane1_early(mul_generation_early_1),
        .write_enable_div_early(div_write_enable_early_0),   .index_div_early(div_index_early_0),   .generation_div_early(div_generation_early_0),
        .write_enable_divide_lane1_early(div_write_enable_early_1),  .index_divide_lane1_early(div_index_early_1),  .generation_divide_lane1_early(div_generation_early_1),
        .write_enable_ld_early(load_write_enable_early_lsu),   .index_ld_early(load_index_early_lsu),   .generation_ld_early(load_generation_early_lsu),

        .write_enable_a(commit_we0), .destination_register_a(commit_rd0),
        .write_enable_b(commit_we1), .destination_register_b(commit_rd1),
        .commit_slot0(commit_slot0), .commit_slot1(commit_slot1),
        .commit_gen0(commit_gen0),  .commit_gen1(commit_gen1),
        .commit_data0(commit_data0), .commit_data1(commit_data1),

        .flush_inclusive_registered(flush_inclusive_registered), .flush_index_registered(flush_index_registered),
        .flush_pending(flush_pending),
        .trap_fire_registered(trap_fire), .head_one_hot_registered(head_one_hot_registered),
        .above(above_registered),

        .lsu_full(~carriage_free_lsu),   // 只看结构（访存车 / SQ）
        .load_wake_valid(load_wake_valid), .load_wake_index(load_wake_index_lsu), .load_wake_generation(load_wake_generation_lsu),
        .bypass_load_a_0(bypass_load_a_0), .bypass_load_b_0(bypass_load_b_0),
        .bypass_load_a_1(bypass_load_a_1), .bypass_load_b_1(bypass_load_b_1),
        .load_hazard_valid_0(load_hazard_valid_0), .load_hazard_rd_0(load_hazard_rd_0),
        .load_hazard_valid_1(load_hazard_valid_1), .load_hazard_rd_1(load_hazard_rd_1),
        .mdu_mul_ok_0(mul_ready_0), .mdu_div_ok_0(div_ready_0),
        .mdu_mul_ok_1(mul_ready_1), .mdu_div_ok_1(div_ready_1),
        .bju_ok_0(1'b1), .bju_ok_1(1'b1),

        .rdata1_in(rdata1), .rdata2_in(rdata2),
        .rdata3_in(rdata3), .rdata4_in(rdata4),

        .issue_valid_0(issue_valid_0), .destination_register_0(destination_register_0), .issue_index_0(issue_index_0),
        .issue_generation_0(issue_generation_0), .write_enable_0(write_enable_0), .sel_class_0(),
        .opcode_0(), .function10_0(function10_0), .function10_load_store_0(function10_load_store_0),
        .alu_function4_0(alu_func4_0), .csr_func3_0(csr_func3_0),
        .csr_write_enable_0(csr_write_enable_0), .csr_write_action_0(csr_write_action_0),
        .csr_address_0(csr_address_0),
        .auxiliary_address_0(auxiliary_address_0), .off_mem_0(), .beq_off_0(beq_offset_0),
        .jump_link_register_prediction_address_0(jump_link_register_prediction_address_0), .jump_link_target_0(jump_link_target_0),
        .operand_data_a_0(operand_data_a_0), .operand_data_b_0(operand_data_b_0),
        .source_register_a_0(), .source_register_b_0(),
        .branch_flag_0(branch_flag_0), .jump_link_flag_0(jump_link_flag_0), .jump_link_register_flag_0(jump_link_register_flag_0),
        .branch_prediction_taken_0(branch_prediction_taken_0),
        .bju_function4_0(bju_func4_0),
        .mul_go_0(mul_go_0), .div_go_0(div_go_0),

        .issue_valid_1(issue_valid_1), .destination_register_1(destination_register_1), .issue_index_1(issue_index_1),
        .issue_generation_1(issue_generation_1), .write_enable_1(write_enable_1), .sel_class_1(),
        .opcode_1(), .function10_1(function10_1), .function10_load_store_1(function10_load_store_1),
        .alu_function4_1(alu_func4_1), .csr_func3_1(csr_func3_1),
        .csr_write_enable_1(csr_write_enable_1), .csr_write_action_1(csr_write_action_1),
        .csr_address_1(csr_address_1),
        .auxiliary_address_1(auxiliary_address_1), .off_mem_1(), .beq_off_1(beq_offset_1),
        .jump_link_register_prediction_address_1(jump_link_register_prediction_address_1), .jump_link_target_1(jump_link_target_1),
        .operand_data_a_1(operand_data_a_1), .operand_data_b_1(operand_data_b_1),
        .source_register_a_1(), .source_register_b_1(),
        .branch_flag_1(branch_flag_1), .jump_link_flag_1(jump_link_flag_1), .jump_link_register_flag_1(jump_link_register_flag_1),
        .branch_prediction_taken_1(branch_prediction_taken_1),
        .bju_function4_1(bju_func4_1),
        .mul_go_1(mul_go_1), .div_go_1(div_go_1),

        .lsu_valid(lsu_valid), .lsu_lane(lsu_lane), .lsu_opcode(lsu_opcode), .lsu_function10(lsu_function10),
        .lsu_destination_register(lsu_destination_register), .lsu_index(lsu_index), .lsu_generation(lsu_generation),
        .lsu_pc(lsu_pc),
        .lsu_a(lsu_a), .lsu_b(lsu_b), .lsu_imm(lsu_imm),
        .lsu_source_register_a(lsu_rs_a), .lsu_source_register_b(lsu_rs_b),

        .accept_0(accept_0), .accept_1(accept_1),
        .issue_pop_0(issue_pop_0), .issue_pop_1(issue_pop_1)
    );

//==============================================================================
// 执行单元：每 lane 一套 alu / mdu / bju
//==============================================================================
    alu u_alu_0 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]),
        .write_enable_in(write_enable_0), .jal_flag(jump_link_flag_0), .jalr_flag(jump_link_register_flag_0),
        .cs_wr_en(csr_write_enable_0),
        .destination_in(destination_register_0), .index_in(issue_index_0), .generation_in(issue_generation_0),
        .alu_function4(alu_func4_0), .csr_function3(csr_func3_0),
        .auxiliary_address_in(auxiliary_address_0),
        .source_a_data(operand_data_a_0), .r2_data(operand_data_b_0), .cs_data(csr_data_combinational),
        .bypass_load_a(bypass_load_a_0), .bypass_load_b(bypass_load_b_0), .load_data_forward(load_data_early_lsu),
        .destination_out(), .index_out(index_alu_0), .generation_out(generation_alu_0),
        .result(source_index_alu_0), .result_csr(csr_result_alu_0),
        .result_now(), .result_early(source_index_alu_early_0),
        .write_enable_early(write_enable_alu_early_0), .index_early(index_alu_early_0), .generation_early(generation_alu_early_0),
        .write_enable(write_enable_alu_0)
    );

    alu u_alu_1 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]),
        .write_enable_in(write_enable_1), .jal_flag(jump_link_flag_1), .jalr_flag(jump_link_register_flag_1),
        .cs_wr_en(csr_write_enable_1),
        .destination_in(destination_register_1), .index_in(issue_index_1), .generation_in(issue_generation_1),
        .alu_function4(alu_func4_1), .csr_function3(csr_func3_1),
        .auxiliary_address_in(auxiliary_address_1),
        .source_a_data(operand_data_a_1), .r2_data(operand_data_b_1), .cs_data(csr_data_combinational),
        .bypass_load_a(bypass_load_a_1), .bypass_load_b(bypass_load_b_1), .load_data_forward(load_data_early_lsu),
        .destination_out(), .index_out(index_alu_1), .generation_out(generation_alu_1),
        .result(source_index_alu_1), .result_csr(csr_result_alu_1),
        .result_now(), .result_early(source_index_alu_early_1),
        .write_enable_early(write_enable_alu_early_1), .index_early(index_alu_early_1), .generation_early(generation_alu_early_1),
        .write_enable(write_enable_alu_1)
    );

    mdu u_mdu_0 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]),
        .multiply_go(mul_go_0), .multiply_function10(function10_0), .multiply_destination(destination_register_0),
        .multiply_index_in(issue_index_0), .multiply_generation_in(issue_generation_0),
        .multiply_source_a(operand_data_a_0), .mul_r2(operand_data_b_0),
        .divide_go(div_go_0), .divide_function10(function10_0), .divide_destination(destination_register_0),
        .divide_index_in(issue_index_0), .divide_generation_in(issue_generation_0),
        .divide_source_a(operand_data_a_0), .div_r2(operand_data_b_0),
        .multiply_data_out(mul_data_0), .multiply_loaded(mul_write_enable_0), .multiply_write_enable(),
        .multiply_data_early(mul_data_early_0), .divide_data_early(div_data_early_0),
        .multiply_write_enable_early(mul_write_enable_early_0), .multiply_index_early(mul_index_early_0), .multiply_generation_early(mul_generation_early_0),
        .divide_write_enable_early(div_write_enable_early_0), .divide_index_early(div_index_early_0), .divide_generation_early(div_generation_early_0),
        .rd_mul(), .multiply_index(mul_index_0), .multiply_generation(mul_generation_0),
        .divide_data_out(div_data_0), .divide_loaded(div_write_enable_0), .divide_write_enable(),
        .rd_div(), .divide_index(div_index_0), .divide_generation(div_generation_0),
        .multiply_ready(mul_ready_0), .divide_ready(div_ready_0),
        .multiply_wake_registered(), .multiply_wake_index_registered(), .multiply_wake_generation_registered(),
        .multiply_nack_registered(), .multiply_nack_index_registered(), .multiply_nack_generation_registered()
    );

    mdu u_mdu_1 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]),
        .multiply_go(mul_go_1), .multiply_function10(function10_1), .multiply_destination(destination_register_1),
        .multiply_index_in(issue_index_1), .multiply_generation_in(issue_generation_1),
        .multiply_source_a(operand_data_a_1), .mul_r2(operand_data_b_1),
        .divide_go(div_go_1), .divide_function10(function10_1), .divide_destination(destination_register_1),
        .divide_index_in(issue_index_1), .divide_generation_in(issue_generation_1),
        .divide_source_a(operand_data_a_1), .div_r2(operand_data_b_1),
        .multiply_data_out(mul_data_1), .multiply_loaded(mul_write_enable_1), .multiply_write_enable(),
        .multiply_data_early(mul_data_early_1), .divide_data_early(div_data_early_1),
        .multiply_write_enable_early(mul_write_enable_early_1), .multiply_index_early(mul_index_early_1), .multiply_generation_early(mul_generation_early_1),
        .divide_write_enable_early(div_write_enable_early_1), .divide_index_early(div_index_early_1), .divide_generation_early(div_generation_early_1),
        .rd_mul(), .multiply_index(mul_index_1), .multiply_generation(mul_generation_1),
        .divide_data_out(div_data_1), .divide_loaded(div_write_enable_1), .divide_write_enable(),
        .rd_div(), .divide_index(div_index_1), .divide_generation(div_generation_1),
        .multiply_ready(mul_ready_1), .divide_ready(div_ready_1),
        .multiply_wake_registered(), .multiply_wake_index_registered(), .multiply_wake_generation_registered(),
        .multiply_nack_registered(), .multiply_nack_index_registered(), .multiply_nack_generation_registered()
    );

    bju u_bju_0 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]),
        .source_a_data_in(operand_data_a_0), .source_b_data_in(operand_data_b_0),
        .branch_unit_function4_in(bju_func4_0),
        .pc_plus4_in(auxiliary_address_0), .br_off_in(beq_offset_0),
        .jalr_pred_tgt_in(jump_link_register_prediction_address_0),
        .branch_prediction_taken_in(branch_prediction_taken_0),
        .jal_tgt_in(jump_link_target_0),
        .advance_in(issue_valid_0),
        .success(success_0), .branch_failure(branch_fail_0), .jump_link_register_failure(jump_link_register_fail_0),
        .index_in(issue_index_0), .generation_in(issue_generation_0),
        .index_registered(bju_index_registered_0), .judged_registered(judged_0), .generation_registered(generation_registered_0), .redirect_registered(redir_registered_0),
        .jump_target(jump_target_0), .jump_link_register_target_registered(jump_link_register_target_0),
        .branch_pc_index(branch_pc_index_0), .branch_pc_registered(branch_pc_0),
        .branch_unit_kind(bju_kind_0), .branch_unit_taken(bju_taken_0),
        .branch_unit_pc(bju_pc_0), .branch_unit_target(bju_target_0),
        .exception_target_misalign(exception_target_misalign_0),
        .jump_link_register_target_valid_registered(jump_link_register_valid_0), .branch_prediction_taken_registered(branch_prediction_taken_registered_0),
        .exception_branch_unit(), .judged_index(bju_index_i_0), .exception_pc_registered(exception_pc_registered_0),
        .exception_ecall_judged(exception_ecall_0), .exception_ebreak_judged(exception_ebreak_0),
        .exception_illegal_judged(exception_illegal_0), .exception_interrupt_return_judged(exception_interrupt_ret_0),
        .exception_ecall_registered(exception_ecall_registered_0), .exception_ebreak_registered(exception_ebreak_registered_0),
        .exception_illegal_registered(exception_illegal_registered_0), .exception_interrupt_return_registered(exception_interrupt_ret_registered_0),
        .exception_pc_judged(exception_pc_i_0),
        .flush_index_in(flush_index)
    );

    bju u_bju_1 (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]),
        .source_a_data_in(operand_data_a_1), .source_b_data_in(operand_data_b_1),
        .branch_unit_function4_in(bju_func4_1),
        .pc_plus4_in(auxiliary_address_1), .br_off_in(beq_offset_1),
        .jalr_pred_tgt_in(jump_link_register_prediction_address_1),
        .branch_prediction_taken_in(branch_prediction_taken_1),
        .jal_tgt_in(jump_link_target_1),
        .advance_in(issue_valid_1),
        .success(success_1), .branch_failure(branch_fail_1), .jump_link_register_failure(jump_link_register_fail_1),
        .index_in(issue_index_1), .generation_in(issue_generation_1),
        .index_registered(bju_index_registered_1), .judged_registered(judged_1), .generation_registered(generation_registered_1), .redirect_registered(redir_registered_1),
        .jump_target(jump_target_1), .jump_link_register_target_registered(jump_link_register_target_1),
        .branch_pc_index(branch_pc_index_1), .branch_pc_registered(branch_pc_1),
        .branch_unit_kind(bju_kind_1), .branch_unit_taken(bju_taken_1),
        .branch_unit_pc(bju_pc_1), .branch_unit_target(bju_target_1),
        .exception_target_misalign(exception_target_misalign_1),
        .jump_link_register_target_valid_registered(jump_link_register_valid_1), .branch_prediction_taken_registered(branch_prediction_taken_registered_1),
        .exception_branch_unit(), .judged_index(bju_index_i_1), .exception_pc_registered(exception_pc_registered_1),
        .exception_ecall_judged(exception_ecall_1), .exception_ebreak_judged(exception_ebreak_1),
        .exception_illegal_judged(exception_illegal_1), .exception_interrupt_return_judged(exception_interrupt_ret_1),
        .exception_ecall_registered(exception_ecall_registered_1), .exception_ebreak_registered(exception_ebreak_registered_1),
        .exception_illegal_registered(exception_illegal_registered_1), .exception_interrupt_return_registered(exception_interrupt_ret_registered_1),
        .exception_pc_judged(exception_pc_i_1),
        .flush_index_in(flush_index)
    );

//==============================================================================
// 共享访存单元
//==============================================================================
    lsu #(.ADDER_AT_D(ADDER_AT_D)) u_lsu (
        .clk(clk), .rst(rst),
        .flush_control_in(flag_bus[7:5]), .exec_in(flag_bus[4]),
//★ 总线被按住只在 LSU 内部生效（2026-10-31）：不再经 flag_bus[0] 去停整个前端。
        .bus_hold_in(bus_hold_in),
        .payload_go(lsu_valid),
        .opcode(lsu_opcode), .function10(lsu_function10),
        .destination_in(lsu_destination_register), .r1_post(lsu_rs_a), .r2_post(lsu_rs_b),
        .idx_in(lsu_index), .gen_in(lsu_generation),
        .source_a_data_final(lsu_a), .source_b_data_final(lsu_b), .source_imm(lsu_imm),
        .load_hazard_valid_0(load_hazard_valid_0), .load_hazard_rd_0(load_hazard_rd_0),
        .load_hazard_valid_1(load_hazard_valid_1), .load_hazard_rd_1(load_hazard_rd_1),
        
        .bus_data_external(bus_data_in_ext), .bus_data_from_dcache(data_dcache),
        .bus_data_from_timer(data_tim),
        .dcache_hold(hold_dcache), .dcache_blocked(request_blocked_dcache), .load_ready_next(load_ready_next_dcache),
//站没了之后"更老的 BJU 在飞"只剩这一个来源（见 spec §6.4）
        .head_one_hot_registered(head_one_hot_registered), .redirect_any(rob_redir_any),
        
        .ready_dcache(load_ready_dcache), .ready_timer(ready_tim), .ready_external(bus_loaded_in),
        .dcache_address_out(dcache_address_lsu), .dcache_data_out(dcache_data_lsu),
        .dcache_byte_enable_out(dcache_byte_enable_lsu), .dcache_we_out(dcache_write_enable_lsu),
        .dcache_valid_out(dcache_valid_lsu),
        .stage1_address_out(bus_address_lsu), .stage1_data_out(bus_data_lsu),
        .stage1_byte_enable_out(bus_byte_enable_lsu), .stage1_we_out(bus_write_enable_lsu), .stage1_valid_out(bus_valid_lsu),
        .load_data_out(load_data_lsu), .loaded(), .load_write_enable(load_write_enable_lsu),
        .load_data_early(load_data_early_lsu),
        .load_write_enable_early(load_write_enable_early_lsu), .ld_index_e(load_index_early_lsu), .ld_generation_e(load_generation_early_lsu),
        .stage_free(carriage_free_lsu),
        .load_wake_registered(load_wake_registered_lsu), .load_wake_index_registered(load_wake_index_lsu), .load_wake_generation_registered(load_wake_generation_lsu),
        .load_nack_registered(), .load_nack_index_registered(), .load_nack_generation_registered(),
        .destination_load(),
        .ld_index(load_index_lsu), .ld_generation(load_generation_lsu),
        .store_done(store_done_lsu), .store_index_out(store_index_lsu), .store_generation_out(store_generation_lsu),
        .ret0_valid(ret0_valid_rob), .ret0_index(ret0_index_rob), .ret0_generation(ret0_generation_rob),
        .ret1_valid(ret1_valid_rob), .ret1_index(ret1_index_rob), .ret1_generation(ret1_generation_rob),
        .exception_load_store_misalign_out(exception_ldst_misalign_lsu),
        .exception_load_store_store_out(exception_ldst_store_lsu),
        .exception_load_store_address_out(exception_ldst_address_lsu),
        .exception_load_store_index_out(exception_ldst_index_lsu),
        .exception_load_store_pc_out(exception_ldst_pc_lsu),
        .pc_in(lsu_pc)
    );

//==============================================================================
// 重排序缓冲
//==============================================================================
    rob #(.DEPTH(ROB_DEPTH)) u_rob (
        .clk(clk), .rst(rst),
        .allocation_en_0(alloc_enable_0), .allocation_en_1(alloc_enable_1),
        .allocation_rd_0(alloc_destination_register_0), .allocation_rd_1(alloc_destination_register_1),
        .allocation_we_0(alloc_write_enable_0), .allocation_we_1(alloc_write_enable_1),
        .allocation_jmp_0(alloc_jmp_0), .allocation_jmp_1(alloc_jmp_1),
        .allocation_pend_0(alloc_pending_0), .allocation_pend_1(alloc_pending_1),
        .allocation_ecall_0(alloc_ecall_0), .allocation_ebreak_0(alloc_ebreak_0),
        .allocation_illegal_0(alloc_illegal_0), .allocation_mret_0(alloc_mret_0),
        .allocation_ecall_1(alloc_ecall_1), .allocation_ebreak_1(alloc_ebreak_1),
        .allocation_illegal_1(alloc_illegal_1), .allocation_mret_1(alloc_mret_1),
        .allocation_index_0(alloc_index_0), .allocation_index_1(alloc_index_1),
        .allocation_generation_0(alloc_generation_0), .allocation_generation_1(alloc_generation_1),
        .allocation_pc_0(h0_address), .allocation_pc_1(h1_address),
        .head_pc_out(head_pc_rob), .head_valid_out(head_valid_rob),

        .bju_judged(judged_0), .bju_index(bju_index_registered_0), .bju_generation(generation_registered_0),
        .bju_redir(redir_registered_0), .bju_tgt(jump_target_0),
        .branch_unit_lane1_redir(redir_registered_1), .branch_unit_lane1_tgt(jump_target_1),
        .redirect_valid_delayed(rob_redir_valid), .redirect_target_delayed(rob_redir_pc), .redirect_any(rob_redir_any),
        .mret_mark(mret_mark), .mret_index(mret_index), .mret_tgt(mret_target), .mret_valid_delayed(mret_valid),
        .branch_unit_lane1_judged(judged_1), .branch_unit_lane1_index(bju_index_registered_1), .branch_unit_lane1_generation(generation_registered_1),
        .alu_done(write_enable_alu_0), .alu_index(index_alu_0), .alu_data(source_index_alu_0), .alu_generation(generation_alu_0),
        .alu_lane1_done(write_enable_alu_1), .alu_lane1_index(index_alu_1), .alu_lane1_data(source_index_alu_1), .alu_lane1_generation(generation_alu_1),
        .mul_done(mul_write_enable_0), .mul_index(mul_index_0), .mul_data(mul_data_0), .mul_generation(mul_generation_0),
        .multiply_lane1_done(mul_write_enable_1), .multiply_lane1_index(mul_index_1), .multiply_lane1_data(mul_data_1), .multiply_lane1_generation(mul_generation_1),
        .div_done(div_write_enable_0), .div_index(div_index_0), .div_data(div_data_0), .div_generation(div_generation_0),
        .divide_lane1_done(div_write_enable_1), .divide_lane1_index(div_index_1), .divide_lane1_data(div_data_1), .divide_lane1_generation(div_generation_1),
        .ld_done(load_write_enable_lsu), .ld_index(load_index_lsu), .ld_data(load_data_lsu), .ld_generation(load_generation_lsu),
        .st_done(store_done_lsu), .st_index(store_index_lsu), .st_generation(store_generation_lsu),
        .acc_alu_o(), .acc_alu2_o(), .acc_mul_o(), .acc_mul2_o(),
        .acc_div_o(), .acc_div2_o(), .acc_ld_o(),
        .ret0_v(ret0_valid_rob), .ret0_index(ret0_index_rob), .ret0_generation(ret0_generation_rob),
        .ret1_v(ret1_valid_rob), .ret1_index(ret1_index_rob), .ret1_generation(ret1_generation_rob),

        .issue_enable_0(issue_valid_0), .issue_index_0(issue_index_0), .issue_generation_0(issue_generation_0),
        .issue_enable_1(issue_valid_1), .issue_index_1(issue_index_1), .issue_generation_1(issue_generation_1),

        .flush_control_rob(flush_con_rob),
        
        .exc0_en(exc0_enable), .exc0_index(exc0_index), .exc0_cause(exc0_cause),
        .exc0_pc(exc0_pc), .exc0_tval(exc0_trap_value),
        .exc1_en(exc1_enable), .exc1_index(exc1_index), .exc1_cause(exc1_cause),
        .exc1_pc(exc1_pc), .exc1_tval(exc1_trap_value),
        .exception_selected_enable(excls_enable), .exception_selected_index(excls_index), .exception_selected_cause(excls_cause),
        .exception_selected_pc(excls_pc), .exception_selected_trap_value(excls_trap_value),

        .commit_write_enable_0(commit_we0), .commit_destination_0(commit_rd0), .commit_data0(commit_data0),
        .commit_slot0(commit_slot0), .commit_gen0(commit_gen0),
        .commit_write_enable_1(commit_we1), .commit_destination_1(commit_rd1), .commit_data1(commit_data1),
        .commit_slot1(commit_slot1), .commit_gen1(commit_gen1),

        .head_pointer(head_pointer), .full(rob_full), .slot_above_head(above),
        .slot_above_head_registered(above_registered), .head_one_hot_registered(head_one_hot_registered), .retire_step(rob_ret_step),
        .empty(rob_empty), .occupancy(occupancy_rob),
        .exception_valid_delayed(trap_fire), .exception_cause_delayed(trap_cause),
        .exception_pc_delayed(trap_pc), .exception_trap_value_delayed(trap_trap_value)
    );

//==============================================================================
// 寄存器堆（两条提交写口）
//==============================================================================
    regfile u_regfile (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
//顺序发射：读口每拍跟着地址刷新，"上一拍冻结"那一档不再存在
        .stall_read_set_full(1'b0),
//★ 读口地址来自【出队格 F 级】（不是 idu2 的 D）：寄存一拍之后正好与"这一条进 D"同拍到达，
//  于是 rdata 与 D 里的指令对齐 —— 这就是 idu2 那边"base 与指令一起来"的口径。
//  lane1 到底拿第二格 h1 还是队头 h0，由 idu1 本拍的装载决定（两条 lane 独立推进）。
        .read_1(rs1_idu1_0), .read_2(rs2_idu1_0),
        .read_3(rs1_idu1_1), .read_4(rs2_idu1_1),
        .read_5(lsu_source_a_idu1), .read_6(lsu_source_b_idu1),
        .write_enable_a(commit_we0), .destination_a(commit_rd0), .data_a(commit_data0),
        .write_enable_b(commit_we1), .destination_b(commit_rd1), .data_b(commit_data1),
        .read_1_data(rdata1), .read_2_data(rdata2), .read_3_data(rdata3), .read_4_data(rdata4), .read_5_data(rdata5), .read_6_data(rdata6)
    );

//==============================================================================
// 控制 / CSR
//==============================================================================
    cont u_cont (
        .clk(clk), .rst(rst),
        .jalr_fail_0(jump_link_register_fail_0), .jalr_fail_1(jump_link_register_fail_1),
        .br2(br2), .br3(br3),
        .redirect_in(rob_redir_valid),
        .success_0(success_0), .success_1(success_1),
        .br_fail_0(branch_fail_0), .br_fail_1(branch_fail_1),
        .exc_tgt_mis_0(exception_target_misalign_0), .exc_tgt_mis_1(exception_target_misalign_1),
        .bju_pc_in_0(exception_pc_registered_0), .bju_pc_in_1(exception_pc_registered_1),
        .bju_tgt_in_0(jump_target_0), .bju_tgt_in_1(jump_target_1),
        .exc_ecall_q_0(exception_ecall_registered_0), .exc_ebreak_q_0(exception_ebreak_registered_0),
        .exc_illegal_q_0(exception_illegal_registered_0), .exc_irq_ret_q_0(exception_interrupt_ret_registered_0),
        .exc_ecall_q_1(exception_ecall_registered_1), .exc_ebreak_q_1(exception_ebreak_registered_1),
        .exc_illegal_q_1(exception_illegal_registered_1), .exc_irq_ret_q_1(exception_interrupt_ret_registered_1),
        .exception_load_store_misalign_in(exception_ldst_misalign_lsu),
        .exc_ldst_st_in(exception_ldst_store_lsu),
        .exception_load_store_address_in(exception_ldst_address_lsu),
        .exception_pc_in(exception_ldst_pc_lsu),
        .exception_load_store_index_in(exception_ldst_index_lsu),
        .issue_index_in(lsu_index),
        .interrupt_head_index_in(head_pointer),
        .interrupt_head_valid(head_valid_rob),
        .bju_index_in_0(bju_index_registered_0), .bju_index_in_1(bju_index_registered_1),
        .stall_icache_miss(busy_icache),
        .stall_bus_hold(bus_hold_in),
        .stall_rob_full(rob_full),
        .slot_above_head(above),
        .rob_empty(rob_empty),
        .rob_trap(trap_fire), .rob_ret_step(rob_ret_step), .rob_trap_cause(trap_cause),
        .rob_trap_pc(trap_pc), .rob_trap_tval(trap_trap_value),
        .flush_control_rob(flush_con_rob), .flush_index(flush_index), .flush_inclusive(flush_inclusive),
        .flush_index_q(flush_index_registered), .flush_con_rob_q(flush_con_rob_registered),
        .flush_incl_q(flush_inclusive_registered), .flush_con_exc_q(flush_con_exception_registered),
        .flush_pending(flush_pending),
        .jump_link(1'b0), .np_take(pc_take0 | pc_take1),
        .csr_write_enable(csr_write_enable_m), .csr_write_action(csr_write_action_m),
        .exti(exti), .timi(timer_interrupt), .softi(1'b0),
        .csr_address(csr_address_m), .csr_data_in(csr_data_m),
        .pc_address_in(head_pc_rob),
        .interrupt_service_address_2(isr_addr2), .mcause(mcause), .csr_data_combinational(csr_data_combinational),
        .exception_interrupt_action(exception_interrupt_action), .exception_interrupt_processing(exception_interrupt_processing),
        .exc_irq(exception_interrupt), .interrupt_return_address_2(iret_addr2),
        .flag_bus(flag_bus),
        .flush_control_exception(flush_con_exception),
        .exc0_en(exc0_enable), .exc0_index(exc0_index), .exc0_cause(exc0_cause),
        .exc0_pc(exc0_pc), .exc0_tval(exc0_trap_value),
        .exc1_en(exc1_enable), .exc1_index(exc1_index), .exc1_cause(exc1_cause),
        .exc1_pc(exc1_pc), .exc1_tval(exc1_trap_value),
        .exception_selected_enable(excls_enable), .exception_selected_index(excls_index), .exception_selected_cause(excls_cause),
        .exception_selected_pc(excls_pc), .exception_selected_trap_value(excls_trap_value),
        .exception_interrupt_return_ok(exception_interrupt_ret_ok),
        .mret_mark(mret_mark), .mret_index(mret_index), .mret_target(mret_target), .mret_valid(mret_valid),
        .redirect_go(redir_go), .redirect_target(redir_target),
        .flush_pc_redirect(flush_pc_redir)
    );

//==============================================================================
// 数据侧存储
//==============================================================================
    dcache #(.DC_SETS_B(DC_SETS_B)) u_dcache (
        .clk(clk), .rst(rst),
        .bus_address_in(dcache_address_lsu), .bus_data_in(dcache_data_lsu),
        .bus_byte_enable_in(dcache_byte_enable_lsu), .bus_write_enable_in(dcache_write_enable_lsu),
        .bus_valid_in(dcache_valid_lsu),
        .mem_data(mem_data_dtcm), .mem_valid(mem_valid_dtcm), .mem_ready(1'b1),
        .bus_data_out(data_dcache),
        .load_ready(load_ready_dcache), .busy(busy_dcache), .hold(hold_dcache), .request_blocked(request_blocked_dcache),
        .load_ready_next(load_ready_next_dcache),
        .mem_req(mem_req_dcache), .mem_write_enable(mem_write_enable_dcache),
        .mem_addr(mem_address_dcache), .mem_write_data(mem_wdata_dcache), .mem_byte_enable(mem_byte_enable_dcache)
    );

//==============================================================================
// 二级存储：原 itcm + dtcm 合成一块 128KB BRAM
//   代码窗口 → L2 偏移 0x0000（64KB）；数据窗口 → L2 偏移 0x4000（8KB）
//   一致性：L1D 写穿到 L2（dcache 本来就是写穿），L2 的写脉冲回灌 icache 作废同一行
//==============================================================================
    l2cache #(.L2_AW(15), .L2_ILINE(32), .L2_DLINE(8)) u_l2cache (
        .clk(clk), .rst(rst),
        .i_req(mem_req_icache), .i_addr(mem_address_icache),
        .i_data(mem_data_icache), .i_valid(mem_valid_icache),
        .d_req(mem_req_dcache), .d_we(mem_write_enable_dcache),
        .d_addr(mem_address_dcache), .d_wdata(mem_wdata_dcache), .d_be(mem_byte_enable_dcache),
        .d_data(mem_data_dtcm), .d_valid(mem_valid_dtcm), .d_ready(),
        .w_pulse(l2_write_pulse), .w_addr(l2_write_addr),
        .m_haddr(l2_haddr), .m_hwrite(l2_hwrite), .m_htrans(l2_htrans), .m_hsize(l2_hsize),
        .m_hwdata(l2_hwdata), .m_hwstrb(l2_hwstrb),
        .m_hrdata(l2_hrdata), .m_hready(l2_hready), .m_busy(l2_busy)
    );

    tim_in u_tim_in (
        .clk(clk), .rst(rst),
        .bus_address_in(bus_address_lsu), .bus_data_in(bus_data_lsu),
        .bus_byte_enable_in(bus_byte_enable_lsu), .bus_write_enable_in(bus_write_enable_lsu),
        .bus_valid_in(bus_valid_lsu),
        .bus_data_out(data_tim), .timer_interrupt(timer_interrupt), .ready(ready_tim)
    );

//===============================================================
// 宏化参数自检（综合丢弃，仿真时恒生效）—— 与 icache 的参数自检同构
//===============================================================
// synthesis translate_off
    initial begin
        if ((ROB_DEPTH < 4) || (ROB_DEPTH > 16) || ((ROB_DEPTH & (ROB_DEPTH-1)) != 0))
            $display("CFG_FAIL: ROB_DEPTH=%0d 必须是 4..16 的 2 的幂", ROB_DEPTH);
        else if (VALUE_TABLE_DEPTH != ROB_DEPTH)
            $display("CFG_FAIL: VALUE_TABLE_DEPTH=%0d 必须等于 ROB_DEPTH=%0d", VALUE_TABLE_DEPTH, ROB_DEPTH);
        else
            $display("CFG_OK: LANES=%0d ROB=%0d IFFU=%0d VT=%0d IC_INDEX_B=%0d BHT=%0d BTB=%0d ADDER_AT_D=%0d",
                     LANES, ROB_DEPTH, IFFU_DEPTH, VALUE_TABLE_DEPTH, IC_INDEX_B, BHT_B, BTB_B, ADDER_AT_D);
    end
// synthesis translate_on
endmodule
