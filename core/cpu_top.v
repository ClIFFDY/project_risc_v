`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: cpu_top
// Description:
//   顺序双发射核的顶层：前端（pc / bpu / icache / iffu / itcm）+ 译码（idu）
//   + 发射（isq，两条 lane 各一级 D→Q）+ 重排序缓冲（rob）+ 每 lane 一套
//   alu / mdu / bju + 共享 lsu + 控制（cont，内含 csr）+ 寄存器堆 + 数据侧存储。
//
//   本层不做运算，只连线。允许的两类例外：
//     ① 拍差换算（cur_addr 的 bti 偏置、dcache 的握手直连），它们都是"哪一根线"的问题；
//     ② 两条 lane 的合流（前端回报 / 乘法结果口 / CSR 写口）—— 选择码由 cont 给（bju_fe_sel），
//        其余按"有事件的那条优先"，逐条写明。
//
//   流水线（一拍一级）：
//     F  iffu 出队格 h0/h1 ──► idu 的 S 级（D）
//     D  idu 的 S 级寄存器  ──► isq 的 D 端（译码、rob 分配、生产者表登记）
//     Q  isq 的 Q 寄存器    ──► 单元（发射判据在 Q 端组合，发出那一拍锁字段）
//     W  单元结果口（寄存）  ──► rob 的完成回填；rob 按队头提交
//   相位契约见 docs/spec_isq_v2.md 第三节。
//////////////////////////////////////////////////////////////////////////////////

module cpu_top #(
//★★ 宏化旋钮：一份源码切档。默认值 = 大核（交付配置，行为与原树逐位相同）。
    parameter integer LANES             = 2,    // 2 = 双发 / 1 = 单发
    parameter integer ROB_DEPTH         = 32,   // 32 / 16（必须是 2 的幂）
    parameter integer IFFU_DEPTH        = 16,   // 16 / 8（取指改 16B 块后按 4 的倍数取）
    parameter integer VALUE_TABLE_DEPTH = 32,   // 必须 = ROB_DEPTH
    parameter integer IC_INDEX_B        = 6,    // 8 = 64KB / 7 = 32KB / 6 = 16KB
    parameter integer BHT_B             = 11,   // 11 / 9
    parameter integer BTB_B             = 11,   // 11 / 9
    parameter integer DC_SETS_B         = 8,   // 9 = 32KB / 8 = 16KB / 7 = 8KB
    parameter integer ITCM_AW           = 14,  // 14 = 64KB / 13 = 32KB
    parameter integer ADDER_AT_D        = 0     // 1 = 访存地址加法搬到 D 端
) (
    input clk, rst,
    input [31:0] bus_data_in_ext,
    input bus_loaded_in, bus_hold_in, exti,
    output reg [31:0] bus_addr_out, bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out, bus_valid_out,
//---- L2 的 AHB 主口（2026-11-12：itcm/dtcm 撤掉，由 64KB 统一 L2 顶替）----
    output     [31:0] l2_haddr,
    output     [1:0]  l2_htrans,
    output            l2_hwrite,
    output     [2:0]  l2_hsize,
    output     [2:0]  l2_hburst,
    output     [3:0]  l2_hprot,
    output            l2_hmastlock,
    output     [31:0] l2_hwdata,
    output     [3:0]  l2_hwstrb,
    input      [31:0] l2_hrdata,
    input             l2_hready,
    input             l2_rd_valid,
    input             l2_hgrant,
    input             l2_hresp,
    output            l2_busy,
//---- L2 维护与非缓存窗口（自举/重装代码时用）----
    input      [1:0]  l2_maint,
    output            l2_maint_done,
    input      [31:0] l2_uncached_base,
    input      [31:0] l2_uncached_mask
    );

//==============================================================================
// 前端
//==============================================================================
    wire [31:0] pc_address_0;
//★ 2026-11-08：取指改 16B 块。给 bpu 的是这一块里四个地址（块基址 +0/4/8/12），
//  给 icache 的仍然是【下一条要执行】的指令地址；块内偏移用来把比它老的那几条挡在仲裁外。
    wire [31:0] pc_block_0, pc_block_1;
    wire [1:0]  pc_offset;
//测试台接口：几乎每个 tb 都用 dut.address_pc 当"取指 PC 活动窗口"的判据（这是原 pc 模块的 pc_addr 口，
//  双 lane 改造后改成了 pc_address_0/pc_address_1）。这里只做一次别名，不进任何逻辑；综合时无人读会被自动裁掉。
    wire [31:0] address_pc = pc_address_0;
    reg  [31:0] current_address_0, current_address_1;
    wire [31:0] instruction_0_icache, instruction_1_icache;
    wire [31:0] instruction_addr0_icache, instruction_addr1_icache;
    wire        instruction_valid0_icache, instruction_valid1_icache;
    wire        dpred0_icache, dpred1_icache;
    wire [31:0] djpred0_icache, djpred1_icache;
    wire [1:0]  block_offset_icache;
    wire        icache_miss_icache, busy_icache, deliver_wrong_path_icache;
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

//前端"这一拍有冲刷发生"（交付/边带都要按它作废）。★ 提前声明：边带两级都要清。
//  （这几根由下面的 clint / rob 例化驱动，提前的只是【名字】，不影响谁驱动。）
    wire        irq_kill;
    wire [1:0]  redir_cause;                 // 与 redir_frontend / redir_backend_0 同拍：0=分支 1=陷阱 2=mret
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
//  （这两个寄存器放在下面前端信号声明之后，见「前端：取指地址」段。）

    wire [31:0] h0_instruction, h0_address, h0_jump_link_register_prediction, h1_instruction, h1_address, h1_jump_link_register_prediction;
    wire        h0_valid, h1_valid;
    wire        h0_branch_prediction, h1_branch_prediction, full_iffu;
    wire [31:0] h2_instruction, h2_address, h2_jump_link_register_prediction;
    wire        h2_branch_prediction, h2_valid;

//★★ 预测不再是另打一拍的旁路，它就是这一对指令的延长位：
//  icache 在【交付寄存器装载的那一个沿】把 pc_take/btb_tgt 一起装进同一个寄存器组
//  （见 icache 的 inst_out 那一块），所以这里直接吃交付口的输出即可 ——
//  本文件不再需要 pr0/pr_l/jp0/jp_l/fz_d/tk_en 这一整套相位补偿。
//  原来为什么错：pr0 打一拍、pr_l 再打一拍，而 iffu 吃的 pr_l 在同一个沿上采的是
//  pr0 的【旧值】⇒ 记录的是【上一对】的预测（实测 CoreMark：pc 按落点表跳了，
//  队列条目里 br_pred 却是 0 ⇒ bju 的 br3 永不发 ⇒ bne 该退出不退出，死循环）。
//==============================================================================
// 译码级（idu）→ 分配
//==============================================================================
    wire [2:0]  advance_idu;
    wire        lane_valid_0, lane_valid_1, accept_0, accept_1;
    wire        credit_stall_idu;

    wire        alloc_enable_0, alloc_enable_1, alloc_write_enable_0, alloc_write_enable_1;
    wire        alloc_jmp_0, alloc_jmp_1, alloc_pending_0, alloc_pending_1;
//分配拍的 SYSTEM 类译码（与 alloc_jmp 同相）—— 给 ROB 判"退休要不要等判定延迟级"
    wire        alloc_ecall_0, alloc_ebreak_0, alloc_illegal_0, alloc_mret_0;
    wire        alloc_ecall_1, alloc_ebreak_1, alloc_illegal_1, alloc_mret_1;
//★ 2026-11-13（fence）：MISC-MEM 的两条，随分配写进 ROB 项（与 alloc_mret 同型）
    wire        alloc_fence_0, alloc_fence_i_0, alloc_fence_1, alloc_fence_i_1;
    wire        alloc_fence_2, alloc_fence_i_2;
    wire [4:0]  alloc_destination_register_0, alloc_destination_register_1;
//★ 2026-11-04 真 PRF：物理号与冲刷掩码
    wire [5:0]  alloc_phys_0, alloc_phys_1, commit_phys0, commit_phys1;
    wire [31:0] alloc_pc_1;
//精确冲刷第 1 步：分配时顶掉的旧映射（随 ROB 项存，将来回滚 rat 用；现在没有读者）
//★ 2026-11-04（CSR 步骤 C）：CSR 随 ROB 项走、在提交拍落地。载荷 {写使能 1, 地址 12, funct3 3, 源 6}。
    wire        alloc_csr_0, alloc_csr_1;
    wire [21:0] csr_payload_0, csr_payload_1;
    wire        commit_csr_valid;
    wire [21:0] commit_csr_payload;
    wire [4:0]  alloc_index_0, alloc_index_1;
    wire        alloc_generation_0, alloc_generation_1, alloc_generation_2;
    wire        alloc_enable_2, alloc_write_enable_2, alloc_jmp_2, alloc_pending_2;
    wire        alloc_ecall_2, alloc_ebreak_2, alloc_illegal_2, alloc_mret_2, alloc_csr_2;
    wire [4:0]  alloc_destination_register_2, alloc_index_2;
    wire [5:0]  alloc_phys_2;
//★ 2026-11-10：这条指令将来从哪个 PRF 写口落笔（给 prf 的 owner 表，分配那一拍写入）
    wire [2:0] alloc_owner_0, alloc_owner_1, alloc_owner_2;

    wire [31:0] instruction_out_0, instruction_out_1, auxiliary_address_out_0, auxiliary_address_out_1;
    wire [4:0]  rs1_idu_0, rs2_idu_0, rs1_idu_1, rs2_idu_1;
    wire [4:0]  index_out_0, index_out_1;
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
// 发射级（isq）↔ 单元
//==============================================================================

    wire        bypass_load_a_0, bypass_load_b_0, bypass_load_a_1, bypass_load_b_1;
    wire        issue_valid_0, issue_valid_1, write_enable_0, write_enable_1;
    wire [4:0]  issue_index_0, issue_index_1;
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
    wire [4:0]  lsu_index;
    wire [31:0] lsu_pc;
    wire [31:0] lsu_a, lsu_b, lsu_imm;

    wire [31:0] head_pc_rob;
    wire        head_valid_rob, issue_pop_0, issue_pop_1;
//---- 访存专用流：idu 的 LSU FIFO <-> isq ----
    wire [31:0] lsu_instruction_idu, lsu_auxiliary_address_idu, lsu_immediate_idu;
    wire [4:0]  lsu_source_a_idu, lsu_source_b_idu, lsu_destination_register_idu;
    wire [2:0]  lsu_load_store_function3_idu;
    wire [4:0]  lsu_index_idu;
    wire        lsu_is_store_idu, lsu_write_enable_idu, lsu_generation_idu, lsu_valid_idu;
    wire        lsu_pop_isq;
    wire        payload_fresh_0, payload_fresh_1;
//★ 2026-11-04 真 PRF：idu/isq 之间走的是【物理号】，不再有生产者 tag
    wire [5:0]  phys_a_out_0, phys_b_out_0, phys_a_out_1, phys_b_out_1;
//★ 2026-11-06 提前唤醒：目的物理号随载荷走，isq 的站拿它当唤醒源
    wire [5:0]  phys_d_out_0, phys_d_out_1;
    wire [5:0]  lsu_phys_a_idu, lsu_phys_b_idu, lsu_phys_d_idu;
//★ 2026-11-12：目的物理号从访存站随路带到 lsu 的返回级，ROB 不再现查 entry_phys
    wire [5:0]  lsu_phys_d_isq, load_phys_lsu;
//除法那一笔：两条 lane 各自的 mdu 报 done+1 的索引，由 ROB 换成物理号（同时最多一条在报）
    wire        divide_wake_valid_0, divide_wake_valid_1;
    wire [4:0]  divide_wake_index_0, divide_wake_index_1;
    wire [5:0]  divide_wake_phys_0, divide_wake_phys_1;
    wire        divide_wake_valid;
    wire [5:0]  divide_wake_phys;
    assign divide_wake_valid = divide_wake_valid_0 | divide_wake_valid_1;
    assign divide_wake_phys  = divide_wake_valid_0 ? divide_wake_phys_0 : divide_wake_phys_1;
    wire [63:0] alloc_onehot, arch_next_onehot;
// idu ↔ rename（rename 在顶层例化）
    wire [5:0]  phys_rd_0, phys_rd_1, phys_rd_2, phys_rd_3;
    wire [5:0]  lookup_a_phys, lookup_b_phys, lookup_c_phys;
    wire [5:0]  lookup_d_phys, lookup_e_phys, lookup_f_phys;
    wire        bank_empty_0, bank_empty_1, bank_empty_2, bank_empty_3;
    wire        bank_low_even, bank_low_odd;
//★ 2026-11-04 精确冲刷：rat 回滚值 + 幸存在飞项掩码（都由 rob 当拍算好）
    wire [31:0]  squash_arch_hit;
    wire [191:0] squash_arch_phys;
    wire [31:0]  squash_survive_hit;
    wire         redirect_stop_alloc;
//★ 2026-11-04 精确冲刷：按槽的击杀掩码。当拍版给 lsu/mdu（它们与冲刷同拍杀），
//  寄存版给 isq（它晚一拍杀）。
//---- PRF 读口 ----
    wire [5:0]  prf_read_a_0_phys, prf_read_b_0_phys, prf_read_a_1_phys, prf_read_b_1_phys;
    wire [5:0]  prf_read_a_m_phys, prf_read_b_m_phys;
    wire [31:0] prf_data_a_0, prf_data_b_0, prf_data_a_1, prf_data_b_1, prf_data_a_mem, prf_data_b_mem;
//提交拍的 CSR 源（第 7 个读口，地址 = ROB 提交口吐出来的 rs1 物理号）
    wire [31:0] prf_read_c_data;
//---- PRF 写口（物理号由 rob 按槽查出来；live = 本项还有效且世代对得上）----
    wire [5:0]  prf_wb_phys_0, prf_wb_phys_1;
//★ 2026-11-12：直接从 lsu 的返回级来（见 lsu 的 phys_in/ld_phys）
    wire [5:0]  prf_ld_phys_wb = load_phys_lsu;
    wire        prf_wb_live_0, prf_wb_live_1, prf_ld_live_wb, prf_ld_early_live;
//★ 2026-11-07：发射级的唤醒取【值表写口真的落笔】那一拍，不取发射拍。
//  发射拍和"值进阵列"之间隔着结果口仲裁（div>mul>alu，抢不到进影子晚一拍）和 live 门，
//  拿发射拍当唤醒会让消费者下一拍从 PRF 读到旧值。两个结果口的实际使能在这里算（与 prf 内部同式），
//  LOAD 早口的实际使能由 prf 吐 load_write（早写被晚写门掉那一拍它是 0）。
    wire        prf_result_write_0, prf_result_write_1;
    wire        prf_alu_write_0, prf_alu_write_1;
    wire        prf_alu_early_live_0, prf_alu_early_live_1;
    wire        prf_load_write;
    wire [5:0]  phys_alu_early_0, phys_alu_early_1, ld_phys_early, prf_load_write_phys;
//idu 本拍 lane0 装没装：决定 lane1 的读口地址取队头 h0 还是第二格 h1
    wire        lane0_load;

//==============================================================================
// 单元结果口
//==============================================================================
    wire [31:0] source_index_alu_0, source_index_alu_1, csr_result_alu_0, csr_result_alu_1;
//★★ 2026-11-04：执行单元收成 exu（bju/alu/mdu 一层），每 lane 只留【一个】值表写口。
//  原来 7 个早写口（alu0 alu1 mul0 mul1 div0 div1 ld），值表因此要复制好几份。
//  exu 内部按 div > mul > alu 抢口，抢不到的进本类影子寄存器 ⇒ lane 侧只剩一根结果线。
//  现在值表写口 = 3 个：lane0 / lane1 / LD，寄存器堆才谈得上按 bank 复制两份。
    wire [31:0] result_port_0_data, result_port_1_data;
    wire        result_port_0_enable, result_port_1_enable;
//★ 2026-11-09：结果口这一拍是不是除法 —— 发射级用它把乘法从动态唤醒里摘掉。
    wire        result_is_divide_0, result_is_divide_1;
//★ 值表写口这一拍真的落笔 = 发射级唤醒的判据（下一拍这个号一定读得到）
    assign prf_result_write_0 = result_port_0_enable & prf_wb_live_0 & (|prf_wb_phys_0);
    assign prf_result_write_1 = result_port_1_enable & prf_wb_live_1 & (|prf_wb_phys_1);
    wire [4:0]  result_port_0_index, result_port_1_index;
    wire        result_port_0_generation, result_port_1_generation;
    wire        result_port_0_overflow, result_port_1_overflow;
//旁路那一组【只认 ALU】：mdu 的结果不进前送网（这是硬规矩），所以它不能跟结果口共用信号
    wire [31:0] source_index_alu_early_0, source_index_alu_early_1;
    wire        write_enable_alu_early_0, write_enable_alu_early_1;
//★ 2026-11-07：ALU 自己的值表写口（结果口不再仲裁 ALU）。落笔判据与 prf 里逐字同式。
    assign prf_alu_write_0 = write_enable_alu_early_0 & prf_alu_early_live_0 & (|phys_alu_early_0);
    assign prf_alu_write_1 = write_enable_alu_early_1 & prf_alu_early_live_1 & (|phys_alu_early_1);
    wire [4:0]  index_alu_early_0, index_alu_early_1;
    wire        generation_alu_early_0, generation_alu_early_1;
    wire [31:0] load_data_early_lsu;
    wire        load_write_enable_early_lsu, load_generation_early_lsu;
//★ 2026-11-11：load_write_enable_lsu 原来声明在下面（这里先用）⇒ Vivado 报 8-6901。
//  声明一律上移，和全文件一致。
    wire        load_generation_lsu, load_write_enable_lsu, carriage_free_lsu;
//★ 返回级这一笔 load 真的落进 PRF（= prf 的 write_ok_2）。站的 LOAD 前送与唤醒都用它，
//  与 load_data_lsu 同一级；早写口（s2）只留给 PRF 自己，不再参与唤醒/旁路。
    wire        load_write_return = load_write_enable_lsu & prf_ld_live_wb & (|prf_ld_phys_wb);
    wire [4:0]  load_index_early_lsu;
    wire [4:0]  index_alu_0, index_alu_1;
    wire        generation_alu_0, generation_alu_1, write_enable_alu_0, write_enable_alu_1;

    wire [31:0] mul_data_0, mul_data_1;
    wire [4:0]  mul_index_0, mul_index_1;
    wire        mul_generation_0, mul_generation_1, mul_write_enable_0, mul_write_enable_1;
//除法独立口
    wire [31:0] div_data_0, div_data_1;
    wire [4:0]  div_index_0, div_index_1;
    wire        div_generation_0, div_generation_1, div_write_enable_0, div_write_enable_1;
    wire        mul_ready_0, div_ready_0, mul_ready_1, div_ready_1;

//rob 本拍退休的两条 → lsu（退休过的项不许再按年龄作废）
    wire        ret0_valid_rob, ret1_valid_rob, ret0_generation_rob, ret1_generation_rob;
    wire [4:0]  ret0_index_rob, ret1_index_rob;

    wire [31:0] load_data_lsu;
    wire [4:0]  load_index_lsu;
    wire        store_done_lsu; wire [4:0] store_index_lsu; wire store_generation_lsu;

// 两条 mdu 的结果口在 isq 侧只有一条（早唤醒用；rob 侧两条都有）

//==============================================================================
// bju ×2
//==============================================================================
    wire        success_0, branch_fail_0, jump_link_register_fail_0, success_1, branch_fail_1, jump_link_register_fail_1;
    wire [31:0] jump_target_0, jump_target_1, jump_link_register_target_0, jump_link_register_target_1;
    wire [4:0]  bju_index_registered_0, bju_index_registered_1, bju_index_i_0, bju_index_i_1;
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
    wire [4:0]  exc0_index, exc1_index, excls_index;
    wire [3:0]  exc0_cause, exc1_cause, excls_cause;
    wire [31:0] exc0_pc, exc0_trap_value, exc1_pc, exc1_trap_value, excls_pc, excls_trap_value;
    wire        exception_ecall_0, exception_ecall_1, exception_ebreak_0, exception_ebreak_1, exception_illegal_0, exception_illegal_1, exception_interrupt_ret_0, exception_interrupt_ret_1;

//==============================================================================
// rob
//==============================================================================
    wire        commit_we0, commit_we1;
    wire [4:0]  commit_rd0, commit_rd1;
    wire [4:0]  head_pointer;
    wire        rob_redir_valid, redir_registered_0, redir_registered_1, rob_redir_any;
//全核就这三根重定向信号，名字写穿到各级触发器的使能端：
//  redir_frontend  —— A 域，判定拍：清前端 + 改 PC + 冻分配
//  redir_backend_0 —— B 域，队头那一拍：清 ROB 自己 + 执行级（alu/mdu/bju/lsu）
//  redir_backend_1 —— B 域再打一拍（影子）：清 isq / idu 后端
    wire        redir_frontend;
    wire        redir_backend_0;
    wire        redir_backend_1;


    wire        rob_redir_front;
    wire [31:0] rob_redir_pc;
    wire        rob_full, rob_empty;
//保留站空位：isq → idu 的分配门（每条 lane 各自一条）
    wire        station_room_0_valid, station_room_1_valid;
    wire [31:0] above, head_one_hot_registered;
    wire [1:0]  rob_ret_step;
    wire [4:0]  occupancy_rob;
    wire        rob_room_2 = (occupancy_rob <= (ROB_DEPTH - 3));
    wire [3:0]  trap_cause;
    wire [31:0] trap_pc, trap_value;

//==============================================================================
// cont / csr
//==============================================================================
    wire [4:0]  flush_index;
    wire [3:0]  exception_cause;
    wire [31:0] exception_pc, exception_trap_value;
    wire [4:0]  exception_index;
    wire        mret_valid;
    wire [31:0] isr_addr2, mcause, iret_addr2, csr_data_combinational;
    wire [4:0]  exception_ldst_index_lsu;
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
    wire [31:0] dcache_address_lsu;
    wire        dcache_valid_lsu;
//★ 2026-11-04：store 排水走 dcache 的 B 口（只写）。三根载荷都是 ELSQ 的寄存器。
    wire [31:0] store_address_lsu, store_data_lsu;
    wire [3:0]  store_byte_enable_lsu;
    wire        store_valid_lsu, store_blocked_dcache;
    wire [31:0] mem_address_dcache, mem_wdata_dcache, mem_data_dtcm;
    wire [3:0]  mem_byte_enable_dcache;
    wire        mem_req_dcache, mem_write_enable_dcache, mem_valid_dtcm;
//L2 的 D 口 ready：读看排队，写看写队列有没有空位（满的时候不许发，否则那笔写会被丢掉）
    wire        l2_d_ready_w;
    wire        ready_tim, timer_interrupt;
    wire        hold_dcache, load_ready_dcache, load_ready_next_dcache, busy_dcache, request_blocked_dcache;
//load 早唤醒：载入那条在【地址进 dcache 那一拍】就宣告下一拍有数（load_ready_next 的当拍版）
    wire        load_wake_registered_lsu, load_wake_generation_lsu;
    wire [4:0]  load_wake_index_lsu;
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
    reg  [11:0] csr_address_m;

//==============================================================================
//  CSR 提交口（2026-11-04，CSR 步骤 C）
//  载荷全在这里解释，csr.v 只吃已经拆好的四根 —— 拆包是纯连线，不进任何锥。
//  源：function3[2]=1 是立即数形式（rs1 字段即 zimm），取载荷里的源字段；
//      否则是寄存器形式，源字段是 rs1 物理号，走 PRF 的第 7 个读口。
//  csr_commit_write = "这条指令要写 CSR"：CSRRW/CSRRWI 恒写，CSRRS/CSRRC 及其立即数形式
//      只有源非 0 才写（规范口径）。寄存器形式下 rs1=x0 ⇔ 物理号 0 ⇒ 源字段为 0 就是"不写"。
//==============================================================================
    wire        csr_commit_action      = commit_csr_payload[21];
    wire [11:0] csr_commit_address     = commit_csr_payload[20:9];
    wire [2:0]  csr_commit_function3   = commit_csr_payload[8:6];
    wire [5:0]  csr_commit_source_field = commit_csr_payload[5:0];
    wire        csr_commit_write       = commit_csr_valid & csr_commit_action;
    wire [31:0] csr_commit_source      = csr_commit_function3[2] ? {27'd0, csr_commit_source_field[4:0]}
                                                                 : prf_read_c_data;
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
//★ 落点按 redir_cause 选（A 域那一拍，三根 rob 信号同拍）：
//  分支 = 随项存的落点（rob_redir_pc）；mret = csr 的 mepc；陷阱 = csr 的 mtvec。
//  mret 的 mepc 直接从 csr 取：让 ROB 把它搬进 kill_redirect_target 只是多一级寄存器、还错相位。
        fetch_jump_target     = (redir_cause == 2'd2) ? iret_addr2 :
                                (redir_cause == 2'd1) ? isr_addr2  : rob_redir_pc;
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
//CSR 读口（执行级）：地址选本拍要发的那条（两条同拍都是 CSR 指令时只落 lane0，并发 CSR 写不在本核口径内）
        csr_address_m   = csr_write_enable_1 ? csr_address_1 : csr_address_0;
    end

//★ 2026-10-31：原来这里有个 exception_pc_in_mux（lsu_lane ? aux1 : aux0）给访存非对齐当 pc。
//  访存条走 idu 的专用 FIFO，与 lane 的 auxiliary_address 早已脱钩（mt11：故障条在 0x30，
//  那一刻 aux0 冻在 0x18）⇒ 已删。pc 改由 isq 随 LSU FIFO 排队送出（lsu_pc）。

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
//★ pc 只吃：两根背压 + 两根改向量 + R0 的两个状态位。
//  不再自己解冲刷/停顿位，也不吃 icache 合成的 advance。
        .icache_miss(icache_miss_icache), .fifo_full(full_iffu), .busy(busy_icache),
        .redir_frontend(redir_frontend),
        .btb_take_0(btb_take_0), .btb_target_0(btb_target_0),
        .btb_take_1(btb_take_1), .btb_target_1(btb_target_1),
        .jump_target(fetch_jump_target),
        .irq_kill(irq_kill), .irq_target(isr_addr2),
        .pc_address_0(pc_address_0),
        .pc_block_0(pc_block_0), .pc_block_1(pc_block_1),
        .pc_offset(pc_offset)
    );

//==============================================================================
// 前端：预测器
//==============================================================================
    bpu #(.BHT_B(BHT_B), .BTB_B(BTB_B)) u_bpu (
        .clk(clk), .rst(rst),
        .redir_frontend(redir_frontend), .irq_kill(irq_kill), .icache_miss(icache_miss_icache), .fifo_full(full_iffu), .busy(busy_icache),
//学习源只有 bju 一家：它同时知道"这是哪种跳转""它自己去哪""它自己的地址"
        .branch_unit_kind(fetch_kind), .branch_unit_taken(fetch_taken),
        .branch_unit_pc(fetch_pc), .bju_tgt(fetch_target),
        .branch_prediction_taken_in(fetch_bpred),
//训练门不再看 ROB 的在册改向（见 bpu.v 文件头 ③）
//读口：pc 的两个取指地址（addr0 优先级 > addr1，仲裁在 bpu 内做完）
        .pc_addr_0(pc_block_0), .pc_addr_1(pc_block_1),
        .fetch_offset(pc_offset),
//给 pc：落点
        .btb_take_0(btb_take_0), .btb_take_1(btb_take_1),
        .btb_target_0(btb_target_0),   .btb_target_1(btb_target_1),
        .branch_stage2(br2), .branch_stage3(br3)
    );

//==============================================================================
// 前端：指令缓存 + 指令紧耦合存储
//★ 2026-11-13：icache 全失效脉冲。由 rob 的提交口状态机驱动（fence.i 那一拍，P3 接上）。
    wire icache_invalidate;
//★ 2026-11-13（P2b）：三层失效的另一半 —— dcache 的世代位与 L2 的 maint 口。
//  P3 只把 icache 接到 fence.i 上：dcache 是写穿、L2 对同址写是写失效 ⇒ 改指令的那笔 store
//  已经把这两层的旧副本作废了，fence.i 不必再付一次全清。这两根留给 P4 的软件失效原语
//  （DMA 一致性要用），在那之前钉 0。
//  L2 的 maint 是 2 位编码（2'b10 = 清全部 valid），所以这里拼成一根 2 位的"内部源"，
//  再和外部口 l2_maint 相或 —— 外部口现在 mcu_top 钉常量，将来接 MIG/mcu 的硬件维护用。
    wire dcache_invalidate = 1'b0;
    wire l2_maint_internal = 1'b0;
    wire [1:0] l2_maint_source = {l2_maint_internal, 1'b0} | l2_maint;
//---- 排空判据：只喂 rob 的提交口，那边先打一拍再用（见 rob 的 drain_ok 端口注释）----
//  · elsq_retired_pending：ELSQ 里还压着"比队头老、已退休、没排出去"的访存项
//  · bus_hold_in        ：LSU 的 AHB 主设备手上还有一笔没被接受（= 非缓存写还在飞）
//  · l2_write_queue_empty：L2 的 4 深写队列空了 ⇒ 缓存写已经落到后端
//  · ~l2_busy           ：L2 不在回填/写穿途中（软件失效要用；fence.i 顺带一起等，代价可忽略）
    wire elsq_retired_pending_lsu;
    wire l2_write_queue_empty;
    wire drain_ok = ~elsq_retired_pending_lsu & ~bus_hold_in & l2_write_queue_empty & ~l2_busy;

//==============================================================================
    icache #(.IC_INDEX_B(IC_INDEX_B)) u_icache (
        .clk(clk), .rst(rst),
        .pc_address_0(pc_address_0),
        .redir_frontend(redir_frontend), .irq_kill(irq_kill),
        .invalidate(icache_invalidate),     // ★ 2026-11-13：fence.i / 软件失效原语
        .mem_valid(mem_valid_icache), .mem_data(mem_data_icache),
        .fifo_full(full_iffu),
        .instruction_0(instruction_0_icache), .instruction_1(instruction_1_icache),
        .instruction_address_0(instruction_addr0_icache), .instruction_address_1(instruction_addr1_icache),
//★ 预测随地址进 icache、随指令出 icache（就是这一对指令的延长位，不是另打一拍的旁路）
        .btb_take_0(btb_take_0), .btb_take_1(btb_take_1),
        .btb_target_0(btb_target_0), .btb_target_1(btb_target_1),

        .prediction_taken_0(dpred0_icache), .prediction_taken_1(dpred1_icache),
        .prediction_jump_target_0(djpred0_icache), .prediction_jump_target_1(djpred1_icache),
        .instruction_valid_0(instruction_valid0_icache), .instruction_valid_1(instruction_valid1_icache),
        .block_offset(block_offset_icache),
        .busy(busy_icache),
        .icache_miss(icache_miss_icache),
        .deliver_wrong_path(deliver_wrong_path_icache),
        .mem_req(mem_req_icache), .mem_write_enable(mem_write_enable_icache),
        .mem_address(mem_address_icache), .mem_write_data(mem_wdata_icache), .mem_byte_enable(mem_byte_enable_icache)
    );

//★ 2026-11-12：u_itcm 撤掉。指令侧的后备存储改由下面的 64KB 统一 L2 顶替
//  （itcm 与 L2 的 I 口同形：mem_req 给一行起始地址，连吐 32 个字）。

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
        .redir_frontend(redir_frontend), .irq_kill(irq_kill), .exec_in(1'b1),
        .instruction_0(instruction_0_icache), .instruction_1(instruction_1_icache),
        .inst_valid_0(instruction_valid0_icache), .inst_valid_1(instruction_valid1_icache),
        .block_offset(block_offset_icache),
        .slot_replay(deliver_wrong_path_icache),
//★ 必须和 push_br_pred_0 用同一个值：队满被按住的那几拍 pc_take0 是 0（那拍没有改向动作），
//  但那条分支【当时】确实跳了 —— 用现采的值会让 lane1（错路那条）被放进来（实测 +400 条/千轮）。
//  pr_l* 就是"随这一对指令走的那一份"，按住时它跟着一起按住。
//块内截断现在由 iffu 自己按 push_branch_prediction_* 算，不再需要单独一根 lane0_take。
//★ 错路作废：icache 交付组里的 dwp 直接给 —— 它与指令同一个寄存器、同一个门，天然带地址。
//  顶层那套 pair_drop + pair_waddr（另建寄存器 + 地址判据）整段删掉了。
//★ 队列里存的地址是**"指令自己地址 + 4"**（全核 aux_addr 口径：jal/link/异常 pc 都靠它，
//  分支立即数在 idu 里已经预减过 4）。icache 交付的是指令自己的地址 ⇒ 进队前在这里 +4。
        .push_address_0(current_address_0 + 32'd4), .push_address_1(current_address_1 + 32'd4),
//★ 随指令记进队列的"预测跳"必须就是 pc 这一拍【实际照做】的那一个。
//  两份预测不一致的后果：表里没条目时 pc 顺下去、指令却被标成"预测跳"，真跳过时
//  br2 判"预测对了" ⇒ 永不改向 ⇒ 整核一路顺序跑（实测：3000 拍里一次改向都没有）。
        .push_branch_prediction_0(dpred0_icache), .push_branch_prediction_1(dpred1_icache),
        .push_jump_link_register_prediction_0(djpred0_icache),
        .push_jump_link_register_prediction_1(djpred1_icache),
        .advance(advance_idu),
        .half0_instruction(h0_instruction), .half0_address(h0_address), .half0_jump_link_register_prediction(h0_jump_link_register_prediction),
        .half0_branch_prediction(h0_branch_prediction),
        .half1_instruction(h1_instruction), .half1_address(h1_address), .half1_jump_link_register_prediction(h1_jump_link_register_prediction),
        .half1_branch_prediction(h1_branch_prediction),
        .half2_instruction(h2_instruction), .half2_address(h2_address), .half2_jump_link_register_prediction(h2_jump_link_register_prediction),
        .half2_branch_prediction(h2_branch_prediction),
        .half0_valid(h0_valid), .half1_valid(h1_valid), .half2_valid(h2_valid),
        .full(full_iffu)
    );

//==============================================================================
// 译码级
//==============================================================================
    rename u_rename (
        .clk(clk), .rst(rst),
        .redir_frontend(redir_frontend), .redir_backend_0(redir_backend_0), .irq_kill(irq_kill),
        .exec_in(1'b1), .stall_rob_full_in(rob_full),
        .redir_backend_1(redir_backend_1),
//★ idu 访存 FIFO 的清零吃 redir_backend_1（B 域再打一拍，相对 A 是 T+2）。
//  d1 只在 T+1 有效 —— 接错相位会拿到"没改向时"的全亮默认值，等于又变回整窗清空。
//冲刷的年龄边界：输出级里挂着的那条比冲刷点老时不许清（见 idu 里 keep_* 的注释）
//本树没有保留站：弹出量由发射级的 issue_pop 计（credit 走 acc_in）
        .store_pop({1'b0, issue_pop_0} + {1'b0, issue_pop_1}),
        .store_flush_pop(4'd0),
        .credit_stall_out(),
        .advance(advance_idu),
        .lane0_load_o(),

        .instruction_in_0(h0_instruction), .address_in_0(h0_address), .instruction_valid_0(h0_valid),
        .branch_prediction_in_0(h0_branch_prediction), .jump_link_register_prediction_in_0(h0_jump_link_register_prediction),
        .index_in_0(alloc_index_0), .generation_in_0(alloc_generation_0),
//顺序发射：发射级在 Q 端等操作数，D 侧永远"就绪"
        .ready1_in_0(1'b1), .ready2_in_0(1'b1),
        .accept_in_0(accept_0),
        .station_room_0_valid(station_room_0_valid),
        .lane_valid_0(),
        .instruction_out_0(), .auxiliary_address_out_0(),
        .branch_prediction_taken_out_0(),
        .source_a_out_0(), .source_b_out_0(),
        .index_out_0(), .generation_out_0(),
        .payload_valid_out_0(),
        .alloc_en_0(alloc_enable_0), .alloc_we_0(alloc_write_enable_0),
        .alloc_jmp_0(alloc_jmp_0), .alloc_pend_0(alloc_pending_0),
        .alloc_ecall_0(alloc_ecall_0), .alloc_ebreak_0(alloc_ebreak_0),
        .alloc_illegal_0(alloc_illegal_0), .alloc_mret_0(alloc_mret_0),
        .alloc_fence_0(alloc_fence_0), .alloc_fence_i_0(alloc_fence_i_0),
        .alloc_rd_0(alloc_destination_register_0),
        .alloc_csr_0(alloc_csr_0),
        .alloc_phys_0(alloc_phys_0), .alloc_pc_1(alloc_pc_1),
        .dec_class_0(), .dec_we_0(), .dec_rd_0(),
        .dec_is_st_0(), .dec_mulop_0(), .dec_lsf3_0(),
        .dec_func4_0(), .dec_csrf3_0(),
        .dec_csrw_en_0(),
        .dec_csrw_wr_0(), .dec_csra_0(),
        .dec_jval_0(), .dec_brf_0(),
        .dec_jalf_0(), .dec_jalfr_0(),
        .dec_bjuf4_0(),
        .dec_r1_imm_0(), .dec_r2_imm_0(),
        .dec_r1_immv_0(), .dec_r2_immv_0(),

        .instruction_in_1(h1_instruction), .address_in_1(h1_address), .instruction_valid_1(h1_valid),
        .branch_prediction_in_1(h1_branch_prediction), .jump_link_register_prediction_in_1(h1_jump_link_register_prediction),
        .index_in_1(alloc_index_1), .generation_in_1(alloc_generation_1),
        .instruction_in_2(h2_instruction), .address_in_2(h2_address), .instruction_valid_2(h2_valid),
        .branch_prediction_in_2(h2_branch_prediction), .jump_link_register_prediction_in_2(h2_jump_link_register_prediction),
        .index_in_2(alloc_index_2), .generation_in_2(alloc_generation_2),
        .ready1_in_1(1'b1), .ready2_in_1(1'b1),
        .accept_in_1(accept_1),
        .station_room_1_valid(station_room_1_valid),
        .lane_valid_1(),
        .instruction_out_1(), .auxiliary_address_out_1(),
        .branch_prediction_taken_out_1(),
        .source_a_out_1(), .source_b_out_1(),
        .index_out_1(), .generation_out_1(),
        .payload_valid_out_1(),
        .alloc_en_1(alloc_enable_1), .alloc_we_1(alloc_write_enable_1),
        .alloc_jmp_1(alloc_jmp_1), .alloc_pend_1(alloc_pending_1),
        .alloc_ecall_1(alloc_ecall_1), .alloc_ebreak_1(alloc_ebreak_1),
        .alloc_illegal_1(alloc_illegal_1), .alloc_mret_1(alloc_mret_1),
        .alloc_fence_1(alloc_fence_1), .alloc_fence_i_1(alloc_fence_i_1),
        .alloc_rd_1(alloc_destination_register_1),
        .alloc_csr_1(alloc_csr_1),
        .alloc_phys_1(alloc_phys_1),
        .alloc_en_2(alloc_enable_2), .alloc_we_2(alloc_write_enable_2),
        .alloc_jmp_2(alloc_jmp_2), .alloc_pend_2(alloc_pending_2),
        .alloc_ecall_2(alloc_ecall_2), .alloc_ebreak_2(alloc_ebreak_2),
        .alloc_illegal_2(alloc_illegal_2), .alloc_mret_2(alloc_mret_2),
        .alloc_fence_2(alloc_fence_2), .alloc_fence_i_2(alloc_fence_i_2),
        .alloc_rd_2(alloc_destination_register_2),
        .alloc_csr_2(alloc_csr_2),
        .alloc_phys_2(alloc_phys_2),
        .alloc_owner_0(alloc_owner_0), .alloc_owner_1(alloc_owner_1), .alloc_owner_2(alloc_owner_2),
        .dec_class_1(), .dec_we_1(), .dec_rd_1(),
        .dec_is_st_1(), .dec_mulop_1(), .dec_lsf3_1(),
        .dec_func4_1(), .dec_csrf3_1(),
        .dec_csrw_en_1(),
        .dec_csrw_wr_1(), .dec_csra_1(),
        .dec_jval_1(), .dec_brf_1(),
        .dec_jalf_1(), .dec_jalfr_1(),
        .dec_bjuf4_1(),
        .dec_r1_imm_1(), .dec_r2_imm_1(),
        .dec_r1_immv_1(), .dec_r2_immv_1(),
        .lsu_pop_in(lsu_pop_isq),
        .dsq_room(dsq_room),
        .push0_valid(push0_valid), .push0_data(push0_data),
        .push1_valid(push1_valid), .push1_data(push1_data),
        .push2_valid(push2_valid), .push2_data(push2_data),
        .payload_fresh_out_0(), .payload_fresh_out_1(),
        .write_enable_a_in(commit_we0), .write_enable_b_in(commit_we1),
        .commit_rd0_in(commit_rd0), .commit_rd1_in(commit_rd1),
        .commit_phys0_in(commit_phys0), .commit_phys1_in(commit_phys1),
        .squash_arch_hit_in(squash_arch_hit), .squash_arch_phys_in(squash_arch_phys),
        .squash_survive_hit_in(squash_survive_hit), .alloc_stop_in(redirect_stop_alloc),
        .rob_full_in(rob_full), .rob_room_2_in(rob_room_2),
        .phys_a_out_0(), .phys_b_out_0(),
        .phys_a_out_1(), .phys_b_out_1(),
        .phys_d_out_0(), .phys_d_out_1(),
        .alloc_onehot_0(), .arch_next_onehot_0(),
        .lsu_instruction_out(),
        .lsu_auxiliary_address_out(),
        .lsu_immediate_out(),
        .lsu_source_a_out(), .lsu_source_b_out(),
        .lsu_destination_register_out(),
        .lsu_load_store_function3_out(),
        .lsu_is_store_out(),
        .lsu_write_enable_out(),
        .lsu_index_out(),
        .lsu_generation_out(),
        .lsu_valid_out(),
        .lsu_phys_a_out(), .lsu_phys_b_out(),
        .lsu_phys_d_out(),
//---- 与 rename 的口（rename 在本层例化，位置是 iffu 出队寄存器与 idu 的 S 级寄存器之间）----













        .phys_rd_0(phys_rd_0), .phys_rd_1(phys_rd_1),
        .phys_rd_2(phys_rd_2), .phys_rd_3(phys_rd_3),
        .lookup_a_phys(lookup_a_phys), .lookup_b_phys(lookup_b_phys),
        .lookup_c_phys(lookup_c_phys), .lookup_d_phys(lookup_d_phys),
        .lookup_e_phys(lookup_e_phys), .lookup_f_phys(lookup_f_phys),
        .bank_empty_0(bank_empty_0), .bank_empty_1(bank_empty_1),
        .bank_empty_2(bank_empty_2), .bank_empty_3(bank_empty_3),
        .bank_low_even(bank_low_even), .bank_low_odd(bank_low_odd),
        .alloc_onehot(alloc_onehot), .arch_next_onehot(arch_next_onehot)
    );

//==============================================================================
// 发射级
//==============================================================================
//==============================================================================
// 分配队列 dsq：rename（译码+改名+分配） → 队列 → idu（分 lane + 推站）
//   入口条目身份已定死（物理号 / ROB 槽 / 类别），出口按类取最老。
//   iffu 的背压只看 dsq 的剩余空间，分配结论不再穿回前端。
//==============================================================================
    wire [1:0]  dsq_room;
    wire        push0_valid, push1_valid, push2_valid;
    wire [132:0] push0_data, push1_data, push2_data;
    wire        dq0_valid, dq1_valid, dqm_valid;
    wire [132:0] dq0_data, dq1_data, dqm_data;
    wire        dq0_pop, dq1_pop, dqm_pop;

    dsq #(.PW(133)) u_dsq (
        .clk(clk), .rst(rst), .flush(irq_kill | redir_backend_0),
        .push0_valid(push0_valid), .push0_data(push0_data),
        .push1_valid(push1_valid), .push1_data(push1_data),
        .push2_valid(push2_valid), .push2_data(push2_data),
        .room(dsq_room),
        .lane0_valid(dq0_valid), .lane0_data(dq0_data), .lane0_pop(dq0_pop),
        .lane1_valid(dq1_valid), .lane1_data(dq1_data), .lane1_pop(dq1_pop),
        .mem_valid(dqm_valid),   .mem_data(dqm_data),   .mem_pop(dqm_pop)
    );

    idu u_idu (
        .clk(clk), .rst(rst),
        .redir_frontend(redir_frontend), .redir_backend_0(redir_backend_0),
        .redir_backend_1(redir_backend_1), .irq_kill(irq_kill),
        .exec_in(1'b1),
        .store_pop({1'b0, issue_pop_0} + {1'b0, issue_pop_1}),
        .store_flush_pop(4'd0),
        .dq0_valid(dq0_valid), .dq0_data(dq0_data), .dq0_pop(dq0_pop),
        .dq1_valid(dq1_valid), .dq1_data(dq1_data), .dq1_pop(dq1_pop),
        .dqm_valid(dqm_valid), .dqm_data(dqm_data), .dqm_pop(dqm_pop),
        .accept_in_0(accept_0), .accept_in_1(accept_1),
        .lane_valid_0(lane_valid_0), .lane_valid_1(lane_valid_1),
        .instruction_out_0(instruction_out_0), .auxiliary_address_out_0(auxiliary_address_out_0),
        .branch_prediction_taken_out_0(branch_prediction_taken_out_0),
        .source_a_out_0(rs1_idu_0), .source_b_out_0(rs2_idu_0),
        .index_out_0(index_out_0), .generation_out_0(generation_out_0),
        .phys_a_out_0(phys_a_out_0), .phys_b_out_0(phys_b_out_0), .phys_d_out_0(phys_d_out_0),
        .payload_valid_out_0(), .payload_fresh_out_0(payload_fresh_0),
        .dec_class_0(decoded_class_0), .dec_we_0(decoded_write_enable_0),
        .dec_rd_0(decoded_destination_register_0), .dec_is_st_0(decoded_is_store_0),
        .dec_mulop_0(decoded_mulop_0), .dec_lsf3_0(decoded_load_store_function3_0),
        .dec_func4_0(decoded_func4_0), .dec_csrf3_0(decoded_csr_function3_0),
        .dec_csrw_en_0(decoded_csr_write_enable_0), .dec_csrw_wr_0(decoded_csr_write_write_0),
        .dec_csra_0(decoded_csr_address_0), .dec_jval_0(decoded_jump_value_0),
        .dec_brf_0(decoded_branch_flag_0), .dec_jalf_0(decoded_jump_link_flag_0),
        .dec_jalfr_0(decoded_jump_link_register_flag_0), .dec_bjuf4_0(decoded_bjuf4_0),
        .dec_r1_imm_0(decoded_r1_immediate_0), .dec_r2_imm_0(decoded_r2_immediate_0),
        .dec_r1_immv_0(decoded_r1_immediate_value_0), .dec_r2_immv_0(decoded_r2_immediate_value_0),
        .instruction_out_1(instruction_out_1), .auxiliary_address_out_1(auxiliary_address_out_1),
        .branch_prediction_taken_out_1(branch_prediction_taken_out_1),
        .source_a_out_1(rs1_idu_1), .source_b_out_1(rs2_idu_1),
        .index_out_1(index_out_1), .generation_out_1(generation_out_1),
        .phys_a_out_1(phys_a_out_1), .phys_b_out_1(phys_b_out_1), .phys_d_out_1(phys_d_out_1),
        .payload_valid_out_1(), .payload_fresh_out_1(payload_fresh_1),
        .dec_class_1(decoded_class_1), .dec_we_1(decoded_write_enable_1),
        .dec_rd_1(decoded_destination_register_1), .dec_is_st_1(decoded_is_store_1),
        .dec_mulop_1(decoded_mulop_1), .dec_lsf3_1(decoded_load_store_function3_1),
        .dec_func4_1(decoded_func4_1), .dec_csrf3_1(decoded_csr_function3_1),
        .dec_csrw_en_1(decoded_csr_write_enable_1), .dec_csrw_wr_1(decoded_csr_write_write_1),
        .dec_csra_1(decoded_csr_address_1), .dec_jval_1(decoded_jump_value_1),
        .dec_brf_1(decoded_branch_flag_1), .dec_jalf_1(decoded_jump_link_flag_1),
        .dec_jalfr_1(decoded_jump_link_register_flag_1), .dec_bjuf4_1(decoded_bjuf4_1),
        .dec_r1_imm_1(decoded_r1_immediate_1), .dec_r2_imm_1(decoded_r2_immediate_1),
        .dec_r1_immv_1(decoded_r1_immediate_value_1), .dec_r2_immv_1(decoded_r2_immediate_value_1),
        .lsu_pop_in(lsu_pop_isq),
        .lsu_auxiliary_address_out(lsu_auxiliary_address_idu),
        .lsu_immediate_out(lsu_immediate_idu),
        .lsu_source_a_out(lsu_source_a_idu), .lsu_source_b_out(lsu_source_b_idu),
        .lsu_destination_register_out(lsu_destination_register_idu),
        .lsu_load_store_function3_out(lsu_load_store_function3_idu),
        .lsu_is_store_out(lsu_is_store_idu),
        .lsu_write_enable_out(lsu_write_enable_idu),
        .lsu_index_out(lsu_index_idu),
        .lsu_generation_out(lsu_generation_idu),
        .lsu_valid_out(lsu_valid_idu),
        .lsu_phys_a_out(lsu_phys_a_idu), .lsu_phys_b_out(lsu_phys_b_idu),
        .lsu_phys_d_out(lsu_phys_d_idu)
    );
    isq u_isq (
        .clk(clk), .rst(rst),
        .redir_backend_0(redir_backend_0), .redir_backend_1(redir_backend_1), .irq_kill(irq_kill),
        
        .stage_advance(lane_valid_0), .stage_advance_lane1(lane_valid_1),

        .rs1_in(rs1_idu_0), .rs2_in(rs2_idu_0),
        .aux_addr_in(auxiliary_address_out_0),
        .branch_prediction_taken_in(branch_prediction_taken_out_0),
        .index_in(index_out_0), .alloc_generation_in(generation_out_0),
        .decoded_class(decoded_class_0), .decoded_write_enable(decoded_write_enable_0), .decoded_destination_register(decoded_destination_register_0),
        .decoded_multiply_operation(decoded_mulop_0),
        .decoded_function4(decoded_func4_0), .decoded_csr_function3(decoded_csr_function3_0),
        .decoded_csr_write_enable(decoded_csr_write_enable_0),
        .decoded_csr_write_commit(decoded_csr_write_write_0), .decoded_csr_address(decoded_csr_address_0),
        .decoded_jump_value(decoded_jump_value_0), .decoded_branch_flag(decoded_branch_flag_0),
        .decoded_jump_link_flag(decoded_jump_link_flag_0), .decoded_jump_link_register_flag(decoded_jump_link_register_flag_0),
        .decoded_branch_jump_function4(decoded_bjuf4_0),
        .decoded_r1_immediate(decoded_r1_immediate_0), .decoded_r2_immediate(decoded_r2_immediate_0),
        .decoded_r1_immediate_value(decoded_r1_immediate_value_0), .decoded_r2_immediate_value(decoded_r2_immediate_value_0),

        .rs1_in_lane1(rs1_idu_1), .rs2_in1(rs2_idu_1),
        .aux_addr_in1(auxiliary_address_out_1),
        .branch_prediction_taken_in_lane1(branch_prediction_taken_out_1),
        .index_in_lane1(index_out_1), .alloc_generation_in_lane1(generation_out_1),
        .decoded_class_lane1(decoded_class_1), .decoded_write_enable_lane1(decoded_write_enable_1), .decoded_destination_register_lane1(decoded_destination_register_1),
        .decoded_multiply_operation_lane1(decoded_mulop_1),
        .decoded_function4_lane1(decoded_func4_1), .decoded_csr_function3_lane1(decoded_csr_function3_1),
        .decoded_csr_write_enable_lane1(decoded_csr_write_enable_1),
        .decoded_csr_write_commit_lane1(decoded_csr_write_write_1), .decoded_csr_address_lane1(decoded_csr_address_1),
        .decoded_jump_value_lane1(decoded_jump_value_1), .decoded_branch_flag_lane1(decoded_branch_flag_1),
        .decoded_jump_link_flag_lane1(decoded_jump_link_flag_1), .decoded_jump_link_register_flag_lane1(decoded_jump_link_register_flag_1),
        .decoded_branch_jump_function4_lane1(decoded_bjuf4_1),
        .decoded_r1_immediate_lane1(decoded_r1_immediate_1), .decoded_r2_immediate_lane1(decoded_r2_immediate_1),
        .decoded_r1_immediate_value_lane1(decoded_r1_immediate_value_1), .decoded_r2_immediate_value_lane1(decoded_r2_immediate_value_1),

        .mem_rs1_in(lsu_source_a_idu), .mem_rs2_in(lsu_source_b_idu),
        .mem_destination_register_in(lsu_destination_register_idu),
        .mem_auxiliary_address_in(lsu_auxiliary_address_idu),
        .mem_immediate_in(lsu_immediate_idu),
        .mem_index_in(lsu_index_idu), .mem_generation_in(lsu_generation_idu),
        .mem_is_store_in(lsu_is_store_idu),
        .mem_load_store_function3_in(lsu_load_store_function3_idu),
        .mem_valid_in(lsu_valid_idu),
//★ 真 PRF：访存条目里的两个源物理号（idu 在进队那一拍就查好了，随条目走）
        .mem_phys_a_in(lsu_phys_a_idu), .mem_phys_b_in(lsu_phys_b_idu),
        .mem_pop_out(lsu_pop_isq),

//结果口：只接 32 位数据 + 前送身份（按物理号）
        .data_alu_early(source_index_alu_early_0),
        .data_alu_lane1_early(source_index_alu_early_1),
        .data_ld_early(load_data_lsu),
//旁路：只吃 ALU 自己那一组 + LOAD；mdu 的结果绝不进旁路网
        .write_enable_alu_early(write_enable_alu_early_0),       .phys_alu_early_0(phys_alu_early_0),
        .write_enable_alu_lane1_early(write_enable_alu_early_1), .phys_alu_early_1(phys_alu_early_1),
//★ 2026-11-11：LOAD 的前送/唤醒从【早写口 s2】搬到【返回级】。选通与数据必须同一级，
//  所以这三根和下面的 load_write 一起换，不是只换数据。
        .write_enable_ld_early(load_write_return),               .ld_phys_early(prf_ld_phys_wb),
//唤醒：值表写口这一拍真的落笔（下一拍这个号一定读得到）
        .result_write_valid_0(prf_result_write_0), .result_write_phys_0(prf_wb_phys_0),
        .result_is_divide_0(result_is_divide_0), .result_is_divide_1(result_is_divide_1),
        .result_write_valid_1(prf_result_write_1), .result_write_phys_1(prf_wb_phys_1),
        .load_write(load_write_return),
        .load_write_phys(prf_ld_phys_wb),

//D 级那一格两个源的物理号 + PRF 读口 / 就绪位
        .rs1_phys_in(phys_a_out_0),       .rs2_phys_in(phys_b_out_0),
        .rs1_phys_in_lane1(phys_a_out_1), .rs2_phys_in_lane1(phys_b_out_1),
        .phys_d_in(phys_d_out_0), .phys_d_in_lane1(phys_d_out_1),
        .lsu_phys_d_in(lsu_phys_d_idu),
        .lsu_phys_d(lsu_phys_d_isq),
        .divide_wake_valid(divide_wake_valid), .divide_wake_phys(divide_wake_phys),
        .prf_read_a_0_phys(prf_read_a_0_phys), .prf_read_b_0_phys(prf_read_b_0_phys),
        .prf_read_a_1_phys(prf_read_a_1_phys), .prf_read_b_1_phys(prf_read_b_1_phys),
        .prf_read_a_m_phys(prf_read_a_m_phys), .prf_read_b_m_phys(prf_read_b_m_phys),
        .prf_data_a_0(prf_data_a_0), .prf_data_b_0(prf_data_b_0),
        .prf_data_a_1(prf_data_a_1), .prf_data_b_1(prf_data_b_1),
        .prf_data_a_mem(prf_data_a_mem), .prf_data_b_mem(prf_data_b_mem),
        .alloc_onehot_in(alloc_onehot),

//---- 冲刷 / 陷阱 / 年龄基准 ----
        .head_one_hot_registered(head_one_hot_registered),
        .csr_write_landing(csr_commit_write),
//---- 单元就绪 / load 早唤醒 ----
        .lsu_full(~carriage_free_lsu),
        .bypass_load_a_0(bypass_load_a_0), .bypass_load_b_0(bypass_load_b_0),
        .bypass_load_a_1(bypass_load_a_1), .bypass_load_b_1(bypass_load_b_1),
        .mdu_mul_ok_0(mul_ready_0), .mdu_div_ok_0(div_ready_0),
        .mdu_mul_ok_1(mul_ready_1), .mdu_div_ok_1(div_ready_1),
        .bju_ok_0(1'b1), .bju_ok_1(1'b1),
        .issue_valid_0(issue_valid_0), .issue_index_0(issue_index_0),
        .issue_generation_0(issue_generation_0), .write_enable_0(write_enable_0), .sel_class_0(),
        .opcode_0(), .function10_0(function10_0), .function10_load_store_0(function10_load_store_0),
        .alu_function4_0(alu_func4_0), .csr_func3_0(csr_func3_0),
        .csr_write_enable_0(csr_write_enable_0), .csr_write_action_0(csr_write_action_0),
        .csr_address_0(csr_address_0),
        .auxiliary_address_0(auxiliary_address_0), .off_mem_0(), .beq_off_0(beq_offset_0),
        .jump_link_register_prediction_address_0(jump_link_register_prediction_address_0), .jump_link_target_0(jump_link_target_0),
        .operand_data_a_0(operand_data_a_0), .operand_data_b_0(operand_data_b_0),
        .branch_flag_0(branch_flag_0), .jump_link_flag_0(jump_link_flag_0), .jump_link_register_flag_0(jump_link_register_flag_0),
        .branch_prediction_taken_0(branch_prediction_taken_0),
        .bju_function4_0(bju_func4_0),
        .mul_go_0(mul_go_0), .div_go_0(div_go_0),

        .issue_valid_1(issue_valid_1), .issue_index_1(issue_index_1),
        .issue_generation_1(issue_generation_1), .write_enable_1(write_enable_1), .sel_class_1(),
        .opcode_1(), .function10_1(function10_1), .function10_load_store_1(function10_load_store_1),
        .alu_function4_1(alu_func4_1), .csr_func3_1(csr_func3_1),
        .csr_write_enable_1(csr_write_enable_1), .csr_write_action_1(csr_write_action_1),
        .csr_address_1(csr_address_1),
        .auxiliary_address_1(auxiliary_address_1), .off_mem_1(), .beq_off_1(beq_offset_1),
        .jump_link_register_prediction_address_1(jump_link_register_prediction_address_1), .jump_link_target_1(jump_link_target_1),
        .operand_data_a_1(operand_data_a_1), .operand_data_b_1(operand_data_b_1),
        .branch_flag_1(branch_flag_1), .jump_link_flag_1(jump_link_flag_1), .jump_link_register_flag_1(jump_link_register_flag_1),
        .branch_prediction_taken_1(branch_prediction_taken_1),
        .bju_function4_1(bju_func4_1),
        .mul_go_1(mul_go_1), .div_go_1(div_go_1),

        .lsu_valid(lsu_valid), .lsu_lane(lsu_lane), .lsu_opcode(lsu_opcode), .lsu_function10(lsu_function10),
        .lsu_index(lsu_index), .lsu_generation(lsu_generation),
        .lsu_pc(lsu_pc),
        .lsu_a(lsu_a), .lsu_b(lsu_b), .lsu_imm(lsu_imm),

        .accept_0(accept_0), .accept_1(accept_1),
        .station_room_0_valid(station_room_0_valid), .station_room_1_valid(station_room_1_valid),
        .issue_pop_0(issue_pop_0), .issue_pop_1(issue_pop_1),
        .csr_payload_0(csr_payload_0), .csr_payload_1(csr_payload_1)
    );

//==============================================================================
// 真 PRF：64 个物理号 × 32 位的寄存器堆
//   6 个组合读口（lane0 a/b、lane1 a/b、访存流 a/b），3 个写口（lane0 结果 / lane1 结果 / LOAD）。
//   写口的物理号由 rob 按槽查出来 —— 结果口上带的还是槽号，物理号是实现细节，放在 rob 里最省。
//   live：冲刷之后在飞的结果还会落完，那些项的号可能已经被回收再分配，晚到的死写一律拦掉。
//==============================================================================
    prf u_prf (
        .clk(clk), .rst(rst),
        .read_a_0_phys(prf_read_a_0_phys), .read_b_0_phys(prf_read_b_0_phys),
        .read_a_1_phys(prf_read_a_1_phys), .read_b_1_phys(prf_read_b_1_phys),
        .read_a_m_phys(prf_read_a_m_phys), .read_b_m_phys(prf_read_b_m_phys),
        .read_c_phys(csr_commit_source_field),
        .read_a_0_data(prf_data_a_0), .read_b_0_data(prf_data_b_0),
        .read_a_1_data(prf_data_a_1), .read_b_1_data(prf_data_b_1),
        .read_a_m_data(prf_data_a_mem), .read_b_m_data(prf_data_b_mem),
        .read_c_data(prf_read_c_data),
//★ 2026-11-10：阵列按写者拆开之后，读口要先知道这个物理号由哪个写口写。owner 在【分配那一拍】定，
//  与 ROB 的 allocation_we 同口径（有写回才算），phys 与 owner 必须同一拍、同一口取。
        .alloc_write_0(alloc_enable_0 & alloc_write_enable_0), .alloc_phys_0(alloc_phys_0), .alloc_owner_0(alloc_owner_0),
        .alloc_write_1(alloc_enable_1 & alloc_write_enable_1), .alloc_phys_1(alloc_phys_1), .alloc_owner_1(alloc_owner_1),
        .alloc_write_2(alloc_enable_2 & alloc_write_enable_2), .alloc_phys_2(alloc_phys_2), .alloc_owner_2(alloc_owner_2),
        .load_write(prf_load_write), .load_write_phys(prf_load_write_phys),

        .write_en_0(result_port_0_enable), .write_live_0(prf_wb_live_0),
        .write_phys_0(prf_wb_phys_0),      .write_data_0(result_port_0_data),
        .write_en_1(result_port_1_enable), .write_live_1(prf_wb_live_1),
        .write_phys_1(prf_wb_phys_1),      .write_data_1(result_port_1_data),
        .write_en_2(load_write_enable_lsu), .write_live_2(prf_ld_live_wb),
        .write_phys_2(prf_ld_phys_wb),            .write_data_2(load_data_lsu),
        .write_en_3(load_write_enable_early_lsu), .write_live_3(prf_ld_early_live),
        .write_phys_3(ld_phys_early),             .write_data_3(load_data_early_lsu),
//★ ALU 专属写口：吃 ALU 自己的组合早出口（不再经结果口仲裁）
        .write_en_4(write_enable_alu_early_0), .write_live_4(prf_alu_early_live_0),
        .write_phys_4(phys_alu_early_0),       .write_data_4(source_index_alu_early_0),
        .write_en_5(write_enable_alu_early_1), .write_live_5(prf_alu_early_live_1),
        .write_phys_5(phys_alu_early_1),       .write_data_5(source_index_alu_early_1)

    );

//==============================================================================
// 执行单元：每 lane 一个 exu（alu / mdu / bju 收进一层，结果口在层内仲裁）
//   对外只剩一个值表写口：div > mul > alu 抢，抢不到的进本类影子（ALU 最低，本来就是旁路源）
//   早唤醒那份（*_early）也只剩一根：值表按结果口写，不再按类各写一份。
//==============================================================================
    exu u_exu_0 (
        .clk(clk), .rst(rst),
        .redir_backend_0(redir_backend_0), .redir_backend_1(redir_backend_1), .irq_kill(irq_kill), .flush_index_in(flush_index),

        .write_enable_in(write_enable_0), .jal_flag(jump_link_flag_0), .jalr_flag(jump_link_register_flag_0),
        .cs_wr_en(csr_write_enable_0),
        .index_in(issue_index_0), .generation_in(issue_generation_0),
        .alu_function4(alu_func4_0), .csr_function3(csr_func3_0),
        .auxiliary_address_in(auxiliary_address_0),
        .source_a_data(operand_data_a_0), .r2_data(operand_data_b_0), .cs_data(csr_data_combinational),
        .bypass_load_a(bypass_load_a_0), .bypass_load_b(bypass_load_b_0), .load_data_forward(load_data_early_lsu),

        .branch_unit_function4_in(bju_func4_0),
        .beq_offset_in(beq_offset_0),
        .jump_link_register_prediction_address_in(jump_link_register_prediction_address_0),
        .jump_link_target_in(jump_link_target_0),
        .branch_prediction_taken_in(branch_prediction_taken_0),
        .advance_in(issue_valid_0),

        .multiply_go(mul_go_0), .multiply_function10(function10_0),
        .divide_go(div_go_0),   .divide_function10(function10_0),

        .alu_index_out(index_alu_0), .alu_generation_out(generation_alu_0),
        .alu_result(source_index_alu_0), .alu_result_csr(csr_result_alu_0),
        .alu_result_early(source_index_alu_early_0),
        .alu_write_enable_early(write_enable_alu_early_0), .alu_index_early(index_alu_early_0), .alu_generation_early(generation_alu_early_0),
        .alu_write_enable(write_enable_alu_0),

        .multiply_data_out(mul_data_0), .multiply_loaded(mul_write_enable_0),
        .multiply_data_early(), .divide_data_early(),
        .multiply_write_enable_early(), .multiply_index_early(), .multiply_generation_early(),
        .divide_write_enable_early(), .divide_index_early(), .divide_generation_early(),
        .divide_wake_early(divide_wake_valid_0), .divide_wake_index_early(divide_wake_index_0),
        .multiply_index_out(mul_index_0), .multiply_generation_out(mul_generation_0),
        .divide_data_out(div_data_0), .divide_loaded(div_write_enable_0),
        .divide_index_out(div_index_0), .divide_generation_out(div_generation_0),
        .multiply_ready(mul_ready_0), .divide_ready(div_ready_0),

        .success(success_0), .branch_failure(branch_fail_0), .jump_link_register_failure(jump_link_register_fail_0),
        .exception_target_misalign(exception_target_misalign_0),
        .jump_target(jump_target_0), .jump_link_register_target_registered(jump_link_register_target_0),
        .branch_pc_index(branch_pc_index_0), .branch_pc_registered(branch_pc_0),
        .jump_link_register_target_valid_registered(jump_link_register_valid_0),
        .branch_prediction_taken_registered(branch_prediction_taken_registered_0),
        .judged_index(bju_index_i_0), .index_registered(bju_index_registered_0),
        .judged_registered(judged_0), .redirect_registered(redir_registered_0), .generation_registered(generation_registered_0),
        .branch_unit_kind(bju_kind_0), .branch_unit_taken(bju_taken_0),
        .branch_unit_pc(bju_pc_0), .branch_unit_target(bju_target_0),
        .exception_pc_registered(exception_pc_registered_0), .exception_pc_judged(exception_pc_i_0),
        .exception_ecall_judged(exception_ecall_0), .exception_ebreak_judged(exception_ebreak_0),
        .exception_illegal_judged(exception_illegal_0), .exception_interrupt_return_judged(exception_interrupt_ret_0),
        .exception_ecall_registered(exception_ecall_registered_0), .exception_ebreak_registered(exception_ebreak_registered_0),
        .exception_illegal_registered(exception_illegal_registered_0), .exception_interrupt_return_registered(exception_interrupt_ret_registered_0),

        .result_write_enable(result_port_0_enable), .result_is_divide(result_is_divide_0), .result_index(result_port_0_index),
        .result_generation(result_port_0_generation), .result_data(result_port_0_data),
        .result_overflow(result_port_0_overflow)
    );

    exu u_exu_1 (
        .clk(clk), .rst(rst),
        .redir_backend_0(redir_backend_0), .redir_backend_1(redir_backend_1), .irq_kill(irq_kill), .flush_index_in(flush_index),

        .write_enable_in(write_enable_1), .jal_flag(jump_link_flag_1), .jalr_flag(jump_link_register_flag_1),
        .cs_wr_en(csr_write_enable_1),
        .index_in(issue_index_1), .generation_in(issue_generation_1),
        .alu_function4(alu_func4_1), .csr_function3(csr_func3_1),
        .auxiliary_address_in(auxiliary_address_1),
        .source_a_data(operand_data_a_1), .r2_data(operand_data_b_1), .cs_data(csr_data_combinational),
        .bypass_load_a(bypass_load_a_1), .bypass_load_b(bypass_load_b_1), .load_data_forward(load_data_early_lsu),

        .branch_unit_function4_in(bju_func4_1),
        .beq_offset_in(beq_offset_1),
        .jump_link_register_prediction_address_in(jump_link_register_prediction_address_1),
        .jump_link_target_in(jump_link_target_1),
        .branch_prediction_taken_in(branch_prediction_taken_1),
        .advance_in(issue_valid_1),

        .multiply_go(mul_go_1), .multiply_function10(function10_1),
        .divide_go(div_go_1),   .divide_function10(function10_1),

        .alu_index_out(index_alu_1), .alu_generation_out(generation_alu_1),
        .alu_result(source_index_alu_1), .alu_result_csr(csr_result_alu_1),
        .alu_result_early(source_index_alu_early_1),
        .alu_write_enable_early(write_enable_alu_early_1), .alu_index_early(index_alu_early_1), .alu_generation_early(generation_alu_early_1),
        .alu_write_enable(write_enable_alu_1),

        .multiply_data_out(mul_data_1), .multiply_loaded(mul_write_enable_1),
        .multiply_data_early(), .divide_data_early(),
        .multiply_write_enable_early(), .multiply_index_early(), .multiply_generation_early(),
        .divide_write_enable_early(), .divide_index_early(), .divide_generation_early(),
        .divide_wake_early(divide_wake_valid_1), .divide_wake_index_early(divide_wake_index_1),
        .multiply_index_out(mul_index_1), .multiply_generation_out(mul_generation_1),
        .divide_data_out(div_data_1), .divide_loaded(div_write_enable_1),
        .divide_index_out(div_index_1), .divide_generation_out(div_generation_1),
        .multiply_ready(mul_ready_1), .divide_ready(div_ready_1),

        .success(success_1), .branch_failure(branch_fail_1), .jump_link_register_failure(jump_link_register_fail_1),
        .exception_target_misalign(exception_target_misalign_1),
        .jump_target(jump_target_1), .jump_link_register_target_registered(jump_link_register_target_1),
        .branch_pc_index(branch_pc_index_1), .branch_pc_registered(branch_pc_1),
        .jump_link_register_target_valid_registered(jump_link_register_valid_1),
        .branch_prediction_taken_registered(branch_prediction_taken_registered_1),
        .judged_index(bju_index_i_1), .index_registered(bju_index_registered_1),
        .judged_registered(judged_1), .redirect_registered(redir_registered_1), .generation_registered(generation_registered_1),
        .branch_unit_kind(bju_kind_1), .branch_unit_taken(bju_taken_1),
        .branch_unit_pc(bju_pc_1), .branch_unit_target(bju_target_1),
        .exception_pc_registered(exception_pc_registered_1), .exception_pc_judged(exception_pc_i_1),
        .exception_ecall_judged(exception_ecall_1), .exception_ebreak_judged(exception_ebreak_1),
        .exception_illegal_judged(exception_illegal_1), .exception_interrupt_return_judged(exception_interrupt_ret_1),
        .exception_ecall_registered(exception_ecall_registered_1), .exception_ebreak_registered(exception_ebreak_registered_1),
        .exception_illegal_registered(exception_illegal_registered_1), .exception_interrupt_return_registered(exception_interrupt_ret_registered_1),

        .result_write_enable(result_port_1_enable), .result_is_divide(result_is_divide_1), .result_index(result_port_1_index),
        .result_generation(result_port_1_generation), .result_data(result_port_1_data),
        .result_overflow(result_port_1_overflow)
    );

//==============================================================================
// 共享访存单元
//==============================================================================
    lsu #(.ADDER_AT_D(ADDER_AT_D)) u_lsu (
        .clk(clk), .rst(rst),
        .redir_backend_0(redir_backend_0), .irq_kill(irq_kill), .exec_in(1'b1),
//★ 总线被按住只在 LSU 内部生效（2026-10-31）：不去停整个前端。
        .bus_hold_in(bus_hold_in),
        .payload_go(lsu_valid),
        .opcode(lsu_opcode), .function10(lsu_function10),
        .idx_in(lsu_index), .gen_in(lsu_generation), .phys_in(lsu_phys_d_isq),
        .source_a_data_final(lsu_a), .source_b_data_final(lsu_b), .source_imm(lsu_imm),
        
        .bus_data_external(bus_data_in_ext), .bus_data_from_dcache(data_dcache),
        .bus_data_from_timer(data_tim),
        .dcache_hold(hold_dcache), .dcache_blocked(request_blocked_dcache), .load_ready_next(load_ready_next_dcache),
//站没了之后"更老的 BJU 在飞"只剩这一个来源（见 spec §6.4）
        .head_one_hot_registered(head_one_hot_registered), .redirect_any(rob_redir_any),
        
        .ready_dcache(load_ready_dcache), .ready_timer(ready_tim), .ready_external(bus_loaded_in),
        .dcache_address_out(dcache_address_lsu),
        .dcache_valid_out(dcache_valid_lsu),
        .store_address_out(store_address_lsu), .store_data_out(store_data_lsu),
        .store_byte_enable_out(store_byte_enable_lsu), .store_valid_out(store_valid_lsu),
        .dcache_store_blocked(store_blocked_dcache),
        .stage1_address_out(bus_address_lsu), .stage1_data_out(bus_data_lsu),
        .stage1_byte_enable_out(bus_byte_enable_lsu), .stage1_we_out(bus_write_enable_lsu), .stage1_valid_out(bus_valid_lsu),
        .load_data_out(load_data_lsu), .loaded(), .load_write_enable(load_write_enable_lsu),
        .load_data_early(load_data_early_lsu),
        .load_write_enable_early(load_write_enable_early_lsu), .ld_index_e(load_index_early_lsu), .ld_generation_e(load_generation_early_lsu),
        .stage_free(carriage_free_lsu),
        .load_wake_registered(load_wake_registered_lsu), .load_wake_index_registered(load_wake_index_lsu), .load_wake_generation_registered(load_wake_generation_lsu),
        .load_nack_registered(), .load_nack_index_registered(), .load_nack_generation_registered(),
        .ld_index(load_index_lsu), .ld_generation(load_generation_lsu), .ld_phys(load_phys_lsu),
        .store_done(store_done_lsu), .store_index_out(store_index_lsu), .store_generation_out(store_generation_lsu),
        .ret0_valid(ret0_valid_rob), .ret0_index(ret0_index_rob), .ret0_generation(ret0_generation_rob),
        .ret1_valid(ret1_valid_rob), .ret1_index(ret1_index_rob), .ret1_generation(ret1_generation_rob),
        .exception_load_store_misalign_out(exception_ldst_misalign_lsu),
        .exception_load_store_store_out(exception_ldst_store_lsu),
        .exception_load_store_address_out(exception_ldst_address_lsu),
        .exception_load_store_index_out(exception_ldst_index_lsu),
        .exception_load_store_pc_out(exception_ldst_pc_lsu),
//★ 2026-11-13（P2b）：只引出给 P3 的排空判据，本阶段没有消费者。
        .elsq_retired_pending(elsq_retired_pending_lsu),
        .pc_in(lsu_pc)
    );

//==============================================================================
// 重排序缓冲
//==============================================================================
    rob #(.DEPTH(ROB_DEPTH)) u_rob (
        .clk(clk), .rst(rst),
        .allocation_en_0(alloc_enable_0), .allocation_en_1(alloc_enable_1),
        .allocation_rd_0(alloc_destination_register_0), .allocation_rd_1(alloc_destination_register_1),
        .allocation_phys_0(alloc_phys_0), .allocation_phys_1(alloc_phys_1),
        .allocation_we_0(alloc_write_enable_0), .allocation_we_1(alloc_write_enable_1),
        .allocation_jmp_0(alloc_jmp_0), .allocation_jmp_1(alloc_jmp_1),
        .allocation_pend_0(alloc_pending_0), .allocation_pend_1(alloc_pending_1),
        .allocation_ecall_0(alloc_ecall_0), .allocation_ebreak_0(alloc_ebreak_0),
        .allocation_illegal_0(alloc_illegal_0), .allocation_mret_0(alloc_mret_0),
        .allocation_fence_0(alloc_fence_0), .allocation_fence_i_0(alloc_fence_i_0),
        .allocation_ecall_1(alloc_ecall_1), .allocation_ebreak_1(alloc_ebreak_1),
        .allocation_illegal_1(alloc_illegal_1), .allocation_mret_1(alloc_mret_1),
        .allocation_fence_1(alloc_fence_1), .allocation_fence_i_1(alloc_fence_i_1),
        .allocation_index_0(alloc_index_0), .allocation_index_1(alloc_index_1),
        .allocation_generation_0(alloc_generation_0), .allocation_generation_1(alloc_generation_1),
        .allocation_csr_0(alloc_csr_0),
        .issue_csr_payload_0(csr_payload_0), .issue_csr_payload_1(csr_payload_1),
        .allocation_csr_1(alloc_csr_1),
        .allocation_pc_0(h0_address), .allocation_pc_1(alloc_pc_1), .allocation_pc_2(h2_address),
        .allocation_en_2(alloc_enable_2), .allocation_rd_2(alloc_destination_register_2),
        .allocation_phys_2(alloc_phys_2), .allocation_we_2(alloc_write_enable_2),
        .allocation_jmp_2(alloc_jmp_2), .allocation_pend_2(alloc_pending_2),
        .allocation_ecall_2(alloc_ecall_2), .allocation_ebreak_2(alloc_ebreak_2),
        .allocation_illegal_2(alloc_illegal_2), .allocation_mret_2(alloc_mret_2),
        .allocation_fence_2(alloc_fence_2), .allocation_fence_i_2(alloc_fence_i_2),
        .allocation_index_2(alloc_index_2), .allocation_generation_2(alloc_generation_2),
        .allocation_csr_2(alloc_csr_2),
        .head_pc_out(head_pc_rob), .head_valid_out(head_valid_rob),

        .bju_judged(judged_0), .bju_index(bju_index_registered_0), .bju_generation(generation_registered_0),
        .bju_redir(redir_registered_0), .bju_tgt(jump_target_0),
        .branch_unit_lane1_redir(redir_registered_1), .branch_unit_lane1_tgt(jump_target_1),
        .redir_backend_0(redir_backend_0), .kill_redirect_target(rob_redir_pc), .redirect_any(rob_redir_any),
        .icache_invalidate(icache_invalidate), .drain_ok(drain_ok),
        .redir_frontend(redir_frontend),
        .kill_redirect_mret(mret_valid),
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

        .irq_kill(irq_kill), .redir_backend_1(redir_backend_1),
        
        .exc0_en(exc0_enable), .exc0_index(exc0_index), .exc0_cause(exc0_cause),
        .exc0_pc(exc0_pc), .exc0_tval(exc0_trap_value),
        .exc1_en(exc1_enable), .exc1_index(exc1_index), .exc1_cause(exc1_cause),
        .exc1_pc(exc1_pc), .exc1_tval(exc1_trap_value),
        .exception_selected_enable(excls_enable), .exception_selected_index(excls_index), .exception_selected_cause(excls_cause),
        .exception_selected_pc(excls_pc), .exception_selected_trap_value(excls_trap_value),

        .commit_write_enable_0(commit_we0), .commit_destination_0(commit_rd0),
        .commit_write_enable_1(commit_we1), .commit_destination_1(commit_rd1),
        .commit_phys0(commit_phys0), .commit_phys1(commit_phys1),
        .commit_csr_valid(commit_csr_valid), .commit_csr_payload(commit_csr_payload),
        .squash_arch_hit(squash_arch_hit), .squash_arch_phys(squash_arch_phys),
        .squash_survive_hit(squash_survive_hit), .redirect_stop_alloc(redirect_stop_alloc),
//★ 真 PRF 的写口：物理号按槽查出来 + 存活判据
        .prf_wb_en_0(result_port_0_enable), .prf_wb_index_0(result_port_0_index), .prf_wb_generation_0(result_port_0_generation),
        .prf_wb_en_1(result_port_1_enable), .prf_wb_index_1(result_port_1_index), .prf_wb_generation_1(result_port_1_generation),
        .prf_ld_index(load_index_lsu), .prf_ld_generation(load_generation_lsu),
        .prf_ld_early_index(load_index_early_lsu), .prf_ld_early_generation(load_generation_early_lsu),
        .prf_ld_early_phys(ld_phys_early), .prf_ld_early_live(prf_ld_early_live),
        .prf_wb_phys_0(prf_wb_phys_0), .prf_wb_phys_1(prf_wb_phys_1), .prf_ld_phys(),
        .prf_wb_live_0(prf_wb_live_0), .prf_wb_live_1(prf_wb_live_1), .prf_ld_live(prf_ld_live_wb),
        .prf_alu_early_index_0(index_alu_early_0), .prf_alu_early_index_1(index_alu_early_1),
        .prf_alu_early_generation_0(generation_alu_early_0), .prf_alu_early_generation_1(generation_alu_early_1),
        .prf_alu_early_live_0(prf_alu_early_live_0), .prf_alu_early_live_1(prf_alu_early_live_1),
        .prf_div_wake_index_0(divide_wake_index_0), .prf_div_wake_index_1(divide_wake_index_1),
        .prf_div_wake_phys_0(divide_wake_phys_0), .prf_div_wake_phys_1(divide_wake_phys_1),
        .prf_alu_early_phys_0(phys_alu_early_0), .prf_alu_early_phys_1(phys_alu_early_1),

        .head_pointer(head_pointer), .full(rob_full), .slot_above_head(above),
        .head_one_hot_registered(head_one_hot_registered), .retire_step(rob_ret_step),
        .empty(rob_empty), .occupancy(occupancy_rob),
        .redir_cause(redir_cause), .trap_cause(trap_cause),
        .trap_pc(trap_pc), .trap_value(trap_value)
    );

//==============================================================================
// 寄存器堆：真 PRF 落地后已删。
//  操作数全部从 u_prf 取；架构值就是 PRF 里 arch_map 指着的那些号，提交口只更新 arch_map
//  与空闲表。原先这份 32×32 的 regfile 没有读者，留着只是 1024 个触发器被综合裁掉。
//==============================================================================

//==============================================================================
// 控制 / CSR
//==============================================================================
    clint u_clint (
        .clk(clk), .rst(rst),
        .jalr_fail_0(jump_link_register_fail_0), .jalr_fail_1(jump_link_register_fail_1),
        .redir_backend_0(redir_backend_0), .redir_frontend(redir_frontend),
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
        .redir_cause(redir_cause), .rob_ret_step(rob_ret_step), .trap_cause(trap_cause),
        .trap_pc(trap_pc), .trap_value(trap_value),
        .flush_index(flush_index),
        .jump_link(1'b0), .np_take(btb_take_0 | btb_take_1),
        .csr_address(csr_address_m),
        .csr_commit_valid(csr_commit_write), .csr_commit_address(csr_commit_address),
        .csr_commit_function3(csr_commit_function3), .csr_commit_source(csr_commit_source),
        .exti(exti), .timi(timer_interrupt), .softi(1'b0),
        .pc_address_in(head_pc_rob),
        .interrupt_service_address_2(isr_addr2), .mcause(mcause), .csr_data_combinational(csr_data_combinational),
        .interrupt_return_address_2(iret_addr2),
        .irq_kill(irq_kill),
        .exc0_en(exc0_enable), .exc0_index(exc0_index), .exc0_cause(exc0_cause),
        .exc0_pc(exc0_pc), .exc0_tval(exc0_trap_value),
        .exc1_en(exc1_enable), .exc1_index(exc1_index), .exc1_cause(exc1_cause),
        .exc1_pc(exc1_pc), .exc1_tval(exc1_trap_value),
        .exception_selected_enable(excls_enable), .exception_selected_index(excls_index), .exception_selected_cause(excls_cause),
        .exception_selected_pc(excls_pc), .exception_selected_trap_value(excls_trap_value),
        .mret_valid(mret_valid)
    );

//==============================================================================
// 数据侧存储
//==============================================================================
    dcache #(.DC_SETS_B(DC_SETS_B)) u_dcache (
        .clk(clk), .rst(rst),
        .bus_address_in(dcache_address_lsu),
        .bus_valid_in(dcache_valid_lsu),
        .store_valid_in(store_valid_lsu), .store_address_in(store_address_lsu),
        .store_data_in(store_data_lsu), .store_byte_enable_in(store_byte_enable_lsu),
        .store_blocked(store_blocked_dcache),
        .mem_data(mem_data_dtcm), .mem_valid(mem_valid_dtcm), .mem_ready(1'b1),
        .mem_write_ready(l2_d_ready_w),
        .invalidate(dcache_invalidate),     // ★ 2026-11-13：fence / 软件失效原语
        .bus_data_out(data_dcache),
        .load_ready(load_ready_dcache), .busy(busy_dcache), .hold(hold_dcache), .request_blocked(request_blocked_dcache),
        .load_ready_next(load_ready_next_dcache),
        .mem_req(mem_req_dcache), .mem_write_enable(mem_write_enable_dcache),
        .mem_addr(mem_address_dcache), .mem_write_data(mem_wdata_dcache), .mem_byte_enable(mem_byte_enable_dcache)
    );

//==============================================================================
// 64KB 统一二级缓存（2026-11-12 起顶替 itcm + dtcm）
//
//   ★ 两个上游的握手是【电平】的（itcm/dtcm 时代的口径）：请求一直举着，自己数
//     mem_valid 收字；而 L2 的 i_req/d_req 是【脉冲】。这里做电平→脉冲的转换。
//     · I：icache 的 mem_req 在整个 32 拍填充期都是高 ⇒ 取上升沿；
//     · D 读：dcache 的 read_drive 是 8 拍电平、地址每拍都变（L2 只认起始地址）；
//     · D 写：dcache 的 mem_write_enable 本身已经是 1 拍 one-shot，直接当脉冲用。
//   ★ 不能只按 mem_req 的上升沿唤醒读：一笔写紧接一笔读时 mem_req 全程为高、
//     没有上升沿，那笔读就丢了（写使能是独立的一根，所以读要按"读状态"取沿）。
//==============================================================================
    reg  i_req_active;
    wire i_req_pulse = mem_req_icache & ~i_req_active;
    always @(posedge clk) begin
        if (rst) i_req_active <= 1'b0;
        else     i_req_active <= mem_req_icache;
    end

    reg  d_read_active;
    wire d_read_now   = mem_req_dcache & ~mem_write_enable_dcache;
    wire d_read_pulse = d_read_now & ~d_read_active;
    always @(posedge clk) begin
        if (rst) d_read_active <= 1'b0;
        else     d_read_active <= d_read_now;
    end
    wire d_write_pulse = mem_write_enable_dcache;
    wire d_req_pulse   = d_read_pulse | d_write_pulse;

    l2cache u_l2 (
        .clk(clk), .rst(rst),
//I 口
        .i_req(i_req_pulse), .i_addr(mem_address_icache),
        .i_data(mem_data_icache), .i_valid(mem_valid_icache),
//D 口
        .d_req(d_req_pulse),
        .d_we(mem_write_enable_dcache),
        .d_addr(mem_address_dcache),
        .d_wdata(mem_wdata_dcache),
        .d_be(mem_byte_enable_dcache),
        .d_data(mem_data_dtcm), .d_valid(mem_valid_dtcm), .d_ready(l2_d_ready_w),
//维护 / 非缓存窗口
        .maint(l2_maint_source), .maint_done(l2_maint_done),
        .write_queue_empty(l2_write_queue_empty),
        .unc_base(l2_uncached_base), .unc_mask(l2_uncached_mask),
//AHB 主口
        .m_haddr(l2_haddr), .m_htrans(l2_htrans), .m_hwrite(l2_hwrite), .m_hsize(l2_hsize),
        .m_hburst(l2_hburst), .m_hprot(l2_hprot), .m_hmastlock(l2_hmastlock),
        .m_hwdata(l2_hwdata), .m_hwstrb(l2_hwstrb),
        .m_hrdata(l2_hrdata), .m_hready(l2_hready), .m_rd_valid(l2_rd_valid),
        .m_busy(l2_busy), .m_hgrant(l2_hgrant), .m_hresp(l2_hresp)
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
        if ((ROB_DEPTH < 4) || (ROB_DEPTH > 32) || ((ROB_DEPTH & (ROB_DEPTH-1)) != 0))
            $display("CFG_FAIL: ROB_DEPTH=%0d 必须是 4..32 的 2 的幂", ROB_DEPTH);
        else if (VALUE_TABLE_DEPTH != ROB_DEPTH)
            $display("CFG_FAIL: VALUE_TABLE_DEPTH=%0d 必须等于 ROB_DEPTH=%0d", VALUE_TABLE_DEPTH, ROB_DEPTH);
        else
            $display("CFG_OK: LANES=%0d ROB=%0d IFFU=%0d VT=%0d IC_INDEX_B=%0d BHT=%0d BTB=%0d ADDER_AT_D=%0d",
                     LANES, ROB_DEPTH, IFFU_DEPTH, VALUE_TABLE_DEPTH, IC_INDEX_B, BHT_B, BTB_B, ADDER_AT_D);
    end
// synthesis translate_on
endmodule
