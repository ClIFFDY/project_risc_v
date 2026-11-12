`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

module rename #(
    parameter integer ARCH_COUNT = 32,
    parameter integer PHYS_COUNT = 64,
    parameter integer PHYS_BITS  = 6
)(
    input clk, rst,
    input        redir_frontend,     // 前端重定向（rob 判定拍出的那根）
    input        redir_backend_0, redir_backend_1,  
    input        irq_kill,           // 中断受理（clint 自己判的）
//后端域（清表 / 异常 / 中断）：回滚 rename 与空闲表、挡住本拍落盘与入队，不清输入 lane。
    input       exec_in,            // 流水线推进使能
    input       stall_rob_full_in,  // ROB 满
//后端域的寄存版（晚一拍）：只给访存 FIFO 清零用。
    input [1:0]  store_pop,
    input [3:0]  store_flush_pop,
    output reg   credit_stall_out,
//给 fifo 的推进量 = 本拍真正装了几条（0/1/2/3/4）。iffu 按自己手上有效条数截断后移动指针。
//给 fifo 的推进量 = 本拍真正装了几条（0/1/2/3/4）。iffu 按自己手上有效条数截断后移动指针。
//★ 2026-11-12 试过寄存一拍再送：rand 0/200、CoreMark 0 字符。
//  因为 advance 与 iffu 的弹出是【同一拍的 1:1 契约】（我这一拍从你那几格收 N 条，你走 N 格），
//  晚一拍就重复/漏发。要断这条环得改契约本身，不是插一个寄存器。
    output reg [3:0] advance,
input  [1:0]  dsq_room,    output        push0_valid, output [132:0] push0_data,    output        push1_valid, output [132:0] push1_data,    output        push2_valid, output [132:0] push2_data,
//★ 保留站的空位（isq 出口）：站满就把本 lane 的分配门关掉。
    input        station_room_0_valid,
    input        station_room_1_valid,
//本拍 lane0 装没装：lane1 到底拿出队格的第二格(h1)还是队头(h0)由它定 ⇒ 寄存器堆读口的
    output reg   lane0_load_o,

//============================== lane0 ==============================
    input [31:0] instruction_in_0,
    input [31:0] address_in_0,
    input        branch_prediction_in_0,
    input [31:0] jump_link_register_prediction_in_0,
    input [4:0]  index_in_0,
    input        generation_in_0,
//出队格这一格是不是真的有效（iffu 的 h0_valid/h1_valid）。不许用 addr!=0 去推 —— 那个判据要经过
    input        instruction_valid_0,
//操作数就绪（rob 按冻结生产者槽现算）与发射拍被接收（isq 把"就绪 + FU 空闲"合成这一根）。
    input        ready1_in_0, ready2_in_0,
    input        accept_in_0,
//给发射级：寄存器里有指令、且这条还没发出去。
    output reg   lane_valid_0,
//载荷（rd / func10 / imm + 随行号）
    output reg [31:0] instruction_out_0,
    output reg [31:0] auxiliary_address_out_0,
    output reg        branch_prediction_taken_out_0,
    output reg [4:0]  source_a_out_0, source_b_out_0,
    output reg [4:0]  index_out_0,
    output reg        generation_out_0,
    output reg        payload_valid_out_0,
//rob 分配口（在队头上按 opcode 分类解）
    output reg        payload_fresh_out_0,
    output reg        alloc_en_0,
    output reg        alloc_we_0,
    output reg        alloc_jmp_0,
//这一条会不会报完成口（给 rob 的"发出即置可退"用）：M 类（funct7==1 的 OP）与 LOAD。
    output reg        alloc_pend_0,
//SYSTEM 类（分配拍译码）：这几类的异常/改向标记在 bju 判定延迟级才出 ⇒ ROB 退休要等那一级。
    output reg        alloc_ecall_0, alloc_ebreak_0, alloc_illegal_0, alloc_mret_0,
//★ 2026-11-13（fence）：MISC-MEM 的两条。fence 只要求"更老的写都可见了才许退"，
//  fence.i 还要求把 icache 整表作废并重取。在这里按 MISC-MEM 的 funct3 直接译，
//  不走 idu 的 SYSTEM 那一路 —— 它们不是异常，也不改 PC 的控制流来源。
    output reg        alloc_fence_0, alloc_fence_i_0,
    output reg [4:0]  alloc_rd_0,
//★ 2026-11-04（CSR 步骤 C）：这条是 CSR 指令，以及随 ROB 项走的载荷
    output reg        alloc_csr_0,
//★ 2026-11-04 真 PRF：这一条被分到的物理号，交给 rob 随项存（提交定格 / 冲刷回收都用它）
    output [5:0]      alloc_phys_0,
//★ 2026-11-11：ROB 口 1 的 PC。口 1 在 lane0 没吃的拍上装的是 h0，不能直接接 h1_address。
//  注意也不能用 address_stage1 —— 那个是 lane1 站的口（h1 是访存时它是 h2）。
    output [31:0]     alloc_pc_1,
//★ 2026-11-04（精确冲刷第 1 步）：这一条分配时顶掉的那个映射（= rat[rd] 的旧值），随 ROB 项走。
    output [63:0]     alloc_onehot_0,
    output [63:0]     arch_next_onehot_0,
//全译码（随 S 级寄存器送 isq 的入站：分类 / 各单元字段 / 异常标记 / 非寄存器操作数覆盖）
    output reg [1:0]  dec_class_0,
    output reg        dec_we_0,
    output reg [4:0]  dec_rd_0,
    output reg        dec_is_st_0,
    output reg [2:0]  dec_mulop_0,
    output reg [2:0]  dec_lsf3_0,
    output reg [3:0]  dec_func4_0,
    output reg [2:0]  dec_csrf3_0,
    output reg        dec_csrw_en_0,
    output reg        dec_csrw_wr_0,
    output reg [11:0] dec_csra_0,
    output reg [31:0] dec_jval_0,
    output reg        dec_brf_0,
    output reg        dec_jalf_0,
    output reg        dec_jalfr_0,
    output reg [3:0]  dec_bjuf4_0,
    output reg        dec_r1_imm_0,
    output reg        dec_r2_imm_0,
    output reg [31:0] dec_r1_immv_0,
    output reg [31:0] dec_r2_immv_0,

//============================== lane1 ==============================
    input [31:0] instruction_in_1,
    input [31:0] address_in_1,
    input        branch_prediction_in_1,
    input [31:0] jump_link_register_prediction_in_1,
    input [4:0]  index_in_1,
    input        generation_in_1,
    input        instruction_valid_1,
//★ 2026-11-09 三宽分配：iffu 的第三格（docs/SPEC_alloc3.md）
    input [31:0] instruction_in_2,
    input [31:0] address_in_2,
    input        branch_prediction_in_2,
    input [31:0] jump_link_register_prediction_in_2,
    input [4:0]  index_in_2,
    input        generation_in_2,
    input        instruction_valid_2,
    input        ready1_in_1, ready2_in_1,
    input        accept_in_1,
    output reg   lane_valid_1,
    output reg [31:0] instruction_out_1,
    output reg [31:0] auxiliary_address_out_1,
    output reg        branch_prediction_taken_out_1,
    (* max_fanout = 32 *) output reg [4:0] source_a_out_1,
    output reg [4:0] source_b_out_1,
    output reg [4:0]  index_out_1,
    output reg        generation_out_1,
    output reg        payload_valid_out_1,
    output reg        payload_fresh_out_1,
    output reg        alloc_en_1,
    output reg        alloc_we_1,
    output reg        alloc_jmp_1,
    output reg        alloc_pend_1,
    output reg        alloc_ecall_1, alloc_ebreak_1, alloc_illegal_1, alloc_mret_1,
    output reg        alloc_fence_1, alloc_fence_i_1,
    output reg [4:0]  alloc_rd_1,
//★ 同 lane0：CSR / 载荷
    output reg        alloc_csr_1,
    output [5:0]      alloc_phys_1,
//★ 2026-11-09 三宽分配：ROB 的第 3 组口
    output reg        alloc_en_2,
    output reg        alloc_we_2,
    output reg        alloc_jmp_2,
    output reg        alloc_pend_2,
    output reg        alloc_ecall_2, alloc_ebreak_2, alloc_illegal_2, alloc_mret_2,
    output reg        alloc_fence_2, alloc_fence_i_2,
    output reg [4:0]  alloc_rd_2,
    output reg        alloc_csr_2,
    output reg [5:0]  alloc_phys_2,
//★ 2026-11-10：这一条将来由【哪个 PRF 写口】写（PRF 的 owner 表在分配那一拍记下，发射时按它选阵列）。
    output [2:0]      alloc_owner_0,
    output [2:0]      alloc_owner_1,
    output [2:0]      alloc_owner_2,
//★ 同 lane0：分配时顶掉的旧映射
    output reg [1:0]  dec_class_1,
    output reg        dec_we_1,
    output reg [4:0]  dec_rd_1,
    output reg        dec_is_st_1,
    output reg [2:0]  dec_mulop_1,
    output reg [2:0]  dec_lsf3_1,
    output reg [3:0]  dec_func4_1,
    output reg [2:0]  dec_csrf3_1,
    output reg        dec_csrw_en_1,
    output reg        dec_csrw_wr_1,
    output reg [11:0] dec_csra_1,
    output reg [31:0] dec_jval_1,
    output reg        dec_brf_1,
    output reg        dec_jalf_1,
    output reg        dec_jalfr_1,
    output reg [3:0]  dec_bjuf4_1,
    output reg        dec_r1_imm_1,
    output reg        dec_r2_imm_1,
    output reg [31:0] dec_r1_immv_1,
    output reg [31:0] dec_r2_immv_1,

//==============================================================================
// 访存专用流（LSU FIFO）
    input         lsu_pop_in,
//---- 提交口（rename 的架构映射定格 / 空闲表回收）----
    input         write_enable_a_in, write_enable_b_in,
    input  [4:0]  commit_rd0_in, commit_rd1_in,
//★ 2026-11-04 真 PRF：提交项的物理号
    input  [5:0]  commit_phys0_in, commit_phys1_in,
//★ 2026-11-04 精确冲刷：rat 回滚值（每架构号最老被杀项的 old_phys）+ 幸存在飞项掩码
    input  [31:0]  squash_arch_hit_in,
    input  [191:0] squash_arch_phys_in,
    input  [31:0]  squash_survive_hit_in,
    input          alloc_stop_in,
//★ ROB 满：rename 的分配必须和 rob 的分配口【逐字同门】，否则发了号而 ROB 没接 ⇒ 漏号
    input         rob_full_in,
//★ 2026-11-09 三宽：ROB 还放得下第 3 条吗（occupancy + 2 < DEPTH）。与 rob 的 alloc_take_2 逐字同门。
    input         rob_room_2_in,
//---- 队头（出队侧）----
    output [31:0] lsu_instruction_out, lsu_auxiliary_address_out, lsu_immediate_out,
    output [4:0]  lsu_source_a_out, lsu_source_b_out, lsu_destination_register_out,
    output [2:0]  lsu_load_store_function3_out,
    output        lsu_is_store_out, lsu_write_enable_out,
    output [4:0]  lsu_index_out,
    output        lsu_generation_out, lsu_valid_out,
//★ 2026-11-04 真 PRF：条目里带的是两个源的【物理号】（6 位各一个，与原来的 tag 同宽）
    output [5:0]  lsu_phys_a_out, lsu_phys_b_out, lsu_phys_d_out,
//★ 两条 lane 的载荷队列也各带两个源的物理号（isq 的 D 级直接用）
    output reg [5:0]  phys_a_out_0, phys_b_out_0, phys_a_out_1, phys_b_out_1,
//目的物理号随载荷走（isq 的唤醒源）。
                      phys_d_out_0, phys_d_out_1,

//==============================================================================
// 与 top 的 rename 之间的口（rename 在 cpu_top 例化，位置是 F 级寄存器与 S 级寄存器之间）
//  出去：四个分配口（0=lane0, 1=第二格, 2=h2 走访存, 3=h2 走 lane1）+ 六个源查询地址
    output        rename_alloc_en_0, rename_alloc_en_1, rename_alloc_en_2, rename_alloc_en_3,
    output        rename_alloc_we_0, rename_alloc_we_1, rename_alloc_we_2, rename_alloc_we_3,
    output [4:0] rename_alloc_rd_0, rename_alloc_rd_1, rename_alloc_rd_2, rename_alloc_rd_3,
    output [4:0] rename_alloc_rs1_0, rename_alloc_rs1_1, rename_alloc_rs1_2, rename_alloc_rs1_3,
    output [4:0] rename_alloc_rs2_0, rename_alloc_rs2_1, rename_alloc_rs2_2, rename_alloc_rs2_3,
    output [4:0] rename_lookup_a, rename_lookup_b, rename_lookup_c,
    output [4:0] rename_lookup_d, rename_lookup_e, rename_lookup_f,
//  回来：目的物理号、六个源查到的物理号、bank 空位、分配 one-hot 与提交拍架构映射
    output  [5:0]  phys_rd_0, phys_rd_1, phys_rd_2, phys_rd_3,
    output  [5:0]  lookup_a_phys, lookup_b_phys, lookup_c_phys,
    output  [5:0]  lookup_d_phys, lookup_e_phys, lookup_f_phys,
    output         bank_empty_0, bank_empty_1,
    output         bank_empty_2, bank_empty_3,
    output         bank_low_even, bank_low_odd,
    output  [63:0] alloc_onehot, arch_next_onehot
    );

//==============================================================================
//  参数与声明区：本模块全部 localparam/parameter + reg/wire。
    parameter integer LANES = 2;
//RV32I和Zicsr扩展的opcode集
    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_MISC_MEM = 7'b0001111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_SYSTEM = 7'b1110011;
    localparam CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;
//bju 功能码：把"分支种类 / 跳转种类 / 四类异常"压成一个 4 位码，bju 只认这一个口。
    localparam BJU_NONE    = 4'd0;
    localparam BJU_JAL     = 4'd1;
    localparam BJU_JAL_MIS = 4'd2;
    localparam BJU_JALR    = 4'd3;
    localparam BJU_ECALL   = 4'd4;
    localparam BJU_EBREAK  = 4'd5;
    localparam BJU_ILLEGAL = 4'd6;
    localparam BJU_MRET    = 4'd7;
//==============================================================================
// 访存 FIFO：4 个标量（本仓惯例：数组元素在 always @(*) 里进不了敏感表）
    localparam integer LSU_QUEUE_WIDTH = 98;
//访存 FIFO：8 深、两个写口。
    localparam [3:0]   LSU_DEPTH = 4'd8;
    localparam integer LSU_AW    = 3;

//复位就地打一拍（每个模块各一份，相位一致）。
    reg reset_registered;
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
    reg exec;
//★ 本拍算、下一拍生效的"最多收几条"（见下面 credit 那一段）
    reg [3:0] grant;
//★ 两条 lane 各自的"已发射"位。lane0/lane1 的发射彼此独立之后，"寄存器内容的消费"与"寄存器装载"
    reg lane0_free, lane1_free, lane2_free, go, load_lane0, load_lane1;
//★ 声明必须在使用之前（Vivado 报 VRFC 10-3380；Verilator 不管）：这三根在分配判据（下面的
    reg        pick_from_h2;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_queue_0, lsu_queue_1, lsu_queue_2, lsu_queue_3;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_queue_4, lsu_queue_5, lsu_queue_6, lsu_queue_7;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_entry_0, lsu_entry_1, lsu_entry_2;
    reg        lsu_push_0, lsu_push_1, lsu_push_2;
    reg        lsu_h1_taken;
    reg [2:0]  lsu_slot_1, lsu_slot_2;
    reg [3:0]  lsu_room;
    reg        is_mem_0, is_mem_1, is_mem_2;
    reg        h1_is_mem, h1_store, h2_store, pick_bank_empty, pick_mem;
    reg [4:0]  h2_rs1, h2_rs2, h2_rd;
    reg [4:0]  destination_decoded, destination_decoded_1;
    reg [31:0] instruction_stage1, address_stage1, jump_link_register_stage1;
    reg        alloc_lane0, alloc_lane1, lane0_store, lane1_store;
//==============================================================================
// S 级载荷的 2 深 FIFO（2026-10-31）
    reg        skid_valid_0, skid_valid_1;
    reg        lane0_push, lane1_push, lane0_head_after_pop, lane1_head_after_pop;
//（tag 的提交清见下面 lane 载荷块之前那一段）
    reg [31:0] skid_instruction_out_0;
    reg [31:0] skid_auxiliary_address_out_0;
    reg        skid_branch_prediction_taken_out_0;
    reg [4:0]  skid_source_a_out_0, skid_source_b_out_0;
    reg [4:0]  skid_index_out_0;
    reg        skid_generation_out_0;
//★ 2026-11-04 真 PRF：与 prod 同生命周期的物理号（prod 被提交清掉时它【不清】，号是稳定的）
    reg [5:0]  skid_phys_a_out_0, skid_phys_b_out_0, skid_phys_a_out_1, skid_phys_b_out_1;
    reg [5:0]  skid_phys_d_out_0, skid_phys_d_out_1;
    reg [1:0]  skid_dec_class_0;
    reg        skid_dec_we_0;
    reg [4:0]  skid_dec_rd_0;
    reg        skid_dec_is_st_0;
    reg [2:0]  skid_dec_mulop_0, skid_dec_lsf3_0, skid_dec_csrf3_0;
    reg [3:0]  skid_dec_func4_0;
    reg        skid_dec_csrw_en_0, skid_dec_csrw_wr_0;
    reg [11:0] skid_dec_csra_0;
    reg [31:0] skid_dec_jval_0;
    reg        skid_dec_brf_0, skid_dec_jalf_0, skid_dec_jalfr_0;
    reg [3:0]  skid_dec_bjuf4_0;
    reg        skid_dec_r1_imm_0, skid_dec_r2_imm_0;
    reg [31:0] skid_dec_r1_immv_0, skid_dec_r2_immv_0;
    reg [31:0] skid_instruction_out_1;
    reg [31:0] skid_auxiliary_address_out_1;
    reg        skid_branch_prediction_taken_out_1;
    reg [4:0]  skid_source_a_out_1, skid_source_b_out_1;
    reg [4:0]  skid_index_out_1;
    reg        skid_generation_out_1;
    reg [1:0]  skid_dec_class_1;
    reg        skid_dec_we_1;
    reg [4:0]  skid_dec_rd_1;
    reg        skid_dec_is_st_1;
    reg [2:0]  skid_dec_mulop_1, skid_dec_lsf3_1, skid_dec_csrf3_1;
    reg [3:0]  skid_dec_func4_1;
    reg        skid_dec_csrw_en_1, skid_dec_csrw_wr_1;
    reg [11:0] skid_dec_csra_1;
    reg [31:0] skid_dec_jval_1;
    reg        skid_dec_brf_1, skid_dec_jalf_1, skid_dec_jalfr_1;
    reg [3:0]  skid_dec_bjuf4_1;
    reg        skid_dec_r1_imm_1, skid_dec_r2_imm_1;
    reg [31:0] skid_dec_r1_immv_1, skid_dec_r2_immv_1;
    reg [31:0] instruction_pick, address_pick;
    reg [4:0]  index_pick;
    reg        generation_pick;
//★ 2026-11-11：载荷队列回到 2 深（头 + 影子），没有指针、没有环。
    localparam integer LNQ_W = 240;   // = 一条完整载荷的打包宽度
    wire [LNQ_W-1:0] lane0_pack, lane1_pack;
    wire lane0_skid_load, lane1_skid_load;
    wire lane0_skid_vac, lane1_skid_vac, lane0_skid_hold, lane1_skid_hold;
    reg [2:0]  lsu_read_pointer, lsu_write_pointer;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_head;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_clear_0, lsu_clear_1, lsu_clear_2, lsu_clear_3;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_clear_4, lsu_clear_5, lsu_clear_6, lsu_clear_7;
    reg        lsu_tag_hit_0a, lsu_tag_hit_0b, lsu_tag_hit_1a, lsu_tag_hit_1b;
    reg        lsu_tag_hit_2a, lsu_tag_hit_2b, lsu_tag_hit_3a, lsu_tag_hit_3b;
//★ lane1 这一拍拿 fifo 的哪一格：lane0 也在装 ⇒ 拿第二格；lane0 没装（在等操作数）⇒ 拿队头。
    reg [4:0]  source_a_stage1, source_b_stage1;
    reg [4:0]  source_a_dec_0, source_b_dec_0, source_a_dec_1, source_b_dec_1;
    reg [4:0]  index_stage1;
    reg        branch_prediction_stage1, generation_stage1;
//译码字段的组合解算：源是本级的输入（F 级的指令字），口径与原来那一段逐条一致。
    reg [9:0]  function10_decoded;
    reg [31:0] immediate_decoded;
    reg        jump_decoded;
    reg csr_is_access_dec, csr_exists_dec, csr_ro_dec, csr_wr_dec;
    reg exc_illegal_dec, exception_jump_misalign_decoded;
//入站分类 + 各类字段装配
    reg [1:0]  class_decoded;
    reg        write_enable_decoded, csr_write_enable_decoded, csr_write_commit_decoded;
    reg [4:0]  destination_pick_decoded;
    reg        is_store_decoded;
    reg [2:0]  multiply_operation_decoded, load_store_function3_decoded, csr_function3_decoded;
    reg [3:0]  function4_decoded;
    reg [11:0] csr_address_decoded;
    reg [31:0] jump_value_decoded;
    reg        branch_flag_decoded, jump_link_flag_decoded, jump_link_register_flag_decoded;
//非寄存器操作数的覆盖（立即数在这里并进操作数）
    reg source_a_immediate_decoded, source_b_immediate_decoded;
    reg [31:0] source_a_immediate_value_decoded, source_b_immediate_value_decoded;
//异常标记（ecall/ebreak/mret）
    reg exc_ecall_dec, exc_ebreak_dec, exc_irq_ret_dec;
//bju 功能码：这条指令在 bju 里算什么。
    reg [3:0] branch_jump_function4_decoded;
//lane1 的译码（与 lane0 一模，源换成 instruction_stage1 / address_stage1 / jump_link_register_stage1）
    reg [9:0]  function10_decoded_1;
    reg [31:0] immediate_decoded_1;
    reg        jump_decoded_1;
    reg csr_is_access_dec_1, csr_exists_dec_1, csr_ro_dec_1, csr_wr_dec_1;
    reg exc_illegal_dec_1, exception_jump_misalign_decoded_1;
    reg [1:0]  class_decoded_1;
    reg        write_enable_decoded_1, csr_write_enable_decoded_1, csr_write_commit_decoded_1;
    reg [4:0]  destination_pick_decoded_1;
    reg        is_store_decoded_1;
    reg [2:0]  multiply_operation_decoded_1, load_store_function3_decoded_1, csr_function3_decoded_1;
    reg [3:0]  function4_decoded_1;
    reg [11:0] csr_address_decoded_1;
    reg [31:0] jump_value_decoded_1;
    reg        branch_flag_decoded_1, jump_link_flag_decoded_1, jump_link_register_flag_decoded_1;
    reg source_a_immediate_decoded_1, source_b_immediate_decoded_1;
    reg [31:0] source_a_immediate_value_decoded_1, source_b_immediate_value_decoded_1;
    reg exc_ecall_dec_1, exc_ebreak_dec_1, exc_irq_ret_dec_1;
//bju 功能码（与 lane0 逐条同形）
    reg [3:0] branch_jump_function4_decoded_1;
//==============================================================================
// 访存 FIFO 的队头译码 + 时序
    reg [31:0] lsu_head_instruction, lsu_head_pc;
    reg [4:0]  lsu_head_index;
    reg        lsu_head_generation;
//★ 2026-11-04 真 PRF：进队条目带的是两个源的【物理号】，在进队这一拍由 rename 查出。
    wire [4:0]  lsu_push_source_a, lsu_push_source_b;
    wire [4:0]  push1_source_a, push1_source_b;
//★ 冲刷：按击杀掩码【截断】尾部，不再整条清（rob 的 flush_partial 恒 0 那一套已经不成立了）。

    always @(posedge clk) reset_registered <= rst;

//提取不同类型指令立即数的函数块
    function [31:0] immB;
        input [31:0] instruction;
        immB = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
    endfunction

    function [31:0] immI;
        input [31:0] instruction;
        immI = {{20{instruction[31]}}, instruction[31:20]};
    endfunction

    function [31:0] immS;
        input [31:0] instruction;
        immS = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
    endfunction

    function [31:0] immU;
        input [31:0] instruction;
        immU = {instruction[31:12], 12'd0};
    endfunction

    function [31:0] immJ;
        input [31:0] instruction;
        immJ = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
    endfunction


    reg alloc_block;
    (* max_fanout = 32 *) reg [3:0] lsu_queue_count;
    always @(*) begin
        exec    = exec_in;
//★ 2026-11-11：不再吃 isq 的 station_room —— 本地空位就是背压。
//  三个口各自的"第几条"必须与 dsq 的格数逐条对齐：alloc 一旦成立，ROB 就发槽、
//  条目就必须真的推得进 dsq。少挡一条 ⇒ 条目被静默丢掉、ROB 那槽永远等不到完成。
//★ 2026-11-12：dsq 的三条队列各报一根 full，和 ROB 满【或】起来直接背压。
//  room 只可能是 0/2（见 dsq），所以 lane0/lane1 同门；三宽（lane2）不再开。
        alloc_block = (dsq_room == 2'd0) | stall_rob_full_in;
        lane0_free = ~alloc_block;
        lane1_free = ~alloc_block;
        lane2_free = 1'b0;
//★ 2026-11-11：分配门必须与 rob.v 的 alloc_take_* 逐字同门（`~(redir_backend_0 | irq_kill)`）。
//  rob 那一拍不分配、rename 却把条目推进 dsq ⇒ 条目带着一个没被分配出去的槽号，
//  下一拍同一槽号被别的指令正经分配 ⇒ ROB 项的 rd/jump 与指令对不上（实测 CoreMark 231 字符处卡死）。
        go    = exec & ~stall_rob_full_in & ~(irq_kill | redir_backend_0);
        is_mem_0 = (instruction_in_0[6:0] == OPCODE_LOAD) | (instruction_in_0[6:0] == OPCODE_STORE);
//三宽分配。
        h1_is_mem = (instruction_in_1[6:0] == OPCODE_LOAD) | (instruction_in_1[6:0] == OPCODE_STORE);
        is_mem_2  = (instruction_in_2[6:0] == OPCODE_LOAD) | (instruction_in_2[6:0] == OPCODE_STORE);
        h1_store  = (instruction_in_1[6:0] == OPCODE_STORE);
        h2_store  = (instruction_in_2[6:0] == OPCODE_STORE);
//访存条的字段位置固定（store/load 的 rs1[19:15]、rd[11:7] 同一位置）；store 不读 rs2 的"假源"给 0。
        h2_rs1 = instruction_in_2[19:15];
        h2_rs2 = h2_store ? instruction_in_2[24:20] : 5'd0;
        h2_rd  = h2_store ? 5'd0 : instruction_in_2[11:7];
//★ 只看本地计数器：不许再依赖 isq 当拍弹不弹（那会把 lsu 的 stage_free 拉进 iffu 的推进锥）
        lsu_room = 4'd8;   //★ 2026-11-11：LSU FIFO 已搬到后半段，这里恒给满额；真正的限制是 dsq 空位
//★ 2026-11-05：改向一经判定，分配冻结到那条分支提交。冻结住 alloc_lane0 就把
//★ lane0_free（dsq 有空位）对所有类别都必须门：访存条原来只查 lsu_room（拆环后恒 8），
//  dsq 满的时候照样分配 ⇒ 条目推不进去、ROB 那槽永远等不到完成。
        alloc_lane0 = go & instruction_valid_0 & ~alloc_stop_in & lane0_free
                    & (is_mem_0 ? (lsu_room != 4'd0) : 1'b1)
                    & ~(bank_empty_0 & (destination_decoded != 5'd0));
//★ 2026-11-09：口 1 描述【h1 这条访存】的前提是 alloc_lane0 也成立。
        pick_mem = alloc_lane0 & h1_is_mem;
        pick_from_h2 = pick_mem;

//冲刷门挂在【落盘】上，不挂在推进上。
        lane0_store = alloc_lane0 & ~is_mem_0 & ~(irq_kill | redir_frontend | redir_backend_0);
        lsu_push_0  = alloc_lane0 &  is_mem_0 & ~(irq_kill | redir_frontend | redir_backend_0);
//2 深载荷的下一态判据（组合，只在本级寄存器上）。
        lane0_head_after_pop = (accept_in_0 & payload_valid_out_0) ? skid_valid_0 : payload_valid_out_0;
        lane1_head_after_pop = (accept_in_1 & payload_valid_out_1) ? skid_valid_1 : payload_valid_out_1;
        lane0_push = lane0_store & (address_in_0 != 32'd0);
//lane1 这一拍拿的是哪一格：lane0 吃了 h0 且 h1 是访存 ⇒ 顺延到 h2；lane0 没吃 ⇒ 拿 h0 顶。
        instruction_pick = alloc_lane0 ? (h1_is_mem ? instruction_in_2 : instruction_in_1)
                                       : instruction_in_0;
        address_pick     = alloc_lane0 ? (h1_is_mem ? address_in_2 : address_in_1) : address_in_0;
        index_pick       = alloc_lane0 ? (h1_is_mem ? index_in_2   : index_in_1)   : index_in_0;
        generation_pick  = alloc_lane0 ? (h1_is_mem ? generation_in_2 : generation_in_1) : generation_in_0;
        is_mem_1 = (instruction_pick[6:0] == OPCODE_LOAD) | (instruction_pick[6:0] == OPCODE_STORE);
//h1 自己是访存 ⇒ 直接进 LSU 口 1（不占 lane1）
        lsu_push_1  = alloc_lane0 & instruction_valid_1 & h1_is_mem & ~(irq_kill | redir_frontend | redir_backend_0)
                    & (lsu_room >= (lsu_push_0 ? 4'd2 : 4'd1))
                    & lane1_free
                    & ~(bank_empty_1 & ((h1_store ? 5'd0 : instruction_in_1[11:7]) != 5'd0));
//lane1 拿哪条：h1 非访存拿 h1；h1 是访存就顺延到 h2（且 h2 必须非访存 —— 两条访存挤在同拍就走不动）
//  ★ h1 是访存时 lane1 这一条是【第三条】，要 lane2_free；其余情形是第二条（或 lane0 没吃时的第一条）。
        pick_bank_empty = pick_mem ? bank_empty_3 : bank_empty_1;
//★ 2026-11-11：lane1 只在 lane0 也分配时才成立。
//  口 0 = lane0 = alu0 阵列、口 1 = lane1 = alu1 阵列，这条对应关系必须硬绑：
//  PRF 读靠分配期写下的 owner 表选阵列，叉着走（口 1 的号却落在 lane0 执行）
//  就会"值写进 alu0、读却查 alu1" ⇒ 读到旧值（seed13：x2/x10/x13/x14 全错）。
        alloc_lane1 = (LANES == 1) ? 1'b0 :
                      (go & ~alloc_stop_in & alloc_lane0
                       & (h1_is_mem ? (lsu_push_1 & instruction_valid_2 & ~is_mem_1 & rob_room_2_in)
                                    : instruction_valid_1)
                       & (h1_is_mem ? lane2_free : lane1_free)
                       & ~(pick_bank_empty & (destination_decoded_1 != 5'd0)));

        lane1_store = alloc_lane1 & ~is_mem_1 & ~(irq_kill | redir_frontend | redir_backend_0);
//★ 2026-11-09：队列加了第三个写口 ⇒ h2 是访存时不再要求前两条都非访存。
        lsu_h1_taken = h1_is_mem ? lsu_push_1 : alloc_lane1;
        lsu_push_2  = alloc_lane0 & lsu_h1_taken & instruction_valid_2 & is_mem_2
                    & ~(irq_kill | redir_frontend | redir_backend_0)
                    & (lsu_room >= ({2'd0, lsu_push_0} + {2'd0, lsu_push_1} + 4'd1))
                    & lane2_free
                    & ~(bank_empty_2 & (h2_rd != 5'd0)) & rob_room_2_in;
//★ 2026-11-11：额度（credit）。iffu 按 advance 弹，本模块必须【全收】——
//  所以判据只能吃寄存器量：尾队占用 / LSU 队列 / ROB 余量 / 两个 bank 的剩余号，
//  不许有任何一项来自"这一拍收到的指令长什么样"。advance 本身打一拍再送出去。
//给 fifo 的推进量 = 本拍真正消费了几格（0..3）：指针按它走，不是"成对就 +2"。
        grant = 4'd0;
        advance = {1'b0, alloc_lane0}
                + {1'b0, (alloc_lane0 ? (h1_is_mem ? lsu_push_1 : alloc_lane1) : alloc_lane1)}
                + {1'b0, (alloc_lane0 & h1_is_mem & alloc_lane1) | lsu_push_2};
        lane0_load_o = alloc_lane0;
//进队条目：口 0 = h0（最老，源用无旁路的 lookup_a/b），口 1 = h1，口 2 = h2。
        lsu_entry_0 = {phys_rd_0,
                       instruction_in_0[19:15], instruction_in_0[24:20],
                       instruction_in_0, address_in_0, index_in_0, generation_in_0,
                       lookup_a_phys, lookup_b_phys};
        lsu_entry_1 = {phys_rd_1,
                       instruction_in_1[19:15], h1_store ? instruction_in_1[24:20] : 5'd0,
                       instruction_in_1, address_in_1, index_in_1, generation_in_1,
                       lookup_c_phys, lookup_d_phys};
        lsu_entry_2 = {phys_rd_2,
                       h2_rs1, h2_rs2,
                       instruction_in_2, address_in_2, index_in_2, generation_in_2,
                       lookup_e_phys, lookup_f_phys};
        lane1_push = lane1_store & (address_pick != 32'd0);
    end

//★ 输出级的冲刷清必须【按年龄】，不能无条件清。

    always @(*) begin
//★ 2026-11-09：第二格的译码跟着 pick 走（h1 是访存时 pick = h2）
        instruction_stage1 = alloc_lane0 ? (h1_is_mem ? instruction_in_2 : instruction_in_1) : instruction_in_0;
        address_stage1 = alloc_lane0 ? (h1_is_mem ? address_in_2 : address_in_1) : address_in_0;
        jump_link_register_stage1 = alloc_lane0 ? (h1_is_mem ? jump_link_register_prediction_in_2 : jump_link_register_prediction_in_1)
                                                 : jump_link_register_prediction_in_0;
        branch_prediction_stage1  = alloc_lane0 ? (h1_is_mem ? branch_prediction_in_2 : branch_prediction_in_1)
                                                : branch_prediction_in_0;
        source_a_stage1   = alloc_lane0 ? source_a_dec_1 : source_a_dec_0;
        source_b_stage1   = alloc_lane0 ? source_b_dec_1 : source_b_dec_0;
        index_stage1  = alloc_lane0 ? (h1_is_mem ? index_in_2 : index_in_1) : index_in_0;
        generation_stage1  = alloc_lane0 ? (h1_is_mem ? generation_in_2 : generation_in_1) : generation_in_0;
    end

//本 lane 的 D 有指令、且还没被收：发射级按它给 D 级发 credit
    always @(*) begin
        lane_valid_0 = payload_valid_out_0;
        lane_valid_1 = payload_valid_out_1;
        credit_stall_out = 1'b0;
    end

    always @(*) begin
        destination_decoded = 5'd0;
        function10_decoded = 10'd0;
        immediate_decoded = 32'd0;
        jump_decoded = 1'b0;
        case (instruction_in_0[6:0])
            OPCODE_OP: begin
                destination_decoded = instruction_in_0[11:7];
                function10_decoded = {instruction_in_0[31:25], instruction_in_0[14:12]};
            end
            OPCODE_OP_IMM: begin
                immediate_decoded = immI(instruction_in_0);
                destination_decoded = instruction_in_0[11:7];
                function10_decoded = {(instruction_in_0[14:12] == 3'b001 || instruction_in_0[14:12] == 3'b101) ? instruction_in_0[31:25] : 7'd0,
                instruction_in_0[14:12]};
            end
            OPCODE_JAL: begin
                destination_decoded = instruction_in_0[11:7];
                jump_decoded = 1'b1;
            end
            OPCODE_JALR: begin
                destination_decoded = instruction_in_0[11:7];
                immediate_decoded = immI(instruction_in_0);
                jump_decoded = 1'b1;
            end
            OPCODE_BRANCH: begin
                immediate_decoded = immB(instruction_in_0);
                function10_decoded = {7'd0, instruction_in_0[14:12]};
                jump_decoded = 1'b1;
            end
            OPCODE_LOAD: begin
                immediate_decoded = immI(instruction_in_0);
                destination_decoded = instruction_in_0[11:7];
            end
            OPCODE_STORE: begin
                immediate_decoded = immS(instruction_in_0);
            end
            OPCODE_LUI: begin
                immediate_decoded = immU(instruction_in_0);
                destination_decoded = instruction_in_0[11:7];
            end
            OPCODE_AUIPC: begin
                immediate_decoded = immU(instruction_in_0) - 4'd4;
                destination_decoded = instruction_in_0[11:7];
            end
            OPCODE_SYSTEM: begin
                destination_decoded = instruction_in_0[11:7];
                function10_decoded = {7'd0, instruction_in_0[14:12]};
            end
        endcase
    end

//---- 源寄存器号（2026-10-31 从 iffu 搬过来）----
    always @(*) begin
        source_a_dec_0 = 5'd0;
        source_b_dec_0 = 5'd0;
        case (instruction_in_0[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
            OPCODE_LOAD, OPCODE_STORE: source_a_dec_0 = instruction_in_0[19:15];
        endcase
        if ((instruction_in_0[6:0] == OPCODE_SYSTEM)
         && (instruction_in_0[14:12] != 3'b101)
         && (instruction_in_0[14:12] != 3'b110)
         && (instruction_in_0[14:12] != 3'b111))
            source_a_dec_0 = instruction_in_0[19:15];
        case (instruction_in_0[6:0])
            OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: source_b_dec_0 = instruction_in_0[24:20];
        endcase
//★ 2026-11-09 三宽分配：源字段跟【lane1 真正拿到的那一条】走，不跟 h1 走。
        source_a_dec_1 = 5'd0;
        source_b_dec_1 = 5'd0;
        case (instruction_stage1[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
            OPCODE_LOAD, OPCODE_STORE: source_a_dec_1 = instruction_stage1[19:15];
        endcase
        if ((instruction_stage1[6:0] == OPCODE_SYSTEM)
         && (instruction_stage1[14:12] != 3'b101)
         && (instruction_stage1[14:12] != 3'b110)
         && (instruction_stage1[14:12] != 3'b111))
            source_a_dec_1 = instruction_stage1[19:15];
        case (instruction_stage1[6:0])
            OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: source_b_dec_1 = instruction_stage1[24:20];
        endcase
    end

    always @(*) begin
        csr_is_access_dec = (instruction_in_0[14:12] != 3'b000) && (instruction_in_0[14:12] != 3'b100);
//★ "写"的规范口径：CSRRW/CSRRWI 恒写；CSRRS/CSRRC/CSRRSI/CSRRCI 只有源（rs1 或 uimm）非 0 才写。
        csr_wr_dec        = (instruction_in_0[14:12] == 3'b001) | (instruction_in_0[14:12] == 3'b101) |
                            (instruction_in_0[19:15] != 5'd0);
        csr_ro_dec        = (instruction_in_0[31:30] == 2'b11);
        csr_exists_dec    = 1'b0;
        case (instruction_in_0[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec = 1'b1;
            default: csr_exists_dec = 1'b0;
        endcase
    end

    always @(*) begin
        exc_illegal_dec = 1'b0;
        exception_jump_misalign_decoded    = (instruction_in_0[6:0] == OPCODE_JAL) & instruction_in_0[21];
        if (instruction_in_0 != 32'd0) begin
            case (instruction_in_0[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec = 1'b0;
                OPCODE_LOAD: begin
                    if (instruction_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b110) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b111) exc_illegal_dec = 1'b1;
                end
                OPCODE_STORE: begin
                    if (instruction_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b100) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b101) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b110) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b111) exc_illegal_dec = 1'b1;
                end
                OPCODE_JALR: begin
                    if (instruction_in_0[14:12] != 3'b000) exc_illegal_dec = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (instruction_in_0[14:12] == 3'b010) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (instruction_in_0[14:12] == 3'b001) begin
                        if (instruction_in_0[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                    if (instruction_in_0[14:12] == 3'b101) begin
                        if ((instruction_in_0[31:25] != 7'd0) && (instruction_in_0[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (instruction_in_0[31:25] == 7'b0000001) exc_illegal_dec = 1'b0;
                    else if (instruction_in_0[14:12] == 3'b000) begin
                        if ((instruction_in_0[31:25] != 7'd0) && (instruction_in_0[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                    else if (instruction_in_0[14:12] == 3'b001) begin
                        if (instruction_in_0[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                    else if (instruction_in_0[14:12] == 3'b101) begin
                        if ((instruction_in_0[31:25] != 7'd0) && (instruction_in_0[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                    else begin
                        if (instruction_in_0[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (instruction_in_0[14:12] == 3'b010) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (instruction_in_0[14:12] == 3'b100) exc_illegal_dec = 1'b1;
                    if (instruction_in_0[14:12] == 3'b000) begin
                        case (instruction_in_0[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;   // ecall / ebreak / mret / wfi（wfi 当 NOP）
                            default: exc_illegal_dec = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec) begin
                        if (!csr_exists_dec) exc_illegal_dec = 1'b1;
                        if (csr_ro_dec) begin
                            if (csr_wr_dec) exc_illegal_dec = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec = 1'b1;
            endcase
        end
    end

    always @(*) begin
        class_decoded      = CLS_ALU;
        write_enable_decoded         = 1'b0;
        csr_write_enable_decoded    = 1'b0;
        csr_write_commit_decoded    = 1'b0;
        destination_pick_decoded    = 5'd0;
        is_store_decoded      = 1'b0;
        multiply_operation_decoded      = 3'd0;
        load_store_function3_decoded       = 3'd0;
        csr_function3_decoded      = 3'd0;
        function4_decoded      = 4'd0;
        csr_address_decoded       = 12'd0;
        jump_value_decoded       = 32'd0;
        branch_flag_decoded        = 1'b0;
        jump_link_flag_decoded       = 1'b0;
        jump_link_register_flag_decoded      = 1'b0;

        case (instruction_in_0[6:0])
            OPCODE_OP: begin
                if (instruction_in_0[31:25] == 7'b0000001) begin
                    class_decoded = CLS_MUL;
                    multiply_operation_decoded = instruction_in_0[14:12];
                    destination_pick_decoded = destination_decoded;   //★ MUL 也要给 rd：不给 dec_rd=0 ⇒ pstat 不登记它（见 isq 的 d_wr）
                end
                else begin
                    function4_decoded   = {function10_decoded[8], function10_decoded[2:0]};
                    destination_pick_decoded = destination_decoded;
                    write_enable_decoded      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                function4_decoded   = {function10_decoded[8], function10_decoded[2:0]};
                destination_pick_decoded = destination_decoded;
                write_enable_decoded      = 1'b1;
            end
            OPCODE_JAL: begin
                class_decoded   = CLS_BJU;
                jump_link_flag_decoded    = 1'b1;
                destination_pick_decoded = destination_decoded;
                write_enable_decoded      = 1'b1;
                jump_value_decoded    = address_in_0 + immJ(instruction_in_0) - 32'd4;
            end
            OPCODE_JALR: begin
                class_decoded   = CLS_BJU;
                jump_link_register_flag_decoded   = 1'b1;
                destination_pick_decoded = destination_decoded;
                write_enable_decoded      = 1'b1;
                jump_value_decoded    = jump_link_register_prediction_in_0;
            end
            OPCODE_BRANCH: begin
                class_decoded   = CLS_BJU;
                branch_flag_decoded     = 1'b1;
                function4_decoded   = {1'b0, instruction_in_0[14:12]};
                jump_value_decoded    = immediate_decoded - 32'd4;
            end
            OPCODE_LUI: begin
                destination_pick_decoded = destination_decoded;
                write_enable_decoded      = 1'b1;
            end
            OPCODE_AUIPC: begin
                destination_pick_decoded = destination_decoded;
                write_enable_decoded      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_decoded   = CLS_LSU;
                load_store_function3_decoded    = instruction_in_0[14:12];
                destination_pick_decoded = destination_decoded;
                jump_value_decoded    = immediate_decoded;
            end
            OPCODE_STORE: begin
                class_decoded   = CLS_LSU;
                is_store_decoded   = 1'b1;
                load_store_function3_decoded    = instruction_in_0[14:12];
                jump_value_decoded    = immediate_decoded;
            end
            OPCODE_SYSTEM: begin
                class_decoded   = CLS_BJU;
                csr_function3_decoded   = instruction_in_0[14:12];
                destination_pick_decoded = destination_decoded;
                case (instruction_in_0[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        write_enable_decoded         = 1'b1;
                        csr_write_enable_decoded    = 1'b1;
                        csr_write_commit_decoded    = csr_wr_dec;
                    end
                    default: begin
                        write_enable_decoded      = 1'b0;
                        csr_write_enable_decoded = 1'b0;
                    end
                endcase
//★ csr 地址【不写也照给】：地址供 csr.v 的读口用，写不写只决定"锁不锁"。
                csr_address_decoded = (instruction_in_0[14:12] != 3'b000) ? instruction_in_0[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec | exception_jump_misalign_decoded) begin
            write_enable_decoded         = 1'b0;
            csr_write_enable_decoded    = 1'b0;
            csr_write_commit_decoded    = 1'b0;
        end
    end

    always @(*) begin
        source_a_immediate_decoded  = 1'b0;
        source_b_immediate_decoded  = 1'b0;
        source_a_immediate_value_decoded = 32'd0;
        source_b_immediate_value_decoded = 32'd0;
        case (instruction_in_0[6:0])
            OPCODE_OP_IMM: begin source_b_immediate_decoded = 1'b1; source_b_immediate_value_decoded = immediate_decoded; end
            OPCODE_JALR:   begin source_b_immediate_decoded = 1'b1; source_b_immediate_value_decoded = immediate_decoded; end
            OPCODE_LUI:    begin source_b_immediate_decoded = 1'b1; source_b_immediate_value_decoded = immediate_decoded; end
            OPCODE_AUIPC:  begin
                source_a_immediate_decoded  = 1'b1; source_a_immediate_value_decoded = address_in_0;
                source_b_immediate_decoded  = 1'b1; source_b_immediate_value_decoded = immediate_decoded;
            end
            OPCODE_SYSTEM: begin
                if ((instruction_in_0[14:12] == 3'b101) | (instruction_in_0[14:12] == 3'b110) | (instruction_in_0[14:12] == 3'b111)) begin
                    source_a_immediate_decoded  = 1'b1;
                    source_a_immediate_value_decoded = {27'd0, instruction_in_0[19:15]};
                end
            end
            default: ;
        endcase
    end

    always @(*) begin
        exc_ecall_dec   = (instruction_in_0[6:0] == OPCODE_SYSTEM) && (instruction_in_0[31:20] == 12'h000);
        exc_ebreak_dec  = (instruction_in_0[6:0] == OPCODE_SYSTEM) && (instruction_in_0[31:20] == 12'h001);
        exc_irq_ret_dec = (instruction_in_0[6:0] == OPCODE_SYSTEM) && (instruction_in_0[14:12] == 3'b000)
                        && (instruction_in_0[31:20] == 12'h302);
    end

    always @(*) begin
        branch_jump_function4_decoded = BJU_NONE;
        if (exc_illegal_dec)
            branch_jump_function4_decoded = BJU_ILLEGAL;
        else if (exc_ecall_dec)
            branch_jump_function4_decoded = BJU_ECALL;
        else if (exc_ebreak_dec)
            branch_jump_function4_decoded = BJU_EBREAK;
        else if (exc_irq_ret_dec)
            branch_jump_function4_decoded = BJU_MRET;
        else if (branch_flag_decoded)
            branch_jump_function4_decoded = {1'b1, function4_decoded[2:0]};
        else if (jump_link_flag_decoded)
            branch_jump_function4_decoded = exception_jump_misalign_decoded ? BJU_JAL_MIS : BJU_JAL;
        else if (jump_link_register_flag_decoded)
            branch_jump_function4_decoded = BJU_JALR;
    end

//rob 分配口：分配发生在"出队格 h0 换新"那一拍，吃 h0 的 Q 端译码。
    always @(*) begin
        alloc_en_0  = alloc_lane0;
        alloc_rd_0   = destination_decoded;
        alloc_we_0   = (destination_decoded != 5'd0);
        alloc_jmp_0  = jump_decoded;
        alloc_ecall_0   = exc_ecall_dec;
        alloc_ebreak_0  = exc_ebreak_dec;
        alloc_illegal_0 = exc_illegal_dec;
        alloc_mret_0    = exc_irq_ret_dec;
        alloc_fence_0   = (instruction_in_0[6:0] == 7'b0001111) && (instruction_in_0[14:12] == 3'b000);
        alloc_fence_i_0 = (instruction_in_0[6:0] == 7'b0001111) && (instruction_in_0[14:12] == 3'b001);
        alloc_pend_0 = (instruction_in_0[6:0] == OPCODE_LOAD) | (instruction_in_0[6:0] == 7'b0100011) |
                     ((instruction_in_0[6:0] == OPCODE_OP) && (instruction_in_0[31:25] == 7'b0000001));
    end

    always @(*) begin
        destination_decoded_1 = 5'd0;
        function10_decoded_1 = 10'd0;
        immediate_decoded_1 = 32'd0;
        jump_decoded_1 = 1'b0;
        case (instruction_stage1[6:0])
            OPCODE_OP: begin
                destination_decoded_1 = instruction_stage1[11:7];
                function10_decoded_1 = {instruction_stage1[31:25], instruction_stage1[14:12]};
            end
            OPCODE_OP_IMM: begin
                immediate_decoded_1 = immI(instruction_stage1);
                destination_decoded_1 = instruction_stage1[11:7];
                function10_decoded_1 = {(instruction_stage1[14:12] == 3'b001 || instruction_stage1[14:12] == 3'b101) ? instruction_stage1[31:25] : 7'd0,
                            instruction_stage1[14:12]};
            end
            OPCODE_JAL: begin
                destination_decoded_1 = instruction_stage1[11:7];
                jump_decoded_1 = 1'b1;
            end
            OPCODE_JALR: begin
                destination_decoded_1 = instruction_stage1[11:7];
                immediate_decoded_1 = immI(instruction_stage1);
                jump_decoded_1 = 1'b1;
            end
            OPCODE_BRANCH: begin
                immediate_decoded_1 = immB(instruction_stage1);
                function10_decoded_1 = {7'd0, instruction_stage1[14:12]};
                jump_decoded_1 = 1'b1;
            end
            OPCODE_LOAD: begin
                immediate_decoded_1 = immI(instruction_stage1);
                destination_decoded_1 = instruction_stage1[11:7];
            end
            OPCODE_STORE: begin
                immediate_decoded_1 = immS(instruction_stage1);
            end
            OPCODE_LUI: begin
                immediate_decoded_1 = immU(instruction_stage1);
                destination_decoded_1 = instruction_stage1[11:7];
            end
            OPCODE_AUIPC: begin
                immediate_decoded_1 = immU(instruction_stage1) - 4'd4;
                destination_decoded_1 = instruction_stage1[11:7];
            end
            OPCODE_SYSTEM: begin
                destination_decoded_1 = instruction_stage1[11:7];
                function10_decoded_1 = {7'd0, instruction_stage1[14:12]};
            end
        endcase
    end

    always @(*) begin
        csr_is_access_dec_1 = (instruction_stage1[14:12] != 3'b000) && (instruction_stage1[14:12] != 3'b100);
        csr_wr_dec_1        = (instruction_stage1[14:12] == 3'b001) | (instruction_stage1[14:12] == 3'b101) |
                            (instruction_stage1[19:15] != 5'd0);
        csr_ro_dec_1        = (instruction_stage1[31:30] == 2'b11);
        csr_exists_dec_1    = 1'b0;
        case (instruction_stage1[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec_1 = 1'b1;
            default: csr_exists_dec_1 = 1'b0;
        endcase
    end

    always @(*) begin
        exc_illegal_dec_1 = 1'b0;
        exception_jump_misalign_decoded_1    = (instruction_stage1[6:0] == OPCODE_JAL) & instruction_stage1[21];
        if (instruction_stage1 != 32'd0) begin
            case (instruction_stage1[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec_1 = 1'b0;
                OPCODE_LOAD: begin
                    if (instruction_stage1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b110) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b111) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_STORE: begin
                    if (instruction_stage1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b100) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b101) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b110) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b111) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_JALR: begin
                    if (instruction_stage1[14:12] != 3'b000) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (instruction_stage1[14:12] == 3'b010) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (instruction_stage1[14:12] == 3'b001) begin
                        if (instruction_stage1[31:25] != 7'd0) exc_illegal_dec_1 = 1'b1;
                    end
                    if (instruction_stage1[14:12] == 3'b101) begin
                        if ((instruction_stage1[31:25] != 7'd0) && (instruction_stage1[31:25] != 7'b0100000))
                            exc_illegal_dec_1 = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (instruction_stage1[31:25] == 7'b0000001) exc_illegal_dec_1 = 1'b0;
                    else if (instruction_stage1[14:12] == 3'b000) begin
                        if ((instruction_stage1[31:25] != 7'd0) && (instruction_stage1[31:25] != 7'b0100000))
                            exc_illegal_dec_1 = 1'b1;
                    end
                    else if (instruction_stage1[14:12] == 3'b001) begin
                        if (instruction_stage1[31:25] != 7'd0) exc_illegal_dec_1 = 1'b1;
                    end
                    else if (instruction_stage1[14:12] == 3'b101) begin
                        if ((instruction_stage1[31:25] != 7'd0) && (instruction_stage1[31:25] != 7'b0100000))
                            exc_illegal_dec_1 = 1'b1;
                    end
                    else begin
                        if (instruction_stage1[31:25] != 7'd0) exc_illegal_dec_1 = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (instruction_stage1[14:12] == 3'b010) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (instruction_stage1[14:12] == 3'b100) exc_illegal_dec_1 = 1'b1;
                    if (instruction_stage1[14:12] == 3'b000) begin
                        case (instruction_stage1[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;
                            default: exc_illegal_dec_1 = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec_1) begin
                        if (!csr_exists_dec_1) exc_illegal_dec_1 = 1'b1;
                        if (csr_ro_dec_1) begin
                            if (csr_wr_dec_1) exc_illegal_dec_1 = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec_1 = 1'b1;
            endcase
        end
    end

    always @(*) begin
        class_decoded_1      = CLS_ALU;
        write_enable_decoded_1         = 1'b0;
        csr_write_enable_decoded_1    = 1'b0;
        csr_write_commit_decoded_1    = 1'b0;
        destination_pick_decoded_1    = 5'd0;
        is_store_decoded_1      = 1'b0;
        multiply_operation_decoded_1      = 3'd0;
        load_store_function3_decoded_1       = 3'd0;
        csr_function3_decoded_1      = 3'd0;
        function4_decoded_1      = 4'd0;
        csr_address_decoded_1       = 12'd0;
        jump_value_decoded_1       = 32'd0;
        branch_flag_decoded_1        = 1'b0;
        jump_link_flag_decoded_1       = 1'b0;
        jump_link_register_flag_decoded_1      = 1'b0;
        case (instruction_stage1[6:0])
            OPCODE_OP: begin
                if (instruction_stage1[31:25] == 7'b0000001) begin
                    class_decoded_1 = CLS_MUL;
                    multiply_operation_decoded_1 = instruction_stage1[14:12];
                    destination_pick_decoded_1 = destination_decoded_1;
                end
                else begin
                    function4_decoded_1   = {function10_decoded_1[8], function10_decoded_1[2:0]};
                    destination_pick_decoded_1 = destination_decoded_1;
                    write_enable_decoded_1      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                function4_decoded_1   = {function10_decoded_1[8], function10_decoded_1[2:0]};
                destination_pick_decoded_1 = destination_decoded_1;
                write_enable_decoded_1      = 1'b1;
            end
            OPCODE_JAL: begin
                class_decoded_1   = CLS_BJU;
                jump_link_flag_decoded_1    = 1'b1;
                destination_pick_decoded_1 = destination_decoded_1;
                write_enable_decoded_1      = 1'b1;
                jump_value_decoded_1    = address_stage1 + immJ(instruction_stage1) - 32'd4;
            end
            OPCODE_JALR: begin
                class_decoded_1   = CLS_BJU;
                jump_link_register_flag_decoded_1   = 1'b1;
                destination_pick_decoded_1 = destination_decoded_1;
                write_enable_decoded_1      = 1'b1;
                jump_value_decoded_1    = jump_link_register_stage1;
            end
            OPCODE_BRANCH: begin
                class_decoded_1   = CLS_BJU;
                branch_flag_decoded_1     = 1'b1;
                function4_decoded_1   = {1'b0, instruction_stage1[14:12]};
                jump_value_decoded_1    = immediate_decoded_1 - 32'd4;
            end
            OPCODE_LUI: begin
                destination_pick_decoded_1 = destination_decoded_1;
                write_enable_decoded_1      = 1'b1;
            end
            OPCODE_AUIPC: begin
                destination_pick_decoded_1 = destination_decoded_1;
                write_enable_decoded_1      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_decoded_1   = CLS_LSU;
                load_store_function3_decoded_1    = instruction_stage1[14:12];
                destination_pick_decoded_1 = destination_decoded_1;
                jump_value_decoded_1    = immediate_decoded_1;
            end
            OPCODE_STORE: begin
                class_decoded_1   = CLS_LSU;
                is_store_decoded_1   = 1'b1;
                load_store_function3_decoded_1    = instruction_stage1[14:12];
                jump_value_decoded_1    = immediate_decoded_1;
            end
            OPCODE_SYSTEM: begin
                class_decoded_1   = CLS_BJU;
                csr_function3_decoded_1   = instruction_stage1[14:12];
                destination_pick_decoded_1 = destination_decoded_1;
                case (instruction_stage1[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        write_enable_decoded_1         = 1'b1;
                        csr_write_enable_decoded_1    = 1'b1;
                        csr_write_commit_decoded_1    = csr_wr_dec_1;
                    end
                    default: begin
                        write_enable_decoded_1      = 1'b0;
                        csr_write_enable_decoded_1 = 1'b0;
                    end
                endcase
                csr_address_decoded_1 = (instruction_stage1[14:12] != 3'b000) ? instruction_stage1[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec_1 | exception_jump_misalign_decoded_1) begin
            write_enable_decoded_1         = 1'b0;
            csr_write_enable_decoded_1    = 1'b0;
            csr_write_commit_decoded_1    = 1'b0;
        end
    end

    always @(*) begin
        source_a_immediate_decoded_1  = 1'b0;
        source_b_immediate_decoded_1  = 1'b0;
        source_a_immediate_value_decoded_1 = 32'd0;
        source_b_immediate_value_decoded_1 = 32'd0;
        case (instruction_stage1[6:0])
            OPCODE_OP_IMM: begin source_b_immediate_decoded_1 = 1'b1; source_b_immediate_value_decoded_1 = immediate_decoded_1; end
            OPCODE_JALR:   begin source_b_immediate_decoded_1 = 1'b1; source_b_immediate_value_decoded_1 = immediate_decoded_1; end
            OPCODE_LUI:    begin source_b_immediate_decoded_1 = 1'b1; source_b_immediate_value_decoded_1 = immediate_decoded_1; end
            OPCODE_AUIPC:  begin
                source_a_immediate_decoded_1  = 1'b1; source_a_immediate_value_decoded_1 = address_stage1;
                source_b_immediate_decoded_1  = 1'b1; source_b_immediate_value_decoded_1 = immediate_decoded_1;
            end
            OPCODE_SYSTEM: begin
                if ((instruction_stage1[14:12] == 3'b101) | (instruction_stage1[14:12] == 3'b110) | (instruction_stage1[14:12] == 3'b111)) begin
                    source_a_immediate_decoded_1  = 1'b1;
                    source_a_immediate_value_decoded_1 = {27'd0, instruction_stage1[19:15]};
                end
            end
            default: ;
        endcase
    end

    always @(*) begin
        exc_ecall_dec_1   = (instruction_stage1[6:0] == OPCODE_SYSTEM) && (instruction_stage1[31:20] == 12'h000);
        exc_ebreak_dec_1  = (instruction_stage1[6:0] == OPCODE_SYSTEM) && (instruction_stage1[31:20] == 12'h001);
        exc_irq_ret_dec_1 = (instruction_stage1[6:0] == OPCODE_SYSTEM) && (instruction_stage1[14:12] == 3'b000)
                        && (instruction_stage1[31:20] == 12'h302);
    end

    always @(*) begin
        branch_jump_function4_decoded_1 = BJU_NONE;
        if (exc_illegal_dec_1)
            branch_jump_function4_decoded_1 = BJU_ILLEGAL;
        else if (exc_ecall_dec_1)
            branch_jump_function4_decoded_1 = BJU_ECALL;
        else if (exc_ebreak_dec_1)
            branch_jump_function4_decoded_1 = BJU_EBREAK;
        else if (exc_irq_ret_dec_1)
            branch_jump_function4_decoded_1 = BJU_MRET;
        else if (branch_flag_decoded_1)
            branch_jump_function4_decoded_1 = {1'b1, function4_decoded_1[2:0]};
        else if (jump_link_flag_decoded_1)
            branch_jump_function4_decoded_1 = exception_jump_misalign_decoded_1 ? BJU_JAL_MIS : BJU_JAL;
        else if (jump_link_register_flag_decoded_1)
            branch_jump_function4_decoded_1 = BJU_JALR;
    end

    always @(*) begin
//★ 2026-11-09 三宽：口 1 = 第二格（h1：访存走 LSU、非访存走 lane1）；
        alloc_en_1   = (alloc_lane0 ? (h1_is_mem ? lsu_push_1 : alloc_lane1) : alloc_lane1);
//★ h1 自己是访存时，口 1 描述的是 h1（不是 pick）；它的跳转/CSR/系统位恒 0。
        alloc_rd_1   = pick_mem ? instruction_in_1[11:7] : destination_decoded_1;
        alloc_we_1   = pick_mem ? (~h1_store & (instruction_in_1[11:7] != 5'd0)) : (destination_decoded_1 != 5'd0);
        alloc_jmp_1  = pick_mem ? 1'b0 : jump_decoded_1;
        alloc_ecall_1   = pick_mem ? 1'b0 : exc_ecall_dec_1;
        alloc_ebreak_1  = pick_mem ? 1'b0 : exc_ebreak_dec_1;
        alloc_illegal_1 = pick_mem ? 1'b0 : exc_illegal_dec_1;
        alloc_mret_1    = pick_mem ? 1'b0 : exc_irq_ret_dec_1;
        alloc_fence_1   = pick_mem ? 1'b0 : ((instruction_stage1[6:0] == 7'b0001111) && (instruction_stage1[14:12] == 3'b000));
        alloc_fence_i_1 = pick_mem ? 1'b0 : ((instruction_stage1[6:0] == 7'b0001111) && (instruction_stage1[14:12] == 3'b001));
        alloc_pend_1 = pick_mem ? 1'b1
                   : ((instruction_stage1[6:0] == OPCODE_LOAD) | (instruction_stage1[6:0] == 7'b0100011) |
                     ((instruction_stage1[6:0] == OPCODE_OP) && (instruction_stage1[31:25] == 7'b0000001)));
    end

    always @(*) begin
//h2 走 LSU（lsu_push_2）还是走 lane1（h1 是访存时的 pick）
        alloc_en_2   = lsu_push_2 | (alloc_lane0 & h1_is_mem & alloc_lane1 & rob_room_2_in);
        alloc_rd_2   = lsu_push_2 ? h2_rd : destination_decoded_1;
        alloc_we_2   = lsu_push_2 ? (h2_rd != 5'd0) : (destination_decoded_1 != 5'd0);
        alloc_phys_2 = lsu_push_2 ? phys_rd_2 : phys_rd_3;
        alloc_jmp_2  = lsu_push_2 ? 1'b0 : jump_decoded_1;
        alloc_ecall_2   = lsu_push_2 ? 1'b0 : exc_ecall_dec_1;
        alloc_ebreak_2  = lsu_push_2 ? 1'b0 : exc_ebreak_dec_1;
        alloc_illegal_2 = lsu_push_2 ? 1'b0 : exc_illegal_dec_1;
        alloc_mret_2    = lsu_push_2 ? 1'b0 : exc_irq_ret_dec_1;
        alloc_fence_2   = lsu_push_2 ? 1'b0 : ((instruction_stage1[6:0] == 7'b0001111) && (instruction_stage1[14:12] == 3'b000));
        alloc_fence_i_2 = lsu_push_2 ? 1'b0 : ((instruction_stage1[6:0] == 7'b0001111) && (instruction_stage1[14:12] == 3'b001));
        alloc_pend_2 = lsu_push_2 ? 1'b1
                                  : ((instruction_stage1[6:0] == OPCODE_LOAD) | (instruction_stage1[6:0] == 7'b0100011) |
                                     ((instruction_stage1[6:0] == OPCODE_OP) && (instruction_stage1[31:25] == 7'b0000001)));
    end

    reg [PHYS_BITS-1:0] rat      [0:ARCH_COUNT-1];
    reg [PHYS_BITS-1:0] arch_map [0:ARCH_COUNT-1];
    reg [PHYS_COUNT-1:0] free_vector;

//偶数位掩码 / 奇数位掩码 = 两个 bank
    localparam [PHYS_COUNT-1:0] EVEN_MASK = {PHYS_COUNT/2{2'b01}};
    localparam [PHYS_COUNT-1:0] ODD_MASK  = {PHYS_COUNT/2{2'b10}};

    wire [PHYS_COUNT-1:0] free_even = free_vector & EVEN_MASK;
    wire [PHYS_COUNT-1:0] free_odd  = free_vector & ODD_MASK;

//★ 每 bank 的空闲号个数（寄存版）。给前端背压的只有"这个 bank 还剩几个"这一个状态量，
//  不再让 free_vector 的选号扫描钻进 idu 的分配门（那条路会一路拖到 iffu 的推进上）。
    localparam [5:0] BANK_FREE_INIT = (PHYS_COUNT - ARCH_COUNT) / 2;
    reg [5:0] free_count_even, free_count_odd;
    wire [5:0] flush_count_even = flush_count_even_comb;
    wire [5:0] flush_count_odd  = flush_count_odd_comb;

//这一条要不要发号：有写回、且不是 x0
    wire need_0 = rename_alloc_en_0 & rename_alloc_we_0 & (rename_alloc_rd_0 != 5'd0);
    wire need_1 = rename_alloc_en_1 & rename_alloc_we_1 & (rename_alloc_rd_1 != 5'd0);
    wire need_2 = rename_alloc_en_2 & rename_alloc_we_2 & (rename_alloc_rd_2 != 5'd0);
    wire need_3 = rename_alloc_en_3 & rename_alloc_we_3 & (rename_alloc_rd_3 != 5'd0);

//每个 bank 每拍要发【两个】号（口 0/2 取偶数 bank，口 1/3 取奇数 bank）。
//  第一轮照旧取最低空闲位；把它从空闲向量里抠掉再取一次，就是第二个号。
//  倒序 for 让最低位最后赋值 ⇒ 低位优先（与原写法同一个技巧）。
    reg [PHYS_BITS-1:0]   pick_e0, pick_e1, pick_o0, pick_o1;
    reg [PHYS_COUNT-1:0]  pick_e0_onehot, pick_e1_onehot, pick_o0_onehot, pick_o1_onehot;
    reg                   found_e0, found_e1, found_o0, found_o1;
    reg [PHYS_COUNT-1:0]  free_even_2nd, free_odd_2nd;
    integer k;
    always @(*) begin
        pick_e0 = {PHYS_BITS{1'b0}}; pick_e0_onehot = {PHYS_COUNT{1'b0}}; found_e0 = 1'b0;
        pick_o0 = {PHYS_BITS{1'b0}}; pick_o0_onehot = {PHYS_COUNT{1'b0}}; found_o0 = 1'b0;
        for (k = PHYS_COUNT - 1; k >= 0; k = k - 1) begin
            if (free_even[k]) begin
                pick_e0        = k;
                pick_e0_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_e0       = 1'b1;
            end
            if (free_odd[k]) begin
                pick_o0        = k;
                pick_o0_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_o0       = 1'b1;
            end
        end
        free_even_2nd = free_even & ~pick_e0_onehot;
        free_odd_2nd  = free_odd  & ~pick_o0_onehot;
        pick_e1 = {PHYS_BITS{1'b0}}; pick_e1_onehot = {PHYS_COUNT{1'b0}}; found_e1 = 1'b0;
        pick_o1 = {PHYS_BITS{1'b0}}; pick_o1_onehot = {PHYS_COUNT{1'b0}}; found_o1 = 1'b0;
        for (k = PHYS_COUNT - 1; k >= 0; k = k - 1) begin
            if (free_even_2nd[k]) begin
                pick_e1        = k;
                pick_e1_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_e1       = 1'b1;
            end
            if (free_odd_2nd[k]) begin
                pick_o1        = k;
                pick_o1_onehot = {{(PHYS_COUNT-1){1'b0}}, 1'b1} << k;
                found_o1       = 1'b1;
            end
        end
    end

//★ 口 0 没要号时口 2 直接顶上【第一个】号：不让它空等"第二个"。
    wire alloc_ok_0 = need_0 & found_e0;
    wire alloc_ok_1 = need_1 & found_o0;
    wire [PHYS_BITS-1:0]  pick_b0_1 = alloc_ok_0 ? pick_e1 : pick_e0;
    wire [PHYS_BITS-1:0]  pick_b1_1 = alloc_ok_1 ? pick_o1 : pick_o0;
    wire [PHYS_COUNT-1:0] pick_b0_1_onehot = alloc_ok_0 ? pick_e1_onehot : pick_e0_onehot;
    wire [PHYS_COUNT-1:0] pick_b1_1_onehot = alloc_ok_1 ? pick_o1_onehot : pick_o0_onehot;
    wire                  found_b0_1 = alloc_ok_0 ? found_e1 : found_e0;
    wire                  found_b1_1 = alloc_ok_1 ? found_o1 : found_o0;
    wire alloc_ok_2 = need_2 & found_b0_1;
    wire alloc_ok_3 = need_3 & found_b1_1;

//★ 纯状态：本 bank 已经没有空号（口 2/3 是"只剩一个"，因为口 0/1 进来还要给它留一个）。
//  给前端做保守门（与 alloc_en 无组合环），防止"发了号但 ROB 那边没接"⇒ 号漏光
//  ⇒ 输出物理 0 ⇒ 架构映射被写成 x0 那一格。
//  ★ 判据只吃 free_count_* 这两个寄存器，不再从 free_vector 现扫一遍。
    assign bank_empty_0 = (free_count_even == 6'd0);
    assign bank_empty_1 = (free_count_odd  == 6'd0);
    assign bank_empty_2 = (free_count_even == 6'd0) | ((free_count_even == 6'd1) & need_0);
    assign bank_empty_3 = (free_count_odd  == 6'd0) | ((free_count_odd  == 6'd1) & need_1);
    assign bank_low_even = (free_count_even <= 6'd2);
    assign bank_low_odd  = (free_count_odd  <= 6'd2);
    assign alloc_stall_0 = need_0 & bank_empty_0;
    assign alloc_stall_1 = need_1 & bank_empty_1;
    assign alloc_stall_2 = need_2 & bank_empty_2;
    assign alloc_stall_3 = need_3 & bank_empty_3;

//---- 源操作数：查 rat，带【只对更老的写】旁路 ----
//  ★ 自己这一条的写不能旁路到自己的读：`add x5, x5, x1` 的源必须是旧 x5。
//    lane0 最老 ⇒ 看不到任何本拍写；lane1 只看到 lane0 的写。
    assign phys_rs1_0 = rat[rename_alloc_rs1_0];
    assign phys_rs2_0 = rat[rename_alloc_rs2_0];
    assign phys_rs1_1 = (need_0 & (rename_alloc_rd_0 == rename_alloc_rs1_1)) ? pick_e0 : rat[rename_alloc_rs1_1];
    assign phys_rs2_1 = (need_0 & (rename_alloc_rd_0 == rename_alloc_rs2_1)) ? pick_e0 : rat[rename_alloc_rs2_1];
//★ 口 2 看得见口 0/1 的写，口 3 看得见口 0/1/2 的写；同一个 rs 上更年轻的那个赢。
//  （门用 alloc_ok：没抢到号的那条根本不会写 rat，拿它的号去旁路是错的。）
    assign phys_rs1_2 = (alloc_ok_1 & (rename_alloc_rd_1 == rename_alloc_rs1_2)) ? pick_o0
                      : (alloc_ok_0 & (rename_alloc_rd_0 == rename_alloc_rs1_2)) ? pick_e0
                      : rat[rename_alloc_rs1_2];
    assign phys_rs2_2 = (alloc_ok_1 & (rename_alloc_rd_1 == rename_alloc_rs2_2)) ? pick_o0
                      : (alloc_ok_0 & (rename_alloc_rd_0 == rename_alloc_rs2_2)) ? pick_e0
                      : rat[rename_alloc_rs2_2];
    assign phys_rs1_3 = (alloc_ok_2 & (rename_alloc_rd_2 == rename_alloc_rs1_3)) ? pick_b0_1
                      : (alloc_ok_1 & (rename_alloc_rd_1 == rename_alloc_rs1_3)) ? pick_o0
                      : (alloc_ok_0 & (rename_alloc_rd_0 == rename_alloc_rs1_3)) ? pick_e0
                      : rat[rename_alloc_rs1_3];
    assign phys_rs2_3 = (alloc_ok_2 & (rename_alloc_rd_2 == rename_alloc_rs2_3)) ? pick_b0_1
                      : (alloc_ok_1 & (rename_alloc_rd_1 == rename_alloc_rs2_3)) ? pick_o0
                      : (alloc_ok_0 & (rename_alloc_rd_0 == rename_alloc_rs2_3)) ? pick_e0
                      : rat[rename_alloc_rs2_3];
    assign phys_rd_0  = pick_e0;
    assign phys_rd_1  = pick_o0;
    assign phys_rd_2  = pick_b0_1;
    assign phys_rd_3  = pick_b1_1;

//==============================================================================
// 四个分配口 + 六个源查询地址（2026-11-11 合并后必须在 rename 内部直接生成）
//  口 1 = 第二格（h1）：它是访存时用它的访存字段，否则用 pick 的译码（此时 pick 就是 h1）。
//  口 2 = h2 走 LSU；口 3 = h2 走 lane1。两者互斥。
//==============================================================================
    assign rename_alloc_en_0 = alloc_lane0 & ~rob_full_in;
    assign rename_alloc_we_0 = (destination_decoded != 5'd0);
    assign rename_alloc_rd_0 = destination_decoded;
    assign rename_alloc_rs1_0 = source_a_dec_0;
    assign rename_alloc_rs2_0 = source_b_dec_0;

    assign rename_alloc_en_1 = (alloc_lane0 ? (h1_is_mem ? lsu_push_1 : alloc_lane1) : alloc_lane1) & ~rob_full_in;
    assign rename_alloc_we_1 = pick_mem ? (~h1_store & (instruction_in_1[11:7] != 5'd0)) : (destination_decoded_1 != 5'd0);
    assign rename_alloc_rd_1 = pick_mem ? instruction_in_1[11:7] : destination_decoded_1;
    assign rename_alloc_rs1_1 = pick_mem ? instruction_in_1[19:15] : source_a_dec_1;
    assign rename_alloc_rs2_1 = pick_mem ? (h1_store ? instruction_in_1[24:20] : 5'd0) : source_b_dec_1;

    assign rename_alloc_en_2 = lsu_push_2 & ~rob_full_in;
    assign rename_alloc_we_2 = (h2_rd != 5'd0);
    assign rename_alloc_rd_2 = h2_rd;
    assign rename_alloc_rs1_2 = h2_rs1;
    assign rename_alloc_rs2_2 = h2_rs2;

    assign rename_alloc_en_3 = (alloc_lane0 & h1_is_mem & alloc_lane1) & ~rob_full_in;
    assign rename_alloc_we_3 = (destination_decoded_1 != 5'd0);
    assign rename_alloc_rd_3 = destination_decoded_1;
    assign rename_alloc_rs1_3 = source_a_dec_1;
    assign rename_alloc_rs2_3 = source_b_dec_1;

//★ 2026-11-11：这四个是六个查询口的源地址，合并时整块漏了 ⇒ 全 0
    assign lsu_push_source_a = source_a_dec_0;
    assign lsu_push_source_b = source_b_dec_0;
    assign push1_source_a    = alloc_lane0 ? source_a_dec_1 : source_a_dec_0;
    assign push1_source_b    = alloc_lane0 ? source_b_dec_1 : source_b_dec_0;

    assign rename_lookup_a = lsu_push_source_a;
    assign rename_lookup_b = lsu_push_source_b;
    assign rename_lookup_c = pick_mem ? instruction_in_1[19:15] : push1_source_a;
    assign rename_lookup_d = pick_mem ? (h1_store ? instruction_in_1[24:20] : 5'd0) : push1_source_b;
    assign rename_lookup_e = lsu_push_2 ? h2_rs1 : source_a_dec_1;
    assign rename_lookup_f = lsu_push_2 ? h2_rs2 : source_b_dec_1;

    assign alloc_phys_0 = phys_rd_0;
    assign alloc_phys_1 = phys_rd_1;
//ROB 口 1 装的是哪条：lane0 吃了就是 h1，没吃就是 h0。
    assign alloc_pc_1   = alloc_lane0 ? address_in_1 : address_in_0;

//★ 2026-11-10：owner = "这条指令的结果将来从哪个 PRF 写口落笔"。
    assign alloc_owner_0 = (class_decoded == CLS_MUL) ? 3'd1 :
                           (class_decoded == CLS_LSU) ? 3'd4 : 3'd0;
    assign alloc_owner_1 = (pick_mem | (class_decoded_1 == CLS_LSU)) ? 3'd4 :
                           ((class_decoded_1 == CLS_MUL) ? 3'd3 : 3'd2);
    assign alloc_owner_2 = lsu_push_2 ? 3'd4 : ((class_decoded_1 == CLS_MUL) ? 3'd3 : 3'd2);

    always @(*) begin
        alloc_csr_0 = csr_write_enable_decoded;
        alloc_csr_1 = pick_mem ? 1'b0 : csr_write_enable_decoded_1;
        alloc_csr_2 = lsu_push_2 ? 1'b0 : csr_write_enable_decoded_1;
    end

//---- 通用查找口 ----
//  ★ 查找口看到的必须是【本拍分配之前】的映射：一条 `add x5,x5,x1` 的 rs 读的是旧 x5。
//    所以这里只查 rat，不做任何本拍写的旁路。
//  a/b 那一组挂的是 lane0 自己（最老），没有更老的写要旁路；
//  c/d 那一组可能落在 lane1 上，必须看见同拍 lane0 的分配，否则 `add x5,..` / `add ..,x5` 连发会拿到旧号。
    wire bypass_0 = alloc_ok_0 & (rename_alloc_rd_0 != 5'd0);
    assign lookup_a_phys = rat[rename_lookup_a];
    assign lookup_b_phys = rat[rename_lookup_b];
    assign lookup_c_phys = (bypass_0 & (rename_alloc_rd_0 == rename_lookup_c)) ? pick_e0 : rat[rename_lookup_c];
    assign lookup_d_phys = (bypass_0 & (rename_alloc_rd_0 == rename_lookup_d)) ? pick_e0 : rat[rename_lookup_d];
//★ 2026-11-09 三宽分配：第三条的查找口。它看得见口 0/1 的写（口 1 更新，赢）。
    assign lookup_e_phys = (alloc_ok_1 & (rename_alloc_rd_1 == rename_lookup_e)) ? pick_o0
                         : (alloc_ok_0 & (rename_alloc_rd_0 == rename_lookup_e)) ? pick_e0
                         : rat[rename_lookup_e];
    assign lookup_f_phys = (alloc_ok_1 & (rename_alloc_rd_1 == rename_lookup_f)) ? pick_o0
                         : (alloc_ok_0 & (rename_alloc_rd_0 == rename_lookup_f)) ? pick_e0
                         : rat[rename_lookup_f];


//---- 提交口要释放的旧号 ----
//  `old = arch_map[rd]` 现读现用，不在 ROB 里另存 old_phys。
//  同拍两条写同一个 rd：后一条（B）释放的是 A 刚定格进去的号，不是原来的那个。
    wire old_valid_a = write_enable_a_in & (commit_rd0_in != 5'd0);
    wire old_valid_b = write_enable_b_in & (commit_rd1_in != 5'd0);
    wire same_rd     = old_valid_a & old_valid_b & (commit_rd0_in == commit_rd1_in);

    wire [PHYS_BITS-1:0] old_a = arch_map[commit_rd0_in];
    wire [PHYS_BITS-1:0] old_b = same_rd ? commit_phys0_in : arch_map[commit_rd1_in];

    wire [PHYS_COUNT-1:0] freed_a_onehot = old_valid_a ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << old_a) : {PHYS_COUNT{1'b0}};
    wire [PHYS_COUNT-1:0] freed_b_onehot = old_valid_b ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << old_b) : {PHYS_COUNT{1'b0}};
    wire [PHYS_COUNT-1:0] freed_mask     = freed_a_onehot | freed_b_onehot;

    wire [PHYS_COUNT-1:0] alloc_mask = (alloc_ok_0 ? pick_e0_onehot   : {PHYS_COUNT{1'b0}})
                                     | (alloc_ok_1 ? pick_o0_onehot   : {PHYS_COUNT{1'b0}})
                                     | (alloc_ok_2 ? pick_b0_1_onehot : {PHYS_COUNT{1'b0}})
                                     | (alloc_ok_3 ? pick_b1_1_onehot : {PHYS_COUNT{1'b0}});
    assign alloc_onehot = alloc_mask;

//★ 2026-11-12：冲刷后的空闲表 = arch_map 那 32 个号的补集。
//  晚一拍重建（用户定的）：T 拍冲刷时 arch_map 已经被那条提交更新过，
//  所以 T+1 拍只读 arch_map 这个【寄存器】就能重建 —— 不再碰 rob 的提交口，
//  跨模块那一跳 1.9ns 和提交口 mux 全部消失。
    reg                  flush_pending_q;   // 冲刷打一拍：重建晚一拍落地
    reg [PHYS_COUNT-1:0] arch_mask_now;
    integer m;
    always @(*) begin
        arch_mask_now = {PHYS_COUNT{1'b0}};
        for (m = 0; m < ARCH_COUNT; m = m + 1)
            arch_mask_now = arch_mask_now | ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << arch_map[m]);
    end
    wire flush_pending = flush_pending_q;
    wire [PHYS_COUNT-1:0] flush_free_mask = ~arch_mask_now;
    assign arch_next_onehot = ~arch_mask_now;

//---- 两个 bank 的空闲号个数：按位向量同一套事件加减，冲刷时按构造重算 ----
//  分配扣、提交还，口径与 free_vector 逐条同源 ⇒ 两者不会走散。
    wire [3:0] alloc_cnt_even = {3'd0, alloc_ok_0} + {3'd0, alloc_ok_2};
    wire [3:0] alloc_cnt_odd  = {3'd0, alloc_ok_1} + {3'd0, alloc_ok_3};
    wire [3:0] freed_cnt_even = {3'd0, (old_valid_a & ~old_a[0])} + {3'd0, (old_valid_b & ~old_b[0])};
    wire [3:0] freed_cnt_odd  = {3'd0, (old_valid_a &  old_a[0])} + {3'd0, (old_valid_b &  old_b[0])};

//★ popcount 树：32 个偶位两两相加，5 级出 6 位。输入是寄存器量（arch_mask_now）。
    wire [1:0] fce_l0_0 = flush_free_mask[0] + flush_free_mask[2];
    wire [1:0] fce_l0_1 = flush_free_mask[4] + flush_free_mask[6];
    wire [1:0] fce_l0_2 = flush_free_mask[8] + flush_free_mask[10];
    wire [1:0] fce_l0_3 = flush_free_mask[12] + flush_free_mask[14];
    wire [1:0] fce_l0_4 = flush_free_mask[16] + flush_free_mask[18];
    wire [1:0] fce_l0_5 = flush_free_mask[20] + flush_free_mask[22];
    wire [1:0] fce_l0_6 = flush_free_mask[24] + flush_free_mask[26];
    wire [1:0] fce_l0_7 = flush_free_mask[28] + flush_free_mask[30];
    wire [1:0] fce_l0_8 = flush_free_mask[32] + flush_free_mask[34];
    wire [1:0] fce_l0_9 = flush_free_mask[36] + flush_free_mask[38];
    wire [1:0] fce_l0_10 = flush_free_mask[40] + flush_free_mask[42];
    wire [1:0] fce_l0_11 = flush_free_mask[44] + flush_free_mask[46];
    wire [1:0] fce_l0_12 = flush_free_mask[48] + flush_free_mask[50];
    wire [1:0] fce_l0_13 = flush_free_mask[52] + flush_free_mask[54];
    wire [1:0] fce_l0_14 = flush_free_mask[56] + flush_free_mask[58];
    wire [1:0] fce_l0_15 = flush_free_mask[60] + flush_free_mask[62];
    wire [2:0] fce_l1_0 = fce_l0_0 + fce_l0_1;
    wire [2:0] fce_l1_1 = fce_l0_2 + fce_l0_3;
    wire [2:0] fce_l1_2 = fce_l0_4 + fce_l0_5;
    wire [2:0] fce_l1_3 = fce_l0_6 + fce_l0_7;
    wire [2:0] fce_l1_4 = fce_l0_8 + fce_l0_9;
    wire [2:0] fce_l1_5 = fce_l0_10 + fce_l0_11;
    wire [2:0] fce_l1_6 = fce_l0_12 + fce_l0_13;
    wire [2:0] fce_l1_7 = fce_l0_14 + fce_l0_15;
    wire [3:0] fce_l2_0 = fce_l1_0 + fce_l1_1;
    wire [3:0] fce_l2_1 = fce_l1_2 + fce_l1_3;
    wire [3:0] fce_l2_2 = fce_l1_4 + fce_l1_5;
    wire [3:0] fce_l2_3 = fce_l1_6 + fce_l1_7;
    wire [4:0] fce_l3_0 = fce_l2_0 + fce_l2_1;
    wire [4:0] fce_l3_1 = fce_l2_2 + fce_l2_3;
    wire [5:0] fce_l4_0 = fce_l3_0 + fce_l3_1;
    wire [1:0] fco_l0_0 = flush_free_mask[1] + flush_free_mask[3];
    wire [1:0] fco_l0_1 = flush_free_mask[5] + flush_free_mask[7];
    wire [1:0] fco_l0_2 = flush_free_mask[9] + flush_free_mask[11];
    wire [1:0] fco_l0_3 = flush_free_mask[13] + flush_free_mask[15];
    wire [1:0] fco_l0_4 = flush_free_mask[17] + flush_free_mask[19];
    wire [1:0] fco_l0_5 = flush_free_mask[21] + flush_free_mask[23];
    wire [1:0] fco_l0_6 = flush_free_mask[25] + flush_free_mask[27];
    wire [1:0] fco_l0_7 = flush_free_mask[29] + flush_free_mask[31];
    wire [1:0] fco_l0_8 = flush_free_mask[33] + flush_free_mask[35];
    wire [1:0] fco_l0_9 = flush_free_mask[37] + flush_free_mask[39];
    wire [1:0] fco_l0_10 = flush_free_mask[41] + flush_free_mask[43];
    wire [1:0] fco_l0_11 = flush_free_mask[45] + flush_free_mask[47];
    wire [1:0] fco_l0_12 = flush_free_mask[49] + flush_free_mask[51];
    wire [1:0] fco_l0_13 = flush_free_mask[53] + flush_free_mask[55];
    wire [1:0] fco_l0_14 = flush_free_mask[57] + flush_free_mask[59];
    wire [1:0] fco_l0_15 = flush_free_mask[61] + flush_free_mask[63];
    wire [2:0] fco_l1_0 = fco_l0_0 + fco_l0_1;
    wire [2:0] fco_l1_1 = fco_l0_2 + fco_l0_3;
    wire [2:0] fco_l1_2 = fco_l0_4 + fco_l0_5;
    wire [2:0] fco_l1_3 = fco_l0_6 + fco_l0_7;
    wire [2:0] fco_l1_4 = fco_l0_8 + fco_l0_9;
    wire [2:0] fco_l1_5 = fco_l0_10 + fco_l0_11;
    wire [2:0] fco_l1_6 = fco_l0_12 + fco_l0_13;
    wire [2:0] fco_l1_7 = fco_l0_14 + fco_l0_15;
    wire [3:0] fco_l2_0 = fco_l1_0 + fco_l1_1;
    wire [3:0] fco_l2_1 = fco_l1_2 + fco_l1_3;
    wire [3:0] fco_l2_2 = fco_l1_4 + fco_l1_5;
    wire [3:0] fco_l2_3 = fco_l1_6 + fco_l1_7;
    wire [4:0] fco_l3_0 = fco_l2_0 + fco_l2_1;
    wire [4:0] fco_l3_1 = fco_l2_2 + fco_l2_3;
    wire [5:0] fco_l4_0 = fco_l3_0 + fco_l3_1;
    wire [5:0] flush_count_even_comb = fce_l4_0;
    wire [5:0] flush_count_odd_comb  = fco_l4_0;
    integer i;
    always @(posedge clk) begin
        if (reset_registered) begin
            for (i = 0; i < ARCH_COUNT; i = i + 1) begin
                rat[i]      <= i;
                arch_map[i] <= i;
            end
//0..ARCH_COUNT-1 被架构映射占着，上面那些才是空闲池

        end
        else if (irq_kill | redir_frontend | redir_backend_0) begin
//一拍恢复：整片抄回 + 把在册的号收回来。
//★ 冲刷拍【常常同时压着一笔打拍后的提交】：rob.v 明确写了"打拍里那笔在边界之前，清了会丢一笔合法写"。
//★ 2026-11-05：改向判定后分配一直冻到那条分支提交（rob 的 redirect_stop_alloc）

            for (i = 0; i < ARCH_COUNT; i = i + 1) begin
                if      (old_valid_b && (commit_rd1_in == i)) rat[i] <= commit_phys1_in;
                else if (old_valid_a && (commit_rd0_in == i)) rat[i] <= commit_phys0_in;
                else                                        rat[i] <= arch_map[i];
            end
            if (old_valid_a) arch_map[commit_rd0_in] <= commit_phys0_in;
            if (old_valid_b) arch_map[commit_rd1_in] <= commit_phys1_in;

        end
        else begin
//分配：同一个 rd 被同拍多条写时，最年轻的那条赢（按 age 次序赋值，后写覆盖先写）
            if (alloc_ok_0) rat[rename_alloc_rd_0] <= pick_e0;
            if (alloc_ok_1) rat[rename_alloc_rd_1] <= pick_o0;
            if (alloc_ok_2) rat[rename_alloc_rd_2] <= pick_b0_1;
            if (alloc_ok_3) rat[rename_alloc_rd_3] <= pick_b1_1;

//提交：定格架构映射
            if (old_valid_a) arch_map[commit_rd0_in] <= commit_phys0_in;
            if (old_valid_b) arch_map[commit_rd1_in] <= commit_phys1_in;


        end
    end


//==============================================================================
//  空闲表：正常增量更新；冲刷晚一拍按 arch_map 重建（T+1，只读寄存器）
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            free_vector     <= ~({{(PHYS_COUNT-ARCH_COUNT){1'b0}}, {ARCH_COUNT{1'b1}}});
            free_count_even <= BANK_FREE_INIT;
            free_count_odd  <= BANK_FREE_INIT;
            flush_pending_q <= 1'b0;
        end
        else begin
            flush_pending_q <= irq_kill | redir_frontend | redir_backend_0;
            if (flush_pending_q) begin
                free_vector     <= flush_free_mask;
                free_count_even <= flush_count_even;
                free_count_odd  <= flush_count_odd;
            end
            else begin
                free_vector <= (free_vector & ~alloc_mask) | freed_mask;
                free_count_even <= free_count_even - {2'b0, alloc_cnt_even} + {2'b0, freed_cnt_even};
                free_count_odd  <= free_count_odd  - {2'b0, alloc_cnt_odd}  + {2'b0, freed_cnt_odd};
            end
        end
    end

//==============================================================================
//  2026-11-11 拆分（方案 B）：进 dsq 的 133 位载荷
//    ★ 三条各自独立，只看【这一条自己】的 opcode 和位段，不经过 instruction_stage1 那个 mux。
//==============================================================================
    function writes_of;
        input [31:0] inst;
        begin
            writes_of = (inst[11:7] != 5'd0)
                     && (inst[6:0] != OPCODE_STORE)
                     && (inst[6:0] != OPCODE_BRANCH)
                     && (inst[6:0] != OPCODE_MISC_MEM);
        end
    endfunction
    function [1:0] class_of;
        input        mem;
        input [31:0] inst;
        begin
            if (mem)                             class_of = 2'd2;
            else if (inst[6:0] == OPCODE_BRANCH) class_of = 2'd3;
            else if (inst[6:0] == OPCODE_JAL)    class_of = 2'd3;
            else if (inst[6:0] == OPCODE_JALR)   class_of = 2'd3;
            else if ((inst[6:0] == OPCODE_OP) && (inst[31:25] == 7'b0000001)) class_of = 2'd1;
            else                                 class_of = 2'd0;
        end
    endfunction

    wire [4:0] h1_rs1 = instruction_in_1[19:15];
    wire [4:0] h1_rs2 = h1_store ? 5'd0 : instruction_in_1[24:20];
//★ 2026-11-11：alloc_lane0=0 而 alloc_lane1=1 时，h0 是从 ROB 口 1 分配出去的
//  （alloc_en_1 拿的是 alloc_lane1）。载荷必须带口 1 的目的号，不是口 0 的。
    wire h0_via_port1 = ~alloc_lane0 & alloc_lane1;
    wire [5:0] dest_phys_0 = writes_of(instruction_in_0)
                           ? (h0_via_port1 ? phys_rd_1 : phys_rd_0) : 6'd0;
    wire [5:0] dest_phys_1 = writes_of(instruction_in_1) ? phys_rd_1 : 6'd0;
    wire [5:0] dest_phys_2 = writes_of(instruction_in_2)
                           ? (is_mem_2 ? phys_rd_2 : phys_rd_3) : 6'd0;

    assign push0_valid = alloc_lane0 | alloc_lane1;
    assign push0_data  = {instruction_in_0, jump_link_register_prediction_in_0, address_in_0,
                          dest_phys_0, lookup_b_phys, lookup_a_phys,
                          generation_in_0, index_in_0, branch_prediction_in_0,
                          source_b_dec_0, source_a_dec_0, class_decoded};
    assign push1_valid = alloc_lane0 & (h1_is_mem ? lsu_push_1 : alloc_lane1);
    assign push1_data  = {instruction_in_1, jump_link_register_prediction_in_1, address_in_1,
                          dest_phys_1, lookup_d_phys, lookup_c_phys,
                          generation_in_1, index_in_1, branch_prediction_in_1,
                          h1_rs2, h1_rs1, class_of(h1_is_mem, instruction_in_1)};
    assign push2_valid = (alloc_lane0 & h1_is_mem & alloc_lane1) | lsu_push_2;
    assign push2_data  = {instruction_in_2, jump_link_register_prediction_in_2, address_in_2,
                          dest_phys_2, lookup_f_phys, lookup_e_phys,
                          generation_in_2, index_in_2, branch_prediction_in_2,
                          h2_rs2, h2_rs1, class_of(is_mem_2, instruction_in_2)};

endmodule
