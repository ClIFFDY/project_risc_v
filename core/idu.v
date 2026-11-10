`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu
// Description:
//   取指队列与载荷之间的寄存器级（= 原 mid_decoder 的位置），并入 rd / func10 / imm 的译码。
//
//   输入是 iffu 的出队格（F 级，原样指令字 + 随行号），输出是本级的寄存器（S 级）：
//   全译码（入站分类 / 各单元字段 / 异常标记 / 非寄存器操作数覆盖）由 S 级的指令字
//   【组合】解出、并随载荷同沿寄存进 S 级（原来这一份解在 isq 的入站拍上）。
//
//   ★ 两套译码源不同、用途不同：
//     - 载荷面（rd/func10/imm）与全译码解在 S 级的 instruction 上，给 isq 的入站；
//     - rob 分配口只判"这一类写不写 rd"，解在 **fifo 队头**（nh0）上 —— 入队发生在
//       "队头进 出队格"那一拍，rob 要的是那一条的 rd，不能裸取 instruction[11:7]
//       （BRANCH/STORE 会变出幽灵写者）。
//
//   ★ 扫槽口（scan_rs）给的是【本级输入那一级】（F）的 rs：rob 的读拍在本级输出（S），
//     扫描必须提前一拍（见 rob.v 的相位说明）。
//////////////////////////////////////////////////////////////////////////////////

module idu(
    input clk, rst,
//★ 2026-10-31：不再接整根 flag_bus —— 只接本模块真正用到的三根。
//  原来的 [2] pc_redirect 与 [0] bus_hold 与本级的推进无关（前者是取指侧的事，
//  后者已挪进 lsu 的 blank_bus），留着只会让人以为它们还在这儿起作用。
    input        redir_frontend,     // 前端重定向（rob 判定拍出的那根）
    input        redir_backend_0,    // 后端重定向（rob 队头那一拍出的那根）
    input        exc_kill,           // 异常交付（rob 的 trap_fire）
    input        irq_kill,           // 中断受理（clint 自己判的）
//★ S5：后端域 = flag_bus[7:5]（提交口清表 / 异常 / 中断）。只做两件事：
//  rename / 空闲表回滚、挡住本拍的落盘与入队。**不清输入 lane** —— 提交口那一拍
//  前端正在往分配口送正确路径的指令，那批必须留着。
    input       exec_in,            // = flag_bus[4]
    input       stall_rob_full_in,  // = flag_bus[3]
//★ 冲刷的【寄存版】（cont 的 flush_pending，晚一拍）。只给访存 FIFO 的清零用：
//  组合版 flush_control_in 要穿过 csr 的中断锥（含 rob 的 head_valid），那条链实测 18 级、
//  终点正是 lsu_queue_*_reg 的 R 脚。清零晚一拍无害 —— 冲刷拍进队门本来就关着，
//  而冲刷之后 iffu 队头被清空、下一拍也不会有人进队。
    input       flush_pend,
//★ 2026-11-08：击杀掩码端口已摘。改向只在提交口发一次、队头退休那拍整窗作废，
//  rob 的掩码本来就是 MASK_ALL 常量 ⇒ 按槽查表等价于"整条清空"，这里直接判 flush_pend。
//冲刷的年龄边界（与 isq 的 kill 判据逐项同门）。输出级里挂着的那条【比冲刷点老】就不是错的，
//必须留住 —— 见下面 keep_0/keep_1 那段。
    input [1:0]  store_pop,
    input [3:0]  store_flush_pop,
    output reg   credit_stall_out,
//给 fifo 的推进量 = 本拍真正装了几条（0/1/2/3/4）。iffu 按自己手上有效条数截断后移动指针。
//★ 2026-11-10：3 位 → 4 位（前端一拍灌 4 条，docs/SPEC_frontend_4wide.txt）
    output reg [3:0] advance,
//★ 保留站的空位（isq 出口）：站满就把本 lane 的分配门关掉。
//  本级的 2 深载荷从此只剩"管线寄存器"的作用 —— 缓冲在站里。
    input        station_room_0_valid,
    input        station_room_1_valid,
//本拍 lane0 装没装：lane1 到底拿出队格的第二格(h1)还是队头(h0)由它定 ⇒ 寄存器堆读口的
//lane1 地址要跟着它选（rs 在 F 级就要发出去，见 cpu_top）
    output reg   lane0_load_o,

//============================== lane0 ==============================
//来自 iffu 出队格（F 级）
    input [31:0] instruction_in_0,
    input [31:0] address_in_0,
    input        branch_prediction_in_0,
    input [31:0] jump_link_register_prediction_in_0,
    input [4:0]  index_in_0,
    input        generation_in_0,
//出队格这一格是不是真的有效（iffu 的 h0_valid/h1_valid）。不许用 addr!=0 去推 —— 那个判据要经过
//iffu 的组合 pop/sidx 才能在载荷上看到，拿它做 load 门会和 pop 成组合环。
    input        instruction_valid_0,
//操作数就绪（rob 按冻结生产者槽现算）与发射拍被接收（isq 把"就绪 + FU 空闲"合成这一根）。
//"原地等"必须【整组一起停】：否则本级照推会把 S 级内容原地覆盖掉、那条永远进不了载荷。
    input        ready1_in_0, ready2_in_0,
    input        accept_in_0,
//给发射级：寄存器里有指令、且这条还没发出去。
//不能直接用 payload_valid_out_0 —— 它描述"寄存器里有内容"，而 lane0 先发出去之后内容还在。
    output reg   lane_valid_0,
//载荷（原 mid_decoder 的输出集合 + 随行号）
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
//它们即使不写 rd（rd=x0），单元照样会报一笔完成口 ⇒ 不能当"没人会回报"提前置可退；
//否则它立刻退休、槽被回收，而它的完成口（除法 30+ 拍）晚回来时会撞上槽的新住户。
    output reg        alloc_pend_0,
//SYSTEM 类（分配拍译码）：这几类的异常/改向标记在 bju 判定延迟级才出 ⇒ ROB 退休要等那一级。
//★ 保持四根分开，ROB 只在判据里或，不合成一根。
    output reg        alloc_ecall_0, alloc_ebreak_0, alloc_illegal_0, alloc_mret_0,
    output reg [4:0]  alloc_rd_0,
//★ 2026-11-04（CSR 步骤 C）：这条是 CSR 指令，以及随 ROB 项走的载荷
//  {写使能 1, csr 地址 12, funct3 3, 源 6}。源 = 立即数形式的 zimm，或寄存器形式的 rs1 物理号；
//  提交拍由 funct3[2] 二选一（1xx = 立即数形式，恰好就是本条）。
    output reg        alloc_csr_0,
//★ 2026-11-04 真 PRF：这一条被分到的物理号，交给 rob 随项存（提交定格 / 冲刷回收都用它）
    output [5:0]      alloc_phys_0,
//★ 2026-11-04（精确冲刷第 1 步）：这一条分配时顶掉的那个映射（= rat[rd] 的旧值），随 ROB 项走。
//  精确冲刷时用它把 rat 回滚到边界那一刻，而不必整表抄 arch_map。本步只存不用，对通路透明。
//★ 2026-11-04 真 PRF：给 PRF 的就绪位当门（本拍分配的号 / 冲刷后仍在用的号）
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
//与 lane0 逐条同形，一条不少、一条不多
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
    output reg [4:0]  alloc_rd_2,
    output reg        alloc_csr_2,
    output reg [5:0]  alloc_phys_2,
//★ 2026-11-10：这一条将来由【哪个 PRF 写口】写（PRF 的 owner 表在分配那一拍记下，发射时按它选阵列）。
//  编码必须与 prf.v 的 OWNER_* 一致：0=ALU0 早口 1=EU0 结果口 2=ALU1 早口 3=EU1 结果口 4=LOAD。
//  依据同 alloc_phys_2 那条 mux：口 2 是访存走 LSU、否则是 h1 走 lane1。
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
//   iffu 队头按【这一格的 opcode】分流三路：lane0 / lane1 / 这里。每拍最多进一条。
//   条目只存 {instruction, pc, rob 索引, 世代, 两个源的生产者 tag} —— rs1/rs2/imm 一律不存：
//   队头是寄存器，组合译码正好一拍落到 isq 的 D；操作数不在这里读，由 isq 按物理号从 prf 取。
//★ 生产者 tag 为什么随条目走：访存条在 FIFO 里可能躺好几拍才轮到 isq 的 D，
//  而 pstat 只保留"最年轻的写者"。等它出队时才查表，查到的是那些【比它年轻】、
//  在它排队期间已经登记的写者 ⇒ 拿错值（实测 seed8：`sw x31,116(x1)` 的 x1 被一条更年轻的
//  写者顶掉，地址算成 0x468，tohost 的写丢）。所以在【进队那一拍】就把 tag 查好、随条目带走。
//==============================================================================
    input         lsu_pop_in,
//---- 提交口（rename 的架构映射定格 / 空闲表回收）----
    input         write_enable_a_in, write_enable_b_in,
    input  [4:0]  commit_rd0_in, commit_rd1_in,
//★ 2026-11-04 真 PRF：提交项的物理号
    input  [5:0]  commit_phys0_in, commit_phys1_in,
//★ 2026-11-04 精确冲刷：rat 回滚值（每架构号最老被杀项的 old_phys）+ 幸存在飞项掩码
//★ 2026-11-05：前两个掩码已经不用（回滚改成"提交拍整片抄回 arch_map"），端口留着
    input  [31:0]  squash_arch_hit_in,
    input  [191:0] squash_arch_phys_in,
    input  [31:0]  squash_survive_hit_in,
    input          alloc_stop_in,
    input          restore_in,
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
//★ 2026-11-06 提前唤醒：目的物理号也随载荷走。isq 的站里要拿"这条会写哪个物理号"当唤醒源，
//  挂在自己发射口上的项比结果上总线早一拍就能被叫起来。ROB 里也有这个号，但绕过去是
//  isq→rob 查表→isq 两跳回来，直接随载荷带过来没有那一跳。
                      phys_d_out_0, phys_d_out_1
    );

//==============================================================================
//  参数与声明区：本模块全部 localparam/parameter + reg/wire。
//  Verilog 要求先声明后用，所以全放这里；正文只留逻辑。
//==============================================================================
//★宏化旋钮：2 = 双发（大核）、1 = 单发（小核）。
//  LANES=1 时 alloc_lane1 恒 0 ⇒ iffu 每拍弹 1 条、h1 下一拍自己变队头，指令不丢；
//  lane1 的载荷/单元随之变成死逻辑，综合自动裁掉。
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
//★ 分支档用 bit3 当类标志、低 3 位就是 funct3 ⇒ bju 的比较 case 与原来逐字一致；
//★ 非法编码覆盖一切（在 bjuf4 的译码里排第一）：非法分支 / 非法 jalr 一律不许再判。
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
//   位域（低位在前）： [5:0] b 口物理号  [11:6] a 口物理号  [12] 世代  [17:13] rob 槽号
//          [49:18] pc  [81:50] instruction  [86:82] rs2  [91:87] rs1  [97:92] 目的物理号
//   ★ rs1/rs2 只为一件事存在：提交清 tag 时要比对 rd。只比 (槽, 世代) 会误清 —— 世代只有 1 位，
//     同一个槽转两圈就又对上了，别的指令一提交就把我的 tag 清掉（实测 seed20：`remu x28` 的 tag
//     被一条无关的提交清掉 ⇒ 尾声那条 `sw x28` 读到架构旧值 0）。
//==============================================================================
    localparam integer LSU_QUEUE_WIDTH = 98;
//★ 2026-11-06：队列加深到 8、两个写口。理由是分配口本来就该是"iffu → rename → 各去处"，
//  lane1 不该被"这一条是访存"门掉：一对 (普通, 访存) 或者 (访存, 访存) 都应该一拍全收。
//  两个写口只在【两条都是访存】时才用得上；那种对的第二条只能去队列，不能进 lane1 的载荷。
    localparam [3:0]   LSU_DEPTH = 4'd8;
    localparam integer LSU_AW    = 3;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，工具只能在布局阶段自己复制。每个模块各自打一拍，
//寄存器就落在本模块旁边；全核都只打一拍，彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
//flush_control_in / exec_in / stall_rob_full_in 合起来 = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
    reg exec, flush_window, flush_back_window;
//★ 两条 lane 各自的"已发射"位。lane0/lane1 的发射彼此独立之后，"寄存器内容的消费"与"寄存器装载"
//  不再同拍：lane0 先发出去、lane1 还要等它的操作数的那几拍里，lane0 的寄存器里躺着的仍是同一条
//  指令 —— 没有这一对位，它会被每拍重新发一次（同一条指令重复进 ROB）。
    reg lane0_free, lane1_free, go, load_lane0, load_lane1;
//★ 声明必须在使用之前（Vivado 报 VRFC 10-3380；Verilator 不管）：这三根在分配判据（下面的
//  alloc_lane0/1）里就用到了，而它们的赋值/例化口在文件末尾。
    wire       rename_bank_empty_0, rename_bank_empty_1;
    wire       rename_bank_empty_2, rename_bank_empty_3;
    reg [4:0]  destination_decoded, destination_decoded_1;
    reg [31:0] instruction_stage1, address_stage1, jump_link_register_stage1;
    wire [5:0] lookup_a_phys, lookup_b_phys, lookup_c_phys, lookup_d_phys;
    wire [5:0] lookup_e_phys, lookup_f_phys;
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
    reg        alloc_lane0, alloc_lane1, lane0_store, lane1_store;
//==============================================================================
// S 级载荷的 2 深 FIFO（2026-10-31）
//   头 = 下面那组原来的 Q 寄存器（一个字段没动，isq 看到的端口一个字不改）
//   影子 = 这一组。推入只看本地空位（lane0_free = ~头 | ~影子），不再问 isq 的 accept；
//   出队时影子整体抬进头 —— 这就是 advance 的扇入里不再有 accept 的原因。
//   结构与 lsu fifo 同形：队头还是原来那组寄存器，只多一格影子。
//==============================================================================
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
    reg [2:0]  lsu_read_pointer, lsu_write_pointer;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_head;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_clear_0, lsu_clear_1, lsu_clear_2, lsu_clear_3;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_clear_4, lsu_clear_5, lsu_clear_6, lsu_clear_7;
    reg        lsu_tag_hit_0a, lsu_tag_hit_0b, lsu_tag_hit_1a, lsu_tag_hit_1b;
    reg        lsu_tag_hit_2a, lsu_tag_hit_2b, lsu_tag_hit_3a, lsu_tag_hit_3b;
//★ lane1 这一拍拿 fifo 的哪一格：lane0 也在装 ⇒ 拿第二格；lane0 没装（在等操作数）⇒ 拿队头。
//  所以 lane 编号不再代表程序序 —— 程序序由随指令带下去的 rob 索引保证，这里只按"装了几条"走指针。
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
//bju 功能码（本级的唯一一个"这条指令在 bju 里算什么"的口）：
//原来 bju 要分别吃 func4 / br_flag / jalr_flag / 四路异常五个口，其中 func4 与这四个
//没有任何区分力（addi/beq/jalr 的 func4 都是 0000），"是不是分支"只能靠额外的类位。
//这里一次译清：非法 > ecall > ebreak > mret > 分支（带 funct3）> jal（带非对齐）> jalr。
//★ 非法排第一：非法分支/非法 jalr 的编码不许再当控制指令判，否则会一边报非法一边改向。
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
//==============================================================================
//队头只做字段提取，不复制主译码的大 case：LOAD/STORE 的 rs1/rs2/rd/funct3 都在固定位上。
    reg [31:0] lsu_head_instruction, lsu_head_pc;
    reg [4:0]  lsu_head_index;
    reg        lsu_head_generation;
//★ 2026-11-04 真 PRF：进队条目带的是两个源的【物理号】，在进队这一拍由 rename 查出。
//  查找口的驱动必须用【译码后的 rs】：LUI/AUIPC/JAL 这些没有 rs 的，
//  instruction[19:15]/[24:20] 是立即数位段，裸切片会给自己认一个假号（实测 w11）。
    wire [4:0]  lsu_push_source_a, lsu_push_source_b;
    wire [4:0]  push1_source_a, push1_source_b;
//★ 冲刷：按击杀掩码【截断】尾部，不再整条清（rob 的 flush_partial 恒 0 那一套已经不成立了）。
//  击杀集合在程序序上是一段后缀，而本 FIFO 就是程序序 ⇒ 从第一项被杀的开始往后全丢，
//  前面那几项（比边界老、ROB 里还在等）必须留住，否则它们的 ROB 项永远等不到完成。
//  不动读指针；写指针对齐到"读指针 + 存活数"，维持 write = read + count 这个不变式。
//==============================================================================
// 真 PRF 的重命名表（2026-11-04）
//   rat/arch_map/free_vector 都在 rename 里。这里只用它算两件事：
//   ① 本条指令 rd 分到的物理号（交给 rob 随项存）
//   ② rs1/rs2 翻译成的物理号（下一步替换 isq 的 (槽,世代) 生产者表时用）
//   分配点选在 idu 的解码/分配拍 —— 和 rob 的槽分配是同一拍、同一条指令。
//==============================================================================
    wire [5:0] rename_phys_rd_0, rename_phys_rd_1, rename_phys_rd_2, rename_phys_rd_3;
    wire [5:0] rename_phys_rs1_0, rename_phys_rs2_0;
    wire [5:0] rename_phys_rs1_1, rename_phys_rs2_1;
    wire       rename_stall_0, rename_stall_1;
    wire       rename_stall_2, rename_stall_3;
    wire [63:0] rename_alloc_onehot, rename_arch_next_onehot;

//★ 交给站的"目的物理号"必须只在【本条真的会写】时才有意义。rename 的 pick 是空闲扫描的
//  结果，非写指令（或 rd=x0）也带着它 —— 实测：连续两条 rd=x0 的指令载荷里 phys_d 都是 22，
//  而 ROB 给它们记的是 0；站里据此发提前唤醒就把一个没人写的号报成"下一拍可读"，
//  消费者下一拍在 PRF 里读到旧值（rand 种子 11 / 14 挂在这上面）。与 rob.v 的 entry_phys 同口径。
    wire [5:0] write_phys_decoded_0, write_phys_decoded_1;
//★ 2026-11-09：MUL/DIV 那一支原来不置 WE ⇒ 这里恒给 0，发射级拿不到乘法的目的号。
//  只给 phys_d 补上，WE 位保持原样（改 WE 会牵动别处，实测会挂死）。
    assign write_phys_decoded_0 = ((write_enable_decoded   || (class_decoded   == CLS_MUL)) && (destination_pick_decoded   != 5'd0)) ? rename_phys_rd_0 : 6'd0;
    assign write_phys_decoded_1 = ((write_enable_decoded_1 || (class_decoded_1 == CLS_MUL)) && (destination_pick_decoded_1 != 5'd0)) ? rename_phys_rd_1 : 6'd0;

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


    (* max_fanout = 32 *) reg [3:0] lsu_queue_count;
    always @(*) begin
        flush_window = exc_kill | irq_kill | redir_frontend;
        flush_back_window = exc_kill | irq_kill | redir_backend_0;
        exec    = exec_in;
        lane0_free = (~payload_valid_out_0 | ~skid_valid_0) & station_room_0_valid;
        lane1_free = (~payload_valid_out_1 | ~skid_valid_1) & station_room_1_valid;
        go    = exec & ~stall_rob_full_in;
        is_mem_0 = (instruction_in_0[6:0] == OPCODE_LOAD) | (instruction_in_0[6:0] == OPCODE_STORE);
//★ 2026-11-09 三宽分配（docs/SPEC_alloc3.md）：
//  · h1_is_mem 说的是【h1 自己】是不是访存（它走 LSU）——与"lane1 最终拿哪条"分开；
//  · lane1 拿的是"第二条非访存"，所以 h1 是访存时它顺延到 h2。
        h1_is_mem = (instruction_in_1[6:0] == OPCODE_LOAD) | (instruction_in_1[6:0] == OPCODE_STORE);
        is_mem_2  = (instruction_in_2[6:0] == OPCODE_LOAD) | (instruction_in_2[6:0] == OPCODE_STORE);
        h1_store  = (instruction_in_1[6:0] == OPCODE_STORE);
        h2_store  = (instruction_in_2[6:0] == OPCODE_STORE);
//访存条的字段位置固定（store/load 的 rs1[19:15]、rd[11:7] 同一位置）；store 不读 rs2 的"假源"给 0。
        h2_rs1 = instruction_in_2[19:15];
        h2_rs2 = h2_store ? instruction_in_2[24:20] : 5'd0;
        h2_rd  = h2_store ? 5'd0 : instruction_in_2[11:7];
//★ 只看本地计数器：不许再依赖 isq 当拍弹不弹（那会把 lsu 的 stage_free 拉进 iffu 的推进锥）
        lsu_room = LSU_DEPTH - lsu_queue_count;
//★ 2026-11-05：改向一经判定，分配冻结到那条分支提交。冻结住 alloc_lane0 就把
//  ROB 分配 / 重命名 / 载荷落盘 / LSU 入队 / iffu 推进一起冻住（都挂在这一位上），
//  所以冻住期间没有任何消费者读 rat ⇒ 回滚不需要按架构号的扫描。
        alloc_lane0 = go & instruction_valid_0 & ~alloc_stop_in & (is_mem_0 ? (lsu_room != 4'd0) : lane0_free)
                    & ~(rename_bank_empty_0 & (destination_decoded != 5'd0));
//★ 2026-11-09：口 1 描述【h1 这条访存】的前提是 alloc_lane0 也成立。
//  alloc_lane0=0 时口 1 描述的是 h0（lane1 顶上去的那条），此时 h1 是不是访存无关;
//  照 h1_is_mem 选字段会把 h1 的 rd 当成 h0 的写回登记进 ROB（CoreMark 架构映射被写坏）。
        pick_mem = alloc_lane0 & h1_is_mem;
        pick_from_h2 = pick_mem;

//★ `~flush_window` 挂在【落盘】上，不挂在推进上（2026-10-31）：
//  冲刷拍照样从 iffu 弹（advance 不为 0，指针照走），但弹出来的那条【不进任何队列】——
//  它本来就是错路的。ROB 的分配口自带 `!flush_control_rob`，所以这是全链一致的。
        lane0_store = alloc_lane0 & ~is_mem_0 & ~flush_window & ~flush_back_window;
        lsu_push_0  = alloc_lane0 &  is_mem_0 & ~flush_window & ~flush_back_window;
//★ 2 深载荷的下一态判据（组合，只在本级寄存器上）：
//  · head_after_pop = 出队（accept）之后头里还有没有东西 —— 影子有货就抬上来，所以还是"有"。
//  · push           = 本拍真的要落一条进载荷（空格 addr==0 不算）。
//  · shadow_fill_0  = lane0 这一条落进【影子】而不是头 ⇒ 它会藏在第二格里，pstat 看不见它。
//  入队次序固定在载荷块里：先出队，再入队（头空进头，否则进影子）。
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
        lsu_push_1  = alloc_lane0 & instruction_valid_1 & h1_is_mem & ~flush_window & ~flush_back_window
                    & (lsu_room >= (lsu_push_0 ? 4'd2 : 4'd1))
                    & ~(rename_bank_empty_1 & ((h1_store ? 5'd0 : instruction_in_1[11:7]) != 5'd0));
//lane1 拿哪条：h1 非访存拿 h1；h1 是访存就顺延到 h2（且 h2 必须非访存 —— 两条访存挤在同拍就走不动）
        pick_bank_empty = pick_mem ? rename_bank_empty_3 : rename_bank_empty_1;
        alloc_lane1 = (LANES == 1) ? 1'b0 :
                      (go & ~alloc_stop_in
                       & (alloc_lane0 ? (h1_is_mem ? (lsu_push_1 & instruction_valid_2 & ~is_mem_1 & rob_room_2_in)
                                                   : instruction_valid_1)
                                      : (instruction_valid_0 & ~is_mem_0))
                       & lane1_free
                       & ~(pick_bank_empty & (destination_decoded_1 != 5'd0)));

        lane1_store = alloc_lane1 & ~is_mem_1 & ~flush_window & ~flush_back_window;
//★ 2026-11-09：队列加了第三个写口 ⇒ h2 是访存时不再要求前两条都非访存。
//  唯一的次序约束是"前两格必须都吃掉"（第二格可能是 LSU 口 1，也可能是 lane1 的那条）。
        lsu_h1_taken = h1_is_mem ? lsu_push_1 : alloc_lane1;
        lsu_push_2  = alloc_lane0 & lsu_h1_taken & instruction_valid_2 & is_mem_2
                    & ~flush_window & ~flush_back_window
                    & (lsu_room >= ({2'd0, lsu_push_0} + {2'd0, lsu_push_1} + 4'd1))
                    & ~(rename_bank_empty_2 & (h2_rd != 5'd0)) & rob_room_2_in;
//给 fifo 的推进量 = 本拍真正消费了几格（0..3）：指针按它走，不是"成对就 +2"。
//★ 推进量 = 真正消费了几格，逐槽算：h0 / h1 / h2。写成 alloc_lane0+alloc_lane1 会漏掉
//  "h0 吃了、h1 是访存也吃了、但 h2 没吃" 这一档（h1-mem 且 alloc_lane1=0）⇒ 少弹一格 ⇒ 同一条被分配两次。
//★ 三格逐槽算：h0 / 第二格（LSU 口 1 或 lane1）/ 第三格（LSU 口 2 或 lane1）。
//  h1 是访存时 alloc_lane1 的含义就是"h2 进 lane1"，所以第三格的两种去处要显式或起来。
        advance = {1'b0, alloc_lane0}
                + {1'b0, (alloc_lane0 ? (h1_is_mem ? lsu_push_1 : alloc_lane1) : alloc_lane1)}
                + {1'b0, (alloc_lane0 & h1_is_mem & alloc_lane1) | lsu_push_2};
        lane0_load_o = alloc_lane0;
//进队条目：口 0 = h0（最老，源用无旁路的 lookup_a/b），口 1 = h1，口 2 = h2。
        lsu_entry_0 = {rename_phys_rd_0,
                       instruction_in_0[19:15], instruction_in_0[24:20],
                       instruction_in_0, address_in_0, index_in_0, generation_in_0,
                       lookup_a_phys, lookup_b_phys};
        lsu_entry_1 = {rename_phys_rd_1,
                       instruction_in_1[19:15], h1_store ? instruction_in_1[24:20] : 5'd0,
                       instruction_in_1, address_in_1, index_in_1, generation_in_1,
                       lookup_c_phys, lookup_d_phys};
        lsu_entry_2 = {rename_phys_rd_2,
                       h2_rs1, h2_rs2,
                       instruction_in_2, address_in_2, index_in_2, generation_in_2,
                       lookup_e_phys, lookup_f_phys};
        lane1_push = lane1_store & (address_pick != 32'd0);
    end

//★ 输出级的冲刷清必须【按年龄】，不能无条件清。
//  两条 lane 独立推进 ⇒ lane0 可以跑到一条跳转、判定、冲刷，而此时 lane1 的那条**还停在输出级**
//  （它的 Q 被一条等除法的 MUL 顶着）。那条比跳转老 ⇒ 按 ROB 年龄它不在杀集里 ⇒ 它必须活下来继续发。
//  无条件清会把它从管线上抹掉，而前端已经重定向到跳转目标 ⇒ 它再也不会被取回，
//  ROB 里那一项永远 wr=0 ⇒ 队头卡死（实测：随机差分 seed14 死锁在 pc=0x310，e8 一直不完成）。
//  判据与 isq 的 kill_0/1 逐字同门：冲刷部分杀（rob_flush_partial=0 是陷阱/中断的整队杀）才谈年龄。
//★ 年龄判据改成查 rob 的击杀弧掩码（见 rob 端口注释）：本模块只问"我这条在不在被杀的弧上"，
//  head_ptr 不再扇进来。这里吃【当拍版】，因为 keep 要和 ROB 自己的作废同拍判定。
//★ keep_0 / keep_1 已删（2026-10-31）：rob 的 flush_partial 恒 0、kill_arc 恒 1
//  ⇒ 这两个量恒 0，冲刷拍两条 lane 一律清空，没有"留住比边界老的条目"这一档。

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
//★ 原来这一段在 iffu 的【推入口】上：icache 输出 → opcode case → 5 位 rs → 拼进 107 位队列包。
//  而 idu 这里本来就有一条对同一个指令字的 opcode case（上面那段）⇒ 同一份译码做了两遍，
//  且 iffu 那份还压在队列写数据的锥上。搬到这里，opcode 分类可以复用，iffu 的写口只剩拼接。
//★ 口径与 iffu 原版逐条一致：只有真读这两个源的指令才填，别的留 0 ——
//  留 0 是必须的，不留就会在冒险比较/生产者查表里误命中（LUI 的 [19:15] 是立即数）。
//★ csrrwi/si/ci 的 [19:15] 是 zimm 不是寄存器号：只对 rs1 形（funct3=001/010/011）填。
//★ slot_replay 的作废不用管：iffu 在【推入前】就把 instruction_effective 清 0 了，
//  队列里存的那条本来就是 0 ⇒ 这里解出来自然是 0。
//★ 不用 task/function：iverilog 的 `always @(*)` 不会把函数体内读到的信号放进敏感表
//  （本仓已踩过），写成两个平铺的块最稳。
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
//  h1 是访存时 lane1 拿的是 h2（instruction_stage1 已按 pick 选好）——照 instruction_in_1 解
//  会把 h1 的 rs 当成 h2 的 rs，rename 的物理号查错、载荷装错寄存器（rand seed1/3/4 死锁）。
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
//  这一根同时决定"真写使能"和"对只读 CSR 的写要不要报非法"，两者是同一个判据。
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
//  `csrr rd, csr` 若被清成地址 0 就会读回 0（实测：启动宏读 mtvec 读回 0 ⇒ 误判没写进去）。
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

//rob 分配口：分配发生在"出队格 h0 换新"那一拍，吃的就是本级的译码输入（= h0 的 Q 端），
//与 idu 自己的译码同源 —— 不再另解一份队头。rd 直接复用主译码的 destination_decoded（按 opcode 分类
//填过，不裸取 instruction[11:7]）。
//★ 发号拍 = 本级真装进一条【真指令】的那一拍。空泡（addr==0，队列空/被挡时出现）不占项：
//  它进不了发射级（lane_valid 按 addr!=0 判），拿了项就没人发、没人写 ⇒ 那项永远退不掉 ⇒
//  ROB 排空 ⇒ 队头卡死（实测 icache 自举期间 15 拍就把 ROB 灌满，整核再不推进）。
    always @(*) begin
        alloc_en_0  = alloc_lane0;
        alloc_rd_0   = destination_decoded;
        alloc_we_0   = (destination_decoded != 5'd0);
        alloc_jmp_0  = jump_decoded;
        alloc_ecall_0   = exc_ecall_dec;
        alloc_ebreak_0  = exc_ebreak_dec;
        alloc_illegal_0 = exc_illegal_dec;
        alloc_mret_0    = exc_irq_ret_dec;
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
//  口 2 = 第三格（h2：h1 是访存时它进 lane1，否则它只能是访存条）。两者互斥。
        alloc_en_1   = (alloc_lane0 ? (h1_is_mem ? lsu_push_1 : alloc_lane1) : alloc_lane1);
//★ h1 自己是访存时，口 1 描述的是 h1（不是 pick）；它的跳转/CSR/系统位恒 0。
        alloc_rd_1   = pick_mem ? instruction_in_1[11:7] : destination_decoded_1;
        alloc_we_1   = pick_mem ? (~h1_store & (instruction_in_1[11:7] != 5'd0)) : (destination_decoded_1 != 5'd0);
        alloc_jmp_1  = pick_mem ? 1'b0 : jump_decoded_1;
        alloc_ecall_1   = pick_mem ? 1'b0 : exc_ecall_dec_1;
        alloc_ebreak_1  = pick_mem ? 1'b0 : exc_ebreak_dec_1;
        alloc_illegal_1 = pick_mem ? 1'b0 : exc_illegal_dec_1;
        alloc_mret_1    = pick_mem ? 1'b0 : exc_irq_ret_dec_1;
        alloc_pend_1 = pick_mem ? 1'b1
                   : ((instruction_stage1[6:0] == OPCODE_LOAD) | (instruction_stage1[6:0] == 7'b0100011) |
                     ((instruction_stage1[6:0] == OPCODE_OP) && (instruction_stage1[31:25] == 7'b0000001)));
    end

    always @(*) begin
//h2 走 LSU（lsu_push_2）还是走 lane1（h1 是访存时的 pick）
        alloc_en_2   = lsu_push_2 | (alloc_lane0 & h1_is_mem & alloc_lane1 & rob_room_2_in);
        alloc_rd_2   = lsu_push_2 ? h2_rd : destination_decoded_1;
        alloc_we_2   = lsu_push_2 ? (h2_rd != 5'd0) : (destination_decoded_1 != 5'd0);
        alloc_phys_2 = lsu_push_2 ? rename_phys_rd_2 : rename_phys_rd_3;
        alloc_jmp_2  = lsu_push_2 ? 1'b0 : jump_decoded_1;
        alloc_ecall_2   = lsu_push_2 ? 1'b0 : exc_ecall_dec_1;
        alloc_ebreak_2  = lsu_push_2 ? 1'b0 : exc_ebreak_dec_1;
        alloc_illegal_2 = lsu_push_2 ? 1'b0 : exc_illegal_dec_1;
        alloc_mret_2    = lsu_push_2 ? 1'b0 : exc_irq_ret_dec_1;
        alloc_pend_2 = lsu_push_2 ? 1'b1
                                  : ((instruction_stage1[6:0] == OPCODE_LOAD) | (instruction_stage1[6:0] == 7'b0100011) |
                                     ((instruction_stage1[6:0] == OPCODE_OP) && (instruction_stage1[31:25] == 7'b0000001)));
    end

//★ 单一状态：pay_valid_out_* 本身就是"这条 lane 上还躺着一条没被 isq 收走的指令"。
//  只有三种事件能改它：① 装入新指令(load*) ② 被 isq 收走(acc_in_*) ③ 被年龄判死(flush_window & ~keep_*)。
//  不再另设 l0_con/l1_con：同一个事实存两份、各有一套更新规则，必然分叉 ——
//  实测 seed14 就是冲刷清了一份、而 isq 早已收走另一份 ⇒ 同一条 store 被发两次 ⇒
//  SQ 里多出一个没人认领的项 ⇒ 队列占满 ⇒ 队头 pend 永不完结（死锁在 0x4bc）。
//  ★ 这一段【不能】放进下面那个 exec 门控的载荷块：acc_in_* 可能在 exec=0 的拍成立
//  （Q 空时 isq 照样给 credit），被门掉就等于没收到确认 ⇒ 重发。
//★ payload_fresh_out_* = "这条 lane 的 D 里这一条刚装进来"（单拍脉冲）。
//  下游（isq）靠它判"新出现"，不再比 (rob 索引, 世代)：那个二元组只有 5 位，16 槽 × 2 圈就重复；
//  分流之后 lane1 看到的序列不再是连续程序流，重复更容易撞上 —— 实测 seed15：`addi x5,x0,1` 的
//  (idx=1,gen=1) 与前一条 lane1 的指令完全相同 ⇒ 它被当成"不是新的" ⇒ 没登记成 x5 的生产者
//  ⇒ 后面那条 `sw x5,...` 读到架构旧值 0（tohost 的写丢）。
//  ★ 必须是脉冲而不是"保持到被收走"：Q 满时同一条会每拍重新登记，把更年轻那条的登记盖掉。
    always @(posedge clk) begin
        if (reset_registered) begin
            payload_valid_out_0 <= 1'b0;
            payload_valid_out_1 <= 1'b0;
            payload_fresh_out_0 <= 1'b0;
            payload_fresh_out_1 <= 1'b0;
        end
        else begin
//★ 2 深之后"头上有没有有效指令" = 出队后头里还剩东西，或者本拍刚推进去一条。
            payload_valid_out_0 <= lane0_head_after_pop | lane0_push;
            payload_valid_out_1 <= lane1_head_after_pop | lane1_push;
//★ fresh 仍然是"头里这一条第一次出现在输出口上"的单拍脉冲，只是多了一档：
//  · 影子抬进头 → 它是新的（第一次可见，必须重新登记生产者）
//  · 直接推进空头 → 它是新的
//  · 推进影子 / 头原地不动 → 不是新的
            payload_fresh_out_0 <= (accept_in_0 & payload_valid_out_0 & skid_valid_0)
                                 | (lane0_push & ~lane0_head_after_pop);
            payload_fresh_out_1 <= (accept_in_1 & payload_valid_out_1 & skid_valid_1)
                                 | (lane1_push & ~lane1_head_after_pop);
//★ 冲刷清放【最后】：冲刷拍弹出来的那两条是错路的，必须被抹掉（后写者赢）。
//  原来它是 else-if 链的最后一档，装载会赢过它 ⇒ `go` 一旦不吃 flush_window 就会漏进错路指令。
//★ 2026-11-08：整窗杀 ⇒ 冲刷拍两条一起清（头死了影子必死，skid 那一块单独清）。
            if (flush_pend) begin
                payload_valid_out_0  <= 1'b0;
                payload_fresh_out_0  <= 1'b0;
                payload_valid_out_1  <= 1'b0;
                payload_fresh_out_1  <= 1'b0;
            end
        end
    end

    always @(*) begin
        lsu_head = lsu_read_pointer == 3'd0 ? lsu_queue_0 :
                   lsu_read_pointer == 3'd1 ? lsu_queue_1 :
                   lsu_read_pointer == 3'd2 ? lsu_queue_2 :
                   lsu_read_pointer == 3'd3 ? lsu_queue_3 :
                   lsu_read_pointer == 3'd4 ? lsu_queue_4 :
                   lsu_read_pointer == 3'd5 ? lsu_queue_5 :
                   lsu_read_pointer == 3'd6 ? lsu_queue_6 : lsu_queue_7;
        lsu_head_instruction = lsu_head[81:50];
        lsu_head_pc          = lsu_head[49:18];
        lsu_head_index       = lsu_head[17:13];
        lsu_head_generation  = lsu_head[12];
    end
    assign lsu_instruction_out          = lsu_head_instruction;
    assign lsu_auxiliary_address_out    = lsu_head_pc;
    assign lsu_immediate_out            = lsu_head_instruction[6:0] == OPCODE_STORE
                                        ? immS(lsu_head_instruction) : immI(lsu_head_instruction);
    assign lsu_source_a_out             = lsu_head_instruction[19:15];
    assign lsu_source_b_out             = lsu_head_instruction[24:20];
    assign lsu_destination_register_out = (lsu_head_instruction[6:0] == OPCODE_STORE) ? 5'd0
                                                                  : lsu_head_instruction[11:7];
    assign lsu_load_store_function3_out = lsu_head_instruction[14:12];
    assign lsu_is_store_out             = (lsu_head_instruction[6:0] == OPCODE_STORE);
    assign lsu_write_enable_out         = (lsu_head_instruction[6:0] == OPCODE_LOAD)
                                        & (lsu_head_instruction[11:7] != 5'd0);
    assign lsu_index_out                = lsu_head_index;
    assign lsu_generation_out           = lsu_head_generation;
    assign lsu_valid_out                = (lsu_queue_count != 4'd0);
//★ 2026-11-04 真 PRF：条目 [11:6] = rs1 的物理号，[5:0] = rs2 的物理号（与原 tag 同宽）
//★ 2026-11-06：最上面 6 位 [97:92] = 目的物理号，给 isq 的提前唤醒用
    assign lsu_phys_d_out                = lsu_head[97:92];
    assign lsu_phys_a_out                = lsu_head[11:6];
    assign lsu_phys_b_out                = lsu_head[5:0];

    assign lsu_push_source_a = source_a_dec_0;
    assign lsu_push_source_b = source_b_dec_0;
//★ push1 = "lane1 这一拍收的那一条"，不一定是 h1：alloc_lane0=0 时 lane1 拿的是 h0。
//  这一格必须跟着走，否则"h0 被 lane1 收"那条的源就查错了。
    assign push1_source_a    = alloc_lane0 ? source_a_dec_1 : source_a_dec_0;
    assign push1_source_b    = alloc_lane0 ? source_b_dec_1 : source_b_dec_0;

//★ 2026-11-08：整窗杀 ⇒ 冲刷后队里一条都不留（原来那 8 项查表的"保留前缀"整块删掉）。
//  清空 = 计数归 0、写指针跟着读指针走，读指针本身不动 —— 直接在时序块里做，不再要中间量。

//★ 2026-11-04 真 PRF：条目里不再有"生产者 tag"要按提交清 —— 换成物理号之后，
//  值在写回那一拍就进了 PRF，谁提交都不改它。这里只剩【进队覆盖】一条路径。
    always @(posedge clk) begin
        if (reset_registered) begin
            lsu_queue_0 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_1 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_2 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_3 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_4 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_5 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_6 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_7 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_count <= 4'd0;
            lsu_read_pointer <= 3'd0;
            lsu_write_pointer <= 3'd0;
        end
        else if (flush_pend) begin
//★ 清用【寄存版】冲刷（组合版会把 csr 中断锥拉进本组寄存器）。数据格不动。
            lsu_queue_count   <= 4'd0;
            lsu_write_pointer <= lsu_read_pointer;
        end
        else begin
            lsu_clear_0 = lsu_queue_0;
            lsu_clear_1 = lsu_queue_1;
            lsu_clear_2 = lsu_queue_2;
            lsu_clear_3 = lsu_queue_3;
            lsu_clear_4 = lsu_queue_4;
            lsu_clear_5 = lsu_queue_5;
            lsu_clear_6 = lsu_queue_6;
            lsu_clear_7 = lsu_queue_7;
//★ 2026-11-09：三个写口（h0 / h1 / h2 各一条），落点按【前面进了几条】压紧：
//  一条都不许跳格 —— FIFO 的读侧是按 wp/rp 顺序取的，留洞就把别人的条目顶掉。
            lsu_slot_1 = lsu_write_pointer + {2'd0, lsu_push_0};
            lsu_slot_2 = lsu_write_pointer + {2'd0, lsu_push_0} + {2'd0, lsu_push_1};
            if (lsu_push_0) begin
                case (lsu_write_pointer)
                    3'd0: lsu_clear_0 = lsu_entry_0;
                    3'd1: lsu_clear_1 = lsu_entry_0;
                    3'd2: lsu_clear_2 = lsu_entry_0;
                    3'd3: lsu_clear_3 = lsu_entry_0;
                    3'd4: lsu_clear_4 = lsu_entry_0;
                    3'd5: lsu_clear_5 = lsu_entry_0;
                    3'd6: lsu_clear_6 = lsu_entry_0;
                    default: lsu_clear_7 = lsu_entry_0;
                endcase
            end
            if (lsu_push_1) begin
                case (lsu_slot_1)
                    3'd0: lsu_clear_0 = lsu_entry_1;
                    3'd1: lsu_clear_1 = lsu_entry_1;
                    3'd2: lsu_clear_2 = lsu_entry_1;
                    3'd3: lsu_clear_3 = lsu_entry_1;
                    3'd4: lsu_clear_4 = lsu_entry_1;
                    3'd5: lsu_clear_5 = lsu_entry_1;
                    3'd6: lsu_clear_6 = lsu_entry_1;
                    default: lsu_clear_7 = lsu_entry_1;
                endcase
            end
            if (lsu_push_2) begin
                case (lsu_slot_2)
                    3'd0: lsu_clear_0 = lsu_entry_2;
                    3'd1: lsu_clear_1 = lsu_entry_2;
                    3'd2: lsu_clear_2 = lsu_entry_2;
                    3'd3: lsu_clear_3 = lsu_entry_2;
                    3'd4: lsu_clear_4 = lsu_entry_2;
                    3'd5: lsu_clear_5 = lsu_entry_2;
                    3'd6: lsu_clear_6 = lsu_entry_2;
                    default: lsu_clear_7 = lsu_entry_2;
                endcase
            end
            lsu_queue_0 <= lsu_clear_0;
            lsu_queue_1 <= lsu_clear_1;
            lsu_queue_2 <= lsu_clear_2;
            lsu_queue_3 <= lsu_clear_3;
            lsu_queue_4 <= lsu_clear_4;
            lsu_queue_5 <= lsu_clear_5;
            lsu_queue_6 <= lsu_clear_6;
            lsu_queue_7 <= lsu_clear_7;
            if (lsu_push_0 | lsu_push_1 | lsu_push_2)
                lsu_write_pointer <= lsu_write_pointer + lsu_push_0 + lsu_push_1 + lsu_push_2;
            if (lsu_pop_in & (lsu_queue_count != 4'd0))
                lsu_read_pointer <= lsu_read_pointer + 3'd1;
//★ 推进量是【两个口的和】，不是 {push_0, push_1} 那个拼接 —— 拼出来的是 2*push_0+push_1，
//  push_0 单独成立时会当成进了两格（实测：写指针 +2、队长 +2，队头指向没写过的格）。
            lsu_queue_count <= lsu_queue_count
                             + lsu_push_0 + lsu_push_1 + lsu_push_2
                             - (lsu_pop_in & (lsu_queue_count != 4'd0));
        end
    end

//==============================================================================
// lane0 载荷 = 2 深 FIFO（头 + 影子）
//   一次算下一态，次序固定为【先出队，再入队】：
//     1. 出队：accept 且头里有货 → 影子抬进头（影子空就只是清头）
//     2. 入队：lane0_push 且出队后头是空的 → 进头；否则 → 进影子
//   能不能收已在 alloc 处按本地空位判过（lane0_free = ~头 | ~影子），这里不再看 accept。
//★ 这个块刻意【不吃 exec】：accept 可以在 exec=0 的拍成立（Q 空时 isq 照样还 credit），
//  门掉就等于把影子按在原地、跟头上那条换了序。推进本身已被 go(含 exec) 挡过，不用再挡一次。
//★ 头寄存器仍是原来那 27 个，下游 isq 一个字不改；多出来的只有影子这一组。
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            instruction_out_0 <= 32'd0;
            auxiliary_address_out_0 <= 32'd0;
            branch_prediction_taken_out_0 <= 1'b0;
            source_a_out_0 <= 5'd0;
            source_b_out_0 <= 5'd0;
            index_out_0 <= 4'd0;
            generation_out_0 <= 1'b0;
            phys_a_out_0 <= 6'd0;
            phys_d_out_0 <= 6'd0;
            phys_b_out_0 <= 6'd0;
            dec_class_0 <= 2'd0;
            dec_we_0 <= 1'b0;
            dec_rd_0 <= 5'd0;
            dec_is_st_0 <= 1'b0;
            dec_mulop_0 <= 3'd0;
            dec_lsf3_0 <= 3'd0;
            dec_func4_0 <= 4'd0;
            dec_csrf3_0 <= 3'd0;
            dec_csrw_en_0 <= 1'b0;
            dec_csrw_wr_0 <= 1'b0;
            dec_csra_0 <= 12'd0;
            dec_jval_0 <= 32'd0;
            dec_brf_0 <= 1'b0;
            dec_jalf_0 <= 1'b0;
            dec_jalfr_0 <= 1'b0;
            dec_bjuf4_0 <= 4'd0;
            dec_r1_imm_0 <= 1'b0;
            dec_r2_imm_0 <= 1'b0;
            dec_r1_immv_0 <= 32'd0;
            dec_r2_immv_0 <= 32'd0;
            skid_valid_0 <= 1'b0;
        end
        else begin
//---- 1) 出队：影子抬进头（影子空 ⇒ 什么都不写，有效位由旁边那一块清）----
            if (accept_in_0 & payload_valid_out_0 & skid_valid_0) begin
                instruction_out_0 <= skid_instruction_out_0;
                auxiliary_address_out_0 <= skid_auxiliary_address_out_0;
                branch_prediction_taken_out_0 <= skid_branch_prediction_taken_out_0;
                source_a_out_0 <= skid_source_a_out_0;
                source_b_out_0 <= skid_source_b_out_0;
                index_out_0 <= skid_index_out_0;
                generation_out_0 <= skid_generation_out_0;
                phys_a_out_0 <= skid_phys_a_out_0;
                phys_d_out_0 <= skid_phys_d_out_0;
                phys_b_out_0 <= skid_phys_b_out_0;
                dec_class_0 <= skid_dec_class_0;
                dec_we_0 <= skid_dec_we_0;
                dec_rd_0 <= skid_dec_rd_0;
                dec_is_st_0 <= skid_dec_is_st_0;
                dec_mulop_0 <= skid_dec_mulop_0;
                dec_lsf3_0 <= skid_dec_lsf3_0;
                dec_func4_0 <= skid_dec_func4_0;
                dec_csrf3_0 <= skid_dec_csrf3_0;
                dec_csrw_en_0 <= skid_dec_csrw_en_0;
                dec_csrw_wr_0 <= skid_dec_csrw_wr_0;
                dec_csra_0 <= skid_dec_csra_0;
                dec_jval_0 <= skid_dec_jval_0;
                dec_brf_0 <= skid_dec_brf_0;
                dec_jalf_0 <= skid_dec_jalf_0;
                dec_jalfr_0 <= skid_dec_jalfr_0;
                dec_bjuf4_0 <= skid_dec_bjuf4_0;
                dec_r1_imm_0 <= skid_dec_r1_imm_0;
                dec_r2_imm_0 <= skid_dec_r2_imm_0;
                dec_r1_immv_0 <= skid_dec_r1_immv_0;
                dec_r2_immv_0 <= skid_dec_r2_immv_0;
            end
//---- 2) 入队到头：出队后头是空的 ----
            else if (lane0_push & ~lane0_head_after_pop) begin
                instruction_out_0 <= instruction_in_0;
                auxiliary_address_out_0 <= address_in_0;
                branch_prediction_taken_out_0 <= branch_prediction_in_0;
                source_a_out_0 <= source_a_dec_0;
                source_b_out_0 <= source_b_dec_0;
                index_out_0 <= index_in_0;
                generation_out_0 <= generation_in_0;
                phys_a_out_0 <= lookup_a_phys;
                phys_d_out_0 <= write_phys_decoded_0;
                phys_b_out_0 <= lookup_b_phys;
                dec_class_0 <= class_decoded;
                dec_we_0 <= write_enable_decoded;
                dec_rd_0 <= destination_pick_decoded;
                dec_is_st_0 <= is_store_decoded;
                dec_mulop_0 <= multiply_operation_decoded;
                dec_lsf3_0 <= load_store_function3_decoded;
                dec_func4_0 <= function4_decoded;
                dec_csrf3_0 <= csr_function3_decoded;
                dec_csrw_en_0 <= csr_write_enable_decoded;
                dec_csrw_wr_0 <= csr_write_commit_decoded;
                dec_csra_0 <= csr_address_decoded;
                dec_jval_0 <= jump_value_decoded;
                dec_brf_0 <= branch_flag_decoded;
                dec_jalf_0 <= jump_link_flag_decoded;
                dec_jalfr_0 <= jump_link_register_flag_decoded;
                dec_bjuf4_0 <= branch_jump_function4_decoded;
                dec_r1_imm_0 <= source_a_immediate_decoded;
                dec_r2_imm_0 <= source_b_immediate_decoded;
                dec_r1_immv_0 <= source_a_immediate_value_decoded;
                dec_r2_immv_0 <= source_b_immediate_value_decoded;
            end
//---- 头原地不动：生产者可能就在这一拍提交 ⇒ 清掉它的 tag ----
            else begin
            end
//---- 3) 入队到影子：出队后头仍占着（此时 lane0_free 已保证影子是空的）----
            if (lane0_push & lane0_head_after_pop) begin
                skid_instruction_out_0 <= instruction_in_0;
                skid_auxiliary_address_out_0 <= address_in_0;
                skid_branch_prediction_taken_out_0 <= branch_prediction_in_0;
                skid_source_a_out_0 <= source_a_dec_0;
                skid_source_b_out_0 <= source_b_dec_0;
                skid_index_out_0 <= index_in_0;
                skid_generation_out_0 <= generation_in_0;
                skid_phys_a_out_0 <= lookup_a_phys;
                skid_phys_d_out_0 <= write_phys_decoded_0;
                skid_phys_b_out_0 <= lookup_b_phys;
                skid_dec_class_0 <= class_decoded;
                skid_dec_we_0 <= write_enable_decoded;
                skid_dec_rd_0 <= destination_pick_decoded;
                skid_dec_is_st_0 <= is_store_decoded;
                skid_dec_mulop_0 <= multiply_operation_decoded;
                skid_dec_lsf3_0 <= load_store_function3_decoded;
                skid_dec_func4_0 <= function4_decoded;
                skid_dec_csrf3_0 <= csr_function3_decoded;
                skid_dec_csrw_en_0 <= csr_write_enable_decoded;
                skid_dec_csrw_wr_0 <= csr_write_commit_decoded;
                skid_dec_csra_0 <= csr_address_decoded;
                skid_dec_jval_0 <= jump_value_decoded;
                skid_dec_brf_0 <= branch_flag_decoded;
                skid_dec_jalf_0 <= jump_link_flag_decoded;
                skid_dec_jalfr_0 <= jump_link_register_flag_decoded;
                skid_dec_bjuf4_0 <= branch_jump_function4_decoded;
                skid_dec_r1_imm_0 <= source_a_immediate_decoded;
                skid_dec_r2_imm_0 <= source_b_immediate_decoded;
                skid_dec_r1_immv_0 <= source_a_immediate_value_decoded;
                skid_dec_r2_immv_0 <= source_b_immediate_value_decoded;
                skid_valid_0 <= 1'b1;
            end
            else if (accept_in_0 & payload_valid_out_0) begin
                skid_valid_0 <= 1'b0;
            end
            else begin
//第二格原地不动：同样要按提交清 tag（它比队头更晚进 isq，窗口更长）
            end
//★ 冲刷清与原来同门（装载赢）：有效位在相邻那一块已经清掉，影子这里只要清 skid_valid ——
//  数据留着没人读（头有效位=0 时下游一个字都不看，与原来留脏头的做法一致）。
//★ 精确冲刷：头死了（必连带影子）或影子自己被判死才清；头活着、影子活着就留着。
            if (flush_pend & ~lane0_store)
                skid_valid_0 <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            instruction_out_1 <= 32'd0;
            auxiliary_address_out_1 <= 32'd0;
            branch_prediction_taken_out_1 <= 1'b0;
            source_a_out_1 <= 5'd0;
            source_b_out_1 <= 5'd0;
            index_out_1 <= 4'd0;
            generation_out_1 <= 1'b0;
            phys_a_out_1 <= 6'd0;
            phys_d_out_1 <= 6'd0;
            phys_b_out_1 <= 6'd0;
            dec_class_1 <= 2'd0;
            dec_we_1 <= 1'b0;
            dec_rd_1 <= 5'd0;
            dec_is_st_1 <= 1'b0;
            dec_mulop_1 <= 3'd0;
            dec_lsf3_1 <= 3'd0;
            dec_func4_1 <= 4'd0;
            dec_csrf3_1 <= 3'd0;
            dec_csrw_en_1 <= 1'b0;
            dec_csrw_wr_1 <= 1'b0;
            dec_csra_1 <= 12'd0;
            dec_jval_1 <= 32'd0;
            dec_brf_1 <= 1'b0;
            dec_jalf_1 <= 1'b0;
            dec_jalfr_1 <= 1'b0;
            dec_bjuf4_1 <= 4'd0;
            dec_r1_imm_1 <= 1'b0;
            dec_r2_imm_1 <= 1'b0;
            dec_r1_immv_1 <= 32'd0;
            dec_r2_immv_1 <= 32'd0;
            skid_valid_1 <= 1'b0;
        end
        else begin
//---- 1) 出队：影子抬进头 ----
            if (accept_in_1 & payload_valid_out_1 & skid_valid_1) begin
                instruction_out_1 <= skid_instruction_out_1;
                auxiliary_address_out_1 <= skid_auxiliary_address_out_1;
                branch_prediction_taken_out_1 <= skid_branch_prediction_taken_out_1;
                source_a_out_1 <= skid_source_a_out_1;
                source_b_out_1 <= skid_source_b_out_1;
                index_out_1 <= skid_index_out_1;
                generation_out_1 <= skid_generation_out_1;
                phys_a_out_1 <= skid_phys_a_out_1;
                phys_d_out_1 <= skid_phys_d_out_1;
                phys_b_out_1 <= skid_phys_b_out_1;
                dec_class_1 <= skid_dec_class_1;
                dec_we_1 <= skid_dec_we_1;
                dec_rd_1 <= skid_dec_rd_1;
                dec_is_st_1 <= skid_dec_is_st_1;
                dec_mulop_1 <= skid_dec_mulop_1;
                dec_lsf3_1 <= skid_dec_lsf3_1;
                dec_func4_1 <= skid_dec_func4_1;
                dec_csrf3_1 <= skid_dec_csrf3_1;
                dec_csrw_en_1 <= skid_dec_csrw_en_1;
                dec_csrw_wr_1 <= skid_dec_csrw_wr_1;
                dec_csra_1 <= skid_dec_csra_1;
                dec_jval_1 <= skid_dec_jval_1;
                dec_brf_1 <= skid_dec_brf_1;
                dec_jalf_1 <= skid_dec_jalf_1;
                dec_jalfr_1 <= skid_dec_jalfr_1;
                dec_bjuf4_1 <= skid_dec_bjuf4_1;
                dec_r1_imm_1 <= skid_dec_r1_imm_1;
                dec_r2_imm_1 <= skid_dec_r2_imm_1;
                dec_r1_immv_1 <= skid_dec_r1_immv_1;
                dec_r2_immv_1 <= skid_dec_r2_immv_1;
            end
//---- 2) 入队到头：出队后头是空的 ----
            else if (lane1_push & ~lane1_head_after_pop) begin
                instruction_out_1 <= instruction_stage1;
                auxiliary_address_out_1 <= address_stage1;
                branch_prediction_taken_out_1 <= branch_prediction_stage1;
                source_a_out_1 <= source_a_stage1;
                source_b_out_1 <= source_b_stage1;
                index_out_1 <= index_stage1;
                generation_out_1 <= generation_stage1;
//★ lane1 的 tag 恒取自 push1 那一组（它已经按实际拿到的格子偏好了）。
                phys_a_out_1 <= pick_from_h2 ? lookup_e_phys : lookup_c_phys;
                phys_d_out_1 <= pick_from_h2 ? (((write_enable_decoded_1 || (class_decoded_1 == CLS_MUL)) & (destination_decoded_1 != 5'd0)) ? rename_phys_rd_3 : 6'd0) : write_phys_decoded_1;
                phys_b_out_1 <= pick_from_h2 ? lookup_f_phys : lookup_d_phys;
                dec_class_1 <= class_decoded_1;
                dec_we_1 <= write_enable_decoded_1;
                dec_rd_1 <= destination_pick_decoded_1;
                dec_is_st_1 <= is_store_decoded_1;
                dec_mulop_1 <= multiply_operation_decoded_1;
                dec_lsf3_1 <= load_store_function3_decoded_1;
                dec_func4_1 <= function4_decoded_1;
                dec_csrf3_1 <= csr_function3_decoded_1;
                dec_csrw_en_1 <= csr_write_enable_decoded_1;
                dec_csrw_wr_1 <= csr_write_commit_decoded_1;
                dec_csra_1 <= csr_address_decoded_1;
                dec_jval_1 <= jump_value_decoded_1;
                dec_brf_1 <= branch_flag_decoded_1;
                dec_jalf_1 <= jump_link_flag_decoded_1;
                dec_jalfr_1 <= jump_link_register_flag_decoded_1;
                dec_bjuf4_1 <= branch_jump_function4_decoded_1;
                dec_r1_imm_1 <= source_a_immediate_decoded_1;
                dec_r2_imm_1 <= source_b_immediate_decoded_1;
                dec_r1_immv_1 <= source_a_immediate_value_decoded_1;
                dec_r2_immv_1 <= source_b_immediate_value_decoded_1;
            end
//---- 头原地不动：生产者可能就在这一拍提交 ⇒ 清掉它的 tag ----
            else begin
            end
//---- 3) 入队到影子：出队后头仍占着（此时 lane1_free 已保证影子是空的）----
            if (lane1_push & lane1_head_after_pop) begin
                skid_instruction_out_1 <= instruction_stage1;
                skid_auxiliary_address_out_1 <= address_stage1;
                skid_branch_prediction_taken_out_1 <= branch_prediction_stage1;
                skid_source_a_out_1 <= source_a_stage1;
                skid_source_b_out_1 <= source_b_stage1;
                skid_index_out_1 <= index_stage1;
                skid_generation_out_1 <= generation_stage1;
                skid_phys_a_out_1 <= pick_from_h2 ? lookup_e_phys : lookup_c_phys;
                skid_phys_d_out_1 <= pick_from_h2 ? (((write_enable_decoded_1 || (class_decoded_1 == CLS_MUL)) & (destination_decoded_1 != 5'd0)) ? rename_phys_rd_3 : 6'd0) : write_phys_decoded_1;
                skid_phys_b_out_1 <= pick_from_h2 ? lookup_f_phys : lookup_d_phys;
                skid_dec_class_1 <= class_decoded_1;
                skid_dec_we_1 <= write_enable_decoded_1;
                skid_dec_rd_1 <= destination_pick_decoded_1;
                skid_dec_is_st_1 <= is_store_decoded_1;
                skid_dec_mulop_1 <= multiply_operation_decoded_1;
                skid_dec_lsf3_1 <= load_store_function3_decoded_1;
                skid_dec_func4_1 <= function4_decoded_1;
                skid_dec_csrf3_1 <= csr_function3_decoded_1;
                skid_dec_csrw_en_1 <= csr_write_enable_decoded_1;
                skid_dec_csrw_wr_1 <= csr_write_commit_decoded_1;
                skid_dec_csra_1 <= csr_address_decoded_1;
                skid_dec_jval_1 <= jump_value_decoded_1;
                skid_dec_brf_1 <= branch_flag_decoded_1;
                skid_dec_jalf_1 <= jump_link_flag_decoded_1;
                skid_dec_jalfr_1 <= jump_link_register_flag_decoded_1;
                skid_dec_bjuf4_1 <= branch_jump_function4_decoded_1;
                skid_dec_r1_imm_1 <= source_a_immediate_decoded_1;
                skid_dec_r2_imm_1 <= source_b_immediate_decoded_1;
                skid_dec_r1_immv_1 <= source_a_immediate_value_decoded_1;
                skid_dec_r2_immv_1 <= source_b_immediate_value_decoded_1;
                skid_valid_1 <= 1'b1;
            end
            else if (accept_in_1 & payload_valid_out_1) begin
                skid_valid_1 <= 1'b0;
            end
            else begin
            end
//★ 同 lane0：冲刷清与原来同门（装载赢），有效位在相邻那一块已清。
            if (flush_pend & ~lane1_store)
                skid_valid_1 <= 1'b0;
        end
    end

    rename u_rename (
        .clk(clk), .rst(rst),
        .alloc_en_0(alloc_lane0 & ~rob_full_in), .alloc_we_0(destination_decoded != 5'd0), .alloc_rd_0(destination_decoded),
        .alloc_rs1_0(source_a_dec_0), .alloc_rs2_0(source_b_dec_0),
        .phys_rd_0(rename_phys_rd_0), .phys_rs1_0(rename_phys_rs1_0), .phys_rs2_0(rename_phys_rs2_0),
//★ 口 1 = 第二格（h1）：它是访存时用它的访存字段，否则用 pick 的译码（此时 pick 就是 h1）。
        .alloc_en_1((alloc_lane0 ? (h1_is_mem ? lsu_push_1 : alloc_lane1) : alloc_lane1) & ~rob_full_in),
        .alloc_we_1(pick_mem ? (~h1_store & (instruction_in_1[11:7] != 5'd0)) : (destination_decoded_1 != 5'd0)),
        .alloc_rd_1(pick_mem ? instruction_in_1[11:7] : destination_decoded_1),
        .alloc_rs1_1(pick_mem ? instruction_in_1[19:15] : source_a_dec_1),
        .alloc_rs2_1(pick_mem ? (h1_store ? instruction_in_1[24:20] : 5'd0) : source_b_dec_1),
        .phys_rd_1(rename_phys_rd_1), .phys_rs1_1(rename_phys_rs1_1), .phys_rs2_1(rename_phys_rs2_1),
        .phys_rd_2(rename_phys_rd_2), .phys_rd_3(rename_phys_rd_3),
        .alloc_stall_0(rename_stall_0), .alloc_stall_1(rename_stall_1),
        .alloc_stall_2(rename_stall_2), .alloc_stall_3(rename_stall_3),
        .bank_empty_0(rename_bank_empty_0), .bank_empty_1(rename_bank_empty_1),
        .bank_empty_2(rename_bank_empty_2), .bank_empty_3(rename_bank_empty_3),
//★ 口 2 = h2 走 LSU（偶数 bank）；口 3 = h2 走 lane1（奇数 bank）。两者互斥。
        .alloc_en_2(lsu_push_2 & ~rob_full_in), .alloc_we_2(h2_rd != 5'd0), .alloc_rd_2(h2_rd),
        .alloc_rs1_2(h2_rs1), .alloc_rs2_2(h2_rs2),
        .alloc_en_3((alloc_lane0 & h1_is_mem & alloc_lane1) & ~rob_full_in),
        .alloc_we_3(destination_decoded_1 != 5'd0), .alloc_rd_3(destination_decoded_1),
        .alloc_rs1_3(source_a_dec_1), .alloc_rs2_3(source_b_dec_1),
        .lookup_a(lsu_push_source_a), .lookup_b(lsu_push_source_b),
        .lookup_c(pick_mem ? instruction_in_1[19:15] : push1_source_a),
        .lookup_d(pick_mem ? (h1_store ? instruction_in_1[24:20] : 5'd0) : push1_source_b),
        .lookup_e(lsu_push_2 ? h2_rs1 : source_a_dec_1),
        .lookup_f(lsu_push_2 ? h2_rs2 : source_b_dec_1),
        .lookup_a_phys(lookup_a_phys), .lookup_b_phys(lookup_b_phys),
        .lookup_c_phys(lookup_c_phys), .lookup_d_phys(lookup_d_phys),
        .lookup_e_phys(lookup_e_phys), .lookup_f_phys(lookup_f_phys),
        .commit_we_a(write_enable_a_in), .commit_rd_a(commit_rd0_in), .commit_phys_a(commit_phys0_in),
        .commit_we_b(write_enable_b_in), .commit_rd_b(commit_rd1_in), .commit_phys_b(commit_phys1_in),
        .flush(exc_kill | irq_kill | redir_frontend | redir_backend_0),
        .restore(restore_in),
        .alloc_onehot(rename_alloc_onehot), .arch_next_onehot(rename_arch_next_onehot)
    );

    assign alloc_phys_0 = rename_phys_rd_0;
    assign alloc_onehot_0 = rename_alloc_onehot;
    assign arch_next_onehot_0 = rename_arch_next_onehot;
    assign alloc_phys_1 = rename_phys_rd_1;

//★ 2026-11-10：owner = "这条指令的结果将来从哪个 PRF 写口落笔"。
//  口 0 / 口 2 恒走 lane0 / lane1（访存不占这两个口，它从 LSU 那条路写），
//  所以只需分 ALU 早口 / 结果口（MUL、DIV）两种；口 1 / 口 2 里凡是访存就是 LOAD 口。
//  jal / jalr / CSR 的写回走 ALU，class 不是 CLS_MUL ⇒ 落到 ALU 早口，正确。
//  ★ 访存必须按【类别】判，不能按 pick_mem：pick_mem = alloc_lane0 & h1_is_mem，
//    而 alloc_lane0 为 0 时 h1 的访存指令走口 1 的 alloc_lane1 分支，那时 pick_mem 是 0。
//    只按 pick_mem 判会让那条 load 的 owner 落成 ALU 的 0/2，写进 array_aluN 而读口去 ALU 阵列找 ⇒ 读到 0。
    assign alloc_owner_0 = (class_decoded == CLS_MUL) ? 3'd1 :
                           (class_decoded == CLS_LSU) ? 3'd4 : 3'd0;
    assign alloc_owner_1 = (pick_mem | (class_decoded_1 == CLS_LSU)) ? 3'd4 :
                           ((class_decoded_1 == CLS_MUL) ? 3'd3 : 3'd2);
//  上面三条必须跟 alloc_phys_0/1/2 的取值条件逐条对齐（同一拍、同一个口），否则 owner 和 phys 会错配。
//★ 2026-11-10：这里原来是【两条】assign alloc_owner_2（多驱动）。Verilator 按后写者赢跑得动，
//  Vivado 的 DRC 直接报 MDRV-1、opt_design 不跑 —— 远端时序流就是这么挂掉的。只留下面这一条。
    assign alloc_owner_2 = lsu_push_2 ? 3'd4 : ((class_decoded_1 == CLS_MUL) ? 3'd3 : 3'd2);

//==============================================================================
//  CSR 随项载荷（2026-11-04，CSR 步骤 C：读-改-写搬出执行级）
//  ★ 为什么放在文件末尾：lookup_*_phys 的声明在 rename 例化旁边（本文件一贯的声明顺序），
//    写在前面会变成"先使用后声明"（iverilog 直接当错误）。
//  ★ 源字段两种形态：csrrwi/rsrsi/rrci 的 rs1 字段是 zimm，idu 已经按立即数解释
//    （source_a_immediate_*），这里就填 zimm；其余填 rs1 的物理号。
//    提交口用 funct3[2] 二选一 —— 1xx 恰好就是立即数形式。
//  ★ 寄存器形式下 rs1=x0 ⇔ 物理号 0（0 号永不写入）⇒ 提交口可以用"源字段 != 0"重算
//    csr_wr_dec 那个"源非 0 才写"的判据，不必再单独带一根。
//==============================================================================
    always @(*) begin
        alloc_csr_0 = csr_write_enable_decoded;
        alloc_csr_1 = pick_mem ? 1'b0 : csr_write_enable_decoded_1;
        alloc_csr_2 = lsu_push_2 ? 1'b0 : csr_write_enable_decoded_1;
    end

endmodule
