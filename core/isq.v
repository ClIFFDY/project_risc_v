`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: isq —— 发射级：每 lane 一个指针式保留站
//
//   结构：两拍，中间只有一个出站寄存器
//     idu S 级 lane0 ──► RS0(8 项) ──t0 就绪+选优──► Q0 ──t1 直接发──► 单元口 0
//     idu S 级 lane1 ──► RS1(8 项) ──t0 就绪+选优──► Q1 ──t1 直接发──► 单元口 1
//     访存条不进 lane 站：iffu 队头分流 → idu 的 LSU FIFO → 本模块的访存站（4 项，2026-11-09 起）→ lsu
//       ★ 访存站允许 load 之间乱序发射（取最老可发项），store 只能当队头发。
//         两条顺序铁律与论证见 docs/SPEC_mem_ooo.md §2。
//
//   ★ 为什么按 lane 分家，不做一个共用的池子：
//     PRF 是按 bank 分阵列的 —— bank = 物理号最低位，rename 让 lane0 只拿偶数号、lane1 只拿奇数号，
//     写口 0 只能写偶数那一块、写口 1 只能写奇数那一块。
//     共用一个池子就必须按 bank 再分一次才能发，等于什么都没买到，还多一层选优和一位 bank 载荷。
//     分家之后 lane 归属在【分配那一拍】就定死了（idu 的 alloc_lane0/1），站里不用再存。
//
//   ★ 指针式：条目永不搬家，只搬 3 位槽号（RS_AW；每 lane 8 项）。
//     在册项恒占一段连续窗口 [head, tail)；中途发射掉的槽留空、不复用
//     ⇒「从 head 数起的槽序」= 入队序 = 程序序 ⇒ 年龄比较退化成一次旋转 + 优先编码。
//     本地年龄系统全在本模块内，ROB 的 head_pointer 不进这里（CSR 那条纪律除外）。
//     ★ 2026-11-10：从压缩式换回（压缩式的载荷位移网络是全核 TNS 主力）。
//
//   ★ 选优锥的终点是 Q 的寄存器，不接 ALU、不接 PRF 读口。
//     t0 = 就绪判定 + 选优 + 载荷落 Q；t1 = 那条指令直接发往执行级并出 PRF 读地址。
//     ★ 唤醒分两组，按"消费者 t1 从哪儿取到值"定时刻：
//       A · 发射拍（ALU）：ALU 的早出口在发射后一拍有效，正好是消费者 t1 那一拍，
//           走出口 mux 的前送档 ⇒ 比写口早一整拍叫。
//       B · 写口落笔（ALU / MUL / DIV / LOAD）：消费者 t1 从寄存器堆取值；写口这一拍 =
//           下一拍一定读得到。A 组那一叫被 Q 占住没接住时，B 组这一拍兜底。
//       ★ 踩过的坑：拿 LOAD 早写口（ld_fwd_ok）当就绪 —— 它只说"这一拍总线上有这个号"，
//         下一拍既不在总线上、阵列里也可能没写（被同拍晚口门掉，prf 的 wp3_active）
//         ⇒ CoreMark 12k 拍消费者读到旧值，load 拿 0 当地址，外设 load 永不回。
//       ★ ALU 从结果口仲裁里拆出来单独写值表（值表是触发器阵列，不受"每阵列 ≤2 写口"
//         约束）之后，ALU 的落笔时刻也确定了。
//
//   ★ 实测账（2026-11-07，同一棵树 A/B）：
//     只有 B 组那版：CoreMark/MHz 3.469、IPC 0.999；双槽口旧版 3.825 / 1.102。
//     加上 A 组（ALU 发射拍唤醒）：3.689 / 1.063，rand 200/200、CoreMark CRC 全对。
//     站加深到每 lane 16 项只把"站满"从 72109 拍降到 136 拍，性能不变（3.477）
//     ⇒ 剩下那段差距主要是每条 ALU 依赖链多等的那一拍，站容量不是原因。
//
//   ★ 操作数就绪比的是【物理号】：每项常驻 ready_q | 六条唤醒总线
//     （ALU 发射拍 2 / 结果口 2 / LOAD 早·晚口 2）。prf 里已经没有就绪位了。
//     三条前送（ALU/LOAD 的早口）【不进就绪】，只留在出口的操作数 mux 里 ——
//     它们比阵列里那一笔早，load 早写被同拍晚写门掉时会撒谎。
//
//   ★ 推进：两条 lane 各自独立。站里还有空位，idu 那一拍就可以装下一条；
//     站满由 station_room_*_valid 回压 idu 的分配门（访存条走 lsu_room，不受这条影响）。
//////////////////////////////////////////////////////////////////////////////////

module isq(
    input clk, rst,
//★ 只接本模块真正用到的位。
    input        redir_backend_0,   // 后端重定向（rob 队头那一拍出的那根）
    input        redir_backend_1,   // 同一笔的寄存版，晚一拍
    input        irq_kill,           // 中断受理（clint 自己判的）
//前端：本 lane 的 D 有指令、且还没被收
    input        stage_advance,
    input        stage_advance_lane1,

//---- D 级 lane0（idu 的 S 级寄存器）----
    input [4:0]  rs1_in, rs2_in,
    input [31:0] aux_addr_in,
    input        branch_prediction_taken_in,
    input [4:0]  index_in,
    input        alloc_generation_in,
    input [1:0]  decoded_class,
    input        decoded_write_enable,
    input [4:0]  decoded_destination_register,
    input [2:0]  decoded_multiply_operation,
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
    input [31:0] aux_addr_in1,
    input        branch_prediction_taken_in_lane1,
    input [4:0]  index_in_lane1,
    input        alloc_generation_in_lane1,
    input [1:0]  decoded_class_lane1,
    input        decoded_write_enable_lane1,
    input [4:0]  decoded_destination_register_lane1,
    input [2:0]  decoded_multiply_operation_lane1,
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
// 访存专用流（idu 的 LSU FIFO）
//   ★ 两个相位分开：pstat 的【登记 + 查 tag】发生在【进队那一拍】，tag 随条目在 FIFO 里等，
//     出队时直接拿来用。
//==============================================================================
    input [4:0]  mem_rs1_in, mem_rs2_in, mem_destination_register_in,
    input [31:0] mem_auxiliary_address_in, mem_immediate_in,
    input [4:0]  mem_index_in,
    input        mem_generation_in,
    input        mem_is_store_in,
    input [2:0]  mem_load_store_function3_in,
    input        mem_valid_in,
    input [5:0]  mem_phys_a_in, mem_phys_b_in,
    output reg   mem_pop_out,

//---- 结果口（寄存值）----
    input [31:0] data_alu_early,
    input [31:0] data_alu_lane1_early,
    input [31:0] data_ld_early,
//---- 前送身份（只认 ALU 与 LOAD；mdu 的结果绝不进旁路网）----
    input        write_enable_alu_early,
    input [5:0] phys_alu_early_0,
    input        write_enable_alu_lane1_early,
    input [5:0] phys_alu_early_1,
    input        write_enable_ld_early,
    input [5:0] ld_phys_early,
//---- 唤醒：结果口 / LOAD 早口这一拍真的把值写进了寄存器堆 ----
//   这是全核唯一保证"下一拍这个号一定读得到"的时刻，发射级的 t1 直接出站读 PRF 就靠它。
    input        result_write_valid_0, input [5:0] result_write_phys_0,
    input        result_write_valid_1, input [5:0] result_write_phys_1,
//★ 2026-11-09：结果口这一拍服务的是不是除法。乘法改走本地的静态延迟链（见八之二点五）。
    input        result_is_divide_0, result_is_divide_1,
    input        load_write,  input [5:0] load_write_phys,
//---- D 级那一格的两个源物理号 ----
    input [5:0]  rs1_phys_in, rs2_phys_in, rs1_phys_in_lane1, rs2_phys_in_lane1,
//---- 目的物理号（载荷里存一份）----
//  A 组唤醒（wake_issue_*）读它：只在这一条真的会写时才有意义，idu 已按 rob 的 entry_phys
//  同口径给 0（非写 / rd=x0）。lsu_phys_d_in 那条路目前没人读：站载荷里不存它。
//  divide_wake 同理没人接：除法唤醒比写口早两拍，t1 去读会读到旧值，端口留着不接逻辑。
    input [5:0]  phys_d_in, phys_d_in_lane1, lsu_phys_d_in,
//  除法：串行除法的源唤醒本来就比写口早一拍，唤醒再往前提一拍 ⇒ 由 mdu 的提交链给号。
    input        divide_wake_valid, input [5:0] divide_wake_phys,
//---- 真 PRF：读口地址出去，数据与就绪回来 ----
    output [5:0] prf_read_a_0_phys, prf_read_b_0_phys,
    output [5:0] prf_read_a_1_phys, prf_read_b_1_phys,
    output [5:0] prf_read_a_m_phys, prf_read_b_m_phys,
    input [31:0] prf_data_a_0, prf_data_b_0, prf_data_a_1, prf_data_b_1, prf_data_a_mem, prf_data_b_mem,
//★ 在飞表的置位源：rename 本拍发出去的物理号（表按【发号】置位，不按入站）
    input [63:0] alloc_onehot_in,

//---- 冲刷 / 陷阱 ----

// CSR 的写口在【提交拍】：上一拍的写正在这一拍落地，本拍放一条 CSR 出去会读到旧值。
    input        csr_write_landing,
//★ CSR 的"必须在 ROB 队头才能发"这条纪律仍要看 ROB 头。它不进本地年龄系统。
    input [31:0] head_one_hot_registered,
//---- 单元就绪 ----
    input        lsu_full,
    output reg   bypass_load_a_0, bypass_load_b_0, bypass_load_a_1, bypass_load_b_1,
    input        mdu_mul_ok_0, mdu_div_ok_0, mdu_mul_ok_1, mdu_div_ok_1,
    input        bju_ok_0, bju_ok_1,
//---- 给 idu 的推进门：本 lane 的站还有空位 ----
    output       station_room_0_valid,
    output       station_room_1_valid,

//---- 出站 lane0 ----
    output reg        issue_valid_0,
    output reg [4:0]  issue_index_0,
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
    output reg        branch_flag_0, jump_link_flag_0, jump_link_register_flag_0, branch_prediction_taken_0,
    output reg [3:0]  bju_function4_0,
    output reg        mul_go_0, div_go_0,
//---- 出站 lane1 ----
    output reg        issue_valid_1,
    output reg [4:0]  issue_index_1,
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
    output reg        branch_flag_1, jump_link_flag_1, jump_link_register_flag_1, branch_prediction_taken_1,
    output reg [3:0]  bju_function4_1,
    output reg        mul_go_1, div_go_1,

//---- 共享 LSU 口（只由访存专用流驱动）----
    output reg        lsu_valid,
    output reg        lsu_lane,
    output reg [6:0]  lsu_opcode,
    output reg [9:0]  lsu_function10,
    output reg [31:0] lsu_imm,
    output reg [4:0]  lsu_index,
    output reg [5:0]  lsu_phys_d,
    output reg        lsu_generation,
    output reg [31:0] lsu_pc,
    output reg [31:0] lsu_a, lsu_b,

//---- 中断边界 / 状态 / credit ----
    output reg        accept_0, accept_1,
    output reg        issue_pop_0, issue_pop_1,
//CSR 载荷（发射那一拍交给 ROB 锁一深）：{写使能 1, 地址 12, funct3 3, 源 6}
    output [21:0]     csr_payload_0, csr_payload_1
    );

//==============================================================================
//  参数与声明区：本模块全部 localparam/parameter + reg/wire。
//  Verilog 要求先声明后用，所以全放这里；正文只留逻辑。
//==============================================================================
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;
//---- 保留站几何：每 lane 4 项 ----
    localparam RS_DEPTH = 8;
    localparam RS_AW    = 3;
    localparam RS_PW    = 4;
//---- 站载荷布局（低位在前）----
    localparam RS_IDX_LO   = 0;     //  5  ROB 槽号
    localparam RS_GEN_LO   = 5;     //  1  ROB 世代
    localparam RS_CLASS_LO = 6;     //  2
    localparam RS_WE_LO    = 8;     //  1
    localparam RS_MULOP_LO = 9;     //  3
    localparam RS_FUNC4_LO = 12;    //  4
    localparam RS_CSRF3_LO = 16;    //  3
    localparam RS_CSRWE_LO = 19;    //  1
    localparam RS_CSRWR_LO = 20;    //  1
    localparam RS_CSRA_LO  = 21;    // 12
    localparam RS_JV_LO    = 33;    // 32
    localparam RS_AUX_LO   = 65;    // 32
    localparam RS_BRF_LO   = 97;    //  1
    localparam RS_JALF_LO  = 98;    //  1
    localparam RS_JALFR_LO = 99;    //  1
    localparam RS_BJUF4_LO = 100;   //  4
    localparam RS_IMMA_LO  = 104;   //  1
    localparam RS_IMMB_LO  = 105;   //  1
//★ 2026-11-07 载荷瘦身（199→163）：源架构号无读者（cpu_top 那两个口本来就是空接），删掉 10 位；
//  IMMVA 只有 csrrwi 的 zimm 用得上（AUIPC 的 A 侧立即数就是已经存着的 AUX），32 位压成
//  5 位 zimm + 1 位"这是 zimm"。
//  ★ IMMVB 和 JV 不能合成"32 位值 + 种类"：jalr 两条同时要（JV = BTIC 预测目标，要走
//  jalr_pred_tgt 去 bju 比预测；IMMVB = immI 是操作数），合了就没法同时取。所以两者各留 32 位。
    localparam RS_IMMZ_LO  = 106;   //  5    csrrwi 的 zimm（A 侧立即数）
    localparam RS_IMMZSEL_LO = 111; //  1    1 = A 侧立即数是 zimm，0 = 用 AUX
    localparam RS_IMMVB_LO = 112;   // 32
    localparam RS_BPRED_LO = 144;   //  1
    localparam RS_PHYSA_LO = 145;   //  6
    localparam RS_PHYSB_LO = 151;   //  6
    localparam RS_PHYSD_LO = 157;   //  6    目的物理号（提前唤醒用）
//★ 2026-11-08：CSR 写这一位在入站算好。原来选优锥里每项要现算"12 位 !=0 & 3 位 !=0"
//  （两条 lane 各 8 项 = 32 个比较器、每个还带一棵 12 位或树），只为得出这一位。
    localparam RS_CSRW_LO  = 163;   //  1    1 = CSR 写（csra 非 0 且 funct3 非 0）
    localparam RS_PAYLOAD_W = 164;
//---- 访存站几何与载荷布局（2026-11-09 方案二：mem Q 1 格 → 4 项压缩式站，见 docs/SPEC_mem_ooo.md）----
//  压缩式：槽 0 = 最老，取走一条最多补一格孔（与 lane 站逐字同形）。
//  载荷比 lane 站小得多：访存只要 ROB 身份、store/load 区分、funct3、两个地址源。
    localparam MEM_ST_DEPTH = 4;
    localparam MEM_AW       = 2;
    localparam MEM_IDX_LO   = 0;    //  5  ROB 槽号
    localparam MEM_GEN_LO   = 5;    //  1  ROB 世代
    localparam MEM_ISST_LO  = 6;    //  1  1 = store
    localparam MEM_F3_LO    = 7;    //  3  funct3（宽度/符号）
    localparam MEM_AUX_LO   = 10;   // 32  指令地址 + 4
    localparam MEM_IMM_LO   = 42;   // 32  立即数
    localparam MEM_PHYSA_LO = 74;   //  6  基址物理号
    localparam MEM_PHYSB_LO = 80;   //  6  数据物理号（load 侧是垃圾译码，与原来同悲观）
//★ 2026-11-12：目的物理号随站条目一起存。原来它不进站，load 写回时 ROB 要现查一次
//  entry_phys（分布式 RAM）把槽号换成物理号 —— 那一次读正压在 lsu→isq 那条链中间。
    localparam MEM_PHYSD_LO = 86;   //  6  目的物理号
    localparam MEM_PAYLOAD_W = 92;
//---- 站阵列（访存只有一份，不分 lane）----
    reg  [MEM_PAYLOAD_W-1:0] mem_data_m [0:MEM_ST_DEPTH-1];
    reg  [MEM_ST_DEPTH-1:0]  mem_valid_m;
//每项两个源的常驻就绪位：入站定初值，此后 | 六条唤醒总线，发射走 / 被杀清 0
    reg  [MEM_ST_DEPTH-1:0]  mem_rdy_a_m, mem_rdy_b_m;
    reg  [2:0]               mem_count_m;
    reg  [MEM_ST_DEPTH-1:0]  mem_shift_m;
    reg  [2:0]               mem_append_m;
    reg  [MEM_ST_DEPTH-1:0]  mem_entry_ready_a, mem_entry_ready_b, mem_pickable;
    reg  [MEM_ST_DEPTH-1:0]  mem_fwd_a, mem_fwd_b;
//★ 当拍前送电平：这一格【没有出站寄存器】，一就绪就当拍被 LSU 取走 ——
//  所以"常驻位"之外还要认这三条前送（它们说的就是"这一拍总线上有这个号"）。
//  只认常驻位会把消费者往后压一拍；只认前送则会漏掉已经写进阵列的号。
    reg  [2:0]               mem_store_barrier;
    reg                      mem_sel_found;
    reg  [MEM_AW-1:0]        mem_sel;
    reg  [MEM_PAYLOAD_W-1:0] mem_enq_payload;
//---- 站阵列与本地年龄（lane0 / lane1 各一份）----
    reg  [RS_PAYLOAD_W-1:0] rs_data_0 [0:RS_DEPTH-1];
    reg  [RS_PAYLOAD_W-1:0] rs_data_1 [0:RS_DEPTH-1];
    reg  [RS_DEPTH-1:0]     rs_valid_0, rs_valid_1;
//★ 2026-11-10 指针式：载荷只在入站写 tail 那一格，没有位移网络。
//  ★ rs_count 是【窗口长度】tail-head，不是活项数：孔占位到 head 走过去 ⇒ room 比压缩式保守。
    reg  [3:0]              rs_count_0, rs_count_1;
//---- 每项两个源的【常驻】就绪位（标签化）----
//   ★ 入站那一拍定初值（在飞表 + 当拍唤醒），此后 ready_q <= ready_q | ready_w，
//     只有"发射走 / 被杀"才清。不能只存入站快照：唤醒是一拍脉冲，那一拍没被选中就永远丢了。
//   ★ 独立小寄存器阵列，不进载荷 —— 载荷只在入站写一次，改不了。
    reg  [RS_DEPTH-1:0]     rdy_a_0, rdy_b_0, rdy_a_1, rdy_b_1;
    integer                 i;
//年龄指针（位宽取 RS_PW 只为让台子的 rs_tail-rs_head 探针口径不变；加法只用低 RS_AW 位）。
    reg  [RS_PW-1:0]        rs_head_0, rs_head_1, rs_tail_0, rs_tail_1;
    reg                     head_move_0, head_move_1;
    reg  [RS_PW-1:0]        rs_room_0, rs_room_1;
    reg  [RS_PAYLOAD_W-1:0] enqueue_payload_0, enqueue_payload_1;
    reg                     enqueue_0, enqueue_1;
    reg                     load_0, load_1;
//---- 站内每项的就绪 / 可发 ----
    reg  [RS_DEPTH-1:0]     entry_ready_a_0, entry_ready_b_0, entry_ready_a_1, entry_ready_b_1;
    reg  [RS_DEPTH-1:0]     pickable_0, pickable_1;
    reg  [RS_AW-1:0]        pe0_idx, pe1_idx;
    reg                     pe0_found, pe1_found;
//---- 乘法静态唤醒的 2 拍移位寄存器（[0] 当拍发，[1] 下一拍）----
    reg                     multiply_wake_valid_q_0 [0:1];
    reg  [5:0]              multiply_wake_phys_q_0  [0:1];
    reg                     multiply_wake_valid_q_1 [0:1];
    reg  [5:0]              multiply_wake_phys_q_1  [0:1];
    integer                 mw;
//---- 出站寄存器 Q（每 lane 一个）：t0 选优落在这里，t1 直接发往执行级 ----
//   ★ 只有一格。选优那一拍要判断它本拍末会不会空出来（q_room），空不出来就不推。
//   ★ 出口这一拍重判四样：单元收不收、CSR 顺序、冲刷击杀、操作数取不取得到。
//     后两样任意一样不过就压在 Q 上等 —— 压着会堵本 lane 的选优，但不会读错值。
    reg  [RS_PAYLOAD_W-1:0] q_data_0, q_data_1;
    reg                     q_valid_0, q_valid_1;
    reg                     q_unit_ok_0, q_unit_ok_1;
    reg                     q_csr_block_0, q_csr_block_1;
    reg                     q_issue_0, q_issue_1;
    reg                     q_room_0, q_room_1;
//---- 唤醒总线 ----
//  A 组（发射拍，ALU / BJU）：消费者 t1 那一拍从 ALU 的早出口取值，所以发射拍就能叫。
//  B 组（写口落笔，ALU / MUL / DIV / LOAD）：消费者 t1 那一拍从寄存器堆取值。
    wire                    alu_issue_0, alu_issue_1;
    reg                     result_wake_valid_q_0, result_wake_valid_q_1;
    reg  [5:0]              result_wake_phys_q_0, result_wake_phys_q_1;
//---- 在飞表：1 = 这个物理号【已发号】但还没发唤醒（= 值下一拍还读不到）----
//  语义只有一条：不在表里 ⇔ 消费者从入站到读 PRF 之间一定读得到。
//  ★ 置位 = 【发号】那一拍，不是入站。两条 lane 的站各自推进，年轻的那条可能比它的
//    生产者先入站 —— 放"入站置位"时实测读到旧值（种子 4，第 22 个字）。
//  清位 = 唤醒那一拍（每个单元的唤醒都定义成"值下一拍可取"）；
//  冲刷 = 整表清 0。
    reg  [63:0]             inflight_phys;
    wire [63:0]             wake_onehot;
    reg                     q_take_0, q_take_1;
    reg  [31:0]             operand_a_0, operand_b_0, operand_a_1, operand_b_1;
    reg                     alu_lane0_forward_a_0, alu_lane0_forward_b_0, alu_lane1_forward_a_0, alu_lane1_forward_b_0;
    reg                     alu_lane0_forward_a_1, alu_lane0_forward_b_1, alu_lane1_forward_a_1, alu_lane1_forward_b_1;
    reg                     load_forward_a_0, load_forward_b_0, load_forward_a_1, load_forward_b_1;
    reg                     load_forward_a_mem, load_forward_b_mem;
    reg                     alu_lane0_forward_a_mem, alu_lane1_forward_a_mem;
    reg                     alu_lane0_forward_b_mem, alu_lane1_forward_b_mem;
//---- 访存站的出站口（选中项的组合镜像）----
//   ★ 站里是"按项就绪"，所以出站这一拍选中的是谁，就用谁的物理号去读 PRF。
    reg  [5:0]              mem_sel_phys_a, mem_sel_phys_b;
    reg  [31:0]             operand_a_mem, operand_b_mem;
    reg                     mem_issue_valid, mem_load, mem_room;
//---- 控制位 / 复位 ----
    reg                     flush_window;
    reg                     reset_registered;
//---- D 级"这条被冲刷杀着" ----
    reg                     dead_registered_0, dead_registered_1;
    reg                     decode_dead_0, decode_dead_1;
    reg                     decode_dead_mem;
//---- Q 里那一格的载荷字段（从 q_data 拆出来）----
    wire [31:0]             q_immediate_value_a_0, q_immediate_value_a_1;
//---- 前送命中 ----
    wire                    alu_fwd_ok_0, alu_fwd_ok_1, ld_fwd_ok;



//==============================================================================
// 零、控制位 / 复位
//==============================================================================
    always @(*) begin
        flush_window         = irq_kill | redir_backend_0;
    end

    always @(posedge clk) reset_registered <= rst;

//==============================================================================
// 一、冲刷 / 陷阱
//==============================================================================
    always @(*) begin

    end

//==============================================================================
// 三、Q 载荷字段拆线
//==============================================================================
//A 侧立即数只有两种：csrrwi 的 zimm（零扩展 5 位）和 AUIPC 的 PC —— 后者就是 AUX，不用另存
    assign q_immediate_value_a_0      = (q_data_0[RS_IMMZSEL_LO +: 1]) ? {27'd0, (q_data_0[RS_IMMZ_LO  +: 5])} : (q_data_0[RS_AUX_LO   +: 32]);

    assign q_immediate_value_a_1      = (q_data_1[RS_IMMZSEL_LO +: 1]) ? {27'd0, (q_data_1[RS_IMMZ_LO  +: 5])} : (q_data_1[RS_AUX_LO   +: 32]);

//==============================================================================
// 四、入队：两条 lane 各自推进
//   ★ 顺序由结构保证：同一条 lane 里 idu 的 S 级只有一条在等，先来的先入站。
//   ★ 站满就把 station_room_*_valid 拉低，idu 那一拍不再分配 —— 站就是那条缓冲，
//     idu 自己的 2 深载荷只剩下"管线寄存器"的作用。
//==============================================================================
    always @(*) begin
        rs_room_0 = 4'd8 - rs_count_0;
        rs_room_1 = 4'd8 - rs_count_1;
        enqueue_0 = stage_advance       & (rs_room_0 != 4'd0);
        enqueue_1 = stage_advance_lane1 & (rs_room_1 != 4'd0);
        accept_0  = (rs_room_0 != 4'd0);
        accept_1  = (rs_room_1 != 4'd0);
    end
    assign station_room_0_valid = (rs_room_0 != 4'd0);
    assign station_room_1_valid = (rs_room_1 != 4'd0);



//落盘门只认【进来的这一条】自己死没死（错路 D 被消费掉但不进站）。
    always @(*) begin
        decode_dead_0 = dead_registered_0;
        decode_dead_1 = dead_registered_1;
        decode_dead_mem = 1'b0;
        load_0 = enqueue_0 & ~decode_dead_0;
        load_1 = enqueue_1 & ~decode_dead_1;
    end

//A 侧立即数是 zimm 的唯一来源是 csrrwi（SYSTEM ⇒ class 恒为 CLS_BJU）；
//  AUIPC 是 CLS_ALU，它的 A 侧立即数就是 PC（AUX）。这一位决定出站读 zimm 还是读 AUX。
    wire imm_a_is_zimm_0 = decoded_r1_immediate & (decoded_class == CLS_BJU);
    wire imm_a_is_zimm_1 = decoded_r1_immediate_lane1 & (decoded_class_lane1 == CLS_BJU);

//CSR 写 = 地址非 0 且 funct3 非 0（funct3=0 是 ecall/mret 那一类 SYSTEM，不是 CSR 访存）。
//  只在入站算一次：选优锥里"CSR 写必须在 ROB 队头才能发"这条纪律要它，不能每拍每项再算。
    wire decoded_is_csr_write_0 = (decoded_csr_address      != 12'd0) & (decoded_csr_function3      != 3'd0);
    wire decoded_is_csr_write_1 = (decoded_csr_address_lane1 != 12'd0) & (decoded_csr_function3_lane1 != 3'd0);

//==============================================================================
//  入站拍唤醒旁路
//==============================================================================
    wire enq_wake_a_0 = (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == rs1_phys_in))
                      | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == rs1_phys_in))
                      | (result_wake_valid_q_0 & (result_wake_phys_q_0 == rs1_phys_in))
                      | (result_wake_valid_q_1 & (result_wake_phys_q_1 == rs1_phys_in))
                      | (load_write     & (load_write_phys     == rs1_phys_in))
                      | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == rs1_phys_in))
                      | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == rs1_phys_in));
    wire enq_wake_b_0 = (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == rs2_phys_in))
                      | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == rs2_phys_in))
                      | (result_wake_valid_q_0 & (result_wake_phys_q_0 == rs2_phys_in))
                      | (result_wake_valid_q_1 & (result_wake_phys_q_1 == rs2_phys_in))
                      | (load_write     & (load_write_phys     == rs2_phys_in))
                      | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == rs2_phys_in))
                      | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == rs2_phys_in));
    wire enq_wake_a_1 = (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == rs1_phys_in_lane1))
                      | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == rs1_phys_in_lane1))
                      | (result_wake_valid_q_0 & (result_wake_phys_q_0 == rs1_phys_in_lane1))
                      | (result_wake_valid_q_1 & (result_wake_phys_q_1 == rs1_phys_in_lane1))
                      | (load_write     & (load_write_phys     == rs1_phys_in_lane1))
                      | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == rs1_phys_in_lane1))
                      | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == rs1_phys_in_lane1));
    wire enq_wake_b_1 = (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == rs2_phys_in_lane1))
                      | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == rs2_phys_in_lane1))
                      | (result_wake_valid_q_0 & (result_wake_phys_q_0 == rs2_phys_in_lane1))
                      | (result_wake_valid_q_1 & (result_wake_phys_q_1 == rs2_phys_in_lane1))
                      | (load_write     & (load_write_phys     == rs2_phys_in_lane1))
                      | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == rs2_phys_in_lane1))
                      | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == rs2_phys_in_lane1));
    wire enq_ready_a_0 = decoded_r1_immediate | ~inflight_phys[rs1_phys_in] | enq_wake_a_0;
    wire enq_ready_b_0 = decoded_r2_immediate | ~inflight_phys[rs2_phys_in] | enq_wake_b_0;
    wire enq_ready_a_1 = decoded_r1_immediate_lane1 | ~inflight_phys[rs1_phys_in_lane1] | enq_wake_a_1;
    wire enq_ready_b_1 = decoded_r2_immediate_lane1 | ~inflight_phys[rs2_phys_in_lane1] | enq_wake_b_1;
//访存队列那一格：store 的 B 侧是 rs2 真数据；load 的 B 侧是立即数域（当垃圾译码，与原来同悲观）
    wire enq_wake_a_mem = (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == mem_phys_a_in))
                        | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == mem_phys_a_in))
                        | (result_wake_valid_q_0 & (result_wake_phys_q_0 == mem_phys_a_in))
                        | (result_wake_valid_q_1 & (result_wake_phys_q_1 == mem_phys_a_in))
                        | (load_write     & (load_write_phys     == mem_phys_a_in))
                        | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == mem_phys_a_in))
                        | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == mem_phys_a_in));
    wire enq_wake_b_mem = (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == mem_phys_b_in))
                        | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == mem_phys_b_in))
                        | (result_wake_valid_q_0 & (result_wake_phys_q_0 == mem_phys_b_in))
                        | (result_wake_valid_q_1 & (result_wake_phys_q_1 == mem_phys_b_in))
                        | (load_write     & (load_write_phys     == mem_phys_b_in))
                        | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == mem_phys_b_in))
                        | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == mem_phys_b_in));
    wire enq_ready_a_mem = ~inflight_phys[mem_phys_a_in] | enq_wake_a_mem;
    wire enq_ready_b_mem = ~inflight_phys[mem_phys_b_in] | enq_wake_b_mem;

    always @(*) begin
        enqueue_payload_0 = {
            decoded_is_csr_write_0,
            phys_d_in,
            rs2_phys_in,
            rs1_phys_in,
            branch_prediction_taken_in,
            decoded_r2_immediate_value,
            imm_a_is_zimm_0,
            decoded_r1_immediate_value[4:0],
            decoded_r2_immediate,
            decoded_r1_immediate,
            decoded_branch_jump_function4,
            decoded_jump_link_register_flag,
            decoded_jump_link_flag,
            decoded_branch_flag,
            aux_addr_in,
            decoded_jump_value,
            decoded_csr_address,
            decoded_csr_write_commit,
            decoded_csr_write_enable,
            decoded_csr_function3,
            decoded_function4,
            decoded_multiply_operation,
            decoded_write_enable,
            decoded_class,
            alloc_generation_in,
            index_in };
        enqueue_payload_1 = {
            decoded_is_csr_write_1,
            phys_d_in_lane1,
            rs2_phys_in_lane1,
            rs1_phys_in_lane1,
            branch_prediction_taken_in_lane1,
            decoded_r2_immediate_value_lane1,
            imm_a_is_zimm_1,
            decoded_r1_immediate_value_lane1[4:0],
            decoded_r2_immediate_lane1,
            decoded_r1_immediate_lane1,
            decoded_branch_jump_function4_lane1,
            decoded_jump_link_register_flag_lane1,
            decoded_jump_link_flag_lane1,
            decoded_branch_flag_lane1,
            aux_addr_in1,
            decoded_jump_value_lane1,
            decoded_csr_address_lane1,
            decoded_csr_write_commit_lane1,
            decoded_csr_write_enable_lane1,
            decoded_csr_function3_lane1,
            decoded_function4_lane1,
            decoded_multiply_operation_lane1,
            decoded_write_enable_lane1,
            decoded_class_lane1,
            alloc_generation_in_lane1,
            index_in_lane1 };
    end

//==============================================================================
// 五、站内每项的就绪 / 可发（t0：决定能不能选进 Q）
//   就绪比【物理号】：本项 ready_q | 六条唤醒总线；三条前送只留在操作数 mux 里。
//   ★ 三条前送（phys_alu_early / ld_phys_early）【不进就绪】：它们只说"这一拍总线上有这个号"，
//     不等于"值进了寄存器堆"。实测：load 的早写口被同拍晚写口门掉时（prf.v 的 wp3_active），
//     前送亮、阵列却没写 ⇒ 消费者下一拍在 PRF 里读到旧值（CoreMark 12k 拍：lw 拿 0 当地址、
//     外设 load 永不回）。前送只留在操作数 mux 里 —— 那一拍的数据本身是对的。
//   可发 = valid & 两源就绪 & 单元空闲 & 不在等 CSR 顺序 & 没被冲刷杀。
//==============================================================================
    assign alu_fwd_ok_0 = write_enable_alu_early        & (|phys_alu_early_0);
    assign alu_fwd_ok_1 = write_enable_alu_lane1_early  & (|phys_alu_early_1);
    assign ld_fwd_ok    = write_enable_ld_early         & (|ld_phys_early);

    always @(*) begin : blk_rs_ready
        reg [5:0] entry_phys_a, entry_phys_b;
        reg [4:0] entry_index;
        reg [1:0] entry_class;
        reg [2:0] entry_mulop;
        reg entry_csrw, entry_div, entry_csr_block;
        integer i;
        for (i = 0; i < RS_DEPTH; i = i + 1) begin
            entry_phys_a = rs_data_0[i][RS_PHYSA_LO +: 6];
            entry_phys_b = rs_data_0[i][RS_PHYSB_LO +: 6];
            entry_index  = rs_data_0[i][RS_IDX_LO   +: 5];
            entry_class  = rs_data_0[i][RS_CLASS_LO +: 2];
            entry_mulop  = rs_data_0[i][RS_MULOP_LO +: 3];
            entry_csrw   = rs_data_0[i][RS_CSRW_LO];
            entry_div    = entry_mulop[2];
            entry_ready_a_0[i] = rdy_a_0[i]
                             | (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == entry_phys_a))
                             | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == entry_phys_a))
                             | (result_wake_valid_q_0 & (result_wake_phys_q_0 == entry_phys_a))
                             | (result_wake_valid_q_1 & (result_wake_phys_q_1 == entry_phys_a))
                             | (load_write     & (load_write_phys     == entry_phys_a))
                             | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == entry_phys_a))
                             | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == entry_phys_a));
            entry_ready_b_0[i] = rdy_b_0[i]
                             | (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == entry_phys_b))
                             | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == entry_phys_b))
                             | (result_wake_valid_q_0 & (result_wake_phys_q_0 == entry_phys_b))
                             | (result_wake_valid_q_1 & (result_wake_phys_q_1 == entry_phys_b))
                             | (load_write     & (load_write_phys     == entry_phys_b))
                             | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == entry_phys_b))
                             | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == entry_phys_b));
            entry_csr_block = entry_csrw
                            & (~head_one_hot_registered[entry_index] | csr_write_landing);
//★ 2026-11-08：单元就绪这一项只留【除法】那一路 —— multiply_ready 恒 1（pipe_stall 恒 0），
//  综合时那一支自己会折成常数；除法是真门（口上现在只剩"接受缓冲空不空"这一位）。
//  "选进来再等"不行：MDU 项占了 Q 一格就把本 lane 别的项挡住了。
            pickable_0[i] = rs_valid_0[i] & entry_ready_a_0[i] & entry_ready_b_0[i] & ~entry_csr_block
                          & ((entry_class != CLS_MUL) | (entry_div ? mdu_div_ok_0 : mdu_mul_ok_0));
        end
    end

    always @(*) begin : blk_rs_ready_1
        reg [5:0] entry_phys_a, entry_phys_b;
        reg [4:0] entry_index;
        reg [1:0] entry_class;
        reg [2:0] entry_mulop;
        reg entry_csrw, entry_div, entry_csr_block;
        integer i;
        for (i = 0; i < RS_DEPTH; i = i + 1) begin
            entry_phys_a = rs_data_1[i][RS_PHYSA_LO +: 6];
            entry_phys_b = rs_data_1[i][RS_PHYSB_LO +: 6];
            entry_index  = rs_data_1[i][RS_IDX_LO   +: 5];
            entry_class  = rs_data_1[i][RS_CLASS_LO +: 2];
            entry_mulop  = rs_data_1[i][RS_MULOP_LO +: 3];
            entry_csrw   = rs_data_1[i][RS_CSRW_LO];
            entry_div    = entry_mulop[2];
            entry_ready_a_1[i] = rdy_a_1[i]
                             | (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == entry_phys_a))
                             | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == entry_phys_a))
                             | (result_wake_valid_q_0 & (result_wake_phys_q_0 == entry_phys_a))
                             | (result_wake_valid_q_1 & (result_wake_phys_q_1 == entry_phys_a))
                             | (load_write     & (load_write_phys     == entry_phys_a))
                             | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == entry_phys_a))
                             | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == entry_phys_a));
            entry_ready_b_1[i] = rdy_b_1[i]
                             | (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == entry_phys_b))
                             | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == entry_phys_b))
                             | (result_wake_valid_q_0 & (result_wake_phys_q_0 == entry_phys_b))
                             | (result_wake_valid_q_1 & (result_wake_phys_q_1 == entry_phys_b))
                             | (load_write     & (load_write_phys     == entry_phys_b))
                             | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == entry_phys_b))
                             | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == entry_phys_b));
            entry_csr_block = entry_csrw
                            & (~head_one_hot_registered[entry_index] | csr_write_landing);
            pickable_1[i] = rs_valid_1[i] & entry_ready_a_1[i] & entry_ready_b_1[i] & ~entry_csr_block
                          & ((entry_class != CLS_MUL) | (entry_div ? mdu_div_ok_1 : mdu_mul_ok_1));
        end
    end

//==============================================================================
//  六、冲刷击杀：一个全局位，不查表
//   ★ 改向只在提交口发一次，队头那项退休的拍整窗作废 ⇒ 每一次冲刷都是全杀。
//     rob 的击杀掩码本来就是 MASK_ALL 常量（rob.v 里那两根 assign），逐项查 32:1 mux
//     综合后折成常数 1。这里直接用一个位，判据仍然是这一拍死不死。
//==============================================================================

//==============================================================================
//  六之二、常驻就绪位：入站定初值，唤醒置 1，发射走 / 被杀清 0
//   ★ entry_ready_* 已经是"常驻位 | 当拍命中"，所以非入站拍直接把它写回去就是"置 1 保持"。
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            rdy_a_0 <= {RS_DEPTH{1'b0}};
            rdy_b_0 <= {RS_DEPTH{1'b0}};
            rdy_a_1 <= {RS_DEPTH{1'b0}};
            rdy_b_1 <= {RS_DEPTH{1'b0}};
        end
        else begin
            for (i = 0; i < RS_DEPTH; i = i + 1) begin
//三路优先：入站 > 本拍被取走 > 保持（保持那一路照旧把当拍唤醒 OR 进去）。
                if (load_0 & (rs_tail_0[RS_AW-1:0] == i[RS_AW-1:0])) begin
                    rdy_a_0[i] <= enq_ready_a_0;
                    rdy_b_0[i] <= enq_ready_b_0;
                end
                else if (q_take_0 & (pe0_idx == i[RS_AW-1:0])) begin
                    rdy_a_0[i] <= 1'b0;
                    rdy_b_0[i] <= 1'b0;
                end
                else begin
                    rdy_a_0[i] <= entry_ready_a_0[i];
                    rdy_b_0[i] <= entry_ready_b_0[i];
                end
                if (load_1 & (rs_tail_1[RS_AW-1:0] == i[RS_AW-1:0])) begin
                    rdy_a_1[i] <= enq_ready_a_1;
                    rdy_b_1[i] <= enq_ready_b_1;
                end
                else if (q_take_1 & (pe1_idx == i[RS_AW-1:0])) begin
                    rdy_a_1[i] <= 1'b0;
                    rdy_b_1[i] <= 1'b0;
                end
                else begin
                    rdy_a_1[i] <= entry_ready_a_1[i];
                    rdy_b_1[i] <= entry_ready_b_1[i];
                end
            end
        end
    end

//==============================================================================
// 七、选优：每条 lane 各自从自己的 rs_head 起转一圈，取最老的可发项
//==============================================================================
    always @(*) begin : blk_select
        reg [RS_AW-1:0] scan_slot_0, scan_slot_1;
        integer j;
//指针式：从 head 起转一圈取第一个可发项（孔自动跳过，pickable 对孔恒 0）。
        pe0_found = 1'b0;
        pe0_idx   = {RS_AW{1'b0}};
        for (j = RS_DEPTH-1; j >= 0; j = j - 1) begin
            scan_slot_0 = rs_head_0[RS_AW-1:0] + j[RS_AW-1:0];
            if (pickable_0[scan_slot_0]) begin
                pe0_found = 1'b1;
                pe0_idx   = scan_slot_0;
            end
        end
        pe1_found = 1'b0;
        pe1_idx   = {RS_AW{1'b0}};
        for (j = RS_DEPTH-1; j >= 0; j = j - 1) begin
            scan_slot_1 = rs_head_1[RS_AW-1:0] + j[RS_AW-1:0];
            if (pickable_1[scan_slot_1]) begin
                pe1_found = 1'b1;
                pe1_idx   = scan_slot_1;
            end
        end
    end

//==============================================================================
// 八、出站寄存器 Q 的出口判据（每 lane 一份）
//   单元肯不肯收 / 等不等 CSR 顺序 / 有没有被冲刷杀 —— 这三样按 Q 里那一条重判一遍。
//   操作数不重判：能落进 Q 的项，t0 两源就已就绪（见文件头那条前提）。
//==============================================================================
    always @(*) begin : blk_q_exit_0
        reg [4:0]  held_index;
        reg [1:0]  held_class;
        reg        held_div;
        reg        held_csrw;
        held_index = q_data_0[RS_IDX_LO    +: 5];
        held_class = q_data_0[RS_CLASS_LO  +: 2];
        held_div   = q_data_0[RS_MULOP_LO  + 2];
        held_csrw  = q_data_0[RS_CSRW_LO];
        q_unit_ok_0   = (held_class != CLS_MUL) | (held_div ? mdu_div_ok_0 : mdu_mul_ok_0);
        q_csr_block_0 = held_csrw
                      & (~head_one_hot_registered[held_index] | csr_write_landing);
    end

    always @(*) begin : blk_q_exit_1
        reg [4:0]  held_index;
        reg [1:0]  held_class;
        reg        held_div;
        reg        held_csrw;
        held_index = q_data_1[RS_IDX_LO    +: 5];
        held_class = q_data_1[RS_CLASS_LO  +: 2];
        held_div   = q_data_1[RS_MULOP_LO  + 2];
        held_csrw  = q_data_1[RS_CSRW_LO];
        q_unit_ok_1   = (held_class != CLS_MUL) | (held_div ? mdu_div_ok_1 : mdu_mul_ok_1);
        q_csr_block_1 = held_csrw
                      & (~head_one_hot_registered[held_index] | csr_write_landing);
    end

//==============================================================================
// 八之二、唤醒：两组，区别在"消费者 t1 从哪儿拿到值"
//
//   A 组 · 发射拍（wake_issue_*，本次只发 CLS_ALU）
//     ALU 的操作数在发射后一拍才进 op_* 寄存器，早出口（result_early / write_enable_early /
//     index_early）也在那一拍有效 ⇒ 消费者 t1 出站时，值正好在这条早出口上，
//     走的是出口 mux 里的前送档。所以"这条要发出去"那一拍叫它就对，比写口早一整拍。
//     条件：class == CLS_ALU、write_enable、phys_d != 0。
//     ★ phys_d 必须是本条【真的会写】的物理号（idu 现在按 rob 的 entry_phys 同口径给 0）。
//       不改这一条时实测：连续两条 rd=x0 的指令载荷都带着空闲扫描值 22，而 ROB 给它们记 0
//       ⇒ 一场"没人写的号可读了"的假唤醒，消费者读回旧值（rand 11 / 14）。
//     ★ BJU 的 jal/jalr 连接值同样从 ALU 早出口出去，机制上也能放进 A 组；本次没开，
//       保持与测量口径一致。
//
//   B 组 · 写口落笔（wake_alu_* / wake_result_* / load_write）
//     消费者 t1 从寄存器堆取值。写口落笔那一拍 = 下一拍一定读得到。
//     wake_alu_*   ：ALU 自己的写口（结果口不再仲裁 ALU，落笔时刻确定）；
//     wake_result_*：结果口，现在只剩 MUL / DIV（仲裁和 live 都算过）；
//     load_write    ：LOAD 落笔（早口或返回级，prf 合并后的那一笔）。
//
//   ★ 两组都保证"消费者 t1 那一拍取得到值"，所以出口不用再判一遍操作数。
//     反面教材（2026-11-07 实测）：拿 LOAD 早写口（ld_fwd_ok）当就绪 —— 它是"这一拍总线上
//     有这个号"，下一拍既不在总线上、阵列里也可能没写（被同拍晚口门掉）⇒ CoreMark 12k 拍
//     消费者读到旧值，load 拿 0 当地址，外设 load 永不回。LOAD 只许走 B 组。
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            result_wake_valid_q_0 <= 1'b0;
            result_wake_valid_q_1 <= 1'b0;
            result_wake_phys_q_0  <= 6'd0;
            result_wake_phys_q_1  <= 6'd0;
        end
        else begin
            result_wake_valid_q_0 <= result_write_valid_0 & result_is_divide_0 & (result_write_phys_0 != 6'd0);
            result_wake_valid_q_1 <= result_write_valid_1 & result_is_divide_1 & (result_write_phys_1 != 6'd0);
            result_wake_phys_q_0  <= result_write_phys_0;
            result_wake_phys_q_1  <= result_write_phys_1;
        end
    end
//★ 晚写口：早口被同拍晚口门掉时，这笔 load 只从这里落笔（prf 的 wp3_active）。
//★ 2026-11-08：ALU 的写口 4/5 不再单独发唤醒 —— 一个单元只留一条通路（A 组发射拍）。
//  就绪位是常驻的，A 组那一拍置 1 后不会丢；之后任何时候选中，值要么在旁路、要么已进阵列。

//==============================================================================
// 八之二点五
// ALU+乘法静态唤醒
//==============================================================================

    assign alu_issue_0      = q_issue_0 & (q_data_0[RS_WE_LO    +: 1]) & ((q_data_0[RS_CLASS_LO +: 2]) != CLS_MUL)
                             & ((q_data_0[RS_PHYSD_LO +: 6]) != 6'd0);
    assign alu_issue_1      = q_issue_1 & (q_data_1[RS_WE_LO    +: 1]) & ((q_data_1[RS_CLASS_LO +: 2]) != CLS_MUL)
                             & ((q_data_1[RS_PHYSD_LO +: 6]) != 6'd0);

//★ 2026-11-11：这五个声明原来落在这里（第 452 行就开始用了）⇒ Vivado 报 8-6901
//  "used before declaration"。全文件按"声明全部上移"的约定，声明一律放在上面那一块。
    wire multiply_issue_0 = q_issue_0 & ((q_data_0[RS_CLASS_LO +: 2]) == CLS_MUL) & ~(q_data_0[RS_MULOP_LO + 2]) & ((q_data_0[RS_PHYSD_LO +: 6]) != 6'd0);
    wire multiply_issue_1 = q_issue_1 & ((q_data_1[RS_CLASS_LO +: 2]) == CLS_MUL) & ~(q_data_1[RS_MULOP_LO + 2]) & ((q_data_1[RS_PHYSD_LO +: 6]) != 6'd0);

    always @(posedge clk) begin
        if (reset_registered | flush_window | redir_backend_1) begin
            for (mw = 0; mw < 2; mw = mw + 1) begin
                multiply_wake_valid_q_0[mw] <= 1'b0;
                multiply_wake_phys_q_0[mw]  <= 6'd0;
                multiply_wake_valid_q_1[mw] <= 1'b0;
                multiply_wake_phys_q_1[mw]  <= 6'd0;
            end
        end
        else begin
            multiply_wake_valid_q_0[0] <= multiply_issue_0;
            multiply_wake_phys_q_0[0]  <= (q_data_0[RS_PHYSD_LO +: 6]);
            multiply_wake_valid_q_1[0] <= multiply_issue_1;
            multiply_wake_phys_q_1[0]  <= (q_data_1[RS_PHYSD_LO +: 6]);
            for (mw = 1; mw < 2; mw = mw + 1) begin
                multiply_wake_valid_q_0[mw] <= multiply_wake_valid_q_0[mw-1];
                multiply_wake_phys_q_0[mw]  <= multiply_wake_phys_q_0[mw-1];
                multiply_wake_valid_q_1[mw] <= multiply_wake_valid_q_1[mw-1];
                multiply_wake_phys_q_1[mw]  <= multiply_wake_phys_q_1[mw-1];
            end
        end
    end

//==============================================================================
// 八之三、在飞表
//   置位：【发号】那一拍（alloc_onehot_in），不是入站；store 与 x0 不置位。
//   清位：六条唤醒总线（A 组 2 + 结果口 2 + LOAD 早/晚口 2）。
//==============================================================================
//★ 2026-11-10 时序：这里原来是 `inflight_phys | alloc_onehot_in`。
//  那个"或"是 rename 的空闲号扫描（free_vector → 64 位扫描 → pick）一路组合钻到
//  站的就绪位 D 端（全核最差一族：free_vector_reg → rdy_a_0_reg，−3.978 / 14~17 级）。
//  ★ 去掉它是对的，不是拿功能换时序：站的入站恒在【分配的后一拍】——
//    载荷先落 idu 的 S 级寄存器，mem 条先落 LSU FIFO，两者都比发号晚至少一拍
//    ⇒ 入站那一拍 inflight_phys 里已经有本周期发出去的号了。
//    同拍发号的两条（槽 0/1）也都在同一个沿写进 inflight_phys，入站时看得到。
    assign wake_onehot =
          (alu_issue_0  ? (64'd1 << (q_data_0[RS_PHYSD_LO +: 6]))  : 64'd0)
        | (alu_issue_1  ? (64'd1 << (q_data_1[RS_PHYSD_LO +: 6]))  : 64'd0)
        | (result_wake_valid_q_0 ? (64'd1 << result_wake_phys_q_0) : 64'd0)
        | (result_wake_valid_q_1 ? (64'd1 << result_wake_phys_q_1) : 64'd0)
        | (load_write ? (64'd1 << load_write_phys) : 64'd0)
        | (multiply_wake_valid_q_0[1] ? (64'd1 << multiply_wake_phys_q_0[1]) : 64'd0)
        | (multiply_wake_valid_q_1[1] ? (64'd1 << multiply_wake_phys_q_1[1]) : 64'd0);

    always @(posedge clk) begin
        if (reset_registered) begin
            inflight_phys <= 64'd0;
        end
        else if ((redir_backend_0 | redir_backend_1)) begin
            inflight_phys <= 64'd0;
        end
        else begin
            inflight_phys <= (inflight_phys | alloc_onehot_in) & ~wake_onehot;
        end
    end


//==============================================================================
// 九、操作数取值（t1，PRF 组合读 + 三条前送）
//   下面这些 q_* 都是 Q 里那一条的镜像：读口地址、载荷、前送命中全按它来。
//==============================================================================
//CSR 载荷在发射那一拍交给 ROB（那边只锁一深）。源那一栏是 rs1 的物理号；
//csrrwi 这类带立即数的，填立即数低 5 位（与 idu 分配时的口径逐字一致）。
    assign csr_payload_0 = {(q_data_0[RS_CSRWR_LO +: 1]), (q_data_0[RS_CSRA_LO  +: 12]), (q_data_0[RS_CSRF3_LO +: 3]),
                            (q_data_0[RS_IMMA_LO  +: 1]) ? {1'b0, q_immediate_value_a_0[4:0]} : (q_data_0[RS_PHYSA_LO +: 6])};
    assign csr_payload_1 = {(q_data_1[RS_CSRWR_LO +: 1]), (q_data_1[RS_CSRA_LO  +: 12]), (q_data_1[RS_CSRF3_LO +: 3]),
                            (q_data_1[RS_IMMA_LO  +: 1]) ? {1'b0, q_immediate_value_a_1[4:0]} : (q_data_1[RS_PHYSA_LO +: 6])};
    always @(*) begin
        load_forward_a_0 = ld_fwd_ok & (ld_phys_early == (q_data_0[RS_PHYSA_LO +: 6]));
        load_forward_b_0 = ld_fwd_ok & (ld_phys_early == (q_data_0[RS_PHYSB_LO +: 6]));
        load_forward_a_1 = ld_fwd_ok & (ld_phys_early == (q_data_1[RS_PHYSA_LO +: 6]));
        load_forward_b_1 = ld_fwd_ok & (ld_phys_early == (q_data_1[RS_PHYSB_LO +: 6]));
        alu_lane0_forward_a_0 = alu_fwd_ok_0 & (phys_alu_early_0 == (q_data_0[RS_PHYSA_LO +: 6]));
        alu_lane1_forward_a_0 = alu_fwd_ok_1 & (phys_alu_early_1 == (q_data_0[RS_PHYSA_LO +: 6]));
        alu_lane0_forward_b_0 = alu_fwd_ok_0 & (phys_alu_early_0 == (q_data_0[RS_PHYSB_LO +: 6]));
        alu_lane1_forward_b_0 = alu_fwd_ok_1 & (phys_alu_early_1 == (q_data_0[RS_PHYSB_LO +: 6]));
        alu_lane0_forward_a_1 = alu_fwd_ok_0 & (phys_alu_early_0 == (q_data_1[RS_PHYSA_LO +: 6]));
        alu_lane1_forward_a_1 = alu_fwd_ok_1 & (phys_alu_early_1 == (q_data_1[RS_PHYSA_LO +: 6]));
        alu_lane0_forward_b_1 = alu_fwd_ok_0 & (phys_alu_early_0 == (q_data_1[RS_PHYSB_LO +: 6]));
        alu_lane1_forward_b_1 = alu_fwd_ok_1 & (phys_alu_early_1 == (q_data_1[RS_PHYSB_LO +: 6]));
//档位顺序：load 前送紧贴 PRF（它的数据到得最晚）。各档互斥，换顺序不改行为。
        operand_a_0 = (q_data_0[RS_IMMA_LO  +: 1]) ? q_immediate_value_a_0 :
                      alu_lane0_forward_a_0 ? data_alu_early :
                      alu_lane1_forward_a_0 ? data_alu_lane1_early :
                      load_forward_a_0 ? data_ld_early :
                      prf_data_a_0;
        operand_b_0 = (q_data_0[RS_IMMB_LO  +: 1]) ? (q_data_0[RS_IMMVB_LO +: 32]) :
                      alu_lane0_forward_b_0 ? data_alu_early :
                      alu_lane1_forward_b_0 ? data_alu_lane1_early :
                      load_forward_b_0 ? data_ld_early :
                      prf_data_b_0;
        operand_a_1 = (q_data_1[RS_IMMA_LO  +: 1]) ? q_immediate_value_a_1 :
                      alu_lane0_forward_a_1 ? data_alu_early :
                      alu_lane1_forward_a_1 ? data_alu_lane1_early :
                      load_forward_a_1 ? data_ld_early :
                      prf_data_a_1;
        operand_b_1 = (q_data_1[RS_IMMB_LO  +: 1]) ? (q_data_1[RS_IMMVB_LO +: 32]) :
                      alu_lane0_forward_b_1 ? data_alu_early :
                      alu_lane1_forward_b_1 ? data_alu_lane1_early :
                      load_forward_b_1 ? data_ld_early :
                      prf_data_b_1;
    end

//==============================================================================
// 十、发射判据
//   ★ t1 补两样：单元收不收、CSR 顺序轮到没有。冲刷不在这判 —— 冲刷那一拍照发。
//     错路那条的 PRF 写有 write_live 挡，CSR 写与访存都在提交口才落地。
//     操作数不重判 —— 唤醒的定义就是"这一拍值进了寄存器堆"，t1 去读一定读得到。
//   ★ q_room = Q 这一拍末会不会空出来：空着或这一拍发出去，就能接新项。
//==============================================================================
    always @(*) begin
        q_issue_0 = q_valid_0 & q_unit_ok_0 & ~q_csr_block_0;
        q_issue_1 = q_valid_1 & q_unit_ok_1 & ~q_csr_block_1;
        q_room_0 = ~q_valid_0 | q_issue_0;
        q_room_1 = ~q_valid_1 | q_issue_1;
        issue_valid_0 = q_issue_0;
        issue_valid_1 = q_issue_1;
        issue_pop_0 = issue_valid_0;
        issue_pop_1 = issue_valid_1;
    end
//★ q_take 单独一块：q_issue → 唤醒 → 站内就绪 → sel_found → q_take → q_room，
//  写在同一个 always 里 Verilator 会把这个块当成一个组合节点，报出一条假的零延时环。
    always @(*) begin
        q_take_0 = pe0_found & q_room_0;
        q_take_1 = pe1_found & q_room_1;
    end

//==============================================================================
// 十一、站阵列时序：入队 / 释放 / 本地年龄指针
//   ★ 窗口 [head, tail) 恒连续，孔不复用 ⇒ 槽序 = 程序序。
//   ★ 指针加法只取低 RS_AW 位：4 位的话 tail=7 加一得 8，不回卷。
//==============================================================================
//head 前进：本拍把 head 那项取走了，或 head 那格已经是个孔。
    always @(*) begin
        head_move_0 = (rs_count_0 != 4'd0)
                    & (~rs_valid_0[rs_head_0[RS_AW-1:0]]
                       | (q_take_0 & (pe0_idx == rs_head_0[RS_AW-1:0])));
        head_move_1 = (rs_count_1 != 4'd0)
                    & (~rs_valid_1[rs_head_1[RS_AW-1:0]]
                       | (q_take_1 & (pe1_idx == rs_head_1[RS_AW-1:0])));
    end

    always @(posedge clk) begin
        if (reset_registered | (redir_backend_0 | redir_backend_1)) begin
            rs_valid_0 <= {RS_DEPTH{1'b0}};
            rs_valid_1 <= {RS_DEPTH{1'b0}};
            rs_count_0 <= 4'd0;
            rs_count_1 <= 4'd0;
            rs_head_0  <= {RS_PW{1'b0}};
            rs_head_1  <= {RS_PW{1'b0}};
            rs_tail_0  <= {RS_PW{1'b0}};
            rs_tail_1  <= {RS_PW{1'b0}};
        end
        else begin
            for (i = 0; i < RS_DEPTH; i = i + 1) begin
//两路：入站（写 tail 那一格）> 保持。被取走的那一格只清 valid，载荷留着不管。
                if (load_0 & (rs_tail_0[RS_AW-1:0] == i[RS_AW-1:0])) begin
                    rs_data_0[i]  <= enqueue_payload_0;
                    rs_valid_0[i] <= 1'b1;
                end
                else if (q_take_0 & (pe0_idx == i[RS_AW-1:0])) begin
                    rs_valid_0[i] <= 1'b0;
                end
                if (load_1 & (rs_tail_1[RS_AW-1:0] == i[RS_AW-1:0])) begin
                    rs_data_1[i]  <= enqueue_payload_1;
                    rs_valid_1[i] <= 1'b1;
                end
                else if (q_take_1 & (pe1_idx == i[RS_AW-1:0])) begin
                    rs_valid_1[i] <= 1'b0;
                end
            end
//两个操作数都写成 RS_AW 位，结果才是 RS_AW 位 ⇒ 7+1 回卷到 0。
            rs_tail_0  <= {1'b0, rs_tail_0[RS_AW-1:0] + {{(RS_AW-1){1'b0}}, load_0}};
            rs_tail_1  <= {1'b0, rs_tail_1[RS_AW-1:0] + {{(RS_AW-1){1'b0}}, load_1}};
            rs_head_0  <= {1'b0, rs_head_0[RS_AW-1:0] + {{(RS_AW-1){1'b0}}, head_move_0}};
            rs_head_1  <= {1'b0, rs_head_1[RS_AW-1:0] + {{(RS_AW-1){1'b0}}, head_move_1}};
            rs_count_0 <= rs_count_0 + {3'd0, load_0} - {3'd0, head_move_0};
            rs_count_1 <= rs_count_1 + {3'd0, load_1} - {3'd0, head_move_1};
        end
    end

//---- 出站寄存器：t0 选优落进来，t1 发出去或被杀掉 ----
    always @(posedge clk) begin
        if (reset_registered) begin
            q_valid_0 <= 1'b0;
            q_valid_1 <= 1'b0;
            q_data_0  <= {RS_PAYLOAD_W{1'b0}};
            q_data_1  <= {RS_PAYLOAD_W{1'b0}};
        end
        else if ((redir_backend_0 | redir_backend_1)) begin
            q_valid_0 <= 1'b0;
            q_valid_1 <= 1'b0;
        end
        else begin
            if (q_take_0) q_data_0 <= rs_data_0[pe0_idx];
            if (q_take_1) q_data_1 <= rs_data_1[pe1_idx];
//发出去腾出来；压着等单元的那一拍保持（那一拍 q_take 必为 0）。
            q_valid_0 <= q_take_0 | (q_valid_0 & ~q_issue_0);
            q_valid_1 <= q_take_1 | (q_valid_1 & ~q_issue_1);
        end
    end

//==============================================================================
//  十二、访存站：可发判定 / 选优 / 出站（2026-11-09 方案二）
//   1 = 每项两个源的常驻就绪位（写法与 lane 站逐字同形）；
//   2 = 选优取【最老的可发项】，不是队头；
//   3 = 两条顺序铁律（论证见 docs/SPEC_mem_ooo.md §2）：
//       · store 只有在自己是站里最老项时才能发 —— 进 ELSQ 的次序就是程序序，
//         年轻 store 抢在老的外设 load 前面会让那条 load 永远不是队头 ⇒ 死锁；
//       · load 不许越过站里更老的 store —— 那条 store 的地址还没算，LSU 无从比较，
//         放过去就只能读到旧值。
//==============================================================================
    always @(*) begin : blk_mem_pick
        reg [5:0] entry_phys_a, entry_phys_b;
        reg       entry_is_store;
        integer k;
//站里最老的 store 的槽号；没有 store 时 = DEPTH（不挡任何 load）
        mem_store_barrier = MEM_ST_DEPTH[2:0];
        for (k = MEM_ST_DEPTH-1; k >= 0; k = k - 1)
            if (mem_valid_m[k] & mem_data_m[k][MEM_ISST_LO])
                mem_store_barrier = k[2:0];
        for (k = 0; k < MEM_ST_DEPTH; k = k + 1) begin
            entry_phys_a   = mem_data_m[k][MEM_PHYSA_LO +: 6];
            entry_phys_b   = mem_data_m[k][MEM_PHYSB_LO +: 6];
            entry_is_store = mem_data_m[k][MEM_ISST_LO];
//★ 2026-11-12：只留 load 那一项。实测（CoreMark 327,706 条访存发射）fwd-only 的
//  55,433 条 **全部**来自 load 项，ALU 那两项贡献精确为 0 —— 因为 ALU 生产者在发射拍
//  就叫醒了（alu_issue_0/1 进就绪列表），下一拍值上旁路时常驻位已经置好。
//  load 不一样：它的早唤醒 load_wake_valid 根本没接进站（isq 里引用数 0），
//  只有晚一拍的 load_write，所以这一项必须留。
//  删掉 ALU 两项 = 把 ALU 的物理号从访存站选优锥上摘掉。
            mem_fwd_a[k] = (ld_fwd_ok & (ld_phys_early == entry_phys_a));
            mem_fwd_b[k] = (ld_fwd_ok & (ld_phys_early == entry_phys_b));
            mem_entry_ready_a[k] = mem_rdy_a_m[k]
                             | (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == entry_phys_a))
                             | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == entry_phys_a))
                             | (result_wake_valid_q_0 & (result_wake_phys_q_0 == entry_phys_a))
                             | (result_wake_valid_q_1 & (result_wake_phys_q_1 == entry_phys_a))
                             | (load_write     & (load_write_phys     == entry_phys_a))
                             | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == entry_phys_a))
                             | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == entry_phys_a));
            mem_entry_ready_b[k] = mem_rdy_b_m[k]
                             | (alu_issue_0  & ((q_data_0[RS_PHYSD_LO +: 6])  == entry_phys_b))
                             | (alu_issue_1  & ((q_data_1[RS_PHYSD_LO +: 6])  == entry_phys_b))
                             | (result_wake_valid_q_0 & (result_wake_phys_q_0 == entry_phys_b))
                             | (result_wake_valid_q_1 & (result_wake_phys_q_1 == entry_phys_b))
                             | (load_write     & (load_write_phys     == entry_phys_b))
                             | (multiply_wake_valid_q_0[1] & (multiply_wake_phys_q_0[1] == entry_phys_b))
                             | (multiply_wake_valid_q_1[1] & (multiply_wake_phys_q_1[1] == entry_phys_b));
//★ 2026-11-12：前送不再进 pickable。唤醒是静态的 —— 醒了就一定可发，前送只决定【值从哪来】。
//  原来这里挂着 mem_fwd_a/b（"没醒但旁路有货也发"），那条判据把 ALU 的物理号
//  (phys_alu_early) 拉进了选优锥 ⇒ 选优 → PRF 读地址 → store 基址 → lsu 一条 13 级链。
//★ 2026-11-12：与两条 lane 站同构 —— pickable 吃【活的】ready（常驻位 | 当拍唤醒），
//  不是只吃常驻位。原来只吃 mem_rdy_a_m，看不见当拍唤醒，于是拿 mem_fwd（"旁路有货就算能发"）
//  补那一拍；那条判据把 ALU 的物理号拉进了选优锥（选优 → PRF 读地址 → store 基址 → lsu）。
//★ 2026-11-12：这里【不能】照抄 lane 站吃活的 entry_ready —— 访存站没有 Q 阶段，
//  选中和读操作数同一拍，唤醒那一拍值还没上总线（实测照抄后 rand 63/200）。
//  所以只能用 mem_fwd（"上一拍的结果已经在旁路上"）来补，这是结构差异不是冗余。
            mem_pickable[k] = mem_valid_m[k]
                            & (mem_rdy_a_m[k] | mem_fwd_a[k])
//★ B 侧只有 store 才要：load 的 B 就是立即数位域译出来的"假 rs2"，
//  lsu 那条路上 source_b 只在 store_stage_write_data 用（lsu.v:276/593）。
//  等它 = 干等一个没人要的物理号（而且那个号可能正好在飞）——这一条白送。
                            & (~entry_is_store | mem_rdy_b_m[k] | mem_fwd_b[k])
                            & (entry_is_store ? (k == 0) : (k[2:0] <= mem_store_barrier));
        end
    end
//选优：槽号就是年龄，从 0 往上扫，最后一个命中的就是最老的可发项（与 lane 站同写法）。
    always @(*) begin : blk_mem_select
        integer j;
        mem_sel_found = 1'b0;
        mem_sel       = {MEM_AW{1'b0}};
        for (j = MEM_ST_DEPTH-1; j >= 0; j = j - 1) begin
            if (mem_pickable[j]) begin
                mem_sel_found = 1'b1;
                mem_sel       = j[MEM_AW-1:0];
            end
        end
    end
//选中项的两个源物理号：PRF 读地址从这里出（多的一层 4:1 加在【地址】上，不在数据上）。
    always @(*) begin
        mem_sel_phys_a = mem_data_m[mem_sel][MEM_PHYSA_LO +: 6];
        mem_sel_phys_b = mem_data_m[mem_sel][MEM_PHYSB_LO +: 6];
    end
//入站载荷：IDU FIFO 那一套字段原样搬进站。低位在前，与上面的 *_LO 表一致。
    always @(*) begin
        mem_enq_payload = {
            lsu_phys_d_in, mem_phys_b_in, mem_phys_a_in,
            mem_immediate_in, mem_auxiliary_address_in,
            mem_load_store_function3_in, mem_is_store_in,
            mem_generation_in, mem_index_in };
    end
    always @(*) begin
        load_forward_a_mem = ld_fwd_ok & (ld_phys_early == mem_sel_phys_a);
        load_forward_b_mem = ld_fwd_ok & (ld_phys_early == mem_sel_phys_b);
        alu_lane0_forward_a_mem = alu_fwd_ok_0 & (phys_alu_early_0 == mem_sel_phys_a);
        alu_lane1_forward_a_mem = alu_fwd_ok_1 & (phys_alu_early_1 == mem_sel_phys_a);
        alu_lane0_forward_b_mem = alu_fwd_ok_0 & (phys_alu_early_0 == mem_sel_phys_b);
        alu_lane1_forward_b_mem = alu_fwd_ok_1 & (phys_alu_early_1 == mem_sel_phys_b);
        operand_a_mem = alu_lane0_forward_a_mem ? data_alu_early :
                        alu_lane1_forward_a_mem ? data_alu_lane1_early :
                        load_forward_a_mem ? data_ld_early :
                        prf_data_a_mem;
        operand_b_mem = alu_lane0_forward_b_mem ? data_alu_early :
                        alu_lane1_forward_b_mem ? data_alu_lane1_early :
                        load_forward_b_mem ? data_ld_early :
                        prf_data_b_mem;
        mem_room        = (mem_count_m != MEM_ST_DEPTH[2:0]);
        mem_issue_valid = mem_sel_found & ~(redir_backend_0 | redir_backend_1) & ~lsu_full;
        mem_pop_out     = mem_valid_in & (mem_room | mem_issue_valid) & ~(redir_backend_0 | redir_backend_1);
        mem_load        = mem_pop_out & ~decode_dead_mem;
        mem_append_m    = mem_count_m - {2'd0, mem_issue_valid};
        for (i = 0; i < MEM_ST_DEPTH; i = i + 1)
            mem_shift_m[i] = mem_issue_valid & (mem_sel <= i[MEM_AW-1:0]);
    end

//==============================================================================
//  十二之二、站阵列时序：入队 / 释放 / 本地年龄 / 常驻就绪位
//   ★ 与 lane 站同构：窗口恒连续，取走一条补一格孔，槽号 = 年龄。
//   ★ 位移那一拍搬的就绪位必须是【当拍更新后】的值（常驻位 | 当拍唤醒）——
//     直接搬 rdy[i+1] 会把正好撞在位移拍的唤醒丢掉（lane 站 seed 7 死在这里）。
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered | (redir_backend_0 | redir_backend_1)) begin
            mem_valid_m <= {MEM_ST_DEPTH{1'b0}};
            mem_count_m <= 3'd0;
        end
        else begin
            for (i = 0; i < MEM_ST_DEPTH; i = i + 1) begin
//三路优先：入站 > 从上一格搬过来 > 保持。
//  ★ i == MEM_ST_DEPTH-1 必须单独挡：越界读 iverilog 给 X、综合按 0。
                if (mem_load & (mem_append_m == i[2:0])) begin
                    mem_data_m[i]  <= mem_enq_payload;
                    mem_valid_m[i] <= 1'b1;
                end
                else if (mem_shift_m[i] & (i < MEM_ST_DEPTH-1)) begin
                    mem_data_m[i]  <= mem_data_m[i+1];
                    mem_valid_m[i] <= mem_valid_m[i+1];
                end
                else if (mem_shift_m[i]) begin
                    mem_valid_m[i] <= 1'b0;
                end
            end
            mem_count_m <= mem_count_m + {2'd0, mem_load} - {2'd0, mem_issue_valid};
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            mem_rdy_a_m <= {MEM_ST_DEPTH{1'b0}};
            mem_rdy_b_m <= {MEM_ST_DEPTH{1'b0}};
        end
        else begin
            for (i = 0; i < MEM_ST_DEPTH; i = i + 1) begin
                if (mem_load & (mem_append_m == i[2:0])) begin
                    mem_rdy_a_m[i] <= enq_ready_a_mem;
                    mem_rdy_b_m[i] <= enq_ready_b_mem;
                end
                else if (mem_shift_m[i] & (i < MEM_ST_DEPTH-1)) begin
                    mem_rdy_a_m[i] <= mem_entry_ready_a[i+1];
                    mem_rdy_b_m[i] <= mem_entry_ready_b[i+1];
                end
                else if (!mem_shift_m[i]) begin
                    mem_rdy_a_m[i] <= mem_entry_ready_a[i];
                    mem_rdy_b_m[i] <= mem_entry_ready_b[i];
                end
                else begin
                    mem_rdy_a_m[i] <= 1'b0;
                    mem_rdy_b_m[i] <= 1'b0;
                end
            end
        end
    end

//==============================================================================
// 十三、D 级"这条被冲刷杀着"的标记
//   ★ 必须是定时器的形状：{idx, gen} 会重复，认身份不可靠。置位 = 冲刷那一拍按年龄判；
//     清零 = 这条被消费（accept）那一拍。
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            dead_registered_0 <= 1'b0;
            dead_registered_1 <= 1'b0;
        end
        else if ((redir_backend_0 | redir_backend_1)) begin
            dead_registered_0 <= 1'b1;
            dead_registered_1 <= 1'b1;
        end
        else begin
            if (~stage_advance        | accept_0) dead_registered_0 <= 1'b0;
            if (~stage_advance_lane1 | accept_1) dead_registered_1 <= 1'b0;
        end
    end

//==============================================================================
// 十四、PRF 读口（地址来自 Q 里的物理号，仍是寄存器）
//==============================================================================
    assign prf_read_a_0_phys = (q_data_0[RS_PHYSA_LO +: 6]);
    assign prf_read_b_0_phys = (q_data_0[RS_PHYSB_LO +: 6]);
    assign prf_read_a_1_phys = (q_data_1[RS_PHYSA_LO +: 6]);
    assign prf_read_b_1_phys = (q_data_1[RS_PHYSB_LO +: 6]);
    assign prf_read_a_m_phys = mem_data_m[mem_sel][MEM_PHYSA_LO +: 6];
    assign prf_read_b_m_phys = mem_data_m[mem_sel][MEM_PHYSB_LO +: 6];

//==============================================================================
// 十五、出站字段
//==============================================================================
    always @(*) begin
        sel_class_0 = (q_data_0[RS_CLASS_LO +: 2]);
        issue_index_0 = (q_data_0[RS_IDX_LO   +: 5]);
        issue_generation_0 = (q_data_0[RS_GEN_LO   +: 1]);
//we 必须被 issue_v 门控：alu 的"登记判据"就是 we_in，Q 空着或没发时把它送出去，
        write_enable_0        = issue_valid_0 & (q_data_0[RS_WE_LO    +: 1]);
        alu_function4_0 = (q_data_0[RS_FUNC4_LO +: 4]);
        csr_func3_0 = (q_data_0[RS_CSRF3_LO +: 3]);
        csr_write_enable_0 = issue_valid_0 & (q_data_0[RS_CSRWE_LO +: 1]);
        csr_write_action_0 = issue_valid_0 & (q_data_0[RS_CSRWR_LO +: 1]);
//地址不乘 issue_valid：它是 csr.v 那个 12:1 case 的输入，乘进去等于把扇出拖进 CSR 锥。
        csr_address_0  = (q_data_0[RS_CSRA_LO  +: 12]);
        auxiliary_address_0  = (q_data_0[RS_AUX_LO   +: 32]);
        off_mem_0   = (q_data_0[RS_JV_LO    +: 32]);
        beq_off_0   = (q_data_0[RS_BRF_LO   +: 1])   ? (q_data_0[RS_JV_LO    +: 32]) : 32'd0;
        jump_link_register_prediction_address_0 = (q_data_0[RS_JALFR_LO +: 1]) ? (q_data_0[RS_JV_LO    +: 32]) : 32'd0;
        jump_link_target_0     = (q_data_0[RS_JALF_LO  +: 1])  ? (q_data_0[RS_JV_LO    +: 32]) : 32'd0;
        operand_data_a_0 = operand_a_0;
        operand_data_b_0 = operand_b_0;
        branch_flag_0   = (q_data_0[RS_BRF_LO   +: 1]);
        jump_link_flag_0  = (q_data_0[RS_JALF_LO  +: 1]);
        jump_link_register_flag_0 = (q_data_0[RS_JALFR_LO +: 1]);
        branch_prediction_taken_0 = (q_data_0[RS_BPRED_LO +: 1]);
        bju_function4_0         = (q_data_0[RS_BJUF4_LO +: 4]);
        bypass_load_a_0 = 1'b0;
        bypass_load_b_0 = 1'b0;
        mul_go_0 = issue_valid_0 & ((q_data_0[RS_CLASS_LO +: 2]) == CLS_MUL) & ~(q_data_0[RS_MULOP_LO + 2]);
        div_go_0 = issue_valid_0 & ((q_data_0[RS_CLASS_LO +: 2]) == CLS_MUL) &  (q_data_0[RS_MULOP_LO + 2]);
        opcode_0 = 7'd0;
        function10_0 = 10'd0;
        function10_load_store_0 = 10'd0;
        if ((q_data_0[RS_CLASS_LO +: 2]) == CLS_MUL) begin
            opcode_0  = OPCODE_OP;
            function10_0 = {7'b0000001, (q_data_0[RS_MULOP_LO +: 3])};
        end

        sel_class_1 = (q_data_1[RS_CLASS_LO +: 2]);
        issue_index_1 = (q_data_1[RS_IDX_LO   +: 5]);
        issue_generation_1 = (q_data_1[RS_GEN_LO   +: 1]);
        write_enable_1        = issue_valid_1 & (q_data_1[RS_WE_LO    +: 1]);
        alu_function4_1 = (q_data_1[RS_FUNC4_LO +: 4]);
        csr_write_enable_1 = issue_valid_1 & (q_data_1[RS_CSRWE_LO +: 1]);
        csr_write_action_1 = issue_valid_1 & (q_data_1[RS_CSRWR_LO +: 1]);
        csr_address_1  = (q_data_1[RS_CSRA_LO  +: 12]);
        auxiliary_address_1  = (q_data_1[RS_AUX_LO   +: 32]);
        off_mem_1   = (q_data_1[RS_JV_LO    +: 32]);
        beq_off_1   = (q_data_1[RS_BRF_LO   +: 1])   ? (q_data_1[RS_JV_LO    +: 32]) : 32'd0;
        jump_link_register_prediction_address_1 = (q_data_1[RS_JALFR_LO +: 1]) ? (q_data_1[RS_JV_LO    +: 32]) : 32'd0;
        jump_link_target_1     = (q_data_1[RS_JALF_LO  +: 1])  ? (q_data_1[RS_JV_LO    +: 32]) : 32'd0;
        operand_data_a_1 = operand_a_1;
        operand_data_b_1 = operand_b_1;
        branch_flag_1   = (q_data_1[RS_BRF_LO   +: 1]);
        jump_link_flag_1  = (q_data_1[RS_JALF_LO  +: 1]);
        jump_link_register_flag_1 = (q_data_1[RS_JALFR_LO +: 1]);
        branch_prediction_taken_1 = (q_data_1[RS_BPRED_LO +: 1]);
        bju_function4_1         = (q_data_1[RS_BJUF4_LO +: 4]);
        bypass_load_a_1 = 1'b0;
        bypass_load_b_1 = 1'b0;
        mul_go_1 = issue_valid_1 & ((q_data_1[RS_CLASS_LO +: 2]) == CLS_MUL) & ~(q_data_1[RS_MULOP_LO + 2]);
        div_go_1 = issue_valid_1 & ((q_data_1[RS_CLASS_LO +: 2]) == CLS_MUL) &  (q_data_1[RS_MULOP_LO + 2]);
        opcode_1 = 7'd0;
        function10_1 = 10'd0;
        function10_load_store_1 = 10'd0;
        if ((q_data_1[RS_CLASS_LO +: 2]) == CLS_MUL) begin
            opcode_1  = OPCODE_OP;
            function10_1 = {7'b0000001, (q_data_1[RS_MULOP_LO +: 3])};
        end

//---- LSU 口：只由访存站的选中项驱动 ----
        lsu_a = operand_a_mem;
        lsu_imm = mem_data_m[mem_sel][MEM_IMM_LO +: 32];
        lsu_b = operand_b_mem;
        lsu_valid = mem_issue_valid;
        lsu_lane = 1'b0;
        lsu_opcode = mem_data_m[mem_sel][MEM_ISST_LO] ? OPCODE_STORE : OPCODE_LOAD;
        lsu_function10 = {7'd0, mem_data_m[mem_sel][MEM_F3_LO +: 3]};
        lsu_index = mem_data_m[mem_sel][MEM_IDX_LO +: 5];
        lsu_phys_d = mem_data_m[mem_sel][MEM_PHYSD_LO +: 6];
        lsu_generation = mem_data_m[mem_sel][MEM_GEN_LO +: 1];
//载荷里存的是 auxiliary 约定（= 指令地址 + 4），这里减回真 pc。
        lsu_pc = mem_data_m[mem_sel][MEM_AUX_LO +: 32] - 32'd4;
    end

endmodule
