`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu1
// Description:
//   取指队列与载荷之间的寄存器级（= 原 mid_decoder 的位置），并入 rd / func10 / imm 的译码。
//
//   输入是 iffu 的出队格（F 级，原样指令字 + 随行号），输出是本级的寄存器（S 级）：
//   全译码（入站分类 / 各单元字段 / 异常标记 / 非寄存器操作数覆盖）由 S 级的指令字
//   【组合】解出、并随载荷同沿寄存进 S 级（原来这一份解在 idu2 的入站拍上）。
//
//   ★ 两套译码源不同、用途不同：
//     - 载荷面（rd/func10/imm）与全译码解在 S 级的 instruction 上，给 idu2 的入站；
//     - rob 分配口只判"这一类写不写 rd"，解在 **fifo 队头**（nh0）上 —— 入队发生在
//       "队头进 出队格"那一拍，rob 要的是那一条的 rd，不能裸取 instruction[11:7]
//       （BRANCH/STORE 会变出幽灵写者）。
//
//   ★ 扫槽口（scan_rs）给的是【本级输入那一级】（F）的 rs：rob 的读拍在本级输出（S），
//     扫描必须提前一拍（见 rob.v 的相位说明）。
//////////////////////////////////////////////////////////////////////////////////


module idu1(
    input clk, rst,
//★ 2026-10-31：不再接整根 flag_bus —— 只接本模块真正用到的三根。
//  原来的 [2] pc_redirect 与 [0] bus_hold 与本级的推进无关（前者是取指侧的事，
//  后者已挪进 lsu 的 blank_bus），留着只会让人以为它们还在这儿起作用。
    input [2:0] flush_control_in,   // = flag_bus[7:5]：{异常, 中断, 改向}
    input       exec_in,            // = flag_bus[4]
    input       stall_rob_full_in,  // = flag_bus[3]
//★ 冲刷的【寄存版】（cont 的 flush_pending，晚一拍）。只给访存 FIFO 的清零用：
//  组合版 flush_control_in 要穿过 csr 的中断锥（含 rob 的 head_valid），那条链实测 18 级、
//  终点正是 lsu_queue_*_reg 的 R 脚。清零晚一拍无害 —— 冲刷拍进队门本来就关着，
//  而冲刷之后 iffu 队头被清空、下一拍也不会有人进队。
    input       flush_pend,
//冲刷的年龄边界（与 idu2 的 kill 判据逐项同门）。输出级里挂着的那条【比冲刷点老】就不是错的，
//必须留住 —— 见下面 keep_0/keep_1 那段。
    input [1:0]  store_pop,
    input [3:0]  store_flush_pop,
    output reg   credit_stall_out,
//给 fifo 的推进量 = 本拍真正装了几条（0/1/2）。iffu 按自己手上有效条数截断后移动指针。
    output reg [1:0] advance,
//本拍 lane0 装没装：lane1 到底拿出队格的第二格(h1)还是队头(h0)由它定 ⇒ 寄存器堆读口的
//lane1 地址要跟着它选（rs 在 F 级就要发出去，见 cpu_top）
    output reg   lane0_load_o,

//============================== lane0 ==============================
//来自 iffu 出队格（F 级）
    input [31:0] instruction_in_0,
    input [31:0] address_in_0,
    input        branch_prediction_in_0,
    input [31:0] jump_link_register_prediction_in_0,
    input [3:0]  index_in_0,
    input        generation_in_0,
//出队格这一格是不是真的有效（iffu 的 h0_valid/h1_valid）。不许用 addr!=0 去推 —— 那个判据要经过
//iffu 的组合 pop/sidx 才能在载荷上看到，拿它做 load 门会和 pop 成组合环。
    input        instruction_valid_0,
//操作数就绪（rob 按冻结生产者槽现算）与发射拍被接收（idu2 把"就绪 + FU 空闲"合成这一根）。
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
    output reg [3:0]  index_out_0,
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
    output reg [21:0] alloc_csr_payload_0,
//★ 2026-11-04 真 PRF：这一条被分到的物理号，交给 rob 随项存（提交定格 / 冲刷回收都用它）
    output [5:0]      alloc_phys_0,
//★ 2026-11-04 真 PRF：给 PRF 的就绪位当门（本拍分配的号 / 冲刷后仍在用的号）
    output [63:0]     alloc_onehot_0,
    output [63:0]     arch_next_onehot_0,
//全译码（随 S 级寄存器送 idu2 的入站：分类 / 各单元字段 / 异常标记 / 非寄存器操作数覆盖）
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
    input [3:0]  index_in_1,
    input        generation_in_1,
    input        instruction_valid_1,
    input        ready1_in_1, ready2_in_1,
    input        accept_in_1,
    output reg   lane_valid_1,
    output reg [31:0] instruction_out_1,
    output reg [31:0] auxiliary_address_out_1,
    output reg        branch_prediction_taken_out_1,
    (* max_fanout = 32 *) output reg [4:0] source_a_out_1,
    output reg [4:0] source_b_out_1,
    output reg [3:0]  index_out_1,
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
    output reg [21:0] alloc_csr_payload_1,
    output [5:0]      alloc_phys_1,
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
//   队头是寄存器，组合译码正好一拍落到 idu2 的 D；操作数不在这里读，由 idu2 按物理号从 prf 取。
//★ 生产者 tag 为什么随条目走：访存条在 FIFO 里可能躺好几拍才轮到 idu2 的 D，
//  而 pstat 只保留"最年轻的写者"。等它出队时才查表，查到的是那些【比它年轻】、
//  在它排队期间已经登记的写者 ⇒ 拿错值（实测 seed8：`sw x31,116(x1)` 的 x1 被一条更年轻的
//  写者顶掉，地址算成 0x468，tohost 的写丢）。所以在【进队那一拍】就把 tag 查好、随条目带走。
//==============================================================================
    input         lsu_pop_in,
//---- 提交口（rename 的架构映射定格 / 空闲表回收）----
    input         write_enable_a_in, write_enable_b_in,
    input  [4:0]  commit_rd0_in, commit_rd1_in,
//★ 2026-11-04 真 PRF：提交项的物理号 + 冲刷掩码（在册全部项的物理号 one-hot OR）
    input  [5:0]  commit_phys0_in, commit_phys1_in,
    input  [63:0] squash_phys_mask_in,
//★ ROB 满：rename 的分配必须和 rob 的分配口【逐字同门】，否则发了号而 ROB 没接 ⇒ 漏号
    input         rob_full_in,
//---- 队头（出队侧）----
    output [31:0] lsu_instruction_out, lsu_auxiliary_address_out, lsu_immediate_out,
    output [4:0]  lsu_source_a_out, lsu_source_b_out, lsu_destination_register_out,
    output [2:0]  lsu_load_store_function3_out,
    output        lsu_is_store_out, lsu_write_enable_out,
    output [3:0]  lsu_index_out,
    output        lsu_generation_out, lsu_valid_out,
//★ 2026-11-04 真 PRF：条目里带的是两个源的【物理号】（6 位各一个，与原来的 tag 同宽）
    output [5:0]  lsu_phys_a_out, lsu_phys_b_out,
//★ 两条 lane 的载荷队列也各带两个源的物理号（idu2 的 D 级直接用）
    output reg [5:0]  phys_a_out_0, phys_b_out_0, phys_a_out_1, phys_b_out_1
    );
//★宏化旋钮：2 = 双发（大核）、1 = 单发（小核）。
//  LANES=1 时 alloc_lane1 恒 0 ⇒ iffu 每拍弹 1 条、h1 下一拍自己变队头，指令不丢；
//  lane1 的载荷/单元随之变成死逻辑，综合自动裁掉。
    parameter integer LANES = 2;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，工具只能在布局阶段自己复制。每个模块各自打一拍，
//寄存器就落在本模块旁边；全核都只打一拍，彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

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

//flush_control_in / exec_in / stall_rob_full_in 合起来 = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
    reg exec, flush_window;
//★ 两条 lane 各自的"已发射"位。lane0/lane1 的发射彼此独立之后，"寄存器内容的消费"与"寄存器装载"
//  不再同拍：lane0 先发出去、lane1 还要等它的操作数的那几拍里，lane0 的寄存器里躺着的仍是同一条
//  指令 —— 没有这一对位，它会被每拍重新发一次（同一条指令重复进 ROB）。
    reg lane0_free, lane1_free, go, load_lane0, load_lane1;

//==============================================================================
// 访存 FIFO：4 个标量（本仓惯例：数组元素在 always @(*) 里进不了敏感表）
//   位域： [80:49] instruction  [48:17] pc  [16:13] rob 索引  [12] 世代
//          [11] a 口有生产者  [10:7] a 口槽号  [6] a 口世代
//          [5]  b 口有生产者  [4:1]  b 口槽号  [0] b 口世代
//          [90:86] rs1  [85:81] rs2
//   ★ rs1/rs2 只为一件事存在：提交清 tag 时要比对 rd。只比 (槽, 世代) 会误清 —— 世代只有 1 位，
//     同一个槽转两圈就又对上了，别的指令一提交就把我的 tag 清掉（实测 seed20：`remu x28` 的 tag
//     被一条无关的提交清掉 ⇒ 尾声那条 `sw x28` 读到架构旧值 0）。
//==============================================================================
    localparam integer LSU_QUEUE_WIDTH = 91;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_queue_0, lsu_queue_1, lsu_queue_2, lsu_queue_3;
    (* max_fanout = 32 *) reg [2:0] lsu_queue_count;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_entry;
    reg        lsu_push_0, lsu_push_1;
    reg [2:0]  lsu_room;
    reg        is_mem_0, is_mem_1;
    reg        alloc_lane0, alloc_lane1, lane0_store, lane1_store;
//==============================================================================
// S 级载荷的 2 深 FIFO（2026-10-31）
//   头 = 下面那组原来的 Q 寄存器（一个字段没动，idu2 看到的端口一个字不改）
//   影子 = 这一组。推入只看本地空位（lane0_free = ~头 | ~影子），不再问 idu2 的 accept；
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
    reg [3:0]  skid_index_out_0;
    reg        skid_generation_out_0;
//★ 2026-11-04 真 PRF：与 prod 同生命周期的物理号（prod 被提交清掉时它【不清】，号是稳定的）
    reg [5:0]  skid_phys_a_out_0, skid_phys_b_out_0, skid_phys_a_out_1, skid_phys_b_out_1;
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
    reg [3:0]  skid_index_out_1;
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
    reg [3:0]  index_pick;
    reg        generation_pick;
    reg [31:0] lsu_push_instruction;
// 读写指针（2026-10-31）：移位版每拍要搬最多 3 条 91 位；指针版只让指针转，数据格不动。
    reg [1:0]  lsu_read_pointer, lsu_write_pointer;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_head;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_clear_0, lsu_clear_1, lsu_clear_2, lsu_clear_3;
    reg        lsu_tag_hit_0a, lsu_tag_hit_0b, lsu_tag_hit_1a, lsu_tag_hit_1b;
    reg        lsu_tag_hit_2a, lsu_tag_hit_2b, lsu_tag_hit_3a, lsu_tag_hit_3b;
    always @(*) begin
        flush_window = flush_control_in[2] | flush_control_in[1] | flush_control_in[0];
//★ 2026-10-31：只留 rob_full。
//  · pc_redirect：改向本身就是一次冲刷，从 iffu 到队头这一段反正全杀，
//    前端多取几条错路的没有任何代价 —— 它是取指侧的事，不属于本模块的推进。
//  · bus_hold：已挪进 lsu 的 blank_bus（只挡 LSU 发新交易）。

        exec    = exec_in;
//★ 正在发射的那条必须【同拍】算作已消费：只看寄存的 l0_con/l1_con 的话，发射那一拍两个寄存器
//  都还不算空，下一拍才装新的一对 ⇒ 每条 lane 变成两拍一条，双发射白做。
//★ 2026-10-31：载荷变成 2 深（头 + 影子）之后，"还能不能再收一条"只看【本地空位】，
//  不再问 idu2 的 accept。advance 的扇入从此只有本级寄存器。
//  ★ 这里刻意【不吃】accept：满满那一拍（头、影子都占着）即使 idu2 正在收，也只等下一拍 ——
//    这只是少收一拍，换的是整条 accept→前送/值表→lane_free 的锥不进 iffu。
        lane0_free = ~payload_valid_out_0  | ~skid_valid_0;
        lane1_free = ~payload_valid_out_1 | ~skid_valid_1;
//★ 拆站之后本级重新对"操作数没到 / FU 忙"负责：下游没有站可以挂住一条指令，S 级这对流水寄存器
//  就是它唯一的落脚点。判据不用 rdy —— rdy 与单元接受条件一起由 idu2 在【发射拍】合成 accept_in_0/accept_in_1，
//  本级只认它们：这样"操作数就绪"与"FU 空闲"落在同一拍、同一个信号上，不会差一拍。
//★ 两条 lane 各自推进：谁空出来谁装，不再成对。lane0 在等操作数时 lane1 照样从 fifo 拿一条。
//★ 2026-10-31：`~flush_window` 从这里【摘掉】。冲刷的正确语义是「从 iffu 到 ROB 队头这一整段
//  全部作废」，不是「停手」—— 冲刷拍弹出来的那 0~2 条本来就该消失，各寄存器块的最后一句
//  冲刷清会把它们抹掉（后写者赢）。ROB 侧本来就是安全的：分配口自带 `!flush_control_rob`。
//★ 2026-10-31：`flush_pend` / `trap_fire_q` 也摘掉了 —— 它们和冲刷是同一件事的两拍延迟版。
//  冲刷之后 iffu 的队头被清空（write_pointer 回卷到 read_pointer），接下来那两拍
//  `instruction_valid_*` 恒 0 ⇒ alloc 自然为 0，不需要再用两个停手位去挡。
//  剩下唯一的推进约束是 ROB 满（结构性的，没有替代品）。
        go    = exec & ~stall_rob_full_in;
//★ load_lane0/load_lane1 必须带【出队格这条是不是真的有效】。addr==0 是全核约定的"空格"（真指令的 addr
//  是"自己地址+4"，最小是 4）。少了这一项就会出现：队列里只有 1 条有效、前端还在回填时，
//  两级一起装、advance=2 ⇒ 把不存在的第 2 格也推进了 ⇒ 那条被重复取出/重复分配
//  （实测：随机差分所有种子的第一条指令都进 ROB 两次，逐条写回比对的第一处不符就落在 pc=4）。
//★ 分流（2026-10-29）：iffu 队头按这一格的 opcode 决定去向 —— 访存（LOAD/STORE）进下面的
//  LSU FIFO，其余照旧进 lane0/lane1 的 S 级。所以"能不能收这一格"要按去处分别判：
//  普通指令看 lane 空不空，访存看 FIFO 有没有位。
//  ROB 分配（alloc_en_0/1）与 advance 跟【这一格被消费了】走，不跟去处走：访存条照样占一项 ROB，
//  索引由 rob 当拍回填（index_in_0/1），随后随条目一起进 FIFO。
//★ 每拍最多进一条，而且【只从 h0 分流】（h1 是访存就等它下一拍变成 h0）。
//  为什么必须钉在 h0：进队那一拍要查 pstat 的"我的生产者"，而跟它同拍被送进 lane 的那条
//  还要**下一拍**才出现在 idu2 的 D 里（S 级差一拍）⇒ 查表时看不见它。
//  h1 当进队条时，h0 恰好是比它老、又还没登记的那一条 —— 实测 seed1：`addi x26,x26,0x640`
//  与紧随的 `sw x?,0(x26)` 同拍，store 查到的是上一条 `lui x26,0x800` ⇒ 地址少 0x640，
//  一 stores 到 tohost 上，整份签名被顶掉。
//  钉在 h0 之后，h0 与 h1 之间不存在这种"同拍、更老、未登记"的邻居：h1 永远比进队条年轻，
//  它下一拍的登记就是"更年轻的写者覆盖"，正好对。
        is_mem_0 = (instruction_in_0[6:0] == OPCODE_LOAD) | (instruction_in_0[6:0] == OPCODE_STORE);
//★ 次序别动：instruction_pick 读的是【本拍刚算出来的 alloc_lane0】。同一个 always 块里
//  先读后写不收敛，本仓踩过。
//★ 只看本地计数器：不许再依赖 idu2 当拍弹不弹（那会把 lsu 的 stage_free 拉进 iffu 的推进锥）
        lsu_room = 3'd4 - lsu_queue_count;
        alloc_lane0 = go & instruction_valid_0 & (is_mem_0 ? (lsu_room != 3'd0) : lane0_free)
                    & ~(rename_bank_empty_0 & (destination_decoded != 5'd0));

//★ `~flush_window` 挂在【落盘】上，不挂在推进上（2026-10-31）：
//  冲刷拍照样从 iffu 弹（advance 不为 0，指针照走），但弹出来的那条【不进任何队列】——
//  它本来就是错路的。ROB 的分配口自带 `!flush_control_rob`，所以这是全链一致的。
        lane0_store = alloc_lane0 & ~is_mem_0 & ~flush_window;
        lsu_push_0  = alloc_lane0 &  is_mem_0 & ~flush_window;
//★ 2 深载荷的下一态判据（组合，只在本级寄存器上）：
//  · head_after_pop = 出队（accept）之后头里还有没有东西 —— 影子有货就抬上来，所以还是"有"。
//  · push           = 本拍真的要落一条进载荷（空格 addr==0 不算）。
//  · shadow_fill_0  = lane0 这一条落进【影子】而不是头 ⇒ 它会藏在第二格里，pstat 看不见它。
//  入队次序固定在载荷块里：先出队，再入队（头空进头，否则进影子）。
        lane0_head_after_pop = (accept_in_0 & payload_valid_out_0) ? skid_valid_0 : payload_valid_out_0;
        lane1_head_after_pop = (accept_in_1 & payload_valid_out_1) ? skid_valid_1 : payload_valid_out_1;
        lane0_push = lane0_store & (address_in_0 != 32'd0);
//lane1 这一拍拿的是哪一格（见下面 instruction_stage1）：lane0 这一格被消费了就拿 h1，否则拿 h0。
        instruction_pick = alloc_lane0 ? instruction_in_1 : instruction_in_0;
        address_pick     = alloc_lane0 ? address_in_1 : address_in_0;
        index_pick       = alloc_lane0 ? index_in_1 : index_in_0;
        generation_pick  = alloc_lane0 ? generation_in_1 : generation_in_0;
        is_mem_1 = (instruction_pick[6:0] == OPCODE_LOAD) | (instruction_pick[6:0] == OPCODE_STORE);
        alloc_lane1 = (LANES == 1) ? 1'b0 :
                      (go & (alloc_lane0 ? instruction_valid_1 : instruction_valid_0)
                          & (is_mem_1 ? 1'b0 : lane1_free))
                      & ~(rename_bank_empty_1 & (destination_decoded_1 != 5'd0));

        lane1_store = alloc_lane1 & ~is_mem_1 & ~flush_window;
        lsu_push_1  = 1'b0;
//给 fifo 的推进量 = 本拍真正消费了几格（0/1/2）：指针按它走，不是"成对就 +2"。
        advance = {1'b0, alloc_lane0} + {1'b0, alloc_lane1};
        lane0_load_o = alloc_lane0;
//进队条目（每拍最多一条），tag 由 idu2 在【这一拍】查表给出。
        lsu_push_instruction    = lsu_push_0 ? instruction_in_0 : instruction_pick;
        lsu_entry = {lsu_push_instruction[19:15], lsu_push_instruction[24:20],
                     lsu_push_0 ? instruction_in_0 : instruction_pick,
                     lsu_push_0 ? address_in_0     : address_pick,
                     lsu_push_0 ? index_in_0       : index_pick,
                     lsu_push_0 ? generation_in_0  : generation_pick,
                     lsu_push_phys_a, lsu_push_phys_b};
        lane1_push = lane1_store & ((alloc_lane0 ? address_in_1 : address_in_0) != 32'd0);
    end

//★ 输出级的冲刷清必须【按年龄】，不能无条件清。
//  两条 lane 独立推进 ⇒ lane0 可以跑到一条跳转、判定、冲刷，而此时 lane1 的那条**还停在输出级**
//  （它的 Q 被一条等除法的 MUL 顶着）。那条比跳转老 ⇒ 按 ROB 年龄它不在杀集里 ⇒ 它必须活下来继续发。
//  无条件清会把它从管线上抹掉，而前端已经重定向到跳转目标 ⇒ 它再也不会被取回，
//  ROB 里那一项永远 wr=0 ⇒ 队头卡死（实测：随机差分 seed14 死锁在 pc=0x310，e8 一直不完成）。
//  判据与 idu2 的 kill_0/1 逐字同门：冲刷部分杀（rob_flush_partial=0 是陷阱/中断的整队杀）才谈年龄。
//★ 年龄判据改成查 rob 的击杀弧掩码（见 rob 端口注释）：本模块只问"我这条在不在被杀的弧上"，
//  head_ptr 不再扇进来。这里吃【当拍版】，因为 keep 要和 ROB 自己的作废同拍判定。
//★ keep_0 / keep_1 已删（2026-10-31）：rob 的 flush_partial 恒 0、kill_arc 恒 1
//  ⇒ 这两个量恒 0，冲刷拍两条 lane 一律清空，没有"留住比边界老的条目"这一档。

//★ lane1 这一拍拿 fifo 的哪一格：lane0 也在装 ⇒ 拿第二格；lane0 没装（在等操作数）⇒ 拿队头。
//  所以 lane 编号不再代表程序序 —— 程序序由随指令带下去的 rob 索引保证，这里只按"装了几条"走指针。
    reg [31:0] instruction_stage1, address_stage1, jump_link_register_stage1;
    reg [4:0]  source_a_stage1, source_b_stage1;
    reg [4:0]  source_a_dec_0, source_b_dec_0, source_a_dec_1, source_b_dec_1;
    reg [3:0]  index_stage1;
    reg        branch_prediction_stage1, generation_stage1;
    always @(*) begin
        instruction_stage1 = alloc_lane0 ? instruction_in_1 : instruction_in_0;
        address_stage1 = alloc_lane0 ? address_in_1 : address_in_0;
        jump_link_register_stage1 = alloc_lane0 ? jump_link_register_prediction_in_1 : jump_link_register_prediction_in_0;
        branch_prediction_stage1  = alloc_lane0 ? branch_prediction_in_1 : branch_prediction_in_0;
        source_a_stage1   = alloc_lane0 ? source_a_dec_1 : source_a_dec_0;
        source_b_stage1   = alloc_lane0 ? source_b_dec_1 : source_b_dec_0;
        index_stage1  = alloc_lane0 ? index_in_1 : index_in_0;
        generation_stage1  = alloc_lane0 ? generation_in_1 : generation_in_0;
    end

//本 lane 的 D 有指令、且还没被收：发射级按它给 D 级发 credit
    always @(*) begin
        lane_valid_0 = payload_valid_out_0;
        lane_valid_1 = payload_valid_out_1;
        credit_stall_out = 1'b0;
    end

//译码字段的组合解算：源是本级的输入（F 级的指令字），口径与原来那一段逐条一致。
    reg [4:0]  destination_decoded;
    reg [9:0]  function10_decoded;
    reg [31:0] immediate_decoded;
    reg        jump_decoded;
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
//  而 idu1 这里本来就有一条对同一个指令字的 opcode case（上面那段）⇒ 同一份译码做了两遍，
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
        source_a_dec_1 = 5'd0;
        source_b_dec_1 = 5'd0;
        case (instruction_in_1[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
            OPCODE_LOAD, OPCODE_STORE: source_a_dec_1 = instruction_in_1[19:15];
        endcase
        if ((instruction_in_1[6:0] == OPCODE_SYSTEM)
         && (instruction_in_1[14:12] != 3'b101)
         && (instruction_in_1[14:12] != 3'b110)
         && (instruction_in_1[14:12] != 3'b111))
            source_a_dec_1 = instruction_in_1[19:15];
        case (instruction_in_1[6:0])
            OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: source_b_dec_1 = instruction_in_1[24:20];
        endcase
    end

    reg csr_is_access_dec, csr_exists_dec, csr_ro_dec, csr_wr_dec;
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

    reg exc_illegal_dec, exception_jump_misalign_decoded;
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
                    destination_pick_decoded = destination_decoded;   //★ MUL 也要给 rd：不给 dec_rd=0 ⇒ pstat 不登记它（见 idu2 的 d_wr）
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

//非寄存器操作数的覆盖（立即数在这里并进操作数）
    reg source_a_immediate_decoded, source_b_immediate_decoded;
    reg [31:0] source_a_immediate_value_decoded, source_b_immediate_value_decoded;
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

//异常标记（ecall/ebreak/mret）
    reg exc_ecall_dec, exc_ebreak_dec, exc_irq_ret_dec;
    always @(*) begin
        exc_ecall_dec   = (instruction_in_0[6:0] == OPCODE_SYSTEM) && (instruction_in_0[31:20] == 12'h000);
        exc_ebreak_dec  = (instruction_in_0[6:0] == OPCODE_SYSTEM) && (instruction_in_0[31:20] == 12'h001);
        exc_irq_ret_dec = (instruction_in_0[6:0] == OPCODE_SYSTEM) && (instruction_in_0[14:12] == 3'b000)
                        && (instruction_in_0[31:20] == 12'h302);
    end

//bju 功能码（本级的唯一一个"这条指令在 bju 里算什么"的口）：
//原来 bju 要分别吃 func4 / br_flag / jalr_flag / 四路异常五个口，其中 func4 与这四个
//没有任何区分力（addi/beq/jalr 的 func4 都是 0000），"是不是分支"只能靠额外的类位。
//这里一次译清：非法 > ecall > ebreak > mret > 分支（带 funct3）> jal（带非对齐）> jalr。
//★ 非法排第一：非法分支/非法 jalr 的编码不许再当控制指令判，否则会一边报非法一边改向。
    reg [3:0] branch_jump_function4_decoded;
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
//与 idu1 自己的译码同源 —— 不再另解一份队头。rd 直接复用主译码的 destination_decoded（按 opcode 分类
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

//lane1 的译码（与 lane0 一模，源换成 instruction_stage1 / address_stage1 / jump_link_register_stage1）
    reg [4:0]  destination_decoded_1;
    reg [9:0]  function10_decoded_1;
    reg [31:0] immediate_decoded_1;
    reg        jump_decoded_1;
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

    reg csr_is_access_dec_1, csr_exists_dec_1, csr_ro_dec_1, csr_wr_dec_1;
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

    reg exc_illegal_dec_1, exception_jump_misalign_decoded_1;
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

    reg [1:0]  class_decoded_1;
    reg        write_enable_decoded_1, csr_write_enable_decoded_1, csr_write_commit_decoded_1;
    reg [4:0]  destination_pick_decoded_1;
    reg        is_store_decoded_1;
    reg [2:0]  multiply_operation_decoded_1, load_store_function3_decoded_1, csr_function3_decoded_1;
    reg [3:0]  function4_decoded_1;
    reg [11:0] csr_address_decoded_1;
    reg [31:0] jump_value_decoded_1;
    reg        branch_flag_decoded_1, jump_link_flag_decoded_1, jump_link_register_flag_decoded_1;
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

    reg source_a_immediate_decoded_1, source_b_immediate_decoded_1;
    reg [31:0] source_a_immediate_value_decoded_1, source_b_immediate_value_decoded_1;
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

    reg exc_ecall_dec_1, exc_ebreak_dec_1, exc_irq_ret_dec_1;
    always @(*) begin
        exc_ecall_dec_1   = (instruction_stage1[6:0] == OPCODE_SYSTEM) && (instruction_stage1[31:20] == 12'h000);
        exc_ebreak_dec_1  = (instruction_stage1[6:0] == OPCODE_SYSTEM) && (instruction_stage1[31:20] == 12'h001);
        exc_irq_ret_dec_1 = (instruction_stage1[6:0] == OPCODE_SYSTEM) && (instruction_stage1[14:12] == 3'b000)
                        && (instruction_stage1[31:20] == 12'h302);
    end

//bju 功能码（与 lane0 逐条同形）
    reg [3:0] branch_jump_function4_decoded_1;
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
        alloc_en_1   = alloc_lane1;
        alloc_rd_1   = destination_decoded_1;
        alloc_we_1   = (destination_decoded_1 != 5'd0);
        alloc_jmp_1  = jump_decoded_1;
        alloc_ecall_1   = exc_ecall_dec_1;
        alloc_ebreak_1  = exc_ebreak_dec_1;
        alloc_illegal_1 = exc_illegal_dec_1;
        alloc_mret_1    = exc_irq_ret_dec_1;
        alloc_pend_1 = (instruction_stage1[6:0] == OPCODE_LOAD) | (instruction_stage1[6:0] == 7'b0100011) |
                     ((instruction_stage1[6:0] == OPCODE_OP) && (instruction_stage1[31:25] == 7'b0000001));
    end

//★ 单一状态：pay_valid_out_* 本身就是"这条 lane 上还躺着一条没被 idu2 收走的指令"。
//  只有三种事件能改它：① 装入新指令(load*) ② 被 idu2 收走(acc_in_*) ③ 被年龄判死(flush_window & ~keep_*)。
//  不再另设 l0_con/l1_con：同一个事实存两份、各有一套更新规则，必然分叉 ——
//  实测 seed14 就是冲刷清了一份、而 idu2 早已收走另一份 ⇒ 同一条 store 被发两次 ⇒
//  SQ 里多出一个没人认领的项 ⇒ 队列占满 ⇒ 队头 pend 永不完结（死锁在 0x4bc）。
//  ★ 这一段【不能】放进下面那个 exec 门控的载荷块：acc_in_* 可能在 exec=0 的拍成立
//  （Q 空时 idu2 照样给 credit），被门掉就等于没收到确认 ⇒ 重发。
//★ payload_fresh_out_* = "这条 lane 的 D 里这一条刚装进来"（单拍脉冲）。
//  下游（idu2）靠它判"新出现"，不再比 (rob 索引, 世代)：那个二元组只有 5 位，16 槽 × 2 圈就重复；
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
            if (flush_pend) begin
                payload_valid_out_0  <= 1'b0;
                payload_fresh_out_0  <= 1'b0;
            end
            if (flush_pend) begin
                payload_valid_out_1  <= 1'b0;
                payload_fresh_out_1  <= 1'b0;
            end
        end
    end

//==============================================================================
// 访存 FIFO 的队头译码 + 时序
//==============================================================================
//队头只做字段提取，不复制主译码的大 case：LOAD/STORE 的 rs1/rs2/rd/funct3 都在固定位上。
    reg [31:0] lsu_head_instruction, lsu_head_pc;
    reg [3:0]  lsu_head_index;
    reg        lsu_head_generation;
    always @(*) begin
        lsu_head = lsu_read_pointer == 2'd0 ? lsu_queue_0 :
                   lsu_read_pointer == 2'd1 ? lsu_queue_1 :
                   lsu_read_pointer == 2'd2 ? lsu_queue_2 : lsu_queue_3;
        lsu_head_instruction = lsu_head[80:49];
        lsu_head_pc          = lsu_head[48:17];
        lsu_head_index       = lsu_head[16:13];
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
    assign lsu_valid_out                = (lsu_queue_count != 3'd0);
//★ 2026-11-04 真 PRF：条目 [11:6] = rs1 的物理号，[5:0] = rs2 的物理号（与原 tag 同宽）
    assign lsu_phys_a_out                = lsu_head[11:6];
    assign lsu_phys_b_out                = lsu_head[5:0];

//★ 2026-11-04 真 PRF：进队条目带的是两个源的【物理号】，在进队这一拍由 rename 查出。
//  查找口的驱动必须用【译码后的 rs】：LUI/AUIPC/JAL 这些没有 rs 的，
//  instruction[19:15]/[24:20] 是立即数位段，裸切片会给自己认一个假号（实测 w11）。
    wire [4:0]  lsu_push_source_a, lsu_push_source_b;
    wire [4:0]  push1_source_a, push1_source_b;
    wire [5:0]  lsu_push_phys_a, lsu_push_phys_b;
    assign lsu_push_source_a = source_a_dec_0;
    assign lsu_push_source_b = source_b_dec_0;
//★ push1 = "lane1 这一拍收的那一条"，不一定是 h1：alloc_lane0=0 时 lane1 拿的是 h0。
//  这一格必须跟着走，否则"h0 被 lane1 收"那条的源就查错了。
    assign push1_source_a    = alloc_lane0 ? source_a_dec_1 : source_a_dec_0;
    assign push1_source_b    = alloc_lane0 ? source_b_dec_1 : source_b_dec_0;
    assign lsu_push_phys_a   = lookup_a_phys;
    assign lsu_push_phys_b   = lookup_b_phys;

//★ 冲刷：整条 FIFO 直接清（rob 的 flush_partial 恒 0、kill_arc 恒 1）。
//★ 2026-11-04 真 PRF：条目里不再有"生产者 tag"要按提交清 —— 换成物理号之后，
//  值在写回那一拍就进了 PRF，谁提交都不改它。这里只剩【进队覆盖】一条路径。
    always @(posedge clk) begin
        if (reset_registered) begin
            lsu_queue_0 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_1 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_2 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_3 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_count <= 3'd0;
            lsu_read_pointer <= 2'd0;
            lsu_write_pointer <= 2'd0;
        end
        else if (flush_pend) begin
//★ 清用【寄存版】冲刷（组合版会把 csr 中断锥拉进本组寄存器）。指针与计数回 0，数据格不动。
            lsu_queue_count <= 3'd0;
            lsu_read_pointer <= 2'd0;
            lsu_write_pointer <= 2'd0;
        end
        else begin
            lsu_clear_0 = lsu_queue_0;
            lsu_clear_1 = lsu_queue_1;
            lsu_clear_2 = lsu_queue_2;
            lsu_clear_3 = lsu_queue_3;
            if (lsu_push_0 | lsu_push_1) begin
                case (lsu_write_pointer)
                    2'd0: lsu_clear_0 = lsu_entry;
                    2'd1: lsu_clear_1 = lsu_entry;
                    2'd2: lsu_clear_2 = lsu_entry;
                    default: lsu_clear_3 = lsu_entry;
                endcase
            end
            lsu_queue_0 <= lsu_clear_0;
            lsu_queue_1 <= lsu_clear_1;
            lsu_queue_2 <= lsu_clear_2;
            lsu_queue_3 <= lsu_clear_3;
            if (lsu_push_0 | lsu_push_1)
                lsu_write_pointer <= lsu_write_pointer + 2'd1;
            if (lsu_pop_in & (lsu_queue_count != 3'd0))
                lsu_read_pointer <= lsu_read_pointer + 2'd1;
            case ({lsu_push_0 | lsu_push_1, lsu_pop_in & (lsu_queue_count != 3'd0)})
                2'b10:   lsu_queue_count <= lsu_queue_count + 3'd1;
                2'b01:   lsu_queue_count <= lsu_queue_count - 3'd1;
                default: ;
            endcase
        end
    end

//==============================================================================
// lane0 载荷 = 2 深 FIFO（头 + 影子）
//   一次算下一态，次序固定为【先出队，再入队】：
//     1. 出队：accept 且头里有货 → 影子抬进头（影子空就只是清头）
//     2. 入队：lane0_push 且出队后头是空的 → 进头；否则 → 进影子
//   能不能收已在 alloc 处按本地空位判过（lane0_free = ~头 | ~影子），这里不再看 accept。
//★ 这个块刻意【不吃 exec】：accept 可以在 exec=0 的拍成立（Q 空时 idu2 照样还 credit），
//  门掉就等于把影子按在原地、跟头上那条换了序。推进本身已被 go(含 exec) 挡过，不用再挡一次。
//★ 头寄存器仍是原来那 27 个，下游 idu2 一个字不改；多出来的只有影子这一组。
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
//第二格原地不动：同样要按提交清 tag（它比队头更晚进 idu2，窗口更长）
            end
//★ 冲刷清与原来同门（装载赢）：有效位在相邻那一块已经清掉，影子这里只要清 skid_valid ——
//  数据留着没人读（头有效位=0 时下游一个字都不看，与原来留脏头的做法一致）。
            if (flush_pend & ~lane0_store) skid_valid_0 <= 1'b0;
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
                phys_a_out_1 <= lookup_c_phys;
                phys_b_out_1 <= lookup_d_phys;
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
                skid_phys_a_out_1 <= lookup_c_phys;
                skid_phys_b_out_1 <= lookup_d_phys;
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
            if (flush_pend & ~lane1_store) skid_valid_1 <= 1'b0;
        end
    end

//==============================================================================
// 真 PRF 的重命名表（2026-11-04）
//   rat/arch_map/free_vector 都在 rename 里。这里只用它算两件事：
//   ① 本条指令 rd 分到的物理号（交给 rob 随项存）
//   ② rs1/rs2 翻译成的物理号（下一步替换 idu2 的 (槽,世代) 生产者表时用）
//   分配点选在 idu1 的解码/分配拍 —— 和 rob 的槽分配是同一拍、同一条指令。
//==============================================================================
    wire [5:0] rename_phys_rd_0, rename_phys_rd_1;
    wire [5:0] rename_phys_rs1_0, rename_phys_rs2_0;
    wire [5:0] rename_phys_rs1_1, rename_phys_rs2_1;
    wire       rename_stall_0, rename_stall_1;
    wire       rename_bank_empty_0, rename_bank_empty_1;
    wire [5:0] lookup_a_phys, lookup_b_phys, lookup_c_phys, lookup_d_phys;
    wire [63:0] rename_alloc_onehot, rename_arch_next_onehot;

    rename u_rename (
        .clk(clk), .rst(rst),
        .alloc_en_0(alloc_lane0 & ~rob_full_in), .alloc_we_0(destination_decoded != 5'd0), .alloc_rd_0(destination_decoded),
        .alloc_rs1_0(source_a_dec_0), .alloc_rs2_0(source_b_dec_0),
        .phys_rd_0(rename_phys_rd_0), .phys_rs1_0(rename_phys_rs1_0), .phys_rs2_0(rename_phys_rs2_0),
        .alloc_en_1(alloc_lane1 & ~rob_full_in), .alloc_we_1(destination_decoded_1 != 5'd0), .alloc_rd_1(destination_decoded_1),
        .alloc_rs1_1(source_a_dec_1), .alloc_rs2_1(source_b_dec_1),
        .phys_rd_1(rename_phys_rd_1), .phys_rs1_1(rename_phys_rs1_1), .phys_rs2_1(rename_phys_rs2_1),
        .alloc_stall_0(rename_stall_0), .alloc_stall_1(rename_stall_1),
        .bank_empty_0(rename_bank_empty_0), .bank_empty_1(rename_bank_empty_1),
        .lookup_a(lsu_push_source_a), .lookup_b(lsu_push_source_b),
        .lookup_c(push1_source_a),              .lookup_d(push1_source_b),
        .lookup_a_phys(lookup_a_phys), .lookup_b_phys(lookup_b_phys),
        .lookup_c_phys(lookup_c_phys), .lookup_d_phys(lookup_d_phys),
        .commit_we_a(write_enable_a_in), .commit_rd_a(commit_rd0_in), .commit_phys_a(commit_phys0_in),
        .commit_we_b(write_enable_b_in), .commit_rd_b(commit_rd1_in), .commit_phys_b(commit_phys1_in),
        .flush(|flush_control_in),
        .squash_mask(squash_phys_mask_in),
        .alloc_onehot(rename_alloc_onehot), .arch_next_onehot(rename_arch_next_onehot)
    );

    assign alloc_phys_0 = rename_phys_rd_0;
    assign alloc_onehot_0 = rename_alloc_onehot;
    assign arch_next_onehot_0 = rename_arch_next_onehot;
    assign alloc_phys_1 = rename_phys_rd_1;

//==============================================================================
//  CSR 随项载荷（2026-11-04，CSR 步骤 C：读-改-写搬出执行级）
//  ★ 为什么放在文件末尾：lookup_*_phys 的声明在 rename 例化旁边（本文件一贯的声明顺序），
//    写在前面会变成"先使用后声明"（iverilog 直接当错误）。
//  ★ 源字段两种形态：csrrwi/rsrsi/rrci 的 rs1 字段是 zimm，idu1 已经按立即数解释
//    （source_a_immediate_*），这里就填 zimm；其余填 rs1 的物理号。
//    提交口用 funct3[2] 二选一 —— 1xx 恰好就是立即数形式。
//  ★ 寄存器形式下 rs1=x0 ⇔ 物理号 0（0 号永不写入）⇒ 提交口可以用"源字段 != 0"重算
//    csr_wr_dec 那个"源非 0 才写"的判据，不必再单独带一根。
//==============================================================================
    always @(*) begin
        alloc_csr_0 = csr_write_enable_decoded;
        alloc_csr_payload_0 = {csr_write_commit_decoded, csr_address_decoded, csr_function3_decoded,
                               source_a_immediate_decoded ? {1'b0, source_a_immediate_value_decoded[4:0]}
                                                          : lookup_a_phys};
        alloc_csr_1 = csr_write_enable_decoded_1;
        alloc_csr_payload_1 = {csr_write_commit_decoded_1, csr_address_decoded_1, csr_function3_decoded_1,
                               source_a_immediate_decoded_1 ? {1'b0, source_a_immediate_value_decoded_1[4:0]}
                                                            : lookup_c_phys};
    end

endmodule
