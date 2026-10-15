`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu2 —— 发射单元（issue unit）：入站取值 + 四类统一保留站
//   由 post_decoder 改造而来。原"载荷寄存器"（一份）换成【入站两槽（A/B）+ 站阵列】。
//
// ★ 入站级为什么是 2 深（1 深结构结构性只能 1 条/2 拍）：
//   取值（cap_op1/cap_op2/w1/w2）必须落在【它自己到达后的第 1 拍】—— regfile 是寄存读，
//   r1_data(n+1) 描述的正是"到达于 n 的那条"；而且结果口的广播只在单元完成那一拍在线上，
//   取值/压站晚一拍就会错过广播、项被压成 wait=1 而此后没有唤醒源。
//   1 深结构里"取值那一拍"与"压站那一拍"共用同一个寄存器组 ⇒ 压站拍不能再收新到达
//   ⇒ 只有 1 条/2 拍（实测前端交付率 0.475、站里 84% 的拍只有 0~1 项、真 IPC 0.455）。
//   拆成两槽后：每槽【到达那一拍装载、到达+1 拍取值并压站】，被压的那个槽与收新到达的槽
//   不是同一组寄存器 ⇒ 交付率与推送率都能到 1 拍/条，而【取值相位与 1 深版逐字相同】。
//   ⇒ 压站只能落在"该槽到达+1 拍"：压站晚一拍，项要多等一拍才进站，中间那一拍里单元
//     结果口的广播既赶不上本站的取值、又赶不上站内唤醒 ⇒ 项被压成 wait=1 却再无唤醒源。
//
// ★ 三个本质差别（相对 post_decoder）：
//   ① 入站门里【没有 rdy1_in & rdy2_in】—— 那是"原地等"的本体。操作数没齐不再停整条流水线，
//      把这条留在站里等（w=1）。
//   ② 操作数在【槽取值那一拍】定型：源就绪 ⇒ 直接锁值（forw 的选择逻辑在这里）；
//      未就绪 ⇒ 锁生产者槽号 + 世代，等三个结果口的 idx/gen 广播唤醒。出站时项里已是终值。
//   ③ 出站口内部两级：拍 N 按【年龄最小 且 满足顺序约束】挑一条并锁 sel_slot_q；拍 N+1 用该
//      槽号索引阵列读操作数/字段、驱动四个单元并释放该槽。单发射，每拍最多一条。
//
// ★ 每槽各自存一份载荷/译码/标签，为什么不能共用：
//   两槽的到达拍不同 ⇒ "到达+1 拍"也不同。取值梯要按【正被压的那一槽】选源，给 rob 的读回
//   索引（pay_idx_out）也要指向那一槽自己的 idx —— rob 的 rdy/pay_done/pay_data 是"按 pay_idx
//   描述那一条"的，指错槽就取错生产者的状态。载荷字段也一律按每槽自己的到达拍锁存。
//
// ★ 站深为什么是 6 而不是 4：
//   ROB 在【出队那一拍】发号（alloc_en = idu1.adv），指令要 3 拍后才 push
//   （alloc → fifo h0 → idu1 输出 → 入站槽 → 站）。
//   "站满"必须把【已分配但还没入站】的在途条数算进去（pending_cnt），稳态下 occupancy+inf ≈ 5；
//   深度 4 会一直假满把前端停死，深度 6 才有真缓冲。
//
// ★ 顺序约束：lsu 互相按序 · bju 后方的 lsu 不得放行 · bju 自身按序 · CSR 类相对全体按序。
//
// ★ 冲刷清项：直接复用 rob 的窗口判据（rob.flush_part），不在站里重推一遍。
//   入站两槽各按【自己那个 idx】判一次；全冲 / trap 两槽一起清。
//////////////////////////////////////////////////////////////////////////////////

module idu2(
    input clk, rst,
    input [7:0] flag_bus,
    input        s_advance,
    input        s_advance1,
    input        pay_v1_in,

//---- 前端（idu1 输出）----
    input [4:0]  rs1_in, rs2_in,
    input [31:0] inst_in,
    input [31:0] pc_operand_in,
    input [31:0] aux_addr_in,
    input        br_pred_taken_in,
    input [3:0]  idx_in,
    input        alloc_gen_in,
    input [4:0]  rs1_in1, rs2_in1,
    input [31:0] inst_in1,
    input [31:0] pc_operand_in1,
    input [31:0] aux_addr_in1,
    input        br_pred_taken_in1,
    input [3:0]  idx_in1,
    input        alloc_gen_in1,
//---- 前端（idu1 S 级寄存的全译码字段）----
    input [1:0]  dec_class,
    input        dec_we,
    input [4:0]  dec_rd,
    input        dec_is_st,
    input [2:0]  dec_mulop,
    input [2:0]  dec_lsf3,
    input [3:0]  dec_func4,
    input [2:0]  dec_csrf3,
    input        dec_csrw_en,
    input        dec_csrw_armed,
    input        dec_csrw_wr,
    input [11:0] dec_csra,
    input [31:0] dec_jval,
    input        dec_brf,
    input        dec_jalf,
    input        dec_jalfr,
    input        dec_exc_illegal,
    input        dec_exc_jmis,
    input        dec_exc_ecall,
    input        dec_exc_ebreak,
    input        dec_exc_irq_ret,
    input        dec_r1_imm,
    input        dec_r2_imm,
    input [31:0] dec_r1_immv,
    input [31:0] dec_r2_immv,
    input [1:0]  dec_class1,
    input        dec_we1,
    input [4:0]  dec_rd1,
    input        dec_is_st1,
    input [2:0]  dec_mulop1,
    input [2:0]  dec_lsf31,
    input [3:0]  dec_func41,
    input [2:0]  dec_csrf31,
    input        dec_csrw_en1,
    input        dec_csrw_armed1,
    input        dec_csrw_wr1,
    input [11:0] dec_csra1,
    input [31:0] dec_jval1,
    input        dec_brf1,
    input        dec_jalf1,
    input        dec_jalfr1,
    input        dec_exc_illegal1,
    input        dec_exc_jmis1,
    input        dec_exc_ecall1,
    input        dec_exc_ebreak1,
    input        dec_exc_irq_ret1,
    input        dec_r1_imm1,
    input        dec_r2_imm1,
    input [31:0] dec_r1_immv1,
    input [31:0] dec_r2_immv1,

//---- 入站操作数（regfile 寄存读）：与到达同拍（读口地址 = iffu 的出队格）----
    input [31:0] r1_data, r2_data, r3_data, r4_data,

//---- rob：扫描槽 / 就绪 / 生产者世代 ----
    input [3:0]  fwd_slot1_in, fwd_slot2_in,
    input        fwd_hit1_in,  fwd_hit2_in,
    input        fwd_pg1_in,   fwd_pg2_in,
    input        rdy1_in, rdy2_in,
//被压那一槽的前送值（rob 出）：与 rdy*_in 同一份判据算出来的 done 与其配对的值。
//★ 入站取值必须带上这一路：生产者已 done 但还没提交时，阵列读还是老值、结果口也不带它了，
//  只有这一路能拿到真值（实测 x2 差 0xb8 那条）。
    input        pay_done1_in, pay_done2_in,
    input [31:0] pay_data1_in, pay_data2_in,
    input [3:0]  fwd_slot1_in1, fwd_slot2_in1,
    input        fwd_hit1_in1,  fwd_hit2_in1,
    input        fwd_pg1_in1,   fwd_pg2_in1,
    input        rdy1_in1, rdy2_in1,
    input        pay_done1_in1, pay_done2_in1,
    input [31:0] pay_data1_in1, pay_data2_in1,

//---- prf 读口数据（地址在选优拍给、值在发射拍可用；prf 内含写穿）----
    input [31:0] rdata1_in, rdata2_in, rdata3_in, rdata4_in,
//---- 三个结果口：站内唤醒 + 出站旁路 + prf 在飞半区写 ----
    input        we_alu, input [3:0] idx_alu, input gen_alu, input [31:0] data_alu,
    input        we_alu2, input [3:0] idx_alu2, input gen_alu2, input [31:0] data_alu2,
    input        we_mul, input [3:0] idx_mul, input gen_mul, input [31:0] data_mul,
    input        we_ld,  input [3:0] idx_ld,  input gen_ld,  input [31:0] data_ld,
//---- 提交口（读口旁路的最后一档：刚落进写口、阵列还没写那一笔）----
    input        we_a, input [4:0] rd_a, input [31:0] data_a,
    input        we_b, input [4:0] rd_b, input [31:0] data_b,

//---- 冲刷 / 陷阱 / 顺序 ----
    input        flush_con_rob,
    input        rob_flush_part,
    input        flush_incl,
    input [3:0]  flush_idx,
    input        trap_fire,
    input [3:0]  head_ptr,
    input        lsu_full_in,
//mdu 的两条 ready：乘法项看 mul_ready_in、除法项看 div_ready_in（按站项自己的算子门控）
    input        mul_ready_in, div_ready_in,
    input        mul_wb_nx,
    input [3:0]  mul_nx_idx,
    input        mul_nx_gen,
    input        ld_wb_nx,
    input [3:0]  ld_nx_idx,
    input        ld_nx_gen,

//---- 给 rob 的入站（分配）使能 ----
    output       alloc_en,
    output [3:0] pay_idx_out,
    input        alloc_rob_in,
    input        alloc_rob_in1,

//---- 出站：驱动四个单元的输入口 ----
    output       issue_v,
    output [3:0] head_idx_o,
    output [31:0] head_pc_o,
    output       head_v_o,
    output       has_older_bju_sel_o,
//mdu 两条接收口的 valid（一拍只出一条，所以两者至多一个为 1）
    output       mul_go, div_go,
    output [31:0] r1_final_out, r2_final_out,
    output       issue_v1,
    output [31:0] r1_final_out1, r2_final_out1,
    output reg [4:0]  rd_out1,
    output reg [3:0]  issue_idx1,
    output reg        issue_gen1,
    output reg [3:0]  alu_func4_1,
    output reg        we1,
    output       bju_in_flight,
    output reg [4:0]  rd_out,
    output reg [3:0]  issue_idx,
    output reg        issue_gen,
    output reg [6:0]  opc_out,
    output reg [9:0]  fn10_out,
    output reg [9:0]  fn10_ls_out,
    output reg [31:0] off_mem_out,
    output reg [4:0]  rs1_out, rs2_out,
    output reg [3:0]  alu_func4,
    output reg [2:0]  csr_func3,
    output reg        we, csr_wr_en,
    output reg        csr_wr_act,
    output reg [11:0] csr_addr,
    output reg [31:0] aux_addr_out,
    output reg [31:0] beq_off_q2,
    output reg [31:0] jalr_pred_addr_out,
    output reg [31:0] jal_target_out,
    output reg        br_flag, jal_flag, jalr_flag, br_pred_taken_out,
    output reg        exc_irq_ret, exc_ecall, exc_ebreak,
    output reg        exc_jal_misalign_out, exc_illegal_out,

//---- 给 prf 的读口地址（选优拍组合给出；值为 ent_ps*[sel_slot]）----
    output [5:0]  raddr1_out, raddr2_out, raddr3_out, raddr4_out,

//---- 状态 / 组合译码 ----
    output reg        rs_full,
    output reg [3:0]  occupancy
    );

//==============================================================================
// 零、控制位 / 复位
//==============================================================================
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    always @(*) begin
        flush_con_exc   = flag_bus[7];
        flush_con_irq   = flag_bus[6];
        flush_con_jump  = flag_bus[5];
        exec            = flag_bus[4];
    end
    reg flush_w;
    always @(*) flush_w = flush_con_exc | flush_con_irq | flush_con_jump;

    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;

    localparam CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;

    localparam DEPTH = 8;

    reg [2:0] sel_slot_q;
    reg       sel_vld_q;
    reg [2:0] sel_class_q;
    reg       sel_is_div_q;
    reg       sel_irq_ret_q;
    reg [5:0]  sel_ps1_q, sel_ps2_q, sel_ps1b_q, sel_ps2b_q;
    reg [31:0] sel_aux_q;
    reg [2:0] sel_slot1_q;
    reg       sel_vld1_q;
    wire       issue_v0 = issue_v;
    wire [2:0] sel_slot0_q = sel_slot_q;

//==============================================================================
// 一、JAL 目标（压站锥上组合算）：源是【到达那一拍】的输入（*_in）
//==============================================================================
    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction
    reg [31:0] jal_target_now;
    always @(*) jal_target_now = pc_operand_in + immJ(inst_in) - 32'd4;

    reg [31:0] jal_target_now_1;
    always @(*) jal_target_now_1 = pc_operand_in1 + immJ(inst_in1) - 32'd4;

//==============================================================================
// 二、入站：到达那一拍就地取值、就地压站（读口地址提前到 iffu 的出队格）
//==============================================================================
//★ 取值梯的档序：立即数 → rob 项里那份值（done）→ 三个结果口（idx+世代都比）→
//  提交口 rd 匹配（只在【本条没有在飞生产者】时用）→ 阵列读。
//  结果口那三支必须比世代：只比槽号的话，槽复用后会把别人的结果当成生产者的值。
//  而且它必须在本拍认出来 —— 广播只在完成那一拍在线上，认不出就永远等不到唤醒。
//  提交口那一档兜的是"生产者恰在本拍退休"：阵列要到下一拍才被写、它的 ROB 项本拍已经撤了，
//  只有写口上还留着那份值。有在飞生产者时不能取它 —— 那份是更老的生产者的值。
    reg [31:0] cap_op1, cap_op2;
    always @(*) begin
        cap_op1 = dec_r1_immv;
        cap_op2 = dec_r2_immv;
    end

    reg [31:0] cap_op1_1, cap_op2_1;
    always @(*) begin
        cap_op1_1 = dec_r1_immv1;
        cap_op2_1 = dec_r2_immv1;
    end

    reg w1_now, w2_now;
    always @(*) begin
        w1_now = ~dec_r1_imm & ~rdy1_in;
        w2_now = ~dec_r2_imm & ~rdy2_in;
    end

    reg w1_now_1, w2_now_1;
    always @(*) begin
        w1_now_1 = ~dec_r1_imm1 & ~rdy1_in1;
        w2_now_1 = ~dec_r2_imm1 & ~rdy2_in1;
    end

//给 rob 的读回索引：就是【本拍要压站那一条】自己的号（它 = idu1 输出级那一条的号）
    assign pay_idx_out = idx_in;

    reg        src_en0, src_en1, push_ok0, push_ok1, push_en0, push_en1;
    reg [2:0]  pending_cnt;
    reg [3:0]  free_slot0, free_slot1;
    reg        have_free_slot, have_free_slot_2;
    reg [4:0]  flush_part_age;
    always @(*) flush_part_age = {1'b0, (flush_idx - head_ptr)};

//★ 背压只由【寄存器】决定（不许出 advance ← rs_full ← push_en ← advance 这条环）：
//  站真满 / 没有空槽 / 已分配未入站在途数 —— 三项都不是 push_en 自己的函数。
//★ push_en ≡ src_en（= idu1 的 advance）：这一条压不下去就必须把 advance 挡掉，
//  否则它既没进站、又被 idu1 的输出覆盖 = 丢条。
    reg [3:0] occ_live;
    always @(*) occ_live = occupancy;
    always @(*) begin
        src_en0 = s_advance;
        src_en1 = s_advance1;
        rs_full = ((occ_live + {1'b0, pending_cnt} + (pay_v1_in ? 4'd1 : 4'd0) + 4'd1) > 4'd8)
                  | ~have_free_slot | (pay_v1_in & ~have_free_slot_2);
        push_ok0 = src_en0 & exec & ~flush_w
                  & ~((occ_live + {1'b0, pending_cnt} + (pay_v1_in ? 4'd1 : 4'd0) + 4'd1) > 4'd8)
                  & have_free_slot;
        push_ok1 = src_en1 & exec & ~flush_w
                  & ~((occ_live + {1'b0, pending_cnt} + (pay_v1_in ? 4'd1 : 4'd0) + 4'd1) > 4'd8)
                  & have_free_slot_2;
        push_en0 = push_ok0;
        push_en1 = push_ok1;
    end


//==============================================================================
// 三、站阵列（深度 DEPTH）：深度 6 的理由见文件头
//==============================================================================
    assign alloc_en = src_en0;
//★ 这里【没有】rdy1_in & rdy2_in —— 那就是"原地等"，本次改造要拿掉的东西。

    reg        ent_valid   [0:DEPTH-1];
    reg [1:0]  ent_class [0:DEPTH-1];
    reg [3:0]  ent_idx   [0:DEPTH-1];
    reg        ent_gen   [0:DEPTH-1];
    reg        ent_we  [0:DEPTH-1];
    reg [4:0]  ent_rd  [0:DEPTH-1];
    reg        ent_is_store  [0:DEPTH-1];
    reg [2:0]  ent_mul_op  [0:DEPTH-1];
//M 类的算子家族：funct3[2]==1 ⇒ 除法家族（出站时分别门控 mdu 两条 ready 用）
    reg        ent_is_div  [0:DEPTH-1];
    reg [2:0]  ent_ls_func3 [0:DEPTH-1];
    reg [3:0]  ent_alu_func4  [0:DEPTH-1];
    reg [2:0]  ent_csr_func3[0:DEPTH-1];
    reg        ent_csr_wr_en[0:DEPTH-1];
    reg        ent_csr_armed[0:DEPTH-1];
    reg        ent_csr_wr_act[0:DEPTH-1];
    reg [11:0] ent_csr_addr[0:DEPTH-1];
    reg [31:0] ent_jmp_val  [0:DEPTH-1];
    reg [31:0] ent_aux_addr [0:DEPTH-1];
    reg        ent_real   [0:DEPTH-1];
    reg        ent_br_flag [0:DEPTH-1];
    reg        ent_jal_flag[0:DEPTH-1];
    reg        ent_jalr_flag[0:DEPTH-1];
    reg        ent_br_pred_taken[0:DEPTH-1];
    reg        ent_exc_illegal [0:DEPTH-1];
    reg        ent_exc_ecall[0:DEPTH-1];
    reg        ent_exc_ebreak[0:DEPTH-1];
    reg        ent_exc_irq_ret[0:DEPTH-1];
    reg        ent_exc_jal_misalign[0:DEPTH-1];
    reg [31:0] ent_op1 [0:DEPTH-1];
    reg [31:0] ent_op2 [0:DEPTH-1];
    reg        ent_wait1  [0:DEPTH-1];
    reg        ent_wait2  [0:DEPTH-1];
    reg [5:0]  ent_ps1  [0:DEPTH-1];
    reg [5:0]  ent_ps2  [0:DEPTH-1];
    reg        ent_imm1 [0:DEPTH-1];
    reg        ent_imm2 [0:DEPTH-1];
//结果口 / 发射拍的身份 tag：bit5=1（在飞物理寄存器），低 5 位 = {ROB 槽号, 世代}
    wire [5:0] pt_alu  = {1'b1, idx_alu,  gen_alu};
    wire [5:0] pt_alu2 = {1'b1, idx_alu2, gen_alu2};
    wire [5:0] pt_mul  = {1'b1, idx_mul,  gen_mul};
    wire [5:0] pt_ld   = {1'b1, idx_ld,   gen_ld};
    wire [5:0] pt_mulnx = {1'b1, mul_nx_idx, mul_nx_gen};
    wire [5:0] pt_ldnx  = {1'b1, ld_nx_idx,  ld_nx_gen};
    wire [5:0] pt_is0  = {1'b1, issue_idx,  issue_gen};
    wire [5:0] pt_is1  = {1'b1, issue_idx1, issue_gen1};

    reg [3:0] rank_new0, rank_new1;
    always @(*) begin
        rank_new0 = occupancy - (issue_v ? 4'd1 : 4'd0) - (issue_v1 ? 4'd1 : 4'd0);
        rank_new1 = rank_new0 + 4'd1;
    end

    integer i, j;
    always @(*) begin
        free_slot0 = (rank_new0 > 4'd7) ? 4'd7 : rank_new0;
        free_slot1 = (rank_new1 > 4'd7) ? 4'd7 : rank_new1;
        have_free_slot   = (occ_live < 4'd8);
        have_free_slot_2 = (occ_live < 4'd7);
    end
    reg [2:0] pending_inc, pending_dec;
    always @(*) begin
        pending_inc = 3'd0;
        if (alloc_rob_in & (pending_cnt + pending_inc < 3'd4))  pending_inc = pending_inc + 3'd1;
        if (alloc_rob_in1 & (pending_cnt + pending_inc < 3'd4)) pending_inc = pending_inc + 3'd1;
        pending_dec = 3'd0;
        if (push_en0) pending_dec = pending_dec + 3'd1;
        if (push_en1) pending_dec = pending_dec + 3'd1;
    end
    reg [2:0] pending_nx;
    always @(*) begin
        pending_nx = pending_cnt + pending_inc;
        if (pending_nx > 3'd4) pending_nx = 3'd4;
        if (pending_nx >= pending_dec) pending_nx = pending_nx - pending_dec;
        else pending_nx = 3'd0;
    end

//---- 唤醒（组合）：结果口广播按 idx+gen 双匹配 ----
    reg [DEPTH-1:0] wake1, wake2, wake1p, wake2p;
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            wake1[i] = ent_wait1[i] & ((we_alu  & (pt_alu  == ent_ps1[i])) |
                               (we_alu2 & (pt_alu2 == ent_ps1[i])) |
                               (we_mul  & (pt_mul  == ent_ps1[i])) |
                               (we_ld   & (pt_ld   == ent_ps1[i])));
            wake2[i] = ent_wait2[i] & ((we_alu  & (pt_alu  == ent_ps2[i])) |
                               (we_alu2 & (pt_alu2 == ent_ps2[i])) |
                               (we_mul  & (pt_mul  == ent_ps2[i])) |
                               (we_ld   & (pt_ld   == ent_ps2[i])));
            wake1p[i] = ent_wait1[i] & ((mul_wb_nx & (pt_mulnx == ent_ps1[i])) |
                               (ld_wb_nx  & (pt_ldnx  == ent_ps1[i])));
            wake2p[i] = ent_wait2[i] & ((mul_wb_nx & (pt_mulnx == ent_ps2[i])) |
                               (ld_wb_nx  & (pt_ldnx  == ent_ps2[i])));
        end
    end

//---- 唤醒后的下一值（组合）：位置搬迁时把它一起带走 ----
    reg        nx_wait1 [0:DEPTH-1];
    reg        nx_wait2 [0:DEPTH-1];
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            nx_wait1[i]= ent_wait1[i];
            if (wake1[i] | wake1p[i]) nx_wait1[i] = 1'b0;
            nx_wait2[i]= ent_wait2[i];
            if (wake2[i] | wake2p[i]) nx_wait2[i] = 1'b0;
        end
    end

//---- 年龄（取队头，服务冲刷边界）----
    reg [4:0] ent_age [0:DEPTH-1];
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1)
            ent_age[i] = {1'b0, (ent_idx[i] - head_ptr)};
    end

//---- 顺序位：位置即年龄 ⇒ "存在更老的 X" = 位置前缀里有没有 X（读寄存器的静态前缀树）----
    reg [DEPTH-1:0] ent_has_older_any, ent_has_older_lsu, ent_has_older_bju, ent_has_older_st;
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            ent_has_older_any[i] = 1'b0;
            ent_has_older_lsu[i] = 1'b0;
            ent_has_older_bju[i] = 1'b0;
            ent_has_older_st[i]  = 1'b0;
            for (j = 0; j < DEPTH; j = j + 1) begin
                if (j < i) begin
                    ent_has_older_any[i] = ent_has_older_any[i] | ent_valid[j];
                    ent_has_older_lsu[i] = ent_has_older_lsu[i] | (ent_valid[j] & (ent_class[j] == CLS_LSU));
                    ent_has_older_bju[i] = ent_has_older_bju[i] | (ent_valid[j] & (ent_class[j] == CLS_BJU));
                    ent_has_older_st[i]  = ent_has_older_st[i]  | (ent_valid[j] & (ent_class[j] == CLS_LSU) & ent_is_store[j]);
                end
            end
        end
    end

//---- bju 单元在途：只剩 DIV 类吃 bju_pending 这道门；LSU 类吃 bju_issuing_now ----
    reg [1:0] bju_shift;
    reg bju_pending;
    reg       sel_go;
    reg       res_ok_q;
    reg       sel_go_irqret;
    reg       issue_v_irqret;
    reg       bju_issuing_now;
    always @(*) begin
        sel_go          = sel_vld_q & exec & ~flush_w;
        res_ok_q        = 1'b1;
        if (sel_class_q == CLS_LSU) res_ok_q = ~lsu_full_in;
        if (sel_class_q == CLS_MUL) res_ok_q = sel_is_div_q ? div_ready_in : mul_ready_in;
        sel_go_irqret   = sel_vld_q & exec & ~flush_con_exc & ~flush_con_jump;
        issue_v_irqret  = sel_go_irqret;
        bju_issuing_now = sel_go & (sel_class_q == CLS_BJU);
        bju_pending     = bju_shift[0] | bju_shift[1] | bju_issuing_now;
    end
    assign bju_in_flight = bju_shift[0];

//==============================================================================
// 四、候选 / 选优 / 输出 mux
//==============================================================================
    reg [DEPTH-1:0] ent_issuable;
    reg       have_sel;
    reg [2:0] sel_slot;

    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1) begin
//★ 早唤醒：本拍发射的那条（we/we1，下一拍必出结果、结果写进 prf[它的 tag]）本拍就能唤醒消费者。
//★ 这两路是【单操作数的就绪项】，必须并进各自那一项里 —— 整项 OR 会让"只有一个操作数到"的项也发出去。
            ent_issuable[i] = ent_valid[i]
                & (~ent_wait1[i] | wake1[i] | wake1p[i] | (we & (pt_is0 == ent_ps1[i])) | (we1 & (pt_is1 == ent_ps1[i])))
                & (~ent_wait2[i] | wake2[i] | wake2p[i] | (we & (pt_is0 == ent_ps2[i])) | (we1 & (pt_is1 == ent_ps2[i])));
            if (ent_class[i] == CLS_LSU)
                ent_issuable[i] = ent_issuable[i]
                         & (ent_is_store[i] ? (~ent_has_older_lsu[i] & ~ent_has_older_bju[i])
                                            : ~ent_has_older_st[i])
                         & ~bju_issuing_now & ~lsu_full_in;
            if (ent_class[i] == CLS_BJU)
                ent_issuable[i] = ent_issuable[i] & ~ent_has_older_bju[i];
//M 类：按算子分两条 ready 门控（乘法家族看 mul_ready_in、除法家族看 div_ready_in）——
//查的是站项自己的寄存器字段，不经过出口 mux ⇒ 不闭零延时环。
            if (ent_class[i] == CLS_MUL)
                ent_issuable[i] = ent_issuable[i] & (ent_is_div[i] ? div_ready_in : mul_ready_in);
//★ 【除法类的入队守卫】：与 CLS_LSU 同一套顺序判据，不许越到更老的未决分支前面去。
//  分支误预测的冲刷边界就是那条分支自己 ⇒ 在途的除法若比它年轻，就会被同一次冲刷作废，
//  而它的完成口还在单元里（32 拍迭代 + 提交链）⇒ 晚回来时槽已住上别人、世代又翻回同值，
//  于是砸错人（实测：一条 div 被冲掉后，它的完成口写进了另一条 remu 的项）。
//  挡这道门之后：已发起的除法恒老于任何【后续】冲刷边界 ⇒ 分支冲刷杀不到它。
//  乘法不加：两级就出结果，完成口不会跨过冲刷窗口迟到。
            if ((ent_class[i] == CLS_MUL) && ent_is_div[i])
                ent_issuable[i] = ent_issuable[i] & ~ent_has_older_bju[i] & ~bju_pending;
            if (ent_csr_armed[i])
                ent_issuable[i] = ent_issuable[i] & ~ent_has_older_any[i];
            ent_issuable[i] = ent_issuable[i] & ~(issue_v & (i[2:0] == sel_slot_q)) & ~(issue_v1 & (i[2:0] == sel_slot1_q));
        end
    end

//★ 站里【最老有效项】（不是"最老可发"）：中断的返回点与它的冲刷边界都以它为准 ——
//  "最老可发"会跳过正在等操作数的那条，于是那条既被冲掉、又不在 mepc 覆盖范围内 ⇒ 静默跳指令。
//  它的地址取项里的 aux_addr（"pc+4"口径），中断那条减 4 得自身地址。
//★ 只能挑【真实指令】：取指队列空的时候，出队格整条压 0（hd0_v=0）而 idu1 照常推进
//  ⇒ 站里会出现"指令字全 0、地址也是 0"的空泡项（它占 ROB 一项、当 NOP 退掉，本来是良性的）。
//  空泡没有地址 ⇒ 拿它当边界就会把 mepc 写成 0-4。真实指令的地址 = pc+4 恒 >= 4，故用 aux_addr!=0 挑。
//  ★ 这个"是否真实"判据在【压站那一拍】算好、寄存进 ent_real（压站锥有余量），不再在本锥现算
//  32 位比较 —— 本锥只读 1 位寄存器，head_idx→cont.flush_idx→mdu 那条长路因此少掉一截。
//  （真出现"整站只有空泡"的短窗时中断就多等一拍，等即可，不会指到假地址。）
    integer hi;
    reg [2:0] head_sel;
    reg       head_found;
    always @(*) begin
        head_found = 1'b0; head_sel = 3'd0;
        for (hi = DEPTH-1; hi >= 0; hi = hi - 1)
            if (ent_valid[hi] && ent_real[hi]) begin head_sel = hi[2:0]; head_found = 1'b1; end
    end
    assign head_idx_o = ent_idx[head_sel];
    assign head_pc_o  = ent_aux_addr[head_sel];
//给中断资格用：站里【至少有一条真实指令没执行】，才谈得上"中断切点"。
// 中断的 mepc 与冲刷边界都取它，并且它自己也要被冲掉（含边界的杀集）。
// 站空 / 只有空泡时，下一条还没执行的指令还在 idu1 输出或取指队列里，本模块给不出它的地址
//（实测 mt13：拿空泡的 aux=0 当边界 ⇒ mepc=0xFFFFFFFC ⇒ 整机跑飞）。
// 这种窗只有几拍，等即可。
    assign head_v_o = head_found;
    assign has_older_bju_sel_o = ent_has_older_bju[sel_slot_q];

    always @(*) begin
        have_sel = 1'b0; sel_slot = 3'd0;
        for (i = DEPTH-1; i >= 0; i = i - 1)
            if (ent_issuable[i]) begin sel_slot = i[2:0]; have_sel = 1'b1; end
    end

    reg [DEPTH-1:0] ent_lane1_ok;
    reg [DEPTH-1:0] ent_issuable1;
    reg [2:0] sel_slot1;
    reg       have_sel1;
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1)
            ent_lane1_ok[i] = (ent_class[i] == CLS_ALU) & ~ent_exc_illegal[i];
    end
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1)
            ent_issuable1[i] = ent_issuable[i] & ent_lane1_ok[i]
                             & (~have_sel | (i[2:0] != sel_slot));
        have_sel1 = 1'b0; sel_slot1 = 3'd0;
        for (i = DEPTH-1; i >= 0; i = i - 1)
            if (ent_issuable1[i]) begin sel_slot1 = i[2:0]; have_sel1 = 1'b1; end
    end
    reg [2:0] byp1_q, byp2_q, byp3_q, byp4_q;
    reg       byp_ok, byp_ok1;
    always @(*) begin
        byp_ok  = 1'b1;
        byp_ok1 = 1'b1;
        if ((byp1_q == 3'd3) & ~(we_mul & (pt_mul == sel_ps1_q)))
            byp_ok = 1'b0;
        if ((byp2_q == 3'd3) & ~(we_mul & (pt_mul == sel_ps2_q)))
            byp_ok = 1'b0;
        if ((byp1_q == 3'd4) & ~(we_ld & (pt_ld == sel_ps1_q)))
            byp_ok = 1'b0;
        if ((byp2_q == 3'd4) & ~(we_ld & (pt_ld == sel_ps2_q)))
            byp_ok = 1'b0;
        if ((byp3_q == 3'd3) & ~(we_mul & (pt_mul == sel_ps1b_q)))
            byp_ok1 = 1'b0;
        if ((byp4_q == 3'd3) & ~(we_mul & (pt_mul == sel_ps2b_q)))
            byp_ok1 = 1'b0;
        if ((byp3_q == 3'd4) & ~(we_ld & (pt_ld == sel_ps1b_q)))
            byp_ok1 = 1'b0;
        if ((byp4_q == 3'd4) & ~(we_ld & (pt_ld == sel_ps2b_q)))
            byp_ok1 = 1'b0;
    end

    assign issue_v1 = sel_vld1_q & exec & ~flush_w & byp_ok1;
    assign issue_v = sel_go & res_ok_q & byp_ok;

//mdu 两条接收口：一拍至多一条（类别与算子家族都取被选中那一项）
    assign mul_go = issue_v & (sel_class_q == CLS_MUL) & ~sel_is_div_q;
    assign div_go = issue_v & (sel_class_q == CLS_MUL) &  sel_is_div_q;

//出站操作数：立即数走项里的常数；否则取 prf 的寄存读（地址 = 选优拍那一条的 ps）；
//发射拍结果口上电且 tag 命中 ⇒ 用口上值（早唤醒/当拍唤醒的旁路，省 1 拍）
    reg [31:0] out_op1, out_op2;
    always @(*) begin
        out_op1 = ent_imm1[sel_slot_q] ? ent_op1[sel_slot_q] :
                  (byp1_q == 3'd1) ? data_alu :
                  (byp1_q == 3'd2) ? data_alu2 :
                  (byp1_q == 3'd3) ? data_mul :
                  (byp1_q == 3'd4) ? data_ld : rdata1_in;
        out_op2 = ent_imm2[sel_slot_q] ? ent_op2[sel_slot_q] :
                  (byp2_q == 3'd1) ? data_alu :
                  (byp2_q == 3'd2) ? data_alu2 :
                  (byp2_q == 3'd3) ? data_mul :
                  (byp2_q == 3'd4) ? data_ld : rdata2_in;
    end
//给 prf 的读口地址（发射拍，用已寄存的选优结果 ⇒ 读口不挂在选优锥尾巴上）
    assign raddr1_out = sel_ps1_q;
    assign raddr2_out = sel_ps2_q;
    assign r1_final_out = out_op1;
    assign r2_final_out = out_op2;

    reg [31:0] out_op1_1, out_op2_1;
    always @(*) begin
        out_op1_1 = ent_imm1[sel_slot1_q] ? ent_op1[sel_slot1_q] :
                  (byp3_q == 3'd1) ? data_alu :
                  (byp3_q == 3'd2) ? data_alu2 :
                  (byp3_q == 3'd3) ? data_mul :
                  (byp3_q == 3'd4) ? data_ld : rdata3_in;
        out_op2_1 = ent_imm2[sel_slot1_q] ? ent_op2[sel_slot1_q] :
                  (byp4_q == 3'd1) ? data_alu :
                  (byp4_q == 3'd2) ? data_alu2 :
                  (byp4_q == 3'd3) ? data_mul :
                  (byp4_q == 3'd4) ? data_ld : rdata4_in;
    end
    assign raddr3_out = sel_ps1b_q;
    assign raddr4_out = sel_ps2b_q;
    assign r1_final_out1 = out_op1_1;
    assign r2_final_out1 = out_op2_1;

//出站字段：由选中项重构各单元要的口（opcode/func10 由 cls 重构，省站项宽度）
    always @(*) begin
        rd_out        = ent_rd[sel_slot_q];
        issue_idx     = ent_idx[sel_slot_q];
        issue_gen     = ent_gen[sel_slot_q];
        we            = ent_we[sel_slot_q];
        alu_func4     = ent_alu_func4[sel_slot_q];
        csr_func3     = ent_csr_func3[sel_slot_q];
        csr_wr_en     = ent_csr_wr_en[sel_slot_q];
        csr_wr_act    = ent_csr_wr_act[sel_slot_q];
        csr_addr      = ent_csr_addr[sel_slot_q];
        aux_addr_out  = sel_aux_q;
        off_mem_out   = ent_jmp_val[sel_slot_q];
        br_flag       = ent_br_flag[sel_slot_q];
        jal_flag      = ent_jal_flag[sel_slot_q];
        jalr_flag     = ent_jalr_flag[sel_slot_q];
        br_pred_taken_out = ent_br_pred_taken[sel_slot_q];
        exc_illegal_out   = ent_exc_illegal[sel_slot_q];
        exc_ecall         = ent_exc_ecall[sel_slot_q];
        exc_ebreak        = ent_exc_ebreak[sel_slot_q];
        exc_irq_ret       = sel_irq_ret_q & issue_v_irqret;
        exc_jal_misalign_out = ent_exc_jal_misalign[sel_slot_q];
        rs1_out       = 5'd0;
        rs2_out       = 5'd0;
        opc_out     = 7'd0;
        fn10_out    = 10'd0;
        fn10_ls_out = 10'd0;
        beq_off_q2        = 32'd0;
        jalr_pred_addr_out= 32'd0;
        jal_target_out    = 32'd0;
        case (sel_class_q)
            CLS_LSU: begin
                opc_out     = ent_is_store[sel_slot_q] ? OPCODE_STORE : OPCODE_LOAD;
                fn10_ls_out = {7'd0, ent_ls_func3[sel_slot_q]};
            end
            CLS_MUL: begin
                opc_out  = OPCODE_OP;
                fn10_out = {7'b0000001, ent_mul_op[sel_slot_q]};
            end
            CLS_BJU: begin
                if (ent_br_flag[sel_slot_q])  beq_off_q2         = ent_jmp_val[sel_slot_q];
                if (ent_jalr_flag[sel_slot_q]) jalr_pred_addr_out = ent_jmp_val[sel_slot_q];
                if (ent_jal_flag[sel_slot_q]) jal_target_out     = ent_jmp_val[sel_slot_q];
            end
            default: ;
        endcase
        if (!issue_v) begin
            rd_out = 5'd0; issue_idx = 3'd0; issue_gen = 1'b0; we = 1'b0;
            alu_func4 = 4'd0; csr_func3 = 3'd0; aux_addr_out = 32'd0; off_mem_out = 32'd0;
            br_flag = 1'b0; jal_flag = 1'b0; jalr_flag = 1'b0; br_pred_taken_out = 1'b0;
            exc_illegal_out = 1'b0; exc_ecall = 1'b0; exc_ebreak = 1'b0;
            exc_jal_misalign_out = 1'b0;
            rs1_out = 5'd0; rs2_out = 5'd0; opc_out = 7'd0;
            fn10_out = 10'd0; fn10_ls_out = 10'd0;
            beq_off_q2 = 32'd0; jalr_pred_addr_out = 32'd0; jal_target_out = 32'd0;
        end
//★ CSR 的写使能/地址只有【真发出去】那一拍才算数（csr.v 直接吃这两根）：
//  "被选中"在冲刷拍照样为 1，而那一拍指令并没发出去、ROB 也没记它 ⇒ 用它当写资格会误写。
//  数据（alu 的组合 result_csr）本来就与发射同拍，所以这三根同门即可、不必另打一拍。
        if (!issue_v) begin
            csr_wr_en = 1'b0; csr_wr_act = 1'b0; csr_addr = 12'd0;
        end
    end

    always @(*) begin
        rd_out1     = ent_rd[sel_slot1_q];
        issue_idx1  = ent_idx[sel_slot1_q];
        issue_gen1  = ent_gen[sel_slot1_q];
        we1         = ent_we[sel_slot1_q];
        alu_func4_1 = ent_alu_func4[sel_slot1_q];
        if (!issue_v1) begin
            rd_out1 = 5'd0; issue_idx1 = 4'd0; issue_gen1 = 1'b0; we1 = 1'b0; alu_func4_1 = 4'd0;
        end
    end

//==============================================================================
// 五、时序
//==============================================================================
    reg [3:0] e_src;
    reg       e_clr;
    wire [3:0] hole_a = sel_slot_q;
    wire [3:0] hole_b = sel_slot1_q;
    wire [3:0] mig_lo   = (hole_a <= hole_b) ? hole_a : hole_b;
    wire [3:0] mig_hi_m1 = ((hole_a <= hole_b) ? hole_b : hole_a) - 4'd1;
    always @(posedge clk) begin
        if (rst_q) begin
            pending_cnt  <= 3'd0;
            bju_shift   <= 2'd0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                ent_valid[i]   <= 1'b0;  ent_wait1[i] <= 1'b0;  ent_wait2[i] <= 1'b0;
                ent_class[i] <= 2'd0;  ent_idx[i]  <= 3'd0;  ent_gen[i]  <= 1'b0;
                ent_we[i]  <= 1'b0;  ent_rd[i] <= 5'd0;  ent_is_store[i] <= 1'b0;
                ent_mul_op[i]  <= 3'd0;  ent_is_div[i] <= 1'b0;  ent_ls_func3[i]<= 3'd0;  ent_alu_func4[i] <= 4'd0;
                ent_csr_func3[i]<= 3'd0;  ent_csr_wr_en[i]<=1'b0;  ent_csr_armed[i]<=1'b0; ent_csr_wr_act[i]<=1'b0;
                ent_csr_addr[i]<= 12'd0; ent_jmp_val[i] <= 32'd0; ent_aux_addr[i]<= 32'd0; ent_real[i] <= 1'b0;
                ent_br_flag[i] <= 1'b0;  ent_jal_flag[i]<=1'b0;  ent_jalr_flag[i]<=1'b0;
                ent_br_pred_taken[i]<= 1'b0;  ent_exc_illegal[i] <= 1'b0; ent_exc_ecall[i]<=1'b0;
                ent_exc_ebreak[i]<= 1'b0;  ent_exc_irq_ret[i]<=1'b0;  ent_exc_jal_misalign[i]<=1'b0;
                ent_op1[i] <= 32'd0; ent_op2[i] <= 32'd0;
                ent_ps1[i]  <= 6'd0;  ent_ps2[i]  <= 6'd0;
                ent_imm1[i] <= 1'b0;  ent_imm2[i] <= 1'b0;
            end
        end
        else begin
//---- 5.1 冲刷（与 rob 同口径）：全冲/trap 清整站；部分冲按年龄清【后缀】（位置即年龄 ⇒ 仍压实）----
            if (trap_fire | (flush_con_rob & ~rob_flush_part)) begin
                for (i = 0; i < DEPTH; i = i + 1) begin
                    ent_valid[i]  <= 1'b0;  ent_wait1[i] <= 1'b0;  ent_wait2[i] <= 1'b0;
                end
            end
            else if (flush_con_rob & rob_flush_part) begin
                for (i = 0; i < DEPTH; i = i + 1) begin
                    if (ent_valid[i] && (flush_incl ? (ent_age[i] >= flush_part_age)
                                                    : (ent_age[i] >  flush_part_age))) begin
                        ent_valid[i]  <= 1'b0;  ent_wait1[i] <= 1'b0;  ent_wait2[i] <= 1'b0;
                    end
                    else begin
                        ent_wait1[i] <= nx_wait1[i];  ent_wait2[i] <= nx_wait2[i];
                        ent_op1[i]   <= ent_op1[i];   ent_op2[i]   <= ent_op2[i];
                    end
                end
            end
//---- 5.2 出站/搬迁：位置即年龄 —— 删掉那一格之后的项整体前移一格（带着唤醒后的新值）----
            else begin
                for (i = 0; i < DEPTH; i = i + 1) begin
                    if (issue_v & issue_v1) begin
                        if (i[2:0] < mig_lo)         e_src = i[2:0];
                        else if (i[2:0] < mig_hi_m1) e_src = i[2:0] + 4'd1;
                        else                         e_src = i[2:0] + 4'd2;
                    end
                    else if (issue_v) begin
                        e_src = (i[2:0] >= sel_slot_q) ? (i[2:0] + 4'd1) : i[2:0];
                    end
                    else if (issue_v1) begin
                        e_src = (i[2:0] >= sel_slot1_q) ? (i[2:0] + 4'd1) : i[2:0];
                    end
                    else begin
                        e_src = i[2:0];
                    end
                    e_clr = (e_src > 4'd7);
                    if (e_clr) begin
                        ent_valid[i]  <= 1'b0;
                        ent_wait1[i] <= 1'b0;  ent_wait2[i] <= 1'b0;
                        ent_op1[i] <= 32'd0;   ent_op2[i] <= 32'd0;
                    end
                    else begin
                        ent_valid[i]  <= ent_valid[e_src];
                        ent_class[i]  <= ent_class[e_src];
                        ent_idx[i]    <= ent_idx[e_src];
                        ent_gen[i]    <= ent_gen[e_src];
                        ent_we[i]     <= ent_we[e_src];
                        ent_rd[i]     <= ent_rd[e_src];
                        ent_is_store[i] <= ent_is_store[e_src];
                        ent_mul_op[i] <= ent_mul_op[e_src];
                        ent_is_div[i] <= ent_is_div[e_src];
                        ent_ls_func3[i] <= ent_ls_func3[e_src];
                        ent_alu_func4[i] <= ent_alu_func4[e_src];
                        ent_csr_func3[i] <= ent_csr_func3[e_src];
                        ent_csr_wr_en[i] <= ent_csr_wr_en[e_src];
                        ent_csr_armed[i] <= ent_csr_armed[e_src];
                        ent_csr_wr_act[i] <= ent_csr_wr_act[e_src];
                        ent_csr_addr[i] <= ent_csr_addr[e_src];
                        ent_jmp_val[i] <= ent_jmp_val[e_src];
                        ent_aux_addr[i] <= ent_aux_addr[e_src];
                        ent_real[i]   <= ent_real[e_src];
                        ent_br_flag[i] <= ent_br_flag[e_src];
                        ent_jal_flag[i] <= ent_jal_flag[e_src];
                        ent_jalr_flag[i] <= ent_jalr_flag[e_src];
                        ent_br_pred_taken[i] <= ent_br_pred_taken[e_src];
                        ent_exc_illegal[i] <= ent_exc_illegal[e_src];
                        ent_exc_ecall[i] <= ent_exc_ecall[e_src];
                        ent_exc_ebreak[i] <= ent_exc_ebreak[e_src];
                        ent_exc_irq_ret[i] <= ent_exc_irq_ret[e_src];
                        ent_exc_jal_misalign[i] <= ent_exc_jal_misalign[e_src];
                        ent_op1[i] <= ent_op1[e_src];
                        ent_op2[i] <= ent_op2[e_src];
                        ent_wait1[i] <= nx_wait1[e_src];
                        ent_wait2[i] <= nx_wait2[e_src];
                        ent_ps1[i] <= ent_ps1[e_src];
                        ent_ps2[i] <= ent_ps2[e_src];
                        ent_imm1[i] <= ent_imm1[e_src];
                        ent_imm2[i] <= ent_imm2[e_src];
                    end
                end
            end
//---- 5.3 入站填充（放最后 ⇒ 同格"腾位+填新"以入站为准；出站那拍输出仍由 Q 驱动单元）----
            if (push_en0) begin
                ent_valid[free_slot0]    <= 1'b1;
                ent_class[free_slot0]  <= dec_class;
                ent_idx[free_slot0]    <= idx_in;
                ent_gen[free_slot0]    <= alloc_gen_in;
                ent_we[free_slot0]   <= dec_we;
                ent_rd[free_slot0]   <= dec_rd;
                ent_is_store[free_slot0]   <= dec_is_st;
                ent_mul_op[free_slot0]   <= dec_mulop;
                ent_is_div[free_slot0]   <= dec_mulop[2];
                ent_ls_func3[free_slot0]  <= dec_lsf3;
                ent_alu_func4[free_slot0]   <= dec_func4;
                ent_csr_func3[free_slot0] <= dec_csrf3;
                ent_csr_wr_en[free_slot0] <= dec_csrw_en;
                ent_csr_armed[free_slot0] <= dec_csrw_armed;
                ent_csr_wr_act[free_slot0] <= dec_csrw_wr;
                ent_csr_addr[free_slot0] <= dec_csra;
                ent_jmp_val[free_slot0]   <= dec_jalf ? jal_target_now : dec_jval;
                ent_aux_addr[free_slot0]  <= aux_addr_in;
                ent_real[free_slot0]      <= (aux_addr_in != 32'd0);
                ent_br_flag[free_slot0]  <= dec_brf;
                ent_jal_flag[free_slot0] <= dec_jalf;
                ent_jalr_flag[free_slot0] <= dec_jalfr;
                ent_br_pred_taken[free_slot0] <= br_pred_taken_in;
                ent_exc_illegal[free_slot0]  <= dec_exc_illegal;
                ent_exc_ecall[free_slot0] <= dec_exc_ecall;
                ent_exc_ebreak[free_slot0] <= dec_exc_ebreak;
                ent_exc_irq_ret[free_slot0] <= dec_exc_irq_ret;
                ent_exc_jal_misalign[free_slot0] <= dec_exc_jmis;
                ent_op1[free_slot0]  <= cap_op1;
                ent_op2[free_slot0]  <= cap_op2;
                ent_wait1[free_slot0]   <= w1_now;
                ent_wait2[free_slot0]   <= w2_now;
                ent_imm1[free_slot0]   <= dec_r1_imm;
                ent_imm2[free_slot0]   <= dec_r2_imm;
                ent_ps1[free_slot0]    <= fwd_hit1_in ? {1'b1, fwd_slot1_in, fwd_pg1_in} : {1'b0, rs1_in};
                ent_ps2[free_slot0]    <= fwd_hit2_in ? {1'b1, fwd_slot2_in, fwd_pg2_in} : {1'b0, rs2_in};
            end
            if (push_en1) begin
                ent_valid[free_slot1]    <= 1'b1;
                ent_class[free_slot1]  <= dec_class1;
                ent_idx[free_slot1]    <= idx_in1;
                ent_gen[free_slot1]    <= alloc_gen_in1;
                ent_we[free_slot1]   <= dec_we1;
                ent_rd[free_slot1]   <= dec_rd1;
                ent_is_store[free_slot1]   <= dec_is_st1;
                ent_mul_op[free_slot1]   <= dec_mulop1;
                ent_is_div[free_slot1]   <= dec_mulop1[2];
                ent_ls_func3[free_slot1]  <= dec_lsf31;
                ent_alu_func4[free_slot1]   <= dec_func41;
                ent_csr_func3[free_slot1] <= dec_csrf31;
                ent_csr_wr_en[free_slot1] <= dec_csrw_en1;
                ent_csr_armed[free_slot1] <= dec_csrw_armed1;
                ent_csr_wr_act[free_slot1] <= dec_csrw_wr1;
                ent_csr_addr[free_slot1] <= dec_csra1;
                ent_jmp_val[free_slot1]   <= dec_jalf1 ? jal_target_now_1 : dec_jval1;
                ent_aux_addr[free_slot1]  <= aux_addr_in1;
                ent_real[free_slot1]      <= (aux_addr_in1 != 32'd0);
                ent_br_flag[free_slot1]  <= dec_brf1;
                ent_jal_flag[free_slot1] <= dec_jalf1;
                ent_jalr_flag[free_slot1] <= dec_jalfr1;
                ent_br_pred_taken[free_slot1] <= br_pred_taken_in1;
                ent_exc_illegal[free_slot1]  <= dec_exc_illegal1;
                ent_exc_ecall[free_slot1] <= dec_exc_ecall1;
                ent_exc_ebreak[free_slot1] <= dec_exc_ebreak1;
                ent_exc_irq_ret[free_slot1] <= dec_exc_irq_ret1;
                ent_exc_jal_misalign[free_slot1] <= dec_exc_jmis1;
                ent_op1[free_slot1]  <= cap_op1_1;
                ent_op2[free_slot1]  <= cap_op2_1;
                ent_wait1[free_slot1]   <= w1_now_1;
                ent_wait2[free_slot1]   <= w2_now_1;
                ent_imm1[free_slot1]   <= dec_r1_imm1;
                ent_imm2[free_slot1]   <= dec_r2_imm1;
                ent_ps1[free_slot1]    <= fwd_hit1_in1 ? {1'b1, fwd_slot1_in1, fwd_pg1_in1} : {1'b0, rs1_in1};
                ent_ps2[free_slot1]    <= fwd_hit2_in1 ? {1'b1, fwd_slot2_in1, fwd_pg2_in1} : {1'b0, rs2_in1};
            end
//---- 5.5 bju 在途移位 ----
            if (flush_w) bju_shift <= 2'd0;
            else         bju_shift <= {bju_shift[0], (issue_v & (sel_class_q == CLS_BJU))};
//---- 5.6 计数 ----
            if (flush_w | trap_fire) pending_cnt <= 3'd0;
            else                     pending_cnt <= pending_nx;
        end
    end

    reg [DEPTH-1:0] bpa1, bpa2, bpb1, bpb2, bpm1, bpm2, bpl1, bpl2;
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            bpa1[i] = we  & (pt_is0 == ent_ps1[i]);
            bpa2[i] = we  & (pt_is0 == ent_ps2[i]);
            bpb1[i] = we1 & (pt_is1 == ent_ps1[i]);
            bpb2[i] = we1 & (pt_is1 == ent_ps2[i]);
            bpm1[i] = mul_wb_nx & (pt_mulnx == ent_ps1[i]);
            bpm2[i] = mul_wb_nx & (pt_mulnx == ent_ps2[i]);
            bpl1[i] = ld_wb_nx & (pt_ldnx == ent_ps1[i]);
            bpl2[i] = ld_wb_nx & (pt_ldnx == ent_ps2[i]);
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            sel_slot_q  <= 3'd0;
            sel_vld_q   <= 1'b0;
            sel_slot1_q <= 3'd0;
            sel_vld1_q  <= 1'b0;
            sel_ps1_q <= 6'd0;  sel_ps2_q <= 6'd0;  sel_aux_q <= 32'd0;
            sel_ps1b_q <= 6'd0; sel_ps2b_q <= 6'd0;
            byp1_q <= 3'd0;  byp2_q <= 3'd0;  byp3_q <= 3'd0;  byp4_q <= 3'd0;
        end
        else begin
            byp1_q <= ~have_sel ? 3'd0 : bpl1[sel_slot] ? 3'd4 : bpm1[sel_slot] ? 3'd3 : bpa1[sel_slot] ? 3'd1 : bpb1[sel_slot] ? 3'd2 : 3'd0;
            byp2_q <= ~have_sel ? 3'd0 : bpl2[sel_slot] ? 3'd4 : bpm2[sel_slot] ? 3'd3 : bpa2[sel_slot] ? 3'd1 : bpb2[sel_slot] ? 3'd2 : 3'd0;
            byp3_q <= ~have_sel1 ? 3'd0 : bpl1[sel_slot1] ? 3'd4 : bpm1[sel_slot1] ? 3'd3 : bpa1[sel_slot1] ? 3'd1 : bpb1[sel_slot1] ? 3'd2 : 3'd0;
            byp4_q <= ~have_sel1 ? 3'd0 : bpl2[sel_slot1] ? 3'd4 : bpm2[sel_slot1] ? 3'd3 : bpa2[sel_slot1] ? 3'd1 : bpb2[sel_slot1] ? 3'd2 : 3'd0;
            sel_slot_q  <= sel_slot  - ({2'd0,(issue_v & (sel_slot_q  < sel_slot))}  + {2'd0,(issue_v1 & (sel_slot1_q < sel_slot))});
            sel_slot1_q <= sel_slot1 - ({2'd0,(issue_v & (sel_slot_q  < sel_slot1))} + {2'd0,(issue_v1 & (sel_slot1_q < sel_slot1))});
            sel_vld_q   <= have_sel & exec & ~flush_w;
            sel_vld1_q  <= have_sel1 & exec & ~flush_w;
            sel_class_q   <= ent_class[sel_slot];
            sel_is_div_q  <= ent_is_div[sel_slot];
            sel_irq_ret_q <= ent_exc_irq_ret[sel_slot];
            sel_ps1_q  <= ent_ps1[sel_slot];
            sel_ps2_q  <= ent_ps2[sel_slot];
            sel_aux_q  <= ent_aux_addr[sel_slot];
            sel_ps1b_q <= ent_ps1[sel_slot1];
            sel_ps2b_q <= ent_ps2[sel_slot1];
        end
    end

    reg [3:0] occupancy_calc;
    always @(*) begin
        occupancy_calc = occupancy;
        if (alloc_en && have_free_slot) occupancy_calc = occupancy_calc + 4'd1;
        if (issue_v)           occupancy_calc = occupancy_calc - 4'd1;
        if (issue_v1)          occupancy_calc = occupancy_calc - 4'd1;
        if (trap_fire | (flush_con_rob & ~rob_flush_part)) occupancy_calc = 4'd0;
        else if (flush_con_rob & rob_flush_part) begin
            for (i = 0; i < DEPTH; i = i + 1)
                if (ent_valid[i] && (flush_incl ? (ent_age[i] >= flush_part_age)
                                                : (ent_age[i] >  flush_part_age))) occupancy_calc = occupancy_calc - 4'd1;
        end
    end
    always @(*) occupancy = {3'd0,ent_valid[0]} + {3'd0,ent_valid[1]} + {3'd0,ent_valid[2]} + {3'd0,ent_valid[3]} + {3'd0,ent_valid[4]} + {3'd0,ent_valid[5]} + {3'd0,ent_valid[6]} + {3'd0,ent_valid[7]};

endmodule
