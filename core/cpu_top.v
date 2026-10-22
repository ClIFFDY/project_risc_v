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

module cpu_top(
    input clk, rst,
    input [31:0] bus_data_in_ext,
    input bus_loaded_in, bus_hold_in, exti,
    output reg [31:0] bus_addr_out, bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out, bus_valid_out
    );

//==============================================================================
// 前端
//==============================================================================
    wire [31:0] pc_addr0, pc_addr1;
    reg  [31:0] cur_addr_0, cur_addr_1;
    wire [31:0] inst_out_ic, inst_next_ic, inst_addr0_ic, inst_addr1_ic;
    wire        inst_valid_ic, inst_valid_n_ic, icache_miss_ic, busy_ic;
    wire [31:0] mem_addr_ic, mem_wdata_ic, mem_data_ic;
    wire [3:0]  mem_be_ic;
    wire        mem_req_ic, mem_we_ic, mem_valid_ic;

    wire [31:0] fch_off_beq_0, fch_off_jal_0, fch_off_beq_1, fch_off_jal_1;
    wire        fch_jal_0, fch_jal_1, fch_jalr_0, fch_jalr_1;
    wire        fch_br_en_0, fch_br_en_1;

    wire        br1_0, br1_1, br2, br3, btb_hit_0, btb_hit_1;
    wire        bti_hit_0, bti_hit_1;
    wire [31:0] jalr_pred_0, jalr_pred_1;
    wire [63:0] bti_inst_0, bti_inst_1;
    wire [1:0]  bti_sel_0, bti_sel_1;

    wire [31:0] h0_inst, h0_addr, h0_jalr_pred, h1_inst, h1_addr, h1_jalr_pred;
    wire        h0_v, h1_v;
    wire        h0_br_pred, h1_br_pred, full_iffu;
    wire [4:0]  h0_r1, h0_r2, h1_r1, h1_r2;

//==============================================================================
// 译码级（idu1）→ 分配
//==============================================================================
    wire [1:0]  advance_idu1;
    wire        lane_v_0, lane_v_1, acc_0, acc_1;
    wire        credit_stall_idu1;

    wire        alloc_en_0, alloc_en_1, alloc_we_0, alloc_we_1;
    wire        alloc_jmp_0, alloc_jmp_1, alloc_pend_0, alloc_pend_1;
    wire [4:0]  alloc_rd_0, alloc_rd_1;
    wire [3:0]  alloc_idx_0, alloc_idx_1;
    wire        alloc_gen_0, alloc_gen_1;

    wire [31:0] inst_out_0, inst_out_1, aux_addr_out_0, aux_addr_out_1;
    wire [4:0]  rs1_idu1_0, rs2_idu1_0, rs1_idu1_1, rs2_idu1_1;
    wire [3:0]  idx_out_0, idx_out_1;
    wire        gen_out_0, gen_out_1, br_pred_taken_out_0, br_pred_taken_out_1;

    wire [1:0]  dec_class_0, dec_class_1;
    wire        dec_we_0, dec_we_1, dec_is_st_0, dec_is_st_1;
    wire [4:0]  dec_rd_0, dec_rd_1;
    wire [2:0]  dec_mulop_0, dec_mulop_1, dec_lsf3_0, dec_lsf3_1;
    wire [3:0]  dec_func4_0, dec_func4_1;
    wire [2:0]  dec_csrf3_0, dec_csrf3_1;
    wire        dec_csrw_en_0, dec_csrw_en_1, dec_csrw_armed_0, dec_csrw_armed_1;
    wire        dec_csrw_wr_0, dec_csrw_wr_1;
    wire [11:0] dec_csra_0, dec_csra_1;
    wire [31:0] dec_jval_0, dec_jval_1;
    wire        dec_brf_0, dec_brf_1, dec_jalf_0, dec_jalf_1, dec_jalfr_0, dec_jalfr_1;
    wire [3:0]  dec_bjuf4_0, dec_bjuf4_1;
    wire        dec_r1_imm_0, dec_r1_imm_1, dec_r2_imm_0, dec_r2_imm_1;
    wire [31:0] dec_r1_immv_0, dec_r1_immv_1, dec_r2_immv_0, dec_r2_immv_1;

//==============================================================================
// 发射级（idu2）↔ 单元
//==============================================================================
    wire [3:0]  qry_slot_a_0, qry_slot_b_0, qry_slot_a_1, qry_slot_b_1;
    wire        qry_gen_a_0, qry_gen_b_0, qry_gen_a_1, qry_gen_b_1;
    wire [31:0] qry_data_a_0, qry_data_b_0, qry_data_a_1, qry_data_b_1;
    wire        qry_ok_a_0, qry_ok_b_0, qry_ok_a_1, qry_ok_b_1;

    wire        issue_v_0, issue_v_1, we_0, we_1;
    wire [4:0]  rd_0, rd_1;
    wire [3:0]  issue_idx_0, issue_idx_1;
    wire        issue_gen_0, issue_gen_1;
    wire [3:0]  alu_func4_0, alu_func4_1;
    wire [2:0]  csr_func3_0, csr_func3_1;
    wire        csr_wr_en_0, csr_wr_en_1, csr_wr_act_0, csr_wr_act_1;
    wire [11:0] csr_addr_0, csr_addr_1;
    wire [31:0] aux_addr_0, aux_addr_1, beq_off_0, beq_off_1;
    wire [31:0] jalr_pred_addr_0, jalr_pred_addr_1, jal_target_0, jal_target_1;
    wire [31:0] op_data_a_0, op_data_b_0, op_data_a_1, op_data_b_1;
    wire        br_flag_0, br_flag_1, jal_flag_0, jal_flag_1, jalr_flag_0, jalr_flag_1;
    wire        br_pred_taken_0, br_pred_taken_1;
    wire [3:0]  bju_func4_0, bju_func4_1;
    wire        mul_go_0, mul_go_1, div_go_0, div_go_1;
    wire [9:0]  fn10_0, fn10_1, fn10_ls_0, fn10_ls_1;

    wire        lsu_v, lsu_lane, lsu_gen;
    wire [6:0]  lsu_opc;
    wire [9:0]  lsu_fn10;
    wire [4:0]  lsu_rd, lsu_rs_a, lsu_rs_b;
    wire [3:0]  lsu_idx;
    wire [31:0] lsu_a, lsu_b, lsu_off;

    wire [3:0]  head_idx_idu2;
    wire [31:0] head_pc_idu2;
    wire        head_v_idu2, bju_in_flight, issue_pop_0, issue_pop_1;
    wire [31:0] rdata1, rdata2, rdata3, rdata4;
//idu1 本拍 lane0 装没装：决定 lane1 的读口地址取队头 h0 还是第二格 h1
    wire        lane0_load;

//==============================================================================
// 单元结果口
//==============================================================================
    wire [31:0] res_alu_0, res_alu_1, csr_result_alu_0, csr_result_alu_1;
    wire [3:0]  idx_alu_0, idx_alu_1;
    wire        gen_alu_0, gen_alu_1, we_alu_0, we_alu_1;

    wire [31:0] mul_data_0, mul_data_1;
    wire [3:0]  mul_idx_0, mul_idx_1;
    wire        mul_gen_0, mul_gen_1, mul_we_0, mul_we_1;
    wire        mul_ready_0, div_ready_0, mul_ready_1, div_ready_1;

//rob 本拍退休的两条 → lsu（退休过的项不许再按年龄作废）
    wire        ret0_v_rob, ret1_v_rob, ret0_gen_rob, ret1_gen_rob;
    wire [3:0]  ret0_idx_rob, ret1_idx_rob;

    wire [31:0] ld_data_lsu;
    wire [3:0]  ld_idx_lsu;
    wire        ld_gen_lsu, ld_we_lsu, car_free_lsu;

// 两条 mdu 的结果口在 idu2 侧只有一条（早唤醒用；rob 侧两条都有）
    reg         mul_we_m;
    reg  [3:0]  mul_idx_m;
    reg         mul_gen_m;
    reg  [31:0] mul_data_m;

//==============================================================================
// bju ×2
//==============================================================================
    wire        success_0, br_fail_0, jalr_fail_0, success_1, br_fail_1, jalr_fail_1;
    wire [31:0] jp_tgt_0, jp_tgt_1, jalr_tgt_0, jalr_tgt_1;
    wire [3:0]  bju_idx_q_0, bju_idx_q_1, bju_idx_i_0, bju_idx_i_1;
    wire        judged_0, judged_1, gen_q_0, gen_q_1;
    wire        exc_tgt_mis_0, exc_tgt_mis_1, jalr_vld_0, jalr_vld_1, br_pred_taken_q_0, br_pred_taken_q_1;
    wire [8:0]  br_pc_idx_0, br_pc_idx_1;
    wire [31:0] exc_pc_q_0, exc_pc_q_1, exc_pc_i_0, exc_pc_i_1;
    wire        exc_ecall_0, exc_ecall_1, exc_ebreak_0, exc_ebreak_1, exc_illegal_0, exc_illegal_1, exc_irq_ret_0, exc_irq_ret_1;

//==============================================================================
// rob
//==============================================================================
    wire        commit_we0, commit_we1, commit_gen0, commit_gen1;
    wire [4:0]  commit_rd0, commit_rd1;
    wire [3:0]  commit_slot0, commit_slot1;
    wire [31:0] commit_data0, commit_data1;
    wire [3:0]  head_ptr;
    wire        rob_full, flush_part, rob_empty, trap_fire;
    wire [4:0]  occupancy_rob;
    wire [3:0]  trap_cause;
    wire [31:0] trap_pc, trap_tval;
    wire [79:0] rb_rd;
    wire [15:0] rb_gen, rb_v;

//==============================================================================
// cont / csr
//==============================================================================
    wire [7:0]  flag_bus;
    wire        flush_con_rob, flush_incl, flush_con_exc;
    wire [3:0]  flush_idx;
    wire        exc_mark;
    wire [3:0]  exc_cause;
    wire [31:0] exc_pc, exc_tval;
    wire [3:0]  exc_idx;
    wire        redir_go, flush_pc_redir, bju_fe_sel, exc_irq_ret_ok;
    wire [31:0] redir_target;
    wire [31:0] isr_addr2, mcause, iret_addr2, csr_data_comb;
    wire        exc_irq_act, exc_irq_processing, exc_irq;
    wire [3:0]  exc_ldst_idx_lsu;
    wire [31:0] exc_ldst_addr_lsu;
    wire        exc_ldst_misalign_lsu, exc_ldst_st_lsu;

//==============================================================================
// 数据侧存储
//==============================================================================
    wire [31:0] bus_addr_lsu, bus_data_lsu, data_dcache, data_tim;
    wire [3:0]  bus_be_lsu;
    wire        bus_we_lsu, bus_valid_lsu;
    wire [31:0] mem_addr_dc, mem_wdata_dc, mem_data_dtcm;
    wire [3:0]  mem_be_dc;
    wire        mem_req_dc, mem_we_dc, mem_valid_dtcm;
    wire        ready_tim, timi;
    wire        hold_dc, ld_ready_dc, ld_ready_nx_dc, busy_dc;

//==============================================================================
// 两条 lane 的合流（前端回报：由 cont 给的选择码选；乘法结果口：谁回来用谁）
//==============================================================================
// 判定延迟拍那一套：success/br_fail/jalr 落点/预测表索引/落点 —— 必须是【同一条指令】的
    reg         fe_success, fe_br_fail, fe_jalr_fail, fe_jalr_vld, fe_bpred;
    reg  [31:0] fe_jalr_tgt, fe_jp_tgt;
    reg  [8:0]  fe_brpc;
    reg         csr_wr_en_m, csr_wr_act_m;
    reg  [11:0] csr_addr_m;
    reg  [31:0] csr_data_m;

    always @(*) begin
        fe_success    = bju_fe_sel ? success_1      : success_0;
        fe_br_fail    = bju_fe_sel ? br_fail_1    : br_fail_0;
        fe_jalr_fail  = bju_fe_sel ? jalr_fail_1  : jalr_fail_0;
        fe_jalr_vld  = bju_fe_sel ? jalr_vld_1   : jalr_vld_0;
        fe_bpred      = bju_fe_sel ? br_pred_taken_q_1   : br_pred_taken_q_0;
        fe_jalr_tgt   = bju_fe_sel ? jalr_tgt_1  : jalr_tgt_0;
        fe_jp_tgt     = bju_fe_sel ? jp_tgt_1 : jp_tgt_0;
        fe_brpc       = bju_fe_sel ? br_pc_idx_1      : br_pc_idx_0;
//乘法结果口：两条同拍都回时给 lane0，lane1 那笔只是少一次早唤醒（下一拍 qry_ok 兜住）
        mul_we_m   = mul_we_0 | mul_we_1;
        mul_idx_m  = mul_we_1 ? mul_idx_1  : mul_idx_0;
        mul_gen_m  = mul_we_1 ? mul_gen_1  : mul_gen_0;
        mul_data_m = mul_we_1 ? mul_data_1 : mul_data_0;
//CSR 写口同理：两条同拍都是 CSR 指令时只落 lane0（并发 CSR 写不在本核口径内）
        csr_wr_en_m  = csr_wr_en_0 | csr_wr_en_1;
        csr_wr_act_m = csr_wr_act_0 | csr_wr_act_1;
        csr_addr_m   = csr_wr_en_1 ? csr_addr_1 : csr_addr_0;
        csr_data_m   = csr_wr_en_1 ? csr_result_alu_1 : csr_result_alu_0;
    end

//dcache → lsu 的两个握手位、以及访存非对齐那条的"指令自己那份 pc"（按 lsu 选中的 lane 取）
    reg  [31:0] exc_pc_in_mux;
    always @(*) exc_pc_in_mux = lsu_lane ? aux_addr_1 : aux_addr_0;

//总线出口：本层只是把 lsu 的请求摆到芯片口上
    always @(*) begin
        bus_addr_out  = bus_addr_lsu;
        bus_data_out  = bus_data_lsu;
        bus_be_out    = bus_be_lsu;
        bus_we_out    = bus_we_lsu;
        bus_valid_out = bus_valid_lsu;
    end

//==============================================================================
// 前端：取指地址
//==============================================================================
    pc u_pc (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .fifo_full(full_iffu),
        .icache_miss(icache_miss_ic),
        .cur_addr_0(cur_addr_0), .cur_addr_1(cur_addr_1),
        .br1_l0(br1_0),           .br1_l1(br1_1),
        .jal_l0(fch_jal_0),       .jal_l1(fch_jal_1),
        .pre_jalr_l0(fch_jalr_0), .pre_jalr_l1(fch_jalr_1),
        .btb_hit_l0(btb_hit_0),   .btb_hit_l1(btb_hit_1),
        .bti_hit_l0(bti_hit_0),   .bti_hit_l1(bti_hit_1),
        .offset_beq_pre_l0(fch_off_beq_0), .offset_jal_pre_l0(fch_off_jal_0),
        .offset_jalr_pre_l0(jalr_pred_0),
        .offset_beq_pre_l1(fch_off_beq_1), .offset_jal_pre_l1(fch_off_jal_1),
        .offset_jalr_pre_l1(jalr_pred_1),
        .jp_tgt(fe_jp_tgt),
        .redir_go(redir_go),
        .redir_target(redir_target),
        .pc_addr0(pc_addr0), .pc_addr1(pc_addr1)
    );

//==============================================================================
// 前端：预测器
//==============================================================================
    bpu u_bpu (
        .clk(clk), .rst(rst),
        .jalr_tgt_q(fe_jalr_tgt),
        .cur_addr_0(cur_addr_0), .cur_addr_1(cur_addr_1),
        .br_pc_idx(fe_brpc),
        .success(fe_success), .br_fail(fe_br_fail),
        .jalr_tgt_vld(fe_jalr_vld), .br_pred_taken_in(fe_bpred),
        .br_en_0(fch_br_en_0), .br_en_1(fch_br_en_1),
        .fch_jal_0(fch_jal_0), .fch_jal_1(fch_jal_1),
        .fch_jalr_0(fch_jalr_0),
        .inst_in_0(inst_out_ic), .inst_in_1(inst_next_ic),
        .inst_valid_0(inst_valid_ic), .inst_valid_1(inst_valid_n_ic),
        .flag_bus(flag_bus),
        .flush_pc_redir(flush_pc_redir),
//本树没有保留站（顺序发射，发射级自带 Q 寄存器）：这一位恒 0
        .stall_rs_full(1'b0),
        .fifo_full(full_iffu),
        .jalr_pred_offset_0(jalr_pred_0), .jalr_pred_offset_1(jalr_pred_1),
        .br1_0(br1_0), .br1_1(br1_1),
        .br2(br2), .br3(br3),
        .jalr_0(btb_hit_0), .jalr_1(btb_hit_1),
        .bti_hit_0(bti_hit_0), .bti_hit_1(bti_hit_1),
        .bti_inst_0(bti_inst_0), .bti_inst_1(bti_inst_1),
        .bti_sel_0(bti_sel_0), .bti_sel_1(bti_sel_1)
    );

//==============================================================================
// 前端：指令缓存 + 指令紧耦合存储
//==============================================================================
    icache u_icache (
        .clk(clk), .rst(rst),
        .pc_addr0(pc_addr0), .pc_addr1(pc_addr1),
        .br2(br2), .br3(br3),
//预测成立那一拍把交付口浇成 NOP：本拍读出来的是跳转后的顺序指令，是错路
        .pre_jalr((fch_jalr_0 & btb_hit_0) | (fch_jalr_1 & btb_hit_1)),
        .btb_hit(1'b1),
        .jalr_fail(fe_jalr_fail),
        .jal_in(fch_jal_0 | fch_jal_1),
        .br1_in(br1_0 | br1_1),
        .exc_irq(exc_irq), .exc_irq_ret(exc_irq_ret_ok),
        .exc_ecall(exc_ecall_0 | exc_ecall_1),
        .flag_bus(flag_bus),
        .mem_valid(mem_valid_ic), .mem_data(mem_data_ic),
        .flush_pc_redir(flush_pc_redir),
        .fifo_full(full_iffu),
        .inst_out(inst_out_ic), .inst_next(inst_next_ic),
        .inst_addr0(inst_addr0_ic), .inst_addr1(inst_addr1_ic),
        .inst_valid(inst_valid_ic), .inst_valid_n(inst_valid_n_ic),
        .busy(busy_ic),
        .icache_miss(icache_miss_ic),
        .mem_req(mem_req_ic), .mem_we(mem_we_ic),
        .mem_addr(mem_addr_ic), .mem_wdata(mem_wdata_ic), .mem_be(mem_be_ic)
    );

    itcm u_itcm (
        .clk(clk), .rst(rst),
        .mem_req(mem_req_ic),
        .mem_addr(mem_addr_ic),
        .mem_data(mem_data_ic),
        .mem_valid(mem_valid_ic)
    );

//交付口上这条指令自己的地址：icache 交付时用 inst_addr；btic 注入那拍 pc 已跳到目标、取 pc−8。
//★ 取指地址与交付口之间的拍差只在这里换算一次。
    always @(*) begin
        if ((bti_sel_0 | bti_sel_1) != 2'd0) begin
            cur_addr_0 = pc_addr0 - 32'd8;
            cur_addr_1 = pc_addr1 - 32'd8;
        end
        else begin
            cur_addr_0 = inst_addr0_ic;
            cur_addr_1 = inst_addr1_ic;
        end
    end

//==============================================================================
// 前端：取指队列
//==============================================================================
    iffu u_iffu (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .flush_pc_redir(flush_pc_redir),
        .inst_in_0(inst_out_ic), .inst_in_1(inst_next_ic),
        .inst_valid_0(inst_valid_ic), .inst_valid_1(inst_valid_n_ic),
        .bti_inst_0(bti_inst_0), .bti_inst_1(bti_inst_1),
        .bti_sel_0(bti_sel_0), .bti_sel_1(bti_sel_1),
        .btb_hit(btb_hit_0),
//★ 队列里存的地址是**"指令自己地址 + 4"**（全核 aux_addr 口径：jal/link/异常 pc 都靠它，
//  分支立即数在 idu1 里已经预减过 4）。icache 交付的是指令自己的地址 ⇒ 进队前在这里 +4。
        .push_addr_0(cur_addr_0 + 32'd4), .push_addr_1(cur_addr_1 + 32'd4),
        .push_br_pred_0(br1_0), .push_br_pred_1(br1_1),
        .push_jalr_pred_0(jalr_pred_0), .push_jalr_pred_1(jalr_pred_1),
        .advance(advance_idu1),
        .h0_inst(h0_inst), .h0_addr(h0_addr), .h0_jalr_pred(h0_jalr_pred),
        .h0_br_pred(h0_br_pred), .h0_r1(h0_r1), .h0_r2(h0_r2),
        .h1_inst(h1_inst), .h1_addr(h1_addr), .h1_jalr_pred(h1_jalr_pred),
        .h1_br_pred(h1_br_pred), .h1_r1(h1_r1), .h1_r2(h1_r2),
        .h0_v(h0_v), .h1_v(h1_v),
        .fch_off_beq_0(fch_off_beq_0), .fch_off_jal_0(fch_off_jal_0),
        .fch_off_beq_1(fch_off_beq_1), .fch_off_jal_1(fch_off_jal_1),
        .fch_br_en_0(fch_br_en_0), .fch_jal_0(fch_jal_0), .fch_jalr_0(fch_jalr_0),
        .fch_br_en_1(fch_br_en_1), .fch_jal_1(fch_jal_1), .fch_jalr_1(fch_jalr_1),
        .full(full_iffu)
    );

//==============================================================================
// 译码级
//==============================================================================
    idu1 u_idu1 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
//冲刷的年龄边界：输出级里挂着的那条比冲刷点老时不许清（见 idu1 里 keep_* 的注释）
        .rob_flush_part(flush_part), .flush_incl(flush_incl),
        .flush_idx(flush_idx), .head_ptr(head_ptr),
//本树没有保留站：弹出量由发射级的 issue_pop 计（credit 走 acc_in）
        .st_pop({1'b0, issue_pop_0} + {1'b0, issue_pop_1}),
        .st_flush_pop(4'd0),
        .credit_stall_o(credit_stall_idu1),
        .advance(advance_idu1),
        .lane0_load_o(lane0_load),

        .inst_in_0(h0_inst), .addr_in_0(h0_addr), .inst_v_0(h0_v),
        .br_pred_in_0(h0_br_pred), .jalr_pred_in_0(h0_jalr_pred),
        .r1_in_0(h0_r1), .r2_in_0(h0_r2),
        .idx_in_0(alloc_idx_0), .gen_in_0(alloc_gen_0),
//顺序发射：发射级在 Q 端等操作数，D 侧永远"就绪"
        .rdy1_in_0(1'b1), .rdy2_in_0(1'b1),
        .acc_in_0(acc_0),
        .lane_v_0(lane_v_0),
        .inst_out_0(inst_out_0), .aux_addr_out_0(aux_addr_out_0),
        .br_pred_taken_out_0(br_pred_taken_out_0),
        .r1_out_0(rs1_idu1_0), .r2_out_0(rs2_idu1_0),
        .idx_out_0(idx_out_0), .gen_out_0(gen_out_0),
        .pay_v_out_0(),
        .alloc_en_0(alloc_en_0), .alloc_we_0(alloc_we_0),
        .alloc_jmp_0(alloc_jmp_0), .alloc_pend_0(alloc_pend_0),
        .alloc_rd_0(alloc_rd_0),
        .dec_class_0(dec_class_0), .dec_we_0(dec_we_0), .dec_rd_0(dec_rd_0),
        .dec_is_st_0(dec_is_st_0), .dec_mulop_0(dec_mulop_0), .dec_lsf3_0(dec_lsf3_0),
        .dec_func4_0(dec_func4_0), .dec_csrf3_0(dec_csrf3_0),
        .dec_csrw_en_0(dec_csrw_en_0), .dec_csrw_armed_0(dec_csrw_armed_0),
        .dec_csrw_wr_0(dec_csrw_wr_0), .dec_csra_0(dec_csra_0),
        .dec_jval_0(dec_jval_0), .dec_brf_0(dec_brf_0),
        .dec_jalf_0(dec_jalf_0), .dec_jalfr_0(dec_jalfr_0),
        .dec_bjuf4_0(dec_bjuf4_0),
        .dec_r1_imm_0(dec_r1_imm_0), .dec_r2_imm_0(dec_r2_imm_0),
        .dec_r1_immv_0(dec_r1_immv_0), .dec_r2_immv_0(dec_r2_immv_0),

        .inst_in_1(h1_inst), .addr_in_1(h1_addr), .inst_v_1(h1_v),
        .br_pred_in_1(h1_br_pred), .jalr_pred_in_1(h1_jalr_pred),
        .r1_in_1(h1_r1), .r2_in_1(h1_r2),
        .idx_in_1(alloc_idx_1), .gen_in_1(alloc_gen_1),
        .rdy1_in_1(1'b1), .rdy2_in_1(1'b1),
        .acc_in_1(acc_1),
        .lane_v_1(lane_v_1),
        .inst_out_1(inst_out_1), .aux_addr_out_1(aux_addr_out_1),
        .br_pred_taken_out_1(br_pred_taken_out_1),
        .r1_out_1(rs1_idu1_1), .r2_out_1(rs2_idu1_1),
        .idx_out_1(idx_out_1), .gen_out_1(gen_out_1),
        .pay_v_out_1(),
        .alloc_en_1(alloc_en_1), .alloc_we_1(alloc_we_1),
        .alloc_jmp_1(alloc_jmp_1), .alloc_pend_1(alloc_pend_1),
        .alloc_rd_1(alloc_rd_1),
        .dec_class_1(dec_class_1), .dec_we_1(dec_we_1), .dec_rd_1(dec_rd_1),
        .dec_is_st_1(dec_is_st_1), .dec_mulop_1(dec_mulop_1), .dec_lsf3_1(dec_lsf3_1),
        .dec_func4_1(dec_func4_1), .dec_csrf3_1(dec_csrf3_1),
        .dec_csrw_en_1(dec_csrw_en_1), .dec_csrw_armed_1(dec_csrw_armed_1),
        .dec_csrw_wr_1(dec_csrw_wr_1), .dec_csra_1(dec_csra_1),
        .dec_jval_1(dec_jval_1), .dec_brf_1(dec_brf_1),
        .dec_jalf_1(dec_jalf_1), .dec_jalfr_1(dec_jalfr_1),
        .dec_bjuf4_1(dec_bjuf4_1),
        .dec_r1_imm_1(dec_r1_imm_1), .dec_r2_imm_1(dec_r2_imm_1),
        .dec_r1_immv_1(dec_r1_immv_1), .dec_r2_immv_1(dec_r2_immv_1)
    );

//==============================================================================
// 发射级
//==============================================================================
    idu2 u_idu2 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .s_advance(lane_v_0), .s_advance1(lane_v_1),

        .rs1_in(rs1_idu1_0), .rs2_in(rs2_idu1_0),
        .inst_in(inst_out_0),
        .pc_operand_in(aux_addr_out_0), .aux_addr_in(aux_addr_out_0),
        .br_pred_taken_in(br_pred_taken_out_0),
        .idx_in(idx_out_0), .alloc_gen_in(gen_out_0),
        .dec_class(dec_class_0), .dec_we(dec_we_0), .dec_rd(dec_rd_0),
        .dec_is_st(dec_is_st_0), .dec_mulop(dec_mulop_0), .dec_lsf3(dec_lsf3_0),
        .dec_func4(dec_func4_0), .dec_csrf3(dec_csrf3_0),
        .dec_csrw_en(dec_csrw_en_0), .dec_csrw_armed(dec_csrw_armed_0),
        .dec_csrw_wr(dec_csrw_wr_0), .dec_csra(dec_csra_0),
        .dec_jval(dec_jval_0), .dec_brf(dec_brf_0),
        .dec_jalf(dec_jalf_0), .dec_jalfr(dec_jalfr_0),
        .dec_bjuf4(dec_bjuf4_0),
        .dec_r1_imm(dec_r1_imm_0), .dec_r2_imm(dec_r2_imm_0),
        .dec_r1_immv(dec_r1_immv_0), .dec_r2_immv(dec_r2_immv_0),

        .rs1_in1(rs1_idu1_1), .rs2_in1(rs2_idu1_1),
        .inst_in1(inst_out_1),
        .pc_operand_in1(aux_addr_out_1), .aux_addr_in1(aux_addr_out_1),
        .br_pred_taken_in1(br_pred_taken_out_1),
        .idx_in1(idx_out_1), .alloc_gen_in1(gen_out_1),
        .dec_class1(dec_class_1), .dec_we1(dec_we_1), .dec_rd1(dec_rd_1),
        .dec_is_st1(dec_is_st_1), .dec_mulop1(dec_mulop_1), .dec_lsf31(dec_lsf3_1),
        .dec_func41(dec_func4_1), .dec_csrf31(dec_csrf3_1),
        .dec_csrw_en1(dec_csrw_en_1), .dec_csrw_armed1(dec_csrw_armed_1),
        .dec_csrw_wr1(dec_csrw_wr_1), .dec_csra1(dec_csra_1),
        .dec_jval1(dec_jval_1), .dec_brf1(dec_brf_1),
        .dec_jalf1(dec_jalf_1), .dec_jalfr1(dec_jalfr_1),
        .dec_bjuf41(dec_bjuf4_1),
        .dec_r1_imm1(dec_r1_imm_1), .dec_r2_imm1(dec_r2_imm_1),
        .dec_r1_immv1(dec_r1_immv_1), .dec_r2_immv1(dec_r2_immv_1),

        .qry_slot_a_0(qry_slot_a_0), .qry_slot_b_0(qry_slot_b_0),
        .qry_slot_a_1(qry_slot_a_1), .qry_slot_b_1(qry_slot_b_1),
        .qry_gen_a_0(qry_gen_a_0), .qry_gen_b_0(qry_gen_b_0),
        .qry_gen_a_1(qry_gen_a_1), .qry_gen_b_1(qry_gen_b_1),
        .qry_data_a_0(qry_data_a_0), .qry_data_b_0(qry_data_b_0),
        .qry_data_a_1(qry_data_a_1), .qry_data_b_1(qry_data_b_1),
        .qry_ok_a_0(qry_ok_a_0), .qry_ok_b_0(qry_ok_b_0),
        .qry_ok_a_1(qry_ok_a_1), .qry_ok_b_1(qry_ok_b_1),

        .we_alu(we_alu_0),   .idx_alu(idx_alu_0),   .gen_alu(gen_alu_0),   .data_alu(res_alu_0),
        .we_alu2(we_alu_1),  .idx_alu2(idx_alu_1),  .gen_alu2(gen_alu_1),  .data_alu2(res_alu_1),
        .we_mul(mul_we_m),   .idx_mul(mul_idx_m),   .gen_mul(mul_gen_m),   .data_mul(mul_data_m),
        .we_ld(ld_we_lsu),   .idx_ld(ld_idx_lsu),   .gen_ld(ld_gen_lsu),   .data_ld(ld_data_lsu),
//同拍组合结果口暂不接（接上之后必须单列一条时序路径）
        .now_v_0(1'b0), .now_idx_0(4'd0), .now_gen_0(1'b0), .now_data_0(32'd0),
        .now_v_1(1'b0), .now_idx_1(4'd0), .now_gen_1(1'b0), .now_data_1(32'd0),

        .we_a(commit_we0), .rd_a(commit_rd0),
        .we_b(commit_we1), .rd_b(commit_rd1),
        .commit_slot0(commit_slot0), .commit_slot1(commit_slot1),
        .commit_gen0(commit_gen0),  .commit_gen1(commit_gen1),
        .commit_data0(commit_data0), .commit_data1(commit_data1),

        .flush_con_rob(flush_con_rob), .rob_flush_part(flush_part),
        .flush_incl(flush_incl), .flush_idx(flush_idx),
        .trap_fire(trap_fire), .head_ptr(head_ptr),
        .rb_rd(rb_rd), .rb_gen(rb_gen), .rb_v(rb_v),

        .lsu_full(~car_free_lsu),
        .mdu_mul_ok_0(mul_ready_0), .mdu_div_ok_0(div_ready_0),
        .mdu_mul_ok_1(mul_ready_1), .mdu_div_ok_1(div_ready_1),
        .bju_ok_0(1'b1), .bju_ok_1(1'b1),

        .rdata1_in(rdata1), .rdata2_in(rdata2),
        .rdata3_in(rdata3), .rdata4_in(rdata4),

        .issue_v_0(issue_v_0), .rd_0(rd_0), .issue_idx_0(issue_idx_0),
        .issue_gen_0(issue_gen_0), .we_0(we_0), .sel_class_0(),
        .opc_0(), .fn10_0(fn10_0), .fn10_ls_0(fn10_ls_0),
        .alu_func4_0(alu_func4_0), .csr_func3_0(csr_func3_0),
        .csr_wr_en_0(csr_wr_en_0), .csr_wr_act_0(csr_wr_act_0),
        .csr_addr_0(csr_addr_0),
        .aux_addr_0(aux_addr_0), .off_mem_0(), .beq_off_0(beq_off_0),
        .jalr_pred_addr_0(jalr_pred_addr_0), .jal_target_0(jal_target_0),
        .op_data_a_0(op_data_a_0), .op_data_b_0(op_data_b_0),
        .rs_a_0(), .rs_b_0(),
        .br_flag_0(br_flag_0), .jal_flag_0(jal_flag_0), .jalr_flag_0(jalr_flag_0),
        .br_pred_taken_0(br_pred_taken_0),
        .bju_func4_0(bju_func4_0),
        .mul_go_0(mul_go_0), .div_go_0(div_go_0),

        .issue_v_1(issue_v_1), .rd_1(rd_1), .issue_idx_1(issue_idx_1),
        .issue_gen_1(issue_gen_1), .we_1(we_1), .sel_class_1(),
        .opc_1(), .fn10_1(fn10_1), .fn10_ls_1(fn10_ls_1),
        .alu_func4_1(alu_func4_1), .csr_func3_1(csr_func3_1),
        .csr_wr_en_1(csr_wr_en_1), .csr_wr_act_1(csr_wr_act_1),
        .csr_addr_1(csr_addr_1),
        .aux_addr_1(aux_addr_1), .off_mem_1(), .beq_off_1(beq_off_1),
        .jalr_pred_addr_1(jalr_pred_addr_1), .jal_target_1(jal_target_1),
        .op_data_a_1(op_data_a_1), .op_data_b_1(op_data_b_1),
        .rs_a_1(), .rs_b_1(),
        .br_flag_1(br_flag_1), .jal_flag_1(jal_flag_1), .jalr_flag_1(jalr_flag_1),
        .br_pred_taken_1(br_pred_taken_1),
        .bju_func4_1(bju_func4_1),
        .mul_go_1(mul_go_1), .div_go_1(div_go_1),

        .lsu_v(lsu_v), .lsu_lane(lsu_lane), .lsu_opc(lsu_opc), .lsu_fn10(lsu_fn10),
        .lsu_rd(lsu_rd), .lsu_idx(lsu_idx), .lsu_gen(lsu_gen),
        .lsu_a(lsu_a), .lsu_b(lsu_b), .lsu_off(lsu_off),
        .lsu_rs_a(lsu_rs_a), .lsu_rs_b(lsu_rs_b),

        .head_idx_o(head_idx_idu2), .head_pc_o(head_pc_idu2), .head_v_o(head_v_idu2),
        .bju_in_flight(bju_in_flight),
        .acc_0(acc_0), .acc_1(acc_1),
        .issue_pop_0(issue_pop_0), .issue_pop_1(issue_pop_1)
    );

//==============================================================================
// 执行单元：每 lane 一套 alu / mdu / bju
//==============================================================================
    alu u_alu_0 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .we_in(we_0), .jal_flag(jal_flag_0), .jalr_flag(jalr_flag_0),
        .cs_wr_en(csr_wr_en_0),
        .rd_in(rd_0), .idx_in(issue_idx_0), .gen_in(issue_gen_0),
        .alu_func4(alu_func4_0), .csr_func3(csr_func3_0),
        .aux_addr_in(aux_addr_0),
        .r1_data(op_data_a_0), .r2_data(op_data_b_0), .cs_data(csr_data_comb),
        .rd_out(), .idx_out(idx_alu_0), .gen_out(gen_alu_0),
        .result(res_alu_0), .result_csr(csr_result_alu_0),
        .result_now(),
        .we(we_alu_0)
    );

    alu u_alu_1 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .we_in(we_1), .jal_flag(jal_flag_1), .jalr_flag(jalr_flag_1),
        .cs_wr_en(csr_wr_en_1),
        .rd_in(rd_1), .idx_in(issue_idx_1), .gen_in(issue_gen_1),
        .alu_func4(alu_func4_1), .csr_func3(csr_func3_1),
        .aux_addr_in(aux_addr_1),
        .r1_data(op_data_a_1), .r2_data(op_data_b_1), .cs_data(csr_data_comb),
        .rd_out(), .idx_out(idx_alu_1), .gen_out(gen_alu_1),
        .result(res_alu_1), .result_csr(csr_result_alu_1),
        .result_now(),
        .we(we_alu_1)
    );

    mdu u_mdu_0 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .flush_idx(flush_idx), .head_ptr(head_ptr), .flush_part(flush_part),
        .mul_go(mul_go_0), .mul_func10(fn10_0), .mul_rd(rd_0),
        .mul_idx_in(issue_idx_0), .mul_gen_in(issue_gen_0),
        .mul_r1(op_data_a_0), .mul_r2(op_data_b_0),
        .div_go(div_go_0), .div_func10(fn10_0), .div_rd(rd_0),
        .div_idx_in(issue_idx_0), .div_gen_in(issue_gen_0),
        .div_r1(op_data_a_0), .div_r2(op_data_b_0),
        .mul_data_out(mul_data_0), .mul_loaded(mul_we_0), .mul_we(),
        .rd_mul(), .mul_idx(mul_idx_0), .mul_gen(mul_gen_0),
        .mul_ready(mul_ready_0), .div_ready(div_ready_0),
        .mul_wake_q(), .mul_wake_idx_q(), .mul_wake_gen_q(),
        .mul_nack_q(), .mul_nack_idx_q(), .mul_nack_gen_q()
    );

    mdu u_mdu_1 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .flush_idx(flush_idx), .head_ptr(head_ptr), .flush_part(flush_part),
        .mul_go(mul_go_1), .mul_func10(fn10_1), .mul_rd(rd_1),
        .mul_idx_in(issue_idx_1), .mul_gen_in(issue_gen_1),
        .mul_r1(op_data_a_1), .mul_r2(op_data_b_1),
        .div_go(div_go_1), .div_func10(fn10_1), .div_rd(rd_1),
        .div_idx_in(issue_idx_1), .div_gen_in(issue_gen_1),
        .div_r1(op_data_a_1), .div_r2(op_data_b_1),
        .mul_data_out(mul_data_1), .mul_loaded(mul_we_1), .mul_we(),
        .rd_mul(), .mul_idx(mul_idx_1), .mul_gen(mul_gen_1),
        .mul_ready(mul_ready_1), .div_ready(div_ready_1),
        .mul_wake_q(), .mul_wake_idx_q(), .mul_wake_gen_q(),
        .mul_nack_q(), .mul_nack_idx_q(), .mul_nack_gen_q()
    );

    bju u_bju_0 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .r1_data_in(op_data_a_0), .r2_data_in(op_data_b_0),
        .bju_func4_in(bju_func4_0),
        .pc_plus4_in(aux_addr_0), .br_off_in(beq_off_0),
        .jalr_pred_tgt_in(jalr_pred_addr_0),
        .br_pred_taken_in(br_pred_taken_0),
        .jal_tgt_in(jal_target_0),
        .adv_in(issue_v_0),
        .success(success_0), .br_fail(br_fail_0), .jalr_fail(jalr_fail_0),
        .idx_in(issue_idx_0), .gen_in(issue_gen_0),
        .idx_q(bju_idx_q_0), .judged_q(judged_0), .gen_q(gen_q_0),
        .jp_tgt(jp_tgt_0), .jalr_tgt_q(jalr_tgt_0),
        .br_pc_idx(br_pc_idx_0),
        .exc_tgt_mis(exc_tgt_mis_0),
        .jalr_tgt_vld_q(jalr_vld_0), .br_pred_taken_q(br_pred_taken_q_0),
        .exc_bju(), .idx_i(bju_idx_i_0), .exc_pc_q(exc_pc_q_0),
        .exc_ecall_i(exc_ecall_0), .exc_ebreak_i(exc_ebreak_0),
        .exc_illegal_i(exc_illegal_0), .exc_irq_ret_i(exc_irq_ret_0),
        .exc_pc_i(exc_pc_i_0),
        .flush_idx_in(flush_idx), .head_ptr_in(head_ptr)
    );

    bju u_bju_1 (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .r1_data_in(op_data_a_1), .r2_data_in(op_data_b_1),
        .bju_func4_in(bju_func4_1),
        .pc_plus4_in(aux_addr_1), .br_off_in(beq_off_1),
        .jalr_pred_tgt_in(jalr_pred_addr_1),
        .br_pred_taken_in(br_pred_taken_1),
        .jal_tgt_in(jal_target_1),
        .adv_in(issue_v_1),
        .success(success_1), .br_fail(br_fail_1), .jalr_fail(jalr_fail_1),
        .idx_in(issue_idx_1), .gen_in(issue_gen_1),
        .idx_q(bju_idx_q_1), .judged_q(judged_1), .gen_q(gen_q_1),
        .jp_tgt(jp_tgt_1), .jalr_tgt_q(jalr_tgt_1),
        .br_pc_idx(br_pc_idx_1),
        .exc_tgt_mis(exc_tgt_mis_1),
        .jalr_tgt_vld_q(jalr_vld_1), .br_pred_taken_q(br_pred_taken_q_1),
        .exc_bju(), .idx_i(bju_idx_i_1), .exc_pc_q(exc_pc_q_1),
        .exc_ecall_i(exc_ecall_1), .exc_ebreak_i(exc_ebreak_1),
        .exc_illegal_i(exc_illegal_1), .exc_irq_ret_i(exc_irq_ret_1),
        .exc_pc_i(exc_pc_i_1),
        .flush_idx_in(flush_idx), .head_ptr_in(head_ptr)
    );

//==============================================================================
// 共享访存单元
//==============================================================================
    lsu u_lsu (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
        .bju_in_flight(bju_in_flight),
        .payload_go(lsu_v),
        .opcode(lsu_opc), .func10(lsu_fn10),
        .rd_in(lsu_rd), .r1_post(lsu_rs_a), .r2_post(lsu_rs_b),
        .idx_in(lsu_idx), .gen_in(lsu_gen),
        .r1_data_final(lsu_a), .r2_data_final(lsu_b),
        .offset_load0(lsu_off), .offset_store0(lsu_off),
        .bus_data_ext(bus_data_in_ext), .bus_data_dcache(data_dcache),
        .bus_data_tim(data_tim),
        .dcache_hold(hold_dc), .ld_ready_nx(ld_ready_nx_dc),
//站没了之后"更老的 BJU 在飞"只剩这一个来源（见 spec §6.4）
        .has_older_bju_in(bju_in_flight),
        .flush_idx_in(flush_idx), .head_ptr_in(head_ptr),
        .ready_dcache(ld_ready_dc), .ready_tim(ready_tim), .ready_ext(bus_loaded_in),
        .bus_addr_out(bus_addr_lsu), .bus_data_out(bus_data_lsu),
        .bus_be_out(bus_be_lsu), .bus_we_out(bus_we_lsu), .bus_valid_out(bus_valid_lsu),
        .ld_data_out(ld_data_lsu), .loaded(), .ld_we(ld_we_lsu),
        .stall_lsu_haz(), .stall_lsu_unload(), .stall_lsu_full(),
        .car_free(car_free_lsu), .mem_inflight(),
        .ld_wake_q(), .ld_wake_idx_q(), .ld_wake_gen_q(),
        .ld_nack_q(), .ld_nack_idx_q(), .ld_nack_gen_q(),
        .rd_load(),
        .ld_idx(ld_idx_lsu), .ld_gen(ld_gen_lsu),
        .ret0_v(ret0_v_rob), .ret0_idx(ret0_idx_rob), .ret0_gen(ret0_gen_rob),
        .ret1_v(ret1_v_rob), .ret1_idx(ret1_idx_rob), .ret1_gen(ret1_gen_rob),
        .exc_ldst_misalign_out(exc_ldst_misalign_lsu),
        .exc_ldst_st_out(exc_ldst_st_lsu),
        .exc_ldst_addr_out(exc_ldst_addr_lsu),
        .exc_ldst_idx_out(exc_ldst_idx_lsu)
    );

//==============================================================================
// 重排序缓冲
//==============================================================================
    rob u_rob (
        .clk(clk), .rst(rst),
        .alloc_en_0(alloc_en_0), .alloc_en_1(alloc_en_1),
        .alloc_rd_0(alloc_rd_0), .alloc_rd_1(alloc_rd_1),
        .alloc_we_0(alloc_we_0), .alloc_we_1(alloc_we_1),
        .alloc_jmp_0(alloc_jmp_0), .alloc_jmp_1(alloc_jmp_1),
        .alloc_pend_0(alloc_pend_0), .alloc_pend_1(alloc_pend_1),
        .alloc_idx_0(alloc_idx_0), .alloc_idx_1(alloc_idx_1),
        .alloc_gen_0(alloc_gen_0), .alloc_gen_1(alloc_gen_1),

        .qry_slot_a_0(qry_slot_a_0), .qry_slot_b_0(qry_slot_b_0),
        .qry_slot_a_1(qry_slot_a_1), .qry_slot_b_1(qry_slot_b_1),
        .qry_gen_a_0(qry_gen_a_0), .qry_gen_b_0(qry_gen_b_0),
        .qry_gen_a_1(qry_gen_a_1), .qry_gen_b_1(qry_gen_b_1),
        .qry_data_a_0(qry_data_a_0), .qry_data_b_0(qry_data_b_0),
        .qry_data_a_1(qry_data_a_1), .qry_data_b_1(qry_data_b_1),
        .qry_ok_a_0(qry_ok_a_0), .qry_ok_b_0(qry_ok_b_0),
        .qry_ok_a_1(qry_ok_a_1), .qry_ok_b_1(qry_ok_b_1),
//生产者表重建口：只在冲刷那一拍被 idu2 用
        .rb_rd(rb_rd), .rb_gen(rb_gen), .rb_v(rb_v),

        .bju_judged(judged_0), .bju_idx(bju_idx_q_0), .bju_gen(gen_q_0),
        .bju2_judged(judged_1), .bju2_idx(bju_idx_q_1), .bju2_gen(gen_q_1),
        .alu_done(we_alu_0), .alu_idx(idx_alu_0), .alu_data(res_alu_0), .alu_gen(gen_alu_0),
        .alu2_done(we_alu_1), .alu2_idx(idx_alu_1), .alu2_data(res_alu_1), .alu2_gen(gen_alu_1),
        .mul_done(mul_we_0), .mul_idx(mul_idx_0), .mul_data(mul_data_0), .mul_gen(mul_gen_0),
        .mul2_done(mul_we_1), .mul2_idx(mul_idx_1), .mul2_data(mul_data_1), .mul2_gen(mul_gen_1),
        .ld_done(ld_we_lsu), .ld_idx(ld_idx_lsu), .ld_data(ld_data_lsu), .ld_gen(ld_gen_lsu),
        .acc_alu_o(), .acc_alu2_o(), .acc_mul_o(), .acc_mul2_o(), .acc_ld_o(),
        .ret0_v(ret0_v_rob), .ret0_idx(ret0_idx_rob), .ret0_gen(ret0_gen_rob),
        .ret1_v(ret1_v_rob), .ret1_idx(ret1_idx_rob), .ret1_gen(ret1_gen_rob),

        .issue_en_0(issue_v_0), .issue_idx_0(issue_idx_0), .issue_gen_0(issue_gen_0),
        .issue_en_1(issue_v_1), .issue_idx_1(issue_idx_1), .issue_gen_1(issue_gen_1),

        .flush_con_rob(flush_con_rob), .flush_idx(flush_idx), .flush_incl(flush_incl),
        .flush_con_exc(flush_con_exc),
        .exc_en(exc_mark), .exc_idx(exc_idx), .exc_cause(exc_cause),
        .exc_pc(exc_pc), .exc_tval(exc_tval),

        .commit_we0(commit_we0), .commit_rd0(commit_rd0), .commit_data0(commit_data0),
        .commit_slot0(commit_slot0), .commit_gen0(commit_gen0),
        .commit_we1(commit_we1), .commit_rd1(commit_rd1), .commit_data1(commit_data1),
        .commit_slot1(commit_slot1), .commit_gen1(commit_gen1),

        .head_ptr(head_ptr), .full(rob_full), .flush_part(flush_part),
        .empty(rob_empty), .occupancy(occupancy_rob),
        .trap_fire(trap_fire), .trap_cause(trap_cause),
        .trap_pc(trap_pc), .trap_tval(trap_tval)
    );

//==============================================================================
// 寄存器堆（两条提交写口）
//==============================================================================
    regfile u_regfile (
        .clk(clk), .rst(rst),
        .flag_bus(flag_bus),
//顺序发射：读口每拍跟着地址刷新，"上一拍冻结"那一档不再存在
        .stall_rs_full(1'b0),
//★ 读口地址来自【出队格 F 级】（不是 idu2 的 D）：寄存一拍之后正好与"这一条进 D"同拍到达，
//  于是 rdata 与 D 里的指令对齐 —— 这就是 idu2 那边"base 与指令一起来"的口径。
//  lane1 到底拿第二格 h1 还是队头 h0，由 idu1 本拍的装载决定（两条 lane 独立推进）。
        .r1(h0_r1), .r2(h0_r2),
        .r3(lane0_load ? h1_r1 : h0_r1), .r4(lane0_load ? h1_r2 : h0_r2),
        .we_a(commit_we0), .rd_a(commit_rd0), .data_a(commit_data0),
        .we_b(commit_we1), .rd_b(commit_rd1), .data_b(commit_data1),
        .r1_data(rdata1), .r2_data(rdata2), .r3_data(rdata3), .r4_data(rdata4)
    );

//==============================================================================
// 控制 / CSR
//==============================================================================
    cont u_cont (
        .clk(clk), .rst(rst),
        .jalr_fail_0(jalr_fail_0), .jalr_fail_1(jalr_fail_1),
        .br2(br2), .br3(br3),
        .success_0(success_0), .success_1(success_1),
        .br_fail_0(br_fail_0), .br_fail_1(br_fail_1),
        .exc_tgt_mis_0(exc_tgt_mis_0), .exc_tgt_mis_1(exc_tgt_mis_1),
        .bju_pc_in_0(exc_pc_q_0), .bju_pc_in_1(exc_pc_q_1),
        .bju_tgt_in_0(jp_tgt_0), .bju_tgt_in_1(jp_tgt_1),
        .exc_ecall_i_0(exc_ecall_0), .exc_ebreak_i_0(exc_ebreak_0),
        .exc_illegal_i_0(exc_illegal_0), .exc_irq_ret_i_0(exc_irq_ret_0),
        .exc_ecall_i_1(exc_ecall_1), .exc_ebreak_i_1(exc_ebreak_1),
        .exc_illegal_i_1(exc_illegal_1), .exc_irq_ret_i_1(exc_irq_ret_1),
        .exc_pc_i_0(exc_pc_i_0), .exc_pc_i_1(exc_pc_i_1),
        .bju_idx_i_0(bju_idx_i_0), .bju_idx_i_1(bju_idx_i_1),
        .exc_ldst_misalign_in(exc_ldst_misalign_lsu),
        .exc_ldst_st_in(exc_ldst_st_lsu),
        .exc_ldst_addr_in(exc_ldst_addr_lsu),
        .exc_pc_in(exc_pc_in_mux),
        .exc_ldst_idx_in(exc_ldst_idx_lsu),
        .issue_idx_in(lsu_idx),
        .irq_head_idx_in(head_idx_idu2),
        .irq_head_v(head_v_idu2),
        .bju_idx_in_0(bju_idx_q_0), .bju_idx_in_1(bju_idx_q_1),
        .stall_icache_miss(busy_ic),
        .stall_bus_hold(bus_hold_in),
        .stall_rob_full(rob_full),
        .rob_head_ptr(head_ptr),
        .rob_empty(rob_empty),
        .rob_trap(trap_fire), .rob_trap_cause(trap_cause),
        .rob_trap_pc(trap_pc), .rob_trap_tval(trap_tval),
        .flush_con_rob(flush_con_rob), .flush_idx(flush_idx), .flush_incl(flush_incl),
        .jal(fch_jal_0 | fch_jal_1), .pre_jalr(fch_jalr_0 | fch_jalr_1),
        .btb_hit(btb_hit_0 | btb_hit_1), .br1(br1_0 | br1_1),
        .csr_wr_en(csr_wr_en_m), .csr_wr_act(csr_wr_act_m),
        .exti(exti), .timi(timi), .softi(1'b0),
        .csr_addr(csr_addr_m), .csr_data_in(csr_data_m),
        .pc_addr_in(head_pc_idu2),
        .isr_addr2(isr_addr2), .mcause(mcause), .csr_data_comb(csr_data_comb),
        .exc_irq_act(exc_irq_act), .exc_irq_processing(exc_irq_processing),
        .exc_irq(exc_irq), .iret_addr2(iret_addr2),
        .flag_bus(flag_bus),
        .flush_con_exc(flush_con_exc), .exc_mark(exc_mark),
        .exc_cause(exc_cause), .exc_pc(exc_pc), .exc_tval(exc_tval),
        .exc_idx(exc_idx), .exc_irq_ret_ok(exc_irq_ret_ok),
        .redir_go(redir_go), .redir_target(redir_target),
        .flush_pc_redir(flush_pc_redir),
        .bju_fe_sel(bju_fe_sel)
    );

//==============================================================================
// 数据侧存储
//==============================================================================
    dcache u_dcache (
        .clk(clk), .rst(rst),
        .bus_addr_in(bus_addr_lsu), .bus_data_in(bus_data_lsu),
        .bus_be_in(bus_be_lsu), .bus_we_in(bus_we_lsu),
        .mem_data(mem_data_dtcm), .mem_valid(mem_valid_dtcm), .mem_ready(1'b1),
        .bus_data_out(data_dcache),
        .ld_ready(ld_ready_dc), .busy(busy_dc), .hold(hold_dc),
        .ld_ready_nx(ld_ready_nx_dc),
        .mem_req(mem_req_dc), .mem_we(mem_we_dc),
        .mem_addr(mem_addr_dc), .mem_wdata(mem_wdata_dc), .mem_be(mem_be_dc)
    );

    dtcm u_dtcm (
        .clk(clk), .rst(rst),
        .mem_req(mem_req_dc), .mem_we(mem_we_dc),
        .mem_addr(mem_addr_dc), .mem_wdata(mem_wdata_dc), .mem_be(mem_be_dc),
        .mem_data(mem_data_dtcm), .mem_valid(mem_valid_dtcm), .mem_ready()
    );

    tim_in u_tim_in (
        .clk(clk), .rst(rst),
        .bus_addr_in(bus_addr_lsu), .bus_data_in(bus_data_lsu),
        .bus_be_in(bus_be_lsu), .bus_we_in(bus_we_lsu),
        .bus_data_out(data_tim), .timi(timi), .ready(ready_tim)
    );

endmodule
