`timescale 1ns / 1ps
module cpu_top(
    input clk, rst,
    input [31:0] bus_data_in_ext,
    input bus_loaded_in, bus_hold_in, exti,
    output reg [31:0] bus_addr_out, bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out, bus_valid_out
    );
//流水线层次块：按信号首生产者的流水线位置排序
    wire [31:0] pc_addr, aux_addr_0;
    (* max_fanout = 32 *) wire [31:0] icache_inst_w;
    (* max_fanout = 32 *) wire [31:0] inst_1;
    wire icache_busy_w;
    wire icache_inst_valid_w;
    wire br1, br2, br3;
    (* max_fanout = 32 *) wire [4:0] rs1_1, rs2_1;
    wire [31:0] aux_addr_1;
    wire jal, br_en, pre_jalr;
    wire br_pred_taken_1;
    wire [31:0] jalr_pred_addr_1;
//mid_decoder 那一级的输出：译码字段由 pre 给出、本级纯寄存；载荷的操作数由 pd 组合装配后直送 forw
    wire [31:0] inst_2;
    wire [4:0] rs1_2, rs2_2;
    wire [31:0] aux_addr_2;
    wire br_pred_taken_2;
    wire [31:0] jalr_pred_addr_2;
    wire [4:0]  rd_2;
    wire [9:0]  func10_2;
    wire [31:0] imm_alu_2;
//pre_decoder 那一级的译码字段输出
    wire [4:0]  rd_1;
    wire [9:0]  func10_1;
    wire [31:0] imm_alu_1;
//非寄存器操作数的载荷（装配已并进 forw）
    wire [31:0] r1_imm_val_w, r2_imm_val_w;
    wire        r1_imm_sel_w, r2_imm_sel_w;
//三个单元各自算好的前送命中位（r1/r2 各一位）：forw 据此直接选源，本级不再比 idx
    wire [4:0] rd_3;
    wire [31:0] aux_addr_3, beq_off_q2, jalr_pred_addr_3;
    wire we_3, br_flag, br_pred_taken_3;
    wire [4:0] rd_4;
    wire [2:0] alu_idx_w;
    wire [31:0] result_4;
    wire we_4;
    wire [31:0] result_back2;
//写口的事件流别名（给 tb 抓"写进寄存器堆的那一笔"）：直接跟写口级的两条口走
    wire we_a_w, we_b_w;
    wire [4:0] rd_a_w, rd_b_w;
    wire [31:0] data_a_w, data_b_w;
//写序号（乱序写回）：post_decoder 出"本拍这条会写 rd"，controller 内发号（tag_w）
    wire issue_we_w;
//★ 号源分两处，看消费者采的是哪一级：
//  lsu / mulu 采的是 E4【组合输入】(rd_1/inst_1) ⇒ 用实时计数器 tag_w —— 计数器现在
//  只在"载荷推进且有写"那一拍加一次，这条指令等在输入口期间它不动，所以 tag_w 就是发给它自己的号；
//  wbu 的 alu 口采的是 E4【寄存载荷】(rd_4/we_4) ⇒ 必须用与载荷同步锁存出来的 issue_tag_w
//  （从前它接 tag_w = 下一跳的号 ⇒ 老指令看起来比年轻指令还新，杀老写杀反）。
//ROB（重排序缓冲）：分配口/完成口/退口。阶段 B-1 先只接线、退口暂不驱动寄存器堆（行为零变化）。
    wire [2:0]  rob_alloc_idx_w;
//前送槽扫描的打拍使能：pre_decoder 的输出推进条件（扫描提前读拍一拍）
    wire        pre_go_w;
    wire [2:0]  issue_idx_w;
//前送槽号：rob 按槽序扫出"本条指令的操作数该由哪个槽供值"，作为载荷经 post_decoder 寄到载荷拍
    wire [2:0]  fwd_slot1_w, fwd_slot2_w;
    wire        fwd_hit1_w,  fwd_hit2_w;
    wire [2:0]  sel_slot1_w, sel_slot2_w;
    wire        sel_v1_w,    sel_v2_w;
    wire        payload_go_w, payload_go_q_w;
    wire [2:0]  rob_head_w;
//世代位：rob 分配时给出 → post_decoder 锁成载荷 → 执行单元随值带走 → 完成口带回 rob 校验
    wire        alloc_gen_w, issue_gen_w;
    wire        alu_gen_w, mul_gen_w, ld_gen_w;
//rob 前送值读口两组：① 正常读（索引 = rob 自己的扫描槽）② 停顿重读（索引 = regfile 的锁存槽）
//  ★ 分成两组是为了把 stall 那条组合量从【正常读】的数据路上拿掉（见 regfile/rob 的注释）
    wire [31:0] rob_fwd_data1_w, rob_fwd_data2_w;
    wire        rob_fwd_done1_w, rob_fwd_done2_w;
    wire [2:0]  st_slot1_w, st_slot2_w;
    wire [31:0] rob_st_data1_w, rob_st_data2_w;
    wire        rob_st_done1_w, rob_st_done2_w;
//mulu 的结果口被 ROB 按索引直接采走 ⇒ 结果口恒"当场被取走"，不再需要写口级的放行握手
    wire        wp_taken_mul;
    wire        rob_empty_w;
    wire [3:0]  rob_trap_cause_w;
    wire [31:0] rob_trap_pc_w, rob_trap_tval_w;
    wire        exc_mark_w, exc_bju_w;
    wire [2:0]  bju_idx_q_w, bju_idx_i_w;
    wire [6:0]  opc_3;
    wire [9:0]  fn10_3, fn10_ls_3;
    wire [31:0] off_mem_3;
//载荷的两个源寄存器号：当拍同时喂 `forw`（点② 的 `r1_post`/`r2_post`）、`lsu`、`mulu`、`bju`
//各自的匹配比较，是个纯广播网。
    wire [4:0]  rs1_3, rs2_3;
    wire [31:0] bju_exc_pc_w;
    wire        bju_older_w;
    wire [3:0]  rob_occ_w;
    wire        rob_rwe0_w, rob_rwe1_w;
    wire [4:0]  rob_rrd0_w, rob_rrd1_w;
    wire [31:0] rob_rdt0_w, rob_rdt1_w;
//写序号（controller 内生成）：跟着每条写一路走；以及落地广播、滞留兜底
    wire [2:0]  ld_idx_w, mul_idx_w;
    wire [4:0] issue_rd_w;
    wire [4:0] lsu_pend_rd0_w, lsu_pend_rd1_w, lsu_pend_rd2_w;
    wire       lsu_pend_v0_w,  lsu_pend_v1_w,  lsu_pend_v2_w;
    wire [4:0] mul_pend_rd0_w, mul_pend_rd1_w, mul_pend_rd2_w, mul_pend_rd3_w;
    wire       mul_pend_v0_w,  mul_pend_v1_w,  mul_pend_v2_w,  mul_pend_v3_w;
    wire [4:0] wbu_pend_rd0_w, wbu_pend_rd1_w, wbu_pend_rd2_w;
    wire       wbu_pend_v0_w,  wbu_pend_v1_w,  wbu_pend_v2_w;
//非流水线层次块（核内控制信号和其他信号）：按信号首生产者所在模块的代码位置排序
    wire [11:0] flag_bus;
    wire [3:0] alu_func4;
    wire [11:0] csr_addr, csr_addr_pre;
    wire [31:0] jalr_predict_offset;
    wire [31:0] isr_addr2, iret_addr2;
    wire [31:0] offset_jal2, offset_beq2;
    wire [31:0] jalr_target_q2, jp_target;
    wire [5:0] br_pc_idx;
    wire [31:0] r1_data_final, r2_data_final;
    wire [31:0] r1_data, r2_data;
    wire [31:0] ld_data_final;
    wire br_fail, success, jalr_fail, jal_flag, jalr_flag, jalr_flag_q, br_pred_taken_q, exc_irq_ret, exc_ecall, exc_ebreak;
    wire exc_irq_ret_ok_w;
    wire flush_bju_exc, exc_jal_misalign_out, exc_illegal_out, exc_ldst_misalign_out, exc_ldst_st_out;
//bju 判定输入级出来的异常（与 flush_bju_pre 同拍、描述同一条）：喂 controller 的异常仲裁
    wire exc_ecall_i_w, exc_ebreak_i_w, exc_illegal_i_w, exc_irq_ret_i_w;
    wire [31:0] exc_pc_i_w;
//controller 输出的 ROB 标记索引（三路 mux 已搬进 controller）
    wire [2:0]  exc_idx_w;
//非对齐故障【自己】的 ROB 索引（跟故障脉冲同拍寄出来）
    wire [2:0] exc_ldst_idx_out;
    wire [31:0] exc_ldst_addr_out;
    wire [31:0] jal_target_e4;
    wire flush_con_exc;
    wire [3:0] exc_cause;
    wire [31:0] exc_pc, exc_tval;
//前置冲刷（早一拍）：由 bju 的组合判定给出，直连 lsu/mulu（不进 flag_bus，见 controller.v）
    wire flush_bju_pre;
//停顿源（逐条，一位一源）：lsu 三条 / mulu 两条 / dcache / icache / bus / rob / pc
    wire stall_lsu_haz_w, stall_lsu_unload_w, stall_lsu_full_w;
    wire loaded, ld_we, lsu_inflight_w;
    wire stall_rob_full_w, stall_pc_redir_w;
    wire flush_con_rob_w, flush_pc_redir_w;
    wire [2:0] flush_idx_w;
    wire flush_rob_trap_w;
    wire [4:0] rd_load;
    wire btb_hit;
//服务 br1/jal 的独立 btb 出口：命中标志给 pc，目标指令与三态选择给 pre_decoder
    wire        bti_hit;
    wire [1:0]  bti_sel_q;
    wire [31:0] bti_inst_q;
    wire [31:0] icache_mem_addr_w, icache_mem_wdata_w, icache_mem_data_w;
    wire [3:0] icache_mem_be_w;
    wire icache_mem_req_w, icache_mem_we_w, icache_mem_valid_w;
    wire csr_wr_en, timi, exc_irq_act, exc_irq_processing, exc_irq;
    wire [2:0] csr_func3;
    wire [31:0] csr_data_wr, csr_data_rd, mcause, csr_result;
    wire [31:0] tim_data_out;
    wire tim_ready;
    wire [31:0] dcache_data_w, dcache_mem_addr_w, dcache_mem_wdata_w, dcache_mem_data_w;
    wire [3:0] dcache_mem_be_w;
    wire dcache_busy_w, dcache_ld_ready_w, dcache_mem_req_w, dcache_mem_we_w, dcache_mem_valid_w;
//dcache 的停顿（busy 的延拓拍，原来在顶层合成 d_hold_int，现在由 dcache 自己输出）。
//★ 它【只给 lsu】：不进 flag_bus 的全核广播 —— "dcache 忙"与"前端该不该冻"是两件事，
//  交给 lsu 之后，dcache 回填期间前端照常推进。
    wire dcache_hold_w;
//RV32M 乘除法单元（与 lsu 流水线同步，独立第三写口）
    wire [31:0] mul_data_final;
    wire mul_loaded, mul_we, stall_mulu_haz_w, stall_mulu_div_w;
    wire [4:0] rd_mul;
//总线层次块
    wire [31:0] bus_addr_out_i, bus_data_out_i;
    wire [3:0] bus_be_out_i;
    wire bus_we_out_i;
    wire bus_valid_out_i;
    always @(*) begin
        bus_addr_out = bus_addr_out_i;
        bus_data_out = bus_data_out_i;
        bus_be_out = bus_be_out_i;
        bus_we_out = bus_we_out_i;
        bus_valid_out = bus_valid_out_i;
    end
    pc u_pc (
        .clk(clk),
        .rst(rst),
        .br1(br1),
        .bti_hit(bti_hit),
        .jal(jal),
        .pre_jalr(pre_jalr),
        .btb_hit(btb_hit),
//★ 重定向输入走"门控后"的版本：判定那拍先只会冲刷，真正跳要等 ROB 排空（见上面排队逻辑）。
//  trap 那一路与 irq 共用落点，已合进 pc_irq_g；exc_w 由 flag_bus[11]（flush_con_exc，精确版）承担
//  冲刷，不再单独进 pc；ROB 满改吃 flag_bus[7]（stall_rob_full），端口已删。
        .exc_irq(exc_irq),
//★ 这两根都换成【交付资格】同源的版本（理由见 pc.v 排队那一块）：武装不能再用裸译码/裸载荷。
        .exc_irq_ret(exc_irq_ret_ok_w),
        .exc_mark(exc_mark_w),
        .rob_empty(rob_empty_w),
        .stall_pc_redir(stall_pc_redir_w),
        .flush_pc_redir(flush_pc_redir_w),
        .flag_bus(flag_bus),
        .jp_target(jp_target),
        .offset_jal2(offset_jal2),
        .offset_jalr2(jalr_predict_offset),
        .offset_beq2(offset_beq2),
        .isr_addr2(isr_addr2),
        .isr_ret_addr2(iret_addr2),
        .pc_addr(pc_addr),
        .aux_addr(aux_addr_0)
    );
    icache u_icache (
        .clk(clk),
        .rst(rst),
        .pc_addr(pc_addr),
        .flush_pc_redir(flush_pc_redir_w),
        .br2(br2),
        .br3(br3),
        .pre_jalr(pre_jalr),
        .btb_hit(btb_hit),
        .jalr_fail(jalr_fail),
        .exc_irq(exc_irq),
        .exc_irq_ret(exc_irq_ret),
        .exc_ecall(exc_ecall),
        .flag_bus(flag_bus),
        .inst_out(icache_inst_w),
        .inst_valid(icache_inst_valid_w),
        .busy(icache_busy_w),
        .mem_req(icache_mem_req_w),
        .mem_we(icache_mem_we_w),
        .mem_addr(icache_mem_addr_w),
        .mem_wdata(icache_mem_wdata_w),
        .mem_be(icache_mem_be_w),
        .mem_valid(icache_mem_valid_w),
        .mem_data(icache_mem_data_w)
    );
    itcm u_itcm (
        .clk(clk),
        .rst(rst),
        .mem_req(icache_mem_req_w),
        .mem_addr(icache_mem_addr_w),
        .mem_data(icache_mem_data_w),
        .mem_valid(icache_mem_valid_w)
    );
    pre_decoder u_pre_decoder (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .inst_valid(icache_inst_valid_w),
        .inst_in(icache_inst_w),
        .bti_sel(bti_sel_q),
        .bti_inst(bti_inst_q),
        .aux_addr_in(aux_addr_0),
        .inst_out(inst_1),
        .rd_out(rd_1),
        .func10_out(func10_1),
        .imm_alu_out(imm_alu_1),
        .br1_in(br1),
        .jalr_pred_addr_in(jalr_predict_offset),
        .br_pred_taken_out(br_pred_taken_1),
        .jalr_pred_addr_out(jalr_pred_addr_1),
        .r1(rs1_1),
        .r2(rs2_1),
        .inst_go(pre_go_w),
        .offset_jal2(offset_jal2),
        .offset_beq2(offset_beq2),
        .aux_addr_out(aux_addr_1),
        .jal(jal),
        .br_en(br_en),
        .jalr(pre_jalr)
    );
    mid_decoder u_mid_decoder (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .inst_in(inst_1),
        .rd_in(rd_1),
        .func10_in(func10_1),
        .imm_alu_in(imm_alu_1),
        .aux_addr_in(aux_addr_1),
        .br_pred_taken_in(br_pred_taken_1),
        .jalr_pred_addr_in(jalr_pred_addr_1),
        .r1_in(rs1_1),
        .r2_in(rs2_1),
        .inst_out(inst_2),
        .rd_out(rd_2),
        .func10_out(func10_2),
        .imm_alu_out(imm_alu_2),
        .aux_addr_out(aux_addr_2),
        .br_pred_taken_out(br_pred_taken_2),
        .jalr_pred_addr_out(jalr_pred_addr_2),
        .r1_out(rs1_2),
        .r2_out(rs2_2)
    );
    post_decoder u_post_decoder (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .func10(func10_2),
        .imm_alu_in(imm_alu_2),
        .csr_wr_en(csr_wr_en),
        .csr_addr(csr_addr),
        .csr_addr_pre(csr_addr_pre),
        .csr_data(csr_data_wr),
        .csr_func3(csr_func3),
        .rd_in(rd_2),
        .rs1_in(rs1_2),
        .rs2_in(rs2_2),
//★ 操作数不再经过本级的寄存器：底值直接取 regfile 的【寄存读】（它正好提供
//  "mid 出拍 → 下一拍"那一拍延迟，与载荷同拍），非寄存器操作数在 pd 内组合覆盖，
//  再作为 forw 点② 的默认值直送执行单元。
        .pc_operand_in(aux_addr_2),
        .offset_beq0_aux(imm_alu_2),
        .r1_imm_val(r1_imm_val_w), .r2_imm_val(r2_imm_val_w),
        .r1_imm_sel(r1_imm_sel_w), .r2_imm_sel(r2_imm_sel_w),
        .inst_in(inst_2),
        .aux_addr_in(aux_addr_2),
        .br_pred_taken_in(br_pred_taken_2),
        .jalr_pred_addr_in(jalr_pred_addr_2),
        .br_flag(br_flag),
        .beq_off_q2(beq_off_q2),
        .br_pred_taken_out(br_pred_taken_3),
        .jalr_pred_addr_out(jalr_pred_addr_3),
        .exc_jal_misalign_out(exc_jal_misalign_out),
        .jal_target_out(jal_target_e4),
        .exc_illegal_out(exc_illegal_out),
        .exc_irq_ret(exc_irq_ret),
        .exc_ecall(exc_ecall),
        .exc_ebreak(exc_ebreak),
        .jal_flag(jal_flag),
        .jalr_flag(jalr_flag),
        .rd_out(rd_3),
        .issue_we(issue_we_w),
        .issue_rd(issue_rd_w),
        .idx_in(rob_alloc_idx_w),
        .issue_idx(issue_idx_w),
        .alloc_gen_in(alloc_gen_w),
        .issue_gen(issue_gen_w),
//前送槽号：当拍由 rob 按槽序扫出，与 issue_idx 同一个 payload_go 沿锁存成载荷
        .fwd_slot1_in(fwd_slot1_w), .fwd_slot2_in(fwd_slot2_w),
        .fwd_hit1_in(fwd_hit1_w),   .fwd_hit2_in(fwd_hit2_w),
        .sel_slot1(sel_slot1_w), .sel_slot2(sel_slot2_w),
        .sel_v1(sel_v1_w),       .sel_v2(sel_v2_w),
        .payload_go(payload_go_w),
        .payload_go_q(payload_go_q_w),
        .opc_out(opc_3),
        .fn10_out(fn10_3),
        .fn10_ls_out(fn10_ls_3),
        .off_mem_out(off_mem_3),
        .rs1_out(rs1_3),
        .rs2_out(rs2_3),
        .alu_func4(alu_func4),
        .we(we_3),
        .aux_addr_out(aux_addr_3)
    );
//分支/跳转判定单元：判定源与 alu 的输入同源（前送后的操作数 / alu_func4），
//比较与加法在本拍做，结果、落点在下一拍生效；flush_bju_pre 是同一判定的组合版本、早一拍
    bju u_bju (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .r1_data_in(r1_data_final),
        .idx_in(issue_idx_w),
        .r2_data_in(r2_data_final),
        .alu_func4_in(alu_func4),
        .br_flag_in(br_flag),
        .jalr_flag_in(jalr_flag),
        .aux_addr_in(aux_addr_3),
        .beq_off_in(beq_off_q2),
        .jalr_pred_addr_in(jalr_pred_addr_3),
        .br_pred_taken_in(br_pred_taken_3),
        .success(success),
        .br_fail(br_fail),
        .jalr_fail(jalr_fail),
        .jp_target(jp_target),
        .jalr_target_q2(jalr_target_q2),
        .flush_bju_exc(flush_bju_exc),
        .exc_bju(exc_bju_w),
        .idx_i(bju_idx_i_w),
        .exc_pc_q(bju_exc_pc_w),
        .older_q(bju_older_w),
        .idx_q(bju_idx_q_w),
        .br_pc_idx(br_pc_idx),
        .jalr_flag_q(jalr_flag_q),
        .br_pred_taken_q(br_pred_taken_q),
        .exc_jal_misalign_in(exc_jal_misalign_out),
        .jal_target_in(jal_target_e4),
//异常源进判定输入寄存级（与操作数/控制位同沿）
        .exc_ecall_in(exc_ecall),
        .exc_ebreak_in(exc_ebreak),
        .exc_illegal_in(exc_illegal_out),
        .exc_irq_ret_in(exc_irq_ret),
        .exc_ecall_i(exc_ecall_i_w),
        .exc_ebreak_i(exc_ebreak_i_w),
        .exc_illegal_i(exc_illegal_i_w),
        .exc_irq_ret_i(exc_irq_ret_i_w),
        .exc_pc_i(exc_pc_i_w),
        .flush_bju_pre(flush_bju_pre)
    );
    lsu u_lsu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .flush_bju_pre(flush_bju_pre),
        .flush_idx(flush_idx_w),
        .payload_go(payload_go_w),
//★ 三个执行单元的输入级数对齐：lsu 也吃 decoder 载荷（原来吃 mem_buf 组合输出，早一级）
        .opcode(opc_3),
        .func10(fn10_ls_3),
        .rd_in(rd_3),
        .r1_post(rs1_3),
        .r2_post(rs2_3),
        .r1_data_final(r1_data_final),
        .r2_data_final(r2_data_final),
        .offset_load0(off_mem_3),
        .offset_store0(off_mem_3),
        .bus_data_ext(bus_data_in_ext),
        .bus_data_dcache(dcache_data_w),
        .bus_data_tim(tim_data_out),
        .ready_dcache(dcache_ld_ready_w),
        .dcache_hold(dcache_hold_w),
        .mem_inflight(lsu_inflight_w),
        .ready_tim(tim_ready),
        .ready_ext(bus_loaded_in),
        .bus_addr_out(bus_addr_out_i),
        .bus_data_out(bus_data_out_i),
        .bus_be_out(bus_be_out_i),
        .bus_we_out(bus_we_out_i),
        .bus_valid_out(bus_valid_out_i),
        .exc_ldst_misalign_out(exc_ldst_misalign_out),
        .exc_ldst_st_out(exc_ldst_st_out),
        .exc_ldst_idx_out(exc_ldst_idx_out),
//索引与操作数同源：既然吃的是 decoder 载荷，索引就是它自己的 issue_idx（三者统一）
        .idx_in(issue_idx_w),
        .gen_in(issue_gen_w),
        .ld_idx(ld_idx_w),
        .ld_gen(ld_gen_w),
        .exc_ldst_addr_out(exc_ldst_addr_out),
        .ld_data_out(ld_data_final),
        .loaded(loaded),
        .ld_we(ld_we),
        .stall_lsu_haz(stall_lsu_haz_w),
        .stall_lsu_unload(stall_lsu_unload_w),
        .stall_lsu_full(stall_lsu_full_w),
        .rd_load(rd_load)
    );
//RV32M：与 lsu 同拍取 mem_buf 输出，自己从 opcode/func10 判 M（不用额外派发标志）
    mulu u_mulu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .flush_idx(flush_idx_w),
        .payload_go(payload_go_w),
//★ 同样对齐到 decoder 载荷（与 lsu/alu 同级）
        .opcode(opc_3),
        .func10(fn10_3),
        .rd_in(rd_3),
        .r1_post(rs1_3),
        .r2_post(rs2_3),
//用 mulu 自己那一份镜像（_mul）：它由 pre_decoder 的 mul 标志单独填充，
//既保证 M 指令一定拿到操作数（_dec/_lsu 是按各自的标志填的），又把扇出按消费者拆开。
        .r1_data_final(r1_data_final),
        .r2_data_final(r2_data_final),
        .mul_data_out(mul_data_final),
        .mul_loaded(mul_loaded),
        .mul_we(mul_we),
        .taken_mul(wp_taken_mul),
        .rd_mul(rd_mul),
        .idx_in(issue_idx_w),
        .gen_in(issue_gen_w),
        .mul_idx(mul_idx_w),
        .mul_gen(mul_gen_w),
        .stall_mulu_haz(stall_mulu_haz_w),
        .stall_mulu_div(stall_mulu_div_w)
    );
    bra_predict u_bra_predict (
        .clk(clk),
        .rst(rst),
        .pc_addr_in(pc_addr),
        .jalr_target_q(jalr_target_q2),
        .br_pc_idx(br_pc_idx),
        .success(success),
        .br_fail(br_fail),
        .br_en(br_en),
        .jalr_flag(jalr_flag_q),
        .jal(jal),
        .inst_in(icache_inst_w),
        .inst_valid(icache_inst_valid_w),
        .flag_bus(flag_bus),
        .br_pred_taken_in(br_pred_taken_q),
        .bti_hit(bti_hit),
        .bti_inst_q(bti_inst_q),
        .bti_sel_q(bti_sel_q),
        .br1(br1),
        .br2(br2),
        .br3(br3),
        .jalr_predict_offset(jalr_predict_offset),
        .jalr(btb_hit)
    );
//写口级：三个结果口 → 两条物理写口（完成即写、同 rd 同拍年轻者胜）+ 处置回报 + 落地广播
//前送级：**只剩一级数据 mux** —— 裁决（"我这一拍是不是在给消费者供值"）已在三个单元里做完
//（alu 在结果寄存沿、mulu/lsu 与各自结果口同拍），这里只按算好的命中位选源。
//  默认源 = regfile 的寄存读（含 5 源读侧旁路）与非寄存器操作数的常量载荷，两者也在这一次选择里落定。
    forw u_forw (
        .clk(clk),
        .rst(rst),
        .data_alu(result_4),
        .data_mul(mul_data_final),
        .data_ld(ld_data_final),
        .we_alu(we_4), .we_mul(mul_we), .we_ld(loaded),
        .idx_alu(alu_idx_w), .idx_mul(mul_idx_w), .idx_ld(ld_idx_w),
        .sel_slot1(sel_slot1_w), .sel_slot2(sel_slot2_w),
        .sel_v1(sel_v1_w),       .sel_v2(sel_v2_w),
        .r1_data(r1_data), .r2_data(r2_data),
        .r1_imm_val(r1_imm_val_w), .r2_imm_val(r2_imm_val_w),
        .r1_imm_sel(r1_imm_sel_w), .r2_imm_sel(r2_imm_sel_w),
        .r1_data_final(r1_data_final),
        .r2_data_final(r2_data_final)
    );
    alu u_alu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .we_in(we_3),
        .jal_flag(jal_flag),
        .jalr_flag(jalr_flag),
        .cs_wr_en(csr_wr_en),
        .rd_in(rd_3),
        .idx_in(issue_idx_w),
        .gen_in(issue_gen_w),
        .alu_func4(alu_func4),
        .aux_addr_in(aux_addr_3),
        .csr_func3(csr_func3),
        .r1_data(r1_data_final),
        .r2_data(r2_data_final),
        .cs_data(csr_data_rd),
        .result_csr(csr_result),
        .rd_out(rd_4),
        .idx_out(alu_idx_w),
        .gen_out(alu_gen_w),
        .result(result_4),
        .we(we_4)
    );
//tb 的事件探针按这两个名字抓"寄存器堆写口" ⇒ 让它们跟着写口级的两条口走（语义不变）
    assign wp_taken_mul = mul_we;
    assign we_a_w = rob_rwe0_w;
    assign rd_a_w = rob_rrd0_w;
    assign data_a_w = rob_rdt0_w;
    assign we_b_w = rob_rwe1_w;
    assign rd_b_w = rob_rrd1_w;
    assign data_b_w = rob_rdt1_w;

    rob u_rob (
        .clk(clk), .rst(rst),
        .alloc_en(payload_go_w), .alloc_idx(rob_alloc_idx_w), .full(stall_rob_full_w),
        .alloc_gen(alloc_gen_w),
//完成口 = 三个单元的结果口直连（值 + 索引 + 世代同拍同源）；ROB 按索引把 wr/data 一起写进那一项
        .alu_done(we_4),   .alu_idx(alu_idx_w), .alu_data(result_4),       .alu_gen(alu_gen_w),
        .mul_done(mul_we), .mul_idx(mul_idx_w), .mul_data(mul_data_final), .mul_gen(mul_gen_w),
        .ld_done(ld_we),   .ld_idx(ld_idx_w),   .ld_data(ld_data_final),   .ld_gen(ld_gen_w),
//前送值读口①（正常读，索引 = 本模块扫描出的槽）与②（停顿重读，索引 = regfile 的锁存槽）
        .fwd_data1(rob_fwd_data1_w), .fwd_data2(rob_fwd_data2_w),
        .fwd_done1(rob_fwd_done1_w), .fwd_done2(rob_fwd_done2_w),
        .st_slot1(st_slot1_w),  .st_slot2(st_slot2_w),
        .st_data1(rob_st_data1_w), .st_data2(rob_st_data2_w),
        .st_done1(rob_st_done1_w), .st_done2(rob_st_done2_w),
//提交口：驱动寄存器堆的两条写口（口 A = head、口 B = head+1）
        .cmt_we0(rob_rwe0_w), .cmt_rd0(rob_rrd0_w), .cmt_data0(rob_rdt0_w),
        .cmt_we1(rob_rwe1_w), .cmt_rd1(rob_rrd1_w), .cmt_data1(rob_rdt1_w),
        .alloc_we(issue_we_w),
        .alloc_rd(issue_rd_w),
//前送槽扫描：消费者是下一拍进载荷那条（本拍还在 pre 级）⇒ 用它的 rs1_1/rs2_1；
//本模块按程序序分配 ⇒ 扫描那一拍在册的槽全是比它老的，"比我老"不需要比较。
//扫描吃【上一级】的 rs（pre_decoder 的组合版），结果在 rob 里打一拍 ⇒ 读拍只剩取数+选源
//槽扫描吃【pre_decoder 的输出触发器】：读拍在 mid 那一级，所以扫描提前一拍、起点是普通触发器
//（不是 icache 的 BRAM 输出寄存器 —— 那 2.45ns 的 clock-to-out 是白吃的）。
        .scan_rs1(rs1_1), .scan_rs2(rs2_1),
        .scan_go(pre_go_w),
        .fwd_slot1(fwd_slot1_w), .fwd_slot2(fwd_slot2_w),
        .fwd_hit1(fwd_hit1_w),   .fwd_hit2(fwd_hit2_w),
//分支/跳转误预测一律【部分冲刷】：边界号 = bju 随判定锁存的那条自己的项。
//  边界项若已失效（分支已退，说明没有更老的活项）⇒ ROB 内部自动退化成全冲。
        .flush_con_rob(flush_con_rob_w), .flush_idx(flush_idx_w), .flush_all(1'b0),
        .flush_con_exc(flush_con_exc),
//标记落哪一项：三路 mux 已在 controller 内按各自相位选好（见 controller 的 exc_idx）
        .exc_en(exc_mark_w),
        .exc_idx(exc_idx_w),
        .exc_cause(exc_cause), .exc_pc(exc_pc), .exc_tval(exc_tval),
        .trap_fire(flush_rob_trap_w),
        .trap_cause(rob_trap_cause_w), .trap_pc(rob_trap_pc_w), .trap_tval(rob_trap_tval_w),
        .occupancy(rob_occ_w), .empty(rob_empty_w),
        .head_p(rob_head_w)
    );
    regfile u_regfile (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
//读口地址由 mid_decoder 那一级给出（比 decoder 载荷早一拍），读值寄存后随载荷往下走
        .r1(rs1_2),
        .r2(rs2_2),
        .we_a(rob_rwe0_w),
        .rd_a(rob_rrd0_w),
        .data_a(rob_rdt0_w),
        .we_b(rob_rwe1_w),
        .rd_b(rob_rrd1_w),
        .data_b(rob_rdt1_w),
//★ 读侧旁路也吃三个单元结果口（共 5 源）：结果离开单元口那一拍还没落进阵列，
//  而消费者的操作数取自"读锁存沿" ⇒ 那一格只有这里补得住（原来由 forw 点① 承担）。
//  选源用 rob 的槽扫描（与 payload 的 sel_slot 同源、取当拍组合版）：扫描口读的就是本模块的
//  读口地址 rs1_1/rs2_1，同一条指令，所以当拍就能用。
        .we_alu(we_4), .data_alu(result_4),
        .we_mul(mul_loaded), .data_mul(mul_data_final),
        .we_ld(loaded), .data_ld(ld_data_final),
        .idx_alu(alu_idx_w), .idx_mul(mul_idx_w), .idx_ld(ld_idx_w),
        .fwd_data1(rob_fwd_data1_w), .fwd_data2(rob_fwd_data2_w),
        .fwd_done1(rob_fwd_done1_w), .fwd_done2(rob_fwd_done2_w),
        .st_slot1(st_slot1_w), .st_slot2(st_slot2_w),
        .st_data1(rob_st_data1_w), .st_data2(rob_st_data2_w),
        .st_done1(rob_st_done1_w), .st_done2(rob_st_done2_w),
        .fwd_slot1(fwd_slot1_w), .fwd_slot2(fwd_slot2_w),
        .fwd_hit1(fwd_hit1_w),   .fwd_hit2(fwd_hit2_w),
        .r1_data(r1_data),
        .r2_data(r2_data)
    );
    dcache u_dcache (
        .clk(clk),
        .rst(rst),
        .bus_addr_in(bus_addr_out_i),
        .bus_data_in(bus_data_out_i),
        .bus_be_in(bus_be_out_i),
        .bus_we_in(bus_we_out_i),
        .bus_data_out(dcache_data_w),
        .ld_ready(dcache_ld_ready_w),
        .busy(dcache_busy_w),
        .hold(dcache_hold_w),
        .mem_req(dcache_mem_req_w),
        .mem_we(dcache_mem_we_w),
        .mem_addr(dcache_mem_addr_w),
        .mem_wdata(dcache_mem_wdata_w),
        .mem_be(dcache_mem_be_w),
        .mem_data(dcache_mem_data_w),
        .mem_valid(dcache_mem_valid_w),
        .mem_ready(1'b1)
    );
    dtcm u_dtcm (
        .clk(clk),
        .rst(rst),
        .mem_req(dcache_mem_req_w),
        .mem_we(dcache_mem_we_w),
        .mem_addr(dcache_mem_addr_w),
        .mem_wdata(dcache_mem_wdata_w),
        .mem_be(dcache_mem_be_w),
        .mem_data(dcache_mem_data_w),
        .mem_valid(dcache_mem_valid_w),
        .mem_ready()
    );
    tim_in u_tim_in (
        .clk(clk),
        .rst(rst),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_out),
        .bus_data_out(tim_data_out),
        .timi(timi),
        .ready(tim_ready)
    );
    controller u_controller (
        .clk(clk),
        .rst(rst),
        .br2(br2),
        .br3(br3),
        .jalr_fail(jalr_fail),
        .jal(jal),
        .pre_jalr(pre_jalr),
        .btb_hit(btb_hit),
        .br1(br1),
        .exc_irq_ret(exc_irq_ret),
        .exc_ecall(exc_ecall),
        .flush_bju_exc(flush_bju_exc),
//异常仲裁源（原 trap_unit 的例化并入 controller）
        .bju_pc_in(bju_exc_pc_w),
        .exc_irq_ret_ok(exc_irq_ret_ok_w),
        .bju_tgt_in(jp_target),
        .bju_older_in(bju_older_w),
//★ 不再有 flush_bju_pre：ecall/ebreak/illegal/mret 改吃 bju 的【判定输入级】输出（同拍配对）
        .exc_ecall_i(exc_ecall_i_w),
        .exc_ebreak_i(exc_ebreak_i_w),
        .exc_illegal_i(exc_illegal_i_w),
        .exc_irq_ret_i(exc_irq_ret_i_w),
        .exc_pc_i(exc_pc_i_w),
        .bju_idx_i(bju_idx_i_w),
        .exc_ldst_misalign_in(exc_ldst_misalign_out),
        .exc_ldst_st_in(exc_ldst_st_out),
        .exc_ldst_addr_in(exc_ldst_addr_out),
        .exc_pc_in(aux_addr_3),
//ROB 冲刷边界：故障项自己的索引 / E4 载荷（中断边界）/ bju 判定那条
        .exc_ldst_idx_in(exc_ldst_idx_out),
        .issue_idx_in(issue_idx_w),
        .bju_idx_in(bju_idx_q_w),
        .flush_con_exc(flush_con_exc),
        .exc_mark(exc_mark_w),
//ROB 标记索引（三路 mux 已在 controller 内）
        .exc_idx(exc_idx_w),
        .exc_cause(exc_cause),
        .exc_pc(exc_pc),
        .exc_tval(exc_tval),
        .rob_trap(flush_rob_trap_w),
        .rob_trap_cause(rob_trap_cause_w),
        .rob_trap_pc(rob_trap_pc_w),
        .rob_trap_tval(rob_trap_tval_w),
        .flush_con_rob(flush_con_rob_w),
        .flush_idx(flush_idx_w),
//八条停顿源：一位一源，controller 只拼装、不做或运算
        .stall_lsu_haz(stall_lsu_haz_w),
        .stall_lsu_full(stall_lsu_full_w),
        .stall_mulu_haz(stall_mulu_haz_w),
        .stall_mulu_div(stall_mulu_div_w),
        .stall_icache_miss(icache_busy_w),
        .stall_bus_hold(bus_hold_in),
        .stall_rob_full(stall_rob_full_w),
        .stall_pc_redir(stall_pc_redir_w),
        .lsu_inflight(lsu_inflight_w),
        .csr_wr_en(csr_wr_en),
        .exti(exti),
        .timi(timi),
        .softi(1'b0),
        .csr_addr(csr_addr),
        .csr_addr_pre(csr_addr_pre),
        .csr_data_in(csr_result),
//中断的 mepc 取 pre_decoder 的 aux_addr_1（那条指令"地址+4"）：它是【正要进发射级、
//还没被发出去】的那一条 = 中断返回点。取指 PC 在前端等 ROB 排空期间会漂，不能用。
        .pc_addr_in(aux_addr_2),
        .csr_data_out(csr_data_rd),
        .isr_addr2(isr_addr2),
        .mcause(mcause),
        .exc_irq_act(exc_irq_act),
        .exc_irq_processing(exc_irq_processing),
        .exc_irq(exc_irq),
        .iret_addr2(iret_addr2),
        .flag_bus(flag_bus)
    );
endmodule
