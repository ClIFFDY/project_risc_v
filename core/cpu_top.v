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
    wire icache_busy_w, icache_busy_q;
    wire br1, br2, br3;
    (* max_fanout = 32 *) wire [4:0] rs1_1, rs2_1;
    wire [31:0] aux_addr_1;
    wire jal, br_en, pre_jalr;
    wire br_pred_taken_1;
    wire [31:0] jalr_pred_addr_1;
    wire [31:0] inst_2;
    (* max_fanout = 32 *) wire [4:0] rs1_2, rs2_2, rd_2;
    wire [4:0] imm5_csr_2;
    wire [6:0] opcode_2, opcode_lsu_2;
    wire [9:0] func10_2, func10_lsu_2;
    wire [31:0] imm_alu_2, pc_operand_2;
    wire [11:0] imm12_csr_2;
    wire [31:0] offset_jalr0_2, offset_beq0_aux_2, offset_load0_2, offset_store0_2;
    wire [31:0] aux_addr_2;
    wire br_pred_taken_2;
    wire [31:0] jalr_pred_addr_2;
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
//  lsu / mulu 采的是 E4【组合输入】(rd_2/opcode_2) ⇒ 用实时计数器 tag_w —— 计数器现在
//  只在"载荷推进且有写"那一拍加一次，这条指令等在输入口期间它不动，所以 tag_w 就是发给它自己的号；
//  wbu 的 alu 口采的是 E4【寄存载荷】(rd_4/we_4) ⇒ 必须用与载荷同步锁存出来的 issue_tag_w
//  （从前它接 tag_w = 下一跳的号 ⇒ 老指令看起来比年轻指令还新，杀老写杀反）。
//ROB（重排序缓冲）：分配口/完成口/退口。阶段 B-1 先只接线、退口暂不驱动寄存器堆（行为零变化）。
    wire [2:0]  rob_alloc_idx_w;
    wire [2:0]  issue_idx_w;
    wire        payload_go_w, payload_go_q_w;
//写口级（wport）：三条结果口 → 两条写口 + 处置回报 + 落地广播
    wire [2:0]  rob_head_w;
    wire        wp_we_a, wp_we_b, wp_taken_mul;
    wire [4:0]  wp_rd_a, wp_rd_b;
    wire [31:0] wp_data_a, wp_data_b;
    wire        wp_fin_alu, wp_fin_mul, wp_fin_ld;
    wire [2:0]  wp_fin_alu_idx, wp_fin_mul_idx, wp_fin_ld_idx;
    wire        wp_b0_we, wp_b1_we;
    wire [4:0]  wp_b0_rd, wp_b1_rd;
    wire [2:0]  wp_b0_idx, wp_b1_idx;
//单元的杀老写标记（写口级拿到它会"不写但照常放行回报"）
    wire        kill_mul_w, kill_ld_w, kill_alu_w;
    wire        rob_empty_w;
    wire [3:0]  rob_trap_cause_w;
    wire [31:0] rob_trap_pc_w, rob_trap_tval_w;
    wire        exc_mark_w, exc_bju_w;
    wire [2:0]  bju_idx_q_w, bju_idx_i_w;
    wire [31:0] r1_data_3, r2_data_3;
    wire [31:0] r1_data_mid_w, r2_data_mid_w;
    wire        r1_reg_en_3, r2_reg_en_3;
    wire [6:0]  opc_3;
    wire [9:0]  fn10_3, fn10_ls_3;
    wire [31:0] off_mem_3;
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
    wire [13:0] flag_bus;
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
//dcache 的 busy 延拓拍（原来在顶层合成 d_hold_int，现在由 dcache 自己输出）＝ stall_dcache_miss
    wire stall_dcache_miss_w;
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
        .jal(jal),
        .pre_jalr(pre_jalr),
        .btb_hit(btb_hit),
//★ 重定向输入走"门控后"的版本：判定那拍先只会冲刷，真正跳要等 ROB 排空（见上面排队逻辑）。
//  trap 那一路与 irq 共用落点，已合进 pc_irq_g；exc_w 由 flag_bus[13]（flush_con_exc）承担
//  冲刷，不再单独进 pc；ROB 满改吃 flag_bus[9]（stall_rob_full），端口已删。
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
        .br1(br1),
        .br2(br2),
        .br3(br3),
        .jal(jal),
        .pre_jalr(pre_jalr),
        .btb_hit(btb_hit),
        .jalr_fail(jalr_fail),
        .exc_irq(exc_irq),
        .exc_irq_ret(exc_irq_ret),
        .exc_ecall(exc_ecall),
        .flag_bus(flag_bus),
        .inst_out(icache_inst_w),
        .busy(icache_busy_w),
        .busy_q(icache_busy_q),
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
        .inst_in(icache_inst_w),
        .aux_addr_in(aux_addr_0),
        .inst_out(inst_1),
        .br1_in(br1),
        .jalr_pred_addr_in(jalr_predict_offset),
        .br_pred_taken_out(br_pred_taken_1),
        .jalr_pred_addr_out(jalr_pred_addr_1),
        .r1(rs1_1),
        .r2(rs2_1),
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
        .inst_out(inst_2),
        .func10_out(func10_2),
        .imm_alu_out(imm_alu_2),
        .imm12_csr_out(imm12_csr_2),
        .imm5_csr_out(imm5_csr_2),
        .rd_out(rd_2),
        .opcode_out(opcode_2),
        .offset_jalr0_out(offset_jalr0_2),
        .offset_beq0_aux_out(offset_beq0_aux_2),
        .pc_operand_out(pc_operand_2),
        .aux_addr_in(aux_addr_1),
        .aux_addr_out(aux_addr_2),
        .br_pred_taken_in(br_pred_taken_1),
        .br_pred_taken_out(br_pred_taken_2),
        .jalr_pred_addr_in(jalr_pred_addr_1),
        .jalr_pred_addr_out(jalr_pred_addr_2),
        .opcode_lsu_out(opcode_lsu_2),
        .func10_lsu_out(func10_lsu_2),
        .offset_load0_out(offset_load0_2),
        .offset_store0_out(offset_store0_2),
        .r1_in(rs1_1),
        .r1_out(rs1_2),
        .r2_in(rs2_1),
        .r2_out(rs2_2)
    );
    post_decoder u_post_decoder (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .func10(func10_2),
        .imm_alu_in(imm_alu_2),
        .imm12_csr_in(imm12_csr_2),
        .imm5_csr_in(imm5_csr_2),
        .csr_wr_en(csr_wr_en),
        .csr_addr(csr_addr),
        .csr_addr_pre(csr_addr_pre),
        .csr_data(csr_data_wr),
        .csr_func3(csr_func3),
        .rd_in(rd_2),
        .rs1_in(rs1_2),
        .rs2_in(rs2_2),
        .opcode(opcode_2),
        .offset_jalr0(offset_jalr0_2),
        .offset_beq0_aux(offset_beq0_aux_2),
        .pc_operand_in(pc_operand_2),
//★ 点①前送的结果必须接进来：pd 的操作数载体默认就是它（寄存器操作数随载荷带到点②）
//  这两根线漏了 ⇒ 默认值写进 X ⇒ 所有 rs1 操作数（sw/lh/beq…）全是 X（实测整核挂死）
        .r1_data_in(r1_data_mid_w),
        .r2_data_in(r2_data_mid_w),
//stall 期间的回灌：前送当前输出（门控在 post_decoder 里用本级自己的 r*_reg_en）
        .r1_data_fb(r1_data_final), .r2_data_fb(r2_data_final),
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
        .payload_go(payload_go_w),
        .payload_go_q(payload_go_q_w),
        .r1_data_out(r1_data_3),
        .r2_data_out(r2_data_3),
        .r1_reg_en(r1_reg_en_3),
        .r2_reg_en(r2_reg_en_3),
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
        .flush_bju_pre(flush_bju_pre)
    );
    lsu u_lsu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .flush_bju_pre(flush_bju_pre),
        .rob_head(rob_head_w),
        .b0_we(wp_b0_we), .b0_rd(wp_b0_rd), .b0_idx(wp_b0_idx),
        .b1_we(wp_b1_we), .b1_rd(wp_b1_rd), .b1_idx(wp_b1_idx),
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
        .ld_idx(ld_idx_w),
        .exc_ldst_addr_out(exc_ldst_addr_out),
        .ld_data_out(ld_data_final),
        .loaded(loaded),
        .ld_we(ld_we),
        .stall_lsu_haz(stall_lsu_haz_w),
        .stall_lsu_unload(stall_lsu_unload_w),
        .stall_lsu_full(stall_lsu_full_w),
        .rd_load(rd_load),
        .kill_ld(kill_ld_w)
    );
//RV32M：与 lsu 同拍取 mem_buf 输出，自己从 opcode/func10 判 M（不用额外派发标志）
    mulu u_mulu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .flush_bju_pre(flush_bju_pre),
        .rob_head(rob_head_w),
        .b0_we(wp_b0_we), .b0_rd(wp_b0_rd), .b0_idx(wp_b0_idx),
        .b1_we(wp_b1_we), .b1_rd(wp_b1_rd), .b1_idx(wp_b1_idx),
        .taken_mul(wp_taken_mul),
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
        .kill_mul(kill_mul_w),
        .rd_mul(rd_mul),
        .idx_in(issue_idx_w),
        .mul_idx(mul_idx_w),
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
        .br_pred_taken_in(br_pred_taken_q),
        .br1(br1),
        .br2(br2),
        .br3(br3),
        .jalr_predict_offset(jalr_predict_offset),
        .jalr(btb_hit)
    );
//写口级：三个结果口 → 两条物理写口（完成即写、同 rd 同拍年轻者胜）+ 处置回报 + 落地广播
    wport u_wport (
        .rob_head(rob_head_w),
//alu 那一笔：alu.v 里寄存一拍（算完的下一拍），正好是紧邻消费者取用那一拍
        .we_alu(we_4), .rd_alu(rd_4), .data_alu(result_4), .idx_alu(alu_idx_w),
//mulu / lsu 各自的结果口（自带保持）
        .we_mul(mul_we), .rd_mul(rd_mul), .data_mul(mul_data_final), .idx_mul(mul_idx_w),
        .we_ld(ld_we), .rd_ld(rd_load), .data_ld(ld_data_final), .idx_ld(ld_idx_w),
        .kill_mul(kill_mul_w), .kill_ld(kill_ld_w), .kill_alu(kill_alu_w),
        .bju_exc(exc_bju_w), .bju_idx_i(bju_idx_i_w), .bju_idx_q(bju_idx_q_w),
        .flag_bus(flag_bus),
        .we_a(wp_we_a), .rd_a(wp_rd_a), .data_a(wp_data_a),
        .we_b(wp_we_b), .rd_b(wp_rd_b), .data_b(wp_data_b),
        .taken_mul(wp_taken_mul),
        .fin_alu(wp_fin_alu), .fin_alu_idx(wp_fin_alu_idx),
        .fin_mul(wp_fin_mul), .fin_mul_idx(wp_fin_mul_idx),
        .fin_ld(wp_fin_ld), .fin_ld_idx(wp_fin_ld_idx),
        .b0_we(wp_b0_we), .b0_rd(wp_b0_rd), .b0_idx(wp_b0_idx),
        .b1_we(wp_b1_we), .b1_rd(wp_b1_rd), .b1_idx(wp_b1_idx)
    );
//前送级：两个前送点，共用同一组前送源（两个写回口）
//  点①：寄存器堆数据 → post_decoder 之间（消费者 = mem_buf 那条）
//  点②：post_decoder → 三个执行单元之间（消费者 = decoder 载荷那条）
    forw u_forw (
        .clk(clk),
        .rst(rst),
        .rob_head(rob_head_w),
        .idx_mid(rob_alloc_idx_w), .idx_post(issue_idx_w),
//★ 三条源 = 三个执行单元的结果口（各自带 ROB 索引）。乱序写回下"谁更年轻"只由索引给，
//  不能再按端口身份/语句顺序定优先级（旧的"口 B 更年轻"只对按序退成立）。
        .we_alu(we_4), .rd_alu(rd_4), .data_alu(result_4), .idx_alu(alu_idx_w),
        .we_mul(mul_loaded), .rd_mul(rd_mul), .data_mul(mul_data_final), .idx_mul(mul_idx_w),
        .we_ld(loaded), .rd_ld(rd_load), .data_ld(ld_data_final), .idx_ld(ld_idx_w),
        .r1_mid(rs1_2),
        .r2_mid(rs2_2),
        .r1_data_mid_in(r1_data),
        .r2_data_mid_in(r2_data),
        .r1_data_mid(r1_data_mid_w),
        .r2_data_mid(r2_data_mid_w),
        .r1_post(rs1_3),
        .r2_post(rs2_3),
        .r1_en(r1_reg_en_3),
        .r2_en(r2_reg_en_3),
        .r1_data_post_in(r1_data_3),
        .r2_data_post_in(r2_data_3),
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
        .bju_idx(bju_idx_q_w),
        .alu_func4(alu_func4),
        .aux_addr_in(aux_addr_3),
        .csr_func3(csr_func3),
        .r1_data(r1_data_final),
        .r2_data(r2_data_final),
        .cs_data(csr_data_rd),
        .result_csr(csr_result),
        .rd_out(rd_4),
        .idx_out(alu_idx_w),
        .kill_out(kill_alu_w),
        .result(result_4),
        .we(we_4)
    );
//tb 的事件探针按这两个名字抓"寄存器堆写口" ⇒ 让它们跟着写口级的两条口走（语义不变）
    assign we_a_w = wp_we_a;
    assign rd_a_w = wp_rd_a;
    assign data_a_w = wp_data_a;
    assign we_b_w = wp_we_b;
    assign rd_b_w = wp_rd_b;
    assign data_b_w = wp_data_b;

    rob u_rob (
        .clk(clk), .rst(rst),
        .alloc_en(payload_go_w), .alloc_idx(rob_alloc_idx_w), .full(stall_rob_full_w),
//完成口 = 写口级的"这一笔已处置"（落地 / 被杀 / 不写），不再带数据（数据在写口那条路上）
        .alu_done(wp_fin_alu), .alu_idx(wp_fin_alu_idx),
        .mul_done(wp_fin_mul), .mul_idx(wp_fin_mul_idx),
        .ld_done(wp_fin_ld),   .ld_idx(wp_fin_ld_idx),
        .alloc_we(issue_we_w),
//分支/跳转误预测一律【部分冲刷】：边界号 = bju 随判定锁存的那条自己的项。
//  边界项若已失效（分支已退，说明没有更老的活项）⇒ ROB 内部自动退化成全冲。
        .flush_con_rob(flush_con_rob_w), .flush_idx(flush_idx_w), .flush_all(1'b0),
        .flush_con_exc(flush_con_exc),
//标记落哪一项：非对齐那条用故障自带的索引（它比载荷晚一拍）；其余三条与 issue_idx 同拍
        .exc_en(exc_mark_w),
        .exc_idx(flush_bju_exc ? bju_idx_q_w
                 : (exc_ldst_misalign_out ? exc_ldst_idx_out : issue_idx_w)),
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
//读口地址由 pre_decoder 那一级给出（比 decoder 载荷早两级），读值寄存后随载荷往下走
        .r1(rs1_1),
        .r2(rs2_1),
        .we_a(wp_we_a),
        .rd_a(wp_rd_a),
        .data_a(wp_data_a),
        .we_b(wp_we_b),
        .rd_b(wp_rd_b),
        .data_b(wp_data_b),
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
        .hold(stall_dcache_miss_w),
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
        .flush_bju_pre(flush_bju_pre),
        .exc_irq_ret_ok(exc_irq_ret_ok_w),
        .bju_tgt_in(jp_target),
        .bju_older_in(bju_older_w),
        .exc_ecall_in(exc_ecall),
        .exc_ebreak_in(exc_ebreak),
        .exc_illegal_in(exc_illegal_out),
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
        .exc_cause(exc_cause),
        .exc_pc(exc_pc),
        .exc_tval(exc_tval),
        .rob_trap(flush_rob_trap_w),
        .rob_trap_cause(rob_trap_cause_w),
        .rob_trap_pc(rob_trap_pc_w),
        .rob_trap_tval(rob_trap_tval_w),
        .flush_con_rob(flush_con_rob_w),
        .flush_idx(flush_idx_w),
//十条停顿源：一位一源，controller 只拼装、不做或运算
        .stall_lsu_haz(stall_lsu_haz_w),
        .stall_lsu_unload(stall_lsu_unload_w),
        .stall_lsu_full(stall_lsu_full_w),
        .stall_mulu_haz(stall_mulu_haz_w),
        .stall_mulu_div(stall_mulu_div_w),
        .stall_dcache_miss(stall_dcache_miss_w),
        .stall_icache_miss(icache_busy_q),
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
//中断的 mepc 取 mid_decoder 的 aux_addr_2（那条指令"地址+4"）：它是【正要进发射级、
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
