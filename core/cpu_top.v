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
    wire [31:0] addr_pc, aux_addr_pc;
    (* max_fanout = 32 *) wire [31:0] inst_icache;
    wire busy_icache;
    wire inst_valid_icache;
    wire br1_bpu, br2_bpu, br3_bpu;
    wire jal_iffu, br_en_iffu, jalr_iffu;
//iffu：出队格 h0 —— 本级唯一输出寄存器，给 idu1 的译码与 rob 的分配口/入队扫描口
    wire [31:0] h0_inst_iffu, h0_addr_iffu;
    wire        h0_br_pred_iffu, full_iffu;
    wire [31:0] h0_jalr_pred_iffu;
    wire [4:0]  h0_r1_iffu, h0_r2_iffu;
//idu1：给 fifo 的推进门、给 rob 的分配口
    wire        advance_idu1, alloc_en_idu1, alloc_we_idu1, alloc_jmp_idu1, alloc_pend_idu1;
    wire        judged_bju, gen_bju;
    wire [4:0]  alloc_rd_idu1;
    wire        rdy1_rob, rdy2_rob;
//载荷那一级的前送（rob 出 → idu2 入站取值用）：done 与配对的值
    wire        pay_done1_rob, pay_done2_rob;
    wire [31:0] pay_data1_rob, pay_data2_rob;
    wire        deliv_v_icache, take_en_iffu;
    wire [2:0]  pay_idx_idu1;
    wire        pay_gen_idu1;
//idu1 那一级的输出（= 原 mid_decoder 的位置）：译码字段由本级解出并寄存，
//载荷的操作数由 pd 组合装配后直送 forw
    wire [31:0] inst_idu1;
    wire [4:0] rs1_idu1, rs2_idu1;
    wire [31:0] aux_addr_idu1;
    wire br_pred_taken_idu1;
    wire [31:0] jalr_pred_addr_idu1;
    wire [4:0]  rd_idu1;
    wire [9:0]  func10_idu1;
    wire [31:0] imm_alu_idu1;
//非寄存器操作数的载荷（装配已并进 forw）
//三个单元各自算好的前送命中位（r1/r2 各一位）：forw 据此直接选源，本级不再比 idx
    wire [4:0] rd_idu2;
    wire [31:0] aux_addr_idu2, beq_off_idu2, jalr_pred_addr_idu2;
    wire we_idu2, br_flag_idu2, br_pred_taken_idu2;
    wire [4:0] rd_alu;
    wire [2:0] idx_alu;
    wire [31:0] result_alu;
    wire we_alu;
//写口的事件流别名（给 tb 抓"写进寄存器堆的那一笔"）：直接跟写口级的两条口走
//写序号（乱序写回）：post_decoder 出"本拍这条会写 rd"，cont 内发号（tag_w）
//★ 号源分两处，看消费者采的是哪一级：
//  lsu / mdu 采的是 E4【组合输入】(rd_1/inst_1) ⇒ 用实时计数器 tag_w —— 计数器现在
//  只在"载荷推进且有写"那一拍加一次，这条指令等在输入口期间它不动，所以 tag_w 就是发给它自己的号；
//  wbu 的 alu 口采的是 E4【寄存载荷】(rd_alu/we_alu) ⇒ 必须用与载荷同步锁存出来的 issue_tag_w
//  （从前它接 tag_w = 下一跳的号 ⇒ 老指令看起来比年轻指令还新，杀老写杀反）。
//ROB（重排序缓冲）：分配口/完成口/退口。阶段 B-1 先只接线、退口暂不驱动寄存器堆（行为零变化）。
    wire [2:0]  alloc_idx_rob;
//前送槽扫描的打拍使能：idu1 的输出推进条件（扫描提前读拍一拍）
    wire [2:0]  issue_idx_idu2;
//前送槽号：rob 按槽序扫出"本条指令的操作数该由哪个槽供值"，作为载荷经 post_decoder 寄到载荷拍
    wire [2:0]  fwd_slot1_rob, fwd_slot2_rob;
    wire        fwd_hit1_rob,  fwd_hit2_rob;
    wire payload_go_idu2;
    wire [2:0]  head_rob;
//世代位：rob 分配时给出 → post_decoder 锁成载荷 → 执行单元随值带走 → 完成口带回 rob 校验
    wire        alloc_gen_rob, issue_gen_idu2;
    wire        gen_alu, gen_mdu, gen_lsu;
//保留站（idu2）用：生产者世代（唤醒校验）· rob 的冲刷窗口判据 · 出站操作数 · 站满
    wire        fwd_pg1_rob, fwd_pg2_rob;
    wire        flush_part_rob;
    wire [31:0] r1_final_out_idu2, r2_final_out_idu2;
    wire        stall_rs_full_idu2;
    wire        rs_alloc_en_idu2;
    wire [2:0]  pay_idx_out_idu2;
//rob 前送值读口两组：① 正常读（索引 = rob 自己的扫描槽）② 停顿重读（索引 = regfile 的锁存槽）
//  ★ 分成两组是为了把 stall 那条组合量从【正常读】的数据路上拿掉（见 regfile/rob 的注释）
    wire [31:0] fwd_data1_rob, fwd_data2_rob;
    wire        fwd_done1_rob, fwd_done2_rob;
    wire [2:0]  st_slot1_regfile, st_slot2_regfile;
    wire [31:0] st_data1_rob, st_data2_rob;
    wire        st_done1_rob, st_done2_rob;
//mdu 的结果口被 ROB 按索引直接采走 ⇒ 结果口恒"当场被取走"，不再需要写口级的放行握手
    wire        empty_rob;
    wire [3:0]  trap_cause_rob;
    wire [31:0] trap_pc_rob, trap_tval_rob;
    wire        exc_mark_cont, exc_bju_bju;
    wire [2:0]  idx_q_bju, idx_bju;
    wire [6:0]  opc_idu2;
    wire [9:0]  fn10_idu2, fn10_ls_idu2;
    wire [31:0] off_mem_idu2;
//载荷的两个源寄存器号：当拍同时喂 `forw`（点② 的 `r1_post`/`r2_post`）、`lsu`、`mdu`、`bju`
//各自的匹配比较，是个纯广播网。
    wire [4:0]  rs1_idu2, rs2_idu2;
    wire [31:0] exc_pc_bju;
    wire        older_bju;
    wire [3:0]  occ_rob;
    wire        rwe0_rob, rwe1_rob;
    wire [4:0]  rrd0_rob, rrd1_rob;
    wire [31:0] rdt0_rob, rdt1_rob;
//写序号（cont 内生成）：跟着每条写一路走；以及落地广播、滞留兜底
    wire [2:0]  idx_lsu, idx_mdu;
//非流水线层次块（核内控制信号和其他信号）：按信号首生产者所在模块的代码位置排序
    wire [7:0] flag_bus_cont;
    wire [3:0] func4_idu2;
    wire [11:0] csr_addr_idu2;
    wire [2:0]  head_idx_idu2;
    wire [31:0] head_pc_idu2;
    wire        head_v_idu2;
    wire [31:0] jalr_predict_offset_bpu;
    wire [31:0] isr_addr_cont, iret_addr_cont;
    wire [31:0] offset_jal_iffu, offset_beq_iffu;
    wire [31:0] jalr_target_bju, jp_target_bju;
    wire [5:0] br_pc_idx_bju;
    wire [31:0] r1_data_regfile, r2_data_regfile;
    wire [31:0] data_final_lsu;
    wire br_fail_bju, success_bju, jalr_fail_bju, jal_flag_idu2, jalr_flag_idu2, jalr_flag_q_bju, br_pred_taken_q_bju, exc_irq_ret_idu2, exc_ecall_idu2, exc_ebreak_idu2;
    wire exc_irq_ret_ok_cont;
    wire flush_bju_exc_bju, exc_jal_misalign_out_idu2, exc_illegal_out_idu2, exc_ldst_misalign_out_lsu, exc_ldst_st_out_lsu;
//bju 判定输入级出来的异常（与 flush_bju_pre_bju 同拍、描述同一条）：喂 cont 的异常仲裁
    wire exc_ecall_bju, exc_ebreak_bju, exc_illegal_bju, exc_irq_ret_bju;
    wire [31:0] exc_pc_i_bju;
//cont 输出的 ROB 标记索引（三路 mux 已搬进 cont）
    wire [2:0]  exc_idx_cont;
//非对齐故障【自己】的 ROB 索引（跟故障脉冲同拍寄出来）
    wire [2:0] exc_ldst_idx_out_lsu;
    wire [31:0] exc_ldst_addr_out_lsu;
    wire [31:0] jal_target_idu2;
    wire flush_con_exc_cont;
    wire [3:0] exc_cause_cont;
    wire [31:0] exc_pc_cont, exc_tval_cont;
//前置冲刷（早一拍）：由 bju 的组合判定给出，直连 lsu/mdu（不进 flag_bus_cont，见 cont.v）
    wire flush_bju_pre_bju;
//停顿源（逐条，一位一源）：lsu 三条 / mdu 两条 / dcache / icache / bus / rob / pc
    wire stall_lsu_haz_lsu, stall_lsu_unload_lsu, stall_lsu_full_lsu;
    wire car_free_lsu;
    wire loaded_lsu, we_lsu, inflight_lsu;
    wire stall_rob_full_rob, stall_pc_redir_pc;
    wire flush_con_rob_cont, flush_incl_cont, flush_pc_redir_pc;
    wire [2:0] flush_idx_cont;
    wire flush_rob_trap_rob;
    wire [4:0] rd_load_lsu;
    wire btb_hit_bpu;
//服务 br1_bpu/jal_iffu 的独立 btb 出口：命中标志给 pc，目标指令与三态选择给 idu1
    wire        bti_hit_bpu;
    wire [1:0]  bti_sel_q_bpu;
    wire [31:0] bti_inst_q_bpu;
    wire [31:0] mem_addr_icache, mem_wdata_icache, mem_data_itcm;
    wire [3:0] mem_be_icache;
    wire mem_req_icache, mem_we_icache, mem_valid_itcm;
    wire csr_wr_en_idu2, csr_wr_act_idu2, timi_tim_in, exc_irq_act_cont, exc_irq_processing_cont, exc_irq_cont;
    wire [2:0] csr_func3_idu2;
    wire [31:0] csr_data_comb_cont, mcause_cont, csr_result_alu;
    wire [31:0] data_out_tim_in;
    wire ready_tim_in;
    wire [31:0] data_dcache, mem_addr_dcache, mem_wdata_dcache, mem_data_dtcm;
    wire [3:0] mem_be_dcache;
    wire busy_dcache, ld_ready_dcache, mem_req_dcache, mem_we_dcache, mem_valid_dtcm;
//dcache 的停顿（busy 的延拓拍，原来在顶层合成 d_hold_int，现在由 dcache 自己输出）。
//★ 它【只给 lsu】：不进 flag_bus_cont 的全核广播 —— "dcache 忙"与"前端该不该冻"是两件事，
//  交给 lsu 之后，dcache 回填期间前端照常推进。
    wire hold_dcache;
//RV32M 乘除法单元（与 lsu 流水线同步，独立第三写口）
    wire [31:0] data_final_mdu;
    wire loaded_mdu, we_mdu;
//mdu 两条 ready（回站的门控）与两条接收口的 valid
    wire mul_ready_mdu, div_ready_mdu, mul_go_idu2, div_go_idu2;
    wire [4:0] rd_mul_mdu;
//总线层次块
    wire [31:0] bus_addr_out_lsu, bus_data_out_lsu;
    wire [3:0] bus_be_out_lsu;
    wire bus_we_out_lsu;
    wire bus_valid_out_lsu;
    always @(*) begin
        bus_addr_out = bus_addr_out_lsu;
        bus_data_out = bus_data_out_lsu;
        bus_be_out = bus_be_out_lsu;
        bus_we_out = bus_we_out_lsu;
        bus_valid_out = bus_valid_out_lsu;
    end
    pc u_pc (
        .clk(clk),
        .rst(rst),
        .br1(br1_bpu),
        .bti_hit(bti_hit_bpu),
        .jal(jal_iffu),
        .pre_jalr(jalr_iffu),
        .btb_hit(btb_hit_bpu),
//★ 重定向输入走"门控后"的版本：判定那拍先只会冲刷，真正跳要等 ROB 排空（见上面排队逻辑）。
//  trap 那一路与 irq 共用落点，已合进 pc_irq_g；exc_w 由 flag_bus_cont[7]（flush_con_exc_cont，精确版）承担
//  冲刷，不再单独进 pc；ROB 满改吃 flag_bus_cont[3]（stall_rob_full），端口已删。
        .exc_irq(exc_irq_cont),
//★ 这两根都换成【交付资格】同源的版本（理由见 pc.v 排队那一块）：武装不能再用裸译码/裸载荷。
        .exc_irq_ret(exc_irq_ret_ok_cont),
        .exc_mark(exc_mark_cont),
        .rob_empty(empty_rob),
        .fifo_full(full_iffu),
        .stall_pc_redir(stall_pc_redir_pc),
        .flush_pc_redir(flush_pc_redir_pc),
        .flag_bus(flag_bus_cont),
        .jp_target(jp_target_bju),
        .offset_jal2(offset_jal_iffu),
        .offset_jalr2(jalr_predict_offset_bpu),
        .offset_beq2(offset_beq_iffu),
        .isr_addr2(isr_addr_cont),
        .isr_ret_addr2(iret_addr_cont),
        .pc_addr(addr_pc),
        .aux_addr(aux_addr_pc)
    );
    icache u_icache (
        .clk(clk),
        .rst(rst),
        .pc_addr(addr_pc),
        .flush_pc_redir(flush_pc_redir_pc),
        .fifo_full(full_iffu),
        .br2(br2_bpu),
        .br3(br3_bpu),
        .pre_jalr(jalr_iffu),
        .btb_hit(btb_hit_bpu),
        .jal_in(jal_iffu), .br1_in(br1_bpu),
        .jalr_fail(jalr_fail_bju),
        .exc_irq(exc_irq_cont),
        .exc_irq_ret(exc_irq_ret_idu2),
        .exc_ecall(exc_ecall_idu2),
        .flag_bus(flag_bus_cont),
        .inst_out(inst_icache),
        .inst_valid(inst_valid_icache),
        .busy(busy_icache),
        .deliv_v(deliv_v_icache),
        .take_en(take_en_iffu),
        .mem_req(mem_req_icache),
        .mem_we(mem_we_icache),
        .mem_addr(mem_addr_icache),
        .mem_wdata(mem_wdata_icache),
        .mem_be(mem_be_icache),
        .mem_valid(mem_valid_itcm),
        .mem_data(mem_data_itcm)
    );
    itcm u_itcm (
        .clk(clk),
        .rst(rst),
        .mem_req(mem_req_icache),
        .mem_addr(mem_addr_icache),
        .mem_data(mem_data_itcm),
        .mem_valid(mem_valid_itcm)
    );
//取指队列（icache 与 idu1 之间）：条目只存携带量（指令字/地址/预测位/两个源号），
//队头组合读 + 本拍写入旁路；取指侧的重定向译码在本模块前端，对刚交付的 inst_eff 组合解。
//rob 入队点 = "队头进出队格"那一拍（alloc_en 由新 idu1 给），号与项同沿。
    iffu u_iffu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
        .flush_pc_redir(flush_pc_redir_pc),
        .inst_in(inst_icache),
        .inst_valid(inst_valid_icache),
        .bti_sel(bti_sel_q_bpu),
        .bti_inst(bti_inst_q_bpu),
        .push_addr(aux_addr_pc),
        .push_br_pred(br1_bpu),
        .push_jalr_pred(jalr_predict_offset_bpu),
        .advance(advance_idu1),
        .h0_inst(h0_inst_iffu),
        .h0_addr(h0_addr_iffu),
        .h0_br_pred(h0_br_pred_iffu),
        .h0_jalr_pred(h0_jalr_pred_iffu),
        .h0_r1(h0_r1_iffu),
        .h0_r2(h0_r2_iffu),
        .fch_br_en(br_en_iffu),
        .fch_jal(jal_iffu),
        .fch_jalr(jalr_iffu),
        .fch_off_beq(offset_beq_iffu),
        .fch_off_jal(offset_jal_iffu),
        .full(full_iffu),
        .take_en(take_en_iffu),
        .deliv_v(deliv_v_icache)
    );
//idu1：把出队格 h0 整条寄存一拍，rd/func10/imm 由本级【组合】解出（rob 的分配口与扫槽口
//都吃同一份译码 —— 分配发生在"出队格换新"那一拍，见 idu1 的分配口）。
    idu1 u_idu1 (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
        .inst_in(h0_inst_iffu),
        .addr_in(h0_addr_iffu),
        .br_pred_in(h0_br_pred_iffu),
        .jalr_pred_in(h0_jalr_pred_iffu),
        .stall_rs_full(stall_rs_full_idu2),
        .r1_in(h0_r1_iffu),
        .r2_in(h0_r2_iffu),
        .idx_in(alloc_idx_rob),
        .gen_in(alloc_gen_rob),
        .rdy1_in(rdy1_rob), .rdy2_in(rdy2_rob),
        .advance(advance_idu1),
        .inst_out(inst_idu1),
        .rd_out(rd_idu1),
        .func10_out(func10_idu1),
        .imm_alu_out(imm_alu_idu1),
        .aux_addr_out(aux_addr_idu1),
        .br_pred_taken_out(br_pred_taken_idu1),
        .jalr_pred_addr_out(jalr_pred_addr_idu1),
        .r1_out(rs1_idu1),
        .r2_out(rs2_idu1),
        .idx_out(pay_idx_idu1),
        .gen_out(pay_gen_idu1),
        .alloc_en(alloc_en_idu1),
        .alloc_we(alloc_we_idu1),
        .alloc_rd(alloc_rd_idu1),
        .alloc_jmp(alloc_jmp_idu1),
        .alloc_pend(alloc_pend_idu1)
    );
//发射单元（issue unit）：组合译码 + 四类统一保留站（深度 4）。由 post_decoder 改造而来。
//★ 出站同时驱动 alu / bju（bju 是每条指令都要过的判定+异常通道），lsu/mdu 由各自的
//  mem_op / is_mul 判据过滤 —— 与原来"载荷同时喂四个单元"同形。
    idu2 u_idu2 (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
        .s_advance(advance_idu1),
        .func10(func10_idu1),
        .rd_in(rd_idu1),
        .rs1_in(rs1_idu1),
        .rs2_in(rs2_idu1),
        .imm_alu_in(imm_alu_idu1),
        .inst_in(inst_idu1),
        .offset_beq0_aux(imm_alu_idu1),
        .pc_operand_in(aux_addr_idu1),
        .aux_addr_in(aux_addr_idu1),
        .br_pred_taken_in(br_pred_taken_idu1),
        .jalr_pred_addr_in(jalr_pred_addr_idu1),
        .idx_in(pay_idx_idu1),
        .alloc_gen_in(pay_gen_idu1),
//操作数底值 = regfile 的寄存读（与入站同拍）；非寄存器操作数在站内入站当拍覆盖
        .r1_data(r1_data_regfile), .r2_data(r2_data_regfile),
//rob 的扫描口：槽号/命中/【生产者世代】（站项入站时把世代一起冻下来做唤醒校验）
        .fwd_slot1_in(fwd_slot1_rob), .fwd_slot2_in(fwd_slot2_rob),
        .fwd_hit1_in(fwd_hit1_rob),   .fwd_hit2_in(fwd_hit2_rob),
        .fwd_pg1_in(fwd_pg1_rob),     .fwd_pg2_in(fwd_pg2_rob),
        .rdy1_in(rdy1_rob), .rdy2_in(rdy2_rob),
        .pay_done1_in(pay_done1_rob), .pay_done2_in(pay_done2_rob),
        .pay_data1_in(pay_data1_rob), .pay_data2_in(pay_data2_rob),
//三个结果口：入站取值 + 站内唤醒 + 出站旁路（当拍被唤醒的项当拍就能出站）
        .we_alu(we_alu),   .idx_alu(idx_alu), .gen_alu(gen_alu), .data_alu(result_alu),
        .we_mul(we_mdu), .idx_mul(idx_mdu), .gen_mul(gen_mdu), .data_mul(data_final_mdu),
        .we_ld(loaded_lsu),  .idx_ld(idx_lsu),   .gen_ld(gen_lsu),   .data_ld(data_final_lsu),
        .flush_con_rob(flush_con_rob_cont),
        .rob_flush_part(flush_part_rob),
        .flush_incl(flush_incl_cont),
        .flush_idx(flush_idx_cont),
        .trap_fire(flush_rob_trap_rob),
        .head_ptr(head_rob),
        .flush_bju_pre(flush_bju_pre_bju),
        .lsu_full_in(~car_free_lsu),
        .mul_ready_in(mul_ready_mdu), .div_ready_in(div_ready_mdu),
        .alloc_rob_in(alloc_en_idu1),
        .alloc_en(rs_alloc_en_idu2),
        .pay_idx_out(pay_idx_out_idu2),
//出站脉冲 = 原来的 payload_go：lsu/mdu 的入口门、bju 的判定输入门都用它
        .issue_v(payload_go_idu2),
        .head_idx_o(head_idx_idu2),
        .head_pc_o(head_pc_idu2),
        .head_v_o(head_v_idu2),
        .mul_go(mul_go_idu2), .div_go(div_go_idu2),
        .r1_final_out(r1_final_out_idu2), .r2_final_out(r2_final_out_idu2),
        .rd_out(rd_idu2),
        .issue_idx(issue_idx_idu2),
        .issue_gen(issue_gen_idu2),
        .opc_out(opc_idu2),
        .fn10_out(fn10_idu2),
        .fn10_ls_out(fn10_ls_idu2),
        .off_mem_out(off_mem_idu2),
        .rs1_out(rs1_idu2), .rs2_out(rs2_idu2),
        .alu_func4(func4_idu2),
        .csr_func3(csr_func3_idu2),
        .we(we_idu2),
        .csr_wr_en(csr_wr_en_idu2),
        .csr_wr_act(csr_wr_act_idu2),
        .csr_addr(csr_addr_idu2),
        .aux_addr_out(aux_addr_idu2),
        .beq_off_q2(beq_off_idu2),
        .jalr_pred_addr_out(jalr_pred_addr_idu2),
        .jal_target_out(jal_target_idu2),
        .br_flag(br_flag_idu2),
        .jal_flag(jal_flag_idu2),
        .jalr_flag(jalr_flag_idu2),
        .br_pred_taken_out(br_pred_taken_idu2),
        .exc_irq_ret(exc_irq_ret_idu2),
        .exc_ecall(exc_ecall_idu2),
        .exc_ebreak(exc_ebreak_idu2),
        .exc_jal_misalign_out(exc_jal_misalign_out_idu2),
        .exc_illegal_out(exc_illegal_out_idu2),
        .rs_full(stall_rs_full_idu2),
        .occupancy()
    );
//分支/跳转判定单元：判定源与 alu 的输入同源（前送后的操作数 / func4_idu2），
//比较与加法在本拍做，结果、落点在下一拍生效；flush_bju_pre_bju 是同一判定的组合版本、早一拍
    bju u_bju (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
        .adv_in(payload_go_idu2),
        .r1_data_in(r1_final_out_idu2),
        .idx_in(issue_idx_idu2),
        .r2_data_in(r2_final_out_idu2),
        .alu_func4_in(func4_idu2),
        .br_flag_in(br_flag_idu2),
        .jalr_flag_in(jalr_flag_idu2),
        .aux_addr_in(aux_addr_idu2),
        .beq_off_in(beq_off_idu2),
        .jalr_pred_addr_in(jalr_pred_addr_idu2),
        .br_pred_taken_in(br_pred_taken_idu2),
        .success(success_bju),
        .br_fail(br_fail_bju),
        .jalr_fail(jalr_fail_bju),
        .jp_target(jp_target_bju),
        .jalr_target_q2(jalr_target_bju),
        .flush_bju_exc(flush_bju_exc_bju),
        .exc_bju(exc_bju_bju),
        .idx_i(idx_bju),
        .exc_pc_q(exc_pc_bju),
        .older_q(older_bju),
        .idx_q(idx_q_bju),
        .bju_judged(judged_bju),
        .bju_gen(gen_bju),
        .gen_in(issue_gen_idu2),
        .br_pc_idx(br_pc_idx_bju),
        .jalr_flag_q(jalr_flag_q_bju),
        .br_pred_taken_q(br_pred_taken_q_bju),
        .exc_jal_misalign_in(exc_jal_misalign_out_idu2),
        .jal_target_in(jal_target_idu2),
//异常源进判定输入寄存级（与操作数/控制位同沿）
        .exc_ecall_in(exc_ecall_idu2),
        .exc_ebreak_in(exc_ebreak_idu2),
        .exc_illegal_in(exc_illegal_out_idu2),
        .exc_irq_ret_in(exc_irq_ret_idu2),
        .exc_ecall_i(exc_ecall_bju),
        .exc_ebreak_i(exc_ebreak_bju),
        .exc_illegal_i(exc_illegal_bju),
        .exc_irq_ret_i(exc_irq_ret_bju),
        .exc_pc_i(exc_pc_i_bju),
        .flush_bju_pre(flush_bju_pre_bju)
    );
    lsu u_lsu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
        .flush_bju_pre(flush_bju_pre_bju),
        .payload_go(payload_go_idu2),
//★ 三个执行单元的输入级数对齐：lsu 也吃 decoder 载荷（原来吃 mem_buf 组合输出，早一级）
        .opcode(opc_idu2),
        .func10(fn10_ls_idu2),
        .rd_in(rd_idu2),
        .r1_post(rs1_idu2),
        .r2_post(rs2_idu2),
        .r1_data_final(r1_final_out_idu2),
        .r2_data_final(r2_final_out_idu2),
        .offset_load0(off_mem_idu2),
        .offset_store0(off_mem_idu2),
        .bus_data_ext(bus_data_in_ext),
        .bus_data_dcache(data_dcache),
        .bus_data_tim(data_out_tim_in),
        .ready_dcache(ld_ready_dcache),
        .dcache_hold(hold_dcache),
        .mem_inflight(inflight_lsu),
        .ready_tim(ready_tim_in),
        .ready_ext(bus_loaded_in),
        .bus_addr_out(bus_addr_out_lsu),
        .bus_data_out(bus_data_out_lsu),
        .bus_be_out(bus_be_out_lsu),
        .bus_we_out(bus_we_out_lsu),
        .bus_valid_out(bus_valid_out_lsu),
        .exc_ldst_misalign_out(exc_ldst_misalign_out_lsu),
        .exc_ldst_st_out(exc_ldst_st_out_lsu),
        .exc_ldst_idx_out(exc_ldst_idx_out_lsu),
//索引与操作数同源：既然吃的是 decoder 载荷，索引就是它自己的 issue_idx（三者统一）
        .idx_in(issue_idx_idu2),
        .gen_in(issue_gen_idu2),
        .ld_idx(idx_lsu),
        .ld_gen(gen_lsu),
        .exc_ldst_addr_out(exc_ldst_addr_out_lsu),
        .ld_data_out(data_final_lsu),
        .loaded(loaded_lsu),
        .ld_we(we_lsu),
        .stall_lsu_haz(stall_lsu_haz_lsu),
        .stall_lsu_unload(stall_lsu_unload_lsu),
        .car_free(car_free_lsu),
        .stall_lsu_full(stall_lsu_full_lsu),
        .rd_load(rd_load_lsu)
    );
//RV32M：与 lsu 同拍取 mem_buf 输出，自己从 opcode/func10 判 M（不用额外派发标志）
    mdu u_mdu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
        .flush_idx(flush_idx_cont),
        .head_ptr(head_rob),
        .flush_part(flush_part_rob),
//两条接收口（乘法 / 除法）：站一拍只出一条，所以两套载荷同源、只有 valid 不同。
//操作数用站里算好的终值；r1_post/r2_post 那对只服务已废的 mul-use 冒险，随之外退场。
        .mul_go(mul_go_idu2),
        .mul_func10(fn10_idu2),
        .mul_rd(rd_idu2),
        .mul_idx_in(issue_idx_idu2),
        .mul_gen_in(issue_gen_idu2),
        .mul_r1(r1_final_out_idu2),
        .mul_r2(r2_final_out_idu2),
        .div_go(div_go_idu2),
        .div_func10(fn10_idu2),
        .div_rd(rd_idu2),
        .div_idx_in(issue_idx_idu2),
        .div_gen_in(issue_gen_idu2),
        .div_r1(r1_final_out_idu2),
        .div_r2(r2_final_out_idu2),
//结果口（乘除共用一条；站式之后两者可同时在途 ⇒ 仲裁在 mdu 内做"除法优先"）
        .mul_data_out(data_final_mdu),
        .mul_loaded(loaded_mdu),
        .mul_we(we_mdu),
        .rd_mul(rd_mul_mdu),
        .mul_idx(idx_mdu),
        .mul_gen(gen_mdu),
//两条 ready 回站：站侧按站项算子分别门控
        .mul_ready(mul_ready_mdu),
        .div_ready(div_ready_mdu)
    );
    bpu u_bpu (
        .clk(clk),
        .rst(rst),
        .pc_addr_in(addr_pc),
        .jalr_target_q(jalr_target_bju),
        .br_pc_idx(br_pc_idx_bju),
        .success(success_bju),
        .br_fail(br_fail_bju),
        .br_en(br_en_iffu),
        .jalr_flag(jalr_flag_q_bju),
        .jal(jal_iffu),
        .inst_in(inst_icache),
        .inst_valid(inst_valid_icache),
        .stall_rs_full(stall_rs_full_idu2),
        .flag_bus(flag_bus_cont),
        .fifo_full(full_iffu),
        .take_en(take_en_iffu),
        .br_pred_taken_in(br_pred_taken_q_bju),
        .bti_hit(bti_hit_bpu),
        .bti_inst_q(bti_inst_q_bpu),
        .bti_sel_q(bti_sel_q_bpu),
        .br1(br1_bpu),
        .br2(br2_bpu),
        .br3(br3_bpu),
        .jalr_predict_offset(jalr_predict_offset_bpu),
        .jalr(btb_hit_bpu)
    );
//写口级：三个结果口 → 两条物理写口（完成即写、同 rd 同拍年轻者胜）+ 处置回报 + 落地广播
//前送级：**只剩一级数据 mux** —— 裁决（"我这一拍是不是在给消费者供值"）已在三个单元里做完
//（alu 在结果寄存沿、mdu/lsu 与各自结果口同拍），这里只按算好的命中位选源。
//  默认源 = regfile 的寄存读（含 5 源读侧旁路）与非寄存器操作数的常量载荷，两者也在这一次选择里落定。
//★ forw 已并入 idu2（plan）："结果口→值"的选择逻辑搬到了入站取值那一段
//  （源就绪 ⇒ 当拍锁值；未就绪 ⇒ 锁生产者 slot+世代，等广播唤醒）。
//  站→单元之间不再需要这一级 —— 站项出站时操作数已经是终值。
//  这两根线保留原名，四个单元的输入口接线一行不用动。
    alu u_alu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
        .we_in(we_idu2),
        .jal_flag(jal_flag_idu2),
        .jalr_flag(jalr_flag_idu2),
        .cs_wr_en(csr_wr_en_idu2),
        .rd_in(rd_idu2),
        .idx_in(issue_idx_idu2),
        .gen_in(issue_gen_idu2),
        .alu_func4(func4_idu2),
        .aux_addr_in(aux_addr_idu2),
        .csr_func3(csr_func3_idu2),
        .r1_data(r1_final_out_idu2),
        .r2_data(r2_final_out_idu2),
        .cs_data(csr_data_comb_cont),
        .result_csr(csr_result_alu),
        .rd_out(rd_alu),
        .idx_out(idx_alu),
        .gen_out(gen_alu),
        .result(result_alu),
        .we(we_alu)
    );
//tb 的事件探针按这两个名字抓"寄存器堆写口" ⇒ 让它们跟着写口级的两条口走（语义不变）

    rob u_rob (
        .clk(clk), .rst(rst),
        .alloc_en(alloc_en_idu1), .alloc_idx(alloc_idx_rob), .full(stall_rob_full_rob),
        .alloc_gen(alloc_gen_rob),
        .alloc_jmp(alloc_jmp_idu1),
        .alloc_pend(alloc_pend_idu1),
//完成口 = 三个单元的结果口直连（值 + 索引 + 世代同拍同源）；ROB 按索引把 wr/data 一起写进那一项
        .bju_judged(judged_bju),
        .bju_idx(idx_q_bju),
        .bju_gen(gen_bju),
        .alu_done(we_alu),   .alu_idx(idx_alu), .alu_data(result_alu),       .alu_gen(gen_alu),
        .mul_done(we_mdu), .mul_idx(idx_mdu), .mul_data(data_final_mdu), .mul_gen(gen_mdu),
        .ld_done(we_lsu),   .ld_idx(idx_lsu),   .ld_data(data_final_lsu),   .ld_gen(gen_lsu),
//发出口：不写 rd 的项靠这一拍置"可退"（站式之后"执行早于退役"不再隐含）
        .issue_en(payload_go_idu2), .issue_idx(issue_idx_idu2), .issue_gen(issue_gen_idu2),
//前送值读口①（正常读，索引 = 本模块扫描出的槽）与②（停顿重读，索引 = regfile 的锁存槽）
        .fwd_data1(fwd_data1_rob), .fwd_data2(fwd_data2_rob),
        .fwd_done1(fwd_done1_rob), .fwd_done2(fwd_done2_rob),
        .st_slot1(st_slot1_regfile),  .st_slot2(st_slot2_regfile),
        .st_data1(st_data1_rob), .st_data2(st_data2_rob),
        .st_done1(st_done1_rob), .st_done2(st_done2_rob),
//提交口：驱动寄存器堆的两条写口（口 A = head、口 B = head+1）
        .commit_we0(rwe0_rob), .commit_rd0(rrd0_rob), .commit_data0(rdt0_rob),
        .commit_we1(rwe1_rob), .commit_rd1(rrd1_rob), .commit_data1(rdt1_rob),
        .alloc_we(alloc_we_idu1),
        .alloc_rd(alloc_rd_idu1),
//前送槽扫描：消费者是下一拍进载荷那条（本拍还在 idu1 的输入级）⇒ 用它的 rs；
//本模块按程序序分配、且入队点前移到"队头进 出队格"那一拍 ⇒ 扫描那一拍在册的槽全是比它老的，
//"比我老"不需要比较。
//槽扫描吃【出队格（iffu 的普通触发器）】：读拍在 idu1 的输出级，所以扫描提前一拍；
//起点不是 icache 的 BRAM 输出寄存器 —— 那 2.45ns 的 clock-to-out 是白吃的。
//入队扫描：判定源是【出队格 h0 的 Q 端】那条（= 本拍要入册的）；读口按 S 级、就绪按载荷那一级现读
        .alloc_rs1(h0_r1_iffu), .alloc_rs2(h0_r2_iffu),
        .s_idx(pay_idx_idu1), .pay_idx(pay_idx_out_idu2), .pay_v(1'b1),
        .fwd_pg1(fwd_pg1_rob), .fwd_pg2(fwd_pg2_rob), .flush_part(flush_part_rob),
        .rdy1(rdy1_rob), .rdy2(rdy2_rob),
        .pay_done1(pay_done1_rob), .pay_done2(pay_done2_rob),
        .pay_data1(pay_data1_rob), .pay_data2(pay_data2_rob),
        .fwd_slot1(fwd_slot1_rob), .fwd_slot2(fwd_slot2_rob),
        .fwd_hit1(fwd_hit1_rob),   .fwd_hit2(fwd_hit2_rob),
//分支/跳转误预测一律【部分冲刷】：边界号 = bju 随判定锁存的那条自己的项。
//  边界项若已失效（分支已退，说明没有更老的活项）⇒ ROB 内部自动退化成全冲。
        .flush_con_rob(flush_con_rob_cont), .flush_idx(flush_idx_cont), .flush_incl(flush_incl_cont),
        .flush_con_exc(flush_con_exc_cont),
//标记落哪一项：三路 mux 已在 cont 内按各自相位选好（见 cont 的 exc_idx）
        .exc_en(exc_mark_cont),
        .exc_idx(exc_idx_cont),
        .exc_cause(exc_cause_cont), .exc_pc(exc_pc_cont), .exc_tval(exc_tval_cont),
        .trap_fire(flush_rob_trap_rob),
        .trap_cause(trap_cause_rob), .trap_pc(trap_pc_rob), .trap_tval(trap_tval_rob),
        .occupancy(occ_rob), .empty(empty_rob),
        .head_ptr(head_rob)
    );
    regfile u_regfile (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus_cont),
//读口地址由 idu1 那一级给出（比载荷早一拍），读值寄存后随载荷往下走
        .stall_rs_full(stall_rs_full_idu2),
        .r1(rs1_idu1),
        .r2(rs2_idu1),
        .we_a(rwe0_rob),
        .rd_a(rrd0_rob),
        .data_a(rdt0_rob),
        .we_b(rwe1_rob),
        .rd_b(rrd1_rob),
        .data_b(rdt1_rob),
//★ 读侧旁路也吃三个单元结果口（共 5 源）：结果离开单元口那一拍还没落进阵列，
//  而消费者的操作数取自"读锁存沿" ⇒ 那一格只有这里补得住（原来由 forw 点① 承担）。
//  选源用 rob 的槽扫描（与 payload 的 sel_slot 同源、取当拍组合版）：扫描口读的就是本模块的
//  读口地址 rs1_idu1/rs2_idu1，同一条指令（扫描早一拍），所以当拍就能用。
        .we_alu(we_alu), .data_alu(result_alu),
        .we_mul(loaded_mdu), .data_mul(data_final_mdu),
        .we_ld(loaded_lsu), .data_ld(data_final_lsu),
        .idx_alu(idx_alu), .idx_mul(idx_mdu), .idx_ld(idx_lsu),
//载荷操作数的就绪（rob 现算）：与 stall_w 一起决定读口是"正常读"还是"按锁存槽重读"
        .rdy1_in(rdy1_rob), .rdy2_in(rdy2_rob),
        .fwd_data1(fwd_data1_rob), .fwd_data2(fwd_data2_rob),
        .fwd_done1(fwd_done1_rob), .fwd_done2(fwd_done2_rob),
        .st_slot1(st_slot1_regfile), .st_slot2(st_slot2_regfile),
        .st_data1(st_data1_rob), .st_data2(st_data2_rob),
        .st_done1(st_done1_rob), .st_done2(st_done2_rob),
        .fwd_slot1(fwd_slot1_rob), .fwd_slot2(fwd_slot2_rob),
        .fwd_hit1(fwd_hit1_rob),   .fwd_hit2(fwd_hit2_rob),
        .r1_data(r1_data_regfile),
        .r2_data(r2_data_regfile)
    );
    dcache u_dcache (
        .clk(clk),
        .rst(rst),
        .bus_addr_in(bus_addr_out_lsu),
        .bus_data_in(bus_data_out_lsu),
        .bus_be_in(bus_be_out_lsu),
        .bus_we_in(bus_we_out_lsu),
        .bus_data_out(data_dcache),
        .ld_ready(ld_ready_dcache),
        .busy(busy_dcache),
        .hold(hold_dcache),
        .mem_req(mem_req_dcache),
        .mem_we(mem_we_dcache),
        .mem_addr(mem_addr_dcache),
        .mem_wdata(mem_wdata_dcache),
        .mem_be(mem_be_dcache),
        .mem_data(mem_data_dtcm),
        .mem_valid(mem_valid_dtcm),
        .mem_ready(1'b1)
    );
    dtcm u_dtcm (
        .clk(clk),
        .rst(rst),
        .mem_req(mem_req_dcache),
        .mem_we(mem_we_dcache),
        .mem_addr(mem_addr_dcache),
        .mem_wdata(mem_wdata_dcache),
        .mem_be(mem_be_dcache),
        .mem_data(mem_data_dtcm),
        .mem_valid(mem_valid_dtcm),
        .mem_ready()
    );
    tim_in u_tim_in (
        .clk(clk),
        .rst(rst),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_out),
        .bus_data_out(data_out_tim_in),
        .timi(timi_tim_in),
        .ready(ready_tim_in)
    );
    cont u_cont (
        .clk(clk),
        .rst(rst),
        .br2(br2_bpu),
        .br3(br3_bpu),
        .jalr_fail(jalr_fail_bju),
        .jal(jal_iffu),
        .pre_jalr(jalr_iffu),
        .btb_hit(btb_hit_bpu),
        .br1(br1_bpu),
        .exc_irq_ret(exc_irq_ret_idu2),
        .exc_ecall(exc_ecall_idu2),
        .flush_bju_exc(flush_bju_exc_bju),
//异常仲裁源（原 trap_unit 的例化并入 cont）
        .bju_pc_in(exc_pc_bju),
        .exc_irq_ret_ok(exc_irq_ret_ok_cont),
        .bju_tgt_in(jp_target_bju),
        .bju_older_in(older_bju),
//★ 不再有 flush_bju_pre_bju：ecall/ebreak/illegal/mret 改吃 bju 的【判定输入级】输出（同拍配对）
        .exc_ecall_i(exc_ecall_bju),
        .exc_ebreak_i(exc_ebreak_bju),
        .exc_illegal_i(exc_illegal_bju),
        .exc_irq_ret_i(exc_irq_ret_bju),
        .exc_pc_i(exc_pc_i_bju),
        .bju_idx_i(idx_bju),
        .exc_ldst_misalign_in(exc_ldst_misalign_out_lsu),
        .exc_ldst_st_in(exc_ldst_st_out_lsu),
        .exc_ldst_addr_in(exc_ldst_addr_out_lsu),
        .exc_pc_in(aux_addr_idu2),
//ROB 冲刷边界：故障项自己的索引 / E4 载荷（中断边界）/ bju 判定那条
        .exc_ldst_idx_in(exc_ldst_idx_out_lsu),
        .issue_idx_in(issue_idx_idu2),
        .irq_head_idx_in(head_idx_idu2),
        .irq_head_v(head_v_idu2),
        .bju_idx_in(idx_q_bju),
        .flush_con_exc(flush_con_exc_cont),
        .exc_mark(exc_mark_cont),
//ROB 标记索引（三路 mux 已在 cont 内）
        .exc_idx(exc_idx_cont),
        .exc_cause(exc_cause_cont),
        .exc_pc(exc_pc_cont),
        .exc_tval(exc_tval_cont),
        .rob_trap(flush_rob_trap_rob),
        .rob_trap_cause(trap_cause_rob),
        .rob_trap_pc(trap_pc_rob),
        .rob_trap_tval(trap_tval_rob),
        .flush_con_rob(flush_con_rob_cont),
        .flush_idx(flush_idx_cont),
        .flush_incl(flush_incl_cont),
//八条停顿源：一位一源，cont 只拼装、不做或运算
//★ 数据相关整族退场：原本这四位（lsu_haz / lsu_full / mulu_haz / mulu_div）进全核广播，
//  现在全部退化成【站内的出站条件】，前端只认识"站满"这一根。
        .stall_icache_miss(busy_icache),
        .stall_bus_hold(bus_hold_in),
        .stall_rob_full(stall_rob_full_rob),
        .stall_pc_redir(stall_pc_redir_pc),
        .lsu_inflight(inflight_lsu),
        .csr_wr_en(csr_wr_en_idu2),
        .csr_wr_act(csr_wr_act_idu2),
        .exti(exti),
        .timi(timi_tim_in),
        .softi(1'b0),
        .csr_addr(csr_addr_idu2),
        .csr_data_in(csr_result_alu),
        .csr_data_comb(csr_data_comb_cont),
//中断的 mepc 取 idu1 的 aux_addr_1（那条指令"地址+4"）：它是【正要进发射级、
//还没被发出去】的那一条 = 中断返回点。取指 PC 在前端等 ROB 排空期间会漂，不能用。
        .pc_addr_in(head_pc_idu2),
        .isr_addr2(isr_addr_cont),
        .mcause(mcause_cont),
        .exc_irq_act(exc_irq_act_cont),
        .exc_irq_processing(exc_irq_processing_cont),
        .exc_irq(exc_irq_cont),
        .iret_addr2(iret_addr_cont),
        .flag_bus(flag_bus_cont)
    );
endmodule
