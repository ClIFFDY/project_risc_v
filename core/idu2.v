`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu2 —— 发射级：两条 lane 各一级 D→Q 直通发射
//
//   由"四类统一保留站"退化而来。站阵列 / 入站两槽 / 入队扫描 / 唤醒比较器阵列 /
//   年龄与前缀树 / 选优（sel_slot）/ 站内搬迁 / free_slot / occupancy **全部删除**：
//   顺序双发射不需要调度器，"谁可以发"就是两条 lane 各自的本地判据。
//
//   ★ 结构：每条 lane 一级 D→Q 寄存器。D = idu1 的 S 级输出，Q = 发射寄存器。
//     Q 端组合算 issue_v 与全部字段，单元当拍收；D→Q 只在 Q 空 / Q 本拍发出时发生。
//
//   ★ 操作数在 Q 端组合选出，四档：立即数 → 结果口(idx+gen 命中) → rob 拉式 fwd 值 → rdata。
//     rdata 这一档是"装 Q 那一拍由上一拍的 D.rs 预取"的值；装进 Q 之后地址改跟 Q 的 rs，
//     于是它每拍被 regfile 刷新 —— 生产者在消费者等待期间提交时，只有这一档兜得住。
//
//   ★ 结果口是寄存值：生产者 T 拍发、口上 T+1 有效；rob 的 ent_wr 要到 T+2 才置起
//     ⇒ 结果口这一档比 qry_ok 早一拍，两档都要。本条自己的结果最早下一拍才回来。
//
//   ★ 前送只认已经发出去的指令的产物。now_* 是可选的"同拍 alu 结果"路径（独立数据路径），
//     不接（恒 0）功能完全正确。
//
//   ★ 冲刷：按 idx 与边界比年龄，年轻清掉；trap / 全冲两条都清。已发出的 lane 不回收
//     （它已经不在 Q 里，完成口还要回 rob）。冲刷拍不给 load，但**照样给 acc**：
//     idu1 的 S 级在冲刷后仍躺着那条错路指令，不给 credit 会把 idu1 永久按死。
//
//   ★ ROB 的 fwd 口是【拉式】：每 lane 每源一份 scoreboard {hit,slot,gen} + wait，
//     slot/gen 交给 rob 换值（qry_ok = 在册 & wr & 世代匹配）。
//     tag 的来源是 32 项生产者表 pstat（每寄存器一个"最年轻的在生产写者"）。
//////////////////////////////////////////////////////////////////////////////////

module idu2(
    input clk, rst,
    input [7:0] flag_bus,
//前端：本 lane 的 D 有指令、且还没被收
    input        s_advance,
    input        s_advance1,

//---- D 级 lane0（idu1 的 S 级寄存器）----
    input [4:0]  rs1_in, rs2_in,
    input [31:0] inst_in, pc_operand_in, aux_addr_in,
    input        br_pred_taken_in,
    input [3:0]  idx_in,
    input        alloc_gen_in,
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
    input [3:0]  dec_bjuf4,
    input        dec_r1_imm,
    input        dec_r2_imm,
    input [31:0] dec_r1_immv,
    input [31:0] dec_r2_immv,
//---- D 级 lane1 ----
    input [4:0]  rs1_in1, rs2_in1,
    input [31:0] inst_in1, pc_operand_in1, aux_addr_in1,
    input        br_pred_taken_in1,
    input [3:0]  idx_in1,
    input        alloc_gen_in1,
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
    input [3:0]  dec_bjuf41,
    input        dec_r1_imm1,
    input        dec_r2_imm1,
    input [31:0] dec_r1_immv1,
    input [31:0] dec_r2_immv1,

//---- rob 拉式值读口：4 组（2 lane × 2 源）----
    output reg [3:0]  qry_slot_a_0, qry_slot_b_0, qry_slot_a_1, qry_slot_b_1,
    output reg        qry_gen_a_0,  qry_gen_b_0,  qry_gen_a_1,  qry_gen_b_1,
    input [31:0] qry_data_a_0, qry_data_b_0, qry_data_a_1, qry_data_b_1,
    input        qry_ok_a_0,   qry_ok_b_0,   qry_ok_a_1,   qry_ok_b_1,

//---- 结果口（寄存值）----
    input        we_alu,  input [3:0] idx_alu,  input gen_alu,  input [31:0] data_alu,
    input        we_alu2, input [3:0] idx_alu2, input gen_alu2, input [31:0] data_alu2,
    input        we_mul,  input [3:0] idx_mul,  input gen_mul,  input [31:0] data_mul,
    input        we_ld,   input [3:0] idx_ld,   input gen_ld,   input [31:0] data_ld,
//---- 可选：alu 的同拍组合结果（不接恒 0 也正确）----
    input        now_v_0, input [3:0] now_idx_0, input now_gen_0, input [31:0] now_data_0,
    input        now_v_1, input [3:0] now_idx_1, input now_gen_1, input [31:0] now_data_1,

//---- 提交口（清 wait / 清生产者表；必须同时比 rd 与 slot+gen）----
    input        we_a, input [4:0] rd_a,
    input        we_b, input [4:0] rd_b,
    input [3:0]  commit_slot0, commit_slot1,
    input        commit_gen0,  commit_gen1,
//提交口的数据：Q 在等的那条生产者若在这一拍提交，base 直接吃这个值（同拍精确旁路）
    input [31:0] commit_data0, commit_data1,

//---- 冲刷 / 陷阱 / 年龄基准 ----
    input        flush_con_rob,
    input        rob_flush_part,
    input        flush_incl,
    input [3:0]  flush_idx,
    input        trap_fire,
    input [3:0]  head_ptr,
//---- 生产者表重建口（rob 送出的"这一沿之后仍活着的项"；物理槽序，槽号即下标）----
    input [79:0] rb_rd,
    input [15:0] rb_gen,
    input [15:0] rb_v,

//---- 单元就绪 ----
    input        lsu_full,
    input        mdu_mul_ok_0, mdu_div_ok_0, mdu_mul_ok_1, mdu_div_ok_1,
    input        bju_ok_0, bju_ok_1,

//---- 寄存器堆读口（寄存读：地址本拍给、数据下一拍到）----
//---- 寄存器堆读值 ----
//★ 地址不在这里出：读口地址由 iffu 的出队格（F 级）直接给 regfile，寄存一拍，
//  正好与"这一条进 D 级"同拍到达 ⇒ rdata 与 D 里的指令对齐。本模块只吃不驱动。
    input [31:0] rdata1_in, rdata2_in, rdata3_in, rdata4_in,

//---- 出站 lane0 ----
    output reg        issue_v_0,
    output reg [4:0]  rd_0,
    output reg [3:0]  issue_idx_0,
    output reg        issue_gen_0,
    output reg        we_0,
    output reg [1:0]  sel_class_0,
    output reg [6:0]  opc_0,
    output reg [9:0]  fn10_0, fn10_ls_0,
    output reg [3:0]  alu_func4_0,
    output reg [2:0]  csr_func3_0,
    output reg        csr_wr_en_0, csr_wr_act_0,
    output reg [11:0] csr_addr_0,
    output reg [31:0] aux_addr_0, off_mem_0, beq_off_0, jalr_pred_addr_0, jal_target_0,
    output reg [31:0] op_data_a_0, op_data_b_0,
    output reg [4:0]  rs_a_0, rs_b_0,
    output reg        br_flag_0, jal_flag_0, jalr_flag_0, br_pred_taken_0,
    output reg [3:0]  bju_func4_0,
    output reg        mul_go_0, div_go_0,
//---- 出站 lane1 ----
    output reg        issue_v_1,
    output reg [4:0]  rd_1,
    output reg [3:0]  issue_idx_1,
    output reg        issue_gen_1,
    output reg        we_1,
    output reg [1:0]  sel_class_1,
    output reg [6:0]  opc_1,
    output reg [9:0]  fn10_1, fn10_ls_1,
    output reg [3:0]  alu_func4_1,
    output reg [2:0]  csr_func3_1,
    output reg        csr_wr_en_1, csr_wr_act_1,
    output reg [11:0] csr_addr_1,
    output reg [31:0] aux_addr_1, off_mem_1, beq_off_1, jalr_pred_addr_1, jal_target_1,
    output reg [31:0] op_data_a_1, op_data_b_1,
    output reg [4:0]  rs_a_1, rs_b_1,
    output reg        br_flag_1, jal_flag_1, jalr_flag_1, br_pred_taken_1,
    output reg [3:0]  bju_func4_1,
    output reg        mul_go_1, div_go_1,

//---- 共享 LSU 口（被选中 lane 的字段 + lsu_lane）----
    output reg        lsu_v,
    output reg        lsu_lane,
    output reg [6:0]  lsu_opc,
    output reg [9:0]  lsu_fn10,
    output reg [4:0]  lsu_rd,
    output reg [3:0]  lsu_idx,
    output reg        lsu_gen,
    output reg [31:0] lsu_a, lsu_b, lsu_off,
    output reg [4:0]  lsu_rs_a, lsu_rs_b,

//---- 中断边界 / 状态 / credit ----
    output reg [3:0]  head_idx_o,
    output reg [31:0] head_pc_o,
    output reg        head_v_o,
    output reg        bju_in_flight,
    output reg        acc_0, acc_1,
    output reg        issue_pop_0, issue_pop_1
    );

//==============================================================================
// 零、控制位 / 复位 / 常量
//==============================================================================
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec, flush_w;
    always @(*) begin
        flush_con_exc   = flag_bus[7];
        flush_con_irq   = flag_bus[6];
        flush_con_jump  = flag_bus[5];
        exec            = flag_bus[4];
        flush_w         = flush_con_exc | flush_con_irq | flush_con_jump;
    end

//D 侧推进门：与 idu1 的 `go = exec & ~flush_w & ~stall` 逐字同门。
//★ 少了它，idu1 被停住那一拍我们照样给 acc ⇒ idu1 把 l0_con 置起、lane_v 落下，
//  而 S 级寄存器并没有换新 ⇒ 那条指令凭空消失（静默）。
    reg pipe_go;
    always @(*) pipe_go = exec & ~((flag_bus[3] | flag_bus[2] | flag_bus[0]) & ~flush_w);

    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;

    localparam CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;

//==============================================================================
// 一、D 级组合量：JAL 目标 / 三态合一的跳转值 / 冲刷边界
//==============================================================================
    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

    reg [31:0] jal_target_now, jal_target_now_1, jmp_val_0, jmp_val_1;
    always @(*) begin
        jal_target_now   = pc_operand_in  + immJ(inst_in)  - 32'd4;
        jal_target_now_1 = pc_operand_in1 + immJ(inst_in1) - 32'd4;
        jmp_val_0 = dec_jalf  ? jal_target_now   : dec_jval;
        jmp_val_1 = dec_jalf1 ? jal_target_now_1 : dec_jval1;
    end

    reg [4:0] flush_age;
    always @(*) flush_age = {1'b0, (flush_idx - head_ptr)};

    reg kill_all;
    always @(*) kill_all = trap_fire | (flush_con_rob & ~rob_flush_part);

//==============================================================================
// 二、生产者表 pstat：每个架构寄存器"最年轻的在生产写者"
//   tag 的唯一来源。按【D 级登记】而不是装 Q 时登记：D 就是程序序的流，
//   被按住的 D 会连续多拍出现，所以只认身份变化（d_new），重复登记会把更年轻的
//   写者覆盖成更老的写者（静默错值）。
//   两条 lane 同拍进 D 时 lane0 恒比 lane1 老（idu1 的装载规则保证）⇒ 登记顺序固定。
//==============================================================================
    reg        pst_v [0:31];
    reg [3:0]  pst_s [0:31];
    reg        pst_g [0:31];
//提交/冲刷清理后的值（= tag 的 lane0 来源）
    reg        clr_v [0:31];
    reg [3:0]  clr_s [0:31];
    reg        clr_g [0:31];
//再叠 lane0 本拍登记（= tag 的 lane1 来源）
    reg        wr0_v [0:31];
    reg [3:0]  wr0_s [0:31];
    reg        wr0_g [0:31];
//下一个寄存器值（再叠 lane1 本拍登记）
    reg        pnv   [0:31];
    reg [3:0]  pns   [0:31];
    reg        png   [0:31];

//★ d_dead 是"这一条还被冲刷杀着"的标记。它必须是**定时器的形状**，不能靠认身份：
//  {idx, gen} 只有 5 位，冲刷回卷 + 反复重分配之后**会重复**（实测 seed3：槽5的 gen 转了一圈又回来，
//  新指令被当成老的那条杀掉 ⇒ 队头永不完成）。身份变化既不是"新的"的充分条件，也不是必要条件。
//  正确的生命周期只有两个事件：冲刷那一拍按【年龄】置位；这条被消费（acc）那一拍清掉。
//  acc 不看 d_dead（acc = s_advance & 空位 & ~flush），所以置位/清零不成环。
    reg        dk_v_0, dk_v_1;
    reg        d_dead_0, d_dead_1;
    reg        d_kill_0, d_kill_1;
    reg [4:0]  d_age_0, d_age_1;
    always @(*) begin
        d_age_0  = {1'b0, (idx_in  - head_ptr)};
        d_age_1  = {1'b0, (idx_in1 - head_ptr)};
        d_dead_0 = dk_v_0;
        d_dead_1 = dk_v_1;
//★ 冲刷那一拍【本拍就死】的那条：dk_v 要到下一拍才生效，可 pstat 的登记就在这一拍发生。
//  漏掉它会留下一个指向已死项的 tag —— 那一项的提交口永远不会出现（它已经被杀了），
//  消费者于是永久等待（实测 seed16：D 里 idx12 已被杀，仍登记成 x15 的生产者，Q 永久等 x15）。
        d_kill_0 = kill_all | (flush_con_rob & rob_flush_part &
                   (flush_incl ? (d_age_0 >= flush_age) : (d_age_0 > flush_age)));
        d_kill_1 = kill_all | (flush_con_rob & rob_flush_part &
                   (flush_incl ? (d_age_1 >= flush_age) : (d_age_1 > flush_age)));
    end
    reg        d_seen_v_0, d_seen_v_1;
    reg [3:0]  d_seen_idx_0, d_seen_idx_1;
    reg        d_seen_gen_0, d_seen_gen_1;
    reg        d_new_0, d_new_1;

    always @(*) begin
        d_new_0 = s_advance  & (~d_seen_v_0 | (d_seen_idx_0 != idx_in)
                                              | (d_seen_gen_0 != alloc_gen_in));
        d_new_1 = s_advance1 & (~d_seen_v_1 | (d_seen_idx_1 != idx_in1)
                                              | (d_seen_gen_1 != alloc_gen_in1));
    end

//---- 冲刷重建（见 §3.5 第 5 条）----
//★ 光"按年龄清表项"是错的：pstat 只记最年轻的写者；它被冲掉、而同一寄存器一个更老的写者
//  还在飞时，清表项会让后来者回落到 rdata（架构旧值）⇒ 静默错值。
//  做法：rob 把"这一沿之后仍活着的项"（rb_v/rb_rd/rb_gen，物理槽序）送来，这里按年龄
//  从老到新扫一遍、后写覆盖先写 ⇒ 留下最年轻的活写者。只在冲刷锥里，不在发射锥上。
    reg [4:0]  rba_rd [0:15];
    reg        rba_g  [0:15];
    reg        rba_v  [0:15];
    reg [3:0]  reb_sel;
    reg [4:0]  reb_rd;
    reg        reb_gv, reb_al;
    reg        reb_v [0:31];
    reg [3:0]  reb_s [0:31];
    reg        reb_g [0:31];
    integer rei, rsi;

    always @(*) begin
        for (rei = 0; rei < 16; rei = rei + 1) begin
            rba_rd[rei] = rb_rd[rei*5 +: 5];
            rba_g[rei]  = rb_gen[rei];
            rba_v[rei]  = rb_v[rei];
        end
    end

    always @(*) begin
        for (rei = 0; rei < 32; rei = rei + 1) begin
            reb_v[rei] = 1'b0; reb_s[rei] = 4'd0; reb_g[rei] = 1'b0;
        end
        for (rsi = 0; rsi < 16; rsi = rsi + 1) begin
            reb_sel = head_ptr + rsi[3:0];
            reb_rd  = rba_rd[reb_sel];
            reb_al  = rba_v[reb_sel];
            reb_gv  = rba_g[reb_sel];
            if (reb_al & (reb_rd != 5'd0)) begin
                reb_v[reb_rd] = 1'b1;
                reb_s[reb_rd] = reb_sel;
                reb_g[reb_rd] = reb_gv;
            end
        end
    end

    reg d_wr_0, d_wr_1, flush_any;
//★ 两条 lane 的 D 级【不再按 lane 分先后】：各自独立推进之后，两条 D 里躺着的身份会错开，
//  lane0 的 D 完全可能比 lane1 的 D 更年轻（lane1 那条被按住、lane0 已经换了下一条）。
//  登记顺序必须按 ROB 年龄：老的那条先写、年轻的覆盖。否则老写者会把年轻写者盖掉
//  ⇒ 后来者拿到一个比自己还年轻的生产者 tag，永远等不到值（实测 RAW 链第三拍就锁死）。
    reg d_sw;
    always @(*) begin
        d_sw = ({1'b0, (idx_in1 - head_ptr)} < {1'b0, (idx_in - head_ptr)});
    end

    integer pi;
    always @(*) begin
//★ 登记判据 = "这条会不会写 rd"，**不是** dec_we。dec_we 只在"结果从 ALU 口写回"时为 1，
//  LOAD 与 MUL 的 dec_we 都是 0（它们的结果从 lsu / mdu 的口回填）⇒ 用它登记会让
//  load/mul **永远不当生产者**：消费者不等它们、直接读架构旧值。
//  （实测 seed12：`lb x20` 之后紧接 `andi x23,x20,imm`，x23 拿到 0。）
//  rd 字段在 branch/store/系统非写类里已被译码器清成 0，所以 (rd != 0) 就是"写 rd"。
        d_wr_0 = d_new_0 & ~d_dead_0 & ~d_kill_0 & (dec_rd  != 5'd0);
        d_wr_1 = d_new_1 & ~d_dead_1 & ~d_kill_1 & (dec_rd1 != 5'd0);
        flush_any = kill_all | (flush_con_rob & rob_flush_part);
        for (pi = 0; pi < 32; pi = pi + 1) begin
//---- 基础值：平时 = 上一拍的 pstat；冲刷拍 = 从活着的 ROB 项重建 ----
            if (flush_any) begin
                clr_v[pi] = reb_v[pi];
                clr_s[pi] = reb_s[pi];
                clr_g[pi] = reb_g[pi];
            end
            else begin
                clr_v[pi] = pst_v[pi];
                clr_s[pi] = pst_s[pi];
                clr_g[pi] = pst_g[pi];
            end
//---- 提交口清（按 slot+gen，只按 rd 会把老写者清掉）----
            if (we_a & (clr_s[pi] == commit_slot0) & (clr_g[pi] == commit_gen0))
                clr_v[pi] = 1'b0;
            if (we_b & (clr_s[pi] == commit_slot1) & (clr_g[pi] == commit_gen1))
                clr_v[pi] = 1'b0;
//---- 本拍登记：先老后年轻（d_sw=1 时 lane1 是老的）----
            wr0_v[pi] = clr_v[pi];
            wr0_s[pi] = clr_s[pi];
            wr0_g[pi] = clr_g[pi];
            if (d_sw) begin
                if (d_wr_1 & (dec_rd1 == pi)) begin
                    wr0_v[pi] = 1'b1; wr0_s[pi] = idx_in1; wr0_g[pi] = alloc_gen_in1;
                end
            end
            else begin
                if (d_wr_0 & (dec_rd == pi)) begin
                    wr0_v[pi] = 1'b1; wr0_s[pi] = idx_in; wr0_g[pi] = alloc_gen_in;
                end
            end
            pnv[pi] = wr0_v[pi];
            pns[pi] = wr0_s[pi];
            png[pi] = wr0_g[pi];
            if (d_sw) begin
                if (d_wr_0 & (dec_rd == pi)) begin
                    pnv[pi] = 1'b1; pns[pi] = idx_in; png[pi] = alloc_gen_in;
                end
            end
            else begin
                if (d_wr_1 & (dec_rd1 == pi)) begin
                    pnv[pi] = 1'b1; pns[pi] = idx_in1; png[pi] = alloc_gen_in1;
                end
            end
        end
    end

//D 级的两源 tag（装 Q 那一沿锁进 Q）
//★ 老的那条看 clr（本拍谁都不算它的生产者）；年轻的那条看 wr0（含老那条本拍的登记）。
    reg        tag_a_0_v, tag_b_0_v, tag_a_1_v, tag_b_1_v;
    reg [3:0]  tag_a_0_s, tag_b_0_s, tag_a_1_s, tag_b_1_s;
    reg        tag_a_0_g, tag_b_0_g, tag_a_1_g, tag_b_1_g;
    always @(*) begin
        if (d_sw) begin
            tag_a_1_v = clr_v[rs1_in1]; tag_a_1_s = clr_s[rs1_in1]; tag_a_1_g = clr_g[rs1_in1];
            tag_b_1_v = clr_v[rs2_in1]; tag_b_1_s = clr_s[rs2_in1]; tag_b_1_g = clr_g[rs2_in1];
            tag_a_0_v = wr0_v[rs1_in];  tag_a_0_s = wr0_s[rs1_in];  tag_a_0_g = wr0_g[rs1_in];
            tag_b_0_v = wr0_v[rs2_in];  tag_b_0_s = wr0_s[rs2_in];  tag_b_0_g = wr0_g[rs2_in];
        end
        else begin
            tag_a_0_v = clr_v[rs1_in];  tag_a_0_s = clr_s[rs1_in];  tag_a_0_g = clr_g[rs1_in];
            tag_b_0_v = clr_v[rs2_in];  tag_b_0_s = clr_s[rs2_in];  tag_b_0_g = clr_g[rs2_in];
            tag_a_1_v = wr0_v[rs1_in1]; tag_a_1_s = wr0_s[rs1_in1]; tag_a_1_g = wr0_g[rs1_in1];
            tag_b_1_v = wr0_v[rs2_in1]; tag_b_1_s = wr0_s[rs2_in1]; tag_b_1_g = wr0_g[rs2_in1];
        end
    end

//★ 到达那一拍就要认【提交旁路】：生产者恰好在同一拍提交时，pstat 同一拍被清掉、tag 也就
//  指不到活人了。若只在 else 分支里认 cmt，这一拍进来的那条会带着 wait=1，而之后既没有
//  wake（tag 已指不到人）也没有 cmt（提交那一拍已经过去）⇒ 永久等待
    reg        cmtt_a_0, cmtt_b_0, cmtt_a_1, cmtt_b_1;
//  （实测 seed16 死锁：Q 里 idx14 等 x15，tag 指 slot12/gen1，那条正在同一拍提交）。
    always @(*) begin
        cmtt_a_0 = (we_a & (rd_a == rs1_in ) & (tag_a_0_s == commit_slot0) & (tag_a_0_g == commit_gen0))
                 | (we_b & (rd_b == rs1_in ) & (tag_a_0_s == commit_slot1) & (tag_a_0_g == commit_gen1));
        cmtt_b_0 = (we_a & (rd_a == rs2_in ) & (tag_b_0_s == commit_slot0) & (tag_b_0_g == commit_gen0))
                 | (we_b & (rd_b == rs2_in ) & (tag_b_0_s == commit_slot1) & (tag_b_0_g == commit_gen1));
        cmtt_a_1 = (we_a & (rd_a == rs1_in1) & (tag_a_1_s == commit_slot0) & (tag_a_1_g == commit_gen0))
                 | (we_b & (rd_b == rs1_in1) & (tag_a_1_s == commit_slot1) & (tag_a_1_g == commit_gen1));
        cmtt_b_1 = (we_a & (rd_a == rs2_in1) & (tag_b_1_s == commit_slot0) & (tag_b_1_g == commit_gen0))
                 | (we_b & (rd_b == rs2_in1) & (tag_b_1_s == commit_slot1) & (tag_b_1_g == commit_gen1));
    end

//★ D 侧的 tag 快照：只在 d_new 那一沿锁。同一身份被按住、在 D 里连躺几拍时**不能重算** ——
//  那时 pstat 里已经有它自己那一项，重算会把"自己"读成自己的生产者（消费者等自己 ⇒ 自锁）。
//  锁下之后再遇到生产者提交（槽+世代匹配），就把 hit 清掉：值已经在寄存器堆里，改吃架构值。
    reg        dt_a_0_v, dt_b_0_v, dt_a_1_v, dt_b_1_v;
    reg [3:0]  dt_a_0_s, dt_b_0_s, dt_a_1_s, dt_b_1_s;
    reg        dt_a_0_g, dt_b_0_g, dt_a_1_g, dt_b_1_g;
    reg [31:0] db_a_0, db_b_0, db_a_1, db_b_1;
//★ base 的取值必须补【提交口旁路】：rdata 是寄存读（地址由 F 级给、值下一拍到 D），
//  所以"刚提交这一拍"的那个值在 rdata 里还是**旧的**。而 pstat 在同一拍按提交口把 tag 清掉
//  （因为值已进寄存器堆） ⇒ 消费者就会拿着旧 rdata 当架构值直接发出去。
//  实测 seed9：`addi x1,x1,0x200` 与 `lh x11,1008(x1)` 的 D 拍重合 ⇒ 地址算成 0x00800000。
//  口径与 regfile 的 bypass() 一致，只是这里的拍数是对的。
    reg [31:0] dbnx_a_0, dbnx_b_0, dbnx_a_1, dbnx_b_1;
//★ 两口同 rd 时【口 B 赢】：B 是 head+1，比 A 年轻；regfile 的 bypass()/写口也是这个优先级。
//  反过来（A 先查）会在"同一拍退掉两条都写同一个 rd"时拿到**老那条**的值
//  （实测 seed301：`lhu x28` 与后一条写 x28 同拍退，`blt x28,x14` 的 A 口拿到 0）。
    always @(*) begin
        dbnx_a_0 = (we_b & (rd_b == rs1_in )) ? commit_data1 :
                   (we_a & (rd_a == rs1_in )) ? commit_data0 : rdata1_in;
        dbnx_b_0 = (we_b & (rd_b == rs2_in )) ? commit_data1 :
                   (we_a & (rd_a == rs2_in )) ? commit_data0 : rdata2_in;
        dbnx_a_1 = (we_b & (rd_b == rs1_in1)) ? commit_data1 :
                   (we_a & (rd_a == rs1_in1)) ? commit_data0 : rdata3_in;
        dbnx_b_1 = (we_b & (rd_b == rs2_in1)) ? commit_data1 :
                   (we_a & (rd_a == rs2_in1)) ? commit_data0 : rdata4_in;
    end
//★ D 侧的 base 快照：rdata 与 D 里的指令【同拍对齐】只在"这条刚进 D 那一拍"成立
//  （读口地址由 F 级给、寄存一拍到 D 拍）。D 一旦被按住，F 就走过去了、rdata 也跟着变，
//  所以在 d_new 那一沿把 base 锁下来，装 Q 的时候直接用。
//★ 它必须和 dt_* 在【同一个】always 块里更新：生产者提交时 tag 清掉、base 要同时换成提交值，
//  两块分开写就会漏掉后者（实测自锁）。
    always @(posedge clk) begin
        if (rst_q) begin
            dt_a_0_v <= 1'b0; dt_b_0_v <= 1'b0; dt_a_1_v <= 1'b0; dt_b_1_v <= 1'b0;
            dt_a_0_s <= 4'd0; dt_b_0_s <= 4'd0; dt_a_1_s <= 4'd0; dt_b_1_s <= 4'd0;
            dt_a_0_g <= 1'b0; dt_b_0_g <= 1'b0; dt_a_1_g <= 1'b0; dt_b_1_g <= 1'b0;
            db_a_0 <= 32'd0; db_b_0 <= 32'd0; db_a_1 <= 32'd0; db_b_1 <= 32'd0;
        end
        else begin
            if (d_new_0) begin
                dt_a_0_v <= tag_a_0_v & ~cmtt_a_0; dt_a_0_s <= tag_a_0_s; dt_a_0_g <= tag_a_0_g;
                dt_b_0_v <= tag_b_0_v & ~cmtt_b_0; dt_b_0_s <= tag_b_0_s; dt_b_0_g <= tag_b_0_g;
                db_a_0 <= dbnx_a_0; db_b_0 <= dbnx_b_0;
            end
            else begin
//★ 生产者在这条还躺在 D 里的时候提交了：清 hit（值已在寄存器堆）**并且**把 base 换成提交值。
//  只清 hit 不改 base 是错的一半 —— base 快照是"生产者还没写回"时拍的，留着它就是拿着旧值
//  直接发出去（实测：`lw xN,0(x31)` 的 A 口拿到 0，访存地址算成 0，dcache 永不应答 ⇒ 整核自锁）。
                if (we_a & (rd_a == rs1_in) & (dt_a_0_s == commit_slot0) & (dt_a_0_g == commit_gen0)) begin
                    dt_a_0_v <= 1'b0; db_a_0 <= commit_data0;
                end
                if (we_b & (rd_b == rs1_in) & (dt_a_0_s == commit_slot1) & (dt_a_0_g == commit_gen1)) begin
                    dt_a_0_v <= 1'b0; db_a_0 <= commit_data1;
                end
                if (we_a & (rd_a == rs2_in) & (dt_b_0_s == commit_slot0) & (dt_b_0_g == commit_gen0)) begin
                    dt_b_0_v <= 1'b0; db_b_0 <= commit_data0;
                end
                if (we_b & (rd_b == rs2_in) & (dt_b_0_s == commit_slot1) & (dt_b_0_g == commit_gen1)) begin
                    dt_b_0_v <= 1'b0; db_b_0 <= commit_data1;
                end
            end
            if (d_new_1) begin
                dt_a_1_v <= tag_a_1_v & ~cmtt_a_1; dt_a_1_s <= tag_a_1_s; dt_a_1_g <= tag_a_1_g;
                dt_b_1_v <= tag_b_1_v & ~cmtt_b_1; dt_b_1_s <= tag_b_1_s; dt_b_1_g <= tag_b_1_g;
                db_a_1 <= dbnx_a_1; db_b_1 <= dbnx_b_1;
            end
            else begin
//★ 同 lane0：清 hit 的同时把 base 换成提交值（base 快照是生产者还没写回时拍的）。
                if (we_a & (rd_a == rs1_in1) & (dt_a_1_s == commit_slot0) & (dt_a_1_g == commit_gen0)) begin
                    dt_a_1_v <= 1'b0; db_a_1 <= commit_data0;
                end
                if (we_b & (rd_b == rs1_in1) & (dt_a_1_s == commit_slot1) & (dt_a_1_g == commit_gen1)) begin
                    dt_a_1_v <= 1'b0; db_a_1 <= commit_data1;
                end
                if (we_a & (rd_a == rs2_in1) & (dt_b_1_s == commit_slot0) & (dt_b_1_g == commit_gen0)) begin
                    dt_b_1_v <= 1'b0; db_b_1 <= commit_data0;
                end
                if (we_b & (rd_b == rs2_in1) & (dt_b_1_s == commit_slot1) & (dt_b_1_g == commit_gen1)) begin
                    dt_b_1_v <= 1'b0; db_b_1 <= commit_data1;
                end
            end
        end
    end

//装 Q 那一拍取哪一份 tag：本拍正是这条指令【新出现】那一拍（d_new）⇒ 用组合那份；
//它已经在 D 里躺过（被按住过）⇒ 用锁下来的快照。两者不能合成一份：
//组合那份在"自己已经登记进 pstat"之后会把自分读成生产者，快照那份又拿不到刚出现那一拍的值。
    reg        ct_a_0_v, ct_b_0_v, ct_a_1_v, ct_b_1_v;
    reg [3:0]  ct_a_0_s, ct_b_0_s, ct_a_1_s, ct_b_1_s;
    reg        ct_a_0_g, ct_b_0_g, ct_a_1_g, ct_b_1_g;
    always @(*) begin
        ct_a_0_v = d_new_0 ? tag_a_0_v : dt_a_0_v;
        ct_b_0_v = d_new_0 ? tag_b_0_v : dt_b_0_v;
        ct_a_1_v = d_new_1 ? tag_a_1_v : dt_a_1_v;
        ct_b_1_v = d_new_1 ? tag_b_1_v : dt_b_1_v;
        ct_a_0_s = d_new_0 ? tag_a_0_s : dt_a_0_s;
        ct_b_0_s = d_new_0 ? tag_b_0_s : dt_b_0_s;
        ct_a_1_s = d_new_1 ? tag_a_1_s : dt_a_1_s;
        ct_b_1_s = d_new_1 ? tag_b_1_s : dt_b_1_s;
        ct_a_0_g = d_new_0 ? tag_a_0_g : dt_a_0_g;
        ct_b_0_g = d_new_0 ? tag_b_0_g : dt_b_0_g;
        ct_a_1_g = d_new_1 ? tag_a_1_g : dt_a_1_g;
        ct_b_1_g = d_new_1 ? tag_b_1_g : dt_b_1_g;
    end

//（db_* 的锁存与提交旁路已并入上面的 dt_* 那个块 —— 两者必须同沿同门更新）

//★ 装 Q／锁 dt 那一拍就要认【提交旁路】：生产者恰好在同一拍提交时，pstat 同一拍被清掉、
//  tag 也已经指不到活人。如果只在 else 分支里认 cmt，这一拍进来的那条就带着 wait=1，
//  而之后既没有 wake（tag 已指不到人）也没有 cmt（提交那一拍已经过去）⇒ 永久等待
//  （实测 seed16 死锁：Q 里 idx14 等 x15，tag 指 slot12/gen1，那条正在同一拍提交）。
    reg        cmtq_a_0, cmtq_b_0, cmtq_a_1, cmtq_b_1;
    always @(*) begin
        cmtq_a_0 = (we_a & (rd_a == rs1_in ) & (ct_a_0_s == commit_slot0) & (ct_a_0_g == commit_gen0))
                 | (we_b & (rd_b == rs1_in ) & (ct_a_0_s == commit_slot1) & (ct_a_0_g == commit_gen1));
        cmtq_b_0 = (we_a & (rd_a == rs2_in ) & (ct_b_0_s == commit_slot0) & (ct_b_0_g == commit_gen0))
                 | (we_b & (rd_b == rs2_in ) & (ct_b_0_s == commit_slot1) & (ct_b_0_g == commit_gen1));
        cmtq_a_1 = (we_a & (rd_a == rs1_in1) & (ct_a_1_s == commit_slot0) & (ct_a_1_g == commit_gen0))
                 | (we_b & (rd_b == rs1_in1) & (ct_a_1_s == commit_slot1) & (ct_a_1_g == commit_gen1));
        cmtq_b_1 = (we_a & (rd_a == rs2_in1) & (ct_b_1_s == commit_slot0) & (ct_b_1_g == commit_gen0))
                 | (we_b & (rd_b == rs2_in1) & (ct_b_1_s == commit_slot1) & (ct_b_1_g == commit_gen1));
    end
//装 Q 那一拍如果正好撞上提交，base 直接吃提交口的数据（与 regfile 的 bypass 同口径）
    reg [31:0] cmtd_a_0, cmtd_b_0, cmtd_a_1, cmtd_b_1;
    always @(*) begin
//装 Q 那一拍如果正好撞上提交，base 直接吃提交口的数据（与 regfile 的 bypass 同口径）。
//★ 同样【口 B 赢】：同一拍退掉的两条都写这个 rd 时，架构值是年轻的那条（B）。
        cmtd_a_0 = (we_b & (rd_b == rs1_in )) ? commit_data1 : commit_data0;
        cmtd_b_0 = (we_b & (rd_b == rs2_in )) ? commit_data1 : commit_data0;
        cmtd_a_1 = (we_b & (rd_b == rs1_in1)) ? commit_data1 : commit_data0;
        cmtd_b_1 = (we_b & (rd_b == rs2_in1)) ? commit_data1 : commit_data0;
    end

//==============================================================================
// 三、Q：两条 lane 各一份发射寄存器
//==============================================================================
    reg        qv_0, qv_1;
    reg [4:0]  q_rs_a_0, q_rs_b_0, q_rs_a_1, q_rs_b_1;
    reg [3:0]  q_idx_0, q_idx_1;
    reg        q_gen_0, q_gen_1;
    reg        q_real_0, q_real_1;
    reg [1:0]  q_class_0, q_class_1;
    reg        q_we_0, q_we_1;
    reg [4:0]  q_rd_0, q_rd_1;
    reg        q_is_st_0, q_is_st_1;
    reg [2:0]  q_mulop_0, q_mulop_1;
    reg [2:0]  q_lsf3_0, q_lsf3_1;
    reg [3:0]  q_func4_0, q_func4_1;
    reg [2:0]  q_csrf3_0, q_csrf3_1;
    reg        q_csrw_en_0, q_csrw_en_1;
    reg        q_csrw_wr_0, q_csrw_wr_1;
    reg [11:0] q_csra_0, q_csra_1;
    reg [31:0] q_jmpval_0, q_jmpval_1;
    reg [31:0] q_aux_0, q_aux_1;
    reg        q_brf_0, q_brf_1, q_jalf_0, q_jalf_1, q_jalfr_0, q_jalfr_1;
    reg  [3:0] q_bjuf4_0, q_bjuf4_1;
    reg        q_imm_a_0, q_imm_b_0, q_imm_a_1, q_imm_b_1;
    reg [31:0] q_immv_a_0, q_immv_b_0, q_immv_a_1, q_immv_b_1;
    reg        q_br_pred_0, q_br_pred_1;
//scoreboard：每源 {hit, slot, gen} + wait
    reg        q_t_hit_a_0, q_t_hit_b_0, q_t_hit_a_1, q_t_hit_b_1;
    reg [3:0]  q_t_slot_a_0, q_t_slot_b_0, q_t_slot_a_1, q_t_slot_b_1;
    reg        q_t_gen_a_0, q_t_gen_b_0, q_t_gen_a_1, q_t_gen_b_1;
    reg        q_wait_a_0, q_wait_b_0, q_wait_a_1, q_wait_b_1;
//唤醒时锁下来的值：带值唤醒之后操作数不再依赖活线
    reg        q_val_ok_a_0, q_val_ok_b_0, q_val_ok_a_1, q_val_ok_b_1;
    reg [31:0] q_val_a_0, q_val_b_0, q_val_a_1, q_val_b_1;
//rdata 兜底档
    reg [31:0] q_base_a_0, q_base_b_0, q_base_a_1, q_base_b_1;

    reg [4:0] q_age_0, q_age_1;
    reg kill_0, kill_1;
    always @(*) begin
        q_age_0 = {1'b0, (q_idx_0 - head_ptr)};
        q_age_1 = {1'b0, (q_idx_1 - head_ptr)};
        kill_0 = qv_0 & (kill_all | (flush_con_rob & rob_flush_part &
                 (flush_incl ? (q_age_0 >= flush_age) : (q_age_0 > flush_age))));
        kill_1 = qv_1 & (kill_all | (flush_con_rob & rob_flush_part &
                 (flush_incl ? (q_age_1 >= flush_age) : (q_age_1 > flush_age))));
    end

//==============================================================================
// 四、唤醒 / 就绪 / 操作数四档
//==============================================================================
    reg        hit_r_a_0, hit_r_b_0, hit_r_a_1, hit_r_b_1;
    reg        wake_a_0, wake_b_0, wake_a_1, wake_b_1;
    reg [31:0] wakev_a_0, wakev_b_0, wakev_a_1, wakev_b_1;
    reg        cmt_a_0, cmt_b_0, cmt_a_1, cmt_b_1;
//按端口分的提交命中：base 直接吃命中那个口的数据（不再靠读口每拍刷新）
    reg        ca_a_0, cb_a_0, ca_b_0, cb_b_0;
    reg        ca_a_1, cb_a_1, ca_b_1, cb_b_1;
    reg        wclr_a_0, wclr_b_0, wclr_a_1, wclr_b_1;
    reg        op_rdy_a_0, op_rdy_b_0, op_rdy_a_1, op_rdy_b_1;
    reg [31:0] op_a_0, op_b_0, op_a_1, op_b_1;
    reg        h_alu_a_0, h_alu_b_0, h_alu_a_1, h_alu_b_1;
    reg        h_alu2_a_0, h_alu2_b_0, h_alu2_a_1, h_alu2_b_1;
    reg        h_mul_a_0, h_mul_b_0, h_mul_a_1, h_mul_b_1;
    reg        h_ld_a_0, h_ld_b_0, h_ld_a_1, h_ld_b_1;
    reg        h_now_a_0, h_now_b_0, h_now_a_1, h_now_b_1;

    always @(*) begin
//---- lane0 源 a ----
        h_alu_a_0  = we_alu  & q_t_hit_a_0 & (idx_alu  == q_t_slot_a_0) & (gen_alu  == q_t_gen_a_0);
        h_alu2_a_0 = we_alu2 & q_t_hit_a_0 & (idx_alu2 == q_t_slot_a_0) & (gen_alu2 == q_t_gen_a_0);
        h_mul_a_0  = we_mul  & q_t_hit_a_0 & (idx_mul  == q_t_slot_a_0) & (gen_mul  == q_t_gen_a_0);
        h_ld_a_0   = we_ld   & q_t_hit_a_0 & (idx_ld   == q_t_slot_a_0) & (gen_ld   == q_t_gen_a_0);
        h_now_a_0  = now_v_0 & q_t_hit_a_0 & (now_idx_0 == q_t_slot_a_0) & (now_gen_0 == q_t_gen_a_0);
        hit_r_a_0  = h_alu_a_0 | h_alu2_a_0 | h_mul_a_0 | h_ld_a_0 | h_now_a_0;
        wake_a_0   = hit_r_a_0 | (qry_ok_a_0 & q_t_hit_a_0);
        wakev_a_0  = h_alu_a_0  ? data_alu  : h_alu2_a_0 ? data_alu2 :
                     h_mul_a_0  ? data_mul  : h_ld_a_0   ? data_ld   :
                     h_now_a_0  ? now_data_0 : qry_data_a_0;
//---- lane0 源 b ----
        h_alu_b_0  = we_alu  & q_t_hit_b_0 & (idx_alu  == q_t_slot_b_0) & (gen_alu  == q_t_gen_b_0);
        h_alu2_b_0 = we_alu2 & q_t_hit_b_0 & (idx_alu2 == q_t_slot_b_0) & (gen_alu2 == q_t_gen_b_0);
        h_mul_b_0  = we_mul  & q_t_hit_b_0 & (idx_mul  == q_t_slot_b_0) & (gen_mul  == q_t_gen_b_0);
        h_ld_b_0   = we_ld   & q_t_hit_b_0 & (idx_ld   == q_t_slot_b_0) & (gen_ld   == q_t_gen_b_0);
        h_now_b_0  = now_v_0 & q_t_hit_b_0 & (now_idx_0 == q_t_slot_b_0) & (now_gen_0 == q_t_gen_b_0);
        hit_r_b_0  = h_alu_b_0 | h_alu2_b_0 | h_mul_b_0 | h_ld_b_0 | h_now_b_0;
        wake_b_0   = hit_r_b_0 | (qry_ok_b_0 & q_t_hit_b_0);
        wakev_b_0  = h_alu_b_0  ? data_alu  : h_alu2_b_0 ? data_alu2 :
                     h_mul_b_0  ? data_mul  : h_ld_b_0   ? data_ld   :
                     h_now_b_0  ? now_data_0 : qry_data_b_0;
//---- lane1 源 a（now_1 是 lane1 自己的 alu）----
        h_alu_a_1  = we_alu  & q_t_hit_a_1 & (idx_alu  == q_t_slot_a_1) & (gen_alu  == q_t_gen_a_1);
        h_alu2_a_1 = we_alu2 & q_t_hit_a_1 & (idx_alu2 == q_t_slot_a_1) & (gen_alu2 == q_t_gen_a_1);
        h_mul_a_1  = we_mul  & q_t_hit_a_1 & (idx_mul  == q_t_slot_a_1) & (gen_mul  == q_t_gen_a_1);
        h_ld_a_1   = we_ld   & q_t_hit_a_1 & (idx_ld   == q_t_slot_a_1) & (gen_ld   == q_t_gen_a_1);
        h_now_a_1  = now_v_1 & q_t_hit_a_1 & (now_idx_1 == q_t_slot_a_1) & (now_gen_1 == q_t_gen_a_1);
        hit_r_a_1  = h_alu_a_1 | h_alu2_a_1 | h_mul_a_1 | h_ld_a_1 | h_now_a_1;
        wake_a_1   = hit_r_a_1 | (qry_ok_a_1 & q_t_hit_a_1);
        wakev_a_1  = h_alu_a_1  ? data_alu  : h_alu2_a_1 ? data_alu2 :
                     h_mul_a_1  ? data_mul  : h_ld_a_1   ? data_ld   :
                     h_now_a_1  ? now_data_1 : qry_data_a_1;
//---- lane1 源 b ----
        h_alu_b_1  = we_alu  & q_t_hit_b_1 & (idx_alu  == q_t_slot_b_1) & (gen_alu  == q_t_gen_b_1);
        h_alu2_b_1 = we_alu2 & q_t_hit_b_1 & (idx_alu2 == q_t_slot_b_1) & (gen_alu2 == q_t_gen_b_1);
        h_mul_b_1  = we_mul  & q_t_hit_b_1 & (idx_mul  == q_t_slot_b_1) & (gen_mul  == q_t_gen_b_1);
        h_ld_b_1   = we_ld   & q_t_hit_b_1 & (idx_ld   == q_t_slot_b_1) & (gen_ld   == q_t_gen_b_1);
        h_now_b_1  = now_v_1 & q_t_hit_b_1 & (now_idx_1 == q_t_slot_b_1) & (now_gen_1 == q_t_gen_b_1);
        hit_r_b_1  = h_alu_b_1 | h_alu2_b_1 | h_mul_b_1 | h_ld_b_1 | h_now_b_1;
        wake_b_1   = hit_r_b_1 | (qry_ok_b_1 & q_t_hit_b_1);
        wakev_b_1  = h_alu_b_1  ? data_alu  : h_alu2_b_1 ? data_alu2 :
                     h_mul_b_1  ? data_mul  : h_ld_b_1   ? data_ld   :
                     h_now_b_1  ? now_data_1 : qry_data_b_1;
//---- 提交口命中（rd + slot + gen 三者全比）----
//按端口分开留：base 的刷新要取【命中那个口的数据】（口 B 更年轻，同 rd 时它赢）。
        ca_a_0 = we_a & (rd_a == q_rs_a_0) & (commit_slot0 == q_t_slot_a_0) & (commit_gen0 == q_t_gen_a_0);
        cb_a_0 = we_b & (rd_b == q_rs_a_0) & (commit_slot1 == q_t_slot_a_0) & (commit_gen1 == q_t_gen_a_0);
        ca_b_0 = we_a & (rd_a == q_rs_b_0) & (commit_slot0 == q_t_slot_b_0) & (commit_gen0 == q_t_gen_b_0);
        cb_b_0 = we_b & (rd_b == q_rs_b_0) & (commit_slot1 == q_t_slot_b_0) & (commit_gen1 == q_t_gen_b_0);
        ca_a_1 = we_a & (rd_a == q_rs_a_1) & (commit_slot0 == q_t_slot_a_1) & (commit_gen0 == q_t_gen_a_1);
        cb_a_1 = we_b & (rd_b == q_rs_a_1) & (commit_slot1 == q_t_slot_a_1) & (commit_gen1 == q_t_gen_a_1);
        ca_b_1 = we_a & (rd_a == q_rs_b_1) & (commit_slot0 == q_t_slot_b_1) & (commit_gen0 == q_t_gen_b_1);
        cb_b_1 = we_b & (rd_b == q_rs_b_1) & (commit_slot1 == q_t_slot_b_1) & (commit_gen1 == q_t_gen_b_1);
        cmt_a_0 = ca_a_0 | cb_a_0;
        cmt_b_0 = ca_b_0 | cb_b_0;
        cmt_a_1 = ca_a_1 | cb_a_1;
        cmt_b_1 = ca_b_1 | cb_b_1;
        wclr_a_0 = wake_a_0 | cmt_a_0;
        wclr_b_0 = wake_b_0 | cmt_b_0;
        wclr_a_1 = wake_a_1 | cmt_a_1;
        wclr_b_1 = wake_b_1 | cmt_b_1;
//---- 就绪 + 四档取值 ----
        op_rdy_a_0 = q_imm_a_0 | ~q_wait_a_0 | wake_a_0;
        op_rdy_b_0 = q_imm_b_0 | ~q_wait_b_0 | wake_b_0;
        op_rdy_a_1 = q_imm_a_1 | ~q_wait_a_1 | wake_a_1;
        op_rdy_b_1 = q_imm_b_1 | ~q_wait_b_1 | wake_b_1;
        op_a_0 = q_imm_a_0 ? q_immv_a_0 :
                 q_val_ok_a_0 ? q_val_a_0 :
                 hit_r_a_0 ? wakev_a_0 :
                 (qry_ok_a_0 & q_t_hit_a_0) ? qry_data_a_0 : q_base_a_0;
        op_b_0 = q_imm_b_0 ? q_immv_b_0 :
                 q_val_ok_b_0 ? q_val_b_0 :
                 hit_r_b_0 ? wakev_b_0 :
                 (qry_ok_b_0 & q_t_hit_b_0) ? qry_data_b_0 : q_base_b_0;
        op_a_1 = q_imm_a_1 ? q_immv_a_1 :
                 q_val_ok_a_1 ? q_val_a_1 :
                 hit_r_a_1 ? wakev_a_1 :
                 (qry_ok_a_1 & q_t_hit_a_1) ? qry_data_a_1 : q_base_a_1;
        op_b_1 = q_imm_b_1 ? q_immv_b_1 :
                 q_val_ok_b_1 ? q_val_b_1 :
                 hit_r_b_1 ? wakev_b_1 :
                 (qry_ok_b_1 & q_t_hit_b_1) ? qry_data_b_1 : q_base_b_1;
    end

//==============================================================================
// 五、发射判据（每 lane 独立）
//==============================================================================
    reg is_div_0, is_div_1;
    reg lane0_wants_lsu, lane0_wants_bju;
    reg bju_iss_0, bju_iss_1, st_iss_0, st_iss_1;
    reg unit_ok_0, unit_ok_1, csr_held_0, csr_held_1;
    reg load_0, load_1;
//---- 访存口的【程序序】仲裁 ----
//★ lsu 只有一条口，它会按【进它的顺序】去读写内存 ⇒ 访存类必须按 ROB 年龄排队。
//  两条 lane 独立发射时这个顺序会被打破：实测 seed16 的尾声，`sw x21,76(x1)`（0x80024C）
//  比 tohost 那条 store 还晚一拍落到 dtcm，签名被提前读走（tb 一看到 dtcm[0]!=0 就收）。
//  判据：本 lane 这条访存是否【最老的未发射访存条】。候选 = 两条 Q + 两条 D
//  （D 是 idu1 输出级，还没进 Q，但它更老时也要挡住年轻的先走）。
//  不会死锁：数据相关一律指向更老的指令，所以"最老的那条"永远等得到它要的东西。
    reg [4:0] ag_q0, ag_q1, ag_d0, ag_d1;
    reg       ls_q0, ls_q1, ls_d0, ls_d1;
    reg       lsu_blk_0, lsu_blk_1;
    always @(*) begin
        ag_q0 = {1'b0, (q_idx_0 - head_ptr)};
        ag_q1 = {1'b0, (q_idx_1 - head_ptr)};
        ag_d0 = d_age_0;
        ag_d1 = d_age_1;
        ls_q0 = qv_0 & ~kill_0 & (q_class_0 == CLS_LSU);
        ls_q1 = qv_1 & ~kill_1 & (q_class_1 == CLS_LSU);
//★ D 那两条必须用【电平】(s_advance)，不能用 d_new 的【到达脉冲】：Q 满时 D 里这条会停
//  两拍以上，脉冲只在第一拍有效 ⇒ 第二拍"更老的 D 条"对仲裁不可见 ⇒ 更年轻的 Q 条先发，
//  两条 store 到 dtcm 的顺序被颠倒（实测 CoreMark c=6049：pc 0x3394 抢在 pc 0x3390 前面写，
//  ISS 的顺序相反；两条同址 store 时就会写错值）。
        ls_d0 = s_advance  & ~d_dead_0 & (dec_class  == CLS_LSU);
        ls_d1 = s_advance1 & ~d_dead_1 & (dec_class1 == CLS_LSU);
        lsu_blk_0 = (ls_q1 & (ag_q1 < ag_q0)) | (ls_d0 & (ag_d0 < ag_q0)) | (ls_d1 & (ag_d1 < ag_q0));
        lsu_blk_1 = (ls_q0 & (ag_q0 < ag_q1)) | (ls_d0 & (ag_d0 < ag_q1)) | (ls_d1 & (ag_d1 < ag_q1));
    end
    always @(*) begin
        is_div_0 = q_mulop_0[2];
        is_div_1 = q_mulop_1[2];
//lane0 的"想发"：lane1 的互斥项只能用组合量，不能反过来依赖 issue_v_1。
//★ 两个口各一项：LSU 是"一个口一拍只能进一条"的资源互斥；BJU 是【判定口】互斥 ——
//  同拍两条 BJU（异常也走这个口）会在下一拍【同拍判定】，而 cont 的判定回报一拍只报一条
//  （SEL 按年龄二选一）⇒ 更年轻那条的改向/异常被静默吞掉（实测 seed200：lane0 的 `bltu`
//  判"不跳"、lane1 的 `bgeu` 判"跳"，两条同拍判定，bgeu 的改向丢了，PC 一路顺下去）。
//  ⇒ 让 lane1 的 BJU 等 lane0 那条发完再发：判定口始终一拍一条，年龄仲裁剩下的就是纯"选谁报"。
//★ CSR 写【不可撤回】：它在发射那一拍就把值锁进 csr 寄存器组（csr.v 只挡"同拍被冲刷"），
//  错路执行会永久污染。⇒ 真写 CSR 的指令必须等到【自己是 ROB 队头】才许发：
//  那时比它老的全退完了，它一定在正确路径上；发射是顺序的，它一停后面也就跟着停，不会乱序。
//  实测 WEIRD seed36：0xcc8 的 `bne` 预测不跳 ⇒ 影子里的 `csrrwi x2,mscratch,24` 先发了、
//  把 mscratch 写成 24；随后 bne 判"跳"的冲刷只能撤销它的 rd 写，撤销不了 CSR 写 ⇒
//  重取后再读 mscratch 得到 24（应为旧值 0），逐条写流在第 24 笔分岔。
//★ 必须在 lane0_wants_* 之前算：它也要进那两个"想发"里（见下），否则 lane1 会被它永久挡住。
        csr_held_0 = q_csrw_wr_0 & (q_idx_0 != head_ptr);
        csr_held_1 = q_csrw_wr_1 & (q_idx_1 != head_ptr);
        lane0_wants_lsu = qv_0 & ~flush_w & ~kill_0 & op_rdy_a_0 & op_rdy_b_0
                        & (q_class_0 == CLS_LSU) & ~lsu_full & ~csr_held_0;
        lane0_wants_bju = qv_0 & ~flush_w & ~kill_0 & op_rdy_a_0 & op_rdy_b_0
                        & (q_class_0 == CLS_BJU) & bju_ok_0 & ~csr_held_0;
//★ CSR 指令也是 CLS_BJU（SYSTEM 走判定口）⇒ 它被 csr_held_0 按住时，lane0_wants_bju 必须
//  跟着落下，否则 lane1 那条【更老的】分支被"lane0 想发"永久挡住 ⇒ 队头永远发不出去、
//  队头也退不了 ⇒ 整核死锁（实测 WEIRD seed40：hd=3 是分支，lane1 的 q idx=3 就是它，held0=1 档住 lane1）。
        unit_ok_0 = 1'b1;
        case (q_class_0)
            CLS_MUL: unit_ok_0 = is_div_0 ? mdu_div_ok_0 : mdu_mul_ok_0;
            CLS_LSU: unit_ok_0 = ~lsu_full & ~lsu_blk_0;
            CLS_BJU: unit_ok_0 = bju_ok_0;
            default: unit_ok_0 = 1'b1;
        endcase
        unit_ok_0 = unit_ok_0 & ~csr_held_0;
//lane1：LSU 口让 lane0 先（资源互斥）；BJU 口同理让 lane0 那条先（判定口一拍一条）。
//MUL 口按两条独立流水各算各的，不互斥。
        unit_ok_1 = 1'b1;
        case (q_class_1)
            CLS_MUL: unit_ok_1 = is_div_1 ? mdu_div_ok_1 : mdu_mul_ok_1;
            CLS_LSU: unit_ok_1 = ~lsu_full & ~lsu_blk_1;
            CLS_BJU: unit_ok_1 = bju_ok_1 & ~lane0_wants_bju;
            default: unit_ok_1 = 1'b1;
        endcase
        unit_ok_1 = unit_ok_1 & ~csr_held_1;
        issue_v_0 = qv_0 & ~flush_w & ~kill_0 & op_rdy_a_0 & op_rdy_b_0 & unit_ok_0;
        issue_v_1 = qv_1 & ~flush_w & ~kill_1 & op_rdy_a_1 & op_rdy_b_1 & unit_ok_1;
//★ 同拍一条 BJU + 一条 store，且 BJU 更老 ⇒ store 这拍【不许发】。
//  原因（实测 CoreMark：list/state 两个 CRC 错）：双发射下 store 与更老的分支同拍发射时，
//  store 的写穿正好落在分支冲刷的【同一拍】上 —— lsu 出口按年龄作废已经晚了一拍，错路 store
//  直接把值写进 dtcm（实测 0x80766c 被错路 `sw x8,12(sp)` 写成上一轮的旧值，list/state CRC 全错）。
//  让它等一拍：下一拍正是 bju_in_flight 窗口（issue_v 全 0），再下一拍分支已落定 ⇒ 要么正常发、
//  要么已被冲刷杀掉。Q 里的条目不会丢（acc 与 issue_v 同门）。
//★ 年龄判据不能用 lane0/lane1：两条 lane 独立推进，lane1 的条目可能比 lane0【更老】
//  （实测这处就是 L0=idx8 store / L1=idx7 bju）。年龄小 = 老。
        bju_iss_0 = issue_v_0 & (q_class_0 == CLS_BJU);
        bju_iss_1 = issue_v_1 & (q_class_1 == CLS_BJU);
        st_iss_0  = issue_v_0 & (q_class_0 == CLS_LSU) & q_is_st_0;
        st_iss_1  = issue_v_1 & (q_class_1 == CLS_LSU) & q_is_st_1;
        issue_v_0 = issue_v_0 & ~(st_iss_0 & bju_iss_1 & ({1'b0,(q_idx_1 - head_ptr)} < {1'b0,(q_idx_0 - head_ptr)}));
        issue_v_1 = issue_v_1 & ~(st_iss_1 & bju_iss_0 & ({1'b0,(q_idx_0 - head_ptr)} < {1'b0,(q_idx_1 - head_ptr)}));
//D→Q：Q 空 / Q 本拍发出才装；冲刷拍一律不装。
//★ 错路 D（d_dead）照样给 acc：不给 credit 就把 idu1 永久按死；
//  它被消费掉但不进 Q（load 带 ~d_dead）。
        acc_0 = s_advance  & (~qv_0 | issue_v_0) & ~flush_w;
        acc_1 = s_advance1 & (~qv_1 | issue_v_1) & ~flush_w;
        load_0 = acc_0 & ~d_dead_0;
        load_1 = acc_1 & ~d_dead_1;
    end

//==============================================================================
// 六、出站字段（无论哪一类都驱动；消费端只在自己那一类里看）
//==============================================================================
    always @(*) begin
        sel_class_0 = q_class_0;
        rd_0        = q_rd_0;
        issue_idx_0 = q_idx_0;
        issue_gen_0 = q_gen_0;
//★ we 必须被 issue_v 门控：alu 的"登记判据"就是 we_in（它没有单独的 valid 输入），
//  Q 空着或没发时把 q_we 直接送出去，alu 会把它当成一条真指令、把上一笔的操作数再寄存一次
//  ⇒ 结果口冒出一笔假的完成，ROB 被打进错误的槽，唤醒阵列还会拿它去命中别人的 tag
//  （实测：lane0 在真生产者还没算完时就"醒"了，RAW 链整条算错）。
        we_0        = issue_v_0 & q_we_0;
        alu_func4_0 = q_func4_0;
        csr_func3_0 = q_csrf3_0;
        csr_wr_en_0 = issue_v_0 & q_csrw_en_0;
        csr_wr_act_0= issue_v_0 & q_csrw_wr_0;
        csr_addr_0  = issue_v_0 ? q_csra_0 : 12'd0;
        aux_addr_0  = q_aux_0;
        off_mem_0   = q_jmpval_0;
        beq_off_0   = q_brf_0   ? q_jmpval_0 : 32'd0;
        jalr_pred_addr_0 = q_jalfr_0 ? q_jmpval_0 : 32'd0;
        jal_target_0     = q_jalf_0  ? q_jmpval_0 : 32'd0;
        op_data_a_0 = op_a_0;
        op_data_b_0 = op_b_0;
        rs_a_0      = q_rs_a_0;
        rs_b_0      = q_rs_b_0;
        br_flag_0   = q_brf_0;
        jal_flag_0  = q_jalf_0;
        jalr_flag_0 = q_jalfr_0;
        br_pred_taken_0 = q_br_pred_0;
        bju_func4_0         = q_bjuf4_0;
        mul_go_0 = issue_v_0 & (q_class_0 == CLS_MUL) & ~is_div_0;
        div_go_0 = issue_v_0 & (q_class_0 == CLS_MUL) &  is_div_0;
        opc_0 = 7'd0; fn10_0 = 10'd0; fn10_ls_0 = 10'd0;
        case (q_class_0)
            CLS_LSU: begin
                opc_0      = q_is_st_0 ? OPCODE_STORE : OPCODE_LOAD;
                fn10_ls_0  = {7'd0, q_lsf3_0};
            end
            CLS_MUL: begin
                opc_0      = OPCODE_OP;
                fn10_0     = {7'b0000001, q_mulop_0};
            end
            default: ;
        endcase

        sel_class_1 = q_class_1;
        rd_1        = q_rd_1;
        issue_idx_1 = q_idx_1;
        issue_gen_1 = q_gen_1;
        we_1        = issue_v_1 & q_we_1;
        alu_func4_1 = q_func4_1;
        csr_func3_1 = q_csrf3_1;
        csr_wr_en_1 = issue_v_1 & q_csrw_en_1;
        csr_wr_act_1= issue_v_1 & q_csrw_wr_1;
        csr_addr_1  = issue_v_1 ? q_csra_1 : 12'd0;
        aux_addr_1  = q_aux_1;
        off_mem_1   = q_jmpval_1;
        beq_off_1   = q_brf_1   ? q_jmpval_1 : 32'd0;
        jalr_pred_addr_1 = q_jalfr_1 ? q_jmpval_1 : 32'd0;
        jal_target_1     = q_jalf_1  ? q_jmpval_1 : 32'd0;
        op_data_a_1 = op_a_1;
        op_data_b_1 = op_b_1;
        rs_a_1      = q_rs_a_1;
        rs_b_1      = q_rs_b_1;
        br_flag_1   = q_brf_1;
        jal_flag_1  = q_jalf_1;
        jalr_flag_1 = q_jalfr_1;
        br_pred_taken_1 = q_br_pred_1;
        bju_func4_1         = q_bjuf4_1;
        mul_go_1 = issue_v_1 & (q_class_1 == CLS_MUL) & ~is_div_1;
        div_go_1 = issue_v_1 & (q_class_1 == CLS_MUL) &  is_div_1;
        opc_1 = 7'd0; fn10_1 = 10'd0; fn10_ls_1 = 10'd0;
        case (q_class_1)
            CLS_LSU: begin
                opc_1      = q_is_st_1 ? OPCODE_STORE : OPCODE_LOAD;
                fn10_ls_1  = {7'd0, q_lsf3_1};
            end
            CLS_MUL: begin
                opc_1      = OPCODE_OP;
                fn10_1     = {7'b0000001, q_mulop_1};
            end
            default: ;
        endcase

//---- 共享 LSU 口：lane0 优先，lane1 不丢（它只是这一拍没发）----
        lsu_v = 1'b0; lsu_lane = 1'b0;
        lsu_opc = 7'd0; lsu_fn10 = 10'd0; lsu_rd = 5'd0;
        lsu_idx = 4'd0; lsu_gen = 1'b0;
        lsu_a = 32'd0; lsu_b = 32'd0; lsu_off = 32'd0;
        lsu_rs_a = 5'd0; lsu_rs_b = 5'd0;
        if (issue_v_0 & (q_class_0 == CLS_LSU)) begin
            lsu_v = 1'b1; lsu_lane = 1'b0;
            lsu_opc = opc_0; lsu_fn10 = fn10_ls_0; lsu_rd = q_rd_0;
            lsu_idx = q_idx_0; lsu_gen = q_gen_0;
            lsu_a = op_a_0; lsu_b = op_b_0; lsu_off = q_jmpval_0;
            lsu_rs_a = q_rs_a_0; lsu_rs_b = q_rs_b_0;
        end
        else if (issue_v_1 & (q_class_1 == CLS_LSU)) begin
            lsu_v = 1'b1; lsu_lane = 1'b1;
            lsu_opc = opc_1; lsu_fn10 = fn10_ls_1; lsu_rd = q_rd_1;
            lsu_idx = q_idx_1; lsu_gen = q_gen_1;
            lsu_a = op_a_1; lsu_b = op_b_1; lsu_off = q_jmpval_1;
            lsu_rs_a = q_rs_a_1; lsu_rs_b = q_rs_b_1;
        end
    end

//==============================================================================
// 七、中断边界：两条 lane 里按 ROB 号更老的**真实指令**（空泡不算）
//==============================================================================
    always @(*) begin
        head_v_o = (qv_0 & q_real_0) | (qv_1 & q_real_1);
        if (qv_0 & q_real_0 & qv_1 & q_real_1) begin
            if (q_age_1 < q_age_0) begin
                head_idx_o = q_idx_1; head_pc_o = q_aux_1;
            end
            else begin
                head_idx_o = q_idx_0; head_pc_o = q_aux_0;
            end
        end
        else if (qv_1 & q_real_1) begin
            head_idx_o = q_idx_1; head_pc_o = q_aux_1;
        end
        else begin
            head_idx_o = q_idx_0; head_pc_o = q_aux_0;
        end
    end

//==============================================================================
// 八、bju 在途（lsu 的早一拍遮挡）
//==============================================================================
    reg [1:0] bju_shift;
    reg       bju_issue_now;
    always @(*) begin
        bju_issue_now = (issue_v_0 & (q_class_0 == CLS_BJU))
                      | (issue_v_1 & (q_class_1 == CLS_BJU));
        bju_in_flight = bju_shift[0];
        issue_pop_0   = issue_v_0;
        issue_pop_1   = issue_v_1;
    end

//==============================================================================
// 九、拉式查询口 / 寄存器堆读口
//==============================================================================
    always @(*) begin
        qry_slot_a_0 = q_t_slot_a_0; qry_gen_a_0 = q_t_gen_a_0;
        qry_slot_b_0 = q_t_slot_b_0; qry_gen_b_0 = q_t_gen_b_0;
        qry_slot_a_1 = q_t_slot_a_1; qry_gen_a_1 = q_t_gen_a_1;
        qry_slot_b_1 = q_t_slot_b_1; qry_gen_b_1 = q_t_gen_b_1;
    end

//==============================================================================
// 十、时序：Q / pstat / D 身份
//==============================================================================
    always @(posedge clk) begin
        if (rst_q) begin
            qv_0 <= 1'b0;
            q_rs_a_0 <= 5'd0; q_rs_b_0 <= 5'd0;
            q_idx_0 <= 4'd0; q_gen_0 <= 1'b0; q_real_0 <= 1'b0;
            q_class_0 <= 2'd0; q_we_0 <= 1'b0; q_rd_0 <= 5'd0;
            q_is_st_0 <= 1'b0; q_mulop_0 <= 3'd0; q_lsf3_0 <= 3'd0;
            q_func4_0 <= 4'd0; q_csrf3_0 <= 3'd0;
            q_csrw_en_0 <= 1'b0; q_csrw_wr_0 <= 1'b0; q_csra_0 <= 12'd0;
            q_jmpval_0 <= 32'd0; q_aux_0 <= 32'd0;
            q_brf_0 <= 1'b0; q_jalf_0 <= 1'b0; q_jalfr_0 <= 1'b0;
            q_bjuf4_0 <= 4'd0;
            q_imm_a_0 <= 1'b0; q_imm_b_0 <= 1'b0;
            q_immv_a_0 <= 32'd0; q_immv_b_0 <= 32'd0;
            q_br_pred_0 <= 1'b0;
            q_t_hit_a_0 <= 1'b0; q_t_hit_b_0 <= 1'b0;
            q_t_slot_a_0 <= 4'd0; q_t_slot_b_0 <= 4'd0;
            q_t_gen_a_0 <= 1'b0; q_t_gen_b_0 <= 1'b0;
            q_wait_a_0 <= 1'b0; q_wait_b_0 <= 1'b0;
            q_val_ok_a_0 <= 1'b0; q_val_ok_b_0 <= 1'b0;
            q_val_a_0 <= 32'd0; q_val_b_0 <= 32'd0;
            q_base_a_0 <= 32'd0; q_base_b_0 <= 32'd0;
        end
        else if (load_0) begin
            qv_0 <= 1'b1;
            q_rs_a_0 <= rs1_in; q_rs_b_0 <= rs2_in;
            q_idx_0 <= idx_in; q_gen_0 <= alloc_gen_in;
            q_real_0 <= (aux_addr_in != 32'd0);
            q_class_0 <= dec_class; q_we_0 <= dec_we; q_rd_0 <= dec_rd;
            q_is_st_0 <= dec_is_st; q_mulop_0 <= dec_mulop; q_lsf3_0 <= dec_lsf3;
            q_func4_0 <= dec_func4; q_csrf3_0 <= dec_csrf3;
            q_csrw_en_0 <= dec_csrw_en; q_csrw_wr_0 <= dec_csrw_wr; q_csra_0 <= dec_csra;
            q_jmpval_0 <= jmp_val_0; q_aux_0 <= aux_addr_in;
            q_imm_a_0 <= dec_r1_imm; q_imm_b_0 <= dec_r2_imm;
            q_immv_a_0 <= dec_r1_immv; q_immv_b_0 <= dec_r2_immv;
            q_brf_0 <= dec_brf; q_jalf_0 <= dec_jalf; q_jalfr_0 <= dec_jalfr;
            q_bjuf4_0 <= dec_bjuf4;
            q_base_a_0 <= cmtq_a_0 ? cmtd_a_0 : (d_new_0 ? dbnx_a_0 : db_a_0);
            q_base_b_0 <= cmtq_b_0 ? cmtd_b_0 : (d_new_0 ? dbnx_b_0 : db_b_0);
            q_br_pred_0 <= br_pred_taken_in;
            q_t_hit_a_0 <= ct_a_0_v & ~cmtq_a_0; q_t_hit_b_0 <= ct_b_0_v & ~cmtq_b_0;
            q_t_slot_a_0 <= ct_a_0_s; q_t_slot_b_0 <= ct_b_0_s;
            q_t_gen_a_0 <= ct_a_0_g; q_t_gen_b_0 <= ct_b_0_g;
            q_wait_a_0 <= ct_a_0_v & ~cmtq_a_0; q_wait_b_0 <= ct_b_0_v & ~cmtq_b_0;
            q_val_ok_a_0 <= 1'b0; q_val_ok_b_0 <= 1'b0;
        end
        else if (issue_v_0 | kill_0) begin
            qv_0 <= 1'b0;
        end
        else begin
            q_wait_a_0 <= q_wait_a_0 & ~wclr_a_0;
            q_wait_b_0 <= q_wait_b_0 & ~wclr_b_0;
            q_t_hit_a_0 <= q_t_hit_a_0 & ~cmt_a_0;
            q_t_hit_b_0 <= q_t_hit_b_0 & ~cmt_b_0;
            q_val_ok_a_0 <= q_val_ok_a_0 | wake_a_0;
            q_val_ok_b_0 <= q_val_ok_b_0 | wake_b_0;
            if (wake_a_0 & ~q_val_ok_a_0) q_val_a_0 <= wakev_a_0;
            if (wake_b_0 & ~q_val_ok_b_0) q_val_b_0 <= wakev_b_0;
//★ base 只被"生产者提交"改写，且直接吃提交口的数据（口 B 更年轻，同 rd 它赢）。
//  读口这一拍跟的是 D 的 rs（为下一条预取），拿这里去刷新 base 会读到别的寄存器的值。
            if (cmt_a_0) q_base_a_0 <= cb_a_0 ? commit_data1 : commit_data0;
            if (cmt_b_0) q_base_b_0 <= cb_b_0 ? commit_data1 : commit_data0;
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            qv_1 <= 1'b0;
            q_rs_a_1 <= 5'd0; q_rs_b_1 <= 5'd0;
            q_idx_1 <= 4'd0; q_gen_1 <= 1'b0; q_real_1 <= 1'b0;
            q_class_1 <= 2'd0; q_we_1 <= 1'b0; q_rd_1 <= 5'd0;
            q_is_st_1 <= 1'b0; q_mulop_1 <= 3'd0; q_lsf3_1 <= 3'd0;
            q_func4_1 <= 4'd0; q_csrf3_1 <= 3'd0;
            q_csrw_en_1 <= 1'b0; q_csrw_wr_1 <= 1'b0; q_csra_1 <= 12'd0;
            q_jmpval_1 <= 32'd0; q_aux_1 <= 32'd0;
            q_brf_1 <= 1'b0; q_jalf_1 <= 1'b0; q_jalfr_1 <= 1'b0;
            q_bjuf4_1 <= 4'd0;
            q_imm_a_1 <= 1'b0; q_imm_b_1 <= 1'b0;
            q_immv_a_1 <= 32'd0; q_immv_b_1 <= 32'd0;
            q_br_pred_1 <= 1'b0;
            q_t_hit_a_1 <= 1'b0; q_t_hit_b_1 <= 1'b0;
            q_t_slot_a_1 <= 4'd0; q_t_slot_b_1 <= 4'd0;
            q_t_gen_a_1 <= 1'b0; q_t_gen_b_1 <= 1'b0;
            q_wait_a_1 <= 1'b0; q_wait_b_1 <= 1'b0;
            q_val_ok_a_1 <= 1'b0; q_val_ok_b_1 <= 1'b0;
            q_val_a_1 <= 32'd0; q_val_b_1 <= 32'd0;
            q_base_a_1 <= 32'd0; q_base_b_1 <= 32'd0;
        end
        else if (load_1) begin
            qv_1 <= 1'b1;
            q_rs_a_1 <= rs1_in1; q_rs_b_1 <= rs2_in1;
            q_idx_1 <= idx_in1; q_gen_1 <= alloc_gen_in1;
            q_real_1 <= (aux_addr_in1 != 32'd0);
            q_class_1 <= dec_class1; q_we_1 <= dec_we1; q_rd_1 <= dec_rd1;
            q_is_st_1 <= dec_is_st1; q_mulop_1 <= dec_mulop1; q_lsf3_1 <= dec_lsf31;
            q_func4_1 <= dec_func41; q_csrf3_1 <= dec_csrf31;
            q_csrw_en_1 <= dec_csrw_en1; q_csrw_wr_1 <= dec_csrw_wr1; q_csra_1 <= dec_csra1;
            q_jmpval_1 <= jmp_val_1; q_aux_1 <= aux_addr_in1;
            q_imm_a_1 <= dec_r1_imm1; q_imm_b_1 <= dec_r2_imm1;
            q_immv_a_1 <= dec_r1_immv1; q_immv_b_1 <= dec_r2_immv1;
            q_brf_1 <= dec_brf1; q_jalf_1 <= dec_jalf1; q_jalfr_1 <= dec_jalfr1;
            q_bjuf4_1 <= dec_bjuf41;
            q_base_a_1 <= cmtq_a_1 ? cmtd_a_1 : (d_new_1 ? dbnx_a_1 : db_a_1);
            q_base_b_1 <= cmtq_b_1 ? cmtd_b_1 : (d_new_1 ? dbnx_b_1 : db_b_1);
            q_br_pred_1 <= br_pred_taken_in1;
            q_t_hit_a_1 <= ct_a_1_v & ~cmtq_a_1; q_t_hit_b_1 <= ct_b_1_v & ~cmtq_b_1;
            q_t_slot_a_1 <= ct_a_1_s; q_t_slot_b_1 <= ct_b_1_s;
            q_t_gen_a_1 <= ct_a_1_g; q_t_gen_b_1 <= ct_b_1_g;
            q_wait_a_1 <= ct_a_1_v & ~cmtq_a_1; q_wait_b_1 <= ct_b_1_v & ~cmtq_b_1;
            q_val_ok_a_1 <= 1'b0; q_val_ok_b_1 <= 1'b0;
        end
        else if (issue_v_1 | kill_1) begin
            qv_1 <= 1'b0;
        end
        else begin
            q_wait_a_1 <= q_wait_a_1 & ~wclr_a_1;
            q_wait_b_1 <= q_wait_b_1 & ~wclr_b_1;
            q_t_hit_a_1 <= q_t_hit_a_1 & ~cmt_a_1;
            q_t_hit_b_1 <= q_t_hit_b_1 & ~cmt_b_1;
            q_val_ok_a_1 <= q_val_ok_a_1 | wake_a_1;
            q_val_ok_b_1 <= q_val_ok_b_1 | wake_b_1;
            if (wake_a_1 & ~q_val_ok_a_1) q_val_a_1 <= wakev_a_1;
            if (wake_b_1 & ~q_val_ok_b_1) q_val_b_1 <= wakev_b_1;
            if (cmt_a_1) q_base_a_1 <= cb_a_1 ? commit_data1 : commit_data0;
            if (cmt_b_1) q_base_b_1 <= cb_b_1 ? commit_data1 : commit_data0;
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            for (pi = 0; pi < 32; pi = pi + 1) begin
                pst_v[pi] <= 1'b0; pst_s[pi] <= 4'd0; pst_g[pi] <= 1'b0;
            end
        end
        else begin
            for (pi = 0; pi < 32; pi = pi + 1) begin
                pst_v[pi] <= pnv[pi]; pst_s[pi] <= pns[pi]; pst_g[pi] <= png[pi];
            end
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            d_seen_v_0 <= 1'b0; d_seen_v_1 <= 1'b0;
            d_seen_idx_0 <= 4'd0; d_seen_idx_1 <= 4'd0;
            d_seen_gen_0 <= 1'b0; d_seen_gen_1 <= 1'b0;
            dk_v_0 <= 1'b0; dk_v_1 <= 1'b0;
            bju_shift <= 2'd0;
        end
        else begin
            if (d_new_0) begin
                d_seen_v_0 <= 1'b1; d_seen_idx_0 <= idx_in; d_seen_gen_0 <= alloc_gen_in;
            end
            if (d_new_1) begin
                d_seen_v_1 <= 1'b1; d_seen_idx_1 <= idx_in1; d_seen_gen_1 <= alloc_gen_in1;
            end
            if (flush_w) begin
//★ 冲刷那一拍，D 里这条**不一定死**：两条 lane 独立推进 ⇒ 它可以比冲刷点老
//  （lane0 跑到跳转并判定，lane1 那条还卡在 D 后面等它的生产者）。只有比冲刷点年轻的才死。
//  死/活与"是不是同一条"必须分开记：死是年龄判据，身份只用来把标志挂到那一条身上。
                dk_v_0 <= kill_all | (flush_con_rob & rob_flush_part &
                          (flush_incl ? (d_age_0 >= flush_age) : (d_age_0 > flush_age)));
                dk_v_1 <= kill_all | (flush_con_rob & rob_flush_part &
                          (flush_incl ? (d_age_1 >= flush_age) : (d_age_1 > flush_age)));
                bju_shift <= 2'd0;
            end
            else begin
//这条被消费掉就把它身上的"死"标记清掉；这一 lane 手上【根本没有活指令】（s_advance=0）时也清 ——
//  那时 D 里没有东西可以被杀，标志留着只会误杀下一个住户（实测 seed3：lane0 空了两拍，
//  标志没清，新分配的 idx3 一进来就被当死的丢掉，ROB 满在 16、队头 e3 永不完成）。
                if (~s_advance  | acc_0) dk_v_0 <= 1'b0;
                if (~s_advance1 | acc_1) dk_v_1 <= 1'b0;
                bju_shift <= {bju_shift[0], bju_issue_now};
            end
        end
    end

endmodule
