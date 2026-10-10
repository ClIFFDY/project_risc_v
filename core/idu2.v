`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu2 —— 发射单元（issue unit）：译码 + 四类统一保留站
//   由 post_decoder 改造而来。原"载荷寄存器"（一份）换成【入站寄存器一份 + 站阵列】。
//
// ★ 三个本质差别（相对 post_decoder）：
//   ① 入站门里【没有 rdy1_in & rdy2_in】—— 那是"原地等"的本体。操作数没齐不再停整条流水线，
//      把这条留在站里等（w=1）。
//   ② 操作数在【入站那一拍】定型：源就绪 ⇒ 直接锁值（forw 的选择逻辑搬到这里）；
//      未就绪 ⇒ 锁生产者槽号 + 世代，等三个结果口的 idx/gen 广播唤醒。出站时项里已是终值。
//   ③ 出站按【年龄最小 且 满足顺序约束】挑一条；单发射，每拍最多一条。
//
// ★ 入站寄存器（q_*）为什么不能省：
//   regfile 是【寄存读】—— r1_data(t) 是按 rs1_2(t-1) 锁出来的，描述的是【上一拍站在 S 级】
//   那条指令的操作数；而 inst_2(t) 是【这一拍】那条。原来的 post_decoder 靠一拍载荷寄存器
//   把 inst_in 延到 t+1，正好与 r1_data(t+1) 对齐。撤掉这一拍 ⇒ 操作数错一拍
//   （前后两条读同一个寄存器时看不出来，一到真程序就炸）。
//   ⇒ 本模块的译码与入站判据一律基于 q_*；csr_addr 与载荷同源，出站那一拍组合给出。
//
// ★ 站深为什么是 6 而不是 4：
//   ROB 在【出队那一拍】发号（alloc_en = idu1.adv），指令要 4 拍后才 push
//   （alloc → fifo h0 → idu1 输出 → q_* 入站寄存器 → 站）。
//   "站满"必须把【已分配但还没入站】的在途条数算进去（pending_cnt），稳态下 occupancy+inf ≈ 5；
//   深度 4 会一直假满把前端停死，深度 6 才有真缓冲。
//
// ★ 顺序约束：lsu 互相按序 · bju 后方的 lsu 不得放行 · bju 自身按序 · CSR 类相对全体按序。
//
// ★ 冲刷清项：直接复用 rob 的窗口判据（rob.flush_part），不在站里重推一遍。
//////////////////////////////////////////////////////////////////////////////////

module idu2(
    input clk, rst,
    input [7:0] flag_bus,
    input        s_advance,

//---- 前端（idu1 输出）----
    input [9:0]  func10,
    input [4:0]  rd_in,
    input [4:0]  rs1_in, rs2_in,
    input [31:0] imm_alu_in,
    input [31:0] inst_in,
    input [31:0] offset_beq0_aux,
    input [31:0] pc_operand_in,
    input [31:0] aux_addr_in,
    input        br_pred_taken_in,
    input [31:0] jalr_pred_addr_in,
    input [2:0]  idx_in,
    input        alloc_gen_in,

//---- 入站操作数（regfile 寄存读，与 q_* 同拍）----
    input [31:0] r1_data, r2_data,

//---- rob：扫描槽 / 就绪 / 生产者世代 ----
    input [2:0]  fwd_slot1_in, fwd_slot2_in,
    input        fwd_hit1_in,  fwd_hit2_in,
    input        fwd_pg1_in,   fwd_pg2_in,
    input        rdy1_in, rdy2_in,
//载荷那一级的前送值（rob 出）：与 rdy*_in 同一份判据算出来的 done 与其配对的值。
//★ 入站取值必须带上这一路：生产者已 done 但还没提交时，阵列读还是老值、结果口也不带它了，
//  只有这一路能拿到真值（实测 x2 差 0xb8 那条）。
    input        pay_done1_in, pay_done2_in,
    input [31:0] pay_data1_in, pay_data2_in,

//---- 三个结果口：入站取值 + 站内唤醒 + 出站旁路 ----
    input        we_alu, input [2:0] idx_alu, input gen_alu, input [31:0] data_alu,
    input        we_mul, input [2:0] idx_mul, input gen_mul, input [31:0] data_mul,
    input        we_ld,  input [2:0] idx_ld,  input gen_ld,  input [31:0] data_ld,

//---- 冲刷 / 陷阱 / 顺序 ----
    input        flush_con_rob,
    input        rob_flush_part,
    input        flush_incl,
    input [2:0]  flush_idx,
    input        trap_fire,
    input [2:0]  head_ptr,
    input        flush_bju_pre,
    input        lsu_full_in,
//mdu 的两条 ready：乘法项看 mul_ready_in、除法项看 div_ready_in（按站项自己的算子门控）
    input        mul_ready_in, div_ready_in,

//---- 给 rob 的入站（分配）使能 ----
    output       alloc_en,
    output [2:0] pay_idx_out,
    input        alloc_rob_in,

//---- 出站：驱动四个单元的输入口 ----
    output       issue_v,
    output [2:0] head_idx_o,
    output [31:0] head_pc_o,
    output       head_v_o,
//mdu 两条接收口的 valid（一拍只出一条，所以两者至多一个为 1）
    output       mul_go, div_go,
    output [31:0] r1_final_out, r2_final_out,
    output reg [4:0]  rd_out,
    output reg [2:0]  issue_idx,
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

    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_MISC_MEM = 7'b0001111;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_SYSTEM = 7'b1110011;

    localparam CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;

    localparam DEPTH = 6;

//==============================================================================
// 一、入站寄存器（原来的"载荷"，不能省）+ 站满/在途计数
//==============================================================================
    reg [31:0] hold_inst, hold_imm, hold_aux, hold_offb, hold_pc, hold_jalr_pred;
    reg [4:0]  hold_rd, hold_rs1, hold_rs2;
    reg [2:0]  hold_idx;
    reg [9:0]  hold_func10;
    reg        hold_gen, hold_br_pred_taken;
//★ 扫描结果必须与 hold_idx 同一沿锁存（原来 post_decoder 就是 sel_slot <= fwd_slot1_in）。
//  rob 的扫描口按【当拍 S 级】那条扫，而 q_ 装的是【上一拍 S 级】那条 —— 不跟着锁就会
//  把下一条的生产者槽安到这一条头上（实测：站项变成"等自己"，永久等死）。
    reg [2:0]  hold_src1, hold_src2;
    reg        hold_hit1, hold_hit2, hold_prod_gen1, hold_prod_gen2;
    reg        src_en;

    reg [3:0]  occupancy_next;
    reg [2:0]  free_slot;
    reg        have_free_slot;
    reg [3:0]  flush_part_age;
    always @(*) flush_part_age = {1'b0, (flush_idx - head_ptr)};

    reg [2:0] pending_cnt;
    reg pending_inc, pending_dec;

//★ 入站这一级是【一个有深度的队列】（深 1），不是一个"每拍覆盖"的直通级。
//  原来 q_ 载荷每拍被覆盖、每拍被推：中间只要 src_en 停一拍，q_ 没被覆盖却仍被推 ⇒
//  推出去的是上一条的【重复】，而新那条永远丢了。ROB 那一项早 2~3 拍就分配好了 ⇒
//  变成"有项、没指令"：发不出、完不成、队头卡死（实测 misalign-beq-01）。
//  原设计不丢，是因为载荷寄存器一直拿着那条指令等门开。
//  hold_pend = "q_ 里还压着一条没推出去的"：它同时顶住前端（rs_full 的输出里带上它）。
    reg hold_pend, push_en;
    always @(*) begin
        src_en  = s_advance;
        rs_full = (((occupancy + {1'b0, pending_cnt}) >= 4'd6) | hold_pend);
        push_en = exec & ~flush_w & ~((occupancy + {1'b0, pending_cnt}) >= 4'd6) & have_free_slot & hold_pend;
    end

    always @(posedge clk) begin
        if (rst_q) begin
            hold_inst <= 32'd0; hold_imm <= 32'd0; hold_aux <= 32'd0; hold_offb <= 32'd0;
            hold_pc <= 32'd0; hold_jalr_pred <= 32'd0;
            hold_rd <= 5'd0; hold_rs1 <= 5'd0; hold_rs2 <= 5'd0;
            hold_idx <= 3'd0; hold_func10 <= 10'd0; hold_gen <= 1'b0; hold_br_pred_taken <= 1'b0;
            hold_src1 <= 3'd0; hold_src2 <= 3'd0; hold_hit1 <= 1'b0; hold_hit2 <= 1'b0;
            hold_prod_gen1 <= 1'b0; hold_prod_gen2 <= 1'b0;
            hold_pend <= 1'b0;
        end
        else begin
            if (src_en) hold_pend <= 1'b1;
            else if (push_en) hold_pend <= 1'b0;
//冲刷：压着的这条若比边界年轻就整条作废（与站内清项同口径）
            if (hold_pend & (trap_fire | (flush_con_rob & ~rob_flush_part)))
                hold_pend <= 1'b0;
            else if (hold_pend & flush_con_rob & rob_flush_part &
                     ({1'b0, (hold_idx - head_ptr)} > flush_part_age))
                hold_pend <= 1'b0;
            if (src_en) begin
            hold_inst <= inst_in;
            hold_imm  <= imm_alu_in;
            hold_aux  <= aux_addr_in;
            hold_offb <= offset_beq0_aux;
            hold_pc   <= pc_operand_in;
            hold_jalr_pred <= jalr_pred_addr_in;
            hold_rd   <= rd_in;
            hold_rs1  <= rs1_in;
            hold_rs2  <= rs2_in;
            hold_idx  <= idx_in;
            hold_func10 <= func10;
            hold_gen  <= alloc_gen_in;
            hold_br_pred_taken <= br_pred_taken_in;
            hold_src1   <= fwd_slot1_in;
            hold_src2   <= fwd_slot2_in;
            hold_hit1   <= fwd_hit1_in;
            hold_hit2   <= fwd_hit2_in;
            hold_prod_gen1  <= fwd_pg1_in;
            hold_prod_gen2  <= fwd_pg2_in;
            end
        end
    end

//给 rob 的 pay_idx：就是入站那条自己的号 ⇒ rob 的 rdy1/rdy2 描述的是【它】的两个源
    assign pay_idx_out = hold_idx;

//==============================================================================
// 二、组合译码（一律基于 q_*）
//==============================================================================
    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction
    reg [31:0] jal_target_now;
    always @(*) jal_target_now = hold_pc + immJ(hold_inst) - 32'd4;

    reg csr_is_access_now, csr_exists_now, csr_ro_now, csr_wr_now;
    always @(*) begin
        csr_is_access_now = (hold_inst[14:12] != 3'b000) && (hold_inst[14:12] != 3'b100);
//★ "写"的规范口径：CSRRW/CSRRWI 恒写；CSRRS/CSRRC/CSRRSI/CSRRCI 只有源（rs1 或 uimm）非 0 才写。
//  这一根同时决定"真写使能"和"对只读 CSR 的写要不要报非法"，两者是同一个判据。
        csr_wr_now        = (hold_inst[14:12] == 3'b001) | (hold_inst[14:12] == 3'b101) |
                            (hold_inst[19:15] != 5'd0);
        csr_ro_now        = (hold_inst[31:30] == 2'b11);
        csr_exists_now    = 1'b0;
        case (hold_inst[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_now = 1'b1;
            default: csr_exists_now = 1'b0;
        endcase
    end

    reg exc_illegal_now, exc_jmis_now;
    always @(*) begin
        exc_illegal_now = 1'b0;
        exc_jmis_now    = (hold_inst[6:0] == OPCODE_JAL) & hold_inst[21];
        if (hold_inst != 32'd0) begin
            case (hold_inst[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_now = 1'b0;
                OPCODE_LOAD: begin
                    if (hold_inst[14:12] == 3'b011) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b110) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b111) exc_illegal_now = 1'b1;
                end
                OPCODE_STORE: begin
                    if (hold_inst[14:12] == 3'b011) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b100) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b101) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b110) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b111) exc_illegal_now = 1'b1;
                end
                OPCODE_JALR: begin
                    if (hold_inst[14:12] != 3'b000) exc_illegal_now = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (hold_inst[14:12] == 3'b010) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b011) exc_illegal_now = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (hold_inst[14:12] == 3'b001) begin
                        if (hold_inst[31:25] != 7'd0) exc_illegal_now = 1'b1;
                    end
                    if (hold_inst[14:12] == 3'b101) begin
                        if ((hold_inst[31:25] != 7'd0) && (hold_inst[31:25] != 7'b0100000))
                            exc_illegal_now = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (hold_inst[31:25] == 7'b0000001) exc_illegal_now = 1'b0;
                    else if (hold_inst[14:12] == 3'b000) begin
                        if ((hold_inst[31:25] != 7'd0) && (hold_inst[31:25] != 7'b0100000))
                            exc_illegal_now = 1'b1;
                    end
                    else if (hold_inst[14:12] == 3'b001) begin
                        if (hold_inst[31:25] != 7'd0) exc_illegal_now = 1'b1;
                    end
                    else if (hold_inst[14:12] == 3'b101) begin
                        if ((hold_inst[31:25] != 7'd0) && (hold_inst[31:25] != 7'b0100000))
                            exc_illegal_now = 1'b1;
                    end
                    else begin
                        if (hold_inst[31:25] != 7'd0) exc_illegal_now = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (hold_inst[14:12] == 3'b010) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b011) exc_illegal_now = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (hold_inst[14:12] == 3'b100) exc_illegal_now = 1'b1;
                    if (hold_inst[14:12] == 3'b000) begin
                        case (hold_inst[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;   // ecall / ebreak / mret / wfi（wfi 当 NOP）
                            default: exc_illegal_now = 1'b1;
                        endcase
                    end
                    if (csr_is_access_now) begin
                        if (!csr_exists_now) exc_illegal_now = 1'b1;
                        if (csr_ro_now) begin
                            if (csr_wr_now) exc_illegal_now = 1'b1;
                        end
                    end
                end
                default: exc_illegal_now = 1'b1;
            endcase
        end
    end

//入站分类 + 各类字段装配
    reg [1:0]  class_now;
    reg        we_now, csrw_en_now, csrw_armed_now, csrw_wr_now;
    reg [4:0]  rd_now;
    reg        is_st_now;
    reg [2:0]  mulop_now, lsf3_now, csrf3_now;
    reg [3:0]  func4_now;
    reg [11:0] csra_now;
    reg [31:0] jval_now;
    reg        brf_now, jalf_now, jalfr_now;

    always @(*) begin
        class_now    = CLS_ALU;
        we_now     = 1'b0;
        csrw_en_now= 1'b0;
        csrw_armed_now = 1'b0;
        csrw_wr_now = 1'b0;
        rd_now     = 5'd0;
        is_st_now  = 1'b0;
        mulop_now  = 3'd0;
        lsf3_now   = 3'd0;
        csrf3_now  = 3'd0;
        func4_now  = 4'd0;
        csra_now   = 12'd0;
        jval_now   = 32'd0;
        brf_now    = 1'b0;
        jalf_now   = 1'b0;
        jalfr_now  = 1'b0;

        case (hold_inst[6:0])
            OPCODE_OP: begin
                if (hold_inst[31:25] == 7'b0000001) begin
                    class_now   = CLS_MUL;
                    mulop_now = hold_inst[14:12];
                end
                else begin
                    func4_now = {hold_func10[8], hold_func10[2:0]};
                    rd_now    = hold_rd;
                    we_now    = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                func4_now = {hold_func10[8], hold_func10[2:0]};
                rd_now    = hold_rd;
                we_now    = 1'b1;
            end
            OPCODE_JAL: begin
                class_now  = CLS_BJU;
                jalf_now = 1'b1;
                rd_now   = hold_rd;
                we_now   = 1'b1;
                jval_now = jal_target_now;
            end
            OPCODE_JALR: begin
                class_now   = CLS_BJU;
                jalfr_now = 1'b1;
                rd_now    = hold_rd;
                we_now    = 1'b1;
                jval_now  = hold_jalr_pred;
            end
            OPCODE_BRANCH: begin
                class_now   = CLS_BJU;
                brf_now   = 1'b1;
                func4_now = {1'b0, hold_inst[14:12]};
                jval_now  = hold_offb - 32'd4;
            end
            OPCODE_LUI: begin
                rd_now = hold_rd;
                we_now = 1'b1;
            end
            OPCODE_AUIPC: begin
                rd_now = hold_rd;
                we_now = 1'b1;
            end
            OPCODE_LOAD: begin
                class_now  = CLS_LSU;
                lsf3_now = hold_inst[14:12];
                rd_now   = hold_rd;
                jval_now = hold_imm;
            end
            OPCODE_STORE: begin
                class_now   = CLS_LSU;
                is_st_now = 1'b1;
                lsf3_now  = hold_inst[14:12];
                jval_now  = hold_imm;
            end
            OPCODE_SYSTEM: begin
                class_now    = CLS_BJU;
                csrf3_now  = hold_inst[14:12];
                rd_now     = hold_rd;
                case (hold_inst[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        we_now         = 1'b1;
                        csrw_en_now    = 1'b1;
                        csrw_armed_now = 1'b1;
                        csrw_wr_now    = csr_wr_now;
                    end
                    default: begin
                        we_now      = 1'b0;
                        csrw_en_now = 1'b0;
                    end
                endcase
//★ csr 地址【不写也照给】：地址供 csr.v 的读口用，写不写只决定"锁不锁"。
//  `csrr rd, csr` 若被清成地址 0 就会读回 0（实测：启动宏读 mtvec 读回 0 ⇒ 误判没写进去）。
                csra_now = (hold_inst[14:12] != 3'b000) ? hold_inst[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_now | exc_jmis_now) begin
            we_now         = 1'b0;
            csrw_en_now    = 1'b0;
            csrw_armed_now = 1'b0;
            csrw_wr_now    = 1'b0;
        end
    end

//非寄存器操作数的覆盖（立即数在入站当拍并进操作数）
    reg r1_imm_now, r2_imm_now;
    reg [31:0] r1_immv_now, r2_immv_now;
    always @(*) begin
        r1_imm_now  = 1'b0;
        r2_imm_now  = 1'b0;
        r1_immv_now = 32'd0;
        r2_immv_now = 32'd0;
        case (hold_inst[6:0])
            OPCODE_OP_IMM: begin r2_imm_now = 1'b1; r2_immv_now = hold_imm; end
            OPCODE_JALR:   begin r2_imm_now = 1'b1; r2_immv_now = hold_imm; end
            OPCODE_LUI:    begin r2_imm_now = 1'b1; r2_immv_now = hold_imm; end
            OPCODE_AUIPC:  begin
                r1_imm_now  = 1'b1; r1_immv_now = hold_pc;
                r2_imm_now  = 1'b1; r2_immv_now = hold_imm;
            end
            OPCODE_SYSTEM: begin
                if ((hold_inst[14:12] == 3'b101) | (hold_inst[14:12] == 3'b110) | (hold_inst[14:12] == 3'b111)) begin
                    r1_imm_now  = 1'b1;
                    r1_immv_now = {27'd0, hold_inst[19:15]};
                end
            end
            default: ;
        endcase
    end

//异常标记（ecall/ebreak/mret）
    reg exc_ecall_now, exc_ebreak_now, exc_irq_ret_now;
    always @(*) begin
        exc_ecall_now   = (hold_inst[6:0] == OPCODE_SYSTEM) && (hold_inst[31:20] == 12'h000);
        exc_ebreak_now  = (hold_inst[6:0] == OPCODE_SYSTEM) && (hold_inst[31:20] == 12'h001);
        exc_irq_ret_now = (hold_inst[6:0] == OPCODE_SYSTEM) && (hold_inst[14:12] == 3'b000)
                        && (hold_inst[31:20] == 12'h302);
    end

//入站取值：forw 的选择逻辑（档序与 forw.v 逐字一致）
    reg [31:0] cap_op1, cap_op2;
    always @(*) begin
//★ 档序：立即数 → rob 项里那份值（done）→ 三个结果口（idx+世代都比）→ 阵列读。
//  结果口那三支必须比世代：只比槽号的话，槽复用后会把别人的结果当成生产者的值。
        if (r1_imm_now)                                  cap_op1 = r1_immv_now;
        else if (hold_hit1 & pay_done1_in)               cap_op1 = pay_data1_in;
        else if (hold_hit1 & we_alu & (idx_alu == hold_src1) & (gen_alu == hold_prod_gen1)) cap_op1 = data_alu;
        else if (hold_hit1 & we_mul & (idx_mul == hold_src1) & (gen_mul == hold_prod_gen1)) cap_op1 = data_mul;
        else if (hold_hit1 & we_ld  & (idx_ld  == hold_src1) & (gen_ld  == hold_prod_gen1)) cap_op1 = data_ld;
        else                                             cap_op1 = r1_data;

        if (r2_imm_now)                                  cap_op2 = r2_immv_now;
        else if (hold_hit2 & pay_done2_in)               cap_op2 = pay_data2_in;
        else if (hold_hit2 & we_alu & (idx_alu == hold_src2) & (gen_alu == hold_prod_gen2)) cap_op2 = data_alu;
        else if (hold_hit2 & we_mul & (idx_mul == hold_src2) & (gen_mul == hold_prod_gen2)) cap_op2 = data_mul;
        else if (hold_hit2 & we_ld  & (idx_ld  == hold_src2) & (gen_ld  == hold_prod_gen2)) cap_op2 = data_ld;
        else                                             cap_op2 = r2_data;
    end

    reg w1_now, w2_now;
    always @(*) begin
        w1_now = ~r1_imm_now & ~rdy1_in;
        w2_now = ~r2_imm_now & ~rdy2_in;
    end

//==============================================================================
// 三、入站门 + 站阵列
//==============================================================================
    assign alloc_en = src_en;
//★ 这里【没有】rdy1_in & rdy2_in —— 那就是"原地等"，本次改造要拿掉的东西。

    reg        ent_valid   [0:DEPTH-1];
    reg [1:0]  ent_class [0:DEPTH-1];
    reg [2:0]  ent_idx   [0:DEPTH-1];
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
    reg [2:0]  ent_src1  [0:DEPTH-1];
    reg [2:0]  ent_src2  [0:DEPTH-1];
    reg        ent_prod_gen1 [0:DEPTH-1];
    reg        ent_prod_gen2 [0:DEPTH-1];

    integer i, j;
    always @(*) begin
        free_slot = 3'd0;
        have_free_slot = 1'b0;
        for (i = DEPTH-1; i >= 0; i = i - 1) begin
            if (~ent_valid[i]) begin
                free_slot = i[2:0];
                have_free_slot = 1'b1;
            end
        end
    end
    always @(*) begin
        pending_inc = alloc_rob_in & (pending_cnt != 3'd4);
        pending_dec = push_en & (pending_cnt != 3'd0);
    end

//---- 唤醒（组合）：结果口广播按 idx+gen 双匹配 ----
    reg [DEPTH-1:0] wake1, wake2;
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            wake1[i] = ent_wait1[i] & ((we_alu & (idx_alu == ent_src1[i]) & (gen_alu == ent_prod_gen1[i])) |
                               (we_mul & (idx_mul == ent_src1[i]) & (gen_mul == ent_prod_gen1[i])) |
                               (we_ld  & (idx_ld  == ent_src1[i]) & (gen_ld  == ent_prod_gen1[i])));
            wake2[i] = ent_wait2[i] & ((we_alu & (idx_alu == ent_src2[i]) & (gen_alu == ent_prod_gen2[i])) |
                               (we_mul & (idx_mul == ent_src2[i]) & (gen_mul == ent_prod_gen2[i])) |
                               (we_ld  & (idx_ld  == ent_src2[i]) & (gen_ld  == ent_prod_gen2[i])));
        end
    end

//---- 年龄前缀扫描（顺序约束）----
    reg [3:0] ent_age [0:DEPTH-1];
    reg [DEPTH-1:0] ent_has_older_any, ent_has_older_lsu, ent_has_older_bju;
    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1)
            ent_age[i] = {1'b0, (ent_idx[i] - head_ptr)};
        for (i = 0; i < DEPTH; i = i + 1) begin
            ent_has_older_any[i] = 1'b0;
            ent_has_older_lsu[i] = 1'b0;
            ent_has_older_bju[i] = 1'b0;
            for (j = 0; j < DEPTH; j = j + 1) begin
                if (ent_valid[j] && (ent_age[j] < ent_age[i])) begin
                    ent_has_older_any[i] = 1'b1;
                    if (ent_class[j] == CLS_LSU) ent_has_older_lsu[i] = 1'b1;
                    if (ent_class[j] == CLS_BJU) ent_has_older_bju[i] = 1'b1;
                end
            end
        end
    end

//---- bju 单元在途：发出 bju 之后的 3 拍内 lsu 一律不许出站（保守但安全）----
    reg [1:0] bju_shift;
    reg bju_pending;
    always @(*) bju_pending = bju_shift[0] | bju_shift[1];

//==============================================================================
// 四、候选 / 选优 / 输出 mux
//==============================================================================
    reg [DEPTH-1:0] ent_issuable;
    reg [3:0] sel_age;
    reg       have_sel;
    reg [2:0] sel_slot;

    always @(*) begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            ent_issuable[i] = ent_valid[i] & (~ent_wait1[i] | wake1[i]) & (~ent_wait2[i] | wake2[i]);
            if (ent_class[i] == CLS_LSU)
                ent_issuable[i] = ent_issuable[i] & ~ent_has_older_lsu[i] & ~ent_has_older_bju[i]
                         & ~bju_pending & ~flush_bju_pre & ~lsu_full_in;
            if (ent_class[i] == CLS_BJU)
                ent_issuable[i] = ent_issuable[i] & ~ent_has_older_bju[i] & ~bju_pending;
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
                ent_issuable[i] = ent_issuable[i] & ~ent_has_older_bju[i] & ~bju_pending & ~flush_bju_pre;
            if (ent_csr_armed[i])
                ent_issuable[i] = ent_issuable[i] & ~ent_has_older_any[i];
        end
    end

//★ 站里【最老有效项】（不是"最老可发"）：中断的返回点与它的冲刷边界都以它为准 ——
//  "最老可发"会跳过正在等操作数的那条，于是那条既被冲掉、又不在 mepc 覆盖范围内 ⇒ 静默跳指令。
//  它的地址取项里的 aux_addr（"pc+4"口径），中断那条减 4 得自身地址。
//★ 只能挑【真实指令】：取指队列空的时候，出队格整条压 0（hd0_v=0）而 idu1 照常推进
//  ⇒ 站里会出现"指令字全 0、地址也是 0"的空泡项（它占 ROB 一项、当 NOP 退掉，本来是良性的）。
//  空泡没有地址 ⇒ 拿它当边界就会把 mepc 写成 0-4。真实指令的地址 = pc+4 恒 >= 4，故用 aux_addr!=0 挑。
//  （真出现"整站只有空泡"的短窗时中断就多等一拍，等即可，不会指到假地址。）
    integer hi;
    reg [3:0] head_best_age;
    reg [2:0] head_sel;
    always @(*) begin
        head_sel = 3'd0;
        head_best_age = 4'd15;
        for (hi = 0; hi < DEPTH; hi = hi + 1) begin
            if (ent_valid[hi] && (ent_aux_addr[hi] != 32'd0) && (ent_age[hi] < head_best_age)) begin
                head_best_age = ent_age[hi];
                head_sel = hi[2:0];
            end
        end
    end
    assign head_idx_o = ent_idx[head_sel];
    assign head_pc_o  = ent_aux_addr[head_sel];
//给中断资格用：站里【至少有一条真实指令没执行】，才谈得上"中断切点"。
// 中断的 mepc 与冲刷边界都取它，并且它自己也要被冲掉（含边界的杀集）。
// 站空 / 只有空泡时，下一条还没执行的指令还在 idu1 输出或取指队列里，本模块给不出它的地址
//（实测 mt13：拿空泡的 aux=0 当边界 ⇒ mepc=0xFFFFFFFC ⇒ 整机跑飞）。
// 这种窗只有几拍，等即可。
    assign head_v_o = (head_best_age != 4'd15);

    always @(*) begin
        have_sel = 1'b0;
        sel_slot    = 3'd0;
        sel_age  = 4'd15;
        for (i = 0; i < DEPTH; i = i + 1) begin
            if (ent_issuable[i] && (ent_age[i] < sel_age)) begin
                sel_age  = ent_age[i];
                sel_slot    = i[2:0];
                have_sel = 1'b1;
            end
        end
    end

    assign issue_v = have_sel & exec & ~flush_w;
//mdu 两条接收口：一拍至多一条（类别与算子家族都取被选中那一项）
    assign mul_go = issue_v & (ent_class[sel_slot] == CLS_MUL) & ~ent_is_div[sel_slot];
    assign div_go = issue_v & (ent_class[sel_slot] == CLS_MUL) &  ent_is_div[sel_slot];

//出站操作数：当拍才被唤醒的 ⇒ 直接用结果口直通值（唤醒旁路，省 1 拍）
    reg [31:0] out_op1, out_op2;
    always @(*) begin
        out_op1 = ent_op1[sel_slot];
        if (ent_wait1[sel_slot] && wake1[sel_slot]) begin
            if      (we_alu & (idx_alu == ent_src1[sel_slot]) & (gen_alu == ent_prod_gen1[sel_slot])) out_op1 = data_alu;
            else if (we_mul & (idx_mul == ent_src1[sel_slot]) & (gen_mul == ent_prod_gen1[sel_slot])) out_op1 = data_mul;
            else if (we_ld  & (idx_ld  == ent_src1[sel_slot]) & (gen_ld  == ent_prod_gen1[sel_slot])) out_op1 = data_ld;
        end
        out_op2 = ent_op2[sel_slot];
        if (ent_wait2[sel_slot] && wake2[sel_slot]) begin
            if      (we_alu & (idx_alu == ent_src2[sel_slot]) & (gen_alu == ent_prod_gen2[sel_slot])) out_op2 = data_alu;
            else if (we_mul & (idx_mul == ent_src2[sel_slot]) & (gen_mul == ent_prod_gen2[sel_slot])) out_op2 = data_mul;
            else if (we_ld  & (idx_ld  == ent_src2[sel_slot]) & (gen_ld  == ent_prod_gen2[sel_slot])) out_op2 = data_ld;
        end
    end
    assign r1_final_out = out_op1;
    assign r2_final_out = out_op2;

//出站字段：由选中项重构各单元要的口（opcode/func10 由 cls 重构，省站项宽度）
    always @(*) begin
        rd_out        = ent_rd[sel_slot];
        issue_idx     = ent_idx[sel_slot];
        issue_gen     = ent_gen[sel_slot];
        we            = ent_we[sel_slot];
        alu_func4     = ent_alu_func4[sel_slot];
        csr_func3     = ent_csr_func3[sel_slot];
        csr_wr_en     = ent_csr_wr_en[sel_slot];
        csr_wr_act    = ent_csr_wr_act[sel_slot];
        csr_addr      = ent_csr_addr[sel_slot];
        aux_addr_out  = ent_aux_addr[sel_slot];
        off_mem_out   = ent_jmp_val[sel_slot];
        br_flag       = ent_br_flag[sel_slot];
        jal_flag      = ent_jal_flag[sel_slot];
        jalr_flag     = ent_jalr_flag[sel_slot];
        br_pred_taken_out = ent_br_pred_taken[sel_slot];
        exc_illegal_out   = ent_exc_illegal[sel_slot];
        exc_ecall         = ent_exc_ecall[sel_slot];
        exc_ebreak        = ent_exc_ebreak[sel_slot];
        exc_irq_ret       = ent_exc_irq_ret[sel_slot];
        exc_jal_misalign_out = ent_exc_jal_misalign[sel_slot];
        rs1_out       = 5'd0;
        rs2_out       = 5'd0;
        opc_out     = 7'd0;
        fn10_out    = 10'd0;
        fn10_ls_out = 10'd0;
        beq_off_q2        = 32'd0;
        jalr_pred_addr_out= 32'd0;
        jal_target_out    = 32'd0;
        case (ent_class[sel_slot])
            CLS_LSU: begin
                opc_out     = ent_is_store[sel_slot] ? OPCODE_STORE : OPCODE_LOAD;
                fn10_ls_out = {7'd0, ent_ls_func3[sel_slot]};
            end
            CLS_MUL: begin
                opc_out  = OPCODE_OP;
                fn10_out = {7'b0000001, ent_mul_op[sel_slot]};
            end
            CLS_BJU: begin
                if (ent_br_flag[sel_slot])  beq_off_q2         = ent_jmp_val[sel_slot];
                if (ent_jalr_flag[sel_slot]) jalr_pred_addr_out = ent_jmp_val[sel_slot];
                if (ent_jal_flag[sel_slot]) jal_target_out     = ent_jmp_val[sel_slot];
            end
            default: ;
        endcase
        if (!have_sel) begin
            rd_out = 5'd0; issue_idx = 3'd0; issue_gen = 1'b0; we = 1'b0;
            alu_func4 = 4'd0; csr_func3 = 3'd0; aux_addr_out = 32'd0; off_mem_out = 32'd0;
            br_flag = 1'b0; jal_flag = 1'b0; jalr_flag = 1'b0; br_pred_taken_out = 1'b0;
            exc_illegal_out = 1'b0; exc_ecall = 1'b0; exc_ebreak = 1'b0;
            exc_irq_ret = 1'b0; exc_jal_misalign_out = 1'b0;
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

//==============================================================================
// 五、时序
//==============================================================================
    always @(posedge clk) begin
        if (rst_q) begin
            occupancy      <= 4'd0;
            pending_cnt  <= 3'd0;
            rs_full  <= 1'b0;
            bju_shift   <= 2'd0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                ent_valid[i]   <= 1'b0;  ent_wait1[i] <= 1'b0;  ent_wait2[i] <= 1'b0;
                ent_class[i] <= 2'd0;  ent_idx[i]  <= 3'd0;  ent_gen[i]  <= 1'b0;
                ent_we[i]  <= 1'b0;  ent_rd[i] <= 5'd0;  ent_is_store[i] <= 1'b0;
                ent_mul_op[i]  <= 3'd0;  ent_is_div[i] <= 1'b0;  ent_ls_func3[i]<= 3'd0;  ent_alu_func4[i] <= 4'd0;
                ent_csr_func3[i]<= 3'd0;  ent_csr_wr_en[i]<=1'b0;  ent_csr_armed[i]<=1'b0; ent_csr_wr_act[i]<=1'b0;
                ent_csr_addr[i]<= 12'd0; ent_jmp_val[i] <= 32'd0; ent_aux_addr[i]<= 32'd0;
                ent_br_flag[i] <= 1'b0;  ent_jal_flag[i]<=1'b0;  ent_jalr_flag[i]<=1'b0;
                ent_br_pred_taken[i]<= 1'b0;  ent_exc_illegal[i] <= 1'b0; ent_exc_ecall[i]<=1'b0;
                ent_exc_ebreak[i]<= 1'b0;  ent_exc_irq_ret[i]<=1'b0;  ent_exc_jal_misalign[i]<=1'b0;
                ent_op1[i] <= 32'd0; ent_op2[i] <= 32'd0;
                ent_src1[i]  <= 3'd0;  ent_src2[i]  <= 3'd0;
                ent_prod_gen1[i] <= 1'b0;  ent_prod_gen2[i] <= 1'b0;
            end
        end
        else begin
//---- 5.1 唤醒：把命中那一源的值锁进项、清 w ----
            for (i = 0; i < DEPTH; i = i + 1) begin
                if (wake1[i]) begin
                    ent_wait1[i]  <= 1'b0;
                    ent_op1[i] <= (we_alu & (idx_alu == ent_src1[i]) & (gen_alu == ent_prod_gen1[i])) ? data_alu :
                              (we_mul & (idx_mul == ent_src1[i]) & (gen_mul == ent_prod_gen1[i])) ? data_mul :
                              (we_ld  & (idx_ld  == ent_src1[i]) & (gen_ld  == ent_prod_gen1[i])) ? data_ld  : ent_op1[i];
                end
                if (wake2[i]) begin
                    ent_wait2[i]  <= 1'b0;
                    ent_op2[i] <= (we_alu & (idx_alu == ent_src2[i]) & (gen_alu == ent_prod_gen2[i])) ? data_alu :
                              (we_mul & (idx_mul == ent_src2[i]) & (gen_mul == ent_prod_gen2[i])) ? data_mul :
                              (we_ld  & (idx_ld  == ent_src2[i]) & (gen_ld  == ent_prod_gen2[i])) ? data_ld  : ent_op2[i];
                end
            end
//---- 5.2 出站：清 v ----
            if (issue_v) begin
                ent_valid[sel_slot] <= 1'b0;
                ent_wait1[sel_slot]<= 1'b0;
                ent_wait2[sel_slot]<= 1'b0;
            end
//---- 5.3 入站填充（放最后 ⇒ 同格"腾位+填新"以入站为准；出站那拍输出仍由 Q 驱动单元）----
            if (push_en) begin
                ent_valid[free_slot]    <= 1'b1;
                ent_class[free_slot]  <= class_now;
                ent_idx[free_slot]    <= hold_idx;
                ent_gen[free_slot]    <= hold_gen;
                ent_we[free_slot]   <= we_now;
                ent_rd[free_slot]   <= rd_now;
                ent_is_store[free_slot]   <= is_st_now;
                ent_mul_op[free_slot]   <= mulop_now;
                ent_is_div[free_slot]   <= mulop_now[2];
                ent_ls_func3[free_slot]  <= lsf3_now;
                ent_alu_func4[free_slot]   <= func4_now;
                ent_csr_func3[free_slot] <= csrf3_now;
                ent_csr_wr_en[free_slot] <= csrw_en_now;
                ent_csr_armed[free_slot] <= csrw_armed_now;
                ent_csr_wr_act[free_slot] <= csrw_wr_now;
                ent_csr_addr[free_slot] <= csra_now;
                ent_jmp_val[free_slot]   <= jval_now;
                ent_aux_addr[free_slot]  <= hold_aux;
                ent_br_flag[free_slot]  <= brf_now;
                ent_jal_flag[free_slot] <= jalf_now;
                ent_jalr_flag[free_slot] <= jalfr_now;
                ent_br_pred_taken[free_slot] <= hold_br_pred_taken;
                ent_exc_illegal[free_slot]  <= exc_illegal_now;
                ent_exc_ecall[free_slot] <= exc_ecall_now;
                ent_exc_ebreak[free_slot] <= exc_ebreak_now;
                ent_exc_irq_ret[free_slot] <= exc_irq_ret_now;
                ent_exc_jal_misalign[free_slot] <= exc_jmis_now;
                ent_op1[free_slot]  <= cap_op1;
                ent_op2[free_slot]  <= cap_op2;
                ent_wait1[free_slot]   <= w1_now;
                ent_wait2[free_slot]   <= w2_now;
                ent_src1[free_slot]   <= hold_src1;
                ent_src2[free_slot]   <= hold_src2;
                ent_prod_gen1[free_slot]  <= hold_prod_gen1;
                ent_prod_gen2[free_slot]  <= hold_prod_gen2;
            end
//---- 5.4 冲刷：与 rob 同口径（部分冲按年龄、全冲/trap 整站清）----
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
                end
            end
//---- 5.5 bju 在途移位 ----
            if (flush_w) bju_shift <= 2'd0;
            else         bju_shift <= {bju_shift[0], (issue_v & (ent_class[sel_slot] == CLS_BJU))};
//---- 5.6 计数 ----
            if (flush_w | trap_fire) pending_cnt <= 3'd0;
            else                     pending_cnt <= pending_cnt + (pending_inc ? 3'd1 : 3'd0)
                                                        - (pending_dec ? 3'd1 : 3'd0);
        end
    end

    reg [3:0] occupancy_calc;
    always @(*) begin
        occupancy_calc = occupancy;
        if (alloc_en && have_free_slot) occupancy_calc = occupancy_calc + 4'd1;
        if (issue_v)           occupancy_calc = occupancy_calc - 4'd1;
        if (trap_fire | (flush_con_rob & ~rob_flush_part)) occupancy_calc = 4'd0;
        else if (flush_con_rob & rob_flush_part) begin
            for (i = 0; i < DEPTH; i = i + 1)
                if (ent_valid[i] && (flush_incl ? (ent_age[i] >= flush_part_age)
                                                : (ent_age[i] >  flush_part_age))) occupancy_calc = occupancy_calc - 4'd1;
        end
    end
    always @(*) occupancy = {3'd0,ent_valid[0]} + {3'd0,ent_valid[1]} + {3'd0,ent_valid[2]}
                          + {3'd0,ent_valid[3]} + {3'd0,ent_valid[4]} + {3'd0,ent_valid[5]};

endmodule
