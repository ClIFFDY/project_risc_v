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
//     - 载荷面（rd/func10/imm）与全译码解在 S 级的 inst 上，给 idu2 的入站；
//     - rob 分配口只判"这一类写不写 rd"，解在 **fifo 队头**（nh0）上 —— 入队发生在
//       "队头进 出队格"那一拍，rob 要的是那一条的 rd，不能裸取 inst[11:7]
//       （BRANCH/STORE 会变出幽灵写者）。
//
//   ★ 扫槽口（scan_rs）给的是【本级输入那一级】（F）的 rs：rob 的读拍在本级输出（S），
//     扫描必须提前一拍（见 rob.v 的相位说明）。
//////////////////////////////////////////////////////////////////////////////////


module idu1(
    input clk, rst,
    input [7:0] flag_bus,
    input [1:0]  st_pop,
    input [3:0]  st_flush_pop,
    output       credit_stall_o,
//来自 iffu 出队格（F 级）
    input [31:0] inst_in,
    input [31:0] addr_in,
    input        br_pred_in,
    input [31:0] jalr_pred_in,
    input [4:0]  r1_in, r2_in,
    input [3:0]  idx_in,
    input        gen_in,
//载荷那条的就绪（rob 按冻结生产者槽现算）：本来只在 post_decoder 门 payload_go，
//但"原地等"必须【整组一起停】—— 否则本级照推会把 S 级内容原地覆盖掉、那条永远进不了载荷。
    input        rdy1_in, rdy2_in,
    input [31:0] inst_in1,
    input [31:0] addr_in1,
    input        br_pred_in1,
    input [31:0] jalr_pred_in1,
    input [4:0]  r1_in1, r2_in1,
    input [3:0]  idx_in1,
    input        gen_in1,
    input        rdy1_in1, rdy2_in1,
//给 fifo 的推进条件（与出队格的"换新"同门）
    output reg   advance,
//给 post_decoder 的载荷（原 mid_decoder 的输出集合 + 随行号）
    output reg [31:0] inst_out,
    output reg [31:0] aux_addr_out,
    output reg        br_pred_taken_out,
    output reg [4:0]  r1_out, r2_out,
    output reg [3:0]  idx_out,
    output reg        gen_out,
    output reg        pay_v_out,
//rob 分配口（在队头上按 opcode 分类解）
    output reg        alloc_en0,
    output reg        alloc_we,
    output reg        alloc_jmp,
//这一条会不会报完成口（给 rob 的"发出即置可退"用）：M 类（funct7==1 的 OP）与 LOAD。
//它们即使不写 rd（rd=x0），单元照样会报一笔完成口 ⇒ 不能当"没人会回报"提前置可退；
//否则它立刻退休、槽被回收，而它的完成口（除法 30+ 拍）晚回来时会撞上槽的新住户。
    output reg        alloc_pend,
    output reg [4:0]  alloc_rd,
//全译码（随 S 级寄存器送 idu2 的入站：分类 / 各单元字段 / 异常标记 / 非寄存器操作数覆盖）
    output reg [1:0]  dec_class,
    output reg        dec_we,
    output reg [4:0]  dec_rd,
    output reg        dec_is_st,
    output reg [2:0]  dec_mulop,
    output reg [2:0]  dec_lsf3,
    output reg [3:0]  dec_func4,
    output reg [2:0]  dec_csrf3,
    output reg        dec_csrw_en,
    output reg        dec_csrw_armed,
    output reg        dec_csrw_wr,
    output reg [11:0] dec_csra,
    output reg [31:0] dec_jval,
    output reg        dec_brf,
    output reg        dec_jalf,
    output reg        dec_jalfr,
    output reg        dec_exc_illegal,
    output reg        dec_exc_jmis,
    output reg        dec_exc_ecall,
    output reg        dec_exc_ebreak,
    output reg        dec_exc_irq_ret,
    output reg        dec_r1_imm,
    output reg        dec_r2_imm,
    output reg [31:0] dec_r1_immv,
    output reg [31:0] dec_r2_immv,
    output reg [31:0] inst_out1,
    output reg [31:0] aux_addr_out1,
    output reg        br_pred_taken_out1,
    output reg [4:0]  r1_out1, r2_out1,
    output reg [3:0]  idx_out1,
    output reg        gen_out1,
    output reg        pay_v_out1,
    output reg        advance1,
    output reg        alloc_en1,
    output reg        alloc_we1,
    output reg        alloc_jmp1,
    output reg        alloc_pend1,
    output reg [4:0]  alloc_rd1,
    output reg [1:0]  dec_class1,
    output reg        dec_we1,
    output reg [4:0]  dec_rd1,
    output reg        dec_is_st1,
    output reg [2:0]  dec_mulop1,
    output reg [2:0]  dec_lsf31,
    output reg [3:0]  dec_func41,
    output reg [2:0]  dec_csrf31,
    output reg        dec_csrw_en1,
    output reg        dec_csrw_armed1,
    output reg        dec_csrw_wr1,
    output reg [11:0] dec_csra1,
    output reg [31:0] dec_jval1,
    output reg        dec_brf1,
    output reg        dec_jalf1,
    output reg        dec_jalfr1,
    output reg        dec_exc_illegal1,
    output reg        dec_exc_jmis1,
    output reg        dec_exc_ecall1,
    output reg        dec_exc_ebreak1,
    output reg        dec_exc_irq_ret1,
    output reg        dec_r1_imm1,
    output reg        dec_r2_imm1,
    output reg [31:0] dec_r1_immv1,
    output reg [31:0] dec_r2_immv1
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，工具只能在布局阶段自己复制。每个模块各自打一拍，
//寄存器就落在本模块旁边；全核都只打一拍，彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

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

//提取不同类型指令立即数的函数块
    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immI;
        input [31:0] inst;
        immI = {{20{inst[31]}}, inst[31:20]};
    endfunction

    function [31:0] immS;
        input [31:0] inst;
        immS = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    endfunction

    function [31:0] immU;
        input [31:0] inst;
        immU = {inst[31:12], 12'd0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
    reg [3:0] credit;
    reg [5:0] credit_nx;
    reg exec, flush_w, stall, credit_ok;
    always @(*) begin
        flush_w = flag_bus[7] | flag_bus[6] | flag_bus[5];
        credit_ok = (credit + {2'd0, st_pop} + {2'd0, st_flush_pop} > {3'd0, pay_v_out1});
        stall = ((flag_bus[3] | flag_bus[2] | flag_bus[0]) & ~flush_w) | ~credit_ok;
        exec    = flag_bus[4];
//★ 保留站改造：`rdy1_in & rdy2_in` 已从本级摘掉 —— 操作数等待由 idu2 的站项承担，
//  S 级不再为它停（否则"原地等"还在，站的解耦白做）。入站就绪只用来决定"锁值还是锁标签"。
        advance     = exec & ~flush_w & ~stall;
    end

    always @(*) credit_nx = {2'd0, credit} - {5'd0, advance} - {5'd0, advance1}
                          + {4'd0, st_pop} + {2'd0, st_flush_pop};
    always @(posedge clk) begin
        if (rst_q) credit <= 4'd8;
        else if (credit_nx[5]) credit <= 4'd0;
        else if (credit_nx > 6'd8) credit <= 4'd8;
        else credit <= credit_nx[3:0];
    end
    assign credit_stall_o = ~credit_ok;

//译码字段的组合解算：源是本级的输入（F 级的指令字），口径与原来那一段逐条一致。
    reg [4:0]  rd_dec;
    reg [9:0]  func10_dec;
    reg [31:0] imm_dec;
    reg        jmp_dec;
    always @(*) begin
        rd_dec = 5'd0;
        func10_dec = 10'd0;
        imm_dec = 32'd0;
        jmp_dec = 1'b0;
        case (inst_in[6:0])
            OPCODE_OP: begin
                rd_dec = inst_in[11:7];
                func10_dec = {inst_in[31:25], inst_in[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_dec = immI(inst_in);
                rd_dec = inst_in[11:7];
                func10_dec = {(inst_in[14:12] == 3'b001 || inst_in[14:12] == 3'b101) ? inst_in[31:25] : 7'd0,
                            inst_in[14:12]};
            end
            OPCODE_JAL: begin
                rd_dec = inst_in[11:7];
                jmp_dec = 1'b1;
            end
            OPCODE_JALR: begin
                rd_dec = inst_in[11:7];
                imm_dec = immI(inst_in);
                jmp_dec = 1'b1;
            end
            OPCODE_BRANCH: begin
                imm_dec = immB(inst_in);
                func10_dec = {7'd0, inst_in[14:12]};
                jmp_dec = 1'b1;
            end
            OPCODE_LOAD: begin
                imm_dec = immI(inst_in);
                rd_dec = inst_in[11:7];
            end
            OPCODE_STORE: begin
                imm_dec = immS(inst_in);
            end
            OPCODE_LUI: begin
                imm_dec = immU(inst_in);
                rd_dec = inst_in[11:7];
            end
            OPCODE_AUIPC: begin
                imm_dec = immU(inst_in) - 4'd4;
                rd_dec = inst_in[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_dec = inst_in[11:7];
                func10_dec = {7'd0, inst_in[14:12]};
            end
        endcase
    end

    reg csr_is_access_dec, csr_exists_dec, csr_ro_dec, csr_wr_dec;
    always @(*) begin
        csr_is_access_dec = (inst_in[14:12] != 3'b000) && (inst_in[14:12] != 3'b100);
//★ "写"的规范口径：CSRRW/CSRRWI 恒写；CSRRS/CSRRC/CSRRSI/CSRRCI 只有源（rs1 或 uimm）非 0 才写。
//  这一根同时决定"真写使能"和"对只读 CSR 的写要不要报非法"，两者是同一个判据。
        csr_wr_dec        = (inst_in[14:12] == 3'b001) | (inst_in[14:12] == 3'b101) |
                            (inst_in[19:15] != 5'd0);
        csr_ro_dec        = (inst_in[31:30] == 2'b11);
        csr_exists_dec    = 1'b0;
        case (inst_in[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec = 1'b1;
            default: csr_exists_dec = 1'b0;
        endcase
    end

    reg exc_illegal_dec, exc_jmis_dec;
    always @(*) begin
        exc_illegal_dec = 1'b0;
        exc_jmis_dec    = (inst_in[6:0] == OPCODE_JAL) & inst_in[21];
        if (inst_in != 32'd0) begin
            case (inst_in[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec = 1'b0;
                OPCODE_LOAD: begin
                    if (inst_in[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b110) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b111) exc_illegal_dec = 1'b1;
                end
                OPCODE_STORE: begin
                    if (inst_in[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b100) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b101) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b110) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b111) exc_illegal_dec = 1'b1;
                end
                OPCODE_JALR: begin
                    if (inst_in[14:12] != 3'b000) exc_illegal_dec = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (inst_in[14:12] == 3'b010) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (inst_in[14:12] == 3'b001) begin
                        if (inst_in[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                    if (inst_in[14:12] == 3'b101) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (inst_in[31:25] == 7'b0000001) exc_illegal_dec = 1'b0;
                    else if (inst_in[14:12] == 3'b000) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                    else if (inst_in[14:12] == 3'b001) begin
                        if (inst_in[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                    else if (inst_in[14:12] == 3'b101) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                    else begin
                        if (inst_in[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (inst_in[14:12] == 3'b010) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (inst_in[14:12] == 3'b100) exc_illegal_dec = 1'b1;
                    if (inst_in[14:12] == 3'b000) begin
                        case (inst_in[31:20])
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
    reg [1:0]  class_dec;
    reg        we_dec, csrw_en_dec, csrw_armed_dec, csrw_wr_dec;
    reg [4:0]  rd_pick_dec;
    reg        is_st_dec;
    reg [2:0]  mulop_dec, lsf3_dec, csrf3_dec;
    reg [3:0]  func4_dec;
    reg [11:0] csra_dec;
    reg [31:0] jval_dec;
    reg        brf_dec, jalf_dec, jalfr_dec;

    always @(*) begin
        class_dec      = CLS_ALU;
        we_dec         = 1'b0;
        csrw_en_dec    = 1'b0;
        csrw_armed_dec = 1'b0;
        csrw_wr_dec    = 1'b0;
        rd_pick_dec    = 5'd0;
        is_st_dec      = 1'b0;
        mulop_dec      = 3'd0;
        lsf3_dec       = 3'd0;
        csrf3_dec      = 3'd0;
        func4_dec      = 4'd0;
        csra_dec       = 12'd0;
        jval_dec       = 32'd0;
        brf_dec        = 1'b0;
        jalf_dec       = 1'b0;
        jalfr_dec      = 1'b0;

        case (inst_in[6:0])
            OPCODE_OP: begin
                if (inst_in[31:25] == 7'b0000001) begin
                    class_dec = CLS_MUL;
                    mulop_dec = inst_in[14:12];
                end
                else begin
                    func4_dec   = {func10_dec[8], func10_dec[2:0]};
                    rd_pick_dec = rd_dec;
                    we_dec      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                func4_dec   = {func10_dec[8], func10_dec[2:0]};
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_JAL: begin
                class_dec   = CLS_BJU;
                jalf_dec    = 1'b1;
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_JALR: begin
                class_dec   = CLS_BJU;
                jalfr_dec   = 1'b1;
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
                jval_dec    = jalr_pred_in;
            end
            OPCODE_BRANCH: begin
                class_dec   = CLS_BJU;
                brf_dec     = 1'b1;
                func4_dec   = {1'b0, inst_in[14:12]};
                jval_dec    = imm_dec - 32'd4;
            end
            OPCODE_LUI: begin
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_AUIPC: begin
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_dec   = CLS_LSU;
                lsf3_dec    = inst_in[14:12];
                rd_pick_dec = rd_dec;
                jval_dec    = imm_dec;
            end
            OPCODE_STORE: begin
                class_dec   = CLS_LSU;
                is_st_dec   = 1'b1;
                lsf3_dec    = inst_in[14:12];
                jval_dec    = imm_dec;
            end
            OPCODE_SYSTEM: begin
                class_dec   = CLS_BJU;
                csrf3_dec   = inst_in[14:12];
                rd_pick_dec = rd_dec;
                case (inst_in[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        we_dec         = 1'b1;
                        csrw_en_dec    = 1'b1;
                        csrw_armed_dec = 1'b1;
                        csrw_wr_dec    = csr_wr_dec;
                    end
                    default: begin
                        we_dec      = 1'b0;
                        csrw_en_dec = 1'b0;
                    end
                endcase
//★ csr 地址【不写也照给】：地址供 csr.v 的读口用，写不写只决定"锁不锁"。
//  `csrr rd, csr` 若被清成地址 0 就会读回 0（实测：启动宏读 mtvec 读回 0 ⇒ 误判没写进去）。
                csra_dec = (inst_in[14:12] != 3'b000) ? inst_in[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec | exc_jmis_dec) begin
            we_dec         = 1'b0;
            csrw_en_dec    = 1'b0;
            csrw_armed_dec = 1'b0;
            csrw_wr_dec    = 1'b0;
        end
    end

//非寄存器操作数的覆盖（立即数在这里并进操作数）
    reg r1_imm_dec, r2_imm_dec;
    reg [31:0] r1_immv_dec, r2_immv_dec;
    always @(*) begin
        r1_imm_dec  = 1'b0;
        r2_imm_dec  = 1'b0;
        r1_immv_dec = 32'd0;
        r2_immv_dec = 32'd0;
        case (inst_in[6:0])
            OPCODE_OP_IMM: begin r2_imm_dec = 1'b1; r2_immv_dec = imm_dec; end
            OPCODE_JALR:   begin r2_imm_dec = 1'b1; r2_immv_dec = imm_dec; end
            OPCODE_LUI:    begin r2_imm_dec = 1'b1; r2_immv_dec = imm_dec; end
            OPCODE_AUIPC:  begin
                r1_imm_dec  = 1'b1; r1_immv_dec = addr_in;
                r2_imm_dec  = 1'b1; r2_immv_dec = imm_dec;
            end
            OPCODE_SYSTEM: begin
                if ((inst_in[14:12] == 3'b101) | (inst_in[14:12] == 3'b110) | (inst_in[14:12] == 3'b111)) begin
                    r1_imm_dec  = 1'b1;
                    r1_immv_dec = {27'd0, inst_in[19:15]};
                end
            end
            default: ;
        endcase
    end

//异常标记（ecall/ebreak/mret）
    reg exc_ecall_dec, exc_ebreak_dec, exc_irq_ret_dec;
    always @(*) begin
        exc_ecall_dec   = (inst_in[6:0] == OPCODE_SYSTEM) && (inst_in[31:20] == 12'h000);
        exc_ebreak_dec  = (inst_in[6:0] == OPCODE_SYSTEM) && (inst_in[31:20] == 12'h001);
        exc_irq_ret_dec = (inst_in[6:0] == OPCODE_SYSTEM) && (inst_in[14:12] == 3'b000)
                        && (inst_in[31:20] == 12'h302);
    end

//rob 分配口：分配发生在"出队格 h0 换新"那一拍，吃的就是本级的译码输入（= h0 的 Q 端），
//与 idu1 自己的译码同源 —— 不再另解一份队头。rd 直接复用主译码的 rd_dec（按 opcode 分类
//填过，不裸取 inst[11:7]）。
//★ 发号拍 = 本级每一个推进拍（不按出队格有效与否区分）：空泡也要占一项 ——
//  载荷的 rob 号与“冻结的生产者槽”都按项索引取，空泡没有项就会取到别人那一项。
    always @(*) begin
        alloc_en0  = advance;
        alloc_rd   = rd_dec;
        alloc_we   = (rd_dec != 5'd0);
        alloc_jmp  = jmp_dec;
        alloc_pend = (inst_in[6:0] == OPCODE_LOAD) |
                     ((inst_in[6:0] == OPCODE_OP) && (inst_in[31:25] == 7'b0000001));
    end

//lane1 的译码（与 lane0 一模，源换成 inst_in1 / addr_in1 / jalr_pred_in1）
    reg [4:0]  rd_dec1;
    reg [9:0]  func10_dec1;
    reg [31:0] imm_dec1;
    reg        jmp_dec1;
    always @(*) begin
        rd_dec1 = 5'd0;
        func10_dec1 = 10'd0;
        imm_dec1 = 32'd0;
        jmp_dec1 = 1'b0;
        case (inst_in1[6:0])
            OPCODE_OP: begin
                rd_dec1 = inst_in1[11:7];
                func10_dec1 = {inst_in1[31:25], inst_in1[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_dec1 = immI(inst_in1);
                rd_dec1 = inst_in1[11:7];
                func10_dec1 = {(inst_in1[14:12] == 3'b001 || inst_in1[14:12] == 3'b101) ? inst_in1[31:25] : 7'd0,
                            inst_in1[14:12]};
            end
            OPCODE_JAL: begin
                rd_dec1 = inst_in1[11:7];
                jmp_dec1 = 1'b1;
            end
            OPCODE_JALR: begin
                rd_dec1 = inst_in1[11:7];
                imm_dec1 = immI(inst_in1);
                jmp_dec1 = 1'b1;
            end
            OPCODE_BRANCH: begin
                imm_dec1 = immB(inst_in1);
                func10_dec1 = {7'd0, inst_in1[14:12]};
                jmp_dec1 = 1'b1;
            end
            OPCODE_LOAD: begin
                imm_dec1 = immI(inst_in1);
                rd_dec1 = inst_in1[11:7];
            end
            OPCODE_STORE: begin
                imm_dec1 = immS(inst_in1);
            end
            OPCODE_LUI: begin
                imm_dec1 = immU(inst_in1);
                rd_dec1 = inst_in1[11:7];
            end
            OPCODE_AUIPC: begin
                imm_dec1 = immU(inst_in1) - 4'd4;
                rd_dec1 = inst_in1[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_dec1 = inst_in1[11:7];
                func10_dec1 = {7'd0, inst_in1[14:12]};
            end
        endcase
    end

    reg csr_is_access_dec1, csr_exists_dec1, csr_ro_dec1, csr_wr_dec1;
    always @(*) begin
        csr_is_access_dec1 = (inst_in1[14:12] != 3'b000) && (inst_in1[14:12] != 3'b100);
        csr_wr_dec1        = (inst_in1[14:12] == 3'b001) | (inst_in1[14:12] == 3'b101) |
                            (inst_in1[19:15] != 5'd0);
        csr_ro_dec1        = (inst_in1[31:30] == 2'b11);
        csr_exists_dec1    = 1'b0;
        case (inst_in1[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec1 = 1'b1;
            default: csr_exists_dec1 = 1'b0;
        endcase
    end

    reg exc_illegal_dec1, exc_jmis_dec1;
    always @(*) begin
        exc_illegal_dec1 = 1'b0;
        exc_jmis_dec1    = (inst_in1[6:0] == OPCODE_JAL) & inst_in1[21];
        if (inst_in1 != 32'd0) begin
            case (inst_in1[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec1 = 1'b0;
                OPCODE_LOAD: begin
                    if (inst_in1[14:12] == 3'b011) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b110) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b111) exc_illegal_dec1 = 1'b1;
                end
                OPCODE_STORE: begin
                    if (inst_in1[14:12] == 3'b011) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b100) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b101) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b110) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b111) exc_illegal_dec1 = 1'b1;
                end
                OPCODE_JALR: begin
                    if (inst_in1[14:12] != 3'b000) exc_illegal_dec1 = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (inst_in1[14:12] == 3'b010) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b011) exc_illegal_dec1 = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (inst_in1[14:12] == 3'b001) begin
                        if (inst_in1[31:25] != 7'd0) exc_illegal_dec1 = 1'b1;
                    end
                    if (inst_in1[14:12] == 3'b101) begin
                        if ((inst_in1[31:25] != 7'd0) && (inst_in1[31:25] != 7'b0100000))
                            exc_illegal_dec1 = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (inst_in1[31:25] == 7'b0000001) exc_illegal_dec1 = 1'b0;
                    else if (inst_in1[14:12] == 3'b000) begin
                        if ((inst_in1[31:25] != 7'd0) && (inst_in1[31:25] != 7'b0100000))
                            exc_illegal_dec1 = 1'b1;
                    end
                    else if (inst_in1[14:12] == 3'b001) begin
                        if (inst_in1[31:25] != 7'd0) exc_illegal_dec1 = 1'b1;
                    end
                    else if (inst_in1[14:12] == 3'b101) begin
                        if ((inst_in1[31:25] != 7'd0) && (inst_in1[31:25] != 7'b0100000))
                            exc_illegal_dec1 = 1'b1;
                    end
                    else begin
                        if (inst_in1[31:25] != 7'd0) exc_illegal_dec1 = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (inst_in1[14:12] == 3'b010) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b011) exc_illegal_dec1 = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (inst_in1[14:12] == 3'b100) exc_illegal_dec1 = 1'b1;
                    if (inst_in1[14:12] == 3'b000) begin
                        case (inst_in1[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;
                            default: exc_illegal_dec1 = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec1) begin
                        if (!csr_exists_dec1) exc_illegal_dec1 = 1'b1;
                        if (csr_ro_dec1) begin
                            if (csr_wr_dec1) exc_illegal_dec1 = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec1 = 1'b1;
            endcase
        end
    end

    reg [1:0]  class_dec1;
    reg        we_dec1, csrw_en_dec1, csrw_armed_dec1, csrw_wr_dec1;
    reg [4:0]  rd_pick_dec1;
    reg        is_st_dec1;
    reg [2:0]  mulop_dec1, lsf3_dec1, csrf3_dec1;
    reg [3:0]  func4_dec1;
    reg [11:0] csra_dec1;
    reg [31:0] jval_dec1;
    reg        brf_dec1, jalf_dec1, jalfr_dec1;
    always @(*) begin
        class_dec1      = CLS_ALU;
        we_dec1         = 1'b0;
        csrw_en_dec1    = 1'b0;
        csrw_armed_dec1 = 1'b0;
        csrw_wr_dec1    = 1'b0;
        rd_pick_dec1    = 5'd0;
        is_st_dec1      = 1'b0;
        mulop_dec1      = 3'd0;
        lsf3_dec1       = 3'd0;
        csrf3_dec1      = 3'd0;
        func4_dec1      = 4'd0;
        csra_dec1       = 12'd0;
        jval_dec1       = 32'd0;
        brf_dec1        = 1'b0;
        jalf_dec1       = 1'b0;
        jalfr_dec1      = 1'b0;
        case (inst_in1[6:0])
            OPCODE_OP: begin
                if (inst_in1[31:25] == 7'b0000001) begin
                    class_dec1 = CLS_MUL;
                    mulop_dec1 = inst_in1[14:12];
                end
                else begin
                    func4_dec1   = {func10_dec1[8], func10_dec1[2:0]};
                    rd_pick_dec1 = rd_dec1;
                    we_dec1      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                func4_dec1   = {func10_dec1[8], func10_dec1[2:0]};
                rd_pick_dec1 = rd_dec1;
                we_dec1      = 1'b1;
            end
            OPCODE_JAL: begin
                class_dec1   = CLS_BJU;
                jalf_dec1    = 1'b1;
                rd_pick_dec1 = rd_dec1;
                we_dec1      = 1'b1;
            end
            OPCODE_JALR: begin
                class_dec1   = CLS_BJU;
                jalfr_dec1   = 1'b1;
                rd_pick_dec1 = rd_dec1;
                we_dec1      = 1'b1;
                jval_dec1    = jalr_pred_in1;
            end
            OPCODE_BRANCH: begin
                class_dec1   = CLS_BJU;
                brf_dec1     = 1'b1;
                func4_dec1   = {1'b0, inst_in1[14:12]};
                jval_dec1    = imm_dec1 - 32'd4;
            end
            OPCODE_LUI: begin
                rd_pick_dec1 = rd_dec1;
                we_dec1      = 1'b1;
            end
            OPCODE_AUIPC: begin
                rd_pick_dec1 = rd_dec1;
                we_dec1      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_dec1   = CLS_LSU;
                lsf3_dec1    = inst_in1[14:12];
                rd_pick_dec1 = rd_dec1;
                jval_dec1    = imm_dec1;
            end
            OPCODE_STORE: begin
                class_dec1   = CLS_LSU;
                is_st_dec1   = 1'b1;
                lsf3_dec1    = inst_in1[14:12];
                jval_dec1    = imm_dec1;
            end
            OPCODE_SYSTEM: begin
                class_dec1   = CLS_BJU;
                csrf3_dec1   = inst_in1[14:12];
                rd_pick_dec1 = rd_dec1;
                case (inst_in1[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        we_dec1         = 1'b1;
                        csrw_en_dec1    = 1'b1;
                        csrw_armed_dec1 = 1'b1;
                        csrw_wr_dec1    = csr_wr_dec1;
                    end
                    default: begin
                        we_dec1      = 1'b0;
                        csrw_en_dec1 = 1'b0;
                    end
                endcase
                csra_dec1 = (inst_in1[14:12] != 3'b000) ? inst_in1[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec1 | exc_jmis_dec1) begin
            we_dec1         = 1'b0;
            csrw_en_dec1    = 1'b0;
            csrw_armed_dec1 = 1'b0;
            csrw_wr_dec1    = 1'b0;
        end
    end

    reg r1_imm_dec1, r2_imm_dec1;
    reg [31:0] r1_immv_dec1, r2_immv_dec1;
    always @(*) begin
        r1_imm_dec1  = 1'b0;
        r2_imm_dec1  = 1'b0;
        r1_immv_dec1 = 32'd0;
        r2_immv_dec1 = 32'd0;
        case (inst_in1[6:0])
            OPCODE_OP_IMM: begin r2_imm_dec1 = 1'b1; r2_immv_dec1 = imm_dec1; end
            OPCODE_JALR:   begin r2_imm_dec1 = 1'b1; r2_immv_dec1 = imm_dec1; end
            OPCODE_LUI:    begin r2_imm_dec1 = 1'b1; r2_immv_dec1 = imm_dec1; end
            OPCODE_AUIPC:  begin
                r1_imm_dec1  = 1'b1; r1_immv_dec1 = addr_in1;
                r2_imm_dec1  = 1'b1; r2_immv_dec1 = imm_dec1;
            end
            OPCODE_SYSTEM: begin
                if ((inst_in1[14:12] == 3'b101) | (inst_in1[14:12] == 3'b110) | (inst_in1[14:12] == 3'b111)) begin
                    r1_imm_dec1  = 1'b1;
                    r1_immv_dec1 = {27'd0, inst_in1[19:15]};
                end
            end
            default: ;
        endcase
    end

    reg exc_ecall_dec1, exc_ebreak_dec1, exc_irq_ret_dec1;
    always @(*) begin
        exc_ecall_dec1   = (inst_in1[6:0] == OPCODE_SYSTEM) && (inst_in1[31:20] == 12'h000);
        exc_ebreak_dec1  = (inst_in1[6:0] == OPCODE_SYSTEM) && (inst_in1[31:20] == 12'h001);
        exc_irq_ret_dec1 = (inst_in1[6:0] == OPCODE_SYSTEM) && (inst_in1[14:12] == 3'b000)
                        && (inst_in1[31:20] == 12'h302);
    end

    always @(*) begin
        alloc_en1   = advance & (inst_in1 != 32'd0);
        alloc_rd1   = rd_dec1;
        alloc_we1   = (rd_dec1 != 5'd0);
        alloc_jmp1  = jmp_dec1;
        alloc_pend1 = (inst_in1[6:0] == OPCODE_LOAD) |
                     ((inst_in1[6:0] == OPCODE_OP) && (inst_in1[31:25] == 7'b0000001));
    end

//推进/保持/清：与 fifo 出队格的"换新/保持/清"【逐字同形】。
//★ 条件必须与上一级一致：本级的"保持"要跟上一级的"保持"落在同一拍，差一拍就是整条流水线错位。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out <= 32'd0;
            aux_addr_out <= 32'd0;
            br_pred_taken_out <= 1'b0;
            r1_out <= 5'd0;
            r2_out <= 5'd0;
            idx_out <= 3'd0;
            gen_out <= 1'b0;
            pay_v_out <= 1'b0;
            dec_class <= 2'd0;
            dec_we <= 1'b0;
            dec_rd <= 5'd0;
            dec_is_st <= 1'b0;
            dec_mulop <= 3'd0;
            dec_lsf3 <= 3'd0;
            dec_func4 <= 4'd0;
            dec_csrf3 <= 3'd0;
            dec_csrw_en <= 1'b0;
            dec_csrw_armed <= 1'b0;
            dec_csrw_wr <= 1'b0;
            dec_csra <= 12'd0;
            dec_jval <= 32'd0;
            dec_brf <= 1'b0;
            dec_jalf <= 1'b0;
            dec_jalfr <= 1'b0;
            dec_exc_illegal <= 1'b0;
            dec_exc_jmis <= 1'b0;
            dec_exc_ecall <= 1'b0;
            dec_exc_ebreak <= 1'b0;
            dec_exc_irq_ret <= 1'b0;
            dec_r1_imm <= 1'b0;
            dec_r2_imm <= 1'b0;
            dec_r1_immv <= 32'd0;
            dec_r2_immv <= 32'd0;
        end
        else if (exec) begin
            if (advance) begin
                inst_out <= inst_in;
                aux_addr_out <= addr_in;
                br_pred_taken_out <= br_pred_in;
                r1_out <= r1_in;
                r2_out <= r2_in;
                idx_out <= idx_in;
                gen_out <= gen_in;
                pay_v_out <= (addr_in != 32'd0);
                dec_class <= class_dec;
                dec_we <= we_dec;
                dec_rd <= rd_pick_dec;
                dec_is_st <= is_st_dec;
                dec_mulop <= mulop_dec;
                dec_lsf3 <= lsf3_dec;
                dec_func4 <= func4_dec;
                dec_csrf3 <= csrf3_dec;
                dec_csrw_en <= csrw_en_dec;
                dec_csrw_armed <= csrw_armed_dec;
                dec_csrw_wr <= csrw_wr_dec;
                dec_csra <= csra_dec;
                dec_jval <= jval_dec;
                dec_brf <= brf_dec;
                dec_jalf <= jalf_dec;
                dec_jalfr <= jalfr_dec;
                dec_exc_illegal <= exc_illegal_dec;
                dec_exc_jmis <= exc_jmis_dec;
                dec_exc_ecall <= exc_ecall_dec;
                dec_exc_ebreak <= exc_ebreak_dec;
                dec_exc_irq_ret <= exc_irq_ret_dec;
                dec_r1_imm <= r1_imm_dec;
                dec_r2_imm <= r2_imm_dec;
                dec_r1_immv <= r1_immv_dec;
                dec_r2_immv <= r2_immv_dec;
            end
//★ 保持判据必须与 advance 逐项同门：advance 比 stall 多出 rdy 那一项，不把它补进这里，
//  暂停那一拍就掉进下面的"清"分支 —— S 级被清零，那条指令永远进不了载荷。
            else if ((stall | ~rdy1_in | ~rdy2_in) & ~flush_w) begin
                inst_out <= inst_out;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                r1_out <= r1_out;
                r2_out <= r2_out;
                idx_out <= idx_out;
                gen_out <= gen_out;
                pay_v_out <= pay_v_out;
                dec_class <= dec_class;
                dec_we <= dec_we;
                dec_rd <= dec_rd;
                dec_is_st <= dec_is_st;
                dec_mulop <= dec_mulop;
                dec_lsf3 <= dec_lsf3;
                dec_func4 <= dec_func4;
                dec_csrf3 <= dec_csrf3;
                dec_csrw_en <= dec_csrw_en;
                dec_csrw_armed <= dec_csrw_armed;
                dec_csrw_wr <= dec_csrw_wr;
                dec_csra <= dec_csra;
                dec_jval <= dec_jval;
                dec_brf <= dec_brf;
                dec_jalf <= dec_jalf;
                dec_jalfr <= dec_jalfr;
                dec_exc_illegal <= dec_exc_illegal;
                dec_exc_jmis <= dec_exc_jmis;
                dec_exc_ecall <= dec_exc_ecall;
                dec_exc_ebreak <= dec_exc_ebreak;
                dec_exc_irq_ret <= dec_exc_irq_ret;
                dec_r1_imm <= dec_r1_imm;
                dec_r2_imm <= dec_r2_imm;
                dec_r1_immv <= dec_r1_immv;
                dec_r2_immv <= dec_r2_immv;
            end
            else begin
                inst_out <= 32'd0;
                aux_addr_out <= 32'd0;
                br_pred_taken_out <= 1'b0;
                r1_out <= 5'd0;
                r2_out <= 5'd0;
                idx_out <= 3'd0;
                gen_out <= 1'b0;
                pay_v_out <= 1'b0;
                dec_class <= 2'd0;
                dec_we <= 1'b0;
                dec_rd <= 5'd0;
                dec_is_st <= 1'b0;
                dec_mulop <= 3'd0;
                dec_lsf3 <= 3'd0;
                dec_func4 <= 4'd0;
                dec_csrf3 <= 3'd0;
                dec_csrw_en <= 1'b0;
                dec_csrw_armed <= 1'b0;
                dec_csrw_wr <= 1'b0;
                dec_csra <= 12'd0;
                dec_jval <= 32'd0;
                dec_brf <= 1'b0;
                dec_jalf <= 1'b0;
                dec_jalfr <= 1'b0;
                dec_exc_illegal <= 1'b0;
                dec_exc_jmis <= 1'b0;
                dec_exc_ecall <= 1'b0;
                dec_exc_ebreak <= 1'b0;
                dec_exc_irq_ret <= 1'b0;
                dec_r1_imm <= 1'b0;
                dec_r2_imm <= 1'b0;
                dec_r1_immv <= 32'd0;
                dec_r2_immv <= 32'd0;
            end
        end
    end

    always @(*) begin
        advance1 = advance & pay_v_out1;
    end

    always @(posedge clk) begin
        if (rst_q) begin
            inst_out1 <= 32'd0;
            aux_addr_out1 <= 32'd0;
            br_pred_taken_out1 <= 1'b0;
            r1_out1 <= 5'd0;
            r2_out1 <= 5'd0;
            idx_out1 <= 3'd0;
            gen_out1 <= 1'b0;
            pay_v_out1 <= 1'b0;
            dec_class1 <= 2'd0;
            dec_we1 <= 1'b0;
            dec_rd1 <= 5'd0;
            dec_is_st1 <= 1'b0;
            dec_mulop1 <= 3'd0;
            dec_lsf31 <= 3'd0;
            dec_func41 <= 4'd0;
            dec_csrf31 <= 3'd0;
            dec_csrw_en1 <= 1'b0;
            dec_csrw_armed1 <= 1'b0;
            dec_csrw_wr1 <= 1'b0;
            dec_csra1 <= 12'd0;
            dec_jval1 <= 32'd0;
            dec_brf1 <= 1'b0;
            dec_jalf1 <= 1'b0;
            dec_jalfr1 <= 1'b0;
            dec_exc_illegal1 <= 1'b0;
            dec_exc_jmis1 <= 1'b0;
            dec_exc_ecall1 <= 1'b0;
            dec_exc_ebreak1 <= 1'b0;
            dec_exc_irq_ret1 <= 1'b0;
            dec_r1_imm1 <= 1'b0;
            dec_r2_imm1 <= 1'b0;
            dec_r1_immv1 <= 32'd0;
            dec_r2_immv1 <= 32'd0;
        end
        else if (exec) begin
            if (advance) begin
                inst_out1 <= inst_in1;
                aux_addr_out1 <= addr_in1;
                br_pred_taken_out1 <= br_pred_in1;
                r1_out1 <= r1_in1;
                r2_out1 <= r2_in1;
                idx_out1 <= idx_in1;
                gen_out1 <= gen_in1;
                pay_v_out1 <= (addr_in1 != 32'd0);
                dec_class1 <= class_dec1;
                dec_we1 <= we_dec1;
                dec_rd1 <= rd_pick_dec1;
                dec_is_st1 <= is_st_dec1;
                dec_mulop1 <= mulop_dec1;
                dec_lsf31 <= lsf3_dec1;
                dec_func41 <= func4_dec1;
                dec_csrf31 <= csrf3_dec1;
                dec_csrw_en1 <= csrw_en_dec1;
                dec_csrw_armed1 <= csrw_armed_dec1;
                dec_csrw_wr1 <= csrw_wr_dec1;
                dec_csra1 <= csra_dec1;
                dec_jval1 <= jval_dec1;
                dec_brf1 <= brf_dec1;
                dec_jalf1 <= jalf_dec1;
                dec_jalfr1 <= jalfr_dec1;
                dec_exc_illegal1 <= exc_illegal_dec1;
                dec_exc_jmis1 <= exc_jmis_dec1;
                dec_exc_ecall1 <= exc_ecall_dec1;
                dec_exc_ebreak1 <= exc_ebreak_dec1;
                dec_exc_irq_ret1 <= exc_irq_ret_dec1;
                dec_r1_imm1 <= r1_imm_dec1;
                dec_r2_imm1 <= r2_imm_dec1;
                dec_r1_immv1 <= r1_immv_dec1;
                dec_r2_immv1 <= r2_immv_dec1;
            end
            else if ((stall | ~rdy1_in1 | ~rdy2_in1) & ~flush_w) begin
                inst_out1 <= inst_out1;
                aux_addr_out1 <= aux_addr_out1;
                br_pred_taken_out1 <= br_pred_taken_out1;
                r1_out1 <= r1_out1;
                r2_out1 <= r2_out1;
                idx_out1 <= idx_out1;
                gen_out1 <= gen_out1;
                pay_v_out1 <= pay_v_out1;
                dec_class1 <= dec_class1;
                dec_we1 <= dec_we1;
                dec_rd1 <= dec_rd1;
                dec_is_st1 <= dec_is_st1;
                dec_mulop1 <= dec_mulop1;
                dec_lsf31 <= dec_lsf31;
                dec_func41 <= dec_func41;
                dec_csrf31 <= dec_csrf31;
                dec_csrw_en1 <= dec_csrw_en1;
                dec_csrw_armed1 <= dec_csrw_armed1;
                dec_csrw_wr1 <= dec_csrw_wr1;
                dec_csra1 <= dec_csra1;
                dec_jval1 <= dec_jval1;
                dec_brf1 <= dec_brf1;
                dec_jalf1 <= dec_jalf1;
                dec_jalfr1 <= dec_jalfr1;
                dec_exc_illegal1 <= dec_exc_illegal1;
                dec_exc_jmis1 <= dec_exc_jmis1;
                dec_exc_ecall1 <= dec_exc_ecall1;
                dec_exc_ebreak1 <= dec_exc_ebreak1;
                dec_exc_irq_ret1 <= dec_exc_irq_ret1;
                dec_r1_imm1 <= dec_r1_imm1;
                dec_r2_imm1 <= dec_r2_imm1;
                dec_r1_immv1 <= dec_r1_immv1;
                dec_r2_immv1 <= dec_r2_immv1;
            end
            else begin
                inst_out1 <= 32'd0;
                aux_addr_out1 <= 32'd0;
                br_pred_taken_out1 <= 1'b0;
                r1_out1 <= 5'd0;
                r2_out1 <= 5'd0;
                idx_out1 <= 3'd0;
                gen_out1 <= 1'b0;
                pay_v_out1 <= 1'b0;
                dec_class1 <= 2'd0;
                dec_we1 <= 1'b0;
                dec_rd1 <= 5'd0;
                dec_is_st1 <= 1'b0;
                dec_mulop1 <= 3'd0;
                dec_lsf31 <= 3'd0;
                dec_func41 <= 4'd0;
                dec_csrf31 <= 3'd0;
                dec_csrw_en1 <= 1'b0;
                dec_csrw_armed1 <= 1'b0;
                dec_csrw_wr1 <= 1'b0;
                dec_csra1 <= 12'd0;
                dec_jval1 <= 32'd0;
                dec_brf1 <= 1'b0;
                dec_jalf1 <= 1'b0;
                dec_jalfr1 <= 1'b0;
                dec_exc_illegal1 <= 1'b0;
                dec_exc_jmis1 <= 1'b0;
                dec_exc_ecall1 <= 1'b0;
                dec_exc_ebreak1 <= 1'b0;
                dec_exc_irq_ret1 <= 1'b0;
                dec_r1_imm1 <= 1'b0;
                dec_r2_imm1 <= 1'b0;
                dec_r1_immv1 <= 32'd0;
                dec_r2_immv1 <= 32'd0;
            end
        end
    end

endmodule
