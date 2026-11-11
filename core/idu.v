`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu（分发级 —— 2026-11-11 拆分后的后半段）
// Description:
//   dsq 三个出口 → 分 lane → 交付寄存器（isq 的 D 级）+ LSU FIFO。
//   本模块不再做分配：物理号、ROB 槽号、类别都随 dsq 的载荷进来，身份已经定死。
//   译码在这里做，而且是【三套并排】：三个出口各一套，起点都是 dsq 的输出寄存器。
//   同一个出口一拍只取一条（按名次最老），所以每套译码只描述"这一拍这一口拿到的那条"。
//////////////////////////////////////////////////////////////////////////////////
module idu(
    input clk, rst,
    input        redir_frontend, redir_backend_0, redir_backend_1, irq_kill,
    input        exec_in,
    input [1:0]  store_pop,
    input [3:0]  store_flush_pop,
//---- dsq 三个出口 ----
    input        dq0_valid, input [132:0] dq0_data, output dq0_pop,
    input        dq1_valid, input [132:0] dq1_data, output dq1_pop,
    input        dqm_valid, input [132:0] dqm_data, output dqm_pop,
//---- lane0 / lane1 交付给 isq ----
    input        accept_in_0, accept_in_1,
    output reg   lane_valid_0, lane_valid_1,
    output reg [31:0] instruction_out_0, auxiliary_address_out_0,
    output reg        branch_prediction_taken_out_0,
    output reg [4:0]  source_a_out_0, source_b_out_0,
    output reg [4:0]  index_out_0,
    output reg        generation_out_0,
    output reg [5:0]  phys_a_out_0, phys_b_out_0, phys_d_out_0,
    output reg        payload_valid_out_0, payload_fresh_out_0,
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
    output reg        dec_brf_0, dec_jalf_0, dec_jalfr_0,
    output reg [3:0]  dec_bjuf4_0,
    output reg        dec_r1_imm_0, dec_r2_imm_0,
    output reg [31:0] dec_r1_immv_0, dec_r2_immv_0,
    output reg [31:0] instruction_out_1, auxiliary_address_out_1,
    output reg        branch_prediction_taken_out_1,
    output reg [4:0]  source_a_out_1, source_b_out_1,
    output reg [4:0]  index_out_1,
    output reg        generation_out_1,
    output reg [5:0]  phys_a_out_1, phys_b_out_1, phys_d_out_1,
    output reg        payload_valid_out_1, payload_fresh_out_1,
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
    output reg        dec_brf_1, dec_jalf_1, dec_jalfr_1,
    output reg [3:0]  dec_bjuf4_1,
    output reg        dec_r1_imm_1, dec_r2_imm_1,
    output reg [31:0] dec_r1_immv_1, dec_r2_immv_1,
//---- LSU FIFO 队头（给 isq 的访存站）----
    input         lsu_pop_in,
    output [31:0] lsu_instruction_out, lsu_auxiliary_address_out, lsu_immediate_out,
    output [4:0]  lsu_source_a_out, lsu_source_b_out, lsu_destination_register_out,
    output [2:0]  lsu_load_store_function3_out,
    output        lsu_is_store_out, lsu_write_enable_out,
    output [4:0]  lsu_index_out,
    output        lsu_generation_out, lsu_valid_out,
    output [5:0]  lsu_phys_a_out, lsu_phys_b_out, lsu_phys_d_out
    );

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

    localparam [1:0] CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;
//★ 编码必须与 bju.v 逐字一致（这里曾经写成另一套，JAL=5 被 bju 当成 EBREAK）。
    localparam [3:0] BJU_NONE = 4'd0, BJU_JAL = 4'd1, BJU_JAL_MIS = 4'd2, BJU_JALR = 4'd3,
                     BJU_ECALL = 4'd4, BJU_EBREAK = 4'd5, BJU_ILLEGAL = 4'd6, BJU_MRET = 4'd7;

    function [31:0] immI; input [31:0] i; immI = {{20{i[31]}}, i[31:20]}; endfunction
    function [31:0] immS; input [31:0] i; immS = {{20{i[31]}}, i[31:25], i[11:7]}; endfunction
    function [31:0] immB; input [31:0] i;
        immB = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0}; endfunction
    function [31:0] immU; input [31:0] i; immU = {i[31:12], 12'd0}; endfunction
    function [31:0] immJ; input [31:0] i;
        immJ = {{12{i[31]}}, i[19:12], i[20], i[30:21], 1'b0}; endfunction

//==============================================================================
//  dsq 载荷拆字段
//    [1:0] 类别 | [6:2] rs1 | [11:7] rs2 | [12] 分支预测 | [17:13] index |
//    [18] 世代 | [24:19] phys_a | [30:25] phys_b | [36:31] phys_d |
//    [68:37] 地址 | [100:69] jalr 预测 | [132:101] 原始指令
//==============================================================================
    wire [31:0] p0_inst = dq0_data[132:101];
    wire [31:0] p0_jlr  = dq0_data[100:69];
    wire [31:0] p0_addr = dq0_data[68:37];
    wire [5:0]  p0_pd   = dq0_data[36:31];
    wire [5:0]  p0_pb   = dq0_data[30:25];
    wire [5:0]  p0_pa   = dq0_data[24:19];
    wire        p0_gen  = dq0_data[18];
    wire [4:0]  p0_idx  = dq0_data[17:13];
    wire        p0_bp   = dq0_data[12];
    wire [4:0]  p0_rs2  = dq0_data[11:7];
    wire [4:0]  p0_rs1  = dq0_data[6:2];
    wire [1:0]  p0_cls  = dq0_data[1:0];

    wire [31:0] p1_inst = dq1_data[132:101];
    wire [31:0] p1_jlr  = dq1_data[100:69];
    wire [31:0] p1_addr = dq1_data[68:37];
    wire [5:0]  p1_pd   = dq1_data[36:31];
    wire [5:0]  p1_pb   = dq1_data[30:25];
    wire [5:0]  p1_pa   = dq1_data[24:19];
    wire        p1_gen  = dq1_data[18];
    wire [4:0]  p1_idx  = dq1_data[17:13];
    wire        p1_bp   = dq1_data[12];
    wire [4:0]  p1_rs2  = dq1_data[11:7];
    wire [4:0]  p1_rs1  = dq1_data[6:2];
    wire [1:0]  p1_cls  = dq1_data[1:0];

    wire [31:0] pm_inst = dqm_data[132:101];
    wire [31:0] pm_jlr  = dqm_data[100:69];
    wire [31:0] pm_addr = dqm_data[68:37];
    wire [5:0]  pm_pd   = dqm_data[36:31];
    wire [5:0]  pm_pb   = dqm_data[30:25];
    wire [5:0]  pm_pa   = dqm_data[24:19];
    wire        pm_gen  = dqm_data[18];
    wire [4:0]  pm_idx  = dqm_data[17:13];
    wire        pm_bp   = dqm_data[12];
    wire [4:0]  pm_rs2  = dqm_data[11:7];
    wire [4:0]  pm_rs1  = dqm_data[6:2];
    wire [1:0]  pm_cls  = dqm_data[1:0];

//---- 三套译码的寄存器（后缀 s0 / s1 / sm）----
    reg [4:0]  destination_decoded_s0, destination_decoded_s1, destination_decoded_sm;
    reg [9:0]  function10_decoded_s0;
    reg [31:0] immediate_decoded_s0;
    reg        jump_decoded_s0;
    reg csr_is_access_dec_s0, csr_exists_dec_s0, csr_ro_dec_s0, csr_wr_dec_s0;
    reg exc_illegal_dec_s0, exception_jump_misalign_decoded_s0;
//入站分类 + 各类字段装配
    reg [1:0]  class_decoded_s0;
    reg        write_enable_decoded_s0, csr_write_enable_decoded_s0, csr_write_commit_decoded_s0;
    reg [4:0]  destination_pick_decoded_s0;
    reg        is_store_decoded_s0;
    reg [2:0]  multiply_operation_decoded_s0, load_store_function3_decoded_s0, csr_function3_decoded_s0;
    reg [3:0]  function4_decoded_s0;
    reg [11:0] csr_address_decoded_s0;
    reg [31:0] jump_value_decoded_s0;
    reg        branch_flag_decoded_s0, jump_link_flag_decoded_s0, jump_link_register_flag_decoded_s0;
//非寄存器操作数的覆盖（立即数在这里并进操作数）
    reg source_a_immediate_decoded_s0, source_b_immediate_decoded_s0;
    reg [31:0] source_a_immediate_value_decoded_s0, source_b_immediate_value_decoded_s0;
//异常标记（ecall/ebreak/mret）
    reg exc_ecall_dec_s0, exc_ebreak_dec_s0, exc_irq_ret_dec_s0;
//bju 功能码：这条指令在 bju 里算什么。
    reg [3:0] branch_jump_function4_decoded_s0;
    reg [9:0]  function10_decoded_s1;
    reg [31:0] immediate_decoded_s1;
    reg        jump_decoded_s1;
    reg csr_is_access_dec_s1, csr_exists_dec_s1, csr_ro_dec_s1, csr_wr_dec_s1;
    reg exc_illegal_dec_s1, exception_jump_misalign_decoded_s1;
//入站分类 + 各类字段装配
    reg [1:0]  class_decoded_s1;
    reg        write_enable_decoded_s1, csr_write_enable_decoded_s1, csr_write_commit_decoded_s1;
    reg [4:0]  destination_pick_decoded_s1;
    reg        is_store_decoded_s1;
    reg [2:0]  multiply_operation_decoded_s1, load_store_function3_decoded_s1, csr_function3_decoded_s1;
    reg [3:0]  function4_decoded_s1;
    reg [11:0] csr_address_decoded_s1;
    reg [31:0] jump_value_decoded_s1;
    reg        branch_flag_decoded_s1, jump_link_flag_decoded_s1, jump_link_register_flag_decoded_s1;
//非寄存器操作数的覆盖（立即数在这里并进操作数）
    reg source_a_immediate_decoded_s1, source_b_immediate_decoded_s1;
    reg [31:0] source_a_immediate_value_decoded_s1, source_b_immediate_value_decoded_s1;
//异常标记（ecall/ebreak/mret）
    reg exc_ecall_dec_s1, exc_ebreak_dec_s1, exc_irq_ret_dec_s1;
//bju 功能码：这条指令在 bju 里算什么。
    reg [3:0] branch_jump_function4_decoded_s1;
    reg [9:0]  function10_decoded_sm;
    reg [31:0] immediate_decoded_sm;
    reg        jump_decoded_sm;
    reg csr_is_access_dec_sm, csr_exists_dec_sm, csr_ro_dec_sm, csr_wr_dec_sm;
    reg exc_illegal_dec_sm, exception_jump_misalign_decoded_sm;
//入站分类 + 各类字段装配
    reg [1:0]  class_decoded_sm;
    reg        write_enable_decoded_sm, csr_write_enable_decoded_sm, csr_write_commit_decoded_sm;
    reg [4:0]  destination_pick_decoded_sm;
    reg        is_store_decoded_sm;
    reg [2:0]  multiply_operation_decoded_sm, load_store_function3_decoded_sm, csr_function3_decoded_sm;
    reg [3:0]  function4_decoded_sm;
    reg [11:0] csr_address_decoded_sm;
    reg [31:0] jump_value_decoded_sm;
    reg        branch_flag_decoded_sm, jump_link_flag_decoded_sm, jump_link_register_flag_decoded_sm;
//非寄存器操作数的覆盖（立即数在这里并进操作数）
    reg source_a_immediate_decoded_sm, source_b_immediate_decoded_sm;
    reg [31:0] source_a_immediate_value_decoded_sm, source_b_immediate_value_decoded_sm;
//异常标记（ecall/ebreak/mret）
    reg exc_ecall_dec_sm, exc_ebreak_dec_sm, exc_irq_ret_dec_sm;
//bju 功能码：这条指令在 bju 里算什么。
    reg [3:0] branch_jump_function4_decoded_sm;
    always @(*) begin
        destination_decoded_s0 = 5'd0;
        function10_decoded_s0 = 10'd0;
        immediate_decoded_s0 = 32'd0;
        jump_decoded_s0 = 1'b0;
        case (p0_inst[6:0])
            OPCODE_OP: begin
                destination_decoded_s0 = p0_inst[11:7];
                function10_decoded_s0 = {p0_inst[31:25], p0_inst[14:12]};
            end
            OPCODE_OP_IMM: begin
                immediate_decoded_s0 = immI(p0_inst);
                destination_decoded_s0 = p0_inst[11:7];
                function10_decoded_s0 = {(p0_inst[14:12] == 3'b001 || p0_inst[14:12] == 3'b101) ? p0_inst[31:25] : 7'd0,
                p0_inst[14:12]};
            end
            OPCODE_JAL: begin
                destination_decoded_s0 = p0_inst[11:7];
                jump_decoded_s0 = 1'b1;
            end
            OPCODE_JALR: begin
                destination_decoded_s0 = p0_inst[11:7];
                immediate_decoded_s0 = immI(p0_inst);
                jump_decoded_s0 = 1'b1;
            end
            OPCODE_BRANCH: begin
                immediate_decoded_s0 = immB(p0_inst);
                function10_decoded_s0 = {7'd0, p0_inst[14:12]};
                jump_decoded_s0 = 1'b1;
            end
            OPCODE_LOAD: begin
                immediate_decoded_s0 = immI(p0_inst);
                destination_decoded_s0 = p0_inst[11:7];
            end
            OPCODE_STORE: begin
                immediate_decoded_s0 = immS(p0_inst);
            end
            OPCODE_LUI: begin
                immediate_decoded_s0 = immU(p0_inst);
                destination_decoded_s0 = p0_inst[11:7];
            end
            OPCODE_AUIPC: begin
                immediate_decoded_s0 = immU(p0_inst) - 4'd4;
                destination_decoded_s0 = p0_inst[11:7];
            end
            OPCODE_SYSTEM: begin
                destination_decoded_s0 = p0_inst[11:7];
                function10_decoded_s0 = {7'd0, p0_inst[14:12]};
            end
        endcase
    end
    always @(*) begin
        csr_is_access_dec_s0 = (p0_inst[14:12] != 3'b000) && (p0_inst[14:12] != 3'b100);
//★ "写"的规范口径：CSRRW/CSRRWI 恒写；CSRRS/CSRRC/CSRRSI/CSRRCI 只有源（rs1 或 uimm）非 0 才写。
        csr_wr_dec_s0        = (p0_inst[14:12] == 3'b001) | (p0_inst[14:12] == 3'b101) |
                            (p0_inst[19:15] != 5'd0);
        csr_ro_dec_s0        = (p0_inst[31:30] == 2'b11);
        csr_exists_dec_s0    = 1'b0;
        case (p0_inst[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec_s0 = 1'b1;
            default: csr_exists_dec_s0 = 1'b0;
        endcase
    end
    always @(*) begin
        exc_illegal_dec_s0 = 1'b0;
        exception_jump_misalign_decoded_s0    = (p0_inst[6:0] == OPCODE_JAL) & p0_inst[21];
        if (p0_inst != 32'd0) begin
            case (p0_inst[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec_s0 = 1'b0;
                OPCODE_LOAD: begin
                    if (p0_inst[14:12] == 3'b011) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b110) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b111) exc_illegal_dec_s0 = 1'b1;
                end
                OPCODE_STORE: begin
                    if (p0_inst[14:12] == 3'b011) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b100) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b101) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b110) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b111) exc_illegal_dec_s0 = 1'b1;
                end
                OPCODE_JALR: begin
                    if (p0_inst[14:12] != 3'b000) exc_illegal_dec_s0 = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (p0_inst[14:12] == 3'b010) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b011) exc_illegal_dec_s0 = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (p0_inst[14:12] == 3'b001) begin
                        if (p0_inst[31:25] != 7'd0) exc_illegal_dec_s0 = 1'b1;
                    end
                    if (p0_inst[14:12] == 3'b101) begin
                        if ((p0_inst[31:25] != 7'd0) && (p0_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_s0 = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (p0_inst[31:25] == 7'b0000001) exc_illegal_dec_s0 = 1'b0;
                    else if (p0_inst[14:12] == 3'b000) begin
                        if ((p0_inst[31:25] != 7'd0) && (p0_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_s0 = 1'b1;
                    end
                    else if (p0_inst[14:12] == 3'b001) begin
                        if (p0_inst[31:25] != 7'd0) exc_illegal_dec_s0 = 1'b1;
                    end
                    else if (p0_inst[14:12] == 3'b101) begin
                        if ((p0_inst[31:25] != 7'd0) && (p0_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_s0 = 1'b1;
                    end
                    else begin
                        if (p0_inst[31:25] != 7'd0) exc_illegal_dec_s0 = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (p0_inst[14:12] == 3'b010) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b011) exc_illegal_dec_s0 = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (p0_inst[14:12] == 3'b100) exc_illegal_dec_s0 = 1'b1;
                    if (p0_inst[14:12] == 3'b000) begin
                        case (p0_inst[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;   // ecall / ebreak / mret / wfi（wfi 当 NOP）
                            default: exc_illegal_dec_s0 = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec_s0) begin
                        if (!csr_exists_dec_s0) exc_illegal_dec_s0 = 1'b1;
                        if (csr_ro_dec_s0) begin
                            if (csr_wr_dec_s0) exc_illegal_dec_s0 = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec_s0 = 1'b1;
            endcase
        end
    end
    always @(*) begin
        class_decoded_s0      = CLS_ALU;
        write_enable_decoded_s0         = 1'b0;
        csr_write_enable_decoded_s0    = 1'b0;
        csr_write_commit_decoded_s0    = 1'b0;
        destination_pick_decoded_s0    = 5'd0;
        is_store_decoded_s0      = 1'b0;
        multiply_operation_decoded_s0      = 3'd0;
        load_store_function3_decoded_s0       = 3'd0;
        csr_function3_decoded_s0      = 3'd0;
        function4_decoded_s0      = 4'd0;
        csr_address_decoded_s0       = 12'd0;
        jump_value_decoded_s0       = 32'd0;
        branch_flag_decoded_s0        = 1'b0;
        jump_link_flag_decoded_s0       = 1'b0;
        jump_link_register_flag_decoded_s0      = 1'b0;

        case (p0_inst[6:0])
            OPCODE_OP: begin
                if (p0_inst[31:25] == 7'b0000001) begin
                    class_decoded_s0 = CLS_MUL;
                    multiply_operation_decoded_s0 = p0_inst[14:12];
                    destination_pick_decoded_s0 = destination_decoded_s0;   //★ MUL 也要给 rd：不给 dec_rd=0 ⇒ pstat 不登记它（见 isq 的 d_wr）
                end
                else begin
                    function4_decoded_s0   = {function10_decoded_s0[8], function10_decoded_s0[2:0]};
                    destination_pick_decoded_s0 = destination_decoded_s0;
                    write_enable_decoded_s0      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                function4_decoded_s0   = {function10_decoded_s0[8], function10_decoded_s0[2:0]};
                destination_pick_decoded_s0 = destination_decoded_s0;
                write_enable_decoded_s0      = 1'b1;
            end
            OPCODE_JAL: begin
                class_decoded_s0   = CLS_BJU;
                jump_link_flag_decoded_s0    = 1'b1;
                destination_pick_decoded_s0 = destination_decoded_s0;
                write_enable_decoded_s0      = 1'b1;
                jump_value_decoded_s0    = p0_addr + immJ(p0_inst) - 32'd4;
            end
            OPCODE_JALR: begin
                class_decoded_s0   = CLS_BJU;
                jump_link_register_flag_decoded_s0   = 1'b1;
                destination_pick_decoded_s0 = destination_decoded_s0;
                write_enable_decoded_s0      = 1'b1;
                jump_value_decoded_s0    = p0_jlr;
            end
            OPCODE_BRANCH: begin
                class_decoded_s0   = CLS_BJU;
                branch_flag_decoded_s0     = 1'b1;
                function4_decoded_s0   = {1'b0, p0_inst[14:12]};
                jump_value_decoded_s0    = immediate_decoded_s0 - 32'd4;
            end
            OPCODE_LUI: begin
                destination_pick_decoded_s0 = destination_decoded_s0;
                write_enable_decoded_s0      = 1'b1;
            end
            OPCODE_AUIPC: begin
                destination_pick_decoded_s0 = destination_decoded_s0;
                write_enable_decoded_s0      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_decoded_s0   = CLS_LSU;
                load_store_function3_decoded_s0    = p0_inst[14:12];
                destination_pick_decoded_s0 = destination_decoded_s0;
                jump_value_decoded_s0    = immediate_decoded_s0;
            end
            OPCODE_STORE: begin
                class_decoded_s0   = CLS_LSU;
                is_store_decoded_s0   = 1'b1;
                load_store_function3_decoded_s0    = p0_inst[14:12];
                jump_value_decoded_s0    = immediate_decoded_s0;
            end
            OPCODE_SYSTEM: begin
                class_decoded_s0   = CLS_BJU;
                csr_function3_decoded_s0   = p0_inst[14:12];
                destination_pick_decoded_s0 = destination_decoded_s0;
                case (p0_inst[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        write_enable_decoded_s0         = 1'b1;
                        csr_write_enable_decoded_s0    = 1'b1;
                        csr_write_commit_decoded_s0    = csr_wr_dec_s0;
                    end
                    default: begin
                        write_enable_decoded_s0      = 1'b0;
                        csr_write_enable_decoded_s0 = 1'b0;
                    end
                endcase
//★ csr 地址【不写也照给】：地址供 csr.v 的读口用，写不写只决定"锁不锁"。
                csr_address_decoded_s0 = (p0_inst[14:12] != 3'b000) ? p0_inst[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec_s0 | exception_jump_misalign_decoded_s0) begin
            write_enable_decoded_s0         = 1'b0;
            csr_write_enable_decoded_s0    = 1'b0;
            csr_write_commit_decoded_s0    = 1'b0;
        end
    end
    always @(*) begin
        source_a_immediate_decoded_s0  = 1'b0;
        source_b_immediate_decoded_s0  = 1'b0;
        source_a_immediate_value_decoded_s0 = 32'd0;
        source_b_immediate_value_decoded_s0 = 32'd0;
        case (p0_inst[6:0])
            OPCODE_OP_IMM: begin source_b_immediate_decoded_s0 = 1'b1; source_b_immediate_value_decoded_s0 = immediate_decoded_s0; end
            OPCODE_JALR:   begin source_b_immediate_decoded_s0 = 1'b1; source_b_immediate_value_decoded_s0 = immediate_decoded_s0; end
            OPCODE_LUI:    begin source_b_immediate_decoded_s0 = 1'b1; source_b_immediate_value_decoded_s0 = immediate_decoded_s0; end
            OPCODE_AUIPC:  begin
                source_a_immediate_decoded_s0  = 1'b1; source_a_immediate_value_decoded_s0 = p0_addr;
                source_b_immediate_decoded_s0  = 1'b1; source_b_immediate_value_decoded_s0 = immediate_decoded_s0;
            end
            OPCODE_SYSTEM: begin
                if ((p0_inst[14:12] == 3'b101) | (p0_inst[14:12] == 3'b110) | (p0_inst[14:12] == 3'b111)) begin
                    source_a_immediate_decoded_s0  = 1'b1;
                    source_a_immediate_value_decoded_s0 = {27'd0, p0_inst[19:15]};
                end
            end
            default: ;
        endcase
    end
    always @(*) begin
        exc_ecall_dec_s0   = (p0_inst[6:0] == OPCODE_SYSTEM) && (p0_inst[31:20] == 12'h000);
        exc_ebreak_dec_s0  = (p0_inst[6:0] == OPCODE_SYSTEM) && (p0_inst[31:20] == 12'h001);
        exc_irq_ret_dec_s0 = (p0_inst[6:0] == OPCODE_SYSTEM) && (p0_inst[14:12] == 3'b000)
                        && (p0_inst[31:20] == 12'h302);
    end
    always @(*) begin
        branch_jump_function4_decoded_s0 = BJU_NONE;
        if (exc_illegal_dec_s0)
            branch_jump_function4_decoded_s0 = BJU_ILLEGAL;
        else if (exc_ecall_dec_s0)
            branch_jump_function4_decoded_s0 = BJU_ECALL;
        else if (exc_ebreak_dec_s0)
            branch_jump_function4_decoded_s0 = BJU_EBREAK;
        else if (exc_irq_ret_dec_s0)
            branch_jump_function4_decoded_s0 = BJU_MRET;
        else if (branch_flag_decoded_s0)
            branch_jump_function4_decoded_s0 = {1'b1, function4_decoded_s0[2:0]};
        else if (jump_link_flag_decoded_s0)
            branch_jump_function4_decoded_s0 = exception_jump_misalign_decoded_s0 ? BJU_JAL_MIS : BJU_JAL;
        else if (jump_link_register_flag_decoded_s0)
            branch_jump_function4_decoded_s0 = BJU_JALR;
    end
    always @(*) begin
        destination_decoded_s1 = 5'd0;
        function10_decoded_s1 = 10'd0;
        immediate_decoded_s1 = 32'd0;
        jump_decoded_s1 = 1'b0;
        case (p1_inst[6:0])
            OPCODE_OP: begin
                destination_decoded_s1 = p1_inst[11:7];
                function10_decoded_s1 = {p1_inst[31:25], p1_inst[14:12]};
            end
            OPCODE_OP_IMM: begin
                immediate_decoded_s1 = immI(p1_inst);
                destination_decoded_s1 = p1_inst[11:7];
                function10_decoded_s1 = {(p1_inst[14:12] == 3'b001 || p1_inst[14:12] == 3'b101) ? p1_inst[31:25] : 7'd0,
                p1_inst[14:12]};
            end
            OPCODE_JAL: begin
                destination_decoded_s1 = p1_inst[11:7];
                jump_decoded_s1 = 1'b1;
            end
            OPCODE_JALR: begin
                destination_decoded_s1 = p1_inst[11:7];
                immediate_decoded_s1 = immI(p1_inst);
                jump_decoded_s1 = 1'b1;
            end
            OPCODE_BRANCH: begin
                immediate_decoded_s1 = immB(p1_inst);
                function10_decoded_s1 = {7'd0, p1_inst[14:12]};
                jump_decoded_s1 = 1'b1;
            end
            OPCODE_LOAD: begin
                immediate_decoded_s1 = immI(p1_inst);
                destination_decoded_s1 = p1_inst[11:7];
            end
            OPCODE_STORE: begin
                immediate_decoded_s1 = immS(p1_inst);
            end
            OPCODE_LUI: begin
                immediate_decoded_s1 = immU(p1_inst);
                destination_decoded_s1 = p1_inst[11:7];
            end
            OPCODE_AUIPC: begin
                immediate_decoded_s1 = immU(p1_inst) - 4'd4;
                destination_decoded_s1 = p1_inst[11:7];
            end
            OPCODE_SYSTEM: begin
                destination_decoded_s1 = p1_inst[11:7];
                function10_decoded_s1 = {7'd0, p1_inst[14:12]};
            end
        endcase
    end
    always @(*) begin
        csr_is_access_dec_s1 = (p1_inst[14:12] != 3'b000) && (p1_inst[14:12] != 3'b100);
//★ "写"的规范口径：CSRRW/CSRRWI 恒写；CSRRS/CSRRC/CSRRSI/CSRRCI 只有源（rs1 或 uimm）非 0 才写。
        csr_wr_dec_s1        = (p1_inst[14:12] == 3'b001) | (p1_inst[14:12] == 3'b101) |
                            (p1_inst[19:15] != 5'd0);
        csr_ro_dec_s1        = (p1_inst[31:30] == 2'b11);
        csr_exists_dec_s1    = 1'b0;
        case (p1_inst[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec_s1 = 1'b1;
            default: csr_exists_dec_s1 = 1'b0;
        endcase
    end
    always @(*) begin
        exc_illegal_dec_s1 = 1'b0;
        exception_jump_misalign_decoded_s1    = (p1_inst[6:0] == OPCODE_JAL) & p1_inst[21];
        if (p1_inst != 32'd0) begin
            case (p1_inst[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec_s1 = 1'b0;
                OPCODE_LOAD: begin
                    if (p1_inst[14:12] == 3'b011) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b110) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b111) exc_illegal_dec_s1 = 1'b1;
                end
                OPCODE_STORE: begin
                    if (p1_inst[14:12] == 3'b011) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b100) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b101) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b110) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b111) exc_illegal_dec_s1 = 1'b1;
                end
                OPCODE_JALR: begin
                    if (p1_inst[14:12] != 3'b000) exc_illegal_dec_s1 = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (p1_inst[14:12] == 3'b010) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b011) exc_illegal_dec_s1 = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (p1_inst[14:12] == 3'b001) begin
                        if (p1_inst[31:25] != 7'd0) exc_illegal_dec_s1 = 1'b1;
                    end
                    if (p1_inst[14:12] == 3'b101) begin
                        if ((p1_inst[31:25] != 7'd0) && (p1_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_s1 = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (p1_inst[31:25] == 7'b0000001) exc_illegal_dec_s1 = 1'b0;
                    else if (p1_inst[14:12] == 3'b000) begin
                        if ((p1_inst[31:25] != 7'd0) && (p1_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_s1 = 1'b1;
                    end
                    else if (p1_inst[14:12] == 3'b001) begin
                        if (p1_inst[31:25] != 7'd0) exc_illegal_dec_s1 = 1'b1;
                    end
                    else if (p1_inst[14:12] == 3'b101) begin
                        if ((p1_inst[31:25] != 7'd0) && (p1_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_s1 = 1'b1;
                    end
                    else begin
                        if (p1_inst[31:25] != 7'd0) exc_illegal_dec_s1 = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (p1_inst[14:12] == 3'b010) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b011) exc_illegal_dec_s1 = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (p1_inst[14:12] == 3'b100) exc_illegal_dec_s1 = 1'b1;
                    if (p1_inst[14:12] == 3'b000) begin
                        case (p1_inst[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;   // ecall / ebreak / mret / wfi（wfi 当 NOP）
                            default: exc_illegal_dec_s1 = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec_s1) begin
                        if (!csr_exists_dec_s1) exc_illegal_dec_s1 = 1'b1;
                        if (csr_ro_dec_s1) begin
                            if (csr_wr_dec_s1) exc_illegal_dec_s1 = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec_s1 = 1'b1;
            endcase
        end
    end
    always @(*) begin
        class_decoded_s1      = CLS_ALU;
        write_enable_decoded_s1         = 1'b0;
        csr_write_enable_decoded_s1    = 1'b0;
        csr_write_commit_decoded_s1    = 1'b0;
        destination_pick_decoded_s1    = 5'd0;
        is_store_decoded_s1      = 1'b0;
        multiply_operation_decoded_s1      = 3'd0;
        load_store_function3_decoded_s1       = 3'd0;
        csr_function3_decoded_s1      = 3'd0;
        function4_decoded_s1      = 4'd0;
        csr_address_decoded_s1       = 12'd0;
        jump_value_decoded_s1       = 32'd0;
        branch_flag_decoded_s1        = 1'b0;
        jump_link_flag_decoded_s1       = 1'b0;
        jump_link_register_flag_decoded_s1      = 1'b0;

        case (p1_inst[6:0])
            OPCODE_OP: begin
                if (p1_inst[31:25] == 7'b0000001) begin
                    class_decoded_s1 = CLS_MUL;
                    multiply_operation_decoded_s1 = p1_inst[14:12];
                    destination_pick_decoded_s1 = destination_decoded_s1;   //★ MUL 也要给 rd：不给 dec_rd=0 ⇒ pstat 不登记它（见 isq 的 d_wr）
                end
                else begin
                    function4_decoded_s1   = {function10_decoded_s1[8], function10_decoded_s1[2:0]};
                    destination_pick_decoded_s1 = destination_decoded_s1;
                    write_enable_decoded_s1      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                function4_decoded_s1   = {function10_decoded_s1[8], function10_decoded_s1[2:0]};
                destination_pick_decoded_s1 = destination_decoded_s1;
                write_enable_decoded_s1      = 1'b1;
            end
            OPCODE_JAL: begin
                class_decoded_s1   = CLS_BJU;
                jump_link_flag_decoded_s1    = 1'b1;
                destination_pick_decoded_s1 = destination_decoded_s1;
                write_enable_decoded_s1      = 1'b1;
                jump_value_decoded_s1    = p1_addr + immJ(p1_inst) - 32'd4;
            end
            OPCODE_JALR: begin
                class_decoded_s1   = CLS_BJU;
                jump_link_register_flag_decoded_s1   = 1'b1;
                destination_pick_decoded_s1 = destination_decoded_s1;
                write_enable_decoded_s1      = 1'b1;
                jump_value_decoded_s1    = p1_jlr;
            end
            OPCODE_BRANCH: begin
                class_decoded_s1   = CLS_BJU;
                branch_flag_decoded_s1     = 1'b1;
                function4_decoded_s1   = {1'b0, p1_inst[14:12]};
                jump_value_decoded_s1    = immediate_decoded_s1 - 32'd4;
            end
            OPCODE_LUI: begin
                destination_pick_decoded_s1 = destination_decoded_s1;
                write_enable_decoded_s1      = 1'b1;
            end
            OPCODE_AUIPC: begin
                destination_pick_decoded_s1 = destination_decoded_s1;
                write_enable_decoded_s1      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_decoded_s1   = CLS_LSU;
                load_store_function3_decoded_s1    = p1_inst[14:12];
                destination_pick_decoded_s1 = destination_decoded_s1;
                jump_value_decoded_s1    = immediate_decoded_s1;
            end
            OPCODE_STORE: begin
                class_decoded_s1   = CLS_LSU;
                is_store_decoded_s1   = 1'b1;
                load_store_function3_decoded_s1    = p1_inst[14:12];
                jump_value_decoded_s1    = immediate_decoded_s1;
            end
            OPCODE_SYSTEM: begin
                class_decoded_s1   = CLS_BJU;
                csr_function3_decoded_s1   = p1_inst[14:12];
                destination_pick_decoded_s1 = destination_decoded_s1;
                case (p1_inst[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        write_enable_decoded_s1         = 1'b1;
                        csr_write_enable_decoded_s1    = 1'b1;
                        csr_write_commit_decoded_s1    = csr_wr_dec_s1;
                    end
                    default: begin
                        write_enable_decoded_s1      = 1'b0;
                        csr_write_enable_decoded_s1 = 1'b0;
                    end
                endcase
//★ csr 地址【不写也照给】：地址供 csr.v 的读口用，写不写只决定"锁不锁"。
                csr_address_decoded_s1 = (p1_inst[14:12] != 3'b000) ? p1_inst[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec_s1 | exception_jump_misalign_decoded_s1) begin
            write_enable_decoded_s1         = 1'b0;
            csr_write_enable_decoded_s1    = 1'b0;
            csr_write_commit_decoded_s1    = 1'b0;
        end
    end
    always @(*) begin
        source_a_immediate_decoded_s1  = 1'b0;
        source_b_immediate_decoded_s1  = 1'b0;
        source_a_immediate_value_decoded_s1 = 32'd0;
        source_b_immediate_value_decoded_s1 = 32'd0;
        case (p1_inst[6:0])
            OPCODE_OP_IMM: begin source_b_immediate_decoded_s1 = 1'b1; source_b_immediate_value_decoded_s1 = immediate_decoded_s1; end
            OPCODE_JALR:   begin source_b_immediate_decoded_s1 = 1'b1; source_b_immediate_value_decoded_s1 = immediate_decoded_s1; end
            OPCODE_LUI:    begin source_b_immediate_decoded_s1 = 1'b1; source_b_immediate_value_decoded_s1 = immediate_decoded_s1; end
            OPCODE_AUIPC:  begin
                source_a_immediate_decoded_s1  = 1'b1; source_a_immediate_value_decoded_s1 = p1_addr;
                source_b_immediate_decoded_s1  = 1'b1; source_b_immediate_value_decoded_s1 = immediate_decoded_s1;
            end
            OPCODE_SYSTEM: begin
                if ((p1_inst[14:12] == 3'b101) | (p1_inst[14:12] == 3'b110) | (p1_inst[14:12] == 3'b111)) begin
                    source_a_immediate_decoded_s1  = 1'b1;
                    source_a_immediate_value_decoded_s1 = {27'd0, p1_inst[19:15]};
                end
            end
            default: ;
        endcase
    end
    always @(*) begin
        exc_ecall_dec_s1   = (p1_inst[6:0] == OPCODE_SYSTEM) && (p1_inst[31:20] == 12'h000);
        exc_ebreak_dec_s1  = (p1_inst[6:0] == OPCODE_SYSTEM) && (p1_inst[31:20] == 12'h001);
        exc_irq_ret_dec_s1 = (p1_inst[6:0] == OPCODE_SYSTEM) && (p1_inst[14:12] == 3'b000)
                        && (p1_inst[31:20] == 12'h302);
    end
    always @(*) begin
        branch_jump_function4_decoded_s1 = BJU_NONE;
        if (exc_illegal_dec_s1)
            branch_jump_function4_decoded_s1 = BJU_ILLEGAL;
        else if (exc_ecall_dec_s1)
            branch_jump_function4_decoded_s1 = BJU_ECALL;
        else if (exc_ebreak_dec_s1)
            branch_jump_function4_decoded_s1 = BJU_EBREAK;
        else if (exc_irq_ret_dec_s1)
            branch_jump_function4_decoded_s1 = BJU_MRET;
        else if (branch_flag_decoded_s1)
            branch_jump_function4_decoded_s1 = {1'b1, function4_decoded_s1[2:0]};
        else if (jump_link_flag_decoded_s1)
            branch_jump_function4_decoded_s1 = exception_jump_misalign_decoded_s1 ? BJU_JAL_MIS : BJU_JAL;
        else if (jump_link_register_flag_decoded_s1)
            branch_jump_function4_decoded_s1 = BJU_JALR;
    end
    always @(*) begin
        destination_decoded_sm = 5'd0;
        function10_decoded_sm = 10'd0;
        immediate_decoded_sm = 32'd0;
        jump_decoded_sm = 1'b0;
        case (pm_inst[6:0])
            OPCODE_OP: begin
                destination_decoded_sm = pm_inst[11:7];
                function10_decoded_sm = {pm_inst[31:25], pm_inst[14:12]};
            end
            OPCODE_OP_IMM: begin
                immediate_decoded_sm = immI(pm_inst);
                destination_decoded_sm = pm_inst[11:7];
                function10_decoded_sm = {(pm_inst[14:12] == 3'b001 || pm_inst[14:12] == 3'b101) ? pm_inst[31:25] : 7'd0,
                pm_inst[14:12]};
            end
            OPCODE_JAL: begin
                destination_decoded_sm = pm_inst[11:7];
                jump_decoded_sm = 1'b1;
            end
            OPCODE_JALR: begin
                destination_decoded_sm = pm_inst[11:7];
                immediate_decoded_sm = immI(pm_inst);
                jump_decoded_sm = 1'b1;
            end
            OPCODE_BRANCH: begin
                immediate_decoded_sm = immB(pm_inst);
                function10_decoded_sm = {7'd0, pm_inst[14:12]};
                jump_decoded_sm = 1'b1;
            end
            OPCODE_LOAD: begin
                immediate_decoded_sm = immI(pm_inst);
                destination_decoded_sm = pm_inst[11:7];
            end
            OPCODE_STORE: begin
                immediate_decoded_sm = immS(pm_inst);
            end
            OPCODE_LUI: begin
                immediate_decoded_sm = immU(pm_inst);
                destination_decoded_sm = pm_inst[11:7];
            end
            OPCODE_AUIPC: begin
                immediate_decoded_sm = immU(pm_inst) - 4'd4;
                destination_decoded_sm = pm_inst[11:7];
            end
            OPCODE_SYSTEM: begin
                destination_decoded_sm = pm_inst[11:7];
                function10_decoded_sm = {7'd0, pm_inst[14:12]};
            end
        endcase
    end
    always @(*) begin
        csr_is_access_dec_sm = (pm_inst[14:12] != 3'b000) && (pm_inst[14:12] != 3'b100);
//★ "写"的规范口径：CSRRW/CSRRWI 恒写；CSRRS/CSRRC/CSRRSI/CSRRCI 只有源（rs1 或 uimm）非 0 才写。
        csr_wr_dec_sm        = (pm_inst[14:12] == 3'b001) | (pm_inst[14:12] == 3'b101) |
                            (pm_inst[19:15] != 5'd0);
        csr_ro_dec_sm        = (pm_inst[31:30] == 2'b11);
        csr_exists_dec_sm    = 1'b0;
        case (pm_inst[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec_sm = 1'b1;
            default: csr_exists_dec_sm = 1'b0;
        endcase
    end
    always @(*) begin
        exc_illegal_dec_sm = 1'b0;
        exception_jump_misalign_decoded_sm    = (pm_inst[6:0] == OPCODE_JAL) & pm_inst[21];
        if (pm_inst != 32'd0) begin
            case (pm_inst[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec_sm = 1'b0;
                OPCODE_LOAD: begin
                    if (pm_inst[14:12] == 3'b011) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b110) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b111) exc_illegal_dec_sm = 1'b1;
                end
                OPCODE_STORE: begin
                    if (pm_inst[14:12] == 3'b011) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b100) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b101) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b110) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b111) exc_illegal_dec_sm = 1'b1;
                end
                OPCODE_JALR: begin
                    if (pm_inst[14:12] != 3'b000) exc_illegal_dec_sm = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (pm_inst[14:12] == 3'b010) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b011) exc_illegal_dec_sm = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (pm_inst[14:12] == 3'b001) begin
                        if (pm_inst[31:25] != 7'd0) exc_illegal_dec_sm = 1'b1;
                    end
                    if (pm_inst[14:12] == 3'b101) begin
                        if ((pm_inst[31:25] != 7'd0) && (pm_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_sm = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (pm_inst[31:25] == 7'b0000001) exc_illegal_dec_sm = 1'b0;
                    else if (pm_inst[14:12] == 3'b000) begin
                        if ((pm_inst[31:25] != 7'd0) && (pm_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_sm = 1'b1;
                    end
                    else if (pm_inst[14:12] == 3'b001) begin
                        if (pm_inst[31:25] != 7'd0) exc_illegal_dec_sm = 1'b1;
                    end
                    else if (pm_inst[14:12] == 3'b101) begin
                        if ((pm_inst[31:25] != 7'd0) && (pm_inst[31:25] != 7'b0100000))
                            exc_illegal_dec_sm = 1'b1;
                    end
                    else begin
                        if (pm_inst[31:25] != 7'd0) exc_illegal_dec_sm = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (pm_inst[14:12] == 3'b010) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b011) exc_illegal_dec_sm = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (pm_inst[14:12] == 3'b100) exc_illegal_dec_sm = 1'b1;
                    if (pm_inst[14:12] == 3'b000) begin
                        case (pm_inst[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;   // ecall / ebreak / mret / wfi（wfi 当 NOP）
                            default: exc_illegal_dec_sm = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec_sm) begin
                        if (!csr_exists_dec_sm) exc_illegal_dec_sm = 1'b1;
                        if (csr_ro_dec_sm) begin
                            if (csr_wr_dec_sm) exc_illegal_dec_sm = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec_sm = 1'b1;
            endcase
        end
    end
    always @(*) begin
        class_decoded_sm      = CLS_ALU;
        write_enable_decoded_sm         = 1'b0;
        csr_write_enable_decoded_sm    = 1'b0;
        csr_write_commit_decoded_sm    = 1'b0;
        destination_pick_decoded_sm    = 5'd0;
        is_store_decoded_sm      = 1'b0;
        multiply_operation_decoded_sm      = 3'd0;
        load_store_function3_decoded_sm       = 3'd0;
        csr_function3_decoded_sm      = 3'd0;
        function4_decoded_sm      = 4'd0;
        csr_address_decoded_sm       = 12'd0;
        jump_value_decoded_sm       = 32'd0;
        branch_flag_decoded_sm        = 1'b0;
        jump_link_flag_decoded_sm       = 1'b0;
        jump_link_register_flag_decoded_sm      = 1'b0;

        case (pm_inst[6:0])
            OPCODE_OP: begin
                if (pm_inst[31:25] == 7'b0000001) begin
                    class_decoded_sm = CLS_MUL;
                    multiply_operation_decoded_sm = pm_inst[14:12];
                    destination_pick_decoded_sm = destination_decoded_sm;   //★ MUL 也要给 rd：不给 dec_rd=0 ⇒ pstat 不登记它（见 isq 的 d_wr）
                end
                else begin
                    function4_decoded_sm   = {function10_decoded_sm[8], function10_decoded_sm[2:0]};
                    destination_pick_decoded_sm = destination_decoded_sm;
                    write_enable_decoded_sm      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                function4_decoded_sm   = {function10_decoded_sm[8], function10_decoded_sm[2:0]};
                destination_pick_decoded_sm = destination_decoded_sm;
                write_enable_decoded_sm      = 1'b1;
            end
            OPCODE_JAL: begin
                class_decoded_sm   = CLS_BJU;
                jump_link_flag_decoded_sm    = 1'b1;
                destination_pick_decoded_sm = destination_decoded_sm;
                write_enable_decoded_sm      = 1'b1;
                jump_value_decoded_sm    = pm_addr + immJ(pm_inst) - 32'd4;
            end
            OPCODE_JALR: begin
                class_decoded_sm   = CLS_BJU;
                jump_link_register_flag_decoded_sm   = 1'b1;
                destination_pick_decoded_sm = destination_decoded_sm;
                write_enable_decoded_sm      = 1'b1;
                jump_value_decoded_sm    = pm_jlr;
            end
            OPCODE_BRANCH: begin
                class_decoded_sm   = CLS_BJU;
                branch_flag_decoded_sm     = 1'b1;
                function4_decoded_sm   = {1'b0, pm_inst[14:12]};
                jump_value_decoded_sm    = immediate_decoded_sm - 32'd4;
            end
            OPCODE_LUI: begin
                destination_pick_decoded_sm = destination_decoded_sm;
                write_enable_decoded_sm      = 1'b1;
            end
            OPCODE_AUIPC: begin
                destination_pick_decoded_sm = destination_decoded_sm;
                write_enable_decoded_sm      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_decoded_sm   = CLS_LSU;
                load_store_function3_decoded_sm    = pm_inst[14:12];
                destination_pick_decoded_sm = destination_decoded_sm;
                jump_value_decoded_sm    = immediate_decoded_sm;
            end
            OPCODE_STORE: begin
                class_decoded_sm   = CLS_LSU;
                is_store_decoded_sm   = 1'b1;
                load_store_function3_decoded_sm    = pm_inst[14:12];
                jump_value_decoded_sm    = immediate_decoded_sm;
            end
            OPCODE_SYSTEM: begin
                class_decoded_sm   = CLS_BJU;
                csr_function3_decoded_sm   = pm_inst[14:12];
                destination_pick_decoded_sm = destination_decoded_sm;
                case (pm_inst[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        write_enable_decoded_sm         = 1'b1;
                        csr_write_enable_decoded_sm    = 1'b1;
                        csr_write_commit_decoded_sm    = csr_wr_dec_sm;
                    end
                    default: begin
                        write_enable_decoded_sm      = 1'b0;
                        csr_write_enable_decoded_sm = 1'b0;
                    end
                endcase
//★ csr 地址【不写也照给】：地址供 csr.v 的读口用，写不写只决定"锁不锁"。
                csr_address_decoded_sm = (pm_inst[14:12] != 3'b000) ? pm_inst[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec_sm | exception_jump_misalign_decoded_sm) begin
            write_enable_decoded_sm         = 1'b0;
            csr_write_enable_decoded_sm    = 1'b0;
            csr_write_commit_decoded_sm    = 1'b0;
        end
    end
    always @(*) begin
        source_a_immediate_decoded_sm  = 1'b0;
        source_b_immediate_decoded_sm  = 1'b0;
        source_a_immediate_value_decoded_sm = 32'd0;
        source_b_immediate_value_decoded_sm = 32'd0;
        case (pm_inst[6:0])
            OPCODE_OP_IMM: begin source_b_immediate_decoded_sm = 1'b1; source_b_immediate_value_decoded_sm = immediate_decoded_sm; end
            OPCODE_JALR:   begin source_b_immediate_decoded_sm = 1'b1; source_b_immediate_value_decoded_sm = immediate_decoded_sm; end
            OPCODE_LUI:    begin source_b_immediate_decoded_sm = 1'b1; source_b_immediate_value_decoded_sm = immediate_decoded_sm; end
            OPCODE_AUIPC:  begin
                source_a_immediate_decoded_sm  = 1'b1; source_a_immediate_value_decoded_sm = pm_addr;
                source_b_immediate_decoded_sm  = 1'b1; source_b_immediate_value_decoded_sm = immediate_decoded_sm;
            end
            OPCODE_SYSTEM: begin
                if ((pm_inst[14:12] == 3'b101) | (pm_inst[14:12] == 3'b110) | (pm_inst[14:12] == 3'b111)) begin
                    source_a_immediate_decoded_sm  = 1'b1;
                    source_a_immediate_value_decoded_sm = {27'd0, pm_inst[19:15]};
                end
            end
            default: ;
        endcase
    end
    always @(*) begin
        exc_ecall_dec_sm   = (pm_inst[6:0] == OPCODE_SYSTEM) && (pm_inst[31:20] == 12'h000);
        exc_ebreak_dec_sm  = (pm_inst[6:0] == OPCODE_SYSTEM) && (pm_inst[31:20] == 12'h001);
        exc_irq_ret_dec_sm = (pm_inst[6:0] == OPCODE_SYSTEM) && (pm_inst[14:12] == 3'b000)
                        && (pm_inst[31:20] == 12'h302);
    end
    always @(*) begin
        branch_jump_function4_decoded_sm = BJU_NONE;
        if (exc_illegal_dec_sm)
            branch_jump_function4_decoded_sm = BJU_ILLEGAL;
        else if (exc_ecall_dec_sm)
            branch_jump_function4_decoded_sm = BJU_ECALL;
        else if (exc_ebreak_dec_sm)
            branch_jump_function4_decoded_sm = BJU_EBREAK;
        else if (exc_irq_ret_dec_sm)
            branch_jump_function4_decoded_sm = BJU_MRET;
        else if (branch_flag_decoded_sm)
            branch_jump_function4_decoded_sm = {1'b1, function4_decoded_sm[2:0]};
        else if (jump_link_flag_decoded_sm)
            branch_jump_function4_decoded_sm = exception_jump_misalign_decoded_sm ? BJU_JAL_MIS : BJU_JAL;
        else if (jump_link_register_flag_decoded_sm)
            branch_jump_function4_decoded_sm = BJU_JALR;
    end
//==============================================================================
//  一、译码结果 → 交付口（三个出口各一套）
//==============================================================================
//★ 2026-11-11：dec_* 必须跟 instruction_out_* 同一个寄存器落地。
//  放到组合里读 dsq 口的话，出队那一拍之后 dsq 口已经换成【下一条】，
//  这一套译码就描述成下一条指令了（isq 拿到的是错类、错 rd、错 bju 功能码）。
    always @(posedge clk) begin
        if (dq0_pop) begin
            dec_class_0   <= class_decoded_s0;
            dec_we_0      <= write_enable_decoded_s0;
            dec_rd_0      <= destination_pick_decoded_s0;
            dec_is_st_0   <= is_store_decoded_s0;
            dec_mulop_0   <= multiply_operation_decoded_s0;
            dec_lsf3_0    <= load_store_function3_decoded_s0;
            dec_func4_0   <= function4_decoded_s0;
            dec_csrf3_0   <= csr_function3_decoded_s0;
            dec_csrw_en_0 <= csr_write_enable_decoded_s0;
            dec_csrw_wr_0 <= csr_write_commit_decoded_s0;
            dec_csra_0    <= csr_address_decoded_s0;
            dec_jval_0    <= jump_value_decoded_s0;
            dec_brf_0     <= branch_flag_decoded_s0;
            dec_jalf_0    <= jump_link_flag_decoded_s0;
            dec_jalfr_0   <= jump_link_register_flag_decoded_s0;
            dec_bjuf4_0   <= branch_jump_function4_decoded_s0;
            dec_r1_imm_0  <= source_a_immediate_decoded_s0;
            dec_r2_imm_0  <= source_b_immediate_decoded_s0;
            dec_r1_immv_0 <= source_a_immediate_value_decoded_s0;
            dec_r2_immv_0 <= source_b_immediate_value_decoded_s0;
        end
        if (dq1_pop) begin
            dec_class_1   <= class_decoded_s1;
            dec_we_1      <= write_enable_decoded_s1;
            dec_rd_1      <= destination_pick_decoded_s1;
            dec_is_st_1   <= is_store_decoded_s1;
            dec_mulop_1   <= multiply_operation_decoded_s1;
            dec_lsf3_1    <= load_store_function3_decoded_s1;
            dec_func4_1   <= function4_decoded_s1;
            dec_csrf3_1   <= csr_function3_decoded_s1;
            dec_csrw_en_1 <= csr_write_enable_decoded_s1;
            dec_csrw_wr_1 <= csr_write_commit_decoded_s1;
            dec_csra_1    <= csr_address_decoded_s1;
            dec_jval_1    <= jump_value_decoded_s1;
            dec_brf_1     <= branch_flag_decoded_s1;
            dec_jalf_1    <= jump_link_flag_decoded_s1;
            dec_jalfr_1   <= jump_link_register_flag_decoded_s1;
            dec_bjuf4_1   <= branch_jump_function4_decoded_s1;
            dec_r1_imm_1  <= source_a_immediate_decoded_s1;
            dec_r2_imm_1  <= source_b_immediate_decoded_s1;
            dec_r1_immv_1 <= source_a_immediate_value_decoded_s1;
            dec_r2_immv_1 <= source_b_immediate_value_decoded_s1;
        end
    end

//==============================================================================
//  二、两条 lane 的交付寄存器（isq 的 D 级）
//     dsq 口给的是"下一条"；头寄存器是当前正被 isq 看着的那条。
//     头被收走 / 头是空的 ⇒ 从 dsq 口装一条（同时 pop）。
//==============================================================================
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

    assign dq0_pop = (~payload_valid_out_0 | accept_in_0) & dq0_valid;
    assign dq1_pop = (~payload_valid_out_1 | accept_in_1) & dq1_valid;

    always @(posedge clk) begin
        if (reset_registered) begin
            payload_valid_out_0 <= 1'b0;
            payload_fresh_out_0 <= 1'b0;
            payload_valid_out_1 <= 1'b0;
            payload_fresh_out_1 <= 1'b0;
        end
        else if (redir_backend_1) begin
            payload_valid_out_0 <= 1'b0;
            payload_fresh_out_0 <= 1'b0;
            payload_valid_out_1 <= 1'b0;
            payload_fresh_out_1 <= 1'b0;
        end
        else begin
            payload_valid_out_0 <= dq0_pop ? 1'b1 : (~accept_in_0 & payload_valid_out_0);
            payload_fresh_out_0 <= dq0_pop;
            payload_valid_out_1 <= dq1_pop ? 1'b1 : (~accept_in_1 & payload_valid_out_1);
            payload_fresh_out_1 <= dq1_pop;
        end
    end

    always @(posedge clk) begin
        if (dq0_pop) begin
            instruction_out_0              <= p0_inst;
            auxiliary_address_out_0        <= p0_addr;
            branch_prediction_taken_out_0  <= p0_bp;
            source_a_out_0                 <= p0_rs1;
            source_b_out_0                 <= p0_rs2;
            index_out_0                    <= p0_idx;
            generation_out_0               <= p0_gen;
            phys_a_out_0                   <= p0_pa;
            phys_b_out_0                   <= p0_pb;
            phys_d_out_0                   <= p0_pd;
        end
        if (dq1_pop) begin
            instruction_out_1              <= p1_inst;
            auxiliary_address_out_1        <= p1_addr;
            branch_prediction_taken_out_1  <= p1_bp;
            source_a_out_1                 <= p1_rs1;
            source_b_out_1                 <= p1_rs2;
            index_out_1                    <= p1_idx;
            generation_out_1               <= p1_gen;
            phys_a_out_1                   <= p1_pa;
            phys_b_out_1                   <= p1_pb;
            phys_d_out_1                   <= p1_pd;
        end
    end

    always @(*) begin
        lane_valid_0 = payload_valid_out_0;
        lane_valid_1 = payload_valid_out_1;
    end

//==============================================================================
//  三、LSU FIFO：访存条按程序序排，队头给 isq 的访存站
//     入口不再是"译码这一拍"，而是 dsq 的访存口；满了就让那条留在 dsq 里等。
//     条目 98 位：[97:92] 目的物理号 [91:87] rs1(架构) [86:82] rs2(架构)
//                 [81:50] 指令 [49:18] 指令地址+4 [17:13] rob 槽 [12] 世代
//                 [11:6] 源 A 物理号 [5:0] 源 B 物理号
//==============================================================================
    localparam integer LSU_QUEUE_WIDTH = 98;

    reg [LSU_QUEUE_WIDTH-1:0] lsu_queue_0, lsu_queue_1, lsu_queue_2, lsu_queue_3;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_queue_4, lsu_queue_5, lsu_queue_6, lsu_queue_7;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_clear_0, lsu_clear_1, lsu_clear_2, lsu_clear_3;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_clear_4, lsu_clear_5, lsu_clear_6, lsu_clear_7;
    (* max_fanout = 32 *) reg [3:0] lsu_queue_count;
    reg [2:0] lsu_read_pointer, lsu_write_pointer;
    reg [LSU_QUEUE_WIDTH-1:0] lsu_head;
    reg [31:0] lsu_head_instruction, lsu_head_pc;
    reg [4:0]  lsu_head_index;
    reg        lsu_head_generation;

    wire lsu_push = dqm_valid & (lsu_queue_count != 4'd8);
    assign dqm_pop = lsu_push;
    wire [LSU_QUEUE_WIDTH-1:0] lsu_entry_m = {pm_pd, pm_rs1, pm_rs2, pm_inst, pm_addr,
                                              pm_idx, pm_gen, pm_pa, pm_pb};

    always @(posedge clk) begin
        if (reset_registered) begin
            lsu_queue_0 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_1 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_2 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_3 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_4 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_5 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_6 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_7 <= {LSU_QUEUE_WIDTH{1'b0}};
            lsu_queue_count <= 4'd0;
            lsu_read_pointer <= 3'd0;
            lsu_write_pointer <= 3'd0;
        end
        else if (redir_backend_1) begin
            lsu_queue_count   <= 4'd0;
            lsu_write_pointer <= lsu_read_pointer;
        end
        else begin
            lsu_clear_0 = lsu_queue_0;
            lsu_clear_1 = lsu_queue_1;
            lsu_clear_2 = lsu_queue_2;
            lsu_clear_3 = lsu_queue_3;
            lsu_clear_4 = lsu_queue_4;
            lsu_clear_5 = lsu_queue_5;
            lsu_clear_6 = lsu_queue_6;
            lsu_clear_7 = lsu_queue_7;
            if (lsu_push) begin
                case (lsu_write_pointer)
                    3'd0: lsu_clear_0 = lsu_entry_m;
                    3'd1: lsu_clear_1 = lsu_entry_m;
                    3'd2: lsu_clear_2 = lsu_entry_m;
                    3'd3: lsu_clear_3 = lsu_entry_m;
                    3'd4: lsu_clear_4 = lsu_entry_m;
                    3'd5: lsu_clear_5 = lsu_entry_m;
                    3'd6: lsu_clear_6 = lsu_entry_m;
                    default: lsu_clear_7 = lsu_entry_m;
                endcase
            end
            lsu_queue_0 <= lsu_clear_0;
            lsu_queue_1 <= lsu_clear_1;
            lsu_queue_2 <= lsu_clear_2;
            lsu_queue_3 <= lsu_clear_3;
            lsu_queue_4 <= lsu_clear_4;
            lsu_queue_5 <= lsu_clear_5;
            lsu_queue_6 <= lsu_clear_6;
            lsu_queue_7 <= lsu_clear_7;
            if (lsu_push) lsu_write_pointer <= lsu_write_pointer + 3'd1;
            if (lsu_pop_in & (lsu_queue_count != 4'd0))
                lsu_read_pointer <= lsu_read_pointer + 3'd1;
            lsu_queue_count <= lsu_queue_count + {3'd0, lsu_push}
                             - {3'd0, (lsu_pop_in & (lsu_queue_count != 4'd0))};
        end
    end

    always @(*) begin
        lsu_head = lsu_read_pointer == 3'd0 ? lsu_queue_0 :
                   lsu_read_pointer == 3'd1 ? lsu_queue_1 :
                   lsu_read_pointer == 3'd2 ? lsu_queue_2 :
                   lsu_read_pointer == 3'd3 ? lsu_queue_3 :
                   lsu_read_pointer == 3'd4 ? lsu_queue_4 :
                   lsu_read_pointer == 3'd5 ? lsu_queue_5 :
                   lsu_read_pointer == 3'd6 ? lsu_queue_6 : lsu_queue_7;
        lsu_head_instruction = lsu_head[81:50];
        lsu_head_pc          = lsu_head[49:18];
        lsu_head_index       = lsu_head[17:13];
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
    assign lsu_valid_out                = (lsu_queue_count != 4'd0);
    assign lsu_phys_d_out               = lsu_head[97:92];
    assign lsu_phys_a_out               = lsu_head[11:6];
    assign lsu_phys_b_out               = lsu_head[5:0];

endmodule
