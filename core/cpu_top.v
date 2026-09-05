`timescale 1ns / 1ps

module cpu_top(
    input clk, rst,
    //
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out,
    output reg [31:0] ibus_addr_out,
    output reg ibus_re_out,
    output reg ibus_req_valid,
    input [31:0] ibus_data_in,
    input [15:0] ibus_addr_in,
    input [31:0] bus_data_in_ext,
    input ibus_we_in,
    input bus_loaded_in,
    input exti,
    input i_busy
    );

    wire [1:0] stage;
    wire [3:0] irq_bubble, alu_func4;
    wire [11:0] csr_addr;
    wire [4:0] rd_back1, rd_back2;

    wire [31:0] pc_addr, aux_addr_0, br_addr1, br_addr2;
    wire [31:0] inst_raw;
    wire [31:0] jalr_predict_offset;
    wire [31:0] isr_addr1, isr_addr2, iret_addr1, iret_addr2;
    wire [31:0] offset_jal1, offset_jal2, offset_beq1, offset_beq2;
    wire [31:0] jalr_target_q, beq_off_q1, beq_off_q2;
    wire br1, br2, br3, jalr_fail, is_ibus_q;

    wire [4:0] rs1_1, rs2_1, rd_1;
    wire [4:0] imm5_csr_1;
    wire [6:0] opcode_1, opcode_lsu_1;
    wire [9:0] func10_1, func10_lsu_1;
    wire [31:0] imm_alu_1, pc_operand_1;
    wire [11:0] imm12_csr_1;
    wire [31:0] offset_jalr0_1, offset_beq0_aux_1, offset_load0_1, offset_store0_1;
    wire [31:0] aux_addr_1;
    wire dec, lsu, jal, br_en, pre_jalr;

    wire [4:0] rs1_2, rs2_2, rd_2;
    wire [4:0] imm5_csr_2;
    wire [6:0] opcode_2, opcode_lsu_2;
    wire [9:0] func10_2, func10_lsu_2;
    wire [31:0] imm_alu_2, pc_operand_2;
    wire [11:0] imm12_csr_2;
    wire [31:0] offset_jalr0_2, offset_beq0_aux_2, offset_load0_2, offset_store0_2;
    wire [31:0] aux_addr_2;

    wire [4:0] rd_3, lsu_rd_3;
    wire [31:0] r1_data_3, r2_data_3, aux_addr_3;
    wire we_3, lsu_we_3;
    wire [31:0] r1_data_final_dec, r2_data_final_dec, r1_data_final_lsu, r2_data_final_lsu;
    wire [31:0] r1_data_dec, r2_data_dec, r1_data_lsu, r2_data_lsu;
    wire [31:0] ld_data_final;
    wire jalr, br_fail, success, jal_flag, jalr_flag, irq_ret, trap, ebreak;
    wire loaded, stall;

    wire [4:0] rd_4;
    wire [31:0] result_4, result_back1;
    wire we_4;

    wire [4:0] rd_5;
    wire [31:0] result_5, result_back2;
    wire we_5;

    wire csr_wr_en, timi, irq_act, irq_processing, irq;
    wire [2:0] csr_func3;
    wire [31:0] csr_data_wr, csr_data_rd, mcause, csr_result;
    wire [31:0] dtcm_data_out, tim_data_out;
    wire dtcm_loaded, tim_loaded;
    wire btb_hit;

    reg softi, retire_w, jalr_pred;
    reg [31:0] bus_data_in_final;
    wire [31:0] bus_addr_out_i, bus_data_out_i;
    wire [3:0] bus_be_out_i;
    wire bus_we_out_i;
    wire [31:0] ibus_addr_out_i;
    wire ibus_re_out_i, ibus_req_valid_i;

    always @(*) begin
        jalr_pred = pre_jalr & btb_hit;
        bus_data_in_final = bus_data_in_ext | dtcm_data_out | tim_data_out;
        softi = 1'b0;
        retire_w = (stage == 2'd1);
    end

    always @(*) begin
        bus_addr_out = bus_addr_out_i;
        bus_data_out = bus_data_out_i;
        bus_be_out = bus_be_out_i;
        bus_we_out = bus_we_out_i;
        ibus_addr_out = ibus_addr_out_i;
        ibus_re_out = ibus_re_out_i;
        ibus_req_valid = ibus_req_valid_i;
    end

    pc u_pc (
        .clk(clk),
        .rst(rst),
        .br1(br1),
        .br2(br2),
        .br3(br3),
        .jal(jal),
        .jalr(jalr_pred),
        .jalr_fail(jalr_fail),
        .irq(irq),
        .irq_ret(irq_ret),
        .stage(stage),
        .offset_jal2(offset_jal2),
        .offset_jalr2(jalr_predict_offset + 4'd4),
        .offset_beq2(offset_beq2),
        .jalr_target_q(jalr_target_q),
        .beq_off_q1(beq_off_q1),
        .br_addr2(br_addr2),
        .isr_addr2(isr_addr2),
        .isr_ret_addr2(iret_addr2),
        .pc_addr(pc_addr),
        .aux_addr(aux_addr_0)
    );

    itcm u_itcm (
        .clk(clk),
        .rst(rst),
        .stage(stage),
        .pc_addr(pc_addr),
        .offset_jal1(offset_jal1),
        .offset_jalr1(jalr_predict_offset),
        .offset_beq1(offset_beq1),
        .isr_addr1(isr_addr1),
        .isr_ret_addr1(iret_addr1),
        .br1(br1),
        .br2(br2),
        .br3(br3),
        .jal(jal),
        .jalr(jalr_pred),
        .jalr_fail(jalr_fail),
        .irq(irq),
        .irq_ret(irq_ret),
        .jalr_target_q(jalr_target_q),
        .beq_off_q2(beq_off_q2),
        .br_addr1(br_addr1),
        .inst_raw_out(inst_raw),
        .is_ibus_q(is_ibus_q),
        .fetch_addr(ibus_addr_out_i),
        .ibus_re_out(ibus_re_out_i),
        .ibus_addr_in(ibus_addr_in),
        .ibus_data_in(ibus_data_in),
        .ibus_we_in(ibus_we_in)
    );

    pre_decoder u_pre_decoder (
        .clk(clk),
        .rst(rst),
        .stage(stage),
        .inst_raw_in(inst_raw),
        .is_ibus_in(is_ibus_q),
        .ibus_data_in(ibus_data_in),
        .aux_addr_in(aux_addr_0),
        .r1(rs1_1),
        .r2(rs2_1),
        .r1_mem(rs1_1),
        .r2_mem(rs2_1),
        .rd(rd_1),
        .func10_dec(func10_1),
        .func10_lsu(func10_lsu_1),
        .imm_alu_out(imm_alu_1),
        .imm12_csr_out(imm12_csr_1),
        .imm5_csr_out(imm5_csr_1),
        .offset_jal1(offset_jal1),
        .offset_jal2(offset_jal2),
        .offset_jalr0(offset_jalr0_1),
        .offset_beq0_aux(offset_beq0_aux_1),
        .offset_beq1(offset_beq1),
        .offset_beq2(offset_beq2),
        .offset_load0(offset_load0_1),
        .offset_store0(offset_store0_1),
        .pc_operand(pc_operand_1),
        .opcode_dec(opcode_1),
        .opcode_lsu(opcode_lsu_1),
        .aux_addr_out(aux_addr_1),
        .dec(dec),
        .lsu(lsu),
        .jal(jal),
        .br_en(br_en),
        .jalr(pre_jalr)
    );

    mem_buf u_mem_buf (
        .clk(clk),
        .rst(rst),
        .stage(stage),
        .func10_in(func10_1),
        .func10_out(func10_2),
        .imm_alu_in(imm_alu_1),
        .imm_alu_out(imm_alu_2),
        .imm12_csr_in(imm12_csr_1),
        .imm12_csr_out(imm12_csr_2),
        .imm5_csr_in(imm5_csr_1),
        .imm5_csr_out(imm5_csr_2),
        .rd_in(rd_1),
        .rd_out(rd_2),
        .opcode_in(opcode_1),
        .opcode_out(opcode_2),
        .offset_jalr0_in(offset_jalr0_1),
        .offset_jalr0_out(offset_jalr0_2),
        .offset_beq0_aux_in(offset_beq0_aux_1),
        .offset_beq0_aux_out(offset_beq0_aux_2),
        .pc_operand_in(pc_operand_1),
        .pc_operand_out(pc_operand_2),
        .aux_addr_in(aux_addr_1),
        .aux_addr_out(aux_addr_2),
        .opcode_lsu_in(opcode_lsu_1),
        .opcode_lsu_out(opcode_lsu_2),
        .func10_lsu_in(func10_lsu_1),
        .func10_lsu_out(func10_lsu_2),
        .offset_load0_in(offset_load0_1),
        .offset_load0_out(offset_load0_2),
        .offset_store0_in(offset_store0_1),
        .offset_store0_out(offset_store0_2),
        .r1_in(rs1_1),
        .r1_out(rs1_2),
        .r2_in(rs2_1),
        .r2_out(rs2_2)
    );

    decoder u_decoder (
        .clk(clk),
        .rst(rst),
        .stage(stage),
        .func10(func10_2),
        .imm_alu_in(imm_alu_2),
        .imm12_csr_in(imm12_csr_2),
        .imm5_csr_in(imm5_csr_2),
        .csr_wr_en(csr_wr_en),
        .csr_addr(csr_addr),
        .csr_data(csr_data_wr),
        .csr_func3(csr_func3),
        .r1_data_final(r1_data_final_dec),
        .r2_data_final(r2_data_final_dec),
        .rd_in(rd_2),
        .opcode(opcode_2),
        .offset_jalr0(offset_jalr0_2),
        .offset_beq0_aux(offset_beq0_aux_2),
        .pc_operand_in(pc_operand_2),
        .aux_addr_in(aux_addr_2),
        .jalr_target_q(jalr_target_q),
        .beq_off_q1(beq_off_q1),
        .beq_off_q2(beq_off_q2),
        .jalr(jalr),
        .br_fail(br_fail),
        .success(success),
        .irq_ret(irq_ret),
        .trap(trap),
        .ebreak(ebreak),
        .jal_flag(jal_flag),
        .jalr_flag(jalr_flag),
        .r1_data_out(r1_data_3),
        .r2_data_out(r2_data_3),
        .rd_out(rd_3),
        .rd_back1(rd_back1),
        .alu_func4(alu_func4),
        .we(we_3),
        .aux_addr_out(aux_addr_3)
    );

    lsu u_lsu (
        .clk(clk),
        .rst(rst),
        .stage(stage),
        .opcode(opcode_lsu_2),
        .func10(func10_lsu_2),
        .rd_in(rd_2),
        .r1_fast(rs1_1),
        .r2_fast(rs2_1),
        .r1_data_final(r1_data_final_lsu),
        .r2_data_final(r2_data_final_lsu),
        .offset_load0(offset_load0_2),
        .offset_store0(offset_store0_2),
        .bus_data_in(bus_data_in_final),
        .dtcm_loaded(dtcm_loaded | tim_loaded),
        .bus_loaded_in(bus_loaded_in),
        .bus_addr_out(bus_addr_out_i),
        .bus_data_out(bus_data_out_i),
        .bus_be_out(bus_be_out_i),
        .bus_we_out(bus_we_out_i),
        .we(lsu_we_3),
        .rd_out(lsu_rd_3),
        .ld_data_out(ld_data_final),
        .loaded(loaded),
        .stall(stall)
    );

    forw u_forw (
        .clk(clk),
        .rst(rst),
        .result_back1(result_back1),
        .result_back2(result_back2),
        .r1(rs1_2),
        .r2(rs2_2),
        .rd_back1(rd_back1),
        .rd_back2(rd_back2),
        .ld_data(ld_data_final),
        .loaded(loaded),
        .stall(stall),
        .r1_data_in_dec(r1_data_dec),
        .r2_data_in_dec(r2_data_dec),
        .r1_data_in_lsu(r1_data_lsu),
        .r2_data_in_lsu(r2_data_lsu),
        .r1_data_final_dec(r1_data_final_dec),
        .r2_data_final_dec(r2_data_final_dec),
        .r1_data_final_lsu(r1_data_final_lsu),
        .r2_data_final_lsu(r2_data_final_lsu)
    );

    bra_predict u_bra_predict (
        .clk(clk),
        .rst(rst),
        .stage(stage),
        .pc_addr_in(pc_addr),
        .jalr_target_q(jalr_target_q),
        .success(success),
        .br_fail(br_fail),
        .br_en(br_en),
        .pre_jalr(pre_jalr),
        .br_addr1(br_addr1),
        .br_addr2(br_addr2),
        .br1(br1),
        .br2(br2),
        .br3(br3),
        .jalr_predict_offset(jalr_predict_offset),
        .jalr(btb_hit),
        .jalr_fail(jalr_fail)
    );

    alu u_alu (
        .we_in(we_3 | lsu_we_3),
        .jal_flag(jal_flag),
        .jalr_flag(jalr_flag),
        .cs_wr_en(csr_wr_en),
        .rd_in(rd_3 | lsu_rd_3),
        .alu_func4(alu_func4),
        .aux_addr_in(aux_addr_3),
        .csr_func3(csr_func3),
        .r1_data(r1_data_3),
        .r2_data(r2_data_3),
        .cs_data(csr_data_rd),
        .result_csr(csr_result),
        .rd_out(rd_4),
        .result(result_4),
        .result_back1(result_back1),
        .we(we_4)
    );

    wb_reg u_wb_reg (
        .clk(clk),
        .rst(rst),
        .we_in(we_4),
        .rd_in(rd_4),
        .result_in(result_4),
        .we_out(we_5),
        .rd_out(rd_5),
        .rd_back2(rd_back2),
        .result_out(result_5),
        .result_back2(result_back2)
    );

    regfile u_regfile (
        .clk(clk),
        .rst(rst),
        .stage(stage),
        .r1(rs1_1),
        .r2(rs2_1),
        .rd(rd_5),
        .rd_data(result_5),
        .ld_data(ld_data_final),
        .we(we_5),
        .loaded(loaded),
        .dec(dec),
        .lsu(lsu),
        .r1_data_dec(r1_data_dec),
        .r2_data_dec(r2_data_dec),
        .r1_data_lsu(r1_data_lsu),
        .r2_data_lsu(r2_data_lsu)
    );

    dtcm u_dtcm (
        .clk(clk),
        .rst(rst),
        .bus_addr_in(bus_addr_out),
        .bus_data_in(bus_data_out),
        .bus_be_in(bus_be_out),
        .bus_we_in(bus_we_out),
        .bus_data_out(dtcm_data_out),
        .loaded(dtcm_loaded)
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
        .loaded(tim_loaded)
    );

    controller u_controller (
        .clk(clk),
        .rst(rst),
        .br2(br2),
        .br3(br3),
        .jalr_fail(jalr_fail),
        .jal(jal),
        .jalr_pred(jalr_pred),
        .br1(br1),
        .irq_ret(irq_ret),
        .trap(trap),
        .ebreak(ebreak),
        .stall(stall | i_busy),
        .csr_wr_en(csr_wr_en),
        .exti(exti),
        .timi(timi),
        .softi(softi),
        .retire(retire_w),
        .csr_addr(csr_addr),
        .csr_data_in(csr_result),
        .pc_addr_in(pc_addr),
        .csr_data_out(csr_data_rd),
        .isr_addr1(isr_addr1),
        .isr_addr2(isr_addr2),
        .mcause(mcause),
        .irq_act(irq_act),
        .irq_processing(irq_processing),
        .irq(irq),
        .iret_addr1(iret_addr1),
        .iret_addr2(iret_addr2),
        .irq_bubble(irq_bubble),
        .stage(stage),
        .req_valid(ibus_req_valid_i)
    );


endmodule
