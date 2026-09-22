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
    wire icache_busy_w, icache_busy_q;
    wire br1, br2, br3;

    (* max_fanout = 32 *) wire [4:0] rs1_1, rs2_1, rd_1;
    wire [4:0] imm5_csr_1;
    wire [6:0] opcode_1, opcode_lsu_1;
    wire [9:0] func10_1, func10_lsu_1;
    wire [31:0] imm_alu_1, pc_operand_1;
    wire [11:0] imm12_csr_1;
    wire [31:0] offset_jalr0_1, offset_beq0_aux_1, offset_load0_1, offset_store0_1;
    wire [31:0] aux_addr_1;
    wire dec, lsu, jal, br_en, pre_jalr;
    wire br_pred_taken_1;
    wire [31:0] jalr_pred_addr_1;

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
    wire [31:0] r1_data_3, r2_data_3, aux_addr_3, beq_off_q2, jalr_pred_addr_3;
    wire we_3, br_flag, br_pred_taken_3;

    wire [4:0] rd_4;
    wire [31:0] result_4, result_back1;
    wire we_4;

    wire [4:0] rd_5;
    wire [31:0] result_5, result_back2;
    wire we_5;

//非流水线层次块（核内控制信号和其他信号）：按信号首生产者所在模块的代码位置排序
    wire [4:0] flag_bus;
    wire [3:0] irq_bubble, alu_func4;
    wire [11:0] csr_addr, csr_addr_pre;
    wire [4:0] rd_back1, rd_back2;
    wire [31:0] jalr_predict_offset;
    wire [31:0] isr_addr1, isr_addr2, iret_addr1, iret_addr2;
    wire [31:0] offset_jal1, offset_jal2, offset_beq1, offset_beq2;
    wire [31:0] jalr_target_q2, jp_target;
    wire [5:0] br_pc_idx;
    wire [31:0] r1_data_final, r2_data_final;
    wire [31:0] r1_data, r2_data;
    wire [31:0] ld_data_final;
    wire br_fail, success, jalr_fail, jal_flag, jalr_flag, jalr_flag_q, br_pred_taken_q, irq_ret, trap, ebreak;
//前置冲刷（早一拍）：由 bju 的组合判定给出，直连 lsu/mulu（不进 flag_bus，见 controller.v）
    wire stallf;
    wire loaded, ld_we, stall;
    wire [4:0] rd_load;
    wire btb_hit;
    wire [31:0] icache_mem_addr_w, icache_mem_wdata_w, icache_mem_data_w;
    wire [3:0] icache_mem_be_w;
    wire icache_mem_req_w, icache_mem_we_w, icache_mem_valid_w;
    wire csr_wr_en, timi, irq_act, irq_processing, irq;
    wire [2:0] csr_func3;
    wire [31:0] csr_data_wr, csr_data_rd, mcause, csr_result;
    wire [31:0] tim_data_out;
    wire tim_ready;
    wire [31:0] dcache_data_w, dcache_mem_addr_w, dcache_mem_wdata_w, dcache_mem_data_w;
    wire [3:0] dcache_mem_be_w;
    wire dcache_busy_w, dcache_ld_ready_w, dcache_mem_req_w, dcache_mem_we_w, dcache_mem_valid_w;
//dcache 的 busy 延拓拍（原来在顶层合成 d_hold_int，现在由 dcache 自己输出）
    wire dcache_hold_w;
//RV32M 乘除法单元（与 lsu 流水线同步，独立第三写口）
    wire [31:0] mul_data_final;
    wire mul_loaded, mul_we, mul_stall;
    wire [4:0] rd_mul;
    wire ibus_req_valid_i;

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
        .irq(irq),
        .irq_ret(irq_ret),
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
        .offset_jal1(offset_jal1),
        .offset_beq1(offset_beq1),
        .isr_addr1(isr_addr1),
        .isr_ret_addr1(iret_addr1),
        .br1(br1),
        .br2(br2),
        .br3(br3),
        .jal(jal),
        .pre_jalr(pre_jalr),
        .btb_hit(btb_hit),
        .jalr_fail(jalr_fail),
        .irq(irq),
        .irq_ret(irq_ret),
        .flag_bus(flag_bus),
        .req_valid(ibus_req_valid_i),
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
        .br1_in(br1),
        .jalr_pred_addr_in(jalr_predict_offset),
        .br_pred_taken_out(br_pred_taken_1),
        .jalr_pred_addr_out(jalr_pred_addr_1),
        .r1(rs1_1),
        .r2(rs2_1),
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
        .flag_bus(flag_bus),
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
        .br_pred_taken_in(br_pred_taken_1),
        .br_pred_taken_out(br_pred_taken_2),
        .jalr_pred_addr_in(jalr_pred_addr_1),
        .jalr_pred_addr_out(jalr_pred_addr_2),
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
        .r1_data_final(r1_data_final),
        .r2_data_final(r2_data_final),
        .rd_in(rd_2),
        .opcode(opcode_2),
        .offset_jalr0(offset_jalr0_2),
        .offset_beq0_aux(offset_beq0_aux_2),
        .pc_operand_in(pc_operand_2),
        .aux_addr_in(aux_addr_2),
        .br_pred_taken_in(br_pred_taken_2),
        .jalr_pred_addr_in(jalr_pred_addr_2),
        .br_flag(br_flag),
        .beq_off_q2(beq_off_q2),
        .br_pred_taken_out(br_pred_taken_3),
        .jalr_pred_addr_out(jalr_pred_addr_3),
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

//分支/跳转判定单元：判定源与 alu 的输入同源（r1_data_3 / r2_data_3 / alu_func4），
//比较与加法在本拍做，结果、落点在下一拍生效；stallf 是同一判定的组合版本、早一拍
    bju u_bju (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .r1_data_in(r1_data_3),
        .r2_data_in(r2_data_3),
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
        .br_pc_idx(br_pc_idx),
        .jalr_flag_q(jalr_flag_q),
        .br_pred_taken_q(br_pred_taken_q),
        .stallf(stallf)
    );

    lsu u_lsu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .stallf(stallf),
        .opcode(opcode_lsu_2),
        .func10(func10_lsu_2),
        .rd_in(rd_2),
        .r1_post(rs1_2),
        .r2_post(rs2_2),
        .r1_data_final(r1_data_final),
        .r2_data_final(r2_data_final),
        .offset_load0(offset_load0_2),
        .offset_store0(offset_store0_2),
        .bus_data_ext(bus_data_in_ext),
        .bus_data_dcache(dcache_data_w),
        .bus_data_tim(tim_data_out),
        .ready_dcache(dcache_ld_ready_w),
        .ready_tim(tim_ready),
        .ready_ext(bus_loaded_in),
        .bus_hold_in(bus_hold_in),
        .dcache_hold(dcache_hold_w),
        .bus_addr_out(bus_addr_out_i),
        .bus_data_out(bus_data_out_i),
        .bus_be_out(bus_be_out_i),
        .bus_we_out(bus_we_out_i),
        .bus_valid_out(bus_valid_out_i),
        .ld_data_out(ld_data_final),
        .loaded(loaded),
        .ld_we(ld_we),
        .stall(stall),
        .rd_load(rd_load)
    );

//RV32M：与 lsu 同拍取 mem_buf 输出，自己从 opcode/func10 判 M（不用额外派发标志）
    mulu u_mulu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .stallf(stallf),
        .lsu_stall(stall),
        .icache_busy(icache_busy_q),
        .bus_hold_in(bus_hold_in),
        .dcache_hold(dcache_hold_w),
        .opcode(opcode_2),
        .func10(func10_2),
        .rd_in(rd_2),
        .r1_post(rs1_2),
        .r2_post(rs2_2),
//用 mulu 自己那一份镜像（_mul）：它由 pre_decoder 的 mul 标志单独填充，
//既保证 M 指令一定拿到操作数（_dec/_lsu 是按各自的标志填的），又把扇出按消费者拆开。
        .r1_data_final(r1_data_final),
        .r2_data_final(r2_data_final),
        .mul_data_out(mul_data_final),
        .mul_loaded(mul_loaded),
        .mul_we(mul_we),
        .rd_mul(rd_mul),
        .stall(mul_stall)
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
        .rd_load(rd_load),
        .ld_data(ld_data_final),
        .loaded(loaded),
        .rd_mul(rd_mul),
        .mul_data(mul_data_final),
        .mul_loaded(mul_loaded),
        .lsu_stall(stall),
        .mul_stall(mul_stall),
        .r1_data_in(r1_data),
        .r2_data_in(r2_data),
        .r1_data_final(r1_data_final),
        .r2_data_final(r2_data_final)
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

    alu u_alu (
        .we_in(we_3),
        .jal_flag(jal_flag),
        .jalr_flag(jalr_flag),
        .cs_wr_en(csr_wr_en),
        .rd_in(rd_3),
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
        .flag_bus(flag_bus),
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
        .flag_bus(flag_bus),
        .r1(rs1_1),
        .r2(rs2_1),
        .rd_alu(rd_5),
        .rd_data_alu(result_5),
        .we_alu(we_5),
        .rd_ld(rd_load),
        .ld_data_ld(ld_data_final),
        .we_ld(ld_we),
        .rd_mul(rd_mul),
        .mul_data_mul(mul_data_final),
        .we_mul(mul_we),
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
        .irq_ret(irq_ret),
        .trap(trap),
        .ebreak(ebreak),
        .lsu_stall(stall),
        .mul_stall(mul_stall),
        .bus_hold_in(bus_hold_in),
        .dcache_hold(dcache_hold_w),
        .icache_busy(icache_busy_q),
        .csr_wr_en(csr_wr_en),
        .exti(exti),
        .timi(timi),
        .softi(1'b0),
        .csr_addr(csr_addr),
        .csr_addr_pre(csr_addr_pre),
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
        .flag_bus(flag_bus),
        .req_valid(ibus_req_valid_i)
    );


endmodule
