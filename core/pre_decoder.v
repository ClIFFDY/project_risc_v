`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/10/04
// Design Name:
// Module Name: pre_decoder
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   取指队列**读侧**的译码级（原 mid_decoder，2026-10-27 改名并接回译码）。
//
//   ★ 结构：icache（交付寄存器）→ fetch_fifo（槽：原始指令字 + 携带量）→ 本级（译码 + 寄存）
//     → post_decoder。原来夹在 icache 与队列之间的那一级 pre_decoder（译码 + 寄存）已删 ——
//     队列的槽本身就是"把指令存住"的寄存器，再在它前面放一级寄存器，就是把同一条信息存两遍。
//
//   ★ 队列条目里只存【队列取消不了的携带量】：inst / addr / br_pred / jalr_pred。
//     后三样是前端针对这一次取指产生的量（addr 是 pc 值、br_pred/jalr_pred 是 bra_predict 的
//     表输出），后面谁也算不出来；而 rd / func10 / imm / r1 / r2 / 类位全部是指令字的纯函数，
//     在本级就地解 —— 与队头同拍，不多花时间，也不占队列宽度。
//
//   ★ 成对判据长在这里：拿队头两条的类位 + "head0 的 rd 是不是 head1 的 rs" 判，
//     判出来回给队列当弹出条数（pop2）。队列写侧不必知道后端怎么配对。
//
//   ★ 队头两个 rs 另有一份【组合】输出（r1_c/r2_c/r1_1_c/r2_1_c）：rob 的槽扫描吃它，
//     要求与队头同拍、且起点是普通触发器（不能用 icache 的 BRAM 输出寄存器 —— 那 2.45ns 的
//     clock-to-out 是白吃的）。队列槽正是普通 FF。
//
//   ★ lane1（一个包的第二条）：与 lane0 同一套寄存器组、同一个推进条件。lane1_v_out 为 0 时
//     它整组无消费者，综合会把它连同下游一起折掉（= 一宽机器的代价）。
//
// Dependencies:
//
// Revision:
//   Revision 0.01 - 重新引入（纯寄存版）
//   Revision 0.02 - 加 lane1 携带组
//   Revision 0.03 - 接回译码：吃队列的原始字，不再吃上游译码结果
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module pre_decoder(
    input clk, rst,
    input [11:0] flag_bus,
//取指队列的队头两条（原始指令字 + 携带量，条目布局见 fetch_fifo 模块头）。
//h1_inst 在 h1_v=0 时被队列侧压成 0，不会把未写过槽的 X 带进来。
    input [31:0] h0_inst, h1_inst,
    input [31:0] h0_addr,
    input        h0_br_pred,
    input [31:0] h0_jalr_pred,
    input        h0_v, h1_v,
//队头两条的源寄存器号：条目里存好的字段（在 fifo 前端解的，见那边的说明）。
//★ 本级不再自己解 rs：rob 的扫槽与队头同拍、而且要求起点短 —— 现解就多一级译码锥。
    input [4:0]  h0_r1, h0_r2, h1_r1, h1_r2,
    output reg [31:0] inst_out,
    output reg [4:0]  rd_out,
    output reg [9:0]  func10_out,
    output reg [31:0] imm_alu_out,
    output reg [31:0] aux_addr_out,
    output reg        br_pred_taken_out,
    output reg [31:0] jalr_pred_addr_out,
    output reg [4:0]  r1_out, r2_out,
    output reg [31:0] inst1_out,
    output reg [4:0]  rd1_out,
    output reg [9:0]  func10_1_out,
    output reg [31:0] imm1_alu_out,
    output reg [4:0]  r1_1_out, r2_1_out,
    output reg        lane1_v_out,
//本拍弹两条（回给取指队列）。与 lane1_v_out 同源：能配才弹两条。
    output reg        pop2
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
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_SYSTEM = 7'b1110011;

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

//队头 lane0 的译码（rd / func10 / imm）：口径与原来那一级逐条一致。
    reg [4:0]  rd_c;
    reg [9:0]  func10_c;
    reg [31:0] imm_c;
    always @(*) begin
        rd_c = 5'd0;
        func10_c = 10'd0;
        imm_c = 32'd0;
        case (h0_inst[6:0])
            OPCODE_OP: begin
                rd_c = h0_inst[11:7];
                func10_c = {h0_inst[31:25], h0_inst[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_c = immI(h0_inst);
                rd_c = h0_inst[11:7];
                func10_c = {(h0_inst[14:12] == 3'b001 || h0_inst[14:12] == 3'b101) ? h0_inst[31:25] : 7'd0,
                            h0_inst[14:12]};
            end
            OPCODE_JAL: begin
                rd_c = h0_inst[11:7];
            end
            OPCODE_JALR: begin
                rd_c = h0_inst[11:7];
                imm_c = immI(h0_inst);
            end
            OPCODE_BRANCH: begin
                imm_c = immB(h0_inst);
                func10_c = {7'd0, h0_inst[14:12]};
            end
            OPCODE_LOAD: begin
                imm_c = immI(h0_inst);
                rd_c = h0_inst[11:7];
            end
            OPCODE_STORE: begin
                imm_c = immS(h0_inst);
            end
            OPCODE_LUI: begin
                imm_c = immU(h0_inst);
                rd_c = h0_inst[11:7];
            end
            OPCODE_AUIPC: begin
                imm_c = immU(h0_inst) - 4'd4;
                rd_c = h0_inst[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_c = h0_inst[11:7];
                func10_c = {7'd0, h0_inst[14:12]};
            end
        endcase
    end

//队头 lane1 的译码：与 lane0 同一套口径、同样覆盖全部 opcode。
//★ 为什么必须译全：队列条目里任何一条都可能被提成 lane0（它前面那条被单独弹走之后），
//  只解"ALU 类四种"的话，别的指令（LOAD/JAL/BRANCH/CSR…）带着 rd=0/func10=0/imm=0
//  走到下游就是**一条错指令**。
    reg [4:0]  rd1_c;
    reg [9:0]  func10_1_c;
    reg [31:0] imm1_c;
    always @(*) begin
        rd1_c = 5'd0;
        func10_1_c = 10'd0;
        imm1_c = 32'd0;
        case (h1_inst[6:0])
            OPCODE_OP: begin
                rd1_c = h1_inst[11:7];
                func10_1_c = {h1_inst[31:25], h1_inst[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm1_c = immI(h1_inst);
                rd1_c = h1_inst[11:7];
                func10_1_c = {(h1_inst[14:12] == 3'b001 || h1_inst[14:12] == 3'b101) ? h1_inst[31:25] : 7'd0,
                              h1_inst[14:12]};
            end
            OPCODE_JAL: begin
                rd1_c = h1_inst[11:7];
            end
            OPCODE_JALR: begin
                rd1_c = h1_inst[11:7];
                imm1_c = immI(h1_inst);
            end
            OPCODE_BRANCH: begin
                imm1_c = immB(h1_inst);
                func10_1_c = {7'd0, h1_inst[14:12]};
            end
            OPCODE_LOAD: begin
                imm1_c = immI(h1_inst);
                rd1_c = h1_inst[11:7];
            end
            OPCODE_STORE: begin
                imm1_c = immS(h1_inst);
            end
            OPCODE_LUI: begin
                imm1_c = immU(h1_inst);
                rd1_c = h1_inst[11:7];
            end
            OPCODE_AUIPC: begin
                imm1_c = immU(h1_inst) - 4'd4;
                rd1_c = h1_inst[11:7];
            end
            OPCODE_SYSTEM: begin
                rd1_c = h1_inst[11:7];
                func10_1_c = {7'd0, h1_inst[14:12]};
            end
        endcase
    end

//成对判据（读侧）：两条都在册 ∧ head0 能当 lane0 ∧ head1 能当 lane1 ∧ 包里没有 RAW。
//★ 为什么 lane1 只能是 {OP/OP-IMM/LUI 非 M}：副路只复刻了一个 alu2，post_decoder 的 lane1
//  那一支在 case 之前就置 we1、且只处理这三种 ⇒ 别的类被当成 lane1 配对发射，就是往一个垃圾
//  寄存器写垃圾值、控制转移还不生效。配不上不是"慢一点"，是**必须不配**。
//★ 包内 RAW：head0 真要写寄存器、而且 head1 读的就是它 —— 队列里的前一条就是程序序上的
//  前一条，所以这一判比原来"查表 + 行首特判"更准。
    reg m0_c, m1_c, cls0_c, cls1_1c, raw_pair, lane1_v_in;
    always @(*) begin
        m0_c    = (h0_inst[6:0] == OPCODE_OP) && (h0_inst[31:25] == 7'b0000001);
        m1_c    = (h1_inst[6:0] == OPCODE_OP) && (h1_inst[31:25] == 7'b0000001);
        cls0_c  = (((h0_inst[6:0] == OPCODE_OP) || (h0_inst[6:0] == OPCODE_OP_IMM)
                 || (h0_inst[6:0] == OPCODE_LUI) || (h0_inst[6:0] == OPCODE_AUIPC)) && !m0_c)
               || (h0_inst[6:0] == OPCODE_LOAD) || (h0_inst[6:0] == OPCODE_STORE);
        cls1_1c = ((h1_inst[6:0] == OPCODE_OP) || (h1_inst[6:0] == OPCODE_OP_IMM)
                || (h1_inst[6:0] == OPCODE_LUI)) && !m1_c;
        raw_pair = 1'b0;
        if (rd_c != 5'd0) begin
            if (rd_c == h1_r1)
                raw_pair = 1'b1;
            if (rd_c == h1_r2)
                raw_pair = 1'b1;
        end
        lane1_v_in = h0_v & h1_v & cls0_c & cls1_1c & ~raw_pair;
        pop2       = lane1_v_in;
    end

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
//★ 本级站在【后端】：停顿位要【全吃】。取指侧那一套只吃 [6]|[1]（后端停顿由队列吸收），
//  这一级要是照抄那一套，就会在 ROB 满 / lsu / mulu / bus 停顿时照样锁存新指令 ⇒
//  载荷与队头错位（队列没弹，本级却换了内容）。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[11] | flag_bus[10] | flag_bus[9];
        stall_w = (flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4] | flag_bus[3]
                 | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//推进/保持/清：与取指队列的读口【逐字同形】。
//★ 条件必须与队列一致：本级的"保持"要跟队列的"弹/不弹"落在同一拍，差一拍就是整条流水线错位。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out <= 32'd0;
            rd_out <= 5'd0;
            func10_out <= 10'd0;
            imm_alu_out <= 32'd0;
            aux_addr_out <= 32'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
            r1_out <= 5'd0;
            r2_out <= 5'd0;
            inst1_out <= 32'd0;
            rd1_out <= 5'd0;
            func10_1_out <= 10'd0;
            imm1_alu_out <= 32'd0;
            r1_1_out <= 5'd0;
            r2_1_out <= 5'd0;
            lane1_v_out <= 1'b0;
        end
        else if (exec) begin
            if (!flush_w && !stall_w) begin
                inst_out <= h0_inst;
                rd_out <= rd_c;
                func10_out <= func10_c;
                imm_alu_out <= imm_c;
                aux_addr_out <= h0_addr;
                br_pred_taken_out <= h0_br_pred;
                jalr_pred_addr_out <= h0_jalr_pred;
                r1_out <= h0_r1;
                r2_out <= h0_r2;
                inst1_out <= h1_inst;
                rd1_out <= rd1_c;
                func10_1_out <= func10_1_c;
                imm1_alu_out <= imm1_c;
                r1_1_out <= h1_r1;
                r2_1_out <= h1_r2;
                lane1_v_out <= lane1_v_in;
            end
            else if (stall_w) begin
                inst_out <= inst_out;
                rd_out <= rd_out;
                func10_out <= func10_out;
                imm_alu_out <= imm_alu_out;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
                r1_out <= r1_out;
                r2_out <= r2_out;
                inst1_out <= inst1_out;
                rd1_out <= rd1_out;
                func10_1_out <= func10_1_out;
                imm1_alu_out <= imm1_alu_out;
                r1_1_out <= r1_1_out;
                r2_1_out <= r2_1_out;
                lane1_v_out <= lane1_v_out;
            end
            else begin
                inst_out <= 32'd0;
                rd_out <= 5'd0;
                func10_out <= 10'd0;
                imm_alu_out <= 32'd0;
                aux_addr_out <= 32'd0;
                br_pred_taken_out <= 1'b0;
                jalr_pred_addr_out <= 32'd0;
                r1_out <= 5'd0;
                r2_out <= 5'd0;
                inst1_out <= 32'd0;
                rd1_out <= 5'd0;
                func10_1_out <= 10'd0;
                imm1_alu_out <= 32'd0;
                r1_1_out <= 5'd0;
                r2_1_out <= 5'd0;
                lane1_v_out <= 1'b0;
            end
        end
    end

endmodule
