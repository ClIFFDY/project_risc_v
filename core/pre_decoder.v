`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 16:02:13
// Design Name:
// Module Name: pre_decoder
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module pre_decoder(
    input clk, rst,
    input [4:0] flag_bus,
    input [31:0] inst_in,
    input [31:0] aux_addr_in,
    input br1_in,
    input [31:0] jalr_pred_addr_in,
    output reg [4:0] r1, r2, rd,
    output reg [9:0] func10_dec, func10_lsu,
    output reg [31:0] imm_alu_out,
    output reg [11:0] imm12_csr_out,
    output reg [31:0] offset_jal1, offset_jal2, offset_beq1, offset_beq2, offset_beq0_aux, offset_jalr0,
    output reg [31:0] offset_load0, offset_store0,
    output reg [31:0] pc_operand,
    output reg [31:0] aux_addr_out,
    output reg [4:0] imm5_csr_out,
    output reg [6:0] opcode_dec, opcode_lsu,
    output reg jal, dec, lsu, br_en, jalr,
    output reg br_pred_taken_out,
    output reg [31:0] jalr_pred_addr_out
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
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
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_SYSTEM = 7'b1110011;

    reg [31:0] inst_effective;

//提取不同类型指令立即数的函数块
    function [31:0] immI;
        input [31:0] inst;
        immI = {{20{inst[31]}}, inst[31:20]};
    endfunction

    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

    function [31:0] immS;
        input [31:0] inst;
        immS = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    endfunction

    function [31:0] immU;
        input [31:0] inst;
        immU = {inst[31:12], 12'd0};
    endfunction

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[3] | flag_bus[2];
        stall_w = (flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

//指令来源：icache 为唯一取指源
    always @(*) inst_effective = inst_in;

//根据不同指令类型对输入的指令进行opcode、function字段、地址和立即数拆分
//func10是RV32I指令集funct7和funct3字段的组合
    always @(posedge clk) begin
        if (rst_q) begin
            func10_dec <= 10'd0;
            func10_lsu <= 10'd0;
            r1 <= 5'd0;
            r2 <= 5'd0;
            rd <= 5'd0;
            dec <= 1'b0;
            lsu <= 1'b0;
            imm_alu_out <= 32'd0;
            offset_jalr0 <= 32'd0;
            offset_beq0_aux <= 32'd0;
            offset_load0 <= 32'd0;
            offset_store0 <= 32'd0;
            aux_addr_out <= 32'd0;
            pc_operand <= 32'd0;
            imm12_csr_out <= 12'd0;
            imm5_csr_out <= 5'd0;
            opcode_dec <= 7'd0;
            opcode_lsu <= 7'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
        end
        else if (exec) begin
            if (!flush_w && !stall_w) begin
                func10_dec <= 10'd0;
                func10_lsu <= 10'd0;
                r1 <= 5'd0;
                r2 <= 5'd0;
                rd <= 5'd0;
                dec <= 1'b0;
                lsu <= 1'b0;
                imm_alu_out <= 32'd0;
                offset_jalr0 <= 32'd0;
                offset_beq0_aux <= 32'd0;
                offset_load0 <= 32'd0;
                offset_store0 <= 32'd0;
                aux_addr_out <= 32'd0;
                pc_operand <= 32'd0;
                imm12_csr_out <= 12'd0;
                imm5_csr_out <= 5'd0;
                opcode_dec <= 7'd0;
                opcode_lsu <= 7'd0;
                br_pred_taken_out <= br1_in;
                jalr_pred_addr_out <= jalr_pred_addr_in;
                case (inst_effective[6:0])
                    OPCODE_OP: begin
                        r2 <= inst_effective[24:20];
                        r1 <= inst_effective[19:15];
                        rd <= inst_effective[11:7];
                        func10_dec <= {inst_effective[31:25], inst_effective[14:12]};
                        opcode_dec <= OPCODE_OP;
                        dec <= 1'b1;
                    end
                    OPCODE_OP_IMM: begin
                        imm_alu_out <= immI(inst_effective);
                        r1 <= inst_effective[19:15];
                        rd <= inst_effective[11:7];
                        func10_dec <= {(inst_effective[14:12] == 3'b001 || inst_effective[14:12] == 3'b101) ? inst_effective[31:25] : 7'd0, inst_effective[14:12]};
                        opcode_dec <= OPCODE_OP_IMM;
                        dec <= 1'b1;
                    end
                    OPCODE_JAL: begin
                        rd <= inst_effective[11:7];
                        opcode_dec <= OPCODE_JAL;
                        aux_addr_out <= aux_addr_in;
                    end
                    OPCODE_JALR: begin
                        rd <= inst_effective[11:7];
                        r1 <= inst_effective[19:15];
                        offset_jalr0 <= immI(inst_effective);
                        opcode_dec <= OPCODE_JALR;
                        aux_addr_out <= aux_addr_in;
                        dec <= 1'b1;
                    end
                    OPCODE_BRANCH: begin
                        opcode_dec <= OPCODE_BRANCH;
                        aux_addr_out <= aux_addr_in;
                        offset_beq0_aux <= immB(inst_effective);
                        r2 <= inst_effective[24:20];
                        r1 <= inst_effective[19:15];
                        func10_dec <= {7'd0, inst_effective[14:12]};
                        dec <= 1'b1;
                    end
                    OPCODE_LOAD: begin
                        opcode_dec <= OPCODE_LOAD;
                        opcode_lsu <= OPCODE_LOAD;
                        offset_load0 <= immI(inst_effective);
                        rd <= inst_effective[11:7];
                        r1 <= inst_effective[19:15];
                        func10_lsu <= {7'd0, inst_effective[14:12]};
                        lsu <= 1'b1;
                    end
                    OPCODE_STORE: begin
                        opcode_dec <= OPCODE_STORE;
                        opcode_lsu <= OPCODE_STORE;
                        offset_store0 <= immS(inst_effective);
                        r2 <= inst_effective[24:20];
                        r1 <= inst_effective[19:15];
                        func10_lsu <= {7'd0, inst_effective[14:12]};
                        lsu <= 1'b1;
                    end
                    OPCODE_LUI: begin
                        opcode_dec <= OPCODE_LUI;
                        opcode_lsu <= OPCODE_LUI;
                        imm_alu_out <= immU(inst_effective);
                        rd <= inst_effective[11:7];
                    end
                    OPCODE_AUIPC: begin
                        opcode_dec <= OPCODE_AUIPC;
                        imm_alu_out <= immU(inst_effective);
                        pc_operand <= aux_addr_in - 4'd4;
                        rd <= inst_effective[11:7];
                    end
                    OPCODE_SYSTEM: begin
                        opcode_dec <= OPCODE_SYSTEM;
                        imm12_csr_out <= inst_effective[31:20];
                        r1 <= inst_effective[19:15];
                        imm5_csr_out <= inst_effective[19:15];
                        rd <= inst_effective[11:7];
                        func10_dec <= {7'd0, inst_effective[14:12]};
                        dec <= 1'b1;
                    end
                endcase
            end
            else if (stall_w) begin
                func10_dec <= func10_dec;
                func10_lsu <= func10_lsu;
                r1 <= r1;
                r2 <= r2;
                rd <= rd;
                dec <= dec;
                lsu <= lsu;
                imm_alu_out <= imm_alu_out;
                offset_jalr0 <= offset_jalr0;
                offset_beq0_aux <= offset_beq0_aux;
                offset_load0 <= offset_load0;
                offset_store0 <= offset_store0;
                aux_addr_out <= aux_addr_out;
                pc_operand <= pc_operand;
                imm12_csr_out <= imm12_csr_out;
                imm5_csr_out <= imm5_csr_out;
                opcode_dec <= opcode_dec;
                opcode_lsu <= opcode_lsu;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
            end
            else begin
                func10_dec <= 10'd0;
                func10_lsu <= 10'd0;
                r1 <= 5'd0;
                r2 <= 5'd0;
                rd <= 5'd0;
                dec <= 1'b0;
                lsu <= 1'b0;
                imm_alu_out <= 32'd0;
                offset_jalr0 <= 32'd0;
                offset_beq0_aux <= 32'd0;
                offset_load0 <= 32'd0;
                offset_store0 <= 32'd0;
                aux_addr_out <= 32'd0;
                pc_operand <= 32'd0;
                imm12_csr_out <= 12'd0;
                imm5_csr_out <= 5'd0;
                opcode_dec <= 7'd0;
                opcode_lsu <= 7'd0;
                br_pred_taken_out <= 1'b0;
                jalr_pred_addr_out <= 32'd0;
            end
        end
    end

//组合透传jal、jalr和分支类预跳转地址，减少流水线空窗
//jal / br_en / offset_* 【不再受 stage 门控】：它们的消费者里有 icache 的取指地址 mux，
//而 stage = f(flush) = f(jalr_fail…)，那条广播在 15ns 下量到 2.9ns，正是当时最差路径的入口。
//"冲刷拍别拿错路指令的 br1/jal 去改取指地址"改在 icache 的 mux 上用 flush 直接挡，
//比"经 stage 编码、再在 pre_decoder 译码回来"短得多。
//jalr 保留门控：它的消费者（pc 的 EXE 分支、icache 的挡拍、controller 的 ird_tmr）都不在
//关键路上；保留它还能顺带保证"冲刷拍不就地打 NOP"（挡拍条件是 jalr|jalr_fail）。
    always @(*) begin
        br_en = 1'b0;
        offset_beq1 = 32'd0;
        offset_beq2 = 32'd0;
        offset_jal1 = 32'd0;
        offset_jal2 = 32'd0;
        jal = 1'b0;
        jalr = 1'b0;
        if (!rst_q) begin
            case (inst_effective[6:0])
                OPCODE_JAL: begin
                    offset_jal1 = $signed(immJ(inst_effective)) - 4'd4;
                    offset_jal2 = $signed(immJ(inst_effective));
                    jal = 1'b1;
                end
                OPCODE_JALR: begin
                    if (!flush_w) jalr = 1'b1;
                end
                OPCODE_BRANCH: begin
                    offset_beq1 = $signed(immB(inst_effective)) - 4'd4;
                    offset_beq2 = $signed(immB(inst_effective));
                    br_en = 1'b1;
                end
            endcase
        end
    end

endmodule
