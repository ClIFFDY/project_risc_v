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
    input [9:0] flag_bus,
    input [31:0] inst_in, inst1_in,
    input pair_fetch_ok_in,
    input [31:0] aux_addr_in,
    input br1_in,
    input [31:0] jalr_pred_addr_in,
//跨拍携带的指令字：各译码字段由下一级 mid_decoder 就地解出，本级不再各寄存一份
    output reg [31:0] inst_out,
    output reg [4:0] r1, r2,
    output reg [31:0] offset_jal2, offset_beq2,
    output reg [31:0] aux_addr_out,
    output reg jal, br_en, jalr,
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
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_SYSTEM = 7'b1110011;

    reg [31:0] inst_effective;
    reg lane0_ok, lane1_ok, raw_in_pair;
    reg pair_class_ok, pair_raw_ok, pair_lane1_v;

//提取不同类型指令立即数的函数块
    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

//flag_bus = {exc, exec, flush_irq, flush_jump, dcache_hold, bus_hold_in, stall_m, stall_v, lsu_stall, icache_busy}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[9] | flag_bus[7] | flag_bus[6];
        stall_w = (flag_bus[5] | flag_bus[4] | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//指令来源：icache 为唯一取指源
    always @(*) inst_effective = inst_in;

//成对判据（2-wide 阶段 2，原 core/pair_form.v 并入本级 —— 它本来就是 E2 输入侧的判断）：
//由 icache 同拍交付的两个字判 lane1 能不能与 lane0 同拍进流水。本阶段只做判据、不接消费者；
//打开成对时 lane1_v 随本级的寄存器一起跨拍携带。三档输出便于把收益缺口归因到具体规则：
//pair_fetch_ok（在 icache）→ pair_class_ok（类别）→ pair_raw_ok（对内 RAW）→ pair_lane1_v
//类别口径：lane0 允许 ALU 类（不含 M）与访存，lane1 只允许单周期 ALU 类（含移位、不含
//M/访存/分支/CSR）。★ 排除 M 是 2-wide 的地基之一：M 永远只在单发包里 ⇒ mulu 与 alu 的
//"写口永不同拍"、"除法冻全流水"两条既有语义一行不用重推。M 的判据（bit31:25 == 0000001）
//与 mulu 同口径。★ 全零指令字（本核的气泡）opcode == 0000000，不在任何白名单里 ⇒ 天然不成对。
//★ lane1 不含访存：lsu 一拍只吃一条（在途读只许一笔）。
    always @(*) begin
        lane0_ok = 1'b0;
        if (inst_effective[6:0] == OPCODE_OP) begin
            if (inst_effective[31:25] != 7'b0000001) lane0_ok = 1'b1;
        end
        else if (inst_effective[6:0] == OPCODE_OP_IMM) begin
            if (inst_effective[31:25] != 7'b0000001) lane0_ok = 1'b1;
        end
        else if (inst_effective[6:0] == OPCODE_LUI)   lane0_ok = 1'b1;
        else if (inst_effective[6:0] == OPCODE_AUIPC) lane0_ok = 1'b1;
        else if (inst_effective[6:0] == OPCODE_LOAD)  lane0_ok = 1'b1;
        else if (inst_effective[6:0] == OPCODE_STORE) lane0_ok = 1'b1;

        lane1_ok = 1'b0;
        if (inst1_in[6:0] == OPCODE_OP) begin
            if (inst1_in[31:25] != 7'b0000001) lane1_ok = 1'b1;
        end
        else if (inst1_in[6:0] == OPCODE_OP_IMM) begin
            if (inst1_in[31:25] != 7'b0000001) lane1_ok = 1'b1;
        end
        else if (inst1_in[6:0] == OPCODE_LUI)   lane1_ok = 1'b1;
        else if (inst1_in[6:0] == OPCODE_AUIPC) lane1_ok = 1'b1;

        pair_class_ok = lane0_ok & lane1_ok;
    end

//对内 RAW：lane0 的结果当拍不转发给 lane1 ⇒ lane0 的 rd 命中 lane1 的任一源即不成对。
//★ 用"按指令字原始字段判 rs1/rs2"的朴素口径（LUI/AUIPC 的那几位其实是立即数），会漏掉
//  一些本来能成对的组合 —— 方向是【保守】，且与静态统计（24.0%）同一口径，可直接对照。
    always @(*) begin
        raw_in_pair = 1'b0;
        if (inst_effective[11:7] != 5'd0) begin
            if (inst_effective[11:7] == inst1_in[19:15]) raw_in_pair = 1'b1;
            if (inst_effective[11:7] == inst1_in[24:20]) raw_in_pair = 1'b1;
        end
        pair_raw_ok = ~raw_in_pair;
    end

    always @(*) pair_lane1_v = pair_fetch_ok_in & pair_class_ok & pair_raw_ok;

//本模块寄存器只留"必须跨拍携带"的四项：指令字本身、两个源寄存器号（regfile 读口要用）、
//PC 载荷与取指期的预测信息。原来那 15 个译码字段各寄存一份、再被原样重寄一遍。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out <= 32'd0;
            r1 <= 5'd0;
            r2 <= 5'd0;
            aux_addr_out <= 32'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
        end
        else if (exec) begin
            if (!flush_w && !stall_w) begin
                inst_out <= inst_effective;
                r1 <= 5'd0;
                r2 <= 5'd0;
                aux_addr_out <= aux_addr_in;
                br_pred_taken_out <= br1_in;
                jalr_pred_addr_out <= jalr_pred_addr_in;
//只有真正读这两个源寄存器的指令才填：别的指令留 0，免得在冒险比较里误命中
                case (inst_effective[6:0])
                    OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
                    OPCODE_LOAD, OPCODE_STORE, OPCODE_SYSTEM: r1 <= inst_effective[19:15];
                endcase
                case (inst_effective[6:0])
                    OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: r2 <= inst_effective[24:20];
                endcase
//AUIPC 也要带上 PC：它的结果由 (aux_addr) + (immU-4) 得到，那个 -4 折在立即数里
                case (inst_effective[6:0])
                    OPCODE_JAL, OPCODE_JALR, OPCODE_BRANCH, OPCODE_AUIPC: aux_addr_out <= aux_addr_in;
                endcase
            end
            else if (stall_w) begin
                inst_out <= inst_out;
                r1 <= r1;
                r2 <= r2;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
            end
            else begin
                inst_out <= 32'd0;
                r1 <= 5'd0;
                r2 <= 5'd0;
                aux_addr_out <= 32'd0;
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
        offset_beq2 = 32'd0;
        offset_jal2 = 32'd0;
        jal = 1'b0;
        jalr = 1'b0;
        if (!rst_q) begin
            case (inst_effective[6:0])
                OPCODE_JAL: begin
                    offset_jal2 = $signed(immJ(inst_effective));
                    jal = 1'b1;
                end
                OPCODE_JALR: begin
                    if (!flush_w) jalr = 1'b1;
                end
                OPCODE_BRANCH: begin
                    offset_beq2 = $signed(immB(inst_effective));
                    br_en = 1'b1;
                end
            endcase
        end
    end

endmodule
