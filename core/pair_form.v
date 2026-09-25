`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/25 23:05:00
// Design Name:
// Module Name: pair_form
// Project Name:
// Target Devices:
// Tool Versions:
// Description: 成对判据（2-wide 顺序多发射的阶段 2）
//   输入 icache 同拍交付的两个字（inst0 = lane0 = inst_out，inst1 = lane1 = inst_next）
//   与取指侧有效位 pair_fetch_ok，输出 lane1 能不能与 lane0 同拍进流水。
//   纯组合、无状态：判据只依赖两个指令字与那一位。
//   本阶段只做【判据】、不含任何载荷（lane1 的 aux_addr 等随"打开成对"那一阶段一起加）。
//   输出分三档，便于把收益缺口归因到具体规则：
//     pair_fetch_ok（在 icache）→ pair_class_ok（类别）→ pair_raw_ok（对内 RAW）→ pair_lane1_v
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module pair_form(
    input [31:0] inst0_in, inst1_in,
    input pair_fetch_ok_in,
    output reg pair_class_ok, pair_raw_ok, pair_lane1_v
    );

    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;

    reg lane0_ok, lane1_ok, raw_in_pair;

//类别判据：lane0 允许 ALU 类（不含 M）与访存，lane1 只允许单周期 ALU 类（不含 M）。
//★ 排除 M 是本方案的地基之一：M 永远只出现在单发包里 ⇒ mulu 与 alu 的"写口永不同拍"、
//  以及"除法冻全流水"这两条既有语义一行都不用重推。M 的判据（bit31:25 == 0000001）
//  与 mulu 同口径。
//★ 全零指令字（本核的流水线气泡）opcode == 0000000，不在任何白名单里 ⇒ 天然不成对。
//★ lane1 不含访存：lsu 一拍只吃一条（在途读只许一笔）；也不含分支/M/CSR、更不含移位外的杂类。
    always @(*) begin
        lane0_ok = 1'b0;
        if (inst0_in[6:0] == OPCODE_OP) begin
            if (inst0_in[31:25] != 7'b0000001) lane0_ok = 1'b1;
        end
        else if (inst0_in[6:0] == OPCODE_OP_IMM) begin
            if (inst0_in[31:25] != 7'b0000001) lane0_ok = 1'b1;
        end
        else if (inst0_in[6:0] == OPCODE_LUI)   lane0_ok = 1'b1;
        else if (inst0_in[6:0] == OPCODE_AUIPC) lane0_ok = 1'b1;
        else if (inst0_in[6:0] == OPCODE_LOAD)  lane0_ok = 1'b1;
        else if (inst0_in[6:0] == OPCODE_STORE) lane0_ok = 1'b1;

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
//★ 这里用"按指令字原始字段判 rs1/rs2"的朴素口径（LUI/AUIPC 的那几位其实是立即数），
//  会漏掉一些本来能成对的组合 —— 方向是【保守】，且与静态统计（24.0%）同一口径，
//  这样动态值与静态值可以直接对照。
    always @(*) begin
        raw_in_pair = 1'b0;
        if (inst0_in[11:7] != 5'd0) begin
            if (inst0_in[11:7] == inst1_in[19:15]) raw_in_pair = 1'b1;
            if (inst0_in[11:7] == inst1_in[24:20]) raw_in_pair = 1'b1;
        end
        pair_raw_ok = ~raw_in_pair;
    end

    always @(*) pair_lane1_v = pair_fetch_ok_in & pair_class_ok & pair_raw_ok;

endmodule
