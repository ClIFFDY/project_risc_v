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
    input [11:0] flag_bus,
    input inst_valid,
    input [31:0] inst_in,
//指令三态来源（bra_predict 里那块服务 br1/jal 的独立 btb）：
//bti_sel=0 用 icache 正常交付；=1 用 btb 送来的"跳转目标那条指令"；=2 交付 NOP。
    input [1:0]  bti_sel,
    input [31:0] bti_inst,
    input [31:0] aux_addr_in,
    input br1_in,
    input [31:0] jalr_pred_addr_in,
//跨拍携带的指令字 + 译码字段。★ 删掉 mid_decoder 之后，rd/func10/imm 的解算回到本级 ——
//历史上它本来就在这儿（37abe17 那次"组合解算"还是时序改善），71b7371 才把它挪进 mid。
//imm12_csr/imm5_csr 不另寄存（下一级从 inst_in 就地切片）；pc_operand = aux_addr_out、
//offset_jalr0/offset_beq0_aux = imm_alu_out（都是别名，顶层复用，不重复寄存）。
    output reg [31:0] inst_out,
    output reg [4:0] r1, r2,
    output reg [4:0]  rd_out,
    output reg [9:0]  func10_out,
    output reg [31:0] imm_alu_out,
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
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_SYSTEM = 7'b1110011;

    reg [31:0] inst_effective;

//提取不同类型指令立即数的函数块
    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
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

//译码字段的组合解算（rd / func10 / 立即数）：口径与 mid_decoder 原来那段 case 逐条一致，
//只是解算源从"上一级的 inst_out"换成本级的 inst_effective —— 少一级就少一个寄存器边界。
    reg [4:0]  rd_c;
    reg [9:0]  func10_c;
    reg [31:0] imm_c;
    always @(*) begin
        rd_c = 5'd0;
        func10_c = 10'd0;
        imm_c = 32'd0;
        case (inst_effective[6:0])
            OPCODE_OP: begin
                rd_c = inst_effective[11:7];
                func10_c = {inst_effective[31:25], inst_effective[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_c = immI(inst_effective);
                rd_c = inst_effective[11:7];
                func10_c = {(inst_effective[14:12] == 3'b001 || inst_effective[14:12] == 3'b101) ? inst_effective[31:25] : 7'd0,
                            inst_effective[14:12]};
            end
            OPCODE_JAL: begin
                rd_c = inst_effective[11:7];
            end
            OPCODE_JALR: begin
                rd_c = inst_effective[11:7];
                imm_c = immI(inst_effective);
            end
            OPCODE_BRANCH: begin
                imm_c = immB(inst_effective);
                func10_c = {7'd0, inst_effective[14:12]};
            end
            OPCODE_LOAD: begin
                imm_c = immI(inst_effective);
                rd_c = inst_effective[11:7];
            end
            OPCODE_STORE: begin
                imm_c = immS(inst_effective);
            end
            OPCODE_LUI: begin
                imm_c = immU(inst_effective);
                rd_c = inst_effective[11:7];
            end
            OPCODE_AUIPC: begin
                imm_c = immU(inst_effective) - 4'd4;
                rd_c = inst_effective[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_c = inst_effective[11:7];
                func10_c = {7'd0, inst_effective[14:12]};
            end
        endcase
    end

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 controller 里已逐条分开，见其 flag_bus 拼装）。
    reg exec, flush_w, stall_w, flush_con_exc;
    always @(*) begin
        flush_con_exc     = flag_bus[11];
        flush_w = flush_con_exc | flag_bus[10] | flag_bus[9];
        stall_w = (flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4] | flag_bus[3]
                 | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//指令来源三态。bti_sel 那两态是跳转那一拍用的：目标那条指令由 bra_predict 的 btb 直接送来
//（icache 那拍交出来的是顺序错路，一律不用）；若方向说跳但 btb 没缓存，就交付一条 NOP，
//让 pc 落 T 由 icache 自己去取目标（冷启动 1 拍）。
//第三态是原来的口径：inst_valid=0 表示这一拍送来的是无效读数（缺失垃圾，或那次回填已被
//冲刷作废），整条压成 0 ⇒ 组合块算不出 jal/br_en（不会拿垃圾去改取指地址），时序块锁进去的
//是一条 NOP。回填期间流水线本来就被 stall 按住，门控只在回填落那一拍起作用。
    always @(*) begin
        if (bti_sel == 2'd1)
            inst_effective = bti_inst;
        else if (bti_sel == 2'd2)
            inst_effective = 32'd0;
        else
            inst_effective = inst_valid ? inst_in : 32'd0;
    end

//本模块寄存器只留"必须跨拍携带"的四项：指令字本身、两个源寄存器号（regfile 读口要用）、
//PC 载荷与取指期的预测信息。原来那 15 个译码字段各寄存一份、再被原样重寄一遍。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out <= 32'd0;
            r1 <= 5'd0;
            r2 <= 5'd0;
            rd_out <= 5'd0;
            func10_out <= 10'd0;
            imm_alu_out <= 32'd0;
            aux_addr_out <= 32'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
        end
        else if (exec) begin
            if (!flush_w && !stall_w) begin
                inst_out <= inst_effective;
                r1 <= 5'd0;
                r2 <= 5'd0;
                rd_out <= rd_c;
                func10_out <= func10_c;
                imm_alu_out <= imm_c;
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
                rd_out <= rd_out;
                func10_out <= func10_out;
                imm_alu_out <= imm_alu_out;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
            end
            else begin
                inst_out <= 32'd0;
                r1 <= 5'd0;
                r2 <= 5'd0;
                rd_out <= 5'd0;
                func10_out <= 10'd0;
                imm_alu_out <= 32'd0;
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
                    if (!flush_w)
                        jalr = 1'b1;
                end
                OPCODE_BRANCH: begin
                    offset_beq2 = $signed(immB(inst_effective));
                    br_en = 1'b1;
                end
            endcase
        end
    end

endmodule
