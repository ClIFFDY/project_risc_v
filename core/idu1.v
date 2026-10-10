`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu1
// Description:
//   取指队列与载荷之间的寄存器级（= 原 mid_decoder 的位置），并入 rd / func10 / imm 的译码。
//
//   输入是 iffu 的出队格（F 级，原样指令字 + 随行号），输出是本级的寄存器（S 级）：
//   载荷字段由 S 级的指令字【组合】解出（不再多寄存一份译码结果）。
//
//   ★ 两套译码源不同、用途不同：
//     - 载荷面（rd/func10/imm）解在 S 级的 inst 上，给 post_decoder；
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
//保留站的"满"【只喂这一段】：取指队头 → 保留站。它不进全核广播 ——
//站满恰恰是发射常态，广播出去会让执行单元（乘法发起、CSR 写）误以为自己该停。
    input stall_rs_full,
//来自 iffu 出队格（F 级）
    input [31:0] inst_in,
    input [31:0] addr_in,
    input        br_pred_in,
    input [31:0] jalr_pred_in,
    input [4:0]  r1_in, r2_in,
    input [2:0]  idx_in,
    input        gen_in,
//载荷那条的就绪（rob 按冻结生产者槽现算）：本来只在 post_decoder 门 payload_go，
//但"原地等"必须【整组一起停】—— 否则本级照推会把 S 级内容原地覆盖掉、那条永远进不了载荷。
    input        rdy1_in, rdy2_in,
//给 fifo 的推进条件（与出队格的"换新"同门）
    output reg   advance,
//给 post_decoder 的载荷（原 mid_decoder 的输出集合 + 随行号）
    output reg [31:0] inst_out,
    output reg [4:0]  rd_out,
    output reg [9:0]  func10_out,
    output reg [31:0] imm_alu_out,
    output reg [31:0] aux_addr_out,
    output reg        br_pred_taken_out,
    output reg [31:0] jalr_pred_addr_out,
    output reg [4:0]  r1_out, r2_out,
    output reg [2:0]  idx_out,
    output reg        gen_out,
//rob 分配口（在队头上按 opcode 分类解）
    output reg        alloc_en,
    output reg        alloc_we,
    output reg        alloc_jmp,
//这一条会不会报完成口（给 rob 的"发出即置可退"用）：M 类（funct7==1 的 OP）与 LOAD。
//它们即使不写 rd（rd=x0），单元照样会报一笔完成口 ⇒ 不能当"没人会回报"提前置可退；
//否则它立刻退休、槽被回收，而它的完成口（除法 30+ 拍）晚回来时会撞上槽的新住户。
    output reg        alloc_pend,
    output reg [4:0]  alloc_rd
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

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
    reg exec, flush_w, stall;
    always @(*) begin
        flush_w = flag_bus[7] | flag_bus[6] | flag_bus[5];
        stall = (flag_bus[3] | flag_bus[2] | stall_rs_full | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
//★ 保留站改造：`rdy1_in & rdy2_in` 已从本级摘掉 —— 操作数等待由 idu2 的站项承担，
//  S 级不再为它停（否则"原地等"还在，站的解耦白做）。入站就绪只用来决定"锁值还是锁标签"。
        advance     = exec & ~flush_w & ~stall;
    end

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

//rob 分配口：分配发生在"出队格 h0 换新"那一拍，吃的就是本级的译码输入（= h0 的 Q 端），
//与 idu1 自己的译码同源 —— 不再另解一份队头。rd 直接复用主译码的 rd_dec（按 opcode 分类
//填过，不裸取 inst[11:7]）。
//★ 发号拍 = 本级每一个推进拍（不按出队格有效与否区分）：空泡也要占一项 ——
//  载荷的 rob 号与“冻结的生产者槽”都按项索引取，空泡没有项就会取到别人那一项。
    always @(*) begin
        alloc_en   = advance;
        alloc_rd   = rd_dec;
        alloc_we   = (rd_dec != 5'd0);
        alloc_jmp  = jmp_dec;
        alloc_pend = (inst_in[6:0] == OPCODE_LOAD) |
                     ((inst_in[6:0] == OPCODE_OP) && (inst_in[31:25] == 7'b0000001));
    end

//推进/保持/清：与 fifo 出队格的"换新/保持/清"【逐字同形】。
//★ 条件必须与上一级一致：本级的"保持"要跟上一级的"保持"落在同一拍，差一拍就是整条流水线错位。
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
            idx_out <= 3'd0;
            gen_out <= 1'b0;
        end
        else if (exec) begin
            if (advance) begin
                inst_out <= inst_in;
                rd_out <= rd_dec;
                func10_out <= func10_dec;
                imm_alu_out <= imm_dec;
                aux_addr_out <= addr_in;
                br_pred_taken_out <= br_pred_in;
                jalr_pred_addr_out <= jalr_pred_in;
                r1_out <= r1_in;
                r2_out <= r2_in;
                idx_out <= idx_in;
                gen_out <= gen_in;
            end
//★ 保持判据必须与 advance 逐项同门：advance 比 stall 多出 rdy 那一项，不把它补进这里，
//  暂停那一拍就掉进下面的"清"分支 —— S 级被清零，那条指令永远进不了载荷。
            else if ((stall | ~rdy1_in | ~rdy2_in) & ~flush_w) begin
                inst_out <= inst_out;
                rd_out <= rd_out;
                func10_out <= func10_out;
                imm_alu_out <= imm_alu_out;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
                r1_out <= r1_out;
                r2_out <= r2_out;
                idx_out <= idx_out;
                gen_out <= gen_out;
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
                idx_out <= 3'd0;
                gen_out <= 1'b0;
            end
        end
    end

endmodule
