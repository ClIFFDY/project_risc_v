`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: fetch_fifo
// Description:
//   夹在 icache 与 pre_decoder 之间的取指队列（单发射版）。
//
//   条目只存"队列取消不了的携带量"：指令字 + 地址 + 分支预测位 + jalr 预测目标 +
//   两个源寄存器号（107 位，深度 8）。rd / func10 / imm 是纯函数，一律不在队里存。
//
//   两个推进门分开：
//     f_req = exec & ~flush_w & ~stall_w & rdy1_in & rdy2_in           （取指侧）
//     adv                                                            （读侧，pre_decoder 给）
//   ★ 取指门必须与 pc / icache 的推进门【逐项同门】：icache 的交付是"pc 的下游流水线"
//     （pc 停住 ⇒ 随后几拍重复交付同一条），本级若对停调解耦，重交付会被当成新指令收下
//     （实测同一条被分配 5 次 ⇒ 提交序列重复）。后端那八条停顿位、pc 的 fifo_full、
//     以及 rob 的就绪门（rdy1/rdy2，见下面端口列表那两行的注释）一个都不能少 —— 少一个就是
//     "队列被灌满 ⇒ full 关掉 pc 的推进门 ⇒ 这一拍刚译出的重定向被静默丢掉"。
//
//   队头是组合读 + 本拍写入旁路：空队列时队头直接是本拍写进去的那条，冲刷后落点那条
//   不用多压一拍。出队格 h0 是本级的寄存器（承接原来 pre_decoder 那一级）。
//
//   取指侧的重定向译码（br_en / jal / jalr / offset_beq / offset_jal）在本模块前端，
//   对刚交付的 inst_eff 组合解 —— 口径与原来 pre_decoder 的那一段一致。
//////////////////////////////////////////////////////////////////////////////////


module fetch_fifo(
    input clk, rst,
    input [11:0] flag_bus,
//取指重定向（pc.v 的 redir_go 落地那拍）：整队作废
    input flush_pc_redir,
//写口：icache 本拍交出的指令（三态在本模块前端合成）
    input [31:0] inst_in,
    input        inst_valid,
    input [1:0]  bti_sel,
    input [31:0] bti_inst,
//本拍这一条的携带量：地址（pc 的 aux_addr 口径，原样存）、分支预测"跳"位、jalr 预测目标
    input [31:0] push_addr,
    input        push_br_pred,
    input [31:0] push_jalr_pred,
//读侧：本拍出队格换不换内容（= 新 pre_decoder 的推进条件，与它逐字同形）
    input        adv,
//这一拍发给队头那条的 ROB 号（跟内容一起锁进出队格）
    input [2:0]  rob_idx,
    input        rob_gen,
//队头（组合读 + 本拍写入旁路）：给 rob 的分配口与新 pre_decoder
    output reg [31:0] nh0_inst, nh0_addr,
    output reg        nh0_br_pred,
    output reg [31:0] nh0_jalr_pred,
    output reg [4:0]  nh0_r1, nh0_r2,
    output reg        nh0_v,
//出队格（本级的出口寄存器）
    output reg [31:0] h0_inst, h0_addr,
    output reg        h0_br_pred,
    output reg [31:0] h0_jalr_pred,
    output reg [4:0]  h0_r1, h0_r2,
    output reg        h0_v,
    output reg [2:0]  idx_out,
    output reg        gen_out,
//取指侧重定向译码（原 pre_decoder 的组合段）：消费者是 pc、bra_predict、icache 的 jalr 门
    output reg        fch_br_en, fch_jal, fch_jalr,
    output reg [31:0] fch_off_beq, fch_off_jal,
//取指侧停：本拍放不下。icache 与 pc 的推进被它按住
    output reg        full,
//★ 就绪门（rob 按冻结生产者槽现算）：【原地等】必须整组同门 —— 前端也算在内。
//  只停后端不停取指 ⇒ 队列被灌满、fifo_full 又把 pc 的推进门关掉，
//  于是这一拍刚译出来的前端重定向（jal/br1）被静默丢掉（实测 divu-01 漏跳一个 jal、整个程序重跑）。
    input             rdy1_in, rdy2_in
    );

//复位就地打一拍（与其它模块同款）
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus 逐位还原（口径与各消费者一致：冲刷压停顿）
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    reg stall_rob_full, stall_pc_redir;
    reg stall_lsu_haz, stall_lsu_full;
    reg stall_mulu_haz, stall_mulu_div;
    reg stall_icache_miss, stall_bus_hold;
    reg stall_w;
    reg flush_w, f_req, f_adv;
    always @(*) begin
        flush_con_exc     = flag_bus[11];
        flush_con_irq     = flag_bus[10];
        flush_con_jump    = flag_bus[9];
        exec              = flag_bus[8];
        stall_rob_full    = flag_bus[7];
        stall_pc_redir    = flag_bus[6];
        stall_lsu_haz     = flag_bus[5];
        stall_lsu_full    = flag_bus[4];
        stall_mulu_haz    = flag_bus[3];
        stall_mulu_div    = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        stall_bus_hold    = flag_bus[0];
        flush_w = flush_con_exc | flush_con_irq | flush_con_jump;
        stall_w = (stall_rob_full | stall_pc_redir | stall_lsu_haz | stall_lsu_full
                 | stall_mulu_haz | stall_mulu_div | stall_icache_miss | stall_bus_hold) & ~flush_w;
//★ 推力门必须与 pc/icache 的推进门【逐项同门】：icache 的交付是"pc 的下游流水线"，
//  pc 停住时它会在随后几拍重复交付同一条 ⇒ 若本模块照推，就会把重交付当新指令收下
//  （实测：同一条指令被分配 5 次 ⇒ 提交序列重复 ⇒ 签名错）。
        f_req = exec & ~flush_w & ~stall_w & rdy1_in & rdy2_in;
        f_adv = f_req & ~full;
    end

//RV32I 里与源寄存器号/改向有关的 opcode
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_SYSTEM = 7'b1110011;

//提取分支/跳转立即数的函数（口径与 pre_decoder 里那两条逐字一致）
    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

//指令来源三态（口径与原 pre_decoder 的 inst_effective 一致）：
//bti_sel=1 用 btb 直送的跳转目标那条；=2 交付 NOP（方向说跳但没缓存）；=0 用 icache 正常交付。
//inst_valid=0 表示这一拍送来的是无效读数（缺失垃圾 / 那次回填已被冲刷作废），压成 0。
    reg [31:0] inst_eff;
    always @(*) begin
        if (bti_sel == 2'd1)
            inst_eff = bti_inst;
        else if (bti_sel == 2'd2)
            inst_eff = 32'd0;
        else
            inst_eff = inst_valid ? inst_in : 32'd0;
    end

//取指侧分支/跳转预译码（原 pre_decoder 末尾那块）：
//jal / br_en / offset_* 不受 stage 门控（消费者里有取指地址那条环），只有 jalr 保留门控。
    always @(*) begin
        fch_br_en = 1'b0;
        fch_off_beq = 32'd0;
        fch_off_jal = 32'd0;
        fch_jal = 1'b0;
        fch_jalr = 1'b0;
        if (!rst_q) begin
            case (inst_eff[6:0])
                OPCODE_JAL: begin
                    fch_off_jal = $signed(immJ(inst_eff));
                    fch_jal = 1'b1;
                end
                OPCODE_JALR: begin
                    if (!flush_w)
                        fch_jalr = 1'b1;
                end
                OPCODE_BRANCH: begin
                    fch_off_beq = $signed(immB(inst_eff));
                    fch_br_en = 1'b1;
                end
            endcase
        end
    end

//写口这一条的源寄存器号：只有真读这两个源的指令才填，别的留 0，免得在冒险比较里误命中。
//csrrwi/si/ci 的 [19:15] 是 zimm、不是寄存器号：只对 rs1 形（funct3=001/010/011）填。
    reg [4:0] w0_r1, w0_r2;
    always @(*) begin
        w0_r1 = 5'd0;
        w0_r2 = 5'd0;
        case (inst_eff[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
            OPCODE_LOAD, OPCODE_STORE: w0_r1 = inst_eff[19:15];
        endcase
        if ((inst_eff[6:0] == OPCODE_SYSTEM)
         && (inst_eff[14:12] != 3'b101)
         && (inst_eff[14:12] != 3'b110)
         && (inst_eff[14:12] != 3'b111))
            w0_r1 = inst_eff[19:15];
        case (inst_eff[6:0])
            OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: w0_r2 = inst_eff[24:20];
        endcase
    end

    localparam [3:0] DEPTH = 4'd8;

//       [106:75] inst   [74:43] addr   [42] br_pred   [41:10] jalr_pred   [9:5] r1   [4:0] r2
    reg [106:0] q_dat [0:7];
    reg [3:0]  wptr, rptr;
    reg [3:0]  cnt_use;
    reg [3:0]  idx0;
    reg [106:0] push0_data, h0_data;
    reg         hd0_v;
    reg        wr0, clr;

//写入的那一条（布局见上）。addr 是"指令地址 + 4"口径的 aux_addr，原样存。
    always @(*) begin
        push0_data = {inst_eff, push_addr, push_br_pred, push_jalr_pred, w0_r1, w0_r2};
    end

    always @(*) begin
        cnt_use  = wptr - rptr;
    end

//full 按"占用 ≥ 6 就报"（阈值），不是"放不下才报"：本拍要推的那条还在飞（写入在本拍结束），
//按"放不下"判会让取指在满/不满之间来回。阈值 6 + 在飞 1 ⇒ 最高占用 7 < 8，不会写丢。
    always @(*) begin
        full = f_req & (cnt_use >= 4'd6);
    end

    always @(*) begin
        clr = flush_w | flush_pc_redir;
    end

    always @(*) begin
        wr0 = f_adv;
    end

//写口
    always @(posedge clk) begin
        if (wr0) begin
            q_dat[wptr[2:0]] <= push0_data;
        end
    end

//写指针
    always @(posedge clk) begin
        if (rst_q) begin
            wptr <= 4'd0;
        end
        else if (clr) begin
            wptr <= rptr;
        end
        else if (wr0) begin
            wptr <= wptr + 4'd1;
        end
    end

//读指针：清队时不动（那些条目由 wptr 作废）；正常按本拍离开队列的条数（0 或 1）走
    always @(posedge clk) begin
        if (rst_q) begin
            rptr <= 4'd0;
        end
        else if (clr) begin
            rptr <= rptr;
        end
        else if (adv & nh0_v) begin
            rptr <= rptr + 4'd1;
        end
    end

//队头（组合读 + 本拍写入旁路）：喂给出队格的那一级
    always @(*) begin
        idx0 = rptr;
    end

    always @(*) begin
        h0_data = q_dat[idx0[2:0]];
        hd0_v   = (idx0 != wptr);
        if ((idx0 == wptr) & wr0) begin
            h0_data = push0_data;
            hd0_v   = 1'b1;
        end
//不在册时整条压 0：不让陈旧值/X 走到译码、冒险比较和下游载荷里
        nh0_inst      = 32'd0;
        nh0_addr      = 32'd0;
        nh0_br_pred   = 1'b0;
        nh0_jalr_pred = 32'd0;
        nh0_r1        = 5'd0;
        nh0_r2        = 5'd0;
        nh0_v         = hd0_v;
        if (hd0_v) begin
            nh0_inst      = h0_data[106:75];
            nh0_addr      = h0_data[74:43];
            nh0_br_pred   = h0_data[42];
            nh0_jalr_pred = h0_data[41:10];
            nh0_r1        = h0_data[9:5];
            nh0_r2        = h0_data[4:0];
        end
    end

//出队格的装载：换新 / 保持 / 清。保持条件与下一级的推进条件同门（adv 就是下一级给的）。
//号与内容同门锁存（idx/gen 跟指令走，出去以后就是载荷的 issue_idx/issue_gen）。
    always @(posedge clk) begin
        if (rst_q) begin
            h0_inst <= 32'd0;
            h0_addr <= 32'd0;
            h0_br_pred <= 1'b0;
            h0_jalr_pred <= 32'd0;
            h0_r1 <= 5'd0;
            h0_r2 <= 5'd0;
            h0_v <= 1'b0;
            idx_out <= 3'd0;
            gen_out <= 1'b0;
        end
        else if (exec) begin
            if (flush_w) begin
                h0_inst <= 32'd0;
                h0_addr <= 32'd0;
                h0_br_pred <= 1'b0;
                h0_jalr_pred <= 32'd0;
                h0_r1 <= 5'd0;
                h0_r2 <= 5'd0;
                h0_v <= 1'b0;
                idx_out <= 3'd0;
                gen_out <= 1'b0;
            end
            else if (adv) begin
                h0_inst <= nh0_inst;
                h0_addr <= nh0_addr;
                h0_br_pred <= nh0_br_pred;
                h0_jalr_pred <= nh0_jalr_pred;
                h0_r1 <= nh0_r1;
                h0_r2 <= nh0_r2;
                h0_v <= nh0_v;
                idx_out <= rob_idx;
                gen_out <= rob_gen;
            end
            else begin
                h0_inst <= h0_inst;
                h0_addr <= h0_addr;
                h0_br_pred <= h0_br_pred;
                h0_jalr_pred <= h0_jalr_pred;
                h0_r1 <= h0_r1;
                h0_r2 <= h0_r2;
                h0_v <= h0_v;
                idx_out <= idx_out;
                gen_out <= gen_out;
            end
        end
    end

endmodule
