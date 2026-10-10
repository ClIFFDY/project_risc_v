`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: iffu
// Description:
//   夹在 icache 与 idu1 之间的取指队列（单发射版）。
//
//   条目只存"队列取消不了的携带量"：指令字 + 地址 + 分支预测位 + jalr 预测目标 +
//   两个源寄存器号（107 位，深度 8）。rd / func10 / imm 是纯函数，一律不在队里存。
//
//   两个推进门分开：
//     f_req = exec & deliv_v                                            （取指侧：交付事件）
//     advance                                                               （读侧，idu1 给）
//   ★ 取指侧收的是【事件】，不是"推进拍"：icache 的交付是"pc 的下游流水线"，
//     pc 停住时它会连着几拍把同一条摆在输出上，照推进拍收就会把重交付当新指令收下
//     （实测同一条被分配 5 次 ⇒ 提交序列重复）。deliv_v = "已交付、还没进队"。
//   ★ 推力门 push_ok 只认真溢出（cnt<7），不看 full：冻门触发的那一拍，
//     在飞的那条（含改向欠下的 bti 交付）仍要能进去。
//
//   队头是组合读：空队列时压成空泡（不做"本拍写入旁路"），它只作出队格 h0 的 D 端。
//   ★ 旁路"队空时队头直接是本拍写进去的那条"曾经在，实测是时序墙：它把
//     icache 的 BRAM 输出一路组合送到 rob 的 ent_h2 D 端（11.0ns / 9 级 / 67% 布线），
//     一去掉失败端点 293→6、WNS -1.175→-0.377，代价只有 +1.3% 拍数。
//   出队格 h0 是本级的【唯一】输出寄存器：rob 的分配口/入队扫描口与 idu1 的译码输入
//   都取它的 Q 端 —— 同一条指令的数据在一拍上同时发给 rob 与 idu1。ROB 号不再随本级的
//   内容锁存（由 rob 直接发 idu1 的输出级），所以出队格里不存号、也无 nh0_* 那一组
//   组合的队头输出。
//
//   取指侧的重定向译码（br_en / jal / jalr / offset_beq / offset_jal）在本模块前端，
//   对刚交付的 inst_eff 组合解 —— 口径与原来 idu1 的那一段一致。
//////////////////////////////////////////////////////////////////////////////////


module iffu(
    input clk, rst,
    input [7:0] flag_bus,
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
//读侧：本拍出队格换不换内容（= 新 idu1 的推进条件，与它逐字同形）
    input        advance,
//出队格（本级的出口寄存器）：iffu 的唯一输出寄存器，rob 与 idu1 都在它的 Q 端取数
    output reg [31:0] h0_inst, h0_addr,
    output reg        h0_br_pred,
    output reg [31:0] h0_jalr_pred,
    output reg [4:0]  h0_r1, h0_r2,
//取指侧重定向译码（原 idu1 的组合段）：消费者是 pc、bpu、icache 的 jalr 门
    output reg        fch_br_en, fch_jal, fch_jalr,
    output reg [31:0] fch_off_beq, fch_off_jal,
//取指侧停：本拍放不下。icache 与 pc 的推进被它按住
    output reg        full,
//icache 的交付事件：有一条已交付、还没进队
    input             deliv_v,
//本条交付被【收下】了（写进去、或是空泡就地丢掉）——欠交付的两边都盯它，别盯"取指门开了"
    output reg        take_en
    );

//复位就地打一拍（与其它模块同款）
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus 逐位还原（口径与各消费者一致：冲刷压停顿）
//停顿位一位都不用了：后端停顿由本队列吸收，推力只看交付事件与空位。
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    reg flush_w, f_req, f_adv, f_nop, push_ok;
//三态合成出来的"本拍这条"（下面给值）：推力判"是不是空泡"要在译码块里用它，故声明提前
    reg [31:0] inst_eff;
    always @(*) begin
        flush_con_exc     = flag_bus[7];
        flush_con_irq     = flag_bus[6];
        flush_con_jump    = flag_bus[5];
        exec              = flag_bus[4];
        flush_w = flush_con_exc | flush_con_irq | flush_con_jump;
        f_req = exec & (deliv_v | (bti_sel != 2'd0));
        f_nop = ~|inst_eff;
        f_adv = f_req & ~f_nop & push_ok;
        take_en = f_req & (f_nop | push_ok);
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

//提取分支/跳转立即数的函数（口径与 idu1 里那两条逐字一致）
    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

//指令来源三态（口径与原 idu1 的 inst_effective 一致）：
//bti_sel=1 用 btb 直送的跳转目标那条；=2 交付 NOP（方向说跳但没缓存）；=0 用 icache 正常交付。
//inst_valid=0 表示这一拍送来的是无效读数（缺失垃圾 / 那次回填已被冲刷作废），压成 0。
    always @(*) begin
        if (bti_sel == 2'd1)
            inst_eff = bti_inst;
        else if (bti_sel == 2'd2)
            inst_eff = 32'd0;
        else
            inst_eff = inst_valid ? inst_in : 32'd0;
    end

//取指侧分支/跳转预译码（原 idu1 末尾那块）：
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
    reg [31:0]  nx_inst, nx_addr, nx_jalr_pred;
    reg         nx_br_pred;
    reg [4:0]   nx_r1, nx_r2;
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
        full    = (cnt_use >= 4'd6);
        push_ok = (cnt_use < 4'd7);
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
        else if (advance & hd0_v) begin
            rptr <= rptr + 4'd1;
        end
    end

//队头（组合读）：出队格的 D 端
    always @(*) begin
        idx0 = rptr;
    end

//不在册时整条压 0：不让陈旧值/X 走到译码、冒险比较和下游载荷里
    always @(*) begin
        h0_data = q_dat[idx0[2:0]];
        hd0_v   = (idx0 != wptr);
        nx_inst      = 32'd0;
        nx_addr      = 32'd0;
        nx_br_pred   = 1'b0;
        nx_jalr_pred = 32'd0;
        nx_r1        = 5'd0;
        nx_r2        = 5'd0;
        if (hd0_v) begin
            nx_inst      = h0_data[106:75];
            nx_addr      = h0_data[74:43];
            nx_br_pred   = h0_data[42];
            nx_jalr_pred = h0_data[41:10];
            nx_r1        = h0_data[9:5];
            nx_r2        = h0_data[4:0];
        end
    end

//出队格的装载：换新 / 保持 / 清。保持条件与下一级的推进条件同门（advance 就是下一级给的）。
//装载源取本级组合读出的队头（不在册时已压成 0）。
    always @(posedge clk) begin
        if (rst_q) begin
            h0_inst <= 32'd0;
            h0_addr <= 32'd0;
            h0_br_pred <= 1'b0;
            h0_jalr_pred <= 32'd0;
            h0_r1 <= 5'd0;
            h0_r2 <= 5'd0;
        end
        else if (exec) begin
            if (flush_w) begin
                h0_inst <= 32'd0;
                h0_addr <= 32'd0;
                h0_br_pred <= 1'b0;
                h0_jalr_pred <= 32'd0;
                h0_r1 <= 5'd0;
                h0_r2 <= 5'd0;
            end
            else if (advance) begin
                h0_inst <= nx_inst;
                h0_addr <= nx_addr;
                h0_br_pred <= nx_br_pred;
                h0_jalr_pred <= nx_jalr_pred;
                h0_r1 <= nx_r1;
                h0_r2 <= nx_r2;
            end
            else begin
                h0_inst <= h0_inst;
                h0_addr <= h0_addr;
                h0_br_pred <= h0_br_pred;
                h0_jalr_pred <= h0_jalr_pred;
                h0_r1 <= h0_r1;
                h0_r2 <= h0_r2;
            end
        end
    end

endmodule
