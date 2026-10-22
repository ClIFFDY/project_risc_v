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
//     f_req = exec & ~flush_w & ~full                                    （取指侧：收一条）
//     advance                                                               （读侧，idu1 给）
//   ★ 取指侧只有一根流控：full。不 full 就收一条，空泡（指令字全 0）就地丢掉、不写队列。
//     icache 那边的契约是"每拍都往口上放一样东西 —— 新指令或 NOP；只有 full 的时候保持"，
//     所以不会出现同一条真指令连摆几拍、被重复收下的情形。
//   ★ 因此这里不需要 take_en/deliv_v 那套握手，也不需要"上一拍那条指令"的锁存。
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
//   对刚交付的 inst_eff0 组合解 —— 口径与原来 idu1 的那一段一致。
//////////////////////////////////////////////////////////////////////////////////


module iffu(
    input clk, rst,
    input [7:0] flag_bus,
//取指重定向（pc.v 的 redir_go 落地那拍）：整队作废
    input        flush_pc_redir,
//icache 交付口：两条 lane 各一条，有效位为 0 时在本模块前端挡成空泡
    input [31:0] inst_in_0, inst_in_1,
    input        inst_valid_0, inst_valid_1,
//bpu 的 btic 注入与 jalr 预测命中位
//注入是【两套完全对称】的：bpu 为每条 lane 各欠一次交付，本模块按"哪一套欠着"选一套用。
//bti_inst_x：目标地址上的【一对】，低 32 = 目标那条(lane0)、高 32 = 目标+4(lane1)
    input [63:0] bti_inst_0, bti_inst_1,
    input [1:0]  bti_sel_0, bti_sel_1,
    input        btb_hit,
//随指令携带的量：两条 lane 各一份 —— 地址（架构 PC 口径，原样存）、预测"跳"位、jalr 预测目标
    input [31:0] push_addr_0, push_addr_1,
    input        push_br_pred_0, push_br_pred_1,
    input [31:0] push_jalr_pred_0, push_jalr_pred_1,
//读侧：本拍出队格换不换内容（= idu1 的推进条件，与它逐字同形）
    input [1:0]  advance,
//出队格：本级的唯一输出寄存器，rob 与 idu1 都在它的 Q 端取数
    output reg [31:0] h0_inst, h0_addr, h0_jalr_pred,
    output reg        h0_br_pred,
    output reg [4:0]  h0_r1, h0_r2,
    output reg [31:0] h1_inst, h1_addr, h1_jalr_pred,
    output reg        h1_br_pred,
    output reg [4:0]  h1_r1, h1_r2,
//出队格的有效位（与本格其它字段同沿寄存）。★ 必须给 idu1：advance 是本级的"弹几条"请求，
//  这里按有效条数截断；idu1 若不知道 h1 有没有，就会把空白格也装进 lane1 ⇒ 同一条被取两次。
    output reg        h0_v, h1_v,
//取指侧重定向译码：两条 lane 对称，消费者是 pc / bpu / icache 的 jalr 门
    output reg [31:0] fch_off_beq_0, fch_off_jal_0, fch_off_beq_1, fch_off_jal_1,
    output reg        fch_br_en_0, fch_jal_0, fch_jalr_0,
    output reg        fch_br_en_1, fch_jal_1, fch_jalr_1,
//取指侧停：本拍放不下，icache 与 pc 的推进被它按住
    output reg        full
    );

//复位就地打一拍（与其它模块同款）
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus 逐位还原（口径与各消费者一致：冲刷压停顿）
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    reg flush_w, f_req;
    reg f_nop0, f_nop1, f_pair, push_l0, push_l1, lane0_redir, lane1_vld;
    reg lane1_go, lane1_jal, lane1_br, lane1_jalr;
    reg lane0_jal, lane0_jalr, lane0_br;
    reg [31:0] inst_eff0, inst_eff1;
    reg [63:0] bti_inst;
    reg [1:0]  bti_sel;
    always @(*) begin
        flush_con_exc     = flag_bus[7];
        flush_con_irq     = flag_bus[6];
        flush_con_jump    = flag_bus[5];
        exec              = flag_bus[4];
        flush_w = flush_con_exc | flush_con_irq | flush_con_jump;
        f_req = exec & ~flush_w & ~full;
    end

//两套注入二选一（本模块是唯一做这个选择的地方）：lane0 欠着就用 lane0 那套 —— 它先于 lane1
//改向，lane1 欠下的那次一定是错路（lane1 在 lane0 跳之后不再被收）。同时欠着不可能发生，
//就算发生也只会丢掉错路那次。
    always @(*) begin
        if (bti_sel_0 != 2'd0) begin
            bti_sel  = bti_sel_0;
            bti_inst = bti_inst_0;
        end
        else begin
            bti_sel  = bti_sel_1;
            bti_inst = bti_inst_1;
        end
    end

    always @(*) begin
        f_nop0 = ~|inst_eff0;
        f_nop1 = ~|inst_eff1;
        f_pair = lane1_vld & ~lane0_redir;
        push_l0 = f_req & ~f_nop0;
        push_l1 = f_req & f_pair & ~f_nop1;
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
//inst_valid_0=0 表示这一拍送来的是无效读数（缺失垃圾 / 那次回填已被冲刷作废），压成 0。
    always @(*) begin
        if (bti_sel == 2'd1) begin
            inst_eff0 = bti_inst[31:0];
            inst_eff1 = bti_inst[63:32];
        end
        else if (bti_sel == 2'd2) begin
            inst_eff0 = 32'd0;
            inst_eff1 = 32'd0;
        end
        else begin
            inst_eff0 = inst_valid_0 ? inst_in_0 : 32'd0;
            inst_eff1 = inst_valid_1 ? inst_in_1 : 32'd0;
        end
//这一拍 lane1 有没有一条要收的：btic 注入成对时两条都有；icache 交付时要看 lane1 的有效位
        lane1_vld = (bti_sel == 2'd1) | ((bti_sel == 2'd0) & inst_valid_1);
    end

//取指侧分支/跳转预译码（原 idu1 末尾那块）：
//jal / br_en / offset_* 不受 stage 门控（消费者里有取指地址那条环），只有 jalr 保留门控。
//输入就是本拍 icache/btic 摆在口上的那条：队满时 icache 保持输出、btic 保持欠交付，
//所以这几拍上它仍然是那条改向指令，不需要另存一份"上一拍的指令"。
    always @(*) begin
        lane0_jal  = (inst_eff0[6:0] == OPCODE_JAL);
        lane0_jalr = (inst_eff0[6:0] == OPCODE_JALR);
        lane0_br   = (inst_eff0[6:0] == OPCODE_BRANCH);
        lane1_go   = lane1_vld;
        lane1_jal  = lane1_go & (inst_eff1[6:0] == OPCODE_JAL);
        lane1_br   = lane1_go & (inst_eff1[6:0] == OPCODE_BRANCH);
        lane1_jalr = lane1_go & (inst_eff1[6:0] == OPCODE_JALR);
    end

//两条 lane 的预测译码与落点各自原样送出，译码这里不做任何仲裁：
//lane0 变了向不代表 lane1 的译码要作废（pc 的 if 链先查 lane0，先到的先算）。
//仲裁只发生在【收不收】那一步：lane0 改向 ⇒ lane1 是错路 ⇒ f_pair 把它挡掉。
    always @(*) begin
        fch_jal_0     = lane0_jal;
        fch_jalr_0    = lane0_jalr & ~flush_w;
        fch_br_en_0   = lane0_br;
        fch_off_beq_0 = $signed(immB(inst_eff0));
        fch_off_jal_0 = $signed(immJ(inst_eff0));
        fch_jal_1     = lane1_jal;
        fch_jalr_1    = lane1_jalr & ~flush_w;
        fch_br_en_1   = lane1_br;
        fch_off_beq_1 = $signed(immB(inst_eff1));
        fch_off_jal_1 = $signed(immJ(inst_eff1));
//只剩"lane1 跟不跟着进队"这一个消费者
        lane0_redir   = ((inst_eff0[6:0] == OPCODE_BRANCH) & push_br_pred_0)
                      | lane0_jal
                      | (fch_jalr_0 & btb_hit);
    end

//写口这一条的源寄存器号：只有真读这两个源的指令才填，别的留 0，免得在冒险比较里误命中。
//csrrwi/si/ci 的 [19:15] 是 zimm、不是寄存器号：只对 rs1 形（funct3=001/010/011）填。
    reg [4:0] w0_r1, w0_r2, w1_r1, w1_r2;
    always @(*) begin
        w0_r1 = 5'd0;
        w0_r2 = 5'd0;
        w1_r1 = 5'd0;
        w1_r2 = 5'd0;
        case (inst_eff0[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
            OPCODE_LOAD, OPCODE_STORE: w0_r1 = inst_eff0[19:15];
        endcase
        if ((inst_eff0[6:0] == OPCODE_SYSTEM)
         && (inst_eff0[14:12] != 3'b101)
         && (inst_eff0[14:12] != 3'b110)
         && (inst_eff0[14:12] != 3'b111))
            w0_r1 = inst_eff0[19:15];
        case (inst_eff0[6:0])
            OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: w0_r2 = inst_eff0[24:20];
        endcase
        case (inst_eff1[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
            OPCODE_LOAD, OPCODE_STORE: w1_r1 = inst_eff1[19:15];
        endcase
        if ((inst_eff1[6:0] == OPCODE_SYSTEM)
         && (inst_eff1[14:12] != 3'b101)
         && (inst_eff1[14:12] != 3'b110)
         && (inst_eff1[14:12] != 3'b111))
            w1_r1 = inst_eff1[19:15];
        case (inst_eff1[6:0])
            OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: w1_r2 = inst_eff1[24:20];
        endcase
    end

    localparam [3:0] DEPTH = 4'd8;

//       [106:75] inst   [74:43] addr   [42] br_pred   [41:10] jalr_pred   [9:5] r1   [4:0] r2
    reg [106:0] q_dat [0:7];
    reg [3:0]  wptr, rptr;
    reg [3:0]  sidx;
    reg [3:0]  cnt_use;
    reg [1:0]  pop;
    reg [106:0] push0_data, push1_data, s0_data, s1_data;
    reg         stg0_v, stg1_v;
    reg [31:0]  nx_inst, nx_addr, nx_jalr_pred;
    reg [31:0]  nx1_inst, nx1_addr, nx1_jalr_pred;
    reg         nx_br_pred, nx1_br_pred;
    reg [4:0]   nx_r1, nx_r2, nx1_r1, nx1_r2;
    reg        wr0, wr1, clr;

//写入的那一条（布局见上）。addr 是"指令地址 + 4"口径的 aux_addr，原样存。
    always @(*) begin
        push0_data = {inst_eff0, push_addr_0, push_br_pred_0, push_jalr_pred_0, w0_r1, w0_r2};
        push1_data = {inst_eff1, push_addr_1, push_br_pred_1, push_jalr_pred_1, w1_r1, w1_r2};
    end

    always @(*) begin
        cnt_use  = wptr - rptr;
    end

//full 是整条前端唯一的流控：它一出来，pc 不推进、icache 保持输出、btic 不响应、本队不收。
//拉起在 7：成对推要两格，6 的时候再推一对正好到 8，7 就放不下了。
//放下在 4：迟滞，免得在 6/7 之间来回抖。
//拉起这一支必须是【组合】的（按当拍 cnt_use），寄存版会晚一拍，7 那一拍还能再推一对就溢出了；
//只有"保持到 4 才放"这一支用寄存器。
    reg full_q;
    always @(*) full = (cnt_use >= 4'd7) | (full_q & (cnt_use > 4'd4));
    always @(posedge clk) begin
        if (rst_q) full_q <= 1'b0;
        else       full_q <= full;
    end

    always @(*) begin
        clr = flush_w | flush_pc_redir;
    end

//★ 曾在这里加过一层"同一地址不许连推两次"的去重（lp_a/lp_v），用来压住 icache 重放
//  （miss 期间输出寄存器保持 ⇒ 同一个包被再推一次）。那层东西【必须撤掉】：
//  正确的预测跳转（bne 回边 + BTIC 命中/欠交付）**不产生冲刷**，而循环的回边目标往往
//  就是刚刚推过的地址 ⇒ 去重把合法的重取也一并压掉，整核在回边处自锁
//  （实测 WEIRD seed5：pc 卡在回边后的顺延地址、iffu 空、icache 口上有 NOP、rd_go=0）。
//  重放的根子在别处（h0_v/h1_v 与装载门修掉之后，默认档与 WEIRD 档都不再需要它）。
    always @(*) begin
        wr0 = f_req & push_l0;
        wr1 = f_req & push_l1;
    end

//写口
    always @(posedge clk) begin
        if (wr0)
            q_dat[wptr[2:0]] <= push0_data;
        else if (wr1)
            q_dat[wptr[2:0]] <= push1_data;
        if (wr0 & wr1)
            q_dat[wptr[2:0] + 3'd1] <= push1_data;
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
            wptr <= wptr + (wr1 ? 4'd2 : 4'd1);
        end
        else if (wr1) begin
            wptr <= wptr + 4'd1;
        end
    end

//本拍实际弹几条 = min(下一级的请求, 消费者手上真正有效的条数)。
//advance 是 idu1 按自己的 free 算出来的【请求】（0/1/2），不是保证：队里没有那么多就给多少。
    always @(*) begin
        if (!exec | clr | ~h0_v)  pop = 2'd0;
        else if (advance == 2'd0) pop = 2'd0;
        else if (!h1_v)           pop = 2'd1;
        else                      pop = advance;
    end

//读指针：清队时不动（那些条目由 wptr 作废）；正常按实际弹出的条数（0/1/2）走
    always @(posedge clk) begin
        if (rst_q) begin
            rptr <= 4'd0;
        end
        else if (clr) begin
            rptr <= rptr;
        end
        else begin
            rptr <= rptr + {2'b00, pop};
        end
    end

//出队格的装载源 = 弹完之后的新队头
    always @(*) begin
        sidx = rptr + {2'b00, pop};
    end

//不在册时整条压 0：不让陈旧值/X 走到译码、冒险比较和下游载荷里。
//★ 有效性按【这一拍的指针】算（sidx != wptr）：q_dat 的读拿到的是沿前的旧值，
//  这一拍刚写进来的那一格还不是新数据 —— 拿它当有效会把陈旧值交付出去。
    always @(*) begin
        s0_data = q_dat[sidx[2:0]];
        stg0_v  = ~rst_q & ~clr & (sidx != wptr);
        nx_inst      = 32'd0;
        nx_addr      = 32'd0;
        nx_br_pred   = 1'b0;
        nx_jalr_pred = 32'd0;
        nx_r1        = 5'd0;
        nx_r2        = 5'd0;
        if (stg0_v) begin
            nx_inst      = s0_data[106:75];
            nx_addr      = s0_data[74:43];
            nx_br_pred   = s0_data[42];
            nx_jalr_pred = s0_data[41:10];
            nx_r1        = s0_data[9:5];
            nx_r2        = s0_data[4:0];
        end
    end

    always @(*) begin
        s1_data = q_dat[sidx[2:0] + 3'd1];
        stg1_v  = ~rst_q & ~clr & (sidx != wptr) & ((sidx + 4'd1) != wptr);
        nx1_inst      = 32'd0;
        nx1_addr      = 32'd0;
        nx1_br_pred   = 1'b0;
        nx1_jalr_pred = 32'd0;
        nx1_r1        = 5'd0;
        nx1_r2        = 5'd0;
        if (stg1_v) begin
            nx1_inst      = s1_data[106:75];
            nx1_addr      = s1_data[74:43];
            nx1_br_pred   = s1_data[42];
            nx1_jalr_pred = s1_data[41:10];
            nx1_r1        = s1_data[9:5];
            nx1_r2        = s1_data[4:0];
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
            h0_v <= 1'b0;
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
            end
//★ 不再用 advance 门控：队头指针会因为"这一拍有人推入 / 弹出 0/1/2 条"而变，
//  必须每拍重新装。旧结构只在 advance 那一拍装、且从当前 rptr 取，弹 1 条时会错位。
            else begin
                h0_inst <= nx_inst;
                h0_addr <= nx_addr;
                h0_br_pred <= nx_br_pred;
                h0_jalr_pred <= nx_jalr_pred;
                h0_r1 <= nx_r1;
                h0_r2 <= nx_r2;
                h0_v <= stg0_v;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            h1_inst <= 32'd0;
            h1_addr <= 32'd0;
            h1_br_pred <= 1'b0;
            h1_jalr_pred <= 32'd0;
            h1_r1 <= 5'd0;
            h1_r2 <= 5'd0;
            h1_v <= 1'b0;
        end
        else if (exec) begin
            if (flush_w) begin
                h1_inst <= 32'd0;
                h1_addr <= 32'd0;
                h1_br_pred <= 1'b0;
                h1_jalr_pred <= 32'd0;
                h1_r1 <= 5'd0;
                h1_r2 <= 5'd0;
                h1_v <= 1'b0;
            end
            else begin
                h1_inst <= nx1_inst;
                h1_addr <= nx1_addr;
                h1_br_pred <= nx1_br_pred;
                h1_jalr_pred <= nx1_jalr_pred;
                h1_r1 <= nx1_r1;
                h1_r2 <= nx1_r2;
                h1_v <= stg1_v;
            end
        end
    end

endmodule
