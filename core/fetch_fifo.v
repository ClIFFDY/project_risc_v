`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/10/05
// Design Name:
// Module Name: fetch_fifo
// Project Name: 取指队列（icache 与 pre_decoder 之间的缓冲级）
// Target Devices:
// Tool Versions:
// Description:
//   夹在 icache 与 pre_decoder 之间的取指缓冲。
//
//   ★ 位置与分工：icache 整个留在【取指侧】（它交付、它预译码跳转、它喂 pc 与 bra_predict），
//     本队列只做两件事：① 前端跑得比后端快时把取到的指令存住（下游停顿、跳转气泡由它吸收）；
//     ② 让**成对发射完全由读侧决定** —— 看队头两条能不能配，能配就一次弹两条。
//
//   ★ 两个推进门必须分开：
//       f_req  = exec & ~flush_w & ~stall_pc_redir & ~stall_icache_miss  —— 取指侧
//       fill_n = pre_decoder 算的"本拍离开队列的条数"（后端停顿它全吃）  —— 读侧
//     前端【不吃】rob_full / lsu / mulu / bus_hold 这些后端停顿：后端停顿时它继续往队里灌，
//     灌到 full 再由 full 把前端按住；反过来队列不会被"后端停住"抽干
//     （凡是能让队列变空的原因（冲刷/redir 排队/取指 miss）同时也在读侧把 fill_n 拉成 0，
//      所以"读了 1 条而队列是空的"不会发生，不需要额外的空队列停顿）。
//
//   ★ 队头是【组合读 + 本拍写入旁路】：空队列时队头直接是本拍写进去的那条 ——
//     冲刷后落点那条不用多压一拍，与"不插队列"同拍就能被读侧取用。
//
//   ★ 条目布局（97 位，只存"队列取消不了的携带量"）：
//       [96:65] inst      [64:33] addr      [32] br_pred      [31:0] jalr_pred
//     addr / br_pred / jalr_pred 是前端针对【这一次取指】产生的量（addr 是 pc 值、
//     br_pred 与 jalr_pred 是 bra_predict 的表输出），后面谁也算不出来，必须随条目走；
//     而 rd / func10 / imm / r1 / r2 / 类位全部是指令字的纯函数，一律不存 ——
//     由 pre_decoder 在队头上就地解（与队头同拍），队列宽度因此从 156 位降到 97 位。
//     包内 RAW 也不存位：读侧拿 head0 的 rd 比 head1 的两个 rs 即可 —— 队列里的前一条
//     就是程序序上的前一条。
//
//   ★ 前端唯一的组合逻辑是【指令来源三态】（见下面 inst_eff 那块）：icache 交付的字与
//     bti 直送/压 NOP 之间选一个，选完就进槽。别的什么都不加 —— 队列的写数据就是
//     "icache 的寄存输出（经这一个 mux）+ 前端携带量"。
//
// Dependencies:
//
// Revision:
//   Revision 0.02 - 挪到 pre_decoder 之后；条目改成译码结果；成对判据移到读侧；深度 8。
//   Revision 0.03 - 条目改成"原始指令字 + 携带量"（97 位）；上游那一级 pre_decoder 删掉，
//                   译码回到读侧的 pre_decoder（原 mid_decoder）。
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module fetch_fifo(
    input clk, rst,
    input [11:0] flag_bus,
//取指重定向（pc.v 的 redir_go 落地那拍）：整队作废
    input flush_pc_redir,
//写口：icache 本拍交出的两个字 + 有效位；指令三态在本模块前端合成（见 inst_eff 那块）
    input [31:0] inst_out, inst_next,
    input        inst_valid, inst_valid_b,
//★ 三态（bra_predict 的 bti_sel_q）：0 = 用 icache 正常交付；1 = 用 bti 直送的目标对；
//  2 = 方向说跳但没缓存 ⇒ 交付 NOP。2026-10-06 试过把它压成"命中位 + 一个 redir_q 抑制"，
//  实测 41 支里 19 支变了（ld_st_dep +39、x0fwd 跑飞）—— 因为**错路交付这件事本来就已经
//  由 icache 的 inst_valid 标废了**（冲刷过的读数压成 NOP），队列这边无条件推进即可；
//  再叠一套 redir_q 就是同一件事的第二套机制，且相位对不上（译码期改向对、冲刷期改向错）。
//这一拍（组合）bti 命不命中（与 pc.v 吃的原本是同一根）：命中档前提要用
    input        bti_hit,
    input [1:0]  bti_sel,
    input [63:0] bti_inst,
//分支预测"跳"（bra_predict 的 br1）与 btb 命中（jalr 预测落点有效）：
//用来压掉"lane0 这一拍就改向"时的第二个字 —— 那是顺序错路。
    input        br1, btb_hit,
//lane1 的方向预测（bra_predict 出）
    input        br1_1,
//本拍这一对的携带量：地址（pc 的 aux_addr）、分支预测"跳"位、jalr 预测目标
    input [31:0] push_addr,
    input        push_br_pred,
    input [31:0] push_jalr_pred,
//读侧：本拍有几条【离开队列】（由 pre_decoder 按补空位算）
    input [1:0]  fill_n,
//★★ 选 lane / 装载条件（全部由 pre_decoder 组合给出，它解的就是本模块这两格）：
//  adv0/adv1 = 载荷那一侧"这一拍换不换内容"（adv1=0 即 lane1 被 load-use 顶住）；
//  cls1_0c   = "队头那条能不能当 lane1"（只有格1 换、格0 保持那一支要它）；
//  lane1_v_in= 配成对时 lane1 的有效位；rob_idx/rob_gen = 这一拍发给这两条的 ROB 号（跟内容一起锁）。
//  ★ 这几位都是【触发器 → 组合 → 触发器】，不构成组合环。
    input        adv0, adv1,
    input        cls1_0c, lane1_v_in,
    input [2:0]  rob_idx, rob_idx1,
    input        rob_gen, rob_gen1,
//队头两条（组合读 + 本拍写入旁路）：给 pre_decoder 的配对/装载判据与 rob 扫槽用
    output reg [31:0] nh0_inst, nh1_inst, nh0_addr, nh1_addr,
    output reg        nh0_br_pred,
    output reg        nh1_br_pred,
    output reg [31:0] nh0_jalr_pred,
    output reg [31:0] nh1_jalr_pred,
//队头两条的两个源寄存器号（条目里存好的字段）
    output reg [4:0]  nh0_r1, nh0_r2, nh1_r1, nh1_r2,
    output reg        nh0_v, nh1_v,
//★★★ 出队两格 = 【本模块的出口寄存器】（从 pre_decoder 的两个槽整体搬过来）：
//  装哪一条由 adv0/adv1 + cls1_0c 现定 —— 这就是"按两条 lane 的 stall 选输出 lane"：
//    格0 被取走（adv0）⇒ 下拍装队头；没取走 ⇒ 原地保持（**格内不搬数据**）。
//    格1 被取走（adv1）⇒ adv0 时装队头下一条、~adv0 时【装队头】（换 lane 输出，格内仍不搬）；
//                        adv1=0 ⇒ 原地保持。
//  号与内容【同门】锁存（idx/gen 跟指令走，出去以后就是载荷的 issue_idx/issue_gen）。
    output reg [31:0] h0_inst, h1_inst, h0_addr, h1_addr,
    output reg        h0_br_pred,
    output reg        h1_br_pred,
    output reg [31:0] h0_jalr_pred,
    output reg [31:0] h1_jalr_pred,
    output reg [4:0]  h0_r1, h0_r2, h1_r1, h1_r2,
    output reg        h0_v, h1_v,
    output reg [2:0]  idx_out, idx1_out,
    output reg        gen_out, gen1_out,
//★ 原来这里回给 rob 的扫槽使能（pop_v/pop2_v）已搬去 pre_decoder 的 scan_v ——
//  扫槽口的口径是"下一拍进载荷那条"，而"哪两条进载荷"现在由读侧的缓冲决定，
//  队列自己只知道推了几条指针，不再知道那两条是谁。
//取指侧预译码（原在 icache 里，2026-10-27 搬到本模块前端）：它们的消费者是 pc 的落点算术、
//bra_predict 的 br_en/jal、controller 的 ird_tmr，以及 icache 自己的 jalr 类 NOP 门。
    output reg        fch_br_en, fch_jal, fch_jalr,
    output reg [31:0] fch_off_beq, fch_off_jal,
//lane1 的 jal 预译码（只此一条）：pc 恒定 +8 之后控制转移可以落在第二个字上，而后端的
//jp_target 对普通 jal【没有出口】（它历来由取指侧兜）。branch/jalr 后端接得住（br2/jalr_fail
//只看自己的载荷），只有 jal 必须在这儿补。
    output reg        fch_br_eff,
    output reg        fch_jal_eff,
    output reg        fch_jalr_eff,
    output reg [31:0] fch_off_beq_eff,
    output reg [31:0] fch_off_jal_eff,
    output reg [31:0] fch_off_jalr_eff,
//★ 这一拍有没有改向（两 lane 三类合一）+ 改向的是不是 jalr：给 bra_predict 当 take 与"禁捕获"判据
    output reg        fch_ct_redir,
    output reg        fch_ct_jalr,
//★★ 命中档的前提：本拍有改向 且 BTIC 在 rd_key 上真有条目 —— pc 落 +8 档要求的正是这个，
//   不能把裸的 bti_hit 交给 pc：那会与"能否武装 FSM"脱钩 ⇒ 落点与交付对错位 ⇒ 下游组合判定自锁
    output reg        fch_hit_ok,
//BTIC 的键：**正在改向那条指令自己的键**（lane0 → pc；lane1 → pc+4）。
//与 BHT/BTB 的 rd_key 不是一回事：rd_key 选的是哪个 lane 需要预测，
//而注入那一拍交付的是注入对、lane0_ct 常常为 0 ⇒ 两者在这一拍会分叉（读错一格）。
    output reg [31:0] fch_bti_key,
//lane0 是不是控制转移（只看 opcode，在 inst_eff 上译）：给 bra_predict 定三张表服务哪条 lane
    output reg        lane0_ct,
//取指侧停：本拍放不下。icache 与 pc 的推进被它按住
    output reg        full
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
//取指侧：只吃"冲刷 + 重定向排队 + 取指自己 miss"
        f_req = exec & ~flush_w & ~stall_pc_redir & ~stall_icache_miss;
        f_adv = f_req & ~full;
    end

//RV32I 里与取指改向有关的 opcode
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_SYSTEM = 7'b1110011;

//写口这一拍两条各自的源寄存器号（组合译出，与 pre_decoder 同一套口径）
    reg [4:0]  w0_r1, w0_r2, w1_r1, w1_r2;

//提取分支/跳转立即数的函数（口径与原来 icache 里那两条逐字一致）
    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

//指令来源三态（口径与原来 icache 里 inst_eff_fch 那块逐字一致）：
//bti_sel=1 用 btb 直送的跳转目标那条；=2 交付 NOP（方向说跳但没缓存）；=0 用 icache 正常交付。
//inst_valid / inst_valid_b 为 0 表示那一个字送来的是无效读数（缺失垃圾 / 那次回填已被冲刷作废），
//压成 0 ⇒ 一条 NOP。
    reg [31:0] inst_eff, inst1_eff, inst1_eff_raw;
    always @(*) begin
        if (bti_sel == 2'd1) begin
            inst_eff  = bti_inst[63:32];
            inst1_eff = bti_inst[31:0];
        end
        else if (bti_sel == 2'd2) begin
            inst_eff  = 32'd0;
            inst1_eff = 32'd0;
        end
        else begin
            inst_eff  = 32'd0;
            inst1_eff = 32'd0;
            if (inst_valid)
                inst_eff = inst_out;
            if (inst_valid_b)
                inst1_eff = inst_next;
        end
        inst1_eff_raw = inst1_eff;
//★ lane1 jal 那一拍【必须空一拍】：pc 已经跳到 T1，下一拍交付的整对都是错路。
//  lane0 的 jal 靠 bti 注入/压 NOP 补上，lane1 没有注入通路，只能在本模块就地压一拍 NOP
//  （把两个字都清 0、且不许成对），否则错路两条会进队列 —— 实测 fence 的 j 5c 就是这么漏的。
    end

//取指侧分支/跳转预译码（原 icache 末尾那块，2026-10-27 搬来）：口径逐字同形 ——
//三态来源、立即数函数、冲刷那一支全部照搬，只是判定源换成本模块前端选出的 inst_eff。
//★ jal / br_en / offset_* 【不受 stage 门控】：它们的消费者里有取指地址那条环；
//  "冲刷拍别拿错路指令的 br1/jal 去改指定位"改在消费侧用 flush 直接挡。
//  只有 jalr 保留门控（它能顺带保证"冲刷拍不就地打 NOP"，挡拍条件是 jalr|jalr_fail）。
    always @(*) begin
        fch_br_en = 1'b0;
        fch_off_beq = 32'd0;
        fch_off_jal = 32'd0;
        fch_jal = 1'b0;
        fch_jalr = 1'b0;
        lane0_ct = 1'b0;
        if (!rst_q) begin
            case (inst_eff[6:0])
                OPCODE_JAL: begin
                    fch_off_jal = $signed(immJ(inst_eff));
                    fch_jal = 1'b1;
                    lane0_ct = 1'b1;
                end
                OPCODE_JALR: begin
                    if (!flush_w)
                        fch_jalr = 1'b1;
                    lane0_ct = 1'b1;
                end
                OPCODE_BRANCH: begin
                    fch_off_beq = $signed(immB(inst_eff));
                    fch_br_en = 1'b1;
                    lane0_ct = 1'b1;
                end
            endcase
        end
    end

//lane1 的 jal：只认 JAL（确定性落点，不需要预测）。fch1_jal_r 是"第二个字本身是 jal"，
//fch1_jal 才是【算数的】—— 要排掉三种：① lane0 这一拍已经改向（那 lane1 就是错路）；
//② 本拍正在压 lane1 jal 的空拍；③ bti 注入拍（那两个字的语义已经被换掉）。
//★ 必须与 pc 的改向【同一个门】（因此带上 f_adv）：j 被冻住那拍（队满）pc 并没跳，
//  这时若把空拍粘上去，就等于把那条 j 压成 NOP 又顺手让 pc 走了 +8 —— 整条跳转没了。
    reg fch1_jal_r, fch1_br_r, fch1_jalr_r;
    reg [31:0] fch1_off_jal, fch1_off_beq;
    always @(*) begin
        fch1_jal_r  = 1'b0;
        fch1_br_r   = 1'b0;
        fch1_jalr_r = 1'b0;
        fch1_off_jal = 32'd0;
        fch1_off_beq = 32'd0;
        if (!rst_q) begin
            case (inst1_eff_raw[6:0])
                OPCODE_JAL: begin
                    fch1_off_jal = $signed(immJ(inst1_eff_raw));
                    fch1_jal_r = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (br1_1) begin
                        fch1_off_beq = $signed(immB(inst1_eff_raw));
                        fch1_br_r = 1'b1;
                    end
                end
                OPCODE_JALR: begin
                    if (btb_hit)
                        fch1_jalr_r = 1'b1;
                end
            endcase
        end
    end

//★ 不再看 `bti_sel`（= 去掉原来那个 `(bti_sel == 2'd0)` 项）：lane1 的译码必须作用在
//  【本拍最终交付的那两个字】上，不管它来自 icache 还是 bti。原来把"字的来源"当成
//  "这条是不是真指令"的判据 ⇒ bti 注入那一拍（bti_sel=1，交付的恰恰就是目标那一对、
//  里面可能就有 lanel 的 jal）被整条否掉：实测 CoreMark 的 core_bench_list ——
//  `bgez a3,520` 落点包 (0x520,0x524) 由注入交付，包里 lane1 的 `j 530` 被挡 ⇒ 不改向
//  ⇒ 错路那对 (0x528,0x52C) 被当正常指令执行，链表游走被带偏，最后取到无人应答的地址
//  把核锁死。三态里那个"压 NOP"（bti_sel=2）交付的是全 0，等价于没有 jal ⇒ 不需要单独挡。
//lane0 与 lane1 这一拍各自的改向条件（组合译在最终交付的字上）
    reg lane0_redir;
    always @(*) begin
        lane0_redir = fch_jal | br1 | (fch_jalr & btb_hit);
    end

    always @(*) begin
        fch_ct_redir = lane0_redir | fch1_jal_r | fch1_br_r | fch1_jalr_r;
        if (lane0_redir)
            fch_ct_jalr = fch_jalr & btb_hit;
        else
            fch_ct_jalr = fch1_jalr_r;
    end

//命中档前提：本拍有改向、取指这一拍能推进（take 会武装 FSM）、且 BTIC 在 rd_key 上命中
    always @(*) begin
        fch_hit_ok = fch_ct_redir & f_adv & bti_hit;
    end

    always @(*) begin
        if (lane0_redir)
            fch_bti_key = push_addr;
        else
            fch_bti_key = push_addr + 32'd4;
    end

//★ 改向【统一成一组信号、lane0 优先】：lane0 这一拍改向就用 lane0 的，否则用 lane1 的。
//  pc.v 的四条支一字不改 —— 它吃的是这一组"有效"信号，lane 的差别全在这里消化掉。
//  ★ lane1 的偏移 = 它自己的立即数 + 4：pc 恒定 +8 之后 lane1 的指令地址是 pc − 4，
//    于是 `pc + (imm + 4) − 8 = (pc − 4) + imm` 正好是它的落点 —— 与 lane0 只差一个 lane 偏移。
//  ★ "冲刷那拍 icache 出 NOP、BTIC 下一拍顶替"由本模块前端那个三态 mux 承担
//    （bti_sel=2 出 NOP、=1 出注入对）⇒ 这里不需要任何"抑制/不推"的额外逻辑。
    always @(*) begin
        fch_off_jalr_eff = push_jalr_pred;   // 两 lane 的 jalr 落点都来自当拍 btb 读（服务谁就是谁的）
        if (lane0_redir) begin
            fch_br_eff      = br1;
            fch_jal_eff     = fch_jal;
            fch_jalr_eff    = fch_jalr & btb_hit;
            fch_off_beq_eff = fch_off_beq;
            fch_off_jal_eff = fch_off_jal;
        end
        else begin
            fch_br_eff      = fch1_br_r;
            fch_jal_eff     = fch1_jal_r;
            fch_jalr_eff    = fch1_jalr_r;
            fch_off_beq_eff = fch1_off_beq + 32'd4;
            fch_off_jal_eff = fch1_off_jal + 32'd4;
        end
    end

//写口这一拍两条各自的源寄存器号：只有真读这两个源的指令才填，别的留 0，免得在冒险比较里
//误命中（只要在途有一条 rd 撞上这个数，前送值就会盖掉操作数）。csrrwi/si/ci 的 [19:15] 是 zimm、
//不是寄存器号：只对 rs1 形（funct3=001/010/011）填。
//★ 为什么在【写侧】解、为什么条目要存这 10 位：rob 的扫槽与队头【同拍】、而且起点必须是普通
//  触发器。读侧现解的话，扫槽锥前面就多一整级译码 ⇒ 实测那条路 16 级 / 14.5ns（布线占 80%）、
//  20 条失败路径里 19 条都堆在 rob 的 fwd_slot/scan_y 上。存字段就是买这一级。
    always @(*) begin
        w0_r1 = 5'd0;
        w0_r2 = 5'd0;
        w1_r1 = 5'd0;
        w1_r2 = 5'd0;
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
        case (inst1_eff[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_BRANCH,
            OPCODE_LOAD, OPCODE_STORE: w1_r1 = inst1_eff[19:15];
        endcase
        if ((inst1_eff[6:0] == OPCODE_SYSTEM)
         && (inst1_eff[14:12] != 3'b101)
         && (inst1_eff[14:12] != 3'b110)
         && (inst1_eff[14:12] != 3'b111))
            w1_r1 = inst1_eff[19:15];
        case (inst1_eff[6:0])
            OPCODE_OP, OPCODE_BRANCH, OPCODE_STORE: w1_r2 = inst1_eff[24:20];
        endcase
    end

//本拍推几条的第二条：lane0 自己不改向、而且第二个字是可信的。
//★ 这里【只】压"错路"（lane0 这一拍就改向 ⇒ pc 落到别处，第二个字是顺序错路）：
//  br1 命中 / jal / jalr 预测命中 / bti 注入（那两拍只有一条或两条注入的字）四种。
//★ 绝不能把"第二条合法只是配不上对"也压掉：配不配是【读侧】按类位判的，压掉就是永久丢一条
//  指令（pc 已经 +8 过去了，它不会再来）。这一位管的是"推不推"，不是"配不配"。
//本拍推几条的第二条：lane0 自己这一拍不改向、而且第二个字可信。
//★ 只管"错路"（lane0 这一拍就改向 ⇒ 第二个字是顺序错路）；"配不上对"由 dispatch 决定，
//  这里不压（压了就是永久丢一条指令）。
    reg push1_en;
    always @(*) begin
        if (bti_sel == 2'd1)
            push1_en = ~(fch_jal | br1 | (fch_jalr & btb_hit));
        else if (bti_sel == 2'd2)
            push1_en = 1'b0;
        else
            push1_en = inst_valid_b & ~(fch_jal | br1 | (fch_jalr & btb_hit));
    end

    localparam [3:0] DEPTH = 4'd8;


//       [106:75] inst   [74:43] addr   [42] br_pred   [41:10] jalr_pred   [9:5] r1   [4:0] r2
    reg [106:0] q_dat [0:7];
    reg [3:0]  wptr, rptr;

    reg [3:0]  cnt_use, cnt_push;
    reg [3:0]  idx0, idx1;
    reg [106:0] push0_data, push1_data;
    reg [31:0] push_addr_l0;
    reg [106:0] h0_data, h1_data;
    reg         hd0_v, hd1_v;
    reg        wr0, wr1, clr;

//本拍要推几条：前端每拍至少一条（f_req 那一拍 —— 每拍都推，"错路那拍交付 NOP"也是推一条），
//成对那一拍两条。★ 不能用 wr0+wr1 反推：那样 wr0 一变这条也跟着变，两处口径会分叉。
    always @(*) begin
        cnt_push = 4'd0;
        if (f_req)
            cnt_push = cnt_push + 4'd1;
        if (f_req & push1_en)
            cnt_push = cnt_push + 4'd1;
    end

//写入的两条（布局见模块头）。lane1 的地址 = lane0 的 + 4（两条相邻，口径都是"指令地址+4"）；
//lane1 这一拍不可能是控制转移（成对判据里 lane1 只收非分支类）⇒ br_pred/jalr_pred 填 0。
    reg        push1_br_pred;
    reg [31:0] push1_jalr_pred;
    always @(*) begin
        push1_br_pred = 1'b0;
        push1_jalr_pred = 32'd0;
        if (inst1_eff[6:0] == OPCODE_BRANCH)
            push1_br_pred = br1_1;
        if ((!lane0_ct) && (inst1_eff[6:0] == OPCODE_JALR))
            push1_jalr_pred = push_jalr_pred;
    end

    always @(*) begin
        push0_data = {inst_eff, push_addr_l0, push_br_pred, push_jalr_pred, w0_r1, w0_r2};
        push1_data = {inst1_eff, push_addr, push1_br_pred, push1_jalr_pred, w1_r1, w1_r2};
    end

//lane0 的地址 = push_addr − 4。
//★ pc 现在是"下一个待取地址"、恒定 +8 ⇒ 本拍交付的那个字是 instr(pc − 8)，而条目 addr 的口径是
//  "指令地址 + 4" ⇒ 正好是 push_addr − 4。lane1 是它后面那条 ⇒ 直接就是 push_addr。
//★ 注入那一拍也是同一条式子：注入的是【目标那一对】{T, T+4} 而 pc 落 T+8（见 pc.v），
//  于是 lane0(=T) 的"指令地址+4"正好是 push_addr − 4、lane1 正好是 push_addr —— 口径统一。
    always @(*) begin
        push_addr_l0 = push_addr - 32'd4;
    end

    always @(*) begin
        cnt_use  = wptr - rptr;
    end

//★ full 按"占用 ≥ 4 就报"（阈值），不是"放不下才报"：
//  本拍要推的 1~2 条还在飞（写入在本拍结束），按"放不下"判会让取指在满/不满之间来回、
//  还把这 1~2 条算漏 ⇒ 周转不开。留够余量，取指推进才是稳定节拍。
//  物理槽给 8（2 的幂，`[2:0]` 直接索引；5 个槽要 mod-5 取模，不值当），
//  阈值 4 + 在飞 2 ⇒ 最高占用 6 < 8，不会写丢。
    always @(*) begin
        full = f_req & (cnt_use >= 4'd4);
    end

    always @(*) begin
        clr = flush_w | flush_pc_redir;
    end

//写口：**每拍都推**（f_adv 那一拍照推一条）。错路那一拍推的是 NOP —— 由 bti_sel=2（三态里
//  "方向说跳但没缓存"）或 icache 的 inst_valid=0（那次读数已被冲刷作废）在 inst_eff 上压出来。
//  ★ 不要再叠"改向那一拍不推"：那会让队列的占用相位跟取指错开半拍，下游配对/发射跟着变。
    always @(*) begin
        wr0 = f_adv;
        wr1 = f_adv & push1_en;
    end

//写口：push0/push1 各写自己那一条语句（FF 阵列允许两个写译码器）
    always @(posedge clk) begin
        if (wr0) begin
            q_dat[wptr[2:0]] <= push0_data;
        end
        if (wr1) begin
            q_dat[wptr[2:0] + 3'd1] <= push1_data;
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
        else if (wr0 | wr1) begin
            wptr <= wptr + cnt_push;
        end
    end

//读指针：清队时不动（那些条目由 wptr 作废）；正常按【本拍离开队列的条数】走
    always @(posedge clk) begin
        if (rst_q) begin
            rptr <= 4'd0;
        end
        else if (clr) begin
            rptr <= rptr;
        end
        else if (fill_n == 2'd2) begin
            rptr <= rptr + 4'd2;
        end
        else if (fill_n == 2'd1) begin
            rptr <= rptr + 4'd1;
        end
    end

//队头（组合读 + 本拍写入旁路）：**这是喂给出队寄存器的那一级**，不是交付口。
//idx 上有条目 ⟺ 它落在 [rptr, wptr-1] 里（已有），或它就是本拍写入的位置。
    always @(*) begin
        idx0 = rptr;
        idx1 = rptr + 4'd1;
    end

    always @(*) begin
        h0_data = q_dat[idx0[2:0]];
        hd0_v    = (idx0 != wptr);
        if ((idx0 == wptr) & wr0) begin
            h0_data = push0_data;
            hd0_v    = 1'b1;
        end
        else if ((idx0 == wptr + 4'd1) & wr1) begin
            h0_data = push1_data;
            hd0_v    = 1'b1;
        end
        h1_data = q_dat[idx1[2:0]];
//★ h1 在册的三种来源：它落在 [rptr, wptr-1] 里（已有条目）、或它正是本拍 push0 写的位置、
//  或它正是本拍 push1 写的位置。写成 `(idx1 != wptr) | wr0` 会在【队空】时把 idx1=rptr+1
//  判成"有条目"（那里其实还没写过）⇒ 拿垃圾当第二条去配对。
        hd1_v    = ((idx1 != wptr) & (idx1 != wptr + 4'd1))
                | ((idx1 == wptr) & wr0)
                | ((idx1 == wptr + 4'd1) & wr1);
        if ((idx1 == wptr) & wr0) begin
            h1_data = push0_data;
            hd1_v    = 1'b1;
        end
        else if ((idx1 == wptr + 4'd1) & wr1) begin
            h1_data = push1_data;
            hd1_v    = 1'b1;
        end
//切给读侧（纯按序出口）：队头 = 阵列里 rptr 那条，第二项 = rptr+1。
//★ 第二项也带全字段（addr/br_pred/jalr_pred）：它在 pre_decoder 的缓冲里可能被挪成 lane0。
//★ 不在册时把字压成 0：不让 X 走到译码/冒险比较里（B6-1 的口径）。
        nh0_inst      = 32'd0;
        nh0_addr      = h0_data[74:43];
        nh0_br_pred   = h0_data[42];
        nh0_jalr_pred = h0_data[41:10];
        nh0_r1        = 5'd0;
        nh0_r2        = 5'd0;
        nh0_v         = hd0_v;
        nh1_inst      = 32'd0;
        nh1_addr      = h1_data[74:43];
        nh1_br_pred   = h1_data[42];
        nh1_jalr_pred = h1_data[41:10];
        nh1_r1        = 5'd0;
        nh1_r2        = 5'd0;
        nh1_v         = hd1_v;
        if (hd0_v) begin
            nh0_inst = h0_data[106:75];
            nh0_r1   = h0_data[9:5];
            nh0_r2   = h0_data[4:0];
        end
        if (hd1_v) begin
            nh1_inst = h1_data[106:75];
            nh1_r1   = h1_data[9:5];
            nh1_r2   = h1_data[4:0];
        end
    end


//★★ 出队两格（发射槽）的装载：换新 / 保持 / 清 —— 与 pre_decoder 原来的两个槽块【逐字同形】。
//★ 两个都换：格0 拿队头、格1 拿队头下一条。
//★ 只有格1 换（格0 被按住）：**队头那条改落格1** —— 这就是"按 lane stall 换 lane 输出"，
//  格0 原地保持、格1 也不搬格内数据，后面的指令按序从格1 走，不必陪格0 一起等。
//★ 有效位：格1 = adv0 ? lane1_v_in : (队头在册 ∧ 队头能当 lane1)。
//★ 清只跟 rst / flush_w / exec 走，与原来一致（停顿拍不动 —— 停顿时 adv0/adv1 已经是 0）。
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
            h1_inst <= 32'd0;
            h1_addr <= 32'd0;
            h1_br_pred <= 1'b0;
            h1_jalr_pred <= 32'd0;
            h1_r1 <= 5'd0;
            h1_r2 <= 5'd0;
            h1_v <= 1'b0;
            idx1_out <= 3'd0;
            gen1_out <= 1'b0;
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
                h1_inst <= 32'd0;
                h1_addr <= 32'd0;
                h1_br_pred <= 1'b0;
                h1_jalr_pred <= 32'd0;
                h1_r1 <= 5'd0;
                h1_r2 <= 5'd0;
                h1_v <= 1'b0;
                idx1_out <= 3'd0;
                gen1_out <= 1'b0;
            end
            else begin
                if (adv0) begin
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
                if (adv1) begin
                    if (adv0) begin
                        h1_inst <= nh1_inst;
                        h1_addr <= nh1_addr;
                        h1_br_pred <= nh1_br_pred;
                        h1_jalr_pred <= nh1_jalr_pred;
                        h1_r1 <= nh1_r1;
                        h1_r2 <= nh1_r2;
                        h1_v <= lane1_v_in;
                    end
                    else begin
                        h1_inst <= nh0_inst;
                        h1_addr <= nh0_addr;
                        h1_br_pred <= nh0_br_pred;
                        h1_jalr_pred <= nh0_jalr_pred;
                        h1_r1 <= nh0_r1;
                        h1_r2 <= nh0_r2;
                        h1_v <= nh0_v & cls1_0c;
                    end
                    idx1_out <= rob_idx1;
                    gen1_out <= rob_gen1;
                end
                else begin
                    h1_inst <= h1_inst;
                    h1_addr <= h1_addr;
                    h1_br_pred <= h1_br_pred;
                    h1_jalr_pred <= h1_jalr_pred;
                    h1_r1 <= h1_r1;
                    h1_r2 <= h1_r2;
                    h1_v <= h1_v;
                    idx1_out <= idx1_out;
                    gen1_out <= gen1_out;
                end
            end
        end
    end

endmodule
