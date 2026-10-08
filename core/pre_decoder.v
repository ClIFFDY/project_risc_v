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
//     判出来换算成"本拍离开队列的条数"（fill_n）回给队列。队列写侧不必知道后端怎么配对。
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
    input [31:0] h0_addr, h1_addr,
    input        h0_br_pred, h1_br_pred,
    input [31:0] h0_jalr_pred, h1_jalr_pred,
    input        h0_v, h1_v,
//上压一格：post_decoder 那一级说这一拍只发了 lane0，两条都往上挪一格
//两条 lane 各自"这一拍换不换内容"（来自 post_decoder：被按住的那条保持，另一条照走）
    input        adv0, adv1,
//队头两条的源寄存器号：条目里存好的字段（在 fifo 前端解的，见那边的说明）。
//★ 本级不再自己解 rs：rob 的扫槽与队头同拍、而且要求起点短 —— 现解就多一级译码锥。
    input [4:0]  h0_r1, h0_r2, h1_r1, h1_r2,
//★★ **两级都存**（用户口径）：
//  fifo 的出队两格（上面那组 h*）= **F 级**：按两条 lane 的 stall 换 lane 输出、格内不搬数据；
//  本级再存一级（下面的 sl0_*/sl1_*）= **S 级**：F 的内容打一拍进 S，S 才是喂载荷那一级。
//  ⇒ 配对/装载/发号的判据吃 **F**（`h*`，决定"下一拍进 S 的是谁"）；
//    载荷面字段吃 **S**（`sl*`，决定"这一拍发给谁"）。两组判据必须分开，混用就差一拍。
//★★ 队头那两条（组合，来自 fetch_fifo 的队头读口）= **H 级**：
//  "下一拍要装进 F 的是谁"只有它说了算 ⇒ 配对判据(cls/m/raw_pair/lane1_v_in)、
//  装载使能(fill_n/alc_ld*)、发号用的 rd 全部吃它。**绝不能拿 F 的内容去判** ——
//  F 里坐的是"已经装进来的那对"，拿它判就是拿上一对去决定下一对装谁。
    input [31:0] nh0_inst, nh1_inst,
    input [4:0]  nh0_r1, nh0_r2, nh1_r1, nh1_r2,
    input        nh0_v, nh1_v,
//★ 随 F 一起过来的号/世代（fetch_fifo 与内容【同门】锁存），S 再锁一拍带走。
    input [2:0]  idx_stage, idx1_stage,
    input        gen_stage, gen1_stage,
//★ 给 fetch_fifo 的装载条件（它才是装载两格的人）："队头能不能当 lane1"与"配成对时 lane1 有效位"
    output       cls1_0c_out, lane1_v_in_out,
//★ S0 的有效位 = "载荷下一拍要执行的那条是真的"（喂 post_decoder 的幽灵项门 / csr 的中断受理门）
    output       sl0_v_out,
//★ 扫槽口的目标消费者号（喂 rob 的年龄判据）：就是"下一拍进 S 的那条"的号
    output [2:0] scan_cidx, scan_cidx1,
    output [31:0] inst_out,
    output [4:0]  rd_out,
    output [9:0]  func10_out,
    output [31:0] imm_alu_out,
    output [31:0] aux_addr_out,
    output        br_pred_taken_out,
    output [31:0] jalr_pred_addr_out,
    output [4:0]  r1_out, r2_out,
//lane1 那一组也要带 addr/br_pred/jalr_pred：上压一格时它会变成 lane0
    output [31:0] inst1_out,
    output [31:0] aux_addr_1_out,
    output        br_pred_taken_1_out,
    output [31:0] jalr_pred_addr_1_out,
    output [4:0]  rd1_out,
    output [9:0]  func10_1_out,
    output [31:0] imm1_alu_out,
    output [4:0]  r1_1_out, r2_1_out,
    output        lane1_v_out,
//槽1 这一拍被挡住（这一对没配上 / 槽空）：给队列当槽1 的换新门
//本拍有几条【离开取指队列】（回给队列推 rptr）：配上了两条、没配上一条、没有队头零条。
    output reg [1:0]  fill_n,
//本拍把队头收进载荷（= 队列的推进条件）：给 rob 的扫槽当时钟使能
    output reg        scan_v,
//本拍收进载荷的那两条各自的源寄存器号（给 rob 的扫槽）。
//★ 扫槽口的口径是"下一拍进载荷那条"（见 rob.v 的扫描块），供它的必须是【这一拍要收的那两条】。
//  原来直接从队列队头取，是因为"要收的"与"队头"恒等；读侧一旦有了缓冲格，两者会分叉。
    output reg [4:0]  f0_r1, f0_r2, f1_r1, f1_r2,
//★ ROB 分配口前移到【进槽那一拍】（= 进载荷的前一拍）：号必须按【程序序】发。
//  按在进载荷那拍发号会错：槽1 被按住（adv1=0）时槽0 照走，槽1 里压着的那条【更老】，
//  等它一起进载荷时按 (lane0, lane1) 发号 ⇒ 更老的那条反拿了更大的号 ⇒ ROB 程序序破
//  （实测 CoreMark 的 store 因此取到更老生产者的值，三项 CRC 全错）。
//  进槽就发号 ⇒ 两条的号永远与"它们进槽的先后"一致（= 队列给的序）✓
    input [2:0]  rob_idx, rob_idx1,
    input        rob_gen, rob_gen1,
//本拍【真的装进槽】的两条（= 要占号的两条）—— 与 fill_n / 槽寄存器分支逐字同门
    output       alloc_en0, alloc_en1o,
    output [4:0] alloc_rd0, alloc_rd1o,
    output       alloc_we0, alloc_we1o,
//满判据用的保守上界（不许含 payload_go，否则 take->need->full->payload_go 成环）
    output [1:0] need_pre,
//随载荷走的号 / 世代（进槽那一拍锁，与槽内容同生共死）
    output [2:0] idx_out, idx1_out,
    output       gen_out, gen1_out
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
//★★ S 级（本级自己的两个槽）：F（fetch_fifo 的出队两格）的下一拍
    reg [31:0] sl0_inst, sl0_addr, sl1_inst, sl1_addr;
    reg [31:0] sl0_jalr_pred, sl1_jalr_pred;
    reg        sl0_br_pred, sl1_br_pred, sl0_v, sl1_v, sl0_gen, sl1_gen;
    reg [4:0]  sl0_r1, sl0_r2, sl1_r1, sl1_r2;
    reg [2:0]  sl0_idx, sl1_idx;
    reg [4:0] alloc_rd0_c, alloc_rd1_c;
    reg [4:0]  rd_c;
    reg [9:0]  func10_c;
    reg [31:0] imm_c;
    always @(*) begin
        rd_c = 5'd0;
        func10_c = 10'd0;
        imm_c = 32'd0;
        case (sl0_inst[6:0])
            OPCODE_OP: begin
                rd_c = sl0_inst[11:7];
                func10_c = {sl0_inst[31:25], sl0_inst[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_c = immI(sl0_inst);
                rd_c = sl0_inst[11:7];
                func10_c = {7'd0, sl0_inst[14:12]};
                if (sl0_inst[14:12] == 3'b001 || sl0_inst[14:12] == 3'b101)
                    func10_c = {sl0_inst[31:25], sl0_inst[14:12]};
            end
            OPCODE_JAL: begin
                rd_c = sl0_inst[11:7];
            end
            OPCODE_JALR: begin
                rd_c = sl0_inst[11:7];
                imm_c = immI(sl0_inst);
            end
            OPCODE_BRANCH: begin
                imm_c = immB(sl0_inst);
                func10_c = {7'd0, sl0_inst[14:12]};
            end
            OPCODE_LOAD: begin
                imm_c = immI(sl0_inst);
                rd_c = sl0_inst[11:7];
            end
            OPCODE_STORE: begin
                imm_c = immS(sl0_inst);
            end
            OPCODE_LUI: begin
                imm_c = immU(sl0_inst);
                rd_c = sl0_inst[11:7];
            end
            OPCODE_AUIPC: begin
                imm_c = immU(sl0_inst) - 4'd4;
                rd_c = sl0_inst[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_c = sl0_inst[11:7];
                func10_c = {7'd0, sl0_inst[14:12]};
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
        case (sl1_inst[6:0])
            OPCODE_OP: begin
                rd1_c = sl1_inst[11:7];
                func10_1_c = {sl1_inst[31:25], sl1_inst[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm1_c = immI(sl1_inst);
                rd1_c = sl1_inst[11:7];
                func10_1_c = {7'd0, sl1_inst[14:12]};
                if (sl1_inst[14:12] == 3'b001 || sl1_inst[14:12] == 3'b101)
                    func10_1_c = {sl1_inst[31:25], sl1_inst[14:12]};
            end
            OPCODE_JAL: begin
                rd1_c = sl1_inst[11:7];
            end
            OPCODE_JALR: begin
                rd1_c = sl1_inst[11:7];
                imm1_c = immI(sl1_inst);
            end
            OPCODE_BRANCH: begin
                imm1_c = immB(sl1_inst);
                func10_1_c = {7'd0, sl1_inst[14:12]};
            end
            OPCODE_LOAD: begin
                imm1_c = immI(sl1_inst);
                rd1_c = sl1_inst[11:7];
            end
            OPCODE_STORE: begin
                imm1_c = immS(sl1_inst);
            end
            OPCODE_LUI: begin
                imm1_c = immU(sl1_inst);
                rd1_c = sl1_inst[11:7];
            end
            OPCODE_AUIPC: begin
                imm1_c = immU(sl1_inst) - 4'd4;
                rd1_c = sl1_inst[11:7];
            end
            OPCODE_SYSTEM: begin
                rd1_c = sl1_inst[11:7];
                func10_1_c = {7'd0, sl1_inst[14:12]};
            end
        endcase
    end

//成对判据（读侧）：两条都在册 ∧ head0 能当 lane0 ∧ head1 能当 lane1 ∧ 包里没有 RAW。
//★ 为什么 lane1 只能是 {OP/OP-IMM/LUI 非 M}：副路只复刻了一个 alu2，post_decoder 的 lane1
//  那一支在 case 之前就置 we1、且只处理这三种 ⇒ 别的类被当成 lane1 配对发射，就是往一个垃圾
//  寄存器写垃圾值、控制转移还不生效。配不上不是"慢一点"，是**必须不配**。
//★ 包内 RAW：head0 真要写寄存器、而且 head1 读的就是它 —— 队列里的前一条就是程序序上的
//  前一条，所以这一判比原来"查表 + 行首特判"更准。
//本拍推进（= 载荷锁存"推进"那一支的条件）：scan_v/fill_n 都要它，声明必须在使用者之前
    reg go;
    reg m0_c, m1_c, cls0_c, cls1_0c, cls1_1c, raw_pair, lane1_v_in;
    always @(*) begin
        m0_c    = (nh0_inst[6:0] == OPCODE_OP) && (nh0_inst[31:25] == 7'b0000001);
        m1_c    = (nh1_inst[6:0] == OPCODE_OP) && (nh1_inst[31:25] == 7'b0000001);
        cls0_c  = (((nh0_inst[6:0] == OPCODE_OP) || (nh0_inst[6:0] == OPCODE_OP_IMM)
                 || (nh0_inst[6:0] == OPCODE_LUI) || (nh0_inst[6:0] == OPCODE_AUIPC)) && !m0_c)
               || (nh0_inst[6:0] == OPCODE_LOAD) || (nh0_inst[6:0] == OPCODE_STORE);
//★ lane1 指令类型放开（2026-10-07）：原来只收 OP/OP-IMM/LUI，现在把 AUIPC 也放进来。
//  依据：lane1 的执行端就是 alu2，只要 post_decoder 的 lane1 那一支把
//  【pc 当 r1、立即数当 r2、func=ADD】配齐，AUIPC 与 lane0 那条走的是同一条算式
//  （lane1 的 pc 操作数 = pre_decoder 的 aux_addr_1_out，就是它自己的"地址+4"）。
//  ★ M（OP + funct7=1）仍不收：写回由 mulu 独立完成，lane1 没有 mulu 通路。
//  ★ LOAD/STORE/BRANCH/JAL(R)/SYSTEM 仍不收：那四类要单元级 lane 仲裁，见"放开"下一段。
        cls1_1c = ((nh1_inst[6:0] == OPCODE_OP) || (nh1_inst[6:0] == OPCODE_OP_IMM)
                || (nh1_inst[6:0] == OPCODE_LUI) || (nh1_inst[6:0] == OPCODE_AUIPC)) && !m1_c;
        raw_pair = 1'b0;
        if (alloc_rd0_c != 5'd0) begin
            if (alloc_rd0_c == nh1_r1)
                raw_pair = 1'b1;
            if (alloc_rd0_c == nh1_r2)
                raw_pair = 1'b1;
        end
        cls1_0c = ((nh0_inst[6:0] == OPCODE_OP) || (nh0_inst[6:0] == OPCODE_OP_IMM)
                || (nh0_inst[6:0] == OPCODE_LUI)) && !m0_c;
        lane1_v_in = nh0_v & nh1_v & cls0_c & cls1_1c & ~raw_pair;
    end
//给 fetch_fifo 的两根（它按这两位装载出队两格）
    assign cls1_0c_out   = cls1_0c;
    assign lane1_v_in_out = lane1_v_in;

//本拍有几条【离开队列】：有队头就至少一条（它被收进载荷）；配上了对第二条也跟着走（两条）；
//配不上第二条留在队列里（一条，下一拍它当队头再判）。没有队头就零条。
//★ 这个数取代原来的 pop2 回给队列：队列只按它推 rptr，不再自己判"能不能配"。
//★ 必须带推进门 go：停顿/冲刷那一拍队列一条都不许走（否则队头被抽掉、载荷却按"保持"处理）。
//★ 上压那一拍只补一条（lane0 由自己的 lane1 顶上，队头那条进 lane1）。
//★ 口径 = 两个槽【这一拍真的各装了几条】，不是"谁想换"：
//  槽0 装队头 ⟺ adv0 & nh0_v；槽1 装队头下一条 ⟺ adv0 & adv1 & lane1_v_in（配不上对就不装）；
//  只有槽1 换时它装队头 ⟺ ~adv0 & adv1 & nh0_v & cls1_0c。
//  写成"adv0 & adv1 就弹 2"会在 lane1_v_in=0 那拍把 h1 白弹掉 —— 又丢一条。
    always @(*) begin
        fill_n = 2'd0;
        if (adv0 & nh0_v)
            fill_n = fill_n + 2'd1;
        else if (adv1 & nh0_v & cls1_0c)
            fill_n = fill_n + 2'd1;
        if (adv0 & adv1 & lane1_v_in)
            fill_n = fill_n + 2'd1;
    end

//★ ROB 分配：本拍真的有哪几条【装进槽】就占哪几个号。
//  alc_ld0/alc_ld1 与 fill_n、与下面两个槽寄存器的分支【逐字同门】：
//  槽0 装队头 ⟺ adv0 & nh0_v；槽1 装队头下一条 ⟺ adv0 & adv1 & lane1_v_in；
//  只有槽1 换时它装队头 ⟺ ~adv0 & adv1 & nh0_v & cls1_0c。
//  号由 rob 按 tail_p 发：槽0 拿 tail_p、槽1 拿 tail_p + alc_ld0（槽0 也装时才是 +1）。
    reg alc_ld0, alc_ld1;
    always @(*) begin
        alc_ld0 = adv0 & nh0_v;
        alc_ld1 = (adv0 & adv1 & lane1_v_in) | (~adv0 & adv1 & nh0_v & cls1_0c);
    end
    assign alloc_en0  = alc_ld0;
    assign alloc_en1o = alc_ld1;
//★★ 发号用的 rd 必须走【与 rd_out 同一条判据】的译码（BRANCH/STORE/MISC_MEM 不写 rd ⇒ 0），
//  不能用 inst[11:7] 裸取。实测裸取会把 `bltu` 的 rs2 位段当成 rd 发给 ROB
//  （0062f863 的 [11:7]=16 ⇒ ROB 里多出一个"写 x16"的项，永远没人回报 ⇒ 队头退不掉 ⇒
//   满载死锁：启动后第 6 笔写就冻住）。这里就地译一次，只判"这一类写不写 rd"。
    always @(*) begin
        alloc_rd0_c = 5'd0;
        case (nh0_inst[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_JAL, OPCODE_JALR,
            OPCODE_LOAD, OPCODE_LUI, OPCODE_AUIPC, OPCODE_SYSTEM:
                alloc_rd0_c = nh0_inst[11:7];
            default:
                alloc_rd0_c = 5'd0;
        endcase
    end
    always @(*) begin
        alloc_rd1_c = 5'd0;
        case (nh1_inst[6:0])
            OPCODE_OP, OPCODE_OP_IMM, OPCODE_LUI, OPCODE_AUIPC:
                alloc_rd1_c = nh1_inst[11:7];
            default:
                alloc_rd1_c = 5'd0;
        endcase
    end
    assign alloc_rd0  = alloc_rd0_c;
    assign alloc_rd1o = adv0 ? alloc_rd1_c : alloc_rd0_c;
//"这一条要不要等写口回报"（口径与 post_decoder 的 issue_we 一致：只排除肯定不写的那几类）
    assign alloc_we0  = alloc_rd0 != 5'd0;
    assign alloc_we1o = alloc_rd1o != 5'd0;
//满判据的保守上界：本拍最多能装几条。★ 绝不含 payload_go（adv0/adv1）
    assign need_pre = {1'b0, nh0_v} + {1'b0, (nh0_v & nh1_v & lane1_v_in)};

//本拍把队头收进载荷（= 下面载荷块"推进"那一支的条件），给 rob 的扫槽当时钟使能。
//★ 必须与载荷锁存【同门】：扫槽提前一拍扫"下一拍进载荷那条"，扫早了扫晚了都是拿错人的 rs。
//★ 口径必须与载荷锁存【同门】：现在载荷是按 adv0/adv1 【逐槽】推进的，
//  所以扫槽的使能与 rs 也要逐槽跟着 —— 这一拍谁真的要进载荷，扫的就是谁的 rs。
    always @(*) begin
        scan_v = adv0 | adv1;
    end

//供给 rob 扫槽口的 rs（口径："下一拍进载荷那条"）。
//★ 槽1 被按住时，下一拍坐在槽1 里的还是【原来那条】（不换新）⇒ 供的必须是【它的】rs，
//  不能供新来那条的：否则"按住这条"的判据会跟着别人走 —— 要么永远解不开，要么提前解开。
    always @(*) begin
//格0 不换新那一拍（adv0=0），下一拍坐在格0 里的还是【原来那条】⇒ 供【它的】rs
        f0_r1 = adv0 ? h0_r1 : sl0_r1;
        f0_r2 = adv0 ? h0_r2 : sl0_r2;
//格1 同理；两格同拍都换时才轮到队头的下一条
        f1_r1 = adv1 ? h1_r1 : sl1_r1;
        f1_r2 = adv1 ? h1_r2 : sl1_r2;
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
        go      = exec & ~flush_w & ~stall_w;
    end

//★★ 本级不再自己存两个槽（2026-10-07：整段搬进 fetch_fifo 的"出队两格"）。
//  原因是这两格的装载是"按两条 lane 的 stall 换 lane 输出"决定的，而 stall（adv0/adv1）与配对判据
//  正好都在这一级算 —— 分散在两处会分裂成两套口径；搬过去以后 fifo 那两格就是唯一的调度寄存器。
//  本级的输出改由 fifo 那两格【组合译码】得到（fixture：同一指令不再存两遍，级数不变）。
//★★ S 级 = F 的下一拍（用户口径"predecoder 同样寄存"）：与 F 同门（adv0/adv1），
//  冲刷/rst 清、停顿拍原地保持。F 自己已经按 lane stall 把"哪一条进哪一格"选好了，
//  所以 S 只是【整齐地打一拍】，不再重复一遍配对判据（重复就会两处口径分叉）。
    always @(posedge clk) begin
        if (rst_q) begin
            sl0_inst <= 32'd0;
            sl0_addr <= 32'd0;
            sl0_br_pred <= 1'b0;
            sl0_jalr_pred <= 32'd0;
            sl0_r1 <= 5'd0;
            sl0_r2 <= 5'd0;
            sl0_v <= 1'b0;
            sl0_idx <= 3'd0;
            sl0_gen <= 1'b0;
            sl1_inst <= 32'd0;
            sl1_addr <= 32'd0;
            sl1_br_pred <= 1'b0;
            sl1_jalr_pred <= 32'd0;
            sl1_r1 <= 5'd0;
            sl1_r2 <= 5'd0;
            sl1_v <= 1'b0;
            sl1_idx <= 3'd0;
            sl1_gen <= 1'b0;
        end
        else if (exec) begin
            if (flush_w) begin
                sl0_inst <= 32'd0;
                sl0_addr <= 32'd0;
                sl0_br_pred <= 1'b0;
                sl0_jalr_pred <= 32'd0;
                sl0_r1 <= 5'd0;
                sl0_r2 <= 5'd0;
                sl0_v <= 1'b0;
                sl0_idx <= 3'd0;
                sl0_gen <= 1'b0;
                sl1_inst <= 32'd0;
                sl1_addr <= 32'd0;
                sl1_br_pred <= 1'b0;
                sl1_jalr_pred <= 32'd0;
                sl1_r1 <= 5'd0;
                sl1_r2 <= 5'd0;
                sl1_v <= 1'b0;
                sl1_idx <= 3'd0;
                sl1_gen <= 1'b0;
            end
            else begin
                if (adv0) begin
                    sl0_inst <= h0_inst;
                    sl0_addr <= h0_addr;
                    sl0_br_pred <= h0_br_pred;
                    sl0_jalr_pred <= h0_jalr_pred;
                    sl0_r1 <= h0_r1;
                    sl0_r2 <= h0_r2;
                    sl0_v <= h0_v;
                    sl0_idx <= idx_stage;
                    sl0_gen <= gen_stage;
                end
                else begin
                    sl0_inst <= sl0_inst;
                    sl0_addr <= sl0_addr;
                    sl0_br_pred <= sl0_br_pred;
                    sl0_jalr_pred <= sl0_jalr_pred;
                    sl0_r1 <= sl0_r1;
                    sl0_r2 <= sl0_r2;
                    sl0_v <= sl0_v;
                    sl0_idx <= sl0_idx;
                    sl0_gen <= sl0_gen;
                end
                if (adv1) begin
                    sl1_inst <= h1_inst;
                    sl1_addr <= h1_addr;
                    sl1_br_pred <= h1_br_pred;
                    sl1_jalr_pred <= h1_jalr_pred;
                    sl1_r1 <= h1_r1;
                    sl1_r2 <= h1_r2;
                    sl1_v <= h1_v;
                    sl1_idx <= idx1_stage;
                    sl1_gen <= gen1_stage;
                end
                else begin
                    sl1_inst <= sl1_inst;
                    sl1_addr <= sl1_addr;
                    sl1_br_pred <= sl1_br_pred;
                    sl1_jalr_pred <= sl1_jalr_pred;
                    sl1_r1 <= sl1_r1;
                    sl1_r2 <= sl1_r2;
                    sl1_v <= sl1_v;
                    sl1_idx <= sl1_idx;
                    sl1_gen <= sl1_gen;
                end
            end
        end
    end

//载荷面：全部由 S 级组合译码直出
    assign inst_out             = sl0_inst;
    assign aux_addr_out         = sl0_addr;
    assign br_pred_taken_out    = sl0_br_pred;
    assign jalr_pred_addr_out   = sl0_jalr_pred;
    assign r1_out               = sl0_r1;
    assign r2_out               = sl0_r2;
    assign rd_out               = rd_c;
    assign func10_out           = func10_c;
    assign imm_alu_out          = imm_c;
    assign inst1_out            = sl1_inst;
    assign aux_addr_1_out       = sl1_addr;
    assign br_pred_taken_1_out  = sl1_br_pred;
    assign jalr_pred_addr_1_out = sl1_jalr_pred;
    assign r1_1_out             = sl1_r1;
    assign r2_1_out             = sl1_r2;
    assign rd1_out              = rd1_c;
    assign func10_1_out         = func10_1_c;
    assign imm1_alu_out         = imm1_c;
    assign lane1_v_out          = sl1_v;
    assign idx_out              = sl0_idx;
    assign idx1_out             = sl1_idx;
    assign gen_out              = sl0_gen;
    assign gen1_out             = sl1_gen;
    assign sl0_v_out            = sl0_v;
    assign scan_cidx            = adv0 ? idx_stage  : sl0_idx;
    assign scan_cidx1           = adv1 ? idx1_stage : sl1_idx;

endmodule