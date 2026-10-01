`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/28
// Design Name:
// Module Name: wport —— 写口级（乱序写回：完成即写 + 同 rd 同拍年轻者胜）
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   三个结果口（alu / mulu / lsu）→ 寄存器堆的两条物理写口。**两级：入口判定（组合）+ 出口寄存（一拍送出）**。
//   本文件内 always 块按【流水级数】排列：
//     入口级（组合）：flag_bus 翻译 → 候选有效性/年龄 → 选口 + 同 rd 压制 → 落 *_e、taken_mul、b0/b1
//     出口级（寄存）  ：把面向寄存器堆与 ROB 的那些整体打一拍
//   相位（以"这笔结果出现在结果口上"那拍 T 为参照）：入口 = T（组合）；出口 = T+1（寄存）。
//
//   | 输出 | 级别 | 去向 |
//   | --- | --- | --- |
//   | we_a·rd_a·data_a·idx_a / we_b·rd_b·data_b·idx_b | 出口寄存 | → regfile（写阵列 + 读侧旁路）、forw 点①（第 4/5 条源） |
//   | fin_alu·fin_mul·fin_ld（+idx） | 出口寄存 | → rob（置 ent_wr，队头才退得动） |
//   | taken_mul | **入口级组合** | → mulu（放行它撤值） |
//   | b0_* / b1_*（真落地那笔的 (rd,idx)） | **入口级组合** | → lsu / mulu（杀更老的同 rd 在途记录） |
//
//   ★ 四条不变式：① 选口优先序不可改（口 A = alu→mul；口 B = ld→mul 捡漏；流式源 alu/ld 等不了）；
//     ② 同 rd 撞两口只放行年轻那笔（老的当拍丢，年龄用 head 相对值、先截 4 位）；
//     ③ 被杀的笔不写、不广播，但照常"放行 + 回报"（否则单元死等、队头也不动）；
//     ④ **`taken_mul` 与 `b0/b1` 必须留在入口级**（理由见下）。
//   ★ 出口为什么打一拍：打拍前"判定 → 写口身份/写使能"这条深锥直接拉到 regfile 的 32×32 阵列 D 脚
//     与读旁路（实测 3010 个失败端点里 1088 条终点在阵列 D）⇒ 布线把锥摊到全片；打拍后锥只驱动本级的出口寄存器。
//   ★ 配套：值在出口多待一拍 ⇒ regfile 当拍采的读值看不到它、单元结果口下一拍又已换人
//     ⇒ 由 forw 点① 吃这两条写口寄存器补上（5 个前送源）。
//   ★ 为什么不设"出口冲刷门"：入口级的 pa 只能来自 c0_v/c1_v，那两条里已经带了
//     `flush_con_jump && (idx != bju_idx_q)` 的逐字判据，且 pa_idx 就是那个 idx
//     ⇒ 出口再加一道同判据的门恒成立、是死逻辑。反过来把它"修"成延后一拍会误杀：
//     冲刷那一拍躺在出口寄存器里的是【更老的、正确路】那笔（判定拍它就已在写口上）。
//
// Revision:
//   0.01 新建（写口级 + 寄存版 alu 结果）
//   0.02 alu 那一笔的寄存器搬进 alu.v ⇒ 本级退化成纯组合；处置回报改三条通道 fin_alu/fin_mul/fin_ld
//   0.03 出口整体打一拍（面向 regfile 与 ROB 的那些）；taken_mul 与 b0/b1 留在入口级组合
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module wport(
    input clk, rst,
//ROB 队头（年龄基准）
    input [2:0]  rob_head,
//alu 结果口（alu.v 内寄存一拍，一拍宽）
    input        we_alu,
    input [4:0]  rd_alu,
    input [31:0] data_alu,
    input [2:0]  idx_alu,
//mulu 结果口（保持到 taken_mul）
    input        we_mul,
    input [4:0]  rd_mul,
    input [31:0] data_mul,
    input [2:0]  idx_mul,
//lsu 结果口（一拍脉冲）
    input        we_ld,
    input [4:0]  rd_ld,
    input [31:0] data_ld,
    input [2:0]  idx_ld,
//已被判"永不落地"的笔：不写、不广播，但照常放行并回报
    input        kill_mul,
    input        kill_ld,
    input        kill_alu,
//F3：bju 判定出的"指令地址非对齐"（寄存版，与本级的呈现同拍）+ 判定那条自己的 ROB 号。
//  alu 的 kill_out 是寄存器 ⇒ 到本级永远晚一拍，撤销不掉故障 jalr 刚进写口的那笔 link
//  （它在判定拍就寄存了）⇒ 这里做【组合】撤销：挂着的那笔就是被判定那条时当拍压掉。
//两个不同的号，对两种情形：
//  bju_idx_i = 判定【输入级】那一条（= 被判那条）—— 判定当拍撤销"故障那条自己的写"（jalr 的 link）
//  bju_idx_q = 判定【寄存】后那一条 —— 冲刷拍撤销"错路那条的写"（它在判定拍已寄存，号与判定那条不同）
    input        bju_exc,
    input [2:0]  bju_idx_i,
    input [2:0]  bju_idx_q,
//flag_bus：进模块先逐位翻译成原名，判定处直接用（不在模块内合成新名字）
    input [11:0] flag_bus,
//写口（到寄存器堆，出口寄存一拍）
    output reg        we_a,
    output reg [4:0]  rd_a,
    output reg [31:0] data_a,
    output reg [2:0]  idx_a,
    output reg        we_b,
    output reg [4:0]  rd_b,
    output reg [31:0] data_b,
    output reg [2:0]  idx_b,
//放行 mulu 的那一笔（可以撤值了）—— **入口级组合**，不进出口寄存
    output reg        taken_mul,
//处置回报（给 ROB：落地或被丢弃都算这一项完了）—— 与写口同拍，出口寄存一拍
    output reg        fin_alu,
    output reg [2:0]  fin_alu_idx,
    output reg        fin_mul,
    output reg [2:0]  fin_mul_idx,
    output reg        fin_ld,
    output reg [2:0]  fin_ld_idx,
//落地广播（给 lsu/mulu 杀老写：只广播真落地的那笔）—— **入口级组合**，不进出口寄存
    output reg        b0_we,
    output reg [4:0]  b0_rd,
    output reg [2:0]  b0_idx,
    output reg        b1_we,
    output reg [4:0]  b1_rd,
    output reg [2:0]  b1_idx
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
//（本模块原来是纯组合、没有这一拍；出口寄存级加进来之后必须有。）
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus 逐位翻译成原名
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    reg stall_rob_full, stall_pc_redir;
    reg stall_lsu_haz, stall_lsu_full;
    reg stall_mulu_haz, stall_mulu_div;
    reg stall_icache_miss, stall_bus_hold;

//入口级（组合）的输出，由下一拍的出口寄存级采走
    reg        we_a_e, we_b_e;
    reg [4:0]  rd_a_e, rd_b_e;
    reg [31:0] data_a_e, data_b_e;
    reg [2:0]  idx_a_e, idx_b_e;
    reg        fin_alu_e, fin_mul_e, fin_ld_e;

//三个候选：有效性与年龄
    reg        c0_v, c1_v, c2_v;
    reg [3:0]  c0_age, c1_age, c2_age;
//端口归属与同 rd 压制
    reg [1:0]  pa, pb;
    reg [4:0]  pa_rd, pb_rd;
    reg [2:0]  pa_idx, pb_idx;
    reg [3:0]  pa_age, pb_age;
    reg        sup_a, sup_b;

//===============================================================
// 入口级（组合）：三个结果口 → "这一拍该处置谁"
//   本级输出全部落 *_e，由下一拍的出口寄存级采走；taken_mul 与 b0/b1 例外，留在本级
//===============================================================
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
    end

//候选有效性：不写 rd=0 的那笔不占口；已判死的笔也不占口（但下面照常放行/回报）
//★ 三条口的撤销判据必须【同构】：跳转冲刷那一拍，挂在本口上的若不是发起者自己那一笔，
//  那就是判定拍溜进来的错路条 ⇒ 不许落地。原先只有口 A（alu）挂了这一项，B（load）与 mul
//  没挂 ⇒ 冲刷拍在途 load 照样把值写进寄存器堆，用错路的值污染寄存器（实测 CoreMark：
//  跳转冲刷同拍一笔在途 load 把 callee 的返回值写进 a0，把前一个调用该留下的值覆盖掉）。
    always @(*) begin
        c0_v   = we_alu && (rd_alu != 5'd0)
              && ~(kill_alu
                 | (bju_exc && (idx_alu == bju_idx_i))          // 判定当拍：故障那条自己的写
                 | (flush_con_jump && (idx_alu != bju_idx_q)));  // 冲刷拍：挂的是错路那条
        c1_v   = we_mul && (rd_mul != 5'd0)
              && ~(kill_mul
                 | (flush_con_jump && (idx_mul != bju_idx_q)));
        c2_v   = we_ld && (rd_ld != 5'd0)
              && ~(kill_ld
                 | (flush_con_jump && (idx_ld != bju_idx_q)));
        c0_age = {1'b0, (idx_alu - rob_head)};
        c1_age = {1'b0, (idx_mul - rob_head)};
        c2_age = {1'b0, (idx_ld - rob_head)};
    end

//端口归属：口 A 只给 alu/mul（ld 走 B），口 B 给 ld，mul 在 A 没占上时捡 B
    always @(*) begin
        pa     = 2'd3;
        pb     = 2'd3;
        pa_rd  = 5'd0;
        pb_rd  = 5'd0;
        pa_idx = 3'd0;
        pb_idx = 3'd0;
        pa_age = 4'd0;
        pb_age = 4'd0;
        if (c0_v) begin
            pa     = 2'd0;
            pa_rd  = rd_alu;
            pa_idx = idx_alu;
            pa_age = c0_age;
        end
        else if (c1_v) begin
            pa     = 2'd1;
            pa_rd  = rd_mul;
            pa_idx = idx_mul;
            pa_age = c1_age;
        end
        if (c2_v) begin
            pb     = 2'd2;
            pb_rd  = rd_ld;
            pb_idx = idx_ld;
            pb_age = c2_age;
        end
        else if (c1_v && (pa != 2'd1)) begin
            pb     = 2'd1;
            pb_rd  = rd_mul;
            pb_idx = idx_mul;
            pb_age = c1_age;
        end
//同 rd 撞两个口：只放行更年轻的那笔，老的那笔当拍丢弃（置 sup，写使能不发）
        sup_a = 1'b0;
        sup_b = 1'b0;
        if ((pa != 2'd3) && (pb != 2'd3) && (pa_rd == pb_rd)) begin
            if (pa_age > pb_age) begin
                sup_b = 1'b1;
            end
            else begin
                sup_a = 1'b1;
            end
        end
    end

//写口 / 放行 / 回报 / 广播（落 *_e）
//  放行口径 = "你这笔已经被处置掉了"（不是"你落地了"）—— rd=0 的写不占口也不落地，同样要放行，
//  否则 mulu 会永远举着一笔没人取的值（死等）。
//  ★ b0/b1 走【入口级组合】：单元侧（lsu/mulu）的杀老写是"当拍比较、下一沿登记"
//    ⇒ 广播必须与候选上口那一拍同步，才能压住"下一拍照样上口的更老那笔"。
//    把它挪进出口寄存器会开出跨拍重排窗口：年轻者进出口那拍、更老的那笔正好上另一个口，
//    单元里的 kill 登记还差一拍 ⇒ 老的写在 T+2 盖掉年轻的（实测机理见 lsu.v 的 s3_kl 登记）。
//  ★ taken_mul 也留在入口级：mul 与 div 共用这根握手，寄存会让 m_pv 多举一拍
//    （写口重复取 ⇒ 同 rd 写两次），并提前清掉 mulu 的 wr_pend ⇒ 除法提交脉冲被冲走、结果永久丢失。
    always @(*) begin
        we_a_e    = 1'b0;
        rd_a_e    = 5'd0;
        data_a_e  = 32'd0;
        idx_a_e   = 3'd0;
        we_b_e    = 1'b0;
        rd_b_e    = 5'd0;
        data_b_e  = 32'd0;
        idx_b_e   = 3'd0;
        taken_mul = 1'b0;
        fin_alu_e = we_alu;
        fin_mul_e = we_mul && (kill_mul || (rd_mul == 5'd0) || (pa == 2'd1) || (pb == 2'd1));
        fin_ld_e  = we_ld;
        b0_we     = 1'b0;
        b0_rd     = 5'd0;
        b0_idx    = 3'd0;
        b1_we     = 1'b0;
        b1_rd     = 5'd0;
        b1_idx    = 3'd0;
        if (pa == 2'd0) begin
            data_a_e = data_alu;
        end
        else if (pa == 2'd1) begin
            data_a_e = data_mul;
        end
        if (pb == 2'd1) begin
            data_b_e = data_mul;
        end
        else if (pb == 2'd2) begin
            data_b_e = data_ld;
        end
        if ((pa != 2'd3) && ~sup_a) begin
            we_a_e  = 1'b1;
            rd_a_e  = pa_rd;
            idx_a_e = pa_idx;
            b0_we   = 1'b1;
            b0_rd   = pa_rd;
            b0_idx  = pa_idx;
        end
        if ((pb != 2'd3) && ~sup_b) begin
            we_b_e  = 1'b1;
            rd_b_e  = pb_rd;
            idx_b_e = pb_idx;
            b1_we   = 1'b1;
            b1_rd   = pb_rd;
            b1_idx  = pb_idx;
        end
        if (fin_mul_e)
            taken_mul = 1'b1;
    end

//===============================================================
// 出口级（寄存一拍）：写口级"走出去"
//   面向寄存器堆与 ROB 的整体打一拍；广播与放行留在上面那一级（见入口级的注释）
//   fin_*_idx 取入口级原样的 idx_*：与 fin_*_e 是同一个沿、同一个周期取样，天然配对
//===============================================================
    always @(posedge clk) begin
        if (rst_q) begin
            we_a <= 1'b0;
            rd_a <= 5'd0;
            data_a <= 32'd0;
            idx_a <= 3'd0;
            we_b <= 1'b0;
            rd_b <= 5'd0;
            data_b <= 32'd0;
            idx_b <= 3'd0;
            fin_alu <= 1'b0;
            fin_alu_idx <= 3'd0;
            fin_mul <= 1'b0;
            fin_mul_idx <= 3'd0;
            fin_ld <= 1'b0;
            fin_ld_idx <= 3'd0;
        end
        else begin
            we_a <= we_a_e;
            rd_a <= rd_a_e;
            data_a <= data_a_e;
            idx_a <= idx_a_e;
            we_b <= we_b_e;
            rd_b <= rd_b_e;
            data_b <= data_b_e;
            idx_b <= idx_b_e;
            fin_alu <= fin_alu_e;
            fin_alu_idx <= idx_alu;
            fin_mul <= fin_mul_e;
            fin_mul_idx <= idx_mul;
            fin_ld <= fin_ld_e;
            fin_ld_idx <= idx_ld;
        end
    end

endmodule
