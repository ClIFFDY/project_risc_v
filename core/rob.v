`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/27
// Design Name:
// Module Name: rob —— 重排序缓冲（reorder buffer）
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   执行可以乱序完成，但写寄存器堆【只走按序退这一条路】⇒ 同 rd 的先后由结构保证。
//   分配：每条指令占一项（包括不写 rd 的和 store —— "谁比谁老"必须覆盖每一条指令，
//         否则陷阱不知道自己在程序序里的位置）。
//   完成：执行单元把（值 + 索引 + 世代）报到完成口，本模块按索引把 wr/data 同条件同一沿
//         写进那项；非 valid 的槽、世代对不上的（冲刷/复用后的迟到上报）一律忽略。
//   退：  head（和 head+1）valid 且 wr 才退，2 宽、严格按序；退口就是寄存器堆的两条写口。
//   值读口（拉式）：发射级按它自己锁下的 {slot,gen} 来读 ent_data + 世代校验。
//   冲刷：作废比 flush_idx 更年轻的项；flush_incl = 边界那条也一起作废（只有中断拉高）。
//   陷阱：故障项的 (cause,pc,tval) 随自己的项存下，退到 head 时【不写寄存器堆】地退掉，
//         比它年轻的项全部作废，同时拉 trap_fire 把载荷交给上级 —— 精确异常。
//
// ★ 本文件按三个区组织（寄存器声明统一放在最前，Verilog 要求先声明后使用）：
//     分配区 —— tail 端：指针与占用、槽号与世代、入项时写进去的静态信息
//     读口区 —— 拉式值读口（不回"谁在生产"，那一层在发射级的本地生产者表里）
//     提交区 —— head 端：退役判据、冲刷、陷阱、提交口（写寄存器堆）
//   项状态阵列（v / wr / ex / data / jdg）是唯一被三块共写的对象，只能有一个 always 块，
//   放在提交区末尾：分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
//
// ★ 与流水级的对应（本模块没有自己的流水级，是"全核的状态机"；每个口挂错一拍就是
//   "回填打到错项 / 标记落在已退的槽上"这类静默错）：
//   alloc_en           —— 分配拍（iffu 出队格换新 / 该条进 idu1 输出级那一拍）
//   alu/alu2/mul/mul2/ld 完成口 —— 结果被写口级处置那一拍，比分配晚一拍起步，mul/ld 不定长
//   bju_judged/bju2_judged —— bju 判定寄存一拍后的 wb 拍（两条 lane 各一套）
//   exc_en             —— 该条在载荷里的那一拍（bju 的组合异常标记）
//   flush_con_rob      —— bju 判定寄存一拍后的 wb 拍；flush_idx = 那条分支/跳转自己的项
//   trap_fire          —— 故障项排到队头那一拍，上级同拍锁 mepc/mcause/mtval 并重定向
//   full/empty/occupancy —— 组合状态；full 喂 cont 的发射级压制
//
//   ★ 年龄判据一律用 `age = {1'b0,(索引-head_ptr)}`：拼接的操作数是自决定的，保证减法
//     在 4 位里做完再扩成 5 位；直接写 `(a-b) < ent_cnt` 会按 5 位求值，回绕时判错。
//
// ★ 拉式值读口的相位（idu2 的 Q 端当拍就要值）：
//     qry_slot/qry_gen 是发射级锁下的 {生产者槽, 世代}；
//     qry_data = ent_data[slot]（16:1 组合选择）；qry_ok = 在册 & wr & 世代匹配。
//   ★ ok=0 不等于"值是错的"，只等于"现在不在可读窗口里"：生产者还没 done，或者已经退休
//     （退休之后值在寄存器堆里，由发射级那边的 rdata 档兜）。
//   ★ 本模块**不做**"按寄存器号找生产者"的扫描：那是 16 项比较 × 每源一份 +
//     动态下标（head_ptr+si）读阵列 ⇒ 实测 12 级。生产者的身份由发射级在 D 级登记
//     （本地生产者表）后随指令带下来，这里只按已经给出的槽号取值。
//
// Dependencies:
//
// Revision:
//   Revision 0.01 - 新建
//   Revision 0.02 - 值读口改成拉式（删掉入队扫描 / 1 深载荷冻结 / 前送槽号 / rdy）；
//                   完成口按 lane 扩成 alu/alu2/mul/mul2/ld 五条
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module rob(
    input clk, rst,
//分配口。相位 = iffu 出队格换新那一拍（本模块无自有流水级）；两条 lane 各一套、独立推进
    input        alloc_en_0,
    input        alloc_en_1,
    input [4:0]  alloc_rd_0,
    input [4:0]  alloc_rd_1,
    input        alloc_we_0,
    input        alloc_we_1,
    input        alloc_jmp_0,
    input        alloc_jmp_1,
//M 类 / LOAD：即使 rd==0 也会回报完成口，不许在"发出即置可退"里退休
    input        alloc_pend_0,
    input        alloc_pend_1,
//分到的槽号与世代（随载荷走到执行单元，完成上报时带回校验）
    output reg [3:0]  alloc_idx_0,
    output reg [3:0]  alloc_idx_1,
    output reg        alloc_gen_0,
    output reg        alloc_gen_1,

//---- 拉式值读口：4 组（2 lane × 2 源）----
    input [3:0]  qry_slot_a_0, qry_slot_b_0, qry_slot_a_1, qry_slot_b_1,
    input        qry_gen_a_0,  qry_gen_b_0,  qry_gen_a_1,  qry_gen_b_1,
    output reg [31:0] qry_data_a_0, qry_data_b_0, qry_data_a_1, qry_data_b_1,
    output reg        qry_ok_a_0,   qry_ok_b_0,   qry_ok_a_1,   qry_ok_b_1,

//---- 生产者表重建口（idu2 的 pstat 在冲刷那一拍用它整表重建）----
//物理槽序，槽号即下标；rb_v 已经是"这一沿之后仍活着的项"（判据与项状态阵列逐字一致）。
    output reg [79:0] rb_rd,     // rb_rd[i*5 +: 5] = ent_rd[i]
    output reg [15:0] rb_gen,    // rb_gen[i]       = ent_gen[i]
    output reg [15:0] rb_v,

//完成口：写口级处置完一笔就回报（落地 / 被杀 / 不写），data / gen 与 done / idx 同源同拍
    input        bju_judged,
    input [3:0]  bju_idx,
    input        bju_gen,
    input        bju2_judged,
    input [3:0]  bju2_idx,
    input        bju2_gen,
    input        alu_done,
    input [3:0]  alu_idx,
    input [31:0] alu_data,
    input        alu_gen,
    input        alu2_done,
    input [3:0]  alu2_idx,
    input [31:0] alu2_data,
    input        alu2_gen,
    input        mul_done,
    input [3:0]  mul_idx,
    input [31:0] mul_data,
    input        mul_gen,
    input        mul2_done,
    input [3:0]  mul2_idx,
    input [31:0] mul2_data,
    input        mul2_gen,
    input        ld_done,
    input [3:0]  ld_idx,
    input [31:0] ld_data,
    input        ld_gen,
//结果口的接受条件（与完成回填 ent_data 的判据逐字同门）
    output reg        acc_alu_o,
    output reg        acc_alu2_o,
    output reg        acc_mul_o,
    output reg        acc_mul2_o,
    output reg        acc_ld_o,
//发出口：不写 rd 的项靠这一拍置"可退"（见完成段）
    input        issue_en_0,
    input [3:0]  issue_idx_0,
    input        issue_gen_0,
    input        issue_en_1,
    input [3:0]  issue_idx_1,
    input        issue_gen_1,
//冲刷口（wb 拍发）：只作废比 flush_idx 更年轻的项；flush_incl = 边界那条也一起作废
    input        flush_con_rob,
    input [3:0]  flush_idx,
    input        flush_incl,
//异常冲刷：边界项已退时用来当场交付
    input        flush_con_exc,
//陷阱标记口（decoder 载荷拍）：(cause,pc,tval) 随故障指令自己的项存下，退到队头才交付
    input        exc_en,
    input [3:0]  exc_idx,
    input [3:0]  exc_cause,
    input [31:0] exc_pc,
    input [31:0] exc_tval,
//退口：寄存器堆的两条写口（口 A = head、口 B = head+1）
    output reg        commit_we0,
    output reg [4:0]  commit_rd0,
    output reg [31:0] commit_data0,
    output reg [3:0]  commit_slot0,
    output reg        commit_gen0,
    output reg        commit_we1,
    output reg [4:0]  commit_rd1,
    output reg [31:0] commit_data1,
    output reg [3:0]  commit_slot1,
    output reg        commit_gen1,
//状态
    output reg [3:0]  head_ptr,
//本拍退休的两条（给 lsu：退休过的项不许再按年龄冲刷作废）
    output reg        ret0_v, ret1_v,
    output reg [3:0]  ret0_idx, ret1_idx,
    output reg        ret0_gen, ret1_gen,
    output reg        full,
    output reg        flush_part,
    output reg        empty,
    output reg [4:0]  occupancy,
    output reg        trap_fire,
    output reg [3:0]  trap_cause,
    output reg [31:0] trap_pc,
    output reg [31:0] trap_tval
    );

localparam [4:0] DEPTH = 5'd16;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，各模块各自打一拍，彼此没有相位差。
reg rst_q;
always @(posedge clk) rst_q <= rst;


//==============================================================================
//  信号声明（Verilog 要求先声明后使用，三区的寄存器统一放在这里，按区归类）
//==============================================================================
//分配区
reg [3:0]  tail_ptr;
reg [4:0]  ent_cnt;
reg [3:0]  tail_ptr_nx;
reg [4:0]  ent_cnt_nx;
//每槽的目的寄存器号（写 x0 与"不写 rd"一律存 0）：值读口不按 rd 比较，这里只给提交口与
//"发出即置可退"用；退项与冲刷不必清（判据里 ent_v 与它相与）。异常载荷不逐槽存：见 exc_side_*。
reg [4:0]  ent_rd [0:DEPTH-1];
//每槽的世代位：每次分配翻转一次。完成上报必须世代一致才被接受 —— 这是"迟到上报
//打进新住户"的唯一根治手段（ent_v 挡不住"新住户也 valid"）。
reg        ent_gen  [0:DEPTH-1];
reg        ent_jmp  [0:DEPTH-1];
reg        ent_pend [0:DEPTH-1];
integer ai;

//读口区：4 组的取值中间量
reg [3:0]  qs_a_0, qs_b_0, qs_a_1, qs_b_1;
reg        qg_a_0, qg_b_0, qg_a_1, qg_b_1;
integer qi;
//重建口：物理槽序，静态下标（零 mux）
integer    rbi;
reg [4:0]  rb_age;
reg        rb_kill;

//提交区
reg [3:0]  head1_ptr;
reg [3:0]  head_ptr_nx;
reg        flush_any;
reg        head_ok;
reg        head1_ok;
reg        exc_hit_head;
reg        exc_hit_head1;
reg        exc_gone;
reg [4:0]  flush_age;
reg [4:0]  flush_keep;
reg        flush_ok;
reg [4:0]  ri_age;
//提交口的组合版（打拍前）：head_ok / ent_rd[head_ptr] / ent_data[head_ptr]
reg        commit_we0_comb, commit_we1_comb;
reg [4:0]  commit_rd0_comb, commit_rd1_comb;
reg [31:0] commit_data0_comb, commit_data1_comb;
reg [3:0]  commit_slot0_comb, commit_slot1_comb;
reg        commit_gen0_comb, commit_gen1_comb;
integer ri;

//项状态阵列（v / wr / ex / data / jdg）：唯一被三个区共写的对象，只能有一个 always 块
//（在本区末尾）：分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
reg        ent_v  [0:DEPTH-1];
reg        ent_wr [0:DEPTH-1];
reg        ent_ex [0:DEPTH-1];
//每槽的结果数据：与 ent_wr 同条件同一沿写，值读口/提交口读；退项/冲刷/陷阱不必清。
reg [31:0] ent_data [0:DEPTH-1];
reg        ent_jdg  [0:DEPTH-1];
reg [3:0]  exc_side_cause;
reg [31:0] exc_side_pc;
reg [31:0] exc_side_tval;


//==============================================================================
//  分配区（tail 端）：这一拍进几条、进到哪个槽、槽里写什么
//==============================================================================
//槽号与世代是组合输出的环位，作为载荷沿流水线传下去。
//★ lane1 的落位取决于 lane0 这一拍分不分配（两条 lane 独立推进，可能只有 lane1 来）。
    always @(*) begin
        alloc_idx_0 = tail_ptr;
        alloc_idx_1 = tail_ptr + {2'b00, alloc_en_0};
        alloc_gen_0 = ~ent_gen[tail_ptr];
        alloc_gen_1 = ~ent_gen[alloc_idx_1];
        occupancy  = ent_cnt;
        empty      = (ent_cnt == 4'd0);
        full       = (ent_cnt >= DEPTH - 4'd1);
    end

//下一拍的指针与占用数：先算退掉的（0/1/2 笔），再算分配的，冲刷最后覆盖。
//★ 判据必须与提交区的项状态块【逐字一致】：边界项失效时走"全冲"，这里也要把 ent_cnt/tail
//  归零，否则"项全清、计数还留着" ⇒ head 指向无效项 ⇒ 队头永不动。
    always @(*) begin
        ent_cnt_nx  = ent_cnt;
        tail_ptr_nx = tail_ptr;
        if (head_ok) begin
            ent_cnt_nx = ent_cnt_nx - 4'd1;
        end
        if (head1_ok) begin
            ent_cnt_nx = ent_cnt_nx - 4'd1;
        end
        if (alloc_en_0 && !full) begin
            tail_ptr_nx = tail_ptr_nx + 3'd1;
            ent_cnt_nx  = ent_cnt_nx + 4'd1;
        end
        if (alloc_en_1 && !full) begin
            tail_ptr_nx = tail_ptr_nx + 3'd1;
            ent_cnt_nx  = ent_cnt_nx + 4'd1;
        end
        if (flush_con_rob && !flush_ok) begin
            ent_cnt_nx  = 4'd0;
            tail_ptr_nx = head_ptr;
        end
        else if (flush_ok) begin
            ent_cnt_nx  = flush_incl ? flush_age : flush_keep;
            tail_ptr_nx = flush_incl ? flush_idx : (flush_idx + 3'd1);
        end
//陷阱交付：队头那条故障指令【不写寄存器堆】地退掉，比它年轻的项全部作废。
//★ 必须放在【冲刷赋值之后】：交付那一拍冲刷口可能同时有效，不能被 flush_keep 盖掉。
        if (trap_fire) begin
            tail_ptr_nx = head_ptr + 3'd1;
            ent_cnt_nx  = 4'd0;
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            tail_ptr <= 3'd0;
            ent_cnt  <= 5'd0;
        end
        else begin
            tail_ptr <= tail_ptr_nx;
            ent_cnt  <= ent_cnt_nx;
        end
    end

//入项：rd / 世代 / 是否会跳 / 会不会报完成口 —— 这四张表只有【复位 + 分配】两个写源。
    always @(posedge clk) begin
        if (rst_q) begin
            for (ai = 0; ai < DEPTH; ai = ai + 1) begin
                ent_rd[ai]   <= 5'd0;
                ent_gen[ai]  <= 1'b0;
                ent_jmp[ai]  <= 1'b0;
                ent_pend[ai] <= 1'b0;
            end
        end
        else begin
            if (alloc_en_0 && !full) begin
//前送表的目的寄存器号（保守侧：SYSTEM 也算写，带上 rd != 0 后只剩非法编码会"报了写其实不写"）。
                ent_rd[tail_ptr]   <= (alloc_we_0 && (alloc_rd_0 != 5'd0)) ? alloc_rd_0 : 5'd0;
                ent_gen[tail_ptr]  <= ~ent_gen[tail_ptr];
                ent_jmp[tail_ptr]  <= alloc_jmp_0;
                ent_pend[tail_ptr] <= alloc_pend_0;
            end
            if (alloc_en_1 && !full) begin
                ent_rd[alloc_idx_1]   <= (alloc_we_1 && (alloc_rd_1 != 5'd0)) ? alloc_rd_1 : 5'd0;
                ent_gen[alloc_idx_1]  <= ~ent_gen[alloc_idx_1];
                ent_jmp[alloc_idx_1]  <= alloc_jmp_1;
                ent_pend[alloc_idx_1] <= alloc_pend_1;
            end
        end
    end


//==============================================================================
//  读口区：拉式值读口（纯组合）
//  ★ 这里是唯一一处"按给出槽号读阵列"：16:1 选择（2 级），世代比较 1 级。
//    不在这里做"按寄存器号找生产者"的扫描 —— 那需要 16 项比较 × 每源一份，且动态下标
//    读阵列会串成 12 级。生产者身份由发射级带下来。
//==============================================================================
    always @(*) begin
        qs_a_0 = qry_slot_a_0; qg_a_0 = qry_gen_a_0;
        qs_b_0 = qry_slot_b_0; qg_b_0 = qry_gen_b_0;
        qs_a_1 = qry_slot_a_1; qg_a_1 = qry_gen_a_1;
        qs_b_1 = qry_slot_b_1; qg_b_1 = qry_gen_b_1;
        qry_data_a_0 = ent_data[qs_a_0];
        qry_data_b_0 = ent_data[qs_b_0];
        qry_data_a_1 = ent_data[qs_a_1];
        qry_data_b_1 = ent_data[qs_b_1];
//★ ok 必须带世代：只比槽号会把"槽复用之后的新住户"当成生产者，消费者会锁下别人的值。
        qry_ok_a_0 = ent_v[qs_a_0] & ent_wr[qs_a_0] & (ent_gen[qs_a_0] == qg_a_0);
        qry_ok_b_0 = ent_v[qs_b_0] & ent_wr[qs_b_0] & (ent_gen[qs_b_0] == qg_b_0);
        qry_ok_a_1 = ent_v[qs_a_1] & ent_wr[qs_a_1] & (ent_gen[qs_a_1] == qg_a_1);
        qry_ok_b_1 = ent_v[qs_b_1] & ent_wr[qs_b_1] & (ent_gen[qs_b_1] == qg_b_1);
    end

//生产者表重建口：只被 idu2 在【冲刷那一拍】用（平常它的值没人看）。
//★ 必须先把"这一拍要作废的项"摘掉再送出去：idu2 拿到的 ent_v 是沿前值，
//  被杀掉的项在上面照样是 1 —— 不摘就等于把死写者当活写者登记回去。
//  判据与提交区末尾的项状态阵列【逐字一致】（含"边界项已退 ⇒ 全冲"那一支）。
//★ 顺手只送 rd != 0 的项：写 x0 的在那张表里没有意义（表项本身用 v 挡掉）。
    always @(*) begin
        rb_rd  = 80'd0;
        rb_gen = 16'd0;
        rb_v   = 16'd0;
        for (rbi = 0; rbi < DEPTH; rbi = rbi + 1) begin
            rb_rd[rbi*5 +: 5] = ent_rd[rbi];
            rb_gen[rbi]       = ent_gen[rbi];
            rb_kill = 1'b0;
            if (trap_fire) begin
                rb_kill = 1'b1;
            end
            else if (flush_con_rob) begin
                if (!flush_ok) begin
                    rb_kill = 1'b1;
                end
                else begin
                    rb_age = {1'b0, (rbi[3:0] - head_ptr)};
                    if (ent_v[rbi] && (flush_incl ? (rb_age >= flush_age) : (rb_age > flush_age))
                        && (rb_age < ent_cnt))
                        rb_kill = 1'b1;
                end
            end
            rb_v[rbi] = ~rst_q & ent_v[rbi] & ~rb_kill;
        end
    end


//==============================================================================
//  提交区（head 端）：退役判据、冲刷、陷阱、提交口（写寄存器堆）
//==============================================================================
//退口判据 / 冲刷判据 / 状态输出
//  可退 = 在册且已完成。exc 标记项不算可退：它不写寄存器堆，由 trap_fire 交上级；
//  同拍标记（exc_en 正落在 head/head+1 上）必须【组合】就压掉退，否则会在 ent_ex 落盘前退掉。
    always @(*) begin
        head1_ptr   = head_ptr + 3'd1;
        ret0_v   = head_ok;
        ret0_idx = head_ptr;
        ret0_gen = ent_gen[head_ptr];
        ret1_v   = head1_ok;
        ret1_idx = head1_ptr;
        ret1_gen = ent_gen[head1_ptr];
//★ 冲刷那一拍一律不退：退是组合的，而冲刷的清位要到边沿才生效 ⇒ 错路那条若正好在 head 上
//  且已 done，会在冲刷拍就写进寄存器堆。
        flush_any = flush_con_rob;
//★ 边界项已经【退掉】的异常冲刷（典型是 store 非对齐）：标记会落进空槽 ⇒ 异常静默丢掉。
//  此时当场交付：顺序退保证"它退了 ⇒ 比它老的都退了"，精确性不变。
        exc_gone = flush_con_exc && !ent_v[flush_idx];
        exc_hit_head  = exc_en && (exc_idx == head_ptr);
        exc_hit_head1  = exc_en && (exc_idx == head1_ptr);
        head_ok   = (ent_cnt != 4'd0) && ent_v[head_ptr] && ent_wr[head_ptr] &&
                    !ent_ex[head_ptr] && !exc_hit_head && !flush_any &&
                    (!ent_jmp[head_ptr] || ent_jdg[head_ptr]);
        head1_ok  = head_ok && (ent_cnt >= 4'd2) && ent_v[head1_ptr] && ent_wr[head1_ptr] &&
                    !ent_ex[head1_ptr] && !exc_hit_head1 &&
                    (!ent_jmp[head1_ptr] || ent_jdg[head1_ptr]);
//★ 交付【不能】要求 ent_wr：被 lsu 拒绝的访存非对齐故障指令永远等不到完成口，
//  要 done 就既不退也不交付 ⇒ 整核死等。它排到队头就说明比它老的都退完了。
        trap_fire = ((ent_cnt != 4'd0) && ent_v[head_ptr] &&
                    (ent_ex[head_ptr] || exc_hit_head)) || exc_gone;
//陷阱载荷取【故障项自己那一份】（不是"当前哪条又出错了"）：队头就是程序序里最老的故障指令。
//★ 同拍命中（exc_hit_head）必须走标记口那一路 —— 侧寄存器此刻还是上一条异常留下的旧值。
        trap_cause = (exc_hit_head || exc_gone) ? exc_cause : exc_side_cause;
        trap_pc    = (exc_hit_head || exc_gone) ? exc_pc    : exc_side_pc;
        trap_tval  = (exc_hit_head || exc_gone) ? exc_tval  : exc_side_tval;
//★ 边界必须在【占用窗口内】才算"部分冲刷"：age(flush_idx) < ent_cnt。只查 ent_v 不够 ——
//  已退槽的 valid 位可能是脏的，边界落在窗口外时 age 的回绕会算出怪值，ROB 状态直接烂掉。
//  窗口外一律按"全冲"处理（活着的项本来就都比它年轻）。
        flush_age  = {1'b0, (flush_idx - head_ptr)};
        flush_keep = flush_age + 4'd1;
        flush_ok   = flush_con_rob && (flush_age < ent_cnt) && ent_v[flush_idx];
        flush_part = flush_ok;
//提交口：只有队头两项可退，且【写 rd 的项】才写寄存器堆。
//★ ent_rd != 0 与 head_ok 里的 !ent_ex 是两道独立的门：故障项的 ent_rd 可能非 0。
        commit_we0_comb   = head_ok && (ent_rd[head_ptr] != 5'd0);
        commit_rd0_comb   = ent_rd[head_ptr];
        commit_data0_comb = ent_data[head_ptr];
        commit_we1_comb   = head1_ok && (ent_rd[head1_ptr] != 5'd0);
        commit_rd1_comb   = ent_rd[head1_ptr];
        commit_data1_comb = ent_data[head1_ptr];
        commit_slot0_comb = head_ptr;
        commit_gen0_comb  = ent_gen[head_ptr];
        commit_slot1_comb = head1_ptr;
        commit_gen1_comb  = ent_gen[head1_ptr];
    end

//队头指针：退 0/1/2 笔；陷阱交付也前进一格（盖过上面）。
    always @(*) begin
        head_ptr_nx = head_ptr;
        if (head1_ok) begin
            head_ptr_nx = head_ptr + 3'd2;
        end
        else if (head_ok) begin
            head_ptr_nx = head_ptr + 3'd1;
        end
        if (trap_fire) begin
            head_ptr_nx = head_ptr + 3'd1;
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            head_ptr <= 3'd0;
        end
        else begin
            head_ptr <= head_ptr_nx;
        end
    end

//提交口打一拍：把"要不要写 / 写哪个 / 写什么"在提交拍算好、寄存，下一拍再写寄存器堆。
//★ 为什么打：regfile 是变址写阵列，综合器表达不出 CE，只能合成 D_i = (we&&rd==i)?data:Q_i
//  ⇒ head_ok 里的中断/异常/分支判定锥会整条压在阵列的 D 引脚上。打一拍后长锥止于此。
//★ 绝不能在 trap_fire / 冲刷拍清：head_ok 与 trap_fire 天然互斥（一个要 !ent_ex、一个要 ent_ex）；
//  冲刷只作废比边界年轻的项，而打拍里那笔在边界之前 ⇒ 清了会丢一笔合法写。
    always @(posedge clk) begin
        if (rst_q) begin
            commit_we0   <= 1'b0;
            commit_rd0   <= 5'd0;
            commit_data0 <= 32'd0;
            commit_we1   <= 1'b0;
            commit_rd1   <= 5'd0;
            commit_data1 <= 32'd0;
            commit_slot0 <= 4'd0;
            commit_slot1 <= 4'd0;
            commit_gen0  <= 1'b0;
            commit_gen1  <= 1'b0;
        end
        else begin
            commit_we0   <= commit_we0_comb;
            commit_rd0   <= commit_rd0_comb;
            commit_data0 <= commit_data0_comb;
            commit_we1   <= commit_we1_comb;
            commit_rd1   <= commit_rd1_comb;
            commit_data1 <= commit_data1_comb;
            commit_slot0 <= commit_slot0_comb;
            commit_slot1 <= commit_slot1_comb;
            commit_gen0  <= commit_gen0_comb;
            commit_gen1  <= commit_gen1_comb;
        end
    end

//结果口的"接受条件"（与下面完成回填 ent_data 的判据逐字同门）
    always @(*) begin
        acc_alu_o  = alu_done  && ent_v[alu_idx]  && (ent_gen[alu_idx]  == alu_gen);
        acc_alu2_o = alu2_done && ent_v[alu2_idx] && (ent_gen[alu2_idx] == alu2_gen);
        acc_mul_o  = mul_done  && ent_v[mul_idx]  && (ent_gen[mul_idx]  == mul_gen);
        acc_mul2_o = mul2_done && ent_v[mul2_idx] && (ent_gen[mul2_idx] == mul2_gen);
        acc_ld_o   = ld_done   && ent_v[ld_idx]   && (ent_gen[ld_idx]   == ld_gen);
    end

//------------------------------------------------------------------------------
//项状态阵列（v / wr / ex / data / jdg）：唯一被三个区共写的对象，只能有一个 always 块。
//分配建项 / 完成回填 / 退役清项 / 冲刷与陷阱清项。
//------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_q) begin
            for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                ent_v[ri]  <= 1'b0;
                ent_wr[ri] <= 1'b0;
                ent_ex[ri] <= 1'b0;
                ent_data[ri] <= 32'd0;
                ent_jdg[ri]  <= 1'b0;
            end
            exc_side_cause <= 4'd0;  exc_side_pc <= 32'd0;  exc_side_tval <= 32'd0;
        end
        else begin
//退掉的槽立刻失效；完成回填只认 valid 的槽。
            if (head_ok) begin
                ent_v[head_ptr]  <= 1'b0;
                ent_wr[head_ptr] <= 1'b0;
                ent_ex[head_ptr] <= 1'b0;
            end
            if (head1_ok) begin
                ent_v[head1_ptr]  <= 1'b0;
                ent_wr[head1_ptr] <= 1'b0;
                ent_ex[head1_ptr] <= 1'b0;
            end
            if (alloc_en_0 && !full) begin
                ent_v[tail_ptr]  <= 1'b1;
//★ 不写 rd 的项不由"出厂"给 wr，由 issue_en_0 那一拍置位（见下）：发射级是 1 深 Q 之后
//  "执行一定早于退役"不再成立 —— 没发出去的分支会被队头直接退掉，跳转就永远不发生。
                ent_wr[tail_ptr] <= 1'b0;
                ent_ex[tail_ptr] <= 1'b0;
                ent_jdg[tail_ptr]  <= 1'b0;
            end
            if (alloc_en_1 && !full) begin
                ent_v[alloc_idx_1]  <= 1'b1;
                ent_wr[alloc_idx_1] <= 1'b0;
                ent_ex[alloc_idx_1] <= 1'b0;
                ent_jdg[alloc_idx_1]  <= 1'b0;
            end
//完成回填：wr 与 data 必须【同条件、同一沿】写入（拆开会出现"wr 已置、data 还是旧值"的一拍窗口）。
//★ 世代校验必需：被冲刷/已释放的槽在新住户身上的迟到上报只有它能挡（ent_v 挡不住"新住户也 valid"）。
//★ "发出即置可退"只给【根本不会有完成口】的项：写 rd 的等完成口；M 类与 LOAD 即使 rd==0
//  也照样回报 ⇒ 必须排掉，否则提前退休后那个晚到的完成口会砸到槽的新住户头上。
            if (issue_en_0 && ent_v[issue_idx_0] && (ent_gen[issue_idx_0] == issue_gen_0) &&
                (ent_rd[issue_idx_0] == 5'd0) && !ent_pend[issue_idx_0]) begin
                ent_wr[issue_idx_0] <= 1'b1;
            end
            if (issue_en_1 && ent_v[issue_idx_1] && (ent_gen[issue_idx_1] == issue_gen_1) &&
                (ent_rd[issue_idx_1] == 5'd0) && !ent_pend[issue_idx_1]) begin
                ent_wr[issue_idx_1] <= 1'b1;
            end
            if (acc_alu_o) begin
                ent_data[alu_idx] <= alu_data;
                ent_wr[alu_idx] <= 1'b1;
            end
            if (acc_alu2_o) begin
                ent_data[alu2_idx] <= alu2_data;
                ent_wr[alu2_idx] <= 1'b1;
            end
//两条 lane 的判定口各一套：同拍两条都判定时，一条都不能丢（丢了 ent_jdg 永远不置 ⇒ 队头卡死）。
            if (bju_judged && ent_jmp[bju_idx] && ent_v[bju_idx] &&
                (ent_gen[bju_idx] == bju_gen)) begin
                ent_jdg[bju_idx] <= 1'b1;
            end
            if (bju2_judged && ent_jmp[bju2_idx] && ent_v[bju2_idx] &&
                (ent_gen[bju2_idx] == bju2_gen)) begin
                ent_jdg[bju2_idx] <= 1'b1;
            end
            if (acc_mul_o) begin
                ent_wr[mul_idx]   <= 1'b1;
                ent_data[mul_idx] <= mul_data;
            end
            if (acc_mul2_o) begin
                ent_wr[mul2_idx]   <= 1'b1;
                ent_data[mul2_idx] <= mul2_data;
            end
            if (acc_ld_o) begin
                ent_wr[ld_idx]   <= 1'b1;
                ent_data[ld_idx] <= ld_data;
            end
//★ 标记陷阱时必须【同时置 wr】：故障指令的写被 pd 挡掉、也不会再有任何单元回报它，
//  只置 exc 不置 wr 它就退不掉 ⇒ ROB 排不空 ⇒ redir 等不到 rob_empty ⇒ 整核挂死。
            if (exc_en && ent_v[exc_idx]) begin
                ent_ex[exc_idx] <= 1'b1;
                ent_wr[exc_idx] <= 1'b1;
                exc_side_cause <= exc_cause;
                exc_side_pc    <= exc_pc;
                exc_side_tval  <= exc_tval;
            end
            if (alloc_en_0 && !full) ent_wr[tail_ptr] <= 1'b0;
            if (alloc_en_1 && !full) ent_wr[alloc_idx_1] <= 1'b0;
            if (flush_con_rob && !flush_ok) begin
//★ 边界项【已经退掉】也必须当成"全冲"：冲刷指令（分支）不写 rd、退得比谁都快，
//  等它把 flush_idx 送过来时那一项早已退掉；此时活着的项一定都比它年轻 ⇒ 全冲才对。
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    ent_v[ri]  <= 1'b0;
                    ent_wr[ri] <= 1'b0;
                    ent_ex[ri] <= 1'b0;
                end
            end
            else if (flush_ok) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    ri_age = {1'b0, (ri[3:0] - head_ptr)};
                    if (ent_v[ri] && (flush_incl ? (ri_age >= flush_age) : (ri_age > flush_age))
                        && (ri_age < ent_cnt)) begin
                        ent_v[ri]  <= 1'b0;
                        ent_wr[ri] <= 1'b0;
                        ent_ex[ri] <= 1'b0;
                    end
                end
            end
//陷阱交付拍：整条 ROB 作废。放在最后 ⇒ 它盖过同拍的一切（含"刚分配的那一项"）。
            if (trap_fire) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    ent_v[ri]  <= 1'b0;
                    ent_wr[ri] <= 1'b0;
                    ent_ex[ri] <= 1'b0;
                end
            end
        end
    end

endmodule
