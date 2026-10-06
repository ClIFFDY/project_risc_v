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
//   执行可以乱序完成，但写寄存器堆【只走按序退这一条路】⇒ 同 rd 的先后由结构保证，
//   不需要任何写序号比较（原 wbu 那套 序号 / 落地广播 / 杀老写 / 1 深槽 / 回压 全部作废）。
//
//   分配：一条指令进 E4 就占一项（**包括不写 rd 的和 store** —— "谁比谁老"必须覆盖每一条指令，
//         否则陷阱不知道自己在程序序里的位置）。alloc_idx 是组合输出的环位，作为载荷沿流水线传下去。
//   完成：执行单元把一笔结果（值 + 索引 + 世代）报到完成口，本模块按索引把 `wr` 与 `data`
//         【同条件、同一沿】写进那一项。非 valid 的槽、或世代对不上的（槽被冲刷/复用后的
//         迟到上报）一律忽略。
//   退：  head（和 head+1）valid 且 wr 才退，2 宽、严格按序。**退口就是寄存器堆的两条写口**
//         （口 A = head、口 B = head+1，见 cmt_*）⇒ 架构态永远是精确的、同 rd 的先后由结构保证。
//   ★ 每槽一个世代位：分配时翻转，随载荷走到执行单元，完成上报时带回来校验。它是"迟到上报
//     打进新住户"的唯一根治手段 —— 冲刷会把槽当场还回去，而 `ent_v` 挡不住"新住户也 valid"。
//   前送：扫描给出的"比我老里最年轻"那一槽，除槽号外还输出【值】（fwd_data1/2）与"已算完"位
//         （fwd_done1/2）—— 结果口只宽一两拍、阵列要等提交才更新，中间那段窗口只有这里兜得住。
//   前送：只在【比消费者更老】的项里找，取其中【最年轻】的 done 项 —— 需要"更老"这个条件，
//         因为比消费者年轻的写者绝不能前送给他。
//   冲刷：把比 flush_idx 更年轻的项作废，tail 回绕到它后面一格；flush_idx 不在窗口内就整条忽略。
//   陷阱：故障指令的 (cause,pc,tval) 随它自己的项存下；它退到 head 时【不写寄存器堆】地退掉
//         （ent_ex 在 head_ok 里就挡掉了正常退），比它年轻的项全部作废，同时拉 trap_fire
//         并把载荷交给上级（由上级去锁 mepc/mcause/mtval 并重定向）—— 这才是精确异常。
//
// ★★ 与流水级的对应（本核的级名：取指 → pre_decoder → mem_buf → decoder(ID/EX) → wb → 写回）：
//   本模块**没有自己的流水级**，它是"全核的状态机"；下面每个口都挂在上表某一级的信号上，
//   时序错一拍就是"回填打到错项 / 标记落在已退的槽上"这类静默错，所以逐个写清：
//
//   | 口 | 挂在哪一级 | 与那条指令的相位（以该指令自身为参照） |
//   | --- | --- | --- |
//   | `alloc_en` = post_decoder 的 `payload_go` | **decoder(ID/EX) 入口拍** | 指令进 decoder 载荷的**当拍**分配；`alloc_idx` = 本拍组合的 tail，随它当载荷往下走 |
//   | `alu_done/alu_idx`（= wport 的 `fin_alu/fin_alu_idx`） | **alu 结果寄存器那一拍** | 该指令进 decoder 载荷的**下一拍**：写口级把它处置掉（落地 / 被杀 / 不写）就回报 |
//   | `mul_done/mul_idx`（= wport 的 `fin_mul/...`） | **mul 结果被处置那一拍** | 比 decoder 载荷晚 2~4 拍以上（乘法多周期 + 写口被挤住时还要等） |
//   | `ld_done/ld_idx`（= wport 的 `fin_ld/...`） | **lsu 结果被处置那一拍** | 比 decoder 载荷晚 2 拍以上（lsu 两级 + 总线等待，不定长） |
//   | `exc_en/exc_idx/cause/pc/tval` | **decoder(ID/EX) 载荷拍** = 该指令在 decoder 载荷里的那一拍 | 用 bju 的【组合】异常标记（寄存版晚一拍，见 trap_unit→controller 的注释） |
//   | `flush_con_rob/flush_idx` | **wb 拍** | bju 判定寄存一拍后发冲刷；`flush_idx` = 那条分支/跳转自己的项（`bju.idx_q`） |
//   | `trap_fire` + `trap_cause/pc/tval` | **退口那一拍** | 故障项排到队头那一拍，上级用它在同拍锁 mepc/mcause/mtval 并重定向 |
//   | `full/empty/occupancy` | 组合状态 | `full` 喂 controller 的 `hold_issue`（发射级压制） |
//
//   ★ 循环判据一律用 `age = {1'b0,(索引-head_p)}`（**先截 4 位再比**）：3 位减法掉进和 4 位 cnt 的
//     比较里，Verilog 会按 4 位上下文求值 —— 回绕时算出 9 而不是 1，判据整体翻错（踩过，见 §30.4）。
//
// Dependencies:
//
// Revision:
//   Revision 0.01 - 新建（ROB 本体 + 模块级 tb）
//
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module rob(
    input clk, rst,
//分配口（decoder ID/EX 入口拍；本模块无自有流水级 ⇒ 所有口的相位都相对"这条指令进载荷那一拍"）
    input        alloc_en,
    input        alloc_we,
    input [4:0]  alloc_rd,
//lane1 的分配口：语义是【本包有没有 lane1】（= 载荷里的 lane1_v），**不是**"这拍要不要分配"。
//★ 绝不能接 payload_go 或任何含 full 的量：`alloc_need` 由它决定，而 `full` 又由 `alloc_need` 决定
//  ⇒ payload_go → alloc_en1 → need → full → stall_rob_full → flag_bus → payload_go 就是一条
//  零延时组合环（xsim 实测 Iteration limit 10000，正好停在自举结束、开始成对那一拍）。
//  真正的"这拍要不要写"由下面的 `alloc_en` 一起门控。
    input        alloc_en1,
    input        alloc_we1,
    input [4:0]  alloc_rd1,
//前送槽扫描口：消费者（下一拍进载荷那条）的两个源寄存器号，用 mid 级的 rs1_2/rs2_2。
//本模块按【程序序】分配（`alloc_en = payload_go`，与进载荷同沿）⇒ 扫描那一拍所有 `ent_v`
//的槽**一格不多一格不少**全是比它老的指令 ⇒ "比我老"不需要任何比较，`ent_v[s]` 即等价；
//"取最年轻的匹配" = 窗口里年龄最大的那个（年龄一律 4 位截断，见下面 red line）。
    input [4:0]  scan_rs1, scan_rs2,
//lane1 的两个源（同一个包的第二条）。扫描口与 1/2 完全同形、同拍、同 scan_go。
    input [4:0]  scan_rs1_1, scan_rs2_1,
//扫描的推进条件（= pre_decoder 本级的锁存条件）：为 1 才把这一拍的扫描结果打拍换新
    input        scan_go,
    output reg [2:0] fwd_slot1, fwd_slot2,
    output reg [2:0] fwd_slot3, fwd_slot4,
    output reg       fwd_hit1,  fwd_hit2,
    output reg       fwd_hit3,  fwd_hit4,
//前送值读口①【正常读】：索引就是本模块当拍扫描出的 fwd_slot1/2（不外引、不经任何选择）
//  ⇒ 这条路上没有任何 stall/flush 组合量，bju 的判定锥进不来。
    output reg [31:0] fwd_data1, fwd_data2,
    output reg [31:0] fwd_data3, fwd_data4,
    output reg        fwd_done1, fwd_done2,
    output reg        fwd_done3, fwd_done4,
//前送值读口②【停顿重读】：索引是消费者读锁存时锁下的槽（s1_q/s2_q，由 regfile 给）。
//  为什么要两组：消费者被停顿拖住时要用【锁存的】槽重读，而正常读用的是扫描槽 ——
//  若把两者在一根地址上 mux，stall_w（来自 flag_bus、即 bju 判定）就会被串进操作数数据路。
    input [2:0]  st_slot1, st_slot2,
    input [2:0]  st_slot3, st_slot4,
    output reg [31:0] st_data1, st_data2,
    output reg [31:0] st_data3, st_data4,
    output reg        st_done1, st_done2,
    output reg        st_done3, st_done4,
//完成口（写口级处置完一笔就回报：落地 / 被杀 / 不写；只认 valid 的槽）
//  data 与 gen 与 done/idx 同拍同源（完成口把"值"和"这一笔的世代"一起带回来）
    input        alu_done,
    input [2:0]  alu_idx,
    input [31:0] alu_data,
    input        alu_gen,
//lane1 的完成口：lane1 只可能是 ALU 类，所以只需要这一条（mulu/lsu 永远只服务 lane0）
    input        alu_done1,
    input [2:0]  alu_idx1,
    input [31:0] alu_data1,
    input        alu_gen1,
    input        mul_done,
    input [2:0]  mul_idx,
    input [31:0] mul_data,
    input        mul_gen,
    input        ld_done,
    input [2:0]  ld_idx,
    input [31:0] ld_data,
    input        ld_gen,
//冲刷口（wb 拍发：分支/跳转误预测；只作废比 flush_idx 更年轻的项。源名 flush_con_rob）
    input        flush_con_rob,
    input        flush_all,
    input [2:0]  flush_idx,
//异常冲刷（与 flush_con_rob 里的跳转/中断两类区分）：边界项已退时用来当场交付
    input        flush_con_exc,
//陷阱标记口（decoder 载荷拍，用 bju 组合判据）：故障指令把 (cause,pc,tval) 随自己的项存下，
//等它退到队头才交付
    input        exc_en,
    input [2:0]  exc_idx,
    input [3:0]  exc_cause,
    input [31:0] exc_pc,
    input [31:0] exc_tval,
//状态 / 退口（组合，当拍退当拍写寄存器堆）/ 前送结果
    output reg [2:0]  alloc_idx,
//本条分配拿到的世代位（= 槽在分配那一沿翻转【之后】的值），随载荷走到执行单元，
//完成上报时带回来与 ent_gen[槽] 比对做身份校验（见文件头"完成："那节）
    output reg        alloc_gen,
//lane1 那一项的环位与世代（= tail_p + 1 与它的翻转后世代）
    output reg [2:0]  alloc_idx1,
    output reg        alloc_gen1,
//提交口：寄存器堆的两条写口（口 A = head 更老、口 B = head+1 更年轻）
    output reg        cmt_we0,
    output reg [4:0]  cmt_rd0,
    output reg [31:0] cmt_data0,
    output reg        cmt_we1,
    output reg [4:0]  cmt_rd1,
    output reg [31:0] cmt_data1,
//队头索引：写口级与各单元算年龄的基准（年龄 = {1'b0,(idx - head_p)}）
//年龄基准：全核 4 个模块（`forw`/`wport`/`lsu`/`mulu`）共 21 个负载点都在拿它做减法基准，
//是个纯广播网。
    output reg [2:0]  head_p,
//停顿源：ROB 满（stall_rob_full，喂 controller 的发射级压制）
    output reg        full,
    output reg        empty,
    output reg [3:0]  occupancy,
//冲刷源：队头故障项交付（flush_rob_trap → pc 的重定向排队）
    output reg        trap_fire,
    output reg [3:0]  trap_cause,
    output reg [31:0] trap_pc,
    output reg [31:0] trap_tval
    );

localparam [3:0] DEPTH = 4'd8;

reg        ent_v  [0:DEPTH-1];
reg        ent_wr [0:DEPTH-1];
reg        ent_ex [0:DEPTH-1];
reg [3:0]  ent_cs [0:DEPTH-1];
reg [31:0] ent_pc [0:DEPTH-1];
reg [31:0] ent_tv [0:DEPTH-1];
//每槽的目的寄存器号（写 x0 与"不写 rd"一律存 0）—— 前送槽扫描用。
//只在复位与分配两处写，退项/冲刷都【不必】清：扫描判据里 `ent_v[s]` 与它相与，空槽天然被挡。
//（与 ent_cs/ent_pc/ent_tv 同款：那三个也只在复位/分配/异常标记时写。）
reg [4:0]  ent_rd [0:DEPTH-1];
//每槽的结果数据：完成口那一沿与 ent_wr 同条件写入，提交时由提交口读出。
//与 ent_cs/ent_pc/ent_tv/ent_rd 同款：只在复位/完成口写，退项/冲刷/陷阱【不清】（ent_v 门控挡空槽）。
reg [31:0] ent_data [0:DEPTH-1];
//每槽的世代位：每次分配翻转一次。完成上报必须与槽内存的世代一致才被接受 ——
//否则被冲刷/已释放的槽在新住户身上的【迟到上报】会把 ent_wr/ent_data 打错人（静默错值）。
reg        ent_gen  [0:DEPTH-1];

//扫描的工作量（纯组合，逐槽一份）
reg [3:0]  scan_ag [0:DEPTH-1];
//"在册且写真实 rd"：只依赖触发器、与 rs 无关 ⇒ 与比较那一段平行算好。
//有它，候选判据才能压成一次 6 输入比较（见扫描块）。
reg        scan_wv [0:DEPTH-1];
//候选掩码（含"同拍正在分配那一项"）与它对应的世代
reg        scan_c1 [0:DEPTH-1];
reg        scan_c2 [0:DEPTH-1];
reg        scan_g1 [0:DEPTH-1];
reg        scan_g2 [0:DEPTH-1];
//打拍后的扫描结果（读拍用）
reg        scan_y1q [0:DEPTH-1];
reg        scan_y2q [0:DEPTH-1];
reg        scan_g1q [0:DEPTH-1];
reg        scan_g2q [0:DEPTH-1];
reg        hit_c1, hit_c2;
reg [2:0]  slot_c1, slot_c2;
//一位有效热码 + "这一项已算完"：读口把【热码 & 数据】直接 OR 起来，不走"先算槽号再 mux"
reg        scan_w1 [0:DEPTH-1];
reg        scan_w2 [0:DEPTH-1];
reg        scan_v1 [0:DEPTH-1];
reg        scan_v2 [0:DEPTH-1];
reg        scan_m1 [0:DEPTH-1];
reg        scan_m2 [0:DEPTH-1];
reg        scan_y1 [0:DEPTH-1];
reg        scan_y2 [0:DEPTH-1];
//lane1 的两个源（扫描口 3/4）：结构与 1/2 逐字并联，只是各自的 rs 不同。
//★ 它们是【并联】的 ⇒ 扫描那一拍的深度一行不变（rs → 比较 → 8×8 → 槽号），
//  多出来的是面积与扇出，不是路径长度。8×8 归约的条数从 2 份变 4 份。
reg        scan_c3 [0:DEPTH-1];
reg        scan_c4 [0:DEPTH-1];
reg        scan_g3 [0:DEPTH-1];
reg        scan_g4 [0:DEPTH-1];
reg        scan_y3q [0:DEPTH-1];
reg        scan_y4q [0:DEPTH-1];
reg        scan_g3q [0:DEPTH-1];
reg        scan_g4q [0:DEPTH-1];
reg        hit_c3, hit_c4;
reg [2:0]  slot_c3, slot_c4;
reg        scan_w3 [0:DEPTH-1];
reg        scan_w4 [0:DEPTH-1];
reg        scan_v3 [0:DEPTH-1];
reg        scan_v4 [0:DEPTH-1];
reg        scan_m3 [0:DEPTH-1];
reg        scan_m4 [0:DEPTH-1];
reg        scan_y3 [0:DEPTH-1];
reg        scan_y4 [0:DEPTH-1];
integer si, sj;

reg [2:0]  tail_p;
reg [3:0]  cnt;
//本包要占几个槽：lane1 有效就 2，否则 1。满的判据必须按【整包】算 ——
//只余 1 槽而成对时，整包等下一拍，绝不进一半（进一半就破了"一个包是一个单位"）。
reg [2:0]  alloc_need;

reg [2:0]  head_nx;
reg [2:0]  tail_nx;
reg [3:0]  cnt_nx;

//提交口的组合版（打拍前）：head_ok / ent_rd[head_p] / ent_data[head_p]
reg        cmt_we0_e, cmt_we1_e;
reg [4:0]  cmt_rd0_e, cmt_rd1_e;
reg [31:0] cmt_data0_e, cmt_data1_e;
reg [2:0]  head1_p;
reg        flush_any;
reg        head_ok;
reg        head1_ok;
reg        exc_hit0;
reg        exc_hit1;
reg        exc_gone;
reg [3:0]  flush_age;
reg [3:0]  flush_keep;
reg        flush_ok;
reg [3:0]  ri_age;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，各模块各自打一拍，彼此没有相位差。
reg rst_q;
always @(posedge clk) rst_q <= rst;

//头部两项的可退判据 / 冲刷判据 / 退口与前送之外的状态输出
//  可退 = 有 occupant 且已完成。exc 标记项不算可退：它不写寄存器堆，由 trap_fire 交上级。
//  同拍标记（exc_en 正落在 head/head+1 上）必须【组合】就压掉退，否则陷阱项会抢在标记
//  写进 ent_ex 之前退出寄存器堆（模块 tb 踩到过）。年龄 age(i) = (i - head_p) mod 8，0 = 最老。
    always @(*) begin
        head1_p   = head_p + 3'd1;
//★ 冲刷那一拍不许退：退是组合的，而冲刷的清位要到边沿才生效 ⇒ 错路那条若正好在 head 上
//  且已 done，它会在冲刷拍就写进寄存器堆（实测 smoke 漏写一笔 x17）。冲刷拍一律不退。
        flush_any = flush_all || flush_con_rob;
//★ 边界项已经【退掉】的异常冲刷：故障指令（典型是 store 非对齐 —— 它的项出厂即 wr，
//  排到队头就退，而故障脉冲在 lsu 里寄存了一拍）已经不在 ROB 里了，标记会落进空槽 ⇒
//  这次异常会静默丢掉。此时就【当场交付】：顺序退保证"它退了 ⇒ 比它老的都退了"，
//  所以直接拿 controller 那拍还活着的 exc_cause/pc/tval 给上级，精确性不变。
        exc_gone = flush_con_exc && !ent_v[flush_idx];
        exc_hit0  = exc_en && (exc_idx == head_p);
        exc_hit1  = exc_en && (exc_idx == head1_p);
        head_ok   = (cnt != 4'd0) && ent_v[head_p] && ent_wr[head_p] &&
                    !ent_ex[head_p] && !exc_hit0 && !flush_any;
        head1_ok  = head_ok && (cnt >= 4'd2) && ent_v[head1_p] && ent_wr[head1_p] &&
                    !ent_ex[head1_p] && !exc_hit1;
//★ 交付【不能】要求 ent_wr：访存非对齐的故障指令是被 lsu【拒绝】的（不车厢、不拉总线），
//  它永远等不到任何完成口 ⇒ ent_wr 永远 0 ⇒ 若在这里要 done，它既不退也不交付 ⇒ 整核死等
//  （实测 exc_ldst_misalign / half_misalign 卡死）。它排到队头就说明比它老的都退完了，
//  直接交付即可；它自己不会写寄存器堆（head_ok 里已用 !ent_ex 挡掉）。
        trap_fire = ((cnt != 4'd0) && ent_v[head_p] &&
                    (ent_ex[head_p] || exc_hit0)) || exc_gone;
//交付给上级的陷阱载荷：故障项自己那一份（不是"当前哪条又出错了"）—— 队头就是程序序里
//最老的那条故障指令 ⇒ 精确异常要求"先交付它"，比它年轻的项一律作废。
//★ 同拍命中（exc_hit0：标记正好落在队头）必须走标记口那一路：此刻 ent_cs/ent_pc/ent_tv
//  里还是这一项【上一住户留下的旧值】（或者初值 0）—— 实测漏了这条：mepc/mcause 全交付成 0，
//  于是 mret 回到地址 4 反复重跑（exc_ecall 的事件流变成同一个 6 笔 store 的循环）。
        trap_cause = (exc_hit0 || exc_gone) ? exc_cause : ent_cs[head_p];
        trap_pc    = (exc_hit0 || exc_gone) ? exc_pc    : ent_pc[head_p];
        trap_tval  = (exc_hit0 || exc_gone) ? exc_tval  : ent_tv[head_p];
//★ 边界必须在【占用窗口内】才算"部分冲刷"：age(flush_idx) < cnt。
//  只查 ent_v[flush_idx] 不够 —— 已退槽的 valid 位可能是脏的，一旦边界落在窗口外，
//  age 的模 8 回绕会把 cnt 算成 8 之类的怪值，ROB 状态就烂了（实测 cnt=8 但 head 项无效 ⇒ 队头永不动）。
//  窗口外一律按"全冲"处理（活着的项本来就都比它年轻）。
//★ age 必须【先截成 4 位】再比较：3 位减法一旦出现在和 4 位 cnt 的比较里，Verilog 会把它
//  按 4 位上下文求值 ⇒ 回绕时算成 9 而不是 1，`age < cnt` 判假 ⇒ flush_ok=0 ⇒ 误走"全冲"，
//  把比边界更老的、还在途的 load 写回一起扔掉（实测 CoreMark 少一笔写、隔离复现过）。
//  拼接的操作数是自决定的，{1'b0, a-b} 保证减法在 3 位里做完再扩位。
        flush_age  = {1'b0, (flush_idx - head_p)};
        flush_keep = flush_age + 4'd1;
        flush_ok   = flush_con_rob && (flush_age < cnt) && ent_v[flush_idx];
        alloc_idx  = tail_p;
        alloc_gen  = ~ent_gen[tail_p];
        alloc_idx1 = tail_p + 3'd1;
        alloc_gen1 = ~ent_gen[tail_p + 3'd1];
        occupancy  = cnt;
        empty      = (cnt == 4'd0);
        full       = ({1'b0, cnt} + {1'b0, alloc_need} > {1'b0, DEPTH});
//提交口：只有队头两项可退，且【写 rd 的项】才写寄存器堆。
//★ `ent_rd != 0` 与 head_ok 里的 `!ent_ex` 是两道独立的门，缺一不可：
//  post_decoder 的 issue_we 把 SYSTEM 保守算作会写 ⇒ 故障项的 ent_rd 可能非 0。
        cmt_we0_e   = head_ok && (ent_rd[head_p] != 5'd0);
        cmt_rd0_e   = ent_rd[head_p];
        cmt_data0_e = ent_data[head_p];
        cmt_we1_e   = head1_ok && (ent_rd[head1_p] != 5'd0);
        cmt_rd1_e   = ent_rd[head1_p];
        cmt_data1_e = ent_data[head1_p];

    end

//本包要占几个槽。lane1 有效就 2 —— 被 full 的判据与 tail 的推进两处共用。
    always @(*) begin
        if (alloc_en1)
            alloc_need = 3'd2;
        else
            alloc_need = 3'd1;
    end

//下一拍的指针与占用数：先算退掉的（0/1/2 笔），再算分配的，冲刷最后覆盖（与旧写法优先级一致）
    always @(*) begin
        head_nx = head_p;
        if (head1_ok) begin
            head_nx = head_p + 3'd2;
        end
        else if (head_ok) begin
            head_nx = head_p + 3'd1;
        end
        cnt_nx  = cnt;
        tail_nx = tail_p;
        if (head_ok) begin
            cnt_nx = cnt_nx - 4'd1;
        end
        if (head1_ok) begin
            cnt_nx = cnt_nx - 4'd1;
        end
        if (alloc_en && !full) begin
            tail_nx = tail_p + alloc_need;
            cnt_nx  = cnt_nx + {1'b0, alloc_need};
        end
//★ 判据必须与时钟块那条【逐字一致】：边界项失效时走的是"全冲"，
//  时钟块清项、这里也必须把 cnt/tail 一起归零 —— 漏了就是"项全清掉、计数还留着"，
//  ROB 立刻不自洽（cnt≠0 而 head 项无效 ⇒ 队头永不动 ⇒ 挂死）。实测：部分冲刷启用后
//  这条路径第一次被走到，smoke 直接卡在 head 无效项上。
        if (flush_all || (flush_con_rob && !flush_ok)) begin
            cnt_nx  = 4'd0;
            tail_nx = head_p;
        end
        else if (flush_ok) begin
            cnt_nx  = flush_keep;
            tail_nx = flush_idx + 3'd1;
        end
//陷阱交付：队头那条故障指令【不写寄存器堆】地退掉，比它年轻的项全部作废（ROB 清空）。
//★ 必须放在【冲刷赋值之后】（时钟块里 trap_fire 的清项也是最后一条）：交付那一拍
//  冲刷口可能同时有效（异常/中断也冲 ROB 了），若被冲刷的 cnt=flush_keep 盖掉，
//  就出现"项全清、计数还留着" ⇒ head 指到无效项 ⇒ 挂死（实测 half_misalign）。
        if (trap_fire) begin
            head_nx = head_p + 3'd1;
            tail_nx = head_p + 3'd1;
            cnt_nx  = 4'd0;
        end
    end

//前送槽扫描（纯组合）：给两个消费者操作数各找一条"比它老、rd 匹配、且最年轻"的在册项。
//★ 年龄一律用 4 位截断 `{1'b0,(idx - head_p)}`，禁止 3 位裸比大小（回绕会判错）。
//★ 槽号唯一 ⇒ "最年轻"不会有并列。
//★★ 输入是【上一级】的 rs（pre_decoder 的 r1_pre/r2_pre：本拍正在进 pre_decoder 那条），
//   结果打拍、下一拍用。为什么要提前：否则"8×8 比较 + 归约"这一段会串进读拍
//   （`rs → 扫描 → 取数 → 旁路 → r1_data`），那正是读锥最深的一段。
//★ 「同拍正在分配的那一项」必须一起进候选：扫描读的是【上一拍】的 ent_v，而紧邻那条指令
//   正好在这一拍 alloc ⇒ 漏掉它 ⇒ hit=0 ⇒ 旁路取到阵列旧值。
//★ 进候选要【三样都补】：scan_c（参与"更年轻的摁掉更老的"的掩码）、scan_y（胜出者，
//   被归约成 hit 的那一份）、scan_g（世代）。第一版只补了 c 和 g —— scan_y 是循环开头由
//   scan_c 定的，后来改 c 不会回写它 ⇒ 新分配那条永远进不了命中集（实测 add a1,a0,x0 假 0）。
//★ 世代必须取【翻转后】的 alloc_gen：扫描在 alloc 那一拍，下一拍 ent_gen[tail_p] 就是它。
    always @(*) begin
        for (si = 0; si < DEPTH; si = si + 1) begin
            scan_ag[si] = {1'b0, (si[2:0] - head_p)};
//候选判据压成一次 6 输入比较：`scan_wv` 只依赖触发器（与 rs 平行算好），剩下 5 个 XNOR + 这一次
//与 = 2 级。写成 `ent_v & (rs!=0) & (ent_rd==rs)` 是 7 个输入 ⇒ 3 级（多一级）。
//★ 别再想"给 rs=0 预置一个永不匹配的比较值"—— ent_rd 的残留值覆盖 0..31，不存在这样的值。
//  `scan_wv` 自带 `|ent_rd`，而写 x0 的项 ent_rd=0 ⇒ rs=0 时天然不会命中。
            scan_wv[si] = ent_v[si] & |ent_rd[si];
            scan_c1[si] = scan_wv[si] & (ent_rd[si] == scan_rs1);
            scan_c2[si] = scan_wv[si] & (ent_rd[si] == scan_rs2);
            scan_c3[si] = scan_wv[si] & (ent_rd[si] == scan_rs1_1);
            scan_c4[si] = scan_wv[si] & (ent_rd[si] == scan_rs2_1);
            scan_g1[si] = ent_gen[si];
            scan_g2[si] = ent_gen[si];
            scan_g3[si] = ent_gen[si];
            scan_g4[si] = ent_gen[si];
            scan_y1[si] = scan_c1[si];
            scan_y2[si] = scan_c2[si];
            scan_y3[si] = scan_c3[si];
            scan_y4[si] = scan_c4[si];
        end
//同拍正在分配那一项：世代取【翻转后】的值（下一拍 ent_gen[tail_p] 就是它）
        if (alloc_en && (alloc_rd != 5'd0) && (alloc_rd == scan_rs1)) begin
            scan_c1[tail_p] = 1'b1;
            scan_y1[tail_p] = 1'b1;
            scan_g1[tail_p] = alloc_gen;
        end
        if (alloc_en && (alloc_rd != 5'd0) && (alloc_rd == scan_rs2)) begin
            scan_c2[tail_p] = 1'b1;
            scan_y2[tail_p] = 1'b1;
            scan_g2[tail_p] = alloc_gen;
        end
//lane1 的两个源：这一拍正在分配的【两项】都要进候选（tail_p 是 lane0、tail_p+1 是 lane1）。
//它俩都比"下一拍进载荷那个消费者"老，所以都是合法生产者。
        if (alloc_en && (alloc_rd != 5'd0) && (alloc_rd == scan_rs1_1)) begin
            scan_c3[tail_p] = 1'b1;
            scan_y3[tail_p] = 1'b1;
            scan_g3[tail_p] = alloc_gen;
        end
//★ 反向也要补：这一拍分配的 lane1 那项（tail_p+1）同样是"比消费者老"的生产者 ——
//  漏了它，紧跟成对包之后的消费者就扫不到 lane1 刚写的那个 rd（实测 tightdep：
//  a4 的项晚一拍进表，addi a5,a4,1 扫不到它、回落阵列读到旧值 0，a5 得 1 而不是 10）。
        if (alloc_en && alloc_en1 && (alloc_rd1 != 5'd0) && (alloc_rd1 == scan_rs1)) begin
            scan_c1[tail_p + 3'd1] = 1'b1;
            scan_y1[tail_p + 3'd1] = 1'b1;
            scan_g1[tail_p + 3'd1] = alloc_gen1;
        end
        if (alloc_en && alloc_en1 && (alloc_rd1 != 5'd0) && (alloc_rd1 == scan_rs2)) begin
            scan_c2[tail_p + 3'd1] = 1'b1;
            scan_y2[tail_p + 3'd1] = 1'b1;
            scan_g2[tail_p + 3'd1] = alloc_gen1;
        end
        if (alloc_en && alloc_en1 && (alloc_rd1 != 5'd0) && (alloc_rd1 == scan_rs1_1)) begin
            scan_c3[tail_p + 3'd1] = 1'b1;
            scan_y3[tail_p + 3'd1] = 1'b1;
            scan_g3[tail_p + 3'd1] = alloc_gen1;
        end
        if (alloc_en && (alloc_rd != 5'd0) && (alloc_rd == scan_rs2_1)) begin
            scan_c4[tail_p] = 1'b1;
            scan_y4[tail_p] = 1'b1;
            scan_g4[tail_p] = alloc_gen;
        end
        if (alloc_en && alloc_en1 && (alloc_rd1 != 5'd0) && (alloc_rd1 == scan_rs2_1)) begin
            scan_c4[tail_p + 3'd1] = 1'b1;
            scan_y4[tail_p + 3'd1] = 1'b1;
            scan_g4[tail_p + 3'd1] = alloc_gen1;
        end
//"有人比你更年轻"就把你摁掉（用含分配项的候选掩码，保证热码唯一）
        for (si = 0; si < DEPTH; si = si + 1) begin
            for (sj = 0; sj < DEPTH; sj = sj + 1) begin
                if (scan_c1[sj] && (scan_ag[sj] > scan_ag[si]))
                    scan_y1[si] = 1'b0;
                if (scan_c2[sj] && (scan_ag[sj] > scan_ag[si]))
                    scan_y2[si] = 1'b0;
                if (scan_c3[sj] && (scan_ag[sj] > scan_ag[si]))
                    scan_y3[si] = 1'b0;
                if (scan_c4[sj] && (scan_ag[sj] > scan_ag[si]))
                    scan_y4[si] = 1'b0;
            end
        end
        hit_c1  = 1'b0;
        hit_c2  = 1'b0;
        hit_c3  = 1'b0;
        hit_c4  = 1'b0;
        slot_c1 = 3'd0;
        slot_c2 = 3'd0;
        slot_c3 = 3'd0;
        slot_c4 = 3'd0;
        for (si = 0; si < DEPTH; si = si + 1) begin
            if (scan_y1[si]) begin
                hit_c1  = 1'b1;
                slot_c1 = si[2:0];
            end
            if (scan_y2[si]) begin
                hit_c2  = 1'b1;
                slot_c2 = si[2:0];
            end
            if (scan_y3[si]) begin
                hit_c3  = 1'b1;
                slot_c3 = si[2:0];
            end
            if (scan_y4[si]) begin
                hit_c4  = 1'b1;
                slot_c4 = si[2:0];
            end
        end
    end

//扫描结果打拍：换新与否跟 pre_decoder 的锁存条件同形（同一条指令走到读口那一拍才换）。
//★ 冲刷拍必须清：被冲掉的那条不会再走到读口。
    always @(posedge clk) begin
        if (rst_q || flush_any) begin
            fwd_slot1 <= 3'd0;
            fwd_slot2 <= 3'd0;
            fwd_slot3 <= 3'd0;
            fwd_slot4 <= 3'd0;
            for (si = 0; si < DEPTH; si = si + 1) begin
                scan_y1q[si] <= 1'b0;
                scan_y2q[si] <= 1'b0;
                scan_y3q[si] <= 1'b0;
                scan_y4q[si] <= 1'b0;
                scan_g1q[si] <= 1'b0;
                scan_g2q[si] <= 1'b0;
                scan_g3q[si] <= 1'b0;
                scan_g4q[si] <= 1'b0;
            end
        end
        else if (scan_go) begin
            fwd_slot1 <= slot_c1;
            fwd_slot2 <= slot_c2;
            fwd_slot3 <= slot_c3;
            fwd_slot4 <= slot_c4;
            for (si = 0; si < DEPTH; si = si + 1) begin
                scan_y1q[si] <= scan_y1[si];
                scan_y2q[si] <= scan_y2[si];
                scan_y3q[si] <= scan_y3[si];
                scan_y4q[si] <= scan_y4[si];
                scan_g1q[si] <= scan_g1[si];
                scan_g2q[si] <= scan_g2[si];
                scan_g3q[si] <= scan_g3[si];
                scan_g4q[si] <= scan_g4[si];
            end
        end
        else begin
            fwd_slot1 <= fwd_slot1;
            fwd_slot2 <= fwd_slot2;
            fwd_slot3 <= fwd_slot3;
            fwd_slot4 <= fwd_slot4;
            for (si = 0; si < DEPTH; si = si + 1) begin
                scan_y1q[si] <= scan_y1q[si];
                scan_y2q[si] <= scan_y2q[si];
                scan_y3q[si] <= scan_y3q[si];
                scan_y4q[si] <= scan_y4q[si];
                scan_g1q[si] <= scan_g1q[si];
                scan_g2q[si] <= scan_g2q[si];
                scan_g3q[si] <= scan_g3q[si];
                scan_g4q[si] <= scan_g4q[si];
            end
        end
    end

//前送值读口（组合，读拍用）：热码来自【打拍后的扫描】，数据与"算完了没"取【当拍】的 ——
//这样值总是最新的，而扫描那一段已经不在本拍的锥里了。
//★ 世代守卫：打拍的热码描述的是上一拍的住户；槽可能已经退项/复用 ⇒ 世代对不上就作废。
//★ `hit`（而非只有 `done`）也必须在这一拍由同一份守卫算出，且必须要求【在册】：
//  ① 槽被复用（世代变）⇒ 那一位已经不属于原来那条指令，命中位必须跟着落；
//  ② 生产者已退休（ent_v=0，世代还没变）⇒ 它已不在 ROB，命中位也要落 ——
//     否则旁路会跳过"刚提交（阵列还没写）"那一档兜底，落到阵列的老值上（实测 store 数据为 0）。
//  槽号 `fwd_slot` 仍打拍：世代守卫只在"这条指令的槽被复用"时落，槽号本身不参与选值。
    always @(*) begin
        for (si = 0; si < DEPTH; si = si + 1) begin
            scan_w1[si] = scan_y1q[si] & (ent_gen[si] == scan_g1q[si]) & ent_v[si];
            scan_w2[si] = scan_y2q[si] & (ent_gen[si] == scan_g2q[si]) & ent_v[si];
            scan_w3[si] = scan_y3q[si] & (ent_gen[si] == scan_g3q[si]) & ent_v[si];
            scan_w4[si] = scan_y4q[si] & (ent_gen[si] == scan_g4q[si]) & ent_v[si];
            scan_v1[si] = scan_w1[si] & ent_wr[si] & ~ent_ex[si];
            scan_v2[si] = scan_w2[si] & ent_wr[si] & ~ent_ex[si];
            scan_v3[si] = scan_w3[si] & ent_wr[si] & ~ent_ex[si];
            scan_v4[si] = scan_w4[si] & ent_wr[si] & ~ent_ex[si];
        end
//★ 命中位必须与"算完了没"用【同一份当拍判据】：世代对不上（槽已换人）时命中位也要落，
//  否则旁路会以为"有个还没算完的 ROB 源"而不回落到阵列读，取到的是新住户的脏值。
        fwd_hit1  = 1'b0;
        fwd_hit2  = 1'b0;
        fwd_hit3  = 1'b0;
        fwd_hit4  = 1'b0;
        fwd_done1 = 1'b0;
        fwd_done2 = 1'b0;
        fwd_done3 = 1'b0;
        fwd_done4 = 1'b0;
        fwd_data1 = 32'd0;
        fwd_data2 = 32'd0;
        fwd_data3 = 32'd0;
        fwd_data4 = 32'd0;
        st_done1  = 1'b0;
        st_done2  = 1'b0;
        st_done3  = 1'b0;
        st_done4  = 1'b0;
        st_data1  = 32'd0;
        st_data2  = 32'd0;
        st_data3  = 32'd0;
        st_data4  = 32'd0;
        for (si = 0; si < DEPTH; si = si + 1) begin
            fwd_hit1  = fwd_hit1  | scan_w1[si];
            fwd_hit2  = fwd_hit2  | scan_w2[si];
            fwd_hit3  = fwd_hit3  | scan_w3[si];
            fwd_hit4  = fwd_hit4  | scan_w4[si];
            fwd_done1 = fwd_done1 | scan_v1[si];
            fwd_done2 = fwd_done2 | scan_v2[si];
            fwd_done3 = fwd_done3 | scan_v3[si];
            fwd_done4 = fwd_done4 | scan_v4[si];
            fwd_data1 = fwd_data1 | (scan_v1[si] ? ent_data[si] : 32'd0);
            fwd_data2 = fwd_data2 | (scan_v2[si] ? ent_data[si] : 32'd0);
            fwd_data3 = fwd_data3 | (scan_v3[si] ? ent_data[si] : 32'd0);
            fwd_data4 = fwd_data4 | (scan_v4[si] ? ent_data[si] : 32'd0);
            st_done1  = st_done1  | ((st_slot1 == si[2:0]) & ent_wr[si] & ~ent_ex[si]);
            st_done2  = st_done2  | ((st_slot2 == si[2:0]) & ent_wr[si] & ~ent_ex[si]);
            st_data1  = st_data1  | (((st_slot1 == si[2:0]) & ent_wr[si] & ~ent_ex[si]) ? ent_data[si] : 32'd0);
            st_data2  = st_data2  | (((st_slot2 == si[2:0]) & ent_wr[si] & ~ent_ex[si]) ? ent_data[si] : 32'd0);
        end
    end

//提交口打一拍：把"要不要写 / 写哪个 / 写什么"在提交拍算好、寄存，下一拍再写寄存器堆。
//★ 为什么要打：regfile 是【变址写一个寄存器阵列】，综合器没法用 CE 表达"32 个里只使能一个"，
//  只能合成 `D_i = (we && rd==i) ? data : Q_i` ⇒ 使能 + 32 路 rd 译码 + 保持 mux 全落进 D 锥。
//  而 cmt_we 里的 head_ok 含 !flush_any / !exc_hit0（都来自 flag_bus、即中断/异常/分支判定锥）
//  ⇒ 整条中断锥压在阵列的 D 引脚上（实测默认流程 13 级、11ns，是当前最大失败族）。
//  打一拍后长锥止于本寄存器，阵列的 D 锥只剩"寄存器 → 译码 → 数据 mux"。
//★ 绝不能在 trap_fire / 冲刷拍清 cmt_*：head_ok 与 trap_fire 天然互斥（一个要 !ent_ex、
//  一个要 ent_ex）；冲刷只作废比边界年轻的项，而打拍里那笔在边界之前 ⇒ 清了会丢一笔合法写。
    always @(posedge clk) begin
        if (rst_q) begin
            cmt_we0   <= 1'b0;
            cmt_rd0   <= 5'd0;
            cmt_data0 <= 32'd0;
            cmt_we1   <= 1'b0;
            cmt_rd1   <= 5'd0;
            cmt_data1 <= 32'd0;
        end
        else begin
            cmt_we0   <= cmt_we0_e;
            cmt_rd0   <= cmt_rd0_e;
            cmt_data0 <= cmt_data0_e;
            cmt_we1   <= cmt_we1_e;
            cmt_rd1   <= cmt_rd1_e;
            cmt_data1 <= cmt_data1_e;
        end
    end

//时序：分配 / 回填 / 退 / 冲刷 / 陷阱标记
//  退掉的槽立刻失效，防止迟到回填污染；完成回填只认 valid 的槽；
//  冲刷把比 flush_idx 更年轻的项作废（tail 的回绕在组合块里算）。
    integer ri;
    always @(posedge clk) begin
        if (rst_q) begin
            cnt    <= 4'd0;
            head_p <= 3'd0;
            tail_p <= 3'd0;
            for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                ent_v[ri]  <= 1'b0;
                ent_wr[ri] <= 1'b0;
                ent_ex[ri] <= 1'b0;
                ent_cs[ri] <= 4'd0;
                ent_pc[ri] <= 32'd0;
                ent_tv[ri] <= 32'd0;
                ent_rd[ri] <= 5'd0;
                ent_data[ri] <= 32'd0;
                ent_gen[ri] <= 1'b0;
            end
        end
        else begin
            head_p <= head_nx;
            tail_p <= tail_nx;
            cnt    <= cnt_nx;
            if (head_ok) begin
                ent_v[head_p]  <= 1'b0;
                ent_wr[head_p] <= 1'b0;
                ent_ex[head_p] <= 1'b0;
            end
            if (head1_ok) begin
                ent_v[head1_p]  <= 1'b0;
                ent_wr[head1_p] <= 1'b0;
                ent_ex[head1_p] <= 1'b0;
            end
            if (alloc_en && !full) begin
                ent_v[tail_p]  <= 1'b1;
//★ 不写 rd 的项【出厂即 wr】：它本来就不用等任何写口回报 —— 只是占个位、按序退。
//  漏了这条：它永远卡在 head 上 ⇒ ROB 排不空 ⇒ 满 ⇒ 顶死前端（实测整核跑飞）。
                ent_wr[tail_p] <= ~alloc_we;
                ent_ex[tail_p] <= 1'b0;
//前送表的目的寄存器号：`alloc_we` 是 post_decoder 的 `issue_we`（保守侧：SYSTEM 一律算写），
//但它的判据里已经带了 `rd_in != 0` ⇒ 标准编码下 ecall/ebreak/mret 的 rd 都是 0、`issue_we=0`，
//所以"报了写但其实不写"的只剩非法编码，不会造成"选中一个永不产值的槽"。
                ent_rd[tail_p] <= (alloc_we && (alloc_rd != 5'd0)) ? alloc_rd : 5'd0;
                ent_gen[tail_p] <= ~ent_gen[tail_p];
            end
//lane1 那一项：与 lane0 同拍、同门控（full 已按整包算过，这里只要跟它一致就不会只进一半）。
            if (alloc_en && alloc_en1 && !full) begin
                ent_v[tail_p + 3'd1]  <= 1'b1;
                ent_wr[tail_p + 3'd1] <= ~alloc_we1;
                ent_ex[tail_p + 3'd1] <= 1'b0;
                ent_rd[tail_p + 3'd1] <= (alloc_we1 && (alloc_rd1 != 5'd0)) ? alloc_rd1 : 5'd0;
                ent_gen[tail_p + 3'd1] <= ~ent_gen[tail_p + 3'd1];
            end
//完成回填：wr 与 data 必须【同条件、同一沿】写入（拆开会出现"wr 已置、data 还是旧值"的一拍窗口）。
//★ 世代校验是必需的：被冲刷/已释放的槽在新住户身上的迟到上报只有它能挡（ent_v 挡不住"新住户也 valid"）。
            if (alu_done && ent_v[alu_idx] && (ent_gen[alu_idx] == alu_gen)) begin
                ent_wr[alu_idx]   <= 1'b1;
                ent_data[alu_idx] <= alu_data;
            end
            if (alu_done1 && ent_v[alu_idx1] && (ent_gen[alu_idx1] == alu_gen1)) begin
                ent_wr[alu_idx1]   <= 1'b1;
                ent_data[alu_idx1] <= alu_data1;
            end
            if (mul_done && ent_v[mul_idx] && (ent_gen[mul_idx] == mul_gen)) begin
                ent_wr[mul_idx]   <= 1'b1;
                ent_data[mul_idx] <= mul_data;
            end
            if (ld_done && ent_v[ld_idx] && (ent_gen[ld_idx] == ld_gen)) begin
                ent_wr[ld_idx]   <= 1'b1;
                ent_data[ld_idx] <= ld_data;
            end
//★ 标记陷阱时必须【同时置 wr】：故障指令的写被 pd 挡掉、也不会再有任何单元回报它
//  ⇒ 只置 exc 不置 wr，
//  它到队头也退不掉 ⇒ ROB 排不空 ⇒ redir 永远等不到 rob_empty ⇒ 整核挂死（实测 exc_illegal）。
            if (exc_en && ent_v[exc_idx]) begin
                ent_ex[exc_idx] <= 1'b1;
                ent_wr[exc_idx] <= 1'b1;
                ent_cs[exc_idx] <= exc_cause;
                ent_pc[exc_idx] <= exc_pc;
                ent_tv[exc_idx] <= exc_tval;
            end
            if (flush_all || (flush_con_rob && !flush_ok)) begin
//★ 边界项【已经退掉】也必须当成"全冲"：冲刷指令（分支）判定在 wb 拍，而它自己不写 rd、
//  退得比谁都快 —— 等它把 flush_idx 送过来时，它那一项早已退掉。此时 ROB 里活着的项
//  一定都比它年轻 ⇒ 全冲才对。从前这里要求"边界项还活着"，于是整条冲刷被忽略、
//  错路指令的写回漏进寄存器堆（实测 smoke 多写一笔 x17）。
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    ent_v[ri]  <= 1'b0;
                    ent_wr[ri] <= 1'b0;
                    ent_ex[ri] <= 1'b0;
                end
            end
            else if (flush_ok) begin
                for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                    ri_age = {1'b0, (ri[2:0] - head_p)};
                    if (ent_v[ri] && (ri_age > flush_age) && (ri_age < cnt)) begin
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
