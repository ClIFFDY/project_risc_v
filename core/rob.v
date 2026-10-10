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
//        flush_incl = 边界那条【也一起作废】（只有中断拉高：边界 = 站里最老的那条还没执行的指令，
//        它要在 mret 之后重跑一次，不能留）。
//   陷阱：故障指令的 (cause,pc,tval) 随它自己的项存下；它退到 head 时【不写寄存器堆】地退掉
//         （ent_ex 在 head_ok 里就挡掉了正常退），比它年轻的项全部作废，同时拉 trap_fire
//         并把载荷交给上级（由上级去锁 mepc/mcause/mtval 并重定向）—— 这才是精确异常。
//
// ★★ 与流水级的对应（本核的级名：取指 → idu1 → mem_buf → decoder(ID/EX) → wb → 写回）：
//   本模块**没有自己的流水级**，它是"全核的状态机"；下面每个口都挂在上表某一级的信号上，
//   时序错一拍就是"回填打到错项 / 标记落在已退的槽上"这类静默错，所以逐个写清：
//
//   | 口 | 挂在哪一级 | 与那条指令的相位（以该指令自身为参照） |
//   | --- | --- | --- |
//   | `alloc_en` = post_decoder 的 `payload_go` | **decoder(ID/EX) 入口拍** | 指令进 decoder 载荷的**当拍**分配；`alloc_idx` = 本拍组合的 tail，随它当载荷往下走 |
//   | `alu_done/alu_idx`（= wport 的 `fin_alu/fin_alu_idx`） | **alu 结果寄存器那一拍** | 该指令进 decoder 载荷的**下一拍**：写口级把它处置掉（落地 / 被杀 / 不写）就回报 |
//   | `mul_done/mul_idx`（= wport 的 `fin_mul/...`） | **mul 结果被处置那一拍** | 比 decoder 载荷晚 2~4 拍以上（乘法多周期 + 写口被挤住时还要等） |
//   | `ld_done/ld_idx`（= wport 的 `fin_ld/...`） | **lsu 结果被处置那一拍** | 比 decoder 载荷晚 2 拍以上（lsu 两级 + 总线等待，不定长） |
//   | `exc_en/exc_idx/cause/pc/tval` | **decoder(ID/EX) 载荷拍** = 该指令在 decoder 载荷里的那一拍 | 用 bju 的【组合】异常标记（寄存版晚一拍，见 trap_unit→cont 的注释） |
//   | `flush_con_rob/flush_idx/flush_incl` | **wb 拍** | bju 判定寄存一拍后发冲刷；`flush_idx` = 那条分支/跳转自己的项（`bju.idx_q`） |
//   | `trap_fire` + `trap_cause/pc/tval` | **退口那一拍** | 故障项排到队头那一拍，上级用它在同拍锁 mepc/mcause/mtval 并重定向 |
//   | `full/empty/occupancy` | 组合状态 | `full` 喂 cont 的 `hold_issue`（发射级压制） |
//
//   ★ 循环判据一律用 `age = {1'b0,(索引-head_ptr)}`（**先截 4 位再比**）：3 位减法掉进和 4 位 ent_cnt 的
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
//分配口（iffu 出队格 h0 换新那一拍 = 该条进 idu1 输出级那一拍；本模块无自有流水级
//⇒ 所有口的相位都相对"出队格换新那一拍"）
    input        alloc_en,
    input        alloc_we,
    input        alloc_jmp,
//这一条会不会报完成口（M 类 / LOAD）：它即使 rd==0 也不能在"发出即置可退"里退休 ——
//那一位的语义是"它不会再等任何回报"，而这两类不管写不写 rd 都会回报（实测：divu x0 提前退休，
//它的完成口 30 拍后从 mdu 出来，正好砸进槽的新住户、把值写脏）。
    input        alloc_pend,
    input [4:0]  alloc_rd,
//入队扫描口：这一拍要入册的那一条（iffu 出队格 h0 的 Q 端）的两个源寄存器号。
//扫描只在 alloc 那一拍跑一次：在册的项全是比它老的（它自己还没进册）⇒ "比我老"不需要比较；
//"取最年轻的匹配" = 窗口里年龄最大的那个（年龄一律 4 位截断，见下面扫描块）。
//结果（ps/pg/h，每个源三位）随分配一起写进新项 —— 这就是"生产者槽入队冻结"。
    input [4:0]  alloc_rs1, alloc_rs2,
//两级索引：s_idx = 本级输出（S 级，读拍）那条的槽号；pay_idx = 载荷那条的槽号。
//值一律按【冻结在项里的 ps】索引现读，不再每拍扫槽。
    input [2:0]  s_idx, pay_idx,
    output reg [2:0] fwd_slot1, fwd_slot2,
    output reg       fwd_hit1,  fwd_hit2,
//扫描槽那一项的【世代】（与 fwd_slot/fwd_hit 同拍）：给保留站的唤醒校验用 ——
//站项在入站时把生产者的世代冻下来，结果口广播时按 idx+gen 双匹配，杜绝槽复用后的误唤醒。
    output reg       fwd_pg1,   fwd_pg2,
//载荷那一级（pay_idx）的前送值：idu2 入站取值要用它与它自己的 done 配对。
//★ 少了这一路的后果（实测）：生产者已 done（值在它项里）但还没排到队头提交时，
//  rdy 说"就绪"（语义是"值可从项里前送"），而 idu2 入站取值只会看结果口与阵列读
//  ⇒ 落到阵列的老值上（那笔写还没进阵列）⇒ 消费者拿到旧值（实测 x2 差 0xb8）。
    output reg       pay_done1, pay_done2,
    output reg [31:0] pay_data1, pay_data2,
//载荷那条的两个操作数"能不能开跑"：~hit（本来就无生产者）或"生产者已可取值"
//（项已不在册 / 值已落项 / 值这一拍正在某个结果口上）⇒ 门住载荷与 S 级（原地等）。
//★ 判据必须按【载荷】这一级算，不能按 S 级：S 级等的生产者往往就是载荷那条自己，
//  而载荷那条要发得出去（lsu/mdu 的入口门）要求本拍 payload_go = 1 —— 按 S 级判就会
//  把载荷那条永久按在门上、它永远等不到完成 ⇒ 整核死锁（实测 sra-01 CoreMark 都挂）。
//  载荷那条的生产者一定比它老 ⇒ 一定已经发出去过 ⇒ 判据一定能收敛（归纳）。
//载荷有效位（post_decoder 随载荷一起锁存）：冲刷后载荷被清空、索引停在旧值，
//此时必须直接放行 —— 否则 rdy 拿着一个不相干的槽号把整组按死（实测挂死）。
    input        pay_v,
    output reg       rdy1, rdy2,
//前送值读口①【正常读】：索引就是本模块当拍扫描出的 fwd_slot1/2（不外引、不经任何选择）
//  ⇒ 这条路上没有任何 stall/flush 组合量，bju 的判定锥进不来。
    output reg [31:0] fwd_data1, fwd_data2,
    output reg        fwd_done1, fwd_done2,
//前送值读口②【停顿重读】：索引是消费者读锁存时锁下的槽（s1_q/s2_q，由 regfile 给）。
//  为什么要两组：消费者被停顿拖住时要用【锁存的】槽重读，而正常读用的是扫描槽 ——
//  若把两者在一根地址上 mux，stall_w（来自 flag_bus、即 bju 判定）就会被串进操作数数据路。
    input [2:0]  st_slot1, st_slot2,
    output reg [31:0] st_data1, st_data2,
    output reg        st_done1, st_done2,
//完成口（写口级处置完一笔就回报：落地 / 被杀 / 不写；只认 valid 的槽）
//  data 与 gen 与 done/idx 同拍同源（完成口把"值"和"这一笔的世代"一起带回来）
    input        bju_judged,
    input [2:0]  bju_idx,
    input        bju_gen,
    input        alu_done,
    input [2:0]  alu_idx,
    input [31:0] alu_data,
    input        alu_gen,
    input        mul_done,
    input [2:0]  mul_idx,
    input [31:0] mul_data,
    input        mul_gen,
    input        ld_done,
//发出口（idu2）：不写 rd 的项靠这一拍置"可退"，见完成段的说明
    input        issue_en,
    input [2:0]  issue_idx,
    input        issue_gen,
    input [2:0]  ld_idx,
    input [31:0] ld_data,
    input        ld_gen,
//冲刷口（wb 拍发：分支/跳转误预测；只作废比 flush_idx 更年轻的项。源名 flush_con_rob）
    input        flush_con_rob,
    input [2:0]  flush_idx,
    input        flush_incl,
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
//提交口：寄存器堆的两条写口（口 A = head 更老、口 B = head+1 更年轻）
    output reg        commit_we0,
    output reg [4:0]  commit_rd0,
    output reg [31:0] commit_data0,
    output reg        commit_we1,
    output reg [4:0]  commit_rd1,
    output reg [31:0] commit_data1,
//队头索引：写口级与各单元算年龄的基准（年龄 = {1'b0,(idx - head_ptr)}）
//年龄基准：全核 4 个模块（`forw`/`wport`/`lsu`/`mdu`）共 21 个负载点都在拿它做减法基准，
//是个纯广播网。
    output reg [2:0]  head_ptr,
//停顿源：ROB 满（stall_rob_full，喂 cont 的发射级压制）
    output reg        full,
//本次冲刷是【部分冲】还是退化成【全冲】（就是本模块 flush_ok 的组合版）。
//★ 给保留站用：站的清项窗口必须与本模块逐字同口径，与其在站里重推一遍
//  （回绕/边界已退/窗口外这三种都容易算错），不如直接用这里算好的结果。
    output reg        flush_part,
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
reg [3:0]  ent_cause [0:DEPTH-1];
reg [31:0] ent_pc [0:DEPTH-1];
reg [31:0] ent_tval [0:DEPTH-1];
//每槽的目的寄存器号（写 x0 与"不写 rd"一律存 0）—— 前送槽扫描用。
//只在复位与分配两处写，退项/冲刷都【不必】清：扫描判据里 `ent_v[s]` 与它相与，空槽天然被挡。
//（与 ent_cause/ent_pc/ent_tval 同款：那三个也只在复位/分配/异常标记时写。）
reg [4:0]  ent_rd [0:DEPTH-1];
//每槽的结果数据：完成口那一沿与 ent_wr 同条件写入，提交时由提交口读出。
//与 ent_cause/ent_pc/ent_tval/ent_rd 同款：只在复位/完成口写，退项/冲刷/陷阱【不清】（ent_v 门控挡空槽）。
reg [31:0] ent_data [0:DEPTH-1];
//每槽的世代位：每次分配翻转一次。完成上报必须与槽内存的世代一致才被接受 ——
//否则被冲刷/已释放的槽在新住户身上的【迟到上报】会把 ent_wr/ent_data 打错人（静默错值）。
reg        ent_gen  [0:DEPTH-1];
reg        ent_jmp  [0:DEPTH-1];
reg        ent_pend [0:DEPTH-1];
reg        ent_jdg  [0:DEPTH-1];

//扫描的工作量（纯组合，逐槽一份）
reg [3:0]  scan_age [0:DEPTH-1];
//"在册且写真实 rd"：只依赖触发器、与 rs 无关 ⇒ 与比较那一段平行算好。
//有它，候选判据才能压成一次 6 输入比较（见扫描块）。
reg        scan_wr_valid [0:DEPTH-1];
//候选掩码（在册且写真实 rd）与它对应的世代
reg        scan_use1 [0:DEPTH-1];
reg        scan_use2 [0:DEPTH-1];
reg        scan_gen1 [0:DEPTH-1];
reg        scan_gen2 [0:DEPTH-1];
//每槽两个源冻结下来的生产者槽号/世代/有无（入队扫描的产物）。
//只在复位与分配两处写；退项/冲刷/换人都不必清 —— 判据里 `ent_v[ps]` 与世代比较会把陈旧引用挡掉：
//生产者已退 ⇒ 值在阵列/提交口；槽已换人 ⇒ 世代不匹配 ⇒ 当作"没有生产者"。
reg [2:0]  ent_slot1 [0:DEPTH-1];
reg        ent_prod_gen1 [0:DEPTH-1];
reg        ent_hit1  [0:DEPTH-1];
reg [2:0]  ent_slot2 [0:DEPTH-1];
reg        ent_prod_gen2 [0:DEPTH-1];
reg        ent_hit2  [0:DEPTH-1];
//扫描的工作量（只在 alloc 那一拍用）
reg        scan_ready1 [0:DEPTH-1];
reg        scan_ready2 [0:DEPTH-1];
reg [2:0]  alloc_slot1, alloc_slot2;
reg        alloc_prod_gen1, alloc_prod_gen2, alloc_hit1, alloc_hit2;
//读口/就绪重算的中间量
reg [2:0]  slot1, slot2, slot1_pend, slot2_pend;
reg        prod_gen1, prod_gen2, prod_gen1_pend, prod_gen2_pend;
reg        gone1, gone2, done1, done2, port1, port2;
integer si, sj;

reg [2:0]  tail_ptr;
reg [3:0]  ent_cnt;

reg [2:0]  head_ptr_nx;
reg [2:0]  tail_ptr_nx;
reg [3:0]  ent_cnt_nx;

//提交口的组合版（打拍前）：head_ok / ent_rd[head_ptr] / ent_data[head_ptr]
reg        commit_we0_comb, commit_we1_comb;
reg [4:0]  commit_rd0_comb, commit_rd1_comb;
reg [31:0] commit_data0_comb, commit_data1_comb;
reg [2:0]  head1_ptr;
reg        flush_any;
reg        head_ok;
reg        head1_ok;
reg        exc_hit_head;
reg        exc_hit_head1;
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
//  写进 ent_ex 之前退出寄存器堆（模块 tb 踩到过）。年龄 age(i) = (i - head_ptr) mod 8，0 = 最老。
    always @(*) begin
        head1_ptr   = head_ptr + 3'd1;
//★ 冲刷那一拍不许退：退是组合的，而冲刷的清位要到边沿才生效 ⇒ 错路那条若正好在 head 上
//  且已 done，它会在冲刷拍就写进寄存器堆（实测 smoke 漏写一笔 x17）。冲刷拍一律不退。
        flush_any = flush_con_rob;
//★ 边界项已经【退掉】的异常冲刷：故障指令（典型是 store 非对齐 —— 它的项出厂即 wr，
//  排到队头就退，而故障脉冲在 lsu 里寄存了一拍）已经不在 ROB 里了，标记会落进空槽 ⇒
//  这次异常会静默丢掉。此时就【当场交付】：顺序退保证"它退了 ⇒ 比它老的都退了"，
//  所以直接拿 cont 那拍还活着的 exc_cause/pc/tval 给上级，精确性不变。
        exc_gone = flush_con_exc && !ent_v[flush_idx];
        exc_hit_head  = exc_en && (exc_idx == head_ptr);
        exc_hit_head1  = exc_en && (exc_idx == head1_ptr);
        head_ok   = (ent_cnt != 4'd0) && ent_v[head_ptr] && ent_wr[head_ptr] &&
                    !ent_ex[head_ptr] && !exc_hit_head && !flush_any &&
                    (!ent_jmp[head_ptr] || ent_jdg[head_ptr]);
        head1_ok  = head_ok && (ent_cnt >= 4'd2) && ent_v[head1_ptr] && ent_wr[head1_ptr] &&
                    !ent_ex[head1_ptr] && !exc_hit_head1 &&
                    (!ent_jmp[head1_ptr] || ent_jdg[head1_ptr]);
//★ 交付【不能】要求 ent_wr：访存非对齐的故障指令是被 lsu【拒绝】的（不车厢、不拉总线），
//  它永远等不到任何完成口 ⇒ ent_wr 永远 0 ⇒ 若在这里要 done，它既不退也不交付 ⇒ 整核死等
//  （实测 exc_ldst_misalign / half_misalign 卡死）。它排到队头就说明比它老的都退完了，
//  直接交付即可；它自己不会写寄存器堆（head_ok 里已用 !ent_ex 挡掉）。
        trap_fire = ((ent_cnt != 4'd0) && ent_v[head_ptr] &&
                    (ent_ex[head_ptr] || exc_hit_head)) || exc_gone;
//交付给上级的陷阱载荷：故障项自己那一份（不是"当前哪条又出错了"）—— 队头就是程序序里
//最老的那条故障指令 ⇒ 精确异常要求"先交付它"，比它年轻的项一律作废。
//★ 同拍命中（exc_hit_head：标记正好落在队头）必须走标记口那一路：此刻 ent_cause/ent_pc/ent_tval
//  里还是这一项【上一住户留下的旧值】（或者初值 0）—— 实测漏了这条：mepc/mcause 全交付成 0，
//  于是 mret 回到地址 4 反复重跑（exc_ecall 的事件流变成同一个 6 笔 store 的循环）。
        trap_cause = (exc_hit_head || exc_gone) ? exc_cause : ent_cause[head_ptr];
        trap_pc    = (exc_hit_head || exc_gone) ? exc_pc    : ent_pc[head_ptr];
        trap_tval  = (exc_hit_head || exc_gone) ? exc_tval  : ent_tval[head_ptr];
//★ 边界必须在【占用窗口内】才算"部分冲刷"：age(flush_idx) < ent_cnt。
//  只查 ent_v[flush_idx] 不够 —— 已退槽的 valid 位可能是脏的，一旦边界落在窗口外，
//  age 的模 8 回绕会把 ent_cnt 算成 8 之类的怪值，ROB 状态就烂了（实测 ent_cnt=8 但 head 项无效 ⇒ 队头永不动）。
//  窗口外一律按"全冲"处理（活着的项本来就都比它年轻）。
//★ age 必须【先截成 4 位】再比较：3 位减法一旦出现在和 4 位 ent_cnt 的比较里，Verilog 会把它
//  按 4 位上下文求值 ⇒ 回绕时算成 9 而不是 1，`age < ent_cnt` 判假 ⇒ flush_ok=0 ⇒ 误走"全冲"，
//  把比边界更老的、还在途的 load 写回一起扔掉（实测 CoreMark 少一笔写、隔离复现过）。
//  拼接的操作数是自决定的，{1'b0, a-b} 保证减法在 3 位里做完再扩位。
        flush_age  = {1'b0, (flush_idx - head_ptr)};
        flush_keep = flush_age + 4'd1;
        flush_ok   = flush_con_rob && (flush_age < ent_cnt) && ent_v[flush_idx];
        alloc_idx  = tail_ptr;
        alloc_gen  = ~ent_gen[tail_ptr];
        occupancy  = ent_cnt;
        empty      = (ent_cnt == 4'd0);
        full       = (ent_cnt == DEPTH);
        flush_part = flush_ok;
//提交口：只有队头两项可退，且【写 rd 的项】才写寄存器堆。
//★ `ent_rd != 0` 与 head_ok 里的 `!ent_ex` 是两道独立的门，缺一不可：
//  post_decoder 的 issue_we 把 SYSTEM 保守算作会写 ⇒ 故障项的 ent_rd 可能非 0。
        commit_we0_comb   = head_ok && (ent_rd[head_ptr] != 5'd0);
        commit_rd0_comb   = ent_rd[head_ptr];
        commit_data0_comb = ent_data[head_ptr];
        commit_we1_comb   = head1_ok && (ent_rd[head1_ptr] != 5'd0);
        commit_rd1_comb   = ent_rd[head1_ptr];
        commit_data1_comb = ent_data[head1_ptr];

    end

//下一拍的指针与占用数：先算退掉的（0/1/2 笔），再算分配的，冲刷最后覆盖（与旧写法优先级一致）
    always @(*) begin
        head_ptr_nx = head_ptr;
        if (head1_ok) begin
            head_ptr_nx = head_ptr + 3'd2;
        end
        else if (head_ok) begin
            head_ptr_nx = head_ptr + 3'd1;
        end
        ent_cnt_nx  = ent_cnt;
        tail_ptr_nx = tail_ptr;
        if (head_ok) begin
            ent_cnt_nx = ent_cnt_nx - 4'd1;
        end
        if (head1_ok) begin
            ent_cnt_nx = ent_cnt_nx - 4'd1;
        end
        if (alloc_en && !full) begin
            tail_ptr_nx = tail_ptr + 3'd1;
            ent_cnt_nx  = ent_cnt_nx + 4'd1;
        end
//★ 判据必须与时钟块那条【逐字一致】：边界项失效时走的是"全冲"，
//  时钟块清项、这里也必须把 ent_cnt/tail 一起归零 —— 漏了就是"项全清掉、计数还留着"，
//  ROB 立刻不自洽（ent_cnt≠0 而 head 项无效 ⇒ 队头永不动 ⇒ 挂死）。实测：部分冲刷启用后
//  这条路径第一次被走到，smoke 直接卡在 head 无效项上。
        if (flush_con_rob && !flush_ok) begin
            ent_cnt_nx  = 4'd0;
            tail_ptr_nx = head_ptr;
        end
        else if (flush_ok) begin
            ent_cnt_nx  = flush_incl ? flush_age : flush_keep;
            tail_ptr_nx = flush_incl ? flush_idx : (flush_idx + 3'd1);
        end
//陷阱交付：队头那条故障指令【不写寄存器堆】地退掉，比它年轻的项全部作废（ROB 清空）。
//★ 必须放在【冲刷赋值之后】（时钟块里 trap_fire 的清项也是最后一条）：交付那一拍
//  冲刷口可能同时有效（异常/中断也冲 ROB 了），若被冲刷的 ent_cnt=flush_keep 盖掉，
//  就出现"项全清、计数还留着" ⇒ head 指到无效项 ⇒ 挂死（实测 half_misalign）。
        if (trap_fire) begin
            head_ptr_nx = head_ptr + 3'd1;
            tail_ptr_nx = head_ptr + 3'd1;
            ent_cnt_nx  = 4'd0;
        end
    end

//前送槽扫描（纯组合）：给两个消费者操作数各找一条"比它老、rd 匹配、且最年轻"的在册项。
//★ 年龄一律用 4 位截断 `{1'b0,(idx - head_ptr)}`，禁止 3 位裸比大小（回绕会判错）。
//★ 槽号唯一 ⇒ "最年轻"不会有并列。
//★★ 输入是【本拍要入册那条】的 rs（iffu 出队格 h0 的 Q 端），扫出来的 ps/pg/h 在本拍
//   随分配一起写进新项（同一沿，不另外打拍）。做成"入项即冻结"是为了不让"8×8 比较 + 归约"
//   串进读拍（`rs → 扫描 → 取数 → 旁路 → r1_data`），那正是读锥最深的一段。
//★ 被扫的那条【还没进册】：`ent_v[tail_ptr]` 本拍读到的还是 0 ⇒ 它天然不会命中自己，
//   不需要额外排除"同拍正在分配项"。
    always @(*) begin
        for (si = 0; si < DEPTH; si = si + 1) begin
            scan_age[si] = {1'b0, (si[2:0] - head_ptr)};
//候选判据压成一次 6 输入比较：`scan_wr_valid` 只依赖触发器（与 rs 平行算好），剩下 5 个 XNOR + 这一次
//与 = 2 级。写成 `ent_v & (rs!=0) & (ent_rd==rs)` 是 7 个输入 ⇒ 3 级（多一级）。
//★ 别再想"给 rs=0 预置一个永不匹配的比较值"—— ent_rd 的残留值覆盖 0..31，不存在这样的值。
//  `scan_wr_valid` 自带 `|ent_rd`，而写 x0 的项 ent_rd=0 ⇒ rs=0 时天然不会命中。
            scan_wr_valid[si] = ent_v[si] & |ent_rd[si];
            scan_use1[si] = scan_wr_valid[si] & (ent_rd[si] == alloc_rs1);
            scan_use2[si] = scan_wr_valid[si] & (ent_rd[si] == alloc_rs2);
            scan_gen1[si] = ent_gen[si];
            scan_gen2[si] = ent_gen[si];
            scan_ready1[si] = scan_use1[si];
            scan_ready2[si] = scan_use2[si];
        end
//"有人比你更年轻"就把你摁掉（保证热码唯一）
        for (si = 0; si < DEPTH; si = si + 1) begin
            for (sj = 0; sj < DEPTH; sj = sj + 1) begin
                if (scan_use1[sj] && (scan_age[sj] > scan_age[si]))
                    scan_ready1[si] = 1'b0;
                if (scan_use2[sj] && (scan_age[sj] > scan_age[si]))
                    scan_ready2[si] = 1'b0;
            end
        end
        alloc_hit1  = 1'b0;
        alloc_hit2  = 1'b0;
        alloc_slot1 = 3'd0;
        alloc_slot2 = 3'd0;
        alloc_prod_gen1 = 1'b0;
        alloc_prod_gen2 = 1'b0;
        for (si = 0; si < DEPTH; si = si + 1) begin
            if (scan_ready1[si]) begin
                alloc_hit1  = 1'b1;
                alloc_slot1 = si[2:0];
                alloc_prod_gen1 = scan_gen1[si];
            end
            if (scan_ready2[si]) begin
                alloc_hit2  = 1'b1;
                alloc_slot2 = si[2:0];
                alloc_prod_gen2 = scan_gen2[si];
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
//读拍那一级：槽号就是冻结在项里的 ps；世代守卫保留（槽换人 ⇒ 命中位落、回落阵列读）
        slot1 = ent_slot1[s_idx];
        slot2 = ent_slot2[s_idx];
        prod_gen1 = ent_prod_gen1[s_idx];
        prod_gen2 = ent_prod_gen2[s_idx];
        fwd_slot1 = slot1;
        fwd_slot2 = slot2;
        fwd_pg1   = prod_gen1;
        fwd_pg2   = prod_gen2;
        fwd_hit1  = ent_hit1[s_idx] & ent_v[slot1] & (ent_gen[slot1] == prod_gen1);
        fwd_hit2  = ent_hit2[s_idx] & ent_v[slot2] & (ent_gen[slot2] == prod_gen2);
        fwd_done1 = fwd_hit1 & ent_wr[slot1] & ~ent_ex[slot1];
        fwd_done2 = fwd_hit2 & ent_wr[slot2] & ~ent_ex[slot2];
        fwd_data1 = fwd_done1 ? ent_data[slot1] : 32'd0;
        fwd_data2 = fwd_done2 ? ent_data[slot2] : 32'd0;
//载荷那一级的就绪：~hit（本来就无生产者）/ 生产者已不在册（值在阵列或提交口）/
//值已落项 / 值这一拍正在某个结果口上（这一项保住 ALU-RAW 的"当拍前送、不停拍"）
        slot1_pend = ent_slot1[pay_idx];
        slot2_pend = ent_slot2[pay_idx];
        prod_gen1_pend = ent_prod_gen1[pay_idx];
        prod_gen2_pend = ent_prod_gen2[pay_idx];
        gone1 = ~ent_v[slot1_pend] | (ent_gen[slot1_pend] != prod_gen1_pend);
        gone2 = ~ent_v[slot2_pend] | (ent_gen[slot2_pend] != prod_gen2_pend);
        done1 = ent_v[slot1_pend] & (ent_gen[slot1_pend] == prod_gen1_pend) & ent_wr[slot1_pend] & ~ent_ex[slot1_pend];
        done2 = ent_v[slot2_pend] & (ent_gen[slot2_pend] == prod_gen2_pend) & ent_wr[slot2_pend] & ~ent_ex[slot2_pend];
//★ 结果口这一路必须【在册 + 世代匹配】：只比槽号，槽复用后会把别人的结果当成生产者的值；
//  更要紧的是它会把 rdy 抬成"就绪"，消费者于是锁下阵列老值（与上面 pay_data 那条同源）。
        port1 = ent_hit1[pay_idx] & ((alu_done & (alu_idx == slot1_pend) & (alu_gen == prod_gen1_pend)) |
                                     (mul_done & (mul_idx == slot1_pend) & (mul_gen == prod_gen1_pend)) |
                                     (ld_done  & (ld_idx  == slot1_pend) & (ld_gen  == prod_gen1_pend)));
        port2 = ent_hit2[pay_idx] & ((alu_done & (alu_idx == slot2_pend) & (alu_gen == prod_gen2_pend)) |
                                     (mul_done & (mul_idx == slot2_pend) & (mul_gen == prod_gen2_pend)) |
                                     (ld_done  & (ld_idx  == slot2_pend) & (ld_gen  == prod_gen2_pend)));
//载荷那一级的两路出口：done 与它配对的值（与 rdy 同源、同拍）
        pay_done1 = done1;
        pay_done2 = done2;
        pay_data1 = done1 ? ent_data[slot1_pend] : 32'd0;
        pay_data2 = done2 ? ent_data[slot2_pend] : 32'd0;
        rdy1 = ~pay_v | ~ent_hit1[pay_idx] | gone1 | done1 | port1;
        rdy2 = ~pay_v | ~ent_hit2[pay_idx] | gone2 | done2 | port2;
    end

//前送值读口②【停顿重读】：索引是消费者读锁存时锁下的槽（s1_q/s2_q，由 regfile 给）——
//这一支与"按冻结 ps 现读"没有相位差（锁存下来的就是当时的 ps）。保留原样。
    always @(*) begin
        st_done1  = 1'b0;
        st_done2  = 1'b0;
        st_data1  = 32'd0;
        st_data2  = 32'd0;
        for (si = 0; si < DEPTH; si = si + 1) begin
            st_done1  = st_done1  | ((st_slot1 == si[2:0]) & ent_wr[si] & ~ent_ex[si]);
            st_done2  = st_done2  | ((st_slot2 == si[2:0]) & ent_wr[si] & ~ent_ex[si]);
            st_data1  = st_data1  | (((st_slot1 == si[2:0]) & ent_wr[si] & ~ent_ex[si]) ? ent_data[si] : 32'd0);
            st_data2  = st_data2  | (((st_slot2 == si[2:0]) & ent_wr[si] & ~ent_ex[si]) ? ent_data[si] : 32'd0);
        end
    end

//提交口打一拍：把"要不要写 / 写哪个 / 写什么"在提交拍算好、寄存，下一拍再写寄存器堆。
//★ 为什么要打：regfile 是【变址写一个寄存器阵列】，综合器没法用 CE 表达"32 个里只使能一个"，
//  只能合成 `D_i = (we && rd==i) ? data : Q_i` ⇒ 使能 + 32 路 rd 译码 + 保持 mux 全落进 D 锥。
//  而 cmt_we 里的 head_ok 含 !flush_any / !exc_hit_head（都来自 flag_bus、即中断/异常/分支判定锥）
//  ⇒ 整条中断锥压在阵列的 D 引脚上（实测默认流程 13 级、11ns，是当前最大失败族）。
//  打一拍后长锥止于本寄存器，阵列的 D 锥只剩"寄存器 → 译码 → 数据 mux"。
//★ 绝不能在 trap_fire / 冲刷拍清 cmt_*：head_ok 与 trap_fire 天然互斥（一个要 !ent_ex、
//  一个要 ent_ex）；冲刷只作废比边界年轻的项，而打拍里那笔在边界之前 ⇒ 清了会丢一笔合法写。
    always @(posedge clk) begin
        if (rst_q) begin
            commit_we0   <= 1'b0;
            commit_rd0   <= 5'd0;
            commit_data0 <= 32'd0;
            commit_we1   <= 1'b0;
            commit_rd1   <= 5'd0;
            commit_data1 <= 32'd0;
        end
        else begin
            commit_we0   <= commit_we0_comb;
            commit_rd0   <= commit_rd0_comb;
            commit_data0 <= commit_data0_comb;
            commit_we1   <= commit_we1_comb;
            commit_rd1   <= commit_rd1_comb;
            commit_data1 <= commit_data1_comb;
        end
    end

//时序：分配 / 回填 / 退 / 冲刷 / 陷阱标记
//  退掉的槽立刻失效，防止迟到回填污染；完成回填只认 valid 的槽；
//  冲刷把比 flush_idx 更年轻的项作废（tail 的回绕在组合块里算）。
    integer ri;
    always @(posedge clk) begin
        if (rst_q) begin
            ent_cnt    <= 4'd0;
            head_ptr <= 3'd0;
            tail_ptr <= 3'd0;
            for (ri = 0; ri < DEPTH; ri = ri + 1) begin
                ent_v[ri]  <= 1'b0;
                ent_wr[ri] <= 1'b0;
                ent_ex[ri] <= 1'b0;
                ent_cause[ri] <= 4'd0;
                ent_pc[ri] <= 32'd0;
                ent_tval[ri] <= 32'd0;
                ent_rd[ri] <= 5'd0;
                ent_data[ri] <= 32'd0;
                ent_gen[ri] <= 1'b0;
                ent_jmp[ri] <= 1'b0;
                ent_pend[ri] <= 1'b0;
                ent_jdg[ri]  <= 1'b0;
                ent_slot1[ri] <= 3'd0;
                ent_prod_gen1[ri] <= 1'b0;
                ent_hit1[ri]  <= 1'b0;
                ent_slot2[ri] <= 3'd0;
                ent_prod_gen2[ri] <= 1'b0;
                ent_hit2[ri]  <= 1'b0;
            end
        end
        else begin
            head_ptr <= head_ptr_nx;
            tail_ptr <= tail_ptr_nx;
            ent_cnt    <= ent_cnt_nx;
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
            if (alloc_en && !full) begin
                ent_v[tail_ptr]  <= 1'b1;
//★ 不写 rd 的项【出厂即 wr】：它本来就不用等任何写口回报 —— 只是占个位、按序退。
//  漏了这条：它永远卡在 head 上 ⇒ ROB 排不空 ⇒ 满 ⇒ 顶死前端（实测整核跑飞）。
//★ 【发出才算数】：不写 rd 的项不再"出厂即可退"，改由 issue_en 那一拍置位（见下面完成段）。
//  原式 `~alloc_we` 隐含"执行一定早于退役" —— 载荷式流水里成立（指令在排到队头前早就发出去了），
//  站式之后不成立：没发出去的分支会被队头直接退掉，那条跳转就永远不发生。
                ent_wr[tail_ptr] <= 1'b0;
                ent_ex[tail_ptr] <= 1'b0;
//前送表的目的寄存器号：`alloc_we` 是 post_decoder 的 `issue_we`（保守侧：SYSTEM 一律算写），
//但它的判据里已经带了 `rd_in != 0` ⇒ 标准编码下 ecall/ebreak/mret 的 rd 都是 0、`issue_we=0`，
//所以"报了写但其实不写"的只剩非法编码，不会造成"选中一个永不产值的槽"。
                ent_rd[tail_ptr] <= (alloc_we && (alloc_rd != 5'd0)) ? alloc_rd : 5'd0;
                ent_gen[tail_ptr] <= ~ent_gen[tail_ptr];
                ent_jmp[tail_ptr] <= alloc_jmp;
                ent_pend[tail_ptr] <= alloc_pend;
                ent_jdg[tail_ptr]  <= 1'b0;
//两个源的生产者冻结（入队扫描）：ps/pg/h 与内容同沿锁进去
                ent_slot1[tail_ptr] <= alloc_slot1;
                ent_prod_gen1[tail_ptr] <= alloc_prod_gen1;
                ent_hit1 [tail_ptr] <= alloc_hit1;
                ent_slot2[tail_ptr] <= alloc_slot2;
                ent_prod_gen2[tail_ptr] <= alloc_prod_gen2;
                ent_hit2 [tail_ptr] <= alloc_hit2;
            end
//完成回填：wr 与 data 必须【同条件、同一沿】写入（拆开会出现"wr 已置、data 还是旧值"的一拍窗口）。
//★ 世代校验是必需的：被冲刷/已释放的槽在新住户身上的迟到上报只有它能挡（ent_v 挡不住"新住户也 valid"）。
//★ 【发出即置 wr】（只对不写 rd 的项）：它们永远不会等到任何单元回报，wr 必须由"发出"给。
//  站式之后"执行早于退役"不再隐含 —— 没发出去的分支被队头退掉，跳转就永远不生效，
//  程序顺着落空那条路跑（实测 sll-01 的 `beq t3,t1,c`：跳转丢失 ⇒ 跑进 mtrampoline 搬迁循环
//  ⇒ 从地址 0 读数无人应答 ⇒ LSU 两节车厢卡死、整核死等）。写 rd 的项仍只由单元完成口置 wr。
//★ "发出即置可退"只给【根本不会有完成口】的项：写 rd 的等完成口；M 类与 LOAD 即使 rd==0
//  也照样回报 ⇒ 把它们排掉，否则提前退休会留下一个晚到的完成口，
//  30 拍后砸到槽的新住户头上（实测 divu x0）。
            if (issue_en && ent_v[issue_idx] && (ent_gen[issue_idx] == issue_gen) &&
                (ent_rd[issue_idx] == 5'd0) && !ent_pend[issue_idx]) begin
                ent_wr[issue_idx] <= 1'b1;
            end
            if (alu_done && ent_v[alu_idx] && (ent_gen[alu_idx] == alu_gen)) begin
                ent_data[alu_idx] <= alu_data;
                ent_wr[alu_idx] <= 1'b1;
            end
            if (bju_judged && ent_jmp[bju_idx] && ent_v[bju_idx] &&
                (ent_gen[bju_idx] == bju_gen)) begin
                ent_jdg[bju_idx] <= 1'b1;
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
                ent_cause[exc_idx] <= exc_cause;
                ent_pc[exc_idx] <= exc_pc;
                ent_tval[exc_idx] <= exc_tval;
            end
            if (alloc_en && !full) ent_wr[tail_ptr] <= 1'b0;
            if (flush_con_rob && !flush_ok) begin
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
                    ri_age = {1'b0, (ri[2:0] - head_ptr)};
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
