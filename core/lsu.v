`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: lsu
// Description: 两级（级1 发送 / 级2 等应答）+ 返回级 + ELSQ。
//   级1：进来当拍把地址/数据/be 摆上总线（miss 当拍由 blank_bus 顶住，落下再摆）。
//   级2：只等应答；store 进级2 即算提交。
//   ELSQ：store 与外设 load 先入队，到 commit 才上总线。
//
//////////////////////////////////////////////////////////////////////////////////


module lsu(
    input clk, rst,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input        redir_backend_0,   // 后端重定向（rob 队头那一拍出的那根）
    input        irq_kill,           // 中断受理（clint 自己判的）
//★ 2026-11-08：击杀掩码端口已摘。改向只在提交口发一次 ⇒ 每次冲刷都是整窗的，
//  原来按槽查表等价于常数 1；本模块只需"这一拍有没有冲刷"（flush_window）。
    input       exec_in,
//★ 总线被按住（外部仲裁器占用）：只在本模块生效 —— 它就是"本拍不许发新交易"。
//  2026-10-31 从 flag_bus[0]（停全前端）挪到这里：停前端是多余的，本模块自己等应答已经把
//  压力传下去了（stage2 等 ready_in → 队列涨 → lsu_room 归零 → 前端自然停）。
    input        bus_hold_in,
//入口门（每条指令只收一次）：出站载荷【这一拍就要推进】才允许收。
//★ 必须是"本拍推进"不是"上一拍推进过"：车厢满时前端被冻住、payload_go 当拍为 0，
//  用事后脉冲会让这条在新车厢里静默丢掉 ⇒ 它已进 E4、ROB 有项却永远等不到写使能 ⇒ 自锁。
//★ 错路访存已无不可撤回副作用（store/外设 load 都在 ELSQ 到 commit 才落地），故不再有错路门。
    input payload_go,
    input [6:0] opcode,
    input [9:0] function10,
//ROB 索引载荷：与其它载荷同使能锁存，回填时用它写进 ROB 自己那一项
    input [4:0]  idx_in,
//★ 2026-11-12：目的物理号随路带。原来写回时 ROB 要按 idx 现查一次 entry_phys，
//  那次分布式 RAM 读压在 lsu→isq 那条链正中间。它发射时就在站载荷里。
    input [5:0]  phys_in,
//那一项的世代位：与 idx 全程同行，完成上报时带回 ROB 做身份校验（见 rob.v 文件头）
    input        gen_in,
//本条指令的操作数（站选中项的终值，访存地址与 store 数据都从这里算）
    input [31:0] source_a_data_final, source_b_data_final,
//访存地址的两个加数：基址 + 立即数，加法由本模块做（在 stage1 寄存器之后）
    input [31:0] source_imm,
//本条指令【自己的 pc】：只用于非对齐故障上报（与 idx_in 同拍进、同拍锁，见 exception_load_store_pc_out）
    input [31:0] pc_in,
    input [31:0] bus_data_external, bus_data_from_dcache, bus_data_from_timer,
//dcache 的停顿：只在本模块内部生效，不进 flag_bus 的全核广播（由 dcache 直连本模块）。
//  miss 当拍不摆总线、等它落下的拍才发送；本模块的入口也照旧不看它（车厢占满由 full 顶住）。
    input dcache_hold,
//dcache 这一拍收不收新请求（纯状态量，见 dcache.v 端口说明）
    input dcache_blocked,
//dcache 的 B 口忙不忙（纯状态量）。排水判据用它，不用任何地址量。
    input dcache_store_blocked,
    input load_ready_next,
// P2/P3: only registered decisions: head_one_hot_registered = head one-hot, redirect_any = redirect outstanding.
// No head pointer (that spread the ROB decision cone into this module's write enables).
    input [31:0] head_one_hot_registered,
    input        redirect_any,
    input ready_dcache, ready_timer, ready_external,
//★ 总线出口【一分为二】：dcache 独占一套内部总线，不和 AHB master（外设/定时器）抢同一组 out。
//  dcache 口只走可缓存流量，外部口只走不可缓存流量。两支的地址/数据取同一个源寄存器，
//  仲裁只落在两个 valid 上 —— "该发给谁"不再穿在 32 位地址路径上。
//★ 2026-11-04：A 口只读（load），store 排水走下面的 store_* 独立口。
    output reg [31:0] dcache_address_out,
    output reg dcache_valid_out,
//---- B 口：ELSQ 排空（可缓存 store）→ dcache 的第二个口 ----
//  三根载荷全是 ELSQ 的寄存器；valid 由【寄存器量】决定（队头是 store 且已退休且 B 口不忙）。
    output [31:0] store_address_out,
    output [31:0] store_data_out,
    output [3:0]  store_byte_enable_out,
    output        store_valid_out,
    output reg [31:0] stage1_address_out,
    output reg [31:0] stage1_data_out,
    output reg [3:0] stage1_byte_enable_out,
    output reg stage1_we_out,
    output reg stage1_valid_out,
    (* max_fanout = 8 *) output reg [31:0] load_data_out,
//返回级 D 端：下一沿 return_data 会装载的那个值（前送表写表用，与 return_data 同拍对齐）
    (* max_fanout = 8 *) output reg [31:0] load_data_early,
//身份的 D 端（下一沿 return_valid/return_index/return_generation 会装载的值）：早唤醒用
    output reg        load_write_enable_early,
    output reg [4:0]  ld_index_e,
    output reg        ld_generation_e,
    (* max_fanout = 8 *) output reg loaded,
//本拍写回有效（与 loaded 同拍）
    output reg load_write_enable,
//★ 只报"两节车厢有没有空位"，与呈现的是哪条指令无关（isq: lsu_full = ~stage_free）。
//  故意不含 mem_op / is_store / full / hit —— 它们是当拍载荷的量，会与发射级仲裁成零延时环。
//  "队列满"只挡发送，不挡接收：收进来先躺在 s1，回压是寄存一拍的。
    output reg  stage_free,
    output reg        load_wake_registered,
    output reg [4:0]  load_wake_index_registered,
    output reg        load_wake_generation_registered,
    output reg        load_nack_registered,
    output reg [4:0]  load_nack_index_registered,
    output reg        load_nack_generation_registered,
//本条写回记录的 ROB 索引（与数据逐支同步；消费者自己拿去比源寄存器号）
    (* max_fanout = 8 *) output reg [4:0]  ld_index,
    (* max_fanout = 8 *) output reg [5:0] ld_phys,
    (* max_fanout = 8 *) output reg        ld_generation,
//ROB 的"本拍退休"回报（0/1 两条）：见下面 stage1_cmt 的注释
    input        ret0_valid, input [4:0] ret0_index, input ret0_generation,
    input        ret1_valid, input [4:0] ret1_index, input ret1_generation,
//store 完成口：进 SQ 那一拍回报（让 ROB 把 ent_wr 置起）。没有它，store 只能靠"发出即可退"
    output reg        store_done,
    output reg [4:0]  store_index_out,
    output reg        store_generation_out,
    output reg exception_load_store_misalign_out, exception_load_store_store_out,
    output reg [31:0] exception_load_store_address_out,
//故障指令【自己的】ROB 索引（与故障同拍寄存）：故障晚一拍到 cont，那时 issue_index 已是下一条
    (* max_fanout = 32 *) output reg [4:0] exception_load_store_index_out,
//故障指令【自己的】pc（与故障同拍寄存，理由同上）。
//★ 2026-10-31：原来 cont 拿前端的 auxiliary_address 延迟一拍再减 4 当 mepc，前提是
//  "报故障那一拍 aux 正好是故障指令 +4"。前端一停就完全不成立 —— mt11：故障在 0x30，
//  aux 冻在 0x18 ⇒ mepc 记成 0x14 ⇒ mret 回到 0x14 ⇒ 0x14/0x18/0x1c 死循环。
//  pc 和 idx 是同性质的伴生量，必须由同一个使能一起锁，不能去借别的级的地址。
    output reg [31:0] exception_load_store_pc_out,
//★ 2026-11-13（fence）：ELSQ 里还压着【已退休、还没排出去】的访存项。
//  只喂 ROB 提交口的排空判据，不进任何常规访存通路（见 docs/SPEC_fence_i_and_cache_range_2026-11-13.txt §2.3）。
//  ★ 为什么不是"队列全空"：年轻 store 在队里时它退不了休（退休在队头），
//    拿"全空"当判据会与队头的 fence 互相等待 ⇒ 死锁。已退休 = 比队头（fence）老，才是该等的。
    output wire       elsq_retired_pending
    );
//★★ 地址加法器位置宏（参数，不用 ifdef）：
//  0 = 现结构：D 端只锁两个加数，地址在 Q 端现算（大核默认）
//  1 = 搬到 D 端：D 端把地址算完再寄存，Q 端只剩 >>2（小核实测 +0.887ns，失败端点 4→0）
//  ★ ADDER_AT_D 是 elaboration 常量，未选中那支不进网表（不是运行期 mux）。
    parameter integer ADDER_AT_D = 0;

//复位就地打一拍（全核各模块都只打一拍，彼此零相位差）。
//★ 本模块最大的扇出网（fo 220，208 个是同步复位端）⇒ 打 max_fanout 让工具复制。
    (* max_fanout = 32 *) reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STORE = 7'b0100011;

//入队与总线选通都挂在同一个 flush_window 上，所以冲刷项必须是全集 —— 漏一项该来路的错路请求就照样上总线。
    reg flush_window;
    always @(*) begin
        flush_window = irq_kill | redir_backend_0;
    end

//总线读数据与停顿的合流（按"顶层不运算"从 cpu_top 下放至此）：
//三源按位或 —— 从设备必须每拍清零，否则残留值会被或进别人的读数据。
    reg [31:0] bus_data_in;
    reg ready_in;
    always @(*) begin
        bus_data_in = bus_data_external | bus_data_from_dcache | bus_data_from_timer;
        ready_in    = ready_dcache | ready_timer | ready_external;
    end

//==============================================================================
// 两条流：入口按 opcode 一分，之后各走各的
//   load  → stage1(stage2) = loadQ → dcache；外设 load 由 stage1 的 Q 端转进 ELSQ
//   store → store_stage     → ELSQ（第二个写口）→ 排队到 commit 才上总线
//★ store 不再借 stage1：原来它要在 stage1 里等 ~blank_bus 才能进 ELSQ，
//  而 blank_bus 在"有一笔读挂在总线上"时恒为 1 ⇒ 每一笔 store 都被前一笔 load 拖住。
//  分开之后 store 的入队只受 ELSQ 有没有位约束，与总线无关。
//==============================================================================
//★ 两条流都只锁【两个加数】，地址在各自的 Q 端由加法器现算 —— 入口那一拍没有任何 32 位加法。
//  代价：地址相关的判据（可缓存/撞在册 store/非对齐）全部落在 Q 端之后，各自带一条寄存器→逻辑的链。
//级1（发送）：load 专用。miss 当拍照进、等回填完再发；常态进级那拍就发。stage1_sent 记"发过没有"。
    reg        stage1_valid, stage1_sent;
    reg        stage1_faulted;      // 非对齐已经报过了（一拍脉冲，别重复报）
    reg [4:0]  stage1_index;
    reg [5:0]  stage1_phys;
    reg        stage1_generation;
    reg [2:0]  stage1_size;
    reg [31:0] stage1_base, stage1_imm, stage1_pc;
//★ ADDER_AT_D=1 时 D 端就把地址算完，stage1_imm 恒 0（Q 端加法被常量折叠掉）
    wire [31:0] stage1_base_next = ADDER_AT_D ? (source_a_data_final + source_imm) : source_a_data_final;
    wire [31:0] stage1_imm_next  = ADDER_AT_D ? 32'd0 : source_imm;
    reg        stage1_misalign;
//★ store 流的一级：入口那一拍锁进来，下一拍进 ELSQ。
//  它挡住自己的只有"ELSQ 没位 / 更老的外设 load 还没进队"，与总线无关。
    reg        store_stage_valid, store_stage_cmt, store_stage_faulted;
    reg [31:0] store_stage_base, store_stage_imm, store_stage_write_data, store_stage_pc;
    reg [4:0]  store_stage_index;
    reg [5:0]  store_stage_phys;
    reg        store_stage_generation;
    reg [2:0]  store_stage_size;
    reg        store_stage_misalign;
//★ 2026-10-31：排空那一笔的【寄存器出口】。原来它直接组合进 dcache_D，
//  ROB 的退休判据（store_queue_a_ret）就一路打到 dcache 的 fill_tag/BRAM 地址/WE 上
//  （实测 68/200 条最差路，源全是 u_rob/entry_valid_reg）。这里先锁一拍再进 dcache。
//★ 2026-11-04：这一段连同 drain_req_* 整个删掉 —— store 排水现在走 dcache 的 B 口，
//  载荷就是 store_queue_a_* 那几个寄存器，不再需要为 A 口单独锁一份。
    reg stage1_kill, stage2_kill, stage1_ret, stage2_ret, stage1_cmt, stage2_cmt;
//  级3（写回）：与 dcache 里锁住的那条是【同一条】—— 进级2 时已经发过请求，
//               在这一级只等数据回来（ready_in），所以不需要再发、也不用带载荷。
    reg        stage2_valid;      // 级3 只装 load（store 已经在 ELSQ 里了）
    reg [4:0]  stage2_index;
    reg [5:0]  stage2_phys;
    reg        stage2_generation;
    (* max_fanout = 8 *) reg [2:0] stage2_size;
    reg [1:0]  stage2_off;
//★ 2026-11-06：级3 这一格可能是"没上 dcache、数据从 SQ 的整字 store 拿的"。
    reg        stage2_forward;
    reg [31:0] stage2_forward_data;

//返回级：rdata 进寄存器，字节/符号扩展在它的 D 端做
    reg        return_valid;
    reg [4:0]  return_index;
    reg [5:0]  return_phys;
    reg        return_generation;
    reg [31:0] return_data;
    reg [31:0] load_data_current;   // 返回级 D 端
//★ 应答拍 = 返回级的装载条件（fo 48，单根 2.233 ns）⇒ 打散。
    reg        load_answer;
//返回口仲裁的两条中间量（见下面返回级那段的注释）
    wire       stage2_answer, elsq_return_deliver;

//组合判据
    reg mem_op, is_store, new_in, new_in_pre, new_go;
    reg take_load, take_store;
//==============================================================================
// ELSQ：store 与外设 load 先入队，**退休那一拍才排空上总线**。
//  把可见性从执行时刻挪到退休时刻，错路可直接丢弃。不前送：load 命中在队项就扣住（CoreMark 仅 1.98%）。
//==============================================================================
    reg        store_queue_a_valid, store_queue_b_valid;
    reg [31:0] store_queue_a_address, store_queue_a_write_data, store_queue_b_address, store_queue_b_write_data;
    reg [4:0]  store_queue_a_index, store_queue_b_index;
    reg [5:0]  store_queue_a_phys,  store_queue_b_phys;
    reg        store_queue_a_generation, store_queue_b_generation;
//kind：1 = store，0 = 外设 load（只有外设 load 才允许 kind=0 进队）。
    reg        store_queue_a_kind, store_queue_b_kind;
    reg [2:0]  store_queue_b_size;
    reg [1:0]  store_queue_b_offset;
//★ 队首项的宽度码/偏移要驱动整个移位/掩码网络（fo 34 / 46）⇒ 打散。
    (* max_fanout = 8 *) reg [2:0] store_queue_a_size;
    (* max_fanout = 8 *) reg [1:0] store_queue_a_offset;
    reg        store_queue_a_ready, store_queue_b_ready;          // 该项的 ROB 项【已经退休】（排空资格）
//ELSQ 的两个写口（按年龄：口1 更老）
    reg        load_push_queue, store_stage_push, store_push_hold, store_stage_leave, store_stage_drop, store_stage_ret, store_stage_fault, stage1_fault;
    reg [1:0]  elsq_after_pop, elsq_free;
    reg        elsq_pop;
    reg        elsq_head_load, elsq_load_go, elsq_load_sent, elsq_load_answer;
//★ 2026-10-31：「发过没有」必须【跟着条目走】。原来是一个全局位，A 位的外设 load 出队后
//  B 位那条挪上来，标志没清 ⇒ 新的队头被当成已发过 ⇒ 永远不发，而 blank_bus 又拿它把整核顶死。
    reg        store_queue_a_sent, store_queue_b_sent;
//★ 2026-11-12：入队要按【年龄】定位（见下面 SQ 次态 "口1/口2 按年龄追加" 那段的说明）。
//  年龄 = 与 ROB 队头的距离。两个候选项同减队头，比较就退化成比两个差值。
    reg  [4:0] elsq_head_number;
    wire [4:0] elsq_age_load_in  = stage1_index      - elsq_head_number;
    wire [4:0] elsq_age_store_in = store_stage_index - elsq_head_number;
    reg [31:0] elsq_load_data;
//★ ELSQ 外设 load 的返回口保持格：返回口一拍只送一笔，dcache 与 ELSQ 应答同拍时存这里。
    reg        elsq_return_hold;
    reg [31:0] elsq_return_data;
    reg [4:0]  elsq_return_index;
    reg [5:0]  elsq_return_phys;
    reg        elsq_return_generation;
//★ load_go = "本拍有一笔读摆上总线"。控制网，打 max_fanout 反而把 SQ 写使能推深，已撤。
    reg        store_queue_a_ret, store_queue_b_ret_now, store_queue_a_drop, store_queue_b_drop, store_queue_drain, load_go;
//★ 2026-11-04：一笔 store 从"排水起飞"到"dcache B 口办完"之间在册。队列项挂到办完才出队。
    reg        store_inflight, store_finish;

    reg        retire_hit_now, store_queue_push_ready, store_queue_hit_stage1;
//st→ld 前送：级1 的 load 同址命中 SQ 里一笔【整字】store ⇒ 直接拿它的数据，不等它排空
    reg        store_forward_a, store_forward_b, store_forward_hit;
    reg [31:0] store_forward_word, store_forward_data;
    reg        next_a_valid, next_b_valid, next_a_generation, next_b_generation, next_a_ready, next_b_ready;
    reg        next_a_sent, next_b_sent;
    reg        next_a_kind, next_b_kind;
    reg [2:0]  next_a_size, next_b_size;
    reg [1:0]  next_a_offset, next_b_offset;
    reg [31:0] next_a_address, next_a_write_data, next_b_address, next_b_write_data;
    reg [4:0]  next_a_index, next_b_index;
    reg [5:0]  next_a_phys,  next_b_phys;
    reg stage1_non_cacheable;
    reg stage2_done, stage2_ok, stage1_move, stage1_leave, stage1_ok, stage1_put_go;
    reg load_room, store_room;
    reg load_send_go, stage1_sent_go;
    reg load_out, blank_bus, stage1_put;
    reg miss;
    reg [31:0] stage1_sum, store_stage_sum, stage1_word, store_stage_word, store_write_data;
    reg size_bad_now;

//===============================================================
// 组合：入级/推进 + 入口
//===============================================================
//两类流的 Q 端加法器：进来的是两个寄存器，出去的就是这一条的地址。
//★ 入口那一拍【没有任何 32 位加法】，所以 entry → stage 寄存器这条路上只有前送 mux。
//★ 这一句是 dcache_D 与 ELSQ 写口的唯一地址来源；非对齐也在这里判（原来的入口全位加法已删）。
    always @(*) begin
        stage1_sum        = stage1_base + stage1_imm;
        store_stage_sum   = store_stage_base + store_stage_imm;
//★ dcache / ELSQ 认的是【字地址】域（dcache.v 里 bus_address_in[2:0] 就是 word），
//  只有异常上报要求字节地址。这两个域必须分开起名字，混用会把可缓存地址判成外设。
        stage1_word       = stage1_sum >> 2;
        store_stage_word  = store_stage_sum >> 2;
        store_write_data  = source_b_data_final;     // 原始值，宽度 mask 与移位都留到排空拍
    end

//非对齐：只看和的最低位（1 字节看 bit0，2 字节看 bit1:0）。
    always @(*) begin
        stage1_misalign = 1'b0;
        if (stage1_size[1:0] == 2'b01 && stage1_sum[0] != 1'b0) stage1_misalign = 1'b1;
        if (stage1_size[1:0] == 2'b10 && stage1_sum[1:0] != 2'd0) stage1_misalign = 1'b1;
        store_stage_misalign = 1'b0;
        if (store_stage_size[1:0] == 2'b01 && store_stage_sum[0] != 1'b0) store_stage_misalign = 1'b1;
        if (store_stage_size[1:0] == 2'b10 && store_stage_sum[1:0] != 2'd0) store_stage_misalign = 1'b1;
//★ one-shot：报过一次就不再报（这条要多等一拍冲刷窗口）
        stage1_fault = stage1_valid & stage1_misalign & ~stage1_faulted;
    end

//本条摆上总线要用的载荷
    always @(*) begin
        is_store    = (opcode == OPCODE_STORE);
    end

//入口唯一的合法性判据：宽度本身（ld/sd 在 RV32 非法，需根本不进流）。
//★ 非对齐不在这里判了 —— 地址要等 Q 端加法器，见下面 stage1/store_stage 的 misalign。
//★ 这张白名单必须与上游译码的白名单一致。
    always @(*) begin
        size_bad_now = 1'b0;
        if (opcode == OPCODE_LOAD) begin
            if (function10[2:0] == 3'b011)
                size_bad_now = 1'b1;
            if (function10[2:0] == 3'b110)
                size_bad_now = 1'b1;
            if (function10[2:0] == 3'b111)
                size_bad_now = 1'b1;
        end
        if (opcode == OPCODE_STORE) begin
            if (function10[2:0] == 3'b011)
                size_bad_now = 1'b1;
            if (function10[2:0] == 3'b100)
                size_bad_now = 1'b1;
            if (function10[2:0] == 3'b101)
                size_bad_now = 1'b1;
            if (function10[2:0] == 3'b110)
                size_bad_now = 1'b1;
            if (function10[2:0] == 3'b111)
                size_bad_now = 1'b1;
        end
    end

//让位与入口：load 等应答；store 进 ELSQ 就撒手。两条流各判各的空位。
//★★ 本块内赋值次序 = 依赖次序：同一 always 里先读后写不会重新触发。
    always @(*) begin
        mem_op   = (opcode == OPCODE_LOAD) | (opcode == OPCODE_STORE);
//★ 用 dcache 的【纯状态】阻塞位，不用组合的 dcache_hold：
//  dcache_D 组合化后 hold 里含"当拍请求造成的 miss" ⇒
//  load_go→请求→read_request→busy→hold→miss→blank_bus→load_go 成环（实测 delta 震荡）。
        miss     = dcache_blocked;
        stage2_done  = stage2_valid & (ready_in | stage2_forward);
        stage2_ok    = ~stage2_valid | stage2_done;
//空总线（blank）：miss 在跑，或在途那笔读还没被应答（含级2 刚摆上、下一拍才进级3 的那笔）⇒ 不摆。
//★ non_cacheable 必须在 load_out 之前赋值：否则 load_out 拿到的是上一条的译码 
//  （前一条 store、本条外设 load ⇒ 两笔应答撞拍被吞）。
//★ 可缓存判据 ≡ 内存译码（2026-11-13）：字地址 [31:16] 全 0 ⇔ 落在 STCM 的 256 KB 内。
//  原来这里是 [31:14]（= 前 64 KB），那是 64 KB dtcm 年代的遗留 —— 统一 L2 顶上来之后
//  没人跟着改，结果指令能在 256 KB 里任何位置被缓存、数据在 64 KB 以上却不能（取指走
//  icache→L2，不经过这条判据）。dcache.v 的两处收请求判据必须与这里【同一个口径】，
//  四处一起改；接 DDR 时在这里加一项即可，不要再散成四处。
        stage1_non_cacheable = (stage1_word[31:16] != 16'd0);
//★ st→ld 前送：级1 这条 load 的【字地址】命中 SQ 里一笔【整字】store ⇒ 数据现成
//  （store 写满了整字，这条 load 要的字节全在里面），不必等它排空、更不必再去 dcache 读。
//  只认 size==3'd2（sw）：sb/sh 没写满整字。试过按字节区间算覆盖（sb→lb、sh→lh 也前送），
//  CoreMark 上只多认出 16 拍、一拍没省，却要多摆 6 个比较器在地址加法后面 —— 不做。
//  两条 SQ 项都命中时取年轻的 B。SQ 里的项一定比级1 这条老（同一条顺序流水进来的）。
        store_forward_a = store_queue_a_valid & store_queue_a_kind & (store_queue_a_size == 3'd2)
                        & (store_queue_a_address == stage1_word);
        store_forward_b = store_queue_b_valid & store_queue_b_kind & (store_queue_b_size == 3'd2)
                        & (store_queue_b_address == stage1_word);
        store_forward_hit = stage1_valid & ~stage1_sent & ~stage1_misalign & (store_forward_a | store_forward_b);
        store_forward_word = store_forward_b ? store_queue_b_write_data : store_queue_a_write_data;
//按这条 load 的宽度/偏移从那个整字里切（与级3 的 load_data_current 逐位同形）
        case (stage1_size)
            3'b000: store_forward_data = {{24{store_forward_word[8*stage1_sum[1:0] + 7]}}, store_forward_word[8*stage1_sum[1:0] +: 8]};
            3'b001: store_forward_data = {{16{store_forward_word[16*stage1_sum[1] + 15]}}, store_forward_word[16*stage1_sum[1] +: 16]};
            3'b100: store_forward_data = {24'd0, store_forward_word[8*stage1_sum[1:0] +: 8]};
            3'b101: store_forward_data = {16'd0, store_forward_word[16*stage1_sum[1] +: 16]};
            default: store_forward_data = store_forward_word;
        endcase
        load_out     = (stage1_valid & stage1_sent & stage1_non_cacheable) | (stage2_valid & ~stage2_forward);
//★ bus_hold 并进 blank_bus：它挡住 load 的发送（stage1_put）。
//  2026-11-04 起 store 排水不再经过这里（它有自己的 B 口），所以这一句只管读。
        blank_bus  = bus_hold_in | miss | (load_out & ~ready_in) | (elsq_load_sent & ~ready_in) | dcache_store_blocked;
//★ 队列那条门放在【发送】不放【接收】：接收门多一项而 stage_free 没跟上 ⇒ isq 照常弹出、
//  本模块没接 ⇒ 指令静默蒸发。放在发送上只是"进得来、先别发"，下一拍自动重试。
//★ 非对齐的条一律不许进 dcache / ELSQ：Q 端判出来就地作废，只走异常上报。
        stage1_put     = stage1_valid & ~stage1_sent & ~blank_bus & ~stage1_misalign
                   & ~store_queue_hit_stage1 & ~stage1_non_cacheable;   // 外设 load 只在 ELSQ 里等
        stage1_put_go  = stage1_put & exec_in;   // flush 不控发请求：store 退休才摆、可缓存 load 读无副作用
//★ 读上 dcache 口必须等 stage2 这拍能接：否则应答回来时 stage2 里是别人 ⇒ 应答丢 ⇒ 死锁
        load_send_go   = stage1_put_go & stage2_ok;
        stage1_sent_go = load_send_go | load_push_queue;
//★★ 这几条必须写在 load_send_go / load_push_queue 之后！（踩过：应答到了，指令下一拍才进 stage2）
        stage1_move  = stage1_valid & (stage1_sent | load_send_go | store_forward_hit) & stage2_ok;
        stage1_leave = stage1_move | load_push_queue
                     | (stage1_valid & stage1_misalign);       // 故障条就地作废、让出格
        stage1_ok    = ~stage1_valid | stage1_leave;
//两条流各自的空位：只看寄存器 + 本拍让位，不看当拍载荷（store_room 在 SQ 块里算）
        load_room  = stage1_ok & stage2_ok;
//入口门只看本拍推进（payload_go）；自己那三条（haz/unload/full）不算，否则成组合环。
//★ 不要加 stall_rob_full / stall_pc_redir：重定向要等 ROB 排空，而队头访存正被它挡在门外 ⇒ 死锁。
//★ 冲刷窗口内一条都不许进：外设访问已在 commit，冲刷拍没有"必须放行"的合法条。
        new_in_pre = mem_op & ~size_bad_now & ~flush_window & payload_go;
        new_in     = new_in_pre;
        new_go     = load_room & store_room;
//★ 按 opcode 分流：一条指令只会走其中一条
        take_load  = new_in & new_go & ~is_store;
        take_store = new_in & new_go &  is_store;
    end

//排空那一笔该发给谁：dcache 认的地址范围 = (addr[31:24]==0 && addr[23:13]==11'h100)。
//  ★ 与 dcache.v 的 read_request/write_request、以及入口的 stage1_non_cacheable 用【同一个表达式】，
//    三处口径必须一致，否则同一条 store 会同时/都不被某个从设备接走。
//  ★ 只由 store_queue_a_address（寄存器）译码，纯组合、不进地址路径。
//  ★ mask/移位也只算一遍：两支寄存器装的是同一个值，不能一支读另一支的旧值。
    reg store_drain_non_cacheable;
    reg [31:0] store_drain_data;
    reg [3:0]  store_drain_byte_enable;
    always @(*) begin
        store_drain_non_cacheable = (store_queue_a_address[31:16] != 16'd0);
        case (store_queue_a_size)
            3'b000: begin
                store_drain_data        = {24'd0, store_queue_a_write_data[7:0]}  << (8 * store_queue_a_offset);
                store_drain_byte_enable = 4'b0001 << store_queue_a_offset;
            end
            3'b001: begin
                store_drain_data        = {16'd0, store_queue_a_write_data[15:0]} << (16 * store_queue_a_offset[1]);
                store_drain_byte_enable = 4'b0011 << (2 * store_queue_a_offset[1]);
            end
            default: begin
                store_drain_data        = store_queue_a_write_data;
                store_drain_byte_enable = 4'b1111;
            end
        endcase
    end

//dcache 口：★ 2026-10-31 起全【组合】—— 可缓存 load 的地址由 stage1 Q 的加法器直连，
//  排空那一笔取 ELSQ 的寄存器。两者天然互斥。
    always @(*) begin
        dcache_address_out     = 32'd0;
        dcache_valid_out       = 1'b0;
        if (load_go) begin
            dcache_address_out     = stage1_word;         // ← Q 端加法器输出（字地址），当拍进 dcache_D
            dcache_valid_out       = 1'b1;
        end
    end

//B 口（ELSQ 排空）的载荷：全是寄存器，直接连出去，不经过任何当拍的地址量。
    assign store_address_out     = store_queue_a_address;
    assign store_data_out        = store_drain_data;
    assign store_byte_enable_out = store_drain_byte_enable;
    assign store_valid_out       = store_queue_drain & ~store_drain_non_cacheable;

//★ 2026-11-13（fence 排空判据）：还压着"比队头老、已退休、没排出去"的项。
//  A 位是最老的，所以只要 A 位不够退休资格，队里剩下的就全是年轻项（与队头的 fence 无关）；
//  fence 在队头时它自己不在 ELSQ 里 ⇒ 读到的每一项只要是 _ret，就一定是比 fence 老的。
//  A 位的 _ret 包含"已发出去但还没收到落地确认"的那段（store_inflight 期间它保持 _ret）。
//  纯寄存器量相与，没有任何地址参与 ⇒ 不给访存通路加级。
    assign elsq_retired_pending = (store_queue_a_valid & store_queue_a_ret) |
                                  (store_queue_b_valid & store_queue_b_ret_now);

//外部口（AHB master + tim_in）：仍然寄存 —— 两个源都是 ELSQ 的寄存器
    always @(posedge clk) begin
        if (reset_registered) begin
            stage1_address_out <= 32'd0;
            stage1_data_out <= 32'd0;
            stage1_byte_enable_out <= 4'd0;
            stage1_we_out <= 1'b0;
            stage1_valid_out <= 1'b0;
        end
        else begin
//blank 的拍必须把总线清零：残留地址会被从设备当成新请求连答
            stage1_address_out <= 32'd0;
            stage1_data_out <= 32'd0;
            stage1_byte_enable_out <= 4'd0;
            stage1_we_out <= 1'b0;
            stage1_valid_out <= 1'b0;
            if (elsq_load_go) begin
                stage1_address_out  <= store_queue_a_address;
                stage1_data_out     <= 32'd0;
                stage1_byte_enable_out <= 4'b1111;
                stage1_we_out    <= 1'b0;
                stage1_valid_out <= 1'b1;
            end
            else if (store_queue_drain) begin
                stage1_address_out     <= store_queue_a_address;
                stage1_data_out        <= store_drain_data;
                stage1_byte_enable_out <= store_drain_byte_enable;
                stage1_we_out          <= 1'b1;
                stage1_valid_out       <= store_drain_non_cacheable;
            end
        end
    end


    always @(posedge clk) begin
        if (reset_registered) begin
            load_wake_registered <= 1'b0;
            load_wake_index_registered <= 4'd0;
            load_wake_generation_registered <= 1'b0;
            load_nack_registered <= 1'b0;
            load_nack_index_registered <= 4'd0;
            load_nack_generation_registered <= 1'b0;
        end
        else begin
//load_go 就是"本拍有一笔读摆上总线"，与原来的 load_send 是同一个函数（已合并，不再算两遍）
            load_wake_registered <= load_go;
            if (stage1_put_go) begin
                load_wake_index_registered <= stage1_index;
                load_wake_generation_registered <= stage1_generation;
            end
            load_nack_registered <= load_wake_registered & ~load_ready_next;
            load_nack_index_registered <= load_wake_index_registered;
            load_nack_generation_registered <= load_wake_generation_registered;
        end
    end
//==============================================================================
//  一笔 store 的在途位：起飞 → dcache B 口办完
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered)      store_inflight <= 1'b0;
        else if (store_queue_drain) store_inflight <= 1'b1;
        else if (store_finish)      store_inflight <= 1'b0;
    end

//===============================================================
// 时序：两级推进
//===============================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            stage1_valid <= 1'b0;
            stage2_valid <= 1'b0;
            stage2_forward <= 1'b0;
            stage2_forward_data <= 32'd0;
//★ 这些必须复位：sent → load_out → blank_bus → stage1_put → sent 是一个自维持的 X 环。
            stage1_sent <= 1'b0;
            stage1_faulted <= 1'b0;
//★ 载荷也要复位：地址为 X ⇒ non_cacheable 为 X ⇒ 同上把 sent 锁成 X。
            stage1_base <= 32'd0;
            stage1_imm  <= 32'd0;
            stage1_pc   <= 32'd0;
            stage1_index <= 4'd0; stage1_phys <= 6'd0;
            stage1_generation <= 1'b0;
            stage1_size <= 3'd0;
            stage1_cmt <= 1'b0;
            stage2_cmt <= 1'b0;
        end
        else begin
//★ s2 的"出"与"进"必须在同一条优先级链里判：旧住户被冲刷作废的那拍，
//  s1 里活着的那条正好前移进来，块尾独立的 kill 会把它一起抹掉 ⇒ 永不写回、卡队头 ⇒ 自锁。
            if (stage1_move) begin                                          // 级2 → 级3（请求已摆过）
                stage2_valid <= ~stage1_kill;                                       // 来的是错路就不许落进级3
                stage2_size <= stage1_size;
                stage2_off <= stage1_sum[1:0];
                stage2_index <= stage1_index;
                stage2_phys  <= stage1_phys;
                stage2_generation <= stage1_generation;
                stage2_cmt <= stage1_cmt | stage1_ret;
                stage2_forward <= store_forward_hit;
                stage2_forward_data <= store_forward_data;
                stage1_valid <= 1'b0;
            end
            else if (stage2_done | stage2_kill) begin                           // 级3 走掉 / 被冲刷作废
                stage2_valid <= 1'b0;
            end
            if (stage1_sent_go)
                stage1_sent <= 1'b1;  // 真的弄出去了（读上 dcache 口 / 进了 ELSQ）
            if (take_load) begin                                        // 本条是 load：进 loadQ
                stage1_valid <= 1'b1;
                stage1_size <= function10[2:0];
//★ 只锁两个加数，地址由 Q 端加法器现算
                stage1_base <= stage1_base_next;
                stage1_imm  <= stage1_imm_next;
                stage1_pc   <= pc_in;
                stage1_index <= idx_in;
                stage1_phys  <= phys_in;
                stage1_generation <= gen_in;
                stage1_sent <= 1'b0;                                        // 一律从'没发过'起步
                stage1_faulted <= 1'b0;
//★★ 2026-10-31 修（本次改造引入的 bug，代价是一次整核死锁）：
//  loadQ 改成 load 专用之后，`stage1_cmt`（"这条已经退休、不允许再按年龄冲刷"）【没有在新条进来时清零】，
//  它一旦被置 1 就永久粘住 ⇒ 之后每一条 load 都免死 ⇒ 冲刷漏掉它们 ⇒ ELSQ/loadQ 里留下
//  ROB 槽已作废的【过期项】⇒ 槽号被新住户复用时 head_one_hot 误命中 ⇒ 发一笔永不应答的外设读
//  ⇒ blank_bus 恒 1 ⇒ 整核冻死（CoreMark 231 字符处）。
//  语义上 load 【不可能】提前退休（写回才算完），所以这个位对 load 恒为 0。
                stage1_cmt <= 1'b0;
            end
        end
//★ 交给 ELSQ 的那一笔，loadQ 那一格当拍作废（ELSQ 的 valid 就是它活着）。
//★ 必须按「这一拍进 ELSQ 的是谁」分开写：无条件清会把同一沿补进来的新条抹掉 ⇒ 指令消失 ⇒ 冻住。
        if ((load_push_queue | stage1_misalign) & ~take_load) begin
            stage1_valid <= 1'b0;
            stage1_sent  <= 1'b0;
        end
        if (stage1_misalign & ~take_load) stage1_faulted <= 1'b1;

//★ 这里【不再】给 load 置 stage1_cmt：load 是"写回才算完"，不存在提前退休；
//  置了就等于给它免死金牌，冲刷漏杀 ⇒ 过期项滞留在 ELSQ。见上面 take_load 分支的说明。
        if (stage1_kill && ~take_load) stage1_valid <= 1'b0;
    end

//==============================================================================
// store 流的一级
//==============================================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            store_stage_valid <= 1'b0;
            store_stage_cmt <= 1'b0;
            store_stage_faulted <= 1'b0;
            store_stage_base <= 32'd0;
            store_stage_imm <= 32'd0;
            store_stage_write_data <= 32'd0;
            store_stage_pc <= 32'd0;
            store_stage_index <= 4'd0; store_stage_phys <= 6'd0;
            store_stage_generation <= 1'b0;
            store_stage_size <= 3'd0;
        end
        else begin
            if (take_store) begin
                store_stage_valid <= 1'b1;
                store_stage_base <= source_a_data_final;
                store_stage_imm  <= source_imm;
                store_stage_write_data <= source_b_data_final;
                store_stage_pc   <= pc_in;
                store_stage_index <= idx_in;
            store_stage_phys  <= phys_in;
                store_stage_generation <= gen_in;
                store_stage_size <= function10[2:0];
                store_stage_cmt <= retire_hit_now;   // ★ 可能进 LSU 那一拍就已经退休了
            end
            if (store_stage_leave & ~take_store) store_stage_valid <= 1'b0;
            if (store_stage_ret & ~take_store)   store_stage_cmt <= 1'b1;
        end
    end

//SQ 组合：队头 / 两个写口 / 排空资格
//★★ 本块内赋值次序 = 依赖次序（同一 always 里先读后写不会重新触发）。
    always @(*) begin
//---- 退休锁存（必须在移位之前：脉冲只有一拍，B 位上挨过的那拍不锁，挪到 A 位就丢了）----
        store_queue_a_ret = store_queue_a_valid & (store_queue_a_ready | (ret0_valid & (ret0_index == store_queue_a_index) & (ret0_generation == store_queue_a_generation))
                                  | (ret1_valid & (ret1_index == store_queue_a_index) & (ret1_generation == store_queue_a_generation)));
        store_queue_b_ret_now = store_queue_b_valid & (store_queue_b_ready | (ret0_valid & (ret0_index == store_queue_b_index) & (ret0_generation == store_queue_b_generation))
                                    | (ret1_valid & (ret1_index == store_queue_b_index) & (ret1_generation == store_queue_b_generation)));
//★ 截断：比冲刷边界年轻的项直接丢（这条以后就是 S3 唯一的作废手段）
//★ 2026-11-04 精确冲刷：按槽查击杀掩码 —— 比分支更老的、还在排队的 store/load 必须留住。
//  整窗那一档（异常 / 中断）掩码全亮，与改动前逐字等价。
//  ★ ELSQ 里已经摆上总线的 store 本来就不可撤回（~store_queue_a_ret 之外还有 inflight 挡着），这条不变。
        store_queue_a_drop = store_queue_a_valid & flush_window & ~store_queue_a_ret;
        store_queue_b_drop = store_queue_b_valid & flush_window & ~store_queue_b_ret_now;
//本拍是不是一笔【读】摆上总线
        load_go  = load_send_go;
//★ 外设 load 的资格取【在 ROB 队头】：应答回来才出队、才退休 ⇒ 不再需要错路遮挡。
        elsq_head_load   = store_queue_a_valid & ~store_queue_a_kind;
        elsq_load_go     = elsq_head_load & head_one_hot_registered[store_queue_a_index] & ~store_queue_a_sent
                         & ~elsq_return_hold & ~blank_bus & ~load_go & exec_in & ~flush_window;
//排空 = 队头够资格 且 dcache 的 B 口这一拍收得下。
//★ 2026-11-04：不再让位给读。store 有自己的口，「读优先」这条仲裁连同
//  `~blank_bus & ~load_go & ~elsq_load_go` 三项一起删掉 —— 那三项里裹着【当拍地址算出来的】
//  窗口译码与同址比较，是那条 19 级链（地址和 → blank_bus → 排水 → isq 的 CE）的入口。
//  现在这一句里除了 exec_in/flush（都是 flag_bus 的寄存器位）全是 ELSQ 与 dcache 的状态位。
//  ★ elsq_load_go 与它天然互斥（队头是 load 还是 store），不需要再显式排。
//  ★ 2026-11-12：加 ~bus_hold_in —— 外设总线上的那一笔还没被接受时不许换下一笔。
//    上游 AHB 主设备是寄存的（收到→上总线要 2 拍，总线被占时更久），而这里 drain
//    一拍换一个队头，换出来的脉冲会被主设备丢掉（实测丢过 UART 的 'S'）。
//    bus_hold_in 是主设备的 pend 寄存器，单根 FF，不给这条判据加组合级。
        store_queue_drain = store_queue_a_valid & store_queue_a_kind & store_queue_a_ret
                          & ~store_inflight & ~dcache_store_blocked & exec_in & ~flush_window
                          & ~bus_hold_in;
//★ 落地：B 口忙位撤销那一拍才算这笔 store 真的办完（缓存更新 + 写穿都可能被回填推迟）。
//  ★ 队列项必须挂到这一拍才出队 —— 提前出队会让同址的年轻 load 抢在缓存更新前面读到旧值。
        store_finish = store_inflight & ~dcache_store_blocked;
//★ 外设 load 只能认定时器/外部的应答，绝不能认 ready_in（三源相或）：
//  dcache 的应答会把 ELSQ 这条路拉起来 ⇒ 写回写进错的 ROB 项。
        elsq_load_answer = elsq_head_load & store_queue_a_sent & (ready_timer | ready_external);
//  blank_bus 用的是「本拍队头那条读已经摆出去了」
        elsq_load_sent   = elsq_head_load & store_queue_a_sent;
//---- 这一拍能塞几笔：pop 之后剩几格 ----
        elsq_pop       = store_queue_a_drop | store_finish | elsq_load_answer;
        elsq_after_pop = {1'b0, store_queue_a_valid} + {1'b0, store_queue_b_valid} - {1'b0, elsq_pop};
        elsq_free      = 2'd2 - elsq_after_pop;
//---- 写口1（更老）：stage1 Q 端的外设 load ----
        load_push_queue = stage1_valid & ~stage1_sent & ~blank_bus & ~stage1_misalign
                        & stage1_non_cacheable & exec_in & ~flush_window & (elsq_free != 2'd0);
//---- 写口2（更年轻）：store_stage ----
//★ 年龄次序是硬约束：ELSQ 里 store 要等自己退休才排空，而退休要求更老的指令全部完成。
//  年轻 store 一旦排在更老的外设 load 前面，那条 load 因为不是队头就永远发不出去 ⇒ 死锁。
//  所以：口2 必须给口1 让位，且更老的外设 load 还没进队时 store 也不许进。
        store_push_hold   = stage1_valid & ~stage1_sent & ~stage1_misalign & stage1_non_cacheable;
        store_stage_ret   = store_stage_valid & (store_stage_cmt
                          | (ret0_valid & (ret0_index == store_stage_index) & (ret0_generation == store_stage_generation))
                          | (ret1_valid & (ret1_index == store_stage_index) & (ret1_generation == store_stage_generation)));
        store_stage_drop  = store_stage_valid & ~store_stage_ret & flush_window;
        store_stage_fault = store_stage_valid & store_stage_misalign;   // 非对齐：不进 ELSQ，只上报
        store_stage_push  = store_stage_valid & ~store_stage_drop & ~store_stage_fault & ~store_push_hold
                          & exec_in & ~flush_window & (elsq_free > {1'b0, load_push_queue});
        store_stage_leave = store_stage_push | store_stage_drop | store_stage_fault;
        store_room        = ~store_stage_valid | store_stage_leave;
//入队当拍就记下"已经退休"：带过来的锁存位 + 当拍脉冲（RET 可能落在它进 LSU 的那一拍）
        store_queue_push_ready = store_stage_cmt
                    | (ret0_valid & (ret0_index == store_stage_index) & (ret0_generation == store_stage_generation))
                    | (ret1_valid & (ret1_index == store_stage_index) & (ret1_generation == store_stage_generation));
//入口那条 store 的退休脉冲（用来在它进 store_stage 那一拍就锁下 ready）
        retire_hit_now  = (ret0_valid & (ret0_index == idx_in) & (ret0_generation == gen_in)) |
                          (ret1_valid & (ret1_index == idx_in) & (ret1_generation == gen_in));
//★ 两边都是【字地址】，直接整字比较。曾在左侧又右移两位 ⇒ 两边差 4 倍 ⇒ store_queue_hit 恒 0
//  ⇒「load 命中在册同一 word 就压住」这条门是死的（arch-test fence-01 读回旧值）。
        store_queue_hit_stage1 = (store_queue_a_valid & (store_queue_a_address == stage1_word)) |
                    (store_queue_b_valid & (store_queue_b_address == stage1_word));
    end

//SQ 次态：先丢/排空，再入队（两笔同拍时排空腾出的位置给新来的）
//★ 2026-11-12：ROB 队头 one-hot → 槽号。只用寄存器的 Q 端，不进发射级入口门。
    always @(*) begin : blk_elsq_head_number
        integer hb;
        elsq_head_number = 5'd0;
        for (hb = 0; hb < 32; hb = hb + 1)
            if (head_one_hot_registered[hb]) elsq_head_number = hb[4:0];
    end

    always @(*) begin
//1) 先把本拍命中的退休脉冲锁进每一项的 rdy（必须在移位之前）
        next_a_valid = store_queue_a_valid; next_a_address = store_queue_a_address; next_a_write_data = store_queue_a_write_data;
        next_a_kind = store_queue_a_kind; next_a_size = store_queue_a_size; next_a_offset = store_queue_a_offset;
        next_a_index = store_queue_a_index; next_a_generation = store_queue_a_generation;
        next_a_phys  = store_queue_a_phys;
        next_a_ready = store_queue_a_ready | (store_queue_a_valid & ((ret0_valid & (ret0_index == store_queue_a_index) & (ret0_generation == store_queue_a_generation)) |
                                      (ret1_valid & (ret1_index == store_queue_a_index) & (ret1_generation == store_queue_a_generation))));
        next_b_valid = store_queue_b_valid; next_b_address = store_queue_b_address; next_b_write_data = store_queue_b_write_data;
        next_b_kind = store_queue_b_kind; next_b_size = store_queue_b_size; next_b_offset = store_queue_b_offset;
        next_b_index = store_queue_b_index; next_b_generation = store_queue_b_generation;
        next_b_phys  = store_queue_b_phys;
        next_a_sent = store_queue_a_sent | (store_queue_a_valid & ~store_queue_a_kind & elsq_load_go);
        next_b_sent = store_queue_b_sent;
        next_b_ready = store_queue_b_ready | (store_queue_b_valid & ((ret0_valid & (ret0_index == store_queue_b_index) & (ret0_generation == store_queue_b_generation)) |
                                      (ret1_valid & (ret1_index == store_queue_b_index) & (ret1_generation == store_queue_b_generation))));
        if (elsq_pop) begin
            next_a_valid = next_b_valid & ~store_queue_b_drop;
            next_a_address = next_b_address; next_a_write_data = next_b_write_data;
            next_a_index = next_b_index; next_a_generation = next_b_generation; next_a_ready = next_b_ready; next_a_sent = next_b_sent;
            next_a_phys = next_b_phys;
            next_a_kind = next_b_kind; next_a_size = next_b_size; next_a_offset = next_b_offset;
            next_b_valid = 1'b0;
        end
        else if (store_queue_b_drop) begin
            next_b_valid = 1'b0;
        end
//2) 两个写口按【年龄】追加：口1 的外设 load 比口2 的 store 老，必须先占更老的位。
//   elsq_free 已经把这一拍能塞的笔数算进去了（见上面），所以这里不会溢出。
//★★ 2026-11-12：访存站是【乱序】发的（更老那条的操作数还没就绪，年轻那条先发），
//   所以入队的先后不等于年龄先后。而 ELSQ 只有 A 位能发/能排空，且队头唯一
//   ⇒ A 位必须永远是最老的那条。原来的实现只按先后填 A→B：运行中的 store/load 阻挡了
//   更老那条的就绪时，年轻那条先入 A，更老那条后入 B，而 B 位永远等不到"成为队头"
//   ⇒ 两条互等（rand seed 3/8/17/20 实测：ROB 队头卡在 B 位那条上）。
//   修法：入队时按年龄定位 —— 新来的比 A 位更老就把 A 搬到 B、新来的占 A。
//   三条"不许换"的前提（任何一条不成立，A 位都必然是最老的）：
//     · A 位已经退休（ready=1）：比 ROB 队头还老，永远是全场最老；
//     · A 位已经发到总线上（sent=1）：发之前它必须是队头，比它更老的活条不存在；
//     · store 正占着 dcache 的 B 口（inflight=1，且这一拍没 pop）：同上，它已经退休。
//   B 位此刻必空：能入队 ⇒ elsq_free != 0；A 位有货 ⇒ 占用只可能是 1。
        if (load_push_queue) begin
            if (!next_a_valid) begin
                next_a_valid = 1'b1; next_a_address = stage1_word; next_a_write_data = 32'd0;
                next_a_index = stage1_index; next_a_generation = stage1_generation; next_a_phys = stage1_phys;
                next_a_kind = 1'b0; next_a_size = stage1_size; next_a_offset = stage1_sum[1:0];
                next_a_ready = 1'b0; next_a_sent = 1'b0;   // 外设 load 不预置 ready：它是「写回才算完」
            end
            else if (~next_a_sent & ~next_a_ready & (~store_inflight | elsq_pop)
                     & (elsq_age_load_in < (next_a_index - elsq_head_number))) begin
                next_b_valid = 1'b1; next_b_address = next_a_address; next_b_write_data = next_a_write_data;
                next_b_index = next_a_index; next_b_generation = next_a_generation; next_b_phys = next_a_phys;
                next_b_kind = next_a_kind; next_b_size = next_a_size; next_b_offset = next_a_offset;
                next_b_ready = next_a_ready; next_b_sent = next_a_sent;
                next_a_valid = 1'b1; next_a_address = stage1_word; next_a_write_data = 32'd0;
                next_a_index = stage1_index; next_a_generation = stage1_generation; next_a_phys = stage1_phys;
                next_a_kind = 1'b0; next_a_size = stage1_size; next_a_offset = stage1_sum[1:0];
                next_a_ready = 1'b0; next_a_sent = 1'b0;
            end
            else begin
                next_b_valid = 1'b1; next_b_address = stage1_word; next_b_write_data = 32'd0;
                next_b_index = stage1_index; next_b_generation = stage1_generation; next_b_phys = stage1_phys;
                next_b_kind = 1'b0; next_b_size = stage1_size; next_b_offset = stage1_sum[1:0];
                next_b_ready = 1'b0; next_b_sent = 1'b0;
            end
        end
        if (store_stage_push) begin
            if (!next_a_valid) begin
                next_a_valid = 1'b1; next_a_address = store_stage_word; next_a_write_data = store_stage_write_data;
                next_a_index = store_stage_index; next_a_generation = store_stage_generation; next_a_phys = store_stage_phys;
                next_a_kind = 1'b1; next_a_size = store_stage_size; next_a_offset = store_stage_sum[1:0];
                next_a_ready = store_queue_push_ready; next_a_sent = 1'b0;
            end
            else if (~next_a_sent & ~next_a_ready & (~store_inflight | elsq_pop)
                     & (elsq_age_store_in < (next_a_index - elsq_head_number))) begin
                next_b_valid = 1'b1; next_b_address = next_a_address; next_b_write_data = next_a_write_data;
                next_b_index = next_a_index; next_b_generation = next_a_generation; next_b_phys = next_a_phys;
                next_b_kind = next_a_kind; next_b_size = next_a_size; next_b_offset = next_a_offset;
                next_b_ready = next_a_ready; next_b_sent = next_a_sent;
                next_a_valid = 1'b1; next_a_address = store_stage_word; next_a_write_data = store_stage_write_data;
                next_a_index = store_stage_index; next_a_generation = store_stage_generation; next_a_phys = store_stage_phys;
                next_a_kind = 1'b1; next_a_size = store_stage_size; next_a_offset = store_stage_sum[1:0];
                next_a_ready = store_queue_push_ready; next_a_sent = 1'b0;
            end
            else begin
                next_b_valid = 1'b1; next_b_address = store_stage_word; next_b_write_data = store_stage_write_data;
                next_b_index = store_stage_index; next_b_generation = store_stage_generation; next_b_phys = store_stage_phys;
                next_b_kind = 1'b1; next_b_size = store_stage_size; next_b_offset = store_stage_sum[1:0];
                next_b_ready = store_queue_push_ready; next_b_sent = 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            store_done <= 1'b0; store_index_out <= 4'd0; store_generation_out <= 1'b0;
            store_queue_a_valid <= 1'b0; store_queue_b_valid <= 1'b0;
            store_queue_a_address <= 32'd0; store_queue_b_address <= 32'd0;
            store_queue_a_write_data <= 32'd0; store_queue_b_write_data <= 32'd0;
            store_queue_a_index <= 4'd0; store_queue_b_index <= 4'd0;
            store_queue_a_phys  <= 6'd0; store_queue_b_phys  <= 6'd0;
            store_queue_a_generation <= 1'b0; store_queue_b_generation <= 1'b0;
            store_queue_a_kind <= 1'b0; store_queue_b_kind <= 1'b0;
            store_queue_a_size <= 3'd0; store_queue_b_size <= 3'd0;
            store_queue_a_offset <= 2'd0; store_queue_b_offset <= 2'd0;
            store_queue_a_ready <= 1'b0; store_queue_b_ready <= 1'b0;
            store_queue_a_sent <= 1'b0;  store_queue_b_sent <= 1'b0;
        end
        else begin
            store_queue_a_valid <= next_a_valid; store_queue_a_address <= next_a_address; store_queue_a_write_data <= next_a_write_data;
            store_queue_a_index <= next_a_index; store_queue_a_generation <= next_a_generation;
            store_queue_a_phys  <= next_a_phys;
//★ 完成口必须是一拍脉冲 + 同拍锁存的载荷：idx/gen 无条件跟输入口走会认错槽 ⇒ 那条 store 永远退不了。
//★ 完成口现在挂在【入口收下 store 那一拍】：它后面一定会进 ELSQ（没有"被拒"这条路），
//  早报一拍就少占一拍 ROB。只有 store 才报；外设 load 由它自己的写回来给。
            store_done <= take_store;
            if (take_store) begin
                store_index_out <= idx_in;
                store_generation_out <= gen_in;
            end
            store_queue_a_kind <= next_a_kind; store_queue_a_size <= next_a_size; store_queue_a_offset <= next_a_offset;
            store_queue_b_kind <= next_b_kind; store_queue_b_size <= next_b_size; store_queue_b_offset <= next_b_offset;
            store_queue_a_ready <= next_a_ready;
            store_queue_a_sent <= next_a_sent; store_queue_b_sent <= next_b_sent;
            store_queue_b_valid <= next_b_valid; store_queue_b_address <= next_b_address; store_queue_b_write_data <= next_b_write_data;
            store_queue_b_index <= next_b_index; store_queue_b_generation <= next_b_generation;
            store_queue_b_phys  <= next_b_phys;
            store_queue_b_ready <= next_b_ready;
        end
    end



//===============================================================
// 写回 / 提交
//===============================================================
    always @(*) begin
        if (stage2_forward) load_data_current = stage2_forward_data;
        else case (stage2_size)
            3'b000: load_data_current = {{24{bus_data_in[8*stage2_off + 7]}}, bus_data_in[8*stage2_off +: 8]};
            3'b001: load_data_current = {{16{bus_data_in[16*stage2_off[1] + 15]}}, bus_data_in[16*stage2_off[1] +: 16]};
            3'b010: load_data_current = bus_data_in;
            3'b100: load_data_current = {24'd0, bus_data_in[8*stage2_off +: 8]};
            3'b101: load_data_current = {16'd0, bus_data_in[16*stage2_off[1] +: 16]};
            default: load_data_current = bus_data_in;
        endcase
    end

//外设 load 的取数：宽度/偏移随条目一起存在 ELSQ 里（与级3 那套逐位同形）
    always @(*) begin
        case (store_queue_a_size)
            3'b000: elsq_load_data = {{24{bus_data_in[8*store_queue_a_offset + 7]}}, bus_data_in[8*store_queue_a_offset +: 8]};
            3'b001: elsq_load_data = {{16{bus_data_in[16*store_queue_a_offset[1] + 15]}}, bus_data_in[16*store_queue_a_offset[1] +: 16]};
            3'b010: elsq_load_data = bus_data_in;
            3'b100: elsq_load_data = {24'd0, bus_data_in[8*store_queue_a_offset +: 8]};
            3'b101: elsq_load_data = {16'd0, bus_data_in[16*store_queue_a_offset[1] +: 16]};
            default: elsq_load_data = bus_data_in;
        endcase
    end

//★ 返回级：应答那一拍把数据（已做字节/符号扩展）连同身份锁下，下一拍广播 ——
//  这一级把 dcache 的 BRAM clock-to-out 从消费者锥上摘掉，对外口全从这个寄存器的 Q 端出。
//★ 2026-10-30 修：dcache 与 ELSQ 外设 load 可能【同拍】应答。原来 D 端按 elsq 优先 ⇒
//  dcache 那笔写回被静默吞掉 ⇒ ROB 项 entry_written 永为 0 ⇒ 死锁（看着像后面的 mul 卡死）。
//  修法：撞拍时把 ELSQ 这笔抓进一格保持寄存器。一格够用（ELSQ 同时只一笔在途，且 go 已挡在 hold 满时）。
    assign stage2_answer = stage2_valid & (ready_in | stage2_forward);
//ELSQ 那一笔的交付：必须等返回口空（这一拍既没有级3 应答、也没有新的 ELSQ 应答）
    assign elsq_return_deliver = elsq_return_hold & ~stage2_answer & ~elsq_load_answer;

    always @(*) begin
        load_answer = stage2_answer | elsq_load_answer | elsq_return_deliver;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            return_valid <= 1'b0; return_index <= 4'd0; return_generation <= 1'b0; return_data <= 32'd0;
            return_phys <= 6'd0;
            elsq_return_hold <= 1'b0; elsq_return_data <= 32'd0;
            elsq_return_index <= 4'd0; elsq_return_generation <= 1'b0; elsq_return_phys <= 6'd0;
        end
        else begin
            return_valid <= load_answer;
//撞拍：ELSQ 那一笔先存着（这一拍返回口归级3 那条）
            if (elsq_load_answer & stage2_answer) begin
                elsq_return_hold       <= 1'b1;
                elsq_return_data       <= elsq_load_data;
                elsq_return_index      <= store_queue_a_index;
                elsq_return_phys       <= store_queue_a_phys;
                elsq_return_generation <= store_queue_a_generation;
            end
            else if (elsq_return_deliver) begin
                elsq_return_hold       <= 1'b0;
            end

            if (elsq_return_deliver) begin
                return_index  <= elsq_return_index;
                return_phys   <= elsq_return_phys;
                return_generation  <= elsq_return_generation;
                return_data <= elsq_return_data;
            end
            else if (stage2_answer) begin
                return_index  <= stage2_index;
                return_phys   <= stage2_phys;
                return_generation  <= stage2_generation;
                return_data <= load_data_current;
            end
            else if (elsq_load_answer) begin
                return_index  <= store_queue_a_index;
                return_phys   <= store_queue_a_phys;
                return_generation  <= store_queue_a_generation;
                return_data <= elsq_load_data;
            end
        end
    end

//写回口：全部从返回级寄存器的 Q 端出，只亮一拍。
    always @(*) begin
        loaded      = return_valid;
        load_write_enable       = return_valid;

        load_data_out = return_data;
//★ 早唤醒口只属于 s2 那条路：外设 load 走返回级，拉进来索引/数据会对不上 ⇒ 往值表写垃圾。
        load_data_early   = load_answer ? load_data_current : return_data;
        load_write_enable_early     = (stage2_valid & (ready_in | stage2_forward));
        ld_index_e    = stage2_index;
        ld_generation_e    = stage2_generation;
        ld_index      = return_valid ? return_index : 4'd0;
        ld_phys       = return_valid ? return_phys  : 6'd0;
        ld_generation      = return_valid ? return_generation : 1'b0;
    end
//★ stage_free 是喂给发射级的入口门（isq: lsu_full = ~stage_free），必须与 new_in_pre 逐项同门。
//  漏一项就是：请求照发 → 级1 不收 → 载荷静默蒸发 ⇒ 整核自锁。
//★ 取【两条流都有位】而不是"按 opcode 挑一条"：stage_free 一旦依赖 opcode，
//  就变成"当拍载荷决定广播门"，这正是本仓反复踩的零延时环。代价只是 store_stage 被占时
//  load 也停一拍 —— 而 store_stage 正常一拍就排掉，且它被占时本来也是 ELSQ 满/等老 load。
    always @(*) begin
        stage_free         = load_room & store_room;
    end

//★ 已经退休的项不许再按年龄作废。store 是"发出即置可退"，可在级1 里等好几拍，
//  这期间队头越过它 ⇒ 环位回绕成大数 ⇒ 之后的冲刷会把这条其实更老的 store 当错路杀掉。
    always @(*) begin
//★ 作废只看年龄：冲刷边界比我更年轻 ⇒ 我是错路 ⇒ 必须扔（不会丢架构写）。
//  曾用"进来时有一条更老的 bju 在飞"当判据，那是过强假设（更老的分支可能同一拍才发出）；
//  现在外设访问整体搬到 commit，那条自锁路径已从结构上消失。
        stage1_ret = (ret0_valid & (ret0_index == stage1_index) & (ret0_generation == stage1_generation)) |
                 (ret1_valid & (ret1_index == stage1_index) & (ret1_generation == stage1_generation));
        stage2_ret = (ret0_valid & (ret0_index == stage2_index) & (ret0_generation == stage2_generation)) |
                 (ret1_valid & (ret1_index == stage2_index) & (ret1_generation == stage2_generation));
//★ 作废查 rob 的击杀弧掩码（第 i 位 = 槽 i 比边界年轻），一次 16:1 查表。
//  ★ 必须吃当拍版：吃寄存版等于把丢弃晚一拍，错路请求可能先上了总线。
        stage1_kill = stage1_valid & ~stage1_cmt & ~stage1_ret & flush_window;
        stage2_kill = stage2_valid & ~stage2_cmt & ~stage2_ret & flush_window;
    end

//访存非对齐故障：判定挪到 Q 端（地址要等加法器），伴生量（pc / 索引 / 读写标志 / 出错地址）
//  必须描述同一条指令 ⇒ 只能共用一个使能，而且 pc 必须随条目一起排到这一级
//  （mt11 的教训：去借别的级的地址，那一刻早换成别的指令了）。
//★ 不能改成组合输出：故障 → clint → redir_backend_0 → flush_window → new_in_pre → 故障，成环。
//★ one-shot：故障条会在 stage1 / store_stage 里多待一拍（等冲刷），靠 *_faulted 只报一次。
    always @(posedge clk) begin
        if (reset_registered) begin
            exception_load_store_misalign_out <= 1'b0;
            exception_load_store_store_out <= 1'b0;
            exception_load_store_address_out <= 32'd0;
            exception_load_store_index_out <= 4'd0;
            exception_load_store_pc_out <= 32'd0;
        end
        else begin
            exception_load_store_misalign_out <= stage1_fault | store_stage_fault;
            if (stage1_fault) begin
                exception_load_store_store_out   <= 1'b0;
                exception_load_store_address_out <= stage1_sum;
                exception_load_store_index_out  <= stage1_index;
                exception_load_store_pc_out     <= stage1_pc;
            end
            else if (store_stage_fault) begin
                exception_load_store_store_out   <= 1'b1;
                exception_load_store_address_out <= store_stage_sum;
                exception_load_store_index_out  <= store_stage_index;
                exception_load_store_pc_out     <= store_stage_pc;
            end
        end
    end



endmodule
