`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/12 20:19:40
// Design Name:
// Module Name: dcache
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module dcache(
//★★ 2026-11-04：改成【两个请求口】。
//  A 口 = load（只读）：请求拍捉载荷 + tag 字，下一拍用寄存器判命中。
//  B 口 = ELSQ 排空（只写）：自己一份 tag 阵列副本做命中判定，不占 A 口的 tag 读口。
//  动机：原来 store 走 A 口要占两拍（请求拍 + store_pending 拍），LSU 那边为了抢这一个口
//  把「总线本拍归谁」做成组合仲裁，而仲裁的一个输入是【当拍算出来的地址】——
//  实测一条 19 级的链：地址和 → 窗口译码 → blank_bus → 排水资格 → 回到 isq 的寄存器 CE。
//  分开两个口之后 store 排水只看寄存器（ELSQ 项 + B 口忙位），那条链从源头消失。
    input clk, rst,
//---- A 口：load（只读）----
    input [31:0] bus_address_in,
    input bus_valid_in,
//---- B 口：ELSQ 排空（只写）----
    input        store_valid_in,
    input [31:0] store_address_in,   // 字地址（与 A 口同一个域）
    input [31:0] store_data_in,
    input [3:0]  store_byte_enable_in,
//B 口忙：从收下到"缓存更新 + 写穿"都发完。LSU 用它挡排水（纯寄存器，不含任何地址量）。
    output wire  store_blocked,
//后端存储接口
    input [31:0] mem_data,
    input mem_valid, mem_ready,
//★ 2026-11-12：写穿必须等后端"接住了"才算了结。后端（L2）的写队列只有 4 深，
//  满了它会把这一笔静默丢掉；以前 mem_write_enable 是一拍脉冲、发出去就当成功，
//  于是丢写没人知道（实测 FreeRTOS 的 TCB 字段就是这么没的）。现在没被接住就一直举着。
    input mem_write_ready,
//★ 2026-11-13（fence / DMA 一致性）：失效脉冲。抬一拍 ⇒ 世代位取反 ⇒ 整张 tag 表当拍全部作废，
//  不扫阵列、不写阵列。与复位那条是同一套机制（valid_generation），所以只需要一根多路或。
    input        invalidate,

//总线接口
    output [31:0] bus_data_out,
    output reg busy, hold, load_ready_next,
//★ 2026-11-04：load_ready 改成【纯组合】，由寄存下来的命中与 fill_end_q 直接给出（见下面 assign）。
    output load_ready,
//★★ 2026-10-31：给 LSU 的"这一拍能不能收新请求"——只由【寄存器】决定（stage），
//  绝不含当拍请求的贡献。这样上层可以用它做发送门而不会成组合环：
//    load_go → 请求 → read_request → busy → hold → miss → blank_bus → load_go   （原来这一圈）
//  本信号 = "再发我就丢了"，语义比 hold 窄，足够且无环。
    output wire request_blocked,
//后端存储接口
    output reg mem_req, mem_write_enable,
    output reg [31:0] mem_addr, mem_write_data,
    output reg [3:0] mem_byte_enable
    );
//★★ 宏化旋钮：dcache 组数位宽。8 = 256 组/16KB（默认），7 = 128 组/8KB。
//  行恒 32 字节（word 3 位不变）。★ 缩容量的目的是让 tag 阵列（LUTRAM）与数据阵列（BRAM）
//  在版图上靠得更近，直接缓解最差路径那条 8.35ns 的跨列布线。
    parameter integer DC_SETS_B = 8;
    localparam integer DC_SETS   = 1 << DC_SETS_B;
    localparam integer DC_WORDS  = DC_SETS << 3;
    localparam integer DC_TAG_B  = 32 - 3 - DC_SETS_B;
    localparam integer DC_TAGW   = DC_TAG_B + 1;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//寄存器（按级分组）
//第 0 级：本拍组合
    reg [DC_TAG_B-1:0] tag;
    reg [DC_SETS_B-1:0] index;
    reg [2:0] word;
    reg [1:0] hit_way;
    reg cache_hit;

    reg read_request, read_drive;
    reg hit_upd;
    reg lru_write_enable, lru_write_value;
    reg [7:0] lru_write_index;
//★ 2026-10-31：命中更新打一拍再写 LRU。
//  原来 cache_hit 是「读 tag 阵列 → 比较 → 当拍就驱动 LRU 写使能」，实测这一段 4.92 ns；
//  它前面「rob/head_pointer → dcache 地址 mux → tag 地址脚」已经吃掉 8.25 ns
//  ⇒ 合起来 13.17 ns > 12.5，是全核 WNS 的唯一来源（最差 20 条端点全是同一条）。
//  LRU 只是替换提示量，晚一拍写不改变任何架构行为；回填那一路仍走当拍。
    reg        lru_hit_upd_q, lru_hit_value_q;
    reg [7:0]  lru_hit_index_q;
    reg [DC_SETS_B+2:0] read_address;

    reg read_enable;

    reg is_fill_write;
//B 口：命中（寄存器比出）、缓存更新 one-shot、写穿 one-shot
    reg store_hit0, store_hit1, store_cache_write, store_mem_write;
    reg way0_write_enable, way1_write_enable;
    reg [3:0] ram_byte_enable;
    reg [DC_SETS_B+2:0] ram_write_address;

    reg [31:0] ram_write_data;

//★★ 2026-11-04：命中判定【整体推后一拍】，A 口的读与 LRU 共用这一份寄存量。
//  原路径「stage1_base → LSU 加法/译码 → tag 阵列 → 22 位比较 → data_way*/ENARDEN」实测 13.62 ns
//  （logic 5.29 / route 8.33，20 级），200 条最差端点里的 8 条真深路径全是这一条。
//  现在：请求那一拍只锁载荷 —— 使能只看 read_request（不含命中判定），
//  顺带把那一拍 LUTRAM 已经读出的两个 tag 字捉下来；下一拍直接用【寄存器】比出命中。
    reg        access_valid_q;                       // 上一拍 A 口受理了一笔读
    reg [DC_SETS_B+2:0] access_address_q;
    reg [DC_TAG_B-1:0]  access_tag_q;
    reg [DC_TAGW-1:0]   access_tag_word0_q, access_tag_word1_q;

//---- B 口（ELSQ 排空，只写）的状态 ----
//  时序与 A 口的 store 原来那套同相：收下那拍捉载荷 + 本口 tag 副本的字，下一拍用寄存器判命中。
//  之后要办两件事，各自 one-shot：①命中就把该路该字就地更新（与回填写口互斥，回填优先）
//  ②写穿到后端（mem 口被回填占着就等）。两件都办完才撤 busy。
    reg        store_busy;
    reg        store_cache_done, store_mem_done;
    reg [31:0] store_address_q;
    reg [31:0] store_data_q;
    reg [3:0]  store_byte_enable_q;
    reg [DC_TAG_B-1:0]  store_tag_q;
    reg [DC_SETS_B-1:0] store_index_q;
    reg [2:0]  store_word_q;
    reg [DC_TAGW-1:0]   store_tag_word0_q, store_tag_word1_q;

//第 1 级与自举/回填：阵列与状态机
//★ 2026-10-30：valid 折进 tag 字，和 icache 同构（icache.v:162-163、581 就是 `{1'b1, fill_tag}`）。
//  原来 way0_valid/way1_valid 是 `reg [255:0]` 宽向量 + 运行时下标读 ⇒ 综合成 256:1 的 LUT mux 树
//  （MUXF7+MUXF8 各两级），单独的 dcache 实现里它就是最差路径的前两级。
//  折进 tag 后，valid 和 tag 一起从同一块 LUTRAM 读出来、一起比较，多出来的开销只是比较宽度 21→22。
//  代价：tag 阵列必须显式清零（valid=0 编码成整个字为 0），不能再靠独立的 valid 向量兜底。
    (* ram_style = "distributed" *) reg [DC_TAGW-1:0] way0_tag [0:DC_SETS-1];   // [21]=valid, [20:0]=tag
    (* ram_style = "distributed" *) reg [DC_TAGW-1:0] way1_tag [0:DC_SETS-1];
//★ 2026-11-04（B 口）：tag 阵列副本。LUTRAM 只有一个读口，而 A 口每拍都在读，
//  B 口要判命中就只能另开一份。回填时两份一起写（同一沿同一个值），内容恒等。
//  代价 = 2 路 × 256 项 × 22 位 的分布式 RAM。
    (* ram_style = "distributed" *) reg [DC_TAGW-1:0] way0_tag_b [0:DC_SETS-1];
    (* ram_style = "distributed" *) reg [DC_TAGW-1:0] way1_tag_b [0:DC_SETS-1];
//★ 2026-11-01 试过把 data 阵列按【路】拆成两块、两路并行读（way 只落到最后一级 mux），
//  目标是断掉「读 tag → 命中 → 再读 data」这条两条 BRAM 串联。**实测失败，已回退**：
//  拆开后 Vivado 推断不出 BRAM，落成 RAMD64E（分布式 RAM）⇒ 5925 个失败端点、WNS −3.278。
//  根因是「dout <= way ? mem1[ra] : mem0[ra]」把两个存储器读混在一个寄存器里，推断器不认。
//  结论：这条路要成立，必须给每块阵列各配一个输出寄存器再 mux，那等于每次命中多一拍。
    (* ram_style = "block" *) reg [31:0] data_way0 [0:DC_WORDS-1];
    (* ram_style = "block" *) reg [31:0] data_way1 [0:DC_WORDS-1];
    reg [31:0] dout0, dout1;
    reg        out_sel_q;
    reg        out_fill_q;
//★ 2026-10-30：tag 阵列读地址单独复制一份并加 max_fanout。
//  一个 8 位 index 要扇到 88 个 RAMD64E（256 深 × 22 位 × 2 路）的地址脚，实测那条网
//  单是布线就 2.811 ns（扇出 189），把 dcache 的最差路径卡在 8.5 ns。
//  复制是功能安全的（只多几个驱动器，值完全相同）。icache 的 read_address 也用了同一招。
//  ★ 只给【阵列读】用；LRU 写、数据读地址仍用原来的 index，避免把复制带到别的锥上。
//★ 2026-11-04：命中判定所需的两个 tag 字，在【请求那一拍】就顺手从 LUTRAM 捉下来。
//  这只是同一份读结果多接一个触发器，不占第二个读口、也不改地址（地址仍是 bus_address_in 的切片）。
    (* max_fanout = 8 *) wire [DC_SETS_B-1:0] tag_index = bus_address_in[DC_SETS_B+2:3];
//★ 2026-11-04：当拍命中判定（读 tag 阵列 + 22 位比较）整段删掉，比较器一律在 Q 端：
//  `access_tag_word*_q` 是请求拍从 LUTRAM 捉下来的 tag 字，`access_hit_way0/1` 用纯寄存器量比。
//  原来被它挡着的三处——busy / load_ready_next / load_ready——分别改成：
//    busy：去掉 `read_request && !cache_hit` 这一项（那双 bus_valid 加译码直接到 busy 的路径）；
//          缺失由下一拍的 miss_q 承担，busy 本身在本核没有消费者。
//    load_ready_next / load_ready：都改成"上一拍那笔读 + 寄存器命中"，与 dout 同拍。
    reg access_hit_way0, access_hit_way1;
//回填结束那一路的 ready 要晚 fill_end 一拍（老代码就是这么置的），单独留一格
    reg fill_end_q;

//lru 必须是阵列，不能是宽向量：`reg [255:0] lru` 用运行时下标读写，会被综合成 256 个散
//触发器 + 一棵 256:1 mux 树（实测 256 FDRE + 353 LUT6 + 42 MUXF7/F8，还带一条 fo=260 的
//译码网），而阵列才推得成 LUTRAM。
//★ 2026-10-30：valid 也照这个道理折进了 tag 字（见上），所以下面 initial 里必须把 tag 阵列清零 ——
//  valid=0 现在编码成"整个 tag 字为 0"，不再有独立的 valid 向量兜底。
    (* ram_style = "distributed" *) reg lru [0:DC_SETS-1];

//★ valid 用【世代位】而不是常量 1：tag 字里存的是 `{valid_generation, tag}`，
//  命中的判据是"这一位等于当拍的有效世代"。复位只要把这一位取反，整张表【一次全部作废】，
//  不需要扫 256 项去清、也不依赖配置时初值。
//  这比 icache 那种"清成全 0"更稳（icache 靠 initial 清表，热复位后旧表项还在）。
    reg valid_generation;

    reg stage;
    reg fill_end;
    reg [2:0] miss_word;
    reg [2:0] fill_count;
    reg [31:0] fill_addr;
    reg [DC_TAG_B-1:0] fill_tag;
//★ 2026-11-13：回填写进阵列的世代位必须是【开始填充那一拍】锁下来的那一份，不能写当拍的
//  valid_generation。否则一笔跨越失效的填充会把新世代盖到旧数据上 ⇒ 那行永久有效（和 icache
//  的 fill_pvalid 同一个坑，见 icache.v）。
    reg        fill_generation;
    reg [DC_SETS_B-1:0] fill_index;
//★ 2026-10-31：缺失判定整体打一拍。
//  原来「读 tag 阵列 → 比较 → 命中判定」当拍就要驱动 stage 与 fill_* 一整族的 CE，
//  实测 tag 读之后还要 4.86 ns 才到 fill_index 的 CE（末段是一条扇出 30 的 miss_word 网）。
//  现在：payload 用【便宜的使能】先锁进 _q 寄存器（只跟 read_request 走，不看命中），
//  缺失下一拍才真的进回填。
//★ 2026-11-04：miss_q 不再是触发器，改由【寄存过的】命中组合出来（相位与原 miss_q 完全相同，
//  所以不多花一拍）。原来它挂在当拍的 tag 比较上，实测 WNS −0.084 的 200 条最差路径全指向它。
    reg        miss_q;
    reg        access_read_q;                        // 上一拍受理的是读
    reg [DC_SETS_B-1:0] fill_index_q;
    reg [2:0]  fill_word_q;
    reg [31:0] fill_addr_q;
    reg [DC_TAG_B-1:0] fill_tag_q;
    reg        fill_way_q;
    reg fill_way;

    reg busy_delayed;

    integer i;

    initial begin
        for (i = 0; i < DC_WORDS; i = i + 1) begin
            data_way0[i] = 32'd0;
            data_way1[i] = 32'd0;
        end
        for (i = 0; i < DC_SETS;  i = i + 1) begin
            lru[i]  = 1'b0;
            way0_tag[i] = {DC_TAGW{1'b0}};    // 全 0 = valid=0
            way1_tag[i] = {DC_TAGW{1'b0}};
            way0_tag_b[i] = {DC_TAGW{1'b0}};
            way1_tag_b[i] = {DC_TAGW{1'b0}};
        end
        valid_generation = 1'b1;
    end

//===============================================================
// 第 0 级：本拍组合（请求与命中判定 + 写口合成）
//===============================================================
    always @(*) begin
        tag = bus_address_in[31:DC_SETS_B+3];
        index = bus_address_in[DC_SETS_B+2:3];
        word = bus_address_in[2:0];
        hit_way[0] = access_hit_way0;
        hit_way[1] = access_hit_way1;
        cache_hit = access_hit_way0 || access_hit_way1;

//★ 缺失：上一拍那笔读 + 寄存过的命中 ⇒ 这一拍进回填（与原 miss_q 同相）
        miss_q = access_valid_q & access_read_q & ~(access_hit_way0 | access_hit_way1);
//★ ~miss_q：缺失已经判出来、这一拍就要进回填，不许再收同一个请求
//★ 2026-11-04：A 口只收读 —— 写全部走 B 口（见端口注释）。
        read_request = (stage == 1'b0) && !miss_q && bus_valid_in
                    && (bus_address_in[31:16] == 16'd0);
//命中命中即行内就地更新（B 口），一律【不失效】：整字写整字，子字写按 be 只改那几个字节。
//写穿照做，缓存与后端始终一致。原先子字 store 走失效——失效会连带丢掉同行其余 7 个字，
//下一个 lw/lb 立刻回填（实测 9983 次失效 / 10104 次缺失，1:1），这是缺失的全部来源。
//★ 2026-11-04：命中判定不再当拍合成阵列写使能，见 access_* / store_* 的说明。
//lru 收成【单写口】：命中改 index（stage==0），回填改 fill_index（fill_end 拍 stage 仍为 1）。
//★ hit_upd 也改成吃【寄存过的】命中：原来它挂在当拍的 tag 比较上，实测 WNS −0.312 的
//  200 条最差路径全指向 lru_hit_upd_q_reg/D（route 8.7 ns 占 69%），推后一拍就整段从 CLK 出发。
        hit_upd  = access_valid_q & (access_hit_way0 | access_hit_way1);
//命中更新取【打一拍】的那份；回填仍当拍（fill_end 本来就是这一拍末尾才定）。
//两者撞拍时填表优先 ⇒ 丢掉一次命中更新，提示量，无妨。
        lru_write_enable   = lru_hit_upd_q || fill_end;
        lru_write_index = fill_end ? fill_index : lru_hit_index_q;
        lru_write_value = fill_end ? ~fill_way : lru_hit_value_q;

        read_drive = (stage == 1'b1) && !fill_end;

//---- B 口（ELSQ 排空）：全部由寄存器量与 B 口自己的状态决定 ----
//  收下：本口窗口 + 不忙。命中判定用【上一拍捉下来的 tag 字】，纯寄存器比较。
        store_hit0 = (store_tag_word0_q == {valid_generation, store_tag_q});
        store_hit1 = (store_tag_word1_q == {valid_generation, store_tag_q});
//  缓存更新 one-shot：命中才有；回填写口这一拍要用 ⇒ 让开（下一拍再补，回填最多 8 拍）
        store_cache_write = store_busy & ~store_cache_done & (store_hit0 | store_hit1) & ~is_fill_write;
//  写穿 one-shot：mem 口这一拍被回填占着就等。写穿与命中无关（miss 也要落到后端）。
//  ★ 后端没接住（mem_write_ready=0）时不许发：保持 mem_write_enable 高着等，
//    直到被接住那一拍才算 store_mem_done。
        store_mem_write = store_busy & ~store_mem_done & ~read_drive & mem_write_ready;

//mem 口：回填优先，其次 B 口的写穿
//★ 写使能只能由 store_mem_write 拉高 —— 回填那一路是【读】，写成 we 会把整条 dtcm 写坏
        mem_write_enable = store_mem_write;
        mem_req = read_drive | store_mem_write;
        if (store_mem_write) begin
            mem_addr         = store_address_q << 2;
            mem_write_data   = store_data_q;
            mem_byte_enable  = store_byte_enable_q;
        end
        else begin
            mem_addr         = fill_addr + (fill_count << 2);
            mem_write_data   = 32'd0;
            mem_byte_enable  = 4'b1111;
        end

        busy = (stage == 1'b1 && !fill_end) || (read_request && !cache_hit) || miss_q;

//读口：hit 与 fill_end 合成一个地址/一个数据选择（每个 data 只有一个读口）
//★ 读使能不等 cache_hit、地址不含 way 位：两路并行读，way 只选最后那个 mux
//★ 2026-11-01：阵列【恒读】。地址（{index,word}）直接灌 BRAM 地址线，使能恒 1；
//  「这次访问是不是我的窗口」整个交给【输出】判（out_sel_q，寄存一拍）。
//  道理：读到垃圾无所谓（消费者只认 load_ready），而"非本窗口输出 0"的互斥契约本来就成立在输出那一下。
//  ★ 上一次写成 `fill_end || bus_valid_in` 反而更差 —— 因为 bus_valid_in 里裹着 ROB 的冲刷链；
//    现在那条链已从 lsu 的发请求路径摘掉，所以这里可以彻底不看任何跨模块的组合量。
        read_enable  = 1'b1;
        read_address = fill_end ? {fill_index, miss_word} : {index, word};
        load_ready_next = (read_request & cache_hit) | fill_end;
    end

//写口：两路各一个写使能，回填与 B 口 store 写成【各自一个】表达式，仍能推成两块 BRAM ——
//  ① 回填（stage==1，整字，be 恒 1111）→ 由 fill_way（寄存器）二选一
//  ② B 口 store 命中（收下后下一拍，命中由寄存的 tag 字比出）
//互斥性：store_cache_write 里带了 ~is_fill_write ⇒ 两者永不同拍；回填优先，store 下一拍再补。
//整字写与字节写【必须在同一个 if 里按 be 展开成 WEBA[3:0]】：这才是单写口；
//拆成两个写表达式才会变成"多个写操作"、才推不成 BRAM。
    always @(*) begin
        is_fill_write = (stage == 1'b1) && !fill_end && mem_valid;
//命中判定：两个输入都是寄存器（请求拍捉下的 tag 字 + 请求拍的 tag），整段从 CLK 出发
        access_hit_way0 = (access_tag_word0_q == {valid_generation, access_tag_q});
        access_hit_way1 = (access_tag_word1_q == {valid_generation, access_tag_q});
        way0_write_enable = (is_fill_write & ~fill_way) | (store_cache_write & ~store_hit1);
        way1_write_enable = (is_fill_write &  fill_way) | (store_cache_write &  store_hit1);
        ram_byte_enable     = is_fill_write ? 4'b1111 : store_byte_enable_q;
        if (is_fill_write) begin
            ram_write_address = {fill_index, fill_count};
            ram_write_data = mem_data;
        end
        else begin
            ram_write_address = {store_index_q, store_word_q};
            ram_write_data = store_data_q;
        end
    end

//===============================================================
// 第 1 级：数据阵列写入与读数据输出
//===============================================================
    always @(posedge clk) begin
        if (way0_write_enable) begin
            if (ram_byte_enable[0]) data_way0[ram_write_address][7:0]   <= ram_write_data[7:0];
            if (ram_byte_enable[1]) data_way0[ram_write_address][15:8]  <= ram_write_data[15:8];
            if (ram_byte_enable[2]) data_way0[ram_write_address][23:16] <= ram_write_data[23:16];
            if (ram_byte_enable[3]) data_way0[ram_write_address][31:24] <= ram_write_data[31:24];
        end
    end

    always @(posedge clk) begin
        if (way1_write_enable) begin
            if (ram_byte_enable[0]) data_way1[ram_write_address][7:0]   <= ram_write_data[7:0];
            if (ram_byte_enable[1]) data_way1[ram_write_address][15:8]  <= ram_write_data[15:8];
            if (ram_byte_enable[2]) data_way1[ram_write_address][23:16] <= ram_write_data[23:16];
            if (ram_byte_enable[3]) data_way1[ram_write_address][31:24] <= ram_write_data[31:24];
        end
    end

//每拍先清零：cpu_top 把 dcache/tim_in/dtcm 的读数据按【位或】合流成一个 bus_data_in_final，
//必须靠"非本窗口时输出 0"来互斥。若只在 read_enable 时更新，残留值会被或进别人的读数据
//（实测：读 UART 状态口拿到上一次 dtcm 读的值 → ee_printf 的 while 轮询死循环）。
//★ 2026-11-01：两路 data 阵列【各自】一个输出寄存器，way 的 mux 放在寄存器【后面】。
//  关键是不要写「bus_data_out <= way ? mem1[ra] : mem0[ra]」——两个存储器混进同一个寄存器，
//  Vivado 2019.1 判定 ram_style="block" 不可行、退回 LUTRAM（实测 5925 个失败端点、WNS −3.278）。
//  现在每块阵列都是「RAM + 它自己的输出寄存器」，推断干净；mux 只吃两个寄存器的 Q 和
//  一个【也寄存过】的 way，所以它和 BRAM 的读延迟同拍收口，不额外多一拍。
//★ 2026-11-04：way 那一位不再单独存一份（原来的 read_way_q），直接用 access_hit_way1 ——
//  它和 dout0/dout1 是同一拍锁进寄存器的（都指向前一拍那笔请求），只是来源换成寄存过的比较，
//  于是最后一个挂在当拍 tag 比较上的消费者也没了。
    always @(posedge clk) begin
        if (reset_registered) begin
            dout0 <= 32'd0;
            dout1 <= 32'd0;
            out_sel_q <= 1'b0;
            out_fill_q <= 1'b0;
        end
        else begin
            out_fill_q <= fill_end;
//“这次访问属于 dcache 窗口吗”寄存一拍 —— 只有输出那一拍需要它，所以它不在地址/使能链上
            out_sel_q <= fill_end || read_request;
//阵列恒读：地址直接灌地址线，使能恒 1
            dout0 <= data_way0[read_address];
            dout1 <= data_way1[read_address];
        end
    end
    assign bus_data_out = out_sel_q ? ((out_fill_q ? fill_way : access_hit_way1) ? dout1 : dout0) : 32'd0;

//===============================================================
// 缺失回填状态机与 busy/hold 延拓
//===============================================================
    always @(posedge clk) begin
        if (reset_registered) begin
            stage <= 1'b0;
            fill_end <= 1'b0;
            fill_count <= 3'd0;
            fill_way <= 1'b0;
            fill_end_q      <= 1'b0;
            valid_generation <= ~valid_generation;   // 取反一次 ⇒ 整张 tag 表当拍全部作废
            lru_hit_upd_q   <= 1'b0;
            lru_hit_value_q <= 1'b0;
            lru_hit_index_q <= 8'd0;
            access_read_q   <= 1'b0;
            fill_index_q    <= 8'd0;
            fill_word_q     <= 3'd0;
            fill_addr_q     <= 32'd0;
            fill_tag_q      <= 21'd0;
            fill_generation <= 1'b0;
            fill_way_q      <= 1'b0;
            access_valid_q  <= 1'b0;
            access_address_q <= 11'd0;
            access_tag_q    <= 21'd0;
            access_tag_word0_q <= {DC_TAGW{1'b0}};
            access_tag_word1_q <= {DC_TAGW{1'b0}};
            store_busy        <= 1'b0;
            store_cache_done  <= 1'b0;
            store_mem_done    <= 1'b0;
            store_address_q   <= 32'd0;
            store_data_q      <= 32'd0;
            store_byte_enable_q <= 4'd0;
            store_tag_q       <= {DC_TAG_B{1'b0}};
            store_index_q     <= {DC_SETS_B{1'b0}};
            store_word_q      <= 3'd0;
            store_tag_word0_q <= {DC_TAGW{1'b0}};
            store_tag_word1_q <= {DC_TAGW{1'b0}};
        end
        else begin
            fill_end_q <= fill_end;
//★ 2026-11-13（fence / DMA 一致性）：失效 = 世代位取反，与复位那条同一套机制。
//  ★ 同拍受理的填充在下面锁 fill_generation 时读到的是【取反前】的值（非阻塞赋值读旧值），
//    所以跨越失效的那笔填充写进去自动不命中 —— 正好是我们要的。
            if (invalidate)
                valid_generation <= ~valid_generation;
//命中更新：索引和 way 都取【请求拍捉下的那份】，与 hit_upd 的相位对齐。
//  再往下一拍才落到 LRU 阵列（lru_hit_upd_q），即命中后两拍写完，提示量，晚一拍无妨。
            lru_hit_upd_q   <= hit_upd;
            lru_hit_index_q <= access_address_q[DC_SETS_B+2:3];
            lru_hit_value_q <= access_hit_way0;

            if (lru_write_enable)
                lru[lru_write_index] <= lru_write_value;

//便宜的口：只跟 read_request 走、不看命中判定 ⇒ CE 里没有 tag 读和比较。
//  回填要用的一整套（index/word/addr/tag/way）先在这里锁下来。
            if (read_request) begin
                fill_index_q <= index;
                fill_word_q  <= word;
                fill_addr_q  <= (bus_address_in & 32'hFFFFFFF8) << 2;
                fill_tag_q   <= tag;
                fill_generation <= valid_generation;
                fill_way_q   <= ((way0_tag[tag_index][DC_TAG_B] != valid_generation)) ? 1'b0 :
                                ((way1_tag[tag_index][DC_TAG_B] != valid_generation)) ? 1'b1 : lru[index];
            end
//缺失判定上一拍就做完了，这一拍直接进回填
            if (miss_q) begin
                stage <= 1'b1;
                miss_word  <= fill_word_q;
                fill_count <= 3'd0;
                fill_addr  <= fill_addr_q;
                fill_tag   <= fill_tag_q;
                fill_index <= fill_index_q;
                fill_way   <= fill_way_q;
            end

//★★ 命中判定推后一拍：这一拍只锁载荷 + 捉 tag 字。使能只看 read_request，
//   不回读命中判定；命中/way 留到下一拍用这里的寄存量算。
            access_valid_q <= read_request;
            access_read_q  <= read_request;
            if (read_request) begin
                access_address_q   <= {index, word};
                access_tag_q       <= tag;
                access_tag_word0_q <= way0_tag[tag_index];
                access_tag_word1_q <= way1_tag[tag_index];
            end

//---- B 口：收下 → one-shot 缓存更新 / 写穿 → 撤 busy ----
//★ 2026-11-05：回填期间不收 store。收下那拍捉的 tag 字还是【旧行】的（miss 时是 invalid ⇒
//  store_hit=0），而写穿带着 ~read_drive 只能排在回填之后 ⇒ 回填先读到旧内存、把旧值整行写进
//  阵列，这笔 store 又因为"没命中"不写阵列 ⇒ 那一行永久是旧值（实测 CoreMark 卡 core_list_init）。
//  挡到回填结束再收，tag 已是新行 ⇒ 走命中那一路（阵列写 + 写穿），两条路都在。
//  只在 stage==0 收不会引入顺序问题：stage==0 收下 ⇒ 下一拍 store_busy，写穿当拍就发
//  （read_drive 这时还是 0），比同拍收下的 load 判 miss 进回填还早一拍。
            if (store_valid_in && !store_busy && (stage == 1'b0) && !miss_q
                && (store_address_in[31:16] == 16'd0)) begin
                store_busy        <= 1'b1;
                store_cache_done  <= 1'b0;
                store_mem_done    <= 1'b0;
                store_address_q   <= store_address_in;
                store_data_q      <= store_data_in;
                store_byte_enable_q <= store_byte_enable_in;
                store_tag_q       <= store_address_in[31:DC_SETS_B+3];
                store_index_q     <= store_address_in[DC_SETS_B+2:3];
                store_word_q      <= store_address_in[2:0];
                store_tag_word0_q <= way0_tag_b[store_address_in[DC_SETS_B+2:3]];
                store_tag_word1_q <= way1_tag_b[store_address_in[DC_SETS_B+2:3]];
            end
            else if (store_busy) begin
                if (store_cache_write || (store_busy && !(store_hit0 | store_hit1)))
                    store_cache_done <= 1'b1;
                if (store_mem_write)
                    store_mem_done <= 1'b1;
                if ((store_cache_done || !(store_hit0 | store_hit1)) && (store_mem_done || store_mem_write))
                    store_busy <= 1'b0;
            end

            if (stage == 1'b1 && !fill_end && mem_valid) begin
                if (fill_count == 4'd7)
                    fill_end <= 1'b1;
                else
                    fill_count <= fill_count + 3'd1;
            end

            if (fill_end) begin
                if (fill_way) begin
                    way1_tag  [fill_index] <= {fill_generation, fill_tag};
                    way1_tag_b[fill_index] <= {fill_generation, fill_tag};
                end
                else begin
                    way0_tag  [fill_index] <= {fill_generation, fill_tag};
                    way0_tag_b[fill_index] <= {fill_generation, fill_tag};
                end
                stage <= 1'b0;
                fill_end <= 1'b0;
            end
        end
    end

//★ request_blocked：纯状态量（A 口）。只有回填期间不收新读。
    assign request_blocked = (stage != 1'b0);
//★ store_blocked：B 口忙。LSU 用它挡排水 —— 这是【寄存器】，排水判据里再没有地址量。
//★ 2026-11-05：回填期间也算"B 口忙"，否则 store 会在回填中途被收下（收下那拍的 tag 还是旧行，
//  store_hit=0 ⇒ 不写阵列），而写穿带着 ~read_drive 只能排在回填之后 ⇒ 回填先把旧值整行写进
//  阵列、再没人纠正（实测 CoreMark 卡 core_list_init）。
//  ★ 必须【并进 store_blocked】而不是只加在 B 口受理条件里：LSU 是"亮出排水 + 看忙位撤销"
//    这套握手（store_inflight / store_finish = inflight & ~store_blocked），受理条件单方面
//    拒收会让它把没送出去的 store 当成办完了 ⇒ 那笔 store 直接蒸发（实测随机差分 seed8 挂）。
//  ★ 挡的只是【排水】，load 的发送本来就被 request_blocked 挡在回填之外。
//  ★ miss_q 也要算进来：缺失判出来到 stage 置起之间隔着一拍，那一拍 stage 还是 0，
//    只在 stage==0 收 store 会让它正好落在"回填下一拍就开始"的缝里 —— 收下当拍没事，
//    写穿被下一拍起的 read_drive 压到回填结束，回填就已经把旧值读走了（实测 c=64152 收、
//    c=64162 回填读、c=64163 才写穿，`lw x1,12(sp)` 读回 0）。
    assign store_blocked = store_busy | (stage != 1'b0) | miss_q;

//结果口就绪：命中那一路与 fill_end 那一路，都用寄存量出，D 脚路径整段消失。
    assign load_ready = (access_read_q & (access_hit_way0 | access_hit_way1)) | fill_end_q;

//busy 的 1 拍延拓（原 cpu_top 的 d_hold_int，按"顶层不运算"下放到此）：
//fill_end 拍 busy 就掉、load_ready 要再等一拍，这 1 拍空窗必须兜住。
    always @(posedge clk) begin
        if (reset_registered)
            busy_delayed <= 1'b0;
        else
            busy_delayed <= busy;
    end

    always @(*) begin
        hold = busy | busy_delayed;
    end
endmodule
