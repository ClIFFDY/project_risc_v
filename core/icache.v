`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: icache
// Project Name:
// Target Devices:
// Tool Suites:
// Description: 唯一取指源。容量 = IC_SETS 组 × 2 路 × 2^IC_WORD_B 字。
//              内含取指地址生成（原 itcm）。
//              两路组相连，组内 LRU。
//              ★ 2026-11-01：容量参数化。"扩容量"= 只改 IC_INDEX_B（组数位宽）一位，
//                行大小 IC_WORD_B 不动、fill FSM 一行不动、itcm 不动（itcm 本来就是 64KB，
//                read_address = mem_addr[15:2] 14 位顶满）。
//                当前 IC_INDEX_B = 8 ⇒ 256 组 × 2 路 × 32 字 = 64KB。
//                IC_INDEX_B = 7 ⇒ 128 组 = 32KB（旧配置，行为逐位相同）。
//              ★ 2026-10-27：两件事一起做了 ——
//               ① 删掉"复位后把前 16KB 预装进 way0"的 bootloader：它把 4864 拍花在 CoreMark
//                  计分区【外】，只换一个"第一段永远命中"的假象，其余什么也没买到。
//               ② 顺手把 way0 这一路也删了：way0 原本的唯一身份就是"tag==0 那 16KB 的直接
//                  映射家"，自举一走它就是一条特例路径（常量比较、无 tag 阵列），不是容量。
//              现在是标准两路：统一 tag 阵列 + 统一 LRU，没有任何"按地址段认路"的分支。
//              容量账：-O2 正文 29040B（227 行）装得下 2×128 = 256 槽；
//              -O3 36212B、max-opt（unroll+LTO）50388B 都超过 32KB ⇒ 已是 256 组（512 槽），
//              -O3 与 max-opt 均可整份常驻。
//              需要"整份常驻、零冷缺失"的稳态，请由测试台后门装 iram + tag，不要加回 bootloader。
//
//              本文件内 always 块按【流水级数】排列：
//                第一级 取指地址 → 第二级 命中判定与缺失锁 → 第三级 取指输出
//                → 回填状态机。
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module icache(
    input clk, rst,
//取指地址生成（原 itcm）
    input [31:0] pc_address_0, pc_address_1,
    input exception_interrupt,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input [2:0] flush_control_in,   // = flag_bus[7:5]：{异常, 中断, 改向}
    input       stall_pc_redirect_in,   // = stall_pc_redirect_in
    input       stall_icache_miss_in,   // = stall_icache_miss_in
//★ 随地址一起进流水线的预测：pc 这一拍【照做】的落点表预测（pc_take / btb_tgt 的组合输出）。
//  相位：pc_address_0(c-1) 就是本拍 R0 里那一对，而落点表的组合输出也按 pc_address_0(c-1) 算
//  ⇒ 在【交付寄存器装载的那一个沿】，` pc_take ` 与 R0 里的那一对说的是同一条指令。
//  所以它跟指令字一样，在同一个 always 块里、用同一个 deliv_load 装进交付寄存器 ——
//  不是"另外再打一拍对齐"，而是**本来就是同一位**。下面 instruction_out 那个块里一起写。
    input        pc_take_0, pc_take_1,
    input [31:0] btb_target_0, btb_target_1,
//回填应答（接核内 itcm）
    input mem_valid,
    input [31:0] mem_data,
//本拍 pc 落到重定向目标（pc.v 的 redir_go，源名 flush_pc_redirect）：挡掉"跳转前那次取指"的陈旧交付
    input flush_pc_redirect,
//取指队列满（iffu 的 full）：本拍放不下，交付寄存器保持、不再推进
    input fifo_full,

//取指输出
    output reg [31:0] instruction_out, instruction_next,
//与两条 lane 的指令同拍交付的地址：与数据走同一级寄存，每条指令自带自己的地址，
//下游不再需要"拍数"或"基准+4"来推它是哪一条。
    output reg [31:0] instruction_address_0, instruction_address_1,
//★ 与这一对指令【同一个寄存器组、同一个装载门】交付的预测：
//   dpred_*   = 这一对里有被预测跳的（取指当时 pc 照做的那一位）
//   djpred_*  = 那一位对应的预测落点（没预测跳 ⇒ 0）
//  取指队列条目里的 br_pred / jalr_pred 就是这两位，原样存。
    output reg        prediction_taken_0, prediction_taken_1,
    output reg [31:0] prediction_jump_target_0, prediction_jump_target_1,

    output reg instruction_valid,
    output reg instruction_valid_next,
//busy = 本模块自己的停顿源（stall_icache_miss），也是给 idu1 的垃圾指令守卫。
//它是寄存器，不是组合输出：由 stage / fill_end / icache_miss 自己说，不再从 cache_hit 现组合出来。
    output reg busy,
//缺失检测拍（组合，直接送 pc，不走 flag_bus）：
//busy 是寄存的、要到下一拍才起来，pc 若在检测拍照常推进就会走过被丢的那一对，
//回填完 fill_ok 又按"当前 pc"取数 ⇒ 补的是下一对，被丢的那一对永远补不回来。
//实测：16KB 外的循环，负载轻时每圈丢一对（16 个随机背压种子里 7 个多一次错路重定向）。
    output reg icache_miss,
//★★ 唯一前进判据：整个取指级（pc / R0 / 交付口 / 落点表输出寄存器）这一拍前进一格。
//  只在本模块算一次，四组寄存器全吃它 ⇒ 指令与随它的预测（dpred/djpred）结构上不可能差拍。
//  ★ 不许把它寄存一拍：寄存 = 相位错，四组立刻错开。
    output reg       advance,
//★ R0 那一组的两个状态位（与 btb_* 同一沿、同一对，不另打拍）：
//  r0_valid  = 这一对有效；r0_wrong_path = 这一对是【预测跳之后顺延取回来的错路】。
    output reg       r0_valid, r0_wrong_path,
//★ 这一对是错路（与指令同组同门的位）。iffu 的 slot_rep 直接吃它。
    output reg       deliver_wrong_path,
//回填请求（接核内 itcm）
    output reg mem_req, mem_write_enable,
    output reg [31:0] mem_address, mem_write_data,
    output reg [3:0] mem_byte_enable
    );

//★★ 2026-11-01：容量参数（唯一旋钮）。改 IC_INDEX_B 就是"tag 加一位"：
//  行大小 IC_WORD_B 不变（32 字 = 128B），组数翻倍，地址三段的位段全部自动跟随。
//  位段来源：fetch_word_index = pc >>> 2，所以
//    word_*  = fetch_word_index[IC_WORD_B-1 : 0]                     → pc[IC_WORD_B+1 : 2]
//    index_* = fetch_word_index[IC_WORD_B+IC_INDEX_B-1 : IC_WORD_B]  → pc[IC_WORD_B+IC_INDEX_B+1 : IC_WORD_B+2]
//    tag_*   = fetch_word_index[31 : IC_WORD_B+IC_INDEX_B]           → pc[33 : IC_WORD_B+IC_INDEX_B+2]
//  两个恒等式（改参数后由本文件末尾的自检块强制）：
//    IC_TAGW  = IC_TAG_B + 1    ⇐ tag 阵列字宽 = {valid, tag}
//    IC_TAG_B = IC_FOLD + 2     ⇐ tag 高 2 位恒 0（pc[33:32] 不存在），低 IC_FOLD 位才是真 tag
    parameter integer IC_WORD_B  = 5;                                  // 行内字数位宽
    parameter integer IC_INDEX_B = 8;                                  // 组数位宽（256 组）
    localparam integer IC_SETS   = 1 << IC_INDEX_B;                    // 组数
    localparam integer IC_TAG_B  = 32 - IC_WORD_B - IC_INDEX_B;        // tag_* 寄存器位宽
    localparam integer IC_TAGW   = IC_TAG_B + 1;                       // tag 阵列字宽
    localparam integer IC_FOLD   = 32 - IC_INDEX_B - IC_WORD_B - 2;    // tag 的有效位（拼回 pc 用）
    localparam integer IC_ADDR_B = 1 + IC_INDEX_B + IC_WORD_B;         // {way, index, word}
    localparam integer IC_WORDS  = IC_SETS << (IC_WORD_B + 1);         // iram 字数 = 2 路 × 组数 × 行字数
    localparam [IC_WORD_B-1:0] IC_WORD_LAST = {IC_WORD_B{1'b1}};       // 行内末字号

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//flag_bus = {flush_control_exception, flush_control_interrupt, flush_control_jump, exec,
//            stall_rob_full, stall_pc_redirect,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：先把 flag_bus 各位还原成原名，再按名字做逻辑。
//★★ 2026-10-26 前端信号链收口（这一段存在的唯一理由）：
//   原来有一根 req_valid = ~(flush_window | stall_pc_redirect | stall_icache_miss | busy | fifo_full | ~exec)，
//   把六条互不相干的原因压成一根，再分给三处判据：
//     rden_q   ← req_valid            （第一级收没收下这一对）
//     fetch_ok ← req_valid            （发给 pc 的握手）
//     pc 那边再算 fetch_ok & ~ic_freeze（同一条事实第三遍）
//   其中两项是别处信息的重复：stall_icache_miss = stall_icache_miss_in 与同式里的 busy 是同一个
//   信号（见下）；exec 在 cont 里只写 1'b1，`~exec` 是死项。
//   现在：一个判据、一个地方、逐条列。go = "整个前端本拍前进一格" ——
//   R0 收下这一拍呈现的地址、pc 同拍前进、rden_q 同拍置位，三件事同一个沿同一个条件。
//   外部信号按原样进来：flag_bus 每一位各解各的，fifo_full 是它自己一根口；
//   该相或的地方就在下面这个 if 链里相或，不再合成一根意思含混的线发给别人。
    reg flush_control_exception, flush_control_interrupt, flush_control_jump;
    reg stall_pc_redirect;
    reg flush_window;
//★★ 原来 req_valid 里的 stall_icache_miss 就是 stall_icache_miss_in，而 cpu_top 是把 icache 的
//  busy 直接接进 cont（.stall_icache_miss(busy_ic)），cont 的 flag_bus 又是【组合】的
//  ⇒ 那一项与同在那根式子里的 busy 是**同一个信号、同一个相位**（busy | busy）。
//  这里只是把这份重复去掉，语义一字不改；pc 那边同样不再需要它。
    always @(*) begin
        flush_control_exception  = flush_control_in[2];
        flush_control_interrupt  = flush_control_in[1];
        flush_control_jump = flush_control_in[0];
        stall_pc_redirect = stall_pc_redirect_in;
        flush_window = flush_control_exception | flush_control_interrupt | flush_control_jump;
    end

//寄存器（按级分组）
//第 0 级：本拍组合
    (* max_fanout = 32 *) reg [31:0] fetch_word_index_0, fetch_word_index_1;
    (* max_fanout = 32 *) reg [IC_TAG_B-1:0] tag_0, tag_1;
    (* max_fanout = 32 *) reg [IC_INDEX_B-1:0] index_0, index_1;
    (* max_fanout = 32 *) reg [IC_WORD_B-1:0] word_0, word_1;
    reg [1:0] hit_way_0, hit_way_1;
    reg hit_select_0, hit_select_1;
    reg hit_0, hit_1;
    (* max_fanout = 32 *) reg [IC_ADDR_B-1:0] read_address, read_address_lane1;
    reg read_enable;
    reg cache_hit;
//第 1 级：由 Q 端字段拼回来的取指地址（pc = tag[IC_FOLD-1:0] & idx & word & 2'b00）
    reg [31:0] fetch_address_0, fetch_address_1;
//第 0R 级：tag 阵列 Q 端。整组同一个沿、同一套条件，一位都不能单独打拍。
    reg [IC_TAG_B-1:0] fetch_tag_registered_0, fetch_tag_registered_1;
    reg [IC_TAGW-1:0] way1_tag_registered_0, way1_tag_registered_1;
    reg [IC_TAGW-1:0] way2_tag_registered_0, way2_tag_registered_1;
    reg [IC_INDEX_B-1:0]  index_registered_0, index_registered_1;
    reg [IC_WORD_B-1:0]  word_q0, word_q1;
    reg read_lane1;
    reg write_enable;
    reg [IC_ADDR_B-1:0] write_address;
    reg [IC_ADDR_B-1:0] port_b_address;

//阵列与状态机
//tag 阵列存 {valid, tag}：最高位就是有效位，命中判据退化成【一次 20 位比较】。
//原来 valid 是另一份 128 位向量、再和 tag 比完的结果相与，多一级 LUT、多一条 128:1 选择。
    (* ram_style = "distributed" *) reg [IC_TAGW-1:0] way1_tag [0:IC_SETS-1];
    (* ram_style = "distributed" *) reg [IC_TAGW-1:0] way2_tag [0:IC_SETS-1];
    (* ram_style = "block" *) reg [31:0] iram [0:IC_WORDS-1];
    reg [IC_SETS-1:0] lru;

//★★ 2026-10-27：自举 bootloader 与 way0 一路一起删除，见文件头。
//way1_tag/way2_tag 现在就是【两路】：way1_tag = way0（`fill_way/hit_way` 里编号 0），way2_tag = way1（编号 1）。
//  编号沿用旧名不改，只在读写处按 0/1 使用 —— 别再从名字推断"第几路"。
//reset_period_flag 只保留给测试台做起跑对齐（全仓 114 个 tb 写 `wait(!reset_period_flag)`）：现在它【只表示复位期】。
    reg reset_period_flag;

    reg stage;
    reg fill_end;
//★ 回填收尾之后的那一拍（fill_end 打一拍）。见下面 freeze / icache_miss / busy / FSM 起点四处。
    reg fill_done;
    reg [IC_WORD_B-1:0] miss_word;
    reg [IC_WORD_B-1:0] fill_count;
    reg [31:0] fill_addr;
    reg [IC_TAG_B-1:0] fill_tag;
    (* max_fanout = 32 *) reg [IC_INDEX_B-1:0] fill_index;

//★ 前进判据：一条原因一个分支，顺序即优先级。分成两段只为时序：
//  go_nf 里的原因都来自寄存器（flag_bus 是 cont 的寄存器输出、fifo_full 是 iffu 的组合），
//  而 freeze/busy 里含【当拍的 tag 比较】（icache_miss）—— 那一段锥不该进 rden_q 的 D 端。
//  分两段之后：rden_q 的 D 端只吃 go_nf（在 ~freeze 那一支里用，此时 miss/busy 必为 0）；
//  pc 吃它自己那张表（同样含 busy_d，见 pc.v）。

//★ 回填替换路：只依赖【寄存好的 fill_index】，不依赖缺失检测那一拍的任何东西。
//  原来它是"缺失检测拍现算、寄进寄存器"，于是整条
//  iram 输出 → 译码 → pc 加法器 → pc_addr → icache 索引 → valid/lru → 这里
//  全被压在同一个周期里（实测最差路 22 级 / -6.388ns，端点就是 fill_way_reg）。
//  回填期间 valid/lru 在 fill_index 这一格不变（stage==1 不写 lru，valid 只在 fill_end 写），
//  所以改成组合读完全等价。
    wire [IC_TAGW-1:0] fill_tag_way1 = way1_tag[fill_index];
    wire [IC_TAGW-1:0] fill_tag_way2 = way2_tag[fill_index];
//★ 两路：先挑没效的，两路都有效才按 LRU。`lru[idx]` 存的就是【下次该换哪一路】。
    wire fill_way = (!fill_tag_way1[IC_TAG_B]) ? 1'b0 :
                    (!fill_tag_way2[IC_TAG_B]) ? 1'b1 :
                    lru[fill_index];

    integer i;

//阵列初值：未写过的格位最高位是 0，与 {1'b1, tag} 永不相等 ⇒ 天然不命中。
//不用复位沿：复位会给 2x128x20 位各长一根复位线，撑爆 slice 打包（原注释的同一笔账）。
    initial begin
        for (i = 0; i < IC_SETS; i = i + 1) begin
            way1_tag[i] = {IC_TAGW{1'b0}};
            way2_tag[i] = {IC_TAGW{1'b0}};
        end
    end

//===============================================================
// 第 0 级：本拍组合（取指地址 + 命中判定与缺失锁）
//===============================================================
//取指地址：**只取 pc_addr >>> 2**，不吃 jal/br1，也没有那个 32 位加法器。
//jalr 早就是"落点只送 pc"；现在 jal/br1 也一样 —— 它们的改向落点只进 pc，
//而"目标那条指令"由 bpu 里那块独立的 btb 直接交付给 idu1。
//这样一来，"译码器 + 加法器 → 取指地址 → tag 阵列读 → 命中选择 → iram 地址口"
//那条 13 级/14.4ns 的链整条消失。
    always @(*) begin
        fetch_word_index_0 = pc_address_0 >>> 2;
        fetch_word_index_1 = pc_address_1 >>> 2;
    end

//写口：回填写（自举已删，这是【唯一条写语句、唯一地址表达式】）。写成两条写语句（两个地址表达式），
//综合器会各推一个写口 ⇒ iram 变多写口 ⇒ 进不了 BRAM，会溶解成 12K×32 个触发器（实测综合 15 分钟以上）。
    always @(*) begin
        write_enable   = (stage == 1'b1) & ~fill_end & mem_valid;
        write_address = {fill_way, fill_index, fill_count};
    end

    always @(*) begin
        tag_0  = fetch_word_index_0[31:IC_WORD_B+IC_INDEX_B];
        index_0  = fetch_word_index_0[IC_WORD_B+IC_INDEX_B-1:IC_WORD_B];
        word_0 = fetch_word_index_0[IC_WORD_B-1:0];
        tag_1  = fetch_word_index_1[31:IC_WORD_B+IC_INDEX_B];
        index_1  = fetch_word_index_1[IC_WORD_B+IC_INDEX_B-1:IC_WORD_B];
        word_1 = fetch_word_index_1[IC_WORD_B-1:0];
        mem_req = (stage == 1'b1 && !fill_end);
        mem_address = fill_addr + (fill_count << 2);
        mem_write_enable = 1'b0;
        mem_write_data = 32'd0;
        mem_byte_enable = 4'd0;
    end

//第 0R 级：tag 阵列 Q 端打一拍。
//这一刀切在【阵列读之后、比较之前】。实测那一段的时间轴是：
//  pc 出来到阵列地址口 8.336ns（全是线）、阵列读 +2.164、命中判据加选 way +1.1、iram 地址口约 12.9。
//切在 way 选择之后 = 前一半仍有 13.5ns，等于没切；切在阵列 Q 端，前一半只剩
//"pc 的线 + 阵列读"，后一半是"比较 + 选 way + 拼地址"，两边都进预算。
//前进握手是 fetch_ok（"这一拍喂进来的地址被发去读了"）。
//★ 缺失判据比地址晚一拍：miss 回来时 pc 已经走到 miss+8，但这一级里还压着 miss 那一对。
//  所以 miss 拍把【pc 和这一级一起冻住】：pc 由 icache_miss 挡住前进，这一级由 freeze 保持，
//  回填完这一级原样重放一次 ⇒ 那一对不会被跳过、也不会被交付两次。
//冻结信号已经是端口（output reg freeze），这里不再重复声明
//★★ 唯一前进判据。一条原因一个分支，顺序即优先级。
//  ★ 这里【只有一份】这一拍走不走：pc / R0 / 交付口 / 落点表输出寄存器 全吃它。
//    以前它被拆成 go_nf / go / freeze / rden_q / deliv_load 五份，任何一份与别人不等价，
//    指令和它的同伴信号就差一拍 —— 而且不报错，只是静默记错。
//  ★ redirect（flush_pc_redirect）在这里是【冻】：这一拍只有 pc 改值，其它三组原地不动。
//    改向目标下一拍才由 pc_r 呈现给 R0 —— 代价是每次外部改向 +1 拍，换掉的是
//    「改向目标连着两拍出现在口上 ⇒ 同一对交付两次」那一族。
    always @(*) begin
        if (flush_control_exception)           advance = 1'b0;   // 陷阱交付这一拍
        else if (flush_control_interrupt)      advance = 1'b0;   // 中断受理这一拍
        else if (flush_pc_redirect)     advance = 1'b0;   // 外部改向落地这一拍（只改 pc 的值）
        else if (stall_pc_redirect)     advance = 1'b0;   // 改向已登记、ROB 未排空
        else if (fifo_full)          advance = 1'b0;   // 取指队列满：口上这一对收不下
        else if (busy)               advance = 1'b0;   // 回填中（iram 口 B 被占）
        else if (icache_miss)        advance = 1'b0;   // 这一对不在 cache 里 ⇒ 原样重放
        else                         advance = 1'b1;
    end

//★★ G3：tagram 那一级（R0）。整组一个沿、一个使能 —— 地址字段、阵列 Q 端、有效位、错路位
//  全部同时换。唯一例外是 fill_done（回填收尾那一拍只按【同一个 tag】重读阵列，见下）。
    wire flush_all = flush_control_exception | flush_control_interrupt | flush_pc_redirect;
    always @(posedge clk) begin
        if (reset_registered) begin
            fetch_tag_registered_0 <= 0;   fetch_tag_registered_1 <= 0;
            way1_tag_registered_0 <= 0;   way1_tag_registered_1 <= 0;
            way2_tag_registered_0 <= 0;   way2_tag_registered_1 <= 0;
            index_registered_0  <= 0;    index_registered_1  <= 0;
            word_q0 <= 0;    word_q1 <= 0;
            r0_valid    <= 1'b0;    r0_wrong_path   <= 1'b0;
        end
        else if (advance) begin
            fetch_tag_registered_0 <= tag_0;        fetch_tag_registered_1 <= tag_1;
            way1_tag_registered_0 <= way1_tag[index_0];  way1_tag_registered_1 <= way1_tag[index_1];
            way2_tag_registered_0 <= way2_tag[index_0];  way2_tag_registered_1 <= way2_tag[index_1];
            index_registered_0  <= index_0;        index_registered_1  <= index_1;
            word_q0 <= word_0;       word_q1 <= word_1;
            r0_valid    <= 1'b1;
//★ 错路位与地址同沿装进同一组：预测跳之后顺延取回来的那一对，就是这一位 = 1 的那一对。
//  它替代原来的 tk_drop（上拍刚按表改过向）—— 由位本身带着走，不靠拍数推断，
//  冻结/重放时跟着同一对一起保持。
            r0_wrong_path   <= pc_take_0 | pc_take_1;
        end
        else if (flush_all) begin
//★ 顺序：advance > flush_all > fill_done。flush_all 必须优先于 fill_done —— 两者同拍时若先做
//  阵列重读，r0_valid 不会被清 ⇒ 下一拍把改向前那一对当成有效交付出去（错路指令进队列）。
//  跳过那一拍重读无害：改向之后 R0 下一拍就按新地址整组重装。
            r0_valid  <= 1'b0;
            r0_wrong_path <= 1'b0;
        end
        else if (fill_done) begin
//★ 唯一例外：tag 是在 fill_end 那一沿才写进阵列的，同一沿 R0 读回的是【写之前】的值
//  ⇒ 不重读就永远用陈旧有效位 ⇒ 同一行无限回填（16KB 之外第一条指令起）。
//  ★ 地址字段一个字都不能换：R0 里压着的就是缺失那一对，重放要的是同一个 tag 再比一次。
            way1_tag_registered_0 <= way1_tag[index_registered_0]; way1_tag_registered_1 <= way1_tag[index_registered_1];
            way2_tag_registered_0 <= way2_tag[index_registered_0]; way2_tag_registered_1 <= way2_tag[index_registered_1];
        end
    end

//第 1 级：拼回取指地址、比较、选 way、出 iram 地址。
//两路对称：路号直接由命中项给出，不走优先级链（同一 tag 不会同时落在两路）。
    always @(*) begin
        fetch_address_0 = {fetch_tag_registered_0[IC_FOLD-1:0], index_registered_0, word_q0, 2'b00};
        fetch_address_1 = {fetch_tag_registered_1[IC_FOLD-1:0], index_registered_1, word_q1, 2'b00};
        hit_way_0[0] = (way1_tag_registered_0 == {1'b1, fetch_tag_registered_0});
        hit_way_0[1] = (way2_tag_registered_0 == {1'b1, fetch_tag_registered_0});
        hit_way_1[0] = (way1_tag_registered_1 == {1'b1, fetch_tag_registered_1});
        hit_way_1[1] = (way2_tag_registered_1 == {1'b1, fetch_tag_registered_1});
        hit_0 = hit_way_0[0] | hit_way_0[1];
        hit_1 = hit_way_1[0] | hit_way_1[1];
//两条 lane 各配各的 tag：任意一条没命中就要回填，缺谁补谁的行
        cache_hit = hit_0 & hit_1;
        hit_select_0 = hit_way_0[1];   // 0 = way0(way1_tag)，1 = way1(way2_tag)
        hit_select_1 = hit_way_1[1];
        read_enable    = r0_valid;      // R0 里这一对有效（原来是：上一拍推进过）
        read_address  = {hit_select_0, index_registered_0, word_q0};
        read_address_lane1 = {hit_select_1, index_registered_1, word_q1};
        read_lane1    = read_enable & hit_1;
    end

//口 B 地址二选一：回填写，或 lane1 取指读。BRAM 一个口只有【一根地址总线】，读写是它的时分复用，
//两个来源的地址必须先显式合成这一根，再进阵列；否则综合器数出三个地址会判 Infeasible ram_style，
//把整个 32KB 阵列掉进 LUTRAM（且不中断流程）。写与 lane1 读不共拍（写 ⟹ stage==1 ⟹ busy ⟹ 取指停）。
    always @(*) begin
        if (write_enable)
            port_b_address = write_address;
        else
            port_b_address = read_address_lane1;
    end

//缺失检测拍（组合直送 pc，不走 flag_bus）：与 busy 的第三个加项逐字同门。
//这一拍 pc 必须停住，否则它走过被丢的那一对，回填完再也补不回来。
    always @(*) begin
        icache_miss = read_enable & ~cache_hit;
    end

//★ 交付口契约：instruction_out 被刷成 0 的那一拍，instruction_valid 必须同时拉低。
//  原来只刷数据不刷有效位 ⇒ 下游看到"有效 且 指令=0"：BTIC 的冷启动捕获就是用
//  inst_valid_0/1 当资格 ⇒ 把一整对 0 当目标 inst 写进表（bti_v=1、tag 正确、inst=0），
//  此后该 pc"命中"到空指令：pc 为它跳过 8 字节、注入的却全是 0 ⇒ 目标那两条【永久丢失】。
//  有效位与数据必须同一个门 —— 这就是那个门。
    reg discard_output;
    always @(*) begin
//★ 只有"这一拍换 pc"的场合才作废交付。跳转类（jal/br1/jalr）不再在此列：
//  改向已由取指落点表提前一拍完成，这一拍读到的是目标那一对，不存在错路。
//★ 只有"这一拍换 pc"的场合才作废交付，来源只有三个：ROB 队头交付（陷阱）、提交口改向
//  （分支/jalr 的 B、mret 的 A、陷阱/中断落地，都并进 flush_pc_redirect）、irq 受理。
        discard_output = (exception_interrupt | flush_control_exception | flush_pc_redirect);
    end

//★ 交付寄存器的装载条件 = "这一次读回来的那一对，下游【当拍】收得下"。
//  read_enable 是【上一拍】的推进握手（rden_q），拿它当装载条件，fifo_full 就晚了一拍：
//  队满那一拍刚被读回来的一对会顶掉队口上正等着被收的那一对 ⇒ 静默丢两条指令
//  （实测 seed1：推入流 0x8c 直接跳到 0x98，0x70/0x74 与 0x90/0x94 两组在队满沿上蒸发，
//   随后整核在 0x6c8 死锁）。参考树里 read_enable 与 pc 同拍，所以那里的 full 天然是当拍的；
//  切了 tag Q 之后 read_enable 晚了一拍，这个与门必须自己把 full 重新对齐到【装载这一拍】。
//★★ 这一位同时是【交付口的唯一装载握手】，并且从端口输出给顶层当边带使能：
//   随指令走的那几份边带（预测跳/落点/BTIC 补位/错路标记）必须与这一沿逐字同门。
//   顶层曾经用 `fetch_ok 晚一拍 & ~full` 复算它 —— 两者在【冻住】的那几拍不等价：
//   freeze 时 rden_q 保持不变（read_enable 可以还是 1），而 fetch_ok 当拍已经是 0
//   ⇒ 交付口在这一沿换了一对，边带却没跟着换。实测 CoreMark 清 .bss：
//   0x18-pair 因为队满冻了 3 拍，`j 18` 的"错路标记"因此晚了一对，
//   顺延取回的错路对 (0x28,0x2c) 被当成好指令推进队列、`jal main` 真的退休并改向，
//   而**正清 .bss 的那条 `addi t0,t0,4` 的退休写被这次冲刷吃掉**（写回流第 31 笔丢失）。

//★★ 影子：待交付那一对的预测。
//  为什么需要它：pc 按落点表跳的那一拍【不受冻结门控】，而交付口那一沿可能因为队满不装载
//  （deliv_load=0）⇒ 直接"在交付沿采 pc_take"会把这一对的预测漏掉，下一对却拿到了别人的预测
//  （实测前端台 seed9：pc 按 0x5384 的 lane1 跳到 0x5378，队列条目里 br_pred 记成 0 ⇒
//   bju 认为"没预测跳"，那条之后就没有人改向）。
//  pc_pred_v = "这一拍的落点表信息描述的正是 R0 里那一对"，此时把预测锁进影子；
//  交付沿优先取当拍（它描述的正是要交付的那一对），失效时取影子（R0 没换就一直有效）。
    always @(posedge clk) begin
        if (reset_registered)
            instruction_valid <= 1'b0;
//★ 原来这里在自举收满那一拍做"交接"（read_enable 强行置 1 + instruction_valid<=1），把 iram 里那条
//  直接当入口指令交出去。实测那是**重复交付**的唯一来源：交接在自举最后一拍就交付了入口
//  （pc=0），而 pc 又被 busy 多按一拍 ⇒ 下一拍 icache 按同一个地址 0 再读一次、再交付一次
//  ⇒ 入口那条进队列两次、写回两次（逐条写回比对的第一处不符恒落在入口 lui：RTL 有两条
//  x1=0x800000+）。交接整段删掉（read_enable 那一支与 instruction_out 那一支），入口指令只由 c1 那拍
//  的正常读交付一次；代价是自举结束多花 1 拍（可忽略）。
//★ 根治的判据是【握手】（这一笔交付有没有被消费），不是地址 —— 用地址去重会压掉循环回边，
//  见 iffu 里那次教训。删交接不引入任何地址判据，所以循环/回边不受影响。
        else if (advance)
            instruction_valid <= read_enable & hit_0 & hit_1;
        else if (discard_output)
            instruction_valid <= 1'b0;
        else if (fifo_full)
            instruction_valid <= instruction_valid;
        else
            instruction_valid <= 1'b0;
    end

    always @(posedge clk) begin
        if (reset_registered)
            instruction_valid_next <= 1'b0;
        else if (advance)
            instruction_valid_next <= read_lane1;
        else if (discard_output)
            instruction_valid_next <= 1'b0;
        else if (fifo_full)
            instruction_valid_next <= instruction_valid_next;
        else
            instruction_valid_next <= 1'b0;
    end

//===============================================================
// 第 1 级：缺失锁延拓与取指输出
//===============================================================//缺失锁。它由状态自己说，不从 cache_hit 现组合出来：F 拍（fill_end）tag/valid 还没写进去，
//这时看 cache_hit 会把 busy 多留一拍，交付拍就落到 F+2。
//复位值仍然是 1（自举已删，这一条与它无关）：取 0 会让复位释放后第一拍 stage 落到 EXE、
//req_valid 抬起，icache 拿无效 hit_sel 去读 iram 并被 idu1 锁进流水线。
//代价是复位后多等 1 拍，之后 busy 只由 `stage`/`icache_miss` 决定。
    always @(posedge clk) begin
        if (reset_registered)
            busy <= 1'b1;
        else
            busy <= (stage == 1'b1)
                  | (icache_miss & ~fill_done);
    end

//★ 回填收尾的"刷新那一拍"：fill_end 打一拍。
//  为什么光有 fill_end 不够 —— 命中判定看的是 R0（tag Q 级），而 tag 是在 fill_end 那一沿
//  写进阵列的：同一沿上 R0 读回的是【写之前】的值。收尾之后必须再留一拍让 R0 把阵列重读一次
//  （只重读阵列那四位，地址保持 —— 见 Q 级那一段的说明）。
//  这一拍还要做两件事：
//    ① busy 不算（否则 pc 一直被按住、也进不了下一步）
//    ② FSM 的缺失起点不算（否则立刻拿旧有效位重开一次回填）
//  ★ 注意【不要】把 freeze 也一起掩掉：R0 里压着的就是缺失那一对，这一拍必须原样重放。
//  实测：自举只装 way0 的 16KB ⇒ 16KB 之外的第一条指令起，每 38 拍回填同一行、pc 冻死。
    always @(posedge clk) begin
        if (reset_registered) fill_done <= 1'b0;
        else       fill_done <= fill_end;
    end

//交付口：每拍要么是刚读回来的那一对，要么是 NOP；唯一保持的场合是队满（收不走就别换）。
//★ 原来的"jalr 类改向当拍刷成 NOP"那一档整段删掉：改向已由取指落点表提前一拍完成，
//  这一拍读到的就是目标那一对，没有错路要浇零；留着反而把它刷掉（实测：目标对丢失）。
    always @(posedge clk) begin
        if (reset_registered) begin
            instruction_out <= 32'd0;
            instruction_address_0 <= 32'd0;
            prediction_taken_0 <= 1'b0; prediction_taken_1 <= 1'b0;
            prediction_jump_target_0 <= 32'd0; prediction_jump_target_1 <= 32'd0;
            deliver_wrong_path <= 1'b0;
        end
        else if (//★ 只有"这一拍换 pc"才作废。曾经这里还挂着 jalr_fail|br2|br3（bju 的判定拍）：
//  判定拍不再改向之后，这三位照样把【已经读回来的】指令清零而没人叫 pc 重取
//  ⇒ 那条指令永久丢失（实测 CoreMark：落点 0x28 的 `jal x1` 被吞）。
                discard_output) begin
            instruction_out <= 32'd0;
            instruction_address_0 <= 32'd0;
            prediction_taken_0 <= 1'b0; prediction_taken_1 <= 1'b0;
            prediction_jump_target_0 <= 32'd0; prediction_jump_target_1 <= 32'd0;
            deliver_wrong_path <= 1'b0;
        end
        else if (advance) begin
            instruction_out <= iram[read_address];
            instruction_address_0 <= fetch_address_0;
//★ 预测就是这一对指令的延长位：同一个沿、同一个门、同一个地址（见端口那一段）。
//  pc 与 R0 吃同一个"这一拍前进吗" ⇒ pc_take 与口上这一对永远同拍，不需要任何补偿寄存器。
//★ 错路位 deliver_wrong_path 也在这一组里：它描述【口上这一对】，与指令同沿装载 ⇒ 天然带地址，
//  顶层的 pair_drop/pair_waddr 那套地址判据整个不需要。
            prediction_taken_0  <= pc_take_0;
            prediction_taken_1  <= pc_take_1;
            prediction_jump_target_0 <= pc_take_0 ? btb_target_0 : 32'd0;
            prediction_jump_target_1 <= pc_take_1 ? btb_target_1 : 32'd0;
            deliver_wrong_path      <= r0_wrong_path;
        end
//交付口的契约：每拍都放一样东西 —— 新指令，或者 NOP。唯一保持的场合是队满（收不走就别换）。
//"其余情况吐 NOP"这一条是必需的：pc 停住/后端停住那几拍如果原样摆着上一条真指令，
//下游按拍收就会把它重复收进去（实测同一条被分配 5 次）。
        else if (fifo_full) begin
            instruction_out <= instruction_out;
            instruction_address_0 <= instruction_address_0;
            prediction_taken_0 <= prediction_taken_0; prediction_taken_1 <= prediction_taken_1;
            prediction_jump_target_0 <= prediction_jump_target_0; prediction_jump_target_1 <= prediction_jump_target_1;
            deliver_wrong_path <= deliver_wrong_path;
        end
        else begin
            instruction_out <= 32'd0;
            instruction_address_0 <= 32'd0;
            prediction_taken_0 <= 1'b0; prediction_taken_1 <= 1'b0;
            prediction_jump_target_0 <= 32'd0; prediction_jump_target_1 <= 32'd0;
            deliver_wrong_path <= 1'b0;
        end
    end

//===============================================================
// 回填状态机（iram 口 B：回填写 / lane1 取指读 二选一）
//===============================================================
//iram 口 B 只有一根地址总线（port_b_address）读写时分复用：写那拍填 mem_data；不写那拍读 lane1 的字进 instruction_next。
//fill_way ∈ {0,1}：两路对称，没效优先，否则按 LRU（`lru[idx]` 就是"下次换哪一路"）。
//instruction_next 与 instruction_out 同三分支形状（读使能推进 / 冲刷浇零 / 停顿保持）。
    always @(posedge clk) begin
        if (write_enable)
            iram[port_b_address] <= mem_data;
        else if (reset_registered) begin
            instruction_next <= 32'd0;
            instruction_address_1 <= 32'd0;
        end
        else if (discard_output) begin
            instruction_next <= 32'd0;
            instruction_address_1 <= 32'd0;
        end
        else if (advance) begin
            instruction_next <= iram[port_b_address];
            instruction_address_1 <= fetch_address_1;
        end
        else if (fifo_full) begin
            instruction_next <= instruction_next;
            instruction_address_1 <= instruction_address_1;
        end
        else begin
            instruction_next <= 32'd0;
            instruction_address_1 <= 32'd0;
        end
    end

//★ 自举序列已删除（2026-10-27）。这里只剩一个复位期标志 `reset_period_flag`，专供测试台的
//  `wait(!dut.u_icache.reset_period_flag)` 做起跑对齐（全仓 114 个 tb 这么写）；综合上与 reset_registered 等价。
    always @(posedge clk) begin
        if (reset_registered) reset_period_flag <= 1'b1;
        else       reset_period_flag <= 1'b0;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            stage <= 1'b0;
            fill_end <= 1'b0;
            fill_count <= 0;
            fill_addr <= 32'd0;
            fill_tag <= 0;
            fill_index <= 0;
            miss_word <= 0;
            lru <= 0;
//way1_tag/way2_tag 不复位：有效位就在它自己最高位里，上电由 initial 清成 0 ⇒ 不命中。
//带复位会让这 2x128x20 位全被综合成带复位的 FF（撑爆 slice 打包），去掉后能进分布式 RAM。
        end
        else begin
            if (stage == 1'b0) begin
                fill_end <= 1'b0;
                if (hit_way_0[0])
                    lru[index_registered_0] <= 1'b1;   // 用了 way0 ⇒ 下次该换 way1
                else if (hit_way_0[1])
                    lru[index_registered_0] <= 1'b0;
//缺谁补谁的行：pc0 命中 ⇒ 补 pc1 的行；否则补 pc0 的行（两条都缺时先补 pc0，pc1 下一拍再报一次）
//★ ~fill_done：收尾那一拍 R0 里还是旧有效位，不许据此重开一次回填。
                if (icache_miss && ~fill_done) begin
                    stage <= 1'b1;
                    miss_word <= hit_0 ? word_q1 : word_q0;
                    fill_count <= 0;
                    fill_addr <= hit_0 ? {fetch_tag_registered_1[IC_FOLD-1:0], index_registered_1, {IC_WORD_B+2{1'b0}}}
                                       : {fetch_tag_registered_0[IC_FOLD-1:0], index_registered_0, {IC_WORD_B+2{1'b0}}};
                    fill_tag <= hit_0 ? fetch_tag_registered_1 : fetch_tag_registered_0;
                    fill_index <= hit_0 ? index_registered_1 : index_registered_0;
                end
            end
            else begin
                if (mem_valid && !fill_end) begin
                    if (fill_count == IC_WORD_LAST)
                        fill_end <= 1'b1;
                    else
                        fill_count <= fill_count + 1'b1;
                end
                if (fill_end) begin
                    if (fill_way == 1'b0)
                        way1_tag[fill_index] <= {1'b1, fill_tag};
                    else
                        way2_tag[fill_index] <= {1'b1, fill_tag};
//填完这一路 ⇒ 下次该换另一路。
                    lru[fill_index] <= ~fill_way;
                    stage <= 1'b0;
                    fill_end <= 1'b0;
                end
            end
        end
    end

//===============================================================
// 参数自检（综合时整块丢弃；仿真时恒生效）
//===============================================================
//★★ 2026-11-01：扩容只是"改一个数"，那就必须让"改漏了"当场报错。
//  这里锁的是 4 条恒等式：只要其中一条不成立，文件里必有一处切片/声明位宽没跟着改
//  ——而那种错误不会让任何一条指令报错，只会静默读到别的组/别的行。
//  失败一律 $display + $finish（不依赖下游 grep）；成功只在 +icache_cfg 时打印，
//  免得给全核仿真日志添噪音。
// synthesis translate_off
    initial begin
        if ((IC_TAGW   != IC_TAG_B + 1) ||
            (IC_TAG_B  != IC_FOLD + 2) ||
            (IC_ADDR_B != 1 + IC_INDEX_B + IC_WORD_B) ||
            (IC_WORDS  != (2 << (IC_INDEX_B + IC_WORD_B))) ||
            (IC_WORD_B + IC_INDEX_B >= 32)) begin
            $display("ICACHE_CFG_FAIL: sets=%0d word_b=%0d tag_b=%0d tagw=%0d fold=%0d addr_b=%0d words=%0d",
                     IC_SETS, IC_WORD_B, IC_TAG_B, IC_TAGW, IC_FOLD, IC_ADDR_B, IC_WORDS);
            $finish;
        end
        else if ($test$plusargs("icache_cfg"))
            $display("ICACHE_CFG_OK: %0d sets x 2 ways x %0d words = %0d KB (tag %0d bits, fold %0d)",
                     IC_SETS, (1 << IC_WORD_B), (IC_WORDS * 4) / 1024, IC_TAG_B, IC_FOLD);
    end
// synthesis translate_on

endmodule
