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
// Description: 唯一取指源。48KB = 128 组 × 3 路 × 32 字。
//              内含取指地址生成（原 itcm）。
//              way0 存放自举装入的前 16KB 指令并锁定，不参与替换；
//              way1/way2 作后 48KB 的常规缓存，组内两路 LRU。
//              复位释放后 busy 持续拉高，自举从 itcm 顺序填充前 16KB，
//              填满后 busy 落下，功能相当于 itcm + icache + bootloader。
//
//              本文件内 always 块按【流水级数】排列：
//                第一级 取指地址 → 第二级 命中判定与缺失锁 → 第三级 取指输出
//                → 自举/回填状态机。
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
    input [31:0] pc_addr,
    input br2, br3, btb_hit, jalr_fail, exc_irq, exc_irq_ret, exc_ecall,
    input [11:0] flag_bus,
//回填应答（接核内 itcm）
    input mem_valid,
    input [31:0] mem_data,
//本拍 pc 落到重定向目标（pc.v 的 redir_go，源名 flush_pc_redir）：挡掉"跳转前那次取指"的陈旧交付
    input flush_pc_redir,
//取指队列满（fetch_fifo 的 full）：本级读使能被它按住，交付寄存器原地保持
    input fifo_full,
//jalr 的预跳转（fetch_fifo 前端就地译出的组合版）：jalr 类改向那一拍要把取指输出刷成 NOP。
//★ 这仍是"触发器经组合逻辑回到触发器"（inst_out(Q) → fifo 前端译码 → pre_jalr → inst_out(D)），
//  合法；当年在 cpu_top 里对 icache_inst_w 过门才会闭成零延时组合环。
    input pre_jalr,

//取指输出
    output reg [31:0] inst_out, inst_next,
//inst_valid / inst_valid_b = "inst_out / inst_next 里那条是可信的取指结果"，同沿同门控。
//两种情况置 0：①这一条的取指没命中（读回来是未初始化值 / 板上垃圾）；
//②回填期间来过冲刷（这条回填已经作废，交付拍送出的字不是 pc 现在要的那条）。
//给读侧当逐指令守卫：为 0 的那条进队列时被压成 NOP。
//必须是寄存器：终点是 inst_out 的下游，与取指同拍比较会把 tag 读→比较那条链引出去。
    output reg inst_valid, inst_valid_b,
//busy = 本模块自己的停顿源（stall_icache_miss）。
//★ 两个字【任一】没命中都要报：pc 现在恒定 +8（一次吃两个字），少认一个就是永久丢一条指令。
//它是寄存器，不是组合输出：由 stage / fill_end / bts_run 自己说。
    output reg busy,
//回填请求（接核内 itcm）
    output reg mem_req, mem_we,
    output reg [31:0] mem_addr, mem_wdata,
    output reg [3:0] mem_be
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam BOOT_LINES = 7'd127;

//★ 本模块【不再译码】。原在这里的"三态选字 + jal/br_en/offset 预译码 + 每字三位小表"整块
//  已搬走：预译码去 fetch_fifo 的前端，小表直接删掉。取指侧现在只剩"取两个字、报两个字可不可信"。

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：**先把 flag_bus 各位还原成原名，再按名字做逻辑**
//（模块内不直接用位号）。
//★ 本级在【取指侧】：req_valid 只吃"重定向排队 + 取指自己 miss + 队列满"三条 ——
//  rob_full / lsu / mulu / bus_hold 这些后端停顿不吃，它们由取指队列吸收
//  （后端停顿时本级继续取、继续往队里灌；灌满了由 fifo_full 按住本级）。
    reg flush_con_exc, flush_con_irq, flush_con_jump;
    reg stall_rob_full, stall_pc_redir;
    reg stall_lsu_haz, stall_lsu_full;
    reg stall_mulu_haz, stall_mulu_div;
    reg stall_icache_miss, stall_bus_hold;
    reg flush_w, req_valid;
    always @(*) begin
        flush_con_exc     = flag_bus[11];
        flush_con_irq     = flag_bus[10];
        flush_con_jump    = flag_bus[9];
        stall_rob_full    = flag_bus[7];
        stall_pc_redir    = flag_bus[6];
        stall_lsu_haz     = flag_bus[5];
        stall_lsu_full    = flag_bus[4];
        stall_mulu_haz    = flag_bus[3];
        stall_mulu_div    = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        stall_bus_hold    = flag_bus[0];
        flush_w   = flush_con_exc | flush_con_irq | flush_con_jump;
//★ 必须把 fifo_full 也算进来：pc 的推进门是 `!flush && !stall && !fifo_full` —— 队满时
//  取指侧【整体冻住】（pc 不动、交付寄存器保持、队列不写）。这一位要是漏了 fifo_full，
//  队满那一拍下面那条 `(jalr && req_valid)` 清零支照样把 inst_out 抹掉，而 pc 被按住没跳走
//  ⇒ 那条 ret 凭空消失、pc 顺着往下冲（实测 ret_st2 / ret_raw4 整个程序重跑）。
//  改成"与 pc 同一个门"之后，三处消费者（rd_en 与两条清零支）自动同相。
        req_valid = ~(stall_pc_redir | stall_icache_miss | fifo_full);
    end

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）。
//★ jalr 的预译码已搬到 fetch_fifo 的前端（那儿才是取指侧组合译码的新家），本模块只收它的一根
//  组合结果 `pre_jalr`，再与自己这儿的 btb_hit 相与 —— 用来把 jalr 改向那一拍的取指输出刷成 NOP。
//  这样 "inst_out(Q) → fifo 前端 → pre_jalr → inst_out(D)" 仍是触发器经组合逻辑回触发器，合法。
    reg jalr;
    always @(*) jalr = pre_jalr & btb_hit;


//寄存器（按级分组）
//第 0 级：本拍组合
    (* max_fanout = 32 *) reg [31:0] fetch_addr;
    reg cache_hit;
    (* max_fanout = 32 *) reg [18:0] tag;
    (* max_fanout = 32 *) reg [6:0] idx;
    (* max_fanout = 32 *) reg [4:0] word;
    reg [2:0] hit_way;
    reg [1:0] hit_sel;
    (* max_fanout = 32 *) reg [13:0] rd_addr;
    reg [13:0] rd_addr1;
    reg rd_en, fill_ok;
    reg wr_en;
    reg [13:0] wr_addr, portb_addr;
//lane1（第二个字）的取指地址与命中判定：与 lane0 完全独立的一套。
//★ 为什么必须独立：pc 现在【恒定 +8】，一次吃两个字；而两个字只有在 lane0 那一行是行末字
//  （word==31）时才落在不同行。同一行时两套判定给出相同结果（冗余但无害），跨行时只有独立
//  判定才能把 lane1 的 way 选对 —— 老写法 `{hit_sel, idx, word+1}` 里 word+1 的进位被 5 位宽
//  截掉，word==31 会读成同一行的第一个字。那是 pc 还能退化成 +4 时的退路，现在没有了。
//★ 地址由本模块内部派生（fetch_addr + 1 个字），对外仍然只有 pc_addr 一个地址输入。
    (* max_fanout = 32 *) reg [31:0] fetch_addr_b;
    (* max_fanout = 32 *) reg [18:0] tag_b;
    (* max_fanout = 32 *) reg [6:0]  idx_b;
    (* max_fanout = 32 *) reg [4:0]  word_b;
    reg [2:0] hit_way_b;
    reg [1:0] hit_sel_b;
    reg cache_hit_b;
//这一拍两个字都命中 / 缺的是 lane1（lane0 命中）；以及"要填的那一行"属于哪个字
    reg both_hit, miss_b, cross_row;
    reg [18:0] miss_tag;
    reg [6:0]  miss_idx;
    reg [4:0]  miss_word_c;
//本次回填期间来过冲刷：起回填的那一拍（stage==0）清成当拍有没有冲刷，回填期间来一个就置起来。
//回填期 pc 被 stall 按住，唯一能让它走的就是冲刷（flush_w / flush_pc_redir），
//所以这一位就等价于"这次回填已经不是 pc 现在要的那条了"。
    reg fill_flushed;

//自举/回填：阵列与状态机
    reg [18:0] tag1 [0:127];
    reg [18:0] tag2 [0:127];
    (* ram_style = "block" *) reg [31:0] iram [0:12287];
    reg [127:0] valid1, valid2, lru;

//自举（一次性的 bootloader）自成一套信号，不借回填那一群：
//bts_line/bts_cnt 只用来给 iram 编址与数够 32 个字，bts_end 是"本行收满"的一拍脉冲。
    reg bts_run;
    reg [6:0] bts_line;
    reg [4:0] bts_cnt;
    reg bts_end;

    reg stage;
    reg fill_end;
    reg [4:0] miss_word;
    reg [4:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [18:0] fill_tag;
    reg [6:0] fill_idx;
    reg [1:0] fill_way;

    integer i;

//===============================================================
// 第 0 级：本拍组合（取指地址 + 命中判定与缺失锁）
//===============================================================
//取指地址：**只有 pc_addr >>> 2**。本模块对外只有一个地址输入（pc 的寄存输出），
//lane1 的地址是它内部 +1 个字派生的。不吃 jal/br1，也没有那个 32 位加法器。
//jalr 早就是"落点只送 pc"；现在 jal/br1 也一样 —— 它们的改向落点只进 pc，
//而"目标那条指令"由 bra_predict 里那块独立的 bti 直接交付给队列前端。
//这样一来，"译码器 + 加法器 → 取指地址 → tag 阵列读 → 命中选择 → iram 地址口"
//那条 13 级/14.4ns 的链整条消失。
//★ 那个 +1 写成"字地址 +1"而不是"字节地址 +4 再右移 2"：两者恒等
//  （floor((x+4)/4) == floor(x/4)+1 对任意 x 成立），但前者进位链短。
//★ 曾经这里还有"成对那一拍额外 +4"一支，且它由 icache 自己算出的 pair_v 驱动 ⇒
//  `iram 出指令 → 类判据 → fetch_addr → tag → 命中选择 → iram 地址口` 闭在一拍内
//  （最差 16 级/15.0ns）。现在 pc 恒定 +8、成对判据整块搬到队列读侧，这一支连同那条环
//  一起没有了。
    always @(*) begin
        if (rst_q) begin
            fetch_addr   = 32'd0;
            fetch_addr_b = 32'd0;
        end
        else begin
            fetch_addr   = pc_addr >>> 2;
            fetch_addr_b = (pc_addr >>> 2) + 32'd1;
        end
    end

//写口：自举与回填共用【同一条写语句、同一个地址表达式】。写成两条写语句（两个地址表达式），
//综合器会各推一个写口 ⇒ iram 变多写口 ⇒ 进不了 BRAM，会溶解成 12K×32 个触发器（实测综合 15 分钟以上）。
    always @(*) begin
        if (bts_run) begin
            wr_en   = ~bts_end & mem_valid;
            wr_addr = {2'd0, bts_line, bts_cnt};
        end
        else begin
            wr_en   = (stage == 1'b1) & ~fill_end & mem_valid;
            wr_addr = {fill_way, fill_idx, fill_cnt};
        end
    end

//两个字各切一份 tag/idx/word，各做一套命中判定（lane1 那套是本轮新加的）。
//way0 是锁定路：自举把前 16KB 装进去，此后永不改写（替换只在 way1/way2 之间选），
//所以"查 tag0 阵列再比较"等价于"地址是否落在低 16KB"——常量比较即可，整个阵列连同 valid0 都能删。
//自举期间 tag==0 会误报命中，但那时 busy 被 bts_run 强制为 1、req_valid=0（inst_out 不更新），
//两个时钟块也都不走它，三处消费点都不用它。
    always @(*) begin
        tag   = fetch_addr[31:12];
        idx   = fetch_addr[11:5];
        word  = fetch_addr[4:0];
        tag_b  = fetch_addr_b[31:12];
        idx_b  = fetch_addr_b[11:5];
        word_b = fetch_addr_b[4:0];
        hit_way[0] = (tag == 19'd0);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
        hit_way[2] = valid2[idx] && (tag2[idx] == tag);
        cache_hit = hit_way[0] || hit_way[1] || hit_way[2];
        hit_sel = hit_way[0] ? 2'd0 : (hit_way[1] ? 2'd1 : 2'd2);
//★ lane1 的命中与 way 选择【只有跨行那一种才真的要查第二份】：
//  两个字只有在 lane0 是行末字（word==31）时才落在不同行；同一行时 lane1 的 tag/valid/way
//  与 lane0 逐位相同 ⇒ 直接继承。被继承挡住的那一支不进关键路（实测不挡的话，
//  `pc → fetch_addr_b → tag1 阵列 → 命中 → fill_way` 是 18 级 / 14.0ns 的最差族）。
        hit_way_b[0] = (tag_b == 19'd0);
        hit_way_b[1] = valid1[idx_b] && (tag1[idx_b] == tag_b);
        hit_way_b[2] = valid2[idx_b] && (tag2[idx_b] == tag_b);
        cross_row   = (word == 5'd31);
        cache_hit_b = cross_row ? (hit_way_b[0] || hit_way_b[1] || hit_way_b[2]) : cache_hit;
        hit_sel_b   = cross_row ? (hit_way_b[0] ? 2'd0 : (hit_way_b[1] ? 2'd1 : 2'd2)) : hit_sel;
//要填的那一行：lane0 缺就填 lane0 的；lane0 命中而 still 要填，那只可能是跨行 ⇒ 填 idx+1 那一行。
//★ 这一支【只吃 lane0 的 cache_hit 与 word】，不去查 lane1 的 tag 阵列 —— 填行参数挂上
//  第二份查表就把整条路拖长了，而它本来只是个"下一行"。
        both_hit    = cache_hit & cache_hit_b;
        miss_b      = cache_hit & cross_row;
        miss_idx    = idx + {6'd0, miss_b};
        miss_tag    = tag + {18'd0, (miss_b & (idx == 7'd127))};
        miss_word_c = miss_b ? 5'd0 : word;
        mem_req = bts_run ? ~bts_end : (stage == 1'b1 && !fill_end);
        mem_addr = bts_run ? {bts_line, 7'd0} : (fill_addr + (fill_cnt << 2));
        mem_we = 1'b0;
        mem_wdata = 32'd0;
        mem_be = 4'd0;

//读口：把 hit 与 fill_end 两条路合成一个地址/一个使能/一个数据选择，
//这样每个 iram 只有一个读口，才推得出 BRAM
//回填旁路只在"这一拍要取的那个字，就是我欠的那个字（或它后面一个字）"时才走：
//fill_addr>>2 是刚填完那行的首字地址，miss_word 是该行内欠的字。
//缺在行末字（word 31）时取指地址已经进了下一行，靠后一项接住。
//回填期间 pc 被冲刷到别处时两项都不成立 ⇒ 不旁路（那一拍不发废弃的字）。
//★ 只在【两个字同一行】时才旁路（word != 31）：旁路那一拍 lane1 是靠 `miss_word+1` 从
//  这一行里取的，跨行时 lane1 根本不在这行；而跨行只有 lane0 行末那一种，直接走
//  "填完再正常重取"两条路（两个都缺时先填 lane0 那行，下一轮再填 lane1 那行）就对了。
        fill_ok = ((fetch_addr == (fill_addr >> 2) + miss_word)
                |  (fetch_addr == (fill_addr >> 2) + miss_word + 32'd1))
                & (word != 5'd31);

        if (fill_end & fill_ok) begin
            rd_en   = 1'b1;
            rd_addr = {fill_way, fill_idx, miss_word};
            rd_addr1 = {fill_way, fill_idx, miss_word + 5'd1};
        end
        else if (bts_run && (bts_line == BOOT_LINES) && bts_end) begin
            rd_en   = 1'b1;
            rd_addr = 14'd0;
            rd_addr1 = 14'd1;
        end
        else begin
            rd_en   = req_valid;
            rd_addr = {hit_sel, idx, word};
//★ lane1 用它【自己那一套】命中的 way：跨行时它和 lane0 可以落在不同的 way 上。
            rd_addr1 = {hit_sel_b, idx_b, word_b};
        end
    end

//打一拍：它是给"此刻躺在 inst_out 里那条"用的。
//★ 冲刷在这里合并：冲刷拍交付的必是错路（inst_out 那边同一拍已被刷成 NOP）⇒ valid 同时拉低。
//  这样 flush 只出现在一个寄存器的 D 端，fetch_addr 就不必去吃 flag_bus 的全片广播
//  （那条"广播 → fetch_addr → tag 阵列读 → 命中选择 → rd_addr → iram 地址口"实测 23 级/14.49ns）。没发生读的时候 inst_out 不变，标志跟着保持。
//常态读要命中才有意义（没命中时 hit_sel 指向不存在的 way，读回来是未初始化值）；
//回填旁路读到的永远是"我欠的那个字"，但那次回填期间来过冲刷就不能认（标 invalid）。
//自举最后一行收满那一拍是"交接"：直接装入口指令并标有效，与回填路径再无关系。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_valid   <= 1'b0;
            inst_valid_b <= 1'b0;
        end
        else if (bts_run && (bts_line == BOOT_LINES) && bts_end) begin
            inst_valid   <= 1'b1;
            inst_valid_b <= 1'b1;
        end
        else if (rd_en) begin
            inst_valid   <= (fill_end ? ~fill_flushed : cache_hit)   & ~flush_w;
            inst_valid_b <= (fill_end ? ~fill_flushed : cache_hit_b) & ~flush_w;
        end
        else begin
            inst_valid   <= inst_valid;
            inst_valid_b <= inst_valid_b;
        end
    end

    always @(posedge clk) begin
        if (rst_q)                         fill_flushed <= 1'b0;
        else if (stage == 1'b0)            fill_flushed <= flush_w | flush_pc_redir;
        else if (flush_w | flush_pc_redir) fill_flushed <= 1'b1;
    end

//===============================================================
// 第 1 级：缺失锁延拓与取指输出（含 jalr 类 NOP 门）
//===============================================================
//缺失锁。它由状态自己说，不从 cache_hit 现组合出来：F 拍（fill_end）tag/valid 还没写进去，
//这时看 cache_hit 会把 busy 多留一拍，交付拍就落到 F+2，pc 与 inst_out 差 4。
//跨度：缺失检测拍的下沿起、到交付拍落下（F+1），与 pc 的 "缺 6 拍加 4、回填期按住" 对位，
//交付拍 pc_addr 恰为 miss 地址 +4，满足 inst_out(N)=instr(pc_addr(N)-4)。
//复位值必须是 1：复位期间 bts_run=1 ⇒ busy 恒 1，取 0 会让复位后第一拍 stage 落到 EXE、
//req_valid 抬起，icache 拿无效 hit_sel 去读 iram 并被 pre_decoder 锁进流水线。
    always @(posedge clk) begin
        if (rst_q)
            busy <= 1'b1;
        else
            busy <= bts_run
                  | (stage == 1'b1 & ~fill_end)
                  | (stage == 1'b0 & ~both_hit);
    end

//jalr 类改向当拍把取指输出刷成 NOP：icache 不再当拍跳目标，当拍读出来的那条是
//跳转后的顺序指令（错路），必须挡掉；下一拍 pc 已在目标上，照常取到目标指令。
//【必须写在这个触发器的 D 端，不能挪到"喂给 fifo 前端的组合信号"上】：
//jalr = pre_jalr & btb_hit，而 pre_jalr 是 fifo 前端从 inst_out 组合算出来的。
//★ 成对判据（两条各是什么类、包内有没有 RAW）已整块移到队列读侧，这里不再有任何类判据 ——
//  取指地址也不再吃它，那条 `小表输出 → 类判据 → fetch_addr → tag → 命中选择 → iram 地址口`
//  的环整条消失。

//写在 D 端是 inst_out(Q)→pre_decoder→jalr_pred→inst_out(D)，触发器对触发器，合法；
//若在 cpu_top 里对 icache_inst_w 过门再喂 pre_decoder，就闭成了
//inst_g→pre_decoder→pre_jalr→jalr_pred→inst_g 的零延时组合环（xsim 实测 Iteration limit 10000）。
//与 req_valid 相与：停顿期间 req_valid=0，本来就没有新指令要挡，
//而 pc 的 jalr 分支也在 stage==EXE 才生效，两边同步。
    always @(posedge clk) begin
        if (rst_q)
            inst_out <= 32'd0;
        else if (bts_run && (bts_line == BOOT_LINES) && bts_end) begin
            inst_out <= iram[rd_addr];
        end
        else if ((jalr_fail | br2 | br3 | exc_irq | exc_irq_ret | exc_ecall | flush_con_exc | flush_pc_redir) | (jalr && req_valid)) begin
            inst_out <= 32'd0;
        end
        else if (rd_en) begin
            inst_out <= iram[rd_addr];
        end
        else begin
            inst_out <= inst_out;
        end
    end

//===============================================================
// 自举与回填状态机（iram 口 B：回填写 / lane1 取指读 二选一）
//===============================================================
//口 A 只读 lane0（inst_out 那条路，一行未动）；口 B 在"回填写"与"lane1 取指读"之间二选一。
//写 ⟹ stage==1 && !fill_end ⟹ busy ⟹ 取指停，所以两个来源在时序上不共拍，共用一个口是行为等价的。
//★ BRAM 的一个口只有【一根地址总线】，读写是这根线的时分复用，所以两个来源的地址必须先显式合成
//一根（portb_addr）。若直接写成 if (we) iram[写地址] <= d; else q <= iram[读地址];，综合器会数出
//三个地址、判 "Infeasible attribute ram_style"、把整个 48KB 阵列掉进 LUTRAM（实测 RAMB36 16→0、
//RAMD 位点 104→17000、LUT 976→11962），而且那条告警不中断流程。
//自举的 way 是常量 2'd0（way0 只有自举写得进去），回填的 fill_way 恒 ∈ {1,2} —— way0 的写通路物理取消。
    always @(*) begin
        if (wr_en)
            portb_addr = wr_addr;
        else
            portb_addr = rd_addr1;
    end

//★ inst_next 必须复位：iram 里没被写过的条目在仿真里读出来是 X，而 lane1 的字段是从它上面
//  就地切片的 ⇒ 不初始化的话 X 会经"rs 抽取 → 冒险比较"穿进停顿位，整核挂死（实测）。
//★★ 门控必须与 inst_out 那个块【逐字同形】：inst_out 只在 rd_en / 自举交接时换、其余保持，
//  而 inst_next 若只要"不写"就换，两者就会错开一次取指 —— 成对判据拿 inst_effective 与
//  inst_next 比较，错开一次就是把不相邻的两条当成一对，pc 一次前进 8 就把中间那条吃掉
//  （实测：gpio 闭环里一条 store 消失、PLIC 使能寄存器从没被写）。冲刷/挡拍那几支同样要镜像。
    always @(posedge clk) begin
        if (wr_en) begin
            iram[portb_addr] <= mem_data;
        end
        else if (rst_q) begin
            inst_next <= 32'd0;
        end
        else if (bts_run && (bts_line == BOOT_LINES) && bts_end) begin
            inst_next <= iram[portb_addr];
        end
        else if ((jalr_fail | br2 | br3 | exc_irq | exc_irq_ret | exc_ecall | flush_con_exc | flush_pc_redir) | (jalr && req_valid)) begin
            inst_next <= 32'd0;
        end
        else if (rd_en) begin
            inst_next <= iram[portb_addr];
        end
        else begin
            inst_next <= inst_next;
        end
    end

//自举序列：复位释放后逐行填充前 16KB（128 行 × 32 字），固定写 way0。
//它自成一套信号（bts_*），不碰回填那一群；填完最后一行那一拍把入口指令交接进 inst_out。
    always @(posedge clk) begin
        if (rst_q) begin
            bts_run <= 1'b1;
            bts_line <= 7'd0;
            bts_cnt <= 5'd0;
            bts_end <= 1'b0;
        end
        else if (bts_run) begin
            if (mem_valid && !bts_end) begin
                if (bts_cnt == 5'd31)
                    bts_end <= 1'b1;
                else
                    bts_cnt <= bts_cnt + 5'd1;
            end
            if (bts_end) begin
                bts_end <= 1'b0;
                bts_cnt <= 5'd0;
                if (bts_line == BOOT_LINES)
                    bts_run <= 1'b0;
                else
                    bts_line <= bts_line + 7'd1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            stage <= 1'b0;
            fill_end <= 1'b0;
            fill_cnt <= 5'd0;
            fill_way <= 2'd0;
            fill_addr <= 32'd0;
            fill_tag <= 19'd0;
            fill_idx <= 7'd0;
            miss_word <= 5'd0;
            valid1 <= 128'd0;
            valid2 <= 128'd0;
            lru <= 128'd0;
//tag1/tag2 不复位：命中由 valid 挡住，陈旧 tag 无害。
//带复位会让这 3x128x19 位全被综合成带复位的 FF（撑爆 slice 打包），
//去掉后能进分布式 RAM。valid 必须留复位 —— 它才是命中判据。
        end
        else if (!bts_run) begin
            if (stage == 1'b0) begin
                fill_end <= 1'b0;
                if (hit_way[1])
                    lru[idx] <= 1'b1;
                else if (hit_way[2])
                    lru[idx] <= 1'b0;
                if (!both_hit) begin
                    stage <= 1'b1;
                    miss_word <= miss_word_c;
                    fill_cnt <= 5'd0;
                    fill_addr <= {tag, miss_idx, 5'd0} << 2;
                    fill_tag <= miss_tag;
                    fill_idx <= miss_idx;
                    fill_way <= (!valid1[miss_idx]) ? 2'd1 :
                                (!valid2[miss_idx]) ? 2'd2 :
                                (lru[miss_idx] ? 2'd2 : 2'd1);
                end
            end
            else begin
                if (mem_valid && !fill_end) begin
                    if (fill_cnt == 5'd31)
                        fill_end <= 1'b1;
                    else
                        fill_cnt <= fill_cnt + 5'd1;
                end
                if (fill_end) begin
                    if (fill_way == 2'd1) begin
                        tag1[fill_idx] <= fill_tag;
                        valid1[fill_idx] <= 1'b1;
                    end
                    else begin
                        tag2[fill_idx] <= fill_tag;
                        valid2[fill_idx] <= 1'b1;
                    end
                    lru[fill_idx] <= (fill_way == 2'd1);
                    stage <= 1'b0;
                    fill_end <= 1'b0;
                end
            end
        end
    end

endmodule
