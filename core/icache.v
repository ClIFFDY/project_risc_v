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
    input [31:0] pc_addr0, pc_addr1,
    input br2, br3, pre_jalr, btb_hit, jalr_fail, jal_in, br1_in, exc_irq, exc_irq_ret, exc_ecall,
    input [7:0] flag_bus,
//回填应答（接核内 itcm）
    input mem_valid,
    input [31:0] mem_data,
//本拍 pc 落到重定向目标（pc.v 的 redir_go，源名 flush_pc_redir）：挡掉"跳转前那次取指"的陈旧交付
    input flush_pc_redir,
//取指队列满（iffu 的 full）：本拍放不下，交付寄存器保持、不再推进
    input fifo_full,

//取指输出
    output reg [31:0] inst_out, inst_next,
//与两条 lane 的指令同拍交付的地址：与数据走同一级寄存，每条指令自带自己的地址，
//下游不再需要"拍数"或"基准+4"来推它是哪一条。
    output reg [31:0] inst_addr0, inst_addr1,

    output reg inst_valid,
    output reg inst_valid_n,
//busy = 本模块自己的停顿源（stall_icache_miss），也是给 idu1 的垃圾指令守卫。
//它是寄存器，不是组合输出：由 stage / fill_end / bts_run 自己说，不再从 cache_hit 现组合出来。
    output reg busy,
//缺失检测拍（组合，直接送 pc，不走 flag_bus）：
//busy 是寄存的、要到下一拍才起来，pc 若在检测拍照常推进就会走过被丢的那一对，
//回填完 fill_ok 又按"当前 pc"取数 ⇒ 补的是下一对，被丢的那一对永远补不回来。
//实测：16KB 外的循环，负载轻时每圈丢一对（16 个随机背压种子里 7 个多一次错路重定向）。
    output reg icache_miss,
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

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：**先把 flag_bus 各位还原成原名，再按名字做逻辑**
//（模块内不直接用位号）；本模块的取指推进由 req_valid（只含前端那几条）与回填状态自己把关，
//后端停顿不再进这一位 —— 它们由 iffu 吸收，队满时拿 fifo_full 停本级。
    reg flush_con_exc, flush_con_irq, flush_con_jump;
    reg stall_pc_redir, stall_icache_miss, exec;
    reg flush_w, req_valid, rd_go;
    always @(*) begin
        flush_con_exc     = flag_bus[7];
        flush_con_irq     = flag_bus[6];
        flush_con_jump    = flag_bus[5];
        exec              = flag_bus[4];
        stall_pc_redir    = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        flush_w   = flush_con_exc | flush_con_irq | flush_con_jump;
        req_valid = ~(flush_w | stall_pc_redir | stall_icache_miss | busy | fifo_full | ~exec);
        rd_go     = ~(flush_w | stall_pc_redir | fifo_full | ~exec | jal_in | br1_in | (pre_jalr & btb_hit));
    end

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）
    reg jalr;
    always @(*) jalr = pre_jalr & btb_hit;

//寄存器（按级分组）
//第 0 级：本拍组合
    (* max_fanout = 32 *) reg [31:0] fetch_addr_0, fetch_addr_1;
    (* max_fanout = 32 *) reg [18:0] tag_0, tag_1;
    (* max_fanout = 32 *) reg [6:0] idx_0, idx_1;
    (* max_fanout = 32 *) reg [4:0] word_0, word_1;
    reg [2:0] hit_way_0, hit_way_1;
    reg [1:0] hit_sel_0, hit_sel_1;
    reg hit_0, hit_1, lane1_same_line;
    (* max_fanout = 32 *) reg [13:0] rd_addr, rd_addr1;
    reg rd_en, fill_ok;
    reg cache_hit;
    reg rd_l1;
    reg wr_en;
    reg [13:0] wr_addr;
    reg [13:0] portb_addr;
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
//取指地址：**只取 pc_addr >>> 2**，不吃 jal/br1，也没有那个 32 位加法器。
//jalr 早就是"落点只送 pc"；现在 jal/br1 也一样 —— 它们的改向落点只进 pc，
//而"目标那条指令"由 bpu 里那块独立的 btb 直接交付给 idu1。
//这样一来，"译码器 + 加法器 → 取指地址 → tag 阵列读 → 命中选择 → iram 地址口"
//那条 13 级/14.4ns 的链整条消失。
    always @(*) begin
        fetch_addr_0 = pc_addr0 >>> 2;
        fetch_addr_1 = pc_addr1 >>> 2;
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

    always @(*) begin
        tag_0  = fetch_addr_0[31:12];
        idx_0  = fetch_addr_0[11:5];
        word_0 = fetch_addr_0[4:0];
        tag_1  = fetch_addr_1[31:12];
        idx_1  = fetch_addr_1[11:5];
        word_1 = fetch_addr_1[4:0];
//way0 是锁定路：自举把前 16KB 装进去，此后永不改写（替换只在 way1/way2 之间选），
//所以"查 tag0 阵列再比较"等价于"地址是否落在低 16KB"——常量比较即可，整个阵列连同 valid0 都能删。
//自举期间 tag==0 会误报命中，但那时 busy 被 bts_run 强制为 1、req_valid=0（inst_out 不更新），
//两个时钟块也都不走它，三处消费点都不用它。
        hit_way_0[0] = (tag_0 == 19'd0);
        hit_way_0[1] = valid1[idx_0] && (tag1[idx_0] == tag_0);
        hit_way_0[2] = valid2[idx_0] && (tag2[idx_0] == tag_0);
        hit_way_1[0] = (tag_1 == 19'd0);
        hit_way_1[1] = valid1[idx_1] && (tag1[idx_1] == tag_1);
        hit_way_1[2] = valid2[idx_1] && (tag2[idx_1] == tag_1);
        hit_0 = hit_way_0[0] | hit_way_0[1] | hit_way_0[2];
        hit_1 = hit_way_1[0] | hit_way_1[1] | hit_way_1[2];
//两条 lane 各配各的 tag：任意一条没命中就要回填，缺谁补谁的行
        cache_hit = hit_0 & hit_1;
        hit_sel_0 = hit_way_0[0] ? 2'd0 : (hit_way_0[1] ? 2'd1 : 2'd2);
        hit_sel_1 = hit_way_1[0] ? 2'd0 : (hit_way_1[1] ? 2'd1 : 2'd2);
        lane1_same_line = (fetch_addr_1[18:5] == fetch_addr_0[18:5]);
        mem_req = bts_run ? ~bts_end : (stage == 1'b1 && !fill_end);
        mem_addr = bts_run ? {bts_line, 7'd0} : (fill_addr + (fill_cnt << 2));
        mem_we = 1'b0;
        mem_wdata = 32'd0;
        mem_be = 4'd0;

//读口：两个 iram 口各一个地址、一个使能。
//回填旁路：fill_end 那一拍 tag/valid 还是旧值（要下个沿才写进阵列），只有 iram 里已经是新行 ⇒
//直接从 fill_way/fill_idx 读。缺谁补谁的行：
//  ① pc0 命中 ⇒ 补的是 pc1 的行：pc0 照常读自己的行，pc1 读刚填的行；
//  ② pc0 不命中 ⇒ 补的是 pc0 的行：pc0 读它；pc1 只有落在同一条行时才跟着读，
//     否则这一拍不交 lane1，下一拍由它自己报缺、再补一次。
        fill_ok = fill_end & ((~hit_0 & ((fetch_addr_0 >> 5) == (fill_addr >> 7)))
                            | ( hit_0 & ((fetch_addr_1 >> 5) == (fill_addr >> 7))));

        if (fill_ok) begin
            rd_en    = 1'b1;
            rd_addr  = (~hit_0) ? {fill_way, fill_idx, word_0} : {hit_sel_0, idx_0, word_0};
            rd_addr1 = (hit_0 | lane1_same_line) ? {fill_way, fill_idx, word_1}
                                                 : {hit_sel_1, idx_1, word_1};
            rd_l1    = hit_0 | lane1_same_line;
        end
        else begin
            rd_en    = req_valid;
            rd_addr  = {hit_sel_0, idx_0, word_0};
            rd_addr1 = {hit_sel_1, idx_1, word_1};
            rd_l1    = rd_en & hit_1;
        end
    end

//口 B 地址二选一：回填/自举写，或 lane1 取指读。BRAM 一个口只有【一根地址总线】，读写是它的时分复用，
//两个来源的地址必须先显式合成这一根，再进阵列；否则综合器数出三个地址会判 Infeasible ram_style，
//把整个 48KB 阵列掉进 LUTRAM（且不中断流程）。写与 lane1 读不共拍（写 ⟹ stage==1/自举 ⟹ busy ⟹ 取指停）。
    always @(*) begin
        if (wr_en)
            portb_addr = wr_addr;
        else
            portb_addr = rd_addr1;
    end

//缺失检测拍（组合直送 pc，不走 flag_bus）：与 busy 的第三个加项逐字同门。
//这一拍 pc 必须停住，否则它走过被丢的那一对，回填完再也补不回来。
    always @(*) begin
        icache_miss = ~bts_run & (stage == 1'b0) & ~cache_hit & rd_go;
    end

//打一拍：它是给"此刻躺在 inst_out 里那条"用的。
//★ 冲刷在这里合并：冲刷拍交付的必是错路（inst_out 那边同一拍已被刷成 NOP）⇒ valid 同时拉低。
//  这样 flush 只出现在一个寄存器的 D 端，fetch_addr 就不必去吃 flag_bus 的全片广播
//  （那条"广播 → fetch_addr → tag 阵列读 → 命中选择 → rd_addr → iram 地址口"实测 23 级/14.49ns）。没发生读的时候 inst_out 不变，标志跟着保持。
//常态读要命中才有意义（没命中时 hit_sel 指向不存在的 way，读回来是未初始化值）；
//回填旁路读到的永远是"我欠的那个字"，但那次回填期间来过冲刷就不能认（标 invalid）。
//自举最后一行收满那一拍是"交接"：直接装入口指令并标有效，与回填路径再无关系。
    always @(posedge clk) begin
        if (rst_q)
            inst_valid <= 1'b0;
//★ 原来这里在自举收满那一拍做"交接"（rd_en 强行置 1 + inst_valid<=1），把 iram 里那条
//  直接当入口指令交出去。实测那是**重复交付**的唯一来源：交接在自举最后一拍就交付了入口
//  （pc=0），而 pc 又被 busy 多按一拍 ⇒ 下一拍 icache 按同一个地址 0 再读一次、再交付一次
//  ⇒ 入口那条进队列两次、写回两次（逐条写回比对的第一处不符恒落在入口 lui：RTL 有两条
//  x1=0x800000+）。交接整段删掉（rd_en 那一支与 inst_out 那一支），入口指令只由 c1 那拍
//  的正常读交付一次；代价是自举结束多花 1 拍（可忽略）。
//★ 根治的判据是【握手】（这一笔交付有没有被消费），不是地址 —— 用地址去重会压掉循环回边，
//  见 iffu 里那次教训。删交接不引入任何地址判据，所以循环/回边不受影响。
        else if (rd_en)
            inst_valid <= (fill_end ? ~fill_flushed : cache_hit) & ~flush_w;
        else if (fifo_full)
            inst_valid <= inst_valid;
        else
            inst_valid <= 1'b0;
    end

    always @(posedge clk) begin
        if (rst_q)
            inst_valid_n <= 1'b0;
        else if (rd_en)
            inst_valid_n <= rd_l1 & ~flush_w;
        else if (fifo_full)
            inst_valid_n <= inst_valid_n;
        else
            inst_valid_n <= 1'b0;
    end

    always @(posedge clk) begin
        if (rst_q)                         fill_flushed <= 1'b0;
        else if (stage == 1'b0)            fill_flushed <= flush_w | flush_pc_redir;
        else if (flush_w | flush_pc_redir) fill_flushed <= 1'b1;
    end

//===============================================================
// 第 1 级：缺失锁延拓与取指输出（含 jalr 类 NOP 门）
//===============================================================//缺失锁。它由状态自己说，不从 cache_hit 现组合出来：F 拍（fill_end）tag/valid 还没写进去，
//这时看 cache_hit 会把 busy 多留一拍，交付拍就落到 F+2，pc 与 inst_out 差 4。
//跨度：缺失检测拍的下沿起、到交付拍落下（F+1），与 pc 的 "缺 6 拍加 4、回填期按住" 对位，
//交付拍 pc_addr 恰为 miss 地址 +4，满足 inst_out(N)=instr(pc_addr(N)-4)。
//复位值必须是 1：复位期间 bts_run=1 ⇒ busy 恒 1，取 0 会让复位后第一拍 stage 落到 EXE、
//req_valid 抬起，icache 拿无效 hit_sel 去读 iram 并被 idu1 锁进流水线。
    always @(posedge clk) begin
        if (rst_q)
            busy <= 1'b1;
        else
            busy <= bts_run
                  | (stage == 1'b1 & ~fill_end)
                  | (stage == 1'b0 & ~cache_hit & rd_go);
    end

//jalr 类改向当拍把取指输出刷成 NOP：icache 不再当拍跳目标，当拍读出来的那条是
//跳转后的顺序指令（错路），必须挡掉；下一拍 pc 已在目标上，照常取到目标指令。
//【必须写在这个触发器的 D 端，不能挪到"喂给 idu1 的组合信号"上】：
//jalr_pred = pre_jalr & btb_hit，而 pre_jalr 是 idu1 从它的 inst_in 组合算出来的。
//写在 D 端是 inst_out(Q)→idu1→jalr_pred→inst_out(D)，触发器对触发器，合法；
//若在 cpu_top 里对 icache_inst_w 过门再喂 idu1，就闭成了
//inst_g→idu1→pre_jalr→jalr_pred→inst_g 的零延时组合环（xsim 实测 Iteration limit 10000）。
//与 req_valid 相与：停顿期间 req_valid=0，本来就没有新指令要挡，
//而 pc 的 jalr 分支也在 stage==EXE 才生效，两边同步。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out <= 32'd0;
            inst_addr0 <= 32'd0;
        end
        else if ((jalr_fail | br2 | br3 | exc_irq | exc_irq_ret | exc_ecall | flush_con_exc | flush_pc_redir) | ((jalr | jal_in | br1_in) && req_valid && exec && ~stall_pc_redir && ~stall_icache_miss && ~fifo_full)) begin
            inst_out <= 32'd0;
            inst_addr0 <= 32'd0;
        end
        else if (rd_en) begin
            inst_out <= iram[rd_addr];
            inst_addr0 <= pc_addr0;
        end
//交付口的契约：每拍都放一样东西 —— 新指令，或者 NOP。唯一保持的场合是队满（收不走就别换）。
//"其余情况吐 NOP"这一条是必需的：pc 停住/后端停住那几拍如果原样摆着上一条真指令，
//下游按拍收就会把它重复收进去（实测同一条被分配 5 次）。
        else if (fifo_full) begin
            inst_out <= inst_out;
            inst_addr0 <= inst_addr0;
        end
        else begin
            inst_out <= 32'd0;
            inst_addr0 <= 32'd0;
        end
    end

//===============================================================
// 自举与回填状态机（iram 口 B：回填/自举写 / lane1 取指读 二选一）
//===============================================================
//iram 口 B 只有一根地址总线（portb_addr）读写时分复用：写那拍填 mem_data；不写那拍读 lane1 的字进 inst_next。
//自举的 way 是常量 2'd0（way0 只有自举写得进去），回填的 fill_way 恒 ∈ {1,2} —— way0 的写通路物理取消。
//inst_next 与 inst_out 同三分支形状（读使能推进 / 冲刷浇零 / 停顿保持）。
    always @(posedge clk) begin
        if (wr_en)
            iram[portb_addr] <= mem_data;
        else if (rst_q) begin
            inst_next <= 32'd0;
            inst_addr1 <= 32'd0;
        end
        else if ((jalr_fail | br2 | br3 | exc_irq | exc_irq_ret | exc_ecall | flush_con_exc | flush_pc_redir) | ((jalr | jal_in | br1_in) && req_valid && exec && ~stall_pc_redir && ~stall_icache_miss && ~fifo_full)) begin
            inst_next <= 32'd0;
            inst_addr1 <= 32'd0;
        end
        else if (rd_en) begin
            inst_next <= iram[portb_addr];
            inst_addr1 <= pc_addr1;
        end
        else if (fifo_full) begin
            inst_next <= inst_next;
            inst_addr1 <= inst_addr1;
        end
        else begin
            inst_next <= 32'd0;
            inst_addr1 <= 32'd0;
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
                if (hit_way_0[1])
                    lru[idx_0] <= 1'b1;
                else if (hit_way_0[2])
                    lru[idx_0] <= 1'b0;
//缺谁补谁的行：pc0 命中 ⇒ 补 pc1 的行；否则补 pc0 的行（两条都缺时先补 pc0，pc1 下一拍再报一次）
                if (!cache_hit && rd_go) begin
                    stage <= 1'b1;
                    miss_word <= hit_0 ? word_1 : word_0;
                    fill_cnt <= 5'd0;
                    fill_addr <= (hit_0 ? {fetch_addr_1[31:5], 5'd0}
                                        : {fetch_addr_0[31:5], 5'd0}) << 2;
                    fill_tag <= hit_0 ? tag_1 : tag_0;
                    fill_idx <= hit_0 ? idx_1 : idx_0;
                    fill_way <= (!valid1[hit_0 ? idx_1 : idx_0]) ? 2'd1 :
                                (!valid2[hit_0 ? idx_1 : idx_0]) ? 2'd2 :
                                (lru[hit_0 ? idx_1 : idx_0] ? 2'd2 : 2'd1);
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
