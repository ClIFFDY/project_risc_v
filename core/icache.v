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
//
//              ★★ 2026-11-08：取指改【16 字节块】。一次读一个 16B 对齐的块（4 条），
//                 交付口一拍出 4 条。三处结构变化：
//                 ① iram 从「32 位 × N，两个读地址（port A 取 word0、port B 取 word1）」
//                    改成「**四份 32 位阵列**（word0/1/2/3），同一个地址并行读」。
//                    总位数一点没变（宽度×4、深度÷4），但：
//                      · 一次读就出 4 条，只花一个地址；
//                      · port B 只剩回填写，不再被 lane1 取指占用 ⇒ 回填与命中取指不抢口。
//                 ② 命中判定从【两个字各比一次、两个都要中】变成【一次行级 tag 比较】。
//                    16B 块必然整块落在 128B 行内（32 字行、8 块/行），所以不存在"半个块"。
//                 ③ 交付口出 4 条指令 + 一个块内起始偏移 block_offset。
//                    偏移 >0 的拍（改向落在块中间）只有后 4-block_offset 条有效 ——
//                    比它老的几条永远不出去，这就是"字内老指令不出队"。
//
//              本文件内 always 块按【流水级数】排列：
//                第一级 取指地址 → 第二级 命中判定与缺失锁 → 第三级 取指输出
//                → 回填状态机。
//
//////////////////////////////////////////////////////////////////////////////////
module icache(
    input clk, rst,
//取指地址生成（原 itcm）。pc_address_0 = 【下一条要执行的指令】的地址（可能不是 16B 对齐）。
//  块基址与块内偏移都由它现算：{pc_address_0[31:4], 2'b00} 与 pc_address_0[3:2]。
    input [31:0] pc_address_0,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input        redir_frontend,     // 前端重定向（rob 判定拍出的那根）
    input        irq_kill,           // 中断受理（clint 自己判的）
//★ 落点表输出（bpu 已仲裁成 one hot）：take 时取指地址直接吃落点，覆盖 pc_address_0；
//  这几位同时随指令进交付寄存器，当"这一块被预测跳"的凭证。
    input        btb_take_0, btb_take_1,
    input [31:0] btb_target_0, btb_target_1,
//回填应答（接核内 itcm）
    input mem_valid,
    input [31:0] mem_data,
//取指队列满（iffu 的 full）：本拍放不下，交付寄存器保持、不再推进
    input fifo_full,
//取指输出：一拍一块 2 条
    output reg [31:0] instruction_0, instruction_1,
//与两条指令同拍交付的地址：与数据走同一级寄存，每条指令自带自己的地址，
//下游不再需要"拍数"或"基准+4"来推它是哪一条。
    output reg [31:0] instruction_address_0, instruction_address_1,
//★ 与这一块指令【同一个寄存器组、同一个装载门】交付的预测：
//   prediction_taken_*  = 这一条被预测跳（取指当时 pc 照做的那一位）
//   prediction_jump_target_* = 那一位对应的预测落点（没预测跳 ⇒ 0）
//  取指队列条目里的 br_pred / jalr_pred 就是这两位，原样存。
    output reg        prediction_taken_0, prediction_taken_1,
    output reg [31:0] prediction_jump_target_0, prediction_jump_target_1,
//每条自己的有效位 = 块有效 & (序号 >= block_offset)。第 0 条可能就是块内第 2/3 条。
    output reg instruction_valid_0, instruction_valid_1,
//块内起始偏移（随指令同沿交付）：改向落在块中间时，比它老的几条不许出去。
    output reg [1:0] block_offset,
//busy = 本模块自己的停顿源（stall_icache_miss），也是给 idu 的垃圾指令守卫。
//它是寄存器，不是组合输出：由 stage / fill_end / icache_miss 自己说，不再从 cache_hit 现组合出来。
    output reg busy,
//缺失检测拍（组合，直接送 pc，不走 flag_bus）：
//busy 是寄存的、要到下一拍才起来，pc 若在检测拍照常推进就会走过被丢的那一块，
//回填完 fill_ok 又按"当前 pc"取数 ⇒ 补的是下一块，被丢的那一块永远补不回来。
    output reg icache_miss,
//★ 这一块是错路（与指令同组同门的位）。iffu 的 slot_replay 直接吃它。
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
    localparam integer IC_WG_B   = IC_WORD_B - 2;                      // 块内字号（16B = 4 字）⇒ 块号位宽
//★ 2026-11-11 时序：半选位 block_offset[1] 必须进【地址】，不能留在数据端做 2:1 mux。
//  旧写法 instruction_0 <= block_offset_q[1] ? iram_word2[ra] : iram_word0[ra] 让四份阵列的
//  输出寄存器合不进 BRAM，综合判 Infeasible ram_style="block" 后静默退 LUTRAM
//  （4×2,688 个 LUT6，read_address 每 bit 扇出 8,193、造 256 份副本，整条 u_icache→u_icache 族）。
    localparam integer IC_HALF_B = 1;                                  // 8B 块在 16B 行内的半选位
    localparam integer IC_ADDR_B = 1 + IC_INDEX_B + IC_WG_B + IC_HALF_B;// {way, index, block_group, half}
    localparam integer IC_WORDS  = IC_SETS << (IC_WORD_B + 1);         // iram 字数 = 2 路 × 组数 × 行字数
    localparam integer IC_ROWS   = IC_WORDS >> 1;                      // 每份阵列的行数（每份装 8B 块的一侧）
    localparam [IC_WORD_B-1:0] IC_WORD_LAST = {IC_WORD_B{1'b1}};       // 行内末字号
//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;
//寄存器（按级分组）
//第 0 级：本拍组合
    (* max_fanout = 32 *) reg [31:0] fetch_word_index_0;
    (* max_fanout = 32 *) reg [IC_TAG_B-1:0] tag_0;
    (* max_fanout = 32 *) reg [IC_INDEX_B-1:0] index_0;
    (* max_fanout = 32 *) reg [IC_WORD_B-1:0] word_0;
    reg [1:0] hit_way_0;
    reg hit_0;
    (* max_fanout = 32 *) reg [IC_ADDR_B-1:0] read_address;
//第 1 级：由 Q 端字段拼回来的取指地址（pc = tag[IC_FOLD-1:0] & idx & word & 2'b00）
    reg [31:0] fetch_address_0;
//第 0R 级：tag 阵列 Q 端。整组同一个沿、同一套条件，一位都不能单独打拍。
    reg [IC_TAG_B-1:0] fetch_tag_registered_0;
    reg [IC_TAGW-1:0] way1_tag_registered_0, way2_tag_registered_0;
    reg [IC_INDEX_B-1:0]  index_registered_0;
    reg [IC_WORD_B-1:0]  word_q0;
    reg [1:0] block_offset_q;
//这一块算不算数：冲刷拍清 0，装载拍置 1，背压拍保持。
//  它不管命中、不管地址；只回答"R0 里这一块现在作不作数"（缺失判据与交付有效位都吃它）。
    reg r0_valid;
    reg write_enable;
    reg [IC_ADDR_B-1:0] write_address;
//阵列与状态机
//tag 阵列存 {valid, tag}：最高位就是有效位，命中判据退化成【一次 20 位比较】。
//原来 valid 是另一份 128 位向量、再和 tag 比完的结果相与，多一级 LUT、多一条 128:1 选择。
    (* ram_style = "distributed" *) reg [IC_TAGW-1:0] way1_tag [0:IC_SETS-1];
    (* ram_style = "distributed" *) reg [IC_TAGW-1:0] way2_tag [0:IC_SETS-1];
//★ 两份阵列，按【8B 块内的高/低字】分（不是按 16B 行内第几字）：
//  iram_word_low 装 word0/word2（pc[2]=0），iram_word_high 装 word1/word3（pc[2]=1）。
//  行里哪一半（pc[3]）进地址，于是读侧退化成 instruction_0 <= iram_word_low[read_address]，
//  一份阵列一个读口一个写口、数据端没有 mux —— 这才是 BRAM 能认的形状。
    (* ram_style = "block" *) reg [31:0] iram_word_low  [0:IC_ROWS-1];
    (* ram_style = "block" *) reg [31:0] iram_word_high [0:IC_ROWS-1];
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
    reg [IC_WORD_B-1:0] fill_count;
    reg [31:0] fill_addr;
    reg [IC_TAG_B-1:0] fill_tag;
    (* max_fanout = 32 *) reg [IC_INDEX_B-1:0] fill_index;
//★ 回填替换路：只依赖【寄存好的 fill_index】，不依赖缺失检测那一拍的任何东西。
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
//取指地址：块基址 = pc & ~15，然后 >>> 2。块内偏移不进地址（它只在交付口用）。
//★ 预测跳那一拍 pc 已经在口上送了落点（pc.v 里的 mux），本模块不再自己选。
    always @(*) begin
//★ 块基址的字索引 = (pc & ~15) >>> 2 = pc[31:4] << 2 = {pc[31:4], 2'b00}。
//  2026-11-08 踩过一次：多写了一个 >>> 2 ⇒ 得到的是【块号】不是字索引，
//  相邻四个块全落在同一条阵列地址上，读回来的是同一份 4 条指令（rand seed1 当场复现）。
        fetch_word_index_0 = {pc_address_0[31:4], 2'b00};
    end
//写口：回填写（每份阵列唯一一条写语句、唯一地址表达式）。
//★ 块内字号 = fill_count[1:0]：[0] 选高/低字阵列（决定"写哪一份"），[1] 选 16B 行的哪一半（进地址）。
    always @(*) begin
        write_enable   = (stage == 1'b1) & ~fill_end & mem_valid;
        write_address = {fill_way, fill_index, fill_count[IC_WORD_B-1:2], fill_count[1]};
    end
    always @(posedge clk) begin
        if (write_enable) begin
            if (fill_count[0]) iram_word_high[write_address] <= mem_data;
            else               iram_word_low [write_address] <= mem_data;
        end
    end
    always @(*) begin
        tag_0  = fetch_word_index_0[31:IC_WORD_B+IC_INDEX_B];
        index_0  = fetch_word_index_0[IC_WORD_B+IC_INDEX_B-1:IC_WORD_B];
        word_0 = fetch_word_index_0[IC_WORD_B-1:0];
        mem_req = (stage == 1'b1 && !fill_end);
        mem_address = fill_addr + (fill_count << 2);
        mem_write_enable = 1'b0;
        mem_write_data = 32'd0;
        mem_byte_enable = 4'd0;
    end
//第 0R 级：tag 阵列 Q 端打一拍。这一刀切在【阵列读之后、比较之前】。
    always @(posedge clk) begin
        if (reset_registered) begin
            fetch_tag_registered_0 <= 0;
            way1_tag_registered_0 <= 0;
            way2_tag_registered_0 <= 0;
            index_registered_0  <= 0;
            word_q0 <= 0;
            block_offset_q <= 2'd0;
            r0_valid <= 1'b0;
        end
        else if (irq_kill | redir_frontend) begin
// 冲刷：这一块不要了 —— 地址字段无所谓，作废就靠这一位。
            r0_valid <= 1'b0;
        end
        else if (fill_done) begin
//★ 唯一例外：tag 是在 fill_end 那一沿才写进阵列的，同一沿 R0 读回的是【写之前】的值
//  ⇒ 不重读就永远用陈旧有效位 ⇒ 同一行无限回填。
//  ★ 地址字段一个都不能换：R0 里压着的就是缺失那一块，重放要的是同一个 tag 再比一次。
            way1_tag_registered_0 <= way1_tag[index_registered_0];
            way2_tag_registered_0 <= way2_tag[index_registered_0];
            r0_valid <= r0_valid;
        end
        else if (~(icache_miss | fifo_full | busy)) begin
            fetch_tag_registered_0 <= tag_0;
            way1_tag_registered_0 <= way1_tag[index_0];
            way2_tag_registered_0 <= way2_tag[index_0];
            index_registered_0  <= index_0;
            word_q0 <= word_0;
//★ 2026-11-11 取指回 2 宽：块 = 8B。[1] 选 16B 行的哪一半，[0] 是块内偏移（交给 iffu）。
            block_offset_q <= pc_address_0[3:2];
            r0_valid <= 1'b1;
        end
        else begin
// 背压：地址不动，整组保持。
            fetch_tag_registered_0 <= fetch_tag_registered_0;
            way1_tag_registered_0 <= way1_tag_registered_0;
            way2_tag_registered_0 <= way2_tag_registered_0;
            index_registered_0  <= index_registered_0;
            word_q0 <= word_q0;
            r0_valid <= r0_valid;
        end
    end
//第 1 级：拼回取指地址、比较、选 way、出 iram 地址。两路对称：路号直接由命中项给出。
    always @(*) begin
        fetch_address_0 = {fetch_tag_registered_0[IC_FOLD-1:0], index_registered_0,
                           word_q0[IC_WORD_B-1:2], 4'd0};
        hit_way_0[0] = (way1_tag_registered_0 == {1'b1, fetch_tag_registered_0});
        hit_way_0[1] = (way2_tag_registered_0 == {1'b1, fetch_tag_registered_0});
//★ 16B 块恒在一条行内 ⇒ 命中判定只有一次（老版本是两个字各比一次、两个都要中）。
        hit_0 = hit_way_0[0] | hit_way_0[1];
// 命中路号直接进 iram 地址：0 = way0(way1_tag)，1 = way1(way2_tag)
//★ 半选位 block_offset_q[1]（= pc[3]）进地址：这决定读 16B 行的前半还是后半。
        read_address  = {hit_way_0[1], index_registered_0, word_q0[IC_WORD_B-1:2], block_offset_q[1]};
    end
//缺失检测拍（组合直送 pc，不走 flag_bus）：与 busy 的第三个加项逐字同门。
    always @(*) begin
        icache_miss = r0_valid & ~hit_0;
    end
//★ 冲刷之后那一拍交付出来的块是改向前那条流的残块（改向那一拍 R0 里还压着它），
//  标成错路交给 iffu 丢掉（rand seed1：0x2d0）。
    reg flush_bubble;
    always @(posedge clk) begin
        if (reset_registered)       flush_bubble <= 1'b0;
        else                        flush_bubble <= (irq_kill | redir_frontend);
    end
//★ 交付寄存器：四条指令 + 四个地址 + 四份预测 + 块内偏移，一个沿、一个门。
//  冲刷（irq_kill / redir_frontend）：整组清；
//  队满：整组保持；缺失/回填：出 NOP；其余：装这一块。
    always @(posedge clk) begin
        if (reset_registered) begin
            instruction_0 <= 32'd0; instruction_1 <= 32'd0;
            instruction_address_0 <= 32'd0; instruction_address_1 <= 32'd0;
            prediction_taken_0 <= 1'b0; prediction_taken_1 <= 1'b0;
            prediction_jump_target_0 <= 32'd0; prediction_jump_target_1 <= 32'd0;
            instruction_valid_0 <= 1'b0; instruction_valid_1 <= 1'b0;
            deliver_wrong_path <= 1'b0;
            block_offset <= 2'd0;
        end
        else if (irq_kill | redir_frontend) begin
            instruction_0 <= 32'd0; instruction_1 <= 32'd0;
            instruction_address_0 <= 32'd0; instruction_address_1 <= 32'd0;
            prediction_taken_0 <= 1'b0; prediction_taken_1 <= 1'b0;
            prediction_jump_target_0 <= 32'd0; prediction_jump_target_1 <= 32'd0;
            instruction_valid_0 <= 1'b0; instruction_valid_1 <= 1'b0;
            deliver_wrong_path <= 1'b0;
        end
        else if (~(icache_miss | fifo_full | busy)) begin
//★ 只交当前 8B 块的那两个字：半选已经压在 read_address 里（见上），这里就是两条裸读。
            instruction_0 <= iram_word_low[read_address];
            instruction_1 <= iram_word_high[read_address];
            instruction_address_0 <= fetch_address_0 + (block_offset_q[1] ? 32'd8 : 32'd0);
            instruction_address_1 <= fetch_address_0 + (block_offset_q[1] ? 32'd8 : 32'd0) + 32'd4;
//★ 预测就是这一块指令的延长位：同一个沿、同一个门、同一个地址。
            prediction_taken_0  <= btb_take_0;
            prediction_taken_1  <= btb_take_1;
            prediction_jump_target_0 <= btb_take_0 ? btb_target_0 : 32'd0;
            prediction_jump_target_1 <= btb_take_1 ? btb_target_1 : 32'd0;
//★ 块内起始偏移：比它老的几条永远不出去。
//  ★ 用 block_offset_q（随 R0 那一块锁进来的偏移），不是当拍的 pc_address_0 ——
//    当拍 pc 已经是下一块了，用它会错一整块。
            block_offset <= {1'b0, block_offset_q[0]};
            instruction_valid_0 <= r0_valid & hit_0 & (block_offset_q[0] == 1'b0);
            instruction_valid_1 <= r0_valid & hit_0;
            deliver_wrong_path <= flush_bubble;
        end
//交付口的契约：每拍都放一样东西 —— 新指令，或者 NOP。唯一保持的场合是队满（收不走就别换）。
        else if (fifo_full) begin
            instruction_0 <= instruction_0; instruction_1 <= instruction_1;
            instruction_address_0 <= instruction_address_0; instruction_address_1 <= instruction_address_1;
            prediction_taken_0 <= prediction_taken_0; prediction_taken_1 <= prediction_taken_1;
            prediction_jump_target_0 <= prediction_jump_target_0; prediction_jump_target_1 <= prediction_jump_target_1;
            instruction_valid_0 <= instruction_valid_0; instruction_valid_1 <= instruction_valid_1;
            deliver_wrong_path <= deliver_wrong_path;
            block_offset <= block_offset;
        end
        else begin
// 缺失 / 回填中：这一拍没有有效块，出 NOP。
            instruction_0 <= 32'd0; instruction_1 <= 32'd0;
            instruction_address_0 <= 32'd0; instruction_address_1 <= 32'd0;
            prediction_taken_0 <= 1'b0; prediction_taken_1 <= 1'b0;
            prediction_jump_target_0 <= 32'd0; prediction_jump_target_1 <= 32'd0;
            instruction_valid_0 <= 1'b0; instruction_valid_1 <= 1'b0;
            deliver_wrong_path <= 1'b0;
        end
    end
//===============================================================
// 回填状态机（四份阵列的 port B 全是回填写，不再与取指读抢口）
//===============================================================
//fill_way ∈ {0,1}：两路对称，没效优先，否则按 LRU（`lru[idx]` 就是"下次换哪一路"）。
    always @(posedge clk) begin
        if (reset_registered) reset_period_flag <= 1'b1;
        else reset_period_flag <= 1'b0;
    end
    always @(posedge clk) begin
        if (reset_registered) begin
            stage <= 1'b0;
            fill_end <= 1'b0;
            fill_count <= 0;
            fill_addr <= 32'd0;
            fill_tag <= 0;
            fill_index <= 0;
            lru <= 0;
        end
        else begin
            if (stage == 1'b0) begin
                fill_end <= 1'b0;
                if (hit_way_0[0])
                    lru[index_registered_0] <= 1'b1;   // 用了 way0 ⇒ 下次该换 way1
                else if (hit_way_0[1])
                    lru[index_registered_0] <= 1'b0;
//★ ~fill_done：收尾那一拍 R0 里还是旧有效位，不许据此重开一次回填。
                if (icache_miss && ~fill_done) begin
                    stage <= 1'b1;
                    fill_count <= 0;
                    fill_addr <= {fetch_tag_registered_0[IC_FOLD-1:0], index_registered_0, {IC_WORD_B+2{1'b0}}};
                    fill_tag <= fetch_tag_registered_0;
                    fill_index <= index_registered_0;
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
//★ 回填收尾之后的一拍（见上面 r0 重读）。
    always @(posedge clk) begin
        if (reset_registered) fill_done <= 1'b0;
        else       fill_done <= fill_end;
    end
    always @(posedge clk) begin
        if (reset_registered)
            busy <= 1'b1;
        else
            busy <= (stage == 1'b1)
                  | (icache_miss & ~fill_done);
    end
//===============================================================
// 参数自检（综合时整块丢弃；仿真时恒生效）
//===============================================================
// synthesis translate_off
    initial begin
        if ((IC_TAGW   != IC_TAG_B + 1) ||
            (IC_TAG_B  != IC_FOLD + 2) ||
            (IC_ADDR_B != 1 + IC_INDEX_B + IC_WG_B + IC_HALF_B) ||
            (IC_WORDS  != (2 << (IC_INDEX_B + IC_WORD_B))) ||
            (IC_ROWS   != (IC_WORDS >> 1)) ||
            (IC_WG_B   != IC_WORD_B - 2) ||
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
