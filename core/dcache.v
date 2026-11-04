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
    input clk, rst,
//总线接口
    input [31:0] bus_address_in, bus_data_in,
    input [3:0] bus_byte_enable_in,
    input bus_write_enable_in,
//★★ 2026-10-31：本模块现在有【自己的内部总线】（lsu 的 dcache_* 口），不再和 AHB master 共用一组线。
//  补上 valid 之后，请求判据是「valid 且是本段地址」，而不是「地址非 0」——
//  单靠地址译码要求上游"轮空就把地址清零"，那条清零又反过来把"发给谁"的仲裁塞进了地址路径。
    input bus_valid_in,
//后端存储接口
    input [31:0] mem_data,
    input mem_valid, mem_ready,

//总线接口
    output [31:0] bus_data_out,
    output reg load_ready, busy, hold, load_ready_next,
//★★ 2026-10-31：给 LSU 的"这一拍能不能收新请求"——只由【寄存器】决定（stage / write_pending），
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

    reg read_request, write_request, write_pending, write_drive, read_drive;
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
    reg way0_write_enable, way1_write_enable;
    reg [3:0] ram_byte_enable;
    reg [DC_SETS_B+2:0] ram_write_address;

    reg [31:0] ram_write_data;
    reg [31:0] write_address, write_data;
    reg [3:0] write_byte_enable;

//★★ 2026-11-04：命中判定【整体推后一拍】，store 与 LRU 共用这一份寄存量。
//  原路径「stage1_base → LSU 加法/译码 → tag 阵列 → 22 位比较 → data_way*/ENARDEN」实测 13.62 ns
//  （logic 5.29 / route 8.33，20 级），200 条最差端点里的 8 条真深路径全是这一条。
//  现在：请求那一拍只锁载荷 —— 使能只看 read_request/write_request（不含命中判定），
//  顺带把那一拍 LUTRAM 已经读出的两个 tag 字捉下来；下一拍直接用【寄存器】比出命中。
//  store 那一路据此给两路各发一个写使能（way 不再出现在使能上，原来的 ram_write_way = hit_way[1]
//  也是同一条链）；LRU 那一路据此更新。
//  ★ 试过「tag 读地址在 store_pending 拍切到寄存器」——静态时序不认这个 mux，
//    bus → 地址 → LUTRAM → 比较 → ENARDEN 照样成立（200/200 端点仍是 ENARDEN，WNS −2.331）。
//  代价：一笔 store 占 dcache 口两拍（store_pending 期间 request_blocked），tag 读口只有一份。
    reg        store_pending;
    reg        access_valid_q;                       // 上一拍受理了一笔访问（读或写）
    reg [DC_SETS_B+2:0] access_address_q;
    reg [DC_TAG_B-1:0]  access_tag_q;
    reg [DC_TAGW-1:0]   access_tag_word0_q, access_tag_word1_q;
    reg [31:0] store_data_q;
    reg [3:0]  store_byte_enable_q;

//第 1 级与自举/回填：阵列与状态机
//★ 2026-10-30：valid 折进 tag 字，和 icache 同构（icache.v:162-163、581 就是 `{1'b1, fill_tag}`）。
//  原来 way0_valid/way1_valid 是 `reg [255:0]` 宽向量 + 运行时下标读 ⇒ 综合成 256:1 的 LUT mux 树
//  （MUXF7+MUXF8 各两级），单独的 dcache 实现里它就是最差路径的前两级。
//  折进 tag 后，valid 和 tag 一起从同一块 LUTRAM 读出来、一起比较，多出来的开销只是比较宽度 21→22。
//  代价：tag 阵列必须显式清零（valid=0 编码成整个字为 0），不能再靠独立的 valid 向量兜底。
    (* ram_style = "distributed" *) reg [DC_TAGW-1:0] way0_tag [0:DC_SETS-1];   // [21]=valid, [20:0]=tag
    (* ram_style = "distributed" *) reg [DC_TAGW-1:0] way1_tag [0:DC_SETS-1];
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
    reg access_hit_way0, access_hit_way1;

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
        hit_way[0] = (way0_tag[tag_index] == {valid_generation, tag});
        hit_way[1] = (way1_tag[tag_index] == {valid_generation, tag});
        cache_hit = hit_way[0] || hit_way[1];

//★ 缺失：上一拍那笔读 + 寄存过的命中 ⇒ 这一拍进回填（与原 miss_q 同相）
        miss_q = access_valid_q & access_read_q & ~(access_hit_way0 | access_hit_way1);
//★ ~miss_q：缺失已经判出来、这一拍就要进回填，不许再收同一个请求
        read_request = (stage == 1'b0) && !miss_q && bus_valid_in && (bus_address_in[31:24] == 8'd0) && (bus_address_in[23:13] == 11'h100) && !bus_write_enable_in;
        write_request = (stage == 1'b0) && !miss_q && bus_valid_in && (bus_address_in[31:24] == 8'd0) && (bus_address_in[23:13] == 11'h100) && bus_write_enable_in && !write_pending;
//store 命中即行内就地更新，一律【不失效】：整字写整字，子字写按 be 只改那几个字节。
//写穿照做，缓存与后端始终一致。原先子字 store 走失效——失效会连带丢掉同行其余 7 个字，
//下一个 lw/lb 立刻回填（实测 9983 次失效 / 10104 次缺失，1:1），这是缺失的全部来源。
//★ 2026-11-04：命中判定不再当拍合成阵列写使能，见 access_* 的说明。
//lru 收成【单写口】：命中改 index（stage==0），回填改 fill_index（fill_end 拍 stage 仍为 1）。
//read_request / write_request / fill_end 三者两两互斥，所以一个写地址就能覆盖原来散在三处的写，
//索引与值逐字保持原语义（命中时 way0 优先 ⇒ 值取 hit_way[0]）。
//★ hit_upd 也改成吃【寄存过的】命中：原来它挂在当拍的 tag 比较上，实测 WNS −0.312 的
//  200 条最差路径全指向 lru_hit_upd_q_reg/D（route 8.7 ns 占 69%），推后一拍就整段从 CLK 出发。
        hit_upd  = access_valid_q & (access_hit_way0 | access_hit_way1);
//命中更新取【打一拍】的那份；回填仍当拍（fill_end 本来就是这一拍末尾才定）。
//两者撞拍时填表优先 ⇒ 丢掉一次命中更新，提示量，无妨。
        lru_write_enable   = lru_hit_upd_q || fill_end;
        lru_write_index = fill_end ? fill_index : lru_hit_index_q;
        lru_write_value = fill_end ? ~fill_way : lru_hit_value_q;

        write_drive = write_pending || write_request;
        read_drive = (stage == 1'b1) && !fill_end;

        busy = ((stage == 1'b1 && !fill_end) || (read_request && !cache_hit) || miss_q ||
                        (write_pending && !mem_ready));
        mem_req = (write_drive || read_drive);
        mem_write_enable = write_drive;
        if (write_pending) begin
            mem_addr = write_address;
            mem_write_data = write_data;
            mem_byte_enable = write_byte_enable;
        end
        else begin
            if (read_drive)
                mem_addr = fill_addr + (fill_count << 2);
            else
                mem_addr = bus_address_in << 2;
            mem_write_data = bus_data_in;
            mem_byte_enable = bus_byte_enable_in;
        end

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

//写口：两路各一个写使能，回填与 store 写成【各自一个】表达式，仍能推成两块 BRAM ——
//  ① 回填（stage==1，整字，be 恒 1111）→ 由 fill_way（寄存器）二选一
//  ② store 命中（推后一拍，命中在 store_pending 那拍由寄存的 index/tag 比较得出）
//互斥性：store_pending 只能在 stage==0 拍被置起，而回填要求 stage==1，两者不同拍。
//整字写与字节写【必须在同一个 if 里按 be 展开成 WEBA[3:0]】：这才是单写口；
//拆成两个写表达式才会变成"多个写操作"、才推不成 BRAM。
    always @(*) begin
        is_fill_write = (stage == 1'b1) && !fill_end && mem_valid;
//命中判定：两个输入都是寄存器（请求拍捉下的 tag 字 + 请求拍的 tag），整段从 CLK 出发
        access_hit_way0 = (access_tag_word0_q == {valid_generation, access_tag_q});
        access_hit_way1 = (access_tag_word1_q == {valid_generation, access_tag_q});
        way0_write_enable = (is_fill_write & ~fill_way) | (store_pending & access_hit_way0);
        way1_write_enable = (is_fill_write &  fill_way) | (store_pending & access_hit_way1);
        ram_byte_enable     = is_fill_write ? 4'b1111 : store_byte_enable_q;
        if (is_fill_write) begin
            ram_write_address = {fill_index, fill_count};
            ram_write_data = mem_data;
        end
        else begin
            ram_write_address = access_address_q;
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
            write_pending <= 1'b0;
            load_ready <= 1'b0;
            valid_generation <= ~valid_generation;   // 取反一次 ⇒ 整张 tag 表当拍全部作废
            lru_hit_upd_q   <= 1'b0;
            lru_hit_value_q <= 1'b0;
            lru_hit_index_q <= 8'd0;
            access_read_q   <= 1'b0;
            fill_index_q    <= 8'd0;
            fill_word_q     <= 3'd0;
            fill_addr_q     <= 32'd0;
            fill_tag_q      <= 21'd0;
            fill_way_q      <= 1'b0;
            store_pending   <= 1'b0;
            access_valid_q  <= 1'b0;
            access_address_q <= 11'd0;
            access_tag_q    <= 21'd0;
            access_tag_word0_q <= {DC_TAGW{1'b0}};
            access_tag_word1_q <= {DC_TAGW{1'b0}};
            store_data_q    <= 32'd0;
            store_byte_enable_q <= 4'd0;
        end
        else begin
            load_ready <= 1'b0;
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
                fill_way_q   <= ((way0_tag[tag_index][DC_TAG_B] != valid_generation)) ? 1'b0 :
                                ((way1_tag[tag_index][DC_TAG_B] != valid_generation)) ? 1'b1 : lru[index];
            end
            if (read_request & cache_hit)
                load_ready <= 1'b1;
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

            if (write_request) begin
                if (!mem_ready) begin
                    write_pending <= 1'b1;
                    write_address <= bus_address_in << 2;
                    write_data <= bus_data_in;
                    write_byte_enable <= bus_byte_enable_in;
                end
            end
//★★ 命中判定推后一拍：这一拍只锁载荷 + 捉 tag 字。使能只看 read_request/write_request，
//   不回读命中判定；命中/way 留到下一拍用这里的寄存量算。
            store_pending  <= write_request;
            access_valid_q <= read_request | write_request;
            access_read_q  <= read_request;
            if (read_request | write_request) begin
                access_address_q   <= {index, word};
                access_tag_q       <= tag;
                access_tag_word0_q <= way0_tag[tag_index];
                access_tag_word1_q <= way1_tag[tag_index];
            end
            if (write_request) begin
                store_data_q        <= bus_data_in;
                store_byte_enable_q <= bus_byte_enable_in;
            end

            if (write_pending && mem_ready)
                write_pending <= 1'b0;

            if (stage == 1'b1 && !fill_end && mem_valid) begin
                if (fill_count == 4'd7)
                    fill_end <= 1'b1;
                else
                    fill_count <= fill_count + 3'd1;
            end

            if (fill_end) begin
                if (fill_way) begin
                    way1_tag[fill_index] <= {valid_generation, fill_tag};
                end
                else begin
                    way0_tag[fill_index] <= {valid_generation, fill_tag};
                end
                load_ready <= 1'b1;
                stage <= 1'b0;
                fill_end <= 1'b0;
            end
        end
    end

//★ request_blocked：纯状态量，见端口处的说明。写要额外等 write_pending（写回没发完不收新写）。
//  store_pending 那拍 tag 读口被推后的那笔占着，收新请求会把它的命中判定读坏 ⇒ 也挡一拍。
    assign request_blocked = (stage != 1'b0) | write_pending | store_pending;

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
