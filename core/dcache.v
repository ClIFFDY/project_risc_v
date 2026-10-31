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

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//寄存器（按级分组）
//第 0 级：本拍组合
    reg [20:0] tag;
    reg [7:0] index;
    reg [2:0] word;
    reg [1:0] hit_way;
    reg cache_hit;

    reg read_request, write_request, write_pending, write_drive, read_drive;
    reg write_update, hit_upd;
    reg lru_write_enable, lru_write_value;
    reg [7:0] lru_write_index;
//★ 2026-10-31：命中更新打一拍再写 LRU。
//  原来 cache_hit 是「读 tag 阵列 → 比较 → 当拍就驱动 LRU 写使能」，实测这一段 4.92 ns；
//  它前面「rob/head_pointer → dcache 地址 mux → tag 地址脚」已经吃掉 8.25 ns
//  ⇒ 合起来 13.17 ns > 12.5，是全核 WNS 的唯一来源（最差 20 条端点全是同一条）。
//  LRU 只是替换提示量，晚一拍写不改变任何架构行为；回填那一路仍走当拍。
    reg        lru_hit_upd_q, lru_hit_value_q;
    reg [7:0]  lru_hit_index_q;
    reg [10:0] read_address;

    reg read_enable;

    reg is_fill_write, ram_write_enable;
    reg [3:0] ram_byte_enable;
    reg [10:0] ram_write_address;
    reg        ram_write_way;

    reg [31:0] ram_write_data;
    reg [31:0] write_address, write_data;
    reg [3:0] write_byte_enable;

//第 1 级与自举/回填：阵列与状态机
//★ 2026-10-30：valid 折进 tag 字，和 icache 同构（icache.v:162-163、581 就是 `{1'b1, fill_tag}`）。
//  原来 way0_valid/way1_valid 是 `reg [255:0]` 宽向量 + 运行时下标读 ⇒ 综合成 256:1 的 LUT mux 树
//  （MUXF7+MUXF8 各两级），单独的 dcache 实现里它就是最差路径的前两级。
//  折进 tag 后，valid 和 tag 一起从同一块 LUTRAM 读出来、一起比较，多出来的开销只是比较宽度 21→22。
//  代价：tag 阵列必须显式清零（valid=0 编码成整个字为 0），不能再靠独立的 valid 向量兜底。
    (* ram_style = "distributed" *) reg [21:0] way0_tag [0:255];   // [21]=valid, [20:0]=tag
    (* ram_style = "distributed" *) reg [21:0] way1_tag [0:255];
//★ 2026-11-01 试过把 data 阵列按【路】拆成两块、两路并行读（way 只落到最后一级 mux），
//  目标是断掉「读 tag → 命中 → 再读 data」这条两条 BRAM 串联。**实测失败，已回退**：
//  拆开后 Vivado 推断不出 BRAM，落成 RAMD64E（分布式 RAM）⇒ 5925 个失败端点、WNS −3.278。
//  根因是「dout <= way ? mem1[ra] : mem0[ra]」把两个存储器读混在一个寄存器里，推断器不认。
//  结论：这条路要成立，必须给每块阵列各配一个输出寄存器再 mux，那等于每次命中多一拍。
    (* ram_style = "block" *) reg [31:0] data_way0 [0:2047];
    (* ram_style = "block" *) reg [31:0] data_way1 [0:2047];
    reg [31:0] dout0, dout1;
    reg        read_way_q;
    reg        dcache_sel_q;
//★ 2026-10-30：tag 阵列读地址单独复制一份并加 max_fanout。
//  一个 8 位 index 要扇到 88 个 RAMD64E（256 深 × 22 位 × 2 路）的地址脚，实测那条网
//  单是布线就 2.811 ns（扇出 189），把 dcache 的最差路径卡在 8.5 ns。
//  复制是功能安全的（只多几个驱动器，值完全相同）。icache 的 read_address 也用了同一招。
//  ★ 只给【阵列读】用；LRU 写、数据读地址仍用原来的 index，避免把复制带到别的锥上。
    (* max_fanout = 8 *) wire [7:0] tag_index = bus_address_in[10:3];

//lru 必须是阵列，不能是宽向量：`reg [255:0] lru` 用运行时下标读写，会被综合成 256 个散
//触发器 + 一棵 256:1 mux 树（实测 256 FDRE + 353 LUT6 + 42 MUXF7/F8，还带一条 fo=260 的
//译码网），而阵列才推得成 LUTRAM。
//★ 2026-10-30：valid 也照这个道理折进了 tag 字（见上），所以下面 initial 里必须把 tag 阵列清零 ——
//  valid=0 现在编码成"整个 tag 字为 0"，不再有独立的 valid 向量兜底。
    (* ram_style = "distributed" *) reg lru [0:255];

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
    reg [20:0] fill_tag;
    reg [7:0] fill_index;
//★ 2026-10-31：缺失判定整体打一拍。
//  原来「读 tag 阵列 → 比较 → 命中判定」当拍就要驱动 stage 与 fill_* 一整族的 CE，
//  实测 tag 读之后还要 4.86 ns 才到 fill_index 的 CE（末段是一条扇出 30 的 miss_word 网）。
//  现在：payload 用【便宜的使能】先锁进 _q 寄存器（只跟 read_request 走，不看命中），
//  缺失只记一位 miss_q，下一拍才真的进回填。代价：每次缺失多一拍。
    reg        miss_q;
    reg [7:0]  fill_index_q;
    reg [2:0]  fill_word_q;
    reg [31:0] fill_addr_q;
    reg [20:0] fill_tag_q;
    reg        fill_way_q;
    reg fill_way;

    reg busy_delayed;

    integer i;

    initial begin
        for (i = 0; i < 2048; i = i + 1) begin
            data_way0[i] = 32'd0;
            data_way1[i] = 32'd0;
        end
        for (i = 0; i < 256;  i = i + 1) begin
            lru[i]  = 1'b0;
            way0_tag[i] = 22'd0;    // 全 0 = valid=0
            way1_tag[i] = 22'd0;
        end
        valid_generation = 1'b1;
    end

//===============================================================
// 第 0 级：本拍组合（请求与命中判定 + 写口合成）
//===============================================================
    always @(*) begin
        tag = bus_address_in[31:11];
        index = bus_address_in[10:3];
        word = bus_address_in[2:0];
        hit_way[0] = (way0_tag[tag_index] == {valid_generation, tag});
        hit_way[1] = (way1_tag[tag_index] == {valid_generation, tag});
        cache_hit = hit_way[0] || hit_way[1];

//★ ~miss_q：缺失已经判出来、正等着下一拍进回填，这一拍不许再收同一个请求
        read_request = (stage == 1'b0) && !miss_q && bus_valid_in && (bus_address_in[31:24] == 8'd0) && (bus_address_in[23:13] == 11'h100) && !bus_write_enable_in;
        write_request = (stage == 1'b0) && !miss_q && bus_valid_in && (bus_address_in[31:24] == 8'd0) && (bus_address_in[23:13] == 11'h100) && bus_write_enable_in && !write_pending;
//store 命中即行内就地更新，一律【不失效】：整字写整字，子字写按 be 只改那几个字节。
//写穿照做，缓存与后端始终一致。原先子字 store 走失效——失效会连带丢掉同行其余 7 个字，
//下一个 lw/lb 立刻回填（实测 9983 次失效 / 10104 次缺失，1:1），这是缺失的全部来源。
        write_update = write_request && cache_hit;
//lru 收成【单写口】：命中改 index（stage==0），回填改 fill_index（fill_end 拍 stage 仍为 1）。
//read_request / write_request / fill_end 三者两两互斥，所以一个写地址就能覆盖原来散在三处的写，
//索引与值逐字保持原语义（命中时 way0 优先 ⇒ 值取 hit_way[0]）。
        hit_upd  = (read_request || write_request) && cache_hit;
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
//  「这次访问是不是我的窗口」整个交给【输出】判（dcache_sel_q，寄存一拍）。
//  道理：读到垃圾无所谓（消费者只认 load_ready），而"非本窗口输出 0"的互斥契约本来就成立在输出那一下。
//  ★ 上一次写成 `fill_end || bus_valid_in` 反而更差 —— 因为 bus_valid_in 里裹着 ROB 的冲刷链；
//    现在那条链已从 lsu 的发请求路径摘掉，所以这里可以彻底不看任何跨模块的组合量。
        read_enable  = 1'b1;
        read_address = fill_end ? {fill_index, miss_word} : {index, word};
        load_ready_next = (read_request & cache_hit) | fill_end;
    end

//写口：单写口 + 字节使能，三个写源合成【一个】写表达式，仍能推成 BRAM ——
//  ① 回填（stage==1，整字，be 恒 1111）
//  ② store 命中（stage==0，整字或子字，be 来自 lsu）
//互斥性：write_request 要求 stage==0，is_fill_write 要求 stage==1；read_enable 要求 fill_end 或
//  (read_request && hit)，而 read_request 要求 !bus_write_enable_in —— 三者两两不同拍，不会同拍读写同址。
//整字写与字节写【必须在同一个 if 里按 be 展开成 WEBA[3:0]】：这才是单写口；
//拆成两个写表达式才会变成"多个写操作"、才推不成 BRAM。
    always @(*) begin
        is_fill_write = (stage == 1'b1) && !fill_end && mem_valid;
        ram_write_enable     = is_fill_write || write_update;
        ram_byte_enable     = is_fill_write ? 4'b1111 : bus_byte_enable_in;
        if (is_fill_write) begin
            ram_write_address = {fill_index, fill_count};
            ram_write_way     = fill_way;
            ram_write_data = mem_data;
        end
        else begin
            ram_write_address = {index, word};
            ram_write_way     = hit_way[1];
            ram_write_data = bus_data_in;
        end
    end

//===============================================================
// 第 1 级：数据阵列写入与读数据输出
//===============================================================
    always @(posedge clk) begin
        if (ram_write_enable) begin
            if (ram_write_way) begin
                if (ram_byte_enable[0]) data_way1[ram_write_address][7:0]   <= ram_write_data[7:0];
                if (ram_byte_enable[1]) data_way1[ram_write_address][15:8]  <= ram_write_data[15:8];
                if (ram_byte_enable[2]) data_way1[ram_write_address][23:16] <= ram_write_data[23:16];
                if (ram_byte_enable[3]) data_way1[ram_write_address][31:24] <= ram_write_data[31:24];
            end
            else begin
                if (ram_byte_enable[0]) data_way0[ram_write_address][7:0]   <= ram_write_data[7:0];
                if (ram_byte_enable[1]) data_way0[ram_write_address][15:8]  <= ram_write_data[15:8];
                if (ram_byte_enable[2]) data_way0[ram_write_address][23:16] <= ram_write_data[23:16];
                if (ram_byte_enable[3]) data_way0[ram_write_address][31:24] <= ram_write_data[31:24];
            end
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
    always @(posedge clk) begin
        if (reset_registered) begin
            dout0 <= 32'd0;
            dout1 <= 32'd0;
            read_way_q <= 1'b0;
            dcache_sel_q <= 1'b0;
        end
        else begin
            read_way_q <= fill_end ? fill_way : hit_way[1];
//“这次访问属于 dcache 窗口吗”寄存一拍 —— 只有输出那一拍需要它，所以它不在地址/使能链上
            dcache_sel_q <= fill_end || read_request;
//阵列恒读：地址直接灌地址线，使能恒 1
            dout0 <= data_way0[read_address];
            dout1 <= data_way1[read_address];
        end
    end
    assign bus_data_out = dcache_sel_q ? (read_way_q ? dout1 : dout0) : 32'd0;

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
            miss_q          <= 1'b0;
            fill_index_q    <= 8'd0;
            fill_word_q     <= 3'd0;
            fill_addr_q     <= 32'd0;
            fill_tag_q      <= 21'd0;
            fill_way_q      <= 1'b0;
        end
        else begin
            load_ready <= 1'b0;
//命中更新先锁一拍：把「读 tag → 比较」从 LRU 写使能那一拍里拿出来
            lru_hit_upd_q   <= hit_upd;
            lru_hit_index_q <= index;
            lru_hit_value_q <= hit_way[0];

            if (lru_write_enable)
                lru[lru_write_index] <= lru_write_value;

//便宜的口：只跟 read_request 走、不看命中判定 ⇒ CE 里没有 tag 读和比较。
//  回填要用的一整套（index/word/addr/tag/way）先在这里锁下来。
            if (read_request) begin
                fill_index_q <= index;
                fill_word_q  <= word;
                fill_addr_q  <= (bus_address_in & 32'hFFFFFFF8) << 2;
                fill_tag_q   <= tag;
                fill_way_q   <= ((way0_tag[tag_index][21] != valid_generation)) ? 1'b0 :
                                ((way1_tag[tag_index][21] != valid_generation)) ? 1'b1 : lru[index];
            end
            if (read_request & cache_hit)
                load_ready <= 1'b1;
//缺失只记一位，下一拍才真的进回填
            miss_q <= read_request & ~cache_hit;
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
    assign request_blocked = (stage != 1'b0) | write_pending;

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
