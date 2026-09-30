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
    input [31:0] offset_jal2, offset_beq2,
    input br1, br2, br3, jal, pre_jalr, btb_hit, jalr_fail, exc_irq, exc_irq_ret, exc_ecall,
    input [11:0] flag_bus,
//回填应答（接核内 itcm）
    input mem_valid,
    input [31:0] mem_data,
//本拍 pc 落到重定向目标（pc.v 的 redir_go，源名 flush_pc_redir）：挡掉"跳转前那次取指"的陈旧交付
    input flush_pc_redir,

//取指输出
    output reg [31:0] inst_out,
//inst_valid = "inst_out 里这条是可信的取指结果"，与 inst_out 同拍。
//两种情况置 0：①常态读没命中（hit_sel 指向不存在的 way，读回来是未初始化值 / 板上垃圾）；
//②回填期间来过冲刷（这条回填已经作废，交付拍送出的字不是 pc 现在要的那条）。
//给 pre_decoder 当逐指令守卫：为 0 时它算不出 jal/br_en、也不会把这条锁进流水线。
//必须是寄存器：终点是 inst_out 的下游，与取指同拍比较会把 tag 读→比较那条链引出去。
    output reg inst_valid,
//busy = 本模块自己的停顿源（stall_icache_miss），也是给 pre_decoder 的垃圾指令守卫。
//它是寄存器，不是组合输出：由 stage / fill_end / bts_run 自己说，不再从 cache_hit 现组合出来。
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

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：本模块的取指推进由八条 stall 位合出来的 req_valid
//与回填状态自己把关，不用流水线使能，故只取冲刷位与 stall 位（这里是消费者，或运算在本模块内做）。
    reg flush_w, req_valid;
    always @(*) begin
        flush_w   = flag_bus[11] | flag_bus[10] | flag_bus[9];
        req_valid = ~(flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4] | flag_bus[3]
                    | flag_bus[2] | flag_bus[1] | flag_bus[0]);
    end

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）
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
    reg rd_en, fill_ok;
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
//取指地址：默认 pc_addr >>> 2；jal / 预测成立的分支当拍直接用目标，省掉那一拍取指空泡。
//pc 侧用未减 4 的 offset（落 T+4），本侧用 pc_addr + offset − 4（取 T），两边按同一目标对齐。
//jalr 不加：它的目标是 BTB 查表来的，只送 pc。
    always @(*) begin
        if (rst_q)
            fetch_addr = 32'd0;
        else if (jal && !flush_w)
            fetch_addr = (pc_addr + offset_jal2 - 32'd4) >>> 2;
        else if (br1 && !flush_w)
            fetch_addr = (pc_addr + offset_beq2 - 32'd4) >>> 2;
        else
            fetch_addr = pc_addr >>> 2;
    end

    always @(*) begin
        tag = fetch_addr[31:12];
        idx = fetch_addr[11:5];
        word = fetch_addr[4:0];
//way0 是锁定路：自举把前 16KB 装进去，此后永不改写（替换只在 way1/way2 之间选），
//所以"查 tag0 阵列再比较"等价于"地址是否落在低 16KB"——常量比较即可，整个阵列连同 valid0 都能删。
//自举期间 tag==0 会误报命中，但那时 busy 被 bts_run 强制为 1、req_valid=0（inst_out 不更新），
//两个时钟块也都不走它，三处消费点都不用它。
        hit_way[0] = (tag == 19'd0);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
        hit_way[2] = valid2[idx] && (tag2[idx] == tag);
        cache_hit = hit_way[0] || hit_way[1] || hit_way[2];
        hit_sel = hit_way[0] ? 2'd0 : (hit_way[1] ? 2'd1 : 2'd2);
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
        fill_ok = (fetch_addr == (fill_addr >> 2) + miss_word)
                | (fetch_addr == (fill_addr >> 2) + miss_word + 32'd1);

        if (fill_end & fill_ok) begin
            rd_en   = 1'b1;
            rd_addr = {fill_way, fill_idx, miss_word};
        end
        else begin
            rd_en   = req_valid;
            rd_addr = {hit_sel, idx, word};
        end
    end

//打一拍：它是给"此刻躺在 inst_out 里那条"用的。没发生读的时候 inst_out 不变，标志跟着保持。
//常态读要命中才有意义（没命中时 hit_sel 指向不存在的 way，读回来是未初始化值）；
//回填旁路读到的永远是"我欠的那个字"，但那次回填期间来过冲刷就不能认（标 invalid）。
//自举最后一行收满那一拍是"交接"：直接装入口指令并标有效，与回填路径再无关系。
    always @(posedge clk) begin
        if (rst_q)
            inst_valid <= 1'b0;
        else if (bts_run && (bts_line == BOOT_LINES) && bts_end)
            inst_valid <= 1'b1;
        else if (rd_en)
            inst_valid <= fill_end ? ~fill_flushed : cache_hit;
        else
            inst_valid <= inst_valid;
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
                  | (stage == 1'b0 & ~cache_hit);
    end

//jalr 类改向当拍把取指输出刷成 NOP：icache 不再当拍跳目标，当拍读出来的那条是
//跳转后的顺序指令（错路），必须挡掉；下一拍 pc 已在目标上，照常取到目标指令。
//【必须写在这个触发器的 D 端，不能挪到"喂给 pre_decoder 的组合信号"上】：
//jalr_pred = pre_jalr & btb_hit，而 pre_jalr 是 pre_decoder 从它的 inst_in 组合算出来的。
//写在 D 端是 inst_out(Q)→pre_decoder→jalr_pred→inst_out(D)，触发器对触发器，合法；
//若在 cpu_top 里对 icache_inst_w 过门再喂 pre_decoder，就闭成了
//inst_g→pre_decoder→pre_jalr→jalr_pred→inst_g 的零延时组合环（xsim 实测 Iteration limit 10000）。
//与 req_valid 相与：停顿期间 req_valid=0，本来就没有新指令要挡，
//而 pc 的 jalr 分支也在 stage==EXE 才生效，两边同步。
    always @(posedge clk) begin
        if (rst_q)
            inst_out <= 32'd0;
        else if (bts_run && (bts_line == BOOT_LINES) && bts_end)
            inst_out <= iram[0];
        else if ((jalr_fail | br2 | br3 | exc_irq | exc_irq_ret | exc_ecall | flag_bus[11] | flush_pc_redir) | (jalr && req_valid))
            inst_out <= 32'd0;
        else if (rd_en)
            inst_out <= iram[rd_addr];
    end

//===============================================================
// 自举与回填状态机（iram 写口）
//===============================================================
//iram 单写口：地址在自举与回填之间选。
//自举的 way 是常量 2'd0（way0 只有自举写得进去），回填的 fill_way 恒 ∈ {1,2} —— way0 的写通路物理取消。
    always @(posedge clk) begin
        if (bts_run) begin
            if (!bts_end && mem_valid)
                iram[{2'd0, bts_line, bts_cnt}] <= mem_data;
        end
        else if (stage == 1'b1 && !fill_end && mem_valid) begin
            iram[{fill_way, fill_idx, fill_cnt}] <= mem_data;
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
                if (!cache_hit) begin
                    stage <= 1'b1;
                    miss_word <= word;
                    fill_cnt <= 5'd0;
                    fill_addr <= {fetch_addr[31:5], 5'd0} << 2;
                    fill_tag <= tag;
                    fill_idx <= idx;
                    fill_way <= (!valid1[idx]) ? 2'd1 :
                                (!valid2[idx]) ? 2'd2 :
                                (lru[idx] ? 2'd2 : 2'd1);
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
