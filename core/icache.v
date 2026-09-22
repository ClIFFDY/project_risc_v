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
    input [31:0] offset_jal1, offset_beq1,
    input [31:0] isr_addr1, isr_ret_addr1,
    input br1, br2, br3, jal, pre_jalr, btb_hit, jalr_fail, irq, irq_ret,
    input [4:0] flag_bus,
//读口控制
    input req_valid,
//回填应答（接核内 itcm）
    input mem_valid,
    input [31:0] mem_data,

//取指输出
    output reg [31:0] inst_out,
    output reg busy, busy_q,
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

    reg cache_hit;
    (* max_fanout = 32 *) reg [18:0] tag;
    (* max_fanout = 32 *) reg [6:0] idx;
    (* max_fanout = 32 *) reg [4:0] word;
    reg [2:0] hit_way;
    reg [1:0] hit_sel;

    reg [18:0] tag1 [0:127];
    reg [18:0] tag2 [0:127];
    (* ram_style = "block" *) reg [31:0] iram [0:12287];
    reg [127:0] valid1, valid2, lru;

    reg boot;
    reg [6:0] boot_line;

    reg stage;
    reg fill_end;
    reg [4:0] miss_word;
    reg [4:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [18:0] fill_tag;
    reg [6:0] fill_idx;
    reg [1:0] fill_way;

    (* max_fanout = 32 *) reg [13:0] rd_addr;
    reg rd_en, line_match;
//回填交付判据，与缺失地址快照（缺失锁寄存一拍后用来补对位）
    reg deliver_ok;
    reg [31:0] miss_addr_q;

    integer i;

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：本模块的取指推进由 req_valid / 回填状态自己把关，
//不用流水线使能，故只取两条冲刷位。
    reg flush_w;
    always @(*) flush_w = flag_bus[3] | flag_bus[2];

//预测跳转成立（按"顶层不运算"从 cpu_top 下放至此）
    reg jalr;
    always @(*) jalr = pre_jalr & btb_hit;

//取指地址 = pc_addr >>> 2，不再依赖本拍刚读出的指令。
//br1/jal 的改向原先也在这里"当拍"生效，代价是锥里串进了 pre_decoder 的译码 +
//pc_addr+offset 的 32 位进位链 + fetch_addr 两级 mux；那是 icache 那个 0 拍环的头。
//现在改由 pc 寄存一拍（pc 落到目标，icache 下一拍自然跟着 pc_addr>>>2 走到目标），
//环头整段消失。代价：每次预测跳转命中多一拍空泡 —— 那拍由下面的 NOP 门刷掉。
    (* max_fanout = 32 *) reg [31:0] fetch_addr;
    always @(*) begin
        if (rst_q) fetch_addr = 32'd0;
        else     fetch_addr = pc_addr >>> 2;
    end

    always @(*) begin
        tag = fetch_addr[31:12];
        idx = fetch_addr[11:5];
        word = fetch_addr[4:0];
//way0 是锁定路：boot 把 128 组全填成 tag=0 且 valid=1，此后永不改写（替换只在 way1/way2 之间选），
//所以"查 tag0 阵列再比较"等价于"地址是否落在低 16KB"——常量比较即可，整个阵列连同 valid0 都能删。
//boot 期间 tag==0 会误报命中，但那时 busy 被 boot 分支强制为 1、req_valid=0（inst_out 不更新）、
//时钟块也走 boot 分支，三处消费点都不用它。
        hit_way[0] = (tag == 19'd0);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
        hit_way[2] = valid2[idx] && (tag2[idx] == tag);
        cache_hit = hit_way[0] || hit_way[1] || hit_way[2];
        hit_sel = hit_way[0] ? 2'd0 : (hit_way[1] ? 2'd1 : 2'd2);
//缺失锁：自举期间恒 busy；从缺失起点到回填结束之间保持 busy。
//stage==1 的终止判据用 deliver_ok 而非 line_match，理由见下面 deliver_ok 处。
        if (boot)                               busy = 1'b1;
        else if (stage == 1'b1) begin
            if (!fill_end)                      busy = 1'b1;
            else if (!cache_hit && !deliver_ok) busy = 1'b1;
            else                                busy = 1'b0;
        end
        else if (stage == 1'b0) begin
            if (!cache_hit)                     busy = 1'b1;
            else                                busy = 1'b0;
        end
        else                                    busy = 1'b0;
        mem_req = (boot || stage == 1'b1) && !fill_end;
        mem_addr = fill_addr + (fill_cnt << 2);
        mem_we = 1'b0;
        mem_wdata = 32'd0;
        mem_be = 4'd0;

//读口：把 hit 与 fill_end 两条路合成一个地址/一个使能/一个数据选择，
//这样每个 iram 只有一个读口，才推得出 BRAM
        line_match = (fetch_addr & 32'hFFFFFFE0) == (fill_addr >> 2);
//回填完成那拍该不该把 miss_word 交出去。正常情形是 line_match（取指地址还落在刚填完
//的那一行里）；但缺失锁寄存一拍后 pc 停在 X+4，若缺的是行末字（word 31），X+4 已经
//进了下一行 ⇒ line_match 落空 ⇒ 丢指令、还要多起一次回填。
//第二个条件"pc 恰好比缺失地址前进一个字"专门接住这种顺序前进的情形。
        if (line_match)                                    deliver_ok = 1'b1;
        else if ((pc_addr >>> 2) == (miss_addr_q + 32'd1)) deliver_ok = 1'b1;
        else                                               deliver_ok = 1'b0;

        if (fill_end && deliver_ok) begin
            rd_en   = 1'b1;
            rd_addr = {fill_way, fill_idx, miss_word};
        end
        else begin
            rd_en   = req_valid;
            rd_addr = {hit_sel, idx, word};
        end
    end

//回填：一拍收一个字直接写进 iram（BRAM 单写口，一拍只写一个地址）
    always @(posedge clk) begin
        if ((boot || stage == 1'b1) && !fill_end && mem_valid) begin
            iram[{fill_way, fill_idx, fill_cnt}] <= mem_data;
        end
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
        if (rst_q)                                  inst_out <= 32'd0;
        else if ((jalr | jalr_fail | br2 | br3 | irq | irq_ret | br1 | jal) && req_valid) inst_out <= 32'd0;
        else if (rd_en)                           inst_out <= iram[rd_addr];
    end

//缺失锁的延时拍。cache_hit 是组合的：若由它组合地驱动 busy→pipe_stall_w→stage，
//"tag 阵读出→比较→busy"这条链会一路横跨到全片每一个寄存器的 CE/R（15ns 下量到约 8ns）。
//寄存一拍就把链切断：链终止在 busy_q 的 D 端，下游从 busy_q 的 Q 端起算新的一拍。
//pc 与 icache 用的是同一个 stage，所以"晚一拍"是两者一起晚，pc↔icache 对位不变：
//缺失拍后 pc 停在 X+4，回填完那拍旁路送出 instr(X)，恰好满足 inst_out(N)=instr(pc_addr(N)-4)。
//复位值必须是 1：复位期间 boot=1 ⇒ busy 恒 1，取 0 会让复位后第一拍 stage 落到 EXE、
//req_valid 抬起，icache 拿无效 hit_sel 去读 iram 并被 pre_decoder 锁进流水线。
    always @(posedge clk) begin
        if (rst_q) busy_q <= 1'b1;
        else     busy_q <= busy;
    end

//自举序列：复位释放后逐行填充前 16KB（128 行 × 32 字），固定写 way0
    always @(posedge clk) begin
        if (rst_q) begin
            boot <= 1'b1;
            boot_line <= 7'd0;
            stage <= 1'b0;
            fill_end <= 1'b0;
            fill_cnt <= 5'd0;
            fill_way <= 2'd0;
            fill_addr <= 32'd0;
            fill_tag <= 19'd0;
            fill_idx <= 7'd0;
            miss_word <= 5'd0;
            miss_addr_q <= 32'd0;
            valid1 <= 128'd0;
            valid2 <= 128'd0;
            lru <= 128'd0;
//tag1/tag2 不复位：命中由 valid 挡住，陈旧 tag 无害。
//带复位会让这 3x128x19 位全被综合成带复位的 FF（撑爆 slice 打包），
//去掉后能进分布式 RAM。valid 必须留复位 —— 它才是命中判据。
        end
        else if (boot) begin
            if (mem_valid && !fill_end) begin
                if (fill_cnt == 5'd31)
                    fill_end <= 1'b1;
                else
                    fill_cnt <= fill_cnt + 5'd1;
            end
            if (fill_end) begin
                fill_end <= 1'b0;
                fill_cnt <= 5'd0;
                fill_way <= 2'd0;
                fill_tag <= 19'd0;
                miss_word <= 5'd0;
                if (boot_line == BOOT_LINES) begin
                    boot <= 1'b0;
                    stage <= 1'b0;
                end
                else begin
                    boot_line <= boot_line + 7'd1;
                    fill_idx <= boot_line + 7'd1;
                    fill_addr <= {boot_line + 7'd1, 7'd0};
                end
            end
        end
        else begin
            if (stage == 1'b0) begin
                fill_end <= 1'b0;
                if (hit_way[1])      lru[idx] <= 1'b1;
                else if (hit_way[2]) lru[idx] <= 1'b0;
                if (!cache_hit) begin
                    stage <= 1'b1;
                    miss_word <= word;
                    miss_addr_q <= fetch_addr;
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
                    if (!cache_hit && !deliver_ok) begin
                        stage <= 1'b1;
                        miss_word <= word;
                        miss_addr_q <= fetch_addr;
                        fill_cnt <= 5'd0;
                        fill_addr <= {fetch_addr[31:5], 5'd0} << 2;
                        fill_tag <= tag;
                        fill_idx <= idx;
                        fill_way <= (fill_way == 2'd1) ? 2'd2 : 2'd1;
                        fill_end <= 1'b0;
                    end
                    else begin
                        stage <= 1'b0;
                        fill_end <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
