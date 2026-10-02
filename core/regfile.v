`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: regfile
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


module regfile(
    input clk, rst,
    input [11:0] flag_bus,
//读口地址 = pre 级那条（它下一拍就是 decoder 载荷那条）的 rs。读值在本模块寄存一拍、
//正好与载荷同拍，所以这里就是把"本条指令要的操作数"从阵列+旁路里取出来的地方。
    input [4:0] r1, r2,
//两个物理写口的请求（仲裁已在 wbu 内完成）：口 A = alu | mul，口 B = ld
    input we_a,
    input [4:0] rd_a,
    input [31:0] data_a,
    input we_b,
    input [4:0] rd_b,
    input [31:0] data_b,
//三个执行单元的结果口（与 forw 点② 同源）：值离开结果口那一拍它还不在阵列里
//（阵列要下一沿才写）⇒ 读侧旁路必须把它们一起覆盖，否则"读完一拍、消费者才用"这一格是空的。
    input        we_alu,
    input [31:0] data_alu,
    input        we_mul,
    input [31:0] data_mul,
    input        we_ld,
    input [31:0] data_ld,
//选源用：5 个源的 ROB 索引 + rob 的槽扫描结果（**与 payload 的 sel_slot 同源，取当拍组合版**）。
//★ 为什么不能按"寄存器号相同"来选：rd 相同的笔可能同时在场上，而"谁更新"由端口身份/语句顺序
//  定不下来 —— 写口出口寄存器里的值是【上一拍】的单元结果，mul/ld 结果口上守着的又是更早完成、
//  还没被取走的笔 ⇒ 写口优先和结果口优先各有反例。槽号唯一 ⇒ 至多一个口命中，而且命中的必然
//  是 rob 扫出来的那条"比我老里最年轻"的笔（这正是 forw 点① 原来的选法，搬过来一字不改）。
    input [2:0]  idx_alu, idx_mul, idx_ld, idx_a, idx_b,
    input [2:0]  fwd_slot1, fwd_slot2,
    input        fwd_hit1,  fwd_hit2,
//读数据：合并成一对（原来是 dec/lsu/mul 三份按限定分开填）。按消费者复制交给
//max_fanout 在布局阶段做 —— 比手工拆三份更省逻辑，复制点也更贴实际负载。
    output reg [31:0] r1_data, r2_data
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    reg [31:0] regs [0:31];

    reg [4:0] r1_q, r2_q;
//地址与"谁来供值"一起冻住：停顿重读时用同一对（地址、槽号）。
//★ 槽号在停顿期间不会失效：停顿拍不分配（payload_go=0）⇒ 槽不会被重用；生产者若在这期间把值
//  落进阵列，`idx_* == slot` 自然不再命中，阵列值就是它。
    reg [2:0] s1_q, s2_q;
    reg       h1_q, h2_q;
//旁路结果的临时名（含"有没有源口命中"那一位）：只在上面那个时序块里用
    reg [32:0] bp1, bp2;

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
        end
    end

//bypass：读侧旁路，**覆盖 5 个源** —— 三条单元结果口（alu / mul / ld）+ 两条物理写口。
//选源判据 = "源口的 ROB 索引 == 本条指令的槽号"，与 forw 点② 同形（它就是 forw 点① 那一套）。
//返回 {有源口命中, 值}：槽号唯一 ⇒ 至多一个口命中；没命中就回落到阵列值（此时"生产者还没出
//结果"或"它的值已经落进阵列"，两种情况下阵列值就是它）。
//★ 顺序（alu → 口B → 口A → mul → ld）只是防御性的 tie-break。
//★ x0 恒 0：单元结果口的 rd 可以是 0（`add x0,..`），写口的 rd 被 wport 挡过不会为 0
//  ⇒ 这一句是挡掉前者在 rx=0 时的误命中（扫描口本身也带 x0 保护，这里是第二道）。
    function [32:0] bypass;
        input [4:0] rx;
        input       hit;
        input [2:0] slot;
        begin
            if (rx == 5'd0)
                bypass = 33'd0;
            else if (!hit)
                bypass = {1'b0, regs[rx]};
            else if (we_alu && (idx_alu == slot))
                bypass = {1'b1, data_alu};
            else if (we_b && (idx_b == slot))
                bypass = {1'b1, data_b};
            else if (we_a && (idx_a == slot))
                bypass = {1'b1, data_a};
            else if (we_mul && (idx_mul == slot))
                bypass = {1'b1, data_mul};
            else if (we_ld && (idx_ld == slot))
                bypass = {1'b1, data_ld};
            else
                bypass = {1'b0, regs[rx]};
        end
    endfunction

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 controller 里已逐条分开）。
    reg exec, flush_w, stall_w, flush_con_exc;
    always @(*) begin
        flush_con_exc     = flag_bus[11];
        flush_w = flush_con_exc | flag_bus[10] | flag_bus[9];
        stall_w = (flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4] | flag_bus[3]
                 | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//读数据进行 5 源旁路仲裁并输出。【时序读】：地址由 pre_decoder 那一级给出（比 decoder
//载荷早一拍），值在这里寄存后随载荷往下走；前送级（decoder→执行单元之间）只在有更新结果时
//覆盖它。★ 不要再退回"组合读"：那样阵列读会和旁路 mux、前送 mux 压进同一拍（关键路径变长），
//而且 iverilog 不把存储器元素算进 `always @(*)` 的隐含敏感表 ⇒ 写阵列后读口不刷新（实测踩过）。
//停顿重读走 `bypass(r1_q, h1_q, s1_q)`，用的是**单调刷新**：地址与槽号都冻着，命中源口就
//换成源口上的值、没命中就**保持原值**（★ 不能每拍从阵列重算：生产者的值可能只在单元口上出现
//过一拍，之后那笔又被写口丢掉、没做数组写 —— 重算会把它丢成阵列里的老值。HEAD 那版靠操作数
//寄存器 + 停顿回灌达到同样效果，注释里记着 CoreMark 的 crcstate 错就是这个）。
//原来的 dec/lsu/mul 三个限定只用来决定"填哪一份"，合并成一对后不再需要：
//没有限定置位时算出来的值无人消费（JAL 之类），驱出去无害。
    always @(posedge clk) begin
        if (rst_q) begin
            r1_data <= 32'd0;
            r2_data <= 32'd0;
            r1_q <= 5'd0;
            r2_q <= 5'd0;
            s1_q <= 3'd0;
            s2_q <= 3'd0;
            h1_q <= 1'b0;
            h2_q <= 1'b0;
        end
        else if (exec) begin
            if (stall_w) begin
                bp1 = bypass(r1_q, h1_q, s1_q);
                bp2 = bypass(r2_q, h2_q, s2_q);
                if (bp1[32])
                    r1_data <= bp1[31:0];
                else
                    r1_data <= r1_data;
                if (bp2[32])
                    r2_data <= bp2[31:0];
                else
                    r2_data <= r2_data;
            end
            else begin
                bp1 = bypass(r1, fwd_hit1, fwd_slot1);
                bp2 = bypass(r2, fwd_hit2, fwd_slot2);
                r1_data <= bp1[31:0];
                r2_data <= bp2[31:0];
                r1_q <= r1;
                r2_q <= r2;
                s1_q <= fwd_slot1;
                s2_q <= fwd_slot2;
                h1_q <= fwd_hit1;
                h2_q <= fwd_hit2;
            end
        end
    end

//两个物理写口（原来是三个，见 逻辑说明 §27）。现在两口由【ROB 退口】驱动：
//  口 A = head（更老）、口 B = head+1（更年轻）⇒ 同拍同 rd 撞上时【更年轻的那笔必须赢】
//  ⇒ 语句顺序反过来写：先 A 后 B，B 覆盖 A。（旧 wbu 时代是口 A 赢，正好相反。）
    always @(posedge clk) begin
        if (we_a)
            regs[rd_a] <= data_a;
        if (we_b)
            regs[rd_b] <= data_b;
    end

endmodule
