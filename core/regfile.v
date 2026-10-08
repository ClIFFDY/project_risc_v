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
//两个物理写口的请求：由 ROB 的【提交口】驱动（口 A = head 更老、口 B = head+1 更年轻）。
//阵列只在提交时被写 ⇒ 架构态永远精确，同 rd 的先后由结构保证。
    input we_a,
    input [4:0] rd_a,
    input [31:0] data_a,
    input we_b,
    input [4:0] rd_b,
    input [31:0] data_b,
//三个执行单元的结果口：值离开结果口那一拍它还不在阵列里（阵列只在提交时写）
//⇒ 读侧旁路必须把它们一起覆盖，否则"读完一拍、消费者才用"这一格是空的。
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
    input [2:0]  idx_alu, idx_mul, idx_ld,
//载荷那两个源的就绪（rob 按冻结的生产者槽现算）：它和 stall_w 是【同一件事】——
//都是"载荷这一拍的操作数还不是最终值，得按锁存的槽重读"。见下面读口的保持分支。
    input        rdy1_in, rdy2_in,
    input [2:0]  fwd_slot1, fwd_slot2,
    input        fwd_hit1,  fwd_hit2,
//ROB 值读口①【正常读】：索引是 rob 自己的扫描口（不外引）⇒ 这条路上没有 stall 组合量。
    input      [31:0] fwd_data1, fwd_data2,
    input             fwd_done1, fwd_done2,
//ROB 值读口②【停顿重读】：索引 = 本模块读锁存时锁下的槽 s1_q/s2_q（不外引，rob 用不上选择）。
    output reg [2:0]  st_slot1, st_slot2,
    input      [31:0] st_data1, st_data2,
    input             st_done1, st_done2,
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

//bypass：读侧旁路，源按"值能活多久"排：
//  ① ROB 值：命中槽的项【已算完】就取它的 data —— 覆盖"值已进 ROB 但还没提交"的整段窗口
//     （阵列要到提交才有值、结果口只宽一两拍，中间只有这里兜得住）。选源判据不再是"寄存器号
//     相同"，而是扫描/锁存给出的那个【槽号】：槽号唯一 ⇒ 至多一条命中。
//  ② 三条单元结果口：兜"生产者这一拍刚从口上出来、还没被 ROB 采进去"那一格（X = T）。
//  ③ 提交口（刚提交那一拍，按 rd 匹配）：阵列晚一拍写的空档只有它能兜 ✓
//  ④ 阵列：没有在飞的更老写者时就是它；有但没算完时由 hazard 顶住消费者（扫描口/检查点保证）。
//★ ①② 若同拍命中同一槽，值必然相同，顺序只是防御性 tie-break。
//★ x0 恒 0：单元结果口的 rd 可以是 0（`add x0,..`），这一句挡掉它在 rx=0 时的误命中。
    function [32:0] bypass;
        input [4:0]  rx;
        input        hit;
        input        done;
        input [31:0] fwd_d;
        input [2:0]  slot;
        begin
            if (rx == 5'd0)
                bypass = 33'd0;
            else if (hit && done)
                bypass = {1'b1, fwd_d};
            else if (hit && we_alu && (idx_alu == slot))
                bypass = {1'b1, data_alu};
            else if (hit && we_mul && (idx_mul == slot))
                bypass = {1'b1, data_mul};
            else if (hit && we_ld && (idx_ld == slot))
                bypass = {1'b1, data_ld};
//★ 兜"刚提交、阵列还没写"那一拍：提交口打拍之后，那一笔要在【下一拍】才落进阵列，
//  而它在提交拍就已从 ROB 移除（ent_v=0）⇒ 扫描给不出它 ⇒ 上面几档全不命中。
//  ★ 只在 !hit 时才用：hit=1 说明有更年轻的在飞写者，那一档该由 ROB 值/结果口/冒险裁决来管，
//    这里按 rd 兜底会盖掉它。
//  ★ 顺序：口 B（head+1，更年轻）先判 ⇒ 同 rd 时年轻的那笔赢。
            else if (!hit && we_b && (rd_b == rx))
                bypass = {1'b1, data_b};
            else if (!hit && we_a && (rd_a == rx))
                bypass = {1'b1, data_a};
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

//停顿重读的索引：就是锁存下来的那两个槽（纯别名，不做任何选择 ——
//  ★ 绝不能让 stall_w 参与"选地址"：它来自 flag_bus（= bju 的组合判定），
//    一旦进地址路就会把 bju 的锥串进操作数数据路，实测默认流程下 bju→regfile 多出 333 条违例。
    always @(*) begin
        st_slot1 = s1_q;
        st_slot2 = s2_q;
    end

//读数据进行 3 档旁路仲裁并输出。【时序读】：地址由 pre_decoder 那一级给出（比 decoder
//载荷早一拍），值在这里寄存后随载荷往下走；前送级（decoder→执行单元之间）只在有更新结果时
//覆盖它。★ 不要再退回"组合读"：那样阵列读会和旁路 mux、前送 mux 压进同一拍（关键路径变长），
//而且 iverilog 不把存储器元素算进 `always @(*)` 的隐含敏感表 ⇒ 写阵列后读口不刷新（实测踩过）。
//停顿重读走 `bypass(r1_q, h1_q, rd_done1, rd_data1, s1_q)`：地址与槽号冻着，靠**锁存的槽号**
//去 ROB 读口取值。★ ROB 值从"算完"一直保持到"提交"，所以重读总有一次能取到；不能改成每拍
//从阵列重算（生产者的值可能只在单元口上出现过一拍、阵列又还没被写 —— 重算会丢成阵列里的老值）。
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
//★ 判据必须与上游各级（pre_decoder 的 adv / post_decoder 的 payload_go）逐项同门：
//  rdy 只加在上游、不补进这里 ⇒ 暂停那一拍走"正常读"，用【下一条】的 rs 把载荷的操作数
//  覆写成别人的值（实测：div/rem 一族签名不对、CoreMark 跑到 401 字符就断）。
            if ((stall_w | ~rdy1_in | ~rdy2_in) & ~flush_w) begin
                bp1 = bypass(r1_q, h1_q, st_done1, st_data1, s1_q);
                bp2 = bypass(r2_q, h2_q, st_done2, st_data2, s2_q);
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
                bp1 = bypass(r1, fwd_hit1, fwd_done1, fwd_data1, fwd_slot1);
                bp2 = bypass(r2, fwd_hit2, fwd_done2, fwd_data2, fwd_slot2);
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

//两个物理写口 = ROB 的两条提交口：口 A = head（更老）、口 B = head+1（更年轻）。
//同拍同 rd 撞上时【更年轻的那笔必须赢】⇒ 语句顺序：先 A 后 B，B 覆盖 A。
//阵列只在提交时被写 ⇒ 它是精确架构态；在飞值由读侧旁路从 ROB / 单元结果口补。
    always @(posedge clk) begin
        if (we_a)
            regs[rd_a] <= data_a;
        if (we_b)
            regs[rd_b] <= data_b;
    end

endmodule
