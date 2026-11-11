`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: dsq（dispatch queue）
// Description:
//   分配队列。位置在【译码 + 改名 + 分 ROB 槽】之后、【分 lane】之前。
//
//   为什么要它：iffu 一拍最多递 2~3 条，而三个发射口一拍最多吃 3 条，两边宽度不等
//   ⇒ 中间必须有一段弹性。以前这段弹性靠 idu 的"头 + 影子"加一根组合的 advance 回前端，
//   代价是分配结论要当拍穿回 iffu ⇒ 一条 20 级、跨两个模块的流控环。
//   放进队列之后，iffu 只看队列剩余空间（纯寄存器），分配结论不再出本模块。
//
//   ★ 入口的条目【身份已经定死】：物理号、ROB 槽号、世代都随条目走。
//     所以出口可以【按类乱序取】，只要每个口在它自己那一类里守着先进先出。
//
//   ★ 结构 = 三条独立的 4 格环形队列（lane0 / lane1 / mem），照 5af972a
//     （「12.5ns 首次全通」，WNS +0.054 / 失败端点 0）那版"每 lane 一个 head+shadow"
//     以及同版 lsu_room = 4 − count 的深度来。那版没有池子、没有名次、没有空位扫描，
//     所以没有环；这里只是把它按类别扩成三条。
//       写：落格 = (队头 + 占用) & 3     —— 两个寄存器量做一次加
//       读：队头选 4:1 mux              —— 两级 LUT
//       没有 rank_q、没有 8 路扫描、没有名次维护、没有空闲格搜索
//
//   ★ room = min(三条队列各自的空位数)，【与类别无关】。
//     room 一旦依赖"这一条属于哪一类"，就依赖分配结果（lane 由物理号奇偶定），
//     而分配又依赖 room ⇒ 互为前提的环，实测直接死锁（room 报 2、两条都落 lane0、
//     第二条无处可去却被 rename 当成已分配 ⇒ 孤儿 ROB 项）。
//     因为 room ≤ 每类空位，任意 ≤ room 条、无论什么类别组合，都一定各归其位收得下。
//
//   ★ free 只吃【寄存器占用】，不吃当拍弹出。
//     吃了 pop 就把 isq 的接收 → 本站 free → room → rename → iffu 串成一条跨四模块的环，
//     实测（w4e/w4f）它是最大族：worst200 里 u_isq→u_dsq 92 条 + u_isq→u_iffu 61 条。
//     不吃 pop 的代价在 2 深时是 30% IPC（IPC 1.092 → 0.764）—— 因为任何一条队列满
//     就整体封口，而 2 深时 lane0 有 18.9% 的拍是满的。加深到 4 就是把"满"变成罕见事件：
//     实测 lane0 占到 >3 条的拍只有 2.7%、mem 只有 0.35%。
//
//   ★ 深度 4 ⇒ room 上界 3（一拍最多递 3 条）。
//////////////////////////////////////////////////////////////////////////////////

module dsq #(
    parameter integer PW    = 133
)(
    input clk, rst,
//冲刷：改向/中断那一拍整池作废（池里的条目全在边界之后）
    input        flush,
//---- 入队口：iffu 一拍最多递三条（half0/1/2），译码+改名+分槽之后进这里 ----
    input        push0_valid, input [PW-1:0] push0_data,
    input        push1_valid, input [PW-1:0] push1_data,
    input        push2_valid, input [PW-1:0] push2_data,
//给 iffU 的背压：这一拍还能收几条（0..3，口径与类别无关）
    output [1:0] room,
//---- 出队口：每类一条 4 格 FIFO，各取自己那条的队头 ----
    output        lane0_valid, output [PW-1:0] lane0_data, input lane0_pop,
    output        lane1_valid, output [PW-1:0] lane1_data, input lane1_pop,
    output        mem_valid,   output [PW-1:0] mem_data,   input mem_pop
    );

//载荷最低两位 = 类别，编码与 idu 的 CLS_* 一致：0=ALU 1=MUL 2=LSU 3=BJU
    localparam [1:0] CLS_LSU = 2'd2;
//★ 对 integer 参数做位选（QD[1:0]）Verilog-2001 不合法：Verilator 放过、Vivado 不放过。
    localparam integer QD    = 4;
    localparam [2:0]   QD_EXT = QD;

//★ 用函数而不是 "wire 数组 + generate"：Vivado 2019.1 对 unpacked 网表数组支持有限。
    function is_lsu;
        input [PW-1:0] d;
        is_lsu  = (d[1:0] == 2'd2);
    endfunction

    function lane_of;
        input [PW-1:0] d;
        lane_of = (d[36:31] != 6'd0) ? d[31] : d[13];
    endfunction

    function [2:0] min3;
        input [2:0] a;
        input [2:0] b;
        min3 = (a < b) ? a : b;
    endfunction

//==============================================================================
//  一、存储：三条 4 格环形队列
//==============================================================================
    reg [PW-1:0] q0 [0:QD-1];   reg [1:0] h0;   reg [2:0] n0;
    reg [PW-1:0] q1 [0:QD-1];   reg [1:0] h1q;  reg [2:0] n1;
    reg [PW-1:0] qm [0:QD-1];   reg [1:0] hm;   reg [2:0] nm;

    assign lane0_valid = (n0 != 3'd0);
    assign lane0_data  = q0[h0];
    assign lane1_valid = (n1 != 3'd0);
    assign lane1_data  = q1[h1q];
    assign mem_valid   = (nm != 3'd0);
    assign mem_data    = qm[hm];

    wire pop0_take = lane0_pop & (n0 != 3'd0);
    wire pop1_take = lane1_pop & (n1 != 3'd0);
    wire popm_take = mem_pop   & (nm != 3'd0);

//★ 只吃寄存器占用，不吃当拍弹出（理由见文件头）。
    wire [2:0] free0 = QD_EXT - n0;
    wire [2:0] free1 = QD_EXT - n1;
    wire [2:0] freem = QD_EXT - nm;

//★ 与类别无关的条数：取三个最小值，封顶 3。
    wire [2:0] room_min = min3(min3(free0, free1), freem);
    assign room = (room_min >= 3'd2) ? 2'd2 : room_min[1:0];

//==============================================================================
//  二、入队：前缀语义 —— 收第 k 条的先决条件是前面都收了，且 k < room
//==============================================================================
    reg        acc0, acc1, acc2;
    reg        w0_0, w0_1, w0_2;      // 落 lane0 队列
    reg        w1_0, w1_1, w1_2;      // 落 lane1 队列
    reg        wm_0, wm_1, wm_2;      // 落 mem 队列
    reg [1:0]  s0_0, s0_1, s0_2;      // lane0 落格
    reg [1:0]  s1_0, s1_1, s1_2;      // lane1 落格
    reg [1:0]  sm_0, sm_1, sm_2;      // mem 落格
    reg [1:0]  u0, u1, um;            // 本拍已经排进去的条数

//落格起点 = (队头 + 占用) & 3
    wire [1:0] wstart0 = h0  + n0[1:0];
    wire [1:0] wstart1 = h1q + n1[1:0];
    wire [1:0] wstartm = hm  + nm[1:0];

    always @(*) begin
        acc0 = push0_valid & (room >= 2'd1);
        acc1 = acc0 & push1_valid & (room >= 2'd2);
        acc2 = acc1 & push2_valid & (room >= 2'd3);

        w0_0=1'b0; w0_1=1'b0; w0_2=1'b0;
        w1_0=1'b0; w1_1=1'b0; w1_2=1'b0;
        wm_0=1'b0; wm_1=1'b0; wm_2=1'b0;
        s0_0=2'd0; s0_1=2'd0; s0_2=2'd0;
        s1_0=2'd0; s1_1=2'd0; s1_2=2'd0;
        sm_0=2'd0; sm_1=2'd0; sm_2=2'd0;
        u0=2'd0; u1=2'd0; um=2'd0;

        if (acc0) begin
            if (is_lsu(push0_data)) begin
                wm_0 = 1'b1; sm_0 = wstartm + um; um = um + 2'd1;
            end
            else if (lane_of(push0_data)) begin
                w1_0 = 1'b1; s1_0 = wstart1 + u1; u1 = u1 + 2'd1;
            end
            else begin
                w0_0 = 1'b1; s0_0 = wstart0 + u0; u0 = u0 + 2'd1;
            end
        end
        if (acc1) begin
            if (is_lsu(push1_data)) begin
                wm_1 = 1'b1; sm_1 = wstartm + um; um = um + 2'd1;
            end
            else if (lane_of(push1_data)) begin
                w1_1 = 1'b1; s1_1 = wstart1 + u1; u1 = u1 + 2'd1;
            end
            else begin
                w0_1 = 1'b1; s0_1 = wstart0 + u0; u0 = u0 + 2'd1;
            end
        end
        if (acc2) begin
            if (is_lsu(push2_data)) begin
                wm_2 = 1'b1; sm_2 = wstartm + um; um = um + 2'd1;
            end
            else if (lane_of(push2_data)) begin
                w1_2 = 1'b1; s1_2 = wstart1 + u1; u1 = u1 + 2'd1;
            end
            else begin
                w0_2 = 1'b1; s0_2 = wstart0 + u0; u0 = u0 + 2'd1;
            end
        end
    end

//==============================================================================
//  三、时序：写 / 弹出 / 清
//==============================================================================
    wire [1:0] push0_count = {1'b0, w0_0} + {1'b0, w0_1} + {1'b0, w0_2};
    wire [1:0] push1_count = {1'b0, w1_0} + {1'b0, w1_1} + {1'b0, w1_2};
    wire [1:0] pushm_count = {1'b0, wm_0} + {1'b0, wm_1} + {1'b0, wm_2};

    always @(posedge clk) begin
        if (rst | flush) begin
            n0 <= 3'd0; h0  <= 2'd0;
            n1 <= 3'd0; h1q <= 2'd0;
            nm <= 3'd0; hm  <= 2'd0;
        end
        else begin
//---- lane0 ----
            if (w0_0) q0[s0_0] <= push0_data;
            if (w0_1) q0[s0_1] <= push1_data;
            if (w0_2) q0[s0_2] <= push2_data;
            n0 <= n0 + {1'b0, push0_count} - {2'd0, pop0_take};
            if (pop0_take) h0 <= h0 + 2'd1;
//---- lane1 ----
            if (w1_0) q1[s1_0] <= push0_data;
            if (w1_1) q1[s1_1] <= push1_data;
            if (w1_2) q1[s1_2] <= push2_data;
            n1 <= n1 + {1'b0, push1_count} - {2'd0, pop1_take};
            if (pop1_take) h1q <= h1q + 2'd1;
//---- mem ----
            if (wm_0) qm[sm_0] <= push0_data;
            if (wm_1) qm[sm_1] <= push1_data;
            if (wm_2) qm[sm_2] <= push2_data;
            nm <= nm + {1'b0, pushm_count} - {2'd0, popm_take};
            if (popm_take) hm <= hm + 2'd1;
        end
    end

endmodule
