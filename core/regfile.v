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
//读口地址 = 【decoder 载荷里那条】的 rs（消费者在"decoder→执行单元之间"那一拍），
//前送点（forw）就在这一拍，所以这里是【组合读】：阵列值 + 同拍写旁路一起交给 forw。
    input [4:0] r1, r2,
//两个物理写口的请求（仲裁已在 wbu 内完成）：口 A = alu | mul，口 B = ld
    input we_a,
    input [4:0] rd_a,
    input [31:0] data_a,
    input we_b,
    input [4:0] rd_b,
    input [31:0] data_b,
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

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
        end
    end

//bypass 与写口用同一优先级（口 A > 口 B，即"后面那条 if 写的赢"）：真撞上时
//"写进去的"与"前递出去的"是同一个值。
    function [31:0] bypass;
        input [4:0] rx;
        begin
            if (we_b && rx == rd_b) bypass = data_b;      // 更年轻那口优先（同 rd 撞上时）
            else if (we_a && rx == rd_a) bypass = data_a;
            else bypass = regs[rx];
        end
    endfunction

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 controller 里已逐条分开）。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[11] | flag_bus[10] | flag_bus[9];
        stall_w = (flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4] | flag_bus[3]
                 | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//读数据进行双写口旁路仲裁并输出。【时序读】：地址由 pre_decoder 那一级给出（比 decoder
//载荷早两级），值在这里寄存后随载荷往下走；前送级（decoder→执行单元之间）只在有更新结果时
//覆盖它。★ 不要再退回"组合读"：那样阵列读会和旁路 mux、前送 mux 压进同一拍（关键路径变长），
//而且 iverilog 不把存储器元素算进 `always @(*)` 的隐含敏感表 ⇒ 写阵列后读口不刷新（实测踩过）。
//原来的 dec/lsu/mul 三个限定只用来决定"填哪一份"，合并成一对后不再需要：
//没有限定置位时算出来的值无人消费（JAL 之类），驱出去无害。
    always @(posedge clk) begin
        if (rst_q) begin
            r1_data <= 32'd0;
            r2_data <= 32'd0;
            r1_q <= 5'd0;
            r2_q <= 5'd0;
        end
        else if (exec) begin
            if (stall_w) begin
                r1_data <= bypass(r1_q);
                r2_data <= bypass(r2_q);
            end
            else begin
                r1_data <= bypass(r1);
                r2_data <= bypass(r2);
                r1_q <= r1;
                r2_q <= r2;
            end
        end
    end

//两个物理写口（原来是三个，见 逻辑说明 §27）。现在两口由【ROB 退口】驱动：
//  口 A = head（更老）、口 B = head+1（更年轻）⇒ 同拍同 rd 撞上时【更年轻的那笔必须赢】
//  ⇒ 语句顺序反过来写：先 A 后 B，B 覆盖 A。（旧 wbu 时代是口 A 赢，正好相反。）
    always @(posedge clk) begin
        if (we_a) regs[rd_a] <= data_a;
        if (we_b) regs[rd_b] <= data_b;
    end

endmodule
