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
    input [7:0] flag_bus,
//保留站的"满"【只喂前端】：读口要跟着上游一起停，值才不会被下一条覆写
    input stall_rs_full,
//读口地址 = iffu 的出队格那条的 rs。它下一拍就是 idu1 输出级那条 ⇒ 读值寄存一拍后正好与那条同拍，
//所以这里就是把"本条指令要的操作数"从阵列里取出来的地方。
    input [4:0] r1, r2, r3, r4,
//两个物理写口的请求：由 ROB 的【提交口】驱动（口 A = head 更老、口 B = head+1 更年轻）。
//阵列只在提交时被写 ⇒ 架构态永远精确，同 rd 的先后由结构保证。
    input we_a,
    input [4:0] rd_a,
    input [31:0] data_a,
    input we_b,
    input [4:0] rd_b,
    input [31:0] data_b,
//读数据
    output reg [31:0] r1_data, r2_data, r3_data, r4_data
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    reg [31:0] regs [0:31];

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
        end
    end

//bypass：阵列 + 提交口 rd 匹配。读口只剩这两档。
//★ 在飞生产者的值【不在这里补】：①ROB 值、②三个结果口那两档搬到了 idu2 的入站取值梯 ——
//  读口地址放宽到"下一拍进输出级那条"之后，本条指令还没进册 ⇒ 本模块这一拍拿不到它的生产者槽号，
//  而 idu2 取值那一拍（= 它与载荷同拍进站那一拍）拿得到。这里只兜【刚落进写口那一笔】：
//  提交口的写要到下一拍才落进阵列，中间那一格阵列还是老值。
//★ x0 恒 0：单元结果口的 rd 可以是 0（`add x0,..`），这一句挡掉它在 rx=0 时的误命中。
//★ 顺序：口 B（head+1，更年轻）先判 ⇒ 同 rd 时年轻的那笔赢。
    function [31:0] bypass;
        input [4:0]  rx;
        begin
            if (rx == 5'd0)
                bypass = 32'd0;
            else if (we_b && (rd_b == rx))
                bypass = data_b;
            else if (we_a && (rd_a == rx))
                bypass = data_a;
            else
                bypass = regs[rx];
        end
    endfunction

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 cont 里已逐条分开）。
    reg exec, flush_w, stall_w, flush_con_exc;
    always @(*) begin
        flush_con_exc     = flag_bus[7];
        flush_w = flush_con_exc | flag_bus[6] | flag_bus[5];
        stall_w = (flag_bus[3] | flag_bus[2] | stall_rs_full | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

//读数据进行旁路仲裁并输出。【时序读】：地址由 iffu 的出队格给出（比载荷早一拍），值在这里寄存后随载荷
//往下走。★ 不要再退回"组合读"：那样阵列读会和旁路 mux、前送 mux 压进同一拍（关键路径变长），
//而且 iverilog 不把存储器元素算进 `always @(*)` 的隐含敏感表 ⇒ 写阵列后读口不刷新（实测踩过）。
//★ 保持判据必须与 idu1 的 advance 逐项同门（`~advance` 就是 flush_w | stall_w，exec 那一项由外层给）：
//  上游那一拍没推进 ⇒ 出队格与 idu1 输出级一起冻住 ⇒ 本条要的操作数还是【上一条】的，只能保持。
//  差一项就会按新地址去读下一条的 rs，把载荷的操作数覆写成别人的值（实测：div/rem 一族签名不对）。
    reg [4:0] r1_addr_q, r2_addr_q, r3_addr_q, r4_addr_q;
    always @(posedge clk) begin
        if (rst_q) begin
            r1_data <= 32'd0;
            r2_data <= 32'd0;
            r3_data <= 32'd0;
            r4_data <= 32'd0;
            r1_addr_q <= 5'd0;
            r2_addr_q <= 5'd0;
            r3_addr_q <= 5'd0;
            r4_addr_q <= 5'd0;
        end
        else if (exec) begin
            if (flush_w) begin
                r1_data <= r1_data;
                r2_data <= r2_data;
                r3_data <= r3_data;
                r4_data <= r4_data;
                r1_addr_q <= r1_addr_q;
                r2_addr_q <= r2_addr_q;
                r3_addr_q <= r3_addr_q;
                r4_addr_q <= r4_addr_q;
            end
            else if (stall_w) begin
                r1_addr_q <= r1_addr_q;
                r2_addr_q <= r2_addr_q;
                r3_addr_q <= r3_addr_q;
                r4_addr_q <= r4_addr_q;
                r1_data <= (we_b & (rd_b == r1_addr_q)) ? data_b :
                           (we_a & (rd_a == r1_addr_q)) ? data_a : r1_data;
                r2_data <= (we_b & (rd_b == r2_addr_q)) ? data_b :
                           (we_a & (rd_a == r2_addr_q)) ? data_a : r2_data;
                r3_data <= (we_b & (rd_b == r3_addr_q)) ? data_b :
                           (we_a & (rd_a == r3_addr_q)) ? data_a : r3_data;
                r4_data <= (we_b & (rd_b == r4_addr_q)) ? data_b :
                           (we_a & (rd_a == r4_addr_q)) ? data_a : r4_data;
            end
            else begin
                r1_data <= bypass(r1);
                r2_data <= bypass(r2);
                r3_data <= bypass(r3);
                r4_data <= bypass(r4);
                r1_addr_q <= r1;
                r2_addr_q <= r2;
                r3_addr_q <= r3;
                r4_addr_q <= r4;
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
