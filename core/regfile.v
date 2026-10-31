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
    input stall_read_set_full,
//读口地址 = iffu 的出队格那条的 rs。它下一拍就是 idu1 输出级那条 ⇒ 读值寄存一拍后正好与那条同拍，
//所以这里就是把"本条指令要的操作数"从阵列里取出来的地方。
    input [4:0] read_1, read_2, read_3, read_4,
//★ 第 5/6 口：给访存专用流（idu1 的 LSU FIFO 队头）用。组合读，与前面四口同款。
    input [4:0] read_5, read_6,
//两个物理写口的请求：由 ROB 的【提交口】驱动（口 A = head 更老、口 B = head+1 更年轻）。
//阵列只在提交时被写 ⇒ 架构态永远精确，同 rd 的先后由结构保证。
    input write_enable_a,
    input [4:0] destination_a,
    input [31:0] data_a,
    input write_enable_b,
    input [4:0] destination_b,
    input [31:0] data_b,
//读数据
    output [31:0] read_1_data, read_2_data, read_3_data, read_4_data,
    output [31:0] read_5_data, read_6_data
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

    reg [31:0] regs [0:31];

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
        end
    end

//flag_bus = {flush_control_exception, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】；
//或运算在本模块内做（源在 cont 里已逐条分开）。
    reg exec, flush_window, stall_write, flush_control_exception;
    always @(*) begin
        flush_control_exception     = flag_bus[7];
        flush_window = flush_control_exception | flag_bus[6] | flag_bus[5];
        stall_write = (flag_bus[3] | flag_bus[2] | stall_read_set_full | flag_bus[0]) & ~flush_window;
        exec    = flag_bus[4];
    end

//读数据进行旁路仲裁并输出。【时序读】：地址由 iffu 的出队格给出（比载荷早一拍），值在这里寄存后随载荷
//往下走。★ 不要再退回"组合读"：那样阵列读会和旁路 mux、前送 mux 压进同一拍（关键路径变长），
//而且 iverilog 不把存储器元素算进 `always @(*)` 的隐含敏感表 ⇒ 写阵列后读口不刷新（实测踩过）。
//★ 保持判据必须与 idu1 的 advance 逐项同门（`~advance` 就是 flush_window | stall_write，exec 那一项由外层给）：
//  上游那一拍没推进 ⇒ 出队格与 idu1 输出级一起冻住 ⇒ 本条要的操作数还是【上一条】的，只能保持。
//  差一项就会按新地址去读下一条的 rs，把载荷的操作数覆写成别人的值（实测：div/rem 一族签名不对）。
//★ 组合读：地址由 idu1 的 Q 级（输出寄存器）给出，与 idu2 的 D 级是同一拍、同一条指令。
//  数据不再寄存，直接到 idu2 的 decode_base_next_*（那里还会用同两个提交口再判一次，
//  所以这里的 bypass 只是保持与原来逐位相同的行为，并不是必需项）。
    assign read_1_data = regs[read_1];   // x0 恒 0：阵列从不为 rd==0 写入
    assign read_2_data = regs[read_2];   // x0 恒 0：阵列从不为 rd==0 写入
    assign read_3_data = regs[read_3];   // x0 恒 0：阵列从不为 rd==0 写入
    assign read_4_data = regs[read_4];
    assign read_5_data = regs[read_5];
    assign read_6_data = regs[read_6];   // x0 恒 0：阵列从不为 rd==0 写入

//两个物理写口 = ROB 的两条提交口：口 A = head（更老）、口 B = head+1（更年轻）。
//同拍同 rd 撞上时【更年轻的那笔必须赢】⇒ 语句顺序：先 A 后 B，B 覆盖 A。
//阵列只在提交时被写 ⇒ 它是精确架构态；在飞值由读侧旁路从 ROB / 单元结果口补。
    always @(posedge clk) begin
        if (write_enable_a)
            regs[destination_a] <= data_a;
        if (write_enable_b)
            regs[destination_b] <= data_b;
    end

endmodule
