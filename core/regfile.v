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
    input [8:0] flag_bus,
    input [4:0] r1, r2,
    input [4:0] rd_alu,
    input [31:0] rd_data_alu,
    input we_alu,
    input [4:0] rd_ld,
    input [31:0] ld_data_ld,
    input we_ld,
    input [4:0] rd_mul,
    input [31:0] mul_data_mul,
    input we_mul,
//读数据：合并成一对（原来是 dec/lsu/mul 三份按限定分开填）。按消费者复制交给
//max_fanout 在布局阶段做 —— 比手工拆三份更省逻辑，复制点也更贴实际负载。
    (* max_fanout = 32 *) output reg [31:0] r1_data, r2_data
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//同步读写型通用寄存器组，节省lut资源
    (* ram_style = "block" *) reg [31:0] regs [0:31];

    reg [4:0] r1_q, r2_q;
    reg am_we;
    reg [4:0] am_rd;
    reg [31:0] am_data;
    reg ld_we_eff;

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            regs[i] = 32'd0;
        end
    end

//bypass 与写口用同一优先级（alu > mul > ld）：真撞上时"写进去的"与"前递出去的"是同一个值。
//（改前 bypass 是 alu>ld>mul、写口是 mul>ld>alu，方向正好相反 —— 那是 §2.2 记的老隐患。）
    function [31:0] bypass;
        input [4:0] rx;
        begin
            if (am_we && rx == am_rd) bypass = am_data;
            else if (ld_we_eff && rx == rd_ld) bypass = ld_data_ld;
            else bypass = regs[rx];
        end
    endfunction

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_b, stall_m, stall_v, stall_l, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[7] | flag_bus[6];
        stall_w = (flag_bus[5] | flag_bus[4] | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//读数据进行双写口(alu/ld)旁路仲裁并输出。
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

//两个物理写口（原来是三个，见 逻辑说明 §27）：
//  口 A = alu | mul —— 这两者的写沿都由【流水线偏移】决定（c2+3），且 mulu 的两级乘法流水、
//        除法提交链、输出保持都已按 pipe_stall 冻结（照 alu 的写回级，与 lsu 对 stage 的门控同理），
//        所以二者严格同偏移、永不同拍，可以共用一口。
//  口 B = ld —— load 的写沿由【总线事务长度】决定（多拍），与流水线偏移不绑定，
//        与谁合并都可能撞，故单独一口，保持"撞上也是两条独立写"的旧行为。
    always @(posedge clk) begin
        if (ld_we_eff) regs[rd_ld] <= ld_data_ld;
        if (am_we)     regs[am_rd] <= am_data;
    end

//写口仲裁：alu > mul > ld
    always @(*) begin
        if (we_alu && rd_alu != 5'd0) begin
            am_we = 1'b1;
            am_rd = rd_alu;
            am_data = rd_data_alu;
        end
        else if (we_mul && rd_mul != 5'd0) begin
            am_we = 1'b1;
            am_rd = rd_mul;
            am_data = mul_data_mul;
        end
        else begin
            am_we = 1'b0;
            am_rd = 5'd0;
            am_data = 32'd0;
        end
        ld_we_eff = we_ld && (rd_ld != 5'd0);
    end

endmodule
