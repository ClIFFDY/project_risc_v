`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/28 14:41:46
// Design Name:
// Module Name: forw
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


module forw(
    input clk, rst,
    input [31:0] result_back1, result_back2,
    input [4:0] r1, r2,
    input [4:0] rd_back1, rd_back2, rd_load, rd_mul,
    input [31:0] r1_data_in, r2_data_in, ld_data,
    input [31:0] mul_data,
    input loaded, mul_loaded, lsu_stall, mul_stall,
//前递结果：合并成一对（原来是 dec/lsu/mul 三份一模一样的镜像）。按消费者复制交给
//max_fanout 在布局阶段做 —— 优先 mux 树只算一遍，比手工拆三份省 2/3 逻辑。
    (* max_fanout = 32 *) output reg [31:0] r1_data_final, r2_data_final
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;
    reg stalled, stall;

//停顿合流（按"顶层不运算"从 cpu_top 下放至此）
    always @(*) stall = lsu_stall | mul_stall;

//对ld/st指令数据旁路进行延迟仲裁。
//优先级：alu 在途 > load 在途 > mulu 在途 > wb_reg。
    always @(*) begin
        if (r1 != 5'd0) begin
            if (!stalled && r1 == rd_back1) begin
                r1_data_final = result_back1;
            end
            else if (loaded && r1 == rd_load) begin
                r1_data_final = ld_data;
            end
//mulu 在途结果：与 loaded 一样是【独立支路】，绝不能与 back2 合并成一个比较器
//（历史翻车记录见 逻辑说明 §2.2）。mulu 与 lsu 同为在途单元，程序序早于 wb_reg。
            else if (mul_loaded && r1 == rd_mul) begin
                r1_data_final = mul_data;
            end
            else if (r1 == rd_back2) begin
                r1_data_final = result_back2;
            end
            else begin
                r1_data_final = r1_data_in;
            end
        end
        else r1_data_final = r1_data_in;
        if (r2 != 5'd0) begin
            if (!stalled && r2 == rd_back1) begin
                r2_data_final = result_back1;
            end
            else if (loaded && r2 == rd_load) begin
                r2_data_final = ld_data;
            end
            else if (mul_loaded && r2 == rd_mul) begin
                r2_data_final = mul_data;
            end
            else if (r2 == rd_back2) begin
                r2_data_final = result_back2;
            end
            else begin
                r2_data_final = r2_data_in;
            end
        end
        else r2_data_final = r2_data_in;
    end

    always @(posedge clk) begin
        if (rst_q) stalled <= 1'b0;
        else stalled <= stall;
    end
endmodule
