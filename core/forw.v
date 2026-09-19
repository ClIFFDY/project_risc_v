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
    input [31:0] r1_data_in_dec, r2_data_in_dec, r1_data_in_lsu, r2_data_in_lsu, ld_data,
    input [31:0] r1_data_in_mul, r2_data_in_mul,
    input [31:0] mul_data,
    input loaded, mul_loaded, stall,
    output reg [31:0] r1_data_final_dec, r2_data_final_dec, r1_data_final_lsu, r2_data_final_lsu,
    output reg [31:0] r1_data_final_mul, r2_data_final_mul
    );
    reg stalled;

//对ld/st指令数据旁路进行延迟仲裁。
//_mul 是【独立的第三份镜像】（照 pc.v 的 pc_addr/aux_addr 手法）：
//三条镜像链各自只喂一个消费单元，把比较器组和输出网按消费者拆开降扇出。
    always @(*) begin
        if (!stalled && rd_back1 != 5'd0 && r1 == rd_back1) begin
            r1_data_final_dec = result_back1;
            r1_data_final_lsu = result_back1;
            r1_data_final_mul = result_back1;
        end
        else if (loaded && rd_load != 5'd0 && r1 == rd_load) begin
            r1_data_final_dec = ld_data;
            r1_data_final_lsu = ld_data;
            r1_data_final_mul = ld_data;
        end
//mulu 在途结果：与 loaded 一样是【独立支路】，绝不能与 back2 合并成一个比较器
//（历史翻车记录见 逻辑说明 §2.2）。mulu 与 lsu 同为在途单元，程序序早于 wb_reg。
        else if (mul_loaded && rd_mul != 5'd0 && r1 == rd_mul) begin
            r1_data_final_dec = mul_data;
            r1_data_final_lsu = mul_data;
            r1_data_final_mul = mul_data;
        end
        else if (rd_back2 != 5'd0 && r1 == rd_back2) begin
            r1_data_final_dec = result_back2;
            r1_data_final_lsu = result_back2;
            r1_data_final_mul = result_back2;
        end
        else begin
            r1_data_final_dec = r1_data_in_dec;
            r1_data_final_lsu = r1_data_in_lsu;
            r1_data_final_mul = r1_data_in_mul;
        end
        if (!stalled && rd_back1 != 5'd0 && r2 == rd_back1) begin
            r2_data_final_dec = result_back1;
            r2_data_final_lsu = result_back1;
            r2_data_final_mul = result_back1;
        end
        else if (loaded && rd_load != 5'd0 && r2 == rd_load) begin
            r2_data_final_dec = ld_data;
            r2_data_final_lsu = ld_data;
            r2_data_final_mul = ld_data;
        end
        else if (mul_loaded && rd_mul != 5'd0 && r2 == rd_mul) begin
            r2_data_final_dec = mul_data;
            r2_data_final_lsu = mul_data;
            r2_data_final_mul = mul_data;
        end
        else if (rd_back2 != 5'd0 && r2 == rd_back2) begin
            r2_data_final_dec = result_back2;
            r2_data_final_lsu = result_back2;
            r2_data_final_mul = result_back2;
        end
        else begin
            r2_data_final_dec = r2_data_in_dec;
            r2_data_final_lsu = r2_data_in_lsu;
            r2_data_final_mul = r2_data_in_mul;
        end
    end

    always @(posedge clk) begin
        if (rst) stalled <= 1'd0;
        else stalled <= stall;
    end
endmodule
