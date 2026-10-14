`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: prf —— 物理寄存器堆（64 项 × 32 位）
//
//   编号（6 位 tag）：bit5=0 → 架构寄存器（低 5 位 = rs，写口只有提交口）；
//                     bit5=1 → 在飞物理寄存器（低 5 位 = {ROB 槽号, 世代}，写口只有结果口）。
//   写口 6 个：2 提交（架构半区，与 regfile.v 的写口同源）+ 4 结果（在飞半区，使能 = rob
//   回填 ent_data 的接受条件 ⇒ 阵列内容与 rob.ent_data 不会分家）。
//   读口 4 个：地址由 idu2 用【已寄存的 sel_slot*_q】造（= 选优拍那条 ps），阵列读【组合】
//   落在【发射拍】—— 读口地址不挂在选优锥的尾巴上。发射拍组合读总比选优拍寄存读【晚一个沿】，
//   因此"生产者上一拍出结果"的那一格天然被覆盖，无需写穿。
//   初值与 regfile.v 同款：initial 全 0，不接 rst（错开复位会让两个半区与 regfile 分家）。
//////////////////////////////////////////////////////////////////////////////////

module prf(
    input clk,
//---- 提交口（架构半区）：与 regfile.v 的写口逐字同源 ----
    input        we_a, input [4:0] rd_a, input [31:0] data_a,
    input        we_b, input [4:0] rd_b, input [31:0] data_b,
//---- 结果口（在飞半区）：acc_* = rob 的接受条件（we & ent_v & 世代匹配）----
    input        acc_alu,  input [3:0] idx_alu,  input gen_alu,  input [31:0] data_alu,
    input        acc_alu2, input [3:0] idx_alu2, input gen_alu2, input [31:0] data_alu2,
    input        acc_mul,  input [3:0] idx_mul,  input gen_mul,  input [31:0] data_mul,
    input        acc_ld,   input [3:0] idx_ld,   input gen_ld,   input [31:0] data_ld,
//---- 读口地址（发射拍给出：= ent_ps*[sel_slot*_q]）----
    input [5:0]  raddr1, raddr2, raddr3, raddr4,
    output [31:0] rdata1, rdata2, rdata3, rdata4
    );

    wire [5:0] pt_alu  = {1'b1, idx_alu,  gen_alu};
    wire [5:0] pt_alu2 = {1'b1, idx_alu2, gen_alu2};
    wire [5:0] pt_mul  = {1'b1, idx_mul,  gen_mul};
    wire [5:0] pt_ld   = {1'b1, idx_ld,   gen_ld};

    reg [31:0] mem [0:63];

    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1) mem[i] = 32'd0;
    end

//读口：阵列组合读（发射拍）。地址 = 选优拍那条的 ps，已由 idu2 寄存成 sel_slot*_q ⇒ 读不在选优锥上。
    assign rdata1 = mem[raddr1];
    assign rdata2 = mem[raddr2];
    assign rdata3 = mem[raddr3];
    assign rdata4 = mem[raddr4];

//写口：先提交（架构半区）、后结果（在飞半区）—— 地址空间不相交，语句顺序无关
    always @(posedge clk) begin
        if (we_a)    mem[{1'b0, rd_a}] <= data_a;
        if (we_b)    mem[{1'b0, rd_b}] <= data_b;
        if (acc_alu) mem[pt_alu]  <= data_alu;
        if (acc_alu2)mem[pt_alu2] <= data_alu2;
        if (acc_mul) mem[pt_mul]  <= data_mul;
        if (acc_ld)  mem[pt_ld]   <= data_ld;
    end

endmodule
