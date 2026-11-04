`timescale 1ns / 1ps
//==============================================================================
// prf.v —— 按物理号索引的寄存器堆（真 PRF）
//
// 与老的值表（按 ROB 槽索引）的区别：
//   · 索引是【物理号】，一条指令的源操作数在重命名那一拍就定死，不会因为槽被复用而漂。
//   · 判"就绪"用 ready 位（分配时清、写回时置、冲刷时按架构映射重置），
//     不再靠 (槽, 世代) 二元组的比较 —— 那个二元组只有 5 位，会撞。
//
// 端口账（FPGA 上每阵列只有 1 个写口，这是整份设计的约束来源）：
//   读 6 个：lane0 的 a/b、lane1 的 a/b、访存流的 a/b，纯组合。
//   写 4 个源：lane0 结果口、lane1 结果口、LOAD 返回级（晚）、LOAD s2（早）。
//   bank = 物理号最低位（lane0 只从偶数号取、lane1 只从奇数号取）
//     ⇒ lane0 结果和 lane1 结果天然落在不同 bank。
//     ⇒ LOAD 也走 lane0 的分配点，物理号恒为偶数 ⇒ 两个 LOAD 写源都只能进 bank0。
//     所以 bank0 要三个写口（结果口 + LOAD 两个时相），bank1 一个。
//   2026-11-04 改：LOAD 的两个时相共用一块阵列（三块压回两块）。
//     两个时相同拍亮只可能是【两笔不同的 load】（前一笔在返回级、后一笔在 s2），
//     写的是不同项，但一个写口只能落一笔 ⇒ 晚的赢（它是权威值，早的那笔到返回级还会再写一次）。
//     定式：早口被晚口门掉时，那个号的 ready 也不置（write_mask 用的就是 wp3_active），消费者等返回级。
//
// 为什么 LOAD 要两个写源：
//   · 早那个（stage2 & ready_in）覆盖 dcache 命中的快路 —— 少了它，消费者要等返回级，实测 +9.3% 拍。
//   · 晚那个（return_valid）是【权威】的：外设 load 根本不走 s2 那条路，只亮返回级。
//     只留早的会漏外设 load（实测 CoreMark 直接挂死）。
//
// 写口为什么还要 live：冲刷之后在飞的结果还会落完（mul 两级 / lsu 三级），
//   那些项的物理号可能已经被回收再分配。live 来自 ROB 的 "本项还有效且世代对得上"，
//   晚到的死写一律不进阵列。
//==============================================================================
module prf #(
    parameter integer PHYS_COUNT = 64,
    parameter integer PHYS_BITS  = 6
)(
    input clk, rst,

//---- 读口（6 个）----
    input  [PHYS_BITS-1:0] read_a_0_phys, read_b_0_phys,
    input  [PHYS_BITS-1:0] read_a_1_phys, read_b_1_phys,
    input  [PHYS_BITS-1:0] read_a_m_phys, read_b_m_phys,
    output [31:0] read_a_0_data, read_b_0_data,
    output [31:0] read_a_1_data, read_b_1_data,
    output [31:0] read_a_m_data, read_b_m_data,

//---- 写口 0：lane0 结果口（bank0）----
    input                 write_en_0, input write_live_0,
    input  [PHYS_BITS-1:0] write_phys_0, input [31:0] write_data_0,
//---- 写口 1：lane1 结果口（bank1）----
    input                 write_en_1, input write_live_1,
    input  [PHYS_BITS-1:0] write_phys_1, input [31:0] write_data_1,
//---- 写口 2：LOAD 返回级（bank0，权威）----
    input                 write_en_2, input write_live_2,
    input  [PHYS_BITS-1:0] write_phys_2, input [31:0] write_data_2,
//---- 写口 3：LOAD s2 早口（bank0，与写口 2 共用阵列，只为省一拍）----
    input                 write_en_3, input write_live_3,
    input  [PHYS_BITS-1:0] write_phys_3, input [31:0] write_data_3,

//---- 就绪位 ----
    output reg [PHYS_COUNT-1:0] ready,          // 1 = 值的家已经填好，可以读
    input      [PHYS_COUNT-1:0] alloc_mask,     // 本拍分配出去的号
    input                       flush,
    input      [PHYS_COUNT-1:0] flush_arch_mask // 冲刷后仍在用的号 = 架构映射那一组
    );

    localparam integer BANK_DEPTH = PHYS_COUNT / 2;

//复位就地打一拍（全核统一做法）
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//bank0：偶数号，两块阵列各一个写口（结果口 / LOAD 两个时相共用一块）
    reg [31:0] bank0_result [0:BANK_DEPTH-1];
    reg [31:0] bank0_load   [0:BANK_DEPTH-1];
//每一项最后由哪个写口写的（读的时候靠它选）：0 = 结果口，1 = LOAD 口
    reg [BANK_DEPTH-1:0] bank0_version;
//bank1：奇数号，一个写口
    reg [31:0] bank1_result [0:BANK_DEPTH-1];

    wire [PHYS_BITS-2:0] a0_idx = read_a_0_phys[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] b0_idx = read_b_0_phys[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] a1_idx = read_a_1_phys[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] b1_idx = read_b_1_phys[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] am_idx = read_a_m_phys[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] bm_idx = read_b_m_phys[PHYS_BITS-1:1];

//  ★ 函数体内读数组进不了敏感表（本仓踩过），所以读口一律写成 assign 逐项表达。
    assign read_a_0_data = read_a_0_phys[0] ? bank1_result[a0_idx] :
                           (bank0_version[a0_idx] ? bank0_load[a0_idx] : bank0_result[a0_idx]);
    assign read_b_0_data = read_b_0_phys[0] ? bank1_result[b0_idx] :
                           (bank0_version[b0_idx] ? bank0_load[b0_idx] : bank0_result[b0_idx]);
    assign read_a_1_data = read_a_1_phys[0] ? bank1_result[a1_idx] :
                           (bank0_version[a1_idx] ? bank0_load[a1_idx] : bank0_result[a1_idx]);
    assign read_b_1_data = read_b_1_phys[0] ? bank1_result[b1_idx] :
                           (bank0_version[b1_idx] ? bank0_load[b1_idx] : bank0_result[b1_idx]);
    assign read_a_m_data = read_a_m_phys[0] ? bank1_result[am_idx] :
                           (bank0_version[am_idx] ? bank0_load[am_idx] : bank0_result[am_idx]);
    assign read_b_m_data = read_b_m_phys[0] ? bank1_result[bm_idx] :
                           (bank0_version[bm_idx] ? bank0_load[bm_idx] : bank0_result[bm_idx]);

//写口使能：物理 0 是 x0，永远不写
    wire write_ok_0 = write_en_0 & write_live_0 & (|write_phys_0);
    wire write_ok_1 = write_en_1 & write_live_1 & (|write_phys_1);
    wire write_ok_2 = write_en_2 & write_live_2 & (|write_phys_2);
    wire write_ok_3 = write_en_3 & write_live_3 & (|write_phys_3);

    wire [PHYS_BITS-2:0] wp0_idx = write_phys_0[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] wp1_idx = write_phys_1[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] wp2_idx = write_phys_2[PHYS_BITS-1:1];
    wire [PHYS_BITS-2:0] wp3_idx = write_phys_3[PHYS_BITS-1:1];

//写入 bank 的项：lane0 只会给偶数号，lane1 只会给奇数号；这里仍按号判，接线错了也不会写错阵列
    wire wp0_in_bank0 = ~write_phys_0[0];
    wire wp1_in_bank1 =  write_phys_1[0];
    wire wp2_in_bank0 = ~write_phys_2[0];
    wire wp3_in_bank0 = ~write_phys_3[0];
//早口和返回级共用一块阵列：同拍都亮只可能是两笔不同的 load，晚的赢（权威值）
    wire wp3_active   = write_ok_3 & ~write_ok_2;

    integer reset_index;
    always @(posedge clk) begin
        if (reset_registered) begin
            for (reset_index = 0; reset_index < BANK_DEPTH; reset_index = reset_index + 1) begin
                bank0_result[reset_index] <= 32'd0;
                bank0_load[reset_index]   <= 32'd0;
                bank1_result[reset_index] <= 32'd0;
            end
            bank0_version <= {BANK_DEPTH{1'b0}};
        end
        else begin
            if (write_ok_0 & wp0_in_bank0) begin
                bank0_result[wp0_idx] <= write_data_0;
                bank0_version[wp0_idx] <= 1'b0;
            end
            if (write_ok_2 & wp2_in_bank0) begin
                bank0_load[wp2_idx] <= write_data_2;
                bank0_version[wp2_idx] <= 1'b1;
            end
            else if (wp3_active & wp3_in_bank0) begin
                bank0_load[wp3_idx] <= write_data_3;
                bank0_version[wp3_idx] <= 1'b1;
            end
            if (write_ok_1 & wp1_in_bank1) begin
                bank1_result[wp1_idx] <= write_data_1;
            end
        end
    end

//==============================================================================
// 就绪位
//   分配：清（新号还没有值）
//   写回：置（值到家了）
//   冲刷：回到架构映射那一组（在飞的全作废，剩下来的值都在家）
//==============================================================================
    wire [PHYS_COUNT-1:0] write_mask =
          (write_ok_0 ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << write_phys_0) : {PHYS_COUNT{1'b0}})
        | (write_ok_1 ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << write_phys_1) : {PHYS_COUNT{1'b0}})
        | (write_ok_2 ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << write_phys_2) : {PHYS_COUNT{1'b0}})
        | (wp3_active ? ({{(PHYS_COUNT-1){1'b0}}, 1'b1} << write_phys_3) : {PHYS_COUNT{1'b0}});

    always @(posedge clk) begin
        if (reset_registered) begin
//物理 0..31 就是复位后的架构映射，值全 0（与 regfile 复位同口径）
            ready <= {{(PHYS_COUNT-32){1'b0}}, 32'hFFFFFFFF};
        end
        else if (flush) begin
//★ 冲刷拍也可能同时有一笔分配（alloc 用的门里没有 flush_window）⇒ 那一笔照样要清。
            ready <= (flush_arch_mask | write_mask) & ~alloc_mask;
        end
        else begin
            ready <= (ready | write_mask) & ~alloc_mask;
        end
    end

endmodule
