`timescale 1ns / 1ps
//==============================================================================
// prf.v —— 按物理号索引的寄存器堆（真 PRF）
//
// 与老的值表（按 ROB 槽索引）的区别：
//   · 索引是【物理号】，一条指令的源操作数在重命名那一拍就定死，不会因为槽被复用而漂。
//   · 判"就绪"不在本模块：发射级自己维护（在飞表 + 每项常驻 ready_q，见 isq.v）。
//     本模块退回纯阵列：只有写口和读口，外加两个"早/晚口这一拍真的落笔了"的指示。
//
// ★ 2026-11-10 改版：阵列按【写者】拆开，目标是让 Vivado 推断出 distributed RAM（LUTRAM）。
//   老版每块阵列挂了 2~4 条写语句（Vivado 8-4767 只认一条），所以全被推成触发器：
//   6,335 LUT + 3,072 FF。实测（Tools/lutram_probe.v，xc7a100tfgg676-1 满布局布线）：
//     单写者 64x32 + 7 个异步读口       = 308 LUT，逻辑 LUT 为 0（读 mux 被 RAM 格吸收）
//     5 块阵列 + owner 表 + 5 选 1      ≈ 1,700 LUT + 192 FF
//
//   为什么不用"仲裁 + 可停写者"（NaxRiscv 的做法）：仲裁会让"写口落笔那一拍"变得不确定，
//   而发射级的静态唤醒要求每个单元的写口时刻是固定的（ALU = 发射拍、MUL = issue+2）。
//   按写者拆开就完全没有仲裁，写时刻一个字不动。
//
//   读写怎么对上：每个物理号在【分配那一拍】就绑定生产者（= 哪个写口会写它），
//   生命周期内只被写一次。所以 "物理号 -> 写口" 是一张分配时写、发射时读的表 phys_owner。
//   读口先查 owner，再从对应的那块阵列取值。
//     · 它只有 3 个写者（3 宽分配），比"跟着每次写回更新"少一半；
//     · 它是物理号索引的，所以 CSR 提交读口（read_c）也能直接用，不需要额外通路。
//
//   写者归并：LOAD 的早口（s2）和晚口（返回级）本来就靠 wp3_active 互斥，合成一条写语句。
//   ALU 早口和 MUL/DIV 结果口【不能】这样合：同一条 lane 上它们可以同拍
//   （ALU 在发射拍、MUL 在 issue+2），合了就得仲裁。
//
// 端口账：
//   读 7 个：lane0 的 a/b、lane1 的 a/b、访存流的 a/b、提交拍的 CSR 源，纯组合。
//   写 5 个：ALU0 早口 / EU0 结果口 / ALU1 早口 / EU1 结果口 / LOAD。
//
// ★ 阵列不再复位（复位循环也是 LUTRAM 推断的障碍）。代价：没写过的号读到的是不定值，
//   架构上是允许的（未写寄存器本来就未定义）；就绪位保证在写之前不会被读到。
//   物理 0 单独兜底成 0。
//
// 写口为什么还要 live：冲刷之后在飞的结果还会落完（mul 两级 / lsu 三级），
//   那些项的物理号可能已经被回收再分配。live 来自 ROB 的 "本项还有效且世代对得上"，
//   晚到的死写一律不进阵列。
//==============================================================================
module prf #(
    parameter integer PHYS_COUNT = 64,
    parameter PHYS_BITS  = 6
)(
    input clk, rst,

//---- 读口（7 个）----
    input  [PHYS_BITS-1:0] read_a_0_phys, read_b_0_phys,
    input  [PHYS_BITS-1:0] read_a_1_phys, read_b_1_phys,
    input  [PHYS_BITS-1:0] read_a_m_phys, read_b_m_phys,
//★ 2026-11-04（CSR 步骤 C）：第 7 个读口 = 提交拍的 CSR 源操作数。地址是 ROB 提交口那级触发器。
    input  [PHYS_BITS-1:0] read_c_phys,
    output [31:0] read_a_0_data, read_b_0_data,
    output [31:0] read_a_1_data, read_b_1_data,
    output [31:0] read_a_m_data, read_b_m_data,
    output [31:0] read_c_data,
    output                load_write,
    output [PHYS_BITS-1:0] load_write_phys,

//---- 分配口：这一拍新分配的物理号 + 它将来走哪个写口（owner 表就靠它写）----
    input                 alloc_write_0, input [PHYS_BITS-1:0] alloc_phys_0, input [2:0] alloc_owner_0,
    input                 alloc_write_1, input [PHYS_BITS-1:0] alloc_phys_1, input [2:0] alloc_owner_1,
    input                 alloc_write_2, input [PHYS_BITS-1:0] alloc_phys_2, input [2:0] alloc_owner_2,

//---- 写口 0：EU0 结果口（MUL / DIV）----
    input                 write_en_0, input write_live_0,
    input  [PHYS_BITS-1:0] write_phys_0, input [31:0] write_data_0,
//---- 写口 1：EU1 结果口（MUL / DIV）----
    input                 write_en_1, input write_live_1,
    input  [PHYS_BITS-1:0] write_phys_1, input [31:0] write_data_1,
//---- 写口 2：LOAD 返回级（权威）----
    input                 write_en_2, input write_live_2,
    input  [PHYS_BITS-1:0] write_phys_2, input [31:0] write_data_2,
//---- 写口 3：LOAD s2 早口（与写口 2 共用一块阵列，只为省一拍）----
    input                 write_en_3, input write_live_3,
    input  [PHYS_BITS-1:0] write_phys_3, input [31:0] write_data_3,
//---- 写口 4：ALU lane0 早口 ----
    input                 write_en_4, input write_live_4,
    input  [PHYS_BITS-1:0] write_phys_4, input [31:0] write_data_4,
//---- 写口 5：ALU lane1 早口 ----
    input                 write_en_5, input write_live_5,
    input  [PHYS_BITS-1:0] write_phys_5, input [31:0] write_data_5
    );

//---- owner 编码：必须与 idu.v 里 ALLOC_OWNER_* 一致 ----
    localparam [2:0] OWNER_ALU0 = 3'd0;   // 写口 4
    localparam [2:0] OWNER_MUL0 = 3'd1;   // 写口 0
    localparam [2:0] OWNER_ALU1 = 3'd2;   // 写口 5
    localparam [2:0] OWNER_MUL1 = 3'd3;   // 写口 1
    localparam [2:0] OWNER_LOAD = 3'd4;   // 写口 2/3

//复位就地打一拍（全核统一做法）
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//==============================================================================
// 阵列：五块，每块只有一个写者 ⇒ 每块只有一条写语句
//==============================================================================
    (* ram_style = "distributed" *) reg [31:0] array_alu0 [0:PHYS_COUNT-1];
    (* ram_style = "distributed" *) reg [31:0] array_mul0 [0:PHYS_COUNT-1];
    (* ram_style = "distributed" *) reg [31:0] array_alu1 [0:PHYS_COUNT-1];
    (* ram_style = "distributed" *) reg [31:0] array_mul1 [0:PHYS_COUNT-1];
    (* ram_style = "distributed" *) reg [31:0] array_load [0:PHYS_COUNT-1];

//写口使能：物理 0 是 x0，永远不写
    wire write_ok_0 = write_en_0 & write_live_0 & (|write_phys_0);
    wire write_ok_1 = write_en_1 & write_live_1 & (|write_phys_1);
    wire write_ok_2 = write_en_2 & write_live_2 & (|write_phys_2);
    wire write_ok_3 = write_en_3 & write_live_3 & (|write_phys_3);
    wire write_ok_4 = write_en_4 & write_live_4 & (|write_phys_4);
    wire write_ok_5 = write_en_5 & write_live_5 & (|write_phys_5);

//早口和返回级共用一块阵列：同拍都亮只可能是两笔不同的 load，晚的赢（权威值）。
//合成一条写语句 —— 这正是 LUTRAM 推断要求的形状。
    wire        wp3_active      = write_ok_3 & ~write_ok_2;

    assign load_write           = write_ok_2 | wp3_active;
    assign load_write_phys      = write_ok_2 ? write_phys_2 : write_phys_3;
    wire [31:0] load_write_data = write_ok_2 ? write_data_2 : write_data_3;

    always @(posedge clk) if (write_ok_4) array_alu0[write_phys_4] <= write_data_4;
    always @(posedge clk) if (write_ok_0) array_mul0[write_phys_0] <= write_data_0;
    always @(posedge clk) if (write_ok_5) array_alu1[write_phys_5] <= write_data_5;
    always @(posedge clk) if (write_ok_1) array_mul1[write_phys_1] <= write_data_1;
    always @(posedge clk) if (load_write)  array_load[load_write_phys] <= load_write_data;

//==============================================================================
// owner 表：物理号 -> 它会由哪个写口写。分配那一拍定，之后不再改。
//==============================================================================
    reg [2:0] phys_owner [0:PHYS_COUNT-1];

    always @(posedge clk) begin
        if (alloc_write_0) phys_owner[alloc_phys_0] <= alloc_owner_0;
        if (alloc_write_1) phys_owner[alloc_phys_1] <= alloc_owner_1;
        if (alloc_write_2) phys_owner[alloc_phys_2] <= alloc_owner_2;
    end

//==============================================================================
// 读口：先查 owner 选阵列，再取值。物理 0 恒为 0（阵列不复位，不兜底会读到不定值）。
//  ★ 函数体内读数组进不了敏感表（本仓踩过），所以读口一律写成 assign 逐项表达。
//==============================================================================
    wire [2:0] own_a_0 = phys_owner[read_a_0_phys];
    wire [2:0] own_b_0 = phys_owner[read_b_0_phys];
    wire [2:0] own_a_1 = phys_owner[read_a_1_phys];
    wire [2:0] own_b_1 = phys_owner[read_b_1_phys];
    wire [2:0] own_a_m = phys_owner[read_a_m_phys];
    wire [2:0] own_b_m = phys_owner[read_b_m_phys];
    wire [2:0] own_c   = phys_owner[read_c_phys];

    assign read_a_0_data = (|read_a_0_phys) ?
        ((own_a_0==OWNER_ALU0) ? array_alu0[read_a_0_phys] :
         (own_a_0==OWNER_MUL0) ? array_mul0[read_a_0_phys] :
         (own_a_0==OWNER_ALU1) ? array_alu1[read_a_0_phys] :
         (own_a_0==OWNER_MUL1) ? array_mul1[read_a_0_phys] : 
         array_load[read_a_0_phys]) : 32'd0;
    assign read_b_0_data = (|read_b_0_phys) ?
        ((own_b_0==OWNER_ALU0) ? array_alu0[read_b_0_phys] :
         (own_b_0==OWNER_MUL0) ? array_mul0[read_b_0_phys] :
         (own_b_0==OWNER_ALU1) ? array_alu1[read_b_0_phys] :
         (own_b_0==OWNER_MUL1) ? array_mul1[read_b_0_phys] : 
         array_load[read_b_0_phys]) : 32'd0;
    assign read_a_1_data = (|read_a_1_phys) ?
        ((own_a_1==OWNER_ALU0) ? array_alu0[read_a_1_phys] :
         (own_a_1==OWNER_MUL0) ? array_mul0[read_a_1_phys] :
         (own_a_1==OWNER_ALU1) ? array_alu1[read_a_1_phys] :
         (own_a_1==OWNER_MUL1) ? array_mul1[read_a_1_phys] : 
         array_load[read_a_1_phys]) : 32'd0;
    assign read_b_1_data = (|read_b_1_phys) ?
        ((own_b_1==OWNER_ALU0) ? array_alu0[read_b_1_phys] :
         (own_b_1==OWNER_MUL0) ? array_mul0[read_b_1_phys] :
         (own_b_1==OWNER_ALU1) ? array_alu1[read_b_1_phys] :
         (own_b_1==OWNER_MUL1) ? array_mul1[read_b_1_phys] : 
         array_load[read_b_1_phys]) : 32'd0;
    assign read_a_m_data = (|read_a_m_phys) ?
        ((own_a_m==OWNER_ALU0) ? array_alu0[read_a_m_phys] :
         (own_a_m==OWNER_MUL0) ? array_mul0[read_a_m_phys] :
         (own_a_m==OWNER_ALU1) ? array_alu1[read_a_m_phys] :
         (own_a_m==OWNER_MUL1) ? array_mul1[read_a_m_phys] : 
         array_load[read_a_m_phys]) : 32'd0;
    assign read_b_m_data = (|read_b_m_phys) ?
        ((own_b_m==OWNER_ALU0) ? array_alu0[read_b_m_phys] :
         (own_b_m==OWNER_MUL0) ? array_mul0[read_b_m_phys] :
         (own_b_m==OWNER_ALU1) ? array_alu1[read_b_m_phys] :
         (own_b_m==OWNER_MUL1) ? array_mul1[read_b_m_phys] : 
         array_load[read_b_m_phys]) : 32'd0;
    assign read_c_data   = (|read_c_phys)   ?
        ((own_c==OWNER_ALU0) ? array_alu0[read_c_phys] :
         (own_c==OWNER_MUL0) ? array_mul0[read_c_phys] :
         (own_c==OWNER_ALU1) ? array_alu1[read_c_phys] :
         (own_c==OWNER_MUL1) ? array_mul1[read_c_phys] : 
         array_load[read_c_phys]) : 32'd0;

endmodule
