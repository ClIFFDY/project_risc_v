`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/25
// Design Name:
// Module Name: trap_unit
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


module trap_unit(
    input [9:0] flag_bus,
//E5 级（bju 判定延迟拍）异常请求：判定结果与随指令携带的 PC 载荷，都来自寄存器
    input exc_bju_in,
    input [31:0] exc_pc_e5_in,
    input [31:0] exc_tval_e5_in,
//E4 级异常请求：ecall/ebreak 由 post_decoder 的寄存器给出
    input exc_e4_in,
    input exc_ebreak_e4_in,
    input exc_illegal_e4_in,
    input exc_ldst_misalign_e4_in,
    input exc_ldst_st_e4_in,
    input [31:0] exc_ldst_addr_e4_in,
    input [31:0] exc_pc_e4_in,
//仲裁结果
    output reg exc,
    output reg [3:0] exc_cause,
    output reg [31:0] exc_pc, exc_tval
    );

    localparam CAUSE_MISALIGN_INST = 4'd0;
    localparam CAUSE_ILLEGAL_INST  = 4'd2;
    localparam CAUSE_LOAD_MISALIGN  = 4'd4;
    localparam CAUSE_STORE_MISALIGN = 4'd6;
    localparam CAUSE_BREAKPOINT    = 4'd3;
    localparam CAUSE_ECALL_M       = 4'd11;

//flag_bus = {exc, exec, flush_irq, flush_jump, dcache_hold, bus_hold_in, stall_m, stall_v, lsu_stall, icache_busy}
    reg flush_older;
    always @(*) flush_older = flag_bus[7] | flag_bus[6];

    reg exc_e4_gated;
    always @(*) exc_e4_gated = (exc_e4_in | exc_illegal_e4_in | exc_ldst_misalign_e4_in) & ~flush_older;

    reg [31:0] exc_pc_c;
    always @(*) begin
        exc       = exc_bju_in | exc_e4_gated;
        exc_cause = 4'd0;
        exc_pc_c  = 32'd0;
        exc_tval  = 32'd0;
        if (exc_bju_in) begin
            exc_cause = CAUSE_MISALIGN_INST;
            exc_pc_c  = exc_pc_e5_in;
//规范口径：指令地址非对齐的 mtval = 出错的【目标地址】。三路都从 bju 的落点寄存器取：
//br 的目标 / jalr 的目标本来就在那个寄存器里；jal 的目标不在流水里，由 post_decoder
//就地解 immJ 算好后单铺一路载荷送进来，同样落在这个寄存器。
            exc_tval  = exc_tval_e5_in;
        end
        else if (exc_e4_gated) begin
            if (exc_illegal_e4_in) begin
                exc_cause = CAUSE_ILLEGAL_INST;
            end
            else if (exc_ldst_misalign_e4_in) begin
                if (exc_ldst_st_e4_in) begin
                    exc_cause = CAUSE_STORE_MISALIGN;
                end
                else begin
                    exc_cause = CAUSE_LOAD_MISALIGN;
                end
//规范口径：访存地址非对齐的 mtval = 出错的【地址】（不是指令字）
                exc_tval  = exc_ldst_addr_e4_in;
            end
            else begin
                exc_cause = exc_ebreak_e4_in ? CAUSE_BREAKPOINT : CAUSE_ECALL_M;
            end
            exc_pc_c  = exc_pc_e4_in;
        end
    end

    always @(*) exc_pc = exc_pc_c - 32'd4;

endmodule
