`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/28 15:28:51
// Design Name:
// Module Name: itcm
// Project Name:
// Target Devices:
// Tool Versions:
// Description: 取指后备存储器，64KB。icache miss 时按行流出 32 个字。
//              不再承担取指地址生成（已移入 icache）。
//
//              读口必须是"地址寄存器 → RAM → 输出寄存器"的单一形状，否则 Vivado
//              推不出 BRAM（原写法读地址有两个表达式 mem_addr/ base+n，地址路径上
//              挂了加法器，实测 64KB 一块 BRAM 都没成、全塌成 8536 个 LUT）。
//              代价：比原来晚 1 拍出第一个字；mem_valid 与 mem_data 同拍对齐，
//              上游（icache）按 mem_valid 数拍收字，不受影响。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module itcm(
    input clk, rst,
    input mem_req,
    input [31:0] mem_addr,
    output [31:0] mem_data,
    output mem_valid
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//指令存储 64KB（自举/回填来源）
    (* ram_style = "block" *) reg [31:0] itcm [0:16383];
    integer i;
    initial begin
        for (i = 0; i < 16384; i = i + 1)
            itcm[i] = 32'd0;
        $readmemh("e:/Vivado_Projects/project_risc_v/tools/hex/ins.hex", itcm);
    end

//读口：mem_req 上升沿锁行首地址，随后 32 拍连吐一行
    reg [13:0] read_address;
    reg [5:0]  count;
    reg        address_valid;
    reg [31:0] data_registered;
    reg        valid_delayed;
    reg        request_delayed;

    reg start;
    always @(*) start = mem_req & ~request_delayed;

    always @(posedge clk) begin
        if (reset_registered) begin
            read_address <= 14'd0;
            count <= 6'd0;
            address_valid <= 1'b0;
            request_delayed <= 1'b0;
        end
        else begin
            request_delayed <= mem_req;
            if (start) begin
                read_address  <= mem_addr[15:2];
                count    <= 6'd32;
                address_valid <= 1'b1;
            end
            else if (count != 6'd0) begin
                read_address  <= read_address + 14'd1;
                count    <= count - 6'd1;
                address_valid <= 1'b1;
            end
            else
                address_valid <= 1'b0;
        end
    end

//单一地址表达式 + 输出寄存器：BRAM 的可推断模板
    always @(posedge clk) begin
        data_registered <= itcm[read_address];
        valid_delayed <= address_valid;
    end

    assign mem_data  = data_registered;
    assign mem_valid = valid_delayed;

endmodule
