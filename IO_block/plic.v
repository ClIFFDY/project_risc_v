`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/30 21:30:00
// Design Name:
// Module Name: plic
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


module plic(
    input clk,
    input rst,
    input [31:0] irq_sources,
    output reg exti,
    //
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    output reg [31:0] bus_data_out,
    output reg ld_ready,
//---- 总线错误日志（厂商扩展，不是 PLIC 标准寄存器）----
//  实体在 soc/bus_con_top.v（它才看得见各主设备的 hresp），这里只负责
//  把这四个量摆到 0x0900_0000 窗口上，并给出"写 1 清"的清除脉冲。
    input             err_valid,
    input             err_write,
    input      [1:0]  err_owner,
    input      [31:0] err_addr,
    output reg        err_clear
    );

    reg [2:0] irq_prio [1:31];
    reg [31:1] pending;
    reg [31:1] irq_en;
    reg [2:0] threshold;

    reg [31:0] super_id;
    reg [5:0] sel_id;
    reg [2:0] max_prio;
    reg global_pending;
    reg irq_valid;

    integer i;
    integer j;
    always @(*) begin
        sel_id = 6'd0;
        global_pending = 1'd0;
        max_prio = 3'd0;
        for (j = 1; j < 32; j = j + 1) begin
            if (pending[j] && irq_en[j]) begin
                if (irq_prio[j] > max_prio) begin
                    max_prio = irq_prio[j];
                    sel_id = j;
                    global_pending = 1'd1;
                end
            end
        end
        super_id = {27'd0, sel_id[4:0]};
        irq_valid = global_pending && sel_id != 6'd0 && (irq_prio[sel_id] > threshold);
    end

    always @(posedge clk) begin
        if (rst) exti <= 1'b0;
        else     exti <= irq_valid;
    end

    always @(posedge clk) begin
        if (rst) begin
            pending <= 31'd0;
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            err_clear <= 1'b0;
            threshold <= 3'd0;
            for (i = 1; i < 32; i = i + 1) begin
                irq_prio[i] <= 3'd0;
                irq_en[i] <= 1'd0;
            end
        end
        else begin
            for (i = 1; i < 32; i = i + 1) begin
                if (irq_sources[i]) pending[i] <= 1'b1;
            end
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            err_clear <= 1'b0;
            if (bus_addr_in[31:24] == 8'd2) begin
                if (bus_addr_in[23:20] == 4'd0) begin
                    if (bus_addr_in[5:0] >= 1 && bus_addr_in[5:0] <= 31) begin
                        if (!bus_addr_in[6] && bus_we_in && bus_be_in[0]) begin
                            irq_prio[bus_addr_in[5:0]] <= bus_data_in[2:0];
                        end
                        else if (!bus_addr_in[6] && !bus_we_in) begin
                            bus_data_out <= {29'd0, irq_prio[bus_addr_in[5:0]]};
                            ld_ready <= 1'b1;
                        end
                        else if (bus_addr_in[6] && bus_we_in && bus_be_in[0]) begin
                            irq_en[bus_addr_in[5:0]] <= bus_data_in[0];
                        end
                        else if (bus_addr_in[6] && !bus_we_in) begin
                            bus_data_out <= {31'd0, irq_en[bus_addr_in[5:0]]};
                            ld_ready <= 1'b1;
                        end
                    end
                end
                else if (bus_addr_in[23:20] == 4'd1) begin
                    if (bus_we_in && bus_be_in[0]) threshold <= bus_data_in[2:0];
                    else if (!bus_we_in) begin
                        bus_data_out <= {29'd0, threshold};
                        ld_ready <= 1'b1;
                    end
                end
                else if (bus_addr_in[23:20] == 4'd2) begin
                    if (bus_addr_in[4:0] >= 1 && bus_addr_in[4:0] <= 31) begin
                        if (bus_we_in && bus_be_in[0]) pending[bus_addr_in[4:0]] <= 1'd0;
                        else if (!bus_we_in) begin
                            bus_data_out <= {31'd0, pending[bus_addr_in[4:0]]};
                            ld_ready <= 1'b1;
                        end
                    end
                end
                else if (bus_addr_in[23:20] == 4'd3) begin
                    if (!bus_we_in) begin
                        bus_data_out <= super_id;
                        ld_ready <= 1'b1;
                        if (sel_id != 6'd0) pending[sel_id] <= 1'd0;
                    end
                end
                else if (bus_addr_in[23:20] == 4'd4) begin
//错误日志：0x0900_0000 控制 / 0x0900_0004 出错地址。厂商扩展。
                    if (bus_addr_in[5:0] == 6'd0) begin
                        if (bus_we_in && bus_be_in[0] && bus_data_in[0])
                            err_clear <= 1'b1;          // 写 1 清（自清，不锁存）
                        else if (!bus_we_in) begin
                            bus_data_out <= {28'd0, err_owner, err_write, err_valid};
                            ld_ready <= 1'b1;
                        end
                    end
                    else if (bus_addr_in[5:0] == 6'd1) begin
                        if (!bus_we_in) begin
                            bus_data_out <= err_addr;
                            ld_ready <= 1'b1;
                        end
                    end
                end
            end
        end
    end
endmodule
