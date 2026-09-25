`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/08/28 16:20:14
// Design Name: 
// Module Name: csr
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


module csr(
    input clk, rst,
    input csr_wr_en, iret, exti, timi, softi, trap, exc_bju, exc,
    input [9:0] flag_bus,
    input mem_inflight,
    input [11:0] csr_addr,
//读口地址：提前到 c2 级，由 decoder 组合透传（与 csr_addr 同源同语义，非 SYSTEM 已清 0）
    input [11:0] csr_addr_pre,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
    input [3:0] exc_cause,
    input [31:0] exc_pc, exc_tval,
    input [1:0] ird_tmr,
    input [3:0] irq_bubble,
    input jalr_fail, br2, br3,
    output reg [31:0] csr_data_out, isr_addr2, mcause, iret_addr2,
    output reg irq_act, irq_processing
    );

    reg irq_en_reg, irq_en_post_reg, eirq_en, tirq_en, sirq_en;
    reg eirq_pend, tirq_pend, sirq_pend, irq_process, global_pend;
    reg [31:0] isr_addr_reg1, isr_addr_reg2, mcause_reg, mcycle_reg, minstret_reg, mscratch_reg, mtval_reg;
    reg [31:0] iret_addr1;

//输入合流（按"上层不运算"下放至此）：停顿、中断闸门、指令退役
    reg stall, irq_gate, retire, flush_w;
    always @(*) begin
        stall    = flag_bus[5] | flag_bus[4] | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0];
        flush_w  = flag_bus[9] | flag_bus[7] | flag_bus[6];
        irq_gate = (ird_tmr != 2'd0);
//指令退役：取旧 stage==EXE 的口径 = 本拍既未冲刷也未停顿，供 minstret 计数用
        retire   = !(flush_w | stall | mem_inflight);
    end

//即将锁存的写入值（与"意图写入值" csr_data_in 区分开）：
//mtvec 的 MODE[1:0] 是 WARL，本核只实现 Direct ⇒ 写入时把低 2 位规范化为 0（BASE 保持 4 字节对齐）。
//★ 写后读旁路必须用这个值：旁路前递的是"将要写入的值"，一 masking 就会前递未屏蔽的值
//（与 misa 那处同类 —— 旁路前递的必须等于真正锁存的值）。
    reg [31:0] csr_wr_val;
    always @(*) begin
        csr_wr_val = csr_data_in;
        if (csr_addr == 12'h305) csr_wr_val = {csr_data_in[31:2], 2'b00};
    end

    always @(posedge clk) begin
        if (rst) begin
            irq_en_reg <= 1'b0;
            irq_en_post_reg <= 1'b0;
            isr_addr_reg1 <= 32'd0;
            isr_addr_reg2 <= 32'd0;
            iret_addr1 <= 32'd0;
            iret_addr2 <= 32'd0;
            mcause_reg <= 32'd0;
            mscratch_reg <= 32'd0;
            mtval_reg <= 32'd0;
            eirq_en <= 1'b0;
            tirq_en <= 1'b0;
            sirq_en <= 1'b0;
            sirq_pend <= 1'b0;
            irq_process <= 1'b0;
        end
//默认情况中断地址/使能/状态寄存器保持
        else begin
            irq_en_reg <= irq_en_reg;
            irq_en_post_reg <= irq_en_post_reg;
            isr_addr_reg1 <= isr_addr_reg1;
            isr_addr_reg2 <= isr_addr_reg2;
            mcause_reg <= mcause_reg;
            mscratch_reg <= mscratch_reg;
            mtval_reg <= mtval_reg;
            eirq_en <= eirq_en;
            tirq_en <= tirq_en;
            sirq_en <= sirq_en;
            irq_process <= irq_process;
//根据不同的csr写地址写入不同的csr寄存器
            if (csr_wr_en && !stall) begin
                case (csr_addr)
//全局使能设定
                12'h300: begin
                    irq_en_reg <= csr_data_in[3];
                    irq_en_post_reg <= csr_data_in[7];
                end
//三类型中断分别使能：规范位序 bit3=MSIE(软件)、bit7=MTIE(定时器)、bit11=MEIE(外部)
//★ 本核原来把外部/软件写反了（eirq@3、sirq@11），与自家 mcause 的中断码（外部=11/定时器=7/
//  软件=3，规范）自相矛盾 ⇒ 按规范对调。定时器那条本来就对。
                12'h304: begin
                    eirq_en <= csr_data_in[11];
                    tirq_en <= csr_data_in[7];
                    sirq_en <= csr_data_in[3];
                end
//isr跳转目标设定
                12'h305: begin
                    isr_addr_reg1 <= csr_wr_val;
                    isr_addr_reg2 <= csr_wr_val;
                end
                12'h340: begin
                    mscratch_reg <= csr_data_in;
                end
//isr返回目标设定
                12'h341: begin
                    iret_addr1 <= csr_data_in;
                    iret_addr2 <= csr_data_in;
                end
                12'h342: begin
//中断/异常原因寄存器
                    mcause_reg <= csr_data_in;
                end
                12'h343: begin
                    mtval_reg <= csr_data_in;
                end
                12'h344: begin
//软件中断挂起：按规范在 bit3（MSIP）；写 1 清挂起
                    if (csr_data_in[3]) sirq_pend <= 1'b0;
                end
                endcase
            end
//trap信号控制的系统异常处理
            if (exc) begin
                iret_addr1 <= exc_pc;
                iret_addr2 <= exc_pc;
                mcause_reg <= {28'd0, exc_cause};
                mtval_reg <= exc_tval;
                irq_en_post_reg <= irq_en_reg;
                irq_en_reg <= 1'b0;
                irq_process <= 1'b1;
            end
//非isr状态下触发中断，保存上下文并使能受理信号
            else if (irq_act && !irq_process) begin
                iret_addr1 <= pc_addr_in - irq_bubble;
                iret_addr2 <= pc_addr_in - irq_bubble;
                if (eirq_en && eirq_pend) begin
                    mcause_reg <= 32'h8000000B;
                end
                else if (tirq_en && tirq_pend) begin
                    mcause_reg <= 32'h80000007;
                end
                else if (sirq_en && sirq_pend) begin
                    mcause_reg <= 32'h80000003;
                end
                irq_en_post_reg <= irq_en_reg;
                irq_en_reg <= 1'b0;
                irq_process <= 1'b1;
            end
//isr返回（目前只支持机器模式）
            if (iret) begin
                irq_en_reg <= irq_en_post_reg;
                irq_en_post_reg <= 1'b1;
                irq_process <= 1'b0;
            end
        end
    end

//指令退役逻辑，主要用于调试和性能测试
    always @(posedge clk) begin
        if (rst) begin
            mcycle_reg <= 32'd0;
            minstret_reg <= 32'd0;
        end
        else begin
            mcycle_reg <= mcycle_reg + 32'd1;
            if (retire) minstret_reg <= minstret_reg + 32'd1;
        end
    end

//csr读操作逻辑
    always @(*) begin
        if (rst) begin
            global_pend = 1'b0;
            irq_act = 1'b0;
            irq_processing = 1'b0;
            mcause = 32'd0;
            isr_addr2 = 32'd0;
            tirq_pend = 1'b0;
            eirq_pend = 1'b0;
        end
        else begin
            tirq_pend = timi;
            eirq_pend = exti;
            global_pend = (eirq_pend && eirq_en) | (tirq_pend && tirq_en) | (sirq_pend && sirq_en);
            irq_act = irq_en_reg && global_pend && !irq_process && !irq_gate && !(iret | trap | exc_bju | jalr_fail | br2 | br3);
            irq_processing = irq_process;
            mcause = mcause_reg;
            isr_addr2 = isr_addr_reg2;
        end
    end

//csr 读操作：地址取 c2 级的 csr_addr_pre（比 csr_addr 早一拍），结果寄存一拍后供 alu 当 cs_data。
//原设计在 c3 级组合读出，读值要经 alu 直通再喂 forw 的 back1 旁路，于是
//"csr mux → alu → forw → 判定比较"整条链落在同一拍里 —— 那正是全片最差路径的头（实测入口网 1.016ns）。
//提前一拍读出并寄存后，cs_data 的源头变成触发器，链头整段消失；forw/decoder/流水线结构都不用动。
//写后读旁路：本拍 c3 级正要写的 csr（csr_addr/csr_wr_en/csr_data_in）若与本级要读的地址相同，
//直接前递写入值 —— 否则提前读会拿到写之前的老值。
//两个地址都出自 decoder 且非 SYSTEM 已清 0，故不会因为"双方恰好都是 0"而误命中。
    reg [31:0] csr_data_rd;
//旁路资格：只有【真的会锁存】的地址才能前递"将要写入的值"。
//否则"先把值写进一个不锁存的 CSR、紧接着读它"会读到那个被丢弃的写入值（实测 misa 读到写入值）。
//本核不锁存的已实现地址：0x301 misa（只读常量）、0xB00/0xB02（只读计数器）。
//可写地址都在 0x300~0x3FF（[11:8]==0011），只需再把 0x301 摘出去。
    reg csr_latch_w;
    always @(*) begin
        csr_latch_w = 1'b0;
        if (csr_addr[11:8] == 4'b0011) csr_latch_w = 1'b1;
        if (csr_addr == 12'h301) csr_latch_w = 1'b0;
    end
    always@ (*) begin
        if (csr_wr_en && csr_latch_w && (csr_addr == csr_addr_pre)) begin
            csr_data_rd = csr_wr_val;
        end
        else begin
            case (csr_addr_pre)
            12'h300: csr_data_rd = {20'd0, irq_en_post_reg, 3'd0, irq_en_reg, 3'd0};
//misa：MXL[31:30]=01（32 位）+ I[8] + M[12]，只读常量。
//★ 读回 0 是不合规的（MXL=00 是保留编码），arch-test 的启动宏会读它。写被忽略（WARL）。
            12'h301: csr_data_rd = 32'h40001100;
            12'h304: csr_data_rd = {20'd0, eirq_en, 3'd0, tirq_en, 3'd0, sirq_en, 3'd0};
            12'h305: csr_data_rd = isr_addr_reg1;
            12'h340: csr_data_rd = mscratch_reg;
            12'h341: csr_data_rd = iret_addr1;
            12'h342: csr_data_rd = mcause_reg;
            12'h343: csr_data_rd = mtval_reg;
            12'h344: csr_data_rd = {20'd0, eirq_pend, 3'd0, tirq_pend, 3'd0, sirq_pend, 3'd0};
            12'hB00: csr_data_rd = mcycle_reg;
            12'hB02: csr_data_rd = minstret_reg;
//mhartid：单 hart 恒 0，只读（地址 bit[11:10]==11 ⇒ post_decoder 会挡掉对它的写）
            12'hF14: csr_data_rd = 32'd0;
            default: csr_data_rd = 32'd0;
            endcase
        end
    end

//读值寄存器：与上面同拍采样，故 cs_data 相对 c2 级地址晚一拍可用，正好落在该指令进 alu 的那一拍
    always@ (posedge clk) begin
        if (rst) csr_data_out <= 32'd0;
        else     csr_data_out <= csr_data_rd;
    end
endmodule 
