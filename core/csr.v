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
    input csr_wr_en, exc_irq_ret, exti, timi, softi, exc_ecall, flush_bju_exc, flush_con_exc,
    input exc_mark_in,
//站里至少有一个有效项：中断的 mepc/冲刷边界都取"站里最老有效项"，站空时那个号是陈旧槽
    input irq_head_v_in,
//★ 真写使能（idu2 产）：csrrw/rs/rc 里只有"rs1 字段非 0"才真锁存（规范 §Zicsr 的
//  "写"定义）。csr_wr_en 是【双重身份】—— alu 靠它判"这是条 CSR 指令"把读值选进 rd，
//  所以 csrr 也必须为 1；真正锁不锁由本线决定。原来靠"把 csr_addr 清 0"来表达不写，
//  但地址要同时供读口用，站式之后读口没有第二条地址通路了，必须把两件事拆成两根线。
    input csr_wr_act,
//★ 资格版（cont 产）：mret 的使能恢复必须与 pc 侧的排队武装同一根线，
//  否则会出现「跳到 mepc 但使能没恢复」的半生效。:215 的互斥仍用裸线（那是同拍不许受理中断）。
    input exc_irq_ret_ok,
    input [7:0] flag_bus,
    input mem_inflight,
    input [11:0] csr_addr,
    input [31:0] csr_data_in,
    input [31:0] pc_addr_in,
//异常【交付】口（来自 ROB 的 trap_fire）：故障项退到队头那一拍才拉，载荷是它自己那份。
//检测那一拍（flush_con_exc）只负责挡中断，不锁 mepc/mcause/mtval —— 详见下方时序块里的注释。
    input exc_retire,
    input [3:0]  exc_retire_cause,
    input [31:0] exc_retire_pc, exc_retire_tval,
    input [1:0] ird_tmr,
    input jalr_fail, br2, br3,
    output reg [31:0] isr_addr2, mcause, iret_addr2,
//给 alu 的读值（组合、地址 = 本拍要发的 csr_addr）：站式之后 csr 指令在哪一拍发由站决定，
//提前一拍读不再可能 ⇒ 改成"发的那一拍组合读出"。写锁存仍走时钟沿，故同拍读回的是【旧值】。
    output reg [31:0] csr_data_comb,
    output reg exc_irq_act, exc_irq_processing
    );

    reg exc_irq_en_reg, exc_irq_en_post_reg, exc_eirq_en, exc_tirq_en, exc_sirq_en;
    reg exc_eirq_pend, exc_tirq_pend, exc_sirq_pend, exc_irq_process, exc_global_pend;
    reg [31:0] isr_addr_reg1, isr_addr_reg2, mcause_reg, mcycle_reg, minstret_reg, mscratch_reg, mtval_reg;
    reg [31:0] iret_addr1;

//输入合流（按"上层不运算"下放至此）：停顿、中断闸门、指令退役
    reg stall, exc_irq_gate, retire, flush_w;
    always @(*) begin
        stall    = flag_bus[3] | flag_bus[2] | flag_bus[0];
        flush_w  = flush_con_exc | flag_bus[6] | flag_bus[5];
        exc_irq_gate = (ird_tmr != 2'd0);
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
        if (csr_addr == 12'h305)
            csr_wr_val = {csr_data_in[31:2], 2'b00};
    end

    always @(posedge clk) begin
        if (rst) begin
            exc_irq_en_reg <= 1'b0;
            exc_irq_en_post_reg <= 1'b0;
            isr_addr_reg1 <= 32'd0;
            isr_addr_reg2 <= 32'd0;
            iret_addr1 <= 32'd0;
            iret_addr2 <= 32'd0;
            mcause_reg <= 32'd0;
            mscratch_reg <= 32'd0;
            mtval_reg <= 32'd0;
            exc_eirq_en <= 1'b0;
            exc_tirq_en <= 1'b0;
            exc_sirq_en <= 1'b0;
            exc_sirq_pend <= 1'b0;
            exc_irq_process <= 1'b0;
        end
//默认情况中断地址/使能/状态寄存器保持
        else begin
            exc_irq_en_reg <= exc_irq_en_reg;
            exc_irq_en_post_reg <= exc_irq_en_post_reg;
            isr_addr_reg1 <= isr_addr_reg1;
            isr_addr_reg2 <= isr_addr_reg2;
            mcause_reg <= mcause_reg;
            mscratch_reg <= mscratch_reg;
            mtval_reg <= mtval_reg;
            exc_eirq_en <= exc_eirq_en;
            exc_tirq_en <= exc_tirq_en;
            exc_sirq_en <= exc_sirq_en;
            exc_irq_process <= exc_irq_process;
//根据不同的csr写地址写入不同的csr寄存器
//★ 使能与地址由 alu 的寄存级给出（= 真的发出去过的那一组），故这里不再看停顿；只挡"发出后同拍被冲刷"。
            if (csr_wr_act && !flush_w) begin
                case (csr_addr)
//全局使能设定
                12'h300: begin
                    exc_irq_en_reg <= csr_data_in[3];
                    exc_irq_en_post_reg <= csr_data_in[7];
                end
//三类型中断分别使能：规范位序 bit3=MSIE(软件)、bit7=MTIE(定时器)、bit11=MEIE(外部)
//★ 本核原来把外部/软件写反了（eirq@3、sirq@11），与自家 mcause 的中断码（外部=11/定时器=7/
//  软件=3，规范）自相矛盾 ⇒ 按规范对调。定时器那条本来就对。
                12'h304: begin
                    exc_eirq_en <= csr_data_in[11];
                    exc_tirq_en <= csr_data_in[7];
                    exc_sirq_en <= csr_data_in[3];
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
                    iret_addr1 <= {csr_data_in[31:2], 2'b00};
                    iret_addr2 <= {csr_data_in[31:2], 2'b00};
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
                    if (csr_data_in[3])
                        exc_sirq_pend <= 1'b0;
                end
                endcase
            end
//异常【检测】那一拍：只做"关中断 + 置受理态"。真正的锁存等 ROB 把故障项退到队头（下面那条）。
//★ 为什么不能在这里锁 mepc/mcause/mtval：先报出来的异常未必是最老的异常 —— 更老的指令可能
//  还在 lsu 队列里没报（lsu 入口才判非对齐，队列里的那条要到下一拍才报）。检测拍就锁，会锁成
//  "年轻的那条"，而 ROB 是按老的那条交付的 ⇒ 两个侧面对不上。改为按 ROB 的交付口锁，才精确。
            if (exc_mark_in) begin
                exc_irq_en_post_reg <= exc_irq_en_reg;
                exc_irq_en_reg <= 1'b0;
                exc_irq_process <= 1'b1;
            end
//非isr状态下触发中断，保存上下文并使能受理信号
//  mepc 取 pc_addr_in - 4：cpu_top 把 pc_addr_in 接的是 mid_decoder 的 aux_addr_2
//  （那条指令"地址 + 4"），也就是【正要进发射级、还没被发出去】的那一条 = 中断返回点。
//  旧写法是"取指 PC 减一个气泡计数"，ROB 之后前端在等 ROB 排空期间还在取指，那个计数已对不上。
            else if (exc_irq_act && !exc_irq_process) begin
                iret_addr1 <= pc_addr_in - 32'd4;
                iret_addr2 <= pc_addr_in - 32'd4;
                if (exc_eirq_en && exc_eirq_pend) begin
                    mcause_reg <= 32'h8000000B;
                end
                else if (exc_sirq_en && exc_sirq_pend) begin
                    mcause_reg <= 32'h80000003;
                end
                else if (exc_tirq_en && exc_tirq_pend) begin
                    mcause_reg <= 32'h80000007;
                end
                exc_irq_en_post_reg <= exc_irq_en_reg;
                exc_irq_en_reg <= 1'b0;
                exc_irq_process <= 1'b1;
            end
//异常【交付】：ROB 的队头是那条故障指令（比它老的都退完了、比它年轻的已作废）⇒ 这一拍锁上下文。
//  与上面两条不冲突：受理态（irq_process）一置起，中断那条就不可能再进来。
            if (exc_retire) begin
                iret_addr1 <= exc_retire_pc;
                iret_addr2 <= exc_retire_pc;
                mcause_reg <= {28'd0, exc_retire_cause};
                mtval_reg <= exc_retire_tval;
            end
//isr返回（目前只支持机器模式）
            if (exc_irq_ret_ok) begin
                exc_irq_en_reg <= exc_irq_en_post_reg;
                exc_irq_en_post_reg <= 1'b1;
                exc_irq_process <= 1'b0;
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
            if (retire)
                minstret_reg <= minstret_reg + 32'd1;
        end
    end

//csr读操作逻辑
    always @(*) begin
        if (rst) begin
            exc_global_pend = 1'b0;
            exc_irq_act = 1'b0;
            exc_irq_processing = 1'b0;
            mcause = 32'd0;
            isr_addr2 = 32'd0;
            exc_tirq_pend = 1'b0;
            exc_eirq_pend = 1'b0;
        end
        else begin
            exc_tirq_pend = timi;
            exc_eirq_pend = exti;
            exc_global_pend = (exc_eirq_pend && exc_eirq_en) | (exc_tirq_pend && exc_tirq_en) | (exc_sirq_pend && exc_sirq_en);
//★ 同拍只要有任何异常在交付路径上，中断一律让位：flush_con_exc 已经把
//  ecall/ebreak/illegal（经 bju 输入级）与访存非对齐（经 lsu 寄存拍）全含进来，
//  所以不必在这里逐个列 —— 靠 else-if 优先级兜正确性是脆的。
            exc_irq_act = exc_irq_en_reg && exc_global_pend && !exc_irq_process && !exc_irq_gate &&
                          !(exc_irq_ret | jalr_fail | br2 | br3) && !flush_con_exc && irq_head_v_in;
            exc_irq_processing = exc_irq_process;
            mcause = mcause_reg;
            isr_addr2 = isr_addr_reg2;
        end
    end

//给 alu 的组合读口：地址取本拍要发的 csr_addr（与写地址同源）。不做写后读旁路 ——
//站式之后每条 csr 指令各占一个发射拍，前一条的写在前一沿就已落进寄存器，本条组合读到
//的就是它，不需要旁路；没有旁路也顺带修掉"读只读常量表（misa）读到写入值"那类问题。
    always @(*) begin
        case (csr_addr)
        12'h300: csr_data_comb = {19'd0, 2'b11, 3'd0, exc_irq_en_post_reg, 3'd0, exc_irq_en_reg, 3'd0};
        12'h301: csr_data_comb = 32'h40001100;
        12'h304: csr_data_comb = {20'd0, exc_eirq_en, 3'd0, exc_tirq_en, 3'd0, exc_sirq_en, 3'd0};
        12'h305: csr_data_comb = isr_addr_reg1;
        12'h340: csr_data_comb = mscratch_reg;
        12'h341: csr_data_comb = iret_addr1;
        12'h342: csr_data_comb = mcause_reg;
        12'h343: csr_data_comb = mtval_reg;
        12'h344: csr_data_comb = {20'd0, exc_eirq_pend, 3'd0, exc_tirq_pend, 3'd0, exc_sirq_pend, 3'd0};
        12'hB00: csr_data_comb = mcycle_reg;
        12'hB02: csr_data_comb = minstret_reg;
        12'hF14: csr_data_comb = 32'd0;
        default: csr_data_comb = 32'd0;
        endcase
    end

endmodule 
