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
    input exc_irq_ret, exti, timi, softi, exc_ecall, flush_control_exception,
    input exception_mark_in,
//站里至少有一个有效项：中断的 mepc/冲刷边界都取"站里最老有效项"，站空时那个号是陈旧槽
    input interrupt_head_valid_in,
//★ 资格版（cont 产）：mret 的使能恢复必须与 pc 侧的排队武装同一根线，
//  否则会出现「跳到 mepc 但使能没恢复」的半生效。:215 的互斥仍用裸线（那是同拍不许受理中断）。
    input exception_interrupt_return_ok,
//★ 2026-10-31：只接真正用到的两位（本模块从总线上只取中断与改向）。
    input [1:0] flush_control_in,   // = flag_bus[6:5]：{中断, 改向}
//ROB 队头指针：minstret 的真口径 —— 它这一拍推进了几格就是退休了几条（一拍最多 2 条）
// P1: no head pointer. Only the retire step published by the ROB.
    input [1:0] retire_step_in,
    input [11:0] csr_address,
//★ 2026-11-04（CSR 步骤 C）：写口搬出执行级，改成【提交拍】落地。地址 / funct3 / 源都由 ROB 项带来，
//  提交那一拍各是一级触发器 ⇒ 「读侧 12:1 case → ALU 锥 → 写口」整条链从执行级消失。
//  同时至多一条 CSR 在飞（idu2 的 csr_held 要求它发射时就是队头），所以提交拍读到的就是它的旧值。
    input        csr_commit_valid,
    input [11:0] csr_commit_address,
    input [2:0]  csr_commit_function3,
    input [31:0] csr_commit_source,  // 寄存器形式 = PRF 按 rs1 物理号读出；立即数形式 = {27'd0, zimm}
    input [31:0] pc_address_in,
//异常【交付】口（来自 ROB 的 trap_fire）：故障项退到队头那一拍才拉，载荷是它自己那份。
//检测那一拍（flush_control_exception）只负责挡中断，不锁 mepc/mcause/mtval —— 详见下方时序块里的注释。
    input exception_retire,
    input [3:0]  exception_retire_cause,
    input [31:0] exception_retire_pc, exc_retire_tval,
    input [1:0] interrupt_request_timer,
    input jump_link_register_failure, br2, br3,
    output reg [31:0] interrupt_service_address_2, mcause, interrupt_return_address_2,
//给 alu 的读值（组合、地址 = 本拍要发的 csr_address）：站式之后 csr 指令在哪一拍发由站决定，
//提前一拍读不再可能 ⇒ 改成"发的那一拍组合读出"。写锁存仍走时钟沿，故同拍读回的是【旧值】。
    output reg [31:0] csr_data_combinational,
    output reg exception_interrupt_action, exception_interrupt_processing
    );

    reg exception_interrupt_enable_registered, exception_interrupt_enable_post_registered, exception_external_interrupt_enable, exception_timer_interrupt_enable, exception_software_interrupt_enable;
    reg exception_external_interrupt_pending, exception_timer_interrupt_pending, exception_software_interrupt_pending, exception_interrupt_process, exception_global_pending;
    reg [31:0] isr_addr_reg1, isr_addr_reg2, mcause_registered, mcycle_registered, minstret_registered, mscratch_registered, mtval_registered;
    reg [31:0] interrupt_return_address_1;

//输入合流（按"上层不运算"下放至此）：中断闸门
    reg exception_interrupt_gate;
    always @(*) begin
        exception_interrupt_gate = (interrupt_request_timer != 2'd0);
    end

//CSR 读表写两份（执行级一份、提交口一份），不是偷懒：
//  ★ 本仓实测过，iverilog 的 @* 不把【函数体内读的信号】算进敏感表（见记忆
//    iverilog-function-body-not-in-sensitivity）。写成 function 再在两个 always @(*) 里调用，
//    那些 CSR 寄存器变化时两个读口都不会重算 ⇒ 仿真读到陈旧值。所以只能逐项展开。

//读-改-写（提交拍）：function3[1:0] = 01 写 / 10 置位 / 11 清位（[2] 是立即数形式，二选一已在源上做过）
    reg [31:0] csr_commit_read;
    reg [31:0] csr_write_intent;
    always @(*) begin
        case (csr_commit_address)
        12'h300: csr_commit_read = {19'd0, 2'b11, 3'd0, exception_interrupt_enable_post_registered, 3'd0, exception_interrupt_enable_registered, 3'd0};
        12'h301: csr_commit_read = 32'h40001100;
        12'h304: csr_commit_read = {20'd0, exception_external_interrupt_enable, 3'd0, exception_timer_interrupt_enable, 3'd0, exception_software_interrupt_enable, 3'd0};
        12'h305: csr_commit_read = isr_addr_reg1;
        12'h340: csr_commit_read = mscratch_registered;
        12'h341: csr_commit_read = interrupt_return_address_1;
        12'h342: csr_commit_read = mcause_registered;
        12'h343: csr_commit_read = mtval_registered;
        12'h344: csr_commit_read = {20'd0, exception_external_interrupt_pending, 3'd0, exception_timer_interrupt_pending, 3'd0, exception_software_interrupt_pending, 3'd0};
        12'hB00: csr_commit_read = mcycle_registered;
        12'hB02: csr_commit_read = minstret_registered;
        12'hF14: csr_commit_read = 32'd0;
        default: csr_commit_read = 32'd0;
        endcase
        case (csr_commit_function3[1:0])
        2'b01: csr_write_intent = csr_commit_source;
        2'b10: csr_write_intent = csr_commit_source | csr_commit_read;
        2'b11: csr_write_intent = csr_commit_read & ~csr_commit_source;
        default: csr_write_intent = csr_commit_read;
        endcase
    end

//即将锁存的写入值（与"意图写入值" csr_write_intent 区分开）：
//mtvec 的 MODE[1:0] 是 WARL，本核只实现 Direct ⇒ 写入时把低 2 位规范化为 0（BASE 保持 4 字节对齐）。
//★ 写后读旁路必须用这个值：旁路前递的是"将要写入的值"，一 masking 就会前递未屏蔽的值
//（与 misa 那处同类 —— 旁路前递的必须等于真正锁存的值）。
    reg [31:0] csr_write_value;
    always @(*) begin
        csr_write_value = csr_write_intent;
        if (csr_commit_address == 12'h305)
            csr_write_value = {csr_write_intent[31:2], 2'b00};
    end

    always @(posedge clk) begin
        if (rst) begin
            exception_interrupt_enable_registered <= 1'b0;
            exception_interrupt_enable_post_registered <= 1'b0;
            isr_addr_reg1 <= 32'd0;
            isr_addr_reg2 <= 32'd0;
            interrupt_return_address_1 <= 32'd0;
            interrupt_return_address_2 <= 32'd0;
            mcause_registered <= 32'd0;
            mscratch_registered <= 32'd0;
            mtval_registered <= 32'd0;
            exception_external_interrupt_enable <= 1'b0;
            exception_timer_interrupt_enable <= 1'b0;
            exception_software_interrupt_enable <= 1'b0;
            exception_software_interrupt_pending <= 1'b0;
            exception_interrupt_process <= 1'b0;
        end
//默认情况中断地址/使能/状态寄存器保持
        else begin
            exception_interrupt_enable_registered <= exception_interrupt_enable_registered;
            exception_interrupt_enable_post_registered <= exception_interrupt_enable_post_registered;
            isr_addr_reg1 <= isr_addr_reg1;
            isr_addr_reg2 <= isr_addr_reg2;
            mcause_registered <= mcause_registered;
            mscratch_registered <= mscratch_registered;
            mtval_registered <= mtval_registered;
            exception_external_interrupt_enable <= exception_external_interrupt_enable;
            exception_timer_interrupt_enable <= exception_timer_interrupt_enable;
            exception_software_interrupt_enable <= exception_software_interrupt_enable;
            exception_interrupt_process <= exception_interrupt_process;
//根据不同的csr写地址写入不同的csr寄存器
//★ 提交口：valid / 地址 / 值都是 ROB 打完拍的那一组 ⇒ 不看停顿、也不看当拍冲刷 ——
//  这条指令在上一拍已经退休，它的写必须落地；同拍发生的任何冲刷都属于比它年轻的项。
            if (csr_commit_valid) begin
                case (csr_commit_address)
//全局使能设定
                12'h300: begin
                    exception_interrupt_enable_registered <= csr_write_value[3];
                    exception_interrupt_enable_post_registered <= csr_write_value[7];
                end
//三类型中断分别使能：规范位序 bit3=MSIE(软件)、bit7=MTIE(定时器)、bit11=MEIE(外部)
//★ 本核原来把外部/软件写反了（eirq@3、sirq@11），与自家 mcause 的中断码（外部=11/定时器=7/
//  软件=3，规范）自相矛盾 ⇒ 按规范对调。定时器那条本来就对。
                12'h304: begin
                    exception_external_interrupt_enable <= csr_write_value[11];
                    exception_timer_interrupt_enable <= csr_write_value[7];
                    exception_software_interrupt_enable <= csr_write_value[3];
                end
//isr跳转目标设定
                12'h305: begin
                    isr_addr_reg1 <= csr_write_value;
                    isr_addr_reg2 <= csr_write_value;
                end
                12'h340: begin
                    mscratch_registered <= csr_write_value;
                end
//isr返回目标设定
                12'h341: begin
                    interrupt_return_address_1 <= {csr_write_value[31:2], 2'b00};
                    interrupt_return_address_2 <= {csr_write_value[31:2], 2'b00};
                end
                12'h342: begin
//中断/异常原因寄存器
                    mcause_registered <= csr_write_value;
                end
                12'h343: begin
                    mtval_registered <= csr_write_value;
                end
                12'h344: begin
//软件中断挂起：按规范在 bit3（MSIP）；写 1 清挂起
                    if (csr_write_value[3])
                        exception_software_interrupt_pending <= 1'b0;
                end
                endcase
            end
//异常【检测】那一拍：只做"关中断 + 置受理态"。真正的锁存等 ROB 把故障项退到队头（下面那条）。
//★ 为什么不能在这里锁 mepc/mcause/mtval：先报出来的异常未必是最老的异常 —— 更老的指令可能
//  还在 lsu 队列里没报（lsu 入口才判非对齐，队列里的那条要到下一拍才报）。检测拍就锁，会锁成
//  "年轻的那条"，而 ROB 是按老的那条交付的 ⇒ 两个侧面对不上。改为按 ROB 的交付口锁，才精确。
            if (exception_mark_in) begin
                exception_interrupt_enable_post_registered <= exception_interrupt_enable_registered;
                exception_interrupt_enable_registered <= 1'b0;
                exception_interrupt_process <= 1'b1;
            end
//非isr状态下触发中断，保存上下文并使能受理信号
//  mepc 取 pc_address_in - 4：cpu_top 把 pc_address_in 接的是 mid_decoder 的 aux_addr_2
//  （那条指令"地址 + 4"），也就是【正要进发射级、还没被发出去】的那一条 = 中断返回点。
//  旧写法是"取指 PC 减一个气泡计数"，ROB 之后前端在等 ROB 排空期间还在取指，那个计数已对不上。
            else if (exception_interrupt_action && !exception_interrupt_process) begin
                interrupt_return_address_1 <= pc_address_in - 32'd4;
                interrupt_return_address_2 <= pc_address_in - 32'd4;
                if (exception_external_interrupt_enable && exception_external_interrupt_pending) begin
                    mcause_registered <= 32'h8000000B;
                end
                else if (exception_software_interrupt_enable && exception_software_interrupt_pending) begin
                    mcause_registered <= 32'h80000003;
                end
                else if (exception_timer_interrupt_enable && exception_timer_interrupt_pending) begin
                    mcause_registered <= 32'h80000007;
                end
                exception_interrupt_enable_post_registered <= exception_interrupt_enable_registered;
                exception_interrupt_enable_registered <= 1'b0;
                exception_interrupt_process <= 1'b1;
            end
//异常【交付】：ROB 的队头是那条故障指令（比它老的都退完了、比它年轻的已作废）⇒ 这一拍锁上下文。
//  与上面两条不冲突：受理态（irq_process）一置起，中断那条就不可能再进来。
            if (exception_retire) begin
                interrupt_return_address_1 <= exception_retire_pc;
                interrupt_return_address_2 <= exception_retire_pc;
                mcause_registered <= {28'd0, exception_retire_cause};
                mtval_registered <= exc_retire_tval;
            end
//isr返回（目前只支持机器模式）
            if (exception_interrupt_return_ok) begin
                exception_interrupt_enable_registered <= exception_interrupt_enable_post_registered;
                exception_interrupt_enable_post_registered <= 1'b1;
                exception_interrupt_process <= 1'b0;
            end
        end
    end

//指令退役逻辑，主要用于调试和性能测试
//★ minstret 数的必须是【真退休条数】：原来拿"本拍既未冲刷也未停顿"当退休、每拍最多 +1 ⇒
//  它数的是"没停顿的拍数"，与条数无关（实测 CoreMark：653614 拍读出 472257，看着像 IPC 0.722，
//  其实是非停顿拍占比；真退休只有 330916 条 ⇒ IPC 0.455）。ROB 一拍最多退 2 条，
//  唯一的真口径是【队头指针这一拍推进了几格】。
    reg [1:0] retire_count;
    always @(*) retire_count = retire_step_in;
    always @(posedge clk) begin
        if (rst) begin
            mcycle_registered <= 32'd0;
            minstret_registered <= 32'd0;
        end
        else begin
            mcycle_registered <= mcycle_registered + 32'd1;
            minstret_registered <= minstret_registered + {30'd0, retire_count};
        end
    end

//csr读操作逻辑
    always @(*) begin
        if (rst) begin
            exception_global_pending = 1'b0;
            exception_interrupt_action = 1'b0;
            exception_interrupt_processing = 1'b0;
            mcause = 32'd0;
            interrupt_service_address_2 = 32'd0;
            exception_timer_interrupt_pending = 1'b0;
            exception_external_interrupt_pending = 1'b0;
        end
        else begin
            exception_timer_interrupt_pending = timi;
            exception_external_interrupt_pending = exti;
            exception_global_pending = (exception_external_interrupt_pending && exception_external_interrupt_enable) | (exception_timer_interrupt_pending && exception_timer_interrupt_enable) | (exception_software_interrupt_pending && exception_software_interrupt_enable);
//★ 同拍只要有任何异常在交付路径上，中断一律让位：flush_control_exception 已经把
//  ecall/ebreak/illegal（经 bju 输入级）与访存非对齐（经 lsu 寄存拍）全含进来，
//  所以不必在这里逐个列 —— 靠 else-if 优先级兜正确性是脆的。
//★ S3：这里原本用 bju 的判定拍三根（jump_link_register_failure|br2|br3）表达"本拍有一次改向"——
//  旧设计判定拍就是改向拍，成立；改向搬到提交拍后它们先到、改向后到 ⇒
//  判定拍白挡一次中断，真正改向那一拍反倒没人挡（中断与分支改向可能同拍打架）。
//  改成用【真正改向那一拍的事件】flush_control_in[0]（= 提交口改向）。
            exception_interrupt_action = exception_interrupt_enable_registered && exception_global_pending && !exception_interrupt_process && !exception_interrupt_gate &&
                          !(exc_irq_ret | flush_control_in[0]) && !flush_control_exception && interrupt_head_valid_in;
            exception_interrupt_processing = exception_interrupt_process;
            mcause = mcause_registered;
            interrupt_service_address_2 = isr_addr_reg2;
        end
    end

//给 alu 的组合读口：地址取本拍要发的 csr_address（= 发射拍那一条的地址）。不做写后读旁路 ——
//每条 csr 指令各占一个发射拍，前一条的写在前一沿就已落进寄存器，本条组合读到的就是它。
    always @(*) begin
        case (csr_address)
        12'h300: csr_data_combinational = {19'd0, 2'b11, 3'd0, exception_interrupt_enable_post_registered, 3'd0, exception_interrupt_enable_registered, 3'd0};
        12'h301: csr_data_combinational = 32'h40001100;
        12'h304: csr_data_combinational = {20'd0, exception_external_interrupt_enable, 3'd0, exception_timer_interrupt_enable, 3'd0, exception_software_interrupt_enable, 3'd0};
        12'h305: csr_data_combinational = isr_addr_reg1;
        12'h340: csr_data_combinational = mscratch_registered;
        12'h341: csr_data_combinational = interrupt_return_address_1;
        12'h342: csr_data_combinational = mcause_registered;
        12'h343: csr_data_combinational = mtval_registered;
        12'h344: csr_data_combinational = {20'd0, exception_external_interrupt_pending, 3'd0, exception_timer_interrupt_pending, 3'd0, exception_software_interrupt_pending, 3'd0};
        12'hB00: csr_data_combinational = mcycle_registered;
        12'hB02: csr_data_combinational = minstret_registered;
        12'hF14: csr_data_combinational = 32'd0;
        default: csr_data_combinational = 32'd0;
        endcase
    end

endmodule 
