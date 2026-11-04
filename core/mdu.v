`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: mdu
// Project Name:
// Target Devices:
// Tool Versions:
// Description: RV32M 乘除法单元，与 lsu 流水线行为【同步】。
//
//   关键约束：mdu 的提交时刻必须与 lsu 严格同偏移 —— 乘法两级流水、除法提交延迟链
//   都按这个偏移设计，改级数就会打乱它与 lsu 的结果口相位。
//   lsu 的读时序：T（指令在 lsu 级）→ T+1 地址上总线 → T+2 应答 → 写沿 T+3。
//   所以乘法做成【两级】正好对齐：
//     T      ：指令在 mdu 级，组合判该走乘法口还是除法口
//     T→T+1  ：锁操作数 a/b/op
//     T+1    ：33×33 有符号乘法（组合，综合成 DSP48）
//     T+1→T+2：锁乘积
//     T+2    ：multiply_write_enable / multiply_loaded 拉高 → prf 在 T+2→T+3 沿写（= lsu 的写沿）
//     T+3    ：hold（multiply_loaded 第二拍，对应 lsu 的 ld_hold）
//   背靠背乘法 1 笔/拍（两级流水不冲突）。
//
//   除法是 32 拍的移位-相减迭代，组合实现会直接成为新的最差路径，故走 FSM。
//   【站式改动】原来"除法在途期间把整条流水线冻住"（靠 stall_mulu_div 上 flag_bus），
//   除法因此一直摆在 mdu 入口等门开；换成保留站之后站项"只摆一拍、出站即撤"，那条前提没了。
//   现在改成【两条独立接收口】：乘法口 / 除法口，各回一条组合 ready 给站做门控。
//   本模块的接收式与站侧的门控式逐字相同 ⇒ "站撤项"与"本模块收下"永远同拍，不会丢。
//   两条 ready 只由本模块寄存器状态决定（不含对方通道、不含被选指令）⇒ 不闭零延时环。
//
//   乘法与除法因此可以在途重叠（站里一条乘法不必等除法算完），两者只在【唯一的结果口】合流：
//   撞口那拍（divide_state_commit & multiply_product_valid）由 wb_conflict 让除法先走、乘法两级原地冻一拍、下一拍再报。
//   （没有这一条时：撞口拍端口上出的是乘法的号，除法完成口被吞掉 ⇒ 它的 ROB 项永远等不到 ⇒ 队头卡死。）
//
//   乘法用【一个】33×33 有符号乘法器覆盖全部 4 条：把两个操作数按需扩展成 33 位
//   （有符号补符号位、无符号补 0）再相乘，MUL 取低 32、MULH/MULHSU/MULHU 取高 32。
//
//   操作数由站选中项的终值供给（随出站载荷进来），不占用 prf 的读口。
//
//   本文件内 always 块按【流水级数】排列（两条 ready 在最前）：
//     两条 ready → 第一级（锁操作数）→ 第二级（锁乘积）→ 除法迭代 → 提交链/输出保持
//     → 组合输出（写回仲裁）。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module mdu(
    input clk, rst,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input [2:0] flush_control_in,   // = flag_bus[7:5]：{异常, 中断, 改向}
//本次冲刷的边界（cont 的 flush_idx：三种冲刷各取自己那条的号）：
//入口门按它判"本拍站在载荷上的这条，是边界自己、还是比边界更年轻的错路条"。
//两条接收通道（乘法口 / 除法口）：站一拍只出一条 ⇒ 两套载荷同源，只有 valid 不同。
//★ 站侧门控用的就是本模块导出的 multiply_ready / divide_ready，两边逐字同一表达式 ⇒
//  "站撤项"与"本模块收下"永远同拍，任何一条都不会被静默丢掉（本次改造的核心）。
    input        multiply_go,
    input [9:0]  multiply_function10,
    input [4:0]  multiply_destination,
    input [3:0]  multiply_index_in,
    input        multiply_generation_in,
    input [31:0] multiply_source_a, mul_r2,
    input        divide_go,
    input [9:0]  divide_function10,
    input [4:0]  divide_destination,
    input [3:0]  divide_index_in,
    input        divide_generation_in,
    input [31:0] divide_source_a, div_r2,
    (* max_fanout = 8 *) output reg [31:0] multiply_data_out,
//乘法结果口 D 端（下一沿 multiply_product 会装载的那个乘积，按 multiply_stage_operation 选高低半）
    output reg [31:0] multiply_data_early,
//除法结果口 D 端（divide_state_commit 的下一拍就是它，数据寄存器早已锁好）
    output reg [31:0] divide_data_early,
//身份的 D 端（下一沿口上 valid 与 *_idx/_gen 会装载的值）：早唤醒用
    output reg        multiply_write_enable_early,
    output reg [3:0]  multiply_index_early,
    output reg        multiply_generation_early,
    output reg        divide_write_enable_early,
    output reg [3:0]  divide_index_early,
    output reg        divide_generation_early,
    (* max_fanout = 8 *) output reg multiply_loaded, multiply_write_enable,
//本模块这一拍供的是哪一项的 rd（与 multiply_index/multiply_generation 逐支同步，供消费者比源寄存器号）
    (* max_fanout = 8 *) output reg [4:0] rd_mul,
//本条写回记录带的 ROB 索引（跟着数据走，写回口用它定位自己那一项）
    output reg [3:0]  multiply_index,
    (* max_fanout = 8 *) output reg        multiply_generation,
//除法独立结果口：不再与乘法共用一口，两者同拍各自报自己的
    (* max_fanout = 8 *) output reg [31:0] divide_data_out,
    (* max_fanout = 8 *) output reg        divide_loaded, divide_write_enable,
    (* max_fanout = 8 *) output reg [4:0]  rd_div,
    output reg [3:0]  divide_index,
    (* max_fanout = 8 *) output reg        divide_generation,
//两条 ready 回站：只由本模块寄存器状态 + 本拍的总线占用位决定（不含被选指令、不含对方通道）⇒ 不闭零延时环。
//站侧按站项自己的算子门控（乘法项看 multiply_ready、除法项看 divide_ready），门控式与本模块的接收式逐字相同。
    output reg multiply_ready,
    output reg divide_ready,
    output reg        multiply_wake_registered,
    output reg [3:0]  multiply_wake_index_registered,
    output reg        multiply_wake_generation_registered,
    output reg        multiply_nack_registered,
    output reg [3:0]  multiply_nack_index_registered,
    output reg        multiply_nack_generation_registered
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;


//控制位译码（与流水级无关，放本模块最前）
//flag_bus = {flush_control_exception, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//本模块的推进由自己的 stall/pipe_stall 把关（乘除在途语义），不用流水线使能，故只取三条冲刷位。
//本模块的冲刷窗口与别的模块同宽：除法 FSM 的作废（divide_state_kill）与 divide_state_done 的清零都挂在这个 flush_window 上。
//★ 2026-10-28 删除了一条已经不成立的旧前提。原文写的是「乘法第二级 / divide_state_commit_registered / hold
//  有意不吃冲刷（它们冲刷拍握的一定比分支更老）」—— 那是【bju 一拍出冲刷】时代的规矩。
//  现在冲刷在【提交口】发（cont.v:223 `flush_control_jump = redirect_in;`，
//  rob.v:519 的 `redirect_now` 判的是 `entry_valid[head_pointer]`）⇒ **队头就是边界**，
//  在途的每一条都比冲刷点【更年轻】，不存在"更老所以必须照样提交"这一档。
//  乘法两级因此改成按年龄作废（见 multiply_stage1_kill / multiply_stage2_kill）。
    reg flush_window, flush_control_exception;
    reg       divide_state_kill;
    reg       multiply_stage1_kill, multiply_stage2_kill;
    always @(*) begin
        flush_control_exception = flush_control_in[2];
        flush_window = flush_control_exception | flush_control_in[1] | flush_control_in[0];
    end

//冻结信号合流（本模块是消费者，或运算在这里做）：
//  bus_hold   = 总线不可用（外部 hold）
//  pipe_stall = 总线不可用（唯一一项）
//★ dcache 忙已不在 flag_bus 上：乘法与 dcache 无关，本模块不吃它（它只直连 lsu）。
//★ 不要并进 stall_rob_full(3) / stall_pc_redir(2)：ROB 排空要靠队头那条乘法完成，
//  而乘法正被它挡在 mdu 门外 ⇒ ROB 永不排空 ⇒ 死锁（与 lsu 入口门同一个坑）。
//★ 2026-10-31：总线占用（原来的 flag_bus[0]）不再进本模块 —— 乘法/除法不碰总线，
//  被外部 hold 冻住没有道理。这里现在恒 0。
//  ★ 必须用 wire + 连续赋值，不能写成 `always @(*)` 里赋常数：那样敏感表是空的，
//    整块从不触发，reg 停在 X ⇒ multiply_ready = ~X ⇒ 全核 X 传播（实测 mt 直接掉到 11/15）。
    wire pipe_stall = 1'b0;
    wire bus_hold   = 1'b0;

//寄存器声明（按级分组）
//乘法：第一级锁操作数 / 第二级锁乘积
    reg [31:0] multiply_stage_a, multiply_stage_b;
    reg [4:0]  multiply_stage_destination;
    reg [3:0]  multiply_stage_index;
    reg        multiply_stage_generation;
    reg [2:0]  multiply_stage_operation;
    reg        multiply_stage_valid;

    reg [63:0] multiply_product;
    reg [4:0]  multiply_stage_destination_registered;
    reg [3:0]  multiply_stage_index_registered;
    reg        multiply_stage_generation_registered;
    reg [2:0]  multiply_stage_operation_registered;
    (* max_fanout = 8 *)
    reg        multiply_product_valid;

//除法：32 拍移位-相减迭代
    reg [31:0] divide_state_dividend;             // 原被除数（除零时余数要回它）
    reg [31:0] divide_state_a, divide_state_b;          // 取绝对值后的被除数 / 除数
    reg        divide_state_remainder;             // 1 = 求余数
    reg        divide_state_negative_registered, divide_state_negative_result;  // 商 / 余 需要取负
    reg        divide_state_sign;             // 发起那拍锁下的符号性（站式之后端口上那条下一拍就换了）
    reg        divide_state_zero;            // 除数为 0
    reg        divide_state_overflow;             // INT_MIN / -1
    reg        divide_state_sign_b;
    reg [4:0]  divide_state_destination;
    reg [3:0]  divide_state_index;
    reg        divide_state_generation;
    reg [4:0]  divide_state_count;
    reg        divide_state_busy;
    reg        divide_state_issued;          // 本条 div 已发起过（防止离开 mdu 级之前重复发起）
    reg        divide_state_done_registered;
    reg        divide_state_done;            // 完成脉冲（比最后一次迭代晚一拍）

    reg [31:0] divide_state_remainder_partial;           // 迭代中的余数累加器
    reg [31:0] divide_state_quotient_partial;           // 迭代中的商

//提交链
//除法写口延迟：divide_state_done 那拍 div 还在发射级，而程序序在它【前面】的 ALU 指令还要几拍才到写口。
//若除法在 divide_state_done 当拍就写，同 rd 时晚写的 ALU 指令会盖掉除法结果。
//按流水级数对齐，除法写口落在 divide_state_done+3（即 divide_state_commit_registered[2]）。
    reg [2:0]  divide_state_commit_registered;
    reg [4:0]  divide_state_commit_destination;
    reg [3:0]  divide_state_commit_index;
    reg        divide_state_commit_generation;
    reg [31:0] divide_state_commit_data;

//输出保持
    reg [4:0]  hold_rd;
    reg [3:0]  index_hold;
    reg        generation_hold;
    reg [31:0] hold_data;
    reg        hold;

    always @(*) begin
// KILL-ARC: 判据改成查 rob 的击杀弧掩码（第 i 位 = 槽 i 比边界年轻）。
//   `d_age <= flush_age`（不比我年轻）= `~kill_arc[divide_state_index]`，代入原式化简成下面这一行。
//   吃当拍版：flush_window / flush_partial 都是当拍的。
        divide_state_kill    = flush_window;
//乘法两级：判据与除法【同一条】。冲刷在提交口发、队头即边界 ⇒ 在途的乘法一律比边界年轻，
//  冲刷拍必须作废，与"必须照样提交"正好相反。
//  带 `~kill_arc` 那一支：部分冲刷时，比边界老的乘法仍然要留着（否则它那一项永远等不到完成口）。
//  带 `valid` 那一支：`multiply_stage_index` 只在 push 那一拍装载，复位后没有初值，
//  不让它进 kill_arc 的索引，免得复位期走出 X。
        multiply_stage1_kill = multiply_stage_valid & flush_window;
        multiply_stage2_kill = multiply_product_valid & flush_window;
    end

//组合逻辑（全部行为描述：reg + always @(*) 阻塞赋值）
    reg        multiply_push;
    reg        multiply_select;
    reg signed [32:0] multiplicand_ext, multiplier_ext;
    reg signed [65:0] multiply_product_intermediate;
    reg        divide_state_last, divide_state_greater_equal;
    reg [32:0] divide_state_subtract;
    reg [31:0] divide_state_quotient_final, divide_state_remainder_final, divide_state_result;
    reg        divide_state_commit, divide_state_pending;

//===============================================================
// 两条 ready（回站的门控）—— 只由本模块寄存器状态 + 本拍总线占用位决定
//===============================================================
//乘法口：两级流水只要不被冻结就收（站一拍只出一条，容量天然够）。
//★ 把 wb_conflict 也算进来：冲突拍乘法两级原地冻住，此时若还收新的乘法，
//  推进 multiply_stage_valid 那一拍会把冻着的那条冲掉 —— "收了就必须推进"是这条契约的另一半。
    always @(*) begin
        multiply_ready = ~pipe_stall;
//除法口：FSM 空闲 + 提交链空 ⇒ 收下当拍就能发起（除法执行拍数不变：发起不晚于接收那一拍）
        divide_ready = ~divide_state_issued & ~divide_state_busy & ~divide_state_done & ~divide_state_pending & ~pipe_stall;
    end

//第一级：锁操作数。判据与 lsu 的入级门（new_in/new_put）同形。
//★ 接收式 = 站侧门控式（multiply_ready）逐字相同 ⇒ 站撤项那一拍必定被收下，不会静默丢。
//★ 冲刷窗口内不许收：multiply_go 本身就是"本拍真发出去"（issue_v 已经排掉冲刷拍）；
//  中断的边界那条【自己也已经死了】（含边界的杀集）⇒ 没有"边界要放行"这一档。
    always @(*) begin
        multiply_push = multiply_ready && multiply_go;
    end

//乘法第一级：锁操作数。pipe_stall 期间【不推进】—— 与第二级、hold 同一条件，
//这样乘法写口与 lsu 的写口相位始终对齐。
    always @(posedge clk) begin
        if (reset_registered) begin
            multiply_stage_valid <= 1'b0;
            multiply_stage_a <= 32'd0;
            multiply_stage_b <= 32'd0;
            multiply_stage_destination <= 5'd0;
            multiply_stage_operation <= 3'd0;
        end
        else if (multiply_stage1_kill) begin
            multiply_stage_valid <= 1'b0;
        end
        else if (!pipe_stall) begin
            multiply_stage_valid <= multiply_push;
            if (multiply_push) begin
                multiply_stage_a  <= multiply_source_a;
                multiply_stage_b  <= mul_r2;
                multiply_stage_destination <= multiply_destination;
                multiply_stage_index <= multiply_index_in;
                multiply_stage_generation <= multiply_generation_in;
                multiply_stage_operation <= multiply_function10[2:0];
            end
        end
    end

//===============================================================
// 第二级：锁乘积
//===============================================================
//33 位扩展：
//  a 无符号 ⟺ funct3==011 (MULHU)
//  b 无符号 ⟺ funct3∈{010,011} (MULHSU/MULHU) ⟺ funct3[1]==1
    always @(*) begin
        multiplicand_ext   = (multiply_stage_operation == 3'b011) ? {1'b0, multiply_stage_a} : {multiply_stage_a[31], multiply_stage_a};
        multiplier_ext   = (multiply_stage_operation[1])        ? {1'b0, multiply_stage_b} : {multiply_stage_b[31], multiply_stage_b};
        multiply_product_intermediate = multiplicand_ext * multiplier_ext;
    end

//乘法第二级：锁乘积。同样按 pipe_stall 冻结。
//★ 2026-10-28：原来这里写「这一级【不判 flush】—— 能走到这里的乘法，发起它的那条指令一定比
//  正在冲刷的那条更老」。那条前提已经不成立：冲刷在提交口发、队头即边界，在途的乘法一律更年轻。
//  现在按年龄作废（multiply_stage2_kill），与除法同一条判据。
    always @(posedge clk) begin
        if (reset_registered) begin
            multiply_product_valid <= 1'b0;
            multiply_product <= 64'd0;
            multiply_stage_destination_registered <= 5'd0;
            multiply_stage_operation_registered <= 3'd0;
        end
        else if (multiply_stage2_kill) begin
            multiply_product_valid <= 1'b0;
        end
        else if (!pipe_stall) begin
            multiply_product_valid   <= multiply_stage_valid;
            multiply_product    <= multiply_product_intermediate[63:0];
            multiply_stage_destination_registered <= multiply_stage_destination;
            multiply_stage_index_registered <= multiply_stage_index;
            multiply_stage_generation_registered <= multiply_stage_generation;
            multiply_stage_operation_registered <= multiply_stage_operation;
        end
    end

//funct3[1:0]==00 → MUL，取低 32；其余（MULH/MULHSU/MULHU）取高 32
    always @(*) begin
        multiply_select = (multiply_stage_operation_registered[1:0] == 2'b00);
    end

//===============================================================
// 除法：32 拍移位-相减迭代
//===============================================================
//除法迭代一拍：余数左移一位并入被除数最高位，够减则商 1
    always @(*) begin
        divide_state_subtract = {divide_state_remainder_partial[30:0], divide_state_a[31]} - {1'b0, divide_state_b};
        divide_state_greater_equal  = ~divide_state_subtract[32];
        divide_state_last = divide_state_busy && (divide_state_count == 5'd31);
    end

//除法迭代
    always @(posedge clk) begin
        if (reset_registered) begin
            divide_state_busy <= 1'b0;
            divide_state_issued <= 1'b0;
            divide_state_count  <= 5'd0;
            divide_state_remainder_partial <= 32'd0;
            divide_state_quotient_partial <= 32'd0;
            divide_state_dividend <= 32'd0;
            divide_state_a <= 32'd0;
            divide_state_b <= 32'd0;
            divide_state_destination <= 5'd0;
            divide_state_remainder <= 1'b0;
            divide_state_negative_registered <= 1'b0;
            divide_state_negative_result <= 1'b0;
            divide_state_zero <= 1'b0;
            divide_state_overflow <= 1'b0;
            divide_state_sign_b <= 1'b0;
        end
        else if (divide_state_kill) begin
            divide_state_busy   <= 1'b0;
            divide_state_issued <= 1'b0;
        end
        else if (divide_state_done_registered) begin
//★ 【站式改动】原来这里是 (divide_state_done_registered || !is_div)：`!is_div` 判的是"当前摆在端口上那条不是
//  除法"，站式之后【只摆一拍】——下一拍端口上就是别的指令了 ⇒ 若把它加回作废条件，
//  32 拍迭代刚起头就自己作废，那条除法永远完不成、它的 ROB 项再也等不到完成口。
//  真正的作废只由上面 divide_state_kill（flush_window）那一支负责（除法无副作用，冲刷就重来）。
//  清 divide_state_busy 仍要有：迭代分支进不去会让它粘死。
            divide_state_issued <= 1'b0;
            divide_state_busy   <= 1'b0;
        end
        else if (divide_state_busy) begin
            divide_state_a     <= {divide_state_a[30:0], 1'b0};
            divide_state_remainder_partial <= divide_state_greater_equal ? divide_state_subtract[31:0] : {divide_state_remainder_partial[30:0], divide_state_a[31]};
            divide_state_quotient_partial <= {divide_state_quotient_partial[30:0], divide_state_greater_equal};
            if (divide_state_last)
                divide_state_busy <= 1'b0;
            else
                divide_state_count <= divide_state_count + 5'd1;
            if (divide_state_count == 5'd0) begin
                divide_state_zero  <= (divide_state_b == 32'd0);
                divide_state_overflow   <= divide_state_sign && (divide_state_dividend == 32'h80000000) &&
                           (divide_state_b == 32'd1) && divide_state_sign_b;
                divide_state_negative_registered <= divide_state_sign && (divide_state_dividend[31] ^ divide_state_sign_b) &&
                           (divide_state_b != 32'd0);
                divide_state_negative_result <= divide_state_sign && divide_state_dividend[31] && (divide_state_b != 32'd0);
            end
        end
//★ 发起条件 = divide_go & divide_ready：divide_ready 已经排掉 FSM 忙、提交链未空与 pipe_stall，
//  发起侧不再另加判据。★ 判据与回给站的 divide_ready 逐字同源：站侧门控 = divide_ready
//  ⇒ 站撤项那一拍必定在这里发起，不会出现"发出去了、没接住"（老树靠"除法冻在门口等"
//  保证，站式之后那条前提没有了）。
        else if (divide_go && divide_ready) begin
//发起：有符号类先取绝对值，收尾再按符号还原。
//divide_state_issued 保证一条 div 只发起一次；否则算完后会无限重复发起、放行判据永远落不下去。
//★ 操作数与符号位必须在【发起那一拍】从除法口锁下来：divide_state_count==0 那个收尾判据在发起晚一拍执行，
//  那时端口上已经是别的指令了（站一拍只摆一条），现场读会取到别人的编码。
            divide_state_destination     <= divide_destination;
            divide_state_index    <= divide_index_in;
            divide_state_generation    <= divide_generation_in;
            divide_state_remainder    <= divide_function10[1];
            divide_state_sign    <= (divide_function10[0] == 1'b0);
            divide_state_dividend    <= divide_source_a;
            divide_state_sign_b <= div_r2[31];
            divide_state_a     <= ((divide_function10[0] == 1'b0) && divide_source_a[31]) ? (~divide_source_a + 32'd1) : divide_source_a;
            divide_state_b     <= ((divide_function10[0] == 1'b0) && div_r2[31]) ? (~div_r2 + 32'd1) : div_r2;
            divide_state_remainder_partial <= 32'd0;
            divide_state_quotient_partial <= 32'd0;
            divide_state_count   <= 5'd0;
            divide_state_busy  <= 1'b1;
            divide_state_issued <= 1'b1;
        end
    end

//收尾还原符号 + 规范边界值
//  除零 → 商全 1 / 余为原被除数；溢出 INT_MIN/-1 → 商 INT_MIN / 余 0
    always @(*) begin
        divide_state_quotient_final = divide_state_negative_registered ? (~divide_state_quotient_partial + 32'd1) : divide_state_quotient_partial;
        divide_state_remainder_final = divide_state_negative_result ? (~divide_state_remainder_partial + 32'd1) : divide_state_remainder_partial;
        divide_state_result = divide_state_zero ? (divide_state_remainder ? divide_state_dividend : 32'hFFFFFFFF) :
                divide_state_overflow  ? (divide_state_remainder ? 32'd0 : 32'h80000000) :
                         (divide_state_remainder ? divide_state_remainder_final : divide_state_quotient_final);
    end

//完成脉冲（比最后一次迭代晚一拍）
    always @(posedge clk) begin
        if (reset_registered) begin
            divide_state_done   <= 1'b0;
            divide_state_done_registered <= 1'b0;
        end
        else if (divide_state_kill) begin
            divide_state_done   <= 1'b0;
            divide_state_done_registered <= 1'b0;
        end
        else begin
            divide_state_done   <= divide_state_last;
            divide_state_done_registered <= divide_state_done;
        end
    end

//===============================================================
// 提交链（divide_state_done+3 写口）与输出保持
//===============================================================
//除法结果提交延迟链 + 输出保持（对应 lsu 的 ld_hold）
    always @(posedge clk) begin
//【只由 rst 清】这条提交延迟链是"除法已经算完、结果在链上往外走"的那一段，它没有身份判断，
//  也不作废：错路的那一笔照样会送到 ROB / 值阵列，由收端按 {slot, generation} 判掉。
//★ 2026-10-28：原文补的"（与乘法第二级不判 flush 同理）"已删 —— 乘法两级现在判 flush
//  （multiply_stage*_kill），这里不清的理由不来自"更老所以必须提交"，只来自"这一级没有作废的抓手"。
        if (reset_registered) begin
            divide_state_commit_registered <= 3'd0;
//载荷也要复位：D 端（divide_data_early）每拍都往外送，不复位就会把 X 灌进前送表
            divide_state_commit_destination <= 5'd0; divide_state_commit_index <= 4'd0; divide_state_commit_generation <= 1'b0; divide_state_commit_data <= 32'd0;
        end
        else if (!pipe_stall) begin
            divide_state_commit_registered <= {divide_state_commit_registered[1:0], divide_state_done};
            if (divide_state_done) begin
                divide_state_commit_destination   <= divide_state_destination;
                divide_state_commit_index  <= divide_state_index;
                divide_state_commit_generation  <= divide_state_generation;
                divide_state_commit_data <= divide_state_result;
            end
        end
    end

    always @(posedge clk) begin
        if (reset_registered)
            hold <= 1'b0;
        else if (!pipe_stall) begin
            hold <= multiply_product_valid;
            if (multiply_product_valid) begin
                hold_rd   <= multiply_stage_destination_registered;
                index_hold  <= multiply_stage_index_registered;
                generation_hold  <= multiply_stage_generation_registered;
                hold_data <= multiply_select ? multiply_product[31:0] : multiply_product[63:32];
            end
        end
    end

//组合输出：写回仲裁（乘法第二级与除法提交共用一个结果口）
//★ 站式之后乘法与除法可以同时在途（两条通道各自独立收），所以"撞口"从极少数变成常态：
//  撞口拍（divide_state_commit & multiply_product_valid）由 wb_conflict 让【除法先走】，乘法两级原地冻一拍、结果就躺在
//  multiply_product_valid/multiply_product 里，不需要任何新寄存器，下一拍再报。
//★ 没有这一条时的死法：撞口拍端口上出的是乘法的号，除法那一笔的完成口被吞掉，
//  它的 ROB 项永远等不到 ⇒ 队头卡死。
//两个独立结果口：乘法口（multiply_product_valid 那拍 + hold 第二拍）、除法口（divide_state_commit 那拍）。
//结果口的 D 端：表项必须与"口亮起那一拍"的值同拍对齐
//★★ 2026-11-04：乘法口改成从【MDU 结果寄存器】出（multiply_product + 寄存后的 op 选择 + hold），
//  不再用 DSP 参与算出来的组合乘积 multiply_product_intermediate。
//  为什么：组合乘积一挂到结果口，DSP 那条链和 mdu 的选择信号就被拖进 prf 的写口和旁路网，
//  实测 12.5ns 下 `−2.041 / 1691 端点`（改之前 `−0.744 / 242`），失败端点铺到 mdu/iffu/idu1/csr 四处。
//  代价：乘法结果晚一拍 —— 现在乘法早口和晚口是同一份（都从 multiply_data_out 出）。
//  早口的那几项在下面"乘法口"那个 always 块里跟着赋，避免跨块读。
    always @(*) begin
        divide_data_early = divide_state_commit_data;
//身份 D 端：冻结拍那几个寄存器不装载，要跟着保持
//★ 写使能【只取触发器】，不再乘 ~kill。kill 是 rob 的组合链（slot_above_head_registered → redirect_index → kill_arc），
//  乘进 we 就等于把它拉进值阵列每个 entry D 端那个 5 口优先级 mux —— 实测最差路 16 级、78% 是布线。
//★ 解耦之后错路那一笔最多多写【一拍】：写进的是它自己那个槽，而该槽的 valid 在它分配那一拍就被清成 0；
//  重新分配至少还要 5 拍才到 idu2-D，到那时 valid 又被清一次 ⇒ 这一拍设起来的 valid 活不过下一次分配。
//★ kill 本身不动：乘/除各级 valid 的清仍按年龄那个式子来，kill 只出现在寄存器的 D 端，不在这条路上。
        divide_write_enable_early = pipe_stall ? divide_state_commit_registered[2] : divide_state_commit_registered[1];
        divide_index_early = divide_state_commit_index;
        divide_generation_early = divide_state_commit_generation;
    end
    always @(*) begin
        divide_state_commit  = divide_state_commit_registered[2];                           // 除法写口脉冲 = divide_state_done+3
        divide_state_pending = divide_state_commit_registered[0] | divide_state_commit_registered[1] | divide_state_commit_registered[2]; // 除法结果前递窗口
//---- 乘法口 ----
        multiply_write_enable     = multiply_product_valid;
        multiply_loaded = multiply_product_valid | hold;
        if (multiply_product_valid) begin
            rd_mul       = multiply_stage_destination_registered;
            multiply_index      = multiply_stage_index_registered;
            multiply_generation      = multiply_stage_generation_registered;
            multiply_data_out = multiply_select ? multiply_product[31:0] : multiply_product[63:32];
        end
        else if (hold) begin
            rd_mul       = hold_rd;
            multiply_index      = index_hold;
            multiply_generation      = generation_hold;
            multiply_data_out = hold_data;
        end
        else begin
            rd_mul       = 5'd0;
            multiply_index      = 4'd0;
            multiply_generation      = 1'b0;
            multiply_data_out = 32'd0;
        end
//★ 2026-11-04：乘法早口就是晚口那一份（都从 MDU 结果寄存器出），不再有"早一拍"的组合乘积。
        multiply_data_early       = multiply_data_out;
        multiply_write_enable_early = multiply_write_enable;
        multiply_index_early      = multiply_index;
        multiply_generation_early = multiply_generation;
//---- 除法口 ----
        divide_write_enable       = divide_state_commit;
        divide_loaded   = divide_state_commit;
        rd_div       = divide_state_commit_destination;
        divide_index      = divide_state_commit_index;
        divide_generation      = divide_state_commit_generation;
        divide_data_out = divide_state_commit_data;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            multiply_wake_registered <= 1'b0;
            multiply_wake_index_registered <= 4'd0;
            multiply_wake_generation_registered <= 1'b0;
            multiply_nack_registered <= 1'b0;
            multiply_nack_index_registered <= 4'd0;
            multiply_nack_generation_registered <= 1'b0;
        end
        else begin
            multiply_wake_registered <= multiply_push & ~divide_state_commit_registered[0];
            if (multiply_push) begin
                multiply_wake_index_registered <= multiply_index_in;
                multiply_wake_generation_registered <= multiply_generation_in;
            end
            multiply_nack_registered <= multiply_wake_registered & pipe_stall;   // 撞口仲裁已删：乘法口独立，只剩真停顿
            multiply_nack_index_registered <= multiply_wake_index_registered;
            multiply_nack_generation_registered <= multiply_wake_generation_registered;
        end
    end

endmodule
