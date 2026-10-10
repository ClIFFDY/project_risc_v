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
//   关键约束（用户给定，本模块据此设计）：mdu 的提交时刻必须与 lsu 严格同偏移，
//   这样 regfile 的三个写口永不共拍，不需要写口优先级仲裁。
//   lsu 的读时序：T（指令在 lsu/decoder 级）→ T+1 地址上总线 → T+2 应答 → 写沿 T+3。
//   所以乘法做成【两级】正好对齐：
//     T      ：指令在 mdu 级，组合判 is_m
//     T→T+1  ：锁操作数 a/b/op
//     T+1    ：33×33 有符号乘法（组合，综合成 DSP48）
//     T+1→T+2：锁乘积
//     T+2    ：mul_we / mul_loaded 拉高 → regfile 在 T+2→T+3 沿写（= lsu 的写沿）
//     T+3    ：hold（mul_loaded 第二拍，对应 lsu 的 ld_hold）
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
//   撞口那拍（d_cmt & m_pv）由 wb_conflict 让除法先走、乘法两级原地冻一拍、下一拍再报。
//   （没有这一条时：撞口拍端口上出的是乘法的号，除法完成口被吞掉 ⇒ 它的 ROB 项永远等不到 ⇒ 队头卡死。）
//
//   乘法用【一个】33×33 有符号乘法器覆盖全部 4 条：把两个操作数按需扩展成 33 位
//   （有符号补符号位、无符号补 0）再相乘，MUL 取低 32、MULH/MULHSU/MULHU 取高 32。
//
//   操作数由 regfile 的【独立第三份读数据通路】供给（_mul），不与其他消费单元共享。
//
//   本文件内 always 块按【流水级数】排列：
//     第一级（锁操作数）→ 第二级（锁乘积）→ 除法迭代 → 提交链/输出保持
//     → 组合输出（两条 ready + 写回仲裁）。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module mdu(
    input clk, rst,
    input [7:0] flag_bus,
//本次冲刷的边界（cont 的 flush_idx：三种冲刷各取自己那条的号）：
//入口门按它判"本拍站在载荷上的这条，是边界自己、还是比边界更年轻的错路条"。
    input [2:0]  flush_idx,
    input [2:0]  head_ptr,
    input        flush_part,
//两条接收通道（乘法口 / 除法口）：站一拍只出一条 ⇒ 两套载荷同源，只有 valid 不同。
//★ 站侧门控用的就是本模块导出的 mul_ready / div_ready，两边逐字同一表达式 ⇒
//  "站撤项"与"本模块收下"永远同拍，任何一条都不会被静默丢掉（本次改造的核心）。
    input        mul_go,
    input [9:0]  mul_func10,
    input [4:0]  mul_rd,
    input [2:0]  mul_idx_in,
    input        mul_gen_in,
    input [31:0] mul_r1, mul_r2,
    input        div_go,
    input [9:0]  div_func10,
    input [4:0]  div_rd,
    input [2:0]  div_idx_in,
    input        div_gen_in,
    input [31:0] div_r1, div_r2,
    (* max_fanout = 8 *) output reg [31:0] mul_data_out,
    (* max_fanout = 8 *) output reg mul_loaded, mul_we,
//本模块这一拍供的值是不是消费者的（r1/r2 各一位）：与 mul_idx 逐支同步，供 forw 直接选源
    (* max_fanout = 8 *) output reg [4:0] rd_mul,
//本条写回记录带的写序号（跟着数据走，写回级用它判谁更老）
    output reg [2:0]  mul_idx,
    (* max_fanout = 8 *) output reg        mul_gen,
//两条 ready 回站：只由本模块寄存器状态决定（不含对方通道、不含被选指令）⇒ 不闭零延时环。
//站侧按站项自己的算子门控（乘法项看 mul_ready、除法项看 div_ready），门控式与本模块的接收式逐字相同。
    output reg mul_ready,
    output reg div_ready
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;


//控制位译码（与流水级无关，放本模块最前）
//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：本模块的推进由自己的 stall/pipe_stall 把关（乘除在途语义），
//不用流水线使能，故只取三条冲刷位。
//本模块的冲刷窗口回到【与别的模块同宽】：m_push、除法 FSM 的作废、d_done 的清零都挂在这同一个
//flush_w 上。
//乘法第二级 / d_cmt_q / hold 有意不吃冲刷（它们冲刷拍握的一定比分支更老，见文件头注释）。
    reg flush_w, flush_con_exc;
    reg [3:0] d_age, flush_age;
    reg       d_kill;
    always @(*) begin
        flush_con_exc = flag_bus[7];
        flush_w = flush_con_exc | flag_bus[6] | flag_bus[5];
    end

//冻结信号合流（本模块是消费者，或运算在这里做）：
//  bus_hold   = 总线不可用（外部 hold）
//  pipe_stall = 总线保持 / 取指缺失 / lsu 那三条（排掉自己的 haz/div 两位）
//★ dcache 忙(2) 已撤出全核广播：乘法与 dcache 无关，本模块不再吃它（它只在 lsu 内部生效）。
//★ 不要并进 stall_rob_full(9) / stall_pc_redir(8)：ROB 排空要靠队头那条乘法完成，
//  而乘法正被它挡在 mdu 门外 ⇒ ROB 永不排空 ⇒ 死锁（与 lsu 入口门同一个坑）。
    reg pipe_stall, bus_hold;
    always @(*) begin
        bus_hold   = flag_bus[0];
//★ 只留总线占用。【不能再带 flag_bus[2]（= 保留站满）】：站式之后站满恰恰是发射常态，
//  而保留站对指令只【摆一拍】——那一拍门关着，这条乘/除就永远不发起，它的 ROB 项再也等不到
//  完成口 ⇒ 队头卡死、整核死等（实测 div-01：一条 div 在 pc 走到 0x160 处发出后凭空消失）。
//  原设计没有这个问题，是因为载荷会一直摆在门口等门开。
        pipe_stall = flag_bus[0];   // [2]=保留站满（原 stall_lsu_haz 位）、[0]=总线占用
    end

//寄存器声明（按级分组）
//乘法：第一级锁操作数 / 第二级锁乘积
    reg [31:0] m_a, m_b;
    reg [4:0]  m_rd;
    reg [2:0]  m_idx;
    reg        m_gen;
    reg [2:0]  m_op;
    reg        m_v;

    reg [63:0] m_p;
    reg [4:0]  m_rd_q;
    reg [2:0]  m_idx_q;
    reg        m_gen_q;
    reg [2:0]  m_op_q;
    (* max_fanout = 8 *)
    reg        m_pv;

//除法：32 拍移位-相减迭代
    reg [31:0] d_dvd;             // 原被除数（除零时余数要回它）
    reg [31:0] d_a, d_b;          // 取绝对值后的被除数 / 除数
    reg        d_rem;             // 1 = 求余数
    reg        d_neg_q, d_neg_r;  // 商 / 余 需要取负
    reg        d_sgn;             // 发起那拍锁下的符号性（站式之后端口上那条下一拍就换了）
    reg        d_zero;            // 除数为 0
    reg        d_ovf;             // INT_MIN / -1
    reg        d_sign_b;
    reg [4:0]  d_rd;
    reg [2:0]  d_idx;
    reg        d_gen;
    reg [4:0]  d_cnt;
    reg        d_busy;
    reg        d_issued;          // 本条 div 已发起过（防止离开 mdu 级之前重复发起）
    reg        d_done_q;
    reg        d_done;            // 完成脉冲（比最后一次迭代晚一拍）

    reg [31:0] d_q_rem;           // 迭代中的余数累加器
    reg [31:0] d_q_quo;           // 迭代中的商

//提交链
//除法写口延迟：d_done 那拍 div 还停在 c2（stall 尚未放行），而程序序在它【前面】、
//被同一个 stall 冻在 c3/c4 的 ALU 指令，要等放行后再走两级才到写口。若除法在 d_done
//当拍就写，同 rd 时晚写的 ALU 指令会盖掉除法结果（CoreMark rv32im 的首个错误即此）。
//按 c2 → c3 → c4 → 写口 的级数对齐，除法写口落在 d_done+3。
    reg [2:0]  d_cmt_q;
    reg [4:0]  d_cmt_rd;
    reg [2:0]  d_cmt_idx;
    reg        d_cmt_gen;
    reg [31:0] d_cmt_data;

//输出保持
    reg [4:0]  hold_rd;
    reg [2:0]  idx_hold;
    reg        gen_hold;
    reg [31:0] hold_data;
    reg        hold;

    always @(*) begin
        flush_age = {1'b0, (flush_idx - head_ptr)};
        d_age     = {1'b0, (d_idx - head_ptr)};
        d_kill    = flush_w & ~(flush_part & (d_age <= flush_age));
    end

//组合逻辑（全部行为描述：reg + always @(*) 阻塞赋值）
    reg        m_push;
    reg        mul_sel;
    reg signed [32:0] a_ext, b_ext;
    reg signed [65:0] m_p_int;
    reg        d_last, d_ge;
    reg [32:0] d_sub;
    reg [31:0] d_quo_fin, d_rem_fin, d_res;
    reg        d_cmt, d_pend, wb_conflict;

//===============================================================
// 两条 ready（回站的门控）—— 只由本模块寄存器状态决定
//===============================================================
//乘法口：两级流水只要不被冻结就收（站一拍只出一条，容量天然够）。
//★ 把 wb_conflict 也算进来：冲突拍乘法两级原地冻住，此时若还收新的乘法，
//  推进 m_v 那一拍会把冻着的那条冲掉 —— "收了就必须推进"是这条契约的另一半。
    always @(*) begin
        mul_ready = ~pipe_stall & ~wb_conflict;
//除法口：FSM 空闲 + 提交链空 ⇒ 收下当拍就能发起（除法执行拍数不变：发起不晚于接收那一拍）
        div_ready = ~d_issued & ~d_busy & ~d_done & ~d_pend & ~pipe_stall;
    end

//第一级：锁操作数。判据与 lsu 的 ld_enq 同形。
//★ 接收式 = 站侧门控式（mul_ready）逐字相同 ⇒ 站撤项那一拍必定被收下，不会静默丢。
//★ 冲刷窗口内不许收：mul_go 本身就是"本拍真发出去"（issue_v 已经排掉冲刷拍）；
//  中断的边界那条【自己也已经死了】（含边界的杀集）⇒ 没有"边界要放行"这一档。
    always @(*) begin
        m_push = mul_ready && mul_go;
    end

//乘法第一级：锁操作数。pipe_stall 期间【不推进】—— 与 alu 的写回级同呼吸。
    always @(posedge clk) begin
        if (rst_q) begin
            m_v <= 1'b0;
            m_a <= 32'd0;
            m_b <= 32'd0;
            m_rd <= 5'd0;
            m_op <= 3'd0;
        end
        else if (!pipe_stall && !wb_conflict) begin
            m_v <= m_push;
            if (m_push) begin
                m_a  <= mul_r1;
                m_b  <= mul_r2;
                m_rd <= mul_rd;
                m_idx <= mul_idx_in;
                m_gen <= mul_gen_in;
                m_op <= mul_func10[2:0];
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
        a_ext   = (m_op == 3'b011) ? {1'b0, m_a} : {m_a[31], m_a};
        b_ext   = (m_op[1])        ? {1'b0, m_b} : {m_b[31], m_b};
        m_p_int = a_ext * b_ext;
    end

//乘法第二级：锁乘积。同样按 pipe_stall 冻结。这一级【不判 flush】—— 能走到这里的乘法，
//发起它的那条指令一定比正在冲刷的那条更老，必须照样提交（与 lsu 在途队列语义一致）。
    always @(posedge clk) begin
        if (rst_q) begin
            m_pv <= 1'b0;
            m_p <= 64'd0;
            m_rd_q <= 5'd0;
            m_op_q <= 3'd0;
        end
        else if (!pipe_stall && !wb_conflict) begin
            m_pv   <= m_v;
            m_p    <= m_p_int[63:0];
            m_rd_q <= m_rd;
            m_idx_q <= m_idx;
            m_gen_q <= m_gen;
            m_op_q <= m_op;
        end
    end

//funct3[1:0]==00 → MUL，取低 32；其余（MULH/MULHSU/MULHU）取高 32
    always @(*) begin
        mul_sel = (m_op_q[1:0] == 2'b00);
    end

//===============================================================
// 除法：32 拍移位-相减迭代
//===============================================================
//除法迭代一拍：余数左移一位并入被除数最高位，够减则商 1
    always @(*) begin
        d_sub = {d_q_rem[30:0], d_a[31]} - {1'b0, d_b};
        d_ge  = ~d_sub[32];
        d_last = d_busy && (d_cnt == 5'd31);
    end

//除法迭代
    always @(posedge clk) begin
        if (rst_q) begin
            d_busy <= 1'b0;
            d_issued <= 1'b0;
            d_cnt  <= 5'd0;
            d_q_rem <= 32'd0;
            d_q_quo <= 32'd0;
            d_dvd <= 32'd0;
            d_a <= 32'd0;
            d_b <= 32'd0;
            d_rd <= 5'd0;
            d_rem <= 1'b0;
            d_neg_q <= 1'b0;
            d_neg_r <= 1'b0;
            d_zero <= 1'b0;
            d_ovf <= 1'b0;
            d_sign_b <= 1'b0;
        end
        else if (d_kill) begin
            d_busy   <= 1'b0;
            d_issued <= 1'b0;
        end
        else if (d_done_q) begin
//★ 【站式改动】原来这里是 (d_done_q || !is_div)：那个 !is_div 判的是"当前摆在端口上那条不是
//  除法"，原意是"这条除法已离开 mdu 级"。原设计里除法会一直冻在门口（is_div 恒 1），只有完成
//  或冲刷才离开；站式之后【只摆一拍】——下一拍端口上就是别的指令了 ⇒ is_div 一掉，32 拍迭代
//  刚起头就自己作废，那条除法永远完不成、它的 ROB 项再也等不到完成口
//  （实测 div-01：pc 走到 0x160 后整核死等）。真正的作废由上面 flush_w 那一支负责
//  （除法无副作用，冲刷就重来）。清 d_busy 仍要有：迭代分支进不去会让它粘死。
            d_issued <= 1'b0;
            d_busy   <= 1'b0;
        end
        else if (d_busy) begin
            d_a     <= {d_a[30:0], 1'b0};
            d_q_rem <= d_ge ? d_sub[31:0] : {d_q_rem[30:0], d_a[31]};
            d_q_quo <= {d_q_quo[30:0], d_ge};
            if (d_last)
                d_busy <= 1'b0;
            else
                d_cnt <= d_cnt + 5'd1;
            if (d_cnt == 5'd0) begin
                d_zero  <= (d_b == 32'd0);
                d_ovf   <= d_sgn && (d_dvd == 32'h80000000) &&
                           (d_b == 32'd1) && d_sign_b;
                d_neg_q <= d_sgn && (d_dvd[31] ^ d_sign_b) &&
                           (d_b != 32'd0);
                d_neg_r <= d_sgn && d_dvd[31] && (d_b != 32'd0);
            end
        end
//★ 发起条件必须等操作数真的就绪：`!stall_mulu_haz`（前一条乘法的结果还没回来）与 `!pipe_stall`
//  （别的单元在停：载入用法相关、总线/缓存 hold）都要排掉。不能判 `!stall` —— stall 里含
//  `stall_v` 而 stall_v 又含 is_div 本身，判了永远发不出去（死锁）。
//★ 判据与回给站的 div_ready 逐字同源：站侧门控 = div_ready ⇒ 站撤项那一拍必定在这里发起，
//  不会出现"发出去了、没接住"（老树靠"除法冻在门口等"保证，站式之后那条前提没有了）。
        else if (div_go && div_ready) begin
//发起：有符号类先取绝对值，收尾再按符号还原。
//d_issued 保证一条 div 只发起一次；否则算完后会无限重复发起、放行判据永远落不下去。
//★ 操作数与符号位必须在【发起那一拍】从除法口锁下来：d_cnt==0 那个收尾判据在发起晚一拍执行，
//  那时端口上已经是别的指令了（站一拍只摆一条），现场读会取到别人的编码。
            d_rd     <= div_rd;
            d_idx    <= div_idx_in;
            d_gen    <= div_gen_in;
            d_rem    <= div_func10[1];
            d_sgn    <= (div_func10[0] == 1'b0);
            d_dvd    <= div_r1;
            d_sign_b <= div_r2[31];
            d_a     <= ((div_func10[0] == 1'b0) && div_r1[31]) ? (~div_r1 + 32'd1) : div_r1;
            d_b     <= ((div_func10[0] == 1'b0) && div_r2[31]) ? (~div_r2 + 32'd1) : div_r2;
            d_q_rem <= 32'd0;
            d_q_quo <= 32'd0;
            d_cnt   <= 5'd0;
            d_busy  <= 1'b1;
            d_issued <= 1'b1;
        end
    end

//收尾还原符号 + 规范边界值
//  除零 → 商全 1 / 余为原被除数；溢出 INT_MIN/-1 → 商 INT_MIN / 余 0
    always @(*) begin
        d_quo_fin = d_neg_q ? (~d_q_quo + 32'd1) : d_q_quo;
        d_rem_fin = d_neg_r ? (~d_q_rem + 32'd1) : d_q_rem;
        d_res = d_zero ? (d_rem ? d_dvd : 32'hFFFFFFFF) :
                d_ovf  ? (d_rem ? 32'd0 : 32'h80000000) :
                         (d_rem ? d_rem_fin : d_quo_fin);
    end

//完成脉冲（比最后一次迭代晚一拍）
    always @(posedge clk) begin
        if (rst_q || d_kill) begin
            d_done   <= 1'b0;
            d_done_q <= 1'b0;
        end
        else begin
            d_done   <= d_last;
            d_done_q <= d_done;
        end
    end

//===============================================================
// 提交链（d_done+3 写口）与输出保持
//===============================================================
//除法结果提交延迟链 + 输出保持（对应 lsu 的 ld_hold）
    always @(posedge clk) begin
//【只由 rst 清】不能判 flush：d_done 一旦发出，说明这条 div 已在 c2 完成并即将离级，
//此时被冲刷不会重执行，结果必须照样提交（与乘法第二级不判 flush 同理）。
        if (rst_q) begin
            d_cmt_q <= 3'd0;
        end
        else if (!pipe_stall) begin
            d_cmt_q <= {d_cmt_q[1:0], d_done};
            if (d_done) begin
                d_cmt_rd   <= d_rd;
                d_cmt_idx  <= d_idx;
                d_cmt_gen  <= d_gen;
                d_cmt_data <= d_res;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_q)
            hold <= 1'b0;
        else if (!pipe_stall && !wb_conflict) begin
            hold <= m_pv;
            if (m_pv) begin
                hold_rd   <= m_rd_q;
                idx_hold  <= m_idx_q;
                gen_hold  <= m_gen_q;
                hold_data <= mul_sel ? m_p[31:0] : m_p[63:32];
            end
        end
    end

//组合输出：写回仲裁（乘法第二级与除法提交共用一个结果口）
//★ 站式之后乘法与除法可以同时在途（两条通道各自独立收），所以"撞口"从极少数变成常态：
//  撞口拍（d_cmt & m_pv）由 wb_conflict 让【除法先走】，乘法两级原地冻一拍、结果就躺在
//  m_pv/m_p 里，不需要任何新寄存器，下一拍再报。
//★ 没有这一条时的死法：撞口拍端口上出的是乘法的号，除法那一笔的完成口被吞掉，
//  它的 ROB 项永远等不到 ⇒ 队头卡死（随机差分实测）。
    always @(*) begin
        d_cmt  = d_cmt_q[2];                          // 除法写口脉冲 = d_done+3
        d_pend = d_cmt_q[0] | d_cmt_q[1] | d_cmt_q[2]; // 除法结果前递窗口
        wb_conflict = d_cmt & m_pv;                    // 两者同拍要这唯一的口
        mul_we = m_pv | d_cmt;              // 只在提交拍（对应 lsu 的 ld_pop）
        if (d_cmt) begin
            mul_loaded   = 1'b1;
            rd_mul       = d_cmt_rd;
            mul_idx      = d_cmt_idx;
            mul_gen      = d_cmt_gen;
            mul_data_out = d_cmt_data;
        end
        else if (m_pv) begin
            mul_loaded   = 1'b1;
            rd_mul       = m_rd_q;
            mul_idx      = m_idx_q;
            mul_gen      = m_gen_q;
            mul_data_out = mul_sel ? m_p[31:0] : m_p[63:32];
        end
        else if (hold) begin
            mul_loaded   = 1'b1;
            rd_mul       = hold_rd;
            mul_idx      = idx_hold;
            mul_gen      = gen_hold;
            mul_data_out = hold_data;
        end
        else begin
            mul_loaded   = 1'b0;
            rd_mul       = 5'd0;
            mul_gen      = 1'b0;
            mul_data_out = 32'd0;
        end
    end

endmodule
