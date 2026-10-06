`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: mulu
// Project Name:
// Target Devices:
// Tool Versions:
// Description: RV32M 乘除法单元，与 lsu 流水线行为【同步】。
//
//   关键约束（用户给定，本模块据此设计）：mulu 的提交时刻必须与 lsu 严格同偏移，
//   这样 regfile 的三个写口永不共拍，不需要写口优先级仲裁。
//   lsu 的读时序：T（指令在 lsu/decoder 级）→ T+1 地址上总线 → T+2 应答 → 写沿 T+3。
//   所以乘法做成【两级】正好对齐：
//     T      ：指令在 mulu 级，组合判 is_m
//     T→T+1  ：锁操作数 a/b/op
//     T+1    ：33×33 有符号乘法（组合，综合成 DSP48）
//     T+1→T+2：锁乘积
//     T+2    ：mul_we / mul_loaded 拉高 → regfile 在 T+2→T+3 沿写（= lsu 的写沿）
//     T+3    ：hold（mul_loaded 第二拍，对应 lsu 的 ld_hold）
//   背靠背乘法 1 笔/拍（两级流水不冲突）。
//
//   除法是 32 拍的移位-相减迭代，组合实现会直接成为新的最差路径，故走 FSM + stall；
//   期间把整条流水线冻住（提交偏移与乘法不同，但一直停着，不会有别的写口同拍）。
//
//   乘法用【一个】33×33 有符号乘法器覆盖全部 4 条：把两个操作数按需扩展成 33 位
//   （有符号补符号位、无符号补 0）再相乘，MUL 取低 32、MULH/MULHSU/MULHU 取高 32。
//
//   操作数由 regfile 的【独立第三份读数据通路】供给（_mul），不与其他消费单元共享。
//
//   本文件内 always 块按【流水级数】排列：
//     第一级（锁操作数）→ 第二级（锁乘积）→ 除法迭代 → 提交链/输出保持
//     → 冒险判定与 stall 输出 → 组合输出（写回仲裁）。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module mulu(
    input clk, rst,
    input [11:0] flag_bus,
//本次冲刷的边界（controller 的 flush_idx：三种冲刷各取自己那条的号）：
//入口门按它判"本拍站在载荷上的这条，是边界自己、还是比边界更年轻的错路条"。
    input [2:0]  flush_idx,
//写口级"这一笔已处置（落地/被杀/不写）"：本模块据此撤值放行；没被取走就一直举着
    input        taken_mul,
//入口门（每条指令只收一次）：decoder 载荷【这一拍就要推进】才允许收（理由同 lsu：
//  用上一拍的脉冲会让"被挤住那条"在新的一拍里成了上一拍的旧货 ⇒ 静默丢掉）
    input payload_go,
//前送命中判据（消费者载荷里锁存的槽号/有效位，与 forw 当拍看到的同一份）：本模块结果口这一拍供的值是不是它的。
//★ 三条结果路（m_pv / d_pend / hold）的数据都是组合给出的 ⇒ 命中位也走组合、与 `mul_idx` 逐支同步
//  （寄一份反而会和数据错开一拍）。与消费者同一拍 ⇒ 用【载荷里锁存的】槽号比，不用当拍扫描。
    input [2:0]  sel_slot1, sel_slot2,
    input [2:0]  sel_slot3, sel_slot4,
    input        sel_v1,    sel_v2,
    input        sel_v3,    sel_v4,
//冻结信号从 flag_bus 取位（本模块不设专用 stall 端口）：
//本级的两级乘法流水、除法提交链、输出保持全部按它【冻结】，写口沿才能与 alu 的写回沿
//严格同偏移（照 lsu 对 stage 的门控手法）。不冻结的后果：icache 一 miss 就把 mul 冻在 c2，
//m_push 每拍重发 -> m_v 恒 1 -> we_mul 连续多拍。
//【合流里必须排掉本模块自己的两条 stall（stall_mulu_haz/div）】：那正是 div 进行 /
//mul-use 冒险的输出，一并冻住会自锁。
    input [6:0] opcode,
    input [9:0] func10,
    input [4:0] rd_in, r1_post, r2_post,
//lane1 的两个源：它自己不可能是 M 类，但可能读到在途乘法的 rd ⇒ 冒险要带上它。
//★ 独立成项、只或进广播，`stall_mulu_haz` 原式一字不动。
    input [4:0] r3_post, r4_post,
//写序号（与 rd_in 同沿锁进本级）与当前最新号（滞留兜底用）
    input [2:0]  idx_in,
//那一项的世代位：与 idx 全程同行，完成上报时带回 ROB 做身份校验（见 rob.v 文件头）
    input        gen_in,
//落地广播（来自写回级两个写口）：据此把更老的同 rd 在途写（乘法/除法）作废
    input [31:0] r1_data_final, r2_data_final,
    (* max_fanout = 8 *) output reg [31:0] mul_data_out,
    (* max_fanout = 8 *) output reg mul_loaded, mul_we,
//本模块这一拍供的值是不是消费者的（r1/r2 各一位）：与 mul_idx 逐支同步，供 forw 直接选源
    output reg hit1, hit2,
    output reg hit3, hit4,
    (* max_fanout = 8 *) output reg [4:0] rd_mul,
//本条写回记录带的写序号（跟着数据走，写回级用它判谁更老）
    output reg [2:0]  mul_idx,
    (* max_fanout = 8 *) output reg        mul_gen,
//两条停顿源【逐条】对外：controller 原样过路进 flag_bus，或运算在消费者模块内做
    output reg stall_mulu_haz,
    output reg stall_mulu_div
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam OPCODE_OP = 7'b0110011;



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
    always @(*) begin
        flush_con_exc = flag_bus[11];
        flush_w = flush_con_exc | flag_bus[10] | flag_bus[9];
    end

//入口门用的两条按【编号】的判据（口径同 lsu.v）：
//  · 跳转/分支非对齐：flush_idx = 分支自己 ⇒ 冲刷拍站在载荷上的后继比它年轻 ⇒ 挡；
//  · 中断：flush_idx = issue_idx_in = 载荷这条自己（与本模块的 idx_in 同源）⇒ 放行 ——
//    它是【已被 post_decoder 发出、ROB 要留下】的那条，挡住它就永远拿不到 fin_mul。
    reg ent_bnd, ent_young;
    always @(*) begin
        ent_bnd   = flush_w & (idx_in == flush_idx);
        ent_young = flush_w & (idx_in != flush_idx);
    end

//冻结信号合流（本模块是消费者，或运算在这里做）：
//  bus_hold   = 总线不可用（外部 hold）
//  pipe_stall = 总线保持 / 取指缺失 / lsu 那三条（排掉自己的 haz/div 两位）
//★ dcache 忙(2) 已撤出全核广播：乘法与 dcache 无关，本模块不再吃它（它只在 lsu 内部生效）。
//★ 不要并进 stall_rob_full(9) / stall_pc_redir(8)：ROB 排空要靠队头那条乘法完成，
//  而乘法正被它挡在 mulu 门外 ⇒ ROB 永不排空 ⇒ 死锁（与 lsu 入口门同一个坑）。
    reg pipe_stall, bus_hold;
//写口那一笔还没被取走 ⇒ 本模块整条冻住（结果连着乘积一起保持，不许被下一笔覆盖）。
    reg wr_pend;
    always @(*) begin
        bus_hold   = flag_bus[0];
        pipe_stall = flag_bus[5] | flag_bus[4]
                   | flag_bus[1] | flag_bus[0];
    end

//寄存器声明（按级分组）
//乘法：第一级锁操作数 / 第二级锁乘积
    reg [31:0] m_a, m_b;
    reg [4:0]  m_rd;
    reg [2:0]  m_idx;
    reg        m_gen;
    reg [2:0]  m_op;
    reg        m_v;
    reg        stall_mulu_haz_self, stall_mulu_haz_1;

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
    reg        d_zero;            // 除数为 0
    reg        d_ovf;             // INT_MIN / -1
    reg        d_sign_b;
    reg [4:0]  d_rd;
    reg [2:0]  d_idx;
    reg        d_gen;
    reg [4:0]  d_cnt;
    reg        d_busy;
    reg        d_issued;          // 本条 div 已发起过（防止离开 mulu 级之前重复发起）
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

//冒险判定
    reg [4:0] rd_post;
    reg       mstalled;
    reg        div_pend_v;
    reg [4:0]  div_pend_rd;

//组合逻辑（全部行为描述：reg + always @(*) 阻塞赋值）
    reg        is_m, is_mul, is_div;
    reg        stall;
    reg        m_push;
    reg        mul_sel;
    reg signed [32:0] a_ext, b_ext;
    reg signed [65:0] m_p_int;
    reg        d_last, d_ge;
    reg [32:0] d_sub;
    reg [31:0] d_quo_fin, d_rem_fin, d_res;
    reg        d_cmt, d_pend;

//===============================================================
// 第一级：锁操作数
//===============================================================
//RV32M 判据：与普通 ALU 指令共用 OPCODE_OP，只能靠 funct7 区分
//func10 = {inst[31:25], inst[14:12]}，M 的 inst[31:25] = 0000001
//funct3[1:0]：乘法 00=MUL 01=MULH 10=MULHSU 11=MULHU；除法 00=DIV 01=DIVU 10=REM 11=REMU
    always @(*) begin
        is_m         = (opcode == OPCODE_OP) && (func10[9:3] == 7'b0000001);
        is_mul       = is_m && (func10[2] == 1'b0);   // funct3 0xx：MUL 家族
        is_div       = is_m && (func10[2] == 1'b1);   // funct3 1xx：DIV 家族
    end

//第一级：锁操作数。判据 !stall / !flush_w / !bus_hold_in 与 lsu 的 ld_enq 同形。
//★ 冲刷只挡【比边界更年轻】的错路条，边界自己放行（理由与 lsu 的 new_in_pre 同）。
    always @(*) begin
        m_push = is_mul && !stall && !pipe_stall && !ent_young && !bus_hold && !wr_pend
               && (payload_go | ent_bnd);
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
        else if (!pipe_stall && !wr_pend) begin
            m_v <= m_push;
            if (m_push) begin
                m_a  <= r1_data_final;
                m_b  <= r2_data_final;
                m_rd <= rd_in;
                m_idx <= idx_in;
                m_gen <= gen_in;
                m_op <= func10[2:0];
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
        else if (!pipe_stall && !wr_pend) begin
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
        else if (flush_w) begin
//除法无副作用，被冲刷就整体作废，重取指后会重新执行
            d_busy <= 1'b0;
        end
        else if (d_done_q || !is_div) begin
//指令离开 mulu 级 → 清"已发起"
//★ 必须同时清 d_busy：若 is_div 在迭代中掉了（本级换了指令），只清 d_issued 会让
//  下面的迭代分支再也进不去 ⇒ d_busy 粘死 ⇒ stall_mulu_div 永久为 1 ⇒ 前端永久冻死（实测抓到）。
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
                d_ovf   <= (func10[0] == 1'b0) && (d_dvd == 32'h80000000) &&
                           (d_b == 32'd1) && d_sign_b;
                d_neg_q <= (func10[0] == 1'b0) && (d_dvd[31] ^ d_sign_b) &&
                           (d_b != 32'd0);
                d_neg_r <= (func10[0] == 1'b0) && d_dvd[31] && (d_b != 32'd0);
            end
        end
//★ 发起条件必须等操作数真的就绪：`!stall_mulu_haz`（前一条乘法的结果还没回来）与 `!pipe_stall`
//  （别的单元在停：载入用法相关、总线/缓存 hold）都要排掉。不能判 `!stall` —— stall 里含
//  `stall_v` 而 stall_v 又含 is_div 本身，判了永远发不出去（死锁）。
        else if (is_div && !d_issued && !bus_hold && !stall_mulu_haz_self && !pipe_stall && (d_cmt_q == 3'd0)) begin
//发起：有符号类先取绝对值，收尾再按符号还原。
//【不能判 !stall】—— stall 里含 is_div 本身，判了就永远发不出去（死锁）。
//d_issued 保证一条 div 只发起一次；否则算完后 is_div 仍在（指令还冻在 mulu 级），
//会无限重复发起、stall 永远落不下去。
//★ 临时调试（验完删）：把发起那一拍的输入原样锁进寄存器，供 tb 事后打印（避免组合竞争）
            d_rd     <= rd_in;
            d_idx    <= idx_in;
            d_gen    <= gen_in;
            d_rem    <= func10[1];
            d_dvd    <= r1_data_final;
            d_sign_b <= r2_data_final[31];
            d_a     <= ((func10[0] == 1'b0) && r1_data_final[31]) ? (~r1_data_final + 32'd1) : r1_data_final;
            d_b     <= ((func10[0] == 1'b0) && r2_data_final[31]) ? (~r2_data_final + 32'd1) : r2_data_final;
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
        if (rst_q || flush_w) begin
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
        else if (!pipe_stall && !wr_pend) begin
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
        else if (!pipe_stall && !wr_pend) begin
            hold <= m_pv;
            if (m_pv) begin
                hold_rd   <= m_rd_q;
                idx_hold  <= m_idx_q;
                gen_hold  <= m_gen_q;
                hold_data <= mul_sel ? m_p[31:0] : m_p[63:32];
            end
        end
    end

//冒险判定（第二级载荷 + stall 输出）
    always @(posedge clk) begin
        if (rst_q) begin
            rd_post     <= 5'd0;
            mstalled    <= 1'b0;
        end
        else begin
            if (m_push)
                rd_post <= rd_in;
            else
                rd_post <= rd_post;
            if (stall)
                mstalled <= 1'b1;
            else if (mstalled && m_pv)
                mstalled <= 1'b0;
            else
                mstalled <= mstalled;
        end
    end

//冒险：照 lsu 的 load-use 形状
    always @(*) begin
//除法：从"div 还在 mulu 级且尚未发起"那拍起一直停到收尾后。
//d_issued 置起后本项让位给 d_busy/d_done，算完就放行，指令才能离开 mulu 级。
        if (is_div && !d_issued && !bus_hold)
            stall_mulu_div = 1'b1;
        else if (d_busy || d_done)
            stall_mulu_div = 1'b1;
        else
            stall_mulu_div = 1'b0;
    end

//乘法：前一条是 MUL 且当前指令要用它的 rd → 停 1 拍，结果到了就放
//（结果还没上退口的那一格由 forw 点② 的【在途支路】覆盖，压在前端的那一拍不再需要）
//lane1 的同形判据：独立成项，`stall_mulu_haz` 原式一字不动。
    always @(*) begin
        if ((rd_post == r3_post) | (rd_post == r4_post))
            stall_mulu_haz_1 = m_v ? 1'b1 : 1'b0;
        else
            stall_mulu_haz_1 = 1'b0;
    end

    always @(*) begin
        if ((rd_post == r1_post) | (rd_post == r2_post))
            stall_mulu_haz_self = m_v ? 1'b1 : 1'b0;
        else
            stall_mulu_haz_self = 1'b0;
    end

//对外广播的那一份 = 本来的项 | lane1 那一项。
//★ 模块内部（除法发起条件）用的仍只有 `stall_mulu_haz_self` —— 那是"本条自己操作数没就绪"，
//  与 lane1 的读无关，混进去会让除法发起条件多一个无关项。
    always @(*) begin
        stall_mulu_haz = stall_mulu_haz_self | stall_mulu_haz_1;
    end

//  口被 alu 占住时乘法让路是常态里的极少数（三笔同拍才轮到它让），量级可忽略。
    always @(*) wr_pend = (m_pv && ~taken_mul) | (d_cmt_q[2] && ~taken_mul);

//本模块内部用的合流（不回压任何人，只用于 m_push 与 mstalled）：两条停顿源相或。
//旧 wbu 的 hold_mul 端口与它对应的回压已随 ROB 取代 wbu 而作废，整条删除。
    always @(*) stall = stall_mulu_haz | stall_mulu_div;

//组合输出：写回仲裁（乘法第二级与除法收尾共用，同一时刻只有一个在途结果）
    always @(*) begin
        d_cmt  = d_cmt_q[2];                          // 除法写口脉冲 = d_done+3
        d_pend = d_cmt_q[0] | d_cmt_q[1] | d_cmt_q[2]; // 除法结果前递窗口
        mul_we = m_pv | d_cmt;              // 只在提交拍（对应 lsu 的 ld_pop）
        if (m_pv) begin
            mul_loaded   = 1'b1;
            rd_mul       = m_rd_q;
            mul_idx      = m_idx_q;
            mul_gen      = m_gen_q;
            mul_data_out = mul_sel ? m_p[31:0] : m_p[63:32];
            hit1         = sel_v1 & (m_idx_q == sel_slot1);
            hit2         = sel_v2 & (m_idx_q == sel_slot2);
            hit3         = sel_v3 & (m_idx_q == sel_slot3);
            hit4         = sel_v4 & (m_idx_q == sel_slot4);
        end
        else if (d_pend) begin
            mul_loaded   = 1'b1;
            rd_mul       = d_cmt_rd;
            mul_idx      = d_cmt_idx;
            mul_gen      = d_cmt_gen;
            mul_data_out = d_cmt_data;
            hit1         = sel_v1 & (d_cmt_idx == sel_slot1);
            hit2         = sel_v2 & (d_cmt_idx == sel_slot2);
            hit3         = sel_v3 & (d_cmt_idx == sel_slot3);
            hit4         = sel_v4 & (d_cmt_idx == sel_slot4);
        end
        else if (hold) begin
            mul_loaded   = 1'b1;
            rd_mul       = hold_rd;
            mul_idx      = idx_hold;
            mul_gen      = gen_hold;
            mul_data_out = hold_data;
            hit1         = sel_v1 & (idx_hold == sel_slot1);
            hit2         = sel_v2 & (idx_hold == sel_slot2);
            hit3         = sel_v3 & (idx_hold == sel_slot3);
            hit4         = sel_v4 & (idx_hold == sel_slot4);
        end
        else begin
            mul_loaded   = 1'b0;
            rd_mul       = 5'd0;
            mul_gen      = 1'b0;
            mul_data_out = 32'd0;
            hit1         = 1'b0;
            hit2         = 1'b0;
            hit3         = 1'b0;
            hit4         = 1'b0;
        end
    end

endmodule
