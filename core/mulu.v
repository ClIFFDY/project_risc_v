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
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module mulu(
    input clk, rst,
    input [4:0] flag_bus,
//冻结信号的三项源（按"顶层不运算"从 cpu_top 下放至此，由本模块内部合成 pipe_stall）：
//本级的两级乘法流水、除法提交链、输出保持全部按它【冻结】，写口沿才能与 alu 的写回沿
//严格同偏移（照 lsu 对 stage 的门控手法）。不冻结的后果：icache 一 miss 就把 mul 冻在 c2，
//m_push 每拍重发 -> m_v 恒 1 -> we_mul 连续多拍。
//【合流里必须排掉本模块自己的 stall】：那正是 div 进行 / mul-use 冒险的输出，一并冻住会自锁。
    input lsu_stall, icache_busy,
    input bus_hold_in, dcache_hold,
    input [6:0] opcode,
    input [9:0] func10,
    input [4:0] rd_in, r1_post, r2_post,
    input [31:0] r1_data_final, r2_data_final,
    output reg [31:0] mul_data_out,
    output reg mul_loaded, mul_we,
    output reg [4:0] rd_mul,
    output reg stall
    );

    localparam OPCODE_OP = 7'b0110011;

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：本模块的推进由自己的 stall/pipe_stall 把关（乘除在途语义），
//不用流水线使能，故只取两条冲刷位。
    reg flush_w;
    always @(*) flush_w = flag_bus[3] | flag_bus[2];

//冻结信号合流：总线保持（外部 + dcache 延拓拍）与取指缺失、以及 lsu 的 load-use
    reg pipe_stall, bus_hold;
    always @(*) begin
        bus_hold   = bus_hold_in | dcache_hold;
        pipe_stall = lsu_stall | icache_busy | bus_hold;
    end

//===============================================================
// 乘法：两级流水寄存器
//===============================================================
    reg [31:0] m_a, m_b;
    reg [4:0]  m_rd;
    reg [2:0]  m_op;
    reg        m_v;

    reg [63:0] m_p;
    reg [4:0]  m_rd_q;
    reg [2:0]  m_op_q;
    reg        m_pv;

//===============================================================
// 除法：32 拍移位-相减迭代
//===============================================================
    reg [31:0] d_dvd;             // 原被除数（除零时余数要回它）
    reg [31:0] d_a, d_b;          // 取绝对值后的被除数 / 除数
    reg        d_rem;             // 1 = 求余数
    reg        d_neg_q, d_neg_r;  // 商 / 余 需要取负
    reg        d_zero;            // 除数为 0
    reg        d_ovf;             // INT_MIN / -1
    reg [4:0]  d_rd;
    reg [4:0]  d_cnt;
    reg        d_busy;
    reg        d_issued;          // 本条 div 已发起过（防止离开 mulu 级之前重复发起）
    reg        d_done;            // 完成脉冲（比最后一次迭代晚一拍）

    reg [31:0] d_q_rem;           // 迭代中的余数累加器
    reg [31:0] d_q_quo;           // 迭代中的商

//===============================================================
// 输出保持
//===============================================================
    reg [4:0]  hold_rd;
    reg [31:0] hold_data;
    reg        hold;

//除法写口延迟：d_done 那拍 div 还停在 c2（stall 尚未放行），而程序序在它【前面】、
//被同一个 stall 冻在 c3/c4 的 ALU 指令，要等放行后再走两级才到写口。若除法在 d_done
//当拍就写，同 rd 时晚写的 ALU 指令会盖掉除法结果（CoreMark rv32im 的首个错误即此）。
//按 c2 → c3 → c4 → 写口 的级数对齐，除法写口落在 d_done+3。
    reg [2:0]  d_cmt_q;
    reg [4:0]  d_cmt_rd;
    reg [31:0] d_cmt_data;

//===============================================================
// 冒险判定
//===============================================================
    reg [6:0] opcode_post;
    reg [9:0] func10_post;
    reg [4:0] rd_post;
    reg       mstalled;

//===============================================================
// 组合逻辑（全部行为描述：reg + always @(*) 阻塞赋值）
//===============================================================
    reg        is_m, is_mul, is_div;
    reg        is_m_post, is_mul_post;
    reg        m_push;
    reg        mul_sel;
    reg signed [32:0] a_ext, b_ext;
    reg signed [65:0] m_p_int;
    reg        d_last, d_ge;
    reg [32:0] d_sub;
    reg [31:0] d_quo_fin, d_rem_fin, d_res;
    reg        d_cmt, d_pend;

//RV32M 判据：与普通 ALU 指令共用 OPCODE_OP，只能靠 funct7 区分
//func10 = {inst[31:25], inst[14:12]}，M 的 inst[31:25] = 0000001
//funct3[1:0]：乘法 00=MUL 01=MULH 10=MULHSU 11=MULHU；除法 00=DIV 01=DIVU 10=REM 11=REMU
    always @(*) begin
        is_m         = (opcode == OPCODE_OP) && (func10[9:3] == 7'b0000001);
        is_mul       = is_m && (func10[2] == 1'b0);   // funct3 0xx：MUL 家族
        is_div       = is_m && (func10[2] == 1'b1);   // funct3 1xx：DIV 家族
        is_m_post    = (opcode_post == OPCODE_OP) && (func10_post[9:3] == 7'b0000001);
        is_mul_post  = is_m_post && (func10_post[2] == 1'b0);
    end

//第一级：锁操作数。判据 !stall / !flush_w / !bus_hold_in 与 lsu 的 ld_enq 同形
    always @(*) begin
        m_push = is_mul && !stall && !pipe_stall && !flush_w && !bus_hold;
    end

//33 位扩展：
//  a 无符号 ⟺ funct3==011 (MULHU)
//  b 无符号 ⟺ funct3∈{010,011} (MULHSU/MULHU) ⟺ funct3[1]==1
    always @(*) begin
        a_ext   = (m_op == 3'b011) ? {1'b0, m_a} : {m_a[31], m_a};
        b_ext   = (m_op[1])        ? {1'b0, m_b} : {m_b[31], m_b};
        m_p_int = a_ext * b_ext;
    end

//除法迭代一拍：余数左移一位并入被除数最高位，够减则商 1
    always @(*) begin
        d_sub = {d_q_rem[30:0], d_a[31]} - {1'b0, d_b};
        d_ge  = ~d_sub[32];
        d_last = d_busy && (d_cnt == 5'd31);
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

//funct3[1:0]==00 → MUL，取低 32；其余（MULH/MULHSU/MULHU）取高 32
    always @(*) begin
        mul_sel = (m_op_q[1:0] == 2'b00);
    end

//===============================================================
// 时序逻辑
//===============================================================
//乘法第一级：锁操作数。pipe_stall 期间【不推进】—— 与 alu 的写回级同呼吸。
    always @(posedge clk) begin
        if (rst) begin
            m_v <= 1'b0;
            m_a <= 32'd0;  m_b <= 32'd0;  m_rd <= 5'd0;  m_op <= 3'd0;
        end
        else if (!pipe_stall) begin
            m_v <= m_push;
            if (m_push) begin
                m_a  <= r1_data_final;
                m_b  <= r2_data_final;
                m_rd <= rd_in;
                m_op <= func10[2:0];
            end
        end
    end

//乘法第二级：锁乘积。同样按 pipe_stall 冻结。这一级【不判 flush】—— 能走到这里的乘法，
//发起它的那条指令一定比正在冲刷的那条更老，必须照样提交（与 lsu 在途队列语义一致）。
    always @(posedge clk) begin
        if (rst) begin
            m_pv <= 1'b0;
            m_p <= 64'd0;  m_rd_q <= 5'd0;  m_op_q <= 3'd0;
        end
        else if (!pipe_stall) begin
            m_pv   <= m_v;
            m_p    <= m_p_int[63:0];
            m_rd_q <= m_rd;
            m_op_q <= m_op;
        end
    end

//除法迭代
    always @(posedge clk) begin
        if (rst) begin
            d_busy <= 1'b0;
            d_issued <= 1'b0;
            d_cnt  <= 5'd0;
            d_q_rem <= 32'd0;
            d_q_quo <= 32'd0;
            d_dvd <= 32'd0;  d_a <= 32'd0;  d_b <= 32'd0;
            d_rd <= 5'd0;  d_rem <= 1'b0;
            d_neg_q <= 1'b0; d_neg_r <= 1'b0;
            d_zero <= 1'b0;  d_ovf <= 1'b0;
        end
        else if (flush_w) begin
//除法无副作用，被冲刷就整体作废，重取指后会重新执行
            d_busy <= 1'b0;
        end
        else if (!is_div) begin
//指令离开 mulu 级 → 清"已发起"
            d_issued <= 1'b0;
        end
        else if (d_busy) begin
            d_a     <= {d_a[30:0], 1'b0};
            d_q_rem <= d_ge ? d_sub[31:0] : {d_q_rem[30:0], d_a[31]};
            d_q_quo <= {d_q_quo[30:0], d_ge};
            if (d_last) d_busy <= 1'b0;
            else        d_cnt  <= d_cnt + 5'd1;
        end
        else if (is_div && !d_issued && !bus_hold) begin
//发起：有符号类先取绝对值，收尾再按符号还原。
//【不能判 !stall】—— stall 里含 is_div 本身，判了就永远发不出去（死锁）。
//d_issued 保证一条 div 只发起一次；否则算完后 is_div 仍在（指令还冻在 mulu 级），
//会无限重复发起、stall 永远落不下去。
            d_rd    <= rd_in;
            d_rem   <= func10[1];
            d_zero  <= (r2_data_final == 32'd0);
            d_ovf   <= (func10[0] == 1'b0) && (r1_data_final == 32'h80000000) &&
                       (r2_data_final == 32'hFFFFFFFF);
            d_neg_q <= (func10[0] == 1'b0) && (r1_data_final[31] ^ r2_data_final[31]) &&
                       (r2_data_final != 32'd0);
            d_neg_r <= (func10[0] == 1'b0) && r1_data_final[31] && (r2_data_final != 32'd0);
            d_dvd   <= r1_data_final;
            d_a     <= ((func10[0] == 1'b0) && r1_data_final[31]) ? (~r1_data_final + 32'd1) : r1_data_final;
            d_b     <= ((func10[0] == 1'b0) && r2_data_final[31]) ? (~r2_data_final + 32'd1) : r2_data_final;
            d_q_rem <= 32'd0;
            d_q_quo <= 32'd0;
            d_cnt   <= 5'd0;
            d_busy  <= 1'b1;
            d_issued <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst || flush_w) d_done <= 1'b0;
        else              d_done <= d_last;
    end

//除法结果提交延迟链 + 输出保持（对应 lsu 的 ld_hold）
    always @(posedge clk) begin
//【只由 rst 清】不能判 flush：d_done 一旦发出，说明这条 div 已在 c2 完成并即将离级，
//此时被冲刷不会重执行，结果必须照样提交（与乘法第二级不判 flush 同理）。
        if (rst) begin
            d_cmt_q <= 3'd0;
        end
        else if (!pipe_stall) begin
            d_cmt_q <= {d_cmt_q[1:0], d_done};
            if (d_done) begin
                d_cmt_rd   <= d_rd;
                d_cmt_data <= d_res;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) hold <= 1'b0;
        else if (!pipe_stall) begin
            hold <= m_pv;
            if (m_pv) begin
                hold_rd   <= m_rd_q;
                hold_data <= mul_sel ? m_p[31:0] : m_p[63:32];
            end
        end
    end

//冒险用的载荷寄存器（M 与普通 ALU 共用 OPCODE_OP，所以 func10 必须一起寄存）
    always @(posedge clk) begin
        if (rst) begin
            opcode_post <= 7'd0;
            func10_post <= 10'd0;
            rd_post     <= 5'd0;
            mstalled    <= 1'b0;
        end
        else begin
            opcode_post <= opcode;
            func10_post <= func10;
            if (m_push)                rd_post  <= rd_in;
            else                       rd_post  <= rd_post;
            if (stall)                 mstalled <= 1'b1;
            else if (mstalled && m_pv) mstalled <= 1'b0;
            else                       mstalled <= mstalled;
        end
    end

//===============================================================
// 组合输出：写回仲裁（乘法第二级与除法收尾共用，同一时刻只有一个在途结果）
//===============================================================
    always @(*) begin
        d_cmt  = d_cmt_q[2];                          // 除法写口脉冲 = d_done+3
        d_pend = d_cmt_q[0] | d_cmt_q[1] | d_cmt_q[2]; // 除法结果前递窗口
        mul_we = m_pv | d_cmt;              // 只在提交拍（对应 lsu 的 ld_pop）
        if (m_pv) begin
            mul_loaded   = 1'b1;
            rd_mul       = m_rd_q;
            mul_data_out = mul_sel ? m_p[31:0] : m_p[63:32];
        end
        else if (d_pend) begin
            mul_loaded   = 1'b1;
            rd_mul       = d_cmt_rd;
            mul_data_out = d_cmt_data;
        end
        else if (hold) begin
            mul_loaded   = 1'b1;
            rd_mul       = hold_rd;
            mul_data_out = hold_data;
        end
        else begin
            mul_loaded   = 1'b0;
            rd_mul       = 5'd0;
            mul_data_out = 32'd0;
        end
    end

//冒险：照 lsu 的 load-use 形状
    always @(*) begin
//除法：从"div 还在 mulu 级且尚未发起"那拍起一直停到收尾后。
//d_issued 置起后本项让位给 d_busy/d_done，算完就放行，指令才能离开 mulu 级。
        if (is_div && !d_issued && !bus_hold)
            stall = 1'b1;
        else if (d_busy || d_done)
            stall = 1'b1;
//乘法：前一条是 MUL 且当前指令要用它的 rd → 停 1 拍，结果到了就放
        else if (is_mul_post && ((rd_post == r1_post) | (rd_post == r2_post)))
            stall = (mstalled && m_pv) ? 1'b0 : 1'b1;
        else
            stall = 1'b0;
    end

endmodule
