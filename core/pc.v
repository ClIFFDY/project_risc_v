`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: pc
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


module pc(
    input clk, rst,
    input [7:0] flag_bus,
    input       fifo_full,
//icache 缺失检测拍（组合直送，不走 flag_bus）：busy 晚一拍，这一拍不挡就丢一对
    input       icache_miss,
//交付口上这条指令自己的地址（icache 随指令给的；bti 注入那拍由顶层换算）
    input [31:0] cur_addr_0, cur_addr_1,
//bpu 预测：每条 lane 一组，落点一律 offset_xxx_pre_lx
    input        br1_l0, jal_l0, pre_jalr_l0, btb_hit_l0, bti_hit_l0,
                 br1_l1, jal_l1, pre_jalr_l1, btb_hit_l1, bti_hit_l1,
    input [31:0] offset_beq_pre_l0, offset_jal_pre_l0, offset_jalr_pre_l0,
                 offset_beq_pre_l1, offset_jal_pre_l1, offset_jalr_pre_l1,
//bju 判定落点不分 lane：新的 bju 指令判定成立时若还有老的 bju 指令在飞，
//ROB 会把重定向信号寄存并按住，走到这里的已是仲裁过的那一条
    input [31:0] jp_tgt,
    input        redir_go,
    input [31:0] redir_target,
    output reg [31:0] pc_addr0, pc_addr1
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}

    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    reg stall_rob_full, stall_pc_redir, stall_icache_miss, stall_bus_hold;
    reg flush_w, flush_jump_w;
    always @(*) begin
        flush_con_exc     = flag_bus[7];
        flush_con_irq     = flag_bus[6];
        flush_con_jump    = flag_bus[5];
        exec              = flag_bus[4];
        stall_rob_full    = flag_bus[3];
        stall_pc_redir    = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        stall_bus_hold    = flag_bus[0];
//跳转冲刷还要挡住"异常也来了"那一拍：异常优先，跳转改向作废
        flush_jump_w = flush_con_jump & ~flush_con_exc;
        flush_w      = flush_con_exc | flush_con_irq | flush_con_jump;
    end


//程序计数器，传递取指地址
//★ pc_addr0/pc_addr1 = lane0/lane1 各自那条指令的地址，在同一个块里按 +4 成对更新，
//  "pc_addr1 恒为 pc_addr0 + 4"由这一次赋值保证。地址随指令一起在 icache 里流水，
//  pc 不再另存一份"交付口地址"，也不再需要任何"基准+4"的推导。
    always @(posedge clk) begin
        if (rst_q) begin
            pc_addr0 <= 32'd0;
            pc_addr1 <= 32'd4;
        end
        else if (exec) begin
            if (redir_go) begin
                pc_addr0 <= redir_target;
                pc_addr1 <= redir_target + 32'd4;
            end
            else if (!flush_w && !stall_pc_redir && !stall_icache_miss && !fifo_full && !icache_miss) begin
//lane0
//两条落点差 8：命中 bti 的那一拍目标对已经由 bti 交付掉了，pc 直接跳过它落到 Tg+8；
//没命中则 bti 只交付 NOP，pc 要落到 Tg 让 icache 自己把 (Tg, Tg+4) 读出来。
//★ 基址是【交付口上这条指令自己的地址 cur_addr】，不是本拍取指地址 —— 两者平时差 8，
//  但 icache 停顿时（缺失检测拍 pc 被按住）这个差值会塌成 0，用固定的 −8 就会算错落点
//  （实测：16KB 外循环的 jal 落到 0x4004 而不是 0x400c）。
                if (br1_l0) begin
                    if (bti_hit_l0) begin
                        pc_addr0 <= cur_addr_0 + offset_beq_pre_l0 + 32'd8;
                        pc_addr1 <= cur_addr_0 + offset_beq_pre_l0 + 32'd12;
                    end
                    else begin
                        pc_addr0 <= cur_addr_0 + offset_beq_pre_l0;
                        pc_addr1 <= cur_addr_0 + offset_beq_pre_l0 + 32'd4;
                    end
                end
                else if (jal_l0) begin
                    if (bti_hit_l0) begin
                        pc_addr0 <= cur_addr_0 + offset_jal_pre_l0 + 32'd8;
                        pc_addr1 <= cur_addr_0 + offset_jal_pre_l0 + 32'd12;
                    end
                    else begin
                        pc_addr0 <= cur_addr_0 + offset_jal_pre_l0;
                        pc_addr1 <= cur_addr_0 + offset_jal_pre_l0 + 32'd4;
                    end
                end
                else if (pre_jalr_l0 & btb_hit_l0) begin
                    pc_addr0 <= offset_jalr_pre_l0;
                    pc_addr1 <= offset_jalr_pre_l0 + 32'd4;
                end
//lane1
                else if (br1_l1) begin
                    if (bti_hit_l1) begin
                        pc_addr0 <= cur_addr_1 + offset_beq_pre_l1 + 32'd8;
                        pc_addr1 <= cur_addr_1 + offset_beq_pre_l1 + 32'd12;
                    end
                    else begin
                        pc_addr0 <= cur_addr_1 + offset_beq_pre_l1;
                        pc_addr1 <= cur_addr_1 + offset_beq_pre_l1 + 32'd4;
                    end
                end
                else if (jal_l1) begin
                    if (bti_hit_l1) begin
                        pc_addr0 <= cur_addr_1 + offset_jal_pre_l1 + 32'd8;
                        pc_addr1 <= cur_addr_1 + offset_jal_pre_l1 + 32'd12;
                    end
                    else begin
                        pc_addr0 <= cur_addr_1 + offset_jal_pre_l1;
                        pc_addr1 <= cur_addr_1 + offset_jal_pre_l1 + 32'd4;
                    end
                end
                else if (pre_jalr_l1 & btb_hit_l1) begin
                    pc_addr0 <= offset_jalr_pre_l1;
                    pc_addr1 <= offset_jalr_pre_l1 + 32'd4;
                end
                else begin
                    pc_addr0 <= pc_addr0 + 32'd8;
                    pc_addr1 <= pc_addr0 + 32'd12;
                end
            end
            else if (flush_w) begin
                if (flush_jump_w) begin
                    pc_addr0 <= jp_tgt;
                    pc_addr1 <= jp_tgt + 32'd4;
                end
            end
        end
    end


endmodule
