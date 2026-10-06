`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/22
// Design Name:
// Module Name: bju
// Project Name:
// Target Devices:
// Tool Versions:
// Description: 分支/跳转判定单元（branch-jump unit）。两级：
//              ① 判定输入寄存级 —— 操作数与控制位一起寄存（操作数来自 forw 的组合输出，
//                 直接吃会让"计算单元→前送→操作数→比较/加法"串成一条全片最差链）；
//              ② 判定级 —— 比较与加法，结果/落点/预测表限定信号在下一拍生效；
//              另给出一条组合版前置冲刷 flush_bju_pre（判定级的当拍版本），只有 lsu/mulu 消费。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module bju(
    input clk, rst,
    input [11:0] flag_bus,
//判定源：与 alu 的输入同源（decoder 本级的寄存器输出）
    input [31:0] r1_data_in, r2_data_in,
    input [3:0] alu_func4_in,
    input br_flag_in, jalr_flag_in,
    input [31:0] aux_addr_in, beq_off_in, jalr_pred_addr_in,
    input br_pred_taken_in,
    input exc_jal_misalign_in,
    input [31:0] jal_target_in,
//异常源（从 post_decoder 载荷进来）：与操作数/控制位【同沿】锁进判定输入寄存级。
//这样它们天然比载荷晚一拍出，正好与"更老那条的寄存版冲刷"【同拍配对】——
//  本拍出的是 X_m 的异常，本拍 flag_bus[9]/flush_bju_exc 出的是 X_{m-1} 的冲刷，
//  于是消费端用现成的 ~flush_older 就盖住了影子槽，不再需要组合的 pre。
    input exc_ecall_in, exc_ebreak_in, exc_illegal_in, exc_irq_ret_in,
//判定结果与跳转落点（寄存一拍后有效）
    output reg success, br_fail, jalr_fail,
//本拍被判那条指令的 ROB 索引（判定打一拍 ⇒ 冲刷边界必须用【寄存后的】这个号）：
//ROB 的冲刷只作废【比它更年轻】的项，比它老、还在途的（load/mul）必须留下，
//否则它们的写回/总线请求就被连同错路指令一起扔了（实测 CoreMark：两条在途 lw 的写回消失）。
    input [2:0]  idx_in,
    output reg [2:0] idx_q,
    output reg [31:0] jp_target, jalr_target_q2,
    output reg [5:0] br_pc_idx,
    output reg flush_bju_exc,
    output reg jalr_flag_q, br_pred_taken_q,
//F1：被判那条自己的地址（"地址+4"）—— 必须与 jp_target **同块同清**地寄存一份。
//  不能用输入级的 aux_q（它每拍都装，冲刷拍里已经是下一条的地址 ⇒ mepc 会错 4）。
//判定结果的【组合版】（判定级当拍）：只给 wport 做"当拍撤销"—— 故障指令（jalr 的 link）
//  的写口登记就发生在这一拍，任何寄存版都晚一拍、撤不掉。
    output reg exc_bju,
//判定输入级那一条自己的 ROB 号（与 exc_bju 同拍）：wport 用它认"本级挂的是不是它"
    output reg [2:0] idx_i,
    output reg [31:0] exc_pc_q,
//判定【输入级】那一条自己带的异常（与 flush_bju_pre 同拍、描述同一条）：ecall/ebreak/illegal/mret。
//★ 必须从【输入寄存级】出、不能从延迟寄存级出：从延迟级出就与"更老那条的寄存版冲刷"错开两拍，
//  消费端的 ~flush_older 盖不住影子槽（那正是当初要 pre 的原因）。
    output reg exc_ecall_i, exc_ebreak_i, exc_illegal_i, exc_irq_ret_i,
//同一条自己的 PC（= 输入级的 aux_q），给 mepc 用
    output reg [31:0] exc_pc_i,
//F2：判定输入拍采到的"更老指令正在冲刷"（flag_bus[10]|flag_bus[9]）。
//  寄存判定那一拍上 flag_bus[9] 就是【这条指令自己】的跳转冲刷 ⇒ 拿当拍的 flush_older
//  去门标记会自己掐掉自己（br 预测不跳+真跳+非对齐那一支的异常会静默丢失）。
    output reg older_q,
//前置冲刷：同一判定的组合版本，早一拍，只喂 lsu/mulu
    output reg flush_bju_pre
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：判定延迟拍只关心"本拍是不是冲刷拍"。
//★ flush_con_exc 已经是精确版（不再含 pre）⇒ 直接取 [11]。
    reg flush_w;
    always @(*) flush_w = flag_bus[11] | flag_bus[10] | flag_bus[9];

//操作数装载门控：与边界推进（post_decoder 的 payload_go）同一组位 ——
//stall 期间一位都不许动，能装进来的那一刻，源已经就绪。
    reg adv;
    always @(*) adv = flag_bus[8] & ~(flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4]
                                    | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]);

//===============================================================
// 判定输入寄存级（新增）：把"这一拍要判的那条指令"整个寄存一级
//===============================================================
//为什么要有这一级：判定的操作数原来直接吃 forw 的【组合输出】（r*_data_final），
//于是"计算单元 → 前送年龄比较/mux → 操作数 → 比较/加法 → jp_target/mtval"串成一条
//实测 15.6ns 的链（逻辑只占 3ns，全是线）。操作数在这里落一次寄存器，链就被劈成两段。
//★ 控制位必须跟着一起寄存：操作数晚一拍、控制位不晚，就会拿"下一条的 br_flag"去判"这一条"。
//★ 装载严格受 stall 门控（与边界推进同一条件）：stall 期间不装，装进来的那一刻源已就绪 ——
//  进本级的操作数不受任何 stall 门控，判定/发表/前置冲刷照常出。
//★ 冲刷拍必须清：否则错路那条会在下一拍被"补判"一次，变成冲刷后多拉一拍重定向。
    reg [31:0] r1_q, r2_q, aux_q, beq_q, jalr_pred_q, jal_tgt_q;
    reg [3:0]  func4_q;
    reg        brf_q, jalrf_q, pred_q, exc_jal_mis_q;
//异常源也进这一级（与操作数/控制位同生共死）—— 这是"过 bju"的核心：它们比载荷晚一拍出，
//正好与【更老那条】的寄存版冲刷同拍。
    reg        ecall_q, ebreak_q, illegal_q, irq_ret_q;
    always @(posedge clk) begin
        if (rst_q) begin
            r1_q <= 32'd0;
            r2_q <= 32'd0;
            aux_q <= 32'd0;
            beq_q <= 32'd0;
            jalr_pred_q <= 32'd0;
            jal_tgt_q <= 32'd0;
            func4_q <= 4'd0;
            brf_q <= 1'b0;
            jalrf_q <= 1'b0;
            pred_q <= 1'b0;
            exc_jal_mis_q <= 1'b0;
            ecall_q <= 1'b0;
            ebreak_q <= 1'b0;
            illegal_q <= 1'b0;
            irq_ret_q <= 1'b0;
            idx_i <= 3'd0;
        end
        else if (flush_w) begin
            r1_q <= 32'd0;
            r2_q <= 32'd0;
            aux_q <= 32'd0;
            beq_q <= 32'd0;
            jalr_pred_q <= 32'd0;
            jal_tgt_q <= 32'd0;
            func4_q <= 4'd0;
            brf_q <= 1'b0;
            jalrf_q <= 1'b0;
            pred_q <= 1'b0;
            exc_jal_mis_q <= 1'b0;
            ecall_q <= 1'b0;
            ebreak_q <= 1'b0;
            illegal_q <= 1'b0;
            irq_ret_q <= 1'b0;
            idx_i <= 3'd0;
        end
        else if (adv) begin
            r1_q <= r1_data_in;
            r2_q <= r2_data_in;
            aux_q <= aux_addr_in;
            beq_q <= beq_off_in;
            jalr_pred_q <= jalr_pred_addr_in;
            jal_tgt_q <= jal_target_in;
            func4_q <= alu_func4_in;
            brf_q <= br_flag_in;
            jalrf_q <= jalr_flag_in;
            pred_q <= br_pred_taken_in;
            exc_jal_mis_q <= exc_jal_misalign_in;
            ecall_q <= exc_ecall_in;
            ebreak_q <= exc_ebreak_in;
            illegal_q <= exc_illegal_in;
            irq_ret_q <= exc_irq_ret_in;
            idx_i <= idx_in;
        end
    end

//判定输入级那一条自己的异常 / PC（与 flush_bju_pre 同拍、描述同一条）。
//组合给出即可（源是寄存器）：controller 在"更老那条的寄存版冲刷"那一拍拿到它们，
//用现成的 ~flush_older 就把影子槽盖住了。
    always @(*) begin
        exc_ecall_i   = ecall_q;
        exc_ebreak_i  = ebreak_q;
        exc_illegal_i = illegal_q;
        exc_irq_ret_i = irq_ret_q;
        exc_pc_i      = aux_q;
    end

//F2 用：与"操作数被采样的那一拍"对齐的采样（判定级读当拍的 flush_older 会把自己掐掉）
    always @(posedge clk) begin
        if (rst_q) older_q <= 1'b0;
        else       older_q <= flag_bus[10] | flag_bus[9];
    end

//判定组合逻辑用的寄存器
    reg success_now, br_fail_now, br2_now, br3_now;
    reg exc_br_misalign;
    reg exc_jalr_misalign;
    reg [1:0] jalr_low2;
    reg jalr_fail_now;
    reg [31:0] jalr_target_now, jp_target_now;

//===============================================================
// 判定组合（本级生效）
//===============================================================
//跳转判定（组合）：br 的两个操作数与 jalr 的基址/偏移都由 decoder 的寄存器给出
//（r1_data_in/r2_data_in 就是 alu 的输入），故比较与加法落在这一拍，结果在下一拍生效。
//原设计把这两件事串在【decoder 进本级那一拍】的 D 端上，而那条 D 端的入口是 forw 的输出
//（经 csr 读口 → alu 直通），是全片最差路径；搬到这里后入口变成触发器，链头整段消失。
    always @(*) begin
        success_now = 1'b0;
        br_fail_now = 1'b0;
        jalr_fail_now = 1'b0;
        jalr_target_now = 32'd0;
        if (brf_q) begin
            case (func4_q[2:0])
                3'b000: begin
                    if (r1_q == r2_q)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b001: begin
                    if (r1_q != r2_q)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b100: begin
                    if ($signed(r1_q) < $signed(r2_q))
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b101: begin
                    if ($signed(r1_q) >= $signed(r2_q))
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b110: begin
                    if (r1_q < r2_q)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b111: begin
                    if (r1_q >= r2_q)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                default: br_fail_now = 1'b0;
            endcase
        end
        else if (jalrf_q) begin
            jalr_target_now = (r1_q + r2_q) & 32'hFFFFFFFE;
            if (jalr_target_now != 32'd0) begin
                if (jalr_target_now != jalr_pred_q)
                    jalr_fail_now = 1'b1;
            end
        end
    end

//预测表的写索引 = "指令地址 + 8" 的 [8:3]（口径见下面 br_pc_idx 那条注释）。
//Verilog 不允许对表达式做位选，所以先算出来一格。
    reg [31:0] aux_q_p4;
    always @(*) begin
        aux_q_p4 = aux_q + 32'd4;
    end

    always @(*) begin
        jalr_low2 = 2'd0;
        if (jalrf_q) begin
            jalr_low2 = r1_q[1:0] + r2_q[1:0];
        end
    end

//落点与前置冲刷：br2/br3 沿用原定义（成功且预测不跳 / 失败且预测跳）。
//落点按"分支自身地址 + 4"的口径就地算清：beq_off_in 已含 -4、aux_addr_in 已含 +4，
//两者相加正好是 B+immB；br3 的落点就是 aux_addr_in（落空的后继）。
//★ 这个落点寄存器还**兼任"异常目标"的载荷**：命中指令地址非对齐时，规范要求 mtval = 出错的目标地址，
//  而 br 的目标恰好就是这个加法、jalr 的目标恰好就是 jalr_target_now ⇒ 直接复用，不加加法器。
//  非对齐拍 exc 会压掉 jump 重定向（pc.v 的 flush_jump_w = bit6 & ~bit9）⇒ 那一拍它只被 mtval 消费。
//  jal 的目标【不在流水里】（mid_decoder 不为 JAL 置 imm_c）⇒ 由 post_decoder 就地解 immJ 算好后
//  单铺一路载荷送进来（jal_target_in），同样是进这个落点寄存器、同样零额外寄存器。
//flush_bju_pre 是同一判定的组合版本、早一拍：判定结果寄存后只能覆盖 c1..c4 与 wb，而错路指令
//在 c2 上会停留两拍（前一条落前置拍、后一条落寄存拍），那两拍里它已经会去动 FIFO 指针、
//拉总线、推乘法流水，等寄存器清已经收不回来 —— lsu/mulu 因此在入口多挡一拍。
//★ 进本级的操作数不受任何 stall 门控：装载那一拍 stall 位全 0（adv），拿到的就是最终值 ⇒
//  判定、组合前置冲刷、当拍异常标记一律照常出，不再挂 hazard 门。

    always @(*) begin
        exc_br_misalign = success_now & beq_q[1];
        exc_jalr_misalign = jalrf_q & (jalr_low2 != 2'd0);
        exc_bju = exc_br_misalign | exc_jalr_misalign | exc_jal_mis_q;
        br2_now = success_now & ~pred_q;
        br3_now = br_fail_now & pred_q;
        flush_bju_pre = br2_now | br3_now | jalr_fail_now | exc_br_misalign | exc_jalr_misalign;
        if (br2_now | exc_br_misalign)
            jp_target_now = aux_q + beq_q;
        else if (br3_now)
            jp_target_now = aux_q;
        else if (jalr_fail_now | exc_jalr_misalign)
            jp_target_now = jalr_target_now;
        else if (exc_jal_mis_q)
            jp_target_now = jal_tgt_q;
        else
            jp_target_now = 32'd0;
    end

//===============================================================
// 判定延迟拍（下一拍生效）
//===============================================================
//判定延迟拍：判定结果、跳转落点、以及预测表回写要用的索引与限定信号统一寄存一拍。
//冲刷拍必须清（否则同一笔判定会连拉两拍）；停顿拍不清：操作数被冻住时判定值不变，
//与旧设计把 success/br_fail 放在 decoder 主块里自保持的行为一致。
    always @(posedge clk) begin
        if (rst_q) begin
            success <= 1'b0;
            br_fail <= 1'b0;
            jalr_fail <= 1'b0;
            flush_bju_exc <= 1'b0;
            jp_target <= 32'd0;
            jalr_target_q2 <= 32'd0;
            idx_q <= 3'd0;
            br_pc_idx <= 6'd0;
            jalr_flag_q <= 1'b0;
            br_pred_taken_q <= 1'b0;
            idx_q <= 3'd0;
            exc_pc_q <= 32'd0;
        end
        else if (flush_w) begin
            success <= 1'b0;
            br_fail <= 1'b0;
            jalr_fail <= 1'b0;
            flush_bju_exc <= 1'b0;
            jp_target <= 32'd0;
            jalr_target_q2 <= 32'd0;
            br_pc_idx <= 6'd0;
            jalr_flag_q <= 1'b0;
            br_pred_taken_q <= 1'b0;
            exc_pc_q <= 32'd0;
        end
        else begin
            success <= success_now;
            br_fail <= br_fail_now;
            jalr_fail <= jalr_fail_now;
            flush_bju_exc <= exc_br_misalign | exc_jalr_misalign | exc_jal_mis_q;
            jp_target <= jp_target_now;
            exc_pc_q <= aux_q;
            jalr_target_q2 <= jalr_target_now;
//★ 预测表索引 = "指令地址 + 8"：pc 恒定 +8、一次吃两个字 ⇒ 本条指令被【呈现】那一拍的
//  pc_addr 正好是 指令地址 + 8，读口（bra_predict 直接用 pc_addr_in）就是这个口径。写口跟着改，
//  两边才是同一个 key（老口径 aux_q = 指令地址+4，那是 pc 还走 +4/+8 时的对齐方式）。
            br_pc_idx <= aux_q_p4[8:3];
            jalr_flag_q <= jalrf_q & ~exc_jalr_misalign;
            br_pred_taken_q <= pred_q;
            idx_q <= idx_i;
        end
    end
endmodule
