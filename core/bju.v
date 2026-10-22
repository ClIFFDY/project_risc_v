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
//              ① 判定输入寄存级 —— 操作数与控制位、异常源一起寄存（操作数是前送后的终值，
//                 直接吃会让"单元结果→前送→操作数→比较/加法"串成一条全片最差链）；
//              ② 判定级 —— 比较与加法，结果/落点/预测表限定信号在下一拍生效。
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module bju(
    input clk, rst,
    input [7:0] flag_bus,
//判定源：与 alu 的输入同源（发射级选中项的寄存器输出 = 前送后的操作数）
    input [31:0] r1_data_in, r2_data_in,
//本条指令在 bju 里算什么：idu1 一次译好的 4 位功能码（分支档带 funct3 / jal / jalr / 四类异常）。
//★ 不再分开吃 func4 + br_flag + jalr_flag + 四路异常：func4 对 addi/beq/jalr 是同一个值（都是 0000），
//  单靠它分不出"这条是不是分支"，类信息本来就必须另给；这里把"类"和"种类"合成一个码。
//  分支档 = {1'b1, funct3} ⇒ 判定级的比较 case 与原来逐字一致。
    input [3:0]  bju_func4_in,
    input [31:0] pc_plus4_in, br_off_in, jalr_pred_tgt_in, jal_tgt_in,
    input        br_pred_taken_in,
//判定输入级的装载门：原来本模块自己从 flag_bus 拼。lane0 每条出站指令都过本级（判定 + 异常通道），
//lane1 只有分支类过（只做判定）⇒ 装载门 ≈ issue_v
    input        adv_in,
//异常源（ecall/ebreak/illegal/mret/jal 非对齐）也在 bju_func4_in 里，与操作数同沿锁进判定输入级。
//它们天然比载荷晚一拍出，正好与"更老那条的寄存版冲刷"【同拍配对】——
//  本拍出的是 X_m 的异常，本拍 flag_bus[5]/exc_tgt_mis 出的是 X_{m-1} 的冲刷，
//  于是消费端用现成的 ~flush_older 就盖住了影子槽，不再需要组合的前置冲刷。
//本拍冲刷的边界（cont 给的 ROB 号）与队头：两者一起判"本拍正在判的这条还算不算活着"
    input [3:0]  flush_idx_in,
    input [3:0]  head_ptr_in,
    input [3:0]  idx_in,
    input        gen_in,
//判定结果与跳转落点（寄存一拍后有效）
    output reg        success, br_fail, jalr_fail,
    output reg        exc_tgt_mis,
    output reg [31:0] jp_tgt, jalr_tgt_q,
    output reg [8:0]  br_pc_idx,
    output reg        jalr_tgt_vld_q, br_pred_taken_q,
//判定结果的【组合版】（判定级当拍）：任何寄存版都晚一拍，需要"当拍撤销写口登记 / 当拍打标记"
//的消费者只能用这一个
    output reg        exc_bju,
//本拍被判那条指令的 ROB 索引：idx_i 是判定输入级那一条（cont 用它给异常项选号），
//idx_q 是判定打一拍后的版本（冲刷边界必须用【寄存后的】这个号：
//ROB 的冲刷只作废【比它更年轻】的项，比它老、还在途的 load/mul 必须留下）
    output reg [3:0]  idx_i,
    output reg [3:0]  idx_q,
    output reg        judged_q,
    output reg        gen_q,
//F1：被判那条自己的地址（"地址+4"）—— 必须与 jp_tgt **同块同清**地寄存一份。
//  不能用输入级的 pc_plus4_i（它每拍都装，冲刷拍里已经是下一条的地址 ⇒ mepc 会错 4）
    output reg [31:0] exc_pc_q,
    output reg [31:0] exc_pc_i,
//判定【输入级】那一条自己带的异常（与 idu2 的 bju_in_flight 同拍、描述同一条）：ecall/ebreak/illegal/mret。
//★ 必须从【输入寄存级】出、不能从延迟寄存级出：从延迟级出就与"更老那条的寄存版冲刷"错开两拍，
//  消费端的 ~flush_older 盖不住影子槽（那正是当初要组合前置冲刷的原因）。
    output reg        exc_ecall_i, exc_ebreak_i, exc_illegal_i, exc_irq_ret_i
    );

//功能码（与 idu1 的译码逐值对应，两边必须同时改）
//★ 分支档 bit3 = 1、低 3 位就是 funct3；其余各占一个值，互斥。
    localparam BJU_NONE    = 4'd0;
    localparam BJU_JAL     = 4'd1;
    localparam BJU_JAL_MIS = 4'd2;
    localparam BJU_JALR    = 4'd3;
    localparam BJU_ECALL   = 4'd4;
    localparam BJU_EBREAK  = 4'd5;
    localparam BJU_ILLEGAL = 4'd6;
    localparam BJU_MRET    = 4'd7;

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;

//本拍是不是冲刷拍
    reg flush_w;

//本拍正在判的那条【比冲刷源更老】⇒ 它是活的，本拍的判定必须留下。
//  年龄口径与 ROB/idu2 逐字一致：先按 4 位截断做差，再扩到 5 位比较（队头随拍推进）。
//  ★ 年龄【小 = 老】（队头是最老的）⇒ 更老判据是 ag_j < ag_f；冲刷的作废判据反过来是 ag > ag_f。
//★ 判据对【三种冲刷】一律生效：异常冲刷也得看边界年龄。实测 WEIRD seed40：回跳循环里 `bne`
//  （更老，在 lane1 判定）与它影子里的非法指令（更年轻，在 lane0 被标异常）落在【同一拍】⇒
//  flush_con_exc 把更老那条 bne 刚算出的 br3 清成 0 ⇒ 本该怎么都不跳的循环一直跳下去
//  （x30 从 5 数到 −95 还在跳）。异常边界是那条非法指令自己的号，比判定这条年轻 ⇒
//  判据与跳转冲刷逐字同一条，只是别再只看 flag_bus[5]。
    reg        keep_judge;
    reg [3:0]  ag_j, ag_f;

//判定输入寄存级：把"这一拍要判的那条指令"整个寄存一级
    reg [31:0] r1_data_i, r2_data_i, pc_plus4_i, br_off_i, jalr_pred_tgt_i, jal_tgt_i;
    reg [3:0]  bju_func4_i;
    reg        br_pred_i;
    reg        gen_i, adv_d;

//功能码译码（输入级寄存器输出 ⇒ 组合出来直接给判定级用）
    reg        is_br, is_jal, is_jalr, exc_jal_mis;

//判定级组合中间量
    reg        success_now, br_fail_now, br2_now, br3_now;
    reg        exc_br_misalign, exc_jalr_misalign, jalr_fail_now;
    reg [1:0]  jalr_low2;
    reg [31:0] jalr_tgt_now, jp_tgt_now;

//===============================================================
// 控制位译码（本模块最前的逻辑块）
//===============================================================
//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//判定延迟拍只关心"本拍是不是冲刷拍"。
//★ flush_con_exc 已经是精确版（不再含组合的前置冲刷项）⇒ 直接取 [7]。
    always @(*) begin
        flush_w = flag_bus[7] | flag_bus[6] | flag_bus[5];
    end

//===============================================================
// 第 0 级：复位与年龄判据
//===============================================================
    always @(posedge clk) begin
        rst_q <= rst;
    end

    always @(*) begin
        ag_j       = idx_i - head_ptr_in;
        ag_f       = flush_idx_in - head_ptr_in;
        keep_judge = flush_w & ({1'b0, ag_j} < {1'b0, ag_f});
    end

//===============================================================
// 第 1 级：判定输入寄存级
//===============================================================
//为什么要有这一级：操作数若直接吃前送后的终值，就会把"前送年龄比较/mux → 操作数 →
//比较/加法 → jp_tgt/mtval"串成一条长链（逻辑短、几乎全是线）。这里落一次寄存器，链劈成两段。
//★ 控制位必须跟着一起寄存：操作数晚一拍、控制位不晚，就会拿"下一条的 br_flag"去判"这一条"。
//★ 装载受出站脉冲门控：不装的那拍源还没就绪 —— 进本级的操作数不受任何 stall 门控，
//  判定/发表/前置冲刷照常出。
//★ 冲刷拍必须清：否则错路那条会在下一拍被"补判"一次，变成冲刷后多拉一拍重定向。
    always @(posedge clk) begin
        if (rst_q) begin
            r1_data_i <= 32'd0;
            r2_data_i <= 32'd0;
            pc_plus4_i <= 32'd0;
            br_off_i <= 32'd0;
            jalr_pred_tgt_i <= 32'd0;
            jal_tgt_i <= 32'd0;
            bju_func4_i <= 4'd0;
            br_pred_i <= 1'b0;
            idx_i <= 4'd0;
            gen_i <= 1'b0;
        end
        else if (flush_w) begin
            r1_data_i <= 32'd0;
            r2_data_i <= 32'd0;
            pc_plus4_i <= 32'd0;
            br_off_i <= 32'd0;
            jalr_pred_tgt_i <= 32'd0;
            jal_tgt_i <= 32'd0;
            bju_func4_i <= 4'd0;
            br_pred_i <= 1'b0;
            idx_i <= 4'd0;
            gen_i <= 1'b0;
        end
        else if (adv_in) begin
            r1_data_i <= r1_data_in;
            r2_data_i <= r2_data_in;
            pc_plus4_i <= pc_plus4_in;
            br_off_i <= br_off_in;
            jalr_pred_tgt_i <= jalr_pred_tgt_in;
            jal_tgt_i <= jal_tgt_in;
            bju_func4_i <= bju_func4_in;
            br_pred_i <= br_pred_taken_in;
            idx_i <= idx_in;
            gen_i <= gen_in;
        end
    end

//判定输入级那一条自己的种类 / PC（与 idu2 的 bju_in_flight 同拍、描述同一条）。
//组合给出即可（源是寄存器）：cont 在"更老那条的寄存版冲刷"那一拍拿到它们，
//用现成的 ~flush_older 就把影子槽盖住了。种类也进这一级（与操作数/控制位同生共死）——
//这是"过 bju"的核心：它们比载荷晚一拍出，正好与【更老那条】的寄存版冲刷同拍。
    always @(*) begin
        is_br         = bju_func4_i[3];
        is_jal        = (bju_func4_i == BJU_JAL) | (bju_func4_i == BJU_JAL_MIS);
        is_jalr       = (bju_func4_i == BJU_JALR);
        exc_jal_mis   = (bju_func4_i == BJU_JAL_MIS);
        exc_ecall_i   = (bju_func4_i == BJU_ECALL);
        exc_ebreak_i  = (bju_func4_i == BJU_EBREAK);
        exc_illegal_i = (bju_func4_i == BJU_ILLEGAL);
        exc_irq_ret_i = (bju_func4_i == BJU_MRET);
        exc_pc_i      = pc_plus4_i;
    end

//===============================================================
// 第 2 级：判定组合（本级生效）
//===============================================================
//跳转判定（组合）：br/jalr 的操作数取自判定输入寄存级的 r1_data_i/r2_data_i、控制位取自 bju_func4_i，
//故比较与加法落在这一拍，结果在下一拍生效。
    always @(*) begin
        success_now = 1'b0;
        br_fail_now = 1'b0;
        jalr_fail_now = 1'b0;
        jalr_tgt_now = 32'd0;
        if (is_br) begin
            case (bju_func4_i[2:0])
                3'b000: begin
                    if (r1_data_i == r2_data_i)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b001: begin
                    if (r1_data_i != r2_data_i)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b100: begin
                    if ($signed(r1_data_i) < $signed(r2_data_i))
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b101: begin
                    if ($signed(r1_data_i) >= $signed(r2_data_i))
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b110: begin
                    if (r1_data_i < r2_data_i)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                3'b111: begin
                    if (r1_data_i >= r2_data_i)
                        success_now = 1'b1;
                    else
                        br_fail_now = 1'b1;
                end
                default: br_fail_now = 1'b0;
            endcase
        end
        else if (is_jalr) begin
            jalr_tgt_now = (r1_data_i + r2_data_i) & 32'hFFFFFFFE;
            if (jalr_tgt_now != 32'd0) begin
                if (jalr_tgt_now != jalr_pred_tgt_i)
                    jalr_fail_now = 1'b1;
            end
        end
    end

    always @(*) begin
        jalr_low2 = 2'd0;
        if (is_jalr) begin
            jalr_low2 = (r1_data_i[1:0] + r2_data_i[1:0]) & 2'd2;
        end
    end

//落点与前置冲刷：br2/br3 沿用原定义（成功且预测不跳 / 失败且预测跳）。
//落点按"分支自身地址 + 4"的口径就地算清：br_off_i 的偏移已含 −4、pc_plus4_i 已含 +4，
//两者相加正好是 B+immB；br3 的落点就是 pc_plus4_i（落空的后继）。
//★ 这个落点寄存器还**兼任"异常目标"的载荷**：命中指令地址非对齐时，规范要求 mtval = 出错的目标地址，
//  而 br 的目标恰好就是这个加法、jalr 的目标恰好就是 jalr_tgt_now ⇒ 直接复用，不加加法器。
//  非对齐拍 exc 会压掉 jump 重定向（pc.v 的 flush_jump_w = flag_bus[5] & ~flag_bus[7]）⇒ 那一拍它只被 mtval 消费。
//  jal 的目标【不在流水里】（载荷只带分支偏移）⇒ 由上游发射单元就地解 immJ 算好后
//  单铺一路载荷送进来（jal_tgt_in），同样是进这个落点寄存器、同样零额外寄存器。
//早一拍的那道遮挡不由本模块给：判定结果寄存后只覆盖到写回一级，而错路指令在冲刷生效前
//还会停留两拍（前一条落前置拍、后一条落寄存拍），那两拍里它已经会去动 FIFO 指针、
//拉总线、推乘法流水，等寄存器清已经收不回来 —— lsu 因此在入口多挡一拍，挡位由 idu2 的 bju_in_flight 给。
//★ 进本级的操作数不受任何 stall 门控：装载那一拍 stall 位全 0（adv）⇒ 拿到的就是最终值 ⇒
//  判定、当拍异常标记一律照常出，不再挂 hazard 门。
    always @(*) begin
        exc_br_misalign = success_now & br_off_i[1];
        exc_jalr_misalign = is_jalr & (jalr_low2 != 2'd0);
        exc_bju = exc_br_misalign | exc_jalr_misalign | exc_jal_mis;
        br2_now = success_now & ~br_pred_i;
        br3_now = br_fail_now & br_pred_i;
        if (br2_now | exc_br_misalign)
            jp_tgt_now = pc_plus4_i + br_off_i;
        else if (br3_now)
            jp_tgt_now = pc_plus4_i;
        else if (jalr_fail_now | exc_jalr_misalign)
            jp_tgt_now = jalr_tgt_now;
        else if (exc_jal_mis)
            jp_tgt_now = jal_tgt_i;
        else
            jp_tgt_now = 32'd0;
    end

    always @(posedge clk) begin
        if (rst_q)
            adv_d <= 1'b0;
        else if (flush_w)
            adv_d <= 1'b0;
        else
            adv_d <= adv_in;
    end

//===============================================================
// 第 3 级：判定延迟拍（判定结果寄存一拍，下一拍生效）
//===============================================================
//判定结果、跳转落点、以及预测表回写要用的索引与限定信号统一寄存一拍。
//冲刷拍清【判定结果】与落点（否则同一笔判定会连拉两拍），但【判定回报】（idx_q/gen_q/
//judged_q）照 adv_d 出：中断/异常的冲刷边界比判定在途那条更年轻，那条活着却再也等不到
//回报 ⇒ ROB 里 ent_wr=1/ent_jmp=1/ent_jdg=0 的项卡在队头 ⇒ 等 rob_empty 的重定向自锁。
//回报是 1 拍脉冲，被杀集作废的项由 ROB 的 ent_v/ent_gen 过滤，不会误伤。
//停顿拍不清：操作数被冻住时判定值不变。
//★ 冲刷来自【更年轻】那条时（keep_judge），本拍判的是更老的活指令：判定与落点照常寄存。
//  清掉它就是吞掉更老分支的改向。更年轻的照旧清 —— 否则错路那条会在冲刷后被"补判"一次，
//  多拉一拍重定向。
//★ 判定回报必须是【一拍脉冲】（门 = adv_d：它正是"这一拍判的是 adv 那一拍送进来的那条"）。
//  判定输入级只在 adv 那一拍换内容 ⇒ 本 lane 之后不再发射时，它会把同一条一直挂着，
//  判定值每拍重算、每拍重寄存。后果两条，都实测过：
//   ① 预测表每拍被同一笔结果重训一次 ⇒ bht 一路打到边界（方向预测被锁死）；
//   ② cont 的年龄仲裁里这条【陈旧事件】一直赢过另一条 lane 的新事件 ⇒ 更老那条的改向被
//      永久饿死（实测 seed331：lane1 的 `blt` 判"不跳"挂着，把 lane0 的 `bge x16,x16`
//      （必跳）整条挤掉，PC 一路顺下去）。
    always @(posedge clk) begin
        if (rst_q) begin
            success <= 1'b0;
            judged_q <= 1'b0;
            gen_q <= 1'b0;
            br_fail <= 1'b0;
            jalr_fail <= 1'b0;
            exc_tgt_mis <= 1'b0;
            jp_tgt <= 32'd0;
            jalr_tgt_q <= 32'd0;
            idx_q <= 4'd0;
            br_pc_idx <= 9'd0;
            jalr_tgt_vld_q <= 1'b0;
            br_pred_taken_q <= 1'b0;
            exc_pc_q <= 32'd0;
        end
        else if (flush_w) begin
            if (keep_judge & adv_d) begin
                success <= success_now;
                br_fail <= br_fail_now;
                jalr_fail <= jalr_fail_now;
                exc_tgt_mis <= exc_br_misalign | exc_jalr_misalign | exc_jal_mis;
                jp_tgt <= jp_tgt_now;
                exc_pc_q <= pc_plus4_i;
                jalr_tgt_q <= jalr_tgt_now;
                br_pc_idx <= pc_plus4_i[11:3];
                jalr_tgt_vld_q <= is_jalr & ~exc_jalr_misalign;
                br_pred_taken_q <= br_pred_i;
            end
            else begin
                success <= 1'b0;
                br_fail <= 1'b0;
                jalr_fail <= 1'b0;
                exc_tgt_mis <= 1'b0;
                jp_tgt <= 32'd0;
                jalr_tgt_q <= 32'd0;
                br_pc_idx <= 9'd0;
                jalr_tgt_vld_q <= 1'b0;
                br_pred_taken_q <= 1'b0;
                exc_pc_q <= 32'd0;
            end
            idx_q <= idx_i;
            gen_q <= gen_i;
            judged_q <= adv_d;
        end
        else begin
            success <= success_now & adv_d;
            br_fail <= br_fail_now & adv_d;
            jalr_fail <= jalr_fail_now & adv_d;
            exc_tgt_mis <= (exc_br_misalign | exc_jalr_misalign | exc_jal_mis) & adv_d;
            jp_tgt <= adv_d ? jp_tgt_now : 32'd0;
            jalr_tgt_q <= adv_d ? jalr_tgt_now : 32'd0;
            br_pc_idx <= adv_d ? pc_plus4_i[11:3] : 9'd0;
            jalr_tgt_vld_q <= adv_d ? (is_jalr & ~exc_jalr_misalign) : 1'b0;
            br_pred_taken_q <= adv_d ? br_pred_i : 1'b0;
            exc_pc_q <= pc_plus4_i;
            idx_q <= idx_i;
            gen_q <= gen_i;
            judged_q <= adv_d;
        end
    end

endmodule
