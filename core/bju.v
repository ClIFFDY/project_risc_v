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
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input [2:0] flush_control_in,   // = flag_bus[7:5]：{异常, 中断, 改向}
//判定源：与 alu 的输入同源（发射级选中项的寄存器输出 = 前送后的操作数）
    input [31:0] source_a_data_in, source_b_data_in,
//本条指令在 bju 里算什么：idu1 一次译好的 4 位功能码（分支档带 funct3 / jal / jalr / 四类异常）。
//★ 不再分开吃 func4 + br_flag + jalr_flag + 四路异常：func4 对 addi/beq/jalr 是同一个值（都是 0000），
//  单靠它分不出"这条是不是分支"，类信息本来就必须另给；这里把"类"和"种类"合成一个码。
//  分支档 = {1'b1, funct3} ⇒ 判定级的比较 case 与原来逐字一致。
    input [3:0]  branch_unit_function4_in,
    input [31:0] pc_plus4_in, br_off_in, jalr_pred_tgt_in, jal_tgt_in,
    input        branch_prediction_taken_in,
//判定输入级的装载门：原来本模块自己从 flag_bus 拼。lane0 每条出站指令都过本级（判定 + 异常通道），
//lane1 只有分支类过（只做判定）⇒ 装载门 ≈ issue_v
    input        advance_in,
//异常源（ecall/ebreak/illegal/mret/jal 非对齐）也在 branch_unit_function4_in 里，与操作数同沿锁进判定输入级。
//它们天然比载荷晚一拍出，正好与"更老那条的寄存版冲刷"【同拍配对】——
//  本拍出的是 X_m 的异常，本拍 flush_control_in[0]/exception_target_misalign 出的是 X_{m-1} 的冲刷，
//  于是消费端用现成的 ~flush_older 就盖住了影子槽，不再需要组合的前置冲刷。
//本拍冲刷的边界（cont 给的 ROB 号）与队头：两者一起判"本拍正在判的这条还算不算活着"
    input [3:0]  flush_index_in,
    input [3:0]  index_in,
    input        generation_in,
//判定结果与跳转落点（寄存一拍后有效）
    output reg        success, branch_failure, jump_link_register_failure,
    output reg        exception_target_misalign,
    output reg [31:0] jump_target, jump_link_register_target_registered,
    output reg [8:0]  branch_pc_index,
//分支自身的完整 PC（branch_pc_index 只有 9 位，jalr 回写落点表要全地址）
    (* max_fanout = 32 *) output reg [31:0] branch_pc_registered,
    output reg        jump_link_register_target_valid_registered, branch_prediction_taken_registered,
//判定结果的【组合版】（判定级当拍）：任何寄存版都晚一拍，需要"当拍撤销写口登记 / 当拍打标记"
//的消费者只能用这一个
    output reg        exception_branch_unit,
//本拍被判那条指令的 ROB 索引：judged_index 是判定输入级那一条（cont 用它给异常项选号），
//index_registered 是判定打一拍后的版本（冲刷边界必须用【寄存后的】这个号：
//ROB 的冲刷只作废【比它更年轻】的项，比它老、还在途的 load/mul 必须留下）
    output reg [3:0]  judged_index,
    output reg [3:0]  index_registered,
    output reg        judged_registered,
//★ S3：这条判定要不要改向（分支判错 / jalr 落点算错）。与 judged_registered 同沿寄存、随判定写进 ROB 项；
//  真正的改向要等它排到队头才发生。
    output reg        redirect_registered,
    output reg        generation_registered,
//★ 预测表的【唯一训练口】。前端不再自己译码（那会在 iffu 里成环：iffu 的译码反过来改它自己
//  当拍的输入），br/jal/jalr 的种类与落点一律以本模块的判定为准 —— 它是唯一同时知道
//  「这条是什么指令」和「真实下一条 pc 是谁」的地方。
//  四根与 success/branch_failure/branch_pc_registered 同沿寄存，描述同一条指令：
//    branch_unit_kind  = 0 不训（非跳转 / 被冲刷 / 已吞掉）/ 1 条件分支 / 2 jal / 3 jalr
//    branch_unit_taken = 这条判定是 taken（条件分支成立 / jal / jalr 走到）
//    branch_unit_pc    = 这条指令【自己】的地址（不是 pc+4）
//    branch_unit_target   = 它真正要去的下一条 pc（条件分支没跳时是顺延地址，消费端只在 taken 那档用）
    output reg [1:0]  branch_unit_kind,
    output reg        branch_unit_taken,
    output reg [31:0] branch_unit_pc, branch_unit_target,
//F1：被判那条自己的地址（"地址+4"）—— 必须与 jump_target **同块同清**地寄存一份。
//  不能用输入级的 judged_pc_plus4（它每拍都装，冲刷拍里已经是下一条的地址 ⇒ mepc 会错 4）
    output reg [31:0] exception_pc_registered,
    output reg [31:0] exception_pc_judged,
//判定【输入级】那一条自己带的异常（与 idu2 的 bju_in_flight 同拍、描述同一条）：ecall/ebreak/illegal/mret。
//★ 必须从【输入寄存级】出、不能从延迟寄存级出：从延迟级出就与"更老那条的寄存版冲刷"错开两拍，
//  消费端的 ~flush_older 盖不住影子槽（那正是当初要组合前置冲刷的原因）。
    output reg        exception_ecall_judged, exception_ebreak_judged, exception_illegal_judged, exception_interrupt_return_judged,
//★ 输入级异常位【延迟到判定延迟级】：与 exception_target_misalign / index_registered / exception_pc_registered 同拍、同一条指令。
//  这样 ROB 的异常标记口可以做成"每 lane 一条"，cont 里不再需要按年龄二选一。
    output reg        exception_ecall_registered, exception_ebreak_registered, exception_illegal_registered, exception_interrupt_return_registered
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
    reg reset_registered;

//本拍是不是冲刷拍
    reg flush_window;

//本拍正在判的那条【比冲刷源更老】⇒ 它是活的，本拍的判定必须留下。
//  年龄口径与 ROB/idu2 逐字一致：先按 4 位截断做差，再扩到 5 位比较（队头随拍推进）。
//  ★ 年龄【小 = 老】（队头是最老的）⇒ 更老判据是 ag_j < ag_f；冲刷的作废判据反过来是 ag > ag_f。
//★ 判据对【三种冲刷】一律生效：异常冲刷也得看边界年龄。实测 WEIRD seed40：回跳循环里 `bne`
//  （更老，在 lane1 判定）与它影子里的非法指令（更年轻，在 lane0 被标异常）落在【同一拍】⇒
//  flush_con_exc 把更老那条 bne 刚算出的 br3 清成 0 ⇒ 本该怎么都不跳的循环一直跳下去
//  （x30 从 5 数到 −95 还在跳）。异常边界是那条非法指令自己的号，比判定这条年轻 ⇒
//  判据与跳转冲刷逐字同一条，只是别再只看 flush_control_in[0]。

//判定输入寄存级：把"这一拍要判的那条指令"整个寄存一级
    reg [31:0] judged_source_a_data, judged_source_b_data, judged_pc_plus4, judged_branch_offset, judged_jump_link_register_predicted_target, judged_jump_link_target;
    reg [3:0]  judged_function4;
    reg        judged_branch_prediction;
    reg        judged_generation, advance_delayed;

//功能码译码（输入级寄存器输出 ⇒ 组合出来直接给判定级用）
    reg        is_branch, is_jump_link, is_jump_link_register, exception_jump_link_misalign;

//判定级组合中间量
    reg        success_now, branch_failure_now, branch_stage2_now, branch_stage3_now;
    reg        branch_target_failure_now;
    reg        exception_branch_misalign, exception_jump_link_register_misalign, jump_link_register_failure_now, jump_link_no_prediction_now;
    reg [1:0]  jump_link_register_low2;
    reg [31:0] jump_link_register_target_now, jump_target_now;
//预测表回写索引（分支【自己的】地址的 [11:3]，见下方专用 always 块的说明）
    reg [8:0]  branch_pc_own;

//===============================================================
// 控制位译码（本模块最前的逻辑块）
//===============================================================
//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//判定延迟拍只关心"本拍是不是冲刷拍"。
//★ flush_con_exc 已经是精确版（不再含组合的前置冲刷项）⇒ 直接取 [7]。
    always @(*) begin
        flush_window = flush_control_in[2] | flush_control_in[1] | flush_control_in[0];
    end

//===============================================================
// 第 0 级：复位与年龄判据
//===============================================================
    always @(posedge clk) begin
        reset_registered <= rst;
    end

//===============================================================
// 第 1 级：判定输入寄存级
//===============================================================
//为什么要有这一级：操作数若直接吃前送后的终值，就会把"前送年龄比较/mux → 操作数 →
//比较/加法 → jump_target/mtval"串成一条长链（逻辑短、几乎全是线）。这里落一次寄存器，链劈成两段。
//★ 控制位必须跟着一起寄存：操作数晚一拍、控制位不晚，就会拿"下一条的 br_flag"去判"这一条"。
//★ 装载受出站脉冲门控：不装的那拍源还没就绪 —— 进本级的操作数不受任何 stall 门控，
//  判定/发表/前置冲刷照常出。
//★ 冲刷拍必须清：否则错路那条会在下一拍被"补判"一次，变成冲刷后多拉一拍重定向。
    always @(posedge clk) begin
        if (reset_registered) begin
            judged_source_a_data <= 32'd0;
            judged_source_b_data <= 32'd0;
            judged_pc_plus4 <= 32'd0;
            judged_branch_offset <= 32'd0;
            judged_jump_link_register_predicted_target <= 32'd0;
            judged_jump_link_target <= 32'd0;
            judged_function4 <= 4'd0;
            judged_branch_prediction <= 1'b0;
            judged_index <= 4'd0;
            judged_generation <= 1'b0;
        end
        else if (flush_window) begin
            judged_source_a_data <= 32'd0;
            judged_source_b_data <= 32'd0;
            judged_pc_plus4 <= 32'd0;
            judged_branch_offset <= 32'd0;
            judged_jump_link_register_predicted_target <= 32'd0;
            judged_jump_link_target <= 32'd0;
            judged_function4 <= 4'd0;
            judged_branch_prediction <= 1'b0;
            judged_index <= 4'd0;
            judged_generation <= 1'b0;
        end
        else if (advance_in) begin
            judged_source_a_data <= source_a_data_in;
            judged_source_b_data <= source_b_data_in;
            judged_pc_plus4 <= pc_plus4_in;
            judged_branch_offset <= br_off_in;
            judged_jump_link_register_predicted_target <= jalr_pred_tgt_in;
            judged_jump_link_target <= jal_tgt_in;
            judged_function4 <= branch_unit_function4_in;
            judged_branch_prediction <= branch_prediction_taken_in;
            judged_index <= index_in;
            judged_generation <= generation_in;
        end
    end

//判定输入级那一条自己的种类 / PC（与 idu2 的 bju_in_flight 同拍、描述同一条）。
//组合给出即可（源是寄存器）：cont 在"更老那条的寄存版冲刷"那一拍拿到它们，
//用现成的 ~flush_older 就把影子槽盖住了。种类也进这一级（与操作数/控制位同生共死）——
//这是"过 bju"的核心：它们比载荷晚一拍出，正好与【更老那条】的寄存版冲刷同拍。
    always @(*) begin
        is_branch         = judged_function4[3];
        is_jump_link        = (judged_function4 == BJU_JAL) | (judged_function4 == BJU_JAL_MIS);
        is_jump_link_register       = (judged_function4 == BJU_JALR);
        exception_jump_link_misalign   = (judged_function4 == BJU_JAL_MIS);
        exception_ecall_judged   = (judged_function4 == BJU_ECALL);
        exception_ebreak_judged  = (judged_function4 == BJU_EBREAK);
        exception_illegal_judged = (judged_function4 == BJU_ILLEGAL);
        exception_interrupt_return_judged = (judged_function4 == BJU_MRET);
        exception_pc_judged      = judged_pc_plus4;
    end

//===============================================================
// 第 2 级：判定组合（本级生效）
//===============================================================
//跳转判定（组合）：br/jalr 的操作数取自判定输入寄存级的 judged_source_a_data/judged_source_b_data、控制位取自 judged_function4，
//故比较与加法落在这一拍，结果在下一拍生效。
    always @(*) begin
        success_now = 1'b0;
        branch_failure_now = 1'b0;
        jump_link_register_failure_now = 1'b0;
        jump_link_register_target_now = 32'd0;
        if (is_branch) begin
            case (judged_function4[2:0])
                3'b000: begin
                    if (judged_source_a_data == judged_source_b_data)
                        success_now = 1'b1;
                    else
                        branch_failure_now = 1'b1;
                end
                3'b001: begin
                    if (judged_source_a_data != judged_source_b_data)
                        success_now = 1'b1;
                    else
                        branch_failure_now = 1'b1;
                end
                3'b100: begin
                    if ($signed(judged_source_a_data) < $signed(judged_source_b_data))
                        success_now = 1'b1;
                    else
                        branch_failure_now = 1'b1;
                end
                3'b101: begin
                    if ($signed(judged_source_a_data) >= $signed(judged_source_b_data))
                        success_now = 1'b1;
                    else
                        branch_failure_now = 1'b1;
                end
                3'b110: begin
                    if (judged_source_a_data < judged_source_b_data)
                        success_now = 1'b1;
                    else
                        branch_failure_now = 1'b1;
                end
                3'b111: begin
                    if (judged_source_a_data >= judged_source_b_data)
                        success_now = 1'b1;
                    else
                        branch_failure_now = 1'b1;
                end
                default: branch_failure_now = 1'b0;
            endcase
        end
        else if (is_jump_link_register) begin
            jump_link_register_target_now = (judged_source_a_data + judged_source_b_data) & 32'hFFFFFFFE;
            if (jump_link_register_target_now != 32'd0) begin
//★ 除了比落点，还要问【前端到底跳没跳】：judged_branch_prediction=0 ⇒ 前端顺下去了 ⇒ 无论表里残留的落点
//  碰巧对不对，都必须改向。少了这一项，`ret` 会静默落在下一个函数上（实测 CoreMark 0x80）。
                if (!judged_branch_prediction || (jump_link_register_target_now != judged_jump_link_register_predicted_target))
                    jump_link_register_failure_now = 1'b1;
            end
        end
    end

    always @(*) begin
        jump_link_register_low2 = 2'd0;
        if (is_jump_link_register) begin
            jump_link_register_low2 = (judged_source_a_data[1:0] + judged_source_b_data[1:0]) & 2'd2;
        end
    end

//落点与前置冲刷：br2/br3 沿用原定义（成功且预测不跳 / 失败且预测跳）。
//落点按"分支自身地址 + 4"的口径就地算清：judged_branch_offset 的偏移已含 −4、judged_pc_plus4 已含 +4，
//两者相加正好是 B+immB；br3 的落点就是 judged_pc_plus4（落空的后继）。
//★ 这个落点寄存器还**兼任"异常目标"的载荷**：命中指令地址非对齐时，规范要求 mtval = 出错的目标地址，
//  而 br 的目标恰好就是这个加法、jalr 的目标恰好就是 jump_link_register_target_now ⇒ 直接复用，不加加法器。
//  非对齐拍 exc 会压掉 jump 重定向（pc.v 的 flush_jump_w = flush_control_in[0] & ~flush_control_in[2]）⇒ 那一拍它只被 mtval 消费。
//  jal 的目标【不在流水里】（载荷只带分支偏移）⇒ 由上游发射单元就地解 immJ 算好后
//  单铺一路载荷送进来（jal_tgt_in），同样是进这个落点寄存器、同样零额外寄存器。
//早一拍的那道遮挡不由本模块给：判定结果寄存后只覆盖到写回一级，而错路指令在冲刷生效前
//还会停留两拍（前一条落前置拍、后一条落寄存拍），那两拍里它已经会去动 FIFO 指针、
//拉总线、推乘法流水，等寄存器清已经收不回来 —— lsu 因此在入口多挡一拍，挡位由 idu2 的 bju_in_flight 给。
//★ 进本级的操作数不受任何 stall 门控：装载那一拍 stall 位全 0（adv）⇒ 拿到的就是最终值 ⇒
//  判定、当拍异常标记一律照常出，不再挂 hazard 门。
    always @(*) begin
        exception_branch_misalign = success_now & judged_branch_offset[1];
        exception_jump_link_register_misalign = is_jump_link_register & (jump_link_register_low2 != 2'd0);
        exception_branch_unit = exception_branch_misalign | exception_jump_link_register_misalign | exception_jump_link_misalign;
        branch_stage2_now = success_now & ~judged_branch_prediction;
        branch_stage3_now = branch_failure_now & judged_branch_prediction;
//★ 落点表 tag 截断（10 位）会带来假命中：分支的假落点不像 jalr 那样本来就有人比。
//  少了这一项，假命中 ⇒ pc 一路跑错路、bju 却认为预测对了 ⇒ 永不改向（静默错，不卡死）。
        branch_target_failure_now = success_now & judged_branch_prediction &
                          ((judged_pc_plus4 + judged_branch_offset) != judged_jump_link_register_predicted_target);
//★ jal 也要判"预测跳了没有"：改向搬到取指落点表之后，pc 只在【表命中】时按 jal 的落点跳，
//  而表要等这条 jal 第一次被交付才写进去 ⇒ 第一次执行必然顺下去。老 pc 是"译码即跳"，
//  所以这里从来不需要这一项；现在不补，冷启动的 jal 会静默走错路（实测 seed21：0x350 的
//  `jal x20,+160` 不跳，整条后序全错）。命中之后 judged_branch_prediction=1，这一项自然不再触发。
        jump_link_no_prediction_now = is_jump_link & ~judged_branch_prediction;
        if (branch_stage2_now | branch_target_failure_now | exception_branch_misalign)
            jump_target_now = judged_pc_plus4 + judged_branch_offset;
        else if (branch_stage3_now)
            jump_target_now = judged_pc_plus4;
        else if (jump_link_register_failure_now | exception_jump_link_register_misalign)
            jump_target_now = jump_link_register_target_now;
        else if (exception_jump_link_misalign | jump_link_no_prediction_now)
            jump_target_now = judged_jump_link_target;
        else
            jump_target_now = 32'd0;
    end

//-------------------------------------------------------------------------------
//训练载荷：本模块是 br/jal/jalr 的【唯一】译码点与判定点，预测表只认这一份。
//  前端 iffu 不再译码 —— 它在"被自己改内容的那个交付口"上译码就是组合环；而且前端译码
//  给出的落点只是【预测】，本模块给的才是【事实】。表里存的应该是事实。
//  条件分支没跳时 branch_unit_target 给的是顺延地址；消费端（落点表 / 落点指令表）只在 taken 那档用它，
//  所以不会拿顺延地址覆盖掉真正的静态落点。
//  非对齐目标（会走异常）一律报 kind=0，不让任何表学到它，否则 pc 会被拉到非对齐地址。
    reg [1:0]  branch_unit_kind_now;
    reg        branch_unit_taken_now;
    reg [31:0] branch_unit_target_now;
    always @(*) begin
        if (is_branch) begin
            branch_unit_kind_now  = exception_branch_misalign ? 2'd0 : 2'd1;
            branch_unit_taken_now = success_now;
            branch_unit_target_now   = success_now ? (judged_pc_plus4 + judged_branch_offset) : judged_pc_plus4;
        end
        else if (is_jump_link) begin
            branch_unit_kind_now  = exception_jump_link_misalign ? 2'd0 : 2'd2;
            branch_unit_taken_now = 1'b1;
            branch_unit_target_now   = judged_jump_link_target;
        end
        else if (is_jump_link_register) begin
            branch_unit_kind_now  = exception_jump_link_register_misalign ? 2'd0 : 2'd3;
            branch_unit_taken_now = 1'b1;
            branch_unit_target_now   = jump_link_register_target_now;
        end
        else begin
            branch_unit_kind_now  = 2'd0;
            branch_unit_taken_now = 1'b0;
            branch_unit_target_now   = judged_pc_plus4;
        end
    end

//★ 预测表回写索引必须落在【分支自己的地址】上，而载荷 judged_pc_plus4 是"指令地址+4"口径
//  （cpu_top 的 push_addr = cur_addr + 4）⇒ 先减 4 再切位。
//  读口（bpu）吃的是交付口上的 cur_addr，也就是指令自己的地址。两边口径不齐的后果：
//  地址 4 mod 8 的分支（B[2]=1）加 4 会向 bit3 进位 ⇒ 训练落到【下一格】，
//  自己那格永远停在复位值 1（弱不跳）⇒ 这类分支只要真跳就必然误预测，每执行一次错一次。
//  实测 CoreMark：14,666 次冲刷里 12,600 次（86%）落在 B[2]=1 这一极性上，
//  只有 125 条不同的分支在错、重复率 99.1% ⇒ 不是容量、不是冷启动，就是这一格的口径。
//  （bpu.v 开头那段注释写过同一条教训 —— BTIC 按它修了，BHT/BTB 这条路漏了。）
    always @(*) begin
        branch_pc_own = (judged_pc_plus4 - 32'd4) >> 3;
    end

    always @(posedge clk) begin
        if (reset_registered)
            advance_delayed <= 1'b0;
        else if (flush_window)
            advance_delayed <= 1'b0;
        else
            advance_delayed <= advance_in;
    end

//===============================================================
// 第 3 级：判定延迟拍（判定结果寄存一拍，下一拍生效）
//===============================================================
//判定结果、跳转落点、以及预测表回写要用的索引与限定信号统一寄存一拍。
//冲刷拍清【判定结果】与落点（否则同一笔判定会连拉两拍），但【判定回报】（index_registered/generation_registered/
//judged_registered）照 advance_delayed 出：中断/异常的冲刷边界比判定在途那条更年轻，那条活着却再也等不到
//回报 ⇒ ROB 里 ent_wr=1/ent_jmp=1/ent_jdg=0 的项卡在队头 ⇒ 等 rob_empty 的重定向自锁。
//回报是 1 拍脉冲，被杀集作废的项由 ROB 的 ent_v/ent_gen 过滤，不会误伤。
//停顿拍不清：操作数被冻住时判定值不变。
    always @(posedge clk) begin
        if (reset_registered) begin
            success <= 1'b0;
            judged_registered <= 1'b0;
            generation_registered <= 1'b0;
            branch_failure <= 1'b0;
            jump_link_register_failure <= 1'b0;
            exception_target_misalign <= 1'b0;
            jump_target <= 32'd0;
            jump_link_register_target_registered <= 32'd0;
            index_registered <= 4'd0;
            branch_pc_index <= 9'd0;
            branch_pc_registered   <= 32'd0;
            jump_link_register_target_valid_registered <= 1'b0;
            branch_prediction_taken_registered <= 1'b0;
            exception_pc_registered <= 32'd0;
            redirect_registered  <= 1'b0;
            branch_unit_kind  <= 2'd0;
            branch_unit_taken <= 1'b0;
            branch_unit_pc    <= 32'd0;
            branch_unit_target   <= 32'd0;
            exception_ecall_registered <= 1'b0; exception_ebreak_registered <= 1'b0;
            exception_illegal_registered <= 1'b0; exception_interrupt_return_registered <= 1'b0;
        end
        else begin
            success <= success_now & advance_delayed;
            branch_failure <= branch_failure_now & advance_delayed;
            jump_link_register_failure <= jump_link_register_failure_now & advance_delayed;
            exception_target_misalign <= (exception_branch_misalign | exception_jump_link_register_misalign | exception_jump_link_misalign) & advance_delayed;
            jump_target <= advance_delayed ? jump_target_now : 32'd0;
            jump_link_register_target_registered <= advance_delayed ? jump_link_register_target_now : 32'd0;
            branch_pc_index <= advance_delayed ? branch_pc_own : 9'd0;
            branch_pc_registered   <= advance_delayed ? (judged_pc_plus4 - 32'd4) : 32'd0;
            jump_link_register_target_valid_registered <= advance_delayed ? (is_jump_link_register & ~exception_jump_link_register_misalign) : 1'b0;
            branch_prediction_taken_registered <= advance_delayed ? judged_branch_prediction : 1'b0;
            exception_pc_registered <= judged_pc_plus4;
            index_registered <= judged_index;
            generation_registered <= judged_generation;
            judged_registered <= advance_delayed;
//★ jump_link_no_prediction_now 必须在册：它是"这条 jal 没有被取指落点表预测到"的唯一凭证。
//  改向前 pc 是"译码即跳"，jal 从不需要改向；现在冷启动的 jal 靠这一位把 pc 拉回来。
            redirect_registered <= advance_delayed & (branch_stage2_now | branch_stage3_now | jump_link_register_failure_now | jump_link_no_prediction_now);
//训练载荷与 branch_pc_registered 同沿：写口三件套（pc / 是否训 / 落点）必须描述【同一条】指令。
            branch_unit_kind  <= advance_delayed ? branch_unit_kind_now  : 2'd0;
            branch_unit_taken <= advance_delayed ? branch_unit_taken_now : 1'b0;
            branch_unit_pc    <= advance_delayed ? (judged_pc_plus4 - 32'd4) : 32'd0;
            branch_unit_target   <= advance_delayed ? branch_unit_target_now   : 32'd0;
//输入级异常位延迟一级：index_registered / exception_pc_registered 是无条件跟随输入级的，所以事件命中那一拍
//它们正好还是同一条指令的号与地址。
            exception_ecall_registered   <= exception_ecall_judged   & advance_delayed;
            exception_ebreak_registered  <= exception_ebreak_judged  & advance_delayed;
            exception_illegal_registered <= exception_illegal_judged & advance_delayed;
            exception_interrupt_return_registered <= exception_interrupt_return_judged & advance_delayed;
        end
    end

endmodule
