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
// Description: 分支/跳转判定单元（branch-jump unit）。
//              判定源全部来自 decoder 本级的寄存器输出（与 alu 的输入同源），
//              比较与加法落在这一拍，结果、落点、以及预测表回写要用的限定信号
//              在下一拍一并生效；另给出一条组合版的前置冲刷 flush_bju_pre，只有 lsu/mulu 消费。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module bju(
    input clk, rst,
    input [13:0] flag_bus,
//判定源：与 alu 的输入同源（decoder 本级的寄存器输出）
    input [31:0] r1_data_in, r2_data_in,
    input [3:0] alu_func4_in,
    input br_flag_in, jalr_flag_in,
    input [31:0] aux_addr_in, beq_off_in, jalr_pred_addr_in,
    input br_pred_taken_in,
    input exc_jal_misalign_in,
    input [31:0] jal_target_in,
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
//判定结果的【组合版本】（本拍就有效）：ROB 的陷阱标记必须用它 —— 寄存版晚一拍，
//而"不写 rd"的指令出厂即 done，晚一拍标记时它可能已经退掉了，标记就丢了（实测 exc_br_misalign）。
//落点同理给一份组合版，当 mtval 用（寄存器版要给 pc 重定向，不能混）。
    output reg exc_bju,
    output reg [31:0] jp_target_c,
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
//            stall_lsu_haz, stall_lsu_unload, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_dcache_miss, stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：判定延迟拍只关心"本拍是不是冲刷拍"。
    reg flush_w;
    always @(*) flush_w = flag_bus[13] | flag_bus[12] | flag_bus[11];

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
        if (br_flag_in) begin
            case (alu_func4_in[2:0])
                3'b000: begin
                    if (r1_data_in == r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b001: begin
                    if (r1_data_in != r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b100: begin
                    if ($signed(r1_data_in) < $signed(r2_data_in)) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b101: begin
                    if ($signed(r1_data_in) >= $signed(r2_data_in)) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b110: begin
                    if (r1_data_in < r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                3'b111: begin
                    if (r1_data_in >= r2_data_in) success_now = 1'b1;
                    else br_fail_now = 1'b1;
                end
                default: br_fail_now = 1'b0;
            endcase
        end
        else if (jalr_flag_in) begin
            jalr_target_now = (r1_data_in + r2_data_in) & 32'hFFFFFFFE;
            if (jalr_target_now != 32'd0) begin
                if (jalr_target_now != jalr_pred_addr_in) jalr_fail_now = 1'b1;
            end
        end
    end

    always @(*) begin
        jalr_low2 = 2'd0;
        if (jalr_flag_in) begin
            jalr_low2 = r1_data_in[1:0] + r2_data_in[1:0];
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
//★ 操作数在途（lsu 的 load-use / 载荷未应答、mulu 的 mul-use / 除法）那几拍，decoder 输出那一拍上的
//  操作数还是【旧值】⇒ 这一拍的判定、组合前置冲刷、当拍异常标记都不许出。
//  lsu/mulu 的冒险只冻住前端，管不住 bju 的寄存判定与 pc 的冲刷路（pc.v 里 flush_w 优先于 stall_w）。
//  实测 CoreMark：`lw a5,0(a0)` 紧邻 `jr a5`，bju 用旧 a5 算出 jalr_fail ⇒ pc 按错落点跳进 dtcm
//  数据区 ⇒ 反复非对齐 trap ⇒ 死循环（load-use 冒险本身是检出并压住的，问题在判定没被它管住）。
    wire op_haz = flag_bus[7] | flag_bus[6] | flag_bus[5] | flag_bus[4] | flag_bus[3];

    always @(*) begin
        exc_br_misalign = success_now & beq_off_in[1];
        exc_jalr_misalign = jalr_flag_in & (jalr_low2 != 2'd0);
        exc_bju = (exc_br_misalign | exc_jalr_misalign | exc_jal_misalign_in) & ~op_haz;
        br2_now = success_now & ~br_pred_taken_in;
        br3_now = br_fail_now & br_pred_taken_in;
        flush_bju_pre = (br2_now | br3_now | jalr_fail_now | exc_br_misalign | exc_jalr_misalign) & ~op_haz;
        if (br2_now | exc_br_misalign) jp_target_now = aux_addr_in + beq_off_in;
        else if (br3_now) jp_target_now = aux_addr_in;
        else if (jalr_fail_now | exc_jalr_misalign) jp_target_now = jalr_target_now;
        else if (exc_jal_misalign_in) jp_target_now = jal_target_in;
        else jp_target_now = 32'd0;
        jp_target_c = jp_target_now;
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
        end
//操作数在途那几拍【不出判定】（清了等它落下再判；否则下一拍就会带着错落点去重定向）
        else if (op_haz) begin
            success <= 1'b0;
            br_fail <= 1'b0;
            jalr_fail <= 1'b0;
            flush_bju_exc <= 1'b0;
            jp_target <= 32'd0;
        end
        else begin
            success <= success_now;
            br_fail <= br_fail_now;
            jalr_fail <= jalr_fail_now;
            flush_bju_exc <= exc_br_misalign | exc_jalr_misalign | exc_jal_misalign_in;
            jp_target <= jp_target_now;
            jalr_target_q2 <= jalr_target_now;
            br_pc_idx <= aux_addr_in[8:3];
            jalr_flag_q <= jalr_flag_in & ~exc_jalr_misalign;
            br_pred_taken_q <= br_pred_taken_in;
            idx_q <= idx_in;
        end
    end
endmodule
