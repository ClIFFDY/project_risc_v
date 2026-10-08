`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/27 15:26:36
// Design Name:
// Module Name: decoder
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


module post_decoder(
    input clk, rst,
    input [11:0] flag_bus,
//★ 进来的那条真的在册（取指队列队头有效）。adv 与 h0_v 是【未加保护的不变式】：
//  实测抓到过 adv=1 而 h0_v=0（cycle 69619，载荷锁进一条 aux_addr=0 的残留字、ROB 还给它分了个号）
//  —— 幽灵项会推进取指/落点机器。这里把它变成硬门：进来的不有效，载荷就不推进、不发号、不发射。
    input        fifo_h0_v,
    input [9:0] func10,
    input [4:0] rd_in,
    input [4:0] rs1_in, rs2_in,
    input [31:0] imm_alu_in,
    input [31:0] inst_in,
    input [31:0] offset_beq0_aux, pc_operand_in,
//lane1 的 pc 操作数（= 它自己的"地址+4"，pre_decoder 的 aux_addr_1_out）：AUIPC 当 lane1 用
    input [31:0] pc_operand_1_in,
    input [31:0] aux_addr_in,
    input br_pred_taken_in,
    input [31:0] jalr_pred_addr_in,
//非寄存器操作数（立即数 / pc / uimm）的载荷与资格位：装配已并进 forw 的那一次选择，
//本模块只负责把它们锁好、交出去（哪一路生效由 forw 按 imm_sel / 命中位定）。
    output reg [31:0] r1_imm_val, r2_imm_val,
    output reg        r1_imm_sel, r2_imm_sel,
    output reg [4:0] rd_out,
//发射级（E4）写口广播：这一拍要不要写 rd、写的是哪个 rd（给 controller 发写序号用）
    output reg issue_we,
    output reg [4:0] issue_rd,
//ROB 索引载荷（与其它载荷【同使能】锁存）：ROB 每分配一条指令就发一个索引，
//  它随指令走到自己的完成点，回来时用它把结果写进 ROB 自己那一项。
    input [2:0] idx_in,
//载荷的 ROB 索引：当拍要喂 7 个消费者（`forw.idx_post`、`alu`/`lsu`/`mulu`/`bju` 的 `idx_in`、
//`rob.exc_idx`、`controller.issue_idx_in`），而它是关键路径的第一跳。
    output reg [2:0] issue_idx,
//与 idx_in 同生共死的世代位：载荷沿锁存、随持有期一起保持，交执行单元带回来做完成上报的身份校验
    input        alloc_gen_in,
    output reg   issue_gen,
//前送槽号（← rob 的扫描口）：本条指令的两个操作数该由【哪个 ROB 槽】供值。
//★ 它是【载荷】：只在同一个 `payload_go` 沿锁存（与 `issue_idx` 同生共死）。
//  冻结期间**绝不能重算** —— rob 的退项是沿生效、扫描是组合读 `ent_v`，重算会让扫描
//  漂到更老的匹配项 ⇒ 前送陈旧值。这是本方案的头号红线。
//★ 冲刷分支要【显式清 0】：让下游 mux 落回载荷默认值，免得冲刷拍选到垃圾。
    output reg [2:0] sel_slot1, sel_slot2,
    output reg       sel_v1,    sel_v2,
    input  [2:0] fwd_slot1_in, fwd_slot2_in,
    input        fwd_hit1_in,  fwd_hit2_in,
//lane1（一个包的第二条）：只可能是 OP / OP-IMM / LUI（判据在 pre_decoder），
//所以它不需要 opc/fn10_ls/off_mem/csr/分支/异常这一整套，只要一组操作数与一个 alu_func4。
    input [31:0] aux_addr_1_in,
//上压一格：通道已建好，触发【先钉 0】—— 开了会坏 CoreMark，见计划文件
    input        lane1_lsu_blk, lane1_mulu_blk,
//每条 lane 各自"这一拍换不换内容"（上压通道已删：被按住的那条原地保持，另一条照走）
    output adv0, adv1,
//lane1 压着一条更老的没发出去（中断受理点必须让开它，见 csr.exc_irq_gate）
    output lane1_hold,
    input        lane1_v_in,
    input [31:0] inst1_in,
    input [4:0]  rd1_in,
    input [9:0]  func10_1_in,
    input [31:0] imm1_alu_in,
    input [4:0]  rs1_1_in, rs2_1_in,
    input [2:0]  idx1_in,
    input        alloc_gen1_in,
    input [2:0]  fwd_slot1_1_in, fwd_slot2_1_in,
    input        fwd_hit1_1_in,  fwd_hit2_1_in,
//★ 三个执行单元（alu/lsu/mulu）的输入【级数对齐】：都吃这一级（decoder 载荷）。
//  lsu/mulu 原来吃的是 mem_buf 的组合输出（比 alu 早一级），索引也得跟着差一拍；
//  搬到这里之后三者同相：索引一律 issue_idx，操作数一律由 forw（decoder→单元之间）给出。
    output reg [6:0]  opc_out,
    output reg [9:0]  fn10_out,
//给 lsu 的那一份：{7'd0, funct3}，任何 inst_in[6:0] 下都有效（func10 只在 OP/OP_IMM 类有意义）
    output reg [9:0]  fn10_ls_out,
    output reg [31:0] off_mem_out,
    output reg [4:0]  rs1_out, rs2_out,
//本拍载荷推进（不分写不写）—— ROB 的分配使能：每条指令都要占位，
//  否则陷阱不知道自己在程序序里的位置
    output payload_go,
//上压那拍【只分配一条】：新进来的那条照旧占 tail（rob 的号序就按 tail 排），lane1 那口关掉
    output alloc_en1out,
//regfile 读口地址：值寄存一拍、下一拍随消费者进载荷 ⇒ 这里必须给【上压之后】那两条的 rs
    output [4:0] rf_r1, rf_r2, rf_r3, rf_r4,
//寄存一拍的"载荷推进"脉冲：消费端（lsu/mulu）用它做【每条指令只收一次】的入口门 ——
//  载荷因 stall/hold_issue/flush 停住时它是 0，车厢一空也不会把同一条指令再收一遍。
//  寄存版是关键：直接用 payload_go 会经 stall_w 与消费端自己的 stall 成环（实测死锁）。
    output reg payload_go_q,
    output reg [3:0] alu_func4,
    output reg [2:0] csr_func3,
    output reg exc_irq_ret, exc_ecall, exc_ebreak, jal_flag, jalr_flag,
    output we,
    output reg csr_wr_en,
    output reg [11:0] csr_addr,
    output reg [11:0] csr_addr_pre,
    output reg [31:0] csr_data,
//交给 bju 的判定源：判定源与 alu 同源（前送后的操作数 / alu_func4），
//其余是本级寄存下来的跳转载荷与限定位，都由判定单元在下一拍使用
    output reg [31:0] aux_addr_out,
    output reg [31:0] beq_off_q2, jalr_pred_addr_out,
    output reg br_flag, br_pred_taken_out,
    output reg exc_jal_misalign_out,
    output reg [31:0] jal_target_out,
    output reg exc_illegal_out,
//lane1 的发射载荷：与 lane0 那组逐项对应，但只有 alu2 需要的那些
    output reg        lane1_v_out,
    output reg [31:0] r1_1_imm_val, r2_1_imm_val,
    output reg        r1_1_imm_sel, r2_1_imm_sel,
    output reg [4:0]  rd1_out, rs1B_out, rs2B_out,
    output reg        issue_we1,
    output reg [4:0]  issue_rd1,
    output reg [2:0]  issue_idx1,
    output reg        issue_gen1,
    output reg [2:0]  sel_slot1_1, sel_slot2_1,
    output reg        sel_v1_1,    sel_v2_1,
    output reg [3:0]  alu_func4_1,
//上压一格用：lane1 那一侧也带一套 lane0 才用得上的控制字段
    output reg [6:0]  opc_1_q,
    output reg [9:0]  fn10_1_q,
    output reg [9:0]  fn10_ls_1_q,
    output reg [31:0] off_mem_1_q,
    output reg [31:0] aux_addr_1_q,
    output            we1
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//RV32I和Zicsr扩展的opcode集
    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_MISC_MEM = 7'b0001111;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_SYSTEM = 7'b1110011;

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//flag_bus 逐位翻译成原名：本模块内不起新的组合名，判定处直接写或运算。
//冲刷位与停顿位可同时为 1，故判定处一律保持【冲刷优先于停顿】。
    reg flush_con_exc, flush_con_irq, flush_con_jump, exec;
    reg stall_rob_full, stall_pc_redir;
    reg stall_lsu_haz, stall_lsu_full;
    reg stall_mulu_haz, stall_mulu_div;
    reg stall_icache_miss, stall_bus_hold;
    reg exc_illegal_now;
    always @(*) begin
        flush_con_exc     = flag_bus[11];
        flush_con_irq     = flag_bus[10];
        flush_con_jump    = flag_bus[9];
        exec              = flag_bus[8];
        stall_rob_full    = flag_bus[7];
        stall_pc_redir    = flag_bus[6];
        stall_lsu_haz     = flag_bus[5];
        stall_lsu_full    = flag_bus[4];
        stall_mulu_haz    = flag_bus[3];
        stall_mulu_div    = flag_bus[2];
        stall_icache_miss = flag_bus[1];
        stall_bus_hold    = flag_bus[0];
    end

//发射级（E4）写口广播：这条指令会不会写 rd、写哪个（给 controller 发写序号用）。
//判据取保守侧（多报最多多停几拍）：只排除【肯定不写寄存器堆】的 STORE/BRANCH/MISC_MEM。
//SYSTEM 里的 ecall/ebreak/mret 其实不写，这里按会写处理（保守）。
    always @(*) begin
        issue_we = 1'b0;
        issue_rd = rd_in;
        if (exec && ~(flush_con_exc | flush_con_irq | flush_con_jump) && (rd_in != 5'd0)) begin
            case (inst_in[6:0])
                OPCODE_OP, OPCODE_OP_IMM, OPCODE_LOAD, OPCODE_JAL,
                OPCODE_JALR, OPCODE_LUI, OPCODE_AUIPC, OPCODE_SYSTEM: issue_we = 1'b1;
            endcase
        end
    end

//lane1 的写口广播：它的三种 opcode（OP/OP-IMM/LUI）都会写 rd ⇒ 判据就是 lane1_v 本身。
//rd1_in 为 0 那一路与 lane0 同处理（ROB 的 ent_rd 会取 0，不写寄存器堆）。
    always @(*) begin
        issue_we1 = 1'b0;
        issue_rd1 = rd1_in;
        if (exec && ~(flush_con_exc | flush_con_irq | flush_con_jump) && lane1_v_in && (rd1_in != 5'd0))
            issue_we1 = 1'b1;
    end

//csr 读口地址：就是本级【组合输入】里那条指令的 csr 号（即 csr_addr 的下一条流水位置），
//非 SYSTEM 清 0 —— 与 csr_addr 的语义完全一致，读口不必再自己挡非法地址。
//csr 的读值要提前一拍（见 csr.v 的读口注释），所以这条地址必须取组合版、不能取 csr_addr。
    always @(*) begin
        csr_addr_pre = 12'd0;
        if (inst_in[6:0] == OPCODE_SYSTEM)
            csr_addr_pre = inst_in[31:20];
    end

//jal 的目标：本核的立即数 flavour 只在 pre_decoder 解（mid_decoder 不为 JAL 置 imm_c）
//⇒ 目标【不在流水里】。这里就地从同拍的指令字解 immJ、再配一个加法器，比把载荷从 E1
//搬三级省 64 个触发器（pc_operand_in 就是本条指令的地址 +4，与 aux_addr 同一口径）。
//★ 载荷要送到 E5 才交付：E1..E3 是最年轻的几级，从那里驱动全局冲刷会把【更老】的指令清掉。
//  所以这里寄存一拍（E4），再由 bju 寄进落点寄存器（E5，与 br/jalr 完全同相）。
    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction
    reg [31:0] jal_target_now;
    always @(*) jal_target_now = pc_operand_in + immJ(inst_in) - 32'd4;

//CSR 合法性（规范 §Zicsr）：
//  ① 访问【不存在】的 CSR（读或写）⇒ 非法指令
//  ② 访问【只读】CSR（地址 bit[11:10] == 11）且**真的要写**它 ⇒ 非法指令
//  ③ "真的要写" = rs1/uimm 字段（inst[19:15]）非 0 —— csrrw/csrrwi 的该字段为 0 时
//     【不写】（这是 RISC-V 的著名陷阱：`csrw mscratch, x0` 是空操作）；csrrs/csrrc 的
//     该字段为 0 时写的是"老值"，本来就中性，但同样不该算一次写。
//  存在性清单必须与 csr.v 的读 case 一致（那边多一条 0xB00/0xB02 只写丢弃）。
    reg csr_is_access_now, csr_exists_now, csr_ro_now, csr_wr_now;
    always @(*) begin
        csr_is_access_now = (inst_in[14:12] != 3'b000) && (inst_in[14:12] != 3'b100);
        csr_wr_now        = (inst_in[19:15] != 5'd0);
        csr_ro_now        = (inst_in[31:30] == 2'b11);
        csr_exists_now    = 1'b0;
        case (inst_in[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_now = 1'b1;
            default: csr_exists_now = 1'b0;
        endcase
    end

    always @(*) begin
        exc_illegal_now = 1'b0;
        if (inst_in != 32'd0) begin
            case (inst_in[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: begin
                    exc_illegal_now = 1'b0;
                end
//LOAD 合法宽度：LB/LH/LW/LBU/LHU = 000/001/010/100/101 ⇒ 011/110/111 非法
                OPCODE_LOAD: begin
                    if (inst_in[14:12] == 3'b011)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b110)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b111)
                        exc_illegal_now = 1'b1;
                end
//STORE 合法宽度：SB/SH/SW = 000/001/010 ⇒ 011/100/101/110/111 非法
                OPCODE_STORE: begin
                    if (inst_in[14:12] == 3'b011)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b100)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b101)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b110)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b111)
                        exc_illegal_now = 1'b1;
                end
                OPCODE_JALR: begin
                    if (inst_in[14:12] != 3'b000)
                        exc_illegal_now = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (inst_in[14:12] == 3'b010)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b011)
                        exc_illegal_now = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (inst_in[14:12] == 3'b001) begin
                        if (inst_in[31:25] != 7'd0)
                            exc_illegal_now = 1'b1;
                    end
                    if (inst_in[14:12] == 3'b101) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000))
                            exc_illegal_now = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (inst_in[31:25] == 7'b0000001) begin
                        exc_illegal_now = 1'b0;
                    end
                    else if (inst_in[14:12] == 3'b000) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000))
                            exc_illegal_now = 1'b1;
                    end
                    else if (inst_in[14:12] == 3'b001) begin
                        if (inst_in[31:25] != 7'd0)
                            exc_illegal_now = 1'b1;
                    end
                    else if (inst_in[14:12] == 3'b101) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000))
                            exc_illegal_now = 1'b1;
                    end
                    else begin
                        if (inst_in[31:25] != 7'd0)
                            exc_illegal_now = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (inst_in[14:12] == 3'b010)
                        exc_illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b011)
                        exc_illegal_now = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (inst_in[14:12] == 3'b100)
                        exc_illegal_now = 1'b1;
//sret（0x102）：只对 S 模式有意义，本核是 M-only ⇒ 按非法指令（规范允许"实现为非法"）。
//★ wfi（0x105）保持【合法】并当 NOP：规范要求 WFI 是一条合法指令（可以什么都不做），
//  只有 mstatus.TW=1 且在低权限模式执行时才非法。
                    if (inst_in[14:12] == 3'b000) begin
                        if (inst_in[31:20] == 12'h102)
                            exc_illegal_now = 1'b1;
                    end
                    if (csr_is_access_now) begin
                        if (!csr_exists_now)
                            exc_illegal_now = 1'b1;
                        if (csr_ro_now) begin
                            if (csr_wr_now)
                                exc_illegal_now = 1'b1;
                        end
                    end
                end
                default: begin
                    exc_illegal_now = 1'b1;
                end
            endcase
        end
    end

//发号使能：与下面载荷寄存器的推进条件同形（exec 且不冲刷不暂停），再与"本拍有写"相与
//★ 独立成线：只看 lane1 那条的操作数到了没，不与 payload_go（这一拍能不能发）相与。
//  复用 payload_go 会让上压在正需要它的那一拍关着（lsu 满 / 后端正忙时 payload_go=0）。
//  上压的意义就是：lane1 那条没就绪 ⇒ 把它按在槽里，不让它带着没到的操作数往下走。
//★ 必须带【本级 lane1 真的有条指令】这个限定：ls_use_hit_1 比的是寄存器里的 rs，
//  槽空时那个残留值照样能命中在途 load ⇒ 上压会在空槽上开火（实测 shadow：连抬 60 拍）。
//lane0 的换新门就是全局推进门；lane1 额外要求【它自己那条】的操作数到了（在途 load）。
//★ 再没有第二条路：挡住和补位由同一个门决定 —— 上一版把它们拆成两件事，缝里那条指令就丢了。
//★★ 2026-10-07：lane0 也吃 lane1_hold —— 两条必须【整包】推进，不能只按住一条：
//  只按 lane1 时，pre_decoder 的槽1 原地保持、槽0 照收新队头 ⇒ 槽对会变成"槽1 更老"的反序对，
//  而两槽同拍进载荷就是【同拍双发射】—— 反序对之间的 RAW 是 lane1→lane0 方向，
//  本机的旁路只有"lane0 上一拍 → lane1 这一拍"和单元结果口，【没有】lane1→lane0 的同拍通路
//  ⇒ 消费者直接取到陈旧值（实测 CoreMark 的 or a5,a4,a5 取到上一轮的 a5 ⇒ 三项 CRC 全错）。
//  整包推进后，槽对恒按程序序（槽0 更老），pre_decoder 的 raw_pair 判据方向也就恒成立。
    assign adv0 = payload_go & ~lane1_hold;
//★ 必须用【本级 lane1 真有条指令】限定：ls_use_hit_1 比的是寄存器里的 rs，槽空时那个残留值
//  照样能命中 ⇒ 不加限定 lane1 永远填不上（历史实测 shadow：连抬 60 拍）。
//lane1 那条被自己没到的操作数按住：在途 load（lsu）或在途乘法（mulu）任一条命中就算。
//★ 必须用 lane1_v_out 限定：两个 hazard 比的都是寄存器里的 rs，槽空时残留值照样命中。
//★ 本拍只挡 lane1 自己（不再或进 flag_bus 冻整核）—— 这就是 lane1_mulu_blk 这根线原来该接的地方。
//★ 这里【只能】放 lsu 那一项。mulu 的 lane1 判据（lane1_mulu_blk = stall_mulu_haz_1）实测
//  会**永久保持** ⇒ lane1 永远发不出去 ⇒ 它的 ROB 项永不完成 ⇒ 核挂死（二分实测：加上它
//  13k 拍就死、去掉它 25k 拍还活）。这是 mulu 那条判据本身的形状问题，不是接线问题，
//  单独记一条待办，不在这儿硬接。
    assign lane1_hold = lane1_v_out & lane1_lsu_blk;
//lane0 的写使能出口也跟进来那条有效走：队头无效那拍 payload_go=0（载荷不推进），
//而 flag_bus 里没有任何停顿位 => ALU 会按 we=1 再发一遍那条被按住的指令 => 伪写。
    reg we_q;
    assign we = we_q & fifo_h0_v;
    assign adv1 = payload_go & ~lane1_hold;

//★ lane1 的发出门：它自己的操作数没到（在途 load / 乘法结果）就不发它。
//  这就是在 post_decoder 与 lsu 之间那一拍发的 stall，只是只挡 lane1、不冻整核。
//  上压负责把这条没发出去的按在槽里，两条一起才不会丢指令。
    reg we1_q;
//lane1 的写使能跟着它自己的换新门走（不换新那一拍不发，也不刷寄存器的值）
    assign we1 = we1_q & adv1;

//lane1 这一拍要不要占号：有候选、且它没被自己的操作数挡住。
//★ 口径里【绝不含 payload_go/full】，否则 take->need->full 成环。
    assign alloc_en1out = lane1_v_in & ~lane1_hold;

//上压那拍：lane0 由自己的 lane1 顶上来、lane1 收队头（= 本拍 lane0 输入那条）
//★ lane0 的读口地址也要按 adv0 选：读值寄存一拍才交给载荷 ⇒ 地址要提前一拍给
//  【下一拍坐在 lane0 载荷里的那条】。加了 S 级（pre_decoder 的槽）之后，
//  "这一拍进 S 的 rs"（rs1_in = S 的输入 = F 的 rs）与"载荷下一拍那条的 rs"不再同一个：
//  adv0=1 时载荷下一拍就是 F 那条 ⇒ 取 rs1_in；adv0=0 时载荷原地保持 ⇒ 取载荷自己那份
//  （rs1_out，就是载荷里那条的 rs）—— 不用新增任何寄存器。lane1 那两行同款。
    assign rf_r1 = payload_go ? rs1_in : rs1_out;
    assign rf_r2 = payload_go ? rs2_in : rs2_out;
//★ lane1 的读口地址必须与【下一拍坐在 lane1 载荷里的那条】对齐，而不能与槽1 对齐：
//  读值在这里寄存一拍才交给 lane1 载荷 ⇒ 地址要提前一拍给【下一条载荷】的 rs。
//  槽1 与载荷 lane1 同门（adv1），但槽1 比载荷早一级 ⇒ adv1=0（lane1 被 load-use 顶住）那拍：
//  载荷 lane1 原地保持（还是原来那条），槽1 也保持 —— 而槽1 的内容是【再下一条】，
//  此时若照旧用槽1 的 rs，被顶住的那条就会读到自己.rs 之外的值。
//  实测（prog cyc≈104314，CoreMark matrix_test 的 `sub a3,a3,s4`）：lane1 被 lhu 的 load-use
//  顶住 3 拍，期间槽1 已换成下一条 `bltu`，`sub` 于是读到 s0(x8) 而不是 s4(x20)，
//  算出 0x8a 而应为 0x82 —— 之后每隔一笔 x13 就多 8。genreg 首个分歧 #48996。
//  正解：adv1=1 取槽1（下一条载荷就是它）；adv1=0 取【载荷 lane1 自己那份 rs】
//  （rs1B_out/rs2B_out 就是 adv1=0 时载荷里那条的 rs）—— 不用新增任何寄存器。
    assign rf_r3 = adv1 ? rs1_1_in : rs1B_out;
    assign rf_r4 = adv1 ? rs2_1_in : rs2B_out;

    assign payload_go = exec & ~(flush_con_exc | flush_con_irq | flush_con_jump)
                      & ~(stall_rob_full | stall_pc_redir | stall_lsu_haz
                        | stall_lsu_full | stall_mulu_haz | stall_mulu_div
 | stall_icache_miss | stall_bus_hold)
                      & fifo_h0_v;

//二次译码的中间量：拆出来才能在上压那一拍把 lane0 的译码结果发给 lane1
    reg [6:0]    opc_out_d;
    reg [9:0]    fn10_out_d;
    reg [9:0]    fn10_ls_out_d;
    reg [31:0]   off_mem_out_d;
    reg [4:0]    rs1_out_d;
    reg [4:0]    rs2_out_d;
    reg [4:0]    rd_out_d;
    reg [3:0]    alu_func4_d;
    reg [2:0]    csr_func3_d;
    reg we_d;
    reg csr_wr_en_d;
    reg [11:0]   csr_addr_d;
    reg [31:0]   csr_data_d;
    reg br_flag_d;
    reg exc_irq_ret_d;
    reg exc_ecall_d;
    reg exc_ebreak_d;
    reg jal_flag_d;
    reg jalr_flag_d;
    reg [31:0]   beq_off_q2_d;
    reg [31:0]   aux_addr_out_d;
    reg exc_jal_misalign_out_d;
    reg [31:0]   jal_target_out_d;
    reg exc_illegal_out_d;
    reg br_pred_taken_out_d;
    reg [31:0]   jalr_pred_addr_out_d;
    reg [31:0]   r1_imm_val_d;
    reg [31:0]   r2_imm_val_d;
    reg r1_imm_sel_d;
    reg r2_imm_sel_d;

    always @(*) begin
                    opc_out_d = inst_in[6:0];;
                    fn10_out_d = func10;;
                    fn10_ls_out_d = {7'd0, inst_in[14:12]};;
                    off_mem_out_d = imm_alu_in;;
                    rs1_out_d = rs1_in;;
                    rs2_out_d = rs2_in;;
                    rd_out_d = 5'd0;;
                    alu_func4_d = 4'd0;;
                    csr_func3_d = 3'd0;;
                    we_d = 1'b0;;
                    csr_wr_en_d = 1'b0;;
                    csr_addr_d = 12'd0;;
                    csr_data_d = 32'd0;;
                    br_flag_d = 1'b0;;
                    exc_irq_ret_d = 1'b0;;
                    exc_ecall_d = 1'b0;;
                    exc_ebreak_d = 1'b0;;
                    jal_flag_d = 1'b0;;
                    jalr_flag_d = 1'b0;;
                    beq_off_q2_d = 32'd0;;
                    aux_addr_out_d = aux_addr_in;;
                    exc_jal_misalign_out_d = (inst_in[6:0] == OPCODE_JAL) & inst_in[21];;
                    jal_target_out_d = jal_target_now;;
                    exc_illegal_out_d = exc_illegal_now;;
                    br_pred_taken_out_d = br_pred_taken_in;;
                    jalr_pred_addr_out_d = jalr_pred_addr_in;;
//操作数：底值 = 本级的组合输入（删掉 mid_decoder 后它就是 regfile 的寄存读 —— 含读侧 5 源旁路，
//与载荷同拍）；下面按 opcode 把非寄存器操作数覆盖掉。
                    r1_imm_val_d = r1_imm_val;;
                    r2_imm_val_d = r2_imm_val;;
                    r1_imm_sel_d = 1'b0;;
                    r2_imm_sel_d = 1'b0;;
                    case (inst_in[6:0])
                        OPCODE_OP: begin
                            rd_out_d = rd_in;;
//RV32M：写回由 mulu 独立完成，这条路必须让开 —— 否则同一条指令被写两次，
//而且 alu 会按【撞车的 func3】算出垃圾结果。
//M 与普通 ALU 共用 OPCODE_OP，只能靠 funct7 区分（func10[9:3]==7'b0000001）。
                            if (func10[9:3] == 7'b0000001) begin
                                we_d = 1'b0;;
                            end
                            else begin
                                alu_func4_d = {func10[8], func10[2:0]};;
                                we_d = 1'b1;;
                            end
                        end
                        OPCODE_OP_IMM: begin
                            r2_imm_val_d = imm_alu_in;;
                            r2_imm_sel_d = 1'b1;;
                            rd_out_d = rd_in;;
                            alu_func4_d = {func10[8], func10[2:0]};;
                            we_d = 1'b1;;
                        end
//jal存储pc值传递
                        OPCODE_JAL: begin
                            rd_out_d = rd_in;;
                            we_d = 1'b1;;
                            aux_addr_out_d = aux_addr_in;;
                            jal_flag_d = 1'b1;;
                        end
//jalr：偏移与基址各寄存一拍，真目标由判定块在下一拍相加得出
                        OPCODE_JALR: begin
                            r2_imm_val_d = imm_alu_in;;
                            r2_imm_sel_d = 1'b1;;
                            rd_out_d = rd_in;;
                            we_d = 1'b1;;
                            jalr_flag_d = 1'b1;;
                            aux_addr_out_d = aux_addr_in;;
                        end
//分支：两个比较源寄存一拍，比较由判定块在下一拍做
                        OPCODE_BRANCH: begin
                            aux_addr_out_d = aux_addr_in;;
                            beq_off_q2_d = offset_beq0_aux - 4'd4;;
                            alu_func4_d = {1'b0, func10[2:0]};;
                            br_flag_d = 1'b1;;
                        end
                        OPCODE_LUI: begin
                            r2_imm_val_d = imm_alu_in;;
                            r2_imm_sel_d = 1'b1;;
                            rd_out_d = rd_in;;
                            alu_func4_d = 4'd0;;
                            we_d = 1'b1;;
                        end
//AUIPC：r1 = 本条 PC（非寄存器操作数）、r2 = 立即数
                        OPCODE_AUIPC: begin
                            r1_imm_val_d = pc_operand_in;;
                            r1_imm_sel_d = 1'b1;;
                            r2_imm_val_d = imm_alu_in;;
                            r2_imm_sel_d = 1'b1;;
                            rd_out_d = rd_in;;
                            alu_func4_d = 4'd0;;
                            we_d = 1'b1;;
                        end
//LOAD：r1 = 基址（寄存器操作数）；偏移走 off_mem_out（lsu 自己那一份）
                        OPCODE_LOAD: begin
                            rd_out_d = rd_in;;
                        end
//STORE：r1 = 基址、r2 = 存的数据，两个都是寄存器操作数
                        OPCODE_STORE: begin
                        end
//MISC_MEM（fence/fence.i）：不吃寄存器操作数
                        OPCODE_MISC_MEM: begin
                        end
//SYSTEM类指令读写赋能，地址计算
                        OPCODE_SYSTEM: begin
                            rd_out_d = rd_in;;
                            csr_func3_d = func10[2:0];;
                            exc_irq_ret_d = (func10[2:0] == 3'b000) && (inst_in[31:20] == 12'h302);;
//不写时不给出地址（地址 0 不是任何 CSR）：csr.v 的写 case 不命中 ⇒ 天然不写，
//写后读旁路也自然关掉（读要走真寄存器）。★ 不能用"清 csr_wr_en"来表达不写 ——
//csr_wr_en 是【双重身份】：alu 用它判定"这是条 CSR 指令"，据此把读值 cs_data 选进
//result 写 rd（alu.v:57）。清掉它会让 csrr 读回 0（踩过）。
                            if (csr_wr_now) begin
                                csr_addr_d = inst_in[31:20];;
                            end
                            else begin
                                csr_addr_d = 12'd0;;
                            end
                            case (func10[2:0])
                                3'b000: begin
                                    we_d = 1'b0;;
                                    csr_wr_en_d = 1'b0;;
                                    exc_ecall_d = (inst_in[31:20] == 12'h000) || (inst_in[31:20] == 12'h001);;
                                    exc_ebreak_d = (inst_in[31:20] == 12'h001);;
                                end
//r1 的来路：csrrw/csrrs/csrrc 用 rs1（寄存器操作数 ⇒ 走前送）；csrrwi/si/ci 用 uimm。
                                3'b001: begin
                                    we_d = 1'b1;;
                                    csr_wr_en_d = 1'b1;;
                                end
                                3'b010: begin
                                    we_d = 1'b1;;
                                    csr_wr_en_d = 1'b1;;
                                end
                                3'b011: begin
                                    we_d = 1'b1;;
                                    csr_wr_en_d = 1'b1;;
                                end
                                3'b101: begin
                                    r1_imm_val_d = {27'd0, inst_in[19:15]};;
                                    r1_imm_sel_d = 1'b1;;
                                    we_d = 1'b1;;
                                    csr_wr_en_d = 1'b1;;
                                end
                                3'b110: begin
                                    r1_imm_val_d = {27'd0, inst_in[19:15]};;
                                    r1_imm_sel_d = 1'b1;;
                                    we_d = 1'b1;;
                                    csr_wr_en_d = 1'b1;;
                                end
                                3'b111: begin
                                    r1_imm_val_d = {27'd0, inst_in[19:15]};;
                                    r1_imm_sel_d = 1'b1;;
                                    we_d = 1'b1;;
                                    csr_wr_en_d = 1'b1;;
                                end
                                default: begin
                                    we_d = 1'b0;;
                                    csr_wr_en_d = 1'b0;;
                                end
                            endcase
                        end
                    endcase
//★ 故障指令一律不写 rd：乱序写回之后写不再由退口把关（完成即写）⇒ 必须在这里挡掉，
//  否则非法指令 / 非对齐跳转的垃圾结果会先落进寄存器堆（trap 交付时已经来不及收回）。
//  （ecall/ebreak 那条分支本来就置 we=0；访存非对齐由 lsu 拒绝，不发写回。）
                    if (exc_illegal_now | ((inst_in[6:0] == OPCODE_JAL) & inst_in[21]))
                        we_d = 1'b0;;
    end

    always @(posedge clk) begin
        if (rst_q)
            payload_go_q <= 1'b0;
        else
            payload_go_q <= payload_go;
    end

//（操作数装配已并进 forw：本模块只把非寄存器操作数的常量与资格位锁好交出去。
//  ★ 常量必须寄存、不能按当拍 inst_in 组合选：本级载荷装的是"上一拍进本级那条"，
//    而当拍 inst_in 已经是下一条 ⇒ 组合选会晚一条指令。）

    always @(posedge clk) begin
        if (rst_q) begin
            r1_imm_val <= 32'd0;
            r2_imm_val <= 32'd0;
            r1_imm_sel <= 1'b0;
            r2_imm_sel <= 1'b0;
            rd_out <= 5'd0;
            alu_func4 <= 4'd0;
            we_q <= 1'b0;
            csr_wr_en <= 1'b0;
            csr_addr <= 12'd0;
            csr_data <= 32'd0;
            br_flag <= 1'b0;
            exc_irq_ret <= 1'b0;
            exc_ecall <= 1'b0;
            exc_ebreak <= 1'b0;
            jal_flag <= 1'b0;
            jalr_flag <= 1'b0;
            beq_off_q2 <= 32'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
            exc_jal_misalign_out <= 1'b0;
            jal_target_out <= 32'd0;
            exc_illegal_out <= 1'b0;
            issue_idx <= 3'd0;
            issue_gen <= 1'b0;
            sel_slot1 <= 3'd0;
            sel_slot2 <= 3'd0;
            sel_v1 <= 1'b0;
            sel_v2 <= 1'b0;
            lane1_v_out <= 1'b0;
            rs1B_out <= 5'd0;
            rs2B_out <= 5'd0;
            r1_1_imm_val <= 32'd0;
            r2_1_imm_val <= 32'd0;
            r1_1_imm_sel <= 1'b0;
            r2_1_imm_sel <= 1'b0;
            rd1_out <= 5'd0;
            issue_idx1 <= 3'd0;
            issue_gen1 <= 1'b0;
            sel_slot1_1 <= 3'd0;
            sel_slot2_1 <= 3'd0;
            sel_v1_1 <= 1'b0;
            sel_v2_1 <= 1'b0;
            alu_func4_1 <= 4'd0;
            we_q  <= 1'b0;
            we1_q <= 1'b0;
            opc_out <= 7'd0;
            fn10_out <= 10'd0;
            fn10_ls_out <= 10'd0;
            off_mem_out <= 32'd0;
            rs1_out <= 5'd0;
            rs2_out <= 5'd0;
            opc_1_q <= 7'd0;
            fn10_1_q <= 10'd0;
            fn10_ls_1_q <= 10'd0;
            off_mem_1_q <= 32'd0;
            aux_addr_1_q <= 32'd0;
        end
        else if (exec) begin
//在EXE状态下根据不同的opcode对指令进行二次解码
            if (~(flush_con_exc | flush_con_irq | flush_con_jump)
              & ~(stall_rob_full | stall_pc_redir | stall_lsu_haz
                 | stall_lsu_full | stall_mulu_haz | stall_mulu_div
 | stall_icache_miss | stall_bus_hold)
              & fifo_h0_v) begin
                issue_idx <= idx_in;
                issue_gen <= alloc_gen_in;
                sel_slot1 <= fwd_slot1_in;
                sel_slot2 <= fwd_slot2_in;
                sel_v1 <= fwd_hit1_in;
                sel_v2 <= fwd_hit2_in;
                if (adv1) begin
                    lane1_v_out <= lane1_v_in;
                    rs1B_out <= rs1_1_in;
                    rs2B_out <= rs2_1_in;
                    opc_1_q <= inst1_in[6:0];
                    fn10_1_q <= func10_1_in;
                    fn10_ls_1_q <= {7'd0, inst1_in[14:12]};
                    off_mem_1_q <= imm1_alu_in;
                    aux_addr_1_q <= aux_addr_1_in;
                    issue_idx1 <= idx1_in;
                    issue_gen1 <= alloc_gen1_in;
                    sel_slot1_1 <= fwd_slot1_1_in;
                    sel_slot2_1 <= fwd_slot2_1_in;
                    sel_v1_1 <= fwd_hit1_1_in;
                    sel_v2_1 <= fwd_hit2_1_in;
                    rd1_out <= 5'd0;
                    we1_q <= 1'b0;
                    alu_func4_1 <= 4'd0;
                    r1_1_imm_val <= r1_1_imm_val;
                    r2_1_imm_val <= r2_1_imm_val;
                    r1_1_imm_sel <= 1'b0;
                    r2_1_imm_sel <= 1'b0;
                    if (lane1_v_in) begin
                        rd1_out <= rd1_in;
                        we1_q <= 1'b1;
                        case (inst1_in[6:0])
                            OPCODE_OP: begin
                                alu_func4_1 <= {func10_1_in[8], func10_1_in[2:0]};
                            end
                            OPCODE_OP_IMM: begin
                                r2_1_imm_val <= imm1_alu_in;
                                r2_1_imm_sel <= 1'b1;
                                alu_func4_1 <= {func10_1_in[8], func10_1_in[2:0]};
                            end
                            OPCODE_LUI: begin
                                r2_1_imm_val <= imm1_alu_in;
                                r2_1_imm_sel <= 1'b1;
                                alu_func4_1 <= 4'd0;
                            end
//AUIPC 当 lane1（放开的第一档）：pc 走 r1 的立即数口、immU 走 r2，func=ADD
//  —— 与 lane0 那一支逐字同形，只是 pc 操作数换成 lane1 自己那份。
                            OPCODE_AUIPC: begin
                                r1_1_imm_val <= pc_operand_1_in;
                                r1_1_imm_sel <= 1'b1;
                                r2_1_imm_val <= imm1_alu_in;
                                r2_1_imm_sel <= 1'b1;
                                alu_func4_1 <= 4'd0;
                            end
                        endcase
                    end
                end
                else begin
//没换新那拍：lane1 组显式自保持（与载荷块"保持支"同口径）
                    lane1_v_out <= lane1_v_out;
                    rs1B_out <= rs1B_out;
                    rs2B_out <= rs2B_out;
                    opc_1_q <= opc_1_q;
                    fn10_1_q <= fn10_1_q;
                    fn10_ls_1_q <= fn10_ls_1_q;
                    off_mem_1_q <= off_mem_1_q;
                    aux_addr_1_q <= aux_addr_1_q;
                    issue_idx1 <= issue_idx1;
                    issue_gen1 <= issue_gen1;
                    sel_slot1_1 <= sel_slot1_1;
                    sel_slot2_1 <= sel_slot2_1;
                    sel_v1_1 <= sel_v1_1;
                    sel_v2_1 <= sel_v2_1;
                    rd1_out <= rd1_out;
                    we1_q <= we1_q;
                    alu_func4_1 <= alu_func4_1;
                    r1_1_imm_val <= r1_1_imm_val;
                    r2_1_imm_val <= r2_1_imm_val;
                    r1_1_imm_sel <= r1_1_imm_sel;
                    r2_1_imm_sel <= r2_1_imm_sel;
                end
                    opc_out <= opc_out_d;
                    fn10_out <= fn10_out_d;
                    fn10_ls_out <= fn10_ls_out_d;
                    off_mem_out <= off_mem_out_d;
                    rs1_out <= rs1_out_d;
                    rs2_out <= rs2_out_d;
                    rd_out <= rd_out_d;
                    alu_func4 <= alu_func4_d;
                    csr_func3 <= csr_func3_d;
                    we_q <= we_d;
                    csr_wr_en <= csr_wr_en_d;
                    csr_addr <= csr_addr_d;
                    csr_data <= csr_data_d;
                    br_flag <= br_flag_d;
                    exc_irq_ret <= exc_irq_ret_d;
                    exc_ecall <= exc_ecall_d;
                    exc_ebreak <= exc_ebreak_d;
                    jal_flag <= jal_flag_d;
                    jalr_flag <= jalr_flag_d;
                    beq_off_q2 <= beq_off_q2_d;
                    aux_addr_out <= aux_addr_out_d;
                    exc_jal_misalign_out <= exc_jal_misalign_out_d;
                    jal_target_out <= jal_target_out_d;
                    exc_illegal_out <= exc_illegal_out_d;
                    br_pred_taken_out <= br_pred_taken_out_d;
                    jalr_pred_addr_out <= jalr_pred_addr_out_d;
                    r1_imm_val <= r1_imm_val_d;
                    r2_imm_val <= r2_imm_val_d;
                    r1_imm_sel <= r1_imm_sel_d;
                    r2_imm_sel <= r2_imm_sel_d;
            end
            else if ((stall_rob_full | stall_pc_redir | stall_lsu_haz
                    | stall_lsu_full | stall_mulu_haz | stall_mulu_div
 | stall_icache_miss | stall_bus_hold)
                   & ~(flush_con_exc | flush_con_irq | flush_con_jump)) begin
                issue_idx <= issue_idx;
                issue_gen <= issue_gen;
                sel_slot1 <= sel_slot1;
                sel_slot2 <= sel_slot2;
                sel_v1 <= sel_v1;
                sel_v2 <= sel_v2;
                lane1_v_out <= lane1_v_out;
                rs1B_out <= rs1B_out;
                rs2B_out <= rs2B_out;
                r1_1_imm_val <= r1_1_imm_val;
                r2_1_imm_val <= r2_1_imm_val;
                r1_1_imm_sel <= r1_1_imm_sel;
                r2_1_imm_sel <= r2_1_imm_sel;
                rd1_out <= rd1_out;
                issue_idx1 <= issue_idx1;
                issue_gen1 <= issue_gen1;
                sel_slot1_1 <= sel_slot1_1;
                sel_slot2_1 <= sel_slot2_1;
                sel_v1_1 <= sel_v1_1;
                sel_v2_1 <= sel_v2_1;
                alu_func4_1 <= alu_func4_1;
                we1_q <= we1_q;
                opc_out <= opc_out;
                fn10_out <= fn10_out;
                fn10_ls_out <= fn10_ls_out;
                off_mem_out <= off_mem_out;
                rs1_out <= rs1_out;
                rs2_out <= rs2_out;
                rd_out <= rd_out;
                alu_func4 <= alu_func4;
                csr_func3 <= csr_func3;
                we_q <= we;
                csr_wr_en <= csr_wr_en;
                csr_addr <= csr_addr;
                csr_data <= csr_data;
                br_flag <= br_flag;
                exc_irq_ret <= exc_irq_ret;
                exc_ecall <= exc_ecall;
                exc_ebreak <= exc_ebreak;
                jal_flag <= jal_flag;
                jalr_flag <= jalr_flag;
                beq_off_q2 <= beq_off_q2;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
                exc_jal_misalign_out <= exc_jal_misalign_out;
                jal_target_out <= jal_target_out;
                exc_illegal_out <= exc_illegal_out;
                r1_imm_val <= r1_imm_val;
                r2_imm_val <= r2_imm_val;
                r1_imm_sel <= r1_imm_sel;
                r2_imm_sel <= r2_imm_sel;
            end
            else begin
                    issue_idx <= 3'd0;
                    issue_gen <= 1'b0;
                    sel_slot1 <= 3'd0;
                    sel_slot2 <= 3'd0;
                    sel_v1 <= 1'b0;
                    sel_v2 <= 1'b0;
                    lane1_v_out <= 1'b0;
                    rs1B_out <= 5'd0;
                    rs2B_out <= 5'd0;
                    r1_1_imm_val <= 32'd0;
                    r2_1_imm_val <= 32'd0;
                    r1_1_imm_sel <= 1'b0;
                    r2_1_imm_sel <= 1'b0;
                    rd1_out <= 5'd0;
                    issue_idx1 <= 3'd0;
                    issue_gen1 <= 1'b0;
                    sel_slot1_1 <= 3'd0;
                    sel_slot2_1 <= 3'd0;
                    sel_v1_1 <= 1'b0;
                    sel_v2_1 <= 1'b0;
                    alu_func4_1 <= 4'd0;
                    we1_q <= 1'b0;
                opc_out <= 7'd0;
                fn10_out <= 10'd0;
                off_mem_out <= 32'd0;
                rs1_out <= 5'd0;
                rs2_out <= 5'd0;
                rd_out <= 5'd0;
                alu_func4 <= 4'd0;
                we_q <= 1'b0;
                csr_wr_en <= 1'b0;
                csr_addr <= 12'd0;
                csr_data <= 32'd0;
                br_flag <= 1'b0;
                exc_irq_ret <= 1'b0;
                exc_ecall <= 1'b0;
                exc_ebreak <= 1'b0;
                jal_flag <= 1'b0;
                jalr_flag <= 1'b0;
                beq_off_q2 <= 32'd0;
                br_pred_taken_out <= 1'b0;
                jalr_pred_addr_out <= 32'd0;
                exc_jal_misalign_out <= 1'b0;
                jal_target_out <= 32'd0;
                exc_illegal_out <= 1'b0;
            end
        end
    end

endmodule
