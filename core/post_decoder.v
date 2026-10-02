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
    input [9:0] func10,
    input [4:0] rd_in,
    input [4:0] rs1_in, rs2_in,
    input [31:0] imm_alu_in,
//操作数（寄存器操作数走点①前送后的值；非寄存器操作数在下面被立即数覆盖）
    input [31:0] r1_data_in, r2_data_in,
    input [31:0] inst_in,
    input [31:0] offset_beq0_aux, pc_operand_in,
    input [31:0] aux_addr_in,
    input br_pred_taken_in,
    input [31:0] jalr_pred_addr_in,
//操作数装配结果（**组合直出**）：删掉 mid_decoder 后，regfile 的寄存读正好提供"pre 出拍 →
//下一拍"那一拍延迟，与载荷同拍 ⇒ 载荷里不必再存一份数据。
//★ 兜底（没有源口命中时）取的就是 regfile 那个寄存器 —— 它在停顿期间是【单调刷新】的
//  （命中源口才换、没命中保持），所以"源口上出现过之后又没了"的值不会丢。细节见 regfile.v。
    output reg [31:0] r1_data_post, r2_data_post,
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
//前送槽号（← rob 的扫描口）：本条指令的两个操作数该由【哪个 ROB 槽】供值。
//★ 它是【载荷】：只在同一个 `payload_go` 沿锁存（与 `issue_idx` 同生共死）。
//  冻结期间**绝不能重算** —— rob 的退项是沿生效、扫描是组合读 `ent_v`，重算会让扫描
//  漂到更老的匹配项 ⇒ 前送陈旧值。这是本方案的头号红线。
//★ 冲刷分支要【显式清 0】：让下游 mux 落回载荷默认值，免得冲刷拍选到垃圾。
    output reg [2:0] sel_slot1, sel_slot2,
    output reg       sel_v1,    sel_v2,
    input  [2:0] fwd_slot1_in, fwd_slot2_in,
    input        fwd_hit1_in,  fwd_hit2_in,
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
//寄存一拍的"载荷推进"脉冲：消费端（lsu/mulu）用它做【每条指令只收一次】的入口门 ——
//  载荷因 stall/hold_issue/flush 停住时它是 0，车厢一空也不会把同一条指令再收一遍。
//  寄存版是关键：直接用 payload_go 会经 stall_w 与消费端自己的 stall 成环（实测死锁）。
    output reg payload_go_q,
    output reg [3:0] alu_func4,
    output reg [2:0] csr_func3,
    output reg we, exc_irq_ret, exc_ecall, exc_ebreak, jal_flag, jalr_flag,
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
    output reg exc_illegal_out
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
    assign payload_go = exec & ~(flush_con_exc | flush_con_irq | flush_con_jump)
                      & ~(stall_rob_full | stall_pc_redir | stall_lsu_haz
                        | stall_lsu_full | stall_mulu_haz | stall_mulu_div
 | stall_icache_miss | stall_bus_hold);

    always @(posedge clk) begin
        if (rst_q)
            payload_go_q <= 1'b0;
        else
            payload_go_q <= payload_go;
    end

    reg [31:0] r1_imm_val, r2_imm_val;
    reg        r1_imm_sel, r2_imm_sel;

//操作数装配（组合直出）：**寄存器操作数取 live 的 regfile 寄存读**（删掉 mid_decoder 后，
//它正好提供"pre 出拍 → 下一拍"那一拍延迟、与本级载荷同拍）；**非寄存器操作数（立即数/pc/uimm）
//取上一拍就锁好的常量**（`*_imm_val`，与原来 r*_data_out 里那一份同位宽、只是资格位单列）。
//★ 常量必须寄存、不能在这里按 inst_in 组合选：本级的载荷装的是"上一拍进本级那条"，
//  而当拍 inst_in 已经是下一条 ⇒ 组合选会晚一条指令（实测 smoke 第一条写就错）。
    always @(*) begin
        r1_data_post = r1_imm_sel ? r1_imm_val : r1_data_in;
        r2_data_post = r2_imm_sel ? r2_imm_val : r2_data_in;
    end

    always @(posedge clk) begin
        if (rst_q) begin
            r1_imm_val <= 32'd0;
            r2_imm_val <= 32'd0;
            r1_imm_sel <= 1'b0;
            r2_imm_sel <= 1'b0;
            rd_out <= 5'd0;
            alu_func4 <= 4'd0;
            we <= 1'b0;
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
            sel_slot1 <= 3'd0;
            sel_slot2 <= 3'd0;
            sel_v1 <= 1'b0;
            sel_v2 <= 1'b0;
            opc_out <= 7'd0;
            fn10_out <= 10'd0;
            fn10_ls_out <= 10'd0;
            off_mem_out <= 32'd0;
            rs1_out <= 5'd0;
            rs2_out <= 5'd0;
        end
        else if (exec) begin
//在EXE状态下根据不同的opcode对指令进行二次解码
            if (~(flush_con_exc | flush_con_irq | flush_con_jump)
              & ~(stall_rob_full | stall_pc_redir | stall_lsu_haz
                 | stall_lsu_full | stall_mulu_haz | stall_mulu_div
 | stall_icache_miss | stall_bus_hold)) begin
                issue_idx <= idx_in;
                sel_slot1 <= fwd_slot1_in;
                sel_slot2 <= fwd_slot2_in;
                sel_v1 <= fwd_hit1_in;
                sel_v2 <= fwd_hit2_in;
                opc_out <= inst_in[6:0];
                fn10_out <= func10;
                fn10_ls_out <= {7'd0, inst_in[14:12]};
                off_mem_out <= imm_alu_in;
                rs1_out <= rs1_in;
                rs2_out <= rs2_in;
                rd_out <= 5'd0;
                alu_func4 <= 4'd0;
                csr_func3 <= 3'd0;
                we <= 1'b0;
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
                aux_addr_out <= aux_addr_in;
                exc_jal_misalign_out <= (inst_in[6:0] == OPCODE_JAL) & inst_in[21];
                jal_target_out <= jal_target_now;
                exc_illegal_out <= exc_illegal_now;
                br_pred_taken_out <= br_pred_taken_in;
                jalr_pred_addr_out <= jalr_pred_addr_in;
//操作数：底值 = 本级的组合输入（删掉 mid_decoder 后它就是 regfile 的寄存读 —— 含读侧 5 源旁路，
//与载荷同拍）；下面按 opcode 把非寄存器操作数覆盖掉。
                r1_imm_val <= r1_imm_val;
                r2_imm_val <= r2_imm_val;
                r1_imm_sel <= 1'b0;
                r2_imm_sel <= 1'b0;
                case (inst_in[6:0])
                    OPCODE_OP: begin
                        rd_out <= rd_in;
//RV32M：写回由 mulu 独立完成，这条路必须让开 —— 否则同一条指令被写两次，
//而且 alu 会按【撞车的 func3】算出垃圾结果。
//M 与普通 ALU 共用 OPCODE_OP，只能靠 funct7 区分（func10[9:3]==7'b0000001）。
                        if (func10[9:3] == 7'b0000001) begin
                            we <= 1'b0;
                        end
                        else begin
                            alu_func4 <= {func10[8], func10[2:0]};
                            we <= 1'b1;
                        end
                    end
                    OPCODE_OP_IMM: begin
                        r2_imm_val <= imm_alu_in;
                        r2_imm_sel <= 1'b1;
                        rd_out <= rd_in;
                        alu_func4 <= {func10[8], func10[2:0]};
                        we <= 1'b1;
                    end
//jal存储pc值传递
                    OPCODE_JAL: begin
                        rd_out <= rd_in;
                        we <= 1'b1;
                        aux_addr_out <= aux_addr_in;
                        jal_flag <= 1'b1;
                    end
//jalr：偏移与基址各寄存一拍，真目标由判定块在下一拍相加得出
                    OPCODE_JALR: begin
                        r2_imm_val <= imm_alu_in;
                        r2_imm_sel <= 1'b1;
                        rd_out <= rd_in;
                        we <= 1'b1;
                        jalr_flag <= 1'b1;
                        aux_addr_out <= aux_addr_in;
                    end
//分支：两个比较源寄存一拍，比较由判定块在下一拍做
                    OPCODE_BRANCH: begin
                        aux_addr_out <= aux_addr_in;
                        beq_off_q2 <= offset_beq0_aux - 4'd4;
                        alu_func4 <= {1'b0, func10[2:0]};
                        br_flag <= 1'b1;
                    end
                    OPCODE_LUI: begin
                        r2_imm_val <= imm_alu_in;
                        r2_imm_sel <= 1'b1;
                        rd_out <= rd_in;
                        alu_func4 <= 4'd0;
                        we <= 1'b1;
                    end
//AUIPC：r1 = 本条 PC（非寄存器操作数）、r2 = 立即数
                    OPCODE_AUIPC: begin
                        r1_imm_val <= pc_operand_in;
                        r1_imm_sel <= 1'b1;
                        r2_imm_val <= imm_alu_in;
                        r2_imm_sel <= 1'b1;
                        rd_out <= rd_in;
                        alu_func4 <= 4'd0;
                        we <= 1'b1;
                    end
//LOAD：r1 = 基址（寄存器操作数）；偏移走 off_mem_out（lsu 自己那一份）
                    OPCODE_LOAD: begin
                        rd_out <= rd_in;
                    end
//STORE：r1 = 基址、r2 = 存的数据，两个都是寄存器操作数
                    OPCODE_STORE: begin
                    end
//MISC_MEM（fence/fence.i）：不吃寄存器操作数
                    OPCODE_MISC_MEM: begin
                    end
//SYSTEM类指令读写赋能，地址计算
                    OPCODE_SYSTEM: begin
                        rd_out <= rd_in;
                        csr_func3 <= func10[2:0];
                        exc_irq_ret <= (func10[2:0] == 3'b000) && (inst_in[31:20] == 12'h302);
//不写时不给出地址（地址 0 不是任何 CSR）：csr.v 的写 case 不命中 ⇒ 天然不写，
//写后读旁路也自然关掉（读要走真寄存器）。★ 不能用"清 csr_wr_en"来表达不写 ——
//csr_wr_en 是【双重身份】：alu 用它判定"这是条 CSR 指令"，据此把读值 cs_data 选进
//result 写 rd（alu.v:57）。清掉它会让 csrr 读回 0（踩过）。
                        if (csr_wr_now) begin
                            csr_addr <= inst_in[31:20];
                        end
                        else begin
                            csr_addr <= 12'd0;
                        end
                        case (func10[2:0])
                            3'b000: begin
                                we <= 1'b0;
                                csr_wr_en <= 1'b0;
                                exc_ecall <= (inst_in[31:20] == 12'h000) || (inst_in[31:20] == 12'h001);
                                exc_ebreak <= (inst_in[31:20] == 12'h001);
                            end
//r1 的来路：csrrw/csrrs/csrrc 用 rs1（寄存器操作数 ⇒ 走前送）；csrrwi/si/ci 用 uimm。
                            3'b001: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                            end
                            3'b010: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                            end
                            3'b011: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                            end
                            3'b101: begin
                                r1_imm_val <= {27'd0, inst_in[19:15]};
                                r1_imm_sel <= 1'b1;
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                            end
                            3'b110: begin
                                r1_imm_val <= {27'd0, inst_in[19:15]};
                                r1_imm_sel <= 1'b1;
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                            end
                            3'b111: begin
                                r1_imm_val <= {27'd0, inst_in[19:15]};
                                r1_imm_sel <= 1'b1;
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                            end
                            default: begin
                                we <= 1'b0;
                                csr_wr_en <= 1'b0;
                            end
                        endcase
                    end
                endcase
//★ 故障指令一律不写 rd：乱序写回之后写不再由退口把关（完成即写）⇒ 必须在这里挡掉，
//  否则非法指令 / 非对齐跳转的垃圾结果会先落进寄存器堆（trap 交付时已经来不及收回）。
//  （ecall/ebreak 那条分支本来就置 we=0；访存非对齐由 lsu 拒绝，不发写回。）
                if (exc_illegal_now | ((inst_in[6:0] == OPCODE_JAL) & inst_in[21]))
                    we <= 1'b0;
            end
            else if ((stall_rob_full | stall_pc_redir | stall_lsu_haz
                    | stall_lsu_full | stall_mulu_haz | stall_mulu_div
 | stall_icache_miss | stall_bus_hold)
                   & ~(flush_con_exc | flush_con_irq | flush_con_jump)) begin
                issue_idx <= issue_idx;
                sel_slot1 <= sel_slot1;
                sel_slot2 <= sel_slot2;
                sel_v1 <= sel_v1;
                sel_v2 <= sel_v2;
                opc_out <= opc_out;
                fn10_out <= fn10_out;
                fn10_ls_out <= fn10_ls_out;
                off_mem_out <= off_mem_out;
                rs1_out <= rs1_out;
                rs2_out <= rs2_out;
                rd_out <= rd_out;
                alu_func4 <= alu_func4;
                csr_func3 <= csr_func3;
                we <= we;
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
                    sel_slot1 <= 3'd0;
                    sel_slot2 <= 3'd0;
                    sel_v1 <= 1'b0;
                    sel_v2 <= 1'b0;
                opc_out <= 7'd0;
                fn10_out <= 10'd0;
                off_mem_out <= 32'd0;
                rs1_out <= 5'd0;
                rs2_out <= 5'd0;
                rd_out <= 5'd0;
                alu_func4 <= 4'd0;
                we <= 1'b0;
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
