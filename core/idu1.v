`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: idu1
// Description:
//   取指队列与载荷之间的寄存器级（= 原 mid_decoder 的位置），并入 rd / func10 / imm 的译码。
//
//   输入是 iffu 的出队格（F 级，原样指令字 + 随行号），输出是本级的寄存器（S 级）：
//   全译码（入站分类 / 各单元字段 / 异常标记 / 非寄存器操作数覆盖）由 S 级的指令字
//   【组合】解出、并随载荷同沿寄存进 S 级（原来这一份解在 idu2 的入站拍上）。
//
//   ★ 两套译码源不同、用途不同：
//     - 载荷面（rd/func10/imm）与全译码解在 S 级的 inst 上，给 idu2 的入站；
//     - rob 分配口只判"这一类写不写 rd"，解在 **fifo 队头**（nh0）上 —— 入队发生在
//       "队头进 出队格"那一拍，rob 要的是那一条的 rd，不能裸取 inst[11:7]
//       （BRANCH/STORE 会变出幽灵写者）。
//
//   ★ 扫槽口（scan_rs）给的是【本级输入那一级】（F）的 rs：rob 的读拍在本级输出（S），
//     扫描必须提前一拍（见 rob.v 的相位说明）。
//////////////////////////////////////////////////////////////////////////////////


module idu1(
    input clk, rst,
    input [7:0] flag_bus,
//冲刷的年龄边界（与 idu2 的 kill 判据逐项同门）。输出级里挂着的那条【比冲刷点老】就不是错的，
//必须留住 —— 见下面 keep_0/keep_1 那段。
    input        rob_flush_part, flush_incl,
    input [3:0]  flush_idx, head_ptr,
    input [1:0]  st_pop,
    input [3:0]  st_flush_pop,
    output reg   credit_stall_o,
//给 fifo 的推进量 = 本拍真正装了几条（0/1/2）。iffu 按自己手上有效条数截断后移动指针。
    output reg [1:0] advance,
//本拍 lane0 装没装：lane1 到底拿出队格的第二格(h1)还是队头(h0)由它定 ⇒ 寄存器堆读口的
//lane1 地址要跟着它选（rs 在 F 级就要发出去，见 cpu_top）
    output reg   lane0_load_o,

//============================== lane0 ==============================
//来自 iffu 出队格（F 级）
    input [31:0] inst_in_0,
    input [31:0] addr_in_0,
    input        br_pred_in_0,
    input [31:0] jalr_pred_in_0,
    input [4:0]  r1_in_0, r2_in_0,
    input [3:0]  idx_in_0,
    input        gen_in_0,
//出队格这一格是不是真的有效（iffu 的 h0_v/h1_v）。不许用 addr!=0 去推 —— 那个判据要经过
//iffu 的组合 pop/sidx 才能在载荷上看到，拿它做 load 门会和 pop 成组合环。
    input        inst_v_0,
//操作数就绪（rob 按冻结生产者槽现算）与发射拍被接收（idu2 把"就绪 + FU 空闲"合成这一根）。
//"原地等"必须【整组一起停】：否则本级照推会把 S 级内容原地覆盖掉、那条永远进不了载荷。
    input        rdy1_in_0, rdy2_in_0,
    input        acc_in_0,
//给发射级：寄存器里有指令、且这条还没发出去。
//不能直接用 pay_v_out_0 —— 它描述"寄存器里有内容"，而 lane0 先发出去之后内容还在。
    output reg   lane_v_0,
//载荷（原 mid_decoder 的输出集合 + 随行号）
    output reg [31:0] inst_out_0,
    output reg [31:0] aux_addr_out_0,
    output reg        br_pred_taken_out_0,
    output reg [4:0]  r1_out_0, r2_out_0,
    output reg [3:0]  idx_out_0,
    output reg        gen_out_0,
    output reg        pay_v_out_0,
//rob 分配口（在队头上按 opcode 分类解）
    output reg        alloc_en_0,
    output reg        alloc_we_0,
    output reg        alloc_jmp_0,
//这一条会不会报完成口（给 rob 的"发出即置可退"用）：M 类（funct7==1 的 OP）与 LOAD。
//它们即使不写 rd（rd=x0），单元照样会报一笔完成口 ⇒ 不能当"没人会回报"提前置可退；
//否则它立刻退休、槽被回收，而它的完成口（除法 30+ 拍）晚回来时会撞上槽的新住户。
    output reg        alloc_pend_0,
    output reg [4:0]  alloc_rd_0,
//全译码（随 S 级寄存器送 idu2 的入站：分类 / 各单元字段 / 异常标记 / 非寄存器操作数覆盖）
    output reg [1:0]  dec_class_0,
    output reg        dec_we_0,
    output reg [4:0]  dec_rd_0,
    output reg        dec_is_st_0,
    output reg [2:0]  dec_mulop_0,
    output reg [2:0]  dec_lsf3_0,
    output reg [3:0]  dec_func4_0,
    output reg [2:0]  dec_csrf3_0,
    output reg        dec_csrw_en_0,
    output reg        dec_csrw_armed_0,
    output reg        dec_csrw_wr_0,
    output reg [11:0] dec_csra_0,
    output reg [31:0] dec_jval_0,
    output reg        dec_brf_0,
    output reg        dec_jalf_0,
    output reg        dec_jalfr_0,
    output reg [3:0]  dec_bjuf4_0,
    output reg        dec_r1_imm_0,
    output reg        dec_r2_imm_0,
    output reg [31:0] dec_r1_immv_0,
    output reg [31:0] dec_r2_immv_0,

//============================== lane1 ==============================
//与 lane0 逐条同形，一条不少、一条不多
    input [31:0] inst_in_1,
    input [31:0] addr_in_1,
    input        br_pred_in_1,
    input [31:0] jalr_pred_in_1,
    input [4:0]  r1_in_1, r2_in_1,
    input [3:0]  idx_in_1,
    input        gen_in_1,
    input        inst_v_1,
    input        rdy1_in_1, rdy2_in_1,
    input        acc_in_1,
    output reg   lane_v_1,
    output reg [31:0] inst_out_1,
    output reg [31:0] aux_addr_out_1,
    output reg        br_pred_taken_out_1,
    output reg [4:0]  r1_out_1, r2_out_1,
    output reg [3:0]  idx_out_1,
    output reg        gen_out_1,
    output reg        pay_v_out_1,
    output reg        alloc_en_1,
    output reg        alloc_we_1,
    output reg        alloc_jmp_1,
    output reg        alloc_pend_1,
    output reg [4:0]  alloc_rd_1,
    output reg [1:0]  dec_class_1,
    output reg        dec_we_1,
    output reg [4:0]  dec_rd_1,
    output reg        dec_is_st_1,
    output reg [2:0]  dec_mulop_1,
    output reg [2:0]  dec_lsf3_1,
    output reg [3:0]  dec_func4_1,
    output reg [2:0]  dec_csrf3_1,
    output reg        dec_csrw_en_1,
    output reg        dec_csrw_armed_1,
    output reg        dec_csrw_wr_1,
    output reg [11:0] dec_csra_1,
    output reg [31:0] dec_jval_1,
    output reg        dec_brf_1,
    output reg        dec_jalf_1,
    output reg        dec_jalfr_1,
    output reg [3:0]  dec_bjuf4_1,
    output reg        dec_r1_imm_1,
    output reg        dec_r2_imm_1,
    output reg [31:0] dec_r1_immv_1,
    output reg [31:0] dec_r2_immv_1
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核，工具只能在布局阶段自己复制。每个模块各自打一拍，
//寄存器就落在本模块旁边；全核都只打一拍，彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
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
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_SYSTEM = 7'b1110011;

    localparam CLS_ALU = 2'd0, CLS_MUL = 2'd1, CLS_LSU = 2'd2, CLS_BJU = 2'd3;

//bju 功能码：把"分支种类 / 跳转种类 / 四类异常"压成一个 4 位码，bju 只认这一个口。
//★ 分支档用 bit3 当类标志、低 3 位就是 funct3 ⇒ bju 的比较 case 与原来逐字一致；
//★ 非法编码覆盖一切（在 bjuf4 的译码里排第一）：非法分支 / 非法 jalr 一律不许再判。
    localparam BJU_NONE    = 4'd0;
    localparam BJU_JAL     = 4'd1;
    localparam BJU_JAL_MIS = 4'd2;
    localparam BJU_JALR    = 4'd3;
    localparam BJU_ECALL   = 4'd4;
    localparam BJU_EBREAK  = 4'd5;
    localparam BJU_ILLEGAL = 4'd6;
    localparam BJU_MRET    = 4'd7;

//提取不同类型指令立即数的函数块
    function [31:0] immB;
        input [31:0] inst;
        immB = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    endfunction

    function [31:0] immI;
        input [31:0] inst;
        immI = {{20{inst[31]}}, inst[31:20]};
    endfunction

    function [31:0] immS;
        input [31:0] inst;
        immS = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    endfunction

    function [31:0] immU;
        input [31:0] inst;
        immU = {inst[31:12], 12'd0};
    endfunction

    function [31:0] immJ;
        input [31:0] inst;
        immJ = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
    endfunction

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷位与停顿位可同时为 1，故保持【冲刷优先于停顿】。
    reg exec, flush_w, stall;
//★ 两条 lane 各自的"已发射"位。lane0/lane1 的发射彼此独立之后，"寄存器内容的消费"与"寄存器装载"
//  不再同拍：lane0 先发出去、lane1 还要等它的操作数的那几拍里，lane0 的寄存器里躺着的仍是同一条
//  指令 —— 没有这一对位，它会被每拍重新发一次（同一条指令重复进 ROB）。
    reg l0_con, l1_con, l0_free, l1_free, go, load0, load1;
    always @(*) begin
        flush_w = flag_bus[7] | flag_bus[6] | flag_bus[5];
        stall = (flag_bus[3] | flag_bus[2] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
//★ 正在发射的那条必须【同拍】算作已消费：只看寄存的 l0_con/l1_con 的话，发射那一拍两个寄存器
//  都还不算空，下一拍才装新的一对 ⇒ 每条 lane 变成两拍一条，双发射白做。
        l0_free = ~pay_v_out_0  | l0_con | acc_in_0;
        l1_free = ~pay_v_out_1 | l1_con | acc_in_1;
//★ 拆站之后本级重新对"操作数没到 / FU 忙"负责：下游没有站可以挂住一条指令，S 级这对流水寄存器
//  就是它唯一的落脚点。判据不用 rdy —— rdy 与单元接受条件一起由 idu2 在【发射拍】合成 acc_in_0/acc_in_1，
//  本级只认它们：这样"操作数就绪"与"FU 空闲"落在同一拍、同一个信号上，不会差一拍。
//★ 两条 lane 各自推进：谁空出来谁装，不再成对。lane0 在等操作数时 lane1 照样从 fifo 拿一条。
        go    = exec & ~flush_w & ~stall;
//★ load0/load1 必须带【出队格这条是不是真的有效】。addr==0 是全核约定的"空格"（真指令的 addr
//  是"自己地址+4"，最小是 4）。少了这一项就会出现：队列里只有 1 条有效、前端还在回填时，
//  两级一起装、advance=2 ⇒ 把不存在的第 2 格也推进了 ⇒ 那条被重复取出/重复分配
//  （实测：随机差分所有种子的第一条指令都进 ROB 两次，逐条写回比对的第一处不符就落在 pc=4）。
        load0 = go & l0_free & inst_v_0;
//lane1 这一拍拿的是哪一格（见下面 inst_s1）：lane0 也在装就拿 h1，否则拿 h0 ⇒ 有效性跟着选。
        load1 = go & l1_free & (load0 ? inst_v_1 : inst_v_0);
//给 fifo 的推进量 = 本拍真正装了几条（0/1/2）：指针按它走，不是"成对就 +2"。
        advance = {1'b0, load0} + {1'b0, load1};
        lane0_load_o = load0;
    end

//★ 输出级的冲刷清必须【按年龄】，不能无条件清。
//  两条 lane 独立推进 ⇒ lane0 可以跑到一条跳转、判定、冲刷，而此时 lane1 的那条**还停在输出级**
//  （它的 Q 被一条等除法的 MUL 顶着）。那条比跳转老 ⇒ 按 ROB 年龄它不在杀集里 ⇒ 它必须活下来继续发。
//  无条件清会把它从管线上抹掉，而前端已经重定向到跳转目标 ⇒ 它再也不会被取回，
//  ROB 里那一项永远 wr=0 ⇒ 队头卡死（实测：随机差分 seed14 死锁在 pc=0x310，e8 一直不完成）。
//  判据与 idu2 的 kill_0/1 逐字同门：冲刷部分杀（rob_flush_part=0 是陷阱/中断的整队杀）才谈年龄。
    reg [4:0] fl_age, s_age_0, s_age_1;
    reg       keep_0, keep_1;
    always @(*) begin
        fl_age   = {1'b0, (flush_idx - head_ptr)};
        s_age_0  = {1'b0, (idx_out_0 - head_ptr)};
        s_age_1  = {1'b0, (idx_out_1 - head_ptr)};
        keep_0 = rob_flush_part &
                 (flush_incl ? (s_age_0 <  fl_age) : (s_age_0 <= fl_age));
        keep_1 = rob_flush_part &
                 (flush_incl ? (s_age_1 <  fl_age) : (s_age_1 <= fl_age));
    end

    always @(posedge clk) begin
        if (rst_q | flush_w | load0) l0_con <= 1'b0;
        else if (acc_in_0)          l0_con <= 1'b1;
        if (rst_q | flush_w | load1) l1_con <= 1'b0;
        else if (acc_in_1)          l1_con <= 1'b1;
    end

//★ lane1 这一拍拿 fifo 的哪一格：lane0 也在装 ⇒ 拿第二格；lane0 没装（在等操作数）⇒ 拿队头。
//  所以 lane 编号不再代表程序序 —— 程序序由随指令带下去的 rob 索引保证，这里只按"装了几条"走指针。
    reg [31:0] inst_s1, addr_s1, jalr_s1;
    reg [4:0]  r1_s1, r2_s1;
    reg [3:0]  idx_s1;
    reg        brp_s1, gen_s1;
    always @(*) begin
        inst_s1 = load0 ? inst_in_1 : inst_in_0;
        addr_s1 = load0 ? addr_in_1 : addr_in_0;
        jalr_s1 = load0 ? jalr_pred_in_1 : jalr_pred_in_0;
        brp_s1  = load0 ? br_pred_in_1 : br_pred_in_0;
        r1_s1   = load0 ? r1_in_1 : r1_in_0;
        r2_s1   = load0 ? r2_in_1 : r2_in_0;
        idx_s1  = load0 ? idx_in_1 : idx_in_0;
        gen_s1  = load0 ? gen_in_1 : gen_in_0;
    end

//本 lane 的 D 有指令、且还没被收：发射级按它给 D 级发 credit
    always @(*) begin
        lane_v_0 = pay_v_out_0 & ~l0_con;
        lane_v_1 = pay_v_out_1 & ~l1_con;
        credit_stall_o = 1'b0;
    end

//译码字段的组合解算：源是本级的输入（F 级的指令字），口径与原来那一段逐条一致。
    reg [4:0]  rd_dec;
    reg [9:0]  func10_dec;
    reg [31:0] imm_dec;
    reg        jmp_dec;
    always @(*) begin
        rd_dec = 5'd0;
        func10_dec = 10'd0;
        imm_dec = 32'd0;
        jmp_dec = 1'b0;
        case (inst_in_0[6:0])
            OPCODE_OP: begin
                rd_dec = inst_in_0[11:7];
                func10_dec = {inst_in_0[31:25], inst_in_0[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_dec = immI(inst_in_0);
                rd_dec = inst_in_0[11:7];
                func10_dec = {(inst_in_0[14:12] == 3'b001 || inst_in_0[14:12] == 3'b101) ? inst_in_0[31:25] : 7'd0,
                            inst_in_0[14:12]};
            end
            OPCODE_JAL: begin
                rd_dec = inst_in_0[11:7];
                jmp_dec = 1'b1;
            end
            OPCODE_JALR: begin
                rd_dec = inst_in_0[11:7];
                imm_dec = immI(inst_in_0);
                jmp_dec = 1'b1;
            end
            OPCODE_BRANCH: begin
                imm_dec = immB(inst_in_0);
                func10_dec = {7'd0, inst_in_0[14:12]};
                jmp_dec = 1'b1;
            end
            OPCODE_LOAD: begin
                imm_dec = immI(inst_in_0);
                rd_dec = inst_in_0[11:7];
            end
            OPCODE_STORE: begin
                imm_dec = immS(inst_in_0);
            end
            OPCODE_LUI: begin
                imm_dec = immU(inst_in_0);
                rd_dec = inst_in_0[11:7];
            end
            OPCODE_AUIPC: begin
                imm_dec = immU(inst_in_0) - 4'd4;
                rd_dec = inst_in_0[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_dec = inst_in_0[11:7];
                func10_dec = {7'd0, inst_in_0[14:12]};
            end
        endcase
    end

    reg csr_is_access_dec, csr_exists_dec, csr_ro_dec, csr_wr_dec;
    always @(*) begin
        csr_is_access_dec = (inst_in_0[14:12] != 3'b000) && (inst_in_0[14:12] != 3'b100);
//★ "写"的规范口径：CSRRW/CSRRWI 恒写；CSRRS/CSRRC/CSRRSI/CSRRCI 只有源（rs1 或 uimm）非 0 才写。
//  这一根同时决定"真写使能"和"对只读 CSR 的写要不要报非法"，两者是同一个判据。
        csr_wr_dec        = (inst_in_0[14:12] == 3'b001) | (inst_in_0[14:12] == 3'b101) |
                            (inst_in_0[19:15] != 5'd0);
        csr_ro_dec        = (inst_in_0[31:30] == 2'b11);
        csr_exists_dec    = 1'b0;
        case (inst_in_0[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec = 1'b1;
            default: csr_exists_dec = 1'b0;
        endcase
    end

    reg exc_illegal_dec, exc_jmis_dec;
    always @(*) begin
        exc_illegal_dec = 1'b0;
        exc_jmis_dec    = (inst_in_0[6:0] == OPCODE_JAL) & inst_in_0[21];
        if (inst_in_0 != 32'd0) begin
            case (inst_in_0[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec = 1'b0;
                OPCODE_LOAD: begin
                    if (inst_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b110) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b111) exc_illegal_dec = 1'b1;
                end
                OPCODE_STORE: begin
                    if (inst_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b100) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b101) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b110) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b111) exc_illegal_dec = 1'b1;
                end
                OPCODE_JALR: begin
                    if (inst_in_0[14:12] != 3'b000) exc_illegal_dec = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (inst_in_0[14:12] == 3'b010) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (inst_in_0[14:12] == 3'b001) begin
                        if (inst_in_0[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                    if (inst_in_0[14:12] == 3'b101) begin
                        if ((inst_in_0[31:25] != 7'd0) && (inst_in_0[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (inst_in_0[31:25] == 7'b0000001) exc_illegal_dec = 1'b0;
                    else if (inst_in_0[14:12] == 3'b000) begin
                        if ((inst_in_0[31:25] != 7'd0) && (inst_in_0[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                    else if (inst_in_0[14:12] == 3'b001) begin
                        if (inst_in_0[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                    else if (inst_in_0[14:12] == 3'b101) begin
                        if ((inst_in_0[31:25] != 7'd0) && (inst_in_0[31:25] != 7'b0100000))
                            exc_illegal_dec = 1'b1;
                    end
                    else begin
                        if (inst_in_0[31:25] != 7'd0) exc_illegal_dec = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (inst_in_0[14:12] == 3'b010) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b011) exc_illegal_dec = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (inst_in_0[14:12] == 3'b100) exc_illegal_dec = 1'b1;
                    if (inst_in_0[14:12] == 3'b000) begin
                        case (inst_in_0[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;   // ecall / ebreak / mret / wfi（wfi 当 NOP）
                            default: exc_illegal_dec = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec) begin
                        if (!csr_exists_dec) exc_illegal_dec = 1'b1;
                        if (csr_ro_dec) begin
                            if (csr_wr_dec) exc_illegal_dec = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec = 1'b1;
            endcase
        end
    end

//入站分类 + 各类字段装配
    reg [1:0]  class_dec;
    reg        we_dec, csrw_en_dec, csrw_armed_dec, csrw_wr_dec;
    reg [4:0]  rd_pick_dec;
    reg        is_st_dec;
    reg [2:0]  mulop_dec, lsf3_dec, csrf3_dec;
    reg [3:0]  func4_dec;
    reg [11:0] csra_dec;
    reg [31:0] jval_dec;
    reg        brf_dec, jalf_dec, jalfr_dec;

    always @(*) begin
        class_dec      = CLS_ALU;
        we_dec         = 1'b0;
        csrw_en_dec    = 1'b0;
        csrw_armed_dec = 1'b0;
        csrw_wr_dec    = 1'b0;
        rd_pick_dec    = 5'd0;
        is_st_dec      = 1'b0;
        mulop_dec      = 3'd0;
        lsf3_dec       = 3'd0;
        csrf3_dec      = 3'd0;
        func4_dec      = 4'd0;
        csra_dec       = 12'd0;
        jval_dec       = 32'd0;
        brf_dec        = 1'b0;
        jalf_dec       = 1'b0;
        jalfr_dec      = 1'b0;

        case (inst_in_0[6:0])
            OPCODE_OP: begin
                if (inst_in_0[31:25] == 7'b0000001) begin
                    class_dec = CLS_MUL;
                    mulop_dec = inst_in_0[14:12];
                    rd_pick_dec = rd_dec;   //★ MUL 也要给 rd：不给 dec_rd=0 ⇒ pstat 不登记它（见 idu2 的 d_wr）
                end
                else begin
                    func4_dec   = {func10_dec[8], func10_dec[2:0]};
                    rd_pick_dec = rd_dec;
                    we_dec      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                func4_dec   = {func10_dec[8], func10_dec[2:0]};
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_JAL: begin
                class_dec   = CLS_BJU;
                jalf_dec    = 1'b1;
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_JALR: begin
                class_dec   = CLS_BJU;
                jalfr_dec   = 1'b1;
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
                jval_dec    = jalr_pred_in_0;
            end
            OPCODE_BRANCH: begin
                class_dec   = CLS_BJU;
                brf_dec     = 1'b1;
                func4_dec   = {1'b0, inst_in_0[14:12]};
                jval_dec    = imm_dec - 32'd4;
            end
            OPCODE_LUI: begin
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_AUIPC: begin
                rd_pick_dec = rd_dec;
                we_dec      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_dec   = CLS_LSU;
                lsf3_dec    = inst_in_0[14:12];
                rd_pick_dec = rd_dec;
                jval_dec    = imm_dec;
            end
            OPCODE_STORE: begin
                class_dec   = CLS_LSU;
                is_st_dec   = 1'b1;
                lsf3_dec    = inst_in_0[14:12];
                jval_dec    = imm_dec;
            end
            OPCODE_SYSTEM: begin
                class_dec   = CLS_BJU;
                csrf3_dec   = inst_in_0[14:12];
                rd_pick_dec = rd_dec;
                case (inst_in_0[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        we_dec         = 1'b1;
                        csrw_en_dec    = 1'b1;
                        csrw_armed_dec = 1'b1;
                        csrw_wr_dec    = csr_wr_dec;
                    end
                    default: begin
                        we_dec      = 1'b0;
                        csrw_en_dec = 1'b0;
                    end
                endcase
//★ csr 地址【不写也照给】：地址供 csr.v 的读口用，写不写只决定"锁不锁"。
//  `csrr rd, csr` 若被清成地址 0 就会读回 0（实测：启动宏读 mtvec 读回 0 ⇒ 误判没写进去）。
                csra_dec = (inst_in_0[14:12] != 3'b000) ? inst_in_0[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec | exc_jmis_dec) begin
            we_dec         = 1'b0;
            csrw_en_dec    = 1'b0;
            csrw_armed_dec = 1'b0;
            csrw_wr_dec    = 1'b0;
        end
    end

//非寄存器操作数的覆盖（立即数在这里并进操作数）
    reg r1_imm_dec, r2_imm_dec;
    reg [31:0] r1_immv_dec, r2_immv_dec;
    always @(*) begin
        r1_imm_dec  = 1'b0;
        r2_imm_dec  = 1'b0;
        r1_immv_dec = 32'd0;
        r2_immv_dec = 32'd0;
        case (inst_in_0[6:0])
            OPCODE_OP_IMM: begin r2_imm_dec = 1'b1; r2_immv_dec = imm_dec; end
            OPCODE_JALR:   begin r2_imm_dec = 1'b1; r2_immv_dec = imm_dec; end
            OPCODE_LUI:    begin r2_imm_dec = 1'b1; r2_immv_dec = imm_dec; end
            OPCODE_AUIPC:  begin
                r1_imm_dec  = 1'b1; r1_immv_dec = addr_in_0;
                r2_imm_dec  = 1'b1; r2_immv_dec = imm_dec;
            end
            OPCODE_SYSTEM: begin
                if ((inst_in_0[14:12] == 3'b101) | (inst_in_0[14:12] == 3'b110) | (inst_in_0[14:12] == 3'b111)) begin
                    r1_imm_dec  = 1'b1;
                    r1_immv_dec = {27'd0, inst_in_0[19:15]};
                end
            end
            default: ;
        endcase
    end

//异常标记（ecall/ebreak/mret）
    reg exc_ecall_dec, exc_ebreak_dec, exc_irq_ret_dec;
    always @(*) begin
        exc_ecall_dec   = (inst_in_0[6:0] == OPCODE_SYSTEM) && (inst_in_0[31:20] == 12'h000);
        exc_ebreak_dec  = (inst_in_0[6:0] == OPCODE_SYSTEM) && (inst_in_0[31:20] == 12'h001);
        exc_irq_ret_dec = (inst_in_0[6:0] == OPCODE_SYSTEM) && (inst_in_0[14:12] == 3'b000)
                        && (inst_in_0[31:20] == 12'h302);
    end

//bju 功能码（本级的唯一一个"这条指令在 bju 里算什么"的口）：
//原来 bju 要分别吃 func4 / br_flag / jalr_flag / 四路异常五个口，其中 func4 与这四个
//没有任何区分力（addi/beq/jalr 的 func4 都是 0000），"是不是分支"只能靠额外的类位。
//这里一次译清：非法 > ecall > ebreak > mret > 分支（带 funct3）> jal（带非对齐）> jalr。
//★ 非法排第一：非法分支/非法 jalr 的编码不许再当控制指令判，否则会一边报非法一边改向。
    reg [3:0] bjuf4_dec;
    always @(*) begin
        bjuf4_dec = BJU_NONE;
        if (exc_illegal_dec)
            bjuf4_dec = BJU_ILLEGAL;
        else if (exc_ecall_dec)
            bjuf4_dec = BJU_ECALL;
        else if (exc_ebreak_dec)
            bjuf4_dec = BJU_EBREAK;
        else if (exc_irq_ret_dec)
            bjuf4_dec = BJU_MRET;
        else if (brf_dec)
            bjuf4_dec = {1'b1, func4_dec[2:0]};
        else if (jalf_dec)
            bjuf4_dec = exc_jmis_dec ? BJU_JAL_MIS : BJU_JAL;
        else if (jalfr_dec)
            bjuf4_dec = BJU_JALR;
    end

//rob 分配口：分配发生在"出队格 h0 换新"那一拍，吃的就是本级的译码输入（= h0 的 Q 端），
//与 idu1 自己的译码同源 —— 不再另解一份队头。rd 直接复用主译码的 rd_dec（按 opcode 分类
//填过，不裸取 inst[11:7]）。
//★ 发号拍 = 本级真装进一条【真指令】的那一拍。空泡（addr==0，队列空/被挡时出现）不占项：
//  它进不了发射级（lane_v 按 addr!=0 判），拿了项就没人发、没人写 ⇒ 那项永远退不掉 ⇒
//  ROB 排空 ⇒ 队头卡死（实测 icache 自举期间 15 拍就把 ROB 灌满，整核再不推进）。
    always @(*) begin
        alloc_en_0  = load0;
        alloc_rd_0   = rd_dec;
        alloc_we_0   = (rd_dec != 5'd0);
        alloc_jmp_0  = jmp_dec;
        alloc_pend_0 = (inst_in_0[6:0] == OPCODE_LOAD) |
                     ((inst_in_0[6:0] == OPCODE_OP) && (inst_in_0[31:25] == 7'b0000001));
    end

//lane1 的译码（与 lane0 一模，源换成 inst_s1 / addr_s1 / jalr_s1）
    reg [4:0]  rd_dec_1;
    reg [9:0]  func10_dec_1;
    reg [31:0] imm_dec_1;
    reg        jmp_dec_1;
    always @(*) begin
        rd_dec_1 = 5'd0;
        func10_dec_1 = 10'd0;
        imm_dec_1 = 32'd0;
        jmp_dec_1 = 1'b0;
        case (inst_s1[6:0])
            OPCODE_OP: begin
                rd_dec_1 = inst_s1[11:7];
                func10_dec_1 = {inst_s1[31:25], inst_s1[14:12]};
            end
            OPCODE_OP_IMM: begin
                imm_dec_1 = immI(inst_s1);
                rd_dec_1 = inst_s1[11:7];
                func10_dec_1 = {(inst_s1[14:12] == 3'b001 || inst_s1[14:12] == 3'b101) ? inst_s1[31:25] : 7'd0,
                            inst_s1[14:12]};
            end
            OPCODE_JAL: begin
                rd_dec_1 = inst_s1[11:7];
                jmp_dec_1 = 1'b1;
            end
            OPCODE_JALR: begin
                rd_dec_1 = inst_s1[11:7];
                imm_dec_1 = immI(inst_s1);
                jmp_dec_1 = 1'b1;
            end
            OPCODE_BRANCH: begin
                imm_dec_1 = immB(inst_s1);
                func10_dec_1 = {7'd0, inst_s1[14:12]};
                jmp_dec_1 = 1'b1;
            end
            OPCODE_LOAD: begin
                imm_dec_1 = immI(inst_s1);
                rd_dec_1 = inst_s1[11:7];
            end
            OPCODE_STORE: begin
                imm_dec_1 = immS(inst_s1);
            end
            OPCODE_LUI: begin
                imm_dec_1 = immU(inst_s1);
                rd_dec_1 = inst_s1[11:7];
            end
            OPCODE_AUIPC: begin
                imm_dec_1 = immU(inst_s1) - 4'd4;
                rd_dec_1 = inst_s1[11:7];
            end
            OPCODE_SYSTEM: begin
                rd_dec_1 = inst_s1[11:7];
                func10_dec_1 = {7'd0, inst_s1[14:12]};
            end
        endcase
    end

    reg csr_is_access_dec_1, csr_exists_dec_1, csr_ro_dec_1, csr_wr_dec_1;
    always @(*) begin
        csr_is_access_dec_1 = (inst_s1[14:12] != 3'b000) && (inst_s1[14:12] != 3'b100);
        csr_wr_dec_1        = (inst_s1[14:12] == 3'b001) | (inst_s1[14:12] == 3'b101) |
                            (inst_s1[19:15] != 5'd0);
        csr_ro_dec_1        = (inst_s1[31:30] == 2'b11);
        csr_exists_dec_1    = 1'b0;
        case (inst_s1[31:20])
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_dec_1 = 1'b1;
            default: csr_exists_dec_1 = 1'b0;
        endcase
    end

    reg exc_illegal_dec_1, exc_jmis_dec_1;
    always @(*) begin
        exc_illegal_dec_1 = 1'b0;
        exc_jmis_dec_1    = (inst_s1[6:0] == OPCODE_JAL) & inst_s1[21];
        if (inst_s1 != 32'd0) begin
            case (inst_s1[6:0])
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: exc_illegal_dec_1 = 1'b0;
                OPCODE_LOAD: begin
                    if (inst_s1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b110) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b111) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_STORE: begin
                    if (inst_s1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b100) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b101) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b110) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b111) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_JALR: begin
                    if (inst_s1[14:12] != 3'b000) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (inst_s1[14:12] == 3'b010) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (inst_s1[14:12] == 3'b001) begin
                        if (inst_s1[31:25] != 7'd0) exc_illegal_dec_1 = 1'b1;
                    end
                    if (inst_s1[14:12] == 3'b101) begin
                        if ((inst_s1[31:25] != 7'd0) && (inst_s1[31:25] != 7'b0100000))
                            exc_illegal_dec_1 = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (inst_s1[31:25] == 7'b0000001) exc_illegal_dec_1 = 1'b0;
                    else if (inst_s1[14:12] == 3'b000) begin
                        if ((inst_s1[31:25] != 7'd0) && (inst_s1[31:25] != 7'b0100000))
                            exc_illegal_dec_1 = 1'b1;
                    end
                    else if (inst_s1[14:12] == 3'b001) begin
                        if (inst_s1[31:25] != 7'd0) exc_illegal_dec_1 = 1'b1;
                    end
                    else if (inst_s1[14:12] == 3'b101) begin
                        if ((inst_s1[31:25] != 7'd0) && (inst_s1[31:25] != 7'b0100000))
                            exc_illegal_dec_1 = 1'b1;
                    end
                    else begin
                        if (inst_s1[31:25] != 7'd0) exc_illegal_dec_1 = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (inst_s1[14:12] == 3'b010) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b011) exc_illegal_dec_1 = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (inst_s1[14:12] == 3'b100) exc_illegal_dec_1 = 1'b1;
                    if (inst_s1[14:12] == 3'b000) begin
                        case (inst_s1[31:20])
                            12'h000, 12'h001, 12'h302, 12'h105: ;
                            default: exc_illegal_dec_1 = 1'b1;
                        endcase
                    end
                    if (csr_is_access_dec_1) begin
                        if (!csr_exists_dec_1) exc_illegal_dec_1 = 1'b1;
                        if (csr_ro_dec_1) begin
                            if (csr_wr_dec_1) exc_illegal_dec_1 = 1'b1;
                        end
                    end
                end
                default: exc_illegal_dec_1 = 1'b1;
            endcase
        end
    end

    reg [1:0]  class_dec_1;
    reg        we_dec_1, csrw_en_dec_1, csrw_armed_dec_1, csrw_wr_dec_1;
    reg [4:0]  rd_pick_dec_1;
    reg        is_st_dec_1;
    reg [2:0]  mulop_dec_1, lsf3_dec_1, csrf3_dec_1;
    reg [3:0]  func4_dec_1;
    reg [11:0] csra_dec_1;
    reg [31:0] jval_dec_1;
    reg        brf_dec_1, jalf_dec_1, jalfr_dec_1;
    always @(*) begin
        class_dec_1      = CLS_ALU;
        we_dec_1         = 1'b0;
        csrw_en_dec_1    = 1'b0;
        csrw_armed_dec_1 = 1'b0;
        csrw_wr_dec_1    = 1'b0;
        rd_pick_dec_1    = 5'd0;
        is_st_dec_1      = 1'b0;
        mulop_dec_1      = 3'd0;
        lsf3_dec_1       = 3'd0;
        csrf3_dec_1      = 3'd0;
        func4_dec_1      = 4'd0;
        csra_dec_1       = 12'd0;
        jval_dec_1       = 32'd0;
        brf_dec_1        = 1'b0;
        jalf_dec_1       = 1'b0;
        jalfr_dec_1      = 1'b0;
        case (inst_s1[6:0])
            OPCODE_OP: begin
                if (inst_s1[31:25] == 7'b0000001) begin
                    class_dec_1 = CLS_MUL;
                    mulop_dec_1 = inst_s1[14:12];
                    rd_pick_dec_1 = rd_dec_1;
                end
                else begin
                    func4_dec_1   = {func10_dec_1[8], func10_dec_1[2:0]};
                    rd_pick_dec_1 = rd_dec_1;
                    we_dec_1      = 1'b1;
                end
            end
            OPCODE_OP_IMM: begin
                func4_dec_1   = {func10_dec_1[8], func10_dec_1[2:0]};
                rd_pick_dec_1 = rd_dec_1;
                we_dec_1      = 1'b1;
            end
            OPCODE_JAL: begin
                class_dec_1   = CLS_BJU;
                jalf_dec_1    = 1'b1;
                rd_pick_dec_1 = rd_dec_1;
                we_dec_1      = 1'b1;
            end
            OPCODE_JALR: begin
                class_dec_1   = CLS_BJU;
                jalfr_dec_1   = 1'b1;
                rd_pick_dec_1 = rd_dec_1;
                we_dec_1      = 1'b1;
                jval_dec_1    = jalr_s1;
            end
            OPCODE_BRANCH: begin
                class_dec_1   = CLS_BJU;
                brf_dec_1     = 1'b1;
                func4_dec_1   = {1'b0, inst_s1[14:12]};
                jval_dec_1    = imm_dec_1 - 32'd4;
            end
            OPCODE_LUI: begin
                rd_pick_dec_1 = rd_dec_1;
                we_dec_1      = 1'b1;
            end
            OPCODE_AUIPC: begin
                rd_pick_dec_1 = rd_dec_1;
                we_dec_1      = 1'b1;
            end
            OPCODE_LOAD: begin
                class_dec_1   = CLS_LSU;
                lsf3_dec_1    = inst_s1[14:12];
                rd_pick_dec_1 = rd_dec_1;
                jval_dec_1    = imm_dec_1;
            end
            OPCODE_STORE: begin
                class_dec_1   = CLS_LSU;
                is_st_dec_1   = 1'b1;
                lsf3_dec_1    = inst_s1[14:12];
                jval_dec_1    = imm_dec_1;
            end
            OPCODE_SYSTEM: begin
                class_dec_1   = CLS_BJU;
                csrf3_dec_1   = inst_s1[14:12];
                rd_pick_dec_1 = rd_dec_1;
                case (inst_s1[14:12])
                    3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin
                        we_dec_1         = 1'b1;
                        csrw_en_dec_1    = 1'b1;
                        csrw_armed_dec_1 = 1'b1;
                        csrw_wr_dec_1    = csr_wr_dec_1;
                    end
                    default: begin
                        we_dec_1      = 1'b0;
                        csrw_en_dec_1 = 1'b0;
                    end
                endcase
                csra_dec_1 = (inst_s1[14:12] != 3'b000) ? inst_s1[31:20] : 12'd0;
            end
            default: ;
        endcase
        if (exc_illegal_dec_1 | exc_jmis_dec_1) begin
            we_dec_1         = 1'b0;
            csrw_en_dec_1    = 1'b0;
            csrw_armed_dec_1 = 1'b0;
            csrw_wr_dec_1    = 1'b0;
        end
    end

    reg r1_imm_dec_1, r2_imm_dec_1;
    reg [31:0] r1_immv_dec_1, r2_immv_dec_1;
    always @(*) begin
        r1_imm_dec_1  = 1'b0;
        r2_imm_dec_1  = 1'b0;
        r1_immv_dec_1 = 32'd0;
        r2_immv_dec_1 = 32'd0;
        case (inst_s1[6:0])
            OPCODE_OP_IMM: begin r2_imm_dec_1 = 1'b1; r2_immv_dec_1 = imm_dec_1; end
            OPCODE_JALR:   begin r2_imm_dec_1 = 1'b1; r2_immv_dec_1 = imm_dec_1; end
            OPCODE_LUI:    begin r2_imm_dec_1 = 1'b1; r2_immv_dec_1 = imm_dec_1; end
            OPCODE_AUIPC:  begin
                r1_imm_dec_1  = 1'b1; r1_immv_dec_1 = addr_s1;
                r2_imm_dec_1  = 1'b1; r2_immv_dec_1 = imm_dec_1;
            end
            OPCODE_SYSTEM: begin
                if ((inst_s1[14:12] == 3'b101) | (inst_s1[14:12] == 3'b110) | (inst_s1[14:12] == 3'b111)) begin
                    r1_imm_dec_1  = 1'b1;
                    r1_immv_dec_1 = {27'd0, inst_s1[19:15]};
                end
            end
            default: ;
        endcase
    end

    reg exc_ecall_dec_1, exc_ebreak_dec_1, exc_irq_ret_dec_1;
    always @(*) begin
        exc_ecall_dec_1   = (inst_s1[6:0] == OPCODE_SYSTEM) && (inst_s1[31:20] == 12'h000);
        exc_ebreak_dec_1  = (inst_s1[6:0] == OPCODE_SYSTEM) && (inst_s1[31:20] == 12'h001);
        exc_irq_ret_dec_1 = (inst_s1[6:0] == OPCODE_SYSTEM) && (inst_s1[14:12] == 3'b000)
                        && (inst_s1[31:20] == 12'h302);
    end

//bju 功能码（与 lane0 逐条同形）
    reg [3:0] bjuf4_dec_1;
    always @(*) begin
        bjuf4_dec_1 = BJU_NONE;
        if (exc_illegal_dec_1)
            bjuf4_dec_1 = BJU_ILLEGAL;
        else if (exc_ecall_dec_1)
            bjuf4_dec_1 = BJU_ECALL;
        else if (exc_ebreak_dec_1)
            bjuf4_dec_1 = BJU_EBREAK;
        else if (exc_irq_ret_dec_1)
            bjuf4_dec_1 = BJU_MRET;
        else if (brf_dec_1)
            bjuf4_dec_1 = {1'b1, func4_dec_1[2:0]};
        else if (jalf_dec_1)
            bjuf4_dec_1 = exc_jmis_dec_1 ? BJU_JAL_MIS : BJU_JAL;
        else if (jalfr_dec_1)
            bjuf4_dec_1 = BJU_JALR;
    end

    always @(*) begin
        alloc_en_1   = load1;
        alloc_rd_1   = rd_dec_1;
        alloc_we_1   = (rd_dec_1 != 5'd0);
        alloc_jmp_1  = jmp_dec_1;
        alloc_pend_1 = (inst_s1[6:0] == OPCODE_LOAD) |
                     ((inst_s1[6:0] == OPCODE_OP) && (inst_s1[31:25] == 7'b0000001));
    end

//推进/保持/清：与 fifo 出队格的"换新/保持/清"【逐字同形】。
//★ 条件必须与上一级一致：本级的"保持"要跟上一级的"保持"落在同一拍，差一拍就是整条流水线错位。
    always @(posedge clk) begin
        if (rst_q) begin
            inst_out_0 <= 32'd0;
            aux_addr_out_0 <= 32'd0;
            br_pred_taken_out_0 <= 1'b0;
            r1_out_0 <= 5'd0;
            r2_out_0 <= 5'd0;
            idx_out_0 <= 4'd0;
            gen_out_0 <= 1'b0;
            pay_v_out_0 <= 1'b0;
            dec_class_0 <= 2'd0;
            dec_we_0 <= 1'b0;
            dec_rd_0 <= 5'd0;
            dec_is_st_0 <= 1'b0;
            dec_mulop_0 <= 3'd0;
            dec_lsf3_0 <= 3'd0;
            dec_func4_0 <= 4'd0;
            dec_csrf3_0 <= 3'd0;
            dec_csrw_en_0 <= 1'b0;
            dec_csrw_armed_0 <= 1'b0;
            dec_csrw_wr_0 <= 1'b0;
            dec_csra_0 <= 12'd0;
            dec_jval_0 <= 32'd0;
            dec_brf_0 <= 1'b0;
            dec_jalf_0 <= 1'b0;
            dec_jalfr_0 <= 1'b0;
            dec_bjuf4_0 <= 4'd0;
            dec_r1_imm_0 <= 1'b0;
            dec_r2_imm_0 <= 1'b0;
            dec_r1_immv_0 <= 32'd0;
            dec_r2_immv_0 <= 32'd0;
        end
        else if (exec) begin
            if (load0) begin
                inst_out_0 <= inst_in_0;
                aux_addr_out_0 <= addr_in_0;
                br_pred_taken_out_0 <= br_pred_in_0;
                r1_out_0 <= r1_in_0;
                r2_out_0 <= r2_in_0;
                idx_out_0 <= idx_in_0;
                gen_out_0 <= gen_in_0;
                pay_v_out_0 <= (addr_in_0 != 32'd0);
                dec_class_0 <= class_dec;
                dec_we_0 <= we_dec;
                dec_rd_0 <= rd_pick_dec;
                dec_is_st_0 <= is_st_dec;
                dec_mulop_0 <= mulop_dec;
                dec_lsf3_0 <= lsf3_dec;
                dec_func4_0 <= func4_dec;
                dec_csrf3_0 <= csrf3_dec;
                dec_csrw_en_0 <= csrw_en_dec;
                dec_csrw_armed_0 <= csrw_armed_dec;
                dec_csrw_wr_0 <= csrw_wr_dec;
                dec_csra_0 <= csra_dec;
                dec_jval_0 <= jval_dec;
                dec_brf_0 <= brf_dec;
                dec_jalf_0 <= jalf_dec;
                dec_jalfr_0 <= jalfr_dec;
                dec_bjuf4_0 <= bjuf4_dec;
                dec_r1_imm_0 <= r1_imm_dec;
                dec_r2_imm_0 <= r2_imm_dec;
                dec_r1_immv_0 <= r1_immv_dec;
                dec_r2_immv_0 <= r2_immv_dec;
            end
//★ 保持判据现在是"只要不放行就保持"：load0/load1 已经吃进了 stall、操作数、FU 接受三类停顿，
//  再多列一遍必然漏项 —— 漏一项那一拍就掉进下面的"清"分支，S 级被清零、那条指令凭空消失。
//  唯一的例外是冲刷：冲刷要的就是清，所以下面 else 那一档留给 flush_w。
//  ★ 冲刷这一档再按年龄放宽：这条比冲刷点老 ⇒ 留在原地（keep_0），不许清。
            else if (~flush_w | keep_0) begin
                inst_out_0 <= inst_out_0;
                aux_addr_out_0 <= aux_addr_out_0;
                br_pred_taken_out_0 <= br_pred_taken_out_0;
                r1_out_0 <= r1_out_0;
                r2_out_0 <= r2_out_0;
                idx_out_0 <= idx_out_0;
                gen_out_0 <= gen_out_0;
                pay_v_out_0 <= pay_v_out_0;
                dec_class_0 <= dec_class_0;
                dec_we_0 <= dec_we_0;
                dec_rd_0 <= dec_rd_0;
                dec_is_st_0 <= dec_is_st_0;
                dec_mulop_0 <= dec_mulop_0;
                dec_lsf3_0 <= dec_lsf3_0;
                dec_func4_0 <= dec_func4_0;
                dec_csrf3_0 <= dec_csrf3_0;
                dec_csrw_en_0 <= dec_csrw_en_0;
                dec_csrw_armed_0 <= dec_csrw_armed_0;
                dec_csrw_wr_0 <= dec_csrw_wr_0;
                dec_csra_0 <= dec_csra_0;
                dec_jval_0 <= dec_jval_0;
                dec_brf_0 <= dec_brf_0;
                dec_jalf_0 <= dec_jalf_0;
                dec_jalfr_0 <= dec_jalfr_0;
                dec_bjuf4_0 <= dec_bjuf4_0;
                dec_r1_imm_0 <= dec_r1_imm_0;
                dec_r2_imm_0 <= dec_r2_imm_0;
                dec_r1_immv_0 <= dec_r1_immv_0;
                dec_r2_immv_0 <= dec_r2_immv_0;
            end
            else begin
                inst_out_0 <= 32'd0;
                aux_addr_out_0 <= 32'd0;
                br_pred_taken_out_0 <= 1'b0;
                r1_out_0 <= 5'd0;
                r2_out_0 <= 5'd0;
                idx_out_0 <= 4'd0;
                gen_out_0 <= 1'b0;
                pay_v_out_0 <= 1'b0;
                dec_class_0 <= 2'd0;
                dec_we_0 <= 1'b0;
                dec_rd_0 <= 5'd0;
                dec_is_st_0 <= 1'b0;
                dec_mulop_0 <= 3'd0;
                dec_lsf3_0 <= 3'd0;
                dec_func4_0 <= 4'd0;
                dec_csrf3_0 <= 3'd0;
                dec_csrw_en_0 <= 1'b0;
                dec_csrw_armed_0 <= 1'b0;
                dec_csrw_wr_0 <= 1'b0;
                dec_csra_0 <= 12'd0;
                dec_jval_0 <= 32'd0;
                dec_brf_0 <= 1'b0;
                dec_jalf_0 <= 1'b0;
                dec_jalfr_0 <= 1'b0;
                dec_bjuf4_0 <= 4'd0;
                dec_r1_imm_0 <= 1'b0;
                dec_r2_imm_0 <= 1'b0;
                dec_r1_immv_0 <= 32'd0;
                dec_r2_immv_0 <= 32'd0;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            inst_out_1 <= 32'd0;
            aux_addr_out_1 <= 32'd0;
            br_pred_taken_out_1 <= 1'b0;
            r1_out_1 <= 5'd0;
            r2_out_1 <= 5'd0;
            idx_out_1 <= 4'd0;
            gen_out_1 <= 1'b0;
            pay_v_out_1 <= 1'b0;
            dec_class_1 <= 2'd0;
            dec_we_1 <= 1'b0;
            dec_rd_1 <= 5'd0;
            dec_is_st_1 <= 1'b0;
            dec_mulop_1 <= 3'd0;
            dec_lsf3_1 <= 3'd0;
            dec_func4_1 <= 4'd0;
            dec_csrf3_1 <= 3'd0;
            dec_csrw_en_1 <= 1'b0;
            dec_csrw_armed_1 <= 1'b0;
            dec_csrw_wr_1 <= 1'b0;
            dec_csra_1 <= 12'd0;
            dec_jval_1 <= 32'd0;
            dec_brf_1 <= 1'b0;
            dec_jalf_1 <= 1'b0;
            dec_jalfr_1 <= 1'b0;
            dec_bjuf4_1 <= 4'd0;
            dec_r1_imm_1 <= 1'b0;
            dec_r2_imm_1 <= 1'b0;
            dec_r1_immv_1 <= 32'd0;
            dec_r2_immv_1 <= 32'd0;
        end
        else if (exec) begin
            if (load1) begin
                inst_out_1 <= inst_s1;
                aux_addr_out_1 <= addr_s1;
                br_pred_taken_out_1 <= brp_s1;
                r1_out_1 <= r1_s1;
                r2_out_1 <= r2_s1;
                idx_out_1 <= idx_s1;
                gen_out_1 <= gen_s1;
                pay_v_out_1 <= (addr_s1 != 32'd0);
                dec_class_1 <= class_dec_1;
                dec_we_1 <= we_dec_1;
                dec_rd_1 <= rd_pick_dec_1;
                dec_is_st_1 <= is_st_dec_1;
                dec_mulop_1 <= mulop_dec_1;
                dec_lsf3_1 <= lsf3_dec_1;
                dec_func4_1 <= func4_dec_1;
                dec_csrf3_1 <= csrf3_dec_1;
                dec_csrw_en_1 <= csrw_en_dec_1;
                dec_csrw_armed_1 <= csrw_armed_dec_1;
                dec_csrw_wr_1 <= csrw_wr_dec_1;
                dec_csra_1 <= csra_dec_1;
                dec_jval_1 <= jval_dec_1;
                dec_brf_1 <= brf_dec_1;
                dec_jalf_1 <= jalf_dec_1;
                dec_jalfr_1 <= jalfr_dec_1;
                dec_bjuf4_1 <= bjuf4_dec_1;
                dec_r1_imm_1 <= r1_imm_dec_1;
                dec_r2_imm_1 <= r2_imm_dec_1;
                dec_r1_immv_1 <= r1_immv_dec_1;
                dec_r2_immv_1 <= r2_immv_dec_1;
            end
//★ 同 lane0：冲刷清按年龄放宽（比冲刷点老的那条留在原地）。
            else if (~flush_w | keep_1) begin
                inst_out_1 <= inst_out_1;
                aux_addr_out_1 <= aux_addr_out_1;
                br_pred_taken_out_1 <= br_pred_taken_out_1;
                r1_out_1 <= r1_out_1;
                r2_out_1 <= r2_out_1;
                idx_out_1 <= idx_out_1;
                gen_out_1 <= gen_out_1;
                pay_v_out_1 <= pay_v_out_1;
                dec_class_1 <= dec_class_1;
                dec_we_1 <= dec_we_1;
                dec_rd_1 <= dec_rd_1;
                dec_is_st_1 <= dec_is_st_1;
                dec_mulop_1 <= dec_mulop_1;
                dec_lsf3_1 <= dec_lsf3_1;
                dec_func4_1 <= dec_func4_1;
                dec_csrf3_1 <= dec_csrf3_1;
                dec_csrw_en_1 <= dec_csrw_en_1;
                dec_csrw_armed_1 <= dec_csrw_armed_1;
                dec_csrw_wr_1 <= dec_csrw_wr_1;
                dec_csra_1 <= dec_csra_1;
                dec_jval_1 <= dec_jval_1;
                dec_brf_1 <= dec_brf_1;
                dec_jalf_1 <= dec_jalf_1;
                dec_jalfr_1 <= dec_jalfr_1;
                dec_bjuf4_1 <= dec_bjuf4_1;
                dec_r1_imm_1 <= dec_r1_imm_1;
                dec_r2_imm_1 <= dec_r2_imm_1;
                dec_r1_immv_1 <= dec_r1_immv_1;
                dec_r2_immv_1 <= dec_r2_immv_1;
            end
            else begin
                inst_out_1 <= 32'd0;
                aux_addr_out_1 <= 32'd0;
                br_pred_taken_out_1 <= 1'b0;
                r1_out_1 <= 5'd0;
                r2_out_1 <= 5'd0;
                idx_out_1 <= 4'd0;
                gen_out_1 <= 1'b0;
                pay_v_out_1 <= 1'b0;
                dec_class_1 <= 2'd0;
                dec_we_1 <= 1'b0;
                dec_rd_1 <= 5'd0;
                dec_is_st_1 <= 1'b0;
                dec_mulop_1 <= 3'd0;
                dec_lsf3_1 <= 3'd0;
                dec_func4_1 <= 4'd0;
                dec_csrf3_1 <= 3'd0;
                dec_csrw_en_1 <= 1'b0;
                dec_csrw_armed_1 <= 1'b0;
                dec_csrw_wr_1 <= 1'b0;
                dec_csra_1 <= 12'd0;
                dec_jval_1 <= 32'd0;
                dec_brf_1 <= 1'b0;
                dec_jalf_1 <= 1'b0;
                dec_jalfr_1 <= 1'b0;
                dec_bjuf4_1 <= 4'd0;
                dec_r1_imm_1 <= 1'b0;
                dec_r2_imm_1 <= 1'b0;
                dec_r1_immv_1 <= 32'd0;
                dec_r2_immv_1 <= 32'd0;
            end
        end
    end

endmodule
