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
    input [9:0] flag_bus,
    input [9:0] func10,
    input [4:0] rd_in,
    input [6:0] opcode,
    input [31:0] imm_alu_in, r1_data_final, r2_data_final,
    input [11:0] imm12_csr_in,
    input [4:0] imm5_csr_in,
    input [31:0] inst_in,
    input [31:0] offset_jalr0, offset_beq0_aux, pc_operand_in,
    input [31:0] aux_addr_in,
    input br_pred_taken_in,
    input [31:0] jalr_pred_addr_in,
    (* max_fanout = 32 *) output reg [31:0] r1_data_out, r2_data_out,
    output reg [4:0] rd_out, rd_back1,
    output reg [3:0] alu_func4,
    output reg [2:0] csr_func3,
    output reg we, irq_ret, trap, ebreak, jal_flag, jalr_flag,
    output reg csr_wr_en,
    output reg [11:0] csr_addr,
    output reg [11:0] csr_addr_pre,
    output reg [31:0] csr_data,
//交给 bju 的判定源：前三条与 alu 的输入同源（r1_data_out / r2_data_out / alu_func4），
//其余是本级寄存下来的跳转载荷与限定位，都由判定单元在下一拍使用
    output reg [31:0] aux_addr_out,
    output reg [31:0] beq_off_q2, jalr_pred_addr_out,
    output reg br_flag, br_pred_taken_out,
    output reg jal_misalign_out,
    output reg [31:0] jal_target_out,
    output reg illegal_out
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

//flag_bus = {exc, exec, flush_irq, flush_jump, dcache_hold, bus_hold_in, stall_m, stall_v, lsu_stall, icache_busy}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    reg illegal_now;
    always @(*) begin
        flush_w = flag_bus[9] | flag_bus[7] | flag_bus[6];
        stall_w = (flag_bus[5] | flag_bus[4] | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//csr 读口地址：就是本级【组合输入】里那条指令的 csr 号（即 csr_addr 的下一条流水位置），
//非 SYSTEM 清 0 —— 与 csr_addr 的语义完全一致，读口不必再自己挡非法地址。
//csr 的读值要提前一拍（见 csr.v 的读口注释），所以这条地址必须取组合版、不能取 csr_addr。
    always @(*) begin
        csr_addr_pre = 12'd0;
        if (opcode == OPCODE_SYSTEM) csr_addr_pre = imm12_csr_in[11:0];
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
        csr_ro_now        = (imm12_csr_in[11:10] == 2'b11);
        csr_exists_now    = 1'b0;
        case (imm12_csr_in)
            12'h300, 12'h301, 12'h304, 12'h305, 12'h340, 12'h341,
            12'h342, 12'h343, 12'h344, 12'hB00, 12'hB02, 12'hF14: csr_exists_now = 1'b1;
            default: csr_exists_now = 1'b0;
        endcase
    end

    always @(*) begin
        illegal_now = 1'b0;
        if (inst_in != 32'd0) begin
            case (opcode)
                OPCODE_LUI, OPCODE_AUIPC, OPCODE_JAL: begin
                    illegal_now = 1'b0;
                end
//LOAD 合法宽度：LB/LH/LW/LBU/LHU = 000/001/010/100/101 ⇒ 011/110/111 非法
                OPCODE_LOAD: begin
                    if (inst_in[14:12] == 3'b011) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b110) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b111) illegal_now = 1'b1;
                end
//STORE 合法宽度：SB/SH/SW = 000/001/010 ⇒ 011/100/101/110/111 非法
                OPCODE_STORE: begin
                    if (inst_in[14:12] == 3'b011) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b100) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b101) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b110) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b111) illegal_now = 1'b1;
                end
                OPCODE_JALR: begin
                    if (inst_in[14:12] != 3'b000) illegal_now = 1'b1;
                end
                OPCODE_BRANCH: begin
                    if (inst_in[14:12] == 3'b010) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b011) illegal_now = 1'b1;
                end
                OPCODE_OP_IMM: begin
                    if (inst_in[14:12] == 3'b001) begin
                        if (inst_in[31:25] != 7'd0) illegal_now = 1'b1;
                    end
                    if (inst_in[14:12] == 3'b101) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000)) illegal_now = 1'b1;
                    end
                end
                OPCODE_OP: begin
                    if (inst_in[31:25] == 7'b0000001) begin
                        illegal_now = 1'b0;
                    end
                    else if (inst_in[14:12] == 3'b000) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000)) illegal_now = 1'b1;
                    end
                    else if (inst_in[14:12] == 3'b001) begin
                        if (inst_in[31:25] != 7'd0) illegal_now = 1'b1;
                    end
                    else if (inst_in[14:12] == 3'b101) begin
                        if ((inst_in[31:25] != 7'd0) && (inst_in[31:25] != 7'b0100000)) illegal_now = 1'b1;
                    end
                    else begin
                        if (inst_in[31:25] != 7'd0) illegal_now = 1'b1;
                    end
                end
                OPCODE_MISC_MEM: begin
                    if (inst_in[14:12] == 3'b010) illegal_now = 1'b1;
                    if (inst_in[14:12] == 3'b011) illegal_now = 1'b1;
                end
                OPCODE_SYSTEM: begin
                    if (inst_in[14:12] == 3'b100) illegal_now = 1'b1;
//sret（0x102）：只对 S 模式有意义，本核是 M-only ⇒ 按非法指令（规范允许"实现为非法"）。
//★ wfi（0x105）保持【合法】并当 NOP：规范要求 WFI 是一条合法指令（可以什么都不做），
//  只有 mstatus.TW=1 且在低权限模式执行时才非法。
                    if (inst_in[14:12] == 3'b000) begin
                        if (inst_in[31:20] == 12'h102) illegal_now = 1'b1;
                    end
                    if (csr_is_access_now) begin
                        if (!csr_exists_now) illegal_now = 1'b1;
                        if (csr_ro_now) begin
                            if (csr_wr_now) illegal_now = 1'b1;
                        end
                    end
                end
                default: begin
                    illegal_now = 1'b1;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (rst_q) begin
            r1_data_out <= 32'd0;
            r2_data_out <= 32'd0;
            rd_out <= 5'd0;
            rd_back1 <= 5'd0;
            alu_func4 <= 4'd0;
            we <= 1'b0;
            csr_wr_en <= 1'b0;
            csr_addr <= 12'd0;
            csr_data <= 32'd0;
            br_flag <= 1'b0;
            irq_ret <= 1'b0;
            trap <= 1'b0;
            ebreak <= 1'b0;
            jal_flag <= 1'b0;
            jalr_flag <= 1'b0;
            beq_off_q2 <= 32'd0;
            br_pred_taken_out <= 1'b0;
            jalr_pred_addr_out <= 32'd0;
            jal_misalign_out <= 1'b0;
            jal_target_out <= 32'd0;
            illegal_out <= 1'b0;
        end
        else if (exec) begin
//在EXE状态下根据不同的opcode对指令进行二次解码
            if (!flush_w && !stall_w) begin
                r1_data_out <= 32'd0;
                r2_data_out <= 32'd0;
                rd_out <= 5'd0;
                rd_back1 <= 5'd0;
                alu_func4 <= 4'd0;
                csr_func3 <= 3'd0;
                we <= 1'b0;
                csr_wr_en <= 1'b0;
                csr_addr <= 12'd0;
                csr_data <= 32'd0;
                br_flag <= 1'b0;
                irq_ret <= 1'b0;
                trap <= 1'b0;
                ebreak <= 1'b0;
                jal_flag <= 1'b0;
                jalr_flag <= 1'b0;
                beq_off_q2 <= 32'd0;
                aux_addr_out <= aux_addr_in;
                jal_misalign_out <= (opcode == OPCODE_JAL) & inst_in[21];
                jal_target_out <= jal_target_now;
                illegal_out <= illegal_now;
                br_pred_taken_out <= br_pred_taken_in;
                jalr_pred_addr_out <= jalr_pred_addr_in;
                case (opcode)
                    OPCODE_OP: begin
                        r1_data_out <= r1_data_final;
                        r2_data_out <= r2_data_final;
                        rd_out <= rd_in;
//RV32M：写回由 mulu 独立完成，这条路必须让开 —— 否则同一条指令被写两次，
//而且 alu 会按【撞车的 func3】算出垃圾结果、再经 rd_back1 前递给紧邻的下一条。
//M 与普通 ALU 共用 OPCODE_OP，只能靠 funct7 区分（func10[9:3]==7'b0000001）。
                        if (func10[9:3] == 7'b0000001) begin
                            rd_back1 <= 5'd0;
                            we <= 1'b0;
                        end
                        else begin
                            rd_back1 <= rd_in;
                            alu_func4 <= {func10[8], func10[2:0]};
                            we <= 1'b1;
                        end
                    end
                    OPCODE_OP_IMM: begin
                        r1_data_out <= r1_data_final;
                        r2_data_out <= imm_alu_in;
                        rd_out <= rd_in;
                        rd_back1 <= rd_in;
                        alu_func4 <= {func10[8], func10[2:0]};
                        we <= 1'b1;
                    end
//jal存储pc值传递
                    OPCODE_JAL: begin
                        rd_out <= rd_in;
                        rd_back1 <= rd_in;
                        we <= 1'b1;
                        aux_addr_out <= aux_addr_in;
                        jal_flag <= 1'b1;
                    end
//jalr：偏移与基址各寄存一拍，真目标由判定块在下一拍相加得出
                    OPCODE_JALR: begin
                        rd_out <= rd_in;
                        rd_back1 <= rd_in;
                        we <= 1'b1;
                        jalr_flag <= 1'b1;
                        aux_addr_out <= aux_addr_in;
                        r1_data_out <= r1_data_final;
                        r2_data_out <= offset_jalr0;
                    end
//分支：两个比较源寄存一拍，比较由判定块在下一拍做
                    OPCODE_BRANCH: begin
                        aux_addr_out <= aux_addr_in;
                        beq_off_q2 <= offset_beq0_aux - 4'd4;
                        r1_data_out <= r1_data_final;
                        r2_data_out <= r2_data_final;
                        alu_func4 <= {1'b0, func10[2:0]};
                        br_flag <= 1'b1;
                    end
                    OPCODE_LUI: begin
                        r1_data_out <= 32'd0;
                        r2_data_out <= imm_alu_in;
                        rd_out <= rd_in;
                        rd_back1 <= rd_in;
                        alu_func4 <= 4'd0;
                        we <= 1'b1;
                    end
                    OPCODE_AUIPC: begin
                        r1_data_out <= pc_operand_in;
                        r2_data_out <= imm_alu_in;
                        rd_out <= rd_in;
                        rd_back1 <= rd_in;
                        alu_func4 <= 4'd0;
                        we <= 1'b1;
                    end
//SYSTEM类指令读写赋能，地址计算
                    OPCODE_SYSTEM: begin
                        rd_out <= rd_in;
                        rd_back1 <= rd_in;
                        csr_func3 <= func10[2:0];
                        irq_ret <= (func10[2:0] == 3'b000) && (imm12_csr_in == 12'h302);
//不写时不给出地址（地址 0 不是任何 CSR）：csr.v 的写 case 不命中 ⇒ 天然不写，
//写后读旁路也自然关掉（读要走真寄存器）。★ 不能用"清 csr_wr_en"来表达不写 ——
//csr_wr_en 是【双重身份】：alu 用它判定"这是条 CSR 指令"，据此把读值 cs_data 选进
//result 写 rd（alu.v:57）。清掉它会让 csrr 读回 0（踩过）。
                        if (csr_wr_now) begin
                            csr_addr <= imm12_csr_in[11:0];
                        end
                        else begin
                            csr_addr <= 12'd0;
                        end
                        case (func10[2:0])
                            3'b000: begin
                                we <= 1'b0;
                                csr_wr_en <= 1'b0;
                                trap <= (imm12_csr_in == 12'h000) || (imm12_csr_in == 12'h001);
                                ebreak <= (imm12_csr_in == 12'h001);
                            end
                            3'b001: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                                r1_data_out <= r1_data_final;
                            end
                            3'b010: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                                r1_data_out <= r1_data_final;
                            end
                            3'b011: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                                r1_data_out <= r1_data_final;
                            end
                            3'b101: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                                r1_data_out <= {27'd0, imm5_csr_in};
                            end
                            3'b110: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                                r1_data_out <= {27'd0, imm5_csr_in};
                            end
                            3'b111: begin
                                we <= 1'b1;
                                csr_wr_en <= 1'b1;
                                r1_data_out <= {27'd0, imm5_csr_in};
                            end
                            default: begin
                                we <= 1'b0;
                                csr_wr_en <= 1'b0;
                            end
                        endcase
                    end
                endcase
            end
            else if (stall_w) begin
                r1_data_out <= r1_data_out;
                r2_data_out <= r2_data_out;
                rd_out <= rd_out;
                rd_back1 <= rd_back1;
                alu_func4 <= alu_func4;
                csr_func3 <= csr_func3;
                we <= we;
                csr_wr_en <= csr_wr_en;
                csr_addr <= csr_addr;
                csr_data <= csr_data;
                br_flag <= br_flag;
                irq_ret <= irq_ret;
                trap <= trap;
                ebreak <= ebreak;
                jal_flag <= jal_flag;
                jalr_flag <= jalr_flag;
                beq_off_q2 <= beq_off_q2;
                aux_addr_out <= aux_addr_out;
                br_pred_taken_out <= br_pred_taken_out;
                jalr_pred_addr_out <= jalr_pred_addr_out;
                jal_misalign_out <= jal_misalign_out;
                jal_target_out <= jal_target_out;
                illegal_out <= illegal_out;
            end
            else begin
                r1_data_out <= 32'd0;
                r2_data_out <= 32'd0;
                rd_out <= 5'd0;
                rd_back1 <= 5'd0;
                alu_func4 <= 4'd0;
                we <= 1'b0;
                csr_wr_en <= 1'b0;
                csr_addr <= 12'd0;
                csr_data <= 32'd0;
                br_flag <= 1'b0;
                irq_ret <= 1'b0;
                trap <= 1'b0;
                ebreak <= 1'b0;
                jal_flag <= 1'b0;
                jalr_flag <= 1'b0;
                beq_off_q2 <= 32'd0;
                br_pred_taken_out <= 1'b0;
                jalr_pred_addr_out <= 32'd0;
                jal_misalign_out <= 1'b0;
                jal_target_out <= 32'd0;
                illegal_out <= 1'b0;
            end
        end
    end

endmodule
