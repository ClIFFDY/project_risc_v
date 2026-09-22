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


module decoder(
    input clk, rst,
    input [4:0] flag_bus,
    input [9:0] func10,
    input [4:0] rd_in,
    input [6:0] opcode,
    input [31:0] imm_alu_in, r1_data_final, r2_data_final,
    input [11:0] imm12_csr_in,
    input [4:0] imm5_csr_in,
    input [31:0] offset_jalr0, offset_beq0_aux, pc_operand_in,
    input [31:0] aux_addr_in,
    input br_pred_taken_in,
    input [31:0] jalr_pred_addr_in,
    output reg [31:0] r1_data_out, r2_data_out,
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
    output reg br_flag, br_pred_taken_out
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
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_SYSTEM = 7'b1110011;

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[3] | flag_bus[2];
        stall_w = (flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

//csr 读口地址：就是本级【组合输入】里那条指令的 csr 号（即 csr_addr 的下一条流水位置），
//非 SYSTEM 清 0 —— 与 csr_addr 的语义完全一致，读口不必再自己挡非法地址。
//csr 的读值要提前一拍（见 csr.v 的读口注释），所以这条地址必须取组合版、不能取 csr_addr。
    always @(*) begin
        csr_addr_pre = 12'd0;
        if (opcode == OPCODE_SYSTEM) csr_addr_pre = imm12_csr_in[11:0];
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
                aux_addr_out <= 32'd0;
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
                        csr_addr <= imm12_csr_in[11:0];
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
            end
        end
    end

endmodule
