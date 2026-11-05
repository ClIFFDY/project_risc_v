// 执行级外封装（每 lane 一个）：bju / alu / mdu 收进一层，结果口在层内仲裁
//
// 为什么收：这三个单元本来就只跟"同一条 lane 的操作数、同一个结果口"打交道；
//   分开例化时 cpu_top 里有三套结果网各自出线，值表因此是 5 写口。收进 exu 之后，
//   每条 lane 对外只有【一个结果口】（div > mul > alu 抢，抢不到的进影子），
//   值表写口变成 3 个（lane0 / lane1 / LD），寄存器堆才谈得上按 bank 复制两份。
//
// 端口口径：一律按"语义名"引出，lane 号留在 cpu_top 的例化处（`.语义名(实际线名)`），
//   所以顶层两根 lane 的接线是同一份、逐行对称的。
//   不带 lane 语义的公共输入只有 clk / rst / flush_control_in / flush_index_in。
//
// 本层不做的事：
//   · 不做操作数旁路裁决——`source_a_data` / `r2_data` 是 idu2 已经算好的最终值；
//   · 不合并 bju 的判定位与 alu/mdu 的结果口——bju 只回报判定，不产生寄存器值；
//   · 不新增复位级——三个子单元各自的 rst_q 各打一拍，这里不再加第五份。
module exu (
    input  clk,
    input  rst,
    input  [2:0] flush_control_in,      // = flag_bus[7:5]：{异常, 中断, 改向}
    input  [3:0] flush_index_in,        // ROB 冲刷边界（bju 用）
//★ 2026-11-04 精确冲刷：按槽的击杀掩码（透传给 mdu：比分支老的乘除不杀）
    input [15:0] flush_kill_mask_in,

//--------------------------------------------------------------- 操作数与身份
    input        write_enable_in,
    input        jal_flag,
    input        jalr_flag,
    input        cs_wr_en,
    input  [4:0] destination_in,
    input  [3:0] index_in,
    input        generation_in,
    input  [3:0] alu_function4,
    input  [2:0] csr_function3,
    input [31:0] auxiliary_address_in,  // pc（alu 的 AUIPC / bju 的 pc+4 基准）
    input [31:0] source_a_data,
    input [31:0] r2_data,
    input [31:0] cs_data,
    input        bypass_load_a,
    input        bypass_load_b,
    input [31:0] load_data_forward,

//--------------------------------------------------------------- bju 专用输入
    input  [3:0] branch_unit_function4_in,
    input [31:0] beq_offset_in,
    input [31:0] jump_link_register_prediction_address_in,
    input [31:0] jump_link_target_in,
    input        branch_prediction_taken_in,
    input        advance_in,            // = issue_valid

//--------------------------------------------------------------- mdu 专用输入
    input        multiply_go,
    input  [9:0] multiply_function10,
    input        divide_go,
    input  [9:0] divide_function10,

//--------------------------------------------------------------- alu 结果
    (* max_fanout = 8 *) output [4:0]  alu_destination_out,
    (* max_fanout = 8 *) output [3:0]  alu_index_out,
    output        alu_generation_out,
    (* max_fanout = 8 *) output [31:0] alu_result,
    (* max_fanout = 8 *) output [31:0] alu_result_csr,
    (* max_fanout = 8 *) output [31:0] alu_result_early,
    output            alu_write_enable_early,
    output     [3:0]  alu_index_early,
    output            alu_generation_early,
    (* max_fanout = 8 *) output alu_write_enable,

//--------------------------------------------------------------- mdu 结果
    (* max_fanout = 8 *) output [31:0] multiply_data_out,
    output [31:0] multiply_data_early,
    output [31:0] divide_data_early,
    (* max_fanout = 8 *) output multiply_loaded,
    output        multiply_write_enable_early,
    output [3:0]  multiply_index_early,
    output        multiply_generation_early,
    output        divide_write_enable_early,
    output [3:0]  divide_index_early,
    output        divide_generation_early,
    (* max_fanout = 8 *) output [3:0] multiply_index_out,
    (* max_fanout = 8 *) output multiply_generation_out,
    (* max_fanout = 8 *) output [31:0] divide_data_out,
    (* max_fanout = 8 *) output divide_loaded,
    (* max_fanout = 8 *) output [3:0] divide_index_out,
    (* max_fanout = 8 *) output divide_generation_out,
    output multiply_ready,
    output divide_ready,

//--------------------------------------------------------------- bju 结果
    output        success,
    output        branch_failure,
    output        jump_link_register_failure,
    output        exception_target_misalign,
    output [31:0] jump_target,
    output [31:0] jump_link_register_target_registered,
    output [8:0]  branch_pc_index,
    (* max_fanout = 32 *) output [31:0] branch_pc_registered,
    output        jump_link_register_target_valid_registered,
    output        branch_prediction_taken_registered,
    output [3:0]  judged_index,
    output [3:0]  index_registered,
    output        judged_registered,
    output        redirect_registered,
    output        generation_registered,
    output [1:0]  branch_unit_kind,
    output        branch_unit_taken,
    output [31:0] branch_unit_pc,
    output [31:0] branch_unit_target,
    output [31:0] exception_pc_registered,
    output [31:0] exception_pc_judged,
    output        exception_ecall_judged,
    output        exception_ebreak_judged,
    output        exception_illegal_judged,
    output        exception_interrupt_return_judged,
    output        exception_ecall_registered,
    output        exception_ebreak_registered,
    output        exception_illegal_registered,
    output        exception_interrupt_return_registered,

//--------------------------------------------------------------- 收窄后的唯一结果口
    output        result_write_enable,
    output [3:0]  result_index,
    output        result_generation,
    output [31:0] result_data,
    output        result_overflow
);

//==============================================================================
// alu
//==============================================================================
    wire [31:0] alu_result_i, alu_result_csr_i, alu_result_early_i;
    wire [3:0]  alu_index_early_i;
    wire        alu_generation_early_i;
    wire [3:0]  alu_index_i;
    wire        alu_generation_i;
    wire        alu_write_enable_early_i;

    assign alu_result            = alu_result_i;
    assign alu_result_csr        = alu_result_csr_i;
    assign alu_result_early      = alu_result_early_i;
    assign alu_write_enable_early = alu_write_enable_early_i;
    assign alu_index_early       = alu_index_early_i;
    assign alu_generation_early  = alu_generation_early_i;

    alu u_alu (
        .clk(clk), .rst(rst),
        .flush_control_in(flush_control_in),
        .write_enable_in(write_enable_in), .jal_flag(jal_flag), .jalr_flag(jalr_flag),
        .cs_wr_en(cs_wr_en),
        .destination_in(destination_in), .index_in(index_in), .generation_in(generation_in),
        .alu_function4(alu_function4), .csr_function3(csr_function3),
        .auxiliary_address_in(auxiliary_address_in),
        .source_a_data(source_a_data), .r2_data(r2_data), .cs_data(cs_data),
        .bypass_load_a(bypass_load_a), .bypass_load_b(bypass_load_b), .load_data_forward(load_data_forward),
        .destination_out(alu_destination_out), .index_out(alu_index_i), .generation_out(alu_generation_i),
        .result(alu_result_i), .result_csr(alu_result_csr_i),
        .result_now(), .result_early(alu_result_early_i),
        .write_enable_early(alu_write_enable_early_i), .index_early(alu_index_early_i), .generation_early(alu_generation_early_i),
        .write_enable(alu_write_enable)
    );
    assign alu_index_out      = alu_index_i;
    assign alu_generation_out = alu_generation_i;

//==============================================================================
// mdu
//==============================================================================
    mdu u_mdu (
        .clk(clk), .rst(rst),
        .flush_control_in(flush_control_in),
        .flush_kill_mask_in(flush_kill_mask_in),
        .multiply_go(multiply_go), .multiply_function10(multiply_function10), .multiply_destination(destination_in),
        .multiply_index_in(index_in), .multiply_generation_in(generation_in),
        .multiply_source_a(source_a_data), .mul_r2(r2_data),
        .divide_go(divide_go), .divide_function10(divide_function10), .divide_destination(destination_in),
        .divide_index_in(index_in), .divide_generation_in(generation_in),
        .divide_source_a(source_a_data), .div_r2(r2_data),
        .multiply_data_out(multiply_data_out), .multiply_loaded(multiply_loaded), .multiply_write_enable(),
        .multiply_data_early(multiply_data_early), .divide_data_early(divide_data_early),
        .multiply_write_enable_early(multiply_write_enable_early), .multiply_index_early(multiply_index_early), .multiply_generation_early(multiply_generation_early),
        .divide_write_enable_early(divide_write_enable_early), .divide_index_early(divide_index_early), .divide_generation_early(divide_generation_early),
        .rd_mul(), .multiply_index(multiply_index_out), .multiply_generation(multiply_generation_out),
        .divide_data_out(divide_data_out), .divide_loaded(divide_loaded), .divide_write_enable(),
        .rd_div(), .divide_index(divide_index_out), .divide_generation(divide_generation_out),
        .multiply_ready(multiply_ready), .divide_ready(divide_ready),
        .multiply_wake_registered(), .multiply_wake_index_registered(), .multiply_wake_generation_registered(),
        .multiply_nack_registered(), .multiply_nack_index_registered(), .multiply_nack_generation_registered()
    );

//==============================================================================
// bju
//==============================================================================
    bju u_bju (
        .clk(clk), .rst(rst),
        .flush_control_in(flush_control_in),
        .source_a_data_in(source_a_data), .source_b_data_in(r2_data),
        .branch_unit_function4_in(branch_unit_function4_in),
        .pc_plus4_in(auxiliary_address_in), .br_off_in(beq_offset_in),
        .jalr_pred_tgt_in(jump_link_register_prediction_address_in),
        .branch_prediction_taken_in(branch_prediction_taken_in),
        .jal_tgt_in(jump_link_target_in),
        .advance_in(advance_in),
        .success(success), .branch_failure(branch_failure), .jump_link_register_failure(jump_link_register_failure),
        .index_in(index_in), .generation_in(generation_in),
        .index_registered(index_registered), .judged_registered(judged_registered),
        .generation_registered(generation_registered), .redirect_registered(redirect_registered),
        .jump_target(jump_target), .jump_link_register_target_registered(jump_link_register_target_registered),
        .branch_pc_index(branch_pc_index), .branch_pc_registered(branch_pc_registered),
        .branch_unit_kind(branch_unit_kind), .branch_unit_taken(branch_unit_taken),
        .branch_unit_pc(branch_unit_pc), .branch_unit_target(branch_unit_target),
        .exception_target_misalign(exception_target_misalign),
        .jump_link_register_target_valid_registered(jump_link_register_target_valid_registered),
        .branch_prediction_taken_registered(branch_prediction_taken_registered),
        .exception_branch_unit(), .judged_index(judged_index), .exception_pc_registered(exception_pc_registered),
        .exception_ecall_judged(exception_ecall_judged), .exception_ebreak_judged(exception_ebreak_judged),
        .exception_illegal_judged(exception_illegal_judged), .exception_interrupt_return_judged(exception_interrupt_return_judged),
        .exception_ecall_registered(exception_ecall_registered), .exception_ebreak_registered(exception_ebreak_registered),
        .exception_illegal_registered(exception_illegal_registered), .exception_interrupt_return_registered(exception_interrupt_return_registered),
        .exception_pc_judged(exception_pc_judged),
        .flush_index_in(flush_index_in)
    );

//==============================================================================
// 结果口仲裁：div > mul > alu，抢不到的进本类影子
//
// 为什么要收：这三路今天各自独立写值表、各自唤醒、各自报 ROB。值表因此是 5 写口，
//   FPGA 上每个阵列最多 2 个写口 ⇒ 要复制三份。收成"每 lane 一个口"之后写口是 3
//   （lane0 / lane1 / LD），寄存器堆才谈得上按 bank 复制两份。
//★ 本拍来的结果只要没被更高优先级抢走，就【当拍穿过去】，不进影子
//  （写成"先入队再出队"会把每个 ALU 结果都推后一拍 ⇒ CoreMark +8.8%，已改）。
//
// 影子深度每类 2 就够（单发射 lane）：
//   · div：迭代式，同时最多一条在飞，且优先级最高 ⇒ 挂不住；
//   · mul：只有"本拍在服务 div"才会挂，连挂 2 拍要求连续两个 div 结果 ⇒ 不会发生；
//   · alu：只有"本拍在服务 div 或 mul"才会挂；若 mul 每拍都来，说明这条 lane 连发 MUL、
//     根本没有 ALU 指令 ⇒ 不会再有 alu 到达。
//   ⇒ 每类 2 格足够；超了是设计违约，仿真里 result_overflow 会拉高。
//==============================================================================
    reg [1:0]  divide_count, multiply_count, alu_count;
    reg [3:0]  divide_index_q   [0:1];
    reg        divide_gen_q     [0:1];
    reg [31:0] divide_data_q    [0:1];
    reg [3:0]  multiply_index_q [0:1];
    reg        multiply_gen_q   [0:1];
    reg [31:0] multiply_data_q  [0:1];
    reg [3:0]  alu_index_q      [0:1];
    reg        alu_gen_q        [0:1];
    reg [31:0] alu_data_q       [0:1];

    reg        result_port_write_enable;
    reg [3:0]  result_port_index;
    reg        result_port_generation;
    reg [31:0] result_port_data;
    reg        result_port_overflow;

    integer result_index_i;

    assign result_write_enable = result_port_write_enable;
    assign result_index        = result_port_index;
    assign result_generation   = result_port_generation;
    assign result_data         = result_port_data;
    assign result_overflow     = result_port_overflow;

//---- 谁在抢：本类有存货，或者本拍来了新结果 ----
    wire divide_available   = (divide_count   != 2'd0) | divide_write_enable_early;
    wire multiply_available = (multiply_count != 2'd0) | multiply_write_enable_early;
    wire alu_available      = (alu_count      != 2'd0) | alu_write_enable_early_i;

    wire serve_divide   = divide_available;
    wire serve_multiply = ~divide_available & multiply_available;
    wire serve_alu      = ~divide_available & ~multiply_available & alu_available;

    wire divide_from_shadow   = (divide_count   != 2'd0);
    wire multiply_from_shadow = (multiply_count != 2'd0);
    wire alu_from_shadow      = (alu_count      != 2'd0);

    wire divide_pop    = serve_divide    & divide_from_shadow;
    wire multiply_pop  = serve_multiply  & multiply_from_shadow;
    wire alu_pop       = serve_alu       & alu_from_shadow;
    wire divide_push   = divide_write_enable_early   & ~(serve_divide   & ~divide_from_shadow);
    wire multiply_push = multiply_write_enable_early & ~(serve_multiply & ~multiply_from_shadow);
    wire alu_push      = alu_write_enable_early_i    & ~(serve_alu      & ~alu_from_shadow);

    always @(*) begin
        result_port_write_enable = divide_available | multiply_available | alu_available;
        result_port_index        = serve_divide   ? (divide_from_shadow   ? divide_index_q[0]   : divide_index_early)   :
                                   serve_multiply ? (multiply_from_shadow ? multiply_index_q[0] : multiply_index_early) :
                                                    (alu_from_shadow      ? alu_index_q[0]      : alu_index_early_i);
        result_port_generation   = serve_divide   ? (divide_from_shadow   ? divide_gen_q[0]     : divide_generation_early)   :
                                   serve_multiply ? (multiply_from_shadow ? multiply_gen_q[0]   : multiply_generation_early) :
                                                    (alu_from_shadow      ? alu_gen_q[0]        : alu_generation_early_i);
        result_port_data         = serve_divide   ? (divide_from_shadow   ? divide_data_q[0]    : divide_data_early)    :
                                   serve_multiply ? (multiply_from_shadow ? multiply_data_q[0]  : multiply_data_early)  :
                                                    (alu_from_shadow      ? alu_data_q[0]       : alu_result_early_i);
    end

    always @(posedge clk) begin
        if (rst) begin
            divide_count   <= 2'd0;
            multiply_count <= 2'd0;
            alu_count      <= 2'd0;
            result_port_overflow <= 1'b0;
            for (result_index_i = 0; result_index_i < 2; result_index_i = result_index_i + 1) begin
                divide_index_q[result_index_i]   <= 4'd0;  divide_gen_q[result_index_i]   <= 1'b0;  divide_data_q[result_index_i]   <= 32'd0;
                multiply_index_q[result_index_i] <= 4'd0;  multiply_gen_q[result_index_i] <= 1'b0;  multiply_data_q[result_index_i] <= 32'd0;
                alu_index_q[result_index_i]      <= 4'd0;  alu_gen_q[result_index_i]      <= 1'b0;  alu_data_q[result_index_i]      <= 32'd0;
            end
        end
        else begin
            result_port_overflow <= 1'b0;

            if (divide_pop) begin
                divide_index_q[0] <= divide_index_q[1];
                divide_gen_q[0]   <= divide_gen_q[1];
                divide_data_q[0]  <= divide_data_q[1];
            end
            if (divide_push) begin
                if ((divide_count - {1'b0, divide_pop}) == 2'd0) begin
                    divide_index_q[0] <= divide_index_early;
                    divide_gen_q[0]   <= divide_generation_early;
                    divide_data_q[0]  <= divide_data_early;
                end
                else begin
                    divide_index_q[1] <= divide_index_early;
                    divide_gen_q[1]   <= divide_generation_early;
                    divide_data_q[1]  <= divide_data_early;
                end
                if ((divide_count - {1'b0, divide_pop}) >= 2'd2) result_port_overflow <= 1'b1;
            end
            divide_count <= divide_count + {1'b0, divide_push} - {1'b0, divide_pop};

            if (multiply_pop) begin
                multiply_index_q[0] <= multiply_index_q[1];
                multiply_gen_q[0]   <= multiply_gen_q[1];
                multiply_data_q[0]  <= multiply_data_q[1];
            end
            if (multiply_push) begin
                if ((multiply_count - {1'b0, multiply_pop}) == 2'd0) begin
                    multiply_index_q[0] <= multiply_index_early;
                    multiply_gen_q[0]   <= multiply_generation_early;
                    multiply_data_q[0]  <= multiply_data_early;
                end
                else begin
                    multiply_index_q[1] <= multiply_index_early;
                    multiply_gen_q[1]   <= multiply_generation_early;
                    multiply_data_q[1]  <= multiply_data_early;
                end
                if ((multiply_count - {1'b0, multiply_pop}) >= 2'd2) result_port_overflow <= 1'b1;
            end
            multiply_count <= multiply_count + {1'b0, multiply_push} - {1'b0, multiply_pop};

            if (alu_pop) begin
                alu_index_q[0] <= alu_index_q[1];
                alu_gen_q[0]   <= alu_gen_q[1];
                alu_data_q[0]  <= alu_data_q[1];
            end
            if (alu_push) begin
                if ((alu_count - {1'b0, alu_pop}) == 2'd0) begin
                    alu_index_q[0] <= alu_index_early_i;
                    alu_gen_q[0]   <= alu_generation_early_i;
                    alu_data_q[0]  <= alu_result_early_i;
                end
                else begin
                    alu_index_q[1] <= alu_index_early_i;
                    alu_gen_q[1]   <= alu_generation_early_i;
                    alu_data_q[1]  <= alu_result_early_i;
                end
                if ((alu_count - {1'b0, alu_pop}) >= 2'd2) result_port_overflow <= 1'b1;
            end
            alu_count <= alu_count + {1'b0, alu_push} - {1'b0, alu_pop};
        end
    end

endmodule
