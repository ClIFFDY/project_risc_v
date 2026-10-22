`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: fe_top
// Description: 前端独立顶层 —— 只把 pc / bpu / icache / iffu 这四个模块接起来，
//              外加一个行为级的取指后备存储器（fe_mem，协议与 itcm 逐拍一致）。
//
//              存在的目的：后端（idu1/idu2/rob）还没改完，前端要能单独仿真。
//              后端边界在这里被显式化成端口：重定向入口（redir_go/redir_target/
//              flush_pc_redir）、EX 期的分支回报（success/br_fail/br_pc_idx/
//              br_pred_taken_in/jalr_tgt_vld/jalr_tgt_q）、读侧推进（advance）。
//              单测时这几组由 tb 假扮；接真正后端时原样替换成 cont/bju 的输出。
//
//              接线口径：
//                pc_addr0/1  = 本拍向 icache 请求的那一对地址
//                inst_addr0/1 = 与指令同拍交付的地址（每条指令自带自己的地址）
//                push_addr_0/1 取 inst_addr0/1（icache 交付时）或 pc_addr0/1（bti 注入时，
//                              那时 pc 已跳到目标上，pc_addr0 就是目标那条的地址）
//////////////////////////////////////////////////////////////////////////////////


module fe_top(
    input clk, rst,
//控制总线（本厂口径：调用者把 8 位原样送进来，各模块自己按名字译码）
    input [7:0]  flag_bus,
//重定向入口（接 cont）
    input        redir_go,
    input [31:0] redir_target,
    input [31:0] jp_tgt,
    input        flush_pc_redir,
//读侧推进（接 idu1 的 advance）
    input [1:0]  advance,
//EX 期回报（接 bju）：前端单测时由 tb 假扮
    input [31:0] jalr_tgt_q,
    input [8:0]  br_pc_idx,
    input        success, br_fail, jalr_tgt_vld, br_pred_taken_in,
//两条 lane 的交付地址（tb 观测用）
    output [31:0] pc_addr0, pc_addr1,
//出队格（与将来 idu1 / rob 的入队口同一份数据）
    output [31:0] h0_inst, h0_addr, h0_jalr_pred,
    output        h0_br_pred,
    output [31:0] h1_inst, h1_addr, h1_jalr_pred,
    output        h1_br_pred,
//状态
    output        full, busy
    );

//--------------------------------------------------------------------------
// 模块间互连
//--------------------------------------------------------------------------
//bpu → pc / iffu
    wire        br1_0, br1_1, br2, br3, jalr_0, jalr_1;
    wire        bti_hit_0, bti_hit_1;
    wire [31:0] jalr_pred_0, jalr_pred_1;
    wire [63:0] bti_inst_0, bti_inst_1;
    wire [1:0]  bti_sel_0, bti_sel_1;
//iffu → pc / bpu / icache
    wire [31:0] fch_off_beq_0, fch_off_jal_0, fch_off_beq_1, fch_off_jal_1;
    wire        fch_br_en_0, fch_jal_0, fch_jalr_0;
    wire        fch_br_en_1, fch_jal_1, fch_jalr_1;
//icache → iffu / bpu
    wire [31:0] inst_out, inst_next, inst_addr0, inst_addr1;
    wire        inst_valid, inst_valid_n;
//icache → pc：缺失检测拍（组合直连，不进 flag_bus）
    wire        icache_miss;
//icache ↔ 后备存储器
    wire        mem_req, mem_we, mem_valid;
    wire [31:0] mem_addr, mem_wdata, mem_data;
    wire [3:0]  mem_be;

//--------------------------------------------------------------------------
// 交付口上这条指令自己的地址
//   icache 交付时：就是它随指令一起给出的 inst_addr（每条指令自带自己的地址）；
//   bti 注入那一拍：icache 没在交付，注入的那一对是 (Tg, Tg+4)，而 pc 命中时已经跳过 8
//                   落到 Tg+8 ⇒ 取 pc − 8。
// ★ 取指地址与交付口之间的拍差只在这里换算一次：pc / bpu / iffu 内部都不做这种偏置，
//   bpu 的所有表都用这根地址索引（写口吃的是指令自己的地址，读口必须也是）。
//--------------------------------------------------------------------------
    reg [31:0] cur_addr0, cur_addr1;
    always @(*) begin
        if ((bti_sel_0 | bti_sel_1) != 2'd0) begin
            cur_addr0 = pc_addr0 - 32'd8;
            cur_addr1 = pc_addr1 - 32'd8;
        end
        else begin
            cur_addr0 = inst_addr0;
            cur_addr1 = inst_addr1;
        end
    end

//--------------------------------------------------------------------------
// 取指地址（两条 lane 一对）
//--------------------------------------------------------------------------
    pc u_pc (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .fifo_full(full),
        .icache_miss(icache_miss),
        .cur_addr_0(cur_addr0), .cur_addr_1(cur_addr1),
        .br1_l0(br1_0),           .br1_l1(br1_1),
        .jal_l0(fch_jal_0),       .jal_l1(fch_jal_1),
        .pre_jalr_l0(fch_jalr_0), .pre_jalr_l1(fch_jalr_1),
//btb 命中位就是 bpu 的 btb 有效位（旧顶层里 btb_hit 就是 bpu.jalr 这根）
        .btb_hit_l0(jalr_0),      .btb_hit_l1(jalr_1),
        .bti_hit_l0(bti_hit_0),   .bti_hit_l1(bti_hit_1),
        .offset_beq_pre_l0(fch_off_beq_0), .offset_jal_pre_l0(fch_off_jal_0),
        .offset_jalr_pre_l0(jalr_pred_0),
        .offset_beq_pre_l1(fch_off_beq_1), .offset_jal_pre_l1(fch_off_jal_1),
        .offset_jalr_pre_l1(jalr_pred_1),
        .jp_tgt(jp_tgt),
        .redir_go(redir_go),
        .redir_target(redir_target),
        .pc_addr0(pc_addr0),
        .pc_addr1(pc_addr1)
    );

//--------------------------------------------------------------------------
// 预测器：两条 lane 完全对称，本模块内不仲裁
//--------------------------------------------------------------------------
    bpu u_bpu (
        .clk(clk),
        .rst(rst),
        .cur_addr_0(cur_addr0), .cur_addr_1(cur_addr1),
        .jalr_tgt_q(jalr_tgt_q),
        .br_pc_idx(br_pc_idx),
        .success(success), .br_fail(br_fail),
        .jalr_tgt_vld(jalr_tgt_vld), .br_pred_taken_in(br_pred_taken_in),
        .br_en_0(fch_br_en_0), .br_en_1(fch_br_en_1),
        .fch_jal_0(fch_jal_0), .fch_jal_1(fch_jal_1),
        .fch_jalr_0(fch_jalr_0),
        .inst_in_0(inst_out), .inst_in_1(inst_next),
        .inst_valid_0(inst_valid), .inst_valid_1(inst_valid_n),
        .flag_bus(flag_bus),
        .flush_pc_redir(flush_pc_redir),
//前端单测没有保留站：这一位恒 0
        .stall_rs_full(1'b0),
        .fifo_full(full),
        .jalr_pred_offset_0(jalr_pred_0), .jalr_pred_offset_1(jalr_pred_1),
        .br1_0(br1_0), .br1_1(br1_1),
        .br2(br2), .br3(br3),
        .jalr_0(jalr_0), .jalr_1(jalr_1),
        .bti_hit_0(bti_hit_0), .bti_hit_1(bti_hit_1),
        .bti_inst_0(bti_inst_0), .bti_inst_1(bti_inst_1),
        .bti_sel_0(bti_sel_0), .bti_sel_1(bti_sel_1)
    );

//--------------------------------------------------------------------------
// 取指源：icache（含自举） + 后备存储器
//--------------------------------------------------------------------------
    icache u_icache (
        .clk(clk),
        .rst(rst),
        .pc_addr0(pc_addr0), .pc_addr1(pc_addr1),
        .br2(br2), .br3(br3),
//预测成立的那拍把交付口浇成 NOP：本拍读出来的是跳转后的顺序指令，是错路
        .pre_jalr((fch_jalr_0 & jalr_0) | (fch_jalr_1 & jalr_1)),
        .btb_hit(1'b1),
        .jalr_fail(1'b0),
        .jal_in(fch_jal_0 | fch_jal_1),
        .br1_in(br1_0 | br1_1),
        .exc_irq(1'b0), .exc_irq_ret(1'b0), .exc_ecall(1'b0),
        .flag_bus(flag_bus),
        .mem_valid(mem_valid), .mem_data(mem_data),
        .flush_pc_redir(flush_pc_redir),
        .fifo_full(full),
        .icache_miss(icache_miss),
        .inst_out(inst_out), .inst_next(inst_next),
        .inst_addr0(inst_addr0), .inst_addr1(inst_addr1),
        .inst_valid(inst_valid), .inst_valid_n(inst_valid_n),
        .busy(busy),
        .mem_req(mem_req), .mem_we(mem_we),
        .mem_addr(mem_addr), .mem_wdata(mem_wdata), .mem_be(mem_be)
    );

    fe_mem u_mem (
        .clk(clk), .rst(rst),
        .mem_req(mem_req), .mem_addr(mem_addr),
        .mem_data(mem_data), .mem_valid(mem_valid)
    );

//--------------------------------------------------------------------------
// 取指队列
//--------------------------------------------------------------------------
    iffu u_iffu (
        .clk(clk),
        .rst(rst),
        .flag_bus(flag_bus),
        .flush_pc_redir(flush_pc_redir),
        .inst_in_0(inst_out), .inst_in_1(inst_next),
        .inst_valid_0(inst_valid), .inst_valid_1(inst_valid_n),
        .bti_inst_0(bti_inst_0), .bti_inst_1(bti_inst_1),
        .bti_sel_0(bti_sel_0), .bti_sel_1(bti_sel_1),
//lane0 自己的 jalr 预测命中位（iffu 只用它判 lane0 改不改向）
        .btb_hit(jalr_0),
//指令自带地址：取上面那一根（icache 交付 / bti 注入 都已经换算好）
        .push_addr_0(cur_addr0),
        .push_addr_1(cur_addr1),
        .push_br_pred_0(br1_0), .push_br_pred_1(br1_1),
        .push_jalr_pred_0(jalr_pred_0), .push_jalr_pred_1(jalr_pred_1),
        .advance(advance),
        .h0_inst(h0_inst), .h0_addr(h0_addr), .h0_jalr_pred(h0_jalr_pred),
        .h0_br_pred(h0_br_pred),
        .h0_r1(), .h0_r2(),
        .h1_inst(h1_inst), .h1_addr(h1_addr), .h1_jalr_pred(h1_jalr_pred),
        .h1_br_pred(h1_br_pred),
        .h1_r1(), .h1_r2(),
        .fch_off_beq_0(fch_off_beq_0), .fch_off_jal_0(fch_off_jal_0),
        .fch_off_beq_1(fch_off_beq_1), .fch_off_jal_1(fch_off_jal_1),
        .fch_br_en_0(fch_br_en_0), .fch_jal_0(fch_jal_0), .fch_jalr_0(fch_jalr_0),
        .fch_br_en_1(fch_br_en_1), .fch_jal_1(fch_jal_1), .fch_jalr_1(fch_jalr_1),
        .full(full)
    );

endmodule


//////////////////////////////////////////////////////////////////////////////////
// fe_mem：取指后备存储器的行为级替身。
// 协议与 itcm 逐拍一致（逐行 32 字、mem_valid 比地址晚一拍），这样 icache 的
// 自举与回填状态机在仿真里的相位和上板一致。
// 内容从 tb 侧给的 hex 读入；只读，不回写。
//////////////////////////////////////////////////////////////////////////////////
module fe_mem(
    input clk, rst,
    input mem_req,
    input [31:0] mem_addr,
    output reg [31:0] mem_data,
    output reg mem_valid
    );

    reg [31:0] mem [0:16383];

    reg [13:0] raddr;
    reg [5:0]  cnt;
    reg        addr_v;
    reg        req_d;
    reg        start;
    reg [31:0] d_r;
    reg        v_d;
    integer i;

    initial begin
        for (i = 0; i < 16384; i = i + 1)
            mem[i] = 32'd0;
        $readmemh("E:/Vivado_Projects/project_risc_v/RTL1/sim/fe.hex", mem);
    end

    always @(*) start = mem_req & ~req_d;

    always @(posedge clk) begin
        if (rst) begin
            raddr   <= 14'd0;
            cnt     <= 6'd0;
            addr_v  <= 1'b0;
            req_d   <= 1'b0;
        end
        else begin
            req_d <= mem_req;
            if (start) begin
                raddr  <= mem_addr[15:2];
                cnt    <= 6'd32;
                addr_v <= 1'b1;
            end
            else if (cnt != 6'd0) begin
                raddr  <= raddr + 14'd1;
                cnt    <= cnt - 6'd1;
                addr_v <= 1'b1;
            end
            else
                addr_v <= 1'b0;
        end
    end

    always @(posedge clk) begin
        d_r <= mem[raddr];
        v_d <= addr_v;
    end

    always @(*) begin
        mem_data  = d_r;
        mem_valid = v_d;
    end

endmodule
