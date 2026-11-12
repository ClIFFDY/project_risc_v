`timescale 1ns / 1ps
// npu_top — CPU 控制平面
//
// 本模块把上游（ahb_lite_slave）给出的原生请求拆成三个方向：
//   1. 控制寄存器：保存 M/N/K、TK、量化参数和 accumulate；TM/TN 固定为 4；
//   2. 状态寄存器：返回 BUSY、DONE、BUFFER_READY 和 ERROR；
//   3. Buffer 端口：把 0x1000/0x2000/0x3000 地址窗口转换成 RAM 操作。
// 地址译码结果（sel_*）和读数据（control_rdata/status_rdata/buffer_rdata）
// 回送给 ahb_lite_slave，由它组织 AHB 的数据阶段和错误响应。
//
// 请求接口（req_addr/req_wdata/req_byte_en/req_we/req_valid）与已删除的
// mmio_if 输出定义完全一致：req_valid 是数据阶段单拍脉冲，req_addr 与它同拍
// 有效；寄存器读数据同拍组合有效，Buffer 读数据在下一拍返回（见 npu_ram）。
//
// 状态寄存器 0x20 的位定义：
//   bit0 BUSY          正在执行任务
//   bit1 DONE          任务完成，可读 C Buffer
//   bit2 保留，固定 0
//   bit3 BUFFER_READY  CPU 可以访问 Buffer（等价于 !BUSY）
//   bit4 ERROR         上一次 start 的配置非法，任务未启动
// 控制寄存器 0x10 ACC_CFG.bit0=accumulate：跨任务累加。
//
// start_ctrl 在 start_req 到来时先校验配置、再锁存参数并产生 start_pulse；
// 配置非法时进入 ERROR 而不是 BUSY，运行期间软件再写配置寄存器不生效。
// 内含模块：control_regs / start_ctrl / buffer_access_ctrl
`include "npu_defines.vh"

module npu_top(
    input        clk,
    input        rst,
    // 来自 ahb_lite_slave 的原生请求（req_valid 在数据阶段单拍有效）
    input [31:0] req_addr,
    input [31:0] req_wdata,
    input [3:0]  req_byte_en,
    input        req_we,
    input        req_valid,
    // 地址译码结果与读数据回给 ahb_lite_slave
    output reg        sel_ctrl,
    output reg        sel_status,
    output reg        sel_a_buf,
    output reg        sel_bt_buf,
    output reg        sel_c_buf,
    output reg [31:0] control_rdata,
    output reg [31:0] status_rdata,
    output reg [31:0] buffer_rdata,
    // 到 tile_controller（启动脉冲 + 锁存后的任务参数）
    output reg        start_pulse,
    output reg [`NPU_DIM_W-1:0]  cfg_m, cfg_n, cfg_k,
    output reg [`NPU_TK_W-1:0]   cfg_tk,
    output reg [`NPU_QS_W-1:0]   cfg_qshift,
    output reg        cfg_accumulate,
    output reg        core_bank,      // 本任务读哪一组 A/BT（双缓冲）
    // 来自 tile_controller
    input        core_busy,
    input        core_done,
    // CPU 侧 RAM 端口（到 npu_buffer）
    output reg        cpu_a_we,  cpu_a_re,
    output reg        cpu_bt_we, cpu_bt_re,
    output reg        cpu_c_we,  cpu_c_re,
    output reg [`NPU_ABUF_REG_AW-1:0] cpu_a_addr,
    output reg [`NPU_BBUF_REG_AW-1:0] cpu_bt_addr,
    output reg [`NPU_CBUF_AW-1:0] cpu_c_addr,
    output reg [31:0] cpu_a_wdata, cpu_bt_wdata, cpu_c_wdata,
    input [31:0] a_rdata_cpu, bt_rdata_cpu, c_rdata_cpu
);

    // ---- 内部连线：译码出的本地字地址 ----
    reg [`NPU_ABUF_REG_AW-1:0] a_local_addr;
    reg [`NPU_BBUF_REG_AW-1:0] bt_local_addr;
    reg [`NPU_CBUF_AW-1:0] c_local_addr;
    reg [2:0]  ctrl_reg_off;

    wire [31:0] buffer_rdata_w;
    // control_regs 的输出先接内部 wire，再在 always @(*) 里转发到端口
    wire [31:0] control_rdata_w;

    // start_ctrl 的锁存参数：任务开始后由 m_l..qs_l 保持到任务结束。
    wire [`NPU_DIM_W-1:0]  m_l, n_l, k_l;
    wire [`NPU_TK_W-1:0]   tk_l;
    wire [`NPU_QS_W-1:0]   qs_l;
    wire        accumulate_l;
    wire        done_status, error_status;
    wire        start_req, clear_done_req;

    // buffer_access_ctrl 直接输出经过字节屏蔽的 RAM 端口。
    wire        start_pulse_w;
    wire        cpu_a_we_w, cpu_a_re_w, cpu_bt_we_w, cpu_bt_re_w;
    wire        cpu_c_we_w, cpu_c_re_w;
    wire [`NPU_ABUF_REG_AW-1:0] cpu_a_addr_w;
    wire [`NPU_BBUF_REG_AW-1:0] cpu_bt_addr_w;
    wire [`NPU_CBUF_AW-1:0] cpu_c_addr_w;
    wire [31:0] cpu_a_wdata_w, cpu_bt_wdata_w, cpu_c_wdata_w;

    // ---- MMIO 地址译码：req_addr 由 ahb_lite_slave 在数据阶段给出 ----
    always @(*) begin
        sel_ctrl    = (req_addr[31:5] == 27'd0);            // 0x0000-0x001F
        sel_status  = (req_addr[31:4] == 28'h0000002);      // 0x0020-0x002F
        // ★ 双缓冲：bank0 在 0x1000/0x2000，bank1 在 0x1800/0x2800，
        //   页内偏移仍是 [7:2]，bit[11] 选 bank（0x1400 这种中间页不认，保持原有空洞语义）。
        sel_a_buf   = (req_addr[31:12] == 20'h00001) && (req_addr[10:8] == 3'd0);
        sel_bt_buf  = (req_addr[31:12] == 20'h00002) && (req_addr[10:8] == 3'd0);
        sel_c_buf   = (req_addr[31:10] == 22'h00000C);      // 0x3000-0x33FF
        a_local_addr  = {req_addr[11], req_addr[7:2]};
        bt_local_addr = {req_addr[11], req_addr[7:2]};
        c_local_addr  = req_addr[9:2];
        ctrl_reg_off  = req_addr[4:2];
    end

    // control_regs 的实时配置值，仅供 start_ctrl 在启动瞬间采样。
    wire [`NPU_DIM_W-1:0]  cr_m, cr_n, cr_k;
    wire [`NPU_TK_W-1:0]   cr_tk;
    wire [`NPU_QS_W-1:0]   cr_qs;
    wire                   cr_accumulate;
    wire                   cr_bank;

    // ---- control_regs ----
    control_regs u_control_regs(
        .clk(clk), .rst(rst),
        .req_valid(req_valid), .req_we(req_we), .sel_ctrl(sel_ctrl),
        .ctrl_reg_off(ctrl_reg_off), .req_wdata(req_wdata),
        .req_byte_en(req_byte_en),
        .cfg_m(cr_m), .cfg_n(cr_n), .cfg_k(cr_k),
        .cfg_tk(cr_tk), .cfg_qshift(cr_qs), .cfg_accumulate(cr_accumulate),
        .cfg_bank(cr_bank),
        .start_req(start_req), .clear_done_req(clear_done_req),
        .control_rdata(control_rdata_w)
    );

    // ---- 状态寄存器读数据 ----
    always @(*) begin
        status_rdata = (ctrl_reg_off == 3'd0) ?
            {27'd0, error_status, !core_busy, 1'b0, done_status, core_busy} :
            32'd0;
    end

    // ---- start_ctrl ----
    start_ctrl u_start_ctrl(
        .clk(clk), .rst(rst),
        .start_req(start_req), .clear_done_req(clear_done_req),
        .cfg_m(cr_m), .cfg_n(cr_n), .cfg_k(cr_k),
        .cfg_tk(cr_tk), .cfg_qshift(cr_qs), .cfg_accumulate(cr_accumulate),
        .core_done_i(core_done),
        .start_pulse(start_pulse_w),
        .m_l(m_l), .n_l(n_l), .k_l(k_l),
        .tk_l(tk_l), .qs_l(qs_l), .accumulate_l(accumulate_l),
        .done_status(done_status), .error_status(error_status)
    );

    // ---- buffer_access_ctrl ----
    buffer_access_ctrl u_buffer_access_ctrl(
        .req_valid(req_valid), .req_we(req_we), .req_byte_en(req_byte_en),
        .sel_a_buf(sel_a_buf), .sel_bt_buf(sel_bt_buf), .sel_c_buf(sel_c_buf),
        .a_local_addr(a_local_addr), .bt_local_addr(bt_local_addr),
        .c_local_addr(c_local_addr), .req_wdata(req_wdata),
        .core_busy(core_busy), .core_bank(core_bank),
        .cpu_a_we(cpu_a_we_w), .cpu_a_re(cpu_a_re_w),
        .cpu_bt_we(cpu_bt_we_w), .cpu_bt_re(cpu_bt_re_w),
        .cpu_c_we(cpu_c_we_w), .cpu_c_re(cpu_c_re_w),
        .cpu_a_addr(cpu_a_addr_w), .cpu_bt_addr(cpu_bt_addr_w), .cpu_c_addr(cpu_c_addr_w),
        .cpu_a_wdata(cpu_a_wdata_w), .cpu_bt_wdata(cpu_bt_wdata_w), .cpu_c_wdata(cpu_c_wdata_w),
        .a_rdata_cpu(a_rdata_cpu), .bt_rdata_cpu(bt_rdata_cpu),
        .c_rdata_cpu(c_rdata_cpu), .buffer_rdata(buffer_rdata_w)
    );

    // 双缓冲：bank 在 start_pulse 那一拍锁存，运行期间软件改 BANK 不影响当前任务；
    // 这样软件可以在核忙着算 bank b 的时候，往 bank ~b 里预装下一块的数据。
    always @(posedge clk) begin
        if (rst)
            core_bank <= 1'b0;
        else if (start_pulse_w)
            core_bank <= cr_bank;
    end

    always @(*) begin
        control_rdata   = control_rdata_w;
        buffer_rdata    = buffer_rdata_w;
        start_pulse     = start_pulse_w;
        // 对 tile_controller 输出锁存后的任务参数：运行期间软件改动不影响当前任务。
        cfg_m           = m_l;
        cfg_n           = n_l;
        cfg_k           = k_l;
        cfg_tk          = tk_l;
        cfg_qshift      = qs_l;
        cfg_accumulate  = accumulate_l;
        cpu_a_we        = cpu_a_we_w;
        cpu_a_re        = cpu_a_re_w;
        cpu_bt_we       = cpu_bt_we_w;
        cpu_bt_re       = cpu_bt_re_w;
        cpu_c_we        = cpu_c_we_w;
        cpu_c_re        = cpu_c_re_w;
        cpu_a_addr      = cpu_a_addr_w;
        cpu_bt_addr     = cpu_bt_addr_w;
        cpu_c_addr      = cpu_c_addr_w;
        cpu_a_wdata     = cpu_a_wdata_w;
        cpu_bt_wdata    = cpu_bt_wdata_w;
        cpu_c_wdata     = cpu_c_wdata_w;
    end

endmodule
