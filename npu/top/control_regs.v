`timescale 1ns / 1ps
// control_regs — 保存软件配置并产生 start / clear_done 命令脉冲
//
// 控制寄存器地址（偏移 = addr[4:2]）：
//   0x00 CTRL    bit0=1 启动一次任务；bit1=1 清除 DONE/ERROR 状态
//   0x04 M       矩阵行数 1..63
//   0x08 N       矩阵列数 1..63
//   0x0C K       内积长度 1..63
//   0x10 ACC_CFG bit0 跨任务累加：1 时本次任务第一个输出 Tile 不清 PE 累加器
//   0x14 BANK    bit0 本次任务读哪一组 A/BT（双缓冲；另一组可同时被写）
//   0x18 TK      K 方向 Tile 大小 1..16
//   0x1C QUANT   量化右移位数
//
// CTRL 是命令寄存器，不保存 bit0/bit1；例如写 0x02 只产生 clear_done_req，
// 不会启动运算，回读 CTRL 时 bit0/bit1 始终为 0。
// 本模块只负责保存与回读，配置是否合法由 start_ctrl 在启动瞬间判断。
`include "npu_defines.vh"

module control_regs(
    input        clk,
    input        rst,
    input        req_valid,
    input        req_we,
    input        sel_ctrl,
    input [2:0]  ctrl_reg_off,
    input [31:0] req_wdata,
    input [3:0]  req_byte_en,
    output reg  [`NPU_DIM_W-1:0]  cfg_m,
    output reg  [`NPU_DIM_W-1:0]  cfg_n,
    output reg  [`NPU_DIM_W-1:0]  cfg_k,
    output reg  [`NPU_TK_W-1:0]   cfg_tk,
    output reg  [`NPU_QS_W-1:0]   cfg_qshift,
    output reg        cfg_accumulate,
    output reg        cfg_bank,
    output reg        start_req,        // 单拍
    output reg        clear_done_req,   // 单拍
    output reg [31:0] control_rdata
);

    // req_valid 只在 ahb_lite_slave 发布一笔已锁存的请求时有效一个周期。
    // 配置寄存器均位于一个 32 位字的低字节，只接受 byte_en[0] 有效的写入，
    // 这样字节写和整字写行为一致，也不会误改高位保留字段。
    wire wr       = req_valid && sel_ctrl && req_we;
    wire byte0_wr = wr && req_byte_en[0];

    // 两个命令脉冲：只有写 0x00 CTRL 且低字节有效时才产生。
    // 若同时写 CTRL=0x03，两个命令都会产生；start_ctrl 只在对应状态接受命令。
    always @(*) begin
        start_req      = byte0_wr && (ctrl_reg_off == 3'd0) && req_wdata[0];
        clear_done_req = byte0_wr && (ctrl_reg_off == 3'd0) && req_wdata[1];
    end

    always @(posedge clk) begin
        if (rst) begin
            cfg_m          <= 6'd4;
            cfg_n          <= 6'd4;
            cfg_k          <= 6'd4;
            cfg_tk         <= 5'd4;
            cfg_qshift     <= 5'd0;
            cfg_accumulate <= 1'b0;
            cfg_bank       <= 1'b0;
        end else if (byte0_wr) begin
            case (ctrl_reg_off)
                3'd1: cfg_m          <= req_wdata[`NPU_DIM_W-1:0];
                3'd2: cfg_n          <= req_wdata[`NPU_DIM_W-1:0];
                3'd3: cfg_k          <= req_wdata[`NPU_DIM_W-1:0];
                3'd4: cfg_accumulate <= req_wdata[0];
                3'd5: cfg_bank       <= req_wdata[0];
                3'd6: cfg_tk         <= req_wdata[`NPU_TK_W-1:0];
                3'd7: cfg_qshift     <= req_wdata[`NPU_QS_W-1:0];
                default: ;          // 0x00 CTRL 只有命令位，不保存
            endcase
        end
    end

    // 寄存器回读（字读）。CTRL 本身是命令寄存器，所以读 0x00 得到 0。
    always @(*) begin
        control_rdata =
            (ctrl_reg_off == 3'd0) ? {30'd0, 2'b00}                        :
            (ctrl_reg_off == 3'd1) ? {26'd0, cfg_m}                        :
            (ctrl_reg_off == 3'd2) ? {26'd0, cfg_n}                        :
            (ctrl_reg_off == 3'd3) ? {26'd0, cfg_k}                        :
            (ctrl_reg_off == 3'd4) ? {31'd0, cfg_accumulate}               :
            (ctrl_reg_off == 3'd5) ? {31'd0, cfg_bank}                     :
            (ctrl_reg_off == 3'd6) ? {27'd0, cfg_tk}                       :
            (ctrl_reg_off == 3'd7) ? {27'd0, cfg_qshift}                   :
                                     32'd0                                 ;
    end

endmodule
