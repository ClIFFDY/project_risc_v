// ============================================================================
//  npu_common.v —— 由 common/ 目录下的文件合并而成（2026-11-17，扁平化）
//  源文件：npu_ram.v npu_stream_ctrl.v npu_sync_fifo.v 
//  合并方式：原样拼接，模块名/端口一个字没改。
// ============================================================================

// --------------------------------------------------------------- npu_ram.v

`timescale 1ns / 1ps
// npu_ram — 通用单时钟 RAM 原语
//
// 接口约定：
//   we=1 时，在当前 posedge 将 wdata 写入 waddr；
//   re=1 时，raddr 在当前 posedge 被采样，rdata 在该沿之后更新。
// 当前实现每拍都执行读端口赋值，因此即使 re=0，rdata 也会反映 raddr
// 对应的存储单元。上层用 re 产生访问请求，用 rdata 的一拍延迟匹配同步 RAM。
`include "npu_defines.vh"

module npu_ram #(parameter AW = 6) (
    input        clk,
    input        we,
    input [AW-1:0] waddr,
    input [31:0] wdata,
    input        re,
    input [AW-1:0] raddr,
    output reg  [31:0] rdata
);

    // 深度为 2^AW，每个地址保存一个 32 位字。
    reg [31:0] mem [0:(1<<AW)-1];
    integer i;

    // re 由各上层模块保留为端口协议信号；本行为模型采用同步异步混合读法，
    // 每拍都把 raddr 对应的内容送入 rdata，真正的请求门控由上层完成。

    initial begin
        for (i = 0; i < (1<<AW); i = i + 1)
            mem[i] = 32'd0;
        rdata = 32'd0;
    end

    // 非阻塞赋值保证写入和读出都在同一个时钟沿更新，符合同步 RAM 行为。
    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

endmodule

// --------------------------------------------------------------- npu_stream_ctrl.v

`timescale 1ns / 1ps
// npu_stream_ctrl — 连续地址读突发控制器
//
// 突发长度用 NPU_DIM_W(6 位)：上层对 A/BT 的预取是"一个输出 Tile 的整段连续数据"，
// 长度等于 K（最大 63），不再按 K Tile 切成小段，所以长度位宽不能再用 NPU_TK_W。
//
// start_in 收到一拍脉冲后，从 base_in 开始连续发出 len_in 个读请求：
//   start_in 当拍发出第 1 个请求，之后每个时钟沿地址加 1，发满 len_in 个后停止；
//   len_in=0 时不产生任何请求；突发长度和起始地址在 start_in 当拍被锁存，
//   之后改动 base_in/len_in 不影响本次突发。
// 本模块没有 busy 输出：突发期间再给 start_in 会从 base_in 重新开始本次突发，
// 所以上层必须保证两次启动不重叠。当前上层（tile_controller）每个输出 Tile 只启动
// 一次，且下一次启动必然在 K 拍之后（突发早就发完），由这个结构性约束保证。
// rd_re_out 是读请求，RAM 数据在请求后一拍返回（见 npu_ram）。
`include "npu_defines.vh"

module npu_stream_ctrl #(parameter AW = `NPU_ABUF_AW) (
    input        clk,
    input        rst,
    input [AW-1:0] base_in,
    input [`NPU_DIM_W-1:0] len_in,
    input        start_in,
    output reg [AW-1:0] rd_addr_out,
    output reg   rd_re_out
);

    reg [`NPU_DIM_W-1:0] left_q;     // 本次突发还没发出的请求数
    reg                  busy_q;     // 本次突发还没发完（内部使用，"还没发完"由本模块自己判定）

    always @(posedge clk) begin
        if (rst) begin
            rd_addr_out <= {AW{1'b0}};
            rd_re_out   <= 1'b0;
            busy_q      <= 1'b0;
            left_q      <= {`NPU_DIM_W{1'b0}};
        end
        else begin
            rd_re_out <= 1'b0;
            if (start_in && (len_in != 0)) begin
                // 第 1 个请求在启动当拍发出，剩余 len-1 个逐拍发出。
                rd_addr_out <= base_in;
                rd_re_out   <= 1'b1;
                left_q      <= len_in - 1'b1;
                busy_q      <= (len_in > 1'b1);
            end
            else if (busy_q) begin
                rd_addr_out <= rd_addr_out + 1'b1;
                rd_re_out   <= 1'b1;
                left_q      <= left_q - 1'b1;
                busy_q      <= (left_q > 1'b1);
            end
        end
    end

endmodule

// --------------------------------------------------------------- npu_sync_fifo.v

`timescale 1ns / 1ps
// npu_sync_fifo — 通用 FWFT(首字直通)同步 FIFO
//
// rdata 组合直出 rd_ptr 指向的队头，不需要先发起读命令。
// push/pop 只有在 FIFO 未满/非空时才被接受；push 与 pop 可以同拍发生。
// count 的范围是 0..(1<<AW)，valid 等价于 !empty。
`include "npu_defines.vh"

module npu_sync_fifo #(parameter AW = `NPU_FIFO_AW, DW = 32) (
    input        clk,
    input        rst,
    input        push,
    input [DW-1:0] wdata,
    input        pop,
    output reg [DW-1:0] rdata,
    output reg        valid,      // !empty
    output reg        empty,
    output reg        full,
    output reg [AW:0] count
);

    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW-1:0]  rd_ptr, wr_ptr;
    reg [AW:0]    cnt;

    // FWFT：队头数据直接从存储阵列组合读出。
    always @(*) begin
        rdata = mem[rd_ptr];
        valid = (cnt != 0);
        empty = (cnt == 0);
        full  = (cnt == (1<<AW));
        count = cnt;
    end

    // FWFT 读法：pop 只移动读指针，不额外锁存 rdata。
    // push/pop 同拍时两个指针都推进，count 保持不变。
    always @(posedge clk) begin
        if (rst) begin
            rd_ptr <= {AW{1'b0}};
            wr_ptr <= {AW{1'b0}};
            cnt    <= {(AW + 1){1'b0}};
        end
        else begin
            case ({push && !full, pop && !empty})
                2'b10: begin
                    mem[wr_ptr] <= wdata;
                    wr_ptr      <= wr_ptr + 1'b1;
                    cnt         <= cnt + 1'b1;
                end
                2'b01: begin
                    rd_ptr <= rd_ptr + 1'b1;
                    cnt    <= cnt - 1'b1;
                end
                2'b11: begin
                    mem[wr_ptr] <= wdata;
                    wr_ptr      <= wr_ptr + 1'b1;
                    rd_ptr      <= rd_ptr + 1'b1;
                    // count 不变。
                end
                default: begin
                    // 没有合法 push/pop，所有寄存器保持。
                end
            endcase
        end
    end

endmodule
