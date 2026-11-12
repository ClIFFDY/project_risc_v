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
        end else begin
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
