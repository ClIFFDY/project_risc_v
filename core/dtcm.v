`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/28
// Design Name:
// Module Name: dtcm
// Project Name:
// Target Devices:
// Tool Versions:
// Description: 数据紧耦合内存，32KB。作为 dcache 的后备读写存储。
//              读：mem_req 上升沿锁行首，随后连拍 8 个字（每行 8 字 = 32B）
//              写：mem_req&mem_we 电平判据，单拍完成（字节使能）
//
//              读口必须是"地址寄存器 → RAM → 输出寄存器"的单一形状，否则 Vivado
//              推不出 BRAM（原写法读地址有两个表达式 mem_addr/ base+n，实测
//              `8-6849 Infeasible ram_style="block" ... trying to implement using LUTRAM`
//              ——32KB 全塌成 4096 个 LUTRAM）。
//              代价：比原来晚 1 拍出第一个字；mem_valid 与 mem_data 同拍对齐，
//              上游（dcache）按 mem_valid 数拍收字，不受影响。写路径拍数不变。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module dtcm(
    input clk, rst,
    input mem_req, mem_we,
    input [31:0] mem_addr,
    input [31:0] mem_wdata,
    input [3:0] mem_be,
    output [31:0] mem_data,
    output mem_valid,
    output mem_ready
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

//数据存储 32KB
    (* ram_style = "block" *) reg [31:0] dtcm [0:8191];
    integer i;
    initial begin
        for (i = 0; i < 8192; i = i + 1) dtcm[i] = 32'd0;
        $readmemh("e:/Vivado_Projects/project_risc_v/tools/hex/data.hex", dtcm);
    end

    reg [12:0] raddr;
    reg [3:0]  cnt;
    reg        addr_v;
    reg [31:0] d_r;
    reg        v_d;
    reg        req_d;

//写用【电平】判据：连续多拍的 store 每拍都要落（原 flash_spi 有写 FIFO 兜，这里必须自己保证）
    reg start, wr_now;
    always @(*) begin
        start  = mem_req & ~req_d;
        wr_now = mem_req & mem_we;
    end

    always @(posedge clk) begin
        if (rst_q) begin
            raddr <= 13'd0;
            cnt <= 4'd0;
            addr_v <= 1'b0;
            req_d <= 1'b0;
        end
        else begin
            req_d <= mem_req;
//写：单拍，字节使能直接落到 BRAM 的字节写口
            if (wr_now) begin
                if (mem_be[0]) dtcm[mem_addr[14:2]][7:0]   <= mem_wdata[7:0];
                if (mem_be[1]) dtcm[mem_addr[14:2]][15:8]  <= mem_wdata[15:8];
                if (mem_be[2]) dtcm[mem_addr[14:2]][23:16] <= mem_wdata[23:16];
                if (mem_be[3]) dtcm[mem_addr[14:2]][31:24] <= mem_wdata[31:24];
            end
//读：锁行首地址，按行连吐 8 个字
            if (start & ~mem_we) begin
                raddr  <= mem_addr[14:2];
                cnt    <= 4'd8;
                addr_v <= 1'b1;
            end
            else if (cnt != 4'd0) begin
                raddr  <= raddr + 13'd1;
                cnt    <= cnt - 4'd1;
                addr_v <= 1'b1;
            end
            else addr_v <= 1'b0;
        end
    end

//单一地址表达式 + 输出寄存器：BRAM 的可推断模板
    always @(posedge clk) begin
        d_r <= dtcm[raddr];
        v_d <= addr_v;
    end

    assign mem_data  = d_r;
    assign mem_valid = v_d;
    assign mem_ready = 1'b1;

endmodule
