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
    input [31:0] mem_write_data,
    input [3:0] mem_byte_enable,
    output [31:0] mem_data,
    output mem_valid,
    output mem_ready
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//数据存储 32KB
    (* ram_style = "block" *) reg [31:0] dtcm [0:8191];
    integer i;
    initial begin
        for (i = 0; i < 8192; i = i + 1)
            dtcm[i] = 32'd0;
        $readmemh("e:/Vivado_Projects/project_risc_v/tools/hex/data.hex", dtcm);
    end

    reg [12:0] read_address;
    reg [3:0]  count;
    reg        address_valid;
    reg [31:0] data_registered;
    reg        valid_delayed;
    reg        request_delayed;
    reg        request_write_delayed;      // 上一拍的请求是【写】

//写用【电平】判据：连续多拍的 store 每拍都要落（原 flash_spi 有写 FIFO 兜，这里必须自己保证）
    reg start, write_now;
//★ 读启动：上一拍没有请求，或者上一拍是【写】——写不占读状态机，不该逼读再等掉拍。
//  不加这一项，dcache 的"写穿紧接回填"会让 mem_req 连续为高，读永远起不来（实测死锁）。
    always @(*) begin
        start  = mem_req & (~request_delayed | request_write_delayed);
        write_now = mem_req & mem_we;
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            read_address <= 13'd0;
            count <= 4'd0;
            address_valid <= 1'b0;
            request_delayed <= 1'b0;
            request_write_delayed <= 1'b0;
        end
        else begin
            request_delayed <= mem_req;
            request_write_delayed <= mem_req & mem_we;
//写：单拍，字节使能直接落到 BRAM 的字节写口
            if (write_now) begin
                if (mem_byte_enable[0]) dtcm[mem_addr[14:2]][7:0]   <= mem_write_data[7:0];
                if (mem_byte_enable[1]) dtcm[mem_addr[14:2]][15:8]  <= mem_write_data[15:8];
                if (mem_byte_enable[2]) dtcm[mem_addr[14:2]][23:16] <= mem_write_data[23:16];
                if (mem_byte_enable[3]) dtcm[mem_addr[14:2]][31:24] <= mem_write_data[31:24];
            end
//读：锁行首地址，按行连吐 8 个字
            if (start & ~mem_we) begin
                read_address  <= mem_addr[14:2];
                count    <= 4'd8;
                address_valid <= 1'b1;
            end
            else if (count != 4'd0) begin
                read_address  <= read_address + 13'd1;
                count    <= count - 4'd1;
                address_valid <= 1'b1;
            end
            else
                address_valid <= 1'b0;
        end
    end

//单一地址表达式 + 输出寄存器：BRAM 的可推断模板
    always @(posedge clk) begin
        data_registered <= dtcm[read_address];
        valid_delayed <= address_valid;
    end

    assign mem_data  = data_registered;
    assign mem_valid = valid_delayed;
    assign mem_ready = 1'b1;

endmodule
