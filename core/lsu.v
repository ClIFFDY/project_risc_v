`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/08/29 19:30:28
// Design Name:
// Module Name: lsu
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


module lsu(
    input clk, rst,
    input [4:0] flag_bus,
//前置冲刷（早一拍），由 bju 的组合判定直接给出：判定结果寄存后只能覆盖 c1..c4 与 wb，
//而错路指令在 c2 上会停留两拍（前一条落前置拍、后一条落寄存拍），那两拍里它已经会去
//动 FIFO 指针、拉总线（store 也在这条路上），等寄存器清已经收不回来，故入口要多挡一拍。
//不进 flag_bus：绕 controller 一圈会把这条晚到的组合信号挂上全片广播网（实测多花 0.45ns）。
    input stallf,
    input [6:0] opcode,
    input [9:0] func10,
    input [4:0] rd_in, r1_post, r2_post,
    input [31:0] r1_data_final, r2_data_final,
    input [31:0] offset_load0, offset_store0,
    input [31:0] bus_data_ext, bus_data_dcache, bus_data_tim,
    input ready_dcache, ready_tim, ready_ext,
    input bus_hold_in, dcache_hold,
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out,
    output reg bus_valid_out,
    output reg [31:0] ld_data_out,
    output reg loaded,
    output reg ld_we,
    output reg stall,
    output reg [4:0] rd_load
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STORE = 7'b0100011;

    reg [31:0] st_addr;

    reg [4:0] ld_rd_fifo [0:3];
    reg [2:0] ld_size_fifo [0:3];
    reg [1:0] ld_off_fifo [0:3];
    reg [2:0] ld_wr_ptr, ld_rd_ptr;
    reg [3:0] ld_occ;
    reg [4:0] ld_hold_rd;
    reg [31:0] ld_hold_data;
    reg [31:0] ld_data_cur;
    reg ld_hold;

    reg ld_fifo_empty, ld_fifo_full, ld_pop, ld_enq, ld_push_eff;
    reg ld_use_hit;
    reg [3:0] ld_occ_q;
    reg [4:0] ld_rd_cur;
    reg [2:0] ld_size_cur;
    reg [1:0] ld_off_cur;

//flag_bus = {exec, flush_irq, flush_jump, stall_d, stall_i}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
//本模块的冲刷窗口比别的模块【宽一拍】（多 OR 一个 stallf）：入队门控与总线选通都挂在
//这同一个 flush_w 上，多一项即可覆盖两拍，模块内部逻辑一行不用动。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[3] | flag_bus[2] | stallf;
        stall_w = (flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[4];
    end

//总线读数据与停顿的合流（按"顶层不运算"从 cpu_top 下放至此）：
//三源按位或 —— 从设备必须每拍清零，否则残留值会被或进别人的读数据
//（实测：读 UART 状态口拿到上一次 dtcm 读的值 → ee_printf 的 while 轮询死循环）。
    reg [31:0] bus_data_in;
    reg ready_in, bus_hold;
    always @(*) begin
        bus_data_in = bus_data_ext | bus_data_dcache | bus_data_tim;
        ready_in    = ready_dcache | ready_tim | ready_ext;
        bus_hold    = bus_hold_in | dcache_hold;
    end

//在途 load 队列的空满、收发条件与队头载荷
    always @(*) begin
        ld_fifo_empty = (ld_wr_ptr == ld_rd_ptr);
        ld_fifo_full  = (ld_wr_ptr[2] != ld_rd_ptr[2]) && (ld_wr_ptr[1:0] == ld_rd_ptr[1:0]);
        ld_pop        = ready_in && !ld_fifo_empty;
        ld_enq        = (opcode == OPCODE_LOAD) && !stall && !flush_w && !bus_hold;
        ld_push_eff   = ld_enq && !(ld_fifo_full && !ld_pop);
        ld_rd_cur     = ld_rd_fifo[ld_rd_ptr[1:0]];
        ld_size_cur   = ld_size_fifo[ld_rd_ptr[1:0]];
        ld_off_cur    = ld_off_fifo[ld_rd_ptr[1:0]];
    end

//ld/st读写类指令总线地址处理逻辑
    always @(posedge clk) begin
        if (rst_q) begin
            bus_addr_out <= 32'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            bus_valid_out <= 1'b0;
        end
        else if (exec) begin
            if (stall_w) begin
                if (ld_push_eff) begin
                    bus_addr_out <= (r1_data_final + offset_load0) >> 2;
                    bus_we_out <= 1'b0;
                    bus_be_out <= 4'd0;
                    bus_data_out <= 32'd0;
                    bus_valid_out <= 1'b1;
                end
//读地址只摆一拍：外设应答是寄存的，下一拍照常到达，重复摆地址只会让它连答
                else begin
                    bus_addr_out <= 32'd0;
                    bus_we_out <= 1'b0;
                    bus_be_out <= 4'd0;
                    bus_data_out <= 32'd0;
                    bus_valid_out <= 1'b0;
                end
            end
            else if (!flush_w && !stall_w) begin
                bus_addr_out <= 32'd0;
                bus_data_out <= 32'd0;
                bus_be_out <= 4'd0;
                bus_we_out <= 1'b0;
                bus_valid_out <= 1'b0;
                case (opcode)
                    OPCODE_LOAD: begin
                        bus_valid_out <= ld_push_eff;
                        case (func10[2:0])
                            3'b000: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 32'd0;
                            3'b001: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 32'd0;
                            3'b010: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 32'd0;
                            3'b100: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 32'd0;
                            3'b101: bus_addr_out <= ld_push_eff ? ((r1_data_final + offset_load0) >> 2) : 32'd0;
                            default: begin
                                bus_addr_out <= 32'd0;
                                bus_valid_out <= 1'b0;
                            end
                        endcase
                    end
                    OPCODE_STORE: begin
                        bus_we_out <= 1'b1;
                        bus_valid_out <= 1'b1;
                        case (func10[2:0])
                            3'b000: begin
                                bus_addr_out <= st_addr >> 2;
                                bus_be_out <= 4'b0001 << st_addr[1:0];
                                bus_data_out <= {24'd0, r2_data_final[7:0]} << (8 * st_addr[1:0]);
                            end
                            3'b001: begin
                                bus_addr_out <= st_addr >> 2;
                                bus_be_out <= 4'b0011 << (2 * st_addr[1]);
                                bus_data_out <= {16'd0, r2_data_final[15:0]} << (16 * st_addr[1]);
                            end
                            3'b010: begin
                                bus_addr_out <= (r1_data_final + offset_store0) >> 2;
                                bus_be_out <= 4'b1111;
                                bus_data_out <= r2_data_final;
                            end
                            default: begin
                                bus_addr_out <= 32'd0;
                                bus_valid_out <= 1'b0;
                            end
                        endcase
                    end
                endcase
            end
            else begin
                bus_addr_out <= 32'd0;
                bus_data_out <= 32'd0;
                bus_be_out <= 4'd0;
                bus_we_out <= 1'b0;
                bus_valid_out <= 1'b0;
            end
        end
    end

//在途 load 请求队列：load 发射入队，应答出队，rd/size/off 与应答同源出队。
//ld_occ 是与数据阵列并行维护的占用掩码（绝对下标）：出队先清、入队后置，
//两者撞同一格（队满且本拍既收又发）时后置生效——新入队的那项才是有效的那个。
    always @(posedge clk) begin
        if (rst_q) begin
            ld_wr_ptr <= 3'd0;
            ld_rd_ptr <= 3'd0;
            ld_occ    <= 4'd0;
        end
        else begin
            if (ld_pop)      ld_occ[ld_rd_ptr[1:0]] <= 1'b0;
            if (ld_push_eff) ld_occ[ld_wr_ptr[1:0]] <= 1'b1;
            if (ld_push_eff) begin
                ld_rd_fifo[ld_wr_ptr[1:0]]   <= rd_in;
                ld_size_fifo[ld_wr_ptr[1:0]] <= func10[2:0];
                ld_off_fifo[ld_wr_ptr[1:0]]  <= r1_data_final + offset_load0;
                ld_wr_ptr <= ld_wr_ptr + 3'd1;
            end
            if (ld_pop)
                ld_rd_ptr <= ld_rd_ptr + 3'd1;
        end
    end

//应答只摆一拍的话，落在 back2 槽的消费者会取不到，故再保持一拍
    always @(posedge clk) begin
        if (rst_q) ld_hold <= 1'b0;
        else begin
            ld_hold <= ld_pop;
            if (ld_pop) begin
                ld_hold_rd <= ld_rd_cur;
                ld_hold_data <= ld_data_cur;
            end
        end
    end

//load-use 互锁：不按"上一条是 load 吗"做单槽判断，而是扫遍在途队列的每一个占用项。
//队列现在可同时挂 4 条 load，单槽判断只认最近发射的那一条，更早就在途的那几条会漏判——
//而漏判不会报错，只会把还没回来的数据当成已经回来的用。
//本拍就要出队的那一项排除在外：它的数据这拍已经从总线上取回、可以同拍前递，不该再压流水线。
    always @(*) begin
        ld_occ_q   = ld_occ;
        ld_use_hit = 1'b0;
        if (ld_pop) ld_occ_q[ld_rd_ptr[1:0]] = 1'b0;
        if (ld_occ_q[0] && ((ld_rd_fifo[0] == r1_post) | (ld_rd_fifo[0] == r2_post))) ld_use_hit = 1'b1;
        if (ld_occ_q[1] && ((ld_rd_fifo[1] == r1_post) | (ld_rd_fifo[1] == r2_post))) ld_use_hit = 1'b1;
        if (ld_occ_q[2] && ((ld_rd_fifo[2] == r1_post) | (ld_rd_fifo[2] == r2_post))) ld_use_hit = 1'b1;
        if (ld_occ_q[3] && ((ld_rd_fifo[3] == r1_post) | (ld_rd_fifo[3] == r2_post))) ld_use_hit = 1'b1;
    end

//stall信号拉起逻辑
    always @(*) begin
        st_addr = r1_data_final + offset_store0;
        stall   = ld_use_hit;
    end

//字节使能数据返回输出：出队拍按 off/size 取字节
    always @(*) begin
        case (ld_size_cur)
            3'b000: ld_data_cur = {{24{bus_data_in[8*ld_off_cur + 7]}}, bus_data_in[8*ld_off_cur +: 8]};
            3'b001: ld_data_cur = {{16{bus_data_in[16*ld_off_cur[1] + 15]}}, bus_data_in[16*ld_off_cur[1] +: 16]};
            3'b010: ld_data_cur = bus_data_in;
            3'b100: ld_data_cur = {24'd0, bus_data_in[8*ld_off_cur +: 8]};
            3'b101: ld_data_cur = {16'd0, bus_data_in[16*ld_off_cur[1] +: 16]};
            default: ld_data_cur = bus_data_in;
        endcase
    end

//rd 与 data 同拍同源输出；ld_we 只在应答拍置起（避免保持拍重复写寄存器堆）
    always @(*) begin
        ld_we = ld_pop;
        if (ld_pop) begin
            loaded = 1'b1;
            rd_load = ld_rd_cur;
            ld_data_out = ld_data_cur;
        end
        else if (ld_hold) begin
            loaded = 1'b1;
            rd_load = ld_hold_rd;
            ld_data_out = ld_hold_data;
        end
        else begin
            loaded = 1'b0;
            rd_load = 5'd0;
            ld_data_out = bus_data_in;
        end
    end
endmodule
