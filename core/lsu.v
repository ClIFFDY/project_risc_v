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
// Description: 两级（发级 + 写回级）+ 直接入径
//   级2（发送）：常态下 ld/st 直接进这一级，进来的当拍把地址/数据/be 摆上总线；下一拍进级3。
//   级3（写回）：占用 ⇒ 组合拉高 stall（把入口顶住）——所以能进来的新指令只可能撞上级2，
//                hazard 只看级2 就够。store 进级3 即算提交（下一拍让位）；load 等应答。
//   miss（dcache_hold_in 当拍）：新指令进级3 停靠（操作数当拍锁进本级），miss 落下那拍
//               级3 → 级2 前移一级，再照常发送。
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module lsu(
    input clk, rst,
    input [9:0] flag_bus,
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
    input dcache_hold_in,
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out,
    output reg bus_valid_out,
    output reg [31:0] ld_data_out,
    output reg loaded,
    output reg ld_we,
    output reg stall,
    output reg mem_inflight,
    output reg [4:0] rd_load,
    output reg exc_ldst_misalign, exc_ldst_st,
    output reg [31:0] exc_ldst_addr
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STORE = 7'b0100011;

//flag_bus = {exc, exec, flush_irq, flush_jump, dcache_hold, bus_hold_in, stall_m, stall_v, lsu_stall, icache_busy}
//控制位译码（行为块，放本模块最前）：三条互斥 —— 旧 stage 是单值而两条位可同时为 1，
//故这里保持【冲刷优先于停顿】；exec 即本模块的停开机使能。
//本模块的冲刷窗口比别的模块【宽一拍】（多 OR 一个 stallf）：入队门控与总线选通都挂在
//这同一个 flush_w 上，多一项即可覆盖两拍，模块内部逻辑一行不用动。
    reg exec, flush_w, stall_w;
    always @(*) begin
        flush_w = flag_bus[9] | flag_bus[7] | flag_bus[6] | stallf;
        stall_w = (dcache_hold_in | flag_bus[4] | flag_bus[3] | flag_bus[2] | flag_bus[1] | flag_bus[0]) & ~flush_w;
        exec    = flag_bus[8];
    end

//总线读数据与停顿的合流（按"顶层不运算"从 cpu_top 下放至此）：
//三源按位或 —— 从设备必须每拍清零，否则残留值会被或进别人的读数据
//（实测：读 UART 状态口拿到上一次 dtcm 读的值 → ee_printf 的 while 轮询死循环）。
    reg [31:0] bus_data_in;
    reg ready_in, bus_hold;
    always @(*) begin
        bus_data_in = bus_data_ext | bus_data_dcache | bus_data_tim;
        ready_in    = ready_dcache | ready_tim | ready_ext;
        bus_hold    = dcache_hold_in | flag_bus[4];
    end

//两级
//  级2（发送）：miss 当拍照进（操作数锁好），等回填完再发；常态进级那拍就发。
//               载荷带全（addr/wdat/be），s2_sent 记"发过没有"。
    reg        s2_v, s2_kind, s2_sent;
    reg [4:0]  s2_rd;
    reg [2:0]  s2_size;
    reg [1:0]  s2_off;
    reg [31:0] s2_addr, s2_wdat;
    reg [3:0]  s2_be;
//  级3（写回）：与 dcache 里锁住的那条是【同一条】—— 进级2 时已经发过请求，
//               在这一级只等数据回来（ready_in），所以不需要再发、也不用带载荷。
    reg        s3_v, s3_kind;
    reg [4:0]  s3_rd;
    reg [2:0]  s3_size;
    reg [1:0]  s3_off;

//写回保持与数据返回
    reg [4:0] ld_hold_rd;
    reg [31:0] ld_hold_data;
    reg [31:0] ld_data_cur;
    reg ld_hold;

//组合判据
    reg mem_op, is_st, new_in, new_in_pre, new_go, full_stall, bus_go;
    reg s3_done, s3_ok, s2_move, s2_ok, s2_put_go;
    reg ld_out, blank_bus, s2_put, new_put;
    reg ls_use_hit, ls_waw_hit, miss;
    reg [31:0] st_addr, st_wdata, cur_addr;
    reg [3:0]  st_be;
    reg [31:0] byte_addr;
    reg exc_ldst_misalign_now, size_bad_now;

//===============================================================
// 组合：入级/推进 + 冒险（只看级2）+ 入口
//===============================================================
//store 的字节使能与数据（按地址低位对齐后随请求一起摆上总线）
    always @(*) begin
        st_addr = r1_data_final + offset_store0;
        case (func10[2:0])
            3'b000: begin st_be = 4'b0001 << st_addr[1:0]; st_wdata = {24'd0, r2_data_final[7:0]} << (8 * st_addr[1:0]); end
            3'b001: begin st_be = 4'b0011 << (2 * st_addr[1]); st_wdata = {16'd0, r2_data_final[15:0]} << (16 * st_addr[1]); end
            3'b010: begin st_be = 4'b1111; st_wdata = r2_data_final; end
            default: begin st_be = 4'd0; st_wdata = 32'd0; end
        endcase
    end

//本条摆上总线要用的载荷
    always @(*) begin
        is_st    = (opcode == OPCODE_STORE);
        cur_addr = is_st ? (st_addr >> 2) : ((r1_data_final + offset_load0) >> 2);
    end

//入口两道合法性：①非对齐（按访问宽度）②【宽度本身非法】（ld/sd 之类在 RV32 非法）。
//②由 post_decoder 判成非法指令（cause 2）；这里再挡一道是为了让它【根本不进车厢】——
//否则它会先上总线（非法宽度的 store 以 be=0000 上线、非法宽度的 load 读回垃圾还写回 rd），
//而 lsu 两级车厢在冲刷拍是【不清】的。★ 这张白名单必须与 post_decoder 的白名单一致。
    always @(*) begin
        byte_addr = is_st ? st_addr : (r1_data_final + offset_load0);
        exc_ldst_misalign_now = 1'b0;
        size_bad_now = 1'b0;
        if (mem_op) begin
            if (func10[1:0] == 2'b01) begin
                if (byte_addr[0] != 1'b0) exc_ldst_misalign_now = 1'b1;
            end
            if (func10[1:0] == 2'b10) begin
                if (byte_addr[1:0] != 2'd0) exc_ldst_misalign_now = 1'b1;
            end
        end
        if (opcode == OPCODE_LOAD) begin
            if (func10[2:0] == 3'b011) size_bad_now = 1'b1;
            if (func10[2:0] == 3'b110) size_bad_now = 1'b1;
            if (func10[2:0] == 3'b111) size_bad_now = 1'b1;
        end
        if (opcode == OPCODE_STORE) begin
            if (func10[2:0] == 3'b011) size_bad_now = 1'b1;
            if (func10[2:0] == 3'b100) size_bad_now = 1'b1;
            if (func10[2:0] == 3'b101) size_bad_now = 1'b1;
            if (func10[2:0] == 3'b110) size_bad_now = 1'b1;
            if (func10[2:0] == 3'b111) size_bad_now = 1'b1;
        end
    end

//让位与入口：
//  级3：store 进这一级即算提交（下一拍让位）；load 等应答
//  级2：级3 让位那拍就前移一级
//  miss 当拍：新指令进级3 停靠；否则进级2（要求级3、级2 都腾出来）
    always @(*) begin
        mem_op   = (opcode == OPCODE_LOAD) | (opcode == OPCODE_STORE);
        miss     = dcache_hold_in;
        s3_done  = s3_v & (s3_kind | ready_in);
        s3_ok    = ~s3_v | s3_done;
        s2_move  = s2_v & s2_sent & s3_ok;   // 没发过的级不许前移（否则请求就丢了）
        s2_ok    = ~s2_v | s2_move;
        new_in_pre = mem_op & ~size_bad_now & ~flush_w & ~(flag_bus[3] | flag_bus[2] | flag_bus[0]) & ~ls_use_hit & ~ls_waw_hit;
        new_in     = new_in_pre & ~exc_ldst_misalign_now;
//入口【不看 miss】：miss 当拍也要进级锁操作数；只有发送等 miss 落下。
        new_go     = s3_ok & s2_ok;
//空总线（blank）：miss 在跑，或"在途那笔读还没被应答" ⇒ 这一拍总线上不摆。
//在途读 = 级3 里那笔读，或级2 里刚摆上、下一拍才进级3 的那笔读（后一项不能省：快慢设备
//混跑时先回来的应答会串到前一条头上 —— 实测 ls_mix2 两条读结果互换）。
//dcache 1 拍应答 ⇒ 该摆下一笔的那拍 ready_in 正好是 1 ⇒ blank 为 0 ⇒ 快路径一拍不减速。
        ld_out     = (s2_v & ~s2_kind & s2_sent) | (s3_v & ~s3_kind);
        blank_bus  = miss | (ld_out & ~ready_in);
        s2_put     = s2_v & ~s2_sent & ~blank_bus;
        s2_put_go  = s2_put & exec & ~flush_w;
        new_put    = new_in & new_go & ~blank_bus & exec & ~flush_w;
        bus_go     = s2_put_go | new_put;
        full_stall = mem_op & ~new_go;
    end

//总线呈现：摆出来那拍（bus_go）才有值，blank 的拍次全 0 —— 新指令进级2 那拍就摆（与 HEAD 同拍）
    always @(posedge clk) begin
        if (rst_q) begin
            bus_addr_out <= 32'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            bus_valid_out <= 1'b0;
        end
        else if (bus_go && exec && !flush_w) begin
            bus_addr_out  <= s2_put ? s2_addr : cur_addr;
            bus_data_out  <= s2_put ? s2_wdat : st_wdata;
            bus_be_out    <= s2_put ? s2_be   : st_be;
            bus_we_out    <= s2_put ? s2_kind : is_st;
            bus_valid_out <= 1'b1;
        end
//blank 的拍必须把总线清零：从设备是电平判据，残留地址会被当成新请求连答（读地址只摆一拍的原因）
        else begin
            bus_addr_out <= 32'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            bus_valid_out <= 1'b0;
        end
    end

//冒险只扫级2（级3 占着的时候入口已经被 stall 顶住，新指令根本进不来）。
//x0 不算依赖：在途项的 rd=0（store 那一级的 rd 就是 0）与操作数 x0 都排除。
    always @(*) begin
        ls_use_hit = 1'b0;
        ls_waw_hit = 1'b0;
        if (s2_v && s2_rd != 5'd0) begin
            if (r1_post != 5'd0 && (s2_rd == r1_post)) ls_use_hit = 1'b1;
            if (r2_post != 5'd0 && (s2_rd == r2_post)) ls_use_hit = 1'b1;
            if (rd_in   != 5'd0 && (s2_rd == rd_in))   ls_waw_hit = 1'b1;
        end
    end

//级3 有人 ⇒ 组合拉高整个 stall（不分访存/非访存）：级3 里那条的结果还没回来时，
//后面的任何指令都不许越过它去读寄存器——这就是"hazard 只用看级2"的依据。
    always @(*) stall = ls_use_hit | ls_waw_hit | (s3_v & ~s3_done) | full_stall;

//===============================================================
// 时序：两级推进
//===============================================================
    always @(posedge clk) begin
        if (rst_q) begin
            s2_v <= 1'b0; s3_v <= 1'b0;
        end
        else begin
            if (s3_done) s3_v <= 1'b0;                                  // 级3 走掉
            if (s2_put_go) s2_sent <= 1'b1;                             // 补摆
            if (s2_move) begin                                          // 级2 → 级3（请求已摆过）
                s3_v <= 1'b1; s3_kind <= s2_kind; s3_rd <= s2_rd; s3_size <= s2_size; s3_off <= s2_off;
                s2_v <= 1'b0;
            end
            if (new_in && new_go) begin                                 // 新指令进级2（miss 当拍也进；没摆就留着）
                s2_v <= 1'b1; s2_kind <= is_st; s2_rd <= rd_in; s2_size <= func10[2:0];
                s2_off <= is_st ? 2'd0 : (r1_data_final + offset_load0);
                s2_addr <= cur_addr; s2_wdat <= st_wdata; s2_be <= st_be;
                s2_sent <= new_put;                                     // 没摆出去就保持 0，等不 blank 了再补摆
            end
        end
    end

//访存非对齐故障寄存一拍：对齐到 E4 相位，与其它异常共用同一条交付通路。
//★ 判定必须与 new_in_pre（入口条件）相与，不能只看 exc_ldst_misalign_now：
//  被冲刷/停顿/冒险挡住的指令【本来就进不了 lsu】，它那一拍的 byte_addr 是错路指令的
//  垃圾操作数（实测 CoreMark：一个被冲刷的错路 lh 判出 byte_addr=0xffffffff）⇒ 对它抛
//  异常就是误判，会把好程序打断。new_in_pre 里已含 ~flush_w ⇒ 冲刷拍天然为 0，
//  不需要单独的清除分支。
//出错地址与故障同拍寄存：mtval 要的就是它，而 byte_addr 只在入口那一拍有效。
    always @(posedge clk) begin
        if (rst_q) begin
            exc_ldst_misalign <= 1'b0;
            exc_ldst_st <= 1'b0;
            exc_ldst_addr <= 32'd0;
        end
        else begin
            exc_ldst_misalign <= exc_ldst_misalign_now & new_in_pre;
            exc_ldst_st <= is_st;
            exc_ldst_addr <= byte_addr;
        end
    end

//写回口保持：离开级3 那拍给数据，落在 back2 槽的消费者取不到 ⇒ 再保持一拍
    always @(posedge clk) begin
        if (rst_q) ld_hold <= 1'b0;
        else begin
            ld_hold <= ld_we;
            if (ld_we) begin
                ld_hold_rd <= s3_rd;
                ld_hold_data <= ld_data_cur;
            end
        end
    end

//===============================================================
// 写回 / 提交
//===============================================================
    always @(*) begin
        case (s3_size)
            3'b000: ld_data_cur = {{24{bus_data_in[8*s3_off + 7]}}, bus_data_in[8*s3_off +: 8]};
            3'b001: ld_data_cur = {{16{bus_data_in[16*s3_off[1] + 15]}}, bus_data_in[16*s3_off[1] +: 16]};
            3'b010: ld_data_cur = bus_data_in;
            3'b100: ld_data_cur = {24'd0, bus_data_in[8*s3_off +: 8]};
            3'b101: ld_data_cur = {16'd0, bus_data_in[16*s3_off[1] +: 16]};
            default: ld_data_cur = bus_data_in;
        endcase
    end

    always @(*) begin
        ld_we = s3_v & ~s3_kind & ready_in;
        if (ld_we) begin
            loaded = 1'b1; rd_load = s3_rd; ld_data_out = ld_data_cur;
        end
        else if (ld_hold) begin
            loaded = 1'b1; rd_load = ld_hold_rd; ld_data_out = ld_hold_data;
        end
        else begin
            loaded = 1'b0; rd_load = 5'd0; ld_data_out = bus_data_in;
        end
    end

    always @(*) mem_inflight = s2_v | s3_v;

endmodule
