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
    input [11:0] flag_bus,
//前置冲刷（早一拍），由 bju 的组合判定直接给出（源名 flush_bju_pre）：
//判定结果寄存后只能覆盖 c1..c4 与 wb，而错路指令在 c2 上会停留两拍
//（前一条落前置拍、后一条落寄存拍），那两拍里它已经会去
//动 FIFO 指针、拉总线（store 也在这条路上），等寄存器清已经收不回来，故入口要多挡一拍。
//不进 flag_bus：绕 controller 一圈会把这条晚到的组合信号挂上全片广播网（实测多花 0.45ns）。
    input flush_bju_pre,
//本次冲刷的边界（controller 的 flush_idx：三种冲刷各取自己那条的号）：
//入口门按它判"本拍站在载荷上的这条，是边界自己、还是比边界更年轻的错路条"。
    input [2:0]  flush_idx,
//ROB 队头（年龄基准）+ 写口级的落地广播（b0/b1）：本模块据此杀自己更老的同 rd 在途记录
    input [2:0]  rob_head,
//跳转冲刷的边界：那条分支/跳转自己那一项（bju 寄存的）。冲刷时要按它把车厢里的错路记录作废。
    input [2:0]  bju_idx_q,

    input        b0_we,
    input [4:0]  b0_rd,
    input [2:0]  b0_idx,
    input        b1_we,
    input [4:0]  b1_rd,
    input [2:0]  b1_idx,
//入口门（每条指令只收一次）：decoder 载荷【这一拍就要推进】才允许收。
//★ 必须用"本拍推进"而不是"上一拍推进过"：车厢满（stall_lsu_full）时前端会被冻住、
//  payload_go 当拍就是 0，用上一拍那种"事后脉冲"会让这一条在新车厢里被静默丢掉
//  （它已经进了 E4、ROB 也分配了项，却永远等不到 ld_we）⇒ 队头永远退不掉 ⇒ 自锁（实测）。
    input payload_go,
    input [6:0] opcode,
    input [9:0] func10,
    input [4:0] rd_in, r1_post, r2_post,
//ROB 索引载荷：与其它载荷同使能锁存，回填时用它写进 ROB 自己那一项
    input [2:0]  idx_in,
//落地广播（来自写回级两个写口）：本单元据此把更老的同 rd 在途 load 作废
    input [31:0] r1_data_final, r2_data_final,
    input [31:0] offset_load0, offset_store0,
    input [31:0] bus_data_ext, bus_data_dcache, bus_data_tim,
//dcache 的停顿：只在本模块内部生效，不进 flag_bus 的全核广播。
//  miss 当拍不摆总线、等它落下的拍才发送；本模块的入口也照旧不看它（车厢占满由 full 顶住）。
//  原先它是经 controller 绕一圈、经全核广播回来再给本模块用的，现在由 dcache 直连过来。
    input dcache_hold,
    input ready_dcache, ready_tim, ready_ext,
    output reg [31:0] bus_addr_out,
    output reg [31:0] bus_data_out,
    output reg [3:0] bus_be_out,
    output reg bus_we_out,
    output reg bus_valid_out,
    (* max_fanout = 8 *) output reg [31:0] ld_data_out,
    (* max_fanout = 8 *) output reg loaded,
    output reg ld_we,
//被杀标记（给写口级）：这一笔永不落地 —— 写口不发、照常回报，让队头能退
    output reg kill_ld,
//三条停顿源【逐条】对外：controller 原样过路进 flag_bus，或运算在消费者模块内做
    output reg stall_lsu_haz,
    output reg stall_lsu_unload,
    output reg stall_lsu_full,
    output reg mem_inflight,
    (* max_fanout = 8 *) output reg [4:0] rd_load,
//本条写回记录带的 ROB 索引（跟着数据走）
    (* max_fanout = 8 *) output reg [2:0]  ld_idx,
    output reg exc_ldst_misalign_out, exc_ldst_st_out,
    output reg [31:0] exc_ldst_addr_out,
//故障指令【自己的】ROB 索引（与故障同拍寄存）：故障晚一拍到 controller，那时 issue_idx 已是下一条
    output reg [2:0] exc_ldst_idx_out
    );

//复位就地打一拍：rst 由 rst_buf 单点扇出到全核约 2900 个触发器，工具只能在布局阶段自己复制
//14 份，而那时芯片已经快满了。每个模块各自打一拍，寄存器就落在本模块旁边；全核都只打一拍，
//彼此没有相位差，是一起晚一拍出复位（同步复位晚一拍发布是安全的）。
    reg rst_q;
    always @(posedge clk) rst_q <= rst;

    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STORE = 7'b0100011;

//flag_bus = {flush_con_exc, flush_con_irq, flush_con_jump, exec,
//            stall_rob_full, stall_pc_redir,
//            stall_lsu_haz, stall_lsu_full,
//            stall_mulu_haz, stall_mulu_div,
//            stall_icache_miss, stall_bus_hold}
//控制位译码（行为块，放本模块最前）：冲刷优先于停顿；exec 即本模块的停开机使能。
//本模块的冲刷窗口比别的模块【宽一拍】（多 OR 一个 flush_bju_pre，那是 bju 判定的组合版、早一拍）：
//入队门控与总线选通都挂在这同一个 flush_w 上，多一项即可覆盖两拍，模块内部逻辑一行不用动。
    reg exec, flush_w, flush_con_exc;
    always @(*) begin
        flush_con_exc = flag_bus[11];
        flush_w = flush_con_exc | flag_bus[10] | flag_bus[9] | flush_bju_pre;
        exec    = flag_bus[8];
    end

//总线读数据与停顿的合流（按"顶层不运算"从 cpu_top 下放至此）：
//三源按位或 —— 从设备必须每拍清零，否则残留值会被或进别人的读数据
//（实测：读 UART 状态口拿到上一次 dtcm 读的值 → ee_printf 的 while 轮询死循环）。
    reg [31:0] bus_data_in;
    reg ready_in;
    always @(*) begin
        bus_data_in = bus_data_ext | bus_data_dcache | bus_data_tim;
        ready_in    = ready_dcache | ready_tim | ready_ext;
    end

//两级
//  级2（发送）：miss 当拍照进（操作数锁好），等回填完再发；常态进级那拍就发。

//               载荷带全（addr/wdat/be），s2_sent 记"发过没有"。
    reg        s2_v, s2_kind, s2_sent;
    reg [4:0]  s2_rd;
    reg [2:0]  s2_idx;
    reg        s2_kl;
    reg [2:0]  s2_size;
    reg [1:0]  s2_off;
    reg [31:0] s2_addr, s2_wdat;
    reg [3:0]  s2_be;
//  级3（写回）：与 dcache 里锁住的那条是【同一条】—— 进级2 时已经发过请求，
//               在这一级只等数据回来（ready_in），所以不需要再发、也不用带载荷。
    reg        s3_v, s3_kind;
    reg [4:0]  s3_rd;
    reg [2:0]  s3_idx;
    reg        s3_kl;
    reg [2:0]  s3_size;
    reg [1:0]  s3_off;

//写回保持与数据返回
    reg [4:0] ld_hold_rd;
    reg [2:0]  idx_hold;
    reg [31:0] ld_hold_data;
    reg [31:0] ld_data_cur;
    reg ld_hold;

//组合判据
    reg mem_op, is_st, new_in, new_in_pre, new_go, full_stall, bus_go;
    reg ent_bnd, ent_young;
    reg s3_done, s3_ok, s2_move, s2_ok, s2_put_go;
    reg ld_out, blank_bus, s2_put, new_put;
    reg ls_use_hit, miss;
    reg [31:0] addr_sum, st_wdata, cur_addr;
    reg [3:0]  st_be;
    reg [31:0] byte_addr;
    reg exc_ldst_misalign, size_bad_now;

//===============================================================
// 组合：入级/推进 + 冒险（只看级2）+ 入口
//===============================================================
//访存地址 = 本条指令的 (rs1 + 偏移)，以及 store 的字节使能与数据。
//★ 上游 mid_decoder 的 offset_load0_out 与 offset_store0_out 接的是【同一个信号】
//（都是 imm_alu_out），所以这一个加法器同时服务 load 与 store —— 原来把 load 那一支
//另外算了一遍、还在结果上摆 is_st 的 mux，纯属重复（最差路末端那个 LUT3 就是它）。
//offset_load0 端口因此不再使用（保留端口不动接口；综合会把那条无负载的线剪掉）。
    always @(*) begin
        addr_sum = r1_data_final + offset_store0;
        case (func10[2:0])
            3'b000: begin
                st_be = 4'b0001 << addr_sum[1:0];
                st_wdata = {24'd0, r2_data_final[7:0]} << (8 * addr_sum[1:0]);
            end
            3'b001: begin
                st_be = 4'b0011 << (2 * addr_sum[1]);
                st_wdata = {16'd0, r2_data_final[15:0]} << (16 * addr_sum[1]);
            end
            3'b010: begin
                st_be = 4'b1111;
                st_wdata = r2_data_final;
            end
            default: begin
                st_be = 4'd0;
                st_wdata = 32'd0;
            end
        endcase
    end

//本条摆上总线要用的载荷
    always @(*) begin
        is_st    = (opcode == OPCODE_STORE);
        cur_addr = addr_sum >> 2;
    end

//入口两道合法性：①非对齐（按访问宽度）②【宽度本身非法】（ld/sd 之类在 RV32 非法）。
//②由 post_decoder 判成非法指令（cause 2）；这里再挡一道是为了让它【根本不进车厢】——
//否则它会先上总线（非法宽度的 store 以 be=0000 上线、非法宽度的 load 读回垃圾还写回 rd），
//而 lsu 两级车厢在冲刷拍是【不清】的。★ 这张白名单必须与 post_decoder 的白名单一致。
    always @(*) begin
        byte_addr = addr_sum;
        exc_ldst_misalign = 1'b0;
        size_bad_now = 1'b0;
        if (mem_op) begin
            if (func10[1:0] == 2'b01) begin
                if (byte_addr[0] != 1'b0)
                    exc_ldst_misalign = 1'b1;
            end
            if (func10[1:0] == 2'b10) begin
                if (byte_addr[1:0] != 2'd0)
                    exc_ldst_misalign = 1'b1;
            end
        end
        if (opcode == OPCODE_LOAD) begin
            if (func10[2:0] == 3'b011)
                size_bad_now = 1'b1;
            if (func10[2:0] == 3'b110)
                size_bad_now = 1'b1;
            if (func10[2:0] == 3'b111)
                size_bad_now = 1'b1;
        end
        if (opcode == OPCODE_STORE) begin
            if (func10[2:0] == 3'b011)
                size_bad_now = 1'b1;
            if (func10[2:0] == 3'b100)
                size_bad_now = 1'b1;
            if (func10[2:0] == 3'b101)
                size_bad_now = 1'b1;
            if (func10[2:0] == 3'b110)
                size_bad_now = 1'b1;
            if (func10[2:0] == 3'b111)
                size_bad_now = 1'b1;
        end
    end

//让位与入口：
//  级3：store 进这一级即算提交（下一拍让位）；load 等应答
//  级2：级3 让位那拍就前移一级
//  新指令一律进步级2（级3 只有"从级2 前移"这一条路）；miss 当拍进级2 后由 blank_bus 停住不摆。
    always @(*) begin
        mem_op   = (opcode == OPCODE_LOAD) | (opcode == OPCODE_STORE);
        miss     = dcache_hold;
        s3_done  = s3_v & (s3_kind | ready_in);
        s3_ok    = ~s3_v | s3_done;
        s2_move  = s2_v & s2_sent & s3_ok;   // 没发过的级不许前移（否则请求就丢了）
        s2_ok    = ~s2_v | s2_move;
//入口门排掉 mulu 两条与取指缺失（与旧版逐位一致）。自己那三条（haz/unload/full）不算：
//车厢占用由 new_go/s2_ok 自己把关，或进来就成组合环。
//★ 不要再把 stall_rob_full(9) / stall_pc_redir(8) 也排进来：重定向要等 ROB 排空，
//  而队头那条访存正是被它挡在 lsu 门外 ⇒ ROB 永不排空 ⇒ 死锁（实测 exc_ldst_misalign 卡死）。
//★ 冲刷只挡【比边界更年轻】的错路条，边界自己必须放行：
//  · 跳转/分支非对齐：flush_idx = 分支自己 ⇒ 冲刷拍站在载荷上的后继比它年轻 ⇒ 照旧挡；
//  · 中断：flush_idx = issue_idx_in = 载荷这条自己（与本模块的 idx_in 同源）⇒ 放行。
//    它是【已被 post_decoder 发出、ROB 要留下】的那条；挡住它就永远拿不到 ld_we ⇒ ROB 排不空
//    ⇒ pc 的重定向（等 rob_empty）自锁。实测：lw 撞中断 → i2c_irq 冻在 pc=0xa4。
        ent_bnd    = flush_w & (idx_in == flush_idx);
        ent_young  = flush_w & (idx_in != flush_idx);
        new_in_pre = mem_op & ~size_bad_now & ~ent_young & ~ls_use_hit
                   & (payload_go | ent_bnd)
                   & ~(flag_bus[3] | flag_bus[2] | flag_bus[1]);
        new_in     = new_in_pre & ~exc_ldst_misalign;
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
        else begin
//blank 的拍必须把总线清零：从设备是电平判据，残留地址会被当成新请求连答（读地址只摆一拍的原因）
            bus_addr_out <= 32'd0;
            bus_data_out <= 32'd0;
            bus_be_out <= 4'd0;
            bus_we_out <= 1'b0;
            bus_valid_out <= 1'b0;
            if (bus_go && exec && !flush_w) begin
                bus_addr_out  <= s2_put ? s2_addr : cur_addr;
                bus_data_out  <= s2_put ? s2_wdat : st_wdata;
                bus_be_out    <= s2_put ? s2_be   : st_be;
                bus_we_out    <= s2_put ? s2_kind : is_st;
                bus_valid_out <= 1'b1;
            end
        end
    end

//冒险同时扫级2 与级3：级3 那笔还没被应答的 load，它的 rd 也要比。
//  ★ 以往只扫级2，依据是「级3 占着 ⇒ unload 把入口顶住 ⇒ 新指令根本进不来」（见下面三条停顿源那段），
//    即级3 的 RAW 是由 unload 的【无条件冻结】兜的，不是被检测出来的。现在让 haz 自己就能盖住两个槽，
//    这样它才是完整的 RAW 判据（unload 的对外广播撤掉后，这里是唯一保护）。
//  ★ 级3 的判据取 s3_v & ~s3_done（与 unload 同条件）：store 进级3 即 done，自动排除；
//    load 被应答那一拍 s3_done 抬起 ⇒ 不再算在途，消费者改由写口/前送去取数。
//x0 不算依赖：在途项的 rd=0（store 那一级的 rd 就是 0）与操作数 x0 都排除。
    always @(*) begin
        ls_use_hit = 1'b0;
        if (s2_v && s2_rd != 5'd0) begin
            if (r1_post != 5'd0 && (s2_rd == r1_post))
                ls_use_hit = 1'b1;
            if (r2_post != 5'd0 && (s2_rd == r2_post))
                ls_use_hit = 1'b1;
        end
        if (s3_v && ~s3_done && s3_rd != 5'd0) begin
            if (r1_post != 5'd0 && (s3_rd == r1_post))
                ls_use_hit = 1'b1;
            if (r2_post != 5'd0 && (s3_rd == r2_post))
                ls_use_hit = 1'b1;
        end
    end

//杀老写：写口级广播"某笔更年轻的同 rd 写已落地" ⇒ 本模块更老的同 rd 在途 load 置 killed。
//  killed 的记录照旧走完（总线读本来就计划要发，不引入新副作用），只是永不落地（写口用 kill_ld 门掉）。
//  ★ 年龄一律用 head 相对值 {1'b0,(idx - rob_head)}，先截 4 位再比（3 位减法回绕会判错）。
//  ★ 这里原有一条【组合版冲刷作废】判据 kyoung_s2/s3 = flush_bju_pre & (sX_age > 边界年龄)：bju 的
//    组合判定直接驱动它，于是"bju 判定锥 → 本模块杀逻辑 → kill_ld → wport 入口仲裁 → wport 组合写口
//    广播 b0_* → mulu 杀老写"串成一条跨四个模块的单拍回环（实测 12.4ns，其中走线 9.41ns 占 76%）。
//    它在【单发射、且分支不与访存同拍进流水】下恒为 0 —— bju 的装载门 adv 与 post_decoder 的载荷推进
//    只差那 3 个冲刷位、而 bju 的 flush_w 恰好就是那 3 位且优先级更高 ⇒ E4(t) ≡ E3(t-1)；又
//    flush_bju_pre ≠ 0 ⇒ E4(t) 必是 br/jalr ⇒ 不是访存 ⇒ t-1 那沿写不进 s2 ⇒ s2_idx ≤ idx_q，
//    s3 由 s2 搬来同理 ⇒ 年龄比较恒假。故整条判据已摘除，只剩文件尾 `ifdef SIM_PROBE 里的监视。
//  ★ 摘除的前提一旦被破坏（载荷/推进的相位改动，或双发射放开"分支与访存同拍"），必须把判据接回来。
    reg s2_hit, s3_hit;
    reg [3:0] s2_age, s3_age, b0_age, b1_age;
    always @(*) begin
        s2_age = {1'b0, (s2_idx - rob_head)};
        s3_age = {1'b0, (s3_idx - rob_head)};
        b0_age = {1'b0, (b0_idx - rob_head)};
        b1_age = {1'b0, (b1_idx - rob_head)};
        s2_hit = 1'b0;
        s3_hit = 1'b0;
        if (s2_v && ~s2_kind && (s2_rd != 5'd0)) begin
            if (b0_we && (b0_rd == s2_rd) && (b0_age > s2_age))
                s2_hit = 1'b1;
            if (b1_we && (b1_rd == s2_rd) && (b1_age > s2_age))
                s2_hit = 1'b1;
        end
        if (s3_v && ~s3_kind && (s3_rd != 5'd0)) begin
            if (b0_we && (b0_rd == s3_rd) && (b0_age > s3_age))
                s3_hit = 1'b1;
            if (b1_we && (b1_rd == s3_rd) && (b1_age > s3_age))
                s3_hit = 1'b1;
        end
    end

//三条停顿源各自成网（或运算交给消费者）：
//  haz    = 级2 或级3 那条 load 的结果被本条要用（load-use，两个槽都扫）
//  unload = 级3 那笔 load 还没被应答（s3_kind=1 的 store 进级即 done，不产生等待）
//  full   = 本级这条访存进不了级2（车厢满）
//★ haz 与 unload 在「级3 有在途 load」这一段上是重叠的：haz 只顶上命中 rs 的那条（精确），
//  unload 是无条件冻结（把不相关的条一起顶住）。重叠不影响正确性，且为"撤掉 unload 的对外广播"留后路。
    always @(*) begin
        stall_lsu_haz    = ls_use_hit;
        stall_lsu_unload = s3_v & ~s3_done;
        stall_lsu_full   = full_stall;
    end

//===============================================================
// 时序：两级推进
//===============================================================
    always @(posedge clk) begin
        if (rst_q) begin
            s2_v <= 1'b0;
            s3_v <= 1'b0;
            s2_kl <= 1'b0;
            s3_kl <= 1'b0;
        end
        else begin
            if (s3_done) begin                                          // 级3 走掉
                s3_v <= 1'b0;
                s3_kl <= 1'b0;
            end
            else if (s3_hit) begin                                      // 级3 留着且被广播命中
                s3_kl <= 1'b1;
            end
            else begin
                s3_kl <= s3_kl;
            end
            if (s2_v && ~s2_move && s2_hit) begin                       // 级2 留着且被广播命中
                s2_kl <= 1'b1;
            end
            else if (s2_move) begin
                s2_kl <= 1'b0;
            end
            else begin
                s2_kl <= s2_kl;
            end
            if (s2_put_go)
                s2_sent <= 1'b1;  // 补摆
            if (s2_move) begin                                          // 级2 → 级3（请求已摆过）
                s3_v <= 1'b1;
                s3_kind <= s2_kind;
                s3_rd <= s2_rd;
                s3_size <= s2_size;
                s3_off <= s2_off;
                s3_idx <= s2_idx;
                s3_kl <= s2_kl | s2_hit;                                // 杀状态跟着记录走（含"走的这拍被命中"）
                s2_v <= 1'b0;
            end
            if (new_in && new_go) begin                                 // 新指令进级2（miss 当拍也进；没摆就留着）
                s2_v <= 1'b1;
                s2_kind <= is_st;
                s2_rd <= rd_in;
                s2_size <= func10[2:0];
                s2_off <= is_st ? 2'd0 : addr_sum[1:0];
                s2_addr <= cur_addr;
                s2_wdat <= st_wdata;
                s2_be <= st_be;
                s2_idx <= idx_in;
                s2_kl <= 1'b0;
                s2_sent <= new_put;                                     // 没摆出去就保持 0，等不 blank 了再补摆
            end
        end
    end

//访存非对齐故障：判定的三样东西【寄存一拍】，并在同一块里把伴生量一起寄存（索引/读写标志/出错地址）——
//  它们必须描述【同一条指令】，所以只能在这个沿上一起采。
//★ 不能改成组合输出：故障 → controller 的 exc/flush_con_exc → flag_bus[11] → 本模块的 flush_w
//  → new_in_pre → 故障，成组合环（实测：改成组合后第一条故障彻底消失）。
//★ 判定必须与 new_in_pre（入口条件）相与，不能只看 exc_ldst_misalign_now：
//  被冲刷/停顿/冒险挡住的指令【本来就进不了 lsu】，它那一拍的 byte_addr 是错路指令的
//  垃圾操作数（实测 CoreMark：一个被冲刷的错路 lh 判出 byte_addr=0xffffffff）⇒ 对它抛
//  异常就是误判，会把好程序打断。
//★ 入口条件的口径是"错路条进不来、边界条进得来"（见 new_in_pre 那段注释）⇒ 这里跟着新口径走：
//  ① 错路条（含跳转冲刷那一拍站在载荷上的后继）ent_young=1 ⇒ new_in_pre=0 ⇒ 照旧不抛；
//  ② 边界那条（中断落在它身上）ent_bnd=1 ⇒ 放行 ⇒ 若它自己非对齐，就必须在这里抛出来
//     （否则故障静默丢失、它的 ROB 项永远等不到完成回报，又是一种自锁）。
//  访存非对齐这条路原本也不会撞上冲刷：故障要寄存一拍、controller 下一拍才收到 exc_ldst_misalign_in
//  ⇒ 故障条进 lsu 那一拍根本没有冲刷位，此时 ent_bnd=ent_young=0，new_in_pre 与原式逐位相同。
//★ 寄存一拍之后，controller 侧的 pc 载荷(aux_addr_3)与 ROB 索引(issue_idx)已经换成下一条指令，
//  所以它们也得配寄存版（见 controller 的 exc_pc_e4_q、cpu_top 的 exc_idx mux）。
    always @(posedge clk) begin
        if (rst_q) begin
            exc_ldst_misalign_out <= 1'b0;
            exc_ldst_st_out <= 1'b0;
            exc_ldst_addr_out <= 32'd0;
            exc_ldst_idx_out <= 3'd0;
        end
        else begin
//★ 伴生量必须与故障【同一个条件】采：从前是无条件跟着寄存、靠"故障恒在那一拍"的隐含对齐；
//  入口门改成"本拍推进"（payload_go）之后那一拍漂了，伴生量会采到隔壁那条指令
//  （实测：一条 store 非对齐被报成 load，mcause 记成 4）。
            exc_ldst_misalign_out <= exc_ldst_misalign & new_in_pre;
            if (exc_ldst_misalign & new_in_pre) begin
                exc_ldst_st_out   <= is_st;
                exc_ldst_addr_out <= byte_addr;
                exc_ldst_idx_out  <= idx_in;
            end
            else begin
                exc_ldst_st_out   <= exc_ldst_st_out;
                exc_ldst_addr_out <= exc_ldst_addr_out;
                exc_ldst_idx_out  <= exc_ldst_idx_out;
            end
        end
    end

//写回口保持：离开级3 那拍给数据，落在 back2 槽的消费者取不到 ⇒ 再保持一拍
    always @(posedge clk) begin
        if (rst_q)
            ld_hold <= 1'b0;
        else begin
            ld_hold <= ld_we;
            if (ld_we) begin
                ld_hold_rd <= s3_rd;
                idx_hold   <= s3_idx;
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

//写口侧的"永不落地"标记：只由本模块的 killed 记录驱动（原组合版冲刷作废已摘除，见上）
    always @(*) begin
        kill_ld = s3_kl;
        ld_we = s3_v & ~s3_kind & ready_in;
        if (ld_we) begin
            loaded = 1'b1;
            rd_load = s3_rd;
            ld_data_out = ld_data_cur;
            ld_idx = s3_idx;
        end
        else if (ld_hold) begin
            loaded = 1'b1;
            rd_load = ld_hold_rd;
            ld_data_out = ld_hold_data;
            ld_idx = idx_hold;
        end
        else begin
            loaded = 1'b0;
            rd_load = 5'd0;
            ld_data_out = bus_data_in;
            ld_idx = 3'd0;
        end
    end

    always @(*) mem_inflight = s2_v | s3_v;

//在途 load 写记录（→ 发射级互锁）
//在途 load 写记录（→ 发射级互锁）

`ifdef SIM_PROBE
//===============================================================================
// 仿真期监视（综合期整块不存在：零面积、零时序代价）
//===============================================================================
//被摘除的"组合版冲刷作废"判据（见本文件杀老写那节的注释：kyoung_s2/s3 = flush_bju_pre &
//(sX_age > 边界年龄)）在【单发射、分支不与访存同拍】下恒为 0，这里把它算回来做守卫。
//★ 必须带 s2_v/s3_v：原式没有有效性项，空槽上会被"陈旧 idx"点着 —— 实测 CoreMark 全程点着 3464 次
//  全是 s2_v=0 且 s3_v=0，而有在途项时 0 次（对空槽置 s2_kl/s3_kl 是空操作：s2_move = s2_v & …、
//  ld_we = s3_v & … 本就为 0）。所以守卫要看的正是"有在途项时会不会抬"。
//★ 它一旦抬起，说明"E4(t) ≡ E3(t-1)"与"flush_bju_pre ≠ 0 ⇒ E4(t) 非访存"这两条前提被破坏
//  （载荷/推进相位改动，或双发射放开了"分支与访存同拍"）⇒ 必须把判据接回 s2_kl/s3_kl/kill_ld。
    reg kyoung_s2, kyoung_s3, kyoung_live;
    always @(*) begin
        kyoung_s2 = flush_bju_pre & (s2_age > {1'b0, (bju_idx_q - rob_head)});
        kyoung_s3 = flush_bju_pre & (s3_age > {1'b0, (bju_idx_q - rob_head)});
        kyoung_live = (s2_v & kyoung_s2) | (s3_v & kyoung_s3);
    end
    always @(posedge clk) begin
        if (kyoung_live)
            $display("[LSU-KYOUNG] LIVE @%0t s2_v=%b s2_idx=%0d s3_v=%b s3_idx=%0d bju_idx_q=%0d rob_head=%0d",
                     $time, s2_v, s2_idx, s3_v, s3_idx, bju_idx_q, rob_head);
    end
`endif

endmodule
