`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: iffu
// Description:
//   夹在 icache 与 idu 之间的取指队列。
//
//   条目只存"队列取消不了的携带量"：指令字 + 地址 + 分支预测位 + jalr 预测目标（97 位）。
//   rd / func10 / imm 是纯函数，一律不在队里存。
//
//   ★ 2026-11-08：icache 一拍交付【16 字节块 = 4 条】。本模块一次最多收 4 条：
//     - block_offset 之前的那几条【不进队】（改向落在块中间时，它们是改向前的老指令）；
//     - 块内第一条被预测跳的分支之后的那几条也不进队（它们是错路）；
//     - 所以一次真正推进的条数是 1..4，写指针按它走。
//
//   两个推进门分开：
//     fetch_request = exec & ~(irq_kill | redir_frontend) & ~full     （取指侧：收一块）
//     advance                                          （读侧，idu 给，0/1/2）
//   ★ 取指侧只有一根流控：full。不 full 就收一块，空泡（指令字全 0）就地丢掉、不写队列。
//     icache 那边的契约是"每拍都往口上放一样东西 —— 新指令或 NOP；只有 full 的时候保持"，
//     所以不会出现同一条真指令连摆几拍、被重复收下的情形。
//
//   队头是组合读：空队列时压成空泡（不做"本拍写入旁路"）。它只作出队格 h0 的 D 端。
//   出队格 h0/h1 是本级的【唯一】输出寄存器：rob 的分配口/入队扫描口与 idu 的译码输入
//   都取它们的 Q 端 —— 同一条指令的数据在一拍上同时发给 rob 与 idu。
//
//   取指侧的改向信息【不在这里产生】：br/jal/jalr 的落点由 bju 判定、由 bpu 的表给出。
//   在 iffu 里译码会成环：它当拍的输入会被它自己的输出改内容。
//////////////////////////////////////////////////////////////////////////////////


module iffu(
    input clk, rst,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input        redir_frontend,     // 前端重定向（rob 判定拍出的那根）
    input        irq_kill,           // 中断受理（clint 自己判的）
    input        exec_in,            // = exec_in
//icache 交付口：一块 2 条，每条自带有效位（已含"块内偏移之前不算"）
    input [31:0] instruction_0, instruction_1,
    input        inst_valid_0, inst_valid_1,
//块内起始偏移：和四条指令同一个寄存器组交付。写指针的落点要减掉它。
    input [1:0]  block_offset,
//★ 错路作废：与这一块指令同组同门的边带，为 1 时整块不进队。
    input        slot_replay,
//随指令携带的量：一块四份 —— 地址（架构 PC 口径，原样存）、预测"跳"位、jalr 预测目标
    input [31:0] push_address_0, push_address_1,
    input        push_branch_prediction_0, push_branch_prediction_1,
    input [31:0] push_jump_link_register_prediction_0, push_jump_link_register_prediction_1,
//读侧：本拍出队格换不换内容（= idu 的推进条件，与它逐字同形）
//★ 2026-11-09：2 位 → 3 位（分配口 2 宽 → 3 宽，见 docs/SPEC_alloc3.md）
    input [2:0]  advance,
//出队格：本级的唯一输出寄存器，rob 与 idu 都在它的 Q 端取数
    output reg [31:0] half0_instruction, half0_address, half0_jump_link_register_prediction,
    output reg        half0_branch_prediction,
    output reg [31:0] half1_instruction, half1_address, half1_jump_link_register_prediction,
    output reg        half1_branch_prediction,
//★ 2026-11-09：第三个出队口（分配口 3 宽）
    output reg [31:0] half2_instruction, half2_address, half2_jump_link_register_prediction,
    output reg        half2_branch_prediction,
//出队格的有效位（与本格其它字段同沿寄存）。★ 必须给 idu：advance 是本级的"弹几条"请求，
//  这里按有效条数截断；idu 若不知道 h1 有没有，就会把空白格也装进 lane1 ⇒ 同一条被取两次。
    output reg        half0_valid, half1_valid, half2_valid,
//取指侧停：本拍放不下，icache 与 pc 的推进被它按住
    output reg        full
    );

//复位就地打一拍（与其它模块同款）
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//本级的两个判据：冲刷（中断受理 / 前端改向）、取指推进
    reg fetch_request;
    reg [31:0] instruction_effective_0, instruction_effective_1;
    reg        lane_valid_0, lane_valid_1;
    always @(*) begin
        fetch_request = exec_in & ~(irq_kill | redir_frontend) & ~full;
    end

//★ 块内截断：第一条被预测跳的分支之后的那几条是错路，不许进队。
    reg word_ok_0, word_ok_1;
    always @(*) begin
        word_ok_0 = inst_valid_0 & (|instruction_0);
        word_ok_1 = inst_valid_1 & (|instruction_1);
    end

//★ 块内截断：第 0 条被预测跳 ⇒ 第 1 条是错路，不许进队。
    reg stop_after_0;
    always @(*) begin
        stop_after_0 = word_ok_0 & push_branch_prediction_0;
    end

    reg push_ok_0, push_ok_1;
    always @(*) begin
        push_ok_0 = word_ok_0;
        push_ok_1 = word_ok_1 & ~stop_after_0;
    end

    always @(*) begin
        if (slot_replay) begin
            instruction_effective_0 = 32'd0;
            instruction_effective_1 = 32'd0;
            lane_valid_0 = 1'b0;
            lane_valid_1 = 1'b0;
        end
        else begin
            instruction_effective_0 = push_ok_0 ? instruction_0 : 32'd0;
            instruction_effective_1 = push_ok_1 ? instruction_1 : 32'd0;
            lane_valid_0 = push_ok_0;
            lane_valid_1 = push_ok_1;
        end
    end

//★ 源寄存器号（rs1/rs2）的译码已搬到 idu（2026-10-31）。

    parameter integer DEPTH = 16;  // ★宏化旋钮：16=大核 / 8=小核（块 4 宽，深度按 4 的倍数取）
    localparam integer PTR_W = (DEPTH == 32) ? 6 : (DEPTH == 16) ? 5 : (DEPTH == 8) ? 4 : 3;
    localparam integer IDX_W = PTR_W - 1;

//       [96:65] inst   [64:33] addr   [32] br_pred   [31:0] jalr_pred
//★ 4 宽取指后本阵列有【4 个写口】（每条 lane 一个 always 块）。Vivado 推不出 BRAM 时
//  不会退回触发器，而是直接报 `Synth 8-2913 Unsupported Dual Port Block-RAM template`
//  把综合打死（2026-11-08 远端实测）。必须显式钉成触发器。
    (* ram_style = "registers" *) reg [96:0] queue_data [0:DEPTH-1];
    reg [PTR_W-1:0] write_pointer, read_pointer;
    reg [PTR_W-1:0] slot_index;
    reg [PTR_W-1:0] count_in_use;
    reg [PTR_W-1:0] room_available;
    reg [PTR_W-1:0] block_instruction_count;
    reg [PTR_W-1:0] slot_0, slot_1;
    reg [2:0]  pop;
    reg [96:0] push_data_0, push_data_1, stage0_data, stage1_data, stage2_data;
    reg         stage0_valid, stage1_valid, stage2_valid;
    reg [31:0]  next_instruction, next_address, next_jump_link_register_prediction;
    reg [31:0]  next_lane1_instruction, next_lane1_address, next_lane1_jump_link_register_prediction;
    reg         next_branch_prediction, next_lane1_branch_prediction;
    reg [31:0]  next_lane2_instruction, next_lane2_address, next_lane2_jump_link_register_prediction;
    reg         next_lane2_branch_prediction;
    reg        write_back_0, write_back_1;

//写入的那几条（布局见上）。addr 是"指令地址 + 4"口径的 aux_addr，原样存。
    always @(*) begin
        push_data_0 = {instruction_effective_0, push_address_0, push_branch_prediction_0, push_jump_link_register_prediction_0};
        push_data_1 = {instruction_effective_1, push_address_1, push_branch_prediction_1, push_jump_link_register_prediction_1};
    end

    always @(*) begin
//★ 占用按 PTR_W 位算，不能取低 IDX_W 位：DEPTH 本身必须可表示。
//  取低 IDX_W 位时"满 DEPTH 条"会读成 0（D=8 时 8 的低 3 位 = 0）⇒ 满队看着是空队 ⇒
//  继续猛推、覆盖还没交付的指令。IFFU_DEPTH=8 下 rand seed1 当场复现（wp=8 rp=0 读成 0）。
        count_in_use  = write_pointer - read_pointer;
//★ 本拍还能收几条，并且永远留一格不进（上界 DEPTH-1）：
//  一旦让 wp 追平 rp，"满"和"空"就没法用指针比较区分了（stage0_valid 正是靠它）。
//  不变量：count_in_use <= DEPTH-1 由下面的 room_available 逐拍保证。
        room_available = (DEPTH - 1) - count_in_use;
    end

//full 是整条前端唯一的流控：它一出来，pc 不推进、icache 保持输出、本队不收。
//★ 一次最多收 4 条 ⇒ 拉起这一支必须是【组合】的（按当拍 room_available），寄存版会晚一拍就溢出了。
    reg full_registered;
    always @(*) begin
        full = (room_available < block_instruction_count) |
               (full_registered & (~|room_available));
    end

    always @(posedge clk) begin
        if (reset_registered) full_registered <= 1'b0;
        else full_registered <= full;
    end

    always @(*) begin
//本块实际要收的条数（lane_valid 是前缀向量，求和即可）。
        block_instruction_count = {{(PTR_W-1){1'b0}}, lane_valid_0}
                                + {{(PTR_W-1){1'b0}}, lane_valid_1};
    end

    always @(*) begin
//★ 整块要么全收、要么一条不收：pc/icache 每拍推进一整块，收一半就等于把块尾那几条扔了。
//  所以容量判据按【本块条数】算，不做逐条削。
        write_back_0 = fetch_request & lane_valid_0;
        write_back_1 = fetch_request & lane_valid_1;
//★ 落点按【已推入的条数】压紧，不按 lane 序号：块内可能既有 block_offset 之前的、
//  也可能中间夹着空泡（指令字全 0 不推），留洞会让出队格读到没写过的格子。
//  lane_valid_* 已经吸收了 block_offset（inst_valid_* 里就带），所以这里只数前缀。
        slot_0 = write_pointer;
        slot_1 = write_pointer + {{(PTR_W-1){1'b0}}, lane_valid_0};
    end

//写口：一个 always、每格一条语句。
//★ 不能写成"每条 lane 一个 always 块"：那样同一 bit 会被四个块各驱动一次，综合出来是
//  四个寄存器并联到同一根网 ⇒ `DRC MDRV-1 Multiple Driver Nets`（2026-11-08 远端实测，
//  queue_data[0][0] 有 4 个驱动）。按格索引、单条赋值，才是寄存器堆该有的形状。
//  （isq.v 的发射站阵列也是这个写法。）
    integer queue_index;
    always @(posedge clk) begin
        for (queue_index = 0; queue_index < DEPTH; queue_index = queue_index + 1) begin
            if      (write_back_0 & (slot_0[IDX_W-1:0] == queue_index[IDX_W-1:0]))
                queue_data[queue_index] <= push_data_0;
            else if (write_back_1 & (slot_1[IDX_W-1:0] == queue_index[IDX_W-1:0]))
                queue_data[queue_index] <= push_data_1;
        end
    end

//写指针
    always @(posedge clk) begin
        if (reset_registered) begin
            write_pointer <= {PTR_W{1'b0}};
        end
        else if (irq_kill | redir_frontend) begin
            write_pointer <= read_pointer;
        end
        else begin
            write_pointer <= write_pointer
                           + {{(PTR_W-1){1'b0}}, write_back_0}
                           + {{(PTR_W-1){1'b0}}, write_back_1};
        end
    end

//本拍实际弹几条 = min(下一级的请求, 消费者手上真正有效的条数)。
//advance 是 idu 按自己的 free 算出来的【请求】（0/1/2/3），不是保证：队里没有那么多就给多少。
    always @(*) begin
        if (!exec_in | ~half0_valid)  pop = 3'd0;
        else if (advance == 3'd0) pop = 3'd0;
        else if (!half1_valid)    pop = 3'd1;
        else if (advance == 3'd1) pop = 3'd1;
        else if (!half2_valid)    pop = 3'd2;
        else                      pop = advance;
    end

//读指针：清队时不动（那些条目由 write_pointer 作废）；正常按实际弹出的条数（0..3）走
    always @(posedge clk) begin
        if (reset_registered) begin
            read_pointer <= {PTR_W{1'b0}};
        end
        else if (irq_kill | redir_frontend) begin
            read_pointer <= read_pointer;
        end
        else begin
            read_pointer <= read_pointer + {{(PTR_W-3){1'b0}}, pop};
        end
    end

//出队格的装载源 = 弹完之后的新队头
    always @(*) begin
        slot_index = read_pointer + {{(PTR_W-3){1'b0}}, pop};
    end

//不在册时整条压 0：不让陈旧值/X 走到译码、冒险比较和下游载荷里。
//★ 有效性按【这一拍的指针】算（slot_index != write_pointer）：queue_data 的读拿到的是沿前的旧值，
//  这一拍刚写进来的那一格还不是新数据 —— 拿它当有效会把陈旧值交付出去。
    always @(*) begin
        stage0_data = queue_data[slot_index[IDX_W-1:0]];
        stage0_valid  = ~reset_registered & (slot_index != write_pointer);
        next_instruction      = 32'd0;
        next_address      = 32'd0;
        next_branch_prediction   = 1'b0;
        next_jump_link_register_prediction = 32'd0;
        if (stage0_valid) begin
            next_instruction      = stage0_data[96:65];
            next_address      = stage0_data[64:33];
            next_branch_prediction   = stage0_data[32];
            next_jump_link_register_prediction = stage0_data[31:0];
        end
    end

    always @(*) begin
        stage1_data = queue_data[slot_index[IDX_W-1:0] + 1'b1];
        stage1_valid  = ~reset_registered & (slot_index != write_pointer)
                                     & ((slot_index + 1'b1) != write_pointer);
        next_lane1_instruction      = 32'd0;
        next_lane1_address      = 32'd0;
        next_lane1_branch_prediction   = 1'b0;
        next_lane1_jump_link_register_prediction = 32'd0;
        if (stage1_valid) begin
            next_lane1_instruction      = stage1_data[96:65];
            next_lane1_address      = stage1_data[64:33];
            next_lane1_branch_prediction   = stage1_data[32];
            next_lane1_jump_link_register_prediction = stage1_data[31:0];
        end
    end

//出队格的装载：换新 / 保持 / 清。保持条件与下一级的推进条件同门（advance 就是下一级给的）。
//装载源取本级组合读出的队头（不在册时已压成 0）。
    always @(*) begin
        stage2_data = queue_data[slot_index[IDX_W-1:0] + 2'd2];
        stage2_valid  = ~reset_registered & (slot_index != write_pointer)
                                     & ((slot_index + 1'b1) != write_pointer)
                                     & ((slot_index + 2'd2) != write_pointer);
        next_lane2_instruction      = 32'd0;
        next_lane2_address      = 32'd0;
        next_lane2_branch_prediction   = 1'b0;
        next_lane2_jump_link_register_prediction = 32'd0;
        if (stage2_valid) begin
            next_lane2_instruction      = stage2_data[96:65];
            next_lane2_address      = stage2_data[64:33];
            next_lane2_branch_prediction   = stage2_data[32];
            next_lane2_jump_link_register_prediction = stage2_data[31:0];
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            half0_instruction <= 32'd0;
            half0_address <= 32'd0;
            half0_branch_prediction <= 1'b0;
            half0_jump_link_register_prediction <= 32'd0;
            half0_valid <= 1'b0;
        end
        else if (exec_in) begin
            if (irq_kill | redir_frontend) begin
                half0_instruction <= 32'd0;
                half0_address <= 32'd0;
                half0_branch_prediction <= 1'b0;
                half0_jump_link_register_prediction <= 32'd0;
                half0_valid <= 1'b0;
            end
//★ 不再用 advance 门控：队头指针会因为"这一拍有人推入 / 弹出 0/1/2 条"而变，
//  必须每拍重新装。旧结构只在 advance 那一拍装、且从当前 read_pointer 取，弹 1 条时会错位。
            else begin
                half0_instruction <= next_instruction;
                half0_address <= next_address;
                half0_branch_prediction <= next_branch_prediction;
                half0_jump_link_register_prediction <= next_jump_link_register_prediction;
                half0_valid <= stage0_valid;
            end
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            half1_instruction <= 32'd0;
            half1_address <= 32'd0;
            half1_branch_prediction <= 1'b0;
            half1_jump_link_register_prediction <= 32'd0;
            half1_valid <= 1'b0;
        end
        else if (exec_in) begin
            if (irq_kill | redir_frontend) begin
                half1_instruction <= 32'd0;
                half1_address <= 32'd0;
                half1_branch_prediction <= 1'b0;
                half1_jump_link_register_prediction <= 32'd0;
                half1_valid <= 1'b0;
            end
            else begin
                half1_instruction <= next_lane1_instruction;
                half1_address <= next_lane1_address;
                half1_branch_prediction <= next_lane1_branch_prediction;
                half1_jump_link_register_prediction <= next_lane1_jump_link_register_prediction;
                half1_valid <= stage1_valid;
            end
        end
    end

//★ 2026-11-09：第三个出队口，与前两个逐字同形（队头 +2）
    always @(posedge clk) begin
        if (reset_registered) begin
            half2_instruction <= 32'd0;
            half2_address <= 32'd0;
            half2_branch_prediction <= 1'b0;
            half2_jump_link_register_prediction <= 32'd0;
            half2_valid <= 1'b0;
        end
        else if (exec_in) begin
            if (irq_kill | redir_frontend) begin
                half2_instruction <= 32'd0;
                half2_address <= 32'd0;
                half2_branch_prediction <= 1'b0;
                half2_jump_link_register_prediction <= 32'd0;
                half2_valid <= 1'b0;
            end
            else begin
                half2_instruction <= next_lane2_instruction;
                half2_address <= next_lane2_address;
                half2_branch_prediction <= next_lane2_branch_prediction;
                half2_jump_link_register_prediction <= next_lane2_jump_link_register_prediction;
                half2_valid <= stage2_valid;
            end
        end
    end

endmodule
