`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: iffu
// Description:
//   夹在 icache 与 idu1 之间的取指队列（单发射版）。
//
//   条目只存"队列取消不了的携带量"：指令字 + 地址 + 分支预测位 + jalr 预测目标 +
//   两个源寄存器号（107 位，深度 8）。rd / func10 / imm 是纯函数，一律不在队里存。
//
//   两个推进门分开：
//     fetch_request = exec & ~flush_window & ~full                                    （取指侧：收一条）
//     advance                                                               （读侧，idu1 给）
//   ★ 取指侧只有一根流控：full。不 full 就收一条，空泡（指令字全 0）就地丢掉、不写队列。
//     icache 那边的契约是"每拍都往口上放一样东西 —— 新指令或 NOP；只有 full 的时候保持"，
//     所以不会出现同一条真指令连摆几拍、被重复收下的情形。
//   ★ 因此这里不需要 take_en/deliv_v 那套握手，也不需要"上一拍那条指令"的锁存。
//
//   队头是组合读：空队列时压成空泡（不做"本拍写入旁路"），它只作出队格 h0 的 D 端。
//   ★ 旁路"队空时队头直接是本拍写进去的那条"曾经在，实测是时序墙：它把
//     icache 的 BRAM 输出一路组合送到 rob 的 ent_h2 D 端（11.0ns / 9 级 / 67% 布线），
//     一去掉失败端点 293→6、WNS -1.175→-0.377，代价只有 +1.3% 拍数。
//   出队格 h0 是本级的【唯一】输出寄存器：rob 的分配口/入队扫描口与 idu1 的译码输入
//   都取它的 Q 端 —— 同一条指令的数据在一拍上同时发给 rob 与 idu1。ROB 号不再随本级的
//   内容锁存（由 rob 直接发 idu1 的输出级），所以出队格里不存号、也无 nh0_* 那一组
//   组合的队头输出。
//
//   取指侧的改向信息【不在这里产生】：br/jal/jalr 的落点由 bju 判定、由 bpu 的表给出，
//   本模块只在"表刚改向、而那一槽已经被顺延取走"时把那一对整个作废（下面那一次 mux）。
//   在 iffu 里译码会成环：它当拍的输入会被它自己的输出改内容。
//////////////////////////////////////////////////////////////////////////////////


module iffu(
    input clk, rst,
//★ 2026-10-31：只接本模块真正用到的位（原来整根八位总线扇进来，其中好几位没人读）。
    input [2:0] flush_control_in,   // = flag_bus[7:5]：{异常, 中断, 改向}
    input       exec_in,            // = exec_in
//取指重定向（pc.v 的 redir_go 落地那拍）：整队作废
    input        flush_pc_redirect,
//icache 交付口：两条 lane 各一条，有效位为 0 时在本模块前端挡成空泡
    input [31:0] instruction_in_0, inst_in_1,
    input        instruction_valid_0, inst_valid_1,
//pc 这一拍是不是按 lane0 的落点改了向（= 交付的这一对里 lane1 在错路上）
    input        lane0_take,
//★ 错路作废（边带，与这一对指令同生共死 —— 由 cpu_top 在交付拍锁存，
//  队满把这批按住时它也一起被按住，所以队满的那几拍之后仍然描述的是同一对）：
//    slot_replay = 这一对整体作废（上一对被预测改向 ⇒ 这一对是顺延取回来的错路）
    input        slot_replay,
//随指令携带的量：两条 lane 各一份 —— 地址（架构 PC 口径，原样存）、预测"跳"位、jalr 预测目标
    input [31:0] push_address_0, push_addr_1,
    input        push_branch_prediction_0, push_br_pred_1,
    input [31:0] push_jump_link_register_prediction_0, push_jalr_pred_1,
//读侧：本拍出队格换不换内容（= idu1 的推进条件，与它逐字同形）
    input [1:0]  advance,
//出队格：本级的唯一输出寄存器，rob 与 idu1 都在它的 Q 端取数
    output reg [31:0] half0_instruction, half0_address, half0_jump_link_register_prediction,
    output reg        half0_branch_prediction,
    output reg [31:0] half1_instruction, half1_address, half1_jump_link_register_prediction,
    output reg        half1_branch_prediction,
//出队格的有效位（与本格其它字段同沿寄存）。★ 必须给 idu1：advance 是本级的"弹几条"请求，
//  这里按有效条数截断；idu1 若不知道 h1 有没有，就会把空白格也装进 lane1 ⇒ 同一条被取两次。
    output reg        half0_valid, half1_valid,
//取指侧停：本拍放不下，icache 与 pc 的推进被它按住
    output reg        full
    );

//复位就地打一拍（与其它模块同款）
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//flag_bus 逐位还原（口径与各消费者一致：冲刷压停顿）
    reg flush_control_exception, flush_control_interrupt, flush_control_jump, exec;
    reg flush_window, fetch_request;
    reg fetch_nop_0, fetch_nop_1, fetch_pair, push_lane0, push_lane1, lane0_redirect, lane1_valid;
    reg [31:0] instruction_effective_0, instruction_effective_1;
    always @(*) begin
        flush_control_exception     = flush_control_in[2];
        flush_control_interrupt     = flush_control_in[1];
        flush_control_jump    = flush_control_in[0];
        exec              = exec_in;
        flush_window = flush_control_exception | flush_control_interrupt | flush_control_jump;
        fetch_request = exec & ~flush_window & ~full;
    end

    always @(*) begin
        fetch_nop_0 = ~|instruction_effective_0;
        fetch_nop_1 = ~|instruction_effective_1;
        fetch_pair = lane1_valid & ~lane0_redirect;
        push_lane0 = fetch_request & ~fetch_nop_0;
        push_lane1 = fetch_request & fetch_pair & ~fetch_nop_1;
    end

//RV32I 里与源寄存器号有关的 opcode（改向那些已经不在本模块了）
    localparam OPCODE_OP     = 7'b0110011;
    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_SYSTEM = 7'b1110011;

//==============================================================================
//  错路作废（本模块唯一一处"改交付内容"的地方）
//    pc 按落点表改向的那一拍，取指槽已经按顺延地址发出去了（地址寄存器改不动），
//    所以下一拍交付的这一对是【错路】，整对作废（白取一对，功能正确）。
//    ★ slot_replay 是【随指令走】的边带，由 cpu_top 在交付拍锁存，这里直接用：
//      队满把这批按住时边带一起被按住，所以「按住几拍再放行」之后它描述的仍是同一对。
//      在这里自己寄存一拍是不行的 —— 那会把队满时的作废标记错位到下一对上去。
//==============================================================================
    always @(*) begin
        if (slot_replay) begin
            instruction_effective_0 = 32'd0;
            instruction_effective_1 = 32'd0;
            lane1_valid = 1'b0;
        end
        else begin
            instruction_effective_0 = instruction_valid_0 ? instruction_in_0 : 32'd0;
            instruction_effective_1 = inst_valid_1 ? inst_in_1 : 32'd0;
            lane1_valid = inst_valid_1;
        end
    end

//★ lane0_redirect 就是 pc 这一拍【实际照做】的那一位，不再自己拼"译码是分支 & 预测跳"——
//  拼出来的第二份预测只要和 pc 的那份不一致，lane1 就会在错路上被放进来、或者对的那条被挡掉。
    always @(*) begin
        lane0_redirect = lane0_take;
    end

//★ 源寄存器号（rs1/rs2）的译码已搬到 idu1（2026-10-31）。
//  原来这里对 instruction_effective 做一遍 opcode case，把一个 5 位结果拼进 107 位队列包；
//  而 idu1 对同一个指令字本来就有自己的 opcode case（decode 块）⇒ 同一份译码做了两遍，
//  并且压在【队列写数据】的锥上。搬走之后本级的写口只剩拼接，队列包 107 → 97 位。
//  slot_replay 的作废不用补偿：本模块在推入【前】就把 instruction_effective 清 0，
//  队列里存的那条本来就是 0 ⇒ idu1 解出来的 rs 自然是 0。

    localparam [3:0] DEPTH = 4'd4;

//       [96:65] inst   [64:33] addr   [32] br_pred   [31:0] jalr_pred
    reg [96:0] queue_data [0:3];
    reg [3:0]  write_pointer, read_pointer;
    reg [3:0]  slot_index;
    reg [3:0]  count_in_use;
    reg [1:0]  pop;
    reg [96:0] push_lane0_data, push_lane1_data, stage0_data, stage1_data;
    reg         stage0_valid, stage1_valid;
    reg [31:0]  next_instruction, next_address, next_jump_link_register_prediction;
    reg [31:0]  next_lane1_instruction, next_lane1_address, next_lane1_jump_link_register_prediction;
    reg         next_branch_prediction, next_lane1_branch_prediction;
    reg        write_back_0, write_back_1, clear;

//写入的那一条（布局见上）。addr 是"指令地址 + 4"口径的 aux_addr，原样存。
    always @(*) begin
        push_lane0_data = {instruction_effective_0, push_address_0, push_branch_prediction_0, push_jump_link_register_prediction_0};
        push_lane1_data = {instruction_effective_1, push_addr_1, push_br_pred_1, push_jalr_pred_1};
    end

    always @(*) begin
        count_in_use  = write_pointer - read_pointer;
    end

//full 是整条前端唯一的流控：它一出来，pc 不推进、icache 保持输出、本队不收。
//拉起在 7：成对推要两格，6 的时候再推一对正好到 8，7 就放不下了。
//放下在 4：迟滞，免得在 6/7 之间来回抖。
//拉起这一支必须是【组合】的（按当拍 count_in_use），寄存版会晚一拍，7 那一拍还能再推一对就溢出了；
//只有"保持到 4 才放"这一支用寄存器。
    reg full_registered, full_norm;
    always @(*) begin
        full_norm = (count_in_use >= 4'd3) | (full_registered & (count_in_use > 4'd2));
        full      = full_norm;
    end
    always @(posedge clk) begin
        if (reset_registered) full_registered <= 1'b0;
        else       full_registered <= full;
    end

    always @(*) begin
        clear = flush_window | flush_pc_redirect;
    end

//★ 曾在这里加过一层"同一地址不许连推两次"的去重（lp_a/lp_v），用来压住 icache 重放
//  （miss 期间输出寄存器保持 ⇒ 同一个包被再推一次）。那层东西【必须撤掉】：
//  正确的预测跳转（bne 回边）**不产生冲刷**，而循环的回边目标往往
//  就是刚刚推过的地址 ⇒ 去重把合法的重取也一并压掉，整核在回边处自锁
//  （实测 WEIRD seed5：pc 卡在回边后的顺延地址、iffu 空、icache 口上有 NOP、rd_go=0）。
//  重放的根子在别处（half0_valid/half1_valid 与装载门修掉之后，默认档与 WEIRD 档都不再需要它）。
    always @(*) begin
        write_back_0 = fetch_request & push_lane0;
        write_back_1 = fetch_request & push_lane1;
    end

//写口
    always @(posedge clk) begin
        if (write_back_0)
            queue_data[write_pointer[1:0]] <= push_lane0_data;
        else if (write_back_1)
            queue_data[write_pointer[1:0]] <= push_lane1_data;
        if (write_back_0 & write_back_1)
            queue_data[write_pointer[1:0] + 2'd1] <= push_lane1_data;
    end

//写指针
    always @(posedge clk) begin
        if (reset_registered) begin
            write_pointer <= 4'd0;
        end
        else if (clear) begin
            write_pointer <= read_pointer;
        end
        else if (write_back_0) begin
            write_pointer <= write_pointer + (write_back_1 ? 4'd2 : 4'd1);
        end
        else if (write_back_1) begin
            write_pointer <= write_pointer + 4'd1;
        end
    end

//本拍实际弹几条 = min(下一级的请求, 消费者手上真正有效的条数)。
//advance 是 idu1 按自己的 free 算出来的【请求】（0/1/2），不是保证：队里没有那么多就给多少。
    always @(*) begin
//★ 2026-10-31：[H[2J[3J 从 pop 的判据里摘掉 —— 冲刷不该管"弹不弹"。
//  冲刷拍弹出来的条目一律作废（各寄存器块末尾的冲刷清后写者赢），指针照常走。
        if (!exec | ~half0_valid)  pop = 2'd0;
        else if (advance == 2'd0) pop = 2'd0;
        else if (!half1_valid)           pop = 2'd1;
        else                      pop = advance;
    end

//读指针：清队时不动（那些条目由 write_pointer 作废）；正常按实际弹出的条数（0/1/2）走
    always @(posedge clk) begin
        if (reset_registered) begin
            read_pointer <= 4'd0;
        end
        else if (clear) begin
            read_pointer <= read_pointer;
        end
        else begin
            read_pointer <= read_pointer + {2'b00, pop};
        end
    end

//出队格的装载源 = 弹完之后的新队头
    always @(*) begin
        slot_index = read_pointer + {2'b00, pop};
    end

//不在册时整条压 0：不让陈旧值/X 走到译码、冒险比较和下游载荷里。
//★ 有效性按【这一拍的指针】算（slot_index != write_pointer）：queue_data 的读拿到的是沿前的旧值，
//  这一拍刚写进来的那一格还不是新数据 —— 拿它当有效会把陈旧值交付出去。
    always @(*) begin
        stage0_data = queue_data[slot_index[1:0]];
//★ 同上：stage*_valid 不再吃 clear。reason：clear 是"整队作废"，不是"本拍没有"。
//  留在这里等于把冲刷网又接回 idu1 的推进锥（valid → alloc_lane → advance）。
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
        stage1_data = queue_data[slot_index[1:0] + 2'd1];
        stage1_valid  = ~reset_registered & (slot_index != write_pointer) & ((slot_index + 4'd1) != write_pointer);
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
    always @(posedge clk) begin
        if (reset_registered) begin
            half0_instruction <= 32'd0;
            half0_address <= 32'd0;
            half0_branch_prediction <= 1'b0;
            half0_jump_link_register_prediction <= 32'd0;
            half0_valid <= 1'b0;
        end
        else if (exec) begin
            if (flush_window) begin
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
        else if (exec) begin
            if (flush_window) begin
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

endmodule
