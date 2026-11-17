`timescale 1ns / 1ps
// npu_mac —— INT8 MAC 计算平面（单模块版，2026-11-17）
//
// 波前对齐管线、P×Q PE 阵列（每个 PE 原样展开）、排空计时器全部并进本模块。
// 原来那四个子模块只有 npu_mac 一个使用者，端口就是内部连线。
//
// 输入路径把一个 32 位字拆成 4 个带边界屏蔽的 INT8 lane，再用 skew 管线
// 让不同的行/列在正确的周期相遇。PE 阵列只在 array_enable_in 时推进；
// 排空阶段没有新输入但仍保持 enable，让最后一个波前到达远端 PE。
//
// 本模块不做 Tile 调度，也不访问 RAM：它消费成对的 A/BT 流，直接输出
// 16 个 PE 累加器（acc_flat_out）。累加器跨 K Tile 一直保持，只在
// acc_clear_in 时清零，因此部分和不需要搬到阵列外面再搬回来。
//
// 边界补零：lane_en_in 为 0 的 lane 数据被强制为 0，越界的行/列参与乘法
// 但不影响结果（且越界 C 元素不会被写回）。
// 端口布局：
//   a_stream_in/a_lane_en_in 的第 r 位是第 r 行；bt_* 的第 c 位是第 c 列；
//   acc_flat_out 的第 (r*Q+c)*NPU_ACC_W +: NPU_ACC_W 位是 PE[r][c] 的累加器。
`include "npu_defines.vh"

module npu_mac(
    input        clk,
    input        rst,
    // 阵列控制(来自 tile_controller)
    input        acc_clear_in,     // 换输出 Tile：清累加器并冲刷残留波前
    input        array_enable_in,  // 喂数或排空期间为 1，其余时间阵列整体冻结
    input        drain_en_in,      // 排空阶段电平
    // A/BT 成对输入流(来自 tile_controller 的 FIFO 出队)
    input        stream_valid_in,
    input [31:0] a_stream_in,
    input [31:0] bt_stream_in,
    input [3:0]  a_lane_en_in,
    input [3:0]  bt_lane_en_in,
    // 结果与状态
    output reg [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_flat_out,
    output reg   drain_done_out
);

    integer i;

    //====================================================================
    //  一、输入拆包：一个 32 位字 -> 四个带边界屏蔽的 INT8 lane
    //====================================================================
    reg [7:0] a_lane  [0:3];
    reg [7:0] bt_lane [0:3];

    always @(*) begin
        for (i = 0; i < 4; i = i + 1) begin
            a_lane[i]  = a_lane_en_in[i]  ? a_stream_in[i*8 +: 8]  : 8'h00;
            bt_lane[i] = bt_lane_en_in[i] ? bt_stream_in[i*8 +: 8] : 8'h00;
        end
    end

    wire [`NPU_P*8-1:0] a_pack  = {a_lane[3],  a_lane[2],  a_lane[1],  a_lane[0]};
    wire [`NPU_Q*8-1:0] bt_pack = {bt_lane[3], bt_lane[2], bt_lane[1], bt_lane[0]};

    //====================================================================
    //  二、波前对齐（原 a_bt_skew_pipeline）
    //  A 的第 r 个 lane（行）延迟 r 拍，BT 的第 c 个 lane（列）延迟 c 拍，
    //  数据和 valid 使用完全相同的寄存器级数，A[r][k] 与 BT[c][k] 才会在
    //  PE[r][c] 的输入端同一拍到达。
    //  array_enable_in=0 时所有级保持，暂停不会丢失也不会错位波前。
    //  lane 0 直通不占寄存器；只有 lane 1..P-1 / 1..Q-1 需要 1..N-1 级寄存器。
    //====================================================================
    // [lane][stage]：lane l 只用 stage 0..l-1，输出取 stage l-1，即 l 拍延迟。
    reg [7:0] a_sh  [0:`NPU_P-1][0:`NPU_P-1];
    reg       a_shv [0:`NPU_P-1][0:`NPU_P-1];
    reg [7:0] b_sh  [0:`NPU_Q-1][0:`NPU_Q-1];
    reg       b_shv [0:`NPU_Q-1][0:`NPU_Q-1];

    integer l, s;

    always @(posedge clk) begin
        if (rst) begin
            for (l = 1; l < `NPU_P; l = l + 1)
                for (s = 0; s < l; s = s + 1) begin
                    a_sh[l][s]  <= 8'd0;
                    a_shv[l][s] <= 1'b0;
                end
            for (l = 1; l < `NPU_Q; l = l + 1)
                for (s = 0; s < l; s = s + 1) begin
                    b_sh[l][s]  <= 8'd0;
                    b_shv[l][s] <= 1'b0;
                end
        end
        else if (array_enable_in) begin
            // 数据与 valid 必须一起延迟，否则无效数据会被 PE 累加。
            for (l = 1; l < `NPU_P; l = l + 1) begin
                a_sh[l][0]  <= a_pack[l*8 +: 8];
                a_shv[l][0] <= stream_valid_in;
                for (s = 1; s < l; s = s + 1) begin
                    a_sh[l][s]  <= a_sh[l][s-1];
                    a_shv[l][s] <= a_shv[l][s-1];
                end
            end
            for (l = 1; l < `NPU_Q; l = l + 1) begin
                b_sh[l][0]  <= bt_pack[l*8 +: 8];
                b_shv[l][0] <= stream_valid_in;
                for (s = 1; s < l; s = s + 1) begin
                    b_sh[l][s]  <= b_sh[l][s-1];
                    b_shv[l][s] <= b_shv[l][s-1];
                end
            end
        end
    end

    // lane 0 旁路，lane l 取自己的第 l 级。
    reg [`NPU_P*8-1:0] a_sk;
    reg [`NPU_P-1:0]   a_sk_v;
    reg [`NPU_Q*8-1:0] bt_sk;
    reg [`NPU_Q-1:0]   bt_sk_v;

    always @(*) begin
        a_sk  = {`NPU_P*8{1'b0}};
        a_sk_v = {`NPU_P{1'b0}};
        bt_sk  = {`NPU_Q*8{1'b0}};
        bt_sk_v = {`NPU_Q{1'b0}};

        a_sk[7:0]     = a_pack[7:0];
        a_sk_v[0]     = stream_valid_in;
        bt_sk[7:0]    = bt_pack[7:0];
        bt_sk_v[0]    = stream_valid_in;

        for (l = 1; l < `NPU_P; l = l + 1) begin
            a_sk[l*8 +: 8] = a_sh[l][l-1];
            a_sk_v[l]      = a_shv[l][l-1];
        end
        for (l = 1; l < `NPU_Q; l = l + 1) begin
            bt_sk[l*8 +: 8] = b_sh[l][l-1];
            bt_sk_v[l]      = b_shv[l][l-1];
        end
    end

    //====================================================================
    //  三、P×Q PE 阵列（原 pe_array + pe_cell，逐 PE 展开）
    //  PE[r][c] 累加 C[r][c] = sum_k A[r][k] * BT[c][k]（输出驻留）。
    //  A 向右传播、BT 向下传播，每级 1 拍寄存转发。
    //  下标统一是 r*Q+c，与 acc_flat_out 的 PE 编号一致。
    //====================================================================
    wire [`NPU_PE_NUM*8-1:0]          a_fwd_w;
    wire [`NPU_PE_NUM*8-1:0]          bt_fwd_w;
    wire [`NPU_PE_NUM-1:0]            a_v_fwd_w;
    wire [`NPU_PE_NUM-1:0]            bt_v_fwd_w;
    wire [`NPU_PE_NUM*`NPU_ACC_W-1:0] acc_w;

    genvar r, c;

    generate
        for (r = 0; r < `NPU_P; r = r + 1) begin : g_pe_row
            for (c = 0; c < `NPU_Q; c = c + 1) begin : g_pe_col
                // 上游 PE 的下标；边界取 0 是为了避免生成负下标（该分支不会被选到）。
                localparam integer R_UP = (r == 0) ? 0 : (r - 1);
                localparam integer C_UP = (c == 0) ? 0 : (c - 1);

                wire [7:0] pe_a_in  = (c == 0) ? a_sk[r*8 +: 8]
                                               : a_fwd_w[(r*`NPU_Q + C_UP)*8 +: 8];
                wire       pe_a_v   = (c == 0) ? a_sk_v[r]
                                               : a_v_fwd_w[r*`NPU_Q + C_UP];
                wire [7:0] pe_bt_in = (r == 0) ? bt_sk[c*8 +: 8]
                                               : bt_fwd_w[(R_UP*`NPU_Q + c)*8 +: 8];
                wire       pe_bt_v  = (r == 0) ? bt_sk_v[c]
                                               : bt_v_fwd_w[R_UP*`NPU_Q + c];

                reg [7:0] pe_a_out;
                reg       pe_a_v_out;
                reg [7:0] pe_bt_out;
                reg       pe_bt_v_out;
                (* use_dsp = "yes" *) reg [`NPU_ACC_W-1:0] pe_acc;

                // 两路清零合成一根：rst 与 clear_acc 对累加器的效果完全一样，
                // 只是优先级不同，"或"起来以后不存在优先级问题。
                // ★ 两个不同优先级的清零会让综合器放弃 DSP48 推断（实测 DSP=0）。
                wire pe_acc_clear_now = rst | acc_clear_in;

                // ★ 乘积必须是 32 位上下文！直接写 acc + $signed(a)*$signed(b) 时，
                //   iverilog 按"乘法结果是最大操作数宽度"取 8 位再扩展 ⇒ 全错。
                wire signed [`NPU_ACC_W-1:0] pe_prod_ext = $signed(pe_a_in) * $signed(pe_bt_in);

                // 数据/valid 转发（每级 1 拍）
                always @(posedge clk) begin
                    if (rst) begin
                        pe_a_out    <= 8'd0;
                        pe_a_v_out  <= 1'b0;
                        pe_bt_out   <= 8'd0;
                        pe_bt_v_out <= 1'b0;
                    end
                    else if (acc_clear_in) begin
                        pe_a_v_out  <= 1'b0;
                        pe_bt_v_out <= 1'b0;   // 冲刷残留波前
                        pe_a_out    <= 8'd0;
                        pe_bt_out   <= 8'd0;
                    end
                    else if (array_enable_in) begin
                        pe_a_out    <= pe_a_in;
                        pe_a_v_out  <= pe_a_v;
                        pe_bt_out   <= pe_bt_in;
                        pe_bt_v_out <= pe_bt_v;
                    end
                end

                // 累加器单独一个块、只有一个清零条件；乘加写在同一句里才能折成 MACC。
                always @(posedge clk) begin
                    if (pe_acc_clear_now)
                        pe_acc <= {`NPU_ACC_W{1'b0}};
                    else if (array_enable_in && pe_a_v && pe_bt_v)
                        pe_acc <= pe_acc + pe_prod_ext;
                end

                assign a_fwd_w  [(r*`NPU_Q + c)*8 +: 8]      = pe_a_out;
                assign a_v_fwd_w [r*`NPU_Q + c]              = pe_a_v_out;
                assign bt_fwd_w [(r*`NPU_Q + c)*8 +: 8]      = pe_bt_out;
                assign bt_v_fwd_w [r*`NPU_Q + c]             = pe_bt_v_out;
                assign acc_w     [(r*`NPU_Q + c)*`NPU_ACC_W +: `NPU_ACC_W] = pe_acc;
            end
        end
    endgenerate

    //====================================================================
    //  四、排空计时（原 drain_controller）
    //  最后一对输入送进阵列后，波前还要走 P+Q-2 个 enable 拍才能到达最远端的 PE；
    //  这段时间里不能再换输出 Tile，否则尾部乘积会落到下一块结果里。
    //
    //  · 为什么第 DRAIN_BEATS+1 拍才拉高：cnt_q 在第 1 个 enable 拍是 0，之后每拍
    //    加一，比较 cnt_q == DRAIN_BEATS-1 让 drain_done_out 落在第 DRAIN_BEATS+1 拍。
    //    这一拍不是冗余 —— 上层正是在 drain_done 那一拍把累加器采进快照，而最远端
    //    PE[P-1][Q-1] 的最后一个乘积恰好在这一拍的前一个时钟沿才写进累加器；
    //    提前一拍采数只错远角元素（实测 C[3][3] 一类值变小）。
    //  · drain_done_out 是电平不是单拍脉冲，上层必须"只消费一次"。
    //====================================================================
    localparam [3:0] DRAIN_BEATS = `NPU_P + `NPU_Q - 2;

    reg [3:0] drain_cnt_q;

    always @(posedge clk) begin
        if (rst) begin
            drain_cnt_q    <= 4'd0;
            drain_done_out <= 1'b0;
        end
        else if (!drain_en_in) begin
            drain_cnt_q    <= 4'd0;
            drain_done_out <= 1'b0;
        end
        else if (!drain_done_out) begin
            if (drain_cnt_q == DRAIN_BEATS - 4'd1)
                drain_done_out <= 1'b1;
            else
                drain_cnt_q <= drain_cnt_q + 4'd1;
        end
    end

    //====================================================================
    //  五、输出
    //====================================================================
    always @(*) begin
        acc_flat_out = acc_w;
    end

endmodule
