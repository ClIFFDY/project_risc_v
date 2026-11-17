`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: wdt
// Description: 看门狗（APB 10 号槽，0x2800_0000）
//
//   为什么独立成一个外设、不塞进 TIM：TIM 是 FreeRTOS 的 tick 源（100 Hz），
//   而看门狗要管的正好是"系统停摆" —— 两者不能互相依赖。
//
//   两级超时（标准做法）：
//     一级：COUNT 归零 → 拉 wdt_irq（PLIC 源 10），软件还有机会留遗言/落盘
//     二级：再数 HOLD 毫秒 → 拉 wdt_reset，整核复位
//   只开一级（CTRL[2]=0）时，一级到期直接拉复位。
//
//   ★ 复位输出是一根 16 拍的电平脉冲，之后本模块自己回到【关闭】态，
//     等软件重新使能 —— 复位撤销后不能自己又跑起来，否则会变成复位振荡器。
//   ★ 本模块的 rst 输入【不能】含 wdt_reset 自己（否则一拉起来就把自己清了，
//     只剩一两拍）。接的是 soc/bus_con_top 里的 rst_wdt = rst_stable | soft_reset。
//
//   护栏：
//     · 复位后默认【关】。不默认关的话 bios 一进来就可能把板子看死。
//     · CTRL 不能直接写 —— 要写 LOCK = 0x5A5A_0000 | {..., two, irq, en}，
//       防止野指针一笔写到 0x2800_0000 就把狗关了。
//     · FEED 写任意值都算喂，优先级高于本拍的超时。
//
//   毫秒换算：MS_DIV = CLK_MHZ × 1000 拍/毫秒。换核时钟只改 CLK_MHZ 参数。
//   ★ 台子用 CLK_MHZ=1（1 ms = 1000 拍）跑，几万拍就能验完整条链路；
//     真实上板是 120 → 1 ms = 120000 拍。
//////////////////////////////////////////////////////////////////////////////////

module wdt #(
    parameter integer CLK_MHZ = 100
)(
    input clk,
    //★ 见上：这根基里【不含】本模块自己的 wdt_reset
    input             rst,
    input      [31:0] bus_addr_in,      // 字地址（字节地址 >> 2）
    input      [31:0] bus_data_in,
    input      [3:0]  bus_be_in,
    input             bus_we_in,
    input             bus_sel,
    output reg [31:0] bus_data_out,
    output reg        ld_ready,
    output            wdt_irq,
    output reg        wdt_reset
);

//------------------------------------------------------------------ 参数
    localparam integer MS_DIV = (CLK_MHZ < 1) ? 1000 : CLK_MHZ * 1000;
    localparam [16:0]  MS_DIV_LAST = MS_DIV[16:0] - 17'd1;

    //LOCK 的高 16 位必须正好是这个常数，低 16 位直接当 CTRL 用
    localparam [15:0]  LOCK_MAGIC = 16'h5A5A;

    localparam [3:0]
    R_CTRL   = 4'd0,
    R_RELOAD = 4'd1,
    R_HOLD   = 4'd2,
    R_FEED   = 4'd3,
    R_COUNT  = 4'd4,
    R_STATUS = 4'd5,
    R_LOCK   = 4'd6;

//------------------------------------------------------------------ 事务
    wire wr = bus_sel & bus_we_in & (|bus_be_in);
    wire rd = bus_sel & ~bus_we_in;

//------------------------------------------------------------------ 状态
    reg        ctrl_en, ctrl_irq_en, ctrl_two_stage;
    reg [19:0] reload_ms, hold_ms;
    reg [19:0] ms_left;                 // 当前阶段剩余毫秒
    reg [16:0] div_cnt;                 // 毫秒内的拍计数
    reg        in_hold;                 // 正在数第二级
    reg        stage1;                  // 一级已到（同时是中断源的驱动电平）
    reg        reset_done;              // STATUS[1]：复位过（写 1 清）
    reg [4:0]  rst_cnt;                 // 复位脉冲还剩几拍

    //★ 中断源是【电平】，而且必须跟状态位走：stage1 是唯一的驱动源，
    //  不在别处再存一个粘滞位（否则软件清了 stage1 中断还挂着）。
    assign wdt_irq = stage1 & ctrl_irq_en;

//------------------------------------------------------------------ 主逻辑
    always @(posedge clk) begin
        if (rst) begin
            ctrl_en       <= 1'b0;          // ★ 默认关
            ctrl_irq_en   <= 1'b0;
            ctrl_two_stage<= 1'b0;
            reload_ms     <= 20'd1000;      // 1 s
            hold_ms       <= 20'd500;       // 500 ms
            ms_left       <= 20'd1000;
            div_cnt       <= 17'd0;
            in_hold       <= 1'b0;
            stage1        <= 1'b0;
            reset_done    <= 1'b0;
            rst_cnt       <= 5'd0;
            wdt_reset     <= 1'b0;
            bus_data_out  <= 32'd0;
            ld_ready      <= 1'b0;
        end
        else begin
            ld_ready <= 1'b0;

            //-------------------------------------------------- 复位脉冲收尾
            //  ★ 放在最前面：脉冲期间计数停摆，脉冲结束才回到"关闭"态
            if (rst_cnt != 5'd0) begin
                rst_cnt <= rst_cnt - 5'd1;
                if (rst_cnt == 5'd1) begin
                    wdt_reset  <= 1'b0;
                    //★ 三个使能位一起清：只清 en 的话 CTRL 读回来还是 6，
                    //  下次软件"读-改-写"会把旧的 irq_en/two_stage 带回去。
                    ctrl_en        <= 1'b0;
                    ctrl_irq_en    <= 1'b0;
                    ctrl_two_stage <= 1'b0;
                    reset_done <= 1'b1;
                    ms_left    <= reload_ms;
                    div_cnt    <= 17'd0;
                    in_hold    <= 1'b0;
                end
            end
            else if (!ctrl_en) begin
                //-------------------------------------------- 关闭态：计数一直压满
                ms_left  <= reload_ms;
                div_cnt  <= 17'd0;
                in_hold  <= 1'b0;
                stage1   <= 1'b0;
            end
            else begin
                //-------------------------------------------- 计数
                if (div_cnt != MS_DIV_LAST) begin
                    div_cnt <= div_cnt + 17'd1;
                end
                else begin
                    div_cnt <= 17'd0;
                    if (ms_left != 20'd0) begin
                        ms_left <= ms_left - 20'd1;
                    end
                    else if (in_hold) begin
                        // 二级超时：拉复位
                        wdt_reset <= 1'b1;
                        rst_cnt   <= 5'd16;
                        in_hold   <= 1'b0;
                    end
                    else begin
                        // 一级超时
                        stage1 <= 1'b1;
                        if (ctrl_two_stage) begin
                            in_hold <= 1'b1;
                            ms_left <= hold_ms;
                        end
                        else begin
                            wdt_reset <= 1'b1;
                            rst_cnt   <= 5'd16;
                        end
                    end
                end
            end

            //-------------------------------------------------- 寄存器写
            //  ★ 放在计数之后：同一拍里"喂狗"优先于"超时"（软件醒了就算醒）
            if (wr) begin
                case (bus_addr_in[3:0])
                R_RELOAD: reload_ms <= bus_data_in[19:0];
                R_HOLD:   hold_ms   <= bus_data_in[19:0];
                R_FEED: begin
                    ms_left <= reload_ms;
                    div_cnt <= 17'd0;
                    stage1  <= 1'b0;
                    in_hold <= 1'b0;
                end
                R_STATUS: if (bus_data_in[1]) reset_done <= 1'b0;
                R_LOCK: begin
                    // ★ 只有带魔数才当"改配置"，裸写 CTRL 一律忽略
                    if (bus_data_in[31:16] == LOCK_MAGIC) begin
                        ctrl_en        <= bus_data_in[0];
                        ctrl_irq_en    <= bus_data_in[1];
                        ctrl_two_stage <= bus_data_in[2];
                    end
                end
                default: ;      // R_CTRL 直接写：忽略（要解锁）
                endcase
            end

            //-------------------------------------------------- 寄存器读
            if (rd) begin
                case (bus_addr_in[3:0])
                R_CTRL:   bus_data_out <= {29'd0, ctrl_two_stage, ctrl_irq_en, ctrl_en};
                R_RELOAD: bus_data_out <= {12'd0, reload_ms};
                R_HOLD:   bus_data_out <= {12'd0, hold_ms};
                R_COUNT:  bus_data_out <= {12'd0, ms_left};
                R_STATUS: bus_data_out <= {30'd0, reset_done, stage1};
                R_LOCK:   bus_data_out <= {LOCK_MAGIC, 13'd0,
                                           ctrl_two_stage, ctrl_irq_en, ctrl_en};
                default:  bus_data_out <= 32'd0;
                endcase
                ld_ready <= 1'b1;
            end
        end
    end

endmodule
