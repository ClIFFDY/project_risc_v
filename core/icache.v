`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/19
// Design Name:
// Module Name: icache
// Project Name:
// Target Devices:
// Tool Suites:
// Description: 唯一取指源。48KB = 128 组 × 3 路 × 32 字。
//              内含取指地址生成（原 itcm）。
//              way0 存放自举装入的前 16KB 指令并锁定，不参与替换；
//              way1/way2 作后 48KB 的常规缓存，组内两路 LRU。
//              复位释放后 busy 持续拉高，自举从 itcm 顺序填充前 16KB，
//              填满后 busy 落下，功能相当于 itcm + icache + bootloader。
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module icache(
    input clk, rst,
//取指地址生成（原 itcm）
    input [31:0] pc_addr,
    input [31:0] offset_jal1, offset_jalr1, offset_beq1,
    input [31:0] isr_addr1, isr_ret_addr1,
    input [31:0] jalr_target_q, beq_off_q1, br_addr1,
    input br1, br2, br3, jal, jalr, jalr_fail, irq, irq_ret,
//读口控制与取指输出
    input req_valid,
    output reg [31:0] inst_out,
    output reg busy,
//回填接口（接核内 itcm）
    output reg mem_req, mem_we,
    output reg [31:0] mem_addr, mem_wdata,
    output reg [3:0] mem_be,
    input mem_valid,
    input [31:0] mem_data
    );

    localparam BOOT_LINES = 7'd127;

    reg cache_hit;
    reg [18:0] tag;
    reg [6:0] idx;
    reg [4:0] word;
    reg [2:0] hit_way;
    reg [1:0] hit_sel;

    reg [18:0] tag0 [0:127];
    reg [18:0] tag1 [0:127];
    reg [18:0] tag2 [0:127];
    (* ram_style = "block" *) reg [31:0] iram [0:12287];
    reg [127:0] valid0, valid1, valid2, lru;

    reg boot;
    reg [6:0] boot_line;

    reg stage;
    reg fill_end;
    reg [4:0] miss_word;
    reg [4:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [18:0] fill_tag;
    reg [6:0] fill_idx;
    reg [1:0] fill_way;

    reg [13:0] rd_addr;
    reg rd_en, line_match;

    integer i;

//跳转类地址透传处理，减少取值冲刷空窗
    reg [31:0] fetch_addr;
    always @(*) begin
        if (rst) fetch_addr = 32'd0;
        else if (irq)            fetch_addr = isr_addr1 >>> 2;
        else if (irq_ret)        fetch_addr = isr_ret_addr1 >>> 2;
        else if (br2)            fetch_addr = (br_addr1 + beq_off_q1) >>> 2;
        else if (br3)            fetch_addr = br_addr1 >>> 2;
        else if (jalr_fail)      fetch_addr = jalr_target_q >>> 2;
        else if (br1)            fetch_addr = (pc_addr + offset_beq1) >>> 2;
        else if (jal)            fetch_addr = (pc_addr + offset_jal1) >>> 2;
        else if (jalr)           fetch_addr = offset_jalr1 >>> 2;
        else                     fetch_addr = pc_addr >>> 2;
    end

    always @(*) begin
        tag = fetch_addr[31:12];
        idx = fetch_addr[11:5];
        word = fetch_addr[4:0];
        hit_way[0] = valid0[idx] && (tag0[idx] == tag);
        hit_way[1] = valid1[idx] && (tag1[idx] == tag);
        hit_way[2] = valid2[idx] && (tag2[idx] == tag);
        cache_hit = hit_way[0] || hit_way[1] || hit_way[2];
        hit_sel = hit_way[0] ? 2'd0 : (hit_way[1] ? 2'd1 : 2'd2);
//自举期间及自举完成后的一拍都保持 busy
        busy = boot ||
               (stage == 1'd1 && (!fill_end || (!cache_hit &&
                                             !(fill_idx == idx && fill_tag == tag)))) ||
               (stage == 1'd0 && !cache_hit);
        mem_req = (boot || stage == 1'd1) && !fill_end;
        mem_addr = fill_addr + (fill_cnt << 2);
        mem_we = 1'b0;
        mem_wdata = 32'd0;
        mem_be = 4'd0;

//读口：把 hit 与 fill_end 两条路合成一个地址/一个使能/一个数据选择，
//这样每个 iram 只有一个读口，才推得出 BRAM
        line_match = (fetch_addr & 32'hFFFFFFE0) == (fill_addr >> 2);
        rd_en  = (fill_end && line_match) || req_valid;
        rd_addr = (fill_end && line_match) ? {fill_way, fill_idx, miss_word}
                                           : {hit_sel, idx, word};
    end

//回填：一拍收一个字直接写进 iram（BRAM 单写口，一拍只写一个地址）
    always @(posedge clk) begin
        if ((boot || stage == 1'd1) && !fill_end && mem_valid) begin
            iram[{fill_way, fill_idx, fill_cnt}] <= mem_data;
        end
    end

    always @(posedge clk) begin
        if (rst)
            inst_out <= 32'd0;
        else if (rd_en)
            inst_out <= iram[rd_addr];
    end

//自举序列：复位释放后逐行填充前 16KB（128 行 × 32 字），固定写 way0
    always @(posedge clk) begin
        if (rst) begin
            boot <= 1'b1;
            boot_line <= 7'd0;
            stage <= 1'd0;
            fill_end <= 1'b0;
            fill_cnt <= 5'd0;
            fill_way <= 2'd0;
            fill_addr <= 32'd0;
            fill_tag <= 19'd0;
            fill_idx <= 7'd0;
            miss_word <= 5'd0;
            valid0 <= 128'd0;
            valid1 <= 128'd0;
            valid2 <= 128'd0;
            lru <= 128'd0;
//tag0/tag1/tag2 不复位：命中由 valid 挡住，陈旧 tag 无害。
//带复位会让这 3x128x19 位全被综合成带复位的 FF（撑爆 slice 打包），
//去掉后能进分布式 RAM。valid 必须留复位 —— 它才是命中判据。
        end
        else if (boot) begin
            if (mem_valid && !fill_end) begin
                if (fill_cnt == 5'd31)
                    fill_end <= 1'b1;
                else
                    fill_cnt <= fill_cnt + 5'd1;
            end
            if (fill_end) begin
                valid0[fill_idx] <= 1'b1;
                tag0[fill_idx] <= 19'd0;
                fill_end <= 1'b0;
                fill_cnt <= 5'd0;
                fill_way <= 2'd0;
                fill_tag <= 19'd0;
                miss_word <= 5'd0;
                if (boot_line == BOOT_LINES) begin
                    boot <= 1'b0;
                    stage <= 1'd0;
                end
                else begin
                    boot_line <= boot_line + 7'd1;
                    fill_idx <= boot_line + 7'd1;
                    fill_addr <= {boot_line + 7'd1, 7'd0};
                end
            end
        end
        else begin
            if (stage == 1'd0) begin
                fill_end <= 1'b0;
                if (hit_way[1])      lru[idx] <= 1'b1;
                else if (hit_way[2]) lru[idx] <= 1'b0;
                if (!cache_hit) begin
                    stage <= 1'd1;
                    miss_word <= word;
                    fill_cnt <= 5'd0;
                    fill_addr <= {fetch_addr[31:5], 5'd0} << 2;
                    fill_tag <= tag;
                    fill_idx <= idx;
                    fill_way <= (!valid1[idx]) ? 2'd1 :
                                (!valid2[idx]) ? 2'd2 :
                                (lru[idx] ? 2'd2 : 2'd1);
                end
            end
            else begin
                if (mem_valid && !fill_end) begin
                    if (fill_cnt == 5'd31)
                        fill_end <= 1'b1;
                    else
                        fill_cnt <= fill_cnt + 5'd1;
                end
                if (fill_end) begin
                    if (fill_way == 2'd1) begin
                        tag1[fill_idx] <= fill_tag;
                        valid1[fill_idx] <= 1'b1;
                    end
                    else begin
                        tag2[fill_idx] <= fill_tag;
                        valid2[fill_idx] <= 1'b1;
                    end
                    lru[fill_idx] <= (fill_way == 2'd1);
                    if (!cache_hit && !(fill_idx == idx && fill_tag == tag)) begin
                        stage <= 1'd1;
                        miss_word <= word;
                        fill_cnt <= 5'd0;
                        fill_addr <= {fetch_addr[31:5], 5'd0} << 2;
                        fill_tag <= tag;
                        fill_idx <= idx;
                        fill_way <= (fill_way == 2'd1) ? 2'd2 : 2'd1;
                        fill_end <= 1'b0;
                    end
                    else begin
                        stage <= 1'd0;
                        fill_end <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
