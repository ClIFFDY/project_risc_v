`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: stcm
// Description: 64KB 模拟外存（AHB 从设备）。L2 的后备存储，代替将来的 DDR 桥。
//
//   总线约定与 ahb_apb_bridge 一致（本仓的统一口径）：
//     · hready 恒 1：地址相位的每一拍都被接受；
//     · 一次传输 = htrans != IDLE 的一拍（脉冲式 valid），
//     · 读数据晚一拍，用 rd_valid 打一拍交出去。
//   ⇒ 连拍地址（NONSEQ 后跟 SEQ）就是 1 拍 1 个字的突发读，L2 的行填充按这个用。
//
//   写：在地址相位那一拍直接落 BRAM（字节使能 hwstrb）。
//   读：地址相位锁地址 → 下一拍出数据 + rd_valid。
//
//   容量：AW=14 ⇒ 16384 字 = 64KB = 16 × RAMB36。
//   地址范围由互联译码（0x0000_0000 ~ 0x0000_FFFF）。
//////////////////////////////////////////////////////////////////////////////////


module stcm #(
    parameter integer AW = 14
) (
    input  clk, rst,
//---- AHB 从口 ----
    input      [31:0] haddr,
    input      [1:0]  htrans,
    input             hwrite,
    input      [2:0]  hsize,
    input      [2:0]  hburst,
    input      [3:0]  hprot,
    input             hmastlock,
    input      [31:0] hwdata,
    input      [3:0]  hwstrb,
    output reg [31:0] hrdata,
//标准 AHB-Lite 从设备口。rd_valid 是本仓自己的完成脉冲（非标准），
//第一期先和 HREADYOUT 并存，等三个主设备都改认 HREADY/HRESP 之后再删。
    output            hreadyout,
    output            hresp,
    output reg        rd_valid
    );

    localparam integer WORDS = 1 << AW;

//复位就地打一拍（与其它模块同款：全核各模块零相位差）
    reg reset_registered;
    always @(posedge clk) reset_registered <= rst;

//数据阵列。★ 必须"单一地址表达式 + 输出寄存器"才推得出 BRAM（见 dtcm.v 头注释）
    (* ram_style = "block" *) reg [31:0] mem [0:WORDS-1];

    integer i;
    initial begin
        for (i = 0; i < WORDS; i = i + 1)
            mem[i] = 32'd0;
//上板时的启动镜像（平铺：代码 @0，.data 跟在后面按它自己的 VMA 落位）。
//★ 仿真台会在自己的 initial 里整幅覆盖（台子声明在例化之后 ⇒ Verilator 后执行），
//  这里读不到文件只会报一个 warning，不影响仿真。
        $readmemh("e:/Vivado_Projects/project_risc_v/tools/hex/image.hex", mem);
    end

    wire accept = (htrans != 2'b00);
    wire [AW-1:0] word_index = haddr[AW+1:2];

//写：地址相位那一拍直接落
    always @(posedge clk) begin
        if (accept & hwrite) begin
            if (hwstrb[0]) mem[word_index][7:0]   <= hwdata[7:0];
            if (hwstrb[1]) mem[word_index][15:8]  <= hwdata[15:8];
            if (hwstrb[2]) mem[word_index][23:16] <= hwdata[23:16];
            if (hwstrb[3]) mem[word_index][31:24] <= hwdata[31:24];
        end
    end

//读：地址相位锁地址，下一拍出数据
    reg [AW-1:0] read_index;
    reg          read_pending;
    always @(posedge clk) begin
        if (reset_registered) begin
            read_index   <= {AW{1'b0}};
            read_pending <= 1'b0;
        end
        else begin
            read_index   <= word_index;
            read_pending <= accept & ~hwrite;
        end
    end

    always @(posedge clk) begin
        if (reset_registered) begin
            hrdata   <= 32'd0;
            rd_valid <= 1'b0;
        end
        else begin
            hrdata   <= mem[read_index];
            rd_valid <= read_pending;
        end
    end

//STCM 的地址范围由互联按 haddr[31:16]==0 选中，越界地址根本到不了这里 ⇒ 永不报错。
    assign hreadyout = 1'b1;
    assign hresp     = 1'b0;

endmodule
