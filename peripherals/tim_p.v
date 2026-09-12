`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/12 09:04:27
// Design Name:
// Module Name: tim_p
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


module tim_p(
    input clk, rst,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    output reg [31:0] bus_data_out,
    output reg ld_ready,
    output reg tim_p_irq,
    output reg pwm1, pwm2, pwm3, pwm4
    );

    localparam [3:0]
    R_CNT_SET  = 4'd0,
    R_CNT      = 4'd1,
    R_MODE     = 4'd2,
    R_IRQ_CLR  = 4'd3,
    R_DUTY1    = 4'd4,
    R_DUTY2    = 4'd5,
    R_DUTY3    = 4'd6,
    R_DUTY4    = 4'd7,
    R_PWM_EN   = 4'd8,
    R_IRQ_STAT = 4'd9;

    reg [31:0] cnt_set, cnt;
    reg [31:0] pwm_duty [0:3];
    reg [3:0] pwm_en;
    reg periodic, halted;

    integer i;

    wire sel = (bus_addr_in[31:24] == 8'd3);
    wire wr  = sel & bus_we_in & (|bus_be_in);
    wire rd  = sel & ~bus_we_in;
    wire run = (cnt_set != 32'd0) & ~halted;
    wire wr_cnt_set = wr & (bus_addr_in[3:0] == R_CNT_SET);

    always @(posedge clk) begin
        if (rst) begin
            cnt_set <= 32'd0;
            cnt <= 32'd0;
            for (i = 0; i < 4; i = i + 1) pwm_duty[i] <= 32'd0;
            periodic <= 1'b0;
            pwm_en <= 4'd0;
            halted <= 1'b0;
            tim_p_irq <= 1'b0;
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            pwm1 <= 1'b0;
            pwm2 <= 1'b0;
            pwm3 <= 1'b0;
            pwm4 <= 1'b0;
        end
        else begin
            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;

//计数器：到重装值置中断并回零，单次模式触发后停表
            if (run & ~wr_cnt_set) begin
                if (cnt + 32'd1 == cnt_set) begin
                    cnt <= 32'd0;
                    tim_p_irq <= 1'b1;
                    if (~periodic) halted <= 1'b1;
                end
                else begin
                    cnt <= cnt + 32'd1;
                end
            end

//PWM：门限比较，占空 = duty/cnt_set
            pwm1 <= pwm_en[0] & (pwm_duty[0] > cnt);
            pwm2 <= pwm_en[1] & (pwm_duty[1] > cnt);
            pwm3 <= pwm_en[2] & (pwm_duty[2] > cnt);
            pwm4 <= pwm_en[3] & (pwm_duty[3] > cnt);

//写时序严格1拍；写清中断写在本块最后，与置位同拍时写优先
            if (wr) begin
                case (bus_addr_in[3:0])
                R_CNT_SET: begin
                    if (bus_be_in[0]) cnt_set[7:0]   <= bus_data_in[7:0];
                    if (bus_be_in[1]) cnt_set[15:8]  <= bus_data_in[15:8];
                    if (bus_be_in[2]) cnt_set[23:16] <= bus_data_in[23:16];
                    if (bus_be_in[3]) cnt_set[31:24] <= bus_data_in[31:24];
                    cnt <= 32'd0;
                    halted <= 1'b0;
                end
                R_MODE: begin
                    periodic <= bus_data_in[0];
                end
                R_IRQ_CLR: begin
                    tim_p_irq <= 1'b0;
                end
                R_DUTY1, R_DUTY2, R_DUTY3, R_DUTY4: begin
                    for (i = 0; i < 4; i = i + 1) begin
                        if (bus_addr_in[3:0] == R_DUTY1 + i[3:0]) begin
                            if (bus_be_in[0]) pwm_duty[i][7:0]   <= bus_data_in[7:0];
                            if (bus_be_in[1]) pwm_duty[i][15:8]  <= bus_data_in[15:8];
                            if (bus_be_in[2]) pwm_duty[i][23:16] <= bus_data_in[23:16];
                            if (bus_be_in[3]) pwm_duty[i][31:24] <= bus_data_in[31:24];
                        end
                    end
                end
                R_PWM_EN: begin
                    pwm_en <= bus_data_in[3:0];
                end
                default: ;
                endcase
            end
//读时序逻辑，data 与 ready 同沿发出
            else if (rd) begin
                ld_ready <= 1'b1;
                case (bus_addr_in[3:0])
                R_CNT_SET:  bus_data_out <= cnt_set;
                R_CNT:      bus_data_out <= cnt;
                R_MODE:     bus_data_out <= {31'd0, periodic};
                R_DUTY1, R_DUTY2, R_DUTY3, R_DUTY4: begin
                    for (i = 0; i < 4; i = i + 1)
                        if (bus_addr_in[3:0] == R_DUTY1 + i[3:0])
                            bus_data_out <= pwm_duty[i];
                end
                R_PWM_EN:   bus_data_out <= {28'd0, pwm_en};
                R_IRQ_STAT: bus_data_out <= {30'd0, halted, tim_p_irq};
                default:    bus_data_out <= 32'd0;
                endcase
            end
        end
    end
endmodule
