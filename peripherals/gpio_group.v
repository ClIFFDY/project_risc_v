`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/09/12 08:58:46
// Design Name:
// Module Name: gpio_group
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


module gpio_group(
    input clk, rst,
    input [31:0] bus_addr_in,
    input [31:0] bus_data_in,
    input [3:0] bus_be_in,
    input bus_we_in,
    output reg [31:0] bus_data_out,
    output reg ld_ready,
    output reg gpio_irq,

    inout wire [31:0] gpio_pin_bus,
    input sda_oe,
    input scl_oe,
    input tx,
    input pwm1, pwm2, pwm3, pwm4,
    output reg rx, sda_in, scl_in
    );

    localparam [3:0]
        UNUSE = 4'b0000,
        OUT = 4'b0001,
        IN = 4'b0010,
        IRQ = 4'b0011,
        TX = 4'b0101,
        RX = 4'b0110,
        PWM1 = 4'b0111,
        PWM2 = 4'b1000,
        SCL = 4'b1001,
        SDA = 4'b1010,
        PWM3 = 4'b1011,
        PWM4 = 4'b1100;

    localparam [5:0]
        R_OUT = 6'h00,
        R_IN = 6'h04,
        R_MODE0 = 6'h08,
        R_MODE1 = 6'h0c,
        R_MODE2 = 6'h10,
        R_MODE3 = 6'h14,
        R_IRQ_STAT = 6'h18,
        R_IRQ_TYPE0 = 6'h1c,
        R_IRQ_TYPE1 = 6'h20;

    reg [3:0] gpio_mode [0:31];
    reg [31:0] gpio_output;
    reg [31:0] gpio_pin_in_q;
    reg [31:0] gpio_irq_status;
    reg [63:0] gpio_irq_type;
    reg [31:0] gpio_pin_out;
    reg [31:0] gpio_pin_oe;
    reg [5:0] reg_sel;

    integer i;

//引脚方向与输出值：按每脚 mode 组合决定
    always @(*) begin
        gpio_pin_out = 32'd0;
        gpio_pin_oe = 32'd0;
        for (i = 0; i < 32; i = i + 1) begin
            case (gpio_mode[i])
            OUT: begin
                gpio_pin_oe[i] = 1'b1;
                gpio_pin_out[i] = gpio_output[i];
            end
            TX: begin
                gpio_pin_oe[i] = 1'b1;
                gpio_pin_out[i] = tx;
            end
            PWM1: begin
                gpio_pin_oe[i] = 1'b1;
                gpio_pin_out[i] = pwm1;
            end
            PWM2: begin
                gpio_pin_oe[i] = 1'b1;
                gpio_pin_out[i] = pwm2;
            end
            PWM3: begin
                gpio_pin_oe[i] = 1'b1;
                gpio_pin_out[i] = pwm3;
            end
            PWM4: begin
                gpio_pin_oe[i] = 1'b1;
                gpio_pin_out[i] = pwm4;
            end
            SCL: begin
                gpio_pin_oe[i] = scl_oe;
                gpio_pin_out[i] = 1'b0;
            end
            SDA: begin
                gpio_pin_oe[i] = sda_oe;
                gpio_pin_out[i] = 1'b0;
            end
            default: ;
            endcase
        end
    end

    genvar j;
    generate
        for (j = 0; j < 32; j = j + 1) begin: gpio_pad
            assign gpio_pin_bus[j] = gpio_pin_oe[j] ? gpio_pin_out[j] : 1'bz;
        end
    endgenerate

//SDA / SCL 模式脚的电平回读到 i2c
    always @(*) begin
        sda_in = 1'b1;
        scl_in = 1'b1;
        for (i = 0; i < 32; i = i + 1) begin
            if (gpio_mode[i] == SDA) sda_in = gpio_pin_bus[i];
            if (gpio_mode[i] == SCL) scl_in = gpio_pin_bus[i];
        end
    end

//中断聚合：任意 IRQ 模式脚的状态位置起即拉高，只由写 R_IRQ_STAT 清
    always @(*) begin
        gpio_irq = |gpio_irq_status;
    end

//总线读写与中断状态更新
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) gpio_mode[i] <= UNUSE;
            gpio_output <= 32'd0;
            gpio_irq_status <= 32'd0;
            gpio_irq_type <= 64'd0;
            gpio_pin_in_q <= 32'd0;
            rx <= 1'b1;
        end
        else begin
            gpio_pin_in_q <= gpio_pin_bus;
            rx <= 1'b1;

            for (i = 0; i < 32; i = i + 1) begin
                if (gpio_mode[i] == TX) gpio_output[i] <= tx;
                else if (gpio_mode[i] == PWM1) gpio_output[i] <= pwm1;
                else if (gpio_mode[i] == PWM2) gpio_output[i] <= pwm2;
                if (gpio_mode[i] == RX) rx <= gpio_pin_bus[i];

                if (gpio_mode[i] == IRQ) begin
                    case (gpio_irq_type[2*i +: 2])
                    2'd0: if (gpio_pin_in_q[i] && !gpio_pin_bus[i]) gpio_irq_status[i] <= 1'b1;
                    2'd1: if (!gpio_pin_in_q[i] && gpio_pin_bus[i]) gpio_irq_status[i] <= 1'b1;
                    2'd2: if (gpio_pin_in_q[i] != gpio_pin_bus[i])  gpio_irq_status[i] <= 1'b1;
                    2'd3: if (!gpio_pin_bus[i])                     gpio_irq_status[i] <= 1'b1;
                    endcase
                end
            end

            bus_data_out <= 32'd0;
            ld_ready <= 1'b0;
            if (bus_addr_in[31:24] == 8'd4) begin
                reg_sel = bus_addr_in[5:0];
                case (reg_sel)
                R_OUT: begin
                    if (bus_we_in) begin
                        for (i = 0; i < 32; i = i + 1) begin
                            if (bus_be_in[i/8] && gpio_mode[i] == OUT)
                                gpio_output[i] <= bus_data_in[i];
                        end
                    end
                    else begin
                        bus_data_out <= gpio_output;
                        ld_ready <= 1'b1;
                    end
                end
                R_IN: begin
                    if (!bus_we_in) begin
                        bus_data_out <= gpio_pin_bus;
                        ld_ready <= 1'b1;
                    end
                end
                R_MODE0, R_MODE1, R_MODE2, R_MODE3: begin
                    if (bus_we_in) begin
                        for (i = 0; i < 32; i = i + 1) begin
                            if (i/8 == (reg_sel - R_MODE0) >> 2) begin
                                if (bus_be_in[(i%8)/2]) gpio_mode[i] <= bus_data_in[4*(i%8) +: 4];
                            end
                        end
                    end
                    else begin
                        for (i = 0; i < 32; i = i + 1) begin
                            if (i/8 == (reg_sel - R_MODE0) >> 2)
                                bus_data_out[4*(i%8) +: 4] <= gpio_mode[i];
                        end
                        ld_ready <= 1'b1;
                    end
                end
                R_IRQ_STAT: begin
                    if (bus_we_in) begin
                        for (i = 0; i < 32; i = i + 1) begin
                            if (bus_be_in[i/8] && bus_data_in[i]) gpio_irq_status[i] <= 1'b0;
                        end
                    end
                    else begin
                        bus_data_out <= gpio_irq_status;
                        ld_ready <= 1'b1;
                    end
                end
                R_IRQ_TYPE0, R_IRQ_TYPE1: begin
                    if (bus_we_in) begin
                        for (i = 0; i < 32; i = i + 1) begin
                            if (i/16 == (reg_sel - R_IRQ_TYPE0) >> 2) begin
                                if (bus_be_in[(i%16)/4]) gpio_irq_type[2*i +: 2] <= bus_data_in[2*(i%16) +: 2];
                            end
                        end
                    end
                    else begin
                        for (i = 0; i < 32; i = i + 1) begin
                            if (i/16 == (reg_sel - R_IRQ_TYPE0) >> 2)
                                bus_data_out[2*(i%16) +: 2] <= gpio_irq_type[2*i +: 2];
                        end
                        ld_ready <= 1'b1;
                    end
                end
                default: begin
                    if (!bus_we_in) ld_ready <= 1'b1;
                end
                endcase
            end
        end
    end
endmodule
