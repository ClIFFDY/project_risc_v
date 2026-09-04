`timescale 1ns / 1ps

module mtb;
    reg clk, rst;
    integer i, done_cyc;

    cpu_top dut (
        .clk(clk),
        .rst(rst),
        .bus_addr_out(),
        .bus_data_out(),
        .bus_we_out(),
        .bus_data_in_ext(32'd0),
        .ibus_addr_out(),
        .ibus_re_out(),
        .ibus_data_in(32'd0),
        .ibus_addr_in(16'd0),
        .ibus_we_in(1'b0),
        .bus_loaded_in(1'b0),
        .exti(1'b0),
        .i_busy(1'b0));

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst = 1;
        done_cyc = -1;
        repeat (10) @(negedge clk);
        rst = 0;
        for (i = 0; i < 2000000 && done_cyc < 0; i = i + 1) begin
            @(negedge clk);
            if (dut.u_dtcm.dtcm[16'hfff] != 32'd0) done_cyc = i;
        end
        repeat (5) @(negedge clk);
        if (done_cyc >= 0)
            $display("MT done cyc=%0d verdict=0x%08x subid=0x%08x", done_cyc,
                dut.u_dtcm.dtcm[16'hfff], dut.u_dtcm.dtcm[16'd1]);
        else
            $display("MT TIMEOUT");
        for (i = 0; i < 16; i = i + 1) $display("dtcm[%0d]=0x%08x", i, dut.u_dtcm.dtcm[i]);
        $finish;
    end
endmodule
