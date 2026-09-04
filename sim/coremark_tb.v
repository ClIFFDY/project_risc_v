`timescale 1ns / 1ps

module coremark_tb;
    reg clk, rst;
    integer i, done_cyc, committed;

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
        committed = 0;
        repeat (10) @(negedge clk);
        rst = 0;
        for (i = 0; i < 2000000 && done_cyc < 0; i = i + 1) begin
            @(negedge clk);
            if (dut.we_5 || dut.loaded) committed = committed + 1;
            if (dut.u_dtcm.dtcm[16'hfff] != 32'd0) done_cyc = i;
        end
        repeat (5) @(negedge clk);
        if (done_cyc >= 0)
            $display("COREMARK done cyc=%0d result=0x%08x committed=%0d", done_cyc, dut.u_dtcm.dtcm[16'hfff], committed);
        else
            $display("COREMARK TIMEOUT dtcm_last=%h", dut.u_dtcm.dtcm[16'hfff]);
        $display("mcycle=%0d minstret=%0d", dut.u_controller.u_csr.mcycle_reg, dut.u_controller.u_csr.minstret_reg);
        $finish;
    end
endmodule
