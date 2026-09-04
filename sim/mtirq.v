`timescale 1ns / 1ps
module mtirq;
    reg clk, rst, exti;
    integer i, done_cyc, ec;
    cpu_top dut (.clk(clk),.rst(rst),.bus_addr_out(),.bus_data_out(),.bus_we_out(),
        .bus_data_in_ext(32'd0),.ibus_addr_out(),.ibus_re_out(),.ibus_data_in(32'd0),
        .ibus_addr_in(16'd0),.ibus_we_in(1'b0),.bus_loaded_in(1'b0),.exti(exti),.i_busy(1'b0));
    initial clk = 0;
    always #5 clk = ~clk;
    initial begin
        rst = 1; done_cyc = -1; exti = 0; ec = 0;
        repeat (10) @(negedge clk);
        rst = 0;
        for (i = 0; i < 200000 && done_cyc < 0; i = i + 1) begin
            @(negedge clk);
            if (ec == 0 && dut.u_dtcm.dtcm[16'h40] != 32'd0) ec = 1;
            if (ec >= 1) begin
                ec = ec + 1;
                if (ec == 3) exti = 1;
                if (ec == 9) begin exti = 0; ec = -1; end
            end
            if (dut.u_dtcm.dtcm[16'hfff] != 32'd0) done_cyc = i;
        end
        repeat (5) @(negedge clk);
        if (done_cyc >= 0)
            $display("IRQ done cyc=%0d verdict=0x%08x isrcnt=%0d mcause=%h mepc=%h s3=%h",
                done_cyc, dut.u_dtcm.dtcm[16'hfff], dut.u_dtcm.dtcm[16'h50],
                dut.u_dtcm.dtcm[16'h51], dut.u_dtcm.dtcm[16'h52], dut.u_dtcm.dtcm[16'h70]);
        else
            $display("IRQ TIMEOUT");
        $finish;
    end
endmodule
