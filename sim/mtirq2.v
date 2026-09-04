`timescale 1ns / 1ps
module mtirq2;
    reg clk, rst, exti;
    integer i, done_cyc, p1, p2;
    cpu_top dut (.clk(clk),.rst(rst),.bus_addr_out(),.bus_data_out(),.bus_we_out(),
        .bus_data_in_ext(32'd0),.ibus_addr_out(),.ibus_re_out(),.ibus_data_in(32'd0),
        .ibus_addr_in(16'd0),.ibus_we_in(1'b0),.bus_loaded_in(1'b0),.exti(exti),.i_busy(1'b0));
    initial clk = 0;
    always #5 clk = ~clk;
    task pulse; input integer n; begin exti=1; repeat(n) @(negedge clk); exti=0; end endtask
    initial begin
        rst=1; done_cyc=-1; exti=0; p1=0; p2=0;
        repeat (10) @(negedge clk); rst=0;
        for (i=0;i<400000 && done_cyc<0;i=i+1) begin
            @(negedge clk);
            if (!p1 && dut.u_dtcm.dtcm[16'h40]!=0) begin p1=1; pulse(5); end
            if (!p2 && p1 && dut.u_dtcm.dtcm[16'h41]!=0) begin p2=1; pulse(5); end
            if (dut.u_dtcm.dtcm[16'hfff]!=0) done_cyc=i;
        end
        $display("MASK done cyc=%0d verdict=%h isrcnt=%h", done_cyc,
            dut.u_dtcm.dtcm[16'hfff], dut.u_dtcm.dtcm[16'h50]);
        $finish;
    end
endmodule
