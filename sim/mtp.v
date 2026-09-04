`timescale 1ns / 1ps
module mtp;
    reg clk, rst, exti;
    integer i, done_cyc, ec;
    cpu_top dut (.clk(clk),.rst(rst),.bus_addr_out(),.bus_data_out(),.bus_we_out(),
        .bus_data_in_ext(32'd0),.ibus_addr_out(),.ibus_re_out(),.ibus_data_in(32'd0),
        .ibus_addr_in(16'd0),.ibus_we_in(1'b0),.bus_loaded_in(1'b0),.exti(exti),.i_busy(1'b0));
    initial clk = 0;
    always #5 clk = ~clk;
    initial begin
        rst=1; done_cyc=-1; exti=0; ec=0;
        repeat (10) @(negedge clk); rst=0;
        for (i=0;i<200 && done_cyc<0;i=i+1) begin
            @(negedge clk);
            if (ec==0 && dut.u_dtcm.dtcm[16'h40]!=0) ec=1;
            if (ec>=1) begin ec=ec+1; if (ec==3) exti=1; if (ec==9) begin exti=0; ec=-1; end end
            if (i>=15 && i<=60)
                $display("cyc%3d pc=%04x stg=%0d iact=%b proc=%b bubble=%2d iret1=%03x iret2=%03x iret=%b isr=%d ra=%04x",
                    i, dut.pc_addr, dut.stage,
                    dut.u_controller.u_csr.irq_act, dut.u_controller.u_csr.irq_process,
                    dut.irq_bubble, dut.iret_addr1, dut.iret_addr2, dut.irq_ret,
                    dut.u_dtcm.dtcm[16'h50], dut.u_regfile.regs[1]);
            if (dut.u_dtcm.dtcm[16'hfff]!=0) done_cyc=i;
        end
        $finish;
    end
endmodule
