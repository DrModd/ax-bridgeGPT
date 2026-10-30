`timescale 1ns/1ps
// Fault injection: overflow must be recoverable; an input clock may be
// switched only after BOTH FIFO domains and the receiver are in reset.
module tb_recovery;
    reg clk = 0;
    always #10 clk = ~clk;
    reg b0 = 0, b1 = 0;
    always #17 b0 = ~b0;
    always #23 b1 = ~b1;
    tri1 sda;
    top dut (.mclk_in(clk), .i2s_bclk_in(b0), .i2s_lrck_in(1'b0),
        .i2s_sdata_in(1'b0), .dsd_on(1'b0),
        .in1_bclk(b1), .in1_lrck(1'b0), .in1_sdata(1'b0),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0),
        .i2c_scl(1'b1), .i2c_sda(sda));

    integer errors = 0, switches = 0;
    reg [1:0] previous_sel = 0;
    reg reset_before_edge = 0;
    always @(posedge clk) reset_before_edge = !dut.rst_in_n;
    always @(dut.sel_cur) if (dut.rst_n && dut.sel_cur != previous_sel) begin
        switches = switches + 1;
        if (!reset_before_edge || dut.rst_b !== 2'b00 || dut.rst_in_n !== 1'b0) begin
            $display("FAIL: clock mux changed before reset reached receiver");
            errors = errors + 1;
        end
        previous_sel = dut.sel_cur;
    end

    reg rst_n = 0, clr = 0, valid = 0;
    wire fov, cov, fv, cv;
    wire [23:0] fl, fr, cl, cr;
    fir_stage fir (.clk(clk), .rst_n(rst_n), .clr(clr), .in_valid(valid),
        .in_l(24'sd1024), .in_r(-24'sd1024), .out_valid(fv),
        .out_l(fl), .out_r(fr), .ovf(fov));
    cic5_dsd cic (.clk(clk), .rst_n(rst_n), .clr(clr), .r_log2(3'd3),
        .in_valid(valid), .in_l(16'hAAAA), .in_r(16'h5555),
        .out_valid(cv), .out_l(cl), .out_r(cr), .ovf(cov));
    integer fn = 0, cn = 0;
    reg recovering = 0;
    reg [7:0] fpos = 0;
    always @(posedge clk) fpos <= fpos + 1'b1;
    reg aclear = 0;
    wire ayv;
    wire [31:0] ayl, ayr;
    reg asrc_done = 0;
    asrc src (.clk(clk), .rst_n(rst_n), .clr(aclear), .fam48(1'b1),
        .arm(1'b1), .in_valid(fpos == 0), .in_l(24'sd1024), .in_r(-24'sd1024),
        .ev(fpos == 0), .fpos(fpos), .y_valid(ayv), .y_l(ayl), .y_r(ayr));
    // Abort an active convolution, then verify that it cannot publish a
    // stale sample after clear. Clock/rate accuracy is covered by tb_sys.
    initial begin
        wait (src.issuing === 1'b1);
        repeat (10) @(negedge clk);
        aclear = 1;
        @(negedge clk);
        if (src.issuing || ayv || ayl !== 0 || ayr !== 0) begin
            $display("FAIL: ASRC pipeline survives clear"); errors = errors + 1;
        end
        aclear = 0;
        repeat (100) begin
            @(negedge clk);
            if (ayv) begin
                $display("FAIL: stale ASRC result after clear"); errors = errors + 1;
            end
        end
        asrc_done = 1;
    end
    always @(posedge clk) if (recovering) begin
        if (fv) fn = fn + 1;
        if (cv) cn = cn + 1;
    end

    integer i;
    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        valid = 1; // Deliberately exceed throughput to overflow both queues.
        repeat (100) @(negedge clk);
        valid = 0;
        if (fov !== 1'b1 || cov !== 1'b1) begin
            $display("FAIL: fault injection did not overflow both queues"); errors = errors + 1;
        end
        clr = 1;
        repeat (3) @(negedge clk);
        if (fov !== 1'b0 || cov !== 1'b0 || fv || cv) begin
            $display("FAIL: clear did not recover filter status"); errors = errors + 1;
        end
        clr = 0; recovering = 1;
        for (i = 0; i < 40; i = i + 1) begin
            valid = 1;
            @(negedge clk); valid = 0;
            repeat (200) @(negedge clk);
        end
        if (fn == 0 || cn == 0 || fov || cov) begin
            $display("FAIL: filters failed to resume at legal rate"); errors = errors + 1;
        end
        // Control-register injection; I2C transactions are covered by tb_mux.
        force dut.ctl_sel = 2'd1;
        repeat (2200) @(negedge clk);
        force dut.ctl_sel = 2'd2; // stopped clock
        repeat (2200) @(negedge clk);
        force dut.ctl_sel = 2'd0; // recover from stopped clock
        repeat (2200) @(negedge clk);
        release dut.ctl_sel;
        wait (asrc_done);
        if (switches != 3) begin
            $display("FAIL: not all input switches completed"); errors = errors + 1;
        end
        if (errors) $fatal(1, "recovery: %0d errors", errors);
        $display("PASS recovery: overflow clear, resume, live/dead clock switching");
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1, "recovery timeout");
    end
endmodule
