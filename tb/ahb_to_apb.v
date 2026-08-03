
`timescale 1ps/1ps

module ahb_to_apb ();
    reg Hclk;
    reg Hresetn;

    initial Hclk    = 1'b0;     
    initial Hresetn = 1'b0;     

    always #10 Hclk = ~Hclk;

    wire        Hwrite;         // master→bridge: write enable
    wire        Hreadyin;       // master→bridge: master's own HREADY
    wire        Hreadyout;      // bridge→master: stall / accept
    wire [1:0]  Htrans;         // master→bridge: transfer type
    wire [31:0] Haddr;          // master→bridge: address
    wire [31:0] Hwdata;         // master→bridge: write data
    wire [31:0] Hrdata;         // bridge→master: read data
    wire [1:0]  Hresp;          // bridge→master: response (always OKAY)

    wire        Pwrite;         // bridge→slave: write enable
    wire        Penable;        // bridge→slave: enable (phase 2)
    wire [2:0]  Pselx;          // bridge→slave: slave select
    wire [31:0] Paddr;          // bridge→slave: address
    wire [31:0] Pwdata;         // bridge→slave: write data
    wire [31:0] Prdata;         // slave→bridge: read data

    wire        Pwriteout;
    wire        Penableout;
    wire [2:0]  Pselxout;
    wire [31:0] Paddrout;
    wire [31:0] Pwdataout;

    AHB_Master ahb_M1 (
        .Hclk      (Hclk),
        .Hresetn   (Hresetn),
        .Hreadyout (Hreadyout),
        .Hresp     (Hresp),
        .Hrdata    (Hrdata),
        .Hwrite    (Hwrite),
        .Hreadyin  (Hreadyin),
        .Htrans    (Htrans),
        .Hwdata    (Hwdata),
        .Haddr     (Haddr)
    );

    Bridge_Top bt1 (
        .HCLK      (Hclk),
        .HRESETn   (Hresetn),
        .HWRITE    (Hwrite),
        .HREADYin  (Hreadyin),
        .HADDR     (Haddr),
        .HWDATA    (Hwdata),
        .HTRANS    (Htrans),
        .PRDATA    (Prdata),
        .PWRITE    (Pwrite),
        .PENABLE   (Penable),
        .PSELx     (Pselx),
        .PADDR     (Paddr),
        .PWDATA    (Pwdata),
        .HREADYout (Hreadyout),
        .HRESP     (Hresp),
        .HRDATA    (Hrdata)
    );

    APB_Interface apb_i1 (
        .Pclk      (Hclk),         // same system clock — APB uses PCLK = HCLK here
        .Pwrite    (Pwrite),
        .Penable   (Penable),
        .Pselx     (Pselx),
        .Paddr     (Paddr),
        .Pwdata    (Pwdata),
        .Pwriteout (Pwriteout),
        .Penableout(Penableout),
        .Pselxout  (Pselxout),
        .Paddrout  (Paddrout),
        .Pwdataout (Pwdataout),
        .Prdata    (Prdata)
    );

    task reset;
    begin
        @(negedge Hclk);    // wait one half-cycle (Hresetn already 0)
        Hresetn = 1'b0;     // ensure low (redundant but explicit)
        @(negedge Hclk);    // hold for one full cycle
        Hresetn = 1'b1;     // de-assert reset — DUT now running
    end
    endtask

    initial begin
        $dumpfile("ahb_to_apb.vcd");
        $dumpvars(0, ahb_to_apb);

        reset();

        // ---- Test 1: Single Write ------------------------------------------
        $display("\n[TB] ===== Single Write =====");
        ahb_M1.single_write();
        #40;

        // ---- Test 2: Single Read -------------------------------------------
        $display("\n[TB] ===== Single Read =====");
        ahb_M1.single_read();
        #40;

        // ---- Test 3: 4-beat INCR burst write -------------------------------
        $display("\n[TB] ===== Burst INCR4 Write =====");
        ahb_M1.burst_4_incr_write();
        #40;

        // ---- Test 4: 4-beat WRAP4 burst write ------------------------------
        $display("\n[TB] ===== Burst WRAP4 Write =====");
        ahb_M1.burst_write_wrap4();
        #40;

        $display("\n[TB] All tests complete.");
        $finish;
    end

    initial #5000 begin
        $display("[TB] TIMEOUT — simulation did not finish within 5000 ps.");
        $finish;
    end

    always @(posedge Hclk) begin
        if (Penable && |Pselx) begin
            $display("[T=%4t ns] APB %-5s | PSELx=%03b | PADDR=0x%08h | DATA=0x%08h | HREADYOUT=%b",
                $time,
                Pwrite ? "WRITE" : "READ",
                Pselx,
                Paddr,
                Pwrite ? Pwdata : Prdata,
                Hreadyout);
        end
    end

    always @(posedge Hclk) begin
        if (~Pwrite && Penable && |Pselx && Hreadyout)
            $display("[T=%4t ns] AHB  READ  | HRDATA=0x%08h (returned to master)", $time, Hrdata);
    end

endmodule
