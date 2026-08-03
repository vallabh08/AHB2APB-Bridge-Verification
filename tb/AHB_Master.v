
`timescale 1ps/1ps

module AHB_Master (
    input  wire        Hclk,
    input  wire        Hresetn,
    input  wire        Hreadyout,  // bridge's ready — master waits while low
    input  wire [1:0]  Hresp,      // AHB response from bridge (unused here)
    input  wire [31:0] Hrdata,     // AHB read data from bridge

    output reg         Hwrite,     // 1=write, 0=read
    output reg         Hreadyin,   // master's own HREADY (driven to 1 when active)
    output reg  [1:0]  Htrans,     // AHB transfer type
    output reg  [31:0] Hwdata,     // AHB write data
    output reg  [31:0] Haddr       // AHB address
);

    reg [2:0] Hburst; // burst type 
    reg [2:0] Hsize;  
    integer   i;

    initial begin
        Hwrite   = 1'b0;
        Hreadyin = 1'b0;
        Htrans   = 2'b00;   // IDLE
        Hwdata   = 32'h0;
        Haddr    = 32'h0;
        Hburst   = 3'b000;
        Hsize    = 3'b000;
    end


    function [31:0] wrap4_addr;
        input [31:0] base; // burst start address
        input [31:0] cur;  // current address
        input [2:0]  sz;   // Hsize
        reg   [31:0] incr;
        reg   [31:0] mask;
        reg   [31:0] next_raw;
        reg   [31:0] wrap_base;
        begin
            
            incr = (32'h1 << sz);
            mask = (32'h4 << sz) - 32'h1;
            wrap_base = base & ~mask;
            next_raw = cur + incr;
            wrap4_addr = wrap_base | (next_raw & mask);
        end
    endfunction
    
    task single_write;
    begin
        @(posedge Hclk) #2;
        Hwrite   = 1'b1;
        Htrans   = 2'b10;   // NONSE
        Hsize    = 3'b000;  // byte
        Hburst   = 3'b000;  // SINGLE
        Hreadyin = 1'b1;
        Haddr    = 32'h8000_0000;
        
        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        Hwdata = 32'hA4;   

        @(posedge Hclk) #2;
        Htrans = 2'b00;     // IDLE 
        Hwrite = 1'b0;
        Haddr  = 32'h0000_0000;
  
    end
    endtask

    task single_read;
    begin

        @(posedge Hclk) #2;
        Hwrite   = 1'b0;
        Htrans   = 2'b10;   // NONSEQ
        Hsize    = 3'b000;
        Hburst   = 3'b000;
        Hreadyin = 1'b1;
        Haddr    = 32'h8000_0000;


        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        Htrans = 2'b00;     // IDLE
    end
    endtask

    task burst_4_incr_write;
        reg [31:0] d [0:3]; 
        reg [31:0] base_addr;
    begin
        base_addr = 32'h8000_0000;

        d[0] = {$random} % 256;
        d[1] = {$random} % 256;
        d[2] = {$random} % 256;
        d[3] = {$random} % 256;

        @(posedge Hclk) #2;
        Hwrite   = 1'b1;
        Htrans   = 2'b10;   // NONSEQ
        Hsize    = 3'b000;  // byte
        Hburst   = 3'b011;  // INCR4
        Hreadyin = 1'b1;
        Haddr    = base_addr;           // B0

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);
        #2;
        Htrans = 2'b11;                 // SEQ
        Haddr  = base_addr + 1;        // B1
        Hwdata = d[0];                  // D0 

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        Htrans = 2'b11;                 // SEQ
        Haddr  = base_addr + 2;        // B2
        Hwdata = d[1];                  // D1

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        Htrans = 2'b11;                 // SEQ
        Haddr  = base_addr + 3;        // B3
        Hwdata = d[2];                  // D2 

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        Htrans = 2'b00;                 // IDLE — burst complete
        Haddr  = 32'h0000_0000;        // return to idle address
        Hwdata = d[3];                  // D3
        Hwrite = 1'b0;

        $display("[MASTER] burst_4_incr_write complete: D0=%0h D1=%0h D2=%0h D3=%0h",
                 d[0], d[1], d[2], d[3]);
    end
    endtask

    task burst_write_wrap4;
        reg [31:0] d [0:3];
        reg [31:0] burst_base;
        reg [31:0] cur_addr;
    begin
        burst_base = 32'h8000_0038;
        cur_addr   = burst_base;

        d[0] = {$random} % 1024;
        d[1] = {$random} % 1024;
        d[2] = {$random} % 1024;
        d[3] = {$random} % 1024;

        @(posedge Hclk) #2;
        Hwrite   = 1'b1;
        Hreadyin = 1'b1;
        Hsize    = 3'd0;  
        Htrans   = 2'b10;   // NONSEQ
        Hburst   = 3'b010;  // WRAP4
        Haddr    = cur_addr; // 0x8000_0038

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        Htrans   = 2'b11;   // SEQ
        cur_addr = wrap4_addr(burst_base, cur_addr, Hsize); // 0x39
        Haddr    = cur_addr;
        Hwdata   = d[0];    // D0

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        cur_addr = wrap4_addr(burst_base, cur_addr, Hsize); // 0x3A
        Haddr    = cur_addr;
        Hwdata   = d[1];    // D1
        Htrans   = 2'b11;

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        cur_addr = wrap4_addr(burst_base, cur_addr, Hsize); // 0x3B
        Haddr    = cur_addr;
        Hwdata   = d[2];    // D2
        Htrans   = 2'b11;

        @(posedge Hclk);
        while (~Hreadyout) @(posedge Hclk);

        #2;
        Hwrite = 1'b0;
        Hwdata = d[3];      
        Htrans = 2'b00;         // IDLE
        Haddr  = 32'h0000_0000; // release address bus


        begin : wrap_check
            reg [31:0] expected_wrap_back;
            expected_wrap_back = wrap4_addr(burst_base, cur_addr, Hsize);
            if (expected_wrap_back !== burst_base)
                $display("[ERROR] WRAP4 address did not return to base! got=%0h expected=%0h",
                         expected_wrap_back, burst_base);
            else
                $display("[MASTER] burst_write_wrap4 complete: WRAP4 verified. Base=0x%0h D0=%0h D1=%0h D2=%0h D3=%0h",
                         burst_base, d[0], d[1], d[2], d[3]);
        end
    end
    endtask

endmodule
