

module APB_Interface (

    input  wire        Pclk,
    input  wire        Pwrite,     
    input  wire        Penable,    
    input  wire [2:0]  Pselx,      
    input  wire [31:0] Pwdata,  
    input  wire [31:0] Paddr,     
    output wire        Pwriteout,
    output wire        Penableout,
    output wire [2:0]  Pselxout,
    output wire [31:0] Pwdataout,
    output wire [31:0] Paddrout,
    output reg  [31:0] Prdata
);

    assign Penableout = Penable;
    assign Pselxout   = Pselx;
    assign Pwriteout  = Pwrite;
    assign Paddrout   = Paddr;
    assign Pwdataout  = Pwdata;
    reg [31:0] mem [0:15];
    integer k;

    initial begin
        for (k = 0; k < 16; k = k + 1)
            mem[k] = 32'hDEAD_0000 | k[31:0]; 
    end

    always @(posedge Pclk) begin
        if (Penable && Pwrite && |Pselx)
            mem[Paddr[5:2]] <= Pwdata;
    end

    always @(*) begin
        if (Penable && ~Pwrite && |Pselx)
            Prdata = mem[Paddr[5:2]]; // combinational read
        else
            Prdata = 32'h0;           
    end

endmodule
