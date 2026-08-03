module AHB_slave_interface(
    input Hclk, Hresetn, Hwrite, Hreadyin,
    input [31:0] Haddr, Hwdata, Prdata,
    input [1:0] Htrans,
    output reg valid, Hwritereg,
    output reg [31:0] Haddr1, Hwdata1,
    output [31:0] Hrdata,
    output reg [2:0] tempselx,
    output [1:0] Hresp
);
    assign Hrdata = Prdata;
    assign Hresp = 2'b00;

    // AHB Pipeline correctly gated by the Master's HREADYin
    always @(posedge Hclk or negedge Hresetn) begin
        if (~Hresetn) begin
            Haddr1 <= 0; Hwdata1 <= 0; Hwritereg <= 0;
        end else if (Hreadyin) begin
            Haddr1 <= Haddr;
            Hwdata1 <= Hwdata;
            Hwritereg <= Hwrite;
        end
    end

    // Address decode & Valid logic
    always @(*) begin
        valid = 0; tempselx = 0;
        if (Haddr >= 32'h8000_0000 && Haddr < 32'h8000_0400) begin
            tempselx = 3'b001;
            // AMBA Spec: Transfer only valid when bus isn't stalled (HREADYin is HIGH)
            if (Hreadyin && (Htrans == 2'b10 || Htrans == 2'b11)) begin
                valid = 1'b1;
            end
        end
    end
endmodule