module APB_FSM_Controller(
    input Hclk, Hresetn, valid, Hwrite, Hwritereg,
    input [31:0] Haddr, Haddr1, Hwdata, Hwdata1, Prdata,
    input [1:0] Htrans,
    input [2:0] tempselx,
    output reg Pwrite, Penable, Hreadyout,
    output reg [2:0] Pselx,
    output reg [31:0] Paddr, Pwdata
);
    parameter ST_IDLE = 3'b000, ST_WWAIT = 3'b001, ST_READ = 3'b010,
              ST_WRITE = 3'b011, ST_WRITEP = 3'b100, ST_RENABLE = 3'b101,
              ST_WENABLE = 3'b110, ST_WENABLEP = 3'b111;

    reg [2:0] PRESENT_STATE, NEXT_STATE;

    always @(posedge Hclk or negedge Hresetn) begin
        if (~Hresetn) PRESENT_STATE <= ST_IDLE;
        else PRESENT_STATE <= NEXT_STATE;
    end

    // Clean Next-State Logic (No Mealy hacks)
    always @(*) begin
        NEXT_STATE = ST_IDLE;
        case(PRESENT_STATE)
            ST_IDLE: begin
                if (valid && Hwrite) NEXT_STATE = ST_WWAIT;
                else if (valid && ~Hwrite) NEXT_STATE = ST_READ;
                else NEXT_STATE = ST_IDLE;
            end
            ST_WWAIT:  NEXT_STATE = ST_WRITE;  // Unconditional transition during stall
            ST_READ:   NEXT_STATE = ST_RENABLE;
            ST_WRITE:  NEXT_STATE = ST_WENABLE;
            ST_WRITEP: NEXT_STATE = ST_WENABLEP;
            ST_RENABLE, ST_WENABLE, ST_WENABLEP: begin
                if (valid && ~Hwrite) NEXT_STATE = ST_READ;
                else if (valid && Hwrite) NEXT_STATE = ST_WRITEP; // Pipeline the next burst
                else NEXT_STATE = ST_IDLE;
            end
            default: NEXT_STATE = ST_IDLE;
        endcase
    end

    // Moore Hreadyout: 1 in IDLE allows the master to drive the data phase.
    always @(*) begin
        case(PRESENT_STATE)
            ST_IDLE, ST_RENABLE, ST_WENABLE, ST_WENABLEP: Hreadyout = 1'b1;
            default: Hreadyout = 1'b0;
        endcase
    end

    reg [31:0] paddr_reg, pwdata_reg;
    reg [2:0] pselx_reg;

    always @(posedge Hclk or negedge Hresetn) begin
        if (~Hresetn) begin
            paddr_reg <= 0; pwdata_reg <= 0; pselx_reg <= 0;
        end else begin
            // 1. Capture Address when bus is free
            if ((PRESENT_STATE == ST_IDLE || PRESENT_STATE == ST_RENABLE || 
                 PRESENT_STATE == ST_WENABLE || PRESENT_STATE == ST_WENABLEP) && valid) begin
                paddr_reg <= Haddr;
                pselx_reg <= tempselx;
            end

            // 2. Capture Data exactly when the Master is stalled and holding it
            if (PRESENT_STATE == ST_WWAIT || PRESENT_STATE == ST_WRITEP) begin
                pwdata_reg <= Hwdata;
            end
        end
    end

    // APB Bus Routing
    always @(*) begin
        Pwrite = 1'b0; Penable = 1'b0; Pselx = 3'b000; Paddr = 32'h0; Pwdata = 32'h0;
        case(PRESENT_STATE)
            ST_READ: begin Pselx = pselx_reg; Paddr = paddr_reg; Pwrite = 1'b0; Penable = 1'b0; end
            ST_RENABLE: begin Pselx = pselx_reg; Paddr = paddr_reg; Pwrite = 1'b0; Penable = 1'b1; end
            ST_WRITE, ST_WRITEP: begin Pselx = pselx_reg; Paddr = paddr_reg; Pwdata = pwdata_reg; Pwrite = 1'b1; Penable = 1'b0; end
            ST_WENABLE, ST_WENABLEP: begin Pselx = pselx_reg; Paddr = paddr_reg; Pwdata = pwdata_reg; Pwrite = 1'b1; Penable = 1'b1; end
        endcase
    end
endmodule