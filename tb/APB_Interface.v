// =============================================================================
// Module  : APB_Interface
// Project : AHB-to-APB Bridge
// Purpose : Dummy APB peripheral — 16-word register file (word-addressed by
//           Paddr[5:2]) that acts as the APB slave on the far side of the
//           bridge.  Receives APB transactions from APB_FSM_Controller and
//           performs registered writes / combinational (ENABLE-gated) reads.
//
// Bug Fixes Applied
// -----------------
// BUG-5  (Major): The write always @(*) block assigned mem[Paddr[5:2]]=Pwdata
//         without an else branch.  This inferred a 16-element TRANSPARENT
//         LATCH array — not synthesizable to any RAM primitive, and glitch-
//         sensitive because any noise on Paddr/Pwdata during ENABLE phase
//         could corrupt memory.  APB write data must be registered (captured
//         on the rising clock edge during ENABLE phase).
//         FIX : Replaced the combinational write block with a clocked
//               always @(posedge Pclk) block.  Write occurs exactly at the
//               rising edge of PCLK when Penable=1, Pwrite=1, and |Pselx=1
//               — matching the APB specification exactly.  No latches inferred.
//               NOTE: This design uses Hclk as the single system clock; the
//               port is named Pclk here for APB semantic clarity.
//
// BUG-15 (Major): The read always @(*) block fired whenever ~Pwrite && |Pselx,
//         which includes the SETUP phase (Penable=0, Pselx=1).  APB spec
//         requires read data to be valid and stable at the END of the ENABLE
//         phase (rising edge of PCLK with PENABLE=1).  Presenting data one
//         full cycle early exposes Prdata to address-glitch noise during SETUP
//         and is a protocol violation.
//         FIX : Added Penable to the read gate condition:
//               if (Penable && ~Pwrite && |Pselx) → drive Prdata from mem.
//               Data is now only presented during the ENABLE phase.
//
// BUG-18 (Major): Two always @(*) blocks shared the mem[] array — one writing
//         and one reading.  Verilog's event scheduler does not guarantee
//         evaluation order between concurrent always blocks.  A write followed
//         by a read to the same address in consecutive APB cycles produced
//         simulator-dependent (non-deterministic) Prdata values.
//         FIX : Automatically resolved by BUG-5 fix.  Write is now clocked
//               (always @(posedge Pclk)); it completes as a non-blocking
//               assignment and is never visible to the combinational read block
//               in the same simulation time step.  The race condition is gone.
// =============================================================================

module APB_Interface (
    // -------------------------------------------------------------------------
    // Clock — shared system clock (same as AHB Hclk in this design).
    // Named Pclk here for APB semantic clarity.
    // -------------------------------------------------------------------------
    input  wire        Pclk,

    // -------------------------------------------------------------------------
    // APB slave inputs
    // -------------------------------------------------------------------------
    input  wire        Pwrite,     // 1=write, 0=read
    input  wire        Penable,    // APB enable (asserted in phase 2 / ENABLE)
    input  wire [2:0]  Pselx,      // APB slave select (this slave responds to any non-zero)
    input  wire [31:0] Pwdata,     // APB write data
    input  wire [31:0] Paddr,      // APB address (word-addressed on bits [5:2])

    // -------------------------------------------------------------------------
    // Pass-through monitoring outputs (to testbench)
    // -------------------------------------------------------------------------
    output wire        Pwriteout,
    output wire        Penableout,
    output wire [2:0]  Pselxout,
    output wire [31:0] Pwdataout,
    output wire [31:0] Paddrout,

    // -------------------------------------------------------------------------
    // APB read-data output (to bridge, then to AHB master)
    // -------------------------------------------------------------------------
    output reg  [31:0] Prdata
);

    // =========================================================================
    // Pass-through monitoring assigns — waveform visibility only
    // =========================================================================
    assign Penableout = Penable;
    assign Pselxout   = Pselx;
    assign Pwriteout  = Pwrite;
    assign Paddrout   = Paddr;
    assign Pwdataout  = Pwdata;

    // =========================================================================
    // 16-word register file
    // Word-addressed: mem[0] → Paddr[5:2]=4'h0 … mem[15] → Paddr[5:2]=4'hF
    // Pre-loaded with known values so read-before-write returns visible data.
    // =========================================================================
    reg [31:0] mem [0:15];
    integer k;

    initial begin
        for (k = 0; k < 16; k = k + 1)
            mem[k] = 32'hDEAD_0000 | k[31:0]; // e.g. mem[0]=0xDEAD0000, mem[1]=0xDEAD0001
    end

    // =========================================================================
    // Clocked write — APB ENABLE phase
    //
    // BUG-5 FIX: Was always @(*) { if(Penable&&Pwrite&&|Pselx) mem[...]=... }
    //            → inferred transparent latch array, not synthesizable.
    //
    // Write is now captured on the rising clock edge when all three APB
    // ENABLE-phase qualifiers are true:
    //   Penable=1  → ENABLE phase (not SETUP)
    //   Pwrite=1   → write transaction
    //   |Pselx=1   → this slave is selected
    //
    // BUG-18 FIX: Because the write is now a non-blocking clocked assignment,
    //             it completes atomically at the clock edge and is NEVER visible
    //             to the combinational read block in the same delta cycle.
    //             The read/write ordering race is completely eliminated.
    // =========================================================================
    always @(posedge Pclk) begin
        if (Penable && Pwrite && |Pselx)
            mem[Paddr[5:2]] <= Pwdata; // non-blocking: safe, synthesizable, race-free
    end

    // =========================================================================
    // Combinational read — APB ENABLE phase only
    //
    // BUG-15 FIX: Added 'Penable' to the read condition.
    //             Previously fired during SETUP (Penable=0) as well,
    //             presenting read data one cycle early and exposing Prdata
    //             to address-glitch noise.
    //
    // Now Prdata is only driven when:
    //   Penable=1  → ENABLE phase (APB-spec compliant data-valid window)
    //   ~Pwrite    → read transaction
    //   |Pselx=1   → this slave is selected
    //
    // Explicit else clause prevents latch inference on Prdata (it is an
    // output reg — without the else it would latch its last value).
    // =========================================================================
    always @(*) begin
        if (Penable && ~Pwrite && |Pselx)
            Prdata = mem[Paddr[5:2]]; // combinational read — valid during ENABLE
        else
            Prdata = 32'h0;            // explicit default — no latch inferred
    end

endmodule
