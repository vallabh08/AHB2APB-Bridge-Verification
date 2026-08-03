# AMBA AHB-to-APB Bridge

A fully synthesizable, AMBA-compliant AHB-to-APB protocol bridge implemented in Verilog. Designed and verified as part of an internship project at **Maven Silicon**.

---

## Overview

The bridge operates as a **slave on the AHB bus** and a **master on the APB bus**. It absorbs the pipelined, high-speed AHB burst transfers (Single, INCR4, WRAP4) and converts them into the sequential two-cycle APB transfers (SETUP → ENABLE) without data loss.

The core challenge this design solves: AHB is pipelined (address phase and data phase are offset by one cycle), while APB is strictly non-pipelined. The bridge must align these phases correctly across burst transfers while stalling the AHB master as needed.

---

## Architecture

```
AHB Master
    │
    ▼
┌─────────────────────┐
│  AHB Slave Interface │  ← pipeline-delay registers (Haddr1, Hwdata1)
│     (AHBSlave)       │  ← address decode → tempselx
│                      │  ← valid signal generation
└──────────┬──────────┘
           │ valid, Haddr1, Hwdata1, tempselx
           ▼
┌─────────────────────┐
│  APB FSM Controller  │  ← 8-state Moore FSM
│    (APBControl)      │  ← drives PSELx, PENABLE, PWRITE, PADDR, PWDATA
│                      │  ← controls HREADYout to stall AHB master
└──────────┬──────────┘
           │
           ▼
      APB Peripheral
```

---

## Sub-blocks

### 1. `AHB_slave_interface` (AHBSlave)

- Monitors AHB bus and decodes incoming addresses
- Generates `tempselx` slave select based on address range (`0x8000_0000` to `0x8000_03FF` → `PSELx[0]`)
- Implements **pipeline-delay registers** (`Haddr1`, `Hwdata1`, `Hwritereg`) clocked on `Hreadyin` to align AHB address phase with the subsequent data phase
- Asserts `valid` only when `Hreadyin=1` and `HTRANS` is NONSEQ (`2'b10`) or SEQ (`2'b11`)
- Passes `PRDATA` directly back as `HRDATA`; `HRESP` is always OKAY (`2'b00`)

### 2. `APB_FSM_Controller` (APBControl)

An **8-state Moore FSM** that handles the protocol conversion:

| State | Encoding | Description |
|---|---|---|
| `ST_IDLE` | `3'b000` | Waiting for valid transfer |
| `ST_WWAIT` | `3'b001` | Write: waiting one cycle (AHB pipeline delay) |
| `ST_READ` | `3'b010` | Read SETUP phase — PSELx asserted, PENABLE=0 |
| `ST_WRITE` | `3'b011` | Write SETUP phase — PSELx asserted, PENABLE=0 |
| `ST_WRITEP` | `3'b100` | Write SETUP — pipelined next burst pending |
| `ST_RENABLE` | `3'b101` | Read ENABLE phase — PENABLE=1, HREADYout=1 |
| `ST_WENABLE` | `3'b110` | Write ENABLE phase — PENABLE=1, HREADYout=1 |
| `ST_WENABLEP` | `3'b111` | Write ENABLE — pipelined next burst pending |

**Key FSM behavior:**
- `HREADYout = 1` in IDLE, RENABLE, WENABLE, WENABLEP (bus free)
- `HREADYout = 0` in all other states (master stalled)
- Address and PSELx captured from `Haddr`/`tempselx` at IDLE/ENABLE exit when `valid=1`
- Write data captured from `Hwdata` during WWAIT and WRITEP states

---

## Signal Interface

### AHB Side (Slave)

| Signal | Dir | Width | Description |
|---|---|---|---|
| `HCLK` | In | 1 | System clock |
| `HRESETn` | In | 1 | Active-low reset |
| `HADDR` | In | 32 | AHB address bus |
| `HWDATA` | In | 32 | AHB write data |
| `HTRANS` | In | 2 | Transfer type (IDLE/BUSY/NONSEQ/SEQ) |
| `HWRITE` | In | 1 | Write=1, Read=0 |
| `HREADYin` | In | 1 | AHB master ready |
| `HRDATA` | Out | 32 | AHB read data |
| `HREADYout` | Out | 1 | Bridge ready (stalls master when 0) |
| `HRESP` | Out | 2 | Response (always OKAY) |

### APB Side (Master)

| Signal | Dir | Width | Description |
|---|---|---|---|
| `PADDR` | Out | 32 | APB address |
| `PWDATA` | Out | 32 | APB write data |
| `PWRITE` | Out | 1 | Write=1, Read=0 |
| `PSELx` | Out | 3 | Peripheral select (one-hot) |
| `PENABLE` | Out | 1 | APB enable (ENABLE phase) |
| `PRDATA` | In | 32 | APB read data from peripheral |

---

## File Structure

```
ahb2apb-bridge/
├── README.md
├── rtl/
│   ├── Bridge_Top.v              # Top-level structural wrapper
│   ├── AHB_slave_interface.v     # AHB slave + pipeline registers
│   └── APB_FSM_Controller.v     # 8-state Moore FSM
├── tb/
│   └── tb_ahb2apb.v              # Testbench (Single, INCR4, WRAP4)
├── sim/
│   └── Makefile                  # iverilog compile + simulate
├── docs/
│   └── AHB2APB_report.pdf        # Maven Silicon internship report
└── .gitignore
```

---

## Simulation

Verified using **ModelSim**. The testbench covers:
- Single write and read transfers
- INCR4 burst transfers
- WRAP4 burst transfers

Waveforms confirm:
- Correct FSM state sequencing
- Pipeline alignment (`Haddr1`/`Hwdata1` match their data phases)
- No beat skipping across burst transfers
- `HREADYout` correctly stalls the master during bridge processing

To simulate with Icarus Verilog:
```bash
cd sim
make        # runs iverilog + vvp
```

Example `Makefile`:
```makefile
SRC = ../rtl/Bridge_Top.v ../rtl/AHB_slave_interface.v ../rtl/APB_FSM_Controller.v
TB  = ../tb/tb_ahb2apb.v

sim:
	iverilog -o bridge_sim $(SRC) $(TB)
	vvp bridge_sim

clean:
	rm -f bridge_sim *.vcd
```

---

## Synthesis Results

Synthesized targeting **Intel MAX 10 FPGA** (10M04DCU324I7G) using **Quartus Prime 25.1**.

| Metric | Result |
|---|---|
| Total Logic Elements | 149 |
| Total Registers | 71 |
| Total Pins | 206 |
| Memory Bits | 0 |
| PLLs | 0 |
| Timing Models | Final |
| Latches | 0 (latch-free) |

Synthesis status: **Successful** (Jun 17, 2026)

---

## Key Design Decisions

**Problem 1 — Pipeline phase misalignment:** AHB address and data phases are offset by one cycle. Direct forwarding causes address-data mismatch during bursts.  
**Solution:** `Haddr1`/`Hwdata1` capture registers in the AHB slave, clocked on `Hreadyin`, shift the address to align with its corresponding data cycle.

**Problem 2 — Combinational feedback loop:** Using `HREADYout` directly in the `valid` generation logic creates a zero-delay loop.  
**Solution:** `valid` is generated from `HREADYin` (the master's ready, not the bridge's output), breaking the combinational path.

**Problem 3 — Burst data loss:** During back-to-back burst beats, write data must be captured before the FSM transitions to ENABLE state.  
**Solution:** `pwdata_reg` is captured in WWAIT and WRITEP states, one cycle before the WRITE/WENABLEP SETUP assertion.

---

## About

**Author:** Vallabh Gondkar  
**Institution:** NIT Karnataka, Surathkal — B.Tech EEE  
**Internship:** Maven Silicon (Batch DI 66)  
**Tools:** Verilog, ModelSim, Quartus Prime, Intel MAX 10 FPGA
