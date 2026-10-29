# uNPU: 4x4 INT8 Systolic Array Accelerator (RTL to GDSII, SCL 180nm)

A 4x4 weight-stationary systolic array using the Diagonal-Input Permutated (DiP)
dataflow, with a DMA engine, control unit and APB register interface.

## Status
| Stage | Result |
|---|---|
| RTL simulation (Xcelium) | 122 / 122 checks pass |
| Synthesis (Design Compiler, SCL 180nm, ss corner) | Timing met at 20 ns clock (50 MHz), 0 setup/hold violations |
| Area | ~10.8k cells, ~0.31 mm² |
| Gate-level simulation | 122 / 122 checks pass |
| Floorplan, place and route | In progress |

## Layout
- `rtl/` design sources
- `tb/` self-checking testbench (reset, APB register map, invalid configs, extreme values, abort sweep, back-to-back, random regression)
- `constraints/` SDC
- `sim/run_sim.sh` RTL simulation
- `syn/synth.tcl` synthesis script (set `SCL_LIB_DIR` to your library path)

PDK files and synthesized netlists are not included, because the SCL PDK is licensed.
