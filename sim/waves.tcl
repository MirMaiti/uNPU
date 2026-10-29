# waves.tcl
# --------------------------------------------------------------
# SimVision waveform-dump setup for tb_unpu_top.
#
# Usage (source it BEFORE the sim runs, via -input):
# xrun -access +rwc -input waves.tcl -top tb_unpu_top \
#      ../rtl/mac_unit.sv \
#      ../rtl/pe.sv \
#      ../rtl/systolic_array_dip_4x4.sv \
#      ../rtl/unpu_dma_fifo2.sv \
#      ../rtl/unpu_dma_datapath.sv \
#      ../rtl/unpu_dma_ctrl.sv \
#      ../rtl/unpu_dma.sv \
#      ../rtl/controller_dma_cu.sv \
#      ../rtl/APB.sv \
#      ../rtl/unpu_top.sv \
#      ../tb/tb_unpu_top.sv \
#      -l run.log

# Then view it with:
#   simvision waves.shm &
#
# -access +rwc is required at the xrun/elaborate step: by default
# Xcelium optimizes away (or makes read-only) any signal it decides
# isn't needed for correctness, so `probe -create -all` on a design
# elaborated without +rwc silently captures nothing for those signals.
# +rwc forces full read/write/connect visibility everywhere so probing
# actually works.
# --------------------------------------------------------------

# Open (create) a Cadence SHM waveform database named "waves.shm"
database -open waves -shm -default

# Recursively probe every signal -- ports, internal regs/wires, all
# levels of hierarchy, starting at the top of the elaborated design
# (array_dma_tb). This reaches dut and every PE instance inside it
# (weight_reg, input_reg, mac_unit's mult_reg/psum_reg, etc.), not
# just the testbench's own top-level signals.
probe -create -all -depth all -database waves

# Run to completion ($finish in the testbench ends this)
run

# Exit the sim once done. Drop this line if you'd rather stay in the
# xrun shell (or -gui window) and drive SimVision interactively instead.
exit
