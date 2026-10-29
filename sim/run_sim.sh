#!/bin/bash
# RTL simulation (Cadence Xcelium). Run from sim/
xrun -sv -access +rwc -input waves.tcl -top tb_unpu_top \
  ../rtl/mac_unit.sv ../rtl/pe.sv ../rtl/systolic_array_dip_4x4.sv \
  ../rtl/unpu_dma_fifo2.sv ../rtl/unpu_dma_datapath.sv ../rtl/unpu_dma_ctrl.sv \
  ../rtl/unpu_dma.sv ../rtl/controller_dma_cu.sv ../rtl/APB.sv ../rtl/unpu_top.sv \
  ../tb/tb_unpu_top.sv -l run.log
