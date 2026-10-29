    #################### Setup ####################
set DESIGN   unpu_top
set RTL_DIR  ../rtl
set LIB_DIR  $::env(SCL_LIB_DIR)   ;# path to your SCL 180nm liberty folder
set LIB_DB   tsl18fs120_scl_ss.db

define_design_lib WORK -path ./work

set_app_var search_path      [list . $RTL_DIR $LIB_DIR ../constraints]
set_app_var target_library   $LIB_DB
set_app_var link_library     [list * $LIB_DB]

#################### Read RTL ####################
# Leaf modules first. No testbench files.
set RTL_FILES [list \
    $RTL_DIR/mac_unit.sv \
    $RTL_DIR/pe.sv \
    $RTL_DIR/systolic_array_dip_4x4.sv \
    $RTL_DIR/controller_dma_cu.sv \
    $RTL_DIR/unpu_dma_fifo2.sv \
    $RTL_DIR/unpu_dma_datapath.sv \
    $RTL_DIR/unpu_dma_ctrl.sv \
    $RTL_DIR/unpu_dma.sv \
    $RTL_DIR/APB.sv \
    $RTL_DIR/unpu_top.sv ]

analyze -format sverilog $RTL_FILES
elaborate $DESIGN
current_design $DESIGN
set_structure true -timing true  
set_flatten true -effort high
ungroup -all -flatten -simple_names
link
uniquify
check_design > reports/check_design_pre.rpt

#################### Constraints ####################
source ../constraints/template.sdc
check_timing > reports/check_timing_pre.rpt


#################### Compile ####################
compile_ultra
compile_ultra -incremental
compile_ultra -incremental -only_design_rule
#################### Reports ####################
check_design                       > reports/check_design_post.rpt
check_timing                       > reports/check_timing_post.rpt
report_timing -max_paths 10 -delay_type max > reports/timing_setup.rpt
report_timing -max_paths 10 -delay_type min > reports/timing_hold.rpt
report_area -hierarchy             > reports/area.rpt
report_power                       > reports/power.rpt
report_qor                         > reports/qor.rpt
report_constraint -all_violators   > reports/violators.rpt

#################### Outputs ####################
change_names -rules verilog -hierarchy
write -format verilog -hierarchy -output results/${DESIGN}_netlist.v
write -format ddc     -hierarchy -output results/${DESIGN}.ddc
write_sdc results/${DESIGN}.sdc
exit
