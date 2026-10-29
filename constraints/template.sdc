##############################################################################
#
#  template.sdc  --  Synopsys Design Constraints template
#
#  DESIGN   : <top_module_name>
#  MODE     : functional            (make a separate file per mode)
#  CORNER   : worst-case setup / best-case hold
#  STAGE    : pre-CTS               (see STAGE SWITCHES in section 2)
#  AUTHOR   :
#  REVISION :
#
#  HOW TO USE
#    1. Fill in every <angle-bracket> placeholder and every "EDIT:" line.
#    2. Delete any section that does not apply -- do not leave dead
#       constraints behind; an unused constraint that silently matches
#       nothing is worse than no constraint at all.
#    3. Run check_timing and report_exceptions -ignored afterwards.
#       Search this file for "SIGN-OFF CHECKLIST" at the bottom.
#
#  ORDERING RULE
#    Clocks -> clock properties -> clock groups -> modes -> I/O ->
#    environment -> design rules -> exceptions.
#    Nothing may reference a clock before it is created, and exceptions
#    must come last so their precedence is obvious.
#
#  This file is Tcl. It is executed, not parsed. Use variables and loops
#  freely, but remember that bus brackets need escaping: data\[3\] or
#  [get_ports {data[3]}], never [get_ports data[3]].
#
##############################################################################


##############################################################################
# SECTION 0 -- UNITS
##############################################################################
# SDC numbers carry no implicit units; they mean whatever the linked library
# means. Declaring units here documents intent and catches a ps-vs-ns library
# mismatch early. Confirm against the library with report_units.

set_units -time ns -capacitance pF -voltage V -current uA -resistance kOhm


##############################################################################
# SECTION 1 -- PARAMETERS
##############################################################################
# Every magic number in this file lives here. Retargeting the block to a new
# frequency should mean editing this section only.

# --- clock ports and periods -----------------------------------------------
set CLK_PORT            clk                 ;# EDIT: primary clock port name
set CLK_PERIOD          20.000              ;
set CLK_NAME            SYS_CLK

#set JTAG_PORT           tck                 ;# EDIT: delete if no JTAG
#set JTAG_PERIOD         100.000
#set JTAG_NAME           JTAG_CLK

# --- ports that must never receive an input delay --------------------------
# All clock ports, plus any port handled by its own exception below.
set NO_IO_PORTS         [list $CLK_PORT]   ;# EDIT

# --- I/O budget ------------------------------------------------------------
# Fraction of the period given away to the world outside this block.
# Replace with real numbers from the system spec as soon as you have them;
# 30/30 is only a placeholder that keeps the block honest.
set IN_BUDGET_MAX       [expr {$CLK_PERIOD * 0.30}]
set IN_BUDGET_MIN       0.100
set OUT_BUDGET_MAX      [expr {$CLK_PERIOD * 0.30}]
set OUT_BUDGET_MIN      -0.050              ;# negative is normal and legal

# --- clock tree modelling --------------------------------------------------
set CLK_LATENCY_NET     0.000               ;# ideal insertion delay, pre-CTS only (was originally 0.8)
#set CLK_LATENCY_SRC     1.000               ;# off-chip / PLL-to-port, kept post-CTS
set CLK_TRANSITION      0.1               ;# ideal clocks only   (was originally 0.05)

# Pre-CTS uncertainty = jitter + estimated skew + margin.
# Post-CTS uncertainty = jitter + margin (real skew is now computed).
set UNCERT_SETUP_PRECTS 0.150
set UNCERT_HOLD_PRECTS  0.050
set UNCERT_SETUP_POSTCTS 0.080
set UNCERT_HOLD_POSTCTS  0.030

# --- design rule limits ----------------------------------------------------
set MAX_TRAN_DATA       1.1
set MAX_TRAN_CLK        0.300 ;# was originally 0.1
set MAX_CAP             0.800
set MAX_FANOUT          15

# --- I/O environment -------------------------------------------------------
set DRIVING_CELL        buffd4              ;# EDIT: real cell from your library
set DRIVING_PIN         Z                   ;# EDIT: its output pin
set OUTPUT_LOAD         0.040               ;# EDIT: pF seen by each output

# --- stage switch ----------------------------------------------------------
# Set to 1 once the clock tree is built. Controls section 2.
set POST_CTS            0                   ;# EDIT per stage


##############################################################################
# SECTION 2 -- CLOCK DEFINITIONS
##############################################################################
# Nothing in a timing tool works until clocks exist. Define them first.
#
# Default waveform is 50% duty, rising edge at t=0. Use -waveform {rise fall}
# to change it.

#----------------------------------------------------------------------------
# 2.1 Primary clocks -- clocks that enter the block from outside
#----------------------------------------------------------------------------
create_clock -name $CLK_NAME -period $CLK_PERIOD [get_ports $CLK_PORT]

##############################################################################
# SECTION 3 -- CLOCK PROPERTIES  (STAGE SWITCHES LIVE HERE)
##############################################################################
# Pre-CTS the clock tree does not exist, so it is modelled with ideal
# latency, transition and a generous uncertainty. Post-CTS the real tree
# is present: switch to propagated clocks, DROP the network latency
# (keeping it double-counts the tree) and tighten uncertainty.

if {$POST_CTS == 0} {
    #--- ideal clock model, pre-CTS -----------------------------------------
    set_clock_latency $CLK_LATENCY_NET [all_clocks]
    set_clock_transition $CLK_TRANSITION [all_clocks]
    set_clock_uncertainty -setup $UNCERT_SETUP_PRECTS [all_clocks]
    set_clock_uncertainty -hold  $UNCERT_HOLD_PRECTS  [all_clocks]
} else {
    #--- real clock tree, post-CTS ------------------------------------------
    set_propagated_clock [all_clocks]
    set_clock_uncertainty -setup $UNCERT_SETUP_POSTCTS [all_clocks]
    set_clock_uncertainty -hold  $UNCERT_HOLD_POSTCTS  [all_clocks]
    # No set_clock_latency and no set_clock_transition here -- both are now
    # computed from the actual tree. set_clock_transition is ignored anyway
    # once clocks are propagated.
}

# Source latency models delay OUTSIDE the block (PLL to port, board trace).
# It is NOT part of the internal clock tree, so it stays at every stage.
# set_clock_latency -source $CLK_LATENCY_SRC [get_clocks $CLK_NAME]

# Inter-clock uncertainty, if two domains need a different margin:
# set_clock_uncertainty -setup 0.250 -from [get_clocks CLK_A] -to [get_clocks CLK_B]


##############################################################################
# SECTION 4 -- CLOCK RELATIONSHIPS
##############################################################################
# Tell the tool which clocks must NOT be timed against each other.
# Clocks inside the same -group stay mutually timed; clocks in different
# groups have every cross-path cut, setup and hold, in both directions.
#
# Prefer this over a wall of set_false_path -from CLK_A -to CLK_B: it is
# symmetric, self-documenting, and does not rot when a clock is added.
#
# WARNING: -asynchronous also removes HOLD checks. If two domains are in
# fact related, grouping them apart hides real hold failures. Declare
# asynchronous only what is genuinely asynchronous and synchronised in RTL.

# Truly unrelated domains, crossed only through synchronisers:
#set_clock_groups -asynchronous -name async_domains \
#    -group [get_clocks [list $CLK_NAME]] \
#    -group [get_clocks [list $JTAG_NAME]]

# Clocks that never coexist because logic selects one (clock mux):
# set_clock_groups -logically_exclusive -name mode_clocks \
#     -group [get_clocks FUNC_CLK] \
#     -group [get_clocks SCAN_CLK]

# Two definitions on the same physical net (multi-mode on one port):
# set_clock_groups -physically_exclusive -name port_modes \
#     -group [get_clocks MODE_A_CLK] \
#     -group [get_clocks MODE_B_CLK]


##############################################################################
# SECTION 5 -- MODE SETUP / CASE ANALYSIS
##############################################################################
# Pin mode and test signals to their functional values. This prunes the
# unreachable logic from analysis entirely, which both speeds up STA and
# removes paths that cannot occur in this mode.
#
# Cleaner than false-pathing every branch of a mode mux. Each mode gets its
# own SDC file; sign off on all of them.

#set_case_analysis 0 [get_ports test_mode]       ;# EDIT
#set_case_analysis 0 [get_ports scan_en]         ;# EDIT
# set_case_analysis 1 [get_pins u_clk_mux/S]    ;# select functional clock

# Disable a specific timing arc (lockup latches, unused macro pins):
# set_disable_timing [get_pins u_lockup/CK]
# set_disable_timing -from A -to Y [get_cells u_special_gate]


##############################################################################
# SECTION 6 -- I/O TIMING
##############################################################################
# This is the part describing what the tool cannot see, and where most
# designs go wrong.
#
#   set_input_delay  = time AFTER the launch edge at which data becomes
#                      valid at this port. Larger -max leaves LESS time
#                      inside the block.  Larger -min makes hold EASIER.
#
#   set_output_delay = how much of the period the downstream block needs
#                      after data leaves this port. Larger -max again
#                      leaves less time for internal logic.
#
# Use -add_delay whenever a port is constrained against more than one clock
# or against both edges; without it the new constraint SILENTLY REPLACES
# the old one.

# Build the list of real data ports: [all_inputs] includes the clock,
# and constraining the clock port is wrong.
set DATA_IN [remove_from_collection [all_inputs] [get_ports $NO_IO_PORTS]]
set DATA_OUT [all_outputs]

set_input_delay  -clock $CLK_NAME -max $IN_BUDGET_MAX  $DATA_IN
set_input_delay  -clock $CLK_NAME -min $IN_BUDGET_MIN  $DATA_IN

set_output_delay -clock $CLK_NAME -max $OUT_BUDGET_MAX $DATA_OUT
set_output_delay -clock $CLK_NAME -min $OUT_BUDGET_MIN $DATA_OUT

# Falling-edge / DDR interface example:
# set_input_delay -clock $CLK_NAME -clock_fall -max 1.0 -add_delay [get_ports ddr_dq*]

# Port constrained against a second clock -- note -add_delay:
# set_input_delay -clock $JTAG_NAME -max 10.0 -add_delay [get_ports tdi]
# set_output_delay -clock $JTAG_NAME -max 10.0 -add_delay [get_ports tdo]


##############################################################################
# SECTION 7 -- I/O ENVIRONMENT
##############################################################################
# Without these the tool assumes an infinitely strong driver on every input
# and zero load on every output, and the resulting timing is fiction.

set_driving_cell -lib_cell $DRIVING_CELL -pin $DRIVING_PIN $DATA_IN
set_drive 0 [get_ports $CLK_PORT]               ;# clock port: ideal drive

set_load $OUTPUT_LOAD $DATA_OUT

# Load expressed in units of a library pin, which tracks library changes:
# set_load [expr {3 * [load_of <lib_name>/INVX1/A]}] [get_ports critical_out]

# set_input_transition 0.100 $DATA_IN           ;# alternative to set_driving_cell


##############################################################################
# SECTION 8 -- DESIGN RULE CONSTRAINTS
##############################################################################
# Not timing checks, but the tool honours them ahead of timing. DRC
# violations frequently CAUSE timing violations, so fix these first when
# debugging a stubborn path.

set_max_transition  $MAX_TRAN_DATA [current_design]
set_max_capacitance $MAX_CAP       [current_design]
set_max_fanout      $MAX_FANOUT    [current_design]

# Tighter transition on clock nets:
set_max_transition $MAX_TRAN_CLK -clock_path [all_clocks]


##############################################################################
# SECTION 9 -- TIMING EXCEPTIONS
##############################################################################
# Exceptions come LAST so their precedence is visible.
#
# PRECEDENCE, highest first:
#     set_false_path  >  set_max_delay / set_min_delay  >
#     set_multicycle_path  >  default single-cycle
#
# Within one type, the more specific object spec wins:
#     -from -through -to  >  -from -to  >  -from  >  -to
#
# This is why a broad -to [get_clocks X] false path can silently swallow a
# carefully written multicycle added later.
#
# EVERY exception below must carry a comment saying WHY. Six months on,
# nobody can reconstruct the reasoning, and an unexplained exception is one
# nobody dares delete.

#----------------------------------------------------------------------------
# 9.1 False paths -- removes BOTH setup and hold checking
#----------------------------------------------------------------------------

# Asynchronous reset is synchronised internally by u_rst_sync; the
# de-assertion edge is the only timed event and it is handled there.
set_false_path -from [get_ports rst_n]                          ;# EDIT

# CDC: two-flop synchroniser. Timing between the two stages is guaranteed
# by placement, not by this constraint.
# set_false_path -to [get_pins u_sync/sync_reg_reg[0]/D]

# Configuration registers written once at boot, read asynchronously.
# set_false_path -from [get_cells u_cfg/cfg_reg*] -to [get_cells u_core/*]

#----------------------------------------------------------------------------
# 9.2 Multicycle paths
#
#     THE HOLD LINE IS NOT OPTIONAL. By default the hold check follows the
#     setup check: relax setup to N cycles and hold lands at N-1 cycles,
#     becoming wildly pessimistic. The tool will then either report
#     impossible violations or stuff the design with delay buffers.
#
#     Same clock, same edge:   setup N  ->  hold N-1
#----------------------------------------------------------------------------

# 3-cycle multiplier; the handshake guarantees the operands stay stable.
# set_multicycle_path 3 -setup -from [get_cells u_mult/stage1*] \
#                              -to   [get_cells u_mult/result*]
# set_multicycle_path 2 -hold  -from [get_cells u_mult/stage1*] \
#                              -to   [get_cells u_mult/result*]

# Different frequencies: -start counts on the launch clock, -end on the
# capture clock. Defaults are -end for setup, -start for hold.
# set_multicycle_path 2 -setup -end   -from [get_clocks FAST] -to [get_clocks SLOW]
# set_multicycle_path 1 -hold  -start -from [get_clocks FAST] -to [get_clocks SLOW]

#----------------------------------------------------------------------------
# 9.3 Max / min delay -- absolute bounds, ignoring clock relationships.
#     Used for pure combinational feedthrough and asynchronous interfaces.
#----------------------------------------------------------------------------

# set_max_delay 1.500 -from [get_ports async_in] -to [get_ports async_out]
# set_min_delay 0.200 -from [get_ports async_in] -to [get_ports async_out]

# Bounding an async crossing instead of fully false-pathing it is the safer
# habit: the data path stays constrained even though the clock relationship
# is ignored.
# set_max_delay $CLK_PERIOD -from [get_cells u_src/data_reg*] \
#                           -to   [get_cells u_dst/data_reg*] -ignore_clock_latency


##############################################################################
#  SIGN-OFF CHECKLIST -- run these, read every line, before believing any
#  timing report. A clean report_timing with no coverage report is
#  meaningless: the tool may simply be checking nothing.
#
#    check_timing
#        no_clock                -> register clock pin with no clock. Dead logic.
#        unconstrained_endpoints -> paths with no capture event. Invisible.
#        no_input_delay          -> port left unbudgeted.
#        partial_input_delay     -> port constrained vs one clock, not another.
#        generated_clocks        -> generated clock source mismatch.
#        loops                   -> combinational loop broken arbitrarily.
#
#    report_clocks
#    report_clocks -skew -attributes
#    report_port -verbose
#    report_constraint -all_violators
#
#    report_exceptions -ignored    <- exceptions matching zero paths.
#                                     Almost always a typo in an object name.
#    report_analysis_coverage      <- percentage of endpoints actually checked.
#                                     Under 100% is not signed off.
#
#  Check what a wildcard really matched before trusting it:
#    query_objects [get_cells *sync*]
#
##############################################################################