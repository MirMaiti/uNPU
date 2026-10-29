// =============================================================================
// unpu_dma  (top-level wrapper)
//
// DMA engine for a 4x4 INT8 weight-stationary systolic array. Sits between:
//   - the APB register file in unpu_top (control plane, already decoded here
//     into simple pulses/levels -- this module does NOT touch psel/penable/
//     pwrite/paddr/pwdata directly)
//   - the native SRAM master interface (dma_* ports, passed straight through
//     from unpu_top)
//   - the systolic-array Control Unit (CU), via a small handshake interface
//     defined below (cu_*). That CU is not part of this file.
//
// This file is a thin structural wrapper -- all behaviour lives in the
// submodules it instantiates:
//   unpu_dma_ctrl.v       the sequencing FSM (handshakes, busy/done/error)
//   unpu_dma_datapath.v   address pointer, counters
//   unpu_dma_fifo2.v       2-entry FIFO, instantiated twice inside the
//                           datapath (one per direction across the CU)
//
// PIPELINING. Both directions across the CU are decoupled by a FIFO so a
// producer and a consumer can run concurrently instead of strictly taking
// turns:
//   - Loading (weights, then inputs): THREE stages, all running every
//     cycle -- issuing the next SRAM read address, capturing a previously
//     requested word as it comes back, and pushing the oldest captured
//     word to the CU. The address channel is never idled waiting on read
//     data: reads N+1, N+2... are already in the SRAM while read N's data
//     is being captured and handed to the CU, so the steady-state cost is
//     one cycle per word whatever the SRAM's read latency is.
//   - Storing (results): two stages -- pulling the next result from the
//     CU overlaps with writing the current result to SRAM. One cycle per
//     word already, since both stages are same-cycle handshakes.
// The external interface below is unchanged by any of this -- it's purely
// an internal restructuring of unpu_dma_ctrl/unpu_dma_datapath. In
// particular NO new SRAM port was needed: the native interface's
// dma_ready already tells us when an address was accepted, and the SRAM's
// fixed 1-cycle read latency (see A2) tells us exactly which cycle the
// data comes back on, which is all a split-transaction read pipeline
// requires. How many reads may be in flight at once is set by
// LD_FIFO_DEPTH below.
//
// Sequence performed on a START pulse:
//   1. Read LOAD_WORDS = WEIGHT_WORDS + INPUT_WORDS = 8 packed SRAM words
//      (16 INT8 weights then 16 INT8 inputs, 4 elements per word) from
//      src_addr as ONE continuous stream, handing each whole 32-bit word
//      to the CU with cu_load_data asserted throughout, pipelined as
//      described above. This module makes no distinction between a
//      weight word and an input word -- it only knows the combined count.
//      The CU is responsible for unpacking each word's 4 elements,
//      distributing them into the array, AND for knowing (from its own
//      row/col config) where in the stream weights end and inputs begin;
//      this module only moves whole words, it never looks inside one.
//   2. Pulse cu_start_compute and wait for cu_done.
//   3. Pull OUTPUT_WORDS = 16 int32 result words (one per output element,
//      not packed) from the CU and write them to SRAM starting at dst_addr,
//      pipelined the same way (pull word N+1 while writing word N).
//
// ---------------------------------------------------------------------------
// ASSUMPTIONS (call these out to whoever owns the CU / SRAM wrapper):
//
//  A1. Word counts for a 4x4 INT8 array. WEIGHT_WORDS = INPUT_WORDS =
//      (Rows*Cols)/4 = 4 words each, since 4 INT8 elements pack into each
//      32-bit SRAM word, so LOAD_WORDS = 8 total; OUTPUT_WORDS = Rows*Cols
//      = 16, since each int32 result is one full word (no packing on the
//      way out). This module tracks only the combined LOAD_WORDS count --
//      it does not know or care where the weight/input boundary falls
//      within it (see A7). If your CU's actual input/output volumes
//      differ, adjust unpu_dma_datapath.v's load_total/output_total
//      assignments; if weights and inputs are ever NOT equal in size,
//      only the CU-side split logic needs to change, since this module
//      never depended on them being equal to begin with.
//
//  A2. SRAM read latency, and multiple reads in flight. The SRAM is
//      assumed to have a FIXED 1-cycle read latency: dma_ready
//      acknowledges address acceptance in the cycle it's asserted, and
//      dma_rdata is valid on the very next clock cycle, always, with no
//      per-transaction valid signal required. There is therefore no
//      dma_rvalid port on the native interface -- the "read data has
//      landed" pulse used internally (see unpu_dma_ctrl.v) is simply the
//      previous cycle's read-address-accepted pulse, registered once.
//
//      The load side still keeps up to LD_FIFO_DEPTH reads in flight at
//      once rather than one, which puts two requirements on the SRAM
//      side, both of them things a pipelined SRAM wrapper does anyway:
//        (a) Read data is returned IN ORDER, exactly 1 cycle per
//            accepted address. The DMA matches returning words to
//            addresses purely by order and by that fixed 1-cycle
//            delay; there is no read ID.
//        (b) dma_ready must actually mean "this address is accepted".
//            A wrapper that can only hold one read at a time MUST
//            de-assert dma_ready while it is busy, rather than leaving
//            it tied high and ignoring addresses -- doing that just
//            throttles the DMA back to one-at-a-time, which is correct
//            and costs nothing else.
//      SRAM *writes* and CU result handshakes are still assumed to
//      complete in the same cycle valid&ready are both high, no latency
//      to wait out -- that's why the store side needs no analogous extra
//      state or valid signal.
//
//  A3. Status bits (busy/done/error) are level/sticky, matching a
//      register-file NPU_STATUS that the CPU polls: done/error stay set
//      until the next START (or CTRL.RESET / hardware reset) clears them.
//
//  A4. soft_reset is a one-cycle pulse driven from CTRL.RESET (bit 1) in
//      the register file. It aborts any in-flight transfer back to IDLE.
//
//  A5. A watchdog (TIMEOUT_CYCLES) forces an ERROR state if the SRAM or
//      the CU ever stalls every process on the active side for too long.
//      Set TIMEOUT_CYCLES = 0 to disable it.
//
//  A6. cfg_rows*cfg_cols is assumed to divide evenly by 4 (the packing
//      ratio used only for word-count math). For exactly 4x4 = 16 elements
//      this holds exactly.
//
//  A7. This module has no notion of INT8 vs int32, endianness, or which
//      byte lane is which -- it only counts and moves whole 32-bit words.
//      All of that interpretation lives entirely in the CU, including
//      telling a weight element apart from an input element: cu_load_data
//      is asserted for the whole load stream with no weight/input split
//      signalled on the interface, so the CU must count its own
//      LOAD_WORDS (from its row/col config) to know when the boundary
//      passes.
//
//  A8. FIFO overrun. The load side no longer relies on any timing
//      argument for this. Because several reads can be in flight, every
//      word already requested must have somewhere to land, so the issue
//      stage spends a CREDIT per read: it may only send an address if the
//      load FIFO has a slot that is neither occupied nor already claimed
//      by a read in flight (ld_fifo_room in unpu_dma_datapath.v). Overrun
//      is therefore structurally impossible at any read latency, rather
//      than merely unlikely at latencies >= 1 cycle as it was before.
//
//      LD_FIFO_DEPTH sizes that credit pool and so caps the number of
//      reads in flight. 3 keeps a 1- or 2-cycle-latency SRAM's address
//      bus busy every cycle while still absorbing a CU stall; for a
//      deeper-latency SRAM raise it to about (latency + 1) to stay at one
//      cycle per word. It costs one 32-bit register per entry.
//
//      The store side is unchanged: pull still only checks the store
//      FIFO's registered full flag, so on the rare cycle that FIFO goes
//      from full to not-full it waits one extra cycle before resuming.
//      Harmless there, since both of its stages move at most one word per
//      cycle.
// =============================================================================
 
module unpu_dma #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 32,
    parameter LD_FIFO_DEPTH  = 3,          // load FIFO entries == max SRAM reads in flight
    parameter TIMEOUT_CYCLES = 32'd65535   // 0 = watchdog disabled
) (
    input  wire                      clk,
    input  wire                      rst_n,
 
    // ---- Decoded control-plane handshake (from the APB register file) ----
    input  wire                      start,       // 1-cycle pulse: CTRL.START written
    input  wire                      soft_reset,  // 1-cycle pulse: CTRL.RESET written
    input  wire [ADDR_WIDTH-1:0]     src_addr,    // DMA_SRC_ADDR
    input  wire [ADDR_WIDTH-1:0]     dst_addr,    // DMA_DST_ADDR
    input  wire [15:0]               cfg_rows,    // MATRIX_CFG[15:0]  (=4)
    input  wire [15:0]               cfg_cols,    // MATRIX_CFG[31:16] (=4)
 
    // ---- Status back to the register file (NPU_STATUS) ----
    output wire                      busy,        // STATUS bit 1
    output wire                      done,        // STATUS bit 0
    output wire                      error,       // STATUS bit 2
 
    // ---- Native master interface to SRAM (data plane) ----
    output wire [ADDR_WIDTH-1:0]     dma_addr,
    output wire [DATA_WIDTH-1:0]     dma_wdata,
    input  wire [DATA_WIDTH-1:0]     dma_rdata,
    output wire [3:0]                dma_wstrb,
    output wire                      dma_valid,
    input  wire                      dma_ready,   // SRAM is assumed to return dma_rdata exactly
                                                    // 1 clock cycle after a read address is
                                                    // accepted -- no dma_rvalid handshake needed
 
    // ---- Interface to the systolic-array Control Unit ----
    output wire [DATA_WIDTH-1:0]     cu_data,          // whole packed word (4 INT8 elems); CU unpacks
    output wire                      cu_data_valid,
    input  wire                      cu_data_ready,
    output wire                      cu_load_data,      // high through the whole load stream (weights then inputs; CU splits them itself)
    output wire                      cu_start_compute,  // 1-cycle pulse once the load stream is done
    input  wire                      cu_busy,
    input  wire                      cu_done,
    input  wire [DATA_WIDTH-1:0]     cu_result_data,    // int32 accumulator result, one word at a time
    input  wire                      cu_result_valid,
    output wire                      cu_result_ready
);
 
    wire cfg_valid = (cfg_rows == 16'd4) && (cfg_cols == 16'd4);
 
    // ctrl <-> datapath glue (internal, not exposed at this level)
    wire cfg_latch, addr_incr, addr_to_dst;
    wire issue_incr, fetch_clr, fetch_incr, push_clr, push_incr;
    wire ld_fifo_push, ld_fifo_pop, st_fifo_push, st_fifo_pop;
    wire phase_sel;
    wire issue_pending, ld_fifo_room, rd_inflight;
    wire fetch_pending, ld_fifo_empty, st_fifo_full, st_fifo_empty, word_last;
 
    unpu_dma_ctrl #(
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) u_ctrl (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (start),
        .soft_reset       (soft_reset),
        .cfg_valid        (cfg_valid),
        .busy             (busy),
        .done             (done),
        .error            (error),
        .dma_valid        (dma_valid),
        .dma_wstrb        (dma_wstrb),
        .dma_ready        (dma_ready),
        .cu_data_valid    (cu_data_valid),
        .cu_load_data     (cu_load_data),
        .cu_start_compute (cu_start_compute),
        .cu_data_ready    (cu_data_ready),
        .cu_busy          (cu_busy),
        .cu_done          (cu_done),
        .cu_result_ready  (cu_result_ready),
        .cu_result_valid  (cu_result_valid),
        .cfg_latch        (cfg_latch),
        .addr_incr        (addr_incr),
        .addr_to_dst      (addr_to_dst),
        .issue_incr       (issue_incr),
        .fetch_clr        (fetch_clr),
        .fetch_incr       (fetch_incr),
        .push_clr         (push_clr),
        .push_incr        (push_incr),
        .ld_fifo_push     (ld_fifo_push),
        .ld_fifo_pop      (ld_fifo_pop),
        .st_fifo_push     (st_fifo_push),
        .st_fifo_pop      (st_fifo_pop),
        .phase_sel        (phase_sel),
        .issue_pending    (issue_pending),
        .ld_fifo_room     (ld_fifo_room),
        .rd_inflight      (rd_inflight),
        .fetch_pending    (fetch_pending),
        .ld_fifo_empty    (ld_fifo_empty),
        .st_fifo_full     (st_fifo_full),
        .st_fifo_empty    (st_fifo_empty),
        .word_last        (word_last)
    );
 
    unpu_dma_datapath #(
        .ADDR_WIDTH     (ADDR_WIDTH),
        .DATA_WIDTH     (DATA_WIDTH),
        .ELEMS_PER_WORD (4),
        .LD_FIFO_DEPTH  (LD_FIFO_DEPTH)
    ) u_datapath (
        .clk             (clk),
        .rst_n           (rst_n),
        .src_addr        (src_addr),
        .dst_addr        (dst_addr),
        .cfg_rows        (cfg_rows),
        .cfg_cols        (cfg_cols),
        .cfg_latch       (cfg_latch),
        .addr_incr       (addr_incr),
        .addr_to_dst     (addr_to_dst),
        .issue_incr      (issue_incr),
        .fetch_clr       (fetch_clr),
        .fetch_incr      (fetch_incr),
        .push_clr        (push_clr),
        .push_incr       (push_incr),
        .ld_fifo_push    (ld_fifo_push),
        .ld_fifo_pop     (ld_fifo_pop),
        .st_fifo_push    (st_fifo_push),
        .st_fifo_pop     (st_fifo_pop),
        .phase_sel       (phase_sel),
        .dma_rdata       (dma_rdata),
        .cu_result_data  (cu_result_data),
        .dma_addr        (dma_addr),
        .dma_wdata       (dma_wdata),
        .cu_data         (cu_data),
        .issue_pending   (issue_pending),
        .ld_fifo_room    (ld_fifo_room),
        .rd_inflight     (rd_inflight),
        .fetch_pending   (fetch_pending),
        .ld_fifo_empty   (ld_fifo_empty),
        .st_fifo_full    (st_fifo_full),
        .st_fifo_empty   (st_fifo_empty),
        .word_last       (word_last)
    );
 
endmodule