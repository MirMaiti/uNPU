// =============================================================================
// unpu_dma_ctrl
//
// Control FSM for unpu_dma. Owns every handshake-facing signal and drives
// unpu_dma_datapath with single-purpose enable pulses; it never touches a
// data bus directly.
//
// Both directions are pipelined: producer and consumer processes run
// concurrently, decoupled by a FIFO, so words are in flight on each side
// of the CU at once.
//
//   LOAD (M_LOAD) is a THREE-stage pipeline, all three running every
//   cycle, decoupled by the load FIFO:
//       ISSUE   : send a read address to SRAM        (SRAM addr channel)
//       CAPTURE : read data lands (1 cc after issue) -> load FIFO
//                 (SRAM data channel; see dma_rvalid_r below)
//       PUSH    : load FIFO -> CU                    (cu_data handshake)
//   This DMA does not distinguish weight words from input words at all --
//   it just streams load_total words (weights immediately followed by
//   inputs, one contiguous block) to the CU with a single cu_load_data
//   asserted throughout. The CU is responsible for knowing, from its own
//   row/col config, where in that stream weights end and inputs begin
//   (see A1/A7 in unpu_dma.v).
//
//   READ PIPELINING. ISSUE used to be half of a 2-state sub-FSM that
//   parked in FE_DATA until the read data came back before issuing the
//   next address, which cost >=2 cycles per word even against the SRAM's
//   fixed 1-cycle latency, since the address channel sat idle for that
//   whole cycle. It is now purely level logic with no sub-state: it keeps
//   putting the next address out every cycle the SRAM will take one, so
//   read N+1 (and N+2) are already in the SRAM while read N's data is
//   being captured and pushed to the CU. Steady-state cost is one cycle
//   per word, and the old FE_DATA stall disappears entirely.
//
//   What stops ISSUE running away from a slow CU is ld_fifo_room from the
//   datapath: a read may only be issued if the load FIFO has a slot that
//   isn't already spoken for by a read in flight. Every word that has been
//   asked for is therefore guaranteed a landing slot, which is what makes
//   overflow structurally impossible instead of merely unlikely (it
//   replaces the old timing-based argument in A8 of unpu_dma.v). A
//   single-outstanding SRAM that de-asserts dma_ready while busy simply
//   throttles ISSUE back to one-at-a-time behaviour with no change here.
//
//   CAPTURE is one unconditional line -- since the SRAM's read latency is
//   a fixed 1 cycle (A2 in unpu_dma.v), the internally generated
//   dma_rvalid_r pulse (see below) lands exactly one cycle after each
//   accepted read address; whenever it pulses during M_LOAD, that word is
//   this phase's next word (reads return in order, A2) and goes straight
//   into the FIFO. PUSH is unchanged, pure level logic, since
//   cu_data_valid/ready is a same-cycle handshake.
//
//   STORE (M_STORE): producer = PULL (CU -> store FIFO), consumer = WRITE
//   (store FIFO -> SRAM). Two stages, not three, and neither needs a
//   sub-state: pulling a result from the CU (cu_result_valid/ready) and
//   writing a word to SRAM (dma_valid/dma_ready) are both plain same-cycle
//   handshakes with no latency to wait out, so PULL and WRITE are just two
//   level-gated checks against the store FIFO's full/empty flags. This is
//   why the store side was already running at one cycle per word and
//   needed no change.
//
// The pipelines share the datapath's issue_cnt/fetch_cnt/push_cnt counters
// and the word_last flag (see its header for how they're reused across
// producer/consumer roles) -- only phase_sel and which FIFO is being
// driven changes between load and store.
// =============================================================================

module unpu_dma_ctrl #(
    parameter TIMEOUT_CYCLES = 32'd65535   // 0 = watchdog disabled
) (
    input  wire        clk,
    input  wire        rst_n,

    // from the (external) APB register file
    input  wire        start,       // 1-cycle pulse: CTRL.START written
    input  wire        soft_reset,  // 1-cycle pulse: CTRL.RESET written
    input  wire        cfg_valid,   // cfg_rows!=0 && cfg_cols!=0

    // status back to the register file (NPU_STATUS)
    output reg          busy,
    output reg          done,
    output reg          error,

    // native SRAM master handshake
    output reg           dma_valid,
    output reg  [3:0]    dma_wstrb,
    input  wire          dma_ready,    // SRAM returns dma_rdata exactly 1 cycle after an
                                        // accepted read address -- no dma_rvalid port needed;
                                        // the equivalent "data has landed" pulse is generated
                                        // internally below (dma_rvalid_r)

    // systolic-array CU handshake
    output reg           cu_data_valid,
    output reg           cu_load_data,     // high through the whole load stream (weights then inputs)
    output reg           cu_start_compute,
    input  wire          cu_data_ready,
    input  wire          cu_busy,          // not used for sequencing; kept for interface completeness
    input  wire          cu_done,
    output reg           cu_result_ready,
    input  wire          cu_result_valid,

    // datapath control pulses / levels
    output reg            cfg_latch,
    output reg            addr_incr,
    output reg            addr_to_dst,
    output reg            issue_incr,
    output reg            fetch_clr,
    output reg            fetch_incr,
    output reg            push_clr,
    output reg            push_incr,
    output reg            ld_fifo_push,
    output reg            ld_fifo_pop,
    output reg            st_fifo_push,
    output reg            st_fifo_pop,
    output reg             phase_sel,

    // datapath status
    input  wire            issue_pending,   // issue_cnt < phase_total
    input  wire            ld_fifo_room,    // free FIFO slot not already claimed by an in-flight read
    input  wire            rd_inflight,     // at least one SRAM read is outstanding
    input  wire            fetch_pending,   // fetch_cnt < phase_total
    input  wire            ld_fifo_empty,
    input  wire            st_fifo_full,
    input  wire            st_fifo_empty,
    input  wire            word_last        // push_cnt == phase_total-1
);

    localparam PH_LOAD = 1'b0, PH_OUTPUT = 1'b1;

    // ---- main (macro-phase) state ----
    localparam [3:0]
        M_IDLE     = 4'd0,
        M_LOAD     = 4'd1,   // issue+capture+push pipeline running, phase_sel=LOAD (weights then inputs)
        M_START_CU = 4'd2,
        M_WAIT_CU  = 4'd3,
        M_STORE    = 4'd4,   // pull+write pipeline running, phase_sel=OUTPUT
        M_DONE     = 4'd5,
        M_ERROR    = 4'd6;

    reg [3:0]  state, next_state;
    reg [31:0] timeout_cnt;

    wire loading     = (state == M_LOAD);
    wire storing     = (state == M_STORE);
    wire timeout_hit = (TIMEOUT_CYCLES != 0) && (timeout_cnt >= TIMEOUT_CYCLES);

    

    // ------------------------------------------------------------------
    // dma_rvalid_r: internally reconstructed "read data is valid this
    // cycle" pulse. The SRAM is assumed to have a fixed 1-cycle read
    // latency (see A2 in unpu_dma.v), so this is simply last cycle's
    // read-address-accept pulse (ld_issue_ack) registered once -- there is
    // no dma_rvalid port on the native interface to sample instead.
    // rd_inflight above still qualifies its use the same way it always
    // did, so a pulse landing just after a soft_reset aborts a transfer is
    // still dropped rather than mis-captured.
    // ------------------------------------------------------------------
    reg dma_rvalid_r;

// Load-side stage activity, broken out so both the M_LOAD body and the
    // watchdog can see it without duplicating the conditions.
    wire ld_issue_now   = loading && issue_pending && ld_fifo_room;   // address offered to SRAM
    wire ld_issue_ack   = ld_issue_now && dma_ready;                  // ...and accepted
    wire ld_capture_now = loading && dma_rvalid_r && rd_inflight;     // read data landed
    wire ld_push_now    = loading && !ld_fifo_empty && cu_data_ready;  // word handed to the CU

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dma_rvalid_r <= 1'b0;
        else if (soft_reset)
            dma_rvalid_r <= 1'b0;
        else
            dma_rvalid_r <= ld_issue_ack;
    end

    // "Did something actually move this cycle?" -- used by the watchdog.
    // The main state can legitimately sit still for many cycles while a
    // pipeline is healthily streaming words through, so this has to look
    // at the sub-processes directly rather than at `state` itself. On the
    // load side that is now any of the three stages making progress.
    wire progress = loading ? (ld_issue_ack || ld_capture_now || ld_push_now) :
                    storing ? (st_fifo_push || st_fifo_pop) :
                              (next_state != state);

    // ------------------------------------------------------------------
    // Combinational next-state + outputs
    // ------------------------------------------------------------------
    always @(*) begin
        next_state       = state;

        dma_valid        = 1'b0;
        dma_wstrb        = 4'b0000;
        cu_data_valid    = 1'b0;
        cu_load_data     = 1'b0;
        cu_start_compute = 1'b0;
        cu_result_ready  = 1'b0;

        cfg_latch    = 1'b0;
        addr_incr    = 1'b0;
        addr_to_dst  = 1'b0;
        issue_incr   = 1'b0;
        fetch_clr    = 1'b0;
        fetch_incr   = 1'b0;
        push_clr     = 1'b0;
        push_incr    = 1'b0;
        ld_fifo_push = 1'b0;
        ld_fifo_pop  = 1'b0;
        st_fifo_push = 1'b0;
        st_fifo_pop  = 1'b0;
        phase_sel    = PH_LOAD;

        case (state)
            M_IDLE: begin
                if (start) begin
                    if (cfg_valid) begin
                        cfg_latch        = 1'b1;
                        next_state       = M_LOAD;
                    end else begin
                        next_state = M_ERROR;
                    end
                end
            end

            M_LOAD: begin
                phase_sel    = PH_LOAD;
                cu_load_data = 1'b1;

                // ---- stage 1, ISSUE: next read address -> SRAM ----
                // No waiting for the previous read's data: as long as this
                // phase still owes addresses and the load FIFO has an
                // unclaimed slot to land the word in, the address goes out
                // now. addr_ptr advances on acceptance, so the SRAM sees a
                // new address every cycle it can take one.
                if (ld_issue_now) begin
                    dma_valid = 1'b1;
                    dma_wstrb = 4'b0000;             // read
                    if (dma_ready) begin
                        addr_incr  = 1'b1;
                        issue_incr = 1'b1;
                    end
                end

                // ---- stage 2, CAPTURE: SRAM read data -> load FIFO ----
                // dma_rvalid_r pulses exactly 1 cycle after an accepted
                // read address (fixed SRAM latency, A2), so whenever it
                // fires the word now on dma_rdata is this phase's next
                // word in order, and ld_fifo_room reserved a slot for it
                // back when its address was issued -- it can always be
                // enqueued right here with no further checking.
                //
                // rd_inflight ("we actually asked for a word") is the one
                // qualifier: it drops a dma_rvalid_r pulse this phase never
                // requested, which is what a soft_reset issued mid-transfer
                // can leave behind for the one read still working its way
                // through the fixed 1-cycle pipe when the abort happens.
                if (ld_capture_now) begin
                    ld_fifo_push = 1'b1;
                    fetch_incr   = 1'b1;
                end

                // ---- stage 3, PUSH: load FIFO -> CU ----
                if (!ld_fifo_empty) begin
                    cu_data_valid = 1'b1;
                    if (cu_data_ready) begin
                        ld_fifo_pop = 1'b1;
                        if (word_last) begin
                            // Last word of the whole load stream just handed
                            // to the CU. Everything upstream is guaranteed
                            // drained here -- this word only reached the
                            // FIFO head after every earlier one left it, and
                            // it was the last one issued. One state covers
                            // weights and inputs, so this always goes
                            // straight to M_START_CU -- no mid-load branch.
                            push_clr   = 1'b1;
                            fetch_clr  = 1'b1;       // also clears issue_cnt
                            next_state = M_START_CU;
                        end else begin
                            push_incr = 1'b1;
                        end
                    end
                end
            end

            M_START_CU: begin
                cu_start_compute = 1'b1;
                next_state       = M_WAIT_CU;
            end

            M_WAIT_CU: begin
                if (cu_done) begin
                    addr_to_dst = 1'b1;             // repoint addr_ptr at dst_addr
                    push_clr    = 1'b1;             // reused as the store write count
                    fetch_clr   = 1'b1;             // reused as the store pull count
                    next_state  = M_STORE;
                end
            end

            M_STORE: begin
                phase_sel = PH_OUTPUT;

                // ---- pull (producer): CU -> store FIFO ----
                if (fetch_pending && !st_fifo_full) begin
                    cu_result_ready = 1'b1;
                    if (cu_result_valid) begin
                        st_fifo_push = 1'b1;
                        fetch_incr   = 1'b1;
                    end
                end

                // ---- write (consumer): store FIFO -> SRAM ----
                if (!st_fifo_empty) begin
                    dma_valid = 1'b1;
                    dma_wstrb = 4'b1111;            // full-word write
                    if (dma_ready) begin
                        st_fifo_pop = 1'b1;
                        addr_incr   = 1'b1;
                        if (word_last) begin
                            next_state = M_DONE;
                        end else begin
                            push_incr = 1'b1;
                        end
                    end
                end
            end

            M_DONE: begin
                next_state = M_IDLE;               // busy/done stay sticky, see A3 in unpu_dma.v
            end

            M_ERROR: begin
                next_state = M_ERROR;              // sticky until soft_reset / rst_n
            end

            default: next_state = M_IDLE;
        endcase

        if (timeout_hit)
            next_state = M_ERROR;
    end

    // ------------------------------------------------------------------
    // Sequential: state registers + status + watchdog
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= M_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            timeout_cnt <= 32'd0;
        end
        else if (soft_reset) begin
            state       <= M_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            timeout_cnt <= 32'd0;
        end
        else begin
            state       <= next_state;

            if (state != M_IDLE && state != M_ERROR)
                timeout_cnt <= progress ? 32'd0 : (timeout_cnt + 1'b1);
            else
                timeout_cnt <= 32'd0;

            if (timeout_hit) begin
                error <= 1'b1;
                busy  <= 1'b0;
            end
            else begin
                case (state)
                    M_IDLE: begin
                        if (start) begin
                            if (cfg_valid) begin
                                busy  <= 1'b1;
                                done  <= 1'b0;
                                error <= 1'b0;
                            end else begin
                                done  <=1'b0;
                                error <= 1'b1;      // rows!=4 && cols!=4
                            end
                        end
                    end

                    M_DONE: begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end

                    default: ; // no status change in the other states
                endcase
            end
        end
    end

endmodule