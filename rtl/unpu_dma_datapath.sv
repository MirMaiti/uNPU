// =============================================================================
// unpu_dma_datapath
//
// All registers/counters/muxes for unpu_dma. Owns no handshake protocol
// logic (that's unpu_dma_ctrl); it just reacts to single-cycle control
// pulses and exposes status flags + data buses.
//
// Two independent FIFOs decouple producer from consumer on each side of
// the CU:
//   - u_fifo_ld : SRAM read data -> CU (one continuous load stream: all
//                 weight words immediately followed by all input words),
//                 LD_FIFO_DEPTH entries deep
//   - u_fifo_st : CU result data -> SRAM write, 2 entries deep
//
// COUNTERS. The load side is a THREE-stage pipeline (issue address ->
// capture read data -> push to CU), so it needs one counter per stage
// boundary; the store side is two stages and reuses the same counters:
//   - issue_cnt : read addresses ACCEPTED by the SRAM this phase. Load
//                 phase only -- the store side never issues reads, so
//                 issue_cnt just sits at 0 there and issue_pending is
//                 ignored by the controller.
//   - fetch_cnt : words the *producer* stage has completed this phase
//                 (SRAM read words that have actually come back and been
//                 enqueued, during load; CU results pulled, during store)
//   - push_cnt  : words the *consumer* stage has completed this phase
//                 (words handed to the CU, during load; words written to
//                 SRAM, during store)
// All three are cleared by the same fetch_clr/cfg_latch phase-boundary
// pulses, so pipelining the load side added no new clear controls.
//
// READ PIPELINING / rd_outstanding. Because several SRAM reads can now be
// in flight at once, "is there room in the load FIFO?" is no longer the
// right question to gate a new read address with: the words already in
// flight have nowhere else to land, so each of them must keep a FIFO slot
// reserved. rd_outstanding counts addresses accepted but not yet returned
// (up on issue_incr, down on ld_fifo_push i.e. the ctrl-side dma_rvalid_r
// pulse -- generated internally in unpu_dma_ctrl.v from the SRAM's fixed
// 1-cycle read latency, see A2 in unpu_dma.v), and
//     ld_fifo_room = (fifo occupancy + rd_outstanding) < LD_FIFO_DEPTH
// is the credit the controller checks instead. This makes overflow
// structurally impossible, which is what replaces the old "latency >= 1
// cycle protects us" argument in A8 of unpu_dma.v.
//
// LD_FIFO_DEPTH sizes that credit pool, so it is also the maximum number
// of reads in flight. 3 is enough to keep the address bus busy every
// cycle against a 1-cycle-latency SRAM while still absorbing a CU stall;
// raise it for a deeper-latency SRAM, it costs one word of storage per
// entry and nothing else. The store FIFO stays at 2 -- both its stages
// are same-cycle handshakes, so it has no latency to cover.
//
// There's no "captured word" register (rd_word / result_word) -- each
// FIFO itself holds the in-flight word(s), so dma_rdata feeds u_fifo_ld
// directly and cu_result_data feeds u_fifo_st directly.
//
// ASSUMPTIONS (see unpu_dma.v header for the full list):
//   - ELEMS_PER_WORD (=4) is only used here to size load_total/
//     output_total from cfg_rows*cfg_cols -- it never affects a data bus.
//   - cfg_rows*cfg_cols is assumed to divide evenly by ELEMS_PER_WORD.
//   - Output words are NOT packed (one int32 accumulator result per word).
//   - SRAM reads return IN ORDER, exactly 1 cycle after each accepted
//     address (fixed latency, see A2 in unpu_dma.v) -- rd_outstanding is
//     a plain counter, it has no way to match data to a specific
//     outstanding address.
// =============================================================================

module unpu_dma_datapath #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 32,   // SRAM word / CU word / accumulator-result width
    parameter ELEMS_PER_WORD = 4,    // INT8 elements packed per 32-bit SRAM word (count math only)
    parameter LD_FIFO_DEPTH  = 3     // load FIFO entries == max SRAM reads in flight
) (
    input  wire                      clk,
    input  wire                      rst_n,

    // static config, latched on cfg_latch
    input  wire [ADDR_WIDTH-1:0]     src_addr,
    input  wire [ADDR_WIDTH-1:0]     dst_addr,
    input  wire [15:0]               cfg_rows,
    input  wire [15:0]               cfg_cols,

    // control pulses/levels from unpu_dma_ctrl
    input  wire                      cfg_latch,     // latch cfg, addr_ptr<=src_addr, clear everything
    input  wire                      addr_incr,     // addr_ptr += 4 (one SRAM word)
    input  wire                      addr_to_dst,   // addr_ptr <= dst_addr (enter store phase)
    input  wire                      issue_incr,    // issue_cnt += 1 (SRAM accepted a read address)
    input  wire                      fetch_clr,     // issue_cnt/fetch_cnt <= 0 (phase switch)
    input  wire                      fetch_incr,    // fetch_cnt += 1 (producer completed one word)
    input  wire                      push_clr,      // push_cnt <= 0 (phase switch)
    input  wire                      push_incr,      // push_cnt += 1 (consumer completed one word)
    input  wire                      ld_fifo_push,   // enqueue dma_rdata into the load FIFO
    input  wire                      ld_fifo_pop,    // dequeue the load FIFO for the CU
    input  wire                      st_fifo_push,   // enqueue cu_result_data into the store FIFO
    input  wire                      st_fifo_pop,    // dequeue the store FIFO for SRAM
    input  wire                      phase_sel,      // 0=load (weight+input combined) 1=output

    // data in
    input  wire [DATA_WIDTH-1:0]     dma_rdata,
    input  wire [DATA_WIDTH-1:0]     cu_result_data,

    // data out
    output wire [ADDR_WIDTH-1:0]     dma_addr,
    output wire [DATA_WIDTH-1:0]     dma_wdata,      // head of the store FIFO
    output wire [DATA_WIDTH-1:0]     cu_data,        // head of the load FIFO

    // status back to controller
    output wire                      issue_pending,  // issue_cnt < phase_total (more addresses to send)
    output wire                      ld_fifo_room,   // a FIFO slot is free AND unreserved by an in-flight read
    output wire                      rd_inflight,    // at least one SRAM read is outstanding
    output wire                      fetch_pending,  // fetch_cnt < phase_total (more for producer to do)
    output wire                      ld_fifo_empty,
    output wire                      st_fifo_full,
    output wire                      st_fifo_empty,
    output wire                      word_last       // push_cnt == phase_total-1
);

    localparam PH_LOAD = 1'b0, PH_OUTPUT = 1'b1;

    reg [ADDR_WIDTH-1:0]  addr_ptr;
    reg [ADDR_WIDTH-1:0]  lat_dst_addr;
    reg [31:0]            issue_cnt;
    reg [31:0]            fetch_cnt;
    reg [31:0]            push_cnt;
    reg [31:0]            load_total, output_total;
    reg [3:0]             rd_outstanding;   // read addresses accepted, data not yet returned

    wire [31:0] elems_total  = cfg_rows * cfg_cols;
    wire        ld_fifo_clear = cfg_latch | fetch_clr;   // flush at every phase boundary

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_ptr     <= {ADDR_WIDTH{1'b0}};
            lat_dst_addr <= {ADDR_WIDTH{1'b0}};
            issue_cnt    <= 32'd0;
            fetch_cnt    <= 32'd0;
            push_cnt     <= 32'd0;
            load_total   <= 32'd0;
            output_total <= 32'd0;
        end
        else begin
            if (cfg_latch) begin
                addr_ptr     <= src_addr;
                lat_dst_addr <= dst_addr;
                issue_cnt    <= 32'd0;
                fetch_cnt    <= 32'd0;
                push_cnt     <= 32'd0;
                // weight words + input words, one continuous stream (see A1
                // in unpu_dma.v: equal-size assumption folded in here);
                // 16/4 + 16/4 = 8 words for a 4x4 INT8 array
                load_total   <= (elems_total / ELEMS_PER_WORD) * 32'd2;
                output_total <= elems_total;                   // int32 results, one word each
            end

            if (addr_to_dst)
                addr_ptr <= lat_dst_addr;
            else if (addr_incr)
                addr_ptr <= addr_ptr + 32'd4;

            if (fetch_clr)
                issue_cnt <= 32'd0;
            else if (issue_incr)
                issue_cnt <= issue_cnt + 1'b1;

            if (fetch_clr)
                fetch_cnt <= 32'd0;
            else if (fetch_incr)
                fetch_cnt <= fetch_cnt + 1'b1;

            if (push_clr)
                push_cnt <= 32'd0;
            else if (push_incr)
                push_cnt <= push_cnt + 1'b1;
        end
    end

    // ---- in-flight SRAM reads -------------------------------------------
    // Cleared by the same phase-boundary pulse that flushes the load FIFO,
    // where it is guaranteed to be 0 already (the phase only ends once
    // every word it asked for has been pushed to the CU).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_outstanding <= 4'd0;
        else if (ld_fifo_clear)
            rd_outstanding <= 4'd0;
        else begin
            case ({issue_incr, ld_fifo_push})
                2'b10:   rd_outstanding <= rd_outstanding + 4'd1;
                2'b01:   rd_outstanding <= rd_outstanding - 4'd1;
                default: ;   // both or neither: unchanged
            endcase
        end
    end

    reg [31:0] phase_total;
    always @(*) begin
        case (phase_sel)
            PH_LOAD: phase_total = load_total;
            default: phase_total = output_total;
        endcase
    end

    assign issue_pending = (issue_cnt < phase_total);
    assign fetch_pending = (fetch_cnt < phase_total);
    assign word_last     = (push_cnt == phase_total - 1'b1);

    assign dma_addr    = addr_ptr;
    assign rd_inflight = (rd_outstanding != 4'd0);

    // ------------------------------------------------------------------
    // Load FIFO: SRAM read data -> CU
    // ------------------------------------------------------------------
    wire [3:0] ld_fifo_count;
    wire [4:0] ld_fifo_used;

    // Credit for one more read address: a free slot that no in-flight read
    // has already claimed. Both terms are registered, so ld_fifo_room can
    // only ever go UP while a read address is waiting for dma_ready (a pop
    // frees a slot; a returning word swaps an in-flight read for an
    // occupied slot) -- dma_valid therefore never drops before it's
    // acknowledged.
    assign ld_fifo_used = {1'b0, ld_fifo_count} + {1'b0, rd_outstanding};  // 5b: no wrap
    assign ld_fifo_room = (ld_fifo_used < LD_FIFO_DEPTH[4:0]);

    unpu_dma_fifo2 #(
        .WIDTH(DATA_WIDTH),
        .DEPTH(LD_FIFO_DEPTH)
    ) u_fifo_ld (
        .clk   (clk),
        .rst_n (rst_n),
        .clear (ld_fifo_clear),
        .din   (dma_rdata),
        .push  (ld_fifo_push),
        .pop   (ld_fifo_pop),
        .dout  (cu_data),
        .full  (),                  // unused: ld_fifo_room is the real gate
        .empty (ld_fifo_empty),
        .count (ld_fifo_count)
    );

    // ------------------------------------------------------------------
    // Store FIFO: CU result data -> SRAM write
    // ------------------------------------------------------------------
    wire st_fifo_clear = fetch_clr;   // flushed by the same phase-boundary pulse

    unpu_dma_fifo2 #(
        .WIDTH(DATA_WIDTH),
        .DEPTH(2)
    ) u_fifo_st (
        .clk   (clk),
        .rst_n (rst_n),
        .clear (st_fifo_clear),
        .din   (cu_result_data),
        .push  (st_fifo_push),
        .pop   (st_fifo_pop),
        .dout  (dma_wdata),
        .full  (st_fifo_full),
        .empty (st_fifo_empty),
        .count ()
    );

endmodule