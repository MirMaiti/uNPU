// =============================================================================
// unpu_dma_fifo2
//
// Minimal synchronous FIFO (skid buffer) used to decouple the SRAM fetch
// stage from the CU push stage, so the two can run concurrently: while the
// push side is draining the head entry into the CU, the fetch side can be
// filling another entry from SRAM.
//
// DEPTH is now a parameter (still 2 by default, hence the module name --
// kept unchanged so existing filelists/instances don't have to move). It
// need NOT be a power of two: the pointers wrap at DEPTH explicitly and
// full/empty come from an occupancy counter rather than pointer tricks.
// unpu_dma_datapath instantiates it twice:
//   - load side : DEPTH = LD_FIFO_DEPTH (3 by default). The extra entry is
//     what lets the load side keep several SRAM reads in flight at once --
//     see the "read pipelining" note in unpu_dma_ctrl.v.
//   - store side: DEPTH = 2. The store side is two same-cycle handshakes
//     back to back, so one skid entry is all it ever needed.
//
// `count` (occupancy) is exported because the load side needs to reason
// about FIFO space *plus* reads that are already in flight but haven't
// landed yet; full/empty alone can't express that.
//
// Plain push/pop pulses rather than a full ready/valid handshake keeps this
// tiny; the caller (unpu_dma_ctrl) is expected to have already checked
// !full before pulsing push and !empty before pulsing pop.
// =============================================================================

module unpu_dma_fifo2 #(
    parameter WIDTH = 32,
    parameter DEPTH = 2          // >= 2, <= 15; need not be a power of two
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             clear,   // synchronous flush (phase switch)

    input  wire [WIDTH-1:0] din,
    input  wire             push,    // caller must have already checked !full

    input  wire             pop,     // caller must have already checked !empty

    output wire [WIDTH-1:0] dout,    // valid whenever !empty
    output wire             full,
    output wire             empty,
    output wire [3:0]       count    // current occupancy, 0..DEPTH
);

    localparam PTR_W = (DEPTH <= 2) ? 1 : 4;

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [PTR_W-1:0] wr_ptr, rd_ptr;
    reg [3:0]       cnt;
    integer         e;

    wire [PTR_W-1:0] wr_nxt = (wr_ptr == DEPTH-1) ? {PTR_W{1'b0}} : (wr_ptr + 1'b1);
    wire [PTR_W-1:0] rd_nxt = (rd_ptr == DEPTH-1) ? {PTR_W{1'b0}} : (rd_ptr + 1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {PTR_W{1'b0}};
            rd_ptr <= {PTR_W{1'b0}};
            cnt    <= 4'd0;
            // Zero the storage too, so dout (and therefore dma_wdata /
            // cu_data, which are just the two FIFO heads) reads 0 out of
            // reset instead of X. Nothing ever samples dout while empty --
            // the consumer gates on !empty -- but an X on a bus makes
            // waveforms unreadable and propagates into any downstream
            // model that isn't equally careful. A few flops of reset on a
            // 2-3 entry FIFO is cheap insurance.
            for (e = 0; e < DEPTH; e = e + 1)
                mem[e] <= {WIDTH{1'b0}};
        end
        else if (clear) begin
            wr_ptr <= {PTR_W{1'b0}};
            rd_ptr <= {PTR_W{1'b0}};
            cnt    <= 4'd0;
        end
        else begin
            case ({push, pop})
                2'b10: begin                 // push only
                    mem[wr_ptr] <= din;
                    wr_ptr      <= wr_nxt;
                    cnt         <= cnt + 4'd1;
                end
                2'b01: begin                 // pop only
                    rd_ptr <= rd_nxt;
                    cnt    <= cnt - 4'd1;
                end
                2'b11: begin                 // simultaneous push + pop
                    mem[wr_ptr] <= din;
                    wr_ptr      <= wr_nxt;
                    rd_ptr      <= rd_nxt;
                    // count unchanged: one word in, one word out
                end
                default: ; // no-op
            endcase
        end
    end

    assign dout  = mem[rd_ptr];
    assign count = cnt;
    assign full  = (cnt == DEPTH[3:0]);
    assign empty = (cnt == 4'd0);

endmodule