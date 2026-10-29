`timescale 1ns/1ps
// =============================================================================
// Module      : systolic_array_dip_4x4
// Description : 4x4 weight-stationary systolic array using the Diagonal-Input
//               Permutated (DiP) dataflow. Computes one output row
//               C[t][:] = sum_k A[t][k] * W[k][:] per activation row streamed in.
//
// Dataflow summary
//   - Weights : broadcast one raw row per wshift cycle. PE(r,c) latches
//               W[(r+c)%N][c], so the permutation is fixed in wiring rather
//               than performed by the DMA or by a weight shift chain.
//   - Inputs  : enter row 0 unskewed. Each lower row takes its activation
//               diagonally from PE(r-1,(c+1)%N), so PE(r,c) sees A[t][(c+r)%N],
//               which pairs with W[(c+r)%N][c]. This replaces the input-skew
//               FIFOs a conventional weight-stationary array would need.
//   - Psums   : flow straight down each column; row N-1 emits the result.
//
// Interface contract
//   - Byte order : word[31:24] = column 0 ... word[7:0] = column 3, for both
//                  weight_word and act_word. Must match the DMA/SRAM packing.
//   - wshift     : asserted once per weight row, N assertions per job. Gaps
//                  between assertions are tolerated (load_cnt only advances on
//                  wshift), so DMA/SRAM back-pressure is safe.
//   - act_en     : asserted once per activation row, with act_word valid in the
//                  same cycle. Gaps are tolerated: every pipeline register holds
//                  until its own enable fires.
//   - wshift and act_en must not overlap; weights must be fully loaded before
//     the first activation (the PE has no weight double-buffering).
//   - load_clr   : must be asserted at least once between an aborted job and
//                  the next job (driven by the CU as soft_reset | round_start).
//
// Assumptions / limitations
//   - Hardcoded for N=4: 2-bit load_cnt, 2-bit LOAD_ROW, and a 32-bit word
//     carrying exactly four 8-bit lanes. Changing N needs all three re-derived.
//   - Arithmetic is UNSIGNED. PSUM_W=18 holds the worst case
//     4 * 255 * 255 = 260,100 < 2^18. Signed INT8 would need mac_unit changes.
//   - psum_seed is added into row 0; tie to zero for a pure matrix multiply.
//   - Only load_cnt clears on load_clr. PE operand/psum registers are not
//     cleared on soft reset because every job fully overwrites them before use.
//
// @param N       Array dimension (must be 4; see limitations).
// @param ACT_W   Activation width in bits.
// @param WGT_W   Weight width in bits.
// @param PROD_W  Multiplier product width (ACT_W + WGT_W).
// @param PSUM_W  Partial-sum width; must cover N * (2^ACT_W-1) * (2^WGT_W-1).
// =============================================================================

module systolic_array_dip_4x4 #(
  parameter int N      = 4,
  parameter int ACT_W  = 8,
  parameter int WGT_W  = 8,
  parameter int PROD_W = 16,
  parameter int PSUM_W = 18
) (
  input  logic clk,
  input  logic rst_n,                        // async, active-low hardware reset

  input  logic              wshift,          // weight-row strobe (one raw row of W per assertion)
  input  logic              act_en,          // activation-row strobe; the ONLY compute enable driven
                                             // externally; all per-row/per-stage enables derive from it

  input  logic [31:0]       weight_word,     // one raw weight row, 4 x 8b, column 0 in MSB
  input  logic [31:0]       act_word,        // one activation row, 4 x 8b, column 0 in MSB
  input  logic [PSUM_W-1:0] psum_seed   [N-1:0], // per-column psum injected at row 0 (bias hook; 0 for matmul)
  input  logic load_clr,                     // sync clear of load_cnt; takes priority over wshift so an
                                             // abort that coincides with a weight push still lands at 0

  output logic [PSUM_W-1:0] psum_out    [N-1:0], // completed output row, one element per column
  output logic              out_valid,       // one pulse per output row, aligned with psum_out

  output logic [1:0] load_cnt_dbg            // verification visibility only; leave unconnected at top level
);

  // ---------------------------------------------------------------------------
  // Word unpack (pure wiring).
  // MSB-first lane order must match the DMA's pack_row convention; if the
  // packing is ever flipped, change both unpacks to word[c*8 +: 8] together.
  // ---------------------------------------------------------------------------
  logic [WGT_W-1:0] weight_byte [N-1:0];
  genvar wc;
  generate
    for (wc = 0; wc < N; wc++) begin : g_unpack_w
      assign weight_byte[wc] = weight_word[(N-1-wc)*8 +: 8];
    end
  endgenerate

  // Row 0 reads act_data combinationally, so act_word only has to be valid in
  // the act_en cycle; no input register is needed on this port.
  logic [ACT_W-1:0] act_data [N-1:0];
  genvar ac;
  generate
    for (ac = 0; ac < N; ac++) begin : g_unpack_a
      assign act_data[ac] = act_word[(N-1-ac)*8 +: 8];
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // load_cnt: index of the raw weight row currently on weight_word.
  // It is the only progress-tracking state in the array, so it is the only
  // register that needs a soft clear. Without load_clr, an abort after 1-3
  // weight rows would leave it non-zero, and every later job would latch
  // weights into the wrong PEs (the error persists, because each job adds
  // exactly N and the offset never returns to 0) until a hardware reset.
  // ---------------------------------------------------------------------------
  logic [1:0] load_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      load_cnt <= 2'd0;
    else if(load_clr)
      load_cnt <= 2'd0;
    else if (wshift)
      load_cnt <= (load_cnt == N-1) ? 2'd0 : load_cnt + 2'd1;
  end

  assign load_cnt_dbg = load_cnt;

  // ---------------------------------------------------------------------------
  // Row-enable stagger.
  // The diagonal activation link means row r's input is only valid one cycle
  // after row r-1 latched it, so each row's latch enable is the row above's
  // delayed by one cycle. Generating this internally keeps the controller
  // ignorant of the DiP skew: it only drives act_en.
  //
  // Row 0 stays combinational (act_en itself) so it captures act_word in the
  // same cycle it is presented; registering it would misalign row 0 by a cycle.
  // Split into pe_en0 + pe_en_sr to avoid a mixed assign/always_ff driver on
  // one vector.
  //
  // Free-running (not gated by load_clr): after an abort it drains in N cycles
  // on its own, and any stray out_valid pulses are discarded because the CU
  // resets its output-row counter at round_start.
  // ---------------------------------------------------------------------------
  logic              pe_en0;
  logic [N-1:1]      pe_en_sr;
  logic [N-1:0]      pe_en_int;

  assign pe_en0 = act_en;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pe_en_sr <= '0;
    else
      pe_en_sr <= {pe_en_sr[N-2:1], pe_en0};
  end

  assign pe_en_int = {pe_en_sr, pe_en0};

  // ---------------------------------------------------------------------------
  // MAC stage enables.
  // mac_unit is a fixed 2-stage pipeline (latch -> multiply -> accumulate), so
  // each stage's enable is the previous stage's delayed by one cycle. Deriving
  // them here, rather than as ports, makes misaligned enables impossible by
  // construction.
  // ---------------------------------------------------------------------------
  logic [N-1:0] mul_en_int;
  logic [N-1:0] adder_en_int;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mul_en_int   <= '0;
      adder_en_int <= '0;
    end else begin
      mul_en_int   <= pe_en_int;
      adder_en_int <= mul_en_int;
    end
  end

  // ---------------------------------------------------------------------------
  // out_valid.
  // psum_out is the last row's registered psum, which updates on the edge where
  // adder_en_int[N-1] is high. Registering that same enable makes out_valid
  // high in exactly the cycle the new row is visible, so the consumer samples
  // on out_valid instead of counting latency.
  // Latency from act_en to out_valid: N + 2 cycles (N-1 row stagger, 2 MAC
  // stages, 1 output register).
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      out_valid <= 1'b0;
    else
      out_valid <= adder_en_int[N-1];
  end

  logic [ACT_W-1:0]  act_link  [N-1:0][N-1:0];
  logic [PSUM_W-1:0] psum_link [N-1:0][N-1:0];

  // ---------------------------------------------------------------------------
  // PE grid.
  // ---------------------------------------------------------------------------
  genvar r, c;
  generate
    for (r = 0; r < N; r++) begin : g_row
      for (c = 0; c < N; c++) begin : g_pe

        // Elaboration-time constant: 16 fixed 2-bit comparators replace a
        // runtime weight router. 2-bit width is valid only for N <= 4.
        localparam logic [1:0] LOAD_ROW = (r + c) % N;

        logic pe_wshift;
        assign pe_wshift = wshift && (load_cnt == LOAD_ROW);

        logic [ACT_W-1:0]  act_src;
        logic [PSUM_W-1:0] psum_src;

        if (r == 0) begin : g_row0_src
          assign act_src  = act_data[c];
          assign psum_src = psum_seed[c];
        end else begin : g_inner_src
          // (c+1)%N wrap: column N-1 takes from column 0 of the row above.
          // This rotation is what aligns A[t][(c+r)%N] with W[(c+r)%N][c].
          assign act_src  = act_link[r-1][(c + 1) % N];
          assign psum_src = psum_link[r-1][c];
        end

        // weight_down is intentionally unconnected: weights are broadcast, not
        // chained, so pe.sv stays reusable for a chained variant.
        pe #(
          .ACT_W(ACT_W), .WGT_W(WGT_W), .PROD_W(PROD_W), .PSUM_W(PSUM_W)
        ) u_pe (
          .clk(clk), .rst_n(rst_n),
          .wshift(pe_wshift),
          .pe_en(pe_en_int[r]), .mul_en(mul_en_int[r]), .adder_en(adder_en_int[r]),
          .weight_up(weight_byte[c]),
          .act_up_left(act_src),  .act_down_right(act_link[r][c]),
          .psum_up(psum_src),     .psum_down(psum_link[r][c])
        );

      end
    end

    for (c = 0; c < N; c++) begin : g_output
      assign psum_out[c] = psum_link[N-1][c];
    end
  endgenerate

endmodule