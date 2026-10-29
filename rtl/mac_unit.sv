`timescale 1ns/1ps
// mac_unit.sv
//
// MAC unit sized for a single 4x4-array tile pass: unsigned 8-bit
// operands, 16-bit product register (exact for 8x8 unsigned), 18-bit
// accumulator register.
//
// Width derivation for the accumulator:
//   max single product   = 255 * 255      = 65,025   (needs 16 bits)
//   worst-case K=4 sum    = 4 * 65,025    = 260,100  (needs 18 bits)
//   accumulator_bits = product_bits + ceil(log2(K)) = 16 + 2 = 18
//
// !! SCOPE CAVEAT !!
// 18 bits is exact for K=4 -- a single tile pass with psum_in tied to 0.
// It is NOT sufficient the moment psum_in carries a running sum chained
// in from a previous tile (multi-tile / software-tiling accumulation).
// Effective K then exceeds 4 and this width must be revisited -- widen
// PSUM_W using the same formula with the larger effective K before
// using this MAC unit in any chained-accumulation scheme.

module mac_unit #(
  parameter int ACT_W  = 8,   // activation width, UNSIGNED
  parameter int WGT_W  = 8,   // weight width, UNSIGNED
  parameter int PROD_W = 16,  // multiplier register width -- exact for 8x8 unsigned
  parameter int PSUM_W = 18   // adder register width -- exact for K=4, see header
) (
  input  logic                 clk,
  input  logic                 rst_n,     // active-low, synchronous

  input  logic                 mul_en,    // latch a new product this cycle
  input  logic                 adder_en,  // latch a new accumulated sum this cycle

  input  logic [ACT_W-1:0]     act_in,
  input  logic [WGT_W-1:0]     weight_in,
  input  logic [PSUM_W-1:0]    psum_in,

  output logic [PSUM_W-1:0]    psum_out
);

  logic [PROD_W-1:0] mult_reg;
  logic [PSUM_W-1:0] psum_reg;

  // Stage 1: multiply, registered (unsigned, exact fit in 16 bits)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      mult_reg <= '0;
    else if (mul_en)
      mult_reg <= act_in * weight_in;
  end

  // Stage 2: accumulate, registered. mult_reg (16-bit) is zero-extended
  // to PSUM_W (18-bit) automatically in the addition below since psum_reg
  // is the wider operand -- verified explicitly in the testbench rather
  // than assumed.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      psum_reg <= '0;
    else if (adder_en)
      psum_reg <= psum_in + mult_reg;
  end

  assign psum_out = psum_reg;

endmodule
