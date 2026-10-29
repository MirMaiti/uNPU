`timescale 1ns/1ps
// pe.sv
//
// One systolic-array PE for the DiP dataflow: weight register (loaded
// once, directly from the array-level broadcast bus, gated by a
// per-PE-hardcoded `wshift` pulse -- see systolic_array_dip_4x4.sv),
// input/activation register (enabled per-row by `pe_en`), and the
// already-verified mac_unit doing the actual multiply-accumulate.
//
// weight_down was removed: this PE never relays its weight to another
// PE. The array broadcasts weight_data/weight_word to every row directly
// and each PE captures its own value once, so there is no PE-to-PE
// weight chain to be the source of.
//
// This PE is identical to what a conventional WS array would use --
// DiP does not change the PE itself, only how PEs are wired together
// at the array level. The PE does not know or care whether it's in row
// 0 or row 3; the array-level wiring decides what feeds weight_up /
// act_up_left for each instance.

module pe #(
  parameter int ACT_W  = 8,
  parameter int WGT_W  = 8,
  parameter int PROD_W = 16,
  parameter int PSUM_W = 18
) (
  input  logic clk,
  input  logic rst_n,

  // control
  input  logic wshift,      // per-PE hardcoded pulse: latch a new weight this cycle
  input  logic pe_en,       // per-row: latch a new activation this cycle
  input  logic mul_en,      // per-row: latch a new product this cycle
  input  logic adder_en,    // per-row: latch a new accumulated sum this cycle

  // weight -- single capture, no downstream chain
  input  logic [WGT_W-1:0] weight_up,     // from the array-level broadcast bus

  // activation chain -- diagonal, per DiP dataflow (see array-level wiring)
  input  logic [ACT_W-1:0] act_up_left,   // from PE(row-1, col+1 mod N), or external row 0
  output logic [ACT_W-1:0] act_down_right,// this PE's held activation, read by the row below

  // partial-sum chain -- vertical, straight down each column
  input  logic [PSUM_W-1:0] psum_up,      // from the PE above (0, or chained tile, for row 0)
  output logic [PSUM_W-1:0] psum_down     // this PE's running sum, to the PE below
);

  logic [WGT_W-1:0] weight_reg;
  logic [ACT_W-1:0] input_reg;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      weight_reg <= '0;
    else if (wshift)
      weight_reg <= weight_up;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      input_reg <= '0;
    else if (pe_en)
      input_reg <= act_up_left;
  end
  assign act_down_right = input_reg;

  mac_unit #(
    .ACT_W(ACT_W), .WGT_W(WGT_W), .PROD_W(PROD_W), .PSUM_W(PSUM_W)
  ) u_mac (
    .clk(clk), .rst_n(rst_n),
    .mul_en(mul_en), .adder_en(adder_en),
    .act_in(input_reg), .weight_in(weight_reg), .psum_in(psum_up),
    .psum_out(psum_down)
  );

endmodule
