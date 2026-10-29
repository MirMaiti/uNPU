`timescale 1ns / 1ps
// =============================================================================
// tb_unpu_top_gls.sv   --   gate-level (netlist) simulation
//
// Self-checking stress testbench for the UNPU chain:
//   APB -> unpu_dma -> unpu_cu -> systolic_array_dip_4x4   (DUT = unpu_top)
//
// Companion file: tb_unpu_top_rtl.sv runs the same tests on the RTL.
// This file differs from it only where a netlist needs it:
//   - DUT instance   : no parameter overrides (the netlist has no parameters;
//                      overriding them is an elaboration error, *E,CUTMIP)
//   - stimulus timing: every TB-driven DUT input (APB bus, SRAM ready/rdata,
//                      rst_n) changes TB_SKEW after the clock edge, never on
//                      it. Cell models carry setup/hold/recovery/removal
//                      checks; an input changing in the same timestep as
//                      the clock trips them and the flop goes X.
//   - no hierarchical references into the DUT (names do not survive synthesis)
//   - optional SDF back-annotation (see HOW TO RUN)
// Keep the tests (LAYER 6) in sync between the two files.
//
// -----------------------------------------------------------------------------
// FILE LAYOUT  (read top to bottom: each layer only uses the layers above it)
// -----------------------------------------------------------------------------
//   behavioral_sram          SRAM model the DMA talks to (separate module)
//
//   tb_unpu_top_gls
//     SECTION A  Knobs & constants      sim parameters, register map
//     SECTION B  Signals & shared data  clk/rst, buses, matrices, counters
//     SECTION C  DUT + SRAM instances
//     SECTION D  Always-on monitors     global watchdog, bus protocol
//                                        checker, DMA activity counter
//     LAYER 1    Reporting              check_pass / check_fail
//     LAYER 2    APB bus driver         apb_write / apb_read / start / reset
//     LAYER 3    Data helpers           random bytes, matrix fills, golden
//                                        model, SRAM backdoor load/read
//     LAYER 4    DUT operations         program_job, wait_for_completion,
//                                        compare_results, dst snapshot
//     LAYER 5    Round runners          run_legal_round, run_illegal_round,
//                                        run_matmul_round (dispatcher)
//     LAYER 6    Tests                  one task per scenario
//     LAYER 7    Main sequence          the list of tests that run
//
// -----------------------------------------------------------------------------
// HOW TO ADD A TEST
// -----------------------------------------------------------------------------
//   1. Copy the TEMPLATE at the top of LAYER 6.
//   2. Fill tb_W / tb_A with a LAYER 3 helper (or set elements by hand).
//   3. Call run_matmul_round(name, src, dst, rows, cols, stall_mode).
//        - rows=4, cols=4  -> job runs, result checked against golden
//        - anything else   -> job must be rejected (STATUS=error only,
//                             no SRAM traffic, dst left untouched)
//   4. Add your task to the main sequence in LAYER 7.
//   Pick src/dst from a free region in the SRAM MAP below.
//
// -----------------------------------------------------------------------------
// SRAM MAP  (byte addresses used by the directed tests; model window = 64 KB)
// -----------------------------------------------------------------------------
//   0x0100 / 0x0200   illegal configs (zero rows/cols)
//   0x0300 / 0x0400   soft-reset abort sweep
//   0x0500 / 0x0600   START-while-busy
//   0x0700 / 0x0800   illegal shapes + 4x4 positive control
//   0x1000 - 0x1F00   extreme-value corners
//   0x2000 / 0x2020   overlapping src/dst
//   0x3000 / 0x3100   back-to-back rounds
//   0x4000 and up     FREE for new directed tests
//   (random regression uses random addresses anywhere)
//   Each job reads 32 bytes from src (8 words) and writes 64 bytes to dst.
//
// -----------------------------------------------------------------------------
// CODING STYLE
// -----------------------------------------------------------------------------
//   Plain, portable SystemVerilog: no classes, no randomize(), no
//   covergroups, and no unpacked arrays in task/function ports (some
//   simulators only partly support them). Matrices are module-scope
//   arrays instead. Randomization uses only $urandom / $urandom_range.
//
//   Matrix convention: every 4x4 matrix is a flat 16-entry array in
//   row-major order, M[r*4 + c] == element (r, c).
//
// -----------------------------------------------------------------------------
// HOW TO RUN (Xcelium)
// -----------------------------------------------------------------------------
//   Zero-delay functional GLS (after synthesis):
//     xrun -64bit -sv -access +rwc -timescale 1ns/1ps \
//          -v <PDK>/scl180/stdcell/fs120/4M1IL/verilog/vcs_sim_model/tsl18fs120_scl.v \
//          ../syn/results/unpu_top_netlist.v \
//          ../tb/tb_unpu_top_gls.sv -top tb_unpu_top_gls -l gls_functional.log
//
//   Timing GLS with SDF (after place & route): add
//          +define+SDF_FILE=\"<path>/unpu_top.sdf\"
//     and raise TB_SKEW (SECTION A) to about a quarter of CLK_PERIOD so the
//     skewed inputs still meet setup with real delays.
//
//   Do not use -notimingchecks as a fix -- timing checks are the point of GLS.
//   The TB is self-checking and calls $finish; the PASS/FAIL summary is
//   printed at the end.
//
//   Parameters were frozen at synthesis. MAIN_TIMEOUT_CYCLES below must match
//   the TIMEOUT_CYCLES the netlist was synthesized with (default 65535).
// =============================================================================


// =============================================================================
// behavioral_sram -- native SRAM slave model
//   - Fixed 1-cycle read latency (rdata valid exactly 1 cycle after an
//     accepted read address), matching unpu_dma assumption A2.
//   - Writes complete in the cycle they are accepted.
//   - stall_mode: 0 = always ready
//                 1 = random ready (READY_PCT % chance per cycle)
//                 2 = periodic stall (ready low 1 cycle out of every 3)
//                 3 = ready stuck low (watchdog test only)
//   - OUT_DLY delays ready/rdata after the clock edge. Use 0 for RTL; use a
//     small non-zero value for gate-level sim so DUT inputs never change in
//     the same timestep as the clock (avoids setup/hold notifier X's).
// =============================================================================
module behavioral_sram #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter int MEM_BYTES  = 65536,      // 64KB window; addr is masked into it
    parameter int READY_PCT  = 70,         // used by stall_mode==1
    parameter int OUT_DLY    = 0           // output delay on ready/rdata (0 for RTL, >0 for GLS)
) (
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire [ADDR_WIDTH-1:0]    addr,
    input  wire [DATA_WIDTH-1:0]    wdata,
    output wire [DATA_WIDTH-1:0]    rdata,
    input  wire [3:0]               wstrb,
    input  wire                     valid,
    output wire                     ready,

    input  wire [1:0]               stall_mode
);
    localparam int MASK_BITS = $clog2(MEM_BYTES);

    reg [7:0] mem [0:MEM_BYTES-1];

    wire [MASK_BITS-1:0] byte_addr = addr[MASK_BITS-1:0];
    wire                 is_read   = valid && (wstrb == 4'b0000);
    wire                 is_write  = valid && (wstrb != 4'b0000);
    reg                  ready_q;          // registered ready; the port is a delayed copy
    wire                 accept    = valid && ready_q;
    wire                 accept_rd = accept && is_read;
    wire                 accept_wr = accept && is_write;

    // ---- ready generation ----
    reg [1:0] burst_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready_q   <= 1'b1;
            burst_cnt <= 2'd0;
        end else begin
            case (stall_mode)
                2'd0: ready_q <= 1'b1;
                2'd3: ready_q <= 1'b0;
                2'd1: ready_q <= ($urandom_range(0, 99) < READY_PCT);
                2'd2: begin
                    burst_cnt <= (burst_cnt == 2'd2) ? 2'd0 : (burst_cnt + 2'd1);
                    ready_q   <= (burst_cnt != 2'd2);
                end
                default: ready_q <= 1'b1;
            endcase
        end
    end

    // ---- write port (no latency) ----
    always @(posedge clk) begin
        if (accept_wr) begin
            if (wstrb[0]) mem[byte_addr + 0] <= wdata[7:0];
            if (wstrb[1]) mem[byte_addr + 1] <= wdata[15:8];
            if (wstrb[2]) mem[byte_addr + 2] <= wdata[23:16];
            if (wstrb[3]) mem[byte_addr + 3] <= wdata[31:24];
        end
    end

    // ---- read port (fixed 1-cycle latency) ----
    reg [MASK_BITS-1:0] raddr_d;
    reg                 rvalid_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raddr_d  <= '0;
            rvalid_d <= 1'b0;
        end else begin
            rvalid_d <= accept_rd;
            if (accept_rd) raddr_d <= byte_addr;
        end
    end

    // ---- outputs to the DUT (OUT_DLY keeps them off the clock edge in GLS) ----
    assign #(OUT_DLY) ready = ready_q;
    assign #(OUT_DLY) rdata = {mem[raddr_d + 3], mem[raddr_d + 2], mem[raddr_d + 1], mem[raddr_d + 0]};

    // ---- backdoor access (zero-time, used by the TB only) ----
    task automatic backdoor_write_word(input [ADDR_WIDTH-1:0] a, input [31:0] d);
        reg [MASK_BITS-1:0] ba;
        begin
            ba = a[MASK_BITS-1:0];
            mem[ba + 0] = d[7:0];
            mem[ba + 1] = d[15:8];
            mem[ba + 2] = d[23:16];
            mem[ba + 3] = d[31:24];
        end
    endtask

    function automatic [31:0] backdoor_read_word(input [ADDR_WIDTH-1:0] a);
        reg [MASK_BITS-1:0] ba;
        begin
            ba = a[MASK_BITS-1:0];
            backdoor_read_word = {mem[ba + 3], mem[ba + 2], mem[ba + 1], mem[ba + 0]};
        end
    endfunction

endmodule


// =============================================================================
// tb_unpu_top_gls -- top-level testbench (gate-level)
// =============================================================================
module tb_unpu_top_gls;

    // #########################################################################
    // SECTION A : KNOBS & CONSTANTS
    // #########################################################################

    // ---- simulation knobs ----
    localparam int ADDR_WIDTH            = 32;
    localparam int DATA_WIDTH            = 32;
    localparam int CLK_PERIOD            = 10;          // 100 MHz
    localparam int TB_SKEW               = 1;           // ns after posedge that TB inputs change
    localparam int NUM_RANDOM_TESTS      = 60;          // raise for a longer regression
    localparam int MAIN_TIMEOUT_CYCLES   = 32'd65535;   // must match the value the netlist was synthesized with
    localparam bit RUN_WATCHDOG_TEST     = 1'b0;        // slow: waits the full DUT timeout
    localparam int GLOBAL_WATCHDOG_NS    = 50_000_000;  // whole-sim safety net
    localparam int POLL_LIMIT            = 25000;       // max STATUS polls per legal round (3 cycles each;
                                                        // > MAIN_TIMEOUT_CYCLES/3 so the DUT watchdog fires first)
    localparam int ILLEGAL_SETTLE_CYCLES = 20;          // wait before checking a rejected job

    // ---- the only matrix shape the DUT accepts ----
    localparam [15:0] LEGAL_ROWS = 16'd4;
    localparam [15:0] LEGAL_COLS = 16'd4;

    // ---- APB register map (mirrors APB.sv) ----
    localparam [31:0] REG_CTRL    = 32'h00;   // bit0 = START, bit1 = soft RESET
    localparam [31:0] REG_STATUS  = 32'h04;   // bit0 = done, bit1 = busy, bit2 = error
    localparam [31:0] REG_DMA_SRC = 32'h08;
    localparam [31:0] REG_DMA_DST = 32'h0C;
    localparam [31:0] REG_MATRIX  = 32'h10;   // {cols[31:16], rows[15:0]}

    // ---- STATUS bit positions ----
    localparam int ST_DONE  = 0;
    localparam int ST_BUSY  = 1;
    localparam int ST_ERROR = 2;

    // ---- SRAM stall modes (see behavioral_sram) ----
    localparam [1:0] STALL_NONE   = 2'd0;
    localparam [1:0] STALL_RANDOM = 2'd1;
    localparam [1:0] STALL_BURST  = 2'd2;
    localparam [1:0] STALL_STUCK  = 2'd3;

    // ---- wait_for_completion() result codes ----
    localparam int RES_DONE    = 0;
    localparam int RES_ERROR   = 1;
    localparam int RES_TIMEOUT = 2;


    // #########################################################################
    // SECTION B : SIGNALS & SHARED DATA
    // #########################################################################

    // ---- clock / reset ----
    reg clk   = 0;
    reg rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- APB bus ----
    reg         psel, penable, pwrite;
    reg  [31:0] paddr, pwdata;
    wire [31:0] prdata;
    wire        pready;

    // ---- native SRAM master bus ----
    wire [ADDR_WIDTH-1:0] dma_addr;
    wire [DATA_WIDTH-1:0] dma_wdata;
    wire [DATA_WIDTH-1:0] dma_rdata;
    wire [3:0]            dma_wstrb;
    wire                  dma_valid;
    wire                  dma_ready;

    reg [1:0] sram_stall_mode = STALL_NONE;

    // ---- scoreboard ----
    integer pass_count = 0;
    integer fail_count = 0;

    // ---- shared matrices (row-major, index = row*4 + col) ----
    reg [7:0]  tb_W         [0:15];   // weights   (fill before a round)
    reg [7:0]  tb_A         [0:15];   // activations (fill before a round)
    reg [31:0] tb_C_golden  [0:15];   // expected result
    reg [31:0] tb_C_dut     [0:15];   // result read back from SRAM
    reg [31:0] tb_dst_snap  [0:15];   // dst contents before a rejected job

    // ---- DMA bus activity counter (proves a rejected job is inert) ----
    integer dma_valid_cnt = 0;


    // #########################################################################
    // SECTION C : DUT + SRAM MODEL
    // #########################################################################

    // Netlist: parameters were resolved at synthesis -- no overrides here.
    unpu_top dut (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite),
        .paddr(paddr), .pwdata(pwdata), .prdata(prdata), .pready(pready),
        .dma_addr(dma_addr), .dma_wdata(dma_wdata), .dma_rdata(dma_rdata),
        .dma_wstrb(dma_wstrb), .dma_valid(dma_valid), .dma_ready(dma_ready)
    );

    behavioral_sram #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
        .MEM_BYTES(65536), .READY_PCT(65),
        .OUT_DLY(TB_SKEW)                   // GLS: ready/rdata change TB_SKEW after the edge
    ) u_sram (
        .clk(clk), .rst_n(rst_n),
        .addr(dma_addr), .wdata(dma_wdata), .rdata(dma_rdata),
        .wstrb(dma_wstrb), .valid(dma_valid), .ready(dma_ready),
        .stall_mode(sram_stall_mode)
    );

    // ---- optional SDF back-annotation (post-layout timing GLS) ----
`ifdef SDF_FILE
    initial $sdf_annotate(`SDF_FILE, dut);
`endif


    // #########################################################################
    // SECTION D : ALWAYS-ON MONITORS
    // #########################################################################

    // ---- global watchdog: never let the sim hang forever ----
    initial begin
        #(GLOBAL_WATCHDOG_NS);
        $display("[%0t] FATAL: global simulation watchdog expired -- forcing $finish", $time);
        $display("=================================================================");
        $display(" TEST SUMMARY (INCOMPLETE -- GLOBAL WATCHDOG FIRED) ");
        $display("   PASS = %0d   FAIL = %0d", pass_count, fail_count);
        $display("=================================================================");
        $finish;
    end

    // ---- SRAM bus protocol checker ----
    // addr / wdata / wstrb must stay stable while valid && !ready.
    reg [ADDR_WIDTH-1:0] prev_addr;
    reg [DATA_WIDTH-1:0] prev_wdata;
    reg [3:0]            prev_wstrb;
    reg                  prev_valid_pending;

    always @(posedge clk) begin
        if (!rst_n) begin
            prev_valid_pending <= 1'b0;
        end else begin
            if (prev_valid_pending &&
                (dma_addr !== prev_addr || dma_wdata !== prev_wdata || dma_wstrb !== prev_wstrb)) begin
                $display("[%0t] **** PROTOCOL VIOLATION: dma bus changed while valid&&!ready ****", $time);
                fail_count = fail_count + 1;
            end
            prev_valid_pending <= dma_valid && !dma_ready;
            prev_addr          <= dma_addr;
            prev_wdata         <= dma_wdata;
            prev_wstrb         <= dma_wstrb;
        end
    end

    // ---- DMA activity counter ----
    always @(posedge clk)
        if (rst_n && dma_valid) dma_valid_cnt <= dma_valid_cnt + 1;


    // #########################################################################
    // LAYER 1 : REPORTING
    // #########################################################################

    task automatic check_pass(input string name);
        begin
            pass_count = pass_count + 1;
            $display("[%0t] PASS : %s", $time, name);
        end
    endtask

    task automatic check_fail(input string name);
        begin
            fail_count = fail_count + 1;
            $display("[%0t] **** FAIL : %s ****", $time, name);
        end
    endtask


    // #########################################################################
    // LAYER 2 : APB BUS DRIVER
    // #########################################################################

    // Every APB signal changes TB_SKEW after the clock edge, never on it, so
    // the netlist flops see stable inputs around each edge. Cycle-for-cycle
    // this is the same protocol as the RTL testbench.

    // One APB write: SETUP cycle, then ACCESS cycle.
    task automatic apb_write(input [31:0] a, input [31:0] d);
        begin
            @(posedge clk); #(TB_SKEW);
            psel = 1'b1; penable = 1'b0; pwrite = 1'b1; paddr = a; pwdata = d;
            @(posedge clk); #(TB_SKEW);
            penable = 1'b1;
            @(posedge clk); #(TB_SKEW);
            psel = 1'b0; penable = 1'b0; pwrite = 1'b0;
        end
    endtask

    // One APB read; data is sampled on the edge that ends the ACCESS cycle.
    task automatic apb_read(input [31:0] a, output [31:0] d);
        begin
            @(posedge clk); #(TB_SKEW);
            psel = 1'b1; penable = 1'b0; pwrite = 1'b0; paddr = a;
            @(posedge clk); #(TB_SKEW);
            penable = 1'b1;
            @(posedge clk);
            d = prdata;
            #(TB_SKEW);
            psel = 1'b0; penable = 1'b0;
        end
    endtask

    // Write CTRL.START (optionally with RESET in the same write).
    task automatic apb_start(input bit soft_reset_bit);
        begin
            apb_write(REG_CTRL, {30'b0, soft_reset_bit, 1'b1});
        end
    endtask

    // Pulse CTRL.RESET: aborts any job, clears busy/done/error.
    task automatic apb_soft_reset_pulse();
        begin
            apb_write(REG_CTRL, 32'h0000_0002);   // RESET=1, START=0
            apb_write(REG_CTRL, 32'h0000_0000);   // clear it again
        end
    endtask


    // #########################################################################
    // LAYER 3 : DATA HELPERS
    // #########################################################################

    // ---- random byte heavily biased toward boundary values ----
    function automatic [7:0] rand_byte_biased();
        integer pick, sel;
        reg [7:0] ev;
        begin
            pick = $urandom_range(0, 99);
            if (pick < 55) begin
                sel = $urandom_range(0, 6);
                case (sel)
                    0: ev = 8'h00;
                    1: ev = 8'h01;
                    2: ev = 8'h7F;
                    3: ev = 8'h80;
                    4: ev = 8'hFE;
                    5: ev = 8'hFF;
                    default: ev = 8'h55;
                endcase
                rand_byte_biased = ev;
            end else begin
                rand_byte_biased = $urandom_range(0, 255);
            end
        end
    endfunction

    // ---- pack one matrix row into a 32-bit word (col0 = MSB) ----
    // Matches systolic_array_dip_4x4.sv's unpack convention.
    function automatic [31:0] pack_row(input [7:0] c0, input [7:0] c1,
                                       input [7:0] c2, input [7:0] c3);
        begin
            pack_row = {c0, c1, c2, c3};
        end
    endfunction

    // ---- matrix fills: weights ----
    task automatic fill_W_const(input [7:0] val);
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1) tb_W[i] = val;
        end
    endtask

    task automatic fill_W_random(input bit biased);
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1)
                tb_W[i] = biased ? rand_byte_biased() : $urandom_range(0, 255);
        end
    endtask

    task automatic fill_W_identity();
        integer i, j;
        begin
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1)
                    tb_W[i*4+j] = (i == j) ? 8'd1 : 8'd0;
        end
    endtask

    task automatic fill_W_pattern_edges();
        begin
            tb_W[0]=8'h00;  tb_W[1]=8'hFF;  tb_W[2]=8'h00;  tb_W[3]=8'hFF;
            tb_W[4]=8'hFF;  tb_W[5]=8'h7F;  tb_W[6]=8'h80;  tb_W[7]=8'h00;
            tb_W[8]=8'h01;  tb_W[9]=8'hFE;  tb_W[10]=8'h01; tb_W[11]=8'hFE;
            tb_W[12]=8'hFF; tb_W[13]=8'h00; tb_W[14]=8'hFF; tb_W[15]=8'h00;
        end
    endtask

    // ---- matrix fills: activations ----
    task automatic fill_A_const(input [7:0] val);
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1) tb_A[i] = val;
        end
    endtask

    task automatic fill_A_random(input bit biased);
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1)
                tb_A[i] = biased ? rand_byte_biased() : $urandom_range(0, 255);
        end
    endtask

    task automatic fill_A_identity();
        integer i, j;
        begin
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1)
                    tb_A[i*4+j] = (i == j) ? 8'd1 : 8'd0;
        end
    endtask

    task automatic fill_A_pattern_edges();
        begin
            tb_A[0]=8'h00;  tb_A[1]=8'hFF;  tb_A[2]=8'h00;  tb_A[3]=8'hFF;
            tb_A[4]=8'hFF;  tb_A[5]=8'h7F;  tb_A[6]=8'h80;  tb_A[7]=8'h00;
            tb_A[8]=8'h01;  tb_A[9]=8'hFE;  tb_A[10]=8'h01; tb_A[11]=8'hFE;
            tb_A[12]=8'hFF; tb_A[13]=8'h00; tb_A[14]=8'hFF; tb_A[15]=8'h00;
        end
    endtask

    // ---- golden reference: C[i][j] = sum_k A[i][k] * W[k][j] ----
    // Verified bit-exact against a cycle-accurate model of the RTL's
    // diagonal (DiP) dataflow. Reads tb_W / tb_A, writes tb_C_golden.
    task automatic golden_matmul();
        integer i, j, k;
        reg [31:0] acc;
        begin
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    acc = 32'd0;
                    for (k = 0; k < 4; k = k + 1)
                        acc = acc + $unsigned(tb_A[i*4+k]) * $unsigned(tb_W[k*4+j]);
                    tb_C_golden[i*4 + j] = acc;
                end
            end
        end
    endtask

    // ---- SRAM backdoor: write the 8-word load stream at src ----
    // 4 weight rows at src+0..12, then 4 activation rows at src+16..28.
    task automatic load_operands_to_sram(input [31:0] src_addr);
        integer t;
        begin
            for (t = 0; t < 4; t = t + 1)
                u_sram.backdoor_write_word(src_addr + t*4,
                    pack_row(tb_W[t*4+0], tb_W[t*4+1], tb_W[t*4+2], tb_W[t*4+3]));
            for (t = 0; t < 4; t = t + 1)
                u_sram.backdoor_write_word(src_addr + 16 + t*4,
                    pack_row(tb_A[t*4+0], tb_A[t*4+1], tb_A[t*4+2], tb_A[t*4+3]));
        end
    endtask

    // ---- SRAM backdoor: read the 16 result words at dst ----
    task automatic read_results_from_sram(input [31:0] dst_addr);
        integer n;
        begin
            for (n = 0; n < 16; n = n + 1)
                tb_C_dut[n] = u_sram.backdoor_read_word(dst_addr + n*4);
        end
    endtask


    // #########################################################################
    // LAYER 4 : DUT OPERATIONS
    // #########################################################################

    // ---- program the job registers and pulse START ----
    task automatic program_job(input [31:0] src_addr, input [31:0] dst_addr,
                               input [15:0] rows,     input [15:0] cols);
        begin
            apb_write(REG_DMA_SRC, src_addr);
            apb_write(REG_DMA_DST, dst_addr);
            apb_write(REG_MATRIX,  {cols, rows});
            apb_start(1'b0);
        end
    endtask

    // ---- poll STATUS until error or done (error is checked first) ----
    // result: RES_DONE, RES_ERROR, or RES_TIMEOUT (TB-side bound hit)
    task automatic wait_for_completion(input integer max_polls, output integer result);
        reg [31:0] status;
        integer i;
        begin
            result = RES_TIMEOUT;
            for (i = 0; i < max_polls; i = i + 1) begin
                if (result == RES_TIMEOUT) begin
                    apb_read(REG_STATUS, status);
                    if      (status[ST_ERROR]) result = RES_ERROR;
                    else if (status[ST_DONE])  result = RES_DONE;
                end
            end
        end
    endtask

    // ---- compare tb_C_dut against tb_C_golden; print each mismatch ----
    task automatic compare_results(input string name, output integer mismatches);
        integer n;
        begin
            mismatches = 0;
            for (n = 0; n < 16; n = n + 1)
                if (tb_C_dut[n] !== tb_C_golden[n]) mismatches = mismatches + 1;

            if (mismatches == 0)
                check_pass(name);
            else begin
                check_fail({name, " : result mismatch"});
                for (n = 0; n < 16; n = n + 1)
                    if (tb_C_dut[n] !== tb_C_golden[n])
                        $display("    [%0d] expected=%0d got=%0d", n, tb_C_golden[n], tb_C_dut[n]);
            end
        end
    endtask

    // ---- snapshot dst, then later count words that changed ----
    task automatic snapshot_dst(input [31:0] dst_addr);
        integer n;
        begin
            for (n = 0; n < 16; n = n + 1)
                tb_dst_snap[n] = u_sram.backdoor_read_word(dst_addr + n*4);
        end
    endtask

    task automatic count_dst_changes(input [31:0] dst_addr, output integer changed);
        integer n;
        begin
            changed = 0;
            for (n = 0; n < 16; n = n + 1)
                if (u_sram.backdoor_read_word(dst_addr + n*4) !== tb_dst_snap[n])
                    changed = changed + 1;
        end
    endtask


    // #########################################################################
    // LAYER 5 : ROUND RUNNERS
    // #########################################################################

    // ---- legal 4x4 job: run it and compare against the golden model ----
    task automatic run_legal_round(
        input string name,
        input [31:0] src_addr,
        input [31:0] dst_addr,
        input [1:0]  stall_mode
    );
        reg [31:0] status;
        integer    poll_res;
        integer    mismatches;
        begin
            sram_stall_mode = stall_mode;
            load_operands_to_sram(src_addr);
            golden_matmul();

            program_job(src_addr, dst_addr, LEGAL_ROWS, LEGAL_COLS);
            wait_for_completion(POLL_LIMIT, poll_res);

            if (poll_res == RES_TIMEOUT) begin
                check_fail({name, " : TB poll timeout waiting for done/error"});
            end
            else if (poll_res == RES_ERROR) begin
                check_fail({name, " : DUT reported ERROR on a valid config"});
            end
            else begin
                apb_read(REG_STATUS, status);
                if (status[ST_BUSY] !== 1'b0)
                    check_fail({name, " : busy still asserted after done"});
                else begin
                    read_results_from_sram(dst_addr);
                    compare_results(name, mismatches);
                end
            end
        end
    endtask

    // ---- illegal shape: the DUT must reject it cleanly ----
    // Expect STATUS = error only, no SRAM traffic, dst left untouched.
    // Soft-resets before (clean slate) and after (back to IDLE).
    task automatic run_illegal_round(
        input string name,
        input [31:0] src_addr,
        input [31:0] dst_addr,
        input [15:0] rows,
        input [15:0] cols
    );
        reg [31:0] status;
        integer    valid_before;
        integer    changed;
        string     tag;
        begin
            tag = $sformatf("%s : cfg %0dx%0d", name, rows, cols);

            apb_soft_reset_pulse();
            load_operands_to_sram(src_addr);
            snapshot_dst(dst_addr);

            valid_before = dma_valid_cnt;
            program_job(src_addr, dst_addr, rows, cols);
            repeat (ILLEGAL_SETTLE_CYCLES) @(posedge clk);
            apb_read(REG_STATUS, status);
            count_dst_changes(dst_addr, changed);

            if (status[2:0] !== 3'b100)
                check_fail($sformatf("%s expected STATUS=100 (error), got %b", tag, status[2:0]));
            else if (dma_valid_cnt != valid_before)
                check_fail({tag, " rejected but DMA touched SRAM"});
            else if (changed != 0)
                check_fail({tag, " rejected but dst region modified"});
            else
                check_pass({tag, " rejected"});

            apb_soft_reset_pulse();
        end
    endtask

    // ---- dispatcher: the one call most tests need ----
    // 4x4 -> run_legal_round, anything else -> run_illegal_round.
    task automatic run_matmul_round(
        input string name,
        input [31:0] src_addr,
        input [31:0] dst_addr,
        input [15:0] rows,
        input [15:0] cols,
        input [1:0]  stall_mode     // ignored for illegal shapes (no SRAM access expected)
    );
        begin
            if (rows == LEGAL_ROWS && cols == LEGAL_COLS)
                run_legal_round(name, src_addr, dst_addr, stall_mode);
            else
                run_illegal_round(name, src_addr, dst_addr, rows, cols);
        end
    endtask


    // #########################################################################
    // LAYER 6 : TESTS
    // #########################################################################

    // -------------------------------------------------------------------------
    // TEMPLATE -- copy this to add a test, then list it in LAYER 7.
    // -------------------------------------------------------------------------
    // task automatic test_my_new_case();
    //     begin
    //         fill_W_random(1);                       // 1. choose operands
    //         fill_A_identity();
    //         run_matmul_round("my case",             // 2. run + self-check
    //                          32'h0000_4000,         //    src (free region)
    //                          32'h0000_4100,         //    dst
    //                          16'd4, 16'd4,          //    rows, cols
    //                          STALL_RANDOM);         //    SRAM back-pressure
    //     end
    // endtask
    // -------------------------------------------------------------------------

    // ---- TEST 1: reset state ----
    task automatic test_reset_state();
        reg [31:0] v;
        begin
            apb_read(REG_STATUS, v);
            if (v[2:0] === 3'b000) check_pass("reset: STATUS is idle (done=busy=error=0)");
            else check_fail($sformatf("reset: STATUS not idle after reset (got %b)", v[2:0]));

            apb_read(REG_DMA_SRC, v);
            if (v === 32'h0) check_pass("reset: DMA_SRC_ADDR reads back 0");
            else check_fail($sformatf("reset: DMA_SRC_ADDR not 0 after reset (got %h)", v));
        end
    endtask

    // ---- TEST 2: APB register map, including out-of-range reads ----
    task automatic test_apb_register_map();
        reg [31:0] v;
        begin
            apb_write(REG_DMA_SRC, 32'hA5A5_A5A0);
            apb_read (REG_DMA_SRC, v);
            if (v === 32'hA5A5_A5A0) check_pass("apb: DMA_SRC_ADDR write/readback");
            else                     check_fail("apb: DMA_SRC_ADDR write/readback");

            apb_write(REG_DMA_DST, 32'h1234_5670);
            apb_read (REG_DMA_DST, v);
            if (v === 32'h1234_5670) check_pass("apb: DMA_DST_ADDR write/readback");
            else                     check_fail("apb: DMA_DST_ADDR write/readback");

            apb_write(REG_MATRIX, {16'd4, 16'd4});
            apb_read (REG_MATRIX, v);
            if (v === {16'd4, 16'd4}) check_pass("apb: MATRIX_CFG write/readback");
            else                      check_fail("apb: MATRIX_CFG write/readback");

            apb_read(32'h40, v);   // word 0x10, beyond the last register
            if (v === 32'hDEAD_BEEF) check_pass("apb: out-of-range read returns DEAD_BEEF");
            else                     check_fail("apb: out-of-range read did not return DEAD_BEEF");

            apb_read(32'hFC, v);   // word 0x3F
            if (v === 32'hDEAD_BEEF) check_pass("apb: far out-of-range read returns DEAD_BEEF");
            else                     check_fail("apb: far out-of-range read did not return DEAD_BEEF");
        end
    endtask

    // ---- TEST 3: zero rows / zero cols are rejected ----
    task automatic test_invalid_config();
        begin
            run_matmul_round("invalid_cfg cols=0", 32'h0000_0100, 32'h0000_0200,
                             16'd4, 16'd0, STALL_NONE);
            run_matmul_round("invalid_cfg rows=0", 32'h0000_0100, 32'h0000_0200,
                             16'd0, 16'd4, STALL_NONE);
        end
    endtask

    // ---- TEST 4: only 4x4 is accepted; every other shape is rejected ----
    task automatic test_illegal_shapes();
        begin
            fill_W_random(1);
            fill_A_random(1);
            // other factorizations of 16 elements
            run_matmul_round("factor 1x16", 32'h0000_0700, 32'h0000_0800, 16'd1,  16'd16, STALL_NONE);
            run_matmul_round("factor 2x8",  32'h0000_0700, 32'h0000_0800, 16'd2,  16'd8,  STALL_NONE);
            run_matmul_round("factor 8x2",  32'h0000_0700, 32'h0000_0800, 16'd8,  16'd2,  STALL_NONE);
            run_matmul_round("factor 16x1", 32'h0000_0700, 32'h0000_0800, 16'd16, 16'd1,  STALL_NONE);
            // wrong element count
            run_matmul_round("shape 4x8",   32'h0000_0700, 32'h0000_0800, 16'd4,  16'd8,  STALL_NONE);
            run_matmul_round("shape 3x4",   32'h0000_0700, 32'h0000_0800, 16'd3,  16'd4,  STALL_NONE);
            run_matmul_round("shape 1x1",   32'h0000_0700, 32'h0000_0800, 16'd1,  16'd1,  STALL_NONE);
            // low bits look like 4 but upper bits set (needs a full 16-bit compare)
            run_matmul_round("shape upper-bits", 32'h0000_0700, 32'h0000_0800, 16'h0004, 16'h1004, STALL_NONE);
            run_matmul_round("shape all-ones",   32'h0000_0700, 32'h0000_0800, 16'hFFFF, 16'hFFFF, STALL_NONE);
            // positive control: 4x4 still works after all the rejections
            run_matmul_round("factor 4x4",  32'h0000_0700, 32'h0000_0800, 16'd4,  16'd4,  STALL_BURST);
        end
    endtask

    // ---- TEST 5: extreme operand values ----
    task automatic test_extreme_corners();
        begin
            // max accumulation: 4 * 255 * 255 = 260100, must not wrap
            fill_W_const(8'hFF);
            fill_A_const(8'hFF);
            run_matmul_round("extreme: W=0xFF A=0xFF (max accumulation)",
                             32'h0000_1000, 32'h0000_1100, 16'd4, 16'd4, STALL_NONE);
            run_matmul_round("extreme: W=0xFF A=0xFF, random-stall SRAM",
                             32'h0000_1000, 32'h0000_1100, 16'd4, 16'd4, STALL_RANDOM);
            run_matmul_round("extreme: W=0xFF A=0xFF, burst-stall SRAM",
                             32'h0000_1000, 32'h0000_1100, 16'd4, 16'd4, STALL_BURST);

            fill_W_const(8'h00);
            fill_A_const(8'h00);
            run_matmul_round("extreme: W=0x00 A=0x00",
                             32'h0000_1200, 32'h0000_1300, 16'd4, 16'd4, STALL_NONE);

            fill_W_const(8'h00);
            fill_A_const(8'hFF);
            run_matmul_round("extreme: W=0x00 A=0xFF",
                             32'h0000_1400, 32'h0000_1500, 16'd4, 16'd4, STALL_RANDOM);

            fill_W_identity();
            fill_A_const(8'hFF);
            run_matmul_round("extreme: identity(W) A=0xFF (pass-through check)",
                             32'h0000_1600, 32'h0000_1700, 16'd4, 16'd4, STALL_NONE);

            fill_W_const(8'hFF);
            fill_A_identity();
            run_matmul_round("extreme: W=0xFF identity(A)",
                             32'h0000_1800, 32'h0000_1900, 16'd4, 16'd4, STALL_BURST);

            fill_W_pattern_edges();
            fill_A_pattern_edges();
            run_matmul_round("extreme: boundary-value checkerboard both operands",
                             32'h0000_1A00, 32'h0000_1B00, 16'd4, 16'd4, STALL_RANDOM);

            // single hot element in the first PE
            fill_W_const(8'h00);
            fill_A_const(8'h00);
            tb_W[0]  = 8'hFF;
            tb_A[0]  = 8'hFF;
            run_matmul_round("extreme: single hot element W[0][0]=A[0][0]=0xFF",
                             32'h0000_1C00, 32'h0000_1D00, 16'd4, 16'd4, STALL_NONE);

            // single hot element in the last PE
            fill_W_const(8'h00);
            fill_A_const(8'h00);
            tb_W[15] = 8'hFF;
            tb_A[15] = 8'hFF;
            run_matmul_round("extreme: single hot element W[3][3]=A[3][3]=0xFF (last PE)",
                             32'h0000_1E00, 32'h0000_1F00, 16'd4, 16'd4, STALL_NONE);
        end
    endtask

    // ---- TEST 6: dst starts right where the src load region ends ----
    task automatic test_overlap_addresses();
        begin
            fill_W_random(1);
            fill_A_random(1);
            run_matmul_round("overlap: dst immediately follows src load region",
                             32'h0000_2000, 32'h0000_2020, 16'd4, 16'd4, STALL_NONE);
        end
    endtask

    // ---- TEST 7: soft reset at every delay 0..25 cycles after START ----
    task automatic test_soft_reset_abort_sweep();
        integer d;
        begin
            for (d = 0; d <= 25; d = d + 1) begin
                // start a normal job...
                sram_stall_mode = STALL_NONE;
                fill_W_random(1);
                fill_A_random(1);
                load_operands_to_sram(32'h0000_0300);
                program_job(32'h0000_0300, 32'h0000_0400, 16'd4, 16'd4);

                // ...abort it after d cycles, let the pipeline drain...
                repeat (d) @(posedge clk);
                apb_soft_reset_pulse();
                repeat (10) @(posedge clk);

                // ...and check a fresh job still computes correctly
                run_matmul_round($sformatf("abort d=%0d: recovery round", d),
                                 32'h0000_0300, 32'h0000_0400, 16'd4, 16'd4, STALL_NONE);
            end
        end
    endtask

    // ---- TEST 8: a second START while busy must be ignored ----
    task automatic test_start_while_busy();
        integer poll_res;
        integer mismatches;
        begin
            sram_stall_mode = STALL_BURST;   // slow the job so START lands mid-flight
            fill_W_random(1);
            fill_A_random(1);
            load_operands_to_sram(32'h0000_0500);
            golden_matmul();
            program_job(32'h0000_0500, 32'h0000_0600, 16'd4, 16'd4);

            // stage a different src and fire START again
            repeat (4) @(posedge clk);
            apb_write(REG_DMA_SRC, 32'hDEAD_0000);
            apb_start(1'b0);

            wait_for_completion(POLL_LIMIT, poll_res);
            if (poll_res != RES_DONE)
                check_fail("start_while_busy: round never completed cleanly");
            else begin
                read_results_from_sram(32'h0000_0600);
                compare_results("start_while_busy: redundant START ignored, result still correct",
                                mismatches);
            end
            sram_stall_mode = STALL_NONE;
        end
    endtask

    // ---- TEST 9: back-to-back rounds with no idle gap ----
    task automatic test_back_to_back();
        integer i;
        begin
            for (i = 0; i < 5; i = i + 1) begin
                fill_W_random(1);
                fill_A_random(1);
                run_matmul_round($sformatf("back_to_back round %0d", i),
                                 32'h0000_3000, 32'h0000_3100, 16'd4, 16'd4,
                                 $urandom_range(0, 2));
            end
        end
    endtask

    // ---- TEST 10: stuck SRAM trips the DUT watchdog (off by default) ----
    task automatic test_watchdog_timeout();
        integer i;
        reg [31:0] v;
        reg        saw_error;
        begin
            if (!RUN_WATCHDOG_TEST) begin
                $display("[%0t] INFO : watchdog test skipped (RUN_WATCHDOG_TEST=0)", $time);
            end else begin
                sram_stall_mode = STALL_STUCK;
                repeat (3) @(posedge clk);
                program_job(32'h0000_1000, 32'h0000_2000, 16'd4, 16'd4);

                saw_error = 0;
                for (i = 0; i < (MAIN_TIMEOUT_CYCLES + 200) && !saw_error; i = i + 1) begin
                    @(posedge clk);
                    if (i % 64 == 0) begin
                        apb_read(REG_STATUS, v);
                        if (v[ST_ERROR]) saw_error = 1;
                    end
                end
                apb_read(REG_STATUS, v);
                if (saw_error && !v[ST_BUSY]) check_pass("watchdog: stuck SRAM trips timeout -> ERROR");
                else                          check_fail("watchdog: timeout did not fire as expected");

                sram_stall_mode = STALL_NONE;
                apb_soft_reset_pulse();
                repeat (3) @(posedge clk);
                apb_read(REG_STATUS, v);
                if (v[2:0] === 3'b000) check_pass("watchdog: soft_reset clears sticky ERROR");
                else                   check_fail("watchdog: soft_reset did not clear ERROR");
            end
        end
    endtask

    // ---- TEST 11: randomized regression ----
    // Random operands, random word-aligned addresses across the full 32-bit
    // space (dst kept 0x8000 above src so they never alias in the 64 KB
    // SRAM window), random back-pressure mode.
    task automatic test_random_regression();
        integer    i;
        reg [31:0] src_a, dst_a;
        reg [1:0]  mode;
        begin
            for (i = 0; i < NUM_RANDOM_TESTS; i = i + 1) begin
                fill_W_random(1);
                fill_A_random(1);
                src_a = $urandom & 32'hFFFF_FFFC;
                dst_a = (src_a + 32'h0000_8000) & 32'hFFFF_FFFC;
                mode  = $urandom_range(0, 2);
                run_matmul_round($sformatf("random[%0d] mode=%0d", i, mode),
                                 src_a, dst_a, 16'd4, 16'd4, mode);
            end
        end
    endtask


    // #########################################################################
    // LAYER 7 : MAIN SEQUENCE
    // #########################################################################

    initial begin
        // ---- idle bus, hardware reset ----
        psel = 0; penable = 0; pwrite = 0; paddr = 0; pwdata = 0;
        sram_stall_mode = STALL_NONE;

        rst_n = 0;
        repeat (5) @(posedge clk);
        @(negedge clk);              // release mid-cycle: never on a clock edge
        rst_n = 1;
        repeat (5) @(posedge clk);

        $display("=================================================================");
        $display(" UNPU stress testbench starting");
        $display("=================================================================");

        // ---- tests (add new ones here) ----
        test_reset_state();
        test_apb_register_map();
        test_invalid_config();
        test_extreme_corners();
        test_illegal_shapes();
        test_overlap_addresses();
        test_soft_reset_abort_sweep();
        test_start_while_busy();
        test_back_to_back();
        test_watchdog_timeout();
        test_random_regression();

        // ---- summary ----
        $display("=================================================================");
        $display(" TEST SUMMARY");
        $display("   PASS = %0d", pass_count);
        $display("   FAIL = %0d", fail_count);
        if (fail_count == 0) $display("   RESULT: ALL TESTS PASSED");
        else                 $display("   RESULT: %0d TEST(S) FAILED", fail_count);
        $display("=================================================================");

        $finish;
    end

endmodule