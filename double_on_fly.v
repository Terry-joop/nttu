
// -----------------------------------------------------------------------------
// double_on_fly.v  (Verilog-2001)
// Version : v1.0.0
// Author  : (you + assistant)
// -----------------------------------------------------------------------------
// Purpose
//   - Phase-2 "ROW-NTT" on-the-fly twiddle generator & applier (M=16 lanes).
//   - Generates per-lane twist stream with II=1:
//       tw(k,r) = zeta^{ r * (C + 16*k) }  (k=row index, r=lane, C=tile column)
//   - Consumes one 16-wide frame each cycle (after initial 2-cycle warm-up).
//
// How it works (Shoup 16-time-multiplexed for even/odd chains)
//   - Seeds per lane: T0_even[r], T1_odd[r]; ratio H2[r] = zeta^{32r}, H2pre[r].
//   - Cycle t=0: feed T0_even -> Shoup starts (result @ t=2).
//     Cycle t=1: feed T1_odd  -> Shoup starts (result @ t=3).
//     Cycle t>=2: feed previous Shoup result each cycle.
//   - Because Shoup has 2-cycle latency, the output naturally alternates
//     (even chain at even rows, odd chain at odd rows) with no extra regs.
//   - Barrett multiplies input data by the SAME 'twist' sent to Shoup (wire-first).
//
// Interface notes
//   - Use double_* ROMs:
//       t0_even_vec  <= double_t0_even.hex  (bank,tileC -> lanes)
//       t1_odd_vec   <= double_t1_odd.hex
//       h2_vec       <= double_h2.hex       (bank -> lanes)
//       h2pre_vec    <= double_h2pre.hex
//   - q/mu/q_width come from q_table/mu_table/q_width_table.
//   - 'start' is a 1-cycle pulse to (re)load seeds at tile/bank boundary.
//   - 'busy' is asserted after the first 'start' until reset.
//
// Design rules
//   - Verilog-2001 only. (SV slice form [+:] is used like in your single; if you
//     need strict 2001, replace with generate-time indices.)
//   - One reg is driven by exactly one always block.
//   - Wire-first: twist is a wire used by both Barrett(B) and Shoup(a) in the same cycle.
//   - Comments in English.
// -----------------------------------------------------------------------------

`include "ntt_params.vh"
`default_nettype none

module double_on_fly #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH,
    parameter integer LANES      = `LANES
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // Control
    input  wire                         start,          // 1-cycle pulse (tile/bank boundary)
    output wire                         busy,           // asserted after first start

    // Lane-local constants (from double_* ROMs)
    input  wire [LANES*WORD_WIDTH-1:0]  t0_even_vec,    // T0_even[r]  = zeta^{r*C}
    input  wire [LANES*WORD_WIDTH-1:0]  t1_odd_vec,     // T1_odd[r]   = T0_even[r] * zeta^{16r}
    input  wire [LANES*WORD_WIDTH-1:0]  h2_vec,         // H2[r]       = zeta^{32r}
    input  wire [LANES*WORD_WIDTH-1:0]  h2pre_vec,      // H2pre[r]    = floor(H2*2^W/q)

    // Modulus & Barrett params (per-bank)
    input  wire [WORD_WIDTH-1:0]        q,
    input  wire [2*WORD_WIDTH-1:0]      mu,
    input  wire [5:0]                   q_width,        // dynamic bit-length of q

    // Per-lane input stream (A operands for barrett)
    input  wire                         in_valid,       // 1 frame/cycle across lanes
    input  wire [LANES*WORD_WIDTH-1:0]  in_data_vec,

    // Outputs
    output wire                         barrett_valid_out,   // representative valid (lane0)
    output wire [LANES*WORD_WIDTH-1:0]  barrett_result_vec
);

    // =========================================================================
    // 1) RUN/IDLE state and seed phase counter
    // =========================================================================
    reg run_now;             // RUN after start; cleared only by reset
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) run_now <= 1'b0;
        else if (start) run_now <= 1'b1;
        else run_now <= run_now;
    end
    assign busy = run_now;

    // seed_ctr: 0 -> feed T0_even, 1 -> feed T1_odd, >=2 -> feed Shoup result
    reg [1:0] seed_ctr;
    wire      use_seed_phase = (seed_ctr < 2);
    wire      consume_now    = run_now & in_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seed_ctr <= 2'd0;
        end else if (start) begin
            seed_ctr <= 2'd0;                  // reload seeds at boundary
        end else if (consume_now && (seed_ctr != 2'd2)) begin
            seed_ctr <= seed_ctr + 2'd1;       // advance only when we actually consume
        end else begin
            seed_ctr <= seed_ctr;
        end
    end

    // =========================================================================
    // 2) Per-lane datapath: twist selection (wire-first) + Shoup + Barrett
    // =========================================================================
    wire [LANES-1:0]             barrett_valid_out_vec;
    wire [LANES-1:0]             shoup_valid_out_vec;    // not used, but kept for symmetry

    genvar i;
    generate
        for (i = 0; i < LANES; i = i + 1) begin : g_lane
            // ---- Slice lane-local constants & inputs
            wire [WORD_WIDTH-1:0] T0_even_i = t0_even_vec[i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] T1_odd_i  = t1_odd_vec [i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] H2_i      = h2_vec     [i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] H2pre_i   = h2pre_vec  [i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] A_i       = in_data_vec[i*WORD_WIDTH +: WORD_WIDTH];

            // Shoup result (registered inside shoup_mult)
            wire [WORD_WIDTH-1:0] shoup_res_i;

            // ---- Twist selection (wire-first)
            // - During seed phase (seed_ctr=0,1): feed T0_even / T1_odd.
            // - Thereafter: feed back shoup_res_i each cycle.
            // - When not consuming, drive 0 to keep combinational fanout quiet.
            wire [WORD_WIDTH-1:0] twist_i =
                consume_now
                  ? (use_seed_phase
                        ? (seed_ctr == 2'd0 ? T0_even_i
                           : /*seed_ctr==1*/  T1_odd_i)
                        : shoup_res_i)
                  : {WORD_WIDTH{1'b0}};

            // ---- Shoup multiply: a * H2 (mod q) using precomputed H2pre
            shoup_mult #(
                .WORD_WIDTH (WORD_WIDTH),
                .K_WIDTH    (WORD_WIDTH)
            ) u_shoup (
                .clk        (clk),
                .rst_n      (rst_n),
                .valid_in   (consume_now),
                .a          (twist_i),
                .ratio      (H2_i),
                .z_pre      (H2pre_i),
                .q          (q),
                .valid_out  (shoup_valid_out_vec[i]),    // aligned to 'result'
                .result     (shoup_res_i)
            );

            // ---- Barrett multiply: data * twist (mod q)
            barrett_mult #(
                .WORD_WIDTH (WORD_WIDTH)
            ) u_barrett (
                .clk        (clk),
                .rst_n      (rst_n),
                .valid_in   (consume_now),
                .A          (A_i),
                .B          (twist_i),
                .q          (q),
                .mu         (mu),
                .q_width    (q_width),
                .valid_out  (barrett_valid_out_vec[i]),
                .result     (barrett_result_vec[i*WORD_WIDTH +: WORD_WIDTH])
            );
        end
    endgenerate

    // Representative barrett_valid_out (all lanes are time-aligned)
    assign barrett_valid_out = barrett_valid_out_vec[0];

endmodule

`default_nettype wire
