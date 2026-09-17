// -----------------------------------------------------------------------------
// barrett_mult.v (Verilog-2001)
// v3.5t - Tightened S3/S4 widths (K=60 fixed), borrow-based final correction
//
// [What changed vs v3.4]
//   - Stage3 product register width: 3*WORD_WIDTH -> 121 bits (q_est[60:0] * q[59:0])
//   - Stage4 remainder/correction width: ~180b -> 61b core + (62b for borrow math)
//   - Final correction uses two 61-bit subtracts with borrow flags (no wide comparators)
//   - No change to I/O, module name, pipeline depth, or algorithm (bit-exact)
//
// [Assumptions]
//   - WORD_WIDTH == 60
//   - q < 2^60, mu is 2*WORD_WIDTH bits (classical Barrett constant)
//   - q_est <= 2^61-1 by construction
//
// Author  : (you + assistant)
// -----------------------------------------------------------------------------
`include "ntt_params.vh"
`timescale 1ns/1ps
`default_nettype none

module barrett_mult #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH  // e.g., 60
)(
    input  wire                       clk,
    input  wire                       rst_n,

    input  wire                       valid_in,
    input  wire [WORD_WIDTH-1:0]      A,
    input  wire [WORD_WIDTH-1:0]      B,
    input  wire [WORD_WIDTH-1:0]      q,
    // mu is the Barrett constant (floor(2^(2K)/q)), provided as 2*WORD_WIDTH bits
    input  wire [2*WORD_WIDTH-1:0]    mu,
    // q_width is kept for compatibility / sim guards (not used to size logic)
    input  wire [5:0]                 q_width,

    output wire                       valid_out,
    output wire [WORD_WIDTH-1:0]      result
);

    // -------------------------------------------------------------------------
    // Local params (K fixed to 60; tighten only the necessary datapaths)
    // -------------------------------------------------------------------------
    localparam integer K_LOCAL  = 60;               // q < 2^60
    localparam integer W        = WORD_WIDTH;       // 60
    localparam integer TW_W     = (2*W);            // 120 : width of T = A*B
    localparam integer QEST_W   = (K_LOCAL+1);      // 61  : width of q_est
    localparam integer PROD_W   = (K_LOCAL+1);      // 121 : width of q_est*q
    localparam integer R_W      = (K_LOCAL+1);      // 61  : r in [0, 2q)

    // -------------------------------------------------------------------------
    // Valid pipeline (4 stages total)
    // -------------------------------------------------------------------------
    reg [3:0] vld;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vld <= 4'b0;
        else        vld <= {vld[2:0], valid_in};
    end
    assign valid_out = vld[3];

`ifndef SYNTHESIS
    // Simulation-time guard (kept from v3.4 family)
    // Ensure q_width belongs to the expected set when valid_in is asserted.
    always @(posedge clk) begin
        if (valid_in) begin
            if (!(q_width==6'd42 || q_width==6'd58 || q_width==6'd59 || q_width==6'd60)) begin
                $display("%m WARNING: q_width=%0d not in {42,58,59,60} at time %0t", q_width, $time);
            end
        end
    end
`endif

    // -------------------------------------------------------------------------
    // S1: Multiply T = A*B  (120-bit)
    // -------------------------------------------------------------------------
    reg [TW_W-1:0] T_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) T_s1 <= {TW_W{1'b0}};
        else if (valid_in) T_s1 <= A * B;
    end

    // Pass-throughs for q/mu if needed downstream
    reg [W-1:0] q_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) q_s1 <= {W{1'b0}};
        else if (valid_in) q_s1 <= q;
    end

    reg [2*W-1:0] mu_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mu_s1 <= {(2*W){1'b0}};
        else if (valid_in) mu_s1 <= mu;
    end

    // mu_table stores floor(2^(2*q_width)/q), so q_width must travel with
    // T, q, and mu to form the Barrett quotient estimate in the next stage.
    reg [5:0] q_width_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) q_width_s1 <= 6'd0;
        else if (valid_in) q_width_s1 <= q_width;
    end

    // -------------------------------------------------------------------------
    // S2: q_est = floor(T * mu / 2^(2*q_width)).
    //     mu_table contains floor(2^(2*q_width)/q) in the low bits.  Do not
    //     slice the top of mu: for q_width < 60 those bits are zero and the
    //     quotient estimate would always be zero.
    // -------------------------------------------------------------------------
    wire [4*W-1:0] qest_prod_s2  = T_s1 * mu_s1;                  // 120x120 -> 240b
    // Extend before doubling: 6-bit 58 << 1 would otherwise truncate 116 to
    // 52 and produce a grossly overestimated quotient.
    wire [4*W-1:0] qest_shift_s2 = qest_prod_s2 >> ({1'b0, q_width_s1} << 1);
    wire [QEST_W-1:0] q_est_s2_w = qest_shift_s2[QEST_W-1:0];

    reg  [QEST_W-1:0] q_est_s2;    // 61b
    reg  [W-1:0]      q_s2;        // 60b (as provided)
    reg  [TW_W-1:0]   T_s2;        // 120b (carry T forward)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_est_s2 <= {QEST_W{1'b0}};
            q_s2     <= {W{1'b0}};
            T_s2     <= {TW_W{1'b0}};
        end else if (vld[0]) begin
            q_est_s2 <= q_est_s2_w;
            q_s2     <= q_s1;
            T_s2     <= T_s1;
        end
    end

    // -------------------------------------------------------------------------
    // S3: sub_val = q_est * q  (tight: 61x60 -> 121b)
    // -------------------------------------------------------------------------
    wire [PROD_W-1:0] qmul_s3 = q_est_s2 * q_s2;   // 61x60 -> 121b

    reg  [PROD_W-1:0] sub_val_s3;                 // 121b (tightened from ~180b)
    reg  [TW_W-1:0]   T_s3;                        // 120b (carry T forward)
    reg  [W-1:0]      q_s3;                        // 60b (carry q forward)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub_val_s3 <= {PROD_W{1'b0}};
            T_s3       <= {TW_W{1'b0}};
            q_s3       <= {W{1'b0}};
        end else if (vld[1]) begin
            sub_val_s3 <= qmul_s3;
            T_s3       <= T_s2;
            q_s3       <= q_s2;
        end
    end

    // -------------------------------------------------------------------------
    // S4: r = T - q_est*q  (tight math)
    //     - T is 120b; expand with 0 to align to 121b and subtract.
    //     - Result r is within [0, 2q), so 61b is sufficient.
    //     - Final correction: subtract q once or twice using borrow flags.
    // -------------------------------------------------------------------------
    wire [PROD_W-1:0] T_ext_s4 = {1'b0, T_s3[TW_W-1:0]}; // 121b: zero-extend
    wire [PROD_W-1:0] r_tmp_s4 = T_ext_s4 - sub_val_s3;  // 121b

    // Keep only the necessary 61b for subsequent corrections
    wire [R_W-1:0]    r0_s4    = r_tmp_s4[R_W-1:0];      // [60:0]

    // Two subtracts of 61b with explicit borrow detection (MSB of widened diff)
    wire [R_W:0] s1_wide = {1'b0, r0_s4} - {1'b0, q_s3};            // 62b
    wire [R_W:0] s2_wide = {1'b0, s1_wide[R_W-1:0]} - {1'b0, q_s3}; // 62b

    wire b1 = s1_wide[R_W]; // 1 => borrow occurred => r0 < q
    wire b2 = s2_wide[R_W]; // 1 => borrow occurred => (r0 - q) < q

    reg  [R_W:0]       r_full_s4; // keep for debug/trace (62b, tightened from ~181b)
    reg  [W-1:0]       res_s4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_full_s4 <= {R_W+1{1'b0}};
            res_s4    <= {W{1'b0}};
        end else if (vld[2]) begin
            r_full_s4 <= {1'b0, r0_s4};

            // Select among r0, (r0-q), (r0-2q) using borrow flags
            if (b1) begin
                // r0 < q : no correction
                res_s4 <= r0_s4[W-1:0];
            end else if (b2) begin
                // q <= r0 < 2q : subtract once
                res_s4 <= s1_wide[W-1:0];
            end else begin
                // r0 >= 2q : subtract twice (rare but safe)
                res_s4 <= s2_wide[W-1:0];
            end
        end
    end

    assign result = res_s4;

endmodule
`default_nettype wire

