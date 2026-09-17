// -----------------------------------------------------------------------------
// shoup_mult.v (Verilog-2001)
// Version : v2.0f-lean  (U2 removed, MULHI via slice; latency unchanged)
// Purpose : Shoup modular multiply by "ratio" using precomputed z_pre
// Formula : res = (a*ratio - floor(a*z_pre / 2^K) * q) mod q
// Notes   : - II = 1 (new input each cycle)
//           - WORD_WIDTH and K_WIDTH default to 64 (vectors are 2^W-based)
//           - result is aligned with valid_out in the SAME cycle
// Changes : v2.0f-lean vs v2.0e-mod
//   [1] MULHI for a*z_pre: q_est_s1 <= (a*z_pre) >> K (synth can drop low tree)
//   [2] Removed second optional (-q) correction (U2) per Shoup t?(-q,2q)
//   [3] No change to I/O, pipeline depth (2), or timing of valid_out/result
// -----------------------------------------------------------------------------
`include "ntt_params.vh"

module shoup_mult #(
    parameter WORD_WIDTH = `NTT_WORD_WIDTH,
    parameter K_WIDTH    = `NTT_WORD_WIDTH
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  valid_in,
    input  wire [WORD_WIDTH-1:0] a,
    input  wire [WORD_WIDTH-1:0] ratio,
    input  wire [WORD_WIDTH-1:0] z_pre,    // floor(ratio * 2^K / q)
    input  wire [WORD_WIDTH-1:0] q,

    output reg                   valid_out, // registered to align with result
    output reg  [WORD_WIDTH-1:0] result
);

    // -----------------------------
    // (Optional) Build guard: Shoup K should equal WORD_WIDTH in this design
    // -----------------------------
    // synopsys translate_off
    initial begin
        if (K_WIDTH != WORD_WIDTH) begin
            $display("ERROR(shoup_mult): K_WIDTH(%0d) must equal WORD_WIDTH(%0d) for MULHI optimization.", K_WIDTH, WORD_WIDTH);
            $fatal;
        end
    end
    // synopsys translate_on

    // -----------------------------
    // Valid pipeline (2 stages)
    // -----------------------------
    reg vld_s1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vld_s1 <= 1'b0;
        end else begin
            vld_s1 <= valid_in;
        end
    end

    // valid_out is vld_s1 delayed by one cycle (align to result)
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            valid_out <= 1'b0;
        end else begin
            valid_out <= vld_s1;
        end
    end

    // -----------------------------
    // Stage 1: products and capture
    // -----------------------------
    reg  [2*WORD_WIDTH-1:0] AR_s1;      // a * ratio (full 2W bits)
    reg  [WORD_WIDTH-1:0]   q_s1;       // delayed q
    reg  [WORD_WIDTH-1:0]   q_est_s1;   // floor(a * z_pre / 2^K)  (MULHI via slice)

    // Full product for a*ratio is still needed (AR width 2W)
    wire [2*WORD_WIDTH-1:0] a_mul_ratio = a * ratio;

    // MULHI for a*z_pre.  Keep the full 2W-bit product before taking the
    // upper half; `(a * z_pre) >> K_WIDTH` is evaluated at operand width by
    // Verilog here and can truncate the product before the shift.
    wire [2*WORD_WIDTH-1:0] a_mul_zpre = a * z_pre;
    wire [WORD_WIDTH-1:0]   q_est_next = a_mul_zpre[2*WORD_WIDTH-1:K_WIDTH];

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            AR_s1     <= {2*WORD_WIDTH{1'b0}};
            q_s1      <= {WORD_WIDTH{1'b0}};
            q_est_s1  <= {WORD_WIDTH{1'b0}};
        end else if (valid_in) begin
            AR_s1     <= a_mul_ratio;
            q_s1      <= q;
            q_est_s1  <= q_est_next; // uses MULHI via slice
        end
    end

    // -----------------------------
    // Stage 2: reduction (single +q and single -q only)
    // -----------------------------
    wire [2*WORD_WIDTH-1:0] qmul = q_est_s1 * q_s1;

    // signed 2W+1 subtraction to detect negativity
    wire [2*WORD_WIDTH:0]   T = {1'b0, AR_s1} - {1'b0, qmul};

    // if negative, add q once
    wire [2*WORD_WIDTH:0]   U0 = T[2*WORD_WIDTH]
                                 ? (T + { {(2*WORD_WIDTH+1-WORD_WIDTH){1'b0}}, q_s1 })
                                 : T;

    // subtract q once if still >= q
    wire [2*WORD_WIDTH:0]   U1_try = U0 - { {(2*WORD_WIDTH+1-WORD_WIDTH){1'b0}}, q_s1 };
    wire                    ge0_1  = ~U1_try[2*WORD_WIDTH];
    wire [2*WORD_WIDTH:0]   U1     = ge0_1 ? U1_try : U0;

    // (Removed) optional second subtract (-q) ? proven unnecessary for Shoup K=W

    // Output register aligned to valid_out
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            result <= {WORD_WIDTH{1'b0}};
        end else begin
            // latch Stage-2 comb result when Stage-1 valid is high
            if (vld_s1) result <= U1[WORD_WIDTH-1:0];
            // present final result exactly when valid_out asserts (next cycle)
        end
    end

endmodule

