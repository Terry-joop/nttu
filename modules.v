// -----------------------------------------------------------------------------
// modular_adder.v  (Verilog-2001)
//
// Comparator-Free Modular Adder
//
// Description:
// Computes (A + B) mod q efficiently without using a slow comparator.
//
// Algorithm:
// It leverages the carry-out/borrow bit of a wider subtraction.
//  1. `sum_ext = A + B` (W+1 bits)
//  2. `tmp_ext = sum_ext - q`
//  3. If `tmp_ext` results in a borrow (MSB is 1), it means `sum_ext < q`.
//     In this case, the correct result is `sum_ext`.
//  4. If there is no borrow, it means `sum_ext >= q`.
//     The correct result is `tmp_ext` (which is sum_ext - q).
// This is faster and uses fewer resources than a traditional `if (sum >= q)` check.
// -----------------------------------------------------------------------------
`include "ntt_params.vh"
module modular_adder #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH
)(
    input  [WORD_WIDTH-1:0] A,
    input  [WORD_WIDTH-1:0] B,
    input  [WORD_WIDTH-1:0] q,
    output [WORD_WIDTH-1:0] result
);
    wire [WORD_WIDTH:0] sum_ext = {1'b0, A} + {1'b0, B};
    wire [WORD_WIDTH:0] tmp_ext = sum_ext - {1'b0, q};

    assign result = tmp_ext[WORD_WIDTH] ? sum_ext[WORD_WIDTH-1:0] : tmp_ext[WORD_WIDTH-1:0];
endmodule

// -----------------------------------------------------------------------------
// modular_subtractor.v  (Verilog-2001)
//
// Borrow-Based Modular Subtractor
//
// Description:
// Computes (A - B) mod q efficiently.
//
// Algorithm:
//  1. `diff_ext = A - B` (W+1 bits)
//  2. If `diff_ext` results in a borrow (MSB is 1), it means `A < B` and the
//     result is negative. To make it positive, we add `q`.
//  3. If there is no borrow, `A >= B`, and the result is already correct.
// -----------------------------------------------------------------------------
module modular_subtractor #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH
)(
    input  [WORD_WIDTH-1:0] A,
    input  [WORD_WIDTH-1:0] B,
    input  [WORD_WIDTH-1:0] q,
    output [WORD_WIDTH-1:0] result
);
    wire [WORD_WIDTH:0] diff_ext = {1'b0, A} - {1'b0, B};
    wire [WORD_WIDTH:0] addq_ext = diff_ext + {1'b0, q};

    assign result = diff_ext[WORD_WIDTH] ? addq_ext[WORD_WIDTH-1:0] : diff_ext[WORD_WIDTH-1:0];
endmodule

