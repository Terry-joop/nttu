// ============================================================================
// ntt16_buscore · Version v2.3 (CORE, Barrett-based, no mid-slice, single valid)
// Verilog-2001, 16-lane 16-pt DIF NTT core
//
// Changes vs v2.2:
//  - Plumb dynamic q_width from qconst_rom_bank into every butterfly.
//  - Rename twiddle port to .w (standard-domain twiddle), no Montgomery naming.
//  - Still: no inter-stage slices; direct wire stage-to-stage; single valid per stage.
//  - Option-A: upstream must issue 1-cycle prime bubble on bank switch.
//
// Twiddles/Q/MU ROMs are 1-cycle registered; twiddles are standard-domain.
// Stage-3 uses LAST_STAGE=1 (twiddle=1) to bypass the multiplier.
// Output order: DIF bit-reversed.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module ntt16_buscore #(
    parameter integer WORD_WIDTH  = `NTT_WORD_WIDTH,   // e.g., 60
    parameter integer BARRETT_LAT = `NTT_BARRETT_LAT   // e.g., 4
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Frame interface (no ready)
    input  wire                          s_valid,
    input  wire [WORD_WIDTH*16-1:0]      s_data,
    input  wire [`NTT_BANK_BITS-1:0]     bank_id,

    output wire                          m_valid,
    output wire [WORD_WIDTH*16-1:0]      m_data
);
    // No internal bubble (Option A)
    wire s_valid_s0 = s_valid;

    // Unpack input lanes
    wire [WORD_WIDTH-1:0] x0  = s_data[WORD_WIDTH*0  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x1  = s_data[WORD_WIDTH*1  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x2  = s_data[WORD_WIDTH*2  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x3  = s_data[WORD_WIDTH*3  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x4  = s_data[WORD_WIDTH*4  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x5  = s_data[WORD_WIDTH*5  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x6  = s_data[WORD_WIDTH*6  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x7  = s_data[WORD_WIDTH*7  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x8  = s_data[WORD_WIDTH*8  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x9  = s_data[WORD_WIDTH*9  +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x10 = s_data[WORD_WIDTH*10 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x11 = s_data[WORD_WIDTH*11 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x12 = s_data[WORD_WIDTH*12 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x13 = s_data[WORD_WIDTH*13 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x14 = s_data[WORD_WIDTH*14 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] x15 = s_data[WORD_WIDTH*15 +: WORD_WIDTH];

    // ---------------- Q / MU / Q_WIDTH (banked ROM, 1-cycle) ----------------
    wire [WORD_WIDTH-1:0]   q_wire;
    wire [2*WORD_WIDTH-1:0] mu_wire;
    wire [5:0]              q_width_wire;  // dynamic bit-length of q

    // NOTE: your qconst_rom_bank should provide q_width (by file or by bitlen()).
    qconst_rom_bank #(
        .WORD_WIDTH (WORD_WIDTH),
        .NUM_BANKS  (`NTT_NUM_BANKS),
        .BANK_BITS  (`NTT_BANK_BITS),
        .Q_HEX      (`NTT_Q_HEX),
        .MU_HEX     (`NTT_MU_HEX),
        .QW_HEX     (`NTT_Q_WIDTH_HEX)
    ) QROM (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (bank_id),
        .q        (q_wire),
        .mu       (mu_wire),
        .q_width  (q_width_wire)
    );

    // =========================================================================
    // Stage-0 (distance=8), w^0..w^7
    // =========================================================================
    wire [WORD_WIDTH*`NTT_S0_BANK_WORDS-1:0] s0_tw_vec8;
    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (`NTT_S0_BANK_WORDS),
        .NUM_BANKS  (`NTT_NUM_BANKS),
        .BANK_BITS  (`NTT_BANK_BITS),
        .INIT_HEX   (`NTT_TW_S0_HEX)
    ) TW_S0 (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (bank_id),
        .data_out (s0_tw_vec8)
    );

    wire [WORD_WIDTH-1:0] w0_0 = s0_tw_vec8[WORD_WIDTH*0 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w0_1 = s0_tw_vec8[WORD_WIDTH*1 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w0_2 = s0_tw_vec8[WORD_WIDTH*2 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w0_3 = s0_tw_vec8[WORD_WIDTH*3 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w0_4 = s0_tw_vec8[WORD_WIDTH*4 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w0_5 = s0_tw_vec8[WORD_WIDTH*5 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w0_6 = s0_tw_vec8[WORD_WIDTH*6 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w0_7 = s0_tw_vec8[WORD_WIDTH*7 +: WORD_WIDTH];

    wire v0;
    wire [WORD_WIDTH-1:0] y0,y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12,y13,y14,y15;

    // Use BF0's valid as the stage valid (all BFs are lockstep)
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF0 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x0), .B_in(x8), .w(w0_0), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(v0), .A_out(y0), .B_out(y8)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF1 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x1), .B_in(x9), .w(w0_1), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(y1), .B_out(y9)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF2 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x2), .B_in(x10), .w(w0_2), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(y2), .B_out(y10)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF3 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x3), .B_in(x11), .w(w0_3), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(y3), .B_out(y11)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF4 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x4), .B_in(x12), .w(w0_4), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(y4), .B_out(y12)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF5 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x5), .B_in(x13), .w(w0_5), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(y5), .B_out(y13)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF6 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x6), .B_in(x14), .w(w0_6), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(y6), .B_out(y14)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S0_BF7 (
        .clk(clk), .rst_n(rst_n), .valid_in(s_valid_s0),
        .A_in(x7), .B_in(x15), .w(w0_7), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(y7), .B_out(y15)
    );

    // =========================================================================
    // Stage-1 (distance=4), {w^0,w^2,w^4,w^6}
    // =========================================================================
    wire [WORD_WIDTH*`NTT_S1_BANK_WORDS-1:0] s1_tw_vec4;
    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (`NTT_S1_BANK_WORDS),
        .NUM_BANKS  (`NTT_NUM_BANKS),
        .BANK_BITS  (`NTT_BANK_BITS),
        .INIT_HEX   (`NTT_TW_S1_HEX)
    ) TW_S1 (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (bank_id),
        .data_out (s1_tw_vec4)
    );

    wire [WORD_WIDTH-1:0] w1_0 = s1_tw_vec4[WORD_WIDTH*0 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w1_1 = s1_tw_vec4[WORD_WIDTH*1 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w1_2 = s1_tw_vec4[WORD_WIDTH*2 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w1_3 = s1_tw_vec4[WORD_WIDTH*3 +: WORD_WIDTH];

    wire v1;
    wire [WORD_WIDTH-1:0] z0,z1,z2,z3,z4,z5,z6,z7,z8,z9,z10,z11,z12,z13,z14,z15;

    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF0 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y0), .B_in(y4), .w(w1_0), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(v1), .A_out(z0), .B_out(z4)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF1 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y1), .B_in(y5), .w(w1_1), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(z1), .B_out(z5)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF2 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y2), .B_in(y6), .w(w1_2), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(z2), .B_out(z6)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF3 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y3), .B_in(y7), .w(w1_3), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(z3), .B_out(z7)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF4 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y8), .B_in(y12), .w(w1_0), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(z8), .B_out(z12)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF5 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y9), .B_in(y13), .w(w1_1), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(z9), .B_out(z13)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF6 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y10), .B_in(y14), .w(w1_2), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(z10), .B_out(z14)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S1_BF7 (
        .clk(clk), .rst_n(rst_n), .valid_in(v0),
        .A_in(y11), .B_in(y15), .w(w1_3), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(z11), .B_out(z15)
    );

    // =========================================================================
    // Stage-2 (distance=2), {w^0,w^4}
    // =========================================================================
    wire [WORD_WIDTH*`NTT_S2_BANK_WORDS-1:0] s2_tw_vec2;
    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (`NTT_S2_BANK_WORDS),
        .NUM_BANKS  (`NTT_NUM_BANKS),
        .BANK_BITS  (`NTT_BANK_BITS),
        .INIT_HEX   (`NTT_TW_S2_HEX)
    ) TW_S2 (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (bank_id),
        .data_out (s2_tw_vec2)
    );

    wire [WORD_WIDTH-1:0] w2_0 = s2_tw_vec2[WORD_WIDTH*0 +: WORD_WIDTH];
    wire [WORD_WIDTH-1:0] w2_1 = s2_tw_vec2[WORD_WIDTH*1 +: WORD_WIDTH];

    wire v2;
    wire [WORD_WIDTH-1:0] w0_,w1_,w2_,w3_,w4_,w5_,w6_,w7_,w8_,w9_,w10_,w11_,w12_,w13_,w14_,w15_;

    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF0 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z0), .B_in(z2), .w(w2_0), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(v2), .A_out(w0_), .B_out(w2_)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF1 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z1), .B_in(z3), .w(w2_1), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(w1_), .B_out(w3_)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF2 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z4), .B_in(z6), .w(w2_0), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(w4_), .B_out(w6_)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF3 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z5), .B_in(z7), .w(w2_1), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(w5_), .B_out(w7_)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF4 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z8), .B_in(z10), .w(w2_0), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(w8_), .B_out(w10_)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF5 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z9), .B_in(z11), .w(w2_1), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(w9_), .B_out(w11_)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF6 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z12), .B_in(z14), .w(w2_0), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(w12_), .B_out(w14_)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b0)) S2_BF7 (
        .clk(clk), .rst_n(rst_n), .valid_in(v1),
        .A_in(z13), .B_in(z15), .w(w2_1), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(w13_), .B_out(w15_)
    );

    // =========================================================================
    // Stage-3 (distance=1), LAST_STAGE=1 (twiddle=1, bypass)
    // =========================================================================
    wire v3;
    wire [WORD_WIDTH-1:0] o0,o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14,o15;

    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF0 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w0_), .B_in(w1_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(v3), .A_out(o0), .B_out(o1)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF1 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w2_), .B_in(w3_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(o2), .B_out(o3)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF2 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w4_), .B_in(w5_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(o4), .B_out(o5)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF3 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w6_), .B_in(w7_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(o6), .B_out(o7)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF4 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w8_), .B_in(w9_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(o8), .B_out(o9)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF5 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w10_), .B_in(w11_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(o10), .B_out(o11)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF6 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w12_), .B_in(w13_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(o12), .B_out(o13)
    );
    radix2_butterfly_dif #(.WORD_WIDTH(WORD_WIDTH), .BARRETT_LAT(BARRETT_LAT), .LAST_STAGE(1'b1)) S3_BF7 (
        .clk(clk), .rst_n(rst_n), .valid_in(v2),
        .A_in(w14_), .B_in(w15_), .w({WORD_WIDTH{1'b0}}), .q(q_wire), .mu(mu_wire), .q_width(q_width_wire),
        .valid_out(/*unused*/), .A_out(o14), .B_out(o15)
    );

    // Outputs
    assign m_valid = v3;
    assign m_data[WORD_WIDTH*0  +: WORD_WIDTH] = o0;
    assign m_data[WORD_WIDTH*1  +: WORD_WIDTH] = o1;
    assign m_data[WORD_WIDTH*2  +: WORD_WIDTH] = o2;
    assign m_data[WORD_WIDTH*3  +: WORD_WIDTH] = o3;
    assign m_data[WORD_WIDTH*4  +: WORD_WIDTH] = o4;
    assign m_data[WORD_WIDTH*5  +: WORD_WIDTH] = o5;
    assign m_data[WORD_WIDTH*6  +: WORD_WIDTH] = o6;
    assign m_data[WORD_WIDTH*7  +: WORD_WIDTH] = o7;
    assign m_data[WORD_WIDTH*8  +: WORD_WIDTH] = o8;
    assign m_data[WORD_WIDTH*9  +: WORD_WIDTH] = o9;
    assign m_data[WORD_WIDTH*10 +: WORD_WIDTH] = o10;
    assign m_data[WORD_WIDTH*11 +: WORD_WIDTH] = o11;
    assign m_data[WORD_WIDTH*12 +: WORD_WIDTH] = o12;
    assign m_data[WORD_WIDTH*13 +: WORD_WIDTH] = o13;
    assign m_data[WORD_WIDTH*14 +: WORD_WIDTH] = o14;
    assign m_data[WORD_WIDTH*15 +: WORD_WIDTH] = o15;

endmodule

`default_nettype wire

