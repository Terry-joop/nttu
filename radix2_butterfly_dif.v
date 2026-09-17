// -----------------------------------------------------------------------------
// radix2_butterfly_dif.v  (v2.1 · Barrett-based, WORD_WIDTH-parametric)
// Verilog-2001 only.
// Twiddle `w` is in STANDARD domain (not Montgomery).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module radix2_butterfly_dif #(
    parameter integer WORD_WIDTH  = `NTT_WORD_WIDTH,     // e.g., 60
    parameter integer BARRETT_LAT = `NTT_BARRETT_LAT,    // e.g., 4
    parameter         LAST_STAGE  = 1'b0                 // when 1, multiplier is bypassed (w?1)
)(
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire                    valid_in,
    input  wire [WORD_WIDTH-1:0]   A_in,
    input  wire [WORD_WIDTH-1:0]   B_in,
    // Standard-domain twiddle (not Montgomery)
    input  wire [WORD_WIDTH-1:0]   w,

    input  wire [WORD_WIDTH-1:0]   q,
    // Barrett constant: mu = floor(2^(2*q_width)/q), padded to 2*WORD_WIDTH bits
    input  wire [2*WORD_WIDTH-1:0] mu,
    input  wire [5:0]              q_width,

    output wire                    valid_out,
    output wire [WORD_WIDTH-1:0]   A_out,
    output wire [WORD_WIDTH-1:0]   B_out
);
    // 0) Modular sum/diff (combinational)
    wire [WORD_WIDTH-1:0] add_mod, sub_mod;

    modular_adder #(
        .WORD_WIDTH (WORD_WIDTH)
    ) u_add (
        .A      (A_in),
        .B      (B_in),
        .q      (q),
        .result (add_mod)
    );

    modular_subtractor #(
        .WORD_WIDTH (WORD_WIDTH)
    ) u_sub (
        .A      (A_in),
        .B      (B_in),
        .q      (q),
        .result (sub_mod)
    );

    // 1) Valid pipeline (aligns to BARRETT_LAT)
    reg [BARRETT_LAT-1:0] vld_pipe;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vld_pipe <= {BARRETT_LAT{1'b0}};
        else         vld_pipe <= {vld_pipe[BARRETT_LAT-2:0], valid_in};
    end

    // 2) Sum path alignment
    reg [WORD_WIDTH-1:0] add_pipe [0:BARRETT_LAT-1];
    integer pi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (pi=0; pi<BARRETT_LAT; pi=pi+1) add_pipe[pi] <= {WORD_WIDTH{1'b0}};
        end else begin
            if (valid_in) add_pipe[0] <= add_mod;
            for (pi=1; pi<BARRETT_LAT; pi=pi+1)
                if (vld_pipe[pi-1]) add_pipe[pi] <= add_pipe[pi-1];
        end
    end

    // diff path (needed for LAST_STAGE bypass)
    reg [WORD_WIDTH-1:0] sub_pipe [0:BARRETT_LAT-1];
    integer pj;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (pj=0; pj<BARRETT_LAT; pj=pj+1) sub_pipe[pj] <= {WORD_WIDTH{1'b0}};
        end else begin
            if (valid_in) sub_pipe[0] <= sub_mod;
            for (pj=1; pj<BARRETT_LAT; pj=pj+1)
                if (vld_pipe[pj-1]) sub_pipe[pj] <= sub_pipe[pj-1];
        end
    end

    // 3) Multiplier (Barrett) or bypass
    wire                  mul_vld;
    wire [WORD_WIDTH-1:0] mul_res;

    generate
      if (LAST_STAGE) begin : G_BYPASS
        // w is ignored; B_out = diff delayed to match BARRETT_LAT
        assign mul_vld = vld_pipe[BARRETT_LAT-1];
        assign mul_res = sub_pipe[BARRETT_LAT-1];
      end else begin : G_BARRETT
        barrett_mult #(
            .WORD_WIDTH (WORD_WIDTH)
        ) u_mul (
            .clk       (clk),
            .rst_n     (rst_n),
            .valid_in  (valid_in),
            .A         (sub_mod),
            .B         (w),        // twiddle as-is (standard domain)
            .q         (q),
            .mu        (mu),
            .q_width   (q_width),
            .valid_out (mul_vld),
            .result    (mul_res)
        );
      end
    endgenerate

    // 4) Outputs
    assign A_out     = add_pipe[BARRETT_LAT-1];  // sum aligned
    assign B_out     = mul_res;                  // (diff*w) mod q, or diff for LAST_STAGE
    assign valid_out = mul_vld;

endmodule

`default_nettype wire

