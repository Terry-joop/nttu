// -----------------------------------------------------------------------------
// double_twist_rom.v  (Verilog-2001)
// Version: v1.3q
//  - Registered ROM outputs (1c) + optional address pipeline (0/1c)
//  - File-path parametric; T0 can reuse single_even ROM
//  - Interface & style matched to single_twist_rom
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module double_twist_rom #(
    parameter integer WORD_WIDTH       = `NTT_WORD_WIDTH, // e.g., 60
    parameter integer LANES            = `LANES,          // 16
    // Optional synchronous address pipeline stages (0 or 1)
    parameter integer ADDR_PIPE_STAGES = 1,
    // File paths (override at instantiation if needed)
    // NOTE: T0 is identical to single_even; you can reuse that file.
    parameter         INIT_T0_HEX      = "single_vector/single_even_rom_combined.hex",
    parameter         INIT_T1_HEX      = "double_vector/double_t1_odd.hex",
    parameter         INIT_H2_HEX      = "double_vector/double_h2.hex",
    parameter         INIT_H2PRE_HEX   = "double_vector/double_h2pre.hex"
)(
    input  wire                         clk,
    input  wire                         rst_n,      // synchronous reset
    input  wire [`NTT_BANK_BITS-1:0]    bank_id,    // 0..29
    input  wire [3:0]                   tile_r,     // kept for symmetry (unused)
    input  wire [3:0]                   tile_c,     // 0..15

    output wire [LANES*WORD_WIDTH-1:0]  t0_even_vec,
    output wire [LANES*WORD_WIDTH-1:0]  h2_vec,
    output wire [LANES*WORD_WIDTH-1:0]  h2pre_vec,
    output wire [LANES*WORD_WIDTH-1:0]  t1_odd_vec
);
    // -------- Address tuple pipeline ----------------------------------------
    wire [4:0] bank_id_w = bank_id[4:0];
    wire [3:0] tile_c_w  = tile_c[3:0];
    wire [3:0] tile_r_w  = tile_r[3:0]; // unused, for interface symmetry

    reg  [4:0] bank_id_q;
    reg  [3:0] tile_c_q, tile_r_q;

generate
if (ADDR_PIPE_STAGES != 0) begin : G_ADDR_PIPE
    always @(posedge clk) begin
        if (!rst_n) begin
            bank_id_q <= 5'd0;
            tile_c_q  <= 4'd0;
            tile_r_q  <= 4'd0;
        end else begin
            bank_id_q <= bank_id_w;
            tile_c_q  <= tile_c_w;
            tile_r_q  <= tile_r_w;
        end
    end
end else begin : G_ADDR_NOPIPE
    always @(posedge clk) begin
        bank_id_q <= bank_id_w;
        tile_c_q  <= tile_c_w;
        tile_r_q  <= tile_r_w;
    end
end
endgenerate

    // Bank selects (registered)
    wire [8:0] t0_bank_sel    = {bank_id_q, tile_c_q}; // 30*16 banks
    wire [8:0] t1_bank_sel    = {bank_id_q, tile_c_q}; // 30*16 banks
    wire [4:0] h2_bank_sel    =  bank_id_q;            // 30 banks
    wire [4:0] h2pre_bank_sel =  bank_id_q;            // 30 banks

    // -------- ROM instances (1-cycle registered outputs) --------------------
    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (480), // 30 * 16
        .BANK_BITS  (9),
        .INIT_HEX   (INIT_T0_HEX)
    ) U_T0 (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (t0_bank_sel),
        .data_out (t0_even_vec)
    );

    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (480), // 30 * 16
        .BANK_BITS  (9),
        .INIT_HEX   (INIT_T1_HEX)
    ) U_T1 (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (t1_bank_sel),
        .data_out (t1_odd_vec)
    );

    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (30),  // 30
        .BANK_BITS  (5),
        .INIT_HEX   (INIT_H2_HEX)
    ) U_H2 (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (h2_bank_sel),
        .data_out (h2_vec)
    );

    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (30),  // 30
        .BANK_BITS  (5),
        .INIT_HEX   (INIT_H2PRE_HEX)
    ) U_H2PRE (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (h2pre_bank_sel),
        .data_out (h2pre_vec)
    );

    // Trace banner
    localparam integer EFFECTIVE_ROM_LATENCY = (ADDR_PIPE_STAGES != 0) ? 2 : 1;
    initial begin
        $display("[double_twist_rom v1.3q] ADDR_PIPE=%0d -> EFFECTIVE_ROM_LATENCY=%0d (T0=%0s T1=%0s H2=%0s H2PRE=%0s)",
                 (ADDR_PIPE_STAGES!=0), EFFECTIVE_ROM_LATENCY,
                 INIT_T0_HEX, INIT_T1_HEX, INIT_H2_HEX, INIT_H2PRE_HEX);
    end
endmodule

`default_nettype wire

