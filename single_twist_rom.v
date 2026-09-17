// -----------------------------------------------------------------------------
// single_twist_rom.v  (Verilog-2001)
// v1.3p  · Adds optional synchronous address pipeline for timing closure
// -----------------------------------------------------------------------------
// - Wide vectors (even/z2/z2pre/odd) come from twiddle16_rom_multibank instances.
// - Each ROM has 1-cycle registered outputs.
// - This module can optionally register the address tuple {bank_id,tile_r,tile_c}
//   to break long combinational paths into ROMs.
//   * ADDR_PIPE_STAGES=1 (default): effective latency = 1(ROM) + 1 = 2 cycles
//   * ADDR_PIPE_STAGES=0          : effective latency = 1(ROM)     = 1 cycle
//   -> Set single_on_fly_wrap.ROM_OUT_LATENCY accordingly.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module single_twist_rom #(
    parameter integer WORD_WIDTH       = `NTT_WORD_WIDTH, // e.g., 60
    parameter integer LANES            = `LANES,          // 16

    // Optional synchronous address pipeline stages (0 or 1)
    parameter integer ADDR_PIPE_STAGES = 1,

    // File paths (can be overridden at instantiation)
    parameter         INIT_EVEN_HEX    = "single_vector/single_even_rom_combined.hex",
    parameter         INIT_Z2_HEX      = "single_vector/single_z2_rom_combined.hex",
    parameter         INIT_Z2PRE_HEX   = "single_vector/single_z2pre_rom_combined.hex",
    parameter         INIT_ODD_HEX     = "single_vector/single_odd_rom_combined.hex"
)(
    input  wire                         clk,
    input  wire                         rst_n,   // used as synchronous reset here

    // Level/bank selection (5 bits for 30 levels)
    input  wire [`NTT_BANK_BITS-1:0]    bank_id,   // 0..29

    // Tile indices (col-first typical): Tc:0..15, Tr:0..15
    input  wire [3:0]                   tile_r,    // 0..15
    input  wire [3:0]                   tile_c,    // 0..15

    // Wide outputs (ROM-registered; plus optional addr pipe in this module)
    output wire [WORD_WIDTH*LANES-1:0]  even_vec,
    output wire [WORD_WIDTH*LANES-1:0]  z2_vec,
    output wire [WORD_WIDTH*LANES-1:0]  z2pre_vec,
    output wire [WORD_WIDTH*LANES-1:0]  odd_vec
);
    // ----------------------------------------------
    // Address tuple pipeline (synchronous reset)
    // ----------------------------------------------
    // We keep pure concatenation (no mult/div), just an optional 1-stage FF.
    // Widths:
    //   EVEN : {level[4:0], tile_r[3:0]}                  -> 9 bits
    //   Z2   : {level[4:0], tile_c[2:0]} (Tc mod 8)       -> 8 bits
    //   Z2PRE: {level[4:0], tile_c[2:0]}                  -> 8 bits
    //   ODD  : {level[4:0], tile_c[3:0], tile_r[3:0]}     -> 13 bits

    // Raw inputs
    wire [4:0] bank_id_w = bank_id[4:0];
    wire [3:0] tile_r_w  = tile_r[3:0];
    wire [3:0] tile_c_w  = tile_c[3:0];

    // Registered (optional)
    reg  [4:0] bank_id_q;
    reg  [3:0] tile_r_q, tile_c_q;

generate
if (ADDR_PIPE_STAGES != 0) begin : G_ADDR_PIPE
    // Use synchronous reset to avoid recovery time paths on control pins.
    always @(posedge clk) begin
        if (!rst_n) begin
            bank_id_q <= 5'd0;
            tile_r_q  <= 4'd0;
            tile_c_q  <= 4'd0;
        end else begin
            bank_id_q <= bank_id_w;
            tile_r_q  <= tile_r_w;
            tile_c_q  <= tile_c_w;
        end
    end
end else begin : G_ADDR_NOPIPE
    // Tie-through when no pipeline stage is requested
    always @(posedge clk) begin
        // still synth-friendly; ensures defined regs without async control
        bank_id_q <= bank_id_w;
        tile_r_q  <= tile_r_w;
        tile_c_q  <= tile_c_w;
    end
end
endgenerate

    // Compose bank selects from the (maybe-registered) tuple
    wire [8:0]  even_bank_sel   = {bank_id_q, tile_r_q};
    wire [7:0]  z2_bank_sel     = {bank_id_q, tile_c_q[2:0]};
    wire [7:0]  z2pre_bank_sel  = {bank_id_q, tile_c_q[2:0]};
    wire [12:0] odd_bank_sel    = {bank_id_q, tile_c_q, tile_r_q};

    // ----------------------------------------------
    // ROM instances (each has 1-cycle registered output)
    // ----------------------------------------------
    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (480), // 30 * 16
        .BANK_BITS  (9),
        .INIT_HEX   (INIT_EVEN_HEX)
    ) U_EVEN (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (even_bank_sel),
        .data_out (even_vec)
    );

    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (240), // 30 * 8
        .BANK_BITS  (8),
        .INIT_HEX   (INIT_Z2_HEX)
    ) U_Z2 (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (z2_bank_sel),
        .data_out (z2_vec)
    );

    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (240), // 30 * 8
        .BANK_BITS  (8),
        .INIT_HEX   (INIT_Z2PRE_HEX)
    ) U_Z2PRE (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (z2pre_bank_sel),
        .data_out (z2pre_vec)
    );

    twiddle16_rom_multibank #(
        .WORD_WIDTH (WORD_WIDTH),
        .BANK_WORDS (16),
        .NUM_BANKS  (7680), // 30 * 16 * 16
        .BANK_BITS  (13),
        .INIT_HEX   (INIT_ODD_HEX)
    ) U_ODD (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (odd_bank_sel),
        .data_out (odd_vec)
    );

    // ----------------------------------------------
    // Trace banner (effective latency hint)
    // ----------------------------------------------
    localparam integer EFFECTIVE_ROM_LATENCY = (ADDR_PIPE_STAGES != 0) ? 2 : 1;

    initial begin
        $display("[single_twist_rom v1.3p] ADDR_PIPE=%0d -> EFFECTIVE_ROM_LATENCY=%0d  (EVEN=%0s Z2=%0s Z2PRE=%0s ODD=%0s)",
                 (ADDR_PIPE_STAGES!=0), EFFECTIVE_ROM_LATENCY,
                 INIT_EVEN_HEX, INIT_Z2_HEX, INIT_Z2PRE_HEX, INIT_ODD_HEX);
        $display("  >> Set single_on_fly_wrap.ROM_OUT_LATENCY to %0d to match.", EFFECTIVE_ROM_LATENCY);
    end

endmodule

`default_nettype wire

