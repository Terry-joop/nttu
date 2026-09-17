// -----------------------------------------------------------------------------
// twiddle16_rom_multibank.v  (Verilog-2001)
// - Multi-bank ROM, wide-bus gather, 1-cycle registered output
// - Synthesis guard:
//     * `SYNTHESIS` defined -> lightweight 1-cycle stub (no giant MUX)
//     * else (simulation)    -> $readmemh behavioral ROM
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module twiddle16_rom_multibank #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH,
    parameter integer BANK_WORDS = 32,                 // words per bank
    parameter integer NUM_BANKS  = 1,
    parameter integer BANK_BITS  = 1,                  // width of bank_sel
    parameter         INIT_HEX   = "inputvalue_levels.hex"
)(
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire [BANK_BITS-1:0]             bank_sel,    // selects which bank
    output reg  [WORD_WIDTH*BANK_WORDS-1:0] data_out     // concatenated bank words
);

`ifdef SYNTHESIS
    // -------------------------------------------------------------------------
    // **SYNTHESIS STUB**
    // - 1-cycle registered output? ??? ??? ??? ??? ??.
    // - ??? ??? ?? ? ??? ????, ?? ???? ?? ??? ??.
    // - ?? MUX ??(?? ??? ??? ??) ??? ??.
    // -------------------------------------------------------------------------
    wire filler_bit = ^bank_sel;  // ?? ?? ??? ??(??? XOR)
    wire [WORD_WIDTH*BANK_WORDS-1:0] filler_bus = { (WORD_WIDTH*BANK_WORDS){filler_bit} };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) data_out <= {WORD_WIDTH*BANK_WORDS{1'b0}};
        else        data_out <= filler_bus;  // 1-cycle reg
    end

`else
    // -------------------------------------------------------------------------
    // **SIMULATION MODEL**
    // - ??? $readmemh ?? ?? ?? ROM
    // - bank_sel * BANK_WORDS + i ??? ?? ??? ?? (?? ? ???)
    // -------------------------------------------------------------------------
    // synopsys translate_off
    reg [WORD_WIDTH-1:0] rom [0:BANK_WORDS*NUM_BANKS-1];

    initial begin
        $display("[twiddle16_rom] INIT_HEX = %0s", INIT_HEX);
        $readmemh(INIT_HEX, rom);
    end

    wire [WORD_WIDTH*BANK_WORDS-1:0] data_comb;
    genvar gi;
    generate
      for (gi = 0; gi < BANK_WORDS; gi = gi + 1) begin : G_PACK
        assign data_comb[WORD_WIDTH*gi +: WORD_WIDTH]
                       = rom[bank_sel*BANK_WORDS + gi];
      end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) data_out <= {WORD_WIDTH*BANK_WORDS{1'b0}};
        else        data_out <= data_comb;  // 1-cycle reg
    end
    // synopsys translate_on
`endif

endmodule

`default_nettype wire

