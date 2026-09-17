// -----------------------------------------------------------------------------
// qconst_rom_bank.v  (Verilog-2001) // v1.1 - Added Synthesis Stub
// Per-bank constants ROM: supplies (q, mu, q_width) for a given bank (level).
// - Synthesis guard:
//     * `SYNTHESIS` defined -> lightweight 1-cycle stub (no giant MUX)
//     * else (simulation)    -> $readmemh behavioral ROM
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module qconst_rom_bank #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH,
    parameter integer NUM_BANKS  = `NTT_NUM_BANKS,
    parameter integer BANK_BITS  = `NTT_BANK_BITS,
    parameter         Q_HEX      = `NTT_Q_HEX,
    parameter         MU_HEX     = `NTT_MU_HEX,
    parameter         QW_HEX     = `NTT_Q_WIDTH_HEX
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire [BANK_BITS-1:0]       bank_sel,
    output reg  [WORD_WIDTH-1:0]      q,
    output reg  [2*WORD_WIDTH-1:0]    mu,
    output reg  [5:0]                 q_width
);

`ifdef SYNTHESIS
    // -------------------------------------------------------------------------
    // **SYNTHESIS STUB**
    // This is a lightweight model for synthesis tools.
    // It has the same 1-cycle registered output timing as the simulation model,
    // but without the large memory array to ensure fast synthesis and
    // to prevent the tool from creating inefficient logic.
    // -------------------------------------------------------------------------
    wire filler_bit = ^bank_sel; // A simple logic function of the address

    wire [WORD_WIDTH-1:0]      q_filler       = {WORD_WIDTH{filler_bit}};
    wire [2*WORD_WIDTH-1:0]    mu_filler      = {(2*WORD_WIDTH){filler_bit}};
    wire [5:0]                 q_width_filler = {6{filler_bit}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q       <= {WORD_WIDTH{1'b0}};
            mu      <= {(2*WORD_WIDTH){1'b0}};
            q_width <= 6'd0;
        end else begin
            // The output is registered and depends on bank_sel, mimicking ROM behavior.
            q       <= q_filler;
            mu      <= mu_filler;
            q_width <= q_width_filler;
        end
    end

`else
    // -------------------------------------------------------------------------
    // **SIMULATION MODEL**
    // This model uses $readmemh to load real data for functional verification.
    // It is ignored during synthesis.
    // -------------------------------------------------------------------------
    // synopsys translate_off
    reg [WORD_WIDTH-1:0]      q_rom  [0:NUM_BANKS-1];
    reg [2*WORD_WIDTH-1:0]    mu_rom [0:NUM_BANKS-1];
    reg [7:0]                 qw_rom [0:NUM_BANKS-1]; // store q_width (bits)

    initial begin
        $display("[qconst_rom_bank] Loading Q file   : %0s", Q_HEX);
        $display("[qconst_rom_bank] Loading MU file  : %0s", MU_HEX);
        $display("[qconst_rom_bank] Loading QW file  : %0s", QW_HEX);
        $readmemh(Q_HEX,  q_rom);
        $readmemh(MU_HEX, mu_rom);
        $readmemh(QW_HEX, qw_rom);
    end

    // 1-cycle registered outputs
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q       <= {WORD_WIDTH{1'b0}};
            mu      <= {(2*WORD_WIDTH){1'b0}};
            q_width <= 6'd0;
        end else begin
            q       <= q_rom [bank_sel];
            mu      <= mu_rom[bank_sel];
            q_width <= qw_rom[bank_sel][5:0]; // use lower 6 bits
        end
    end

    // Simulation-only guards
    always @(posedge clk) begin
        if (bank_sel >= NUM_BANKS) begin
            $display("[qconst_rom_bank][WARN @%0t] bank_sel=%0d out of range (NUM_BANKS=%0d)",
                     $time, bank_sel, NUM_BANKS);
        end
        if (qw_rom[bank_sel][5:0] == 0 || qw_rom[bank_sel][5:0] > WORD_WIDTH) begin
            $display("[qconst_rom_bank][WARN @%0t] q_width(%0d) invalid for bank %0d (WORD_WIDTH=%0d)",
                     $time, qw_rom[bank_sel][5:0], bank_sel, WORD_WIDTH);
        end
    end
    // synopsys translate_on
`endif

endmodule

`default_nettype wire
