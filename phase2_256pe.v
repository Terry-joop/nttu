// -----------------------------------------------------------------------------
// phase2_256pe.v  (Verilog-2001)
// Version : v1.3-FIXED (Corrected port names and params for synthesis)
// Author  : (you + assistant)
// -----------------------------------------------------------------------------
// Changes vs v1.2s-TC:
//   - Fixed U_NTT_A/B and U_TRP port names to match module definitions (s_*/m_*).
//   - Corrected U_TRP signal connections to use internal wires (v_a, d_a, etc.).
//   - Added missing BARRETT_LAT parameter to U_NTT_A and U_NTT_B instances.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module phase2_256pe #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH, // e.g., 60
    parameter integer LANES      = `LANES           // 16
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // Tile coordinates (match Phase-1 top-level convention)
    input  wire [`NTT_BANK_BITS-1:0]    bank_id,       // 0..NUM_LEVELS-1
    input  wire [3:0]                   tile_r,        // 0..15 (unused by DoF)
    input  wire [3:0]                   tile_c,        // 0..15 (used as initial C sync)

    // Stream in/out
    input  wire                         in_valid,
    input  wire [LANES*WORD_WIDTH-1:0]  in_data_vec,
    output wire                         out_valid,
    output wire [LANES*WORD_WIDTH-1:0]  out_data_vec
);
    // ----------------- Locals -------------------------------------------------
    localparam integer PACK = LANES*WORD_WIDTH;

    // Stage wires
    wire                 v_a;
    wire [PACK-1:0]      d_a;
    wire                 v_trp;
    wire [PACK-1:0]      d_trp;
    wire                 v_dof;
    wire [PACK-1:0]      d_dof;

    // ----------------- Stage A: 16-pt NTT ------------------------------------
    ntt16_buscore #(
        .WORD_WIDTH (WORD_WIDTH),
        .BARRETT_LAT (`NTT_BARRETT_LAT)
    ) U_NTT_A (
        .clk     (clk),
        .rst_n   (rst_n),
        .bank_id (bank_id),
        .s_valid (in_valid),
        .s_data  (in_data_vec),
        .m_valid (v_a),
        .m_data  (d_a)
    );

    // ----------------- Stage B: 16x16 bus transpose --------------------------
    transpose16x16_bus #(
        .WORD_WIDTH (WORD_WIDTH)
    ) U_TRP (
        .clk     (clk),
        .rst_n   (rst_n),
        .s_valid (v_a),
        .s_data  (d_a),
        .m_valid (v_trp),
        .m_data  (d_trp)
    );

    // ----------------- TileC cursor (sync to TRP stream) ---------------------
    // Counts only when TRP produces a valid row. Boundary at pos==15.
    reg [`NTT_BANK_BITS-1:0] bank_q;
    reg [3:0]                tilec_cur;
    reg [4:0]                pos_in_tile; // 0..15 (5-bit to be safe)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bank_q      <= {`NTT_BANK_BITS{1'b0}};
            tilec_cur   <= 4'd0;
            pos_in_tile <= 5'd0;
        end else begin
            // re-sync on bank change (and use input tile_c as starting C)
            if (bank_q != bank_id) begin
                bank_q      <= bank_id;
                tilec_cur   <= tile_c; // initial C provided by top/system
                pos_in_tile <= 5'd0;
            end else begin
                // advance on TRP valid rows only
                if (v_trp) begin
                    if (pos_in_tile == 5'd15) begin
                        pos_in_tile <= 5'd0;
                        tilec_cur   <= (tilec_cur == 4'd15) ? 4'd0 : (tilec_cur + 4'd1);
                    end else begin
                        pos_in_tile <= pos_in_tile + 5'd1;
                    end
                end
            end
        end
    end

    // ----------------- Stage C: Double OF-Twist ------------------------------
    // Wrapper is the version you attached: (clk,rst_n, bank_id, tile_c, in_valid,...)
    // No external 'start' needed (FASTSTART inside wrapper).
    double_on_fly_wrap #(
        .WORD_WIDTH (WORD_WIDTH),
        .LANES      (LANES),
        .LEVEL_BITS (5),   // keep for compatibility if present in wrapper
        .TILEC_BITS (4)
    ) U_DOF (
        .clk          (clk),
        .rst_n        (rst_n),
        .bank_id      (bank_id),   // NOTE: your wrapper uses .bank_id
        .tile_c       (tilec_cur), // <-- use internal cursor (not raw input)
        .in_valid     (v_trp),
        .in_data_vec  (d_trp),
        .out_valid    (v_dof),
        .out_data_vec (d_dof)
    );

    // ----------------- Stage D: 16-pt NTT ------------------------------------
    ntt16_buscore #(
        .WORD_WIDTH (WORD_WIDTH),
        .BARRETT_LAT (`NTT_BARRETT_LAT)
    ) U_NTT_B (
        .clk     (clk),
        .rst_n   (rst_n),
        .bank_id (bank_id),
        .s_valid (v_dof),
        .s_data  (d_dof),
        .m_valid (out_valid),
        .m_data  (out_data_vec)
    );

    initial begin
        $display("[phase2_256pe v1.3-FIXED] NTT16_A -> TRP -> DoF(tile_c_cur) -> NTT16_B; "
                ,"tile_r unused; tile_c synced on bank change, then counted per 16 rows.");
    end
endmodule

`default_nettype wire

