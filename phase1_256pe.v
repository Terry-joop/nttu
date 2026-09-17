// ============================================================================
// phase1_256pe.v  // Verilog-2001  // final (16x16 fixed, hidden-PRIME only)
// ----------------------------------------------------------------------------
// Pipeline: NTT16(col) -> transpose16x16_bus(hidden PRIME) -> SoF -> NTT16(row)
// - transpose m_valid? ??? 16??? ?? 1 (? ?? prime? ???? ??)
// - ?? ??? m_valid? 1? 16???? ????? ?? (?? ??)
// - SoF? ?? ? ??? ??? (tile_r,tile_c)? ?? ?
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module phase1_256pe #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH,  // e.g., 60
    parameter integer LANES      = `LANES,           // 16
    parameter integer TILES_TC   = 16,               // ?? 16
    parameter integer TILES_TR   = 16                // ?? 16
)(
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [`NTT_BANK_BITS-1:0]    bank_id,

    input  wire                         in_valid,
    input  wire [WORD_WIDTH*LANES-1:0]  in_data,

    output wire                         out_valid,
    output wire [WORD_WIDTH*LANES-1:0]  out_data
);
    localparam integer WIDE = WORD_WIDTH*LANES;

    // -------------------- Stage-1: NTT16 (column pass) -----------------------
    wire                  v_ntt_col;
    wire [WIDE-1:0]       d_ntt_col;

    ntt16_buscore #(
        .WORD_WIDTH  (WORD_WIDTH),
        .BARRETT_LAT (`NTT_BARRETT_LAT)
    ) U_NTT16_COL (
        .clk     (clk),
        .rst_n   (rst_n),
        .s_valid (in_valid),
        .s_data  (in_data),
        .bank_id (bank_id),
        .m_valid (v_ntt_col),
        .m_data  (d_ntt_col)
    );

    // -------------------- Stage-2: transpose 16x16 (hidden PRIME) ------------
    wire            v_tr;   // transpose m_valid (??? 16c ?? 1)
    wire [WIDE-1:0] d_tr;   // transpose m_data

    transpose16x16_bus #(
        .WORD_WIDTH (WORD_WIDTH)
    ) U_TRP (
        .clk     (clk),
        .rst_n   (rst_n),
        .s_valid (v_ntt_col),
        .s_data  (d_ntt_col),
        .m_valid (v_tr),
        .m_data  (d_tr)
    );

    // -------------------- tile scheduler (col-first) -------------------------
    // - v_tr? 1? ???? ??? (0..15) ? 16??(=15)?? ?? ??
    // - col-first: (r,c) = (0..15, 0..15) with c fast in phase1 top-level?
    //   ???? ?? ???? r? ?? ???, r? ??? c ??(=row-major).
    //   transpose ?? ??? SoF ???? ?? TB?? ?? ???.
    reg [3:0] tile_r, tile_c;

    // 16x16 ?? ?? (??/?? ?? ??)
    localparam [3:0] TR_MAX = 4'd15;
    localparam [3:0] TC_MAX = 4'd15;

    // v_tr==1? ?? 0..15 ???
    reg  [3:0] tr_cnt;                 // 0..15
    wire       tr_active = v_tr;       // steady-state 1
    wire       tile_start_cnt = (tr_active && (tr_cnt == 4'd0));
    wire       tile_end_cnt   = (tr_active && (tr_cnt == 4'd15));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tr_cnt <= 4'd0;
        end else if (!tr_active) begin
            tr_cnt <= 4'd0;
        end else begin
            tr_cnt <= (tr_cnt == 4'd15) ? 4'd0 : (tr_cnt + 4'd1);
        end
    end

    // ?? ???: ??? ?(row15) ?? ???? tile_end ??,
    // ? ?? ????? ? ??? row0? ???? ??? r/c ????
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tile_r <= 4'd0;
            tile_c <= 4'd0;
        end else if (tile_end_cnt) begin
            if (tile_r == TR_MAX) begin
                tile_r <= 4'd0;
                tile_c <= (tile_c == TC_MAX) ? 4'd0 : (tile_c + 4'd1);
            end else begin
                tile_r <= tile_r + 4'd1;
            end
        end
    end

    // -------------------- Stage-3: Single-On-Fly twist -----------------------
    // SoF? ?? ???(v_tr)? ??? ??, ?? ? ??? ??? ???? ?
    wire            v_sof;
    wire [WIDE-1:0] d_sof;

    single_on_fly_wrap #(
        .WORD_WIDTH (WORD_WIDTH),
        .LANES      (LANES)
    ) U_SOF (
        .clk          (clk),
        .rst_n        (rst_n),
        .bank_id      (bank_id),
        .tile_r       (tile_r),
        .tile_c       (tile_c),
        .in_valid     (v_tr),     // hidden PRIME: 16c/tile, no bubble
        .in_data_vec  (d_tr),
        .out_valid    (v_sof),
        .out_data_vec (d_sof)
    );

    // -------------------- Stage-4: NTT16 (row pass) --------------------------
    ntt16_buscore #(
        .WORD_WIDTH  (WORD_WIDTH),
        .BARRETT_LAT (`NTT_BARRETT_LAT)
    ) U_NTT16_ROW (
        .clk      (clk),
        .rst_n    (rst_n),
        .s_valid  (v_sof),
        .s_data   (d_sof),
        .bank_id  (bank_id),
        .m_valid  (out_valid),
        .m_data   (out_data)
    );

endmodule

`default_nettype wire

