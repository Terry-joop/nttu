// -----------------------------------------------------------------------------
// single_on_fly_wrap.v  (Verilog-2001)
// Version: v1.5s-FASTSTART-FIX
//   - Seamless tiles (prefetch + shadow).
//   - First start is on in_valid? (FASTSTART, 1c earlier than consumption).
//   - Robust seed handoff: capture+commit at boundary+1, row0 uses vec directly.
//   - Parameterizable ROM_OUT_LATENCY (default 1).
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module single_on_fly_wrap #(
    parameter integer WORD_WIDTH        = `NTT_WORD_WIDTH, // e.g., 60
    parameter integer LANES             = `LANES,          // 16
    parameter         SEAMLESS_TILES    = 1,               // 1: enable seamless mode
    parameter integer ROM_OUT_LATENCY   = 2,               // cycles from addr change to vec stable
    parameter integer TILES_PER_ROW     = 16               // tile columns before advancing tile_r
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // Bank / tile indices (col-first schedule)
    input  wire [`NTT_BANK_BITS-1:0]    bank_id,          // 0..NUM_LEVELS-1
    input  wire [3:0]                   tile_r,           // 0..15 (current tile)
    input  wire [3:0]                   tile_c,           // 0..15 (current tile)

    // Stream in/out (aligned with ntt16_buscore)
    input  wire                         in_valid,
    input  wire [LANES*WORD_WIDTH-1:0]  in_data_vec,
    output wire                         out_valid,
    output wire [LANES*WORD_WIDTH-1:0]  out_data_vec
);
    localparam integer PACK = LANES*WORD_WIDTH;

    // ----------------------------- q / mu / q_width --------------------------
    wire [WORD_WIDTH-1:0]   q_wire;
    wire [2*WORD_WIDTH-1:0] mu_wire;
    wire [5:0]              q_width_wire;

    qconst_rom_bank #(
        .WORD_WIDTH (WORD_WIDTH),
        .NUM_BANKS  (`NTT_NUM_BANKS),
        .BANK_BITS  (`NTT_BANK_BITS),
        .Q_HEX      (`NTT_Q_HEX),
        .MU_HEX     (`NTT_MU_HEX),
        .QW_HEX     (`NTT_Q_WIDTH_HEX)
    ) U_QCONST (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_sel (bank_id),
        .q        (q_wire),
        .mu       (mu_wire),
        .q_width  (q_width_wire)
    );

    // --------------------------- Input staging (1c) --------------------------
    // Core consumes on s_valid_d cycle (no throughput loss).
    reg                  s_valid_d;
    reg  [PACK-1:0]      s_data_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_valid_d <= 1'b0;
            s_data_d  <= {PACK{1'b0}};
        end else begin
            s_valid_d <= in_valid;
            s_data_d  <= in_data_vec;
        end
    end

    // --------------------------- Seamless tile logic --------------------------
    // Internal tile cursor (for ROM) and consumed-position counter.
    reg [`NTT_BANK_BITS-1:0] rom_bank;
    reg [3:0]                rom_r, rom_c;

    // Track current tile used by SoF (for next calculation)
    reg [3:0]                cur_r, cur_c;

    // 0..15 position based on what SoF consumes (s_valid_d)
    reg [4:0] tile_pos; // wider to avoid warnings

    // Seed shadows
    reg [PACK-1:0] even_shadow, z2_shadow, z2pre_shadow, odd_shadow;

    // Active seed source selector: 0=ROM vec (first/row0), 1=shadow (rows 1..15)
    reg use_shadow;

    // ROM outputs (assumed registered wide outputs)
    wire [PACK-1:0] even_vec, z2_vec, z2pre_vec, odd_vec;

    // Next-tile calculator (col-first): (r,c)->(r,c+1)->...->(r+1,0).
    // Clamp the parameter to the 4-bit tile address range.
    localparam [3:0] LAST_TILE_C = (TILES_PER_ROW < 1) ? 4'd0 :
                                 ((TILES_PER_ROW > 16) ? 4'd15 : TILES_PER_ROW-1);
    wire [3:0] next_r_calc = (cur_c == LAST_TILE_C) ? (cur_r + 4'd1) : cur_r;
    wire [3:0] next_c_calc = (cur_c == LAST_TILE_C) ? 4'd0           : (cur_c + 4'd1);

    // ----- timing positions derived from ROM latency -----
    // prefetch at 15-L (min clamp to 12 to keep within tile)
    localparam integer LCL                = (ROM_OUT_LATENCY < 0) ? 0 : ROM_OUT_LATENCY;
    localparam integer PREFETCH_POS_INT   = 15 - LCL;
    localparam integer PREFETCH_POS       = (PREFETCH_POS_INT < 12) ? 12 : PREFETCH_POS_INT;
    localparam integer BOUNDARY_POS       = 15;

    // Events on consumption cycle
    wire sof_consume   = s_valid_d;
    wire prefetch_fire = SEAMLESS_TILES && sof_consume && (tile_pos == PREFETCH_POS[4:0]);
    wire boundary_fire = SEAMLESS_TILES && sof_consume && (tile_pos == BOUNDARY_POS[4:0]);

    // boundary+1 (row0 of next tile)
    reg boundary_fire_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) boundary_fire_q <= 1'b0;
        else        boundary_fire_q <= boundary_fire;
    end

    // Cursor & counters
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // initialize to external tile at reset
            rom_bank  <= bank_id;
            rom_r     <= tile_r;
            rom_c     <= tile_c;
            cur_r     <= tile_r;
            cur_c     <= tile_c;
            tile_pos  <= 5'd0;
            use_shadow<= 1'b0;
            even_shadow  <= {PACK{1'b0}};
            z2_shadow    <= {PACK{1'b0}};
            z2pre_shadow <= {PACK{1'b0}};
            odd_shadow   <= {PACK{1'b0}};
        end else begin
            // tile_pos advance when SoF consumes
            if (sof_consume) begin
                tile_pos <= (tile_pos == 5'd15) ? 5'd0 : (tile_pos + 5'd1);
            end

            // prefetch next tile address late in the tile
            if (prefetch_fire) begin
                rom_bank <= bank_id;     // assume bank stable or already updated if changed
                rom_r    <= next_r_calc;
                rom_c    <= next_c_calc;
            end

            // boundary+1: capture next-tile seeds & commit shadow usage
            if (boundary_fire_q) begin
                // ROM vec are already 'next tile' here (prefetched earlier)
                even_shadow  <= even_vec;
                z2_shadow    <= z2_vec;
                z2pre_shadow <= z2pre_vec;
                odd_shadow   <= odd_vec;

                use_shadow   <= 1'b1;    // from row1..15, use shadow
                cur_r        <= next_r_calc;
                cur_c        <= next_c_calc;
            end
        end
    end

    // Per-tile twist ROM (driven by internal ROM cursor)
    single_twist_rom #(
        .WORD_WIDTH     (WORD_WIDTH),
        .LANES          (LANES),
        // Override INIT_* parameters here if your file paths differ.
        .INIT_EVEN_HEX  ("single_vector/single_even_rom_combined.hex"),
        .INIT_Z2_HEX    ("single_vector/single_z2_rom_combined.hex"),
        .INIT_Z2PRE_HEX ("single_vector/single_z2pre_rom_combined.hex"),
        .INIT_ODD_HEX   ("single_vector/single_odd_rom_combined.hex")
    ) U_TWIST (
        .clk        (clk),
        .rst_n      (rst_n),
        .bank_id    (rom_bank),
        .tile_r     (rom_r),
        .tile_c     (rom_c),
        .even_vec   (even_vec),
        .z2_vec     (z2_vec),
        .z2pre_vec  (z2pre_vec),
        .odd_vec    (odd_vec)
    );

    // Active seed selection
    //  - boundary+1(row0): ??? vec ??(?? ???? shadow ??? ?? ??)
    //  - ? ?: ? ?? ?? use_shadow=0?? vec, ?? ??? shadow
    wire switch_pending = boundary_fire_q;

    wire [PACK-1:0] even_active   = switch_pending ? even_vec
                                  : (use_shadow ? even_shadow : even_vec);
    wire [PACK-1:0] z2_active     = switch_pending ? z2_vec
                                  : (use_shadow ? z2_shadow   : z2_vec);
    wire [PACK-1:0] z2pre_active  = switch_pending ? z2pre_vec
                                  : (use_shadow ? z2pre_shadow: z2pre_vec);
    wire [PACK-1:0] odd_active    = switch_pending ? odd_vec
                                  : (use_shadow ? odd_shadow  : odd_vec);

    // ------------------------------- SoF core --------------------------------
    // Start pulse generation:
    //   - First tile: use rising edge of **in_valid** (one cycle before s_valid_d)
    //   - Subsequent tiles: boundary_fire (??)?? start? ?? ? row0?? ??
    reg in_valid_q;
    reg started;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_valid_q <= 1'b0;
            started    <= 1'b0;
        end else begin
            in_valid_q <= in_valid;
            if (SEAMLESS_TILES) begin
                if (!started && (in_valid==1'b1) && (in_valid_q==1'b0))
                    started <= 1'b1;   // first start has happened
            end else begin
                started <= 1'b0;
            end
        end
    end

    wire start_first  = (SEAMLESS_TILES && !started) ?
                        ((in_valid==1'b1) && (in_valid_q==1'b0)) : 1'b0;
    wire start_next   = (SEAMLESS_TILES) ? boundary_fire : 1'b0;
    wire start_legacy = (!SEAMLESS_TILES) ?
                        ((in_valid==1'b1) && (in_valid_q==1'b0)) : 1'b0;

    wire start_pulse = start_first | start_next | start_legacy;

    wire            core_busy;
    wire [PACK-1:0] core_out_vec; // raw lane order from core

    single_on_fly_twist16 #(
        .WORD_WIDTH (WORD_WIDTH),
        .LANES      (LANES)
    ) U_SOF (
        .clk                (clk),
        .rst_n              (rst_n),
        .start              (start_pulse),     // pulses at first tile and every boundary
        .busy               (core_busy),

        .even_seed_vec      (even_active),
        .odd_seed_vec       (odd_active),
        .ratio_vec          (z2_active),       // ratio := zeta^2 per lane
        .z_pre_vec          (z2pre_active),    // z_pre := floor(z2 * 2^K / q)

        .q                  (q_wire),
        .mu                 (mu_wire),
        .q_width            (q_width_wire),

        .in_valid           (s_valid_d),       // core consumes staged input
        .in_data_vec        (s_data_d),

        .barrett_valid_out  (out_valid),
        .barrett_result_vec (core_out_vec)
    );

    // ----------------------- Lane repacker to buscore ------------------------
    // Verilog-2001 part-select (no [+:])
    genvar li;
    generate
        for (li = 0; li < LANES; li = li + 1) begin : G_PACK_ALIGN
            assign out_data_vec[(WORD_WIDTH*li)+WORD_WIDTH-1 : (WORD_WIDTH*li)]
                 = core_out_vec[(WORD_WIDTH*(LANES-1-li))+WORD_WIDTH-1 : (WORD_WIDTH*(LANES-1-li))];
        end
    endgenerate

    // Trace banner
    initial begin
        $display("[single_on_fly_wrap v1.5s-FASTSTART-FIX] ROM_LAT=%0d, seamless tiles; start-first; row0 uses vec; clean handoff.",
                 ROM_OUT_LATENCY);
    end
endmodule

`default_nettype wire

