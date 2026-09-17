// -----------------------------------------------------------------------------
// tb_single_on_fly_wrap_general.v (Verilog-2001)
// Purpose: Verify single_on_fly_wrap (seamless mode) using REAL ROMs with
//          general inputs and precomputed golden for a 4x4 tile quadrant.
//   - Wrapper advances tiles internally (prefetch + shadow + boundary start).
//   - We just feed 16 rows per tile continuously (no bubbles), starting at (r=0,c=0).
//   - Tiles covered: (r,c) = 0..3 × 0..3 (? 16 tiles = 256 frames).
//   - PASS ? all out_valid frames match golden; FAIL ? first mismatch shown.
// -----------------------------------------------------------------------------
// ???:
//  1) bank_id ? ?? ??: ntt_params.vh? `NTT_BANK_BITS`? TB??? ??.
//  2) ??? ?? ??: jmap=lane, rotate=0, lane_rev_in=0 (? RTL? 1:1).
//  3) ??? ??? ??: tb_single_on_fly_wrap_general
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
`default_nettype none

`include "ntt_params.vh"   // ? bank_id ?? ??? ???? ??? ??

module tb_single_on_fly_general;

    localparam integer WORD_WIDTH = 60;
    localparam integer LANES      = 16;
    localparam integer PACK       = WORD_WIDTH*LANES;

    localparam integer TILES_R    = 4;
    localparam integer TILES_C    = 4;
    localparam integer TILES      = TILES_R*TILES_C; // 16
    localparam integer ROWS       = 16;              // per tile
    localparam integer FRAMES     = TILES*ROWS;      // 256

    localparam integer CLK_HALF   = 5;   // 100 MHz
    localparam integer RST_CYC    = 5;

    // DUT I/O (wrapper)
    reg                         clk;
    reg                         rst_n;
    reg  [`NTT_BANK_BITS-1:0]   bank_id;   // ? ?? ?? ??? ?? (??/?z? ??)
    reg  [3:0]                  tile_r;
    reg  [3:0]                  tile_c;

    reg                         in_valid;
    reg  [PACK-1:0]             in_data_vec;
    wire                        out_valid;
    wire [PACK-1:0]             out_data_vec;

    // Device Under Test: ??
    single_on_fly_wrap #(
        .WORD_WIDTH     (WORD_WIDTH),
        .LANES          (LANES),
        .SEAMLESS_TILES (1),
        .TILES_PER_ROW  (TILES_C)
    ) DUT (
        .clk          (clk),
        .rst_n        (rst_n),
        .bank_id      (bank_id),   // bank 0 ?? (?? ? ??? ?)
        .tile_r       (tile_r),    // ?? ?? (0,0)?? ??
        .tile_c       (tile_c),
        .in_valid     (in_valid),
        .in_data_vec  (in_data_vec),
        .out_valid    (out_valid),
        .out_data_vec (out_data_vec)
    );

    // (??? ??? defparam? ?????)
    // defparam tb_single_on_fly_wrap_general.DUT.U_TWIST.INIT_EVEN_HEX  = "single_vector/single_even_rom_combined.hex";
    // defparam tb_single_on_fly_wrap_general.DUT.U_TWIST.INIT_Z2_HEX    = "single_vector/single_z2_rom_combined.hex";
    // defparam tb_single_on_fly_wrap_general.DUT.U_TWIST.INIT_Z2PRE_HEX = "single_vector/single_z2pre_rom_combined.hex";
    // defparam tb_single_on_fly_wrap_general.DUT.U_TWIST.INIT_ODD_HEX   = "single_vector/single_odd_rom_combined.hex";

    // Clock
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    // Stimulus & Golden (4×4 ???)
    reg [PACK-1:0] IN  [0:FRAMES-1];
    reg [PACK-1:0] EXP [0:FRAMES-1];

    integer tx, rx, errors, shown;

    // Reset & preload & drive
    initial begin
        rst_n       = 1'b0;
        bank_id     = '0;          // ? ?-?? ?? ???
        tile_r      = 4'd0;
        tile_c      = 4'd0;
        in_valid    = 1'b0;
        in_data_vec = {PACK{1'b0}};

        // ??/?? (?? ??: j=lane, rot=0, lane_rev_in=0)
        $readmemh("wrap/sof_in_general.hex",                 IN);
        $readmemh("wrap/sof_exp_wrap_jlane_rot0_lr0.hex",    EXP);

        // (???? ??+1? ???? ???? ?? ? ?? ?? ??)
        // $readmemh("wrap/sof_exp_wrap_jlane_rot1_lr0.hex", EXP);

        // Change TB-driven signals on negedge; the DUT samples on posedge.
        repeat (RST_CYC) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // ??? ???? start? ??? ??? ?? ? ??? ?? ?? ?? 16??? ??
        tx = 0;
        while (tx < FRAMES) begin
            @(negedge clk);
            in_valid    = 1'b1;
            in_data_vec = IN[tx];
            tx = tx + 1;
        end
        @(negedge clk);
        in_valid    = 1'b0;
        in_data_vec = {PACK{1'b0}};

        // ?? ? ???
        repeat (10) @(posedge clk);
        if (errors==0) $display("[WARN] finished without collecting all outputs?");
        $finish;
    end

    // Scoreboard
    initial begin
        rx = 0; errors = 0; shown = 0;
        wait(rst_n==1'b1);
        forever begin
            @(posedge clk);
            if (out_valid) begin
                if (out_data_vec !== EXP[rx]) begin
                    errors = errors + 1;
                    if (shown==0) begin
                        $display("[MISMATCH] idx=%0d", rx);
                        $display("  exp=%0h", EXP[rx]);
                        $display("  got=%0h",  out_data_vec);
                        shown = 1;
                    end
                end
                rx = rx + 1;
                if (rx == FRAMES) begin
                    repeat (3) @(posedge clk);
                    if (errors==0) $display("[PASS] single_on_fly_wrap general (4x4 tiles)");
                    else           $display("[FAIL] errors=%0d", errors);
                    $finish;
                end
            end
        end
    end

endmodule

`default_nettype wire

