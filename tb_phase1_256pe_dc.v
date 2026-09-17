// ============================================================================
// tb_phase1_256pe_dc.v  // Verilog-2001
// v1.5c - valid-cycle only dump + single-loop finish (no SV keywords)
// ----------------------------------------------------------------------------
// Streams DC (all lanes = 1) for 4x4 tiles (256 rows) through:
//   in -> NTT16(col) -> transpose16x16_bus (hidden PRIME) -> SoF -> NTT16(row)
// Dumps ONLY cycles where each stage's valid==1.
// Finishes when all stage line-counts reach NUM_LINES or on timeout.
// Files: phase1_result/dc_in.hex, dc_col_ntt.hex, dc_transpose.hex,
//        dc_sof.hex, dc_row_ntt.hex
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module tb_phase1_256pe_dc;

    // ------------------------------------------------------------------------
    // Parameters & widths
    // ------------------------------------------------------------------------
    localparam integer WORD_WIDTH = `NTT_WORD_WIDTH;     // e.g., 60
    localparam integer LANES      = `LANES;              // 16
    localparam integer WIDE       = WORD_WIDTH * LANES;  // 960

    // 4 x 4 = 16 tiles (each 16 cycles) -> 256 lines
    localparam integer TILES_TC   = 4;
    localparam integer TILES_TR   = 4;
    localparam integer TILE_LEN   = 16;
    localparam integer NUM_LINES  = (TILES_TC * TILES_TR * TILE_LEN); // 256

    // Timeout (cycles after reset deassert) to avoid hang if bug exists
    localparam integer MAX_WAIT_CYCLES = 10000;

    // Output file paths
    // Keep DC output in the existing result directory and separate it from
    // the general-test vectors.
    localparam [1023:0] PATH_IN  = "phase1_result/dc_in.hex";
    localparam [1023:0] PATH_COL = "phase1_result/dc_col_ntt.hex";
    localparam [1023:0] PATH_TRP = "phase1_result/dc_transpose.hex";
    localparam [1023:0] PATH_SOF = "phase1_result/dc_sof.hex";
    localparam [1023:0] PATH_ROW = "phase1_result/dc_row_ntt.hex";

    // ------------------------------------------------------------------------
    // Clock / Reset
    // ------------------------------------------------------------------------
    reg clk;
    reg rst_n;
    initial clk = 1'b0;
    always #5 clk = ~clk;  // 100 MHz

    initial begin
        rst_n = 1'b0;
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
    end

    // ------------------------------------------------------------------------
    // DUT I/O
    // ------------------------------------------------------------------------
    reg  [`NTT_BANK_BITS-1:0] bank_id;
    reg                       in_valid;
    reg  [WIDE-1:0]           in_data;
    wire                      out_valid;
    wire [WIDE-1:0]           out_data;

    // ------------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------------
    phase1_256pe #(
        .WORD_WIDTH (WORD_WIDTH),
        .LANES      (LANES),
        .TILES_TC   (TILES_TC),
        .TILES_TR   (TILES_TR)
    ) DUT (
        .clk      (clk),
        .rst_n    (rst_n),
        .bank_id  (bank_id),
        .in_valid (in_valid),
        .in_data  (in_data),
        .out_valid(out_valid),
        .out_data (out_data)
    );

    // ------------------------------------------------------------------------
    // Hierarchical taps (read-only)
    // ------------------------------------------------------------------------
    wire            v_col   = DUT.U_NTT16_COL.m_valid;
    wire [WIDE-1:0] d_col   = DUT.U_NTT16_COL.m_data;

    wire            v_trp   = DUT.U_TRP.m_valid;
    wire [WIDE-1:0] d_trp   = DUT.U_TRP.m_data;

    wire            v_sof   = DUT.U_SOF.out_valid;
    wire [WIDE-1:0] d_sof   = DUT.U_SOF.out_data_vec;

    // ------------------------------------------------------------------------
    // File handles / counters
    // ------------------------------------------------------------------------
    integer fh_in, fh_col, fh_trp, fh_sof, fh_row;
    integer c_in, c_col, c_trp, c_sof, c_row;

    initial begin
        fh_in  = $fopen(PATH_IN,  "w");
        fh_col = $fopen(PATH_COL, "w");
        fh_trp = $fopen(PATH_TRP, "w");
        fh_sof = $fopen(PATH_SOF, "w");
        fh_row = $fopen(PATH_ROW, "w");
        c_in = 0; c_col = 0; c_trp = 0; c_sof = 0; c_row = 0;
        $display("[TB] opened files: IN=%0d COL=%0d TRP=%0d SOF=%0d ROW=%0d",
                 fh_in, fh_col, fh_trp, fh_sof, fh_row);
    end

    // ------------------------------------------------------------------------
    // Drive DC (all lanes = 1'h1) input for NUM_LINES cycles
    // ------------------------------------------------------------------------
    integer n;
    reg [WORD_WIDTH-1:0] one_word;
    initial begin
        bank_id  = {`NTT_BANK_BITS{1'b0}}; // level 0
        in_valid = 1'b0;
        in_data  = {WIDE{1'b0}};
        one_word = {{(WORD_WIDTH-1){1'b0}}, 1'b1};

        @(posedge rst_n);
        @(posedge clk);

        for (n = 0; n < NUM_LINES; n = n + 1) begin
            @(posedge clk);
            in_valid <= 1'b1;
            in_data  <= {LANES{one_word}};
            $fdisplay(fh_in, "%h", {LANES{one_word}});
            c_in <= c_in + 1;
        end

        // stop drive
        @(posedge clk);
        in_valid <= 1'b0;
        in_data  <= {WIDE{1'b0}};
    end

    // ------------------------------------------------------------------------
    // Dumpers (ONLY when each stage's valid=1)
    // ------------------------------------------------------------------------

    // COL NTT: expect exactly NUM_LINES lines
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_col <= 0;
        end else if (v_col) begin
            $fdisplay(fh_col, "%h", d_col);
            c_col <= c_col + 1;
        end
    end

    // TRANSPOSE (hidden PRIME): expect exactly NUM_LINES lines (16/Tile)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_trp <= 0;
        end else if (v_trp) begin
            $fdisplay(fh_trp, "%h", d_trp);
            c_trp <= c_trp + 1;
        end
    end

    // SOF: expect exactly NUM_LINES lines
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_sof <= 0;
        end else if (v_sof) begin
            $fdisplay(fh_sof, "%h", d_sof);
            c_sof <= c_sof + 1;
        end
    end

    // ROW NTT: expect exactly NUM_LINES lines
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_row <= 0;
        end else if (out_valid) begin
            $fdisplay(fh_row, "%h", out_data);
            c_row <= c_row + 1;
        end
    end

    // ------------------------------------------------------------------------
    // Finish logic: wait until all counts hit NUM_LINES or timeout
    // ------------------------------------------------------------------------
    initial begin : FINISH_LOOP
        integer tmo;
        @(posedge rst_n);
        tmo = 0;
        // Single loop, Verilog-2001 friendly
        while ( (c_col < NUM_LINES) ||
                (c_trp < NUM_LINES) ||
                (c_sof < NUM_LINES) ||
                (c_row < NUM_LINES) ) begin
            @(posedge clk);
            tmo = tmo + 1;
            if (tmo >= MAX_WAIT_CYCLES) begin
                $display("[TB][TIMEOUT] after %0d cycles  in=%0d col=%0d tr=%0d sof=%0d out=%0d",
                         MAX_WAIT_CYCLES, c_in, c_col, c_trp, c_sof, c_row);
                $fclose(fh_in);
                $fclose(fh_col);
                $fclose(fh_trp);
                $fclose(fh_sof);
                $fclose(fh_row);
                $stop;
            end
        end

        $display("[TB] COUNT: in=%0d col=%0d tr=%0d sof=%0d out=%0d",
                 c_in, c_col, c_trp, c_sof, c_row);
        $fclose(fh_in);
        $fclose(fh_col);
        $fclose(fh_trp);
        $fclose(fh_sof);
        $fclose(fh_row);
        $stop;
    end

endmodule

`default_nettype wire

