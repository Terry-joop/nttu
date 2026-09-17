// ============================================================================
// tb_phase1_256pe_gen.v  // Verilog-2001  // v1.4 (q-normalize added)
// ----------------------------------------------------------------------------
// ??:
//   - 16x16 ?? ???(= 256 tiles ? 4096 lines)? ?? LFSR ???? ??.
//   - ?? ??: in -> col_ntt -> transpose -> sof -> row_ntt
//   - ? ??? m_valid/out_valid ???? ???? ?? ?? ??.
//   - (D) SoF ROM? ???? ??? ????? tiles_order.log? ??.
//   - (??) 16? ?? ?? ??(= buscore ? ???)? ????? ??,
//           COL/ROW? ???? ?????? ?? ??.
//
// ??(v1.4):
//   - ??(LFSR/FILE_STIM)? q ??? ???(norm_modq)?? ??/??.
//   - ?? ??([59:58])? ????? ???+???? 0..q-1? ??.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module tb_phase1_256pe_gen;

    // ------------------------------------------------------------------------
    // Parameters & widths
    // ------------------------------------------------------------------------
    localparam integer WORD_WIDTH = `NTT_WORD_WIDTH;     // e.g., 60
    localparam integer LANES      = `LANES;              // 16
    localparam integer WIDE       = WORD_WIDTH * LANES;  // 960

    // Full frame: 16 x 16 tiles -> 256 tiles, each 16 lines -> 4096 lines
    localparam integer TILES_TC   = 16;                  // columns of tiles
    localparam integer TILES_TR   = 16;                  // rows of tiles
    localparam integer TILE_LEN   = 16;
    localparam integer NUM_TILES  = (TILES_TC * TILES_TR);
    localparam integer NUM_LINES  = (NUM_TILES * TILE_LEN); // 4096

    // Optional: insert 1-cycle bubble between tiles at the input (default OFF)
    localparam integer INSERT_TILE_BUBBLE = 0;

    // ?? ?? ?? (??? ?? ??)
    localparam [1023:0] PATH_IN    = "phase1_result/in.hex";
    localparam [1023:0] PATH_COL   = "phase1_result/col_ntt.hex";
    localparam [1023:0] PATH_TRP   = "phase1_result/transpose.hex";
    localparam [1023:0] PATH_SOF   = "phase1_result/sof.hex";
    localparam [1023:0] PATH_ROW   = "phase1_result/row_ntt.hex";
    localparam [1023:0] PATH_TILES = "phase1_result/tiles_order.log";

    // ------------------------------------------------------------------------
    // q-normalize: ?? ?? ?? 58?? + ?? 1?? 0..q-1 ???
    // ------------------------------------------------------------------------
    localparam integer Q_WIDTH = 58;
    localparam [WORD_WIDTH-1:0] Q      = 60'h3ffffffffbe0001;               // q
    localparam [WORD_WIDTH-1:0] Q_MASK = ({{(WORD_WIDTH-1){1'b0}},1'b1} << Q_WIDTH) - 1; // 2^58 - 1

    function [WORD_WIDTH-1:0] norm_modq;
      input [WORD_WIDTH-1:0] x;
      reg   [WORD_WIDTH-1:0] t;
      begin
        t = x & Q_MASK;                 // [57:0]? ?? ? t <= 2^58-1
        norm_modq = (t >= Q) ? (t - Q) : t; // q ???? ???? < q
      end
    endfunction

    task normalize_bus_inplace(input [WIDE-1:0] raw, output [WIDE-1:0] out_bus);
      integer i;
      reg [WORD_WIDTH-1:0] w;
      begin
        for (i=0; i<LANES; i=i+1) begin
          w = raw[WORD_WIDTH*i +: WORD_WIDTH];
          out_bus[WORD_WIDTH*i +: WORD_WIDTH] = norm_modq(w);
        end
      end
    endtask

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
        .clk       (clk),
        .rst_n     (rst_n),
        .bank_id   (bank_id),
        .in_valid  (in_valid),
        .in_data   (in_data),
        .out_valid (out_valid),
        .out_data  (out_data)
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
    integer fh_in, fh_col, fh_trp, fh_sof, fh_row, fh_tiles;
    integer c_in, c_col, c_trp, c_sof, c_row;

    // ------------------------------------------------------------------------
    // Timeout/latency cycle counter (??? ?? ??)
    // ------------------------------------------------------------------------
    integer cyc;

    initial begin
        fh_in   = $fopen(PATH_IN,   "w");
        fh_col  = $fopen(PATH_COL,  "w");
        fh_trp  = $fopen(PATH_TRP,  "w");
        fh_sof  = $fopen(PATH_SOF,  "w");
        fh_row  = $fopen(PATH_ROW,  "w");
        fh_tiles= $fopen(PATH_TILES,"w");

        c_in = 0; c_col = 0; c_trp = 0; c_sof = 0; c_row = 0;

        $display("[TB] opened files: IN=%0d COL=%0d TRP=%0d SOF=%0d ROW=%0d TILES=%0d",
                 fh_in, fh_col, fh_trp, fh_sof, fh_row, fh_tiles);
    end

    // ------------------------------------------------------------------------
    // ---- latency probes (??? ?? ???) ----
    // ------------------------------------------------------------------------
    reg got_in0, got_col0, got_trp0, got_sof0, got_row0;
    integer cyc_in0, cyc_col0, cyc_trp0, cyc_sof0, cyc_row0;
    integer L_col, L_trp, L_sof, L_row, L_total;

    initial begin
      got_in0 = 0; got_col0 = 0; got_trp0 = 0; got_sof0 = 0; got_row0 = 0;
      cyc_in0 = 0; cyc_col0 = 0; cyc_trp0 = 0; cyc_sof0 = 0; cyc_row0 = 0;
      L_col = 0; L_trp = 0; L_sof = 0; L_row = 0; L_total = 0;
    end

    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        got_in0 <= 0; got_col0 <= 0; got_trp0 <= 0; got_sof0 <= 0; got_row0 <= 0;
      end else begin
        if (!got_in0 && in_valid)    begin got_in0  <= 1; cyc_in0  <= cyc; end
        if (!got_col0 && v_col)      begin got_col0 <= 1; cyc_col0 <= cyc; end
        if (!got_trp0 && v_trp)      begin got_trp0 <= 1; cyc_trp0 <= cyc; end
        if (!got_sof0 && v_sof)      begin got_sof0 <= 1; cyc_sof0 <= cyc; end
        if (!got_row0 && out_valid)  begin got_row0 <= 1; cyc_row0 <= cyc; end
      end
    end

    // ------------------------------------------------------------------------
    // ?? ??: per-lane LFSR (?? ???? 16?? ??)
    // ------------------------------------------------------------------------
    function [WORD_WIDTH-1:0] lfsr_next;
        input [WORD_WIDTH-1:0] s;
        reg feedback;
        begin
            feedback = s[0] ^ s[1] ^ s[4] ^ s[5];
            lfsr_next = {feedback, s[WORD_WIDTH-1:1]};
        end
    endfunction

    reg [WORD_WIDTH-1:0] lfsr_lane [0:LANES-1];

    integer li;
    initial begin
        for (li=0; li<LANES; li=li+1) begin
            lfsr_lane[li] = {{(WORD_WIDTH-8){1'b0}}, (8'hA5 ^ li[7:0])};
        end
    end

    task gen_next_input(output [WIDE-1:0] bus);
        integer i;
        reg [WORD_WIDTH-1:0] word_i;
        begin
            for (i=0; i<LANES; i=i+1) begin
                lfsr_lane[i] = lfsr_next(lfsr_lane[i]);
                word_i = lfsr_lane[i];
                bus[WORD_WIDTH*i +: WORD_WIDTH] = word_i;
            end
        end
    endtask

    // ---------[NEW] ?? ?? STIM ?? ---------
    integer USE_FILE_STIM;
    reg [8*256-1:0] FILE_STIM;  // ?? ???
    initial begin
        USE_FILE_STIM = 0;
        FILE_STIM = "ntt16_data/ntt16_out/level0/ntt16_stim.hex";
        if ($value$plusargs("USE_FILE_STIM=%d", USE_FILE_STIM))
            $display("[TB] USE_FILE_STIM=%0d", USE_FILE_STIM);
        if ($value$plusargs("FILE_STIM=%s", FILE_STIM))
            $display("[TB] FILE_STIM=%0s", FILE_STIM);
    end

    // ? 16?? ??(??? DUT? ??: WIDE)
    reg [WIDE-1:0] file_stim [0:15];
    initial begin
        if (USE_FILE_STIM) begin
            $readmemh(FILE_STIM, file_stim);
            $display("[TB] loaded first 16 lines from %0s", FILE_STIM);
        end
    end
    // -------------------------------------------

    // ------------------------------------------------------------------------
    // ? ??: ??? ??(4096?) ?? ?? (?? ?? ??)
    // ------------------------------------------------------------------------
    integer n;
    integer tile_line_ctr;
    reg [WIDE-1:0] tmp_bus; // LFSR ?? ??

    // Debug: in_data? q? ??? ??
    // synthesis translate_off
    integer ai;
    always @(posedge clk) if (in_valid) begin
      for (ai=0; ai<LANES; ai=ai+1) begin
        if (in_data[WORD_WIDTH*ai +: WORD_WIDTH] >= Q) begin
          $display("[TB][WARN] in >= q at line %0d lane %0d: %h",
                   c_in+1, ai, in_data[WORD_WIDTH*ai +: WORD_WIDTH]);
          $stop; // ????? ?? (?? bring-up? ??)
        end
      end
    end
    // synthesis translate_on

    initial begin
        bank_id   = {`NTT_BANK_BITS{1'b0}};
        in_valid  = 1'b0;
        in_data   = {WIDE{1'b0}};
        tile_line_ctr = 0;

        @(posedge rst_n);
        @(posedge clk);

        for (n = 0; n < NUM_LINES; n = n + 1) begin
            @(posedge clk);
            if (INSERT_TILE_BUBBLE && (tile_line_ctr == 0)) begin
                in_valid <= 1'b0;
                in_data  <= {WIDE{1'b0}};
                tile_line_ctr <= tile_line_ctr;
            end else begin
                in_valid <= 1'b1;

                // ---------- ?? ?? (??/??) + q-??? ----------
                if (USE_FILE_STIM && (n < 16)) begin
                    normalize_bus_inplace(file_stim[n], in_data);
                end else begin
                    gen_next_input(tmp_bus);                  // per-lane LFSR
                    normalize_bus_inplace(tmp_bus, in_data);  // q-???
                end
                // ------------------------------------------------------

                $fdisplay(fh_in, "%h", in_data);  // ?? ??
                c_in <= c_in + 1;

                tile_line_ctr <= tile_line_ctr + 1;
                if (tile_line_ctr == (TILE_LEN-1)) tile_line_ctr <= 0;
            end
        end

        @(posedge clk);
        in_valid <= 1'b0;
        in_data  <= {WIDE{1'b0}};

        repeat (1024) @(posedge clk);

        $display("[TB] COUNT: in=%0d col=%0d trp=%0d sof=%0d out=%0d",
                 c_in, c_col, c_trp, c_sof, c_row);

        // ---- latency summary (?? ?? ??) ----
        if (got_in0 && got_row0) begin
          L_col   = cyc_col0 - cyc_in0;   // NTT16(col) ??
          L_trp   = cyc_trp0 - cyc_col0;  // transpose ??
          L_sof   = cyc_sof0 - cyc_trp0;  // SoF ??
          L_row   = cyc_row0 - cyc_sof0;  // NTT16(row) ??
          L_total = cyc_row0 - cyc_in0;   // ?? ? ????

          $display("[LAT] first in_valid @%0d",  cyc_in0);
          $display("[LAT] first col_valid @%0d  (L_col=%0d)",  cyc_col0, L_col);
          $display("[LAT] first trp_valid @%0d  (L_trp=%0d)",  cyc_trp0, L_trp);
          $display("[LAT] first sof_valid @%0d  (L_sof=%0d)",  cyc_sof0, L_sof);
          $display("[LAT] first out_valid @%0d  (L_row=%0d)",  cyc_row0, L_row);
          $display("[LAT] >>> TOTAL first-out latency L = %0d cycles", L_total);
          $display("[LAT] >>> TOTAL to finish 4096 lines ? L + 4096 = %0d cycles (no bubbles)", L_total + 4096);
          if (INSERT_TILE_BUBBLE)
            $display("[LAT] (+%0d extra cycles from input tile bubbles)", 256);
        end else begin
          $display("[LAT][WARN] could not measure latency (missing first valid flags).");
        end
        // -----------------------------------------

        $fclose(fh_in);
        $fclose(fh_col);
        $fclose(fh_trp);
        $fclose(fh_sof);
        $fclose(fh_row);
        $fclose(fh_tiles);
        $stop;
    end

    // ------------------------------------------------------------------------
    // Dumpers
    // ------------------------------------------------------------------------
    // COL NTT
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_col <= 0;
        end else if (v_col) begin
            $fdisplay(fh_col, "%h", d_col);
            c_col <= c_col + 1;
        end
    end

    // TRANSPOSE + ?? ?? ?? (TB ??? ?? ???)
    reg [3:0] trp_pos_tb;
    reg [3:0] tile_c_tb;
    reg [3:0] tile_r_tb;

    initial begin
        trp_pos_tb = 4'd0;
        tile_c_tb  = 4'd0;
        tile_r_tb  = 4'd0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_trp      <= 0;
            trp_pos_tb <= 4'd0;
            tile_c_tb  <= 4'd0;
            tile_r_tb  <= 4'd0;
        end else begin
            if (v_trp) begin
                if (trp_pos_tb == 4'd0) begin
                    $fdisplay(fh_tiles, "%0d %0d", tile_c_tb, tile_r_tb);
                end
                $fdisplay(fh_trp, "%h", d_trp);
                c_trp <= c_trp + 1;

                if (trp_pos_tb == 4'd15) begin
                    trp_pos_tb <= 4'd0;
                    if (tile_c_tb == 4'd15) begin
                        tile_c_tb <= 4'd0;
                        tile_r_tb <= tile_r_tb + 1'b1;
                    end else begin
                        tile_c_tb <= tile_c_tb + 1'b1;
                    end
                end else begin
                    trp_pos_tb <= trp_pos_tb + 1'b1;
                end
            end
        end
    end

    // SOF
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_sof <= 0;
        end else if (v_sof) begin
            $fdisplay(fh_sof, "%h", d_sof);
            c_sof <= c_sof + 1;
        end
    end

    // ROW NTT
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_row <= 0;
        end else if (out_valid) begin
            $fdisplay(fh_row, "%h", out_data);
            c_row <= c_row + 1;
        end
    end

    // ------------------------------------------------------------------------
    // Timeout guard (??? ???)
    // ------------------------------------------------------------------------
    initial begin
        cyc = 0;
        forever begin
            @(posedge clk);
            cyc = cyc + 1;
            if (cyc > (NUM_LINES*20)) begin
                $display("[TB][TIMEOUT] after %0d cycles  in=%0d col=%0d trp=%0d sof=%0d out=%0d",
                         cyc, c_in, c_col, c_trp, c_sof, c_row);
                $stop;
            end
        end
    end

endmodule

`default_nettype wire

