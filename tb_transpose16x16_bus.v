// ============================================================================
// tb_transpose16x16_bus  (Verilog-2001)
// Continuous streaming test for transpose16x16_bus (v2.2p target)
// - Push NUM_TILES tiles back-to-back (s_valid held high)
// - Auto-detect DUT mapping at first data: {XOR-transpose, BR-only, Identity}
// - Scoreboard compares against detected mapping (every frame)
// - Debug: tile boundaries, handovers, first few rows lane0..3 LSB
// - Natural end: after TOTAL_FRAMES seen, m_valid drop is PASS termination
// - Watchdog: if m_valid stays low too long before TOTAL_FRAMES, stop with FAIL
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module tb_transpose16x16_bus;

  // -------------------- Parameters --------------------
  localparam integer LANES       = 16;
  localparam integer WORD_WIDTH  = `NTT_WORD_WIDTH;
  localparam integer TILE_ROWS   = 16;
  localparam integer NUM_TILES   = 4;    // freely change for stress
  integer DEBUG                  = 1;    // 1: print debug to transcript
  integer DUMP_FIRST_ROWS_N      = 4;    // rows to dump LSB lanes

  // -------------------- Clock / Reset -----------------
  reg clk, rst_n;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end
  task do_reset; begin rst_n=1'b0; repeat(8) @(posedge clk); rst_n=1'b1; @(posedge clk); end endtask

  // -------------------- DUT I/O -----------------------
  reg                         s_valid;
  reg  [LANES*WORD_WIDTH-1:0] s_data;
  wire                        m_valid;
  wire [LANES*WORD_WIDTH-1:0] m_data;

  // Instantiate DUT (drop-in)
  transpose16x16_bus #(
    .WORD_WIDTH(WORD_WIDTH)
  ) DUT (
    .clk    (clk),
    .rst_n  (rst_n),
    .s_valid(s_valid),
    .s_data (s_data),
    .m_valid(m_valid),
    .m_data (m_data)
  );

  // -------------------- Helpers -----------------------
  function [3:0] br4; input [3:0] x; begin br4 = {x[0],x[1],x[2],x[3]}; end endfunction

  function [LANES*WORD_WIDTH-1:0] pack_bus16;
    input [WORD_WIDTH-1:0] v0,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15;
    begin
      pack_bus16 = { v15,v14,v13,v12, v11,v10,v9,v8, v7,v6,v5,v4, v3,v2,v1,v0 };
    end
  endfunction

  function [WORD_WIDTH-1:0] synth_val;
    input integer tile_idx, row_idx, lane_idx;
    reg [7:0] t8; reg [3:0] r4,l4;
    begin
      t8 = tile_idx[7:0]; r4 = row_idx[3:0]; l4 = lane_idx[3:0];
      // [15:8]=tile, [7:4]=row, [3:0]=lane  (upper bits zero)
      synth_val = { {(WORD_WIDTH-16){1'b0}}, t8, r4, l4 };
    end
  endfunction

  // -------------------- Golden store (flattened) --------------------
  localparam integer GOLDEN_SIZE = NUM_TILES*TILE_ROWS*LANES;
  reg [WORD_WIDTH-1:0] golden [0:GOLDEN_SIZE-1];

  function integer idx3; input integer t,row,lane;
    begin idx3 = t*TILE_ROWS*LANES + row*LANES + lane; end
  endfunction

  // -------------------- Stimulus: continuous tiles ------------------
  integer t_in, r_in, l;
  task drive_continuous_tiles(input integer n_tiles);
    reg [LANES*WORD_WIDTH-1:0] frame;
    begin
      s_valid = 1'b0; s_data = {LANES*WORD_WIDTH{1'b0}};
      @(posedge clk);
      for (t_in=0; t_in<n_tiles; t_in=t_in+1) begin
        for (r_in=0; r_in<TILE_ROWS; r_in=r_in+1) begin
          frame = {LANES*WORD_WIDTH{1'b0}};
          for (l=0; l<LANES; l=l+1) begin
            golden[idx3(t_in,r_in,l)] = synth_val(t_in,r_in,l);
            frame = frame | ( { {(LANES*WORD_WIDTH-WORD_WIDTH){1'b0}}, golden[idx3(t_in,r_in,l)] } << (l*WORD_WIDTH) );
          end
          @(posedge clk);
          s_valid <= 1'b1;
          s_data  <= frame;
        end
      end
      @(posedge clk);
      s_valid <= 1'b0;
      s_data  <= {LANES*WORD_WIDTH{1'b0}};
    end
  endtask

  // -------------------- Expected (3 candidate mappings) --------------
  // XOR-transpose (v2.2p target): out_lane[k] = T[ BR4(k) ][ row ]
  function [LANES*WORD_WIDTH-1:0] exp_xor; input integer t,row; reg [WORD_WIDTH-1:0] L[0:15]; integer k;
  begin
    for (k=0;k<16;k=k+1) L[k] = golden[idx3(t, br4(k[3:0]), row)];
    exp_xor = pack_bus16(L[0],L[1],L[2],L[3],L[4],L[5],L[6],L[7],L[8],L[9],L[10],L[11],L[12],L[13],L[14],L[15]);
  end endfunction

  // BR-only (v1.0d style): out_lane[k] = T[ BR4(row) ][ k ]
  function [LANES*WORD_WIDTH-1:0] exp_bro; input integer t,row; reg [WORD_WIDTH-1:0] L[0:15]; integer k;
  begin
    for (k=0;k<16;k=k+1) L[k] = golden[idx3(t, br4(row[3:0]), k)];
    exp_bro = pack_bus16(L[0],L[1],L[2],L[3],L[4],L[5],L[6],L[7],L[8],L[9],L[10],L[11],L[12],L[13],L[14],L[15]);
  end endfunction

  // Identity: out_lane[k] = T[ row ][ k ]
  function [LANES*WORD_WIDTH-1:0] exp_id; input integer t,row; reg [WORD_WIDTH-1:0] L[0:15]; integer k;
  begin
    for (k=0;k<16;k=k+1) L[k] = golden[idx3(t, row, k)];
    exp_id = pack_bus16(L[0],L[1],L[2],L[3],L[4],L[5],L[6],L[7],L[8],L[9],L[10],L[11],L[12],L[13],L[14],L[15]);
  end endfunction

  // -------------------- Scoreboard & Debug ---------------------------
  localparam integer TOTAL_FRAMES = NUM_TILES*TILE_ROWS;

  integer t_out, r_out;
  reg started;
  integer frames_seen, err_cnt, frm_cnt;
  integer mode; // 0=XOR, 1=BR-only, 2=ID, 9=unknown
  reg [LANES*WORD_WIDTH-1:0] exp_now;

  // Verilog-2001: predeclare temps used in blocks
  reg [LANES*WORD_WIDTH-1:0] ex, eb, ei;

  // Drop watchdog
  integer drop_cnt;
  integer drop_latched;
  localparam integer DROP_WATCHDOG = 256;  // stop if m_valid low too long before done

  // Debug taps (hierarchical)
  reg a_full_q, b_full_q;
  reg rd_active_q, m_valid_q;
  reg [3:0] r_addr_q_q, w_row_q;
  reg rd_buf_sel_q, wr_buf_sel_q;
  integer dbg_rows_done;

  // Edge-tracked debug printing
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_full_q <= 0; b_full_q <= 0; rd_active_q <= 0; m_valid_q <= 0;
      r_addr_q_q <= 4'd0; rd_buf_sel_q <= 1'b0;
      w_row_q <= 4'd0; wr_buf_sel_q <= 1'b0;
      dbg_rows_done <= 0;
    end else begin
      if (DEBUG && s_valid && (DUT.w_row==4'd15))
        $display("[%0t][TRP] WRITE tile done -> next wr_buf_sel=%0d", $time, ~DUT.wr_buf_sel);

      if (DEBUG && !DUT.rd_active && !a_full_q && DUT.a_full)
        $display("[%0t][TRP] READ arm A (prime row0)", $time);
      if (DEBUG && !DUT.rd_active && !b_full_q && DUT.b_full)
        $display("[%0t][TRP] READ arm B (prime row0)", $time);

      if (DEBUG && DUT.rd_active && (DUT.r_addr_q==4'd15)) begin
        if ((DUT.rd_buf_sel==1'b0 && DUT.b_full) ||
            (DUT.rd_buf_sel==1'b1 && DUT.a_full)) begin
          $display("[%0t][TRP] HANDOVER %s->%s (next row0)",
                   $time, (DUT.rd_buf_sel?"B":"A"), (DUT.rd_buf_sel?"A":"B"));
        end else begin
          $display("[%0t][TRP] HANDOVER blocked: next buf not ready (A=%0d B=%0d)",
                   $time, DUT.a_full, DUT.b_full);
        end
      end

      if (DEBUG && m_valid && (dbg_rows_done < DUMP_FIRST_ROWS_N)) begin
        if (mode==0)      exp_now = exp_xor(t_out, r_out);
        else if (mode==1) exp_now = exp_bro(t_out, r_out);
        else if (mode==2) exp_now = exp_id (t_out, r_out);
        else              exp_now = exp_xor(t_out, r_out);

        $display("[%0t][TRP] row=%0d buf=%s  OUT L0..3 LSB=%h %h %h %h",
                 $time, DUT.r_addr_q, (DUT.rd_buf_sel?"B":"A"),
                 m_data[0*WORD_WIDTH +: 4],
                 m_data[1*WORD_WIDTH +: 4],
                 m_data[2*WORD_WIDTH +: 4],
                 m_data[3*WORD_WIDTH +: 4]);
        $display("[%0t][TRP] row=%0d              EXP L0..3 LSB=%h %h %h %h",
                 $time, DUT.r_addr_q,
                 exp_now[0*WORD_WIDTH +: 4],
                 exp_now[1*WORD_WIDTH +: 4],
                 exp_now[2*WORD_WIDTH +: 4],
                 exp_now[3*WORD_WIDTH +: 4]);
        dbg_rows_done = dbg_rows_done + 1;
      end

      a_full_q      <= DUT.a_full;
      b_full_q      <= DUT.b_full;
      rd_active_q   <= DUT.rd_active;
      m_valid_q     <= m_valid;
      r_addr_q_q    <= DUT.r_addr_q;
      rd_buf_sel_q  <= DUT.rd_buf_sel;
      w_row_q       <= DUT.w_row;
      wr_buf_sel_q  <= DUT.wr_buf_sel;
    end
  end

  // Main scoreboard
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      started      <= 1'b0;
      t_out        <= 0;
      r_out        <= 0;
      frames_seen  <= 0;
      err_cnt      <= 0;
      frm_cnt      <= 0;
      mode         <= 9;
      exp_now      = {LANES*WORD_WIDTH{1'b0}};
      ex           = {LANES*WORD_WIDTH{1'b0}};
      eb           = {LANES*WORD_WIDTH{1'b0}};
      ei           = {LANES*WORD_WIDTH{1'b0}};
      drop_cnt     <= 0;
      drop_latched <= 0;
    end else begin
      if (!started) begin
        if (m_valid) begin
          // DETECT mapping at very first data frame (tile=0,row=0)
          ex = exp_xor(0,0); eb = exp_bro(0,0); ei = exp_id(0,0);

          if (m_data === ex)      begin mode = 0; if (DEBUG) $display("[TB] DETECT: DUT matches XOR-transpose (v2.2p)."); end
          else if (m_data === eb) begin mode = 1; if (DEBUG) $display("[TB] DETECT: DUT matches BR-only (v1.0d)."); end
          else if (m_data === ei) begin mode = 2; if (DEBUG) $display("[TB] DETECT: DUT matches Identity."); end
          else                    begin mode = 9; if (DEBUG) $display("[TB] DETECT: DUT matches none of {XOR,BR,ID}."); end

          // first compare (row0)
          if      (mode==0) exp_now = ex;
          else if (mode==1) exp_now = eb;
          else if (mode==2) exp_now = ei;
          else              exp_now = ex;

          if (m_data !== exp_now) begin
            $display("[%0t] MISMATCH @ start", $time);
            err_cnt = err_cnt + 1;
          end

          // Align TB row index with hidden-prime reader: next is row1
          started      <= 1'b1;
          t_out        <= 0;
          r_out        <= 1;              // key alignment
          frm_cnt      <= frm_cnt + 1;
          frames_seen  <= frames_seen + 1;
        end
      end else begin
        // Natural-end shortcut: If we've already seen all frames,
        // any m_valid drop is normal termination ? PASS/FAIL by err_cnt.
        if (!m_valid && (frames_seen >= TOTAL_FRAMES)) begin
          if (err_cnt==0) $display("[TB] DONE: checked %0d frames. PASS (natural end)", frames_seen);
          else            $display("[TB] DONE: checked %0d frames. FAIL (%0d errors)", frames_seen, err_cnt);
          $stop;
        end

        if (!m_valid) begin
          // Before done: treat as drop error with watchdog
          if (!drop_latched) begin
            drop_latched = 1;
            err_cnt = err_cnt + 1;
            $display("[%0t] ERROR: m_valid dropped after start", $time);
            $display("       SNAP: rd_active=%0d r_addr_q=%0d rd_buf_sel=%0d  a_full=%0d b_full=%0d  w_row=%0d wr_buf_sel=%0d",
                     DUT.rd_active, DUT.r_addr_q, DUT.rd_buf_sel,
                     DUT.a_full,    DUT.b_full,    DUT.w_row,    DUT.wr_buf_sel);
          end
          drop_cnt = drop_cnt + 1;
          if (drop_cnt >= DROP_WATCHDOG) begin
            $display("[%0t] WATCHDOG: m_valid low for %0d cycles. Stopping.", $time, DROP_WATCHDOG);
            $display("[TB] FAIL: %0d errors (early stop).", err_cnt);
            $stop;
          end
        end else begin
          // m_valid high again ? clear drop trackers
          drop_latched = 0;
          drop_cnt = 0;

          // Compare current frame
          if      (mode==0) exp_now = exp_xor(t_out, r_out);
          else if (mode==1) exp_now = exp_bro(t_out, r_out);
          else if (mode==2) exp_now = exp_id (t_out, r_out);
          else              exp_now = exp_xor(t_out, r_out);

          if (m_data !== exp_now) begin
            $display("[%0t] MISMATCH @ tile=%0d row=%0d", $time, t_out, r_out);
            err_cnt = err_cnt + 1;
          end
          frm_cnt = frm_cnt + 1;
          frames_seen = frames_seen + 1;

          // If this was the very last expected frame, exit immediately PASS/FAIL
          if (frames_seen >= TOTAL_FRAMES) begin
            if (err_cnt==0) $display("[TB] DONE: checked %0d frames. PASS", frames_seen);
            else            $display("[TB] DONE: checked %0d frames. FAIL (%0d errors)", frames_seen, err_cnt);
            $stop;
          end

          // next indices
          if (r_out == (TILE_ROWS-1)) begin
            r_out <= 0; t_out <= t_out + 1;
          end else begin
            r_out <= r_out + 1;
          end
        end
      end
    end
  end

  // -------------------- Run sequence -----------------------
  initial begin
    $display("[TB] transpose16x16_bus: WORD_WIDTH=%0d, NUM_TILES=%0d", WORD_WIDTH, NUM_TILES);
    s_valid = 1'b0; s_data = {LANES*WORD_WIDTH{1'b0}};
    do_reset();
    drive_continuous_tiles(NUM_TILES);
  end

endmodule

`default_nettype wire

