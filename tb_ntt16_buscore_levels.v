// -----------------------------------------------------------------------------
// tb_ntt16_buscore_levels.v
// Testbench for ntt16_buscore (v2.3): multi-level sweep, frame-range select
// Verilog-2001 only. ModelSim 2020.1 compatible.
//
// Final Version: v1.1
//
// INTENDED OPERATION:
// This testbench is designed to accommodate the 1-cycle latency of the DUT's
// internal ROMs (for q, mu, and twiddles). When switching to a new test level
// (i.e., a new bank_id), the testbench performs the following sequence:
//
// 1. PRIME BUBBLE (1 cycle):
//    - It first drives the new `bank_id` onto the bus while keeping `s_valid` low.
//    - It then waits for exactly one clock cycle (`@(posedge clk)`).
//    - This single idle cycle allows the registered outputs of the ROMs inside
//      the DUT to update with the constants corresponding to the new `bank_id`.
//
// 2. DATA STREAMING:
//    - On the very next clock cycle after the prime bubble, the testbench
//      raises `s_valid` to 1 and begins sending the data frames.
//
// This ensures that when the first data frame enters the DUT's pipeline, the
// correct constants from the ROMs are already available, ensuring perfect
// timing alignment between data and constants.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

`include "ntt_params.vh"  // for `NTT_WORD_WIDTH, `NTT_BANK_BITS, `NTT_BARRETT_LAT

module tb_ntt16_buscore_levels;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam integer LANES        = 16;
    localparam integer WORD_W       = `NTT_WORD_WIDTH;  // e.g., 60
    localparam integer FRAMES_TOT   = 16;
    localparam integer LEVELS_TOT   = 30;
    localparam integer BANK_BITS    = `NTT_BANK_BITS;   // e.g., 5

    localparam integer CLK_NS_DFLT  = 10;               // 100 MHz default
    localparam [8*128-1:0] VEC_DIR_DFLT = "ntt16_data/ntt16_out";

    // -------------------------------------------------------------------------
    // Clock & reset
    // -------------------------------------------------------------------------
    reg clk;
    reg rst_n;

    integer CLK_NS;
    initial begin
        CLK_NS = CLK_NS_DFLT;
        if ($value$plusargs("CLK_NS=%d", CLK_NS)) begin
            $display("[TB] CLK_NS overridden to %0d", CLK_NS);
        end
    end

    initial begin
        clk = 1'b0;
        forever #(CLK_NS/2) clk = ~clk;
    end

    initial begin
        rst_n = 1'b0;
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
    end

    // -------------------------------------------------------------------------
    // Level/Frame selection via plusargs
    // -------------------------------------------------------------------------
    integer L0, L1, F0, F1;
    reg [8*128-1:0] VEC_DIR;

    initial begin
        L0 = 0;
        L1 = LEVELS_TOT-1;
        F0 = 0;
        F1 = FRAMES_TOT-1;
        VEC_DIR = VEC_DIR_DFLT;

        if ($value$plusargs("L0=%d", L0)) $display("[TB] L0=%0d", L0);
        if ($value$plusargs("L1=%d", L1)) $display("[TB] L1=%0d", L1);
        if ($value$plusargs("F0=%d", F0)) $display("[TB] F0=%0d", F0);
        if ($value$plusargs("F1=%d", F1)) $display("[TB] F1=%0d", F1);
        if ($value$plusargs("VEC_DIR=%s", VEC_DIR)) $display("[TB] VEC_DIR=%0s", VEC_DIR);

        if (L0 < 0) L0 = 0;
        if (L1 > LEVELS_TOT-1) L1 = LEVELS_TOT-1;
        if (L1 < L0) L1 = L0;

        if (F0 < 0) F0 = 0;
        if (F1 > FRAMES_TOT-1) F1 = FRAMES_TOT-1;
        if (F1 < F0) F1 = F0;

        $display("[TB] Using vector base dir: %0s", VEC_DIR);
    end

    // -------------------------------------------------------------------------
    // DUT I/O (matches ntt16_buscore v2.3 ports)
    // -------------------------------------------------------------------------
    reg                     s_valid;
    reg  [WORD_W*LANES-1:0] s_data;
    reg  [BANK_BITS-1:0]    bank_id;

    wire                    m_valid;
    wire [WORD_W*LANES-1:0] m_data;

    // -------------------------------------------------------------------------
    // DUT instance
    // -------------------------------------------------------------------------
    ntt16_buscore #(
        .WORD_WIDTH  (WORD_W),
        .BARRETT_LAT (`NTT_BARRETT_LAT)
    ) DUT (
        .clk      (clk),
        .rst_n    (rst_n),
        .s_valid  (s_valid),
        .s_data   (s_data),
        .bank_id  (bank_id),
        .m_valid  (m_valid),
        .m_data   (m_data)
    );

    // -------------------------------------------------------------------------
    // Vector memories (packed lines): 16×WORD_W per line
    // -------------------------------------------------------------------------
    localparam integer PACK_W = LANES * WORD_W;

    reg [PACK_W-1:0] stim_mem   [0:FRAMES_TOT-1];
    reg [PACK_W-1:0] expect_mem [0:FRAMES_TOT-1];

    // -------------------------------------------------------------------------
    // Load vectors for a given level
    // -------------------------------------------------------------------------
    task load_level_vectors;
        input integer lvl;
        reg   [8*256-1:0] stim_path;
        reg   [8*256-1:0] exp_path;
    begin
        $sformat(stim_path, "%0s/level%0d/ntt16_stim.hex",       VEC_DIR, lvl);
        $sformat(exp_path,  "%0s/level%0d/ntt16_exp_bitrev.hex", VEC_DIR, lvl);

        $display("[TB] Loading STIM: %0s", stim_path);
        $readmemh(stim_path,  stim_mem);

        $display("[TB] Loading GOLD: %0s", exp_path);
        $readmemh(exp_path,   expect_mem);
    end
    endtask

    // -------------------------------------------------------------------------
    // Scoreboard with lane-wise reporting
    // -------------------------------------------------------------------------
    integer err_count;

    task compare_line;
        input integer idx;
        integer li;
        reg [WORD_W-1:0] got_lane;
        reg [WORD_W-1:0] exp_lane;
        reg [WORD_W-1:0] xordiff;
        integer mismatch_lanes;
    begin
        mismatch_lanes = 0;

        if (m_data !== expect_mem[idx]) begin
            err_count = err_count + 1;
            $display("[MISMATCH] t=%0t ns level=%0d frame=%0d (packed mismatch)", 
                     $time, bank_id, idx);

            for (li = 0; li < LANES; li = li + 1) begin
                got_lane = m_data      [li*WORD_W +: WORD_W];
                exp_lane = expect_mem[idx][li*WORD_W +: WORD_W];
                if (got_lane !== exp_lane) begin
                    xordiff = got_lane ^ exp_lane;
                    $display("[LANE %0d] DUT=%0h EXP=%0h XOR=%0h", li, got_lane, exp_lane, xordiff);
                    mismatch_lanes = mismatch_lanes + 1;
                end
            end

            $display("[MISMATCH] level=%0d frame=%0d lanes_mismatched=%0d of %0d",
                     bank_id, idx, mismatch_lanes, LANES);
        end
    end
    endtask

    // -------------------------------------------------------------------------
    // Run one level (bank switch ? 1 idle cycle)
    // -------------------------------------------------------------------------
    task run_one_level;
        input integer lvl;
        integer ftx;
        integer frx;
    begin
        // --- Step 1: Prime Bubble (1 cycle) ---
        // Set the new bank_id and keep s_valid low.
        // Wait for exactly one cycle to allow the ROMs to prepare their values.
        bank_id  <= lvl[BANK_BITS-1:0];
        s_valid  <= 1'b0;
        s_data   <= {PACK_W{1'b0}};
        @(posedge clk); // <--- The single cycle wait for ROM latency compensation.

        // --- Step 2: Stream Frames ---
        // Starting from the next cycle, drive s_valid high and send data.
        for (ftx = F0; ftx <= F1; ftx = ftx + 1) begin
            s_data  <= stim_mem[ftx];
            s_valid <= 1'b1;
            // Wait for the next clock edge after driving the data for one cycle.
            @(posedge clk);
        end

        // --- Step 3: Stop Feeding ---
        // De-assert s_valid after all frames for the level have been sent.
        s_valid <= 1'b0;
        s_data  <= {PACK_W{1'b0}};

        // --- Step 4: Wait for and Check Outputs ---
        // Wait until all expected frames have been received and check them.
        frx = F0;
        while (frx <= F1) begin
            if (m_valid) begin
                compare_line(frx);
                frx = frx + 1;
            end
            // Continue waiting for the next clock edge even if m_valid is low.
            @(posedge clk);
        end
    end
    endtask

    // -------------------------------------------------------------------------
    // Main
    // -------------------------------------------------------------------------
    initial begin : MAIN
        integer lvl;
        integer total_frames;

        err_count    = 0;
        s_valid      = 1'b0;
        s_data       = {PACK_W{1'b0}};
        bank_id      = {BANK_BITS{1'b0}};
        total_frames = 0;

        @(posedge rst_n);
        @(posedge clk);

        for (lvl = L0; lvl <= L1; lvl = lvl + 1) begin
            $display("[TB] ========= RUN LEVEL %0d =========", lvl);
            load_level_vectors(lvl);
            run_one_level(lvl);
            total_frames = total_frames + (F1 - F0 + 1);
        end

        if (err_count == 0) begin
            $display("[TB] PASS. Levels %0d..%0d, Frames %0d..%0d, Total frames %0d",
                     L0, L1, F0, F1, total_frames);
        end else begin
            $display("[TB] FAIL with %0d mismatches. Levels %0d..%0d Frames %0d..%0d",
                     err_count, L0, L1, F0, F1);
        end

        $stop;
    end

endmodule

`default_nettype wire

