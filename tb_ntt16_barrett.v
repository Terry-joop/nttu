// ============================================================================
// tb_ntt16_barrett.v  ·  Version v1.3 (lane-dump)
// Verilog-2001 testbench for ntt16_buscore (Barrett, DIF).
//
// - Adds lane-level dump control via +lanedbg=0|1|2
//   0: off (default), 1: dump on mismatch, 2: dump every frame.
// - Each lane is WORD_WIDTH bits (e.g., 60b) and there are 16 lanes.
//
// Other behavior stays the same as v1.2e:
//   ? Drain between levels  ? 1-cycle prime bubble on re-entry
//   ? Scoreboard latency = 4*BARRETT_LAT
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module tb_ntt16_barrett;

    // Parameters from header
    localparam integer WORD_WIDTH   = `NTT_WORD_WIDTH;
    localparam integer BARRETT_LAT  = `NTT_BARRETT_LAT;
    localparam integer LANES        = `LANES;
    localparam integer BUSW         = WORD_WIDTH * LANES;
    localparam integer CORE_LATENCY = 4 * BARRETT_LAT;

    // Defaults (overridable via +args)
    integer bank_lo_default;
    integer bank_hi_default;
    integer frames_per_level_default;
    reg [8*256-1:0] basedir_str_default;
    integer reverse_lanes_default;
    integer lanedbg_default;

    // Resolved knobs
    integer BANK_LO;
    integer BANK_HI;
    integer FRAMES_PER_LEVEL;
    reg [8*256-1:0] BASEDIR;
    integer TB_REVERSE_LANES;
    integer LANE_DEBUG_MODE; // 0:off, 1:mismatch, 2:always

    // +args temps
    integer plus_tmp_i;
    reg [8*256-1:0] plus_tmp_s;

    // DUT I/O
    reg                        clk;
    reg                        rst_n;
    reg                        s_valid;
    reg  [`NTT_BANK_BITS-1:0]  bank_id;
    reg  [BUSW-1:0]            s_data;

    wire                       m_valid;
    wire [BUSW-1:0]            m_data;

    // DUT
    ntt16_buscore #(
        .WORD_WIDTH (WORD_WIDTH),
        .BARRETT_LAT(BARRETT_LAT)

    ) dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .s_valid (s_valid),
        .s_data  (s_data),
        .bank_id (bank_id),
        .m_valid (m_valid),
        .m_data  (m_data)
    );

    // Clock / Reset
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk; // 100 MHz
    end

    initial begin
        rst_n   = 1'b0;
        s_valid = 1'b0;
        bank_id = {`NTT_BANK_BITS{1'b0}};
        s_data  = {BUSW{1'b0}};
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
    end

    // Lane order helper (optional reversal)
    function [BUSW-1:0] lane_reverse;
        input [BUSW-1:0] in_bus;
        integer li;
        reg [BUSW-1:0] tmp;
    begin
        tmp = {BUSW{1'b0}};
        for (li = 0; li < LANES; li = li + 1) begin
            tmp[(WORD_WIDTH*li) +: WORD_WIDTH] =
                in_bus[(WORD_WIDTH*(LANES-1-li)) +: WORD_WIDTH];
        end
        lane_reverse = tmp;
    end
    endfunction

    // Helper: slice a lane
    function [WORD_WIDTH-1:0] lane_at;
        input [BUSW-1:0] bus;
        input integer idx;
    begin
        lane_at = bus[(WORD_WIDTH*idx) +: WORD_WIDTH];
    end
    endfunction

    // ---- Scoreboard / latency alignment ----
    reg [BUSW-1:0] exp_pipe [0:CORE_LATENCY-1];
    reg            val_pipe [0:CORE_LATENCY-1];
    reg [15:0]     lvl_pipe [0:CORE_LATENCY-1];
    reg [7:0]      frm_pipe [0:CORE_LATENCY-1];

    reg [BUSW-1:0] exp_in_bus;
    reg            exp_in_valid;
    reg [15:0]     exp_in_level;
    reg [7:0]      exp_in_frame;

    integer i_sb;
    integer total_errors;
    integer level_errors;
    integer total_frames_seen;

    // Lane-dump task
    task dump_lanes;
        input integer lvl_i;
        input integer frm_i;
        input [BUSW-1:0] got_bus;
        input [BUSW-1:0] exp_bus;
        integer li;
        reg [WORD_WIDTH-1:0] g, e;
        reg [8*16-1:0] tag;
    begin
        $display("  ---- LANE DETAIL (level%0d frame%0d) ----", lvl_i, frm_i);
        for (li = 0; li < LANES; li = li + 1) begin
            g = lane_at(got_bus, li);
            e = lane_at(exp_bus, li);
            tag = (g === e) ? "OK" : "MISMATCH";
            // WORD_WIDTH(60b)+ friendly, hex width left unconstrained (%h)
            $display("    lane%0d  exp=%h  got=%h  %0s", li, e, g, tag);
        end
    end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i_sb = 0; i_sb < CORE_LATENCY; i_sb = i_sb + 1) begin
                exp_pipe[i_sb] <= {BUSW{1'b0}};
                val_pipe[i_sb] <= 1'b0;
                lvl_pipe[i_sb] <= 16'd0;
                frm_pipe[i_sb] <= 8'd0;
            end
            total_errors      <= 0;
            total_frames_seen <= 0;
            level_errors      <= 0;
        end else begin
            // shift
            for (i_sb = CORE_LATENCY-1; i_sb > 0; i_sb = i_sb - 1) begin
                exp_pipe[i_sb] <= exp_pipe[i_sb-1];
                val_pipe[i_sb] <= val_pipe[i_sb-1];
                lvl_pipe[i_sb] <= lvl_pipe[i_sb-1];
                frm_pipe[i_sb] <= frm_pipe[i_sb-1];
            end
            // head insert
            exp_pipe[0] <= exp_in_bus;
            val_pipe[0] <= exp_in_valid;
            lvl_pipe[0] <= exp_in_level;
            frm_pipe[0] <= exp_in_frame;

            // compare
            if (m_valid && val_pipe[CORE_LATENCY-1]) begin
                total_frames_seen <= total_frames_seen + 1;

                if (m_data !== exp_pipe[CORE_LATENCY-1]) begin
                    total_errors <= total_errors + 1;
                    level_errors <= level_errors + 1;
                    $display("MISMATCH level%0d frame%0d",
                             lvl_pipe[CORE_LATENCY-1],
                             frm_pipe[CORE_LATENCY-1]);
                    $display("  EXP(full)=%h", exp_pipe[CORE_LATENCY-1]);
                    $display("  GOT(full)=%h", m_data);

                    if (LANE_DEBUG_MODE >= 1)
                        dump_lanes(lvl_pipe[CORE_LATENCY-1],
                                   frm_pipe[CORE_LATENCY-1],
                                   m_data, exp_pipe[CORE_LATENCY-1]);
                end else if (LANE_DEBUG_MODE >= 2) begin
                    // Verbose lane OK dump for every good frame
                    dump_lanes(lvl_pipe[CORE_LATENCY-1],
                               frm_pipe[CORE_LATENCY-1],
                               m_data, exp_pipe[CORE_LATENCY-1]);
                end
            end
        end
    end

    // ---- Inflight counter (frames in flight) ----
    integer inflight;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) inflight <= 0;
        else begin
            if (s_valid && !m_valid) inflight <= inflight + 1;
            else if (!s_valid && m_valid) inflight <= inflight - 1;
            else inflight <= inflight; // 00 or 11
        end
    end

    // ------------- Driver / I/O -------------
    integer lvl, frm;
    integer stim_fd, exp_fd;

    reg [8*256-1:0] stim_path;
    reg [8*256-1:0] exp_path;

    integer r_stim, r_exp;

    reg [BUSW-1:0] stim_frame_raw;
    reg [BUSW-1:0] exp_frame_raw;
    reg [BUSW-1:0] stim_frame_eff;
    reg [BUSW-1:0] exp_frame_eff;

    // small helper: exactly 2 ASCII digits
    function [15:0] itoa2;
        input integer val;
        integer d1, d0;
        reg [15:0] out16;
    begin
        d1    = (val/10)%10;
        d0    = val%10;
        out16 = {8'd48 + d1[7:0], 8'd48 + d0[7:0]};
        itoa2 = out16;
    end
    endfunction

    initial begin
        // defaults
        bank_lo_default           = 0;
        bank_hi_default           = `NTT_NUM_BANKS - 1;
        frames_per_level_default  = 16;
        basedir_str_default       = "ntt16_data/ntt16_out";
        reverse_lanes_default     = 0;
        lanedbg_default           = 1; // default: dump lanes on mismatch

        BANK_LO          = bank_lo_default;
        BANK_HI          = bank_hi_default;
        FRAMES_PER_LEVEL = frames_per_level_default;
        BASEDIR          = basedir_str_default;
        TB_REVERSE_LANES = reverse_lanes_default;
        LANE_DEBUG_MODE  = lanedbg_default;

        // +args
        if ($value$plusargs("bank_lo=%d", plus_tmp_i)) BANK_LO = plus_tmp_i;
        if ($value$plusargs("bank_hi=%d", plus_tmp_i)) BANK_HI = plus_tmp_i;
        if ($value$plusargs("frames=%d",  plus_tmp_i)) FRAMES_PER_LEVEL = plus_tmp_i;
        if ($value$plusargs("revlanes=%d", plus_tmp_i)) TB_REVERSE_LANES = plus_tmp_i;
        if ($value$plusargs("basedir=%s", plus_tmp_s))  BASEDIR = plus_tmp_s;
        if ($value$plusargs("lanedbg=%d", plus_tmp_i))  LANE_DEBUG_MODE = plus_tmp_i;

        // >>> ?? ??? ???? +bank=N ??
        if ($value$plusargs("bank=%d", plus_tmp_i)) begin
            BANK_LO = plus_tmp_i;
            BANK_HI = plus_tmp_i;
        end

        // ?? ??
        if (BANK_LO < 0) BANK_LO = 0;
        if (BANK_HI > `NTT_NUM_BANKS-1) BANK_HI = `NTT_NUM_BANKS-1;
        if (BANK_LO > BANK_HI) BANK_LO = BANK_HI;

        @(posedge rst_n);
        @(posedge clk);

        $display("TB start: WORD_WIDTH=%0d BARRETT_LAT=%0d TotalLatency=%0d",
                 WORD_WIDTH, BARRETT_LAT, CORE_LATENCY);
        $display("TB config: banks [%0d..%0d] frames/level=%0d basedir=%0s revlanes=%0d lanedbg=%0d",
                 BANK_LO, BANK_HI, FRAMES_PER_LEVEL, BASEDIR, TB_REVERSE_LANES, LANE_DEBUG_MODE);

        // clear head insert for scoreboard
        exp_in_bus   = {BUSW{1'b0}};
        exp_in_valid = 1'b0;
        exp_in_level = 16'd0;
        exp_in_frame = 8'd0;

        // ---- Loop banks ----
        for (lvl = BANK_LO; lvl <= BANK_HI; lvl = lvl + 1) begin
            stim_path = {BASEDIR, "/level", itoa2(lvl), "/ntt16_stim.hex"};
            exp_path  = {BASEDIR, "/level", itoa2(lvl), "/ntt16_exp_bitrev.hex"};

            stim_fd = $fopen(stim_path, "r");
            exp_fd  = $fopen(exp_path,  "r");

            if (stim_fd == 0) begin
                $display("ERROR: cannot open STIM file %0s", stim_path);
                $stop;
            end
            if (exp_fd == 0) begin
                $display("ERROR: cannot open EXP file  %0s", exp_path);
                $stop;
            end

            // ***** ENFORCE EMPTY BEFORE BANK SWITCH *****
            while (inflight != 0) @(posedge clk);

            level_errors = 0;
            $display("INFO: level%02d frames=%0d", lvl, FRAMES_PER_LEVEL);

            // Prime bubble (1 cycle, bank_id stable, s_valid=0)
            s_valid      = 1'b0;
            bank_id      = lvl[`NTT_BANK_BITS-1:0];
            exp_in_valid = 1'b0;
            @(posedge clk);

            // Stream frames
            for (frm = 0; frm < FRAMES_PER_LEVEL; frm = frm + 1) begin
                r_stim = $fscanf(stim_fd, "%h\n", stim_frame_raw);
                r_exp  = $fscanf(exp_fd,  "%h\n", exp_frame_raw);
                if (r_stim != 1) begin
                    $display("ERROR: short read in %0s at frame %0d", stim_path, frm);
                    $stop;
                end
                if (r_exp != 1) begin
                    $display("ERROR: short read in %0s at frame %0d", exp_path, frm);
                    $stop;
                end

                if (TB_REVERSE_LANES != 0) begin
                    stim_frame_eff = lane_reverse(stim_frame_raw);
                    exp_frame_eff  = lane_reverse(exp_frame_raw);
                end else begin
                    stim_frame_eff = stim_frame_raw;
                    exp_frame_eff  = exp_frame_raw;
                end

                s_data       = stim_frame_eff;
                s_valid      = 1'b1;
                bank_id      = lvl[`NTT_BANK_BITS-1:0];

                exp_in_bus   = exp_frame_eff;
                exp_in_valid = 1'b1;
                exp_in_level = lvl[15:0];
                exp_in_frame = frm[7:0];

                @(posedge clk);
                exp_in_valid = 1'b0; // enqueue strobe 1-cycle
            end

            // Stop feeding this level
            s_valid = 1'b0;
            @(posedge clk);

            // ***** DRAIN REMAINING OUTPUTS FOR THIS LEVEL *****
            while (inflight != 0) @(posedge clk);

            // Close files
            $fclose(stim_fd);
            $fclose(exp_fd);

            if (level_errors == 0)
                $display("PASS: level%02d  errors=0 / frames=%0d", lvl, FRAMES_PER_LEVEL);
            else
                $display("FAIL: level%02d  errors=%0d / frames=%0d", lvl, level_errors, FRAMES_PER_LEVEL);
        end

        // Small grace period
        repeat (2) @(posedge clk);

        $display("TOTAL FAILS = %0d", total_errors);
        $stop;
    end

endmodule

`default_nettype wire

