// ============================================================================
// tb_radix2_bfly_hex.v  (Verilog-2001 only)  v2.0  [ALL-STAGES SELF-CHECK]
// - radix2_butterfly_dif (v2.1 interface) ? s0/s1/s2 ?? ??? ??? ??
// - exp/got ?? ?? ??(?? ??) · define ???
// - 60-bit ????? (WORD_WIDTH=`NTT_WORD_WIDTH) · 64?? ?? ?? ??
// - ?? ??:
//     ntt16_data/q_table.hex
//     ntt16_data/mu_table.hex
//     ntt16_data/q_width_table.hex
//     ntt16_data/levels_s0_60_std.hex   (8/??)
//     ntt16_data/levels_s1_60_std.hex   (4/??)
//     ntt16_data/levels_s2_60_std.hex   (2/??)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module tb_radix2_bfly_hex;

    // ------------------------------------------------------------------------
    // Simulation parameters
    // ------------------------------------------------------------------------
    localparam integer CLK_HALF     = 5;    // 100 MHz
    localparam integer RST_CYC      = 5;
    localparam integer WORD_WIDTH   = `NTT_WORD_WIDTH;    // e.g., 60
    localparam integer BARRETT_LAT  = `NTT_BARRETT_LAT;   // e.g., 4

    localparam integer TOTAL_LVLS   = 30;
    localparam integer S0_PER_LVL   = 8;
    localparam integer S1_PER_LVL   = 4;
    localparam integer S2_PER_LVL   = 2;

    // Number of random cases per (level × set × mode)
    localparam integer RAND_VEC     = 32;

    // ------------------------------------------------------------------------
    // Clock / Reset
    // ------------------------------------------------------------------------
    reg clk = 1'b0;
    always #(CLK_HALF) clk = ~clk;

    reg rst_n = 1'b0;

    // ------------------------------------------------------------------------
    // ROMs loaded from hex files
    // ------------------------------------------------------------------------
    reg [WORD_WIDTH-1:0]   q_table      [0:TOTAL_LVLS-1];
    reg [2*WORD_WIDTH-1:0] mu_table     [0:TOTAL_LVLS-1];   // padded to 2W bits
    reg [7:0]              qwidth_table [0:TOTAL_LVLS-1];

    reg [WORD_WIDTH-1:0]   tw_s0 [0:TOTAL_LVLS*S0_PER_LVL-1]; // 8 per level
    reg [WORD_WIDTH-1:0]   tw_s1 [0:TOTAL_LVLS*S1_PER_LVL-1]; // 4 per level
    reg [WORD_WIDTH-1:0]   tw_s2 [0:TOTAL_LVLS*S2_PER_LVL-1]; // 2 per level

    // ------------------------------------------------------------------------
    // DUT #1 : LAST_STAGE = 1 (bypass multiplier; w ignored)
    // ------------------------------------------------------------------------
    reg                       vld_in_b;
    reg  [WORD_WIDTH-1:0]     A_in_b, B_in_b, W_in_b, q_in_b;
    reg  [2*WORD_WIDTH-1:0]   mu_in_b;
    reg  [5:0]                qw_in_b;
    wire                      vld_out_b;
    wire [WORD_WIDTH-1:0]     A_out_b, B_out_b;

    radix2_butterfly_dif #(
        .WORD_WIDTH (WORD_WIDTH),
        .BARRETT_LAT(BARRETT_LAT),
        .LAST_STAGE (1'b1)
    ) dut_bypass (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (vld_in_b),
        .A_in       (A_in_b),
        .B_in       (B_in_b),
        .w          (W_in_b),
        .q          (q_in_b),
        .mu         (mu_in_b),
        .q_width    (qw_in_b),
        .valid_out  (vld_out_b),
        .A_out      (A_out_b),
        .B_out      (B_out_b)
    );

    // ------------------------------------------------------------------------
    // DUT #2 : LAST_STAGE = 0 (normal multiply by w)
    // ------------------------------------------------------------------------
    reg                       vld_in_n;
    reg  [WORD_WIDTH-1:0]     A_in_n, B_in_n, W_in_n, q_in_n;
    reg  [2*WORD_WIDTH-1:0]   mu_in_n;
    reg  [5:0]                qw_in_n;
    wire                      vld_out_n;
    wire [WORD_WIDTH-1:0]     A_out_n, B_out_n;

    radix2_butterfly_dif #(
        .WORD_WIDTH (WORD_WIDTH),
        .BARRETT_LAT(BARRETT_LAT),
        .LAST_STAGE (1'b0)
    ) dut_normal (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (vld_in_n),
        .A_in       (A_in_n),
        .B_in       (B_in_n),
        .w          (W_in_n),
        .q          (q_in_n),
        .mu         (mu_in_n),
        .q_width    (qw_in_n),
        .valid_out  (vld_out_n),
        .A_out      (A_out_n),
        .B_out      (B_out_n)
    );

    // ------------------------------------------------------------------------
    // Expected queues (order-based pop on valid_out)
    // ------------------------------------------------------------------------
    localparam integer MAXV = 32768;
    reg [WORD_WIDTH-1:0] expA_b [0:MAXV-1], expB_b [0:MAXV-1];
    reg [WORD_WIDTH-1:0] expA_n [0:MAXV-1], expB_n [0:MAXV-1];
    integer wr_b, rd_b, wr_n, rd_n;

    // ------------------------------------------------------------------------
    // Helpers: modular add/sub and Barrett-based mul (parameterized widths)
    // ------------------------------------------------------------------------
    function [WORD_WIDTH-1:0] mod_add;
        input [WORD_WIDTH-1:0] a, b, q;
        reg   [WORD_WIDTH:0]   s;
        begin
            s = {1'b0,a} + {1'b0,b};
            if (s >= {1'b0,q}) mod_add = s - {1'b0,q};
            else                mod_add = s[WORD_WIDTH-1:0];
        end
    endfunction

    function [WORD_WIDTH-1:0] mod_sub;
        input [WORD_WIDTH-1:0] a, b, q;
        reg   [WORD_WIDTH:0]   d;
        begin
            if (a >= b) begin
                d = {1'b0,a} - {1'b0,b};
                mod_sub = d[WORD_WIDTH-1:0];
            end else begin
                d = ({1'b0,a} + {1'b0,q}) - {1'b0,b};
                mod_sub = d[WORD_WIDTH-1:0];
            end
        end
    endfunction

    // Calculate q_width from q (fallback / cross-check)
    function [5:0] calc_qw_safe;
        input [WORD_WIDTH-1:0] qW;
        integer k;
        begin
            calc_qw_safe = 6'd1;
            for (k=WORD_WIDTH-1; k>=0; k=k-1) begin
                if (qW[k]) calc_qw_safe = k + 1;
            end
        end
    endfunction

    // Barrett reduction (parameterized)
    // x is up to 2W bits; mu is 2W bits; intermediate products use 4W bits.
    function [WORD_WIDTH-1:0] barrett_reduce;
        input [2*WORD_WIDTH-1:0] x;
        input [WORD_WIDTH-1:0]   q;
        input [2*WORD_WIDTH-1:0] mu;
        input [5:0]              q_width;
        reg   [4*WORD_WIDTH-1:0] prod4W;
        reg   [4*WORD_WIDTH-1:0] t4W;
        reg   [4*WORD_WIDTH-1:0] tq4W;
        reg   [4*WORD_WIDTH-1:0] r4W;
        reg   [7:0]              k;
        reg   [WORD_WIDTH-1:0]   rW;
        begin
            k      = q_width << 1;        // k = 2*q_width
            prod4W = x * mu;              // (2W) * (2W) -> (4W)
            t4W    = prod4W >> k;         // ? 2W
            tq4W   = t4W * q;             // (?3W), keep as 4W
            r4W    = { {2*WORD_WIDTH{1'b0}}, x } - tq4W;
            rW     = r4W[WORD_WIDTH-1:0];
            if (rW >= q) rW = rW - q;
            if (rW >= q) rW = rW - q;
            barrett_reduce = rW;
        end
    endfunction

    function [WORD_WIDTH-1:0] mod_mul_barrett;
        input [WORD_WIDTH-1:0] a, b, q;
        input [2*WORD_WIDTH-1:0] mu;
        input [5:0]            q_width;
        reg   [2*WORD_WIDTH-1:0] x;
        begin
            x = a * b;  // <= 2W
            mod_mul_barrett = barrett_reduce(x, q, mu, q_width);
        end
    endfunction

    // Random WORD_WIDTH-bit number generator (Verilog-2001 requires ?1 input)
    function [WORD_WIDTH-1:0] rndw;
        input dummy;  // pass 0 when calling
        reg [31:0] r0, r1;
        reg [WORD_WIDTH-1:0] m;
        begin
            r0 = $random; r1 = $random;
            m  = { (r0 ^ {r0[15:0], r0[31:16]}), r1 };
            rndw = m & {WORD_WIDTH{1'b1}};
        end
    endfunction

    // ------------------------------------------------------------------------
    // Utilities
    // ------------------------------------------------------------------------
    task reset_env;
        begin
            rst_n   = 1'b0;
            vld_in_b= 1'b0; vld_in_n=1'b0;
            A_in_b  = {WORD_WIDTH{1'b0}}; B_in_b = {WORD_WIDTH{1'b0}};
            W_in_b  = {{(WORD_WIDTH-1){1'b0}},1'b1};
            q_in_b  = {WORD_WIDTH{1'b0}}; mu_in_b = {2*WORD_WIDTH{1'b0}}; qw_in_b = 6'd0;

            A_in_n  = {WORD_WIDTH{1'b0}}; B_in_n = {WORD_WIDTH{1'b0}};
            W_in_n  = {{(WORD_WIDTH-1){1'b0}},1'b1};
            q_in_n  = {WORD_WIDTH{1'b0}}; mu_in_n = {2*WORD_WIDTH{1'b0}}; qw_in_n = 6'd0;

            wr_b=0; rd_b=0; wr_n=0; rd_n=0;

            repeat (RST_CYC) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    // Push expected for BYPASS
    task push_exp_b;
        input [WORD_WIDTH-1:0] AOe, BOe;
        begin
            if (wr_b >= MAXV) begin
                $display("[FATAL] exp queue overflow (bypass)");
                $stop;
            end
            expA_b[wr_b] = AOe;
            expB_b[wr_b] = BOe;
            wr_b = wr_b + 1;
        end
    endtask

    // Push expected for NORMAL
    task push_exp_n;
        input [WORD_WIDTH-1:0] AOe, BOe;
        begin
            if (wr_n >= MAXV) begin
                $display("[FATAL] exp queue overflow (normal)");
                $stop;
            end
            expA_n[wr_n] = AOe;
            expB_n[wr_n] = BOe;
            wr_n = wr_n + 1;
        end
    endtask

    // Drive one-cycle into BYPASS DUT
    task drive_b;
        input [WORD_WIDTH-1:0] A, B, W, Q;
        input [2*WORD_WIDTH-1:0] MU;
        input [5:0]            QW;
        begin
            A_in_b  <= A; B_in_b <= B; W_in_b <= W; q_in_b <= Q; mu_in_b <= MU; qw_in_b <= QW;
            vld_in_b<= 1'b1; @(posedge clk); vld_in_b<=1'b0;
        end
    endtask

    // Drive one-cycle into NORMAL DUT
    task drive_n;
        input [WORD_WIDTH-1:0] A, B, W, Q;
        input [2*WORD_WIDTH-1:0] MU;
        input [5:0]            QW;
        begin
            A_in_n  <= A; B_in_n <= B; W_in_n <= W; q_in_n <= Q; mu_in_n <= MU; qw_in_n <= QW;
            vld_in_n<= 1'b1; @(posedge clk); vld_in_n<=1'b0;
        end
    endtask

    // Pop & print check on every valid_out (ALWAYS print exp/got)
    integer idx_b, idx_n;
    always @(posedge clk) begin
        if (rst_n && vld_out_b) begin
            if (rd_b >= wr_b) begin
                $display("[ERROR t=%0t] BYPASS: output with empty queue", $time);
                $stop;
            end
            idx_b = rd_b;
            $display("[BYPASS t=%0t idx=%0d]  AO: exp=%0h  got=%0h  |  BO: exp=%0h  got=%0h",
                      $time, idx_b, expA_b[idx_b], A_out_b, expB_b[idx_b], B_out_b);
            if (A_out_b !== expA_b[idx_b] || B_out_b !== expB_b[idx_b]) begin
                $display("[ERROR] BYPASS mismatch at idx=%0d", idx_b);
                $stop;
            end
            rd_b = rd_b + 1;
        end
        if (rst_n && vld_out_n) begin
            if (rd_n >= wr_n) begin
                $display("[ERROR t=%0t] NORMAL: output with empty queue", $time);
                $stop;
            end
            idx_n = rd_n;
            $display("[NORMAL t=%0t idx=%0d]  AO: exp=%0h  got=%0h  |  BO: exp=%0h  got=%0h",
                      $time, idx_n, expA_n[idx_n], A_out_n, expB_n[idx_n], B_out_n);
            if (A_out_n !== expA_n[idx_n] || B_out_n !== expB_n[idx_n]) begin
                $display("[ERROR] NORMAL mismatch at idx=%0d", idx_n);
                $stop;
            end
            rd_n = rd_n + 1;
        end
    end

    // ------------------------------------------------------------------------
    // Twiddle access helpers
    // ------------------------------------------------------------------------
    function [WORD_WIDTH-1:0] get_tw;
        input integer L, SET, IDX;
        integer base;
        begin
            case (SET)
                0: begin base = L*S0_PER_LVL; get_tw = tw_s0[base + IDX]; end
                1: begin base = L*S1_PER_LVL; get_tw = tw_s1[base + IDX]; end
                default: begin base = L*S2_PER_LVL; get_tw = tw_s2[base + IDX]; end
            endcase
        end
    endfunction

    function integer get_cnt;
        input integer SET;
        begin
            case (SET)
                0: get_cnt = S0_PER_LVL;
                1: get_cnt = S1_PER_LVL;
                default: get_cnt = S2_PER_LVL;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // Run a single level across s0/s1/s2 (?? + ??)
    // ------------------------------------------------------------------------
    task run_level_all_sets;
        input integer L;
        integer SET, i, cnt;
        reg [WORD_WIDTH-1:0] qW, w1, w2, w_rand;
        reg [2*WORD_WIDTH-1:0] muW2;
        reg [5:0]   qw_file, qw_calc, qw_use;
        reg [WORD_WIDTH-1:0] A, B, AOe, BOe, diff, prod;

        begin
            qW     = q_table[L];
            muW2   = mu_table[L];
            qw_file= qwidth_table[L][5:0];
            qw_calc= calc_qw_safe(qW);
            qw_use = (qw_file == 0) ? qw_calc : qw_file;

            for (SET = 0; SET < 3; SET = SET + 1) begin
                cnt = get_cnt(SET);

                // sX[0] == 1 ??? ??
                if (get_tw(L, SET, 0) !== {{(WORD_WIDTH-1){1'b0}}, 1'b1}) begin
                    $display("[ERROR] Level%0d SET%0d: tw[0] != 1  got=%h", L, SET, get_tw(L,SET,0));
                    $stop;
                end

                // --- BYPASS (LAST_STAGE=1)
                for (i=0; i<4; i=i+1) begin
                    case (i)
                        0: begin A = {WORD_WIDTH{1'b0}}; B = {WORD_WIDTH{1'b0}}; end
                        1: begin A = qW - 1; B = {{(WORD_WIDTH-1){1'b0}},1'b1}; end
                        2: begin A = {{(WORD_WIDTH-1){1'b0}},1'b1}; B = qW - 1; end
                        default: begin A = qW - 1; B = qW - 1; end
                    endcase
                    AOe  = mod_add(A, B, qW);
                    BOe  = mod_sub(A, B, qW);
                    push_exp_b(AOe, BOe);
                    drive_b(A, B, get_tw(L,SET,0), qW, muW2, qw_use); // w? ??
                end
                for (i=0; i<RAND_VEC; i=i+1) begin
                    A   = rndw(0) % qW;
                    B   = rndw(0) % qW;
                    AOe = mod_add(A, B, qW);
                    BOe = mod_sub(A, B, qW);
                    push_exp_b(AOe, BOe);
                    drive_b(A, B, get_tw(L,SET,0), qW, muW2, qw_use);
                end

                // --- NORMAL (LAST_STAGE=0)
                w1 = get_tw(L, SET, 0);                 // = 1
                w2 = get_tw(L, SET, (cnt>1)?1:0);       // ? ??(??? 1)

                // ?? 4???
                for (i=0; i<4; i=i+1) begin
                    case (i)
                        0: begin A = {WORD_WIDTH{1'b0}}; B = {WORD_WIDTH{1'b0}}; end
                        1: begin A = qW - 1; B = {{(WORD_WIDTH-1){1'b0}},1'b1}; end
                        2: begin A = {{(WORD_WIDTH-1){1'b0}},1'b1}; B = qW - 1; end
                        default: begin A = qW - 1; B = qW - 1; end
                    endcase
                    diff = mod_sub(A, B, qW);
                    prod = mod_mul_barrett(diff, w2, qW, muW2, qw_use);
                    AOe  = mod_add(A, B, qW);
                    BOe  = prod;
                    push_exp_n(AOe, BOe);
                    drive_n(A, B, w2, qW, muW2, qw_use);
                end

                // ?? + ?? ??
                for (i=0; i<RAND_VEC; i=i+1) begin
                    A     = rndw(0) % qW;
                    B     = rndw(0) % qW;
                    w_rand= get_tw(L, SET, (i % cnt));
                    diff  = mod_sub(A, B, qW);
                    prod  = mod_mul_barrett(diff, w_rand, qW, muW2, qw_use);
                    AOe   = mod_add(A, B, qW);
                    BOe   = prod;
                    push_exp_n(AOe, BOe);
                    drive_n(A, B, w_rand, qW, muW2, qw_use);
                end
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Find first occurrence levels for widths: 42, 58, 59, 60
    // ------------------------------------------------------------------------
    integer lvl_42, lvl_58, lvl_59, lvl_60;

    task find_levels_by_widths;
        integer i;
        begin
            lvl_42 = -1; lvl_58 = -1; lvl_59 = -1; lvl_60 = -1;
            for (i=0; i<TOTAL_LVLS; i=i+1) begin
                if (qwidth_table[i] == 8'd42 && lvl_42 < 0) lvl_42 = i;
                if (qwidth_table[i] == 8'd58 && lvl_58 < 0) lvl_58 = i;
                if (qwidth_table[i] == 8'd59 && lvl_59 < 0) lvl_59 = i;
                if (qwidth_table[i] == 8'd60 && lvl_60 < 0) lvl_60 = i;
            end
            $display("[INFO] Level indices by q_width: 42->%0d  58->%0d  59->%0d  60->%0d",
                     lvl_42, lvl_58, lvl_59, lvl_60);
        end
    endtask

    // ------------------------------------------------------------------------
    // Pretty dump: show first 2 twiddles for s0/s1/s2 at a level
    // ------------------------------------------------------------------------
    task dump_twiddles_one_level;
        input integer L;
        begin
            $display("[INFO] L%0d tw counts: s0=%0d s1=%0d s2=%0d", L, S0_PER_LVL, S1_PER_LVL, S2_PER_LVL);
            $display("       s0[0]=%h s0[1]=%h | s1[0]=%h s1[1]=%h | s2[0]=%h s2[1]=%h",
                     get_tw(L,0,0), get_tw(L,0,1),
                     get_tw(L,1,0), get_tw(L,1,1),
                     get_tw(L,2,0), get_tw(L,2,1));
        end
    endtask

    // ------------------------------------------------------------------------
    // Main
    // ------------------------------------------------------------------------
    initial begin
        // Load hex tables
        $display("[INFO] Loading q/mu/q_width/twiddle hex tables...");
        $readmemh("ntt16_data/q_table.hex",          q_table);
        $readmemh("ntt16_data/mu_table.hex",         mu_table);
        $readmemh("ntt16_data/q_width_table.hex",    qwidth_table);
        $readmemh("ntt16_data/levels_s0_60_std.hex", tw_s0);
        $readmemh("ntt16_data/levels_s1_60_std.hex", tw_s1);
        $readmemh("ntt16_data/levels_s2_60_std.hex", tw_s2);

        // Info header: show counts & first elements of level0
        $display("[INFO] q[0]=%h mu[0]=%h q_width[0]=%0d | tw_s0[0]=%h tw_s1[0]=%h tw_s2[0]=%h | counts s0=%0d s1=%0d s2=%0d",
                 q_table[0], mu_table[0], qwidth_table[0],
                 tw_s0[0], tw_s1[0], tw_s2[0],
                 S0_PER_LVL, S1_PER_LVL, S2_PER_LVL);

        // Detect target levels
        find_levels_by_widths();

        // Optional detailed dump for each selected level
        if (lvl_42 >= 0) dump_twiddles_one_level(lvl_42);
        if (lvl_58 >= 0) dump_twiddles_one_level(lvl_58);
        if (lvl_59 >= 0) dump_twiddles_one_level(lvl_59);
        if (lvl_60 >= 0) dump_twiddles_one_level(lvl_60);

        // Reset
        reset_env();

        // Run selected levels (skip if not found)
        if (lvl_42 >= 0) begin
            $display("[RUN ] Level %0d (q_width=42) ? s0/s1/s2 all", lvl_42);
            run_level_all_sets(lvl_42);
        end else $display("[WARN] q_width=42 not found in q_width_table.hex");

        if (lvl_58 >= 0) begin
            $display("[RUN ] Level %0d (q_width=58) ? s0/s1/s2 all", lvl_58);
            run_level_all_sets(lvl_58);
        end else $display("[WARN] q_width=58 not found in q_width_table.hex");

        if (lvl_59 >= 0) begin
            $display("[RUN ] Level %0d (q_width=59) ? s0/s1/s2 all", lvl_59);
            run_level_all_sets(lvl_59);
        end else $display("[WARN] q_width=59 not found in q_width_table.hex");

        if (lvl_60 >= 0) begin
            $display("[RUN ] Level %0d (q_width=60) ? s0/s1/s2 all", lvl_60);
            run_level_all_sets(lvl_60);
        end else $display("[WARN] q_width=60 not found in q_width_table.hex");

        // Drain until all outputs verified
        while (rd_b < wr_b || rd_n < wr_n) @(posedge clk);

        $display("[PASS] All selected levels/sets/modes matched using your hex tables.");
        $stop;
    end

endmodule

`default_nettype wire

