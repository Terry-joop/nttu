// -----------------------------------------------------------------------------
// double_on_fly_wrap.v  (Verilog-2001)
// Version: v1.5d2-FASTSTART-FIX
//  - Seamless tiles (prefetch + shadow), same flow as single v1.5s
//  - FASTSTART: first start at in_valid rising (1c earlier than consumption)
//  - Robust seed handoff: capture+commit at boundary+1, row0 uses ROM vec
//  - Parameterizable ROM_OUT_LATENCY (default 2)
//  - qconst_rom_bank included
//  - ACCEPTS legacy parameters LEVEL_BITS/TILEC_BITS (for top compatibility)
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module double_on_fly_wrap #(
    parameter integer WORD_WIDTH        = `NTT_WORD_WIDTH, // e.g., 60
    parameter integer LANES             = `LANES,          // 16
    parameter         SEAMLESS_TILES    = 1,               // 1: enable seamless mode
    parameter integer ROM_OUT_LATENCY   = 2,               // cycles from addr change to vec stable
    // --- Legacy-compat parameters to satisfy upper-level instantiation ---
    parameter integer LEVEL_BITS        = 5,               // not used for port widths; for compat
    parameter integer TILEC_BITS        = 4                // not used for port widths; for compat
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // Phase-2 coordinates (col-first over tiles)
    input  wire [`NTT_BANK_BITS-1:0]    bank_id,          // 0..NUM_LEVELS-1
    input  wire [3:0]                   tile_c,           // 0..15 (current tile col)

    // Stream in/out (aligned with ntt16_buscore)
    input  wire                         in_valid,
    input  wire [LANES*WORD_WIDTH-1:0]  in_data_vec,
    output wire                         out_valid,
    output wire [LANES*WORD_WIDTH-1:0]  out_data_vec
);
    localparam integer PACK = LANES*WORD_WIDTH;

    // Optional elaboration notice (non-fatal) to catch mismatches early
    initial begin
        if (LEVEL_BITS != `NTT_BANK_BITS)
            $display("[double_on_fly_wrap] NOTE: LEVEL_BITS(%0d) != `NTT_BANK_BITS(%0d) ? ports use macro widths.",
                     LEVEL_BITS, `NTT_BANK_BITS);
        if (TILEC_BITS != 4)
            $display("[double_on_fly_wrap] NOTE: TILEC_BITS(%0d) != 4 ? ports are fixed 4b tile_c.", TILEC_BITS);
        $display("[double_on_fly_wrap v1.5d2-FASTSTART-FIX] ROM_LAT=%0d, seamless tiles; start-first; row0 uses ROM vec.",
                 ROM_OUT_LATENCY);
    end

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
    reg [`NTT_BANK_BITS-1:0] rom_bank;
    reg [3:0]                rom_c;
    reg [3:0]                cur_c;
    reg [4:0]                tile_pos;

    reg [PACK-1:0] t0_shadow, t1_shadow, h2_shadow, h2pre_shadow;
    reg            use_shadow;

    wire [PACK-1:0] t0_even_vec, t1_odd_vec, h2_vec, h2pre_vec;

    wire [3:0] next_c_calc = (cur_c == 4'd15) ? 4'd0 : (cur_c + 4'd1);

    localparam integer LCL                = (ROM_OUT_LATENCY < 0) ? 0 : ROM_OUT_LATENCY;
    localparam integer PREFETCH_POS_INT   = 15 - LCL;
    localparam integer PREFETCH_POS       = (PREFETCH_POS_INT < 12) ? 12 : PREFETCH_POS_INT;
    localparam integer BOUNDARY_POS       = 15;

    wire dof_consume   = s_valid_d;
    wire prefetch_fire = (SEAMLESS_TILES && dof_consume && (tile_pos == PREFETCH_POS[4:0]));
    wire boundary_fire = (SEAMLESS_TILES && dof_consume && (tile_pos == BOUNDARY_POS[4:0]));

    reg boundary_fire_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) boundary_fire_q <= 1'b0;
        else        boundary_fire_q <= boundary_fire;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rom_bank     <= bank_id;
            rom_c        <= tile_c;
            cur_c        <= tile_c;
            tile_pos     <= 5'd0;
            use_shadow   <= 1'b0;
            t0_shadow    <= {PACK{1'b0}};
            t1_shadow    <= {PACK{1'b0}};
            h2_shadow    <= {PACK{1'b0}};
            h2pre_shadow <= {PACK{1'b0}};
        end else begin
            if (dof_consume)
                tile_pos <= (tile_pos == 5'd15) ? 5'd0 : (tile_pos + 5'd1);

            if (prefetch_fire) begin
                rom_bank <= bank_id; // bank change is expected to be stable by here
                rom_c    <= next_c_calc;
            end

            if (boundary_fire_q) begin
                t0_shadow    <= t0_even_vec;
                t1_shadow    <= t1_odd_vec;
                h2_shadow    <= h2_vec;
                h2pre_shadow <= h2pre_vec;
                use_shadow   <= 1'b1;
                cur_c        <= next_c_calc;
            end
        end
    end

    // Map ROM_OUT_LATENCY -> ADDR_PIPE_STAGES of double_twist_rom (1c ROM + Xc addr pipe)
    localparam integer DTR_ADDR_PIPE_STAGES = (ROM_OUT_LATENCY > 1) ? 1 : 0;

    double_twist_rom #(
        .WORD_WIDTH       (WORD_WIDTH),
        .LANES            (LANES),
        .ADDR_PIPE_STAGES (DTR_ADDR_PIPE_STAGES),
        .INIT_T0_HEX      ("single_vector/single_even_rom_combined.hex"),
        .INIT_T1_HEX      ("double_vector/double_t1_odd.hex"),
        .INIT_H2_HEX      ("double_vector/double_h2.hex"),
        .INIT_H2PRE_HEX   ("double_vector/double_h2pre.hex")
    ) U_TWIST (
        .clk        (clk),
        .rst_n      (rst_n),
        .bank_id    (rom_bank),
        .tile_r     (4'd0),     // kept for interface symmetry
        .tile_c     (rom_c),
        .t0_even_vec(t0_even_vec),
        .h2_vec     (h2_vec),
        .h2pre_vec  (h2pre_vec),
        .t1_odd_vec (t1_odd_vec)
    );

    // Active seed selection
    wire switch_pending       = boundary_fire_q;
    wire [PACK-1:0] t0_active = switch_pending ? t0_even_vec
                                               : (use_shadow ? t0_shadow    : t0_even_vec);
    wire [PACK-1:0] t1_active = switch_pending ? t1_odd_vec
                                               : (use_shadow ? t1_shadow    : t1_odd_vec);
    wire [PACK-1:0] h2_active = switch_pending ? h2_vec
                                               : (use_shadow ? h2_shadow    : h2_vec);
    wire [PACK-1:0] h2pre_active = switch_pending ? h2pre_vec
                                                  : (use_shadow ? h2pre_shadow : h2pre_vec);

    // ------------------------------- DoF core --------------------------------
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
                    started <= 1'b1;
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
    wire start_pulse  = start_first | start_next | start_legacy;

    wire            core_busy;
    wire [PACK-1:0] core_out_vec;

    double_on_fly #(
        .WORD_WIDTH (WORD_WIDTH),
        .LANES      (LANES)
    ) U_DOF (
        .clk                 (clk),
        .rst_n               (rst_n),
        .start               (start_pulse),
        .busy                (core_busy),
        .t0_even_vec         (t0_active),
        .t1_odd_vec          (t1_active),
        .h2_vec              (h2_active),
        .h2pre_vec           (h2pre_active),
        .q                   (q_wire),
        .mu                  (mu_wire),
        .q_width             (q_width_wire),
        .in_valid            (s_valid_d),
        .in_data_vec         (s_data_d),
        .barrett_valid_out   (out_valid),
        .barrett_result_vec  (core_out_vec)
    );

    // ----------------------- Lane repacker to buscore ------------------------
    genvar li;
    generate
        for (li = 0; li < LANES; li = li + 1) begin : G_PACK_ALIGN
            assign out_data_vec[(WORD_WIDTH*li)+WORD_WIDTH-1 : (WORD_WIDTH*li)]
                 = core_out_vec[(WORD_WIDTH*(LANES-1-li))+WORD_WIDTH-1 : (WORD_WIDTH*(LANES-1-li))];
        end
    endgenerate
endmodule

`default_nettype wire

