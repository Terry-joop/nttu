// -----------------------------------------------------------------------------
// single_on_fly_twist16.v  (Verilog-2001 / SV part-select OK)
// Version: v1.1v + patch (consume first frame on start cycle + reload seed on start)
// -----------------------------------------------------------------------------
// - 'start' ????? ??(run_now)?? ?? 1clk/row ??
// - 'start'? ?? seed_ctr? 0?? ???(?? ????? even?odd ?? ??)
// - [PATCH] twist_i ??? ? ?? 'start'? ?? ???? ??,
//           ?? use_seed_phase/seed_ctr?? ????? ??
// -----------------------------------------------------------------------------

`include "ntt_params.vh"
`default_nettype none

module single_on_fly_twist16 #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH,
    parameter integer LANES      = `LANES
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // Control
    input  wire                         start,          // 1-cycle pulse to enter RUN (also tile boundary)
    output wire                         busy,           // asserted while RUN

    // Lane-local constants
    input  wire [LANES*WORD_WIDTH-1:0]  even_seed_vec,
    input  wire [LANES*WORD_WIDTH-1:0]  odd_seed_vec,
    input  wire [LANES*WORD_WIDTH-1:0]  ratio_vec,
    input  wire [LANES*WORD_WIDTH-1:0]  z_pre_vec,

    // Shared modulus
    input  wire [WORD_WIDTH-1:0]        q,
    input  wire [2*WORD_WIDTH-1:0]      mu,
    input  wire [5:0]                   q_width,   // dynamic bit-length of q

    // Per-lane input stream (A operands for barrett)
    input  wire                         in_valid,       // 1 frame/cycle across lanes
    input  wire [LANES*WORD_WIDTH-1:0]  in_data_vec,

    // Outputs
    output wire                         barrett_valid_out, // representative valid (lane0)
    output wire [LANES*WORD_WIDTH-1:0]  barrett_result_vec
);

    // -------------------------------------------------------------------------
    // FSM: IDLE -> RUN (continuous until reset)
    // -------------------------------------------------------------------------
    localparam [0:0] ST_IDLE = 1'b0;
    localparam [0:0] ST_RUN  = 1'b1;

    reg state, state_n;
    assign busy = (state == ST_RUN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= ST_IDLE;
        else        state <= state_n;
    end

    always @* begin
        case (state)
            ST_IDLE: state_n = start ? ST_RUN : ST_IDLE;
            ST_RUN : state_n = ST_RUN;
            default: state_n = ST_IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // Seed phase controller
    //   seed_ctr = 0 -> even, 1 -> odd, 2+ -> feedback (saturate)
    //   advance when we CONSUME (gated valid into Shoup)
    //   RELOAD to 0 on every 'start' (even during RUN)
    // -------------------------------------------------------------------------
    reg  [1:0] seed_ctr;
    wire       use_seed_phase = (seed_ctr < 2'd2);

    // Treat start cycle as RUN for gating (consume first frame on start)
    wire run_now = (state == ST_RUN) | start;

    // Shoup/Barrett advance exactly when we consume an input frame this cycle
    wire shoup_valid_in   = run_now & in_valid;
    wire barrett_valid_in = run_now & in_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seed_ctr <= 2'd0;
        end else if (start) begin
            // <<< FIX: reload on every start (IDLE or RUN) >>>
            // ?? ????? even?odd ??? ????? 0?? ???
            seed_ctr <= 2'd0;
        end else if (shoup_valid_in) begin
            if (seed_ctr != 2'd3) seed_ctr <= seed_ctr + 2'd1; // 0->1->2->3
        end
        // ??? ?? ??? ?? ??? ??? ??
    end

    // -------------------------------------------------------------------------
    // Lanes
    // -------------------------------------------------------------------------
    wire [LANES*WORD_WIDTH-1:0] shoup_result_vec;
    wire [LANES-1:0]            shoup_valid_out_vec;
    wire [LANES-1:0]            barrett_valid_out_vec;

    genvar i;
    generate
        for (i = 0; i < LANES; i = i + 1) begin : G_LANE
            // Lane-local slices
            wire [WORD_WIDTH-1:0] even_seed_i = even_seed_vec[i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] odd_seed_i  = odd_seed_vec [i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] ratio_i     = ratio_vec    [i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] z_pre_i     = z_pre_vec    [i*WORD_WIDTH +: WORD_WIDTH];
            wire [WORD_WIDTH-1:0] A_i         = in_data_vec  [i*WORD_WIDTH +: WORD_WIDTH];

            wire [WORD_WIDTH-1:0] shoup_res_i = shoup_result_vec[i*WORD_WIDTH +: WORD_WIDTH];

            // -----------------------------------------------------------------
            // [PATCH] twist_i: start? ?? ???? ??
            //  - ?? ???(start==1)?? ??? ?? ?? row15? shoup_res_i? ??
            //  - ?? ????? seed_ctr=0(even)?1(odd) ??
            // -----------------------------------------------------------------
            wire [WORD_WIDTH-1:0] twist_i =
                run_now
                  ? ( use_seed_phase
                        ? ( (seed_ctr == 2'd0) ? even_seed_i
                          : (seed_ctr == 2'd1) ? odd_seed_i
                                               : shoup_res_i )
                        : shoup_res_i )
                  : {WORD_WIDTH{1'b0}};

            // Shoup (2-stage)
            shoup_mult #(
                .WORD_WIDTH (WORD_WIDTH),
                .K_WIDTH    (WORD_WIDTH)
            ) u_shoup_i (
                .clk        (clk),
                .rst_n      (rst_n),
                .valid_in   (shoup_valid_in),
                .a          (twist_i),
                .ratio      (ratio_i),
                .z_pre      (z_pre_i),
                .q          (q),
                .valid_out  (shoup_valid_out_vec[i]),
                .result     (shoup_result_vec[i*WORD_WIDTH +: WORD_WIDTH])
            );

            // Barrett (4-stage)
            barrett_mult #(
                .WORD_WIDTH (WORD_WIDTH)
            ) u_barrett_i (
                .clk        (clk),
                .rst_n      (rst_n),
                .valid_in   (barrett_valid_in),
                .A          (A_i),
                .B          (twist_i),
                .q          (q),
                .mu         (mu),
                .q_width    (q_width),
                .valid_out  (barrett_valid_out_vec[i]),
                .result     (barrett_result_vec[i*WORD_WIDTH +: WORD_WIDTH])
            );
        end
    endgenerate

    // Representative barrett_valid_out (lanes are time-aligned)
    assign barrett_valid_out = barrett_valid_out_vec[0];

endmodule

`default_nettype wire

