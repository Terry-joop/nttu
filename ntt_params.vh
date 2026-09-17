`ifndef NTT_PARAMS_VH
`define NTT_PARAMS_VH
// ============================================================================
// ntt_params.vh
// Global build-time switches and constants for NTT16 bus cores and twist units
// Verilog-2001 only (no SystemVerilog), ASCII-only comments.
//
// This header is a mini "datasheet" for integrating a 16-lane, 16-point NTT
// (R2 DIF) bus core into a larger 2D pipeline. It explains the interface,
// bank/twist tag protocol, multi-bank HEX file layout, and timing slice knobs.
// ============================================================================
//
// -----------------------------------------------------------------------------
// [README / System context]
// -----------------------------------------------------------------------------
// 1) Where this core sits
//    - Target system: 2D NTT of size 16^2 x 16^2 = 256 x 256.
//    - Decompose into four passes of 16-point NTTs with transpose + twist in between.
//    - This project follows a column-first schedule (as in SHARP):
//        Pass A: 1D NTT16 over columns
//        Transpose + on-the-fly twist
//        Pass B: 1D NTT16 over rows
//        (Repeat for remaining tiles with Pass C / Pass D as the schedule dictates)
//
// 2) Column-first rationale (short)
//    - Mathematically row-first or column-first are equivalent.
//    - Hardware-wise, column access is typically strided; the front-end transpose
//      unit converts strided lane reads into contiguous column feeds so the first
//      1D NTT runs locally. After that, a tile-level transpose aligns the next pass.
//
// 3) Bus interface model (per frame)
//    - A "frame" is 16 lanes wide (one 16-point vector).
//    - Input: s_valid=1 means s_data[W*16-1:0] holds 16 lanes this cycle.
//      Lane packing order (little-to-big):
//        lane 0  = s_data[W*0  +: W]
//        lane 1  = s_data[W*1  +: W]
//        ...
//        lane 15 = s_data[W*15 +: W]
//    - Output: m_valid / m_data in DIF order (bit-reversed w.r.t natural).
//
// 4) Tag protocol (VERY IMPORTANT)
//    - bank_id: selects modulus set (RNS limb), twiddle ROMs, q/q_inv tables.
//    - twist_id (Phase-2 / optional): selects pre-computed ?double? twist vector
//      if you place a twist16 stage between NTT16 passes.
//    - Both are frame tags. Drive them on every cycle where s_valid=1.
//    - Tags may change only at frame boundaries (when s_valid goes 0->1).
//
// 5) One-cycle bubble rule (no input skid in base design)
//    - When bank_id changes (and, if used, when twist_id changes) at a frame
//      boundary, upstream must insert exactly ONE idle cycle (s_valid=0) before
//      feeding the first frame under the new tag into the first consumer block
//      (Stage-0 of NTT16 or the twist stage).
//    - Reason: all ROMs (twiddle/q/qinv/twist) are 1-cycle registered; that bubble
//      aligns their outputs to data. Without a skid buffer the first frame would be lost.
//    - If you later add a 1-deep skid buffer at the first consumer, the top can keep
//      s_valid=1 continuously; the skid absorbs the warm-up internally.
//
// 6) Multi-bank HEX files (bank-major layout)
//    - Twiddle stage ROMs (Montgomery domain, W bits per word):
//        S0 (distance=8): 8 words per bank   -> total lines = NUM_BANKS * 8
//            order per bank: w^0, w^1, w^2, w^3, w^4, w^5, w^6, w^7
//        S1 (distance=4): 4 words per bank   -> total lines = NUM_BANKS * 4
//            order per bank: w^0, w^2, w^4, w^6
//        S2 (distance=2): 2 words per bank   -> total lines = NUM_BANKS * 2
//            order per bank: w^0, w^4
//      Indexing per bank i (0-based):
//        S0 lines: i*8 .. i*8+7
//        S1 lines: i*4 .. i*4+3
//        S2 lines: i*2 .. i*2+1
//    - q_table.hex / qinv_table.hex (W-bit words, one line per bank):
//        line i = q (or q_inv) for bank i
//      Note: q_inv is (-q^{-1}) mod 2^W (NOT +q^{-1}).
//    - Phase-2 ?double twist? (optional):
//        Flatten (twist_id, bank_id) -> combined index; store 16 words per set
//        in lane order. Total lines = (NUM_BANKS * NUM_TWIST_SETS) * 16.
//
// 7) Timing slices between NTT16 stages
//    - You may insert one-cycle register slices at S0->S1, S1->S2, S2->S3 to help
//      timing closure. This increases latency by one per boundary, but steady-state
//      throughput remains one frame/cycle once full.
//    - If you insert slices in your top-level, always pipeline bank_id (and twist_id)
//      in lockstep with data/valid.
//
// 8) Validation checklist
//    - bank_id (and twist_id if used) remain constant throughout a full 16-lane frame.
//    - Exactly one bubble is observed at the first consumer of each tag change.
//    - After transpose, the next NTT16 sees the intended lane pairings for distances 8/4/2/1.
//    - Twiddle/Q tables match the same modulus; all values are Montgomery-domain W-bit words.
//    - DIF output order is expected; later transpose/twist restores natural order where needed.
//
// -----------------------------------------------------------------------------
// [Parameter block]
// -----------------------------------------------------------------------------
// Note: Verilog-2001 has no $clog2; keep BANK_BITS/TWIST_BITS consistent with
// the chosen NUM_* values. Paths are relative to the sim/synth working directory.
// -----------------------------------------------------------------------------

// [Word width & butterfly internal pipeline]
// - WORD_WIDTH: datapath width in Montgomery domain (R = 2^WORD_WIDTH).
// - MONT_LAT : internal pipeline depth of radix2_butterfly_dif (add/sub/montmul).
`define NTT_WORD_WIDTH     60
`define NTT_BARRETT_LAT    4
`define LANES              16

// [Banked operation (RNS / multi-modulus)]
// - NUM_BANKS : number of modulus levels you actually use.
// - BANK_BITS : ceil(log2(NUM_BANKS)) (set manually).
`define NTT_NUM_BANKS      30
`define NTT_BANK_BITS       5

// [Twiddle ROM filenames (bank-major; Montgomery words, W bits each)]
// - S0: 8 words per bank (w^0..w^7)
// - S1: 4 words per bank (w^0,w^2,w^4,w^6)
// - S2: 2 words per bank (w^0,w^4)
`define NTT_TW_S0_HEX   "ntt16_data/levels_s0_60_std.hex"
`define NTT_TW_S1_HEX   "ntt16_data/levels_s1_60_std.hex"
`define NTT_TW_S2_HEX   "ntt16_data/levels_s2_60_std.hex"

// [Per-stage words-per-bank constants] (used when instantiating ROMs)
`define NTT_S0_BANK_WORDS  8
`define NTT_S1_BANK_WORDS  4
`define NTT_S2_BANK_WORDS  2

// [Q / Q_width small ROM files (one W-bit word per bank, line i = bank i)]

`define NTT_Q_HEX       "ntt16_data/q_table.hex"
`define NTT_Q_WIDTH_HEX "ntt16_data/q_width_table.hex"
`define NTT_MU_HEX	"ntt16_data/mu_table.hex"

// [Inter-stage timing slices between NTT16 stages]
// - 1 inserts a single pipeline register stage; 0 wires through directly.
// - If you enable a slice in the NTT core or in your top, carry bank_id
//   (and twist_id if used) through a matching register in lockstep.
//`define NTT_REG_S01       1   // Stage0 -> Stage1
//`define NTT_REG_S12       1   // Stage1 -> Stage2
//`define NTT_REG_S23       1   // Stage2 -> Stage3

// [Double on-the-fly twist (Phase-2; optional)]
// - NUM_TWIST_SETS : number of twist vectors per bank you plan to schedule.
// - TWIST_BITS     : ceil(log2(NUM_TWIST_SETS)) (set manually; keep >=1).
// - TWIST16_HEX    : multi-bank, multi-set table with 16 W-bit words per set.
//   Combined index example (flatten):
//     combined_sel = (twist_id << NTT_BANK_BITS) | bank_id
//   Start line in HEX for that set = combined_sel * 16; next 16 lines are lanes 0..15.
`define NTT_NUM_TWIST_SETS     1
`define NTT_TWIST_BITS         1
//`define NTT_TWIST16_HEX        "twiddle/levels_twist16.hex"

// [Handy macros]
// - NTT_LANE(BUS, IDX): Verilog-2001-safe constant slice macro for lane access.
//   IDX must be a compile-time constant (use generate loops for iteration).
`define NTT_LANE(BUS, IDX) BUS[(`NTT_WORD_WIDTH*(IDX)) +: `NTT_WORD_WIDTH]

// -----------------------------------------------------------------------------
// [Usage notes / examples]
// -----------------------------------------------------------------------------
// Example: driving a frame with tags (no skid). Insert 1 bubble on tag change.
//
//   // prev tags (registered in your top)
//   reg [`NTT_BANK_BITS-1:0] bank_d1;
//   reg [`NTT_TWIST_BITS-1:0] twid_d1; // if you use twist
//   wire set_change = new_frame & ((bank_id != bank_d1) /*|| (twist_id != twid_d1)*/);
//
//   always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) begin
//       s_valid <= 1'b0;
//       bank_d1 <= {`NTT_BANK_BITS{1'b0}};
//       twid_d1 <= {`NTT_TWIST_BITS{1'b0}};
//     end else begin
//       bank_d1 <= bank_id;
//       twid_d1 <= twist_id; // if used
//       if (set_change) begin
//         s_valid <= 1'b0;    // the ONE-cycle bubble
//       end else if (new_frame) begin
//         s_valid <= 1'b1;
//         s_data  <= frame_bus_16lanes;
//       end else begin
//         s_valid <= 1'b0;
//       end
//     end
//   end
//
// Notes:
// - Keep bank_id (and twist_id) constant across the 16-lane frame.
// - The NTT16 core is row/col agnostic; column-first vs row-first is decided
//   by your feeder/transpose/twist schedule. This header documents a column-first
//   baseline to match the system.
//
// ============================================================================

`endif

