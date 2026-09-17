// ============================================================================
// transpose16x16_bus  // Verilog-2001  // v2.3p-fix
// ----------------------------------------------------------------------------
// - True 16x16 transpose using XOR banking.
// - Hidden PRIME via prefetch: steady-state 16 cycles/tile (first tile has 1c
//   internal prime).
// - BRAM-friendly: registered address -> next-cycle data.
// - a_full/b_full: single-writer (read-side)? ?? ? multi-driver ??.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`include "ntt_params.vh"

module transpose16x16_bus #(
    parameter integer WORD_WIDTH = `NTT_WORD_WIDTH
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // Input bus (from ntt16_buscore)
    input  wire                         s_valid,
    input  wire [WORD_WIDTH*16-1:0]     s_data,

    // Output bus (to twist / next stage)
    output reg                          m_valid,
    output reg  [WORD_WIDTH*16-1:0]     m_data
);

    // -------------------------------------------------------------------------
    // Local: 4-bit bit-reverse (LSB-first bit order)
    // -------------------------------------------------------------------------
    function [3:0] br4;
        input [3:0] x;
        begin
            // NOTE: With LSB-first packing in this design, this form is intended.
            br4 = {x[0], x[1], x[2], x[3]};
        end
    endfunction

    // -------------------------------------------------------------------------
    // Ping-Pong control / full flags
    //   * a_full/b_full ? read always ????? set/clear (?? ????)
    // -------------------------------------------------------------------------
    reg  wr_buf_sel;     // 0:A write, 1:B write
    reg  rd_buf_sel;     // 0:A read,  1:B read
    reg  a_full, b_full; // tile-ready flags (set by writer-done pulse, cleared by reader-take)

    // -------------------------------------------------------------------------
    // Memories: 2 x (16 banks x 16 depth) = [bank][addr]
    // -------------------------------------------------------------------------
    // synthesis ramstyle = "M9K, M10K, M20K, block"
    reg [WORD_WIDTH-1:0] mem_a [0:15][0:15]; // [bank][addr] (addr = lane)
    reg [WORD_WIDTH-1:0] mem_b [0:15][0:15];

    // -------------------------------------------------------------------------
    // Write path: BR4(row) addressing with XOR banking
    //   * ???? a_full/b_full? ???? ?? (multi-driver ??)
    // -------------------------------------------------------------------------
    reg  [3:0] w_row;             // 0..15 (natural incoming row)
    wire [3:0] w_row_br = br4(w_row);

    // Unpack lanes 0..15 (LSB chunk = lane0)
    wire [WORD_WIDTH-1:0] s_lane [0:15];
    genvar gi;
    generate
        for (gi=0; gi<16; gi=gi+1) begin : G_UNPACK
            assign s_lane[gi] = s_data[WORD_WIDTH*gi +: WORD_WIDTH];
        end
    endgenerate

    integer bj;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_row      <= 4'd0;
            wr_buf_sel <= 1'b0;
        end else begin
            if (s_valid) begin
                // Parallel write: bank = BR4(row) ^ lane, addr = lane
                if (!wr_buf_sel) begin
                    for (bj=0; bj<16; bj=bj+1)
                        mem_a[w_row_br ^ bj][bj] <= s_lane[bj];
                end else begin
                    for (bj=0; bj<16; bj=bj+1)
                        mem_b[w_row_br ^ bj][bj] <= s_lane[bj];
                end

                // Row advance / tile complete -> wr_buf_sel ??, w_row ??
                if (w_row == 4'd15) begin
                    wr_buf_sel <= ~wr_buf_sel;
                    w_row      <= 4'd0;
                end else begin
                    w_row <= w_row + 4'd1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read path: hidden prime (prefetch) + XOR banking
    //   * a_full/b_full set/clear ?? (?? always ??)
    // -------------------------------------------------------------------------
    reg        rd_active;        // streaming active
    reg [3:0]  r_addr_q;         // registered address (0..15)
    reg        stop_after_last;  // drop m_valid on the cycle AFTER row15

    // Parallel read from banks; pack back to bus (combinational)
    wire [WORD_WIDTH-1:0] m_lane [0:15];
    generate
        for (gi=0; gi<16; gi=gi+1) begin : G_READ
            assign m_lane[gi] = (!rd_buf_sel) ? mem_a[gi ^ r_addr_q][r_addr_q]
                                              : mem_b[gi ^ r_addr_q][r_addr_q];
        end
    endgenerate

    wire [WORD_WIDTH*16-1:0] m_data_comb =
        { m_lane[15], m_lane[14], m_lane[13], m_lane[12],
          m_lane[11], m_lane[10], m_lane[9],  m_lane[8],
          m_lane[7],  m_lane[6],  m_lane[5],  m_lane[4],
          m_lane[3],  m_lane[2],  m_lane[1],  m_lane[0] };

    // Read control (+ a_full/b_full ??)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_buf_sel      <= 1'b0;
            r_addr_q        <= 4'd0;
            rd_active       <= 1'b0;
            m_valid         <= 1'b0;
            m_data          <= {WORD_WIDTH*16{1'b0}};
            stop_after_last <= 1'b0;
            a_full          <= 1'b0;  // ?? ????: ??? ???
            b_full          <= 1'b0;
        end else begin
            // Registered output for timing and valid alignment
            m_data <= m_data_comb;

            // --- writer-done pulse: ? ???? row15? ??? ?? ?? full set ---
            if (s_valid && (w_row == 4'd15)) begin
                if (!wr_buf_sel) a_full <= 1'b1; else b_full <= 1'b1;
            end

            if (!rd_active) begin
                // Idle: wait for a ready buffer, clear stop flag
                m_valid         <= 1'b0;
                stop_after_last <= 1'b0;

                if (a_full) begin
                    rd_buf_sel <= 1'b0;
                    r_addr_q   <= 4'd0;     // prime row0 (next cycle data valid)
                    rd_active  <= 1'b1;
                    a_full     <= 1'b0;     // consume A
                end else if (b_full) begin
                    rd_buf_sel <= 1'b1;
                    r_addr_q   <= 4'd0;
                    rd_active  <= 1'b1;
                    b_full     <= 1'b0;     // consume B
                end
            end else begin
                // Streaming
                m_valid <= 1'b1;

                if (stop_after_last) begin
                    // We output row15 last cycle; now stop cleanly
                    m_valid         <= 1'b0;
                    rd_active       <= 1'b0;
                    stop_after_last <= 1'b0;
                end else if (r_addr_q == 4'd15) begin
                    // Last row of current tile is being output this cycle.
                    // Try prefetch next tile to hide PRIME.
                    if ((rd_buf_sel==1'b0 && b_full) ||
                        (rd_buf_sel==1'b1 && a_full)) begin
                        // Switch buffer and start row0 next cycle
                        if (rd_buf_sel==1'b0) begin
                            rd_buf_sel <= 1'b1; b_full <= 1'b0;
                        end else begin
                            rd_buf_sel <= 1'b0; a_full <= 1'b0;
                        end
                        r_addr_q <= 4'd0; // next cycle emits row0 of new tile
                        // m_valid stays 1 (no bubble)
                    end else begin
                        // No next tile ready/existing: finish *after* this valid row
                        stop_after_last <= 1'b1;
                        // r_addr_q can hold; we will drop next cycle
                    end
                end else begin
                    // Inside tile
                    r_addr_q <= r_addr_q + 4'd1;
                end
            end
        end
    end

endmodule

`default_nettype wire

