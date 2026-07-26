/*
 * Minimal MOS 6526 CIA serial port (SDR) for the Plus/4 burst cartridge.
 *
 * Originally derived from Niklas Ekstroem's 8520 CIA implementation
 * (https://github.com/niklasekstrom/cia-verilog), June 2021.
 *
 * MW: removed:
 *     port A/B/FLAG/PC/TOD, control register B, timer A bits 4-15, IRQ, ICR, ICR mask
 *     changed:
 *       timer A (counter and latch) is 1-bit now, without register, only internal
 *       counting down from 1, no latch, no stop
 *       only two registers: SDR data (R/W) vs serial status+output (R) / serial output (W)
 *       status is cleared on register write (send data, change direction) and on
 *       data register read (receive data)
 *     added:
 *       romcs, rom_a15, full address bus for I/O selection, MUX
 *
 * ---------------------------------------------------------------------------
 * 2026-07-26 receive path rewritten.  Three defects were fixed; see
 * REVIEW-ciasdr.md for the evidence.  In short:
 *
 * 1. The old code used the raw IEC SRQ line as a hardware clock
 *    (always @(posedge CNT)).  SRQ is an open-drain line with a 3k3 pull-up
 *    feeding two IEC connectors' worth of cable capacitance, so its rising
 *    edge is a slow RC ramp, and an XC9572XL clock input has no hysteresis.
 *    Any noise on that ramp clocks the shift register more than once, which
 *    inserts phantom bits and rotates every byte from then on.  Neither
 *    reference chip does this: the 6522 shifts "during the first PHI2 clock
 *    cycle following the positive going edge of the CB1 shift pulse", and the
 *    working burstcart-via board additionally resynchronises SRQ through a
 *    74LS74 clocked by the system clock before it reaches CB1.
 *    -> CNT and SP are now synchronised into the PHI0 domain and the rising
 *       edge is detected there.  Timing budget: the 1581 clocks at 7 us/bit
 *       (Timer A = 6 @ 2 MHz), so each CNT phase lasts 3.5 us = 3.1 PHI0
 *       cycles at 0.886 MHz and 6.2 at 1.77 MHz.  SP is valid for 3.5 us
 *       either side of the CNT rising edge, and is sampled at the same PHI0
 *       edge at which CNT was first seen high, i.e. within 1.2 us of the
 *       rising edge.
 *
 * 2. The receive bit counter was only ever reset by RESET or by switching the
 *    port to output, so a single spurious or missed SRQ edge permanently
 *    misaligned every following byte with no way to recover.  The 6522
 *    documents the cure: "Reading or writing the Shift Register resets the
 *    Interrupt flag and initializes the SR counter to count another 8 pulses."
 *    -> shift_in_counter is now cleared on any access to the data register,
 *       so framing re-synchronises on every byte the CPU collects.
 *
 * 3. shift_complete_latched sampled the bus (seladdr / RW / A0) on the RISING
 *    edge of PHI0, where the Plus/4 address bus does not yet hold the CPU's
 *    address, so "clear the flag on a data register read" fired at the wrong
 *    time or not at all.  The register writes in this same file always used
 *    the falling edge, which is the instant that works on this bus.
 *    -> the whole design is now clocked from negedge E_CLK only.  That also
 *       frees the global clock net that CNT used to occupy.
 * ---------------------------------------------------------------------------
 */

module cia(
    // Chip access control.
    input RESET_n,
    input E_CLK, // PHI0
    input RW,
    input MUX,
    input [15:0] A,
    inout [7:0] D,

    // Serial port.
    inout CNT,
    inout SP,

    // ROM
    input c1lo, c1hi, c2lo, c2hi,

    output rom_a15,
    output rom_cs
    );

    // ROM
    assign rom_cs  = !(!c1lo || !c1hi || !c2lo || !c2hi);
    assign rom_a15 = !(!c1lo || !c1hi); // 1 for C1 (high 32K half, default for 32K ROM), 0 for C2 (low 32K half)

    // Plus4 I/O
    wire seladdr = (A[15:4] == 12'hFD9);

    // CIA registers
    localparam REG_SDR = 1'b0;
    localparam REG_CRA = 1'b1;

    // Bus cycle decode.  Everything below samples these at negedge E_CLK,
    // which is the point at which the Plus/4 address bus and RW are valid
    // (the same instant the register writes have always used).
    wire wr_cra  = seladdr && !RW && (A[0] == REG_CRA);
    wire wr_sdr  = seladdr && !RW && (A[0] == REG_SDR);
    wire acc_sdr = seladdr && (A[0] == REG_SDR);   // read OR write of the data register

    // Flag is cleared by any register write, or by a read of the data register.
    wire clr_flag = (seladdr && !RW) || acc_sdr;

    // Control registers.
    reg sp_output;

    // Interval timer A - transmit bit clock only.
    reg ta_counter;
    wire ta_underflowing = (ta_counter == 1'b0);

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            ta_counter <= 1'b0;
        else
            ta_counter <= ~ta_counter;
    end

    // ------------------------------------------------------------------
    // Serial port receive.  Synchronous to negedge E_CLK.
    // ------------------------------------------------------------------
    reg cnt_s1, cnt_s2, cnt_s3;   // CNT metastability filter + edge detect
    reg sp_s1, sp_s2;             // SP delayed by the same amount as cnt_s2

    reg [7:0] sdr_in;
    reg [7:0] shift_in;
    reg [2:0] shift_in_counter;

    // cnt_s2/cnt_s3 are one and two samples older than cnt_s1, so this is the
    // rising edge of CNT as observed two E_CLK cycles ago; sp_s2 is SP sampled
    // at that very same cycle.
    wire cnt_rise = cnt_s2 && !cnt_s3;
    wire rx_bit   = !sp_output && cnt_rise;
    wire byte_done = rx_bit && (shift_in_counter == 3'd7);

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            cnt_s1 <= 1'b1;
            cnt_s2 <= 1'b1;
            cnt_s3 <= 1'b1;
            sp_s1  <= 1'b1;
            sp_s2  <= 1'b1;
        end
        else begin
            cnt_s1 <= CNT;
            cnt_s2 <= cnt_s1;
            cnt_s3 <= cnt_s2;
            sp_s1  <= SP;
            sp_s2  <= sp_s1;
        end
    end

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            shift_in <= 8'd0;
            sdr_in   <= 8'd0;
        end
        else if (rx_bit) begin
            shift_in <= {shift_in[6:0], sp_s2};
            if (shift_in_counter == 3'd7)
                sdr_in <= {shift_in[6:0], sp_s2};
        end
    end

    // Bit counter.  Cleared when the port is switched to output (a real CIA
    // resets its shift counter on an SPMODE change - the 1581 ROM relies on
    // that) and on every access to the data register (6522 behaviour, so that
    // byte framing re-synchronises once per byte instead of drifting forever).
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            shift_in_counter <= 3'd0;
        else if (sp_output || acc_sdr)
            shift_in_counter <= 3'd0;
        else if (rx_bit)
            shift_in_counter <= shift_in_counter + 3'd1;
    end

    // ------------------------------------------------------------------
    // Serial port transmit.  Only ever used to send the wake-up byte that
    // makes the drive believe we can do burst transfers (the data itself is
    // irrelevant), so there is no second-byte buffer.
    // ------------------------------------------------------------------
    reg [7:0] sdr_out;
    reg shift_out_running;
    reg [7:0] shift_out;
    reg [2:0] shift_out_counter;
    reg shift_out_clk;

    wire shift_out_complete = shift_out_running && (shift_out_counter == 3'd7)
                              && shift_out_clk && ta_underflowing;

    // ------------------------------------------------------------------
    // Status flag (bit 3 of the control/status register).
    // Set on shift in/out completed, cleared on register write or data read.
    // ------------------------------------------------------------------
    reg shift_complete_latched;

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            shift_complete_latched <= 1'b0;
        else if (byte_done || shift_out_complete)
            shift_complete_latched <= 1'b1;
        else if (clr_flag)
            shift_complete_latched <= 1'b0;
    end

    // Register writes.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            sp_output <= 1'b0;
            sdr_out   <= 8'd0;
        end
        else if (seladdr && !RW) begin
            case (A[0])
                REG_SDR: sdr_out   <= D;
                REG_CRA: sp_output <= D[6];
            endcase
        end
    end

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            shift_out         <= 8'd0;
            shift_out_clk     <= 1'b0;
            shift_out_counter <= 3'd0;
        end
        else if (!sp_output || (wr_cra && !D[6])) begin
            shift_out         <= 8'd0;
            shift_out_clk     <= 1'b0;
            shift_out_counter <= 3'd0;
        end
        else if (shift_out_running && ta_underflowing) begin
            if (!shift_out_clk) begin
                if (shift_out_counter == 3'd0)
                    shift_out <= sdr_out;
                else
                    shift_out <= {shift_out[6:0], 1'b0};
            end
            else
                shift_out_counter <= shift_out_counter + 3'd1;

            shift_out_clk <= !shift_out_clk;
        end
    end

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            shift_out_running <= 1'b0;
        else if (!sp_output || (wr_cra && !D[6]))
            shift_out_running <= 1'b0;
        else if (wr_sdr)
            shift_out_running <= 1'b1;
        else if (shift_out_complete)
            shift_out_running <= 1'b0;
    end

    // Open drain outputs, exactly as on a real CIA: assert = pull to 0,
    // otherwise release and let the 3k3 pull-up restore the 1.
    assign SP  = (sp_output && !shift_out[7]) ? 1'b0 : 1'bz;
    assign CNT = (sp_output &&  shift_out_clk) ? 1'b0 : 1'bz;

    // ------------------------------------------------------------------
    // Reading.  Data is driven only while MUX is low, which is the window in
    // which the Plus/4 address bus belongs to the CPU.
    // ------------------------------------------------------------------
    reg [7:0] data_out;
    wire drive_data_out = seladdr && RW && !MUX;
    assign D = drive_data_out ? data_out : 8'bz;

    always @(*) begin
        if (seladdr) begin
            case (A[0])
                REG_SDR: data_out <= sdr_in;
                REG_CRA: data_out <= {1'b0, sp_output, 2'b0, shift_complete_latched, 3'b0};
            endcase
        end
    end

endmodule
