/*
 * Implementation of 8520 Complex Interface Adapter (CIA) in Verilog.
 * Written by Niklas Ekström in June 2021.
 *
 * Features:
 * - 2x 8-bit I/O ports (PORTA, PORTB)
 * - Handshaking for I/O port communication (PC, FLAG)
 * - 2x interval timers (TA, TB)
 * - Time of Day clock (TOD)
 * - Serial port (SDR)
 * - Interrupt control (ICR)
 * - Control Registers (CRA, CRB)
 *
 * Features that are deliberately not implemented:
 * - Outputting TA/TB underflow on PB6/7 (PBON, OUTMODE bits are stuck zero)
 * - Using CNT as source for TA/TB (INMODE bit is stuck zero)
 *
 * Features that could probably be removed to save logic elements:
 * - I don't think the TOD alarm feature is used.
 *
 * Gotchas:
 * - The polarity of shift_out_clk and the CNT signal are inverted. It seems like
 *   CNT is active low (and perhaps should have been called CNT_n).
 */

/*
 * MW: options: XC9572XL-5VQ64 and 48/44 one
 * MW: removed:
 *     port A/B/FLAG/PC/TOD, control register B, timer A bits 4-15, IRQ, ICR, ICR mask
 *     changed:
 *		 timer A (counter and latch) is 3-bit now, without register, only internal counting down from 7, no latch, no stop
 *     only two registers: SDR data (R/W) vs serial status+output (R) / serial output (W)
 *     added:
 *     romcs, rom_a15, full address bus for I/O selection, MUX and optional: Phi2/AEC/BA, (E_CLK is Phi0)
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
	 assign rom_cs = !(!c1lo || !c1hi || !c2lo || !c2hi);
	 assign rom_a15 =!(!c1lo || !c1hi); // 1 for C1 (high 32K half, default for 32K ROM), 0 for C2 (low 32K half)

	 // Plus4 I/O
	 assign seladdr = A[15:4] == 12'hFD9;

	 // CIA
    localparam REG_SDR = 1'b0;
    localparam REG_CRA = 1'b1;

    // Control registers.
    reg sp_output;

    // Interval timers.
    reg [2:0] ta_counter;

    wire ta_underflowing = ta_counter == 3'd0;

    // Update ta_counter.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            ta_counter <= 3'd0;
        else begin
                if (ta_counter == 3'd0)
                    ta_counter <= 3'd7;
                else
                    ta_counter <= ta_counter - 3'd1;
        end
    end

    // Serial port.
    reg [7:0] sdr_in;
    reg [7:0] shift_in;
    reg [2:0] shift_in_counter;

    wire sp_in_reset_n = RESET_n && !sp_output;

    always @(posedge CNT or negedge sp_in_reset_n) begin
        if (!sp_in_reset_n) begin
            sdr_in <= 8'd0;
            shift_in <= 8'd0;
            shift_in_counter <= 3'd0;
        end
        else begin
            shift_in <= {shift_in[6:0], SP};
            if (shift_in_counter == 3'd7)
                sdr_in <= {shift_in[6:0], SP};
            shift_in_counter <= shift_in_counter + 3'd1;
        end
    end

    // Clock domain crossing for shift_in_complete.
    reg shift_in_complete_req;
    reg shift_in_complete_ack;

    always @(posedge CNT or negedge RESET_n) begin
        if (!RESET_n)
            shift_in_complete_req <= 1'b0;
        else if (!sp_output && shift_in_counter == 3'd7)
            shift_in_complete_req <= !shift_in_complete_ack;
    end

    reg shift_in_complete;

    always @(posedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            shift_in_complete <= 1'b0;
        else
            shift_in_complete <= shift_in_complete_req != shift_in_complete_ack;
    end

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            shift_in_complete_ack <= 1'b0;
        else if (shift_in_complete)
            shift_in_complete_ack <= shift_in_complete_req;
    end

    // Update sp_output.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            sp_output <= 1'b0;
        else if (seladdr && !RW && A[0] == REG_CRA)
            sp_output <= D[6];
    end

    reg [7:0] sdr_out;
    reg sdr_out_new_data;
    reg shift_out_running;
    reg [7:0] shift_out;
    reg [2:0] shift_out_counter;
    reg shift_out_clk;

    wire shift_out_complete = shift_out_running && shift_out_counter == 3'd7 && shift_out_clk && ta_underflowing;

    wire shift_complete = shift_in_complete | shift_out_complete;

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            sdr_out <= 8'd0;
        else if (seladdr && !RW && A[0] == REG_SDR)
            sdr_out <= D;
    end

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            shift_out <= 8'd0;
            shift_out_clk <= 1'b0;
            shift_out_counter <= 3'd0;
        end
        else if (sp_output) begin
            if (seladdr && !RW && A[0] == REG_CRA && !D[6]) begin
                // FIXME: Should this be combined with reset handling?
                shift_out <= 8'd0;
                shift_out_clk <= 1'b0;
                shift_out_counter <= 3'd0;
            end
            else begin
                if (shift_out_running && ta_underflowing) begin
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
        end
    end

    // Update shift_out_running, sdr_out_new_data.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            shift_out_running <= 1'b0;
            sdr_out_new_data <= 1'b0;
        end
        else if (sp_output) begin
            if (seladdr && !RW && A[0] == REG_CRA && !D[6]) begin
                // FIXME: Should this be combined with reset handling?
                shift_out_running <= 1'b0;
                sdr_out_new_data <= 1'b0;
            end
            else if (seladdr && !RW && A[0] == REG_SDR) begin
                if (!shift_out_running || shift_out_complete)
                    shift_out_running <= 1'b1;
                else
                    sdr_out_new_data <= 1'b1;
            end
            else if (shift_out_complete) begin
                if (!sdr_out_new_data)
                    shift_out_running <= 1'b0;
                else
                    sdr_out_new_data <= 1'b0;
            end
        end
    end

    assign SP = sp_output && !shift_out[7] ? 1'b0 : 1'bz;
    assign CNT = sp_output && shift_out_clk ? 1'b0 : 1'bz;

    reg [7:0] data_out;
	 wire drive_data_out = seladdr && RW && !MUX; // F
	 assign D = drive_data_out ? data_out : 8'bz; 

    // Reading.
    //reg [7:0] data_out;
    //wire drive_data_out = E_CLK && !CS_n && RW;
    //assign D = drive_data_out ? data_out : 8'bz;

    always @(*) begin
		if (seladdr) begin
        case (A[0])
            REG_SDR: data_out <= sdr_in;
            REG_CRA: data_out <= {1'b0, sp_output, 2'b0, shift_complete, 3'b0};
        endcase
		end
    end

endmodule