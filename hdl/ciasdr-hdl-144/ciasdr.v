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
 * MW: options: XC95144XL-5TQ100, XC9572XL-5VQ64 and 48/44 one?
 * MW: removed:
 *     port A/B/FLAG/PC/TOD, control register B, timer A high byte
 *     changed:
 *		 timer A (counter and latch) is 4-bit now, with one register
 *     added:
 *     romcs, rom_a15, full address bus for I/O selection, MUX and optional: Phi2/AEC/BA, (E_CLK is Phi0)
 *     _reset 3.3v (commented out b/c the device doesn't fit otherwise)
 *     what can be optimized (after testing):
 *     - timer A can be running all the time (oneshot)
 *     - timer A can be 1-4 bits wide with fixed top value (1 or 3 or 7 or 15)
 *     - with fixed value no need for timer A latch, register
 *     - no need to indicate timer A underflow
 *     - no need for IRQ
 *     - no need for timer A at all, can count phi2 pulses
 *     - only one status bit (serial port complete shift in/out)
 *     - only two registers: SDR data, control (read: status bit, write: serial direction in/out)
 */

module cia(
    // Chip access control.
    input RESET_n,
    input E_CLK, // PHI0
    input RW,
	 input MUX,
    input [15:0] A,
    inout [7:0] D,

	 // unused inputs
	 input phi2, aec, ba,

    // Serial port.
    inout CNT,
    inout SP,

	 // ROM
	 input c1lo, c1hi, c2lo, c2hi,

	 output rom_a15,
	 output rom_cs,

	 // 3.3V reset
	 output _resetout,

	 // just to keep pin
	 output _unused,

    // Interrupt.
    output IRQ_n
    );

	 // some combination just to keep phi2+aec+ba in the model and auto assign pins
	 assign _unused = phi2 && aec && ba;

	 // 3.3V RESET only low or floating
	 assign _resetout = !RESET_n ? RESET_n : 1'bz;

	 // ROM
	 assign rom_cs = !(!c1lo || !c1hi || !c2lo || !c2hi);
	 assign rom_a15 =!(!c1lo || !c1hi); // 1 for C1 (high 32K half, default for 32K ROM), 0 for C2 (low 32K half)

	 // Plus4 I/O
	 assign seladdr = A[15:4] == 12'hFD9;

	 // CIA
    localparam [3:0] REG_TA_LO = 4'h4;
    localparam [3:0] REG_SDR = 4'hc;
    localparam [3:0] REG_ICR = 4'hd;
    localparam [3:0] REG_CRA = 4'he;

    // Control registers.
    reg ta_running;
    reg ta_oneshot;
    reg sp_output;

    wire [7:0] cra = {1'b0, sp_output, 2'b00, ta_oneshot, 2'b00, ta_running};

    // Interval timers.
    reg [3:0] ta_counter;

    reg [3:0] ta_latch;

    wire ta_underflowing = ta_running && ta_counter == 4'd0;

    // Update ta_latch, tb_latch.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            ta_latch <= 4'd0;
        end
        else if (seladdr && !RW && A[3:0] == REG_TA_LO) begin
				ta_latch <= D[3:0];
        end
    end

    // Update ta_counter.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            ta_counter <= 4'd0;
        else begin
            if (seladdr && !RW && A[3:0] == REG_CRA && D[4])
                ta_counter <= ta_latch;
            else if (ta_running) begin
                if (ta_counter == 4'd0)
                    ta_counter <= ta_latch;
                else
                    ta_counter <= ta_counter - 4'd1;
            end
        end
    end

    // Update ta_running.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            ta_running <= 1'b0;
        else begin
            if (seladdr && !RW && A[3:0] == REG_CRA)
                ta_running <= D[0];
            else if (ta_underflowing && ta_oneshot)
                ta_running <= 1'b0;
        end
    end

    // Update ta_oneshot, tb_oneshot, tb_count_ta_underflow.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            ta_oneshot <= 1'b0;
        end
        else if (seladdr && !RW && A[3:0] == REG_CRA) begin
				ta_oneshot <= D[3];
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
        else if (seladdr && !RW && A[3:0] == REG_CRA)
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
        else if (seladdr && !RW && A[3:0] == REG_SDR)
            sdr_out <= D;
    end

    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n) begin
            shift_out <= 8'd0;
            shift_out_clk <= 1'b0;
            shift_out_counter <= 3'd0;
        end
        else if (sp_output) begin
            if (seladdr && !RW && A[3:0] == REG_CRA && !D[6]) begin
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
            if (seladdr && !RW && A[3:0] == REG_CRA && !D[6]) begin
                // FIXME: Should this be combined with reset handling?
                shift_out_running <= 1'b0;
                sdr_out_new_data <= 1'b0;
            end
            else if (seladdr && !RW && A[3:0] == REG_SDR) begin
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

    // Interrupt handling.
    reg [4:0] icr_mask;
    reg [4:0] icr_data;

    wire ir = |(icr_data & icr_mask);
    assign IRQ_n = ir ? 1'b0 : 1'bz;

    // Update icr_mask.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            icr_mask <= 5'd0;
        else if (seladdr && !RW && A[3:0] == REG_ICR) begin
            if (D[7])
                icr_mask <= icr_mask | D[4:0];
            else
                icr_mask <= icr_mask & ~D[4:0];
        end
    end

    wire [4:0] icr_set = {1'b0, shift_complete, 1'b0, 1'b0, ta_underflowing};

    // Update icr_data.
    always @(negedge E_CLK or negedge RESET_n) begin
        if (!RESET_n)
            icr_data <= 5'd0;
        else begin
            if (seladdr && RW && A[3:0] == REG_ICR)
                icr_data <= icr_set;
            else
                icr_data <= icr_data | icr_set;
        end
    end

    reg [7:0] data_out;
	 wire drive_data_out = seladdr && RW && !MUX; // F
	 assign D = drive_data_out ? data_out : 8'bz; 

    // Reading.
    //reg [7:0] data_out;
    //wire drive_data_out = E_CLK && !CS_n && RW;
    //assign D = drive_data_out ? data_out : 8'bz;

    always @(*) begin
		if (seladdr) begin
        case (A[3:0])
            REG_TA_LO: data_out <= {4'b0, ta_counter};
            REG_SDR: data_out <= sdr_in;
            REG_ICR: data_out <= {ir, 2'b00, icr_data};
            REG_CRA: data_out <= cra;
        endcase
		end
    end

endmodule