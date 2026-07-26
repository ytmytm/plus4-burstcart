#!/usr/bin/env python3
"""
burstsim.py -- bit-level simulation of the Commodore fast-serial (burst) receive
path, comparing:

  * "cpld_old"  : ciasdr.v as it stands  -- shift register clocked directly by the
                  raw IEC SRQ line  (always @(posedge CNT)), bit counter only
                  reset when SPMODE goes to output, flag latch sampled on
                  posedge PHI0 and NOT qualified by MUX.
  * "cpld_new"  : proposed fix -- CNT/SP synchronised into the PHI0 domain,
                  edge-detected there, bit counter re-initialised on SDR access
                  (the documented 6522 behaviour), flag latch sampled on
                  negedge PHI0 and qualified by !MUX.
  * "via_ref"   : the working burstcart-via arrangement -- 74LS74 resynchronises
                  SRQ to the system clock, then the 6522 shifts "during the first
                  PHI2 cycle following the positive going edge of CB1" and
                  re-initialises the modulo-8 counter on every SR read.

Drive model = MOS 6526 in a 1581 as programmed by the 1581 ROM:
    Timer A latch = 6 @ 2 MHz  ->  underflow every 3.5 us
    CNT toggles on each underflow  ->  7 us bit period, 50% duty, MSB first
    data changes on the FALLING edge of CNT, valid until the next falling edge
    (so the RISING edge sits mid-bit with 3.5 us setup and hold)
    per-byte handshake: drive waits for the host to TOGGLE IEC CLK

Host model = the GetByte/GetAndStore loop from parobek/burst-cpld.asm with real
6502 cycle counts, running on a Plus/4 at 0.886 MHz.

The "slow edge" model is the point of the exercise: IEC SRQ is an open-drain
line with a 3k3 pull-up to +5V feeding TWO IEC connectors' worth of cable
capacitance.  Its rising edge is a slow RC ramp of roughly 0.5-1 us, and the
XC9572XL global clock input has no hysteresis, so noise on that ramp re-crosses
the threshold and clocks the shift register more than once.
"""

NS = 1
US = 1000

DT = 100 * NS                   # simulation timestep

ECLK_HZ = 886724                # Plus/4 PAL, single clock rate
ECLK_PERIOD = int(round(1e9 / ECLK_HZ))     # 1128 ns

BIT_PERIOD = 7 * US             # 1581: Timer A = 6 @ 2 MHz, CNT toggles per underflow
HALF_BIT = BIT_PERIOD // 2

DRIVE_TURNAROUND = 27 * US      # measured from the 1581 ROM: ack seen -> SDR write


# --------------------------------------------------------------------------
# IEC line model:  open drain, 3k3 pull-up.  Falling edges are fast (active
# pull-down).  Rising edges are a slow RC ramp on which noise can re-cross the
# CPLD's threshold.
# --------------------------------------------------------------------------
class OpenDrainLine:
    def __init__(self, rise_ns=0, bounce=0, bounce_width_ns=300):
        self.driven_low = False
        self.rise_ns = rise_ns          # RC ramp length on release
        self.bounce = bounce            # extra threshold re-crossings per rising edge
        self.bounce_width = bounce_width_ns
        self.release_t = None
        self._level = 1

    def set_low(self, low, t):
        if low and not self.driven_low:
            self.driven_low = True
            self.release_t = None
        elif not low and self.driven_low:
            self.driven_low = False
            self.release_t = t

    def level(self, t):
        """Logic level seen by a receiver input with no hysteresis."""
        if self.driven_low:
            return 0
        if self.release_t is None:
            return 1
        dt = t - self.release_t
        if dt < self.rise_ns:
            return 0                                    # still below threshold
        # after crossing, inject `bounce` narrow dips back below threshold
        for k in range(self.bounce):
            lo = self.rise_ns + (2 * k + 1) * self.bounce_width
            hi = lo + self.bounce_width
            if lo <= dt < hi:
                return 0
        return 1


# --------------------------------------------------------------------------
# Drive: 1581 burst transmitter
# --------------------------------------------------------------------------
class Drive:
    def __init__(self, payload, srq, data):
        self.payload = list(payload)
        self.srq = srq
        self.data = data
        self.idx = 0
        self.state = "wait_ack"
        self.t_next = 0
        self.shifter = 0
        self.bit = 0
        self.cnt_high = True
        self.last_clk = 0           # remembered IEC CLK level ($76 bit 2)
        self.sent = []

    def step(self, t, host_clk):
        if self.state == "wait_ack":
            if host_clk != self.last_clk:
                self.last_clk = host_clk
                if self.idx >= len(self.payload):
                    self.state = "done"
                    return
                self.state = "turnaround"
                self.t_next = t + DRIVE_TURNAROUND

        elif self.state == "turnaround":
            if t >= self.t_next:
                self.shifter = self.payload[self.idx]
                self.sent.append(self.payload[self.idx])
                self.idx += 1
                self.bit = 0
                self.state = "shift"
                self.t_next = t
                self.cnt_high = True

        elif self.state == "shift":
            if t >= self.t_next:
                if self.cnt_high:
                    # falling edge of CNT: drive the next data bit out (MSB first)
                    bitval = (self.shifter >> (7 - self.bit)) & 1
                    self.data.set_low(bitval == 0, t)
                    self.srq.set_low(True, t)
                    self.cnt_high = False
                    self.t_next = t + HALF_BIT
                else:
                    # rising edge of CNT: data stays valid across it
                    self.srq.set_low(False, t)
                    self.cnt_high = True
                    self.bit += 1
                    if self.bit == 8:
                        self.state = "wait_ack"
                    else:
                        self.t_next = t + HALF_BIT


# --------------------------------------------------------------------------
# Receiver 1: ciasdr.v as written -- raw SRQ used as a hardware clock
# --------------------------------------------------------------------------
class CpldOld:
    name = "cpld_old (ciasdr.v as-is)"

    def __init__(self):
        self.shift_in = 0
        self.counter = 0
        self.sdr_in = 0
        self.flag = 0
        self.prev_cnt = 1
        self.req = 0
        self.ack = 0
        self.complete = 0
        self.extra_edges = 0

    def cnt_edge(self, sp):
        """always @(posedge CNT): async, straight off the pin."""
        newbit = sp
        self.shift_in = ((self.shift_in << 1) | newbit) & 0xFF
        if self.counter == 7:
            self.sdr_in = self.shift_in
            self.req ^= 1
        self.counter = (self.counter + 1) & 7

    def tick(self, cnt, sp, eclk_rising, eclk_falling, seladdr, rw, a0):
        if cnt == 1 and self.prev_cnt == 0:
            self.cnt_edge(sp)
        self.prev_cnt = cnt

        # flag latch: posedge E_CLK, no MUX qualification (as in ciasdr.v)
        if eclk_rising:
            if self.complete:
                self.flag = 1
            elif seladdr and (not rw or a0 == 0):
                self.flag = 0
            self.complete = 1 if (self.req != self.ack) else 0
        if eclk_falling:
            if self.complete:
                self.ack = self.req

    def read_sdr(self):
        return self.sdr_in


# --------------------------------------------------------------------------
# Receiver 2: proposed fix -- everything in the PHI0 domain
# --------------------------------------------------------------------------
class CpldNew:
    name = "cpld_new (PHI0-synchronous + counter resync)"

    def __init__(self):
        self.shift_in = 0
        self.counter = 0
        self.sdr_in = 0
        self.flag = 0
        self.cnt_s1 = 1
        self.cnt_s2 = 1
        self.cnt_d = 1
        self.sp_s = 1

    def tick(self, cnt, sp, eclk_rising, eclk_falling, seladdr, rw, a0):
        if eclk_rising:
            cnt_rise = (self.cnt_s2 == 1 and self.cnt_d == 0)
            if cnt_rise:
                self.shift_in = ((self.shift_in << 1) | self.sp_s) & 0xFF
                if self.counter == 7:
                    self.sdr_in = self.shift_in
                    self.flag = 1
                self.counter = (self.counter + 1) & 7
            # 2-stage synchroniser on CNT, 1 stage on SP (sampled a cycle before
            # the edge is acted on, i.e. mid-bit -- huge margin)
            self.cnt_d = self.cnt_s2
            self.cnt_s2 = self.cnt_s1
            self.cnt_s1 = cnt
            self.sp_s = sp

        if eclk_falling:
            # bus side sampled at the end of PHI2, qualified by !MUX
            if seladdr and (not rw or a0 == 0):
                self.flag = 0
                self.counter = 0          # 6522-style resync on SDR access

    def read_sdr(self):
        return self.sdr_in


# --------------------------------------------------------------------------
# Receiver 3: the working VIA board -- 74LS74 resync + 6522 mode 011
# --------------------------------------------------------------------------
class ViaRef:
    name = "via_ref (74LS74 resync + 6522 mode 011)"

    def __init__(self):
        self.shift_in = 0
        self.counter = 0
        self.sdr_in = 0
        self.flag = 0
        self.ff_q = 1           # 74LS74 output -> CB1
        self.cb1_prev = 1

    def tick(self, cnt, sp, eclk_rising, eclk_falling, seladdr, rw, a0):
        if eclk_rising:
            cb1 = self.ff_q
            if cb1 == 1 and self.cb1_prev == 0:
                self.shift_in = ((self.shift_in << 1) | sp) & 0xFF
                if self.counter == 7:
                    self.sdr_in = self.shift_in
                    self.flag = 1
                self.counter = (self.counter + 1) & 7
            self.cb1_prev = cb1
            self.ff_q = cnt                     # 74LS74 clocked by the system clock
        if eclk_falling:
            if seladdr and (not rw or a0 == 0):
                self.flag = 0
                self.counter = 0                # datasheet: SR access resyncs counter

    def read_sdr(self):
        return self.sdr_in


# --------------------------------------------------------------------------
# Host: the GetByte / GetAndStore loop, real 6502 cycle counts
# --------------------------------------------------------------------------
class Host:
    """
    GetByte:  lda #8            (outside loop)
          -   bit cpldbase+1    4
              beq -             3
              ldy cpldbase      4   <- reads SDR, clears flag
              lda $01           3
              eor #%00000010    2
              sta $01           3   <- ack (toggle IEC CLK)
              tya               2
              rts               6
    GetAndStore adds: jsr 6, inc TED_BORDER 6, ldy #0 2, sta (),y 6,
              inc 5, bne 3, dex 2, bne 3
    """
    def __init__(self, nbytes):
        self.clk = 0
        self.received = []
        self.want = nbytes
        self.phase = "initack"
        self.cycles_left = 0
        self.bus = (False, True, 0)      # seladdr, rw, a0
        self.done = False
        self.hangs = 0

    def bus_state(self):
        return self.bus

    def cycle(self, rx):
        """Called once per E_CLK cycle.  Returns nothing; drives self.bus."""
        if self.done:
            self.bus = (False, True, 0)
            return
        if self.cycles_left > 0:
            self.cycles_left -= 1
            self.bus = (False, True, 0)
            return

        if self.phase == "initack":
            # loader does:  lda cpldbase (clear flag) ; jsr ToggleClk
            self.bus = (True, True, 0)
            self.clk ^= 1
            self.phase = "poll"
            self.cycles_left = 8
            return

        if self.phase == "poll":
            # cycle 4 of `bit cpldbase+1`: read $FD91 (seladdr, rw=1, a0=1)
            self.bus = (True, True, 1)
            if rx.flag:
                self.phase = "readsdr"
                self.cycles_left = 3          # rest of bit + beq not taken
            else:
                self.cycles_left = 6          # bit(4)+beq(3) - this cycle
                self.hangs += 1
            return

        if self.phase == "readsdr":
            # cycle 4 of `ldy cpldbase`: read $FD90 (seladdr, rw=1, a0=0)
            self.bus = (True, True, 0)
            self.received.append(rx.read_sdr())
            self.phase = "ack"
            self.cycles_left = 5              # lda $01 (3) + eor (2)
            return

        if self.phase == "ack":
            self.bus = (False, False, 0)
            self.clk ^= 1                     # sta $01 -> toggle IEC CLK
            if len(self.received) >= self.want:
                self.done = True
            self.phase = "poll"
            self.cycles_left = 30             # tya,rts,store loop overhead
            return


# --------------------------------------------------------------------------
# Test harness
# --------------------------------------------------------------------------
def run(payload, rx, rise_ns=0, bounce=0, max_us=20000, mux_noise=False):
    srq = OpenDrainLine(rise_ns=rise_ns, bounce=bounce)
    data = OpenDrainLine()
    drive = Drive(payload, srq, data)
    host = Host(len(payload))

    t = 0
    prev_eclk = 0
    limit = max_us * US
    while t < limit and not host.done and drive.state != "done":
        eclk = 1 if (t % ECLK_PERIOD) < ECLK_PERIOD // 2 else 0
        rising = (eclk == 1 and prev_eclk == 0)
        falling = (eclk == 0 and prev_eclk == 1)
        prev_eclk = eclk

        drive.step(t, host.clk)

        cnt = srq.level(t)
        sp = data.level(t)

        if rising:
            host.cycle(rx)
        seladdr, rw, a0 = host.bus_state()

        # MUX: 1 during the first half of PHI2 (TED / row address on A0-A7),
        # 0 during the second half (CPU address valid).  A receiver that samples
        # the bus without checking !MUX sees a bogus address during MUX=1.
        mux = 1 if (t % ECLK_PERIOD) < ECLK_PERIOD // 2 else 0
        if mux_noise and mux == 1:
            seladdr, rw, a0 = True, True, 0      # spurious "SDR read"

        rx.tick(cnt, sp, rising, falling, seladdr, rw, a0)
        t += DT

    return host, drive


def compare(label, payload, rise_ns, bounce, mux_noise=False):
    print(f"\n=== {label} ===")
    print(f"    SRQ rise time {rise_ns} ns, {bounce} threshold re-crossing(s) "
          f"per rising edge, MUX hazard {'ON' if mux_noise else 'off'}")
    for cls in (CpldOld, CpldNew, ViaRef):
        rx = cls()
        host, drive = run(payload, rx, rise_ns=rise_ns, bounce=bounce,
                          mux_noise=mux_noise)
        got = host.received
        exp = payload[:len(got)]
        ok = sum(1 for a, b in zip(got, exp) if a == b)
        first_bad = next((i for i, (a, b) in enumerate(zip(got, exp))
                          if a != b), None)
        status = "OK" if (len(got) == len(payload) and ok == len(payload)) else "CORRUPT"
        print(f"    {cls.name:46s} recv={len(got):3d}/{len(payload)} "
              f"correct={ok:3d} first_bad={first_bad} -> {status}")
        if first_bad is not None and first_bad < len(got):
            e = exp[first_bad]; g = got[first_bad]
            print(f"        byte[{first_bad}] expected ${e:02X} ({e:08b})  "
                  f"got ${g:02X} ({g:08b})")


if __name__ == "__main__":
    import random
    random.seed(1)
    payload = [((i * 37 + 11) & 0xFF) for i in range(24)]

    print("1581 burst receive simulation")
    print(f"  bit period {BIT_PERIOD/1000:.1f} us, half {HALF_BIT/1000:.2f} us, "
          f"PHI0 {ECLK_HZ} Hz ({ECLK_PERIOD} ns) = "
          f"{HALF_BIT/ECLK_PERIOD:.2f} PHI0 cycles per CNT phase")

    compare("A. Ideal signal (clean, fast edges)", payload, 0, 0)
    compare("B. Slow RC rising edge, 3k3 pull-up + cable (no noise)",
            payload, 700, 0)
    compare("C. Slow RC rising edge with ONE threshold re-crossing",
            payload, 700, 1)
    compare("D. Slow RC rising edge with TWO threshold re-crossings",
            payload, 700, 2)
    compare("E. Clean signal, but Plus/4 MUX hazard on the flag-clear decode",
            payload, 0, 0, mux_noise=True)
