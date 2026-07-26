#!/usr/bin/env python3
"""
Cycle-exact model of the rewritten ciasdr.v.

The whole design is now clocked from negedge E_CLK, so one evaluation per E_CLK
period reproduces the RTL exactly.  Nonblocking assignment is modelled by
computing all next-state values from the current state before committing.

Checks performed:
  1. the wake-up sequence from parobek/burst-cpld.asm (write CRA=$40, write
     SDR=$08, poll status bit 3) still completes and raises the flag;
  2. a 1581 burst stream is received byte-exact;
  3. the same, with noise on the SRQ rising edge;
  4. a deliberately injected stray SRQ edge mid-stream: the old design is
     corrupted from that point on for ever, the new one loses at most one byte
     and then re-synchronises.
"""

US = 1000.0

ECLK_HZ = 886724.0
P = 1e9 / ECLK_HZ               # ns per E_CLK period

BIT_PERIOD = 7 * US
HALF_BIT = BIT_PERIOD / 2
TURNAROUND = 27 * US


# ---------------------------------------------------------------- drive ----
class Drive:
    """1581 burst transmitter: 7 us/bit, MSB first, data changes on the falling
    edge of CNT and stays valid until the next falling edge."""

    def __init__(self, payload, glitch_ns=0.0, stray_at=None):
        self.payload = list(payload)
        self.i = 0
        self.state = "wait_ack"
        self.tn = 0.0
        self.byte = 0
        self.bit = 0
        self.cnt_high = True
        self.srq_low = False
        self.data_low = False
        self.release_t = None
        self.last_clk = 0
        self.glitch = glitch_ns          # width of one threshold re-crossing
        self.stray_at = stray_at         # byte index after which to inject a stray edge
        self.stray_done = False
        self.stray_t = None

    def advance(self, t, host_clk):
        moved = True
        while moved:
            moved = False
            if self.state == "wait_ack":
                if host_clk != self.last_clk:
                    self.last_clk = host_clk
                    if self.i >= len(self.payload):
                        self.state = "done"
                    else:
                        self.state = "turn"
                        self.tn = t + TURNAROUND
                    moved = True
            elif self.state == "turn":
                if t >= self.tn:
                    self.byte = self.payload[self.i]
                    self.i += 1
                    self.bit = 0
                    self.cnt_high = True
                    self.state = "shift"
                    self.tn = t
                    moved = True
            elif self.state == "shift":
                if t >= self.tn:
                    if self.cnt_high:
                        bitval = (self.byte >> (7 - self.bit)) & 1
                        self._set_data(bitval == 0, t)
                        self._set_srq(True, t)
                        self.cnt_high = False
                        self.tn = t + HALF_BIT
                    else:
                        self._set_srq(False, t)
                        self.cnt_high = True
                        self.bit += 1
                        if self.bit == 8:
                            self.state = "wait_ack"
                            if (self.stray_at is not None
                                    and self.i - 1 == self.stray_at
                                    and not self.stray_done):
                                # one extra SRQ pulse 5 us after the byte ends
                                self.stray_done = True
                                self.stray_t = t + 5 * US
                        else:
                            self.tn = t + HALF_BIT
                    moved = True

    def _set_srq(self, low, t):
        if low and not self.srq_low:
            self.srq_low = True
            self.release_t = None
        elif not low and self.srq_low:
            self.srq_low = False
            self.release_t = t

    def _set_data(self, low, t):
        self.data_low = low

    def srq(self, t):
        # stray pulse: SRQ low for 2 us, modelling one spurious edge
        if self.stray_t is not None and self.stray_t <= t < self.stray_t + 2 * US:
            return 0
        if self.srq_low:
            return 0
        if self.release_t is None:
            return 1
        dt = t - self.release_t
        # slow RC ramp: line reaches the threshold after RISE ns, then noise
        # pulls it back below the threshold once for `glitch` ns.
        if self.glitch:
            RISE = 700.0
            if dt < RISE:
                return 0
            if RISE + self.glitch <= dt < RISE + 2 * self.glitch:
                return 0                   # one threshold re-crossing
        return 1

    def sp(self, t):
        return 0 if self.data_low else 1


# ------------------------------------------------------------ new design ----
class NewRtl:
    """Faithful model of the rewritten ciasdr.v (all negedge E_CLK)."""
    name = "new ciasdr.v"

    def __init__(self):
        self.cnt_s1 = self.cnt_s2 = self.cnt_s3 = 1
        self.sp_s1 = self.sp_s2 = 1
        self.sdr_in = 0
        self.shift_in = 0
        self.cnt = 0                    # shift_in_counter
        self.flag = 0
        self.sp_output = 0
        self.sdr_out = 0
        self.shift_out = 0
        self.shift_out_counter = 0
        self.shift_out_clk = 0
        self.shift_out_running = 0
        self.ta = 0

    # combinational, from current state
    def cnt_rise(self):
        return self.cnt_s2 == 1 and self.cnt_s3 == 0

    def rx_bit(self):
        return (not self.sp_output) and self.cnt_rise()

    def byte_done(self):
        return self.rx_bit() and self.cnt == 7

    def shift_out_complete(self):
        return (self.shift_out_running and self.shift_out_counter == 7
                and self.shift_out_clk and self.ta == 0)

    def drives_cnt_low(self):
        return self.sp_output and self.shift_out_clk

    def drives_sp_low(self):
        return self.sp_output and not ((self.shift_out >> 7) & 1)

    def negedge(self, cnt_pin, sp_pin, seladdr, rw, a0, dbus):
        wr_cra = seladdr and (not rw) and a0 == 1
        wr_sdr = seladdr and (not rw) and a0 == 0
        acc_sdr = seladdr and a0 == 0
        clr_flag = (seladdr and not rw) or acc_sdr

        rxb = self.rx_bit()
        bdone = self.byte_done()
        soc = self.shift_out_complete()
        cra_off = wr_cra and ((dbus >> 6) & 1) == 0

        # --- next state ---
        n_cnt_s1, n_cnt_s2, n_cnt_s3 = cnt_pin, self.cnt_s1, self.cnt_s2
        n_sp_s1, n_sp_s2 = sp_pin, self.sp_s1

        n_shift_in, n_sdr_in = self.shift_in, self.sdr_in
        if rxb:
            n_shift_in = ((self.shift_in << 1) | self.sp_s2) & 0xFF
            if self.cnt == 7:
                n_sdr_in = n_shift_in

        if self.sp_output or acc_sdr:
            n_cnt = 0
        elif rxb:
            n_cnt = (self.cnt + 1) & 7
        else:
            n_cnt = self.cnt

        if bdone or soc:
            n_flag = 1
        elif clr_flag:
            n_flag = 0
        else:
            n_flag = self.flag

        n_sp_output, n_sdr_out = self.sp_output, self.sdr_out
        if seladdr and not rw:
            if a0 == 0:
                n_sdr_out = dbus
            else:
                n_sp_output = (dbus >> 6) & 1

        n_so, n_soclk, n_socnt = self.shift_out, self.shift_out_clk, self.shift_out_counter
        if (not self.sp_output) or cra_off:
            n_so, n_soclk, n_socnt = 0, 0, 0
        elif self.shift_out_running and self.ta == 0:
            if not self.shift_out_clk:
                n_so = self.sdr_out if self.shift_out_counter == 0 \
                       else ((self.shift_out << 1) & 0xFF)
            else:
                n_socnt = (self.shift_out_counter + 1) & 7
            n_soclk = 0 if self.shift_out_clk else 1

        if (not self.sp_output) or cra_off:
            n_sor = 0
        elif wr_sdr:
            n_sor = 1
        elif soc:
            n_sor = 0
        else:
            n_sor = self.shift_out_running

        n_ta = 0 if self.ta else 1

        # --- commit ---
        (self.cnt_s1, self.cnt_s2, self.cnt_s3) = (n_cnt_s1, n_cnt_s2, n_cnt_s3)
        (self.sp_s1, self.sp_s2) = (n_sp_s1, n_sp_s2)
        self.shift_in, self.sdr_in, self.cnt = n_shift_in, n_sdr_in, n_cnt
        self.flag = n_flag
        self.sp_output, self.sdr_out = n_sp_output, n_sdr_out
        self.shift_out, self.shift_out_clk, self.shift_out_counter = n_so, n_soclk, n_socnt
        self.shift_out_running = n_sor
        self.ta = n_ta

    def read(self, a0):
        return self.sdr_in if a0 == 0 else ((self.sp_output << 6) | (self.flag << 3))


# ------------------------------------------------------------ old design ----
class OldRtl:
    """ciasdr.v as it was: shifter clocked by the raw CNT pin, flag latch on
    posedge E_CLK, bit counter never resynchronised."""
    name = "old ciasdr.v"

    def __init__(self):
        self.sdr_in = 0
        self.shift_in = 0
        self.cnt = 0
        self.flag = 0
        self.req = 0
        self.ack = 0
        self.complete = 0
        self.sp_output = 0
        self.prev_pin = 1

    def pin_edges(self, cnt_pin, sp_pin):
        if cnt_pin == 1 and self.prev_pin == 0 and not self.sp_output:
            self.shift_in = ((self.shift_in << 1) | sp_pin) & 0xFF
            if self.cnt == 7:
                self.sdr_in = self.shift_in
                self.req ^= 1
            self.cnt = (self.cnt + 1) & 7
        self.prev_pin = cnt_pin

    def posedge(self, seladdr, rw, a0):
        if self.complete:
            self.flag = 1
        elif seladdr and ((not rw) or a0 == 0):
            self.flag = 0
        self.complete = 1 if self.req != self.ack else 0

    def negedge_ack(self):
        if self.complete:
            self.ack = self.req

    def read(self, a0):
        return self.sdr_in if a0 == 0 else ((self.sp_output << 6) | (self.flag << 3))


# ------------------------------------------------------------------ host ----
class Host:
    """GetByte / GetAndStore from parobek/burst-cpld.asm, real cycle counts."""

    def __init__(self, want):
        self.clk = 0
        self.received = []
        self.want = want
        self.phase = "initack"
        self.wait = 0
        self.bus = (False, True, 0, 0)
        self.done = False
        self.spins = 0

    def cycle(self, rx):
        if self.done:
            self.bus = (False, True, 0, 0)
            return
        if self.wait > 0:
            self.wait -= 1
            self.bus = (False, True, 0, 0)
            return
        if self.phase == "initack":
            self.bus = (True, True, 0, 0)          # lda cpldbase  (clears flag)
            self.clk ^= 1                           # jsr ToggleClk
            self.phase = "poll"
            self.wait = 8
        elif self.phase == "poll":
            self.bus = (True, True, 1, 0)           # bit cpldbase+1
            if rx.read(1) & 0x08:
                self.phase = "read"
                self.wait = 3
            else:
                self.wait = 6
                self.spins += 1
        elif self.phase == "read":
            self.bus = (True, True, 0, 0)           # ldy cpldbase
            self.received.append(rx.read(0))
            self.phase = "ack"
            self.wait = 5
        elif self.phase == "ack":
            self.bus = (False, False, 0, 0)
            self.clk ^= 1                           # sta $01 -> toggle IEC CLK
            if len(self.received) >= self.want:
                self.done = True
            self.phase = "poll"
            self.wait = 30


# ------------------------------------------------------------------- runs ---
def run_new(payload, glitch=0.0, stray_at=None, limit_us=40000):
    rx = NewRtl()
    drv = Drive(payload, glitch_ns=glitch, stray_at=stray_at)
    host = Host(len(payload))
    t = 0.0
    k = 0
    while t < limit_us * US and not host.done and drv.state != "done":
        t = k * P
        drv.advance(t, host.clk)
        host.cycle(rx)
        seladdr, rw, a0, dbus = host.bus
        # resolve the open-drain lines: drive pulls low, CPLD may pull low too
        cnt_pin = 0 if (drv.srq(t) == 0 or rx.drives_cnt_low()) else 1
        sp_pin = 0 if (drv.sp(t) == 0 or rx.drives_sp_low()) else 1
        rx.negedge(cnt_pin, sp_pin, seladdr, rw, a0, dbus)
        k += 1
    return host, drv


def run_old(payload, glitch=0.0, stray_at=None, limit_us=40000):
    rx = OldRtl()
    drv = Drive(payload, glitch_ns=glitch, stray_at=stray_at)
    host = Host(len(payload))
    t = 0.0
    k = 0
    SUB = 16                              # sub-sample the pin for async edges
    while t < limit_us * US and not host.done and drv.state != "done":
        base = k * P
        for s in range(SUB):
            t = base + s * P / SUB
            drv.advance(t, host.clk)
            cnt_pin = 0 if drv.srq(t) == 0 else 1
            rx.pin_edges(cnt_pin, drv.sp(t))
        host.cycle(rx)
        seladdr, rw, a0, dbus = host.bus
        rx.posedge(seladdr, rw, a0)
        rx.negedge_ack()
        k += 1
    return host, drv


def wakeup_test():
    """CRA=$40 (serial out), SDR=$08, then poll status bit 3."""
    rx = NewRtl()
    # cycle 1: write $40 to $FD91
    rx.negedge(1, 1, True, False, 1, 0x40)
    assert rx.sp_output == 1, "sp_output should be set"
    # cycle 2: write $08 to $FD90
    rx.negedge(1, 1, True, False, 0, 0x08)
    edges = 0
    prev = rx.shift_out_clk
    for i in range(200):
        rx.negedge(1, 1, False, True, 0, 0)
        if rx.shift_out_clk and not prev:
            edges += 1
        prev = rx.shift_out_clk
        if rx.flag:
            return True, i, edges
    return False, 200, edges


if __name__ == "__main__":
    payload = [((i * 37 + 11) & 0xFF) for i in range(24)]
    print(f"PHI0 = {ECLK_HZ/1e6:.3f} MHz ({P:.0f} ns), "
          f"1581 bit period {BIT_PERIOD/1000:.1f} us "
          f"({HALF_BIT/P:.2f} PHI0 cycles per CNT phase)\n")

    ok, cycles, edges = wakeup_test()
    print(f"1. wake-up handshake (CRA=$40, SDR=$08, poll bit 3): "
          f"{'flag raised' if ok else 'FLAG NEVER RAISED'} after {cycles} PHI0 "
          f"cycles, {edges} CNT pulses generated")

    def report(tag, fn, **kw):
        host, drv = fn(payload, **kw)
        got = host.received
        exp = payload[:len(got)]
        okc = sum(1 for a, b in zip(got, exp) if a == b)
        bad = [i for i, (a, b) in enumerate(zip(got, exp)) if a != b]
        verdict = "PASS" if (len(got) == len(payload) and not bad) else "FAIL"
        print(f"   {tag:52s} recv={len(got):2d}/{len(payload)} "
              f"correct={okc:2d} wrong_at={bad[:6]} -> {verdict}")

    print("\n2. clean 1581 stream")
    report("new ciasdr.v", run_new)
    report("old ciasdr.v", run_old)

    print("\n3. 200 ns threshold re-crossing on every SRQ rising edge")
    report("new ciasdr.v", run_new, glitch=200.0)
    report("old ciasdr.v", run_old, glitch=200.0)

    print("\n4. one stray SRQ pulse after byte 5 (recovery behaviour)")
    report("new ciasdr.v", run_new, stray_at=5)
    report("old ciasdr.v", run_old, stray_at=5)
