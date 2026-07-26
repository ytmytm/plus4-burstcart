#!/usr/bin/env python3
"""
Replays the EXACT InitBurst + LoadBurst detection sequence from
parobek/burst-cpld.asm against the cycle-exact model of the rewritten
ciasdr.v, to confirm no software change is needed.

cpldbase = $FD90  (burstcart.asm:17)
  $FD90 = SDR      (A0 = 0)
  $FD91 = CRA/status (A0 = 1), bit 6 = direction, bit 3 = shift complete
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_new import NewRtl

IDLE = (False, True, 0, 0)          # seladdr, rw, a0, dbus


class Cpu:
    """Drives the model one PHI0 cycle at a time.  A read returns the value the
    6502 would latch at the end of PHI2, i.e. sampled before the negedge."""

    def __init__(self, rtl):
        self.rtl = rtl
        self.log = []

    def idle(self, n=1):
        for _ in range(n):
            self.rtl.negedge(1, 1, *IDLE)

    def read(self, a0):
        v = self.rtl.read(a0)                       # combinational, pre-edge
        self.rtl.negedge(1, 1, True, True, a0, 0)
        return v

    def write(self, a0, val):
        self.rtl.negedge(1, 1, True, False, a0, val)


def main():
    rtl = NewRtl()
    cpu = Cpu(rtl)
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(f"   [{'ok ' if cond else 'FAIL'}] {msg}")
        if not cond:
            ok = False

    print("InitBurst:")
    # lda #0 / sta cpldbase+1        ; serial IN; clear flag
    cpu.idle(2)
    cpu.write(1, 0x00)
    cpu.idle(2)
    check(rtl.sp_output == 0, "sta $FD91,#$00  -> sp_output = 0 (serial IN)")
    check(rtl.flag == 0, "                -> flag cleared")

    print("\nLoadBurst, CPLD presence detection:")
    # lda cpldbase+1 / cmp cpldbase+1 / bne NotCPLD
    a = cpu.read(1)
    cpu.idle(3)
    b = cpu.read(1)
    check(a == b, f"lda $FD91 / cmp $FD91  -> ${a:02X} == ${b:02X} (branch not taken)")

    # lda #%01000000 / sta cpldbase+1 / cmp cpldbase+1 / bne NotCPLD
    cpu.idle(2)
    cpu.write(1, 0x40)
    cpu.idle(3)
    c = cpu.read(1)
    check(c == 0x40, f"sta $FD91,#$40 / cmp $FD91 -> ${c:02X} == $40 "
                     f"(sp_output readable, flag still clear)")

    # lda #8 / sta cpldbase          ; send the wake-up byte
    cpu.idle(2)
    cpu.write(0, 0x08)
    check(rtl.shift_out_running == 1, "sta $FD90,#$08 -> transmit started")

    # ldy #0 / - iny / bmi NotCPLD / bit cpldbase+1 / bne CPLDFound / beq -
    y = 0
    edges = 0
    prev_clk = rtl.shift_out_clk
    found = False
    while y < 128:
        y += 1                                       # iny
        cpu.idle(2)                                  # iny(2)
        cpu.idle(2)                                  # bmi not taken
        v = cpu.read(1)                              # bit $FD91
        for _ in range(3):
            if rtl.shift_out_clk and not prev_clk:
                edges += 1
            prev_clk = rtl.shift_out_clk
            cpu.idle(1)
        if v & 0x08:
            found = True
            break
    check(found, f"wait loop  -> flag raised at Y={y} (timeout is Y=128)")
    check(edges >= 1, f"           -> CNT pulses generated on the wire: {edges}")

    # CPLDFound: lda #0 / sta cpldbase+1
    cpu.idle(2)
    cpu.write(1, 0x00)
    cpu.idle(2)
    check(rtl.sp_output == 0, "sta $FD91,#$00 -> back to serial IN")
    check(rtl.flag == 0, "               -> flag cleared, ready to receive")
    check(rtl.cnt == 0, "               -> receive bit counter aligned at 0")

    print("\nRESULT:", "burst-cpld.asm needs NO change" if ok
          else "SOFTWARE CHANGE REQUIRED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
