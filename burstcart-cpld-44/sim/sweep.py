import burstsim as B

payload = [((i*37+11)&0xFF) for i in range(16)]

def trial(cls, rise, bounce, width, eclk_hz, mux_noise=False):
    B.ECLK_HZ = eclk_hz
    B.ECLK_PERIOD = int(round(1e9/eclk_hz))
    orig = B.OpenDrainLine.__init__
    def patched(self, rise_ns=0, bounce=0, bounce_width_ns=width):
        orig(self, rise_ns, bounce, width)
    B.OpenDrainLine.__init__ = patched
    rx = cls()
    host, drive = B.run(payload, rx, rise_ns=rise, bounce=bounce,
                        max_us=20000, mux_noise=mux_noise)
    B.OpenDrainLine.__init__ = orig
    got = host.received
    exact = (len(got) == len(payload) and all(a==b for a,b in zip(got,payload)))
    return ("PASS" if exact else
            f"FAIL({len(got)}/{len(payload)},{sum(1 for a,b in zip(got,payload) if a==b)}ok)")

print("Glitch-width sweep -- 1 extra threshold re-crossing per SRQ rising edge")
print("(SRQ rise time fixed at 700 ns; CNT phase = 3.5 us)\n")
hdr = f"{'glitch':>8} {'PHI0':>9} | {'cpld_old':>22} {'cpld_new':>22} {'via_ref':>22}"
print(hdr); print("-"*len(hdr))
for eclk in (886724, 1773448):
    for w in (50, 100, 200, 400, 800, 1600):
        r = [trial(c, 700, 1, w, eclk) for c in (B.CpldOld, B.CpldNew, B.ViaRef)]
        print(f"{w:6d}ns {eclk/1e6:6.3f}MHz | {r[0]:>22} {r[1]:>22} {r[2]:>22}")

print("\nClean-signal baseline and MUX hazard at both Plus/4 clock rates\n")
hdr2 = f"{'case':>26} {'PHI0':>9} | {'cpld_old':>22} {'cpld_new':>22} {'via_ref':>22}"
print(hdr2); print("-"*len(hdr2))
for eclk in (886724, 1773448):
    for name, rise, mux in (("clean, no MUX hazard", 0, False),
                            ("slow edge, no glitch", 700, False),
                            ("clean + MUX hazard", 0, True)):
        r = [trial(c, rise, 0, 300, eclk, mux_noise=mux) for c in (B.CpldOld, B.CpldNew, B.ViaRef)]
        print(f"{name:>26} {eclk/1e6:6.3f}MHz | {r[0]:>22} {r[1]:>22} {r[2]:>22}")
