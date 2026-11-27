#!/usr/bin/env python3
"""
test_mqtt_8pub_8sub.py - Verify the PHASE 2 windows of SimplePLC over Modbus (NOT run on a board yet):

    0x1800..0x197F   MQTT publish   blocks 4..7  (4 x 96 registers)
    0x1980..0x1AFF   MQTT subscribe blocks 2..7  (6 x 64 registers)
    0x1200..0x127F   PULSE_OUT configuration     (8 x 16 registers; storage only, no pulse behaviour)
    0x1280..0x12FF   inside the PULSE window, reads as 0, never writable

Requires: pip install pymodbus pyserial.
Firmware: SPLC_MQTT_ENABLE=1, SPLC_MQTT_SUB_ENABLE=1, SPLC_PULSE_ENABLE=1 (splcopts.h). Its helpers come
from test_mqtt.py / test_tagtype.py in the same folder, like test_mqtt_sub.py.

WARNING: this test WRITES Flash (rule table + every block), REPLACES the rule table and runs
CLEAR_RULES / FACTORY_RESET. Dev board only. It also wipes the MQTT, SUB and MB_READ configuration
that is on the board: Deploy again from Studio afterwards.

This script needs NO broker. It checks the configuration path only (draft, COMMIT, read-back, Flash
restore). It does NOT check that the new publish/subscribe blocks really publish or receive: that
needs the board connected to a broker and is NOT covered here.

Sections (the T-number is the section number; names are this script's own):
  T0  probe: the three windows accept a write; capability word 0x0A08 is not changed by phase 2
  T1  clean start (CLEAR_RULES): every new window reads 0
  T2  draft invisible before COMMIT; byte-exact read-back after it, with legacy blocks 0..3 / sub 0..1
      written in the same Deploy and read back unchanged at their old addresses
  T3  address bounds: a write that crosses 0x0D7F|0x0D80, 0x197F|0x1980, past 0x1AFF or into
      0x1280.. answers exception 0x02 and changes nothing
  T4  a bad ENABLED block in ANY new window rejects the WHOLE COMMIT (error 2); running blocks unchanged;
      a bad DISABLED sub block is not checked
  T5  a Deploy that does not write the new windows keeps them; FC06 changes one register only
  T6  (--reboot) every window identical after REBOOT (restored from Flash, EXT section); CLEAR_RETAIN
      keeps them; CLEAR_RULES and FACTORY_RESET clear them, also after REBOOT
  T7  (--reboot) writing zeros to every NEW window and COMMIT removes the EXT section: new windows
      read 0 after REBOOT while the legacy blocks stay. (Flash content itself is not visible over
      Modbus: this checks the behaviour, not the bytes.)

Usage:
    python test_mqtt_8pub_8sub.py COM17 --unit 1 --baudrate 115200 [--reboot] [--port-after COM18]
    python test_mqtt_8pub_8sub.py COM17 --only T4          (one section; T6/T7 need --reboot)

Exit codes: 0 = ALL PASS, 1 = a check failed, 2 = cannot open port, 3 = link error.
"""
import argparse
import sys

import test_report as R   # Excel evidence report (--report / --no-report)
import test_mqtt as T
import test_mqtt_sub as S
import test_tagtype as TT
from test_mqtt import check, LinkError

REG_PUB_EXT, PUB_EXT_BLOCKS, PUB_BLOCK_REGS = 0x1800, 4, 96
PUB_EXT_REGS = PUB_EXT_BLOCKS * PUB_BLOCK_REGS            # 384 -> 0x1800..0x197F
REG_SUB_EXT, SUB_EXT_BLOCKS, SUB_BLOCK_REGS = 0x1980, 6, 64
SUB_EXT_REGS = SUB_EXT_BLOCKS * SUB_BLOCK_REGS            # 384 -> 0x1980..0x1AFF
REG_PULSE, PULSE_BLOCKS, PULSE_BLOCK_REGS = 0x1200, 8, 16
PULSE_REGS = PULSE_BLOCKS * PULSE_BLOCK_REGS              # 128 -> 0x1200..0x127F
PULSE_WINDOW = 256                                        # 0x1200..0x12FF
REG_PUB_LEGACY, PUB_LEGACY_REGS = 0x0C00, 384             # blocks 0..3
REG_SUB_LEGACY, SUB_LEGACY_REGS = 0x0D80, 128             # blocks 0..1
REG_CAPS = 0x0A08
READ_CHUNK = 64
ALL_SECTIONS = ("T0", "T1", "T2", "T3", "T4", "T5", "T6", "T7")


# ------------------------------------------------------------------ encoding
def pulse_block(mode=1, pin=0, count=1, trig=0, flags=0x01, width=1000, freq=0, duty=0,
                ch_tag=0xFFFF, par_tag=0xFFFF, color=0, tags=(0xFFFF,) * 4, reserved=0):
    """16 registers of one PULSE block (layout of plc_pulse.h; decoded register by register)."""
    return [(mode << 8) | pin, count, (trig << 8) | flags, width,
            (freq >> 16) & 0xFFFF, freq & 0xFFFF, duty, ch_tag, par_tag,
            (color >> 16) & 0xFFFF, color & 0xFFFF, *tags, reserved]


def pub_block(i, tag, mode=None, enabled=True, tags=None):
    """A valid publish block (96 regs) for extension block index i (4..7)."""
    return T.make_block(enabled, T.MODE_PERIODIC if mode is None else mode, qos=i % 3, interval=2000 + i,
                        tags=[tag] if tags is None else tags, topic=f"splc/p2/pub{i}")


def sub_block(i, tag, enabled=True, qos=0, topic=None, key=""):
    return S.block(enabled=enabled, qos=qos, topic=topic or f"splc/p2/sub{i}",
                   pins=[S.pin(tag, S.DT_INT32, i, key)])


def read_range(d, base, count):
    out = []
    for off in range(0, count, READ_CHUNK):
        out += d.read(base + off, min(READ_CHUNK, count - off))
    return out


def read_pub_ext(d):
    return read_range(d, REG_PUB_EXT, PUB_EXT_REGS)


def read_sub_ext(d):
    return read_range(d, REG_SUB_EXT, SUB_EXT_REGS)


def read_pulse_win(d):
    return read_range(d, REG_PULSE, PULSE_WINDOW)


def image(blocks, block_regs, total):
    img = [0] * total
    for i, b in blocks.items():
        img[i * block_regs:(i + 1) * block_regs] = b
    return img


def write_img(d, base, blocks, block_regs):
    """FC16 every block of `blocks` (index -> registers); returns the first error response or None."""
    for i, b in blocks.items():
        resp = d.write_regs(base + i * block_regs, b)
        if resp is not None:
            return resp
    return None


class State:
    """What the board should be running (as register images), for comparisons."""

    def __init__(self):
        self.pub_leg, self.sub_leg = {}, {}
        self.pub_ext, self.sub_ext, self.pulse = {}, {}, {}

    def all_new_zero(self):
        self.pub_ext, self.sub_ext, self.pulse = {}, {}, {}

    def clear_all(self):
        self.pub_leg, self.sub_leg = {}, {}
        self.all_new_zero()


def snapshot(d):
    return {"pub_leg": read_range(d, REG_PUB_LEGACY, PUB_LEGACY_REGS),
            "sub_leg": read_range(d, REG_SUB_LEGACY, SUB_LEGACY_REGS),
            "pub_ext": read_pub_ext(d), "sub_ext": read_sub_ext(d), "pulse": read_pulse_win(d)}


def expected(st):
    """Expected register images. The legacy telemetry window also carries the broker area beyond block 3
    only above 0x0D7F, so the 384 registers 0x0C00..0x0D7F are exactly blocks 0..3."""
    return {"pub_leg": image(st.pub_leg, PUB_BLOCK_REGS, PUB_LEGACY_REGS),
            "sub_leg": image(st.sub_leg, SUB_BLOCK_REGS, SUB_LEGACY_REGS),
            "pub_ext": image(st.pub_ext, PUB_BLOCK_REGS, PUB_EXT_REGS),
            "sub_ext": image(st.sub_ext, SUB_BLOCK_REGS, SUB_EXT_REGS),
            "pulse": image(st.pulse, PULSE_BLOCK_REGS, PULSE_WINDOW)}


def compare(d, st, what):
    got, exp = snapshot(d), expected(st)
    ok = True
    for k in ("pub_leg", "sub_leg", "pub_ext", "sub_ext", "pulse"):
        same = got[k] == exp[k]
        ok &= check(same, f"{what}: {k} window byte-exact {'' if same else (TT.diff(got[k], exp[k]) or '')}")
    return ok


def write_state(d, st):
    """Write every block of the state into the drafts; returns the first error response or None."""
    for base, blocks, regs in ((REG_PUB_LEGACY, st.pub_leg, PUB_BLOCK_REGS), (REG_SUB_LEGACY, st.sub_leg, SUB_BLOCK_REGS),
                               (REG_PUB_EXT, st.pub_ext, PUB_BLOCK_REGS), (REG_SUB_EXT, st.sub_ext, SUB_BLOCK_REGS),
                               (REG_PULSE, st.pulse, PULSE_BLOCK_REGS)):
        resp = write_img(d, base, blocks, regs)
        if resp is not None:
            return resp
    return None


def deploy_ok(d, rule, what):
    s, e = T.commit(d, [rule])
    return check(s == T.CFG_READY and e == 0, f"{what}: COMMIT ok (status {s}, error {e})")


def deploy_rejected(d, rule, what):
    s, e = T.commit(d, [rule])
    return check(s != T.CFG_READY and e == T.ERR_INVALID_PARAMETER,
                 f"{what}: COMMIT rejected (status {s}, error {e}; expect error 2)")


def self_check_encoding():
    """Offline sanity of the encoders against the documented layouts (runs before touching the board)."""
    assert len(pulse_block()) == PULSE_BLOCK_REGS
    b = pulse_block(mode=3, pin=5, count=16, trig=1, flags=0x0A, width=0x1234, freq=0x00010002,
                    duty=0x6419, ch_tag=7, par_tag=8, color=0x00FF8040, tags=(1, 2, 3, 4))
    assert b[0] == 0x0305 and b[1] == 16 and b[2] == 0x010A and b[3] == 0x1234
    assert (b[4], b[5]) == (0x0001, 0x0002) and b[6] == 0x6419 and (b[7], b[8]) == (7, 8)
    assert (b[9], b[10]) == (0x00FF, 0x8040) and b[11:15] == [1, 2, 3, 4] and b[15] == 0
    assert len(pub_block(4, 1)) == PUB_BLOCK_REGS and len(sub_block(2, 1)) == SUB_BLOCK_REGS
    assert PUB_EXT_REGS == 384 and SUB_EXT_REGS == 384
    assert REG_PUB_EXT + PUB_EXT_REGS - 1 == 0x197F and REG_SUB_EXT + SUB_EXT_REGS - 1 == 0x1AFF
    assert REG_PULSE + PULSE_REGS - 1 == 0x127F and REG_PULSE + PULSE_WINDOW - 1 == 0x12FF
    st = State()
    st.pub_ext[3] = pub_block(7, 1)
    assert expected(st)["pub_ext"][3 * 96:4 * 96] == st.pub_ext[3]


def build_state(tags):
    """A state with every new block configured (+ some legacy blocks), all different from each other."""
    A, B, C, D = tags
    st = State()
    st.pub_leg = {0: pub_block(0, A), 3: pub_block(3, B)}
    st.sub_leg = {0: sub_block(0, A, key="k0"), 1: sub_block(1, B, qos=1, key="k1")}
    st.pub_ext = {i - 4: pub_block(i, (A, B, C, D)[i - 4]) for i in range(4, 8)}
    st.sub_ext = {i - 2: sub_block(i, (A, B, C, D)[(i - 2) % 4], qos=i % 3, key=f"k{i}") for i in range(2, 8)}
    st.pulse = {0: pulse_block(mode=1, pin=0, flags=0x01),
                3: pulse_block(mode=2, pin=3, count=5, trig=1, width=500, freq=1000, duty=0x6414),
                7: pulse_block(mode=3, pin=7, count=64, flags=0x08, color=0x00112233, tags=(A, B, C, 0xFFFF))}
    return st


# ------------------------------------------------------------------ sections
def run(d, args):
    only = set(s.upper() for s in args.only) if args.only else set(ALL_SECTIONS)
    sel = lambda t: t in only
    r = d.read(T.REG_RESOURCE, 10)
    di, do, ai, vflag, vreg, retain = r[3], r[4], r[5], r[6], r[7], r[8]
    print(f"Resource: DI={di} DO={do} AI={ai} VFLAG={vflag} VREG={vreg} RETAIN={retain}")
    if vreg < 8:
        print("Need >= 8 VREG tags")
        return None
    vr0 = di + do + ai + vflag
    rule = T.encode_rule(T.TRG_INTERVAL, T.OP_NONE, T.ACT_INC, 0, vr0, for_ms=60000, action_param=1)
    tags = (vr0 + 1, vr0 + 2, vr0 + 3, vr0 + 4)
    st = State()

    print("\nT0. Probe")
    caps0, err = d.try_read(REG_CAPS, 2)
    for name, addr in (("0x1800 (publish 4..7)", REG_PUB_EXT), ("0x1980 (subscribe 2..7)", REG_SUB_EXT),
                       ("0x1200 (PULSE)", REG_PULSE)):
        resp = d.write_regs(addr, [0])
        if resp is not None:
            check(False, f"{name} accepts a write: firmware built with the phase-2 options (MQTT, MQTT_SUB, PULSE = 1)")
            print("  Set SPLC_MQTT_ENABLE, SPLC_MQTT_SUB_ENABLE, SPLC_PULSE_ENABLE to 1 in splcopts.h, clean rebuild, flash. Stopping.")
            return None
        check(True, f"{name} accepts a write")
    deploy_ok(d, rule, "Deploy (drops the probe drafts)")
    caps1, err = d.try_read(REG_CAPS, 2)
    if caps0 is None or caps1 is None:
        check(False, f"0x0A08 readable (capabilities): {err}")
    else:
        check(caps0 == caps1, f"capability word 0x0A08 unchanged by phase 2 (0x{(caps1[0] << 16) | caps1[1]:08X}); no new bit by decision 103")

    if sel("T1") or sel("T2") or sel("T3") or sel("T4") or sel("T5") or sel("T6") or sel("T7"):
        print("\nT1. Clean start: CLEAR_RULES")
        s, e = T.run_syscmd(d, T.SYS_CLEAR_RULES)
        check(s == T.ST_DONE and e == 0, f"CLEAR_RULES result {s}/{e} (expect DONE/0)")
        check(all(v == 0 for v in read_pub_ext(d)), "publish 4..7 window (0x1800..0x197F) reads 0")
        check(all(v == 0 for v in read_sub_ext(d)), "subscribe 2..7 window (0x1980..0x1AFF) reads 0")
        check(all(v == 0 for v in read_pulse_win(d)), "PULSE window (0x1200..0x12FF) reads 0")

    if sel("T2"):
        print("\nT2. Draft invisible before COMMIT, byte-exact after it")
        want = build_state(tags)
        check(write_state(d, want) is None, "FC16 of every block of every window accepted (draft)")
        compare(d, State(), "before COMMIT the RUNNING (empty) blocks are still returned")
        if deploy_ok(d, rule, "Deploy"):
            st = want
        compare(d, st, "after COMMIT")

    if sel("T3"):
        print("\nT3. Address bounds (nothing may change)")
        before = snapshot(d)
        cases = (("0x0D7F..0x0D80 (legacy telemetry into subscribe)", REG_SUB_LEGACY - 1, 2),
                 ("0x197F..0x1980 (publish ext into subscribe ext)", REG_SUB_EXT - 1, 2),
                 ("0x1AFF..0x1B00 (past the subscribe ext window)", REG_SUB_EXT + SUB_EXT_REGS - 1, 2),
                 ("0x127F..0x1280 (PULSE blocks into the unused half)", REG_PULSE + PULSE_REGS - 1, 2),
                 ("0x12FF..0x1300 (past the PULSE window)", REG_PULSE + PULSE_WINDOW - 1, 2),
                 ("0x1280 (unused half of the PULSE window)", REG_PULSE + PULSE_REGS, 1))
        for name, addr, n in cases:
            resp = d.write_regs(addr, [0x5A5A] * n)
            check(resp is not None and T.exc_code(resp) == T.EXC_ADDR, f"write at {name} answers exception 0x02")
        check(not d.fc16(REG_PUB_EXT + PUB_EXT_REGS - 1, [0]).isError(), "FC16 of the very last register of 0x197F accepted")
        check(not d.fc16(REG_SUB_EXT + SUB_EXT_REGS - 1, [0]).isError(), "FC16 of the very last register of 0x1AFF accepted")
        check(not d.fc16(REG_PULSE + PULSE_REGS - 1, [0]).isError(), "FC16 of the very last PULSE register (0x127F) accepted")
        deploy_ok(d, rule, "Deploy (drops the drafts)")
        after = snapshot(d)
        check(after == before, "every window unchanged after the address tests")
        check(all(v == 0 for v in read_pulse_win(d)[PULSE_REGS:]), "0x1280..0x12FF still read 0")

    if sel("T4"):
        print("\nT4. A bad ENABLED block rejects the whole COMMIT")
        before = snapshot(d)
        good = build_state(tags)
        cases = (
            ("publish 5 enabled with 0 tags", REG_PUB_EXT + 1 * PUB_BLOCK_REGS, T.make_block(True, T.MODE_PERIODIC, tags=[], topic="splc/p2/bad")),
            ("subscribe 6 with QoS 3", REG_SUB_EXT + 4 * SUB_BLOCK_REGS, sub_block(6, tags[0], qos=3)),
            ("subscribe 3 enabled with an empty topic", REG_SUB_EXT + 1 * SUB_BLOCK_REGS, sub_block(3, tags[0], topic=" ")),
            ("PULSE block 2 with mode 9", REG_PULSE + 2 * PULSE_BLOCK_REGS, pulse_block(mode=9)),
            ("PULSE block 4 with trigger_mode 2", REG_PULSE + 4 * PULSE_BLOCK_REGS, pulse_block(trig=2)),
            ("PULSE block 5 with reserved register != 0", REG_PULSE + 5 * PULSE_BLOCK_REGS, pulse_block(reserved=1)),
        )
        for name, addr, regs in cases:
            if name.startswith("subscribe 3"):
                regs = list(regs)
                regs[8:24] = [0] * 16          # topic registers all 0 = empty topic
            ok_w = d.write_regs(addr, regs) is None
            check(ok_w, f"{name}: draft write ACKed (checked at COMMIT)")
            # a GOOD block next to it must not be applied either (all-or-nothing)
            d.write_regs(REG_PUB_EXT + 0 * PUB_BLOCK_REGS, pub_block(4, tags[3], mode=T.MODE_ON_CHANGE))
            deploy_rejected(d, rule, name)
            check(snapshot(d) == before, f"{name}: running blocks unchanged (the good block next to it was not applied either)")
        print("  disabled blocks are not checked:")
        bad_off = sub_block(5, tags[0], enabled=False, qos=3)
        d.write_regs(REG_SUB_EXT + 3 * SUB_BLOCK_REGS, bad_off)
        deploy_ok(d, rule, "Deploy with a DISABLED subscribe block 5 that has QoS 3")
        st = State()
        st.sub_ext = {3: bad_off}
        # the board now runs the earlier state + that block; verify the block only (others as before)
        check(read_sub_ext(d)[3 * SUB_BLOCK_REGS:4 * SUB_BLOCK_REGS] == bad_off, "disabled block read back as written")
        # restore a known state for the next sections
        s, e = T.run_syscmd(d, T.SYS_CLEAR_RULES)
        check(s == T.ST_DONE and e == 0, f"CLEAR_RULES result {s}/{e}")
        want = good
        write_state(d, want)
        if deploy_ok(d, rule, "re-deploy the full state"):
            st = want
        else:
            st = State()

    if sel("T5") and not st.pulse:
        print("\n(T5 needs the state of T2: writing it first)")
        want = build_state(tags)
        write_state(d, want)
        if deploy_ok(d, rule, "Deploy"):
            st = want
    if sel("T5"):
        print("\nT5. A Deploy without the blocks keeps them; FC06 changes one register")
        deploy_ok(d, rule, "Deploy without writing any block")
        compare(d, st, "blocks kept")
        # one register of PULSE block 3 (width) through FC06
        addr = REG_PULSE + 3 * PULSE_BLOCK_REGS + 3
        check(not d.fc06(addr, 777).isError(), "FC06 on one PULSE register accepted")
        deploy_ok(d, rule, "Deploy")
        st.pulse[3] = list(st.pulse[3])
        st.pulse[3][3] = 777
        compare(d, st, "only that register changed")

    if args.reboot and (sel("T6") or sel("T7")):
        if not st.pulse:
            want = build_state(tags)
            write_state(d, want)
            if deploy_ok(d, rule, "Deploy (state for the reboot tests)"):
                st = want
    if sel("T6"):
        if not args.reboot:
            print("\nT6. (skipped: add --reboot)")
        else:
            print("\nT6. Persistence across REBOOT and the clear commands")
            nd = d.reboot(args)
            if nd is None:
                check(False, "board came back after REBOOT")
                return None
            d = nd
            compare(d, st, "after REBOOT (restored from Flash)")
            s, e = T.run_syscmd(d, T.SYS_CLEAR_RETAIN)
            check(s == T.ST_DONE and e == 0, f"CLEAR_RETAIN result {s}/{e}")
            compare(d, st, "CLEAR_RETAIN keeps the blocks")
            for name, cmd in (("CLEAR_RULES", T.SYS_CLEAR_RULES), ("FACTORY_RESET", T.SYS_FACTORY)):
                write_state(d, st)
                deploy_ok(d, rule, f"re-deploy the blocks before {name}")
                s, e = T.run_syscmd(d, cmd)
                check(s == T.ST_DONE and e == 0, f"{name} result {s}/{e}")
                compare(d, State(), f"{name} clears every window")
                nd = d.reboot(args)
                if nd is None:
                    check(False, "board came back after REBOOT")
                    return None
                d = nd
                compare(d, State(), f"...and they stay clear after REBOOT ({name} was saved)")

    if sel("T7"):
        if not args.reboot:
            print("\nT7. (skipped: add --reboot)")
        else:
            print("\nT7. Clearing only the NEW windows removes the EXT section")
            st = build_state(tags)
            write_state(d, st)
            deploy_ok(d, rule, "Deploy the full state")
            zero = State()
            zero.pub_leg, zero.sub_leg = st.pub_leg, st.sub_leg      # legacy blocks stay
            write_img(d, REG_PUB_EXT, {i: [0] * PUB_BLOCK_REGS for i in range(PUB_EXT_BLOCKS)}, PUB_BLOCK_REGS)
            write_img(d, REG_SUB_EXT, {i: [0] * SUB_BLOCK_REGS for i in range(SUB_EXT_BLOCKS)}, SUB_BLOCK_REGS)
            write_img(d, REG_PULSE, {i: [0] * PULSE_BLOCK_REGS for i in range(PULSE_BLOCKS)}, PULSE_BLOCK_REGS)
            deploy_ok(d, rule, "Deploy with every new block zero")
            compare(d, zero, "new windows 0, legacy blocks kept")
            nd = d.reboot(args)
            if nd is None:
                check(False, "board came back after REBOOT")
                return None
            d = nd
            compare(d, zero, "after REBOOT")
            st = zero

    print("\nCleanup (CLEAR_RULES)")
    s, e = T.run_syscmd(d, T.SYS_CLEAR_RULES)
    check(s == T.ST_DONE and e == 0, f"CLEAR_RULES result {s}/{e}")
    return d


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("port")
    p.add_argument("--unit", type=int, default=1)
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--chunk", type=int, default=48, help="registers per FC16 when writing the blocks (default 48)")
    p.add_argument("--reboot", action="store_true", help="also run T6 and T7 (persistence across REBOOT)")
    p.add_argument("--port-after", default=None, help="COM port after a reboot if Windows renumbers it")
    p.add_argument("--only", nargs="+", metavar="Tn", choices=ALL_SECTIONS, help="run only these sections")
    R.add_args(p)
    args = p.parse_args()
    R.start(args)
    self_check_encoding()
    T.Dev.chunk = max(1, min(args.chunk, 120))

    client = T.make_client(args.port, args.baudrate)
    if not client.connect():
        print(f"Cannot open {args.port}")
        sys.exit(2)
    d = T.Dev(client, args.unit)
    link_error = None
    try:
        res = run(d, args)
        if res is not None:
            d = res
    except LinkError as e:
        link_error = e
        print(f"\n  [LINK] {e}")
        T.cleanup_after_abort(d)
    except Exception:
        import traceback
        traceback.print_exc()
        print("\n  [ERROR] unexpected exception in the test script (not a firmware verdict)")
        T.cleanup_after_abort(d)
        T._failures += 1
    finally:
        try:
            d.c.close()
        except Exception:
            pass
    if T.Dev.retried or T.Dev.retried_w:
        print(f"\n(info) lost frames needing a retry: {T.Dev.retried} read(s), {T.Dev.retried_w} write(s)")
    if link_error is not None:
        print("\nINCONCLUSIVE: a request outcome was unknown (link error), NOT a firmware failure."
              f" {T._failures} check(s) failed before it.")
        sys.exit(3)
    print("\nALL PASS" if T._failures == 0 else f"\n{T._failures} CHECK(S) FAILED")
    sys.exit(0 if T._failures == 0 else 1)


if __name__ == "__main__":
    main()