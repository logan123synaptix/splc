#!/usr/bin/env python3
"""
test_retain_1sector.py - Board test of the ONE-SECTOR retain page log
(services/plc_retain, format v1: 144-byte pages in sector DATA_BASE+0, compaction
when the main pages are used up, one-time migration from the old 208-byte format)
over Modbus RTU / USB-CDC.

NOT BUILT / NOT RUN on a board when this file was written. A PASS here is evidence only
after YOU have run it and read the log.

Requires: pip install pymodbus pyserial (same as the other test scripts). Put this file
in the same folder as test_retain.py, test_sysclear.py, test_rtc.py and test_report.py
(it reuses their helpers; the Modbus connection and the reboot/reconnect code are the
ones of test_retain.py / test_sysclear.py).

WARNING: phase "new" and "legacy-write" WRITE Flash and REBOOT the board. Phase "new"
also runs CLEAR_RULES and CLEAR_RETAIN (rules and retain values on the board are LOST).
It never runs FACTORY_RESET, so the Wi-Fi credentials and the TLS certificates are not
touched. Phase "legacy-write" only commits retain values (no clear).

PHASES (--phase)
  new            (default) NEW firmware. Needs reboots. Steps:
                   0. baseline CLEAR_RULES + CLEAR_RETAIN
                   1. all 32 retain tags written, COMMIT_RETAIN, REBOOT, read back equal
                   2. N consecutive COMMIT_RETAINs (--commits, default 60). The main pages
                      are 54: 60 commits cross at least one compaction (erase the sector,
                      write page 0). REBOOTs after commits 10, 30, 52, 54, 55, 57 and at the
                      end. The page where the test starts is not known (board history), so
                      the test cannot say WHICH commit compacts, only that none of them is
                      lost. --commits 120 crosses at least two compactions.
                   3. CLEAR_RETAIN after the page log has been used, REBOOT, still zero
                   4. rule save (Rule Table A/B sectors) and retain commit interleaved, REBOOT
                   5. (--periodic) 5 minute safety net, about 6 minutes
                   6. REBOOT right after a rule-driven change (retain saved at REBOOT)
  legacy-write   OLD firmware (the one BEFORE this change, e.g. commit ae3b993). Commits
                 --legacy-commits (default 45: the newest old record ends up in the SECOND
                 sector) and a last commit with a known value for every retain tag; saves the
                 expected values in retain_legacy_expected.json next to this script.
                 Then: dump the retain sector (STM32CubeProgrammer, address of
                 SPLC_FLASH_RETAIN_BASE_ADDR, 3 sectors = 24 KB; 0x08036000 on a 256 KB chip),
                 flash the NEW firmware, run phase legacy-verify.
  legacy-verify  NEW firmware, right after the first boot that follows the flash. Checks that
                 the values of the old format were migrated (live values equal, FLASH_DIRTY
                 clear), still equal after a REBOOT (now read from a page), and that a new
                 commit + REBOOT works. Needs retain_legacy_expected.json.

LIMIT OF THE "survives a REBOOT" CHECKS (read from plc_system_cmd_service.c, lines ~70-85):
the REBOOT command itself saves the retain tags first (retain_snapshot_write() when a tag
differs from the newest page). So a value that was never committed or never written by the
5 minute safety net still survives a REBOOT. Therefore:
  * step 1 and step 2 are made meaningful by two extra checks that do not use a REBOOT:
    FLASH_DIRTY must be clear right after each commit, and commit_values() must report no
    verify error (FLASH_CRC_MISMATCH). A page that was not written shows up there.
  * step 5 (--periodic) cannot tell the periodic write from the save at REBOOT (the same
    weakness as step 5 of test_retain.py). It only proves the value survives.
  * a power cut or a watchdog reset (no REBOOT command) is the only way to show what the
    firmware had really written; the board has no Modbus command for it.

What this test CANNOT see:
  * Which Flash page is used, whether the sector was erased, the "migrate"/"compaction"
    log lines: read them in the board's serial log (tag PLC_RETAIN) while the test runs.
  * The PVD (low-voltage) write: the board has no hold-up hardware, not testable here.
  * Power loss in the middle of a write or an erase.
  * That the Wi-Fi and cert sectors are untouched (check the boot log / reconnect after).

How the check "FLASH_DIRTY clear" works: the SYS_DIAG status_flags word (0x00F3, bit 0x0010)
is set by the firmware while retain_has_unsaved_changes() is true. After a successful
migration or restore it is clear; if the migration failed the firmware keeps it set (design
of plc_retain.c, verified only in a host simulation, not on a board).

The COM port drops during each reboot; the script reconnects (up to 20 s). If Windows gives
the board a NEW COM number, pass --port-after COMx.

Usage (one command per block):
  python test_retain_1sector.py COM14
  python test_retain_1sector.py COM14 --commits 120
  python test_retain_1sector.py COM14 --periodic
  python test_retain_1sector.py COM14 --phase legacy-write
  python test_retain_1sector.py COM14 --phase legacy-verify
"""
import argparse
import json
import os
import sys
import time

import test_report as R   # Excel evidence report (--report / --no-report)
from pymodbus.client import ModbusSerialClient

import test_sysclear as ts
import test_rtc as tr
import test_retain as base      # commit_values(), reboot(), pattern(), tags_equal(), clear_all()

check = ts.check

MAIN_PAGES = 54                               # splc_flash_define.h SPLC_RETAIN_MAIN_PAGES
REG_SYS_STATUS_FLAGS = 0x00F3                 # uptime u32 (F0,F1) | scan_time (F2) | status_flags
SYSF_FLASH_DIRTY = 0x0010
MID_REBOOTS = (10, 30, 52, 54, 55, 57)        # reboot after these commit numbers
EXPECTED_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "retain_legacy_expected.json")


# ---------------------------------------------------------------- helpers
def flash_dirty(d):
    return bool(d.read(REG_SYS_STATUS_FLAGS, 1)[0] & SYSF_FLASH_DIRTY)


def resource(d):
    r = d.read(ts.REG_RESOURCE, 10)
    di, do, ai, vflag, vreg, retain, counter = r[3], r[4], r[5], r[6], r[7], r[8], r[9]
    base_do = di
    base_retain = di + do + ai + vflag + vreg
    return {"do": do, "retain": retain, "base_do": base_do, "base_retain": base_retain}


def mism(bad):
    return f" (tag {bad[0][0]}: want {bad[0][1]}, got {bad[0][2]})" if bad else ""


def two_tag_items(rt0, rtl, i):
    items = {rt0: 100000 + i}
    items[rtl] = -i if rtl != rt0 else items[rt0]
    return items


# ---------------------------------------------------------------- phase: new
def phase_new(d, holder, args):
    res = resource(d)
    retain, RT0, DO0 = res["retain"], res["base_retain"], res["base_do"]
    print(f"retain tags: {retain}, RETAIN0 = tag {RT0}")
    if retain < 1:
        print("Board has no VREG_RETAIN group. Aborting.")
        return
    RTL = RT0 + retain - 1
    tags = list(range(RT0, RT0 + retain))
    ts.ensure_running(d)

    def all_zero():
        return all(d.tag(i) == 0 for i in tags)

    print("\n0. Baseline: CLEAR_RULES + CLEAR_RETAIN")
    check(base.clear_all(d), "CLEAR_RULES and CLEAR_RETAIN both DONE")
    check(all_zero(), f"all {retain} retain tag(s) read 0")

    print(f"\n1. All {retain} VREG_RETAIN tag(s) survive a REBOOT (needs reboot)")
    values = {RT0 + i: base.pattern(i) for i in range(retain)}
    check(base.commit_values(d, values), "all retain tags committed (no dirty flag, no verify error)")
    check(not flash_dirty(d), "FLASH_DIRTY clear after the commit (the page really was written and verified)")
    bad = base.tags_equal(d, values)
    check(not bad, "live values read back as written" + mism(bad))
    d = base.reboot(d, holder, args)
    if d is None:
        return
    bad = base.tags_equal(d, values)
    check(not bad, f"all {retain} values restored after REBOOT" + mism(bad))
    check(not flash_dirty(d), "FLASH_DIRTY clear right after boot (restored values equal Flash)")

    n = args.commits
    if n > 0:
        print(f"\n2. {n} consecutive COMMIT_RETAINs (54 main pages: expect at least one compaction)")
        if n <= MAIN_PAGES:
            print(f"  (info) {n} <= {MAIN_PAGES}: no compaction can be forced; use --commits {MAIN_PAGES + 6} or more")
        t0 = time.time()
        bad_commits = []
        for i in range(1, n + 1):
            if not base.commit_values(d, two_tag_items(RT0, RTL, i)):
                bad_commits.append(i)
            if i % 20 == 0:
                print(f"  ... {i}/{n} commits ({time.time() - t0:.0f} s)")
            if i in MID_REBOOTS and i < n:
                d = base.reboot(d, holder, args)
                if d is None:
                    return
                bad = base.tags_equal(d, two_tag_items(RT0, RTL, i))
                check(not bad, f"after commit {i} + REBOOT: newest values restored" + mism(bad))
        check(not bad_commits, f"all {n} commits clean (no dirty flag, no verify error)"
              + (f"; failed at {bad_commits[:10]}" if bad_commits else ""))
        want = two_tag_items(RT0, RTL, n)
        bad = base.tags_equal(d, want)
        check(not bad, f"live values after commit {n} are the newest" + mism(bad))
        d = base.reboot(d, holder, args)
        if d is None:
            return
        bad = base.tags_equal(d, want)
        check(not bad, f"after {n} commits + REBOOT: newest values restored" + mism(bad))
        check(not flash_dirty(d), "FLASH_DIRTY clear after the last REBOOT")
    else:
        print("\n2. skipped (--commits 0)")

    print("\n3. CLEAR_RETAIN after the page log has been used")
    st, err, _ = d.run_syscmd(ts.SYS_CLEAR_RETAIN)
    check(st == ts.ST_DONE and err == ts.ERR_NONE,
          f"CLEAR_RETAIN result = {ts.ST_NAMES.get(st, st)}/err {err} (expect DONE/0)")
    check(all_zero(), "all retain tags 0")
    d = base.reboot(d, holder, args)
    if d is None:
        return
    check(all_zero(), "retain still ZERO after REBOOT (the clear reached Flash)")

    print("\n4. Rule-table save (Rule A/B sectors) and retain commit interleaved")
    if res["do"] < 1:
        print("  skipped: board has no DO tag for the test rule")
    else:
        ts.upload_rule(d, DO0)
        check(d.rule_count() == 1, "RULE_TABLE_INFO == 1 after upload")
        check(base.commit_values(d, {RT0: 0xBEEF}), "RETAIN0 = 0xBEEF committed after the rule upload")
        d = base.reboot(d, holder, args)
        if d is None:
            return
        check(d.rule_count() == 1, "rule survives REBOOT")
        check(d.tag(RT0) == 0xBEEF, f"RETAIN0 survives REBOOT (got {d.tag(RT0)})")
        check(base.commit_values(d, {RT0: 0xCAFE}), "RETAIN0 = 0xCAFE committed before a second rule upload")
        ts.upload_rule(d, DO0)
        check(d.rule_count() == 1, "RULE_TABLE_INFO == 1 after the second upload")
        d = base.reboot(d, holder, args)
        if d is None:
            return
        check(d.rule_count() == 1, "rule survives REBOOT (second order)")
        check(d.tag(RT0) == 0xCAFE, f"RETAIN0 survives REBOOT (second order, got {d.tag(RT0)})")
        check(base.clear_all(d), "cleanup: CLEAR_RULES and CLEAR_RETAIN DONE")

    if args.periodic:
        print(f"\n5. Periodic safety net: no COMMIT_RETAIN, wait {args.period_s + 30} s")
        d = base.reboot(d, holder, args)
        if d is None:
            return
        check(base.clear_all(d), "baseline: rules and retain cleared")
        tr.set_time(d, tr.local_epoch(2026, 1, 15, 12, 0, 0, tr.TZ_VN))
        check(tr.upload_rules(d, [tr.time_rule(0, 0, 2359, RT0)]), "rule that counts RETAIN0 up committed")
        time.sleep(1.0)
        v0 = d.tag(RT0)
        time.sleep(0.5)
        check(d.tag(RT0) > v0, "RETAIN0 is counting")
        wait = args.period_s + 30
        t0 = time.time()
        while time.time() - t0 < wait:
            left = wait - (time.time() - t0)
            print(f"  ... {left:4.0f} s left (RETAIN0 = {d.tag(RT0)})")
            time.sleep(min(30.0, left))
        d.run_syscmd(ts.SYS_CLEAR_RULES)
        time.sleep(0.3)
        live = d.tag(RT0)
        print(f"  live RETAIN0 just before the REBOOT = {live}")
        d = base.reboot(d, holder, args)
        if d is None:
            return
        got = d.tag(RT0)
        print(f"  RETAIN0 after the REBOOT = {got}")
        check(got > 0, "a non-zero value was restored: retain_service() wrote a page on its own")
        check(got <= live, "restored value is one the live counter had already reached")
        check(base.clear_all(d), "cleanup: CLEAR_RULES and CLEAR_RETAIN DONE")
    else:
        print("\n5. skipped (add --periodic for the ~6 minute safety-net check)")

    print("\n6. REBOOT right after a rule-driven change (no COMMIT_RETAIN, no period elapsed)")
    d = base.reboot(d, holder, args)
    if d is None:
        return
    check(base.clear_all(d), "baseline: rules and retain cleared")
    tr.set_time(d, tr.local_epoch(2026, 1, 15, 12, 0, 0, tr.TZ_VN))
    check(tr.upload_rules(d, [tr.time_rule(0, 0, 2359, RT0)]), "rule that counts RETAIN0 up committed")
    time.sleep(3.0)
    d.run_syscmd(ts.SYS_CLEAR_RULES)
    time.sleep(0.3)
    live = d.tag(RT0)
    print(f"  live RETAIN0 just before the REBOOT = {live}")
    check(live > 0, "RETAIN0 was counting")
    d = base.reboot(d, holder, args)
    if d is None:
        return
    got = d.tag(RT0)
    print(f"  RETAIN0 after the REBOOT = {got}")
    check(got == live, "the value reached before the REBOOT is restored exactly (saved at REBOOT)")
    check(base.clear_all(d), "cleanup: CLEAR_RULES and CLEAR_RETAIN DONE")
    print("\nCleanup: board left with an empty rule table and zero retain.")


# ---------------------------------------------------------------- phase: legacy-write
def phase_legacy_write(d, holder, args):
    res = resource(d)
    retain, RT0 = res["retain"], res["base_retain"]
    if retain < 1:
        print("Board has no VREG_RETAIN group. Aborting.")
        return
    RTL = RT0 + retain - 1
    ts.ensure_running(d)
    print("Run this on the OLD firmware (before the page-log change). Nothing is cleared.")
    n = args.legacy_commits
    print(f"\nL1. {n} commits to move the newest old record into the second sector (39 records per sector)")
    bad_commits = [i for i in range(1, n + 1) if not base.commit_values(d, two_tag_items(RT0, RTL, i))]
    check(not bad_commits, f"all {n} commits clean" + (f"; failed at {bad_commits[:10]}" if bad_commits else ""))
    print(f"\nL2. Final commit with a known value for every one of the {retain} retain tags")
    values = {RT0 + i: base.pattern(i, k=7) for i in range(retain)}
    check(base.commit_values(d, values), "final commit clean")
    bad = base.tags_equal(d, values)
    check(not bad, "live values read back as written" + mism(bad))
    d = base.reboot(d, holder, args)
    if d is None:
        return
    bad = base.tags_equal(d, values)
    check(not bad, "OLD firmware restores them after a REBOOT (baseline of the old behaviour)" + mism(bad))
    with open(EXPECTED_FILE, "w", encoding="utf-8") as f:
        json.dump({"retain_base": RT0, "retain": retain,
                   "values": {str(k): v for k, v in values.items()}}, f, indent=1)
    print(f"\nExpected values saved to {EXPECTED_FILE}")
    print("NEXT (do not skip):")
    print("  1. Dump the retain region with STM32CubeProgrammer (SPLC_FLASH_RETAIN_BASE_ADDR, 24 KB;")
    print("     0x08036000 on a 256 KB chip) and keep the file.")
    print("  2. Flash the NEW firmware (keep the serial log open: look for PLC_RETAIN 'migrate').")
    print("  3. python test_retain_1sector.py <COM> --phase legacy-verify")


# ---------------------------------------------------------------- phase: legacy-verify
def phase_legacy_verify(d, holder, args):
    if not os.path.exists(EXPECTED_FILE):
        print(f"{EXPECTED_FILE} not found: run --phase legacy-write on the OLD firmware first.")
        ts._failures += 1
        return
    with open(EXPECTED_FILE, encoding="utf-8") as f:
        exp = json.load(f)
    res = resource(d)
    values = {int(k): v for k, v in exp["values"].items()}
    check(res["base_retain"] == exp["retain_base"] and res["retain"] == exp["retain"],
          f"retain tag layout unchanged (base {res['base_retain']}, count {res['retain']})")
    ts.ensure_running(d)

    print("\nV1. First boot of the NEW firmware: the old record must have been migrated")
    bad = base.tags_equal(d, values)
    check(not bad, f"all {len(values)} values equal the old firmware's" + mism(bad))
    check(not flash_dirty(d), "FLASH_DIRTY clear (migration finished and verified; if set, the migration failed or is pending)")

    print("\nV2. REBOOT: now the values come from a new-format page")
    d = base.reboot(d, holder, args)
    if d is None:
        return
    bad = base.tags_equal(d, values)
    check(not bad, "values still equal after REBOOT" + mism(bad))
    check(not flash_dirty(d), "FLASH_DIRTY clear")

    print("\nV3. New firmware writes: commit new values, REBOOT")
    RT0 = res["base_retain"]
    nv = {RT0 + i: base.pattern(i, k=9) for i in range(res["retain"])}
    check(base.commit_values(d, nv), "commit of new values clean")
    d = base.reboot(d, holder, args)
    if d is None:
        return
    bad = base.tags_equal(d, nv)
    check(not bad, "new values restored after REBOOT" + mism(bad))
    print("\nLeft on the board: the new values (k=9). Nothing was cleared. Compare with the Flash dump.")


# ---------------------------------------------------------------- main
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("port")
    p.add_argument("--phase", choices=("new", "legacy-write", "legacy-verify"), default="new")
    p.add_argument("--unit", type=int, default=1)
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--commits", type=int, default=60,
                   help="consecutive COMMIT_RETAINs in step 2 of phase new (default 60 > 54 main pages; 0 skips)")
    p.add_argument("--legacy-commits", type=int, default=45,
                   help="commits of phase legacy-write (default 45: newest old record in the second sector)")
    p.add_argument("--periodic", action="store_true", help="also run the ~6 minute safety-net step (phase new)")
    p.add_argument("--period-s", type=int, default=300, help="RETAIN_SNAPSHOT_PERIOD_MS of the firmware in seconds")
    p.add_argument("--port-after", default=None, help="COM port to use after a reboot if Windows renumbers it")
    R.add_args(p)
    args = p.parse_args()
    R.start(args)

    client = ModbusSerialClient(port=args.port, baudrate=args.baudrate, parity="N",
                                stopbits=1, bytesize=8, timeout=1.0)
    if not client.connect():
        print(f"Cannot open {args.port}")
        sys.exit(2)
    holder = [client]
    try:
        d = ts.Dev(client, args.unit)
        {"new": phase_new, "legacy-write": phase_legacy_write,
         "legacy-verify": phase_legacy_verify}[args.phase](d, holder, args)
    finally:
        try:
            holder[0].close()
        except Exception:
            pass
    fails = ts._failures
    print("\nALL PASS" if fails == 0 else f"\n{fails} CHECK(S) FAILED")
    sys.exit(0 if fails == 0 else 1)


if __name__ == "__main__":
    main()