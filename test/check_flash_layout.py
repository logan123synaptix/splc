#!/usr/bin/env python3
"""
check_flash_layout.py - STATIC check of the Flash layout (no board, no compiler needed).

Reads splc_flash_define.h, evaluates its #define's in pure Python and checks that
  * Rule Table A/B, the TLS cert store, the Wi-Fi store and DATA_BASE did NOT move
    (compared with fixed expected values AND, if git is available, with the header at
    a baseline commit, default ae3b993);
  * the retain region is still 3 sectors, only the first one is written (1 sector,
    144-byte pages, 56 pages, 54 main + 2 PVD reserve);
  * the old 208-byte format constants are unchanged (the boot migration needs them);
  * the sector map has no overlap and ends exactly at the end of Flash;
  * every record/page fits inside its sector.

Phase 2 (section [5]): the Rule Table record EXT section (PULSE 256 B, MQTT publish blocks
4..7 768 B, MQTT subscribe blocks 2..7 768 B, 4-byte EXT header, announced by (bit 15, bit 14) =
(0, 1) of the rule_count field): sizes, flag value/mask, worst-case record size for several
feature sets, and that a build without any phase-2 feature keeps the OLD worst-case size.

NOT a build: it does not run the C compiler's _Static_assert's. Prints PASS/FAIL per
check and a total. Exit code 0 = all PASS.

Run (from the simple_plc repo root; one command per block):

    python3 test/check_flash_layout.py

Other repo layout / another header (e.g. the meshwf copy):

    python3 test/check_flash_layout.py --header <path to splc_flash_define.h> --opts <path to splcopts.h>
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

FLASH_BASE = 0x08000000
SECTOR = 0x2000
REL_HEADER = "simple_plc/platforms/stm32/stm32h5/flash_define/splc_flash_define.h"

# Expected absolute addresses for SPLC_FLASH_SIZE_KB = 256 (END = 0x08040000, 5 data sectors).
# Derived from the formulas of the BASELINE header (DATA_BASE = END - 5 sectors,
# Rule B = +3, Rule A = +4, cert store = DATA_BASE - 1 sector, Wi-Fi store = DATA_BASE - 2 sectors).
EXPECT_256 = {
    "DATA_BASE":   0x08036000,
    "RETAIN_BASE": 0x08036000,
    "RULE_B":      0x0803C000,
    "RULE_A":      0x0803E000,
    "CERT_STORE":  0x08034000,
    "WIFI_STORE":  0x08032000,
}
# Old retain format (must stay: the migration reads it).
EXPECT_OLD = {"SPLC_RETAIN_RECORD_SIZE": 208, "SPLC_RETAIN_RECORDS_PER_SECTOR": 39,
              "SPLC_RETAIN_TOTAL_RECORDS": 117, "SPLC_FLASH_RETAIN_SECTOR_COUNT": 3}
# Where meshwf computes the two stores (source text, checked only with --meshwf-root).
CERT_EXPR = "SPLC_FLASH_DATA_BASE_ADDR - SPLC_FLASH_SECTOR_SIZE"
WIFI_EXPR = "SPLC_FLASH_DATA_BASE_ADDR - 2U * SPLC_FLASH_SECTOR_SIZE"


# ----------------------------------------------------------------------------- mini C preprocessor
def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def parse_defines(text, predefined):
    """Object-like #define's, honouring #if/#ifdef/#ifndef/#else/#endif (minimal)."""
    text = strip_comments(text).replace("\\\n", " ")
    defs = dict(predefined)
    stack = []  # list of (active, taken)

    def active():
        return all(a for a, _ in stack)

    for line in text.split("\n"):
        m = re.match(r"\s*#\s*(\w+)\s*(.*)$", line)
        if not m:
            continue
        d, rest = m.group(1), m.group(2).strip()
        if d in ("ifdef", "ifndef", "if"):
            if d == "ifdef":
                cond = rest.split()[0] in defs
            elif d == "ifndef":
                cond = rest.split()[0] not in defs
            else:
                tok = re.match(r"\w+", rest)
                if rest.startswith("!") or "defined" in rest or tok is None:
                    cond = True
                else:
                    cond = (tok.group(0) not in ("0",)) and defs.get(tok.group(0), "1") not in ("0",)
            stack.append((cond, cond))
        elif d == "else" and stack:
            _, taken = stack.pop()
            stack.append((not taken, True))
        elif d == "endif" and stack:
            stack.pop()
        elif d == "define" and active():
            mm = re.match(r"(\w+)(\(?)", rest)
            if not mm or mm.group(2) == "(":
                continue  # function-like macro
            name = mm.group(1)
            defs[name] = rest[len(name):].strip()
    return defs


def evaluate(expr, defs, depth=0):
    if depth > 40:
        raise ValueError("macro recursion")
    expr = re.sub(r"\(\s*(?:u?int\d+_t|unsigned|int)\s*\)", " ", expr)

    def sub_ident(m):
        n = m.group(0)
        if n in defs:
            return "(" + str(evaluate(defs[n], defs, depth + 1)) + ")"
        raise ValueError("undefined macro " + n)

    expr = re.sub(r"\b0[xX][0-9a-fA-F]+[uUlL]*\b|\b\d+[uUlL]*\b", lambda m: re.sub(r"[uUlL]+$", "", m.group(0)), expr)
    expr = re.sub(r"\b[A-Za-z_]\w*\b", lambda m: m.group(0) if re.fullmatch(r"0[xX][0-9a-fA-F]+", m.group(0)) else sub_ident(m), expr)
    expr = expr.replace("/", "//")
    if re.search(r"[^0-9a-fA-FxX+\-*/%&|^~()<>!=?: \t]", expr):
        raise ValueError("cannot evaluate: " + expr)
    expr = _ternary(expr)
    return int(eval(expr, {"__builtins__": {}}, {}))


def _ternary(expr):
    """C `cond ? a : b` -> Python `(a) if (cond) else (b)` (only what the header uses; parenthesised)."""
    if "?" not in expr:
        return expr
    e = expr.strip()
    while e.startswith("(") and e.endswith(")"):  # strip ONE balanced outer pair
        depth = 0
        for i, c in enumerate(e):
            depth += (c == "(") - (c == ")")
            if depth == 0 and i < len(e) - 1:
                break
        else:
            e = e[1:-1].strip()
            continue
        break
    depth = 0
    q = c_ = -1
    for i, c in enumerate(e):
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
        elif depth == 0 and c == "?" and q < 0:
            q = i
        elif depth == 0 and c == ":" and q >= 0:
            c_ = i
            break
    if q < 0 or c_ < 0:
        raise ValueError("cannot evaluate ternary: " + expr)
    return "((%s) if (%s) else (%s))" % (_ternary(e[q + 1:c_]), _ternary(e[:q]), _ternary(e[c_ + 1:]))


# ----------------------------------------------------------------------------- reporting
class R:
    def __init__(self):
        self.ok = 0
        self.bad = 0

    def check(self, cond, name, detail=""):
        if cond:
            self.ok += 1
            print("  PASS  " + name + ((" -- " + detail) if detail else ""))
        else:
            self.bad += 1
            print("  FAIL  " + name + ((" -- " + detail) if detail else ""))


def load(header_path, kb, opts_path):
    pre = {"FLASH_BASE": str(FLASH_BASE), "STM32H5_PLATFORM": "1", "SPLC_FLASH_SIZE_KB": str(kb)}
    if opts_path:
        t = strip_comments(Path(opts_path).read_text(encoding="utf-8", errors="replace"))
        for name in ("SPLC_FLASH_DATA_ADDR", "SPLC_FLASH_SIZE_KB"):
            mm = re.search(r"^\s*#\s*define\s+" + name + r"\s+(.+)$", t, flags=re.M)
            if mm:
                pre[name] = mm.group(1).strip()
    text = Path(header_path).read_text(encoding="utf-8", errors="replace")
    return parse_defines(text, pre), pre


def addrs(defs):
    g = lambda n: evaluate(n, defs)
    data = g("SPLC_FLASH_DATA_BASE_ADDR")
    return {
        "DATA_BASE": data,
        "RETAIN_BASE": g("SPLC_FLASH_RETAIN_BASE_ADDR"),
        "RULE_B": g("SPLC_FLASH_RULE_TABLE_B_ADDR"),
        "RULE_A": g("SPLC_FLASH_RULE_TABLE_A_ADDR"),
        "CERT_STORE": evaluate(CERT_EXPR, defs),
        "WIFI_STORE": evaluate(WIFI_EXPR, defs),
    }


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="Static Flash layout check")
    ap.add_argument("--header", default=str(here.parent / REL_HEADER))
    ap.add_argument("--opts", default=None, help="splcopts.h of the product (SPLC_FLASH_SIZE_KB, SPLC_FLASH_DATA_ADDR)")
    ap.add_argument("--flash-kb", type=int, default=256)
    ap.add_argument("--baseline-ref", default="ae3b993", help="git commit with the OLD header, '' to skip")
    ap.add_argument("--meshwf-root", default=None, help="meshwf repo root: also check the cert/Wi-Fi store source lines")
    a = ap.parse_args()

    r = R()
    print("Header: " + a.header)
    defs, pre = load(a.header, a.flash_kb, a.opts)
    kb = int(evaluate(pre["SPLC_FLASH_SIZE_KB"], {}))
    print("SPLC_FLASH_SIZE_KB = %d, SPLC_FLASH_DATA_ADDR %s" % (kb, pre.get("SPLC_FLASH_DATA_ADDR", "not set (default: last 5 sectors)")))
    g = lambda n: evaluate(n, defs)

    print("\n[1] Addresses that must NOT move")
    try:
        cur = addrs(defs)
    except ValueError as e:
        r.check(False, "evaluate header", str(e))
        print("\nTOTAL: PASS=%d FAIL=%d" % (r.ok, r.bad))
        return 1
    for k, v in cur.items():
        print("      %-12s 0x%08X" % (k, v))
    if kb == 256 and "SPLC_FLASH_DATA_ADDR" not in pre:
        for k, v in EXPECT_256.items():
            r.check(cur[k] == v, "%s == 0x%08X (fixed expected value)" % (k, v), "now 0x%08X" % cur[k])
    else:
        print("  (fixed expected values are for 256 KB without SPLC_FLASH_DATA_ADDR: skipped)")
    if a.baseline_ref:
        try:
            out = subprocess.run(["git", "-C", str(Path(a.header).resolve().parent), "show",
                                  "%s:%s" % (a.baseline_ref, REL_HEADER)],
                                 capture_output=True, text=True, timeout=30)
            if out.returncode != 0:
                raise RuntimeError(out.stderr.strip()[:120])
            pre_old = dict(pre)
            old_defs = parse_defines(out.stdout, pre_old)
            old = addrs(old_defs)
            for k in cur:
                r.check(cur[k] == old[k], "%s same as at %s" % (k, a.baseline_ref), "now 0x%08X, before 0x%08X" % (cur[k], old[k]))
            # old retain constants straight from the old header
            r.check(evaluate("SPLC_FLASH_RETAIN_SECTOR_COUNT", old_defs) == g("SPLC_FLASH_RETAIN_SECTOR_COUNT"),
                    "retain region sector count same as at %s" % a.baseline_ref)
        except Exception as e:  # git missing, commit not in this clone, header at another path...
            print("  (git baseline %s not usable: %s -- only the fixed expected values were checked)" % (a.baseline_ref, e))

    if a.meshwf_root:
        root = Path(a.meshwf_root)
        for rel, expr, what in (("SynaptixSDK/components/st67/st67_cert_store.c", CERT_EXPR, "cert store"),
                                ("SynaptixSDK/ports/splc_port/splc_wifi_store.c", WIFI_EXPR, "Wi-Fi store")):
            p = root / rel
            if not p.exists():
                r.check(False, "%s source found" % what, str(p))
                continue
            t = strip_comments(p.read_text(encoding="utf-8", errors="replace"))
            r.check(re.sub(r"\s+", "", expr) in re.sub(r"\s+", "", t), "%s still computed as DATA_BASE relative (%s)" % (what, expr))

    print("\n[2] Retain: 3 reserved sectors, 1 written")
    for k, v in EXPECT_OLD.items():
        r.check(g(k) == v, "old-format constant %s == %d" % (k, v), "now %d" % g(k))
    try:
        act = g("SPLC_RETAIN_ACTIVE_SECTORS")
        page = g("SPLC_RETAIN_PAGE_SIZE")
        pps = g("SPLC_RETAIN_PAGES_PER_SECTOR")
        main_pages = g("SPLC_RETAIN_MAIN_PAGES")
        reserve = g("SPLC_RETAIN_PVD_RESERVE_PAGES")
        tags = g("SPLC_RETAIN_TAG_COUNT")
        hdr = g("SPLC_RETAIN_PAGE_HEADER_SIZE")
        val = g("SPLC_RETAIN_PAGE_VALUE_SIZE")
        fmt = g("SPLC_RETAIN_PAGE_FMT")
    except ValueError as e:
        r.check(False, "retain page constants present in the header", str(e))
        print("\nTOTAL: PASS=%d FAIL=%d" % (r.ok, r.bad))
        return 1
    r.check(act == 1, "SPLC_RETAIN_ACTIVE_SECTORS == 1", "now %d" % act)
    r.check(act <= g("SPLC_FLASH_RETAIN_SECTOR_COUNT"), "active sectors inside the 3-sector region")
    r.check(page == 144, "page size == 144", "now %d" % page)
    r.check(page % 16 == 0, "page size is a multiple of 16 (quad-word)")
    r.check(page >= hdr + val * tags, "page holds header %d + %d x %d bytes" % (hdr, tags, val))
    r.check(pps * page <= SECTOR, "%d pages x %d B = %d B fit in one %d B sector" % (pps, page, pps * page, SECTOR),
            "%d B left over" % (SECTOR - pps * page))
    r.check(pps == 56, "pages per sector == 56", "now %d" % pps)
    r.check(main_pages + reserve == pps, "main %d + PVD reserve %d == %d pages" % (main_pages, reserve, pps))
    r.check(main_pages == 54 and reserve == 2, "54 main pages, 2 PVD reserve pages")
    r.check(tags <= 255, "tag count fits the one count byte of fmt_count")
    r.check(fmt != 0xFF and fmt != 0x00, "page format byte 0x%02X is neither blank (0xFF) nor 0x00" % fmt)
    # the old record's byte 4 is the high byte of its count (0x00): the new fmt byte must differ
    r.check(fmt != 0x00, "format byte differs from the high byte of the old count field (0x00)")
    r.check(g("SPLC_RETAIN_PAGE_SIZE") != g("SPLC_RETAIN_RECORD_SIZE"), "new and old record sizes differ")

    print("\n[3] Sector map (lowest address first)")
    end = g("SPLC_FLASH_END_ADDR")
    d = cur["DATA_BASE"]
    regions = [
        ("Wi-Fi store",          cur["WIFI_STORE"], SECTOR),
        ("TLS cert store",       cur["CERT_STORE"], SECTOR),
        ("Retain +0 (written)",  d,                 SECTOR),
        ("Retain +1 (reserved)", d + SECTOR,        SECTOR),
        ("Retain +2 (reserved)", d + 2 * SECTOR,    SECTOR),
        ("Rule Table B",         cur["RULE_B"],     g("SPLC_FLASH_RULE_TABLE_SIZE")),
        ("Rule Table A",         cur["RULE_A"],     g("SPLC_FLASH_RULE_TABLE_SIZE")),
    ]
    regions.sort(key=lambda x: x[1])
    for n, s, sz in regions:
        print("      0x%08X..0x%08X  %s" % (s, s + sz - 1, n))
    ok_map = True
    for (n1, s1, z1), (n2, s2, z2) in zip(regions, regions[1:]):
        if s1 + z1 != s2:
            ok_map = False
            r.check(False, "%s ends where %s begins" % (n1, n2), "0x%08X vs 0x%08X" % (s1 + z1, s2))
    if ok_map:
        r.check(True, "regions are contiguous: no overlap, no gap")
    last_n, last_s, last_z = regions[-1]
    r.check(last_s + last_z == end, "last region ends exactly at the end of Flash (0x%08X)" % end)
    r.check(all(s % SECTOR == 0 for _, s, _ in regions), "every region is sector (8 KB) aligned")
    r.check(g("SPLC_FLASH_RETAIN_BASE_ADDR") + g("SPLC_FLASH_RETAIN_TOTAL_SIZE") == cur["RULE_B"],
            "retain region (3 sectors) ends where Rule Table B begins")
    written_end = cur["RETAIN_BASE"] + pps * page
    r.check(cur["RETAIN_BASE"] <= written_end <= cur["RETAIN_BASE"] + SECTOR, "retain pages stay inside sector +0",
            "last used byte 0x%08X" % (written_end - 1))
    r.check(written_end <= d + SECTOR, "nothing of the retain log reaches sector +1 (reserved)")

    print("\n[4] Information only (not a check)")
    print("      Endurance, worst case (a change every RETAIN_SNAPSHOT_PERIOD_MS = 5 min), ASSUMING ~10,000 erases per sector")
    print("      (the figure in the header comment, NOT checked against the datasheet):")
    per_day = 288.0 / main_pages
    print("      %d saves/day / %d pages = %.1f erases/day -> about %.1f years" % (288, main_pages, per_day, 10000 / per_day / 365))

    print("\n[5] Phase 2: Rule Table record EXT section")
    text_hdr = Path(a.header).read_text(encoding="utf-8", errors="replace")
    FEATS = ("SPLC_MQTT_ENABLE", "SPLC_MBM_ENABLE", "SPLC_REAL_ENABLE", "SPLC_MQTT_SUB_ENABLE",
             "SPLC_MBW_ENABLE", "SPLC_ZIGBEE_ENABLE", "SPLC_PULSE_ENABLE")
    # sizes of the sections that exist before phase 2 (bytes), per feature
    OLD_SECT = {"SPLC_MQTT_ENABLE": 960 + 64, "SPLC_MBM_ENABLE": 640, "SPLC_REAL_ENABLE": 128,
                "SPLC_MQTT_SUB_ENABLE": 256, "SPLC_MBW_ENABLE": 512, "SPLC_ZIGBEE_ENABLE": 64}
    EXT_SEG = {"SPLC_MQTT_ENABLE": 768, "SPLC_MQTT_SUB_ENABLE": 768, "SPLC_PULSE_ENABLE": 256}
    try:
        g5 = lambda n, d: evaluate(n, d)
        d_all = parse_defines(text_hdr, dict(pre, MAX_RULES="100", **{f: "1" for f in FEATS}))
        r.check(g5("SPLC_RULE_FLASH_EXT_HDR_SIZE", d_all) == 4, "EXT header is 4 bytes")
        r.check(g5("SPLC_RULE_FLASH_EXT_PULSE_SIZE", d_all) == 256, "PULSE segment 256 B (8 x 32 B)")
        r.check(g5("SPLC_RULE_FLASH_EXT_PUB_SIZE", d_all) == 768, "MQTT publish blocks 4..7 segment 768 B (4 x 192 B)")
        r.check(g5("SPLC_RULE_FLASH_EXT_SUB_SIZE", d_all) == 768, "MQTT subscribe blocks 2..7 segment 768 B (6 x 128 B)")
        r.check((g5("SPLC_RULE_FLASH_EXT_BIT_PULSE", d_all), g5("SPLC_RULE_FLASH_EXT_BIT_PUB", d_all),
                 g5("SPLC_RULE_FLASH_EXT_BIT_SUB", d_all)) == (1, 2, 4), "EXT header bits: PULSE 0x0001, PUB 0x0002, SUB 0x0004")
        fbf, fb2 = g5("SPLC_RULE_FLASH_FB_FLAG", d_all), g5("SPLC_RULE_FLASH_FB_V2_FLAG", d_all)
        r.check(g5("SPLC_RULE_FLASH_EXT_FLAG_VALUE", d_all) == 0x4000 and g5("SPLC_RULE_FLASH_EXT_FLAG_MASK", d_all) == 0xC000,
                "EXT announced by (bit15, bit14) = (0, 1): value 0x4000 under mask 0xC000")
        r.check(g5("SPLC_RULE_FLASH_EXT_FLAG_VALUE", d_all) == fb2 and g5("SPLC_RULE_FLASH_EXT_FLAG_MASK", d_all) == (fbf | fb2),
                "that combination is exactly (FB flag = 0, FB V2 flag = 1): never written before phase 2")
        r.check(g5("SPLC_RULE_FLASH_EXT_KNOWN_BITS", d_all) == 7, "known EXT bits == 0x0007")
        # worst-case record size for several feature sets
        base_fixed = 8 + 100 * 32 + 128   # header + MAX_RULES x 32 + FB V2
        scenarios = [
            ("everything on", {f: 1 for f in FEATS}),
            ("everything off", {f: 0 for f in FEATS}),
            ("phase-2 features off, old sections on", dict({f: 1 for f in FEATS}, SPLC_PULSE_ENABLE=0, SPLC_MQTT_ENABLE=0, SPLC_MQTT_SUB_ENABLE=0)),
            ("MQTT + SUB, no PULSE", dict({f: 0 for f in FEATS}, SPLC_MQTT_ENABLE=1, SPLC_MQTT_SUB_ENABLE=1)),
            ("PULSE only", dict({f: 0 for f in FEATS}, SPLC_PULSE_ENABLE=1)),
        ]
        for name, fs in scenarios:
            dd = parse_defines(text_hdr, dict(pre, MAX_RULES="100", **{k: str(v) for k, v in fs.items()}))
            got = g5("SPLC_RULE_FLASH_RECORD_MAX_SIZE", dd)
            old = base_fixed + sum(OLD_SECT[k] for k in OLD_SECT if fs[k])
            segs = sum(EXT_SEG[k] for k in EXT_SEG if fs[k])
            exp = old + (4 + segs if segs else 0)
            r.check(got == exp, "worst-case record, %s: %d B" % (name, got), "expected %d" % exp)
            r.check(got <= SECTOR, "  ... fits one %d B sector" % SECTOR, "%d B left" % (SECTOR - got))
        d_all_size = g5("SPLC_RULE_FLASH_RECORD_MAX_SIZE", d_all)
        r.check(d_all_size == 7756, "worst case with every feature and PULSE == 7756 B (5960 + 4 + 768 + 768 + 256)", "now %d" % d_all_size)
        d_none = parse_defines(text_hdr, dict(pre, MAX_RULES="100", **{f: "0" for f in FEATS}))
        r.check(g5("SPLC_RULE_FLASH_EXT_RESERVE", d_none) == 0, "no phase-2 feature compiled in: EXT reserve == 0 (baseline record size unchanged)")
    except ValueError as e:
        r.check(False, "phase-2 EXT constants present in the header", str(e))

    print("\nTOTAL: PASS=%d FAIL=%d" % (r.ok, r.bad))
    return 0 if r.bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())