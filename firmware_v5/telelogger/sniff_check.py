#!/usr/bin/env python3
"""Checks a sniff log (/SNIFF/S<n>.LOG, sniff.cpp) for completeness.

  python3 sniff_check.py S20.LOG

- counts frame lines and compares them with the last "# stats" line
  (frames / written / dropped / sd_err / sd_lost);
- for a bench load test (build flag -DTEST_SNIFF_LOAD=<frames/s>, ID 7AA,
  data = 64-bit little-endian sequence number): every gap in the sequence
  is listed - a complete file has none.
"""
import re
import sys


def main(path):
    frames = 0
    seq_prev = None
    gaps = []
    seq_frames = 0
    last_stats = None
    first_ms = last_ms = None
    with open(path, encoding="ascii", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("#"):
                if line.startswith("# stats"):
                    last_stats = dict(re.findall(r"(\w+)=(\d+)", line))
                continue
            parts = line.split(",")
            if len(parts) < 5:
                continue
            frames += 1
            ms = float(parts[0])
            first_ms = ms if first_ms is None else first_ms
            last_ms = ms
            if parts[1] == "7AA" and parts[3] == "8":
                seq = int.from_bytes(bytes.fromhex(parts[4].replace(" ", "")), "little")
                seq_frames += 1
                if seq_prev is not None and seq != seq_prev + 1:
                    gaps.append((seq_prev, seq, ms))
                seq_prev = seq
    print(f"frame lines: {frames}, time {first_ms} .. {last_ms} ms "
          f"({(last_ms - first_ms) / 1000 if frames else 0:.0f} s)")
    if last_stats:
        print("last stats:", {k: last_stats[k] for k in last_stats if k in
              ("frames", "written", "dropped", "sd_err", "sd_lost", "rx_missed")})
        w = int(last_stats.get("written", -1))
        if w >= 0:
            print("file frames == stats written:", frames == w, f"({frames} vs {w})")
    if seq_frames:
        print(f"load-test frames: {seq_frames}, gaps: {len(gaps)}")
        for a, b, ms in gaps[:20]:
            print(f"  gap after seq {a} -> {b} ({b - a - 1} missing) at {ms} ms")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
