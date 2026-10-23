#!/usr/bin/env python3
"""Render results/benchmarks.jsonl (written by benchmark.py) into a Markdown table.

    python3 examples/make_report.py                # prints to stdout
    python3 examples/make_report.py -o BENCHMARKS.md --header docs/benchmarks_header.md
"""

from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, ".."))

DATASET_NAMES = {
    "tumvi": "TUM-VI",
    "euroc": "EuRoC MAV",
    "kitti": "KITTI raw",
    "tum_rgbd": "TUM RGB-D",
}
MODE_ORDER = ["mono", "mono-imu", "stereo", "stereo-imu", "rgbd"]


def fmt(v, unit="", nd=2):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.{nd}f}{unit}" if abs(v) < 100 else f"{v:.1f}{unit}"
    return f"{v}{unit}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jsonl", default=os.path.join(PKG_ROOT, "results", "benchmarks.jsonl"))
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--header", default=None, help="Markdown file prepended to the tables")
    ap.add_argument("--footer", default=None, help="Markdown file appended after the tables")
    args = ap.parse_args()

    rows = []
    with open(args.jsonl) as f:
        for line in f:
            if line.strip():
                rows.append(json.loads(line))
    if not rows:
        sys.exit("no records")

    # number repeated runs of the same (dataset, sequence, mode) so all of them show
    counts = {}
    for r in rows:
        key = (r["dataset"], r["sequence"], r["mode"])
        counts[key] = counts.get(key, 0) + 1
        r["_run"] = counts[key]
    for r in rows:
        if counts[(r["dataset"], r["sequence"], r["mode"])] > 1:
            r["sequence"] = f"{r['sequence']} (run {r['_run']})"

    out = []
    if args.header:
        out.append(open(args.header).read().rstrip() + "\n")

    datasets = sorted({r["dataset"] for r in rows}, key=lambda d: list(DATASET_NAMES).index(d))
    for ds in datasets:
        out.append(f"\n### {DATASET_NAMES.get(ds, ds)}\n")
        out.append("| Sequence | Mode | Frames | Tracked OK | IMU init | ATE sim(3) | scale | ATE SE(3) | fps | track ms |")
        out.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")
        sel = [r for r in rows if r["dataset"] == ds]
        sel.sort(key=lambda r: (r["sequence"].split(" (run")[0], MODE_ORDER.index(r["mode"]), r["_run"]))
        for r in sel:
            imu = fmt(r.get("imu_init_s"), " s", 1) if r["mode"].endswith("imu") else "n/a"
            # plain monocular has no metric scale, so a rigid-alignment error is meaningless
            se3 = "n/a" if r["mode"] == "mono" else fmt(r.get("ate_se3_cm"), " cm")
            out.append(
                f"| {r['sequence']} | {r['mode']} | {r['frames']} | "
                f"{r['ok']} ({r['ok_pct']}%) | {imu} | "
                f"{fmt(r.get('ate_sim3_cm'), ' cm')} | {fmt(r.get('scale'), '', 3)} | "
                f"{se3} | {fmt(r.get('fps'), '', 1)} | {fmt(r.get('track_ms_median'), '', 1)} |"
            )
    if args.footer:
        out.append("\n" + open(args.footer).read().rstrip() + "\n")
    text = "\n".join(out) + "\n"
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
        print(f"wrote {args.out} ({len(rows)} rows)")
    else:
        print(text)


if __name__ == "__main__":
    main()
