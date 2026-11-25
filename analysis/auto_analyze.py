#!/usr/bin/env python3
"""
Run analysis/analyze_flight.py automatically after landing.

Started in the background by scripts/run_sitl.sh. PX4 writes the flight log from
boot until DISARM (SDLOG_MODE = 1), so a log of THIS session whose size has not
changed for --stable seconds is a finished flight. The watcher then analyses it
and writes results/<boot time>/ (tong_ket.md + 4 figures) and prints the table.
It exits when PX4 (--pid) exits. Landing a manual flight (no AUTO_MISSION in the
log) is skipped with a one-line message.

Plan used for the analysis: --plan, else the NEWEST simulation/missions/*.plan (the file
name and altitude are printed in the summary; wrong plan => warning there).
If you quit PX4 before it disarms, the log is cut: run analyze_flight.py by hand.

  python3 tools/auto_analyze.py --pid 12345        # wait, then analyse (run_sitl.sh)
  python3 tools/auto_analyze.py --now              # analyse the newest log right now
  python3 tools/auto_analyze.py --now LOG.ulg      # ... or a given log
"""
import argparse
import datetime as dt
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
LOG_DIR = Path.home() / "PX4-Autopilot/build/px4_sitl_default/rootfs/log"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pid", type=int, help="exit when this process (PX4 / make) is gone")
    p.add_argument("--now", nargs="?", const="newest", metavar="LOG", help="analyse now (newest log or LOG) and exit")
    p.add_argument("--plan", type=Path, help="mission plan (default: newest simulation/missions/*.plan)")
    p.add_argument("--log-dir", type=Path, default=LOG_DIR)
    p.add_argument("--results-dir", type=Path, help="default: results/ in the repo")
    p.add_argument("--stable", type=float, default=15.0, help="log size unchanged this long = flight finished [s]")
    p.add_argument("--poll", type=float, default=3.0, help="check interval [s]")
    return p.parse_args()


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def local_label(log):
    """Log folder/name are UTC (2026-11-10/04_46_58.ulg) -> local boot time label."""
    try:
        t = dt.datetime.strptime(f"{log.parent.name} {log.stem}", "%Y-%m-%d %H_%M_%S")
        return t.replace(tzinfo=dt.timezone.utc).astimezone().strftime("%Y-%m-%d_%H-%M-%S")
    except ValueError:
        return log.stem


def analyse(log, a):
    plans = list((REPO / "simulation" / "missions").glob("*.plan"))
    plan = a.plan or max(plans, key=lambda f: f.stat().st_mtime)
    cmd = [sys.executable, str(HERE / "analyze_flight.py"), str(log), "--label", local_label(log), "--plan", str(plan)]
    if a.results_dir:
        cmd += ["--results-dir", str(a.results_dir)]
    print(f"\n[auto_analyze] Chuyến bay đã kết thúc, đang phân tích {log.parent.name}/{log.name} "
          f"với plan {plan.name} ...", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        reason = (r.stderr.strip().splitlines() or ["lỗi không rõ"])[-1]
        print(f"[auto_analyze] Bỏ qua {log.name}: {reason}", flush=True)
        return
    print("\n" + "=" * 72 + "\n" + r.stdout.rstrip() + "\n" + "=" * 72, flush=True)
    if shutil.which("notify-send"):
        subprocess.run(["notify-send", "X450: đã phân tích xong chuyến bay",
                        f"Xem results/{local_label(log)}/tong_ket.md"], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    a = parse_args()
    if a.now:
        log = Path(a.now) if a.now != "newest" else max(a.log_dir.glob("*/*.ulg"), key=lambda f: f.stat().st_mtime)
        analyse(log, a)
        return
    start = time.time()
    seen = {}      # log -> (size, time the size last changed)
    done = set()
    print("[auto_analyze] Đang chờ chuyến bay. Sau khi drone hạ cánh và tắt motor, mình tự phân tích "
          "log và lưu vào results/<giờ bay>/.", flush=True)
    while a.pid is None or pid_alive(a.pid):
        for f in sorted(a.log_dir.glob("*/*.ulg")):
            if f in done or f.stat().st_mtime < start - 2:        # not a log of this session
                continue
            size, now = f.stat().st_size, time.time()
            if f not in seen or seen[f][0] != size:
                seen[f] = (size, now)
            elif now - seen[f][1] >= a.stable:
                done.add(f)
                analyse(f, a)
        time.sleep(a.poll)


if __name__ == "__main__":
    main()
