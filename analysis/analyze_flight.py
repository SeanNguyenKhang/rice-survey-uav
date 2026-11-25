#!/usr/bin/env python3
"""
Analyse one PX4 SITL flight log (.ulg) of the lawn-mower survey and check the
thesis requirements. Writes figures + a summary to results/<label>/.

Metrics (all during the AUTO_MISSION part of the flight):
  1. Path cross-track RMSE  - true position (Gazebo ground truth) vs the planned
     survey lines; only on the 50 m lines, END_TRIM metres cut at both ends
     (turns), per line and overall.                  -> "bay đúng đường"
  2. Altitude RMSE on the survey lines vs the planned altitude.
  3. Position tracking RMSE - PX4 estimate vs the position setpoint of the
     controller (3D and horizontal).                  -> "PID vị trí"
  4. Attitude error - roll/pitch estimate vs attitude setpoint: RMS, max,
     % of time above 5 deg.                           -> "PID góc"
  5. Take-off altitude response 0 -> ALT: overshoot, 10-90 % rise time,
     2 % settling time. (PX4 shapes the take-off with a smooth trajectory, so
     this is NOT a pure step; a dedicated step test is needed for the
     attitude-loop overshoot / settling requirements.)

Usage:
  python3 analysis/analyze_flight.py                       # newest log, default plan
  python3 analysis/analyze_flight.py LOG.ulg --label wind5 --plan simulation/missions/xxx.plan
"""
import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools" / "mission"))   # field_site.py
from field_site import latlon_to_enu  # noqa: E402

LOG_DIR = Path.home() / "PX4-Autopilot/build/px4_sitl_default/rootfs/log"
REQ = {"pos_rmse": 0.5, "att_err": 5.0, "overshoot": 15.0, "settling": 2.0}   # thesis requirements
NAV_MISSION = 3
END_TRIM = 5.0       # metres ignored at both ends of every survey line (turns)
LINE_MIN = 30.0      # legs longer than this are survey lines, shorter are turns


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", nargs="?", type=Path, help="flight log .ulg (default: newest in the PX4 log dir)")
    p.add_argument("--plan", type=Path, default=REPO / "simulation" / "missions" / "lawnmower_a6_v1.5_s11.2.plan")
    p.add_argument("--label", help="results sub-folder name (default: log date_time)")
    p.add_argument("--results-dir", type=Path, default=REPO / "results", help="where results/<label>/ goes")
    return p.parse_args()


def data(ulog, name):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError):
        return None


def euler(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    return roll, pitch


def rms(x):
    x = np.asarray(x)
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


def main():
    a = parse_args()
    log = a.log or max(LOG_DIR.glob("*/*.ulg"), key=lambda f: f.stat().st_mtime)
    label = a.label or f"{log.parent.name}_{log.stem}"
    out = a.results_dir / label
    out.mkdir(parents=True, exist_ok=True)
    ul = ULog(str(log), ["vehicle_status", "vehicle_global_position_groundtruth", "vehicle_local_position",
                         "vehicle_local_position_setpoint", "vehicle_attitude", "vehicle_attitude_setpoint"])

    # ---- plan: altitude, survey lines (ENU)
    items = json.loads(a.plan.read_text())["mission"]["items"]
    alt_plan = next(i["params"][6] for i in items if i["command"] == 22)
    wps = [latlon_to_enu(i["params"][4], i["params"][5]) for i in items if i["command"] == 16]
    legs = list(zip(wps[:-1], wps[1:]))
    lines = [(p, q) for p, q in legs if math.dist(p, q) > LINE_MIN]

    # ---- mission time window
    st = data(ul, "vehicle_status")
    t_st, nav = st["timestamp"] / 1e6, st["nav_state"]
    on = np.where(nav == NAV_MISSION)[0]
    if not len(on):
        raise SystemExit("no AUTO_MISSION in this log")
    t0 = t_st[on[0]]
    after = np.where((t_st > t0) & (nav != NAV_MISSION))[0]
    t1 = t_st[after[0]] if len(after) else t_st[-1]
    inm = lambda t: (t >= t0) & (t <= t1)

    # ---- 1+2. ground-truth path vs plan
    gt = data(ul, "vehicle_global_position_groundtruth")
    tg = gt["timestamp"] / 1e6
    E = np.array([latlon_to_enu(la, lo) for la, lo in zip(gt["lat"], gt["lon"])])
    alt0 = float(np.median(gt["alt"][tg < t0 + 0.5]))       # ground level before take-off
    h = gt["alt"] - alt0
    xte, alt_err, line_id, t_line = [], [], [], []
    n_win = n_turn = n_far = n_low = 0          # mission samples / dropped ones, for the summary
    for k in np.where(inm(tg))[0]:
        n_win += 1
        best = None
        for j, (p, q) in enumerate(lines):
            d = np.subtract(q, p); L = np.hypot(*d); u = d / L
            r = E[k] - p
            s = r @ u
            if END_TRIM <= s <= L - END_TRIM:
                c = u[0] * r[1] - u[1] * r[0]          # signed cross-track (left +)
                if best is None or abs(c) < abs(best[0]):
                    best = (c, j)
        if best is None:
            n_turn += 1                          # on a turn / within END_TRIM of a line end
        elif abs(best[0]) >= 5.0:
            n_far += 1                           # >= 5 m from every line: NOT in the RMSE
        elif h[k] <= alt_plan * 0.8:
            n_low += 1                           # still climbing / descending
        else:
            xte.append(best[0]); line_id.append(best[1]); t_line.append(tg[k]); alt_err.append(h[k] - alt_plan)
    xte, alt_err, line_id, t_line = map(np.array, (xte, alt_err, line_id, t_line))
    per_line = [rms(xte[line_id == j]) for j in range(len(lines))]

    # ---- 3. tracking: estimate vs setpoint
    lp, sp = data(ul, "vehicle_local_position"), data(ul, "vehicle_local_position_setpoint")
    ts = sp["timestamp"] / 1e6
    ok = inm(ts) & np.isfinite(sp["x"]) & np.isfinite(sp["y"]) & np.isfinite(sp["z"])
    tl = lp["timestamp"] / 1e6
    ex = np.interp(ts[ok], tl, lp["x"]) - sp["x"][ok]
    ey = np.interp(ts[ok], tl, lp["y"]) - sp["y"][ok]
    ez = np.interp(ts[ok], tl, lp["z"]) - sp["z"][ok]
    trk_h, trk_3d = rms(np.hypot(ex, ey)), rms(np.sqrt(ex ** 2 + ey ** 2 + ez ** 2))

    # ---- 4. attitude error
    at, asp = data(ul, "vehicle_attitude"), data(ul, "vehicle_attitude_setpoint")
    ta, tsp = at["timestamp"] / 1e6, asp["timestamp"] / 1e6
    r_a, p_a = euler(*(at[f"q[{i}]"] for i in range(4)))
    r_s, p_s = euler(*(asp[f"q_d[{i}]"] for i in range(4)))
    m = inm(tsp)
    er = np.interp(tsp[m], ta, r_a) - r_s[m]
    ep = np.interp(tsp[m], ta, p_a) - p_s[m]
    e_tilt = np.hypot(er, ep)
    att = {"roll_rms": rms(er), "pitch_rms": rms(ep), "max": float(np.max(np.abs(np.r_[er, ep]))),
           "pct_over": float(100 * np.mean(e_tilt > REQ["att_err"]))}

    # ---- 5. take-off altitude response
    tk = np.where(inm(tg) & (h > 0.2))[0]
    if not len(tk):
        raise SystemExit("drone chưa rời đất trong chế độ nhiệm vụ (không có dữ liệu bay)")
    ts0 = tg[tk[0]]
    w = (tg >= ts0) & (tg <= ts0 + 30)
    tw, hw = tg[w] - ts0, h[w]
    overshoot = max(0.0, (hw.max() - alt_plan) / alt_plan * 100)
    t10 = tw[np.argmax(hw >= 0.1 * alt_plan)]; t90 = tw[np.argmax(hw >= 0.9 * alt_plan)]
    outside = np.where(np.abs(hw - alt_plan) > 0.02 * alt_plan)[0]
    settle = float(tw[outside[-1] + 1]) if len(outside) and outside[-1] + 1 < len(tw) else float("nan")

    # ---- figures
    plt.rcParams.update({"figure.dpi": 110, "font.size": 10})
    f, ax = plt.subplots(figsize=(7, 8))
    path = np.array([(0, 0)] + wps)
    ax.plot(path[:, 0], path[:, 1], "--", color="gray", lw=1.5, label="Kế hoạch")
    mm = inm(tg)
    ax.plot(E[mm, 0], E[mm, 1], color="tab:blue", lw=1.2, label="Thực tế (ground truth)")
    ax.plot(E[tg > t1, 0], E[tg > t1, 1], color="tab:red", lw=1, alpha=0.7, label="Sau nhiệm vụ (RTL)")
    ax.plot(0, 0, "r*", ms=14, label="Home")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("Đông (m)"); ax.set_ylabel("Bắc (m)")
    ax.set_title(f"Quỹ đạo bay — RMSE lệch ngang {rms(xte):.2f} m"); ax.legend(fontsize=8)
    f.tight_layout(); f.savefig(out / "1_quy_dao.png"); plt.close(f)

    f, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for j in range(len(lines)):
        s = line_id == j
        a1.plot(t_line[s] - t0, xte[s], ".", ms=2, label=f"Đường {j + 1}: RMSE {per_line[j]:.2f} m")
    for y in (REQ["pos_rmse"], -REQ["pos_rmse"]):
        a1.axhline(y, color="r", ls="--", lw=1)
    a1.set_ylabel("Lệch ngang (m)"); a1.legend(fontsize=8, ncol=2); a1.grid(alpha=0.3)
    a1.set_title(f"Sai số bám đường khảo sát (±{REQ['pos_rmse']} m = yêu cầu)")
    a2.plot(tg[mm] - t0, h[mm], lw=1, label="Độ cao thực")
    a2.axhline(alt_plan, color="gray", ls="--", label=f"Kế hoạch {alt_plan:g} m")
    a2.set_ylabel("Độ cao (m)"); a2.set_xlabel("Thời gian từ lúc bắt đầu nhiệm vụ (s)")
    a2.legend(fontsize=8); a2.grid(alpha=0.3)
    f.tight_layout(); f.savefig(out / "2_sai_so_vi_tri.png"); plt.close(f)

    f, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for axx, act, setp, name in [(axs[0], r_a, r_s, "Roll"), (axs[1], p_a, p_s, "Pitch")]:
        mt = inm(ta)
        axx.plot(ta[mt] - t0, act[mt], lw=0.8, label=f"{name} thực")
        axx.plot(tsp[m] - t0, setp[m], lw=0.8, label=f"{name} đặt")
        axx.set_ylabel(f"{name} (°)"); axx.legend(fontsize=8); axx.grid(alpha=0.3)
    axs[2].plot(tsp[m] - t0, e_tilt, lw=0.8, color="tab:purple")
    axs[2].axhline(REQ["att_err"], color="r", ls="--", lw=1, label=f"{REQ['att_err']}° = yêu cầu")
    axs[2].set_ylabel("|sai số góc| (°)"); axs[2].set_xlabel("Thời gian từ lúc bắt đầu nhiệm vụ (s)")
    axs[2].legend(fontsize=8); axs[2].grid(alpha=0.3)
    axs[0].set_title(f"Bám góc: RMS roll {att['roll_rms']:.2f}°, pitch {att['pitch_rms']:.2f}°, max {att['max']:.2f}°")
    f.tight_layout(); f.savefig(out / "3_sai_so_goc.png"); plt.close(f)

    f, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(tw, hw, label="Độ cao thực")
    ax.axhline(alt_plan, color="gray", ls="--", label=f"Đích {alt_plan:g} m")
    ax.axhspan(alt_plan * 0.98, alt_plan * 1.02, color="green", alpha=0.15, label="Dải ±2 %")
    if math.isfinite(settle):
        ax.axvline(settle, color="g", ls=":", label=f"Xác lập {settle:.1f} s")
    ax.set_xlabel("Thời gian từ lúc rời đất (s)"); ax.set_ylabel("Độ cao (m)"); ax.grid(alpha=0.3)
    ax.set_title(f"Cất cánh: vọt lố {overshoot:.1f} %, tăng 10–90 % {t90 - t10:.1f} s"); ax.legend(fontsize=8)
    f.tight_layout(); f.savefig(out / "4_cat_canh.png"); plt.close(f)

    # ---- summary
    ok_ = lambda v, lim: "ĐẠT" if v <= lim else "KHÔNG ĐẠT"
    rows = [
        ("RMSE lệch ngang so với đường khảo sát (vị trí thật)", f"{rms(xte):.3f} m", f"< {REQ['pos_rmse']} m",
         ok_(rms(xte), REQ["pos_rmse"])),
        ("RMSE độ cao trên đường khảo sát", f"{rms(alt_err):.3f} m", "(tham khảo)", ""),
        ("RMSE bám điểm đặt, ngang (PID vị trí)", f"{trk_h:.3f} m", f"< {REQ['pos_rmse']} m", ok_(trk_h, REQ["pos_rmse"])),
        ("RMSE bám điểm đặt, 3D", f"{trk_3d:.3f} m", "(tham khảo)", ""),
        ("Sai số góc lớn nhất (roll/pitch)", f"{att['max']:.2f}°", f"< {REQ['att_err']}°", ok_(att["max"], REQ["att_err"])),
        ("RMS sai số roll / pitch", f"{att['roll_rms']:.2f}° / {att['pitch_rms']:.2f}°", "(tham khảo)", ""),
        ("Thời gian sai số góc > 5°", f"{att['pct_over']:.1f} %", "(tham khảo)", ""),
        ("Vọt lố độ cao khi cất cánh*", f"{overshoot:.1f} %", f"< {REQ['overshoot']} %", ok_(overshoot, REQ["overshoot"])),
        ("Thời gian xác lập độ cao ±2 %*", f"{settle:.1f} s", "(tham khảo)", ""),
    ]
    acc_rad = ul.initial_parameters.get("NAV_ACC_RAD", float("nan"))
    warn = []
    if acc_rad > 3.0:
        warn = ["", f"> ⚠️ **NAV_ACC_RAD = {acc_rad:g} m lúc bay (nên ≤ 2 m).** PX4 coi là đã tới điểm bay khi còn "
                    "cách chừng đó, nên drone cắt góc và bỏ qua đoạn ngắn: RMSE lệch ngang bị phóng đại. "
                    "Bay lại với airframe/plan mới (NAV_ACC_RAD = 1,5 m)."]
    if n_far:
        warn += ["", f"> ⚠️ **{n_far} mẫu cách mọi đường bay ≥ 5 m đã bị loại khỏi RMSE lệch ngang** (drone lệch xa): "
                     "RMSE thực tế LỚN HƠN số dưới đây. Xem 1_quy_dao.png."]
    if len(alt_err) and abs(float(np.median(alt_err))) > 1.0:
        warn += ["", f"> ⚠️ Độ cao thực trên đường khảo sát lệch kế hoạch {np.median(alt_err):+.1f} m: "
                     "có thể đang dùng sai file plan (tuỳ chọn --plan)."]
    if len(xte) < 50:
        warn += ["", f"> ⚠️ Chỉ có {len(xte)} mẫu trên đường khảo sát: kết quả không đáng tin."]
    md = [f"# Kết quả bay: {label}", "", f"- Log: `{log}`", f"- Plan: `{a.plan.name}` (độ cao {alt_plan:g} m, "
          f"{len(lines)} đường khảo sát)", f"- Thời gian nhiệm vụ: {t1 - t0:.0f} s",
          f"- Mẫu vị trí trong nhiệm vụ: {n_win}. Dùng cho RMSE lệch ngang: {len(xte)}. Loại: {n_turn} ở chỗ rẽ / "
          f"{END_TRIM:g} m đầu mỗi đường, {n_far} cách mọi đường ≥ 5 m, {n_low} dưới 80 % độ cao",
          f"- NAV_ACC_RAD lúc bay: {acc_rad:g} m", *warn, "",
          "| Chỉ tiêu | Kết quả | Yêu cầu | Đánh giá |", "|---|---|---|---|"]
    md += [f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} |" for r in rows]
    md += ["", "RMSE từng đường: " + ", ".join(f"đường {j + 1} = {v:.3f} m" for j, v in enumerate(per_line)),
           "", "*Cất cánh của PX4 là quỹ đạo mượt, không phải bước nhảy: cần bài thử bước (step test)"
           " riêng để đánh giá vọt lố < 15 % và xác lập < 2 s của vòng điều khiển góc.",
           "", "Biểu đồ: 1_quy_dao.png, 2_sai_so_vi_tri.png, 3_sai_so_goc.png, 4_cat_canh.png"]
    (out / "tong_ket.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))
    print(f"\n-> {out}")


if __name__ == "__main__":
    main()
