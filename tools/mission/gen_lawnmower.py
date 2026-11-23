#!/usr/bin/env python3
"""
Generate a lawn-mower (boustrophedon) survey mission as a QGroundControl .plan.

Pattern (see tools/mission/field_site.py for the survey frame):
  TAKEOFF at home (on the road, NW corner) to ALT -> DO_CHANGE_SPEED
  -> point 2: into the paddy, INSET_WEST from the West bank and MARGIN_NORTH
     from the North bank
  line 1: south along the West bank (parallel to the road), LINE_LENGTH metres
  turn left (east) by SPACING, line 2 back north, line 3 south, ...
  north ends follow the North bank; lines are added while they stay
  MARGIN_EAST inside the East bank
  -> RTL (return altitude = RTL_RETURN_ALT in firmware/airframes/4052_gz_f450, 6 m)

Line spacing:  --spacing S (m)  or  --overlap P (side overlap, 0..1)
  camera footprint across track = 2 * ALT * tan(HFOV / 2)
  spacing = footprint * (1 - overlap)

Usage (QGC: Plan -> Open... -> the .plan file -> Upload -> Fly: Start Mission):
  python3 tools/gen_lawnmower.py                           # 6 m, 1.5 m/s, 30 %
  python3 tools/gen_lawnmower.py --alt 8 --speed 2 --overlap 0.5
"""
import argparse
import json
import math
from pathlib import Path

from field_site import (
    BANK_BEARING_DEG,
    EAST_BANK_MIN_U,
    HOME_LAT,
    HOME_LON,
    INSET_WEST,
    LINE_LENGTH,
    MARGIN_EAST,
    MARGIN_NORTH,
    enu_to_latlon,
    north_bank_v,
    survey_to_enu,
)

REPO = Path(__file__).resolve().parents[2]
CAM_HFOV = 1.855  # rad, models/caddx_cam (image width is across track)

# MAVLink commands / frames
NAV_WAYPOINT, NAV_RTL, NAV_TAKEOFF, DO_CHANGE_SPEED = 16, 20, 22, 178
FRAME_GLOBAL_REL_ALT, FRAME_MISSION = 3, 2


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--alt", type=float, default=6.0, help="altitude AGL [m]")
    p.add_argument("--speed", type=float, default=1.5, help="survey ground speed [m/s]")
    p.add_argument("--accept", type=float, default=1.5,
                   help="waypoint acceptance radius [m] (PX4 default 10 m cuts the turns)")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--overlap", type=float, default=0.3, help="side overlap 0..1")
    g.add_argument("--spacing", type=float, help="line spacing [m] (overrides --overlap)")
    p.add_argument("--out", type=Path, help="output .plan (default: simulation/missions/<name>.plan)")
    return p.parse_args()


def survey_waypoints(spacing):
    """Waypoints (world ENU) after takeoff at home, in flight order."""
    wps, n = [], 0
    while INSET_WEST + n * spacing <= EAST_BANK_MIN_U - MARGIN_EAST + 1e-9:
        u = INSET_WEST + n * spacing
        top = survey_to_enu(u, north_bank_v(u) - MARGIN_NORTH)
        bottom = survey_to_enu(u, north_bank_v(u) - MARGIN_NORTH - LINE_LENGTH)
        wps += [top, bottom] if n % 2 == 0 else [bottom, top]   # 1st line flies south
        n += 1
    return wps, n


def item(seq, cmd, frame, params):
    return {"type": "SimpleItem", "autoContinue": True, "command": cmd,
            "doJumpId": seq, "frame": frame, "params": params}


def main():
    a = parse_args()
    footprint = 2 * a.alt * math.tan(CAM_HFOV / 2)
    spacing = a.spacing if a.spacing else footprint * (1 - a.overlap)
    wps, n = survey_waypoints(spacing)

    items = [item(1, NAV_TAKEOFF, FRAME_GLOBAL_REL_ALT,
                  [0, 0, 0, None, HOME_LAT, HOME_LON, a.alt]),
             item(2, DO_CHANGE_SPEED, FRAME_MISSION, [1, a.speed, -1, 0, 0, 0, 0])]
    for x, y in wps:
        lat, lon = enu_to_latlon(x, y)
        items.append(item(len(items) + 1, NAV_WAYPOINT, FRAME_GLOBAL_REL_ALT,
                          [0, a.accept, 0, None, round(lat, 8), round(lon, 8), a.alt]))
    items.append(item(len(items) + 1, NAV_RTL, FRAME_MISSION, [0] * 7))
    for it in items:   # QGC also wants these on altitude items
        if it["frame"] == FRAME_GLOBAL_REL_ALT:
            it.update({"Altitude": a.alt, "AltitudeMode": 1, "AMSLAltAboveTerrain": None})

    plan = {
        "fileType": "Plan", "version": 1, "groundStation": "QGroundControl",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {"version": 2, "firmwareType": 12, "vehicleType": 2,
                    "cruiseSpeed": 2, "hoverSpeed": a.speed,   # 2 = MPC_XY_CRUISE (RTL)
                    "globalPlanAltitudeMode": 1,
                    "plannedHomePosition": [HOME_LAT, HOME_LON, 6],
                    "items": items},
    }

    name = f"lawnmower_a{a.alt:g}_v{a.speed:g}_s{spacing:.1f}"
    out = a.out or REPO / "simulation" / "missions" / f"{name}.plan"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(plan, indent=2) + "\n")

    path = [(0.0, 0.0)] + wps + [(0.0, 0.0)]
    length = sum(math.dist(p, q) for p, q in zip(path, path[1:], strict=False))
    print(f"{out}")
    print(f"  alt {a.alt:g} m, speed {a.speed:g} m/s, footprint {footprint:.1f} m across track")
    print(f"  {n} lines along bearing {(BANK_BEARING_DEG + 180) % 360:g}/{BANK_BEARING_DEG:g} deg, "
          f"spacing {spacing:.2f} m "
          f"(side overlap {(1 - spacing / footprint) * 100:.0f} %), {LINE_LENGTH:g} m each")
    print("  waypoints ENU: " + "  ".join(f"({x:.1f},{y:.1f})" for x, y in wps))
    print(f"  path ~{length:.0f} m incl. return, ~{length / a.speed / 60:.1f} min at {a.speed:g} m/s")


if __name__ == "__main__":
    main()
