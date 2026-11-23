#!/usr/bin/env python3
"""
Generate models/rice_plot: 3D direct-seeded rice (~17 days after sowing)
filling the paddy of the X450 world.

Output (everything inside models/rice_plot is overwritten, do not edit by hand):
  model.config, model.sdf     Gazebo model, included by worlds/rice.sdf
  meshes/d<C>_v<V>.stl        rice tile meshes: density class C, random variant V
  meshes/weed_v<V>.stl        broad-leaf weed tile meshes
  meshes/soil.stl             mud under the plants (paddy shape)
  ground_truth.csv            real plant count of every tile (world ENU + lat/lon)

Layout (survey frame of tools/mission/field_site.py, origin = NW paddy corner):
  - square tiles (default 2 x 2 m) in columns from the West bank to the east,
    each column starting at the North bank; tiles whose centre is not
    --margin metres inside the paddy (bunds) are dropped.
  - smooth random density map (plants/m^2) quantised into classes; each class
    has a few random tile meshes, placed rotated by 0/90/180/270 deg.
Realism:
  - gaps: a few round patches where seeds failed (~10 plants/m^2)
  - weeds: broad-leaf weeds on ~12 % of the tiles
  - puddles: shallow standing water in low spots
  - leaf colour varies smoothly (darker green <-> yellowish patches)
  - one plant = 3..4 leaves, one leaf = one thin triangle (both windings).
    Visual only: no collision, no shadows from the rice (cheap to render).

Only needs python3 + numpy. Same --seed => same field.
Usage:  python3 tools/mission/gen_rice_field.py            (defaults)
        python3 tools/mission/gen_rice_field.py --help     (all options)
"""
import argparse
import math
import shutil
from pathlib import Path

import numpy as np
from field_site import CORNER_NW, YAW, enu_to_latlon, north_bank_v, paddy_corners_uv, survey_to_enu

REPO = Path(__file__).resolve().parents[2]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", type=Path, default=REPO / "simulation" / "models" / "rice_plot")
    p.add_argument("--seed", type=int, default=17)
    p.add_argument("--tile", type=float, default=2.0, help="tile side [m]")
    p.add_argument("--margin", type=float, default=0.6, help="keep tiles this far inside the bunds [m]")
    p.add_argument("--z", type=float, default=0.02, help="ground height [m]")
    p.add_argument("--density-min", type=float, default=80.0, help="plants/m^2")
    p.add_argument("--density-max", type=float, default=320.0, help="plants/m^2")
    p.add_argument("--classes", type=int, default=5, help="density classes")
    p.add_argument("--variants", type=int, default=3, help="meshes per class")
    p.add_argument("--height", type=float, default=0.22, help="mean plant height [m]")
    p.add_argument("--leaf-width", type=float, default=0.008, help="leaf base width [m]")
    p.add_argument("--gaps", type=int, default=5, help="number of failed-seeding patches")
    p.add_argument("--weeds", type=float, default=0.12, help="fraction of tiles with weeds")
    p.add_argument("--puddles", type=int, default=6, help="number of standing-water puddles")
    return p.parse_args()


# ---------------------------------------------------------------- geometry
def make_tile(rng, n_plants, tile, height, leaf_w, leaves=(3, 5), elev=(45, 80), spread=0.35):
    """Return (T,3,3) triangles of one tile centred at (0,0), ground at z=0."""
    half = tile / 2.0
    tris = []
    for _ in range(n_plants):
        base = np.array([rng.uniform(-half, half), rng.uniform(-half, half), 0.0])
        h = max(0.05, rng.normal(height, 0.15 * height))
        n_leaves = rng.integers(*leaves)
        az0 = rng.uniform(0, 2 * math.pi)
        for k in range(n_leaves):
            # rice leaves are distichous: alternate on two opposite sides
            az = az0 + k * math.pi + rng.normal(0, spread)
            el = rng.uniform(math.radians(elev[0]), math.radians(elev[1]))
            length = h * rng.uniform(0.6, 1.0)
            tip = base + length * np.array([math.cos(el) * math.cos(az),
                                            math.cos(el) * math.sin(az), math.sin(el)])
            side = 0.5 * leaf_w * np.array([-math.sin(az), math.cos(az), 0.0])
            tris.append((base + side, base - side, tip))
    t = np.array(tris, dtype=np.float32)
    t[:, :, :2] = np.clip(t[:, :, :2], -half, half)
    return t


def write_stl(path, tris):
    """Binary STL, every triangle twice (front + back face)."""
    both = np.concatenate([tris, tris[:, ::-1, :]])
    n = np.cross(both[:, 1] - both[:, 0], both[:, 2] - both[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(both), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, both
    with open(path, "wb") as f:
        f.write(b"x450_sim rice tile".ljust(80, b" "))
        f.write(np.uint32(len(both)).tobytes())
        f.write(rec.tobytes())
    return len(both)


def smooth_field(rng, xs, ys, n_bumps=10, scale=(6, 14)):
    """Smooth random field in [0, 1] at points (xs, ys) [m]: sum of Gaussian bumps."""
    g = np.zeros_like(xs, dtype=float)
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    for _ in range(n_bumps):
        cx, cy, s = rng.uniform(x0, x1), rng.uniform(y0, y1), rng.uniform(*scale)
        g += rng.uniform(-1, 1) * np.exp(-((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * s * s))
    return (g - g.min()) / max(np.ptp(g), 1e-9)


def inside(poly, u, v, margin):
    """Point (u, v) at least `margin` inside the clockwise polygon `poly`."""
    for i in range(len(poly)):
        (u1, v1), (u2, v2) = poly[i], poly[(i + 1) % len(poly)]
        L = math.hypot(u2 - u1, v2 - v1)
        if -((u2 - u1) * (v - v1) - (v2 - v1) * (u - u1)) / L < margin:
            return False
    return True


def material(rgb):
    r, g, b = rgb
    return (f"<material><ambient>{r * 0.72:.3f} {g * 0.72:.3f} {b * 0.72:.3f} 1</ambient>"
            f"<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>")


# ---------------------------------------------------------------- main
def main():
    a = parse_args()
    rng = np.random.default_rng(a.seed)
    area = a.tile * a.tile
    levels = np.linspace(a.density_min, a.density_max, a.classes)
    poly = paddy_corners_uv()                          # NW, NE, SE, SW (clockwise)

    out = a.out.resolve()
    if out.name != "rice_plot":   # safety: never wipe an unexpected folder
        raise SystemExit(f"refusing to overwrite {out} (folder must be named rice_plot)")
    if out.exists():
        shutil.rmtree(out)
    (out / "meshes").mkdir(parents=True)

    # 1. meshes: rice per density class (+ "gap" class 0), weeds, soil
    counts, tile_tris = {}, {}
    all_levels = [10.0] + list(levels)                 # class 0 = failed seeding
    for c, d in enumerate(all_levels):
        for v in range(a.variants):
            n_pl = int(round(d * area * rng.uniform(0.95, 1.05)))
            tris = make_tile(rng, n_pl, a.tile, a.height, a.leaf_width)
            tile_tris[("rice", c, v)] = write_stl(out / "meshes" / f"d{c}_v{v}.stl", tris)
            counts[(c, v)] = n_pl
    for v in range(2):     # broad-leaf weeds: few, short, wide, flat leaves
        tris = make_tile(rng, int(rng.uniform(25, 45)), a.tile, 0.10, 0.03,
                         leaves=(4, 7), elev=(15, 45), spread=1.2)
        tile_tris[("weed", 0, v)] = write_stl(out / "meshes" / f"weed_v{v}.stl", tris)
    soil = [(*p, 0.0) for p in poly]
    write_stl(out / "meshes" / "soil.stl",
              np.array([[soil[0], soil[i], soil[i + 1]] for i in range(1, len(soil) - 1)], np.float32))

    # 2. tile grid (model frame = survey frame) clipped to the paddy
    umax = max(p[0] for p in poly)
    vmin = min(p[1] for p in poly)
    tiles = []
    for ix in range(int(math.ceil(umax / a.tile))):
        lx = (ix + 0.5) * a.tile
        for iy in range(int(math.ceil((north_bank_v(lx) - vmin) / a.tile)) + 1):
            ly = north_bank_v(lx) - (iy + 0.5) * a.tile
            if inside(poly, lx, ly, a.margin):
                tiles.append((ix, iy, lx, ly))
    U = np.array([t[2] for t in tiles])
    V = np.array([t[3] for t in tiles])

    # smooth pattern, then rank -> spread evenly over [min, max], skewed to dense (mean ~ 220/m^2)
    rank = smooth_field(rng, U, V).argsort().argsort() / max(len(U) - 1, 1)
    dens = a.density_min + rank ** 0.7 * (a.density_max - a.density_min)
    cls = 1 + np.abs(dens[:, None] - levels).argmin(axis=1)
    gaps = [(rng.uniform(U.min() + 4, U.max() - 4), rng.uniform(V.min() + 4, V.max() - 4),
             rng.uniform(1.5, 3.5)) for _ in range(a.gaps)]
    is_gap = np.array([any(math.hypot(u - gu, v - gv) < gr for gu, gv, gr in gaps) for u, v in zip(U, V, strict=True)])
    cls[is_gap] = 0
    has_weed = rng.random(len(tiles)) < a.weeds
    tint = smooth_field(rng, U, V, n_bumps=8, scale=(5, 12))      # 0 dark green .. 1 yellowish

    visuals, rows, total_tris = [], [], 0
    for k, (ix, iy, lx, ly) in enumerate(tiles):
        c, v = int(cls[k]), int(rng.integers(a.variants))
        yaw = int(rng.integers(4)) * math.pi / 2
        t = tint[k]
        rgb = (0.34 + 0.20 * t + rng.normal(0, 0.015), 0.60 - 0.04 * t + rng.normal(0, 0.015),
               0.16 + 0.02 * t)
        total_tris += tile_tris[("rice", c, v)]
        visuals.append(f"""      <visual name="t_{ix:02d}_{iy:02d}">
        <pose>{lx:.3f} {ly:.3f} 0 0 0 {yaw:.4f}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://rice_plot/meshes/d{c}_v{v}.stl</uri></mesh></geometry>
        {material(rgb)}
      </visual>""")
        if has_weed[k]:
            wv = int(rng.integers(2))
            total_tris += tile_tris[("weed", 0, wv)]
            visuals.append(f"""      <visual name="w_{ix:02d}_{iy:02d}">
        <pose>{lx:.3f} {ly:.3f} 0 0 0 {rng.uniform(0, 6.28):.4f}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://rice_plot/meshes/weed_v{wv}.stl</uri></mesh></geometry>
        {material((0.20, 0.42, 0.14))}
      </visual>""")
        wx, wy = survey_to_enu(lx, ly)
        lat, lon = enu_to_latlon(wx, wy)
        p = counts[(c, v)]
        rows.append(f"{ix},{iy},{wx:.3f},{wy:.3f},{lat:.8f},{lon:.8f},{c},{v},{p},{p / area:.1f},"
                    f"{int(is_gap[k])},{int(has_weed[k])}")

    puddles = []
    for i in range(a.puddles):     # shallow water in low spots: flat disks just above the mud
        k = int(rng.integers(len(tiles)))
        r = rng.uniform(0.8, 2.5)
        puddles.append(f"""      <visual name="puddle_{i}">
        <pose>{U[k] + rng.uniform(-1, 1):.3f} {V[k] + rng.uniform(-1, 1):.3f} 0.004 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><cylinder><radius>{r:.2f}</radius><length>0.004</length></cylinder></geometry>
        <material><ambient>0.08 0.11 0.11 1</ambient><diffuse>0.15 0.20 0.20 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular></material>
      </visual>""")

    # 3. ground_truth.csv
    (out / "ground_truth.csv").write_text(
        "tile_ix,tile_iy,x_enu_m,y_enu_m,lat_deg,lon_deg,density_class,variant,"
        "plants,plants_per_m2,gap,weeds\n" + "\n".join(rows) + "\n")

    # 4. model.config + model.sdf
    (out / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>rice_plot</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>GENERATED by tools/mission/gen_rice_field.py (seed {a.seed}) - do not edit by hand</description>
</model>
""")
    total_plants = sum(int(r.split(",")[8]) for r in rows)
    paddy_area = 0.5 * abs(sum(poly[i][0] * poly[(i + 1) % 4][1] - poly[(i + 1) % 4][0] * poly[i][1]
                               for i in range(4)))
    (out / "model.sdf").write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  GENERATED by tools/mission/gen_rice_field.py - do not edit by hand, re-run the script.
  Direct-seeded rice ~17 days after sowing filling the paddy ({paddy_area:.0f} m2), tiles {a.tile:g} m,
  model origin = NW paddy corner, yaw {math.degrees(YAW):.2f} deg, seed {a.seed}.
  Density {a.density_min:g}..{a.density_max:g} plants/m2 ({a.classes} classes x {a.variants} variants),
  {a.gaps} failed-seeding gaps, weeds on {int(has_weed.sum())} tiles, {a.puddles} puddles.
  Plants: {total_plants}   triangles: {total_tris}
  Ground truth per tile: model://rice_plot/ground_truth.csv
-->
<sdf version="1.9">
  <model name="rice_plot">
    <static>true</static>
    <pose>{CORNER_NW[0]:.3f} {CORNER_NW[1]:.3f} {a.z:.3f} 0 0 {YAW:.5f}</pose>
    <link name="link">
      <visual name="soil">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://rice_plot/meshes/soil.stl</uri></mesh></geometry>
        <material>
          <ambient>0.22 0.18 0.13 1</ambient>
          <diffuse>0.30 0.25 0.18 1</diffuse>
          <specular>0.10 0.10 0.10 1</specular>
        </material>
      </visual>
{chr(10).join(puddles)}
{chr(10).join(visuals)}
    </link>
  </model>
</sdf>
""")

    print(f"rice_plot -> {out}")
    print(f"  {len(tiles)} tiles ({paddy_area:.0f} m2 paddy), {total_plants} plants, "
          f"{total_tris / 1e6:.2f} M triangles")
    print(f"  gaps {int(is_gap.sum())} tiles, weeds {int(has_weed.sum())} tiles, {a.puddles} puddles, "
          f"mean density {total_plants / (len(tiles) * area):.0f} plants/m2")


if __name__ == "__main__":
    main()
