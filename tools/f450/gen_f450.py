#!/usr/bin/env python3
"""
Sinh model Gazebo + airframe PX4 cho drone TEST: khung F450 (clone, chân đáp liền tay)
+ toàn bộ phần điện của đồ án.

Cách dùng:
    python3 tools/f450/gen_f450.py          # in bảng thông số và ghi lại các file bên dưới

Ghi ra:
    simulation/models/f450/model.sdf (+ model.config)
    simulation/models/f450_cam_down/model.sdf (+ model.config)
    firmware/airframes/4052_gz_f450, firmware/airframes/4053_gz_f450_cam_down
    docs/f450_params.md

Hệ trục (Gazebo, thân drone): x tới trước, y sang trái, z lên trên.
Gốc toạ độ: tâm khung, ngang mặt trên của tấm dưới (mặt tay F450 bắt vào).

=> KHI CÓ SỐ ĐO THẬT: chỉ sửa bảng PARTS (khối lượng, vị trí) và các hằng số
   ở đầu file, rồi chạy lại script. Cột "src" ghi rõ số lấy từ đâu.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
G = 9.80665

# ---------------------------------------------------------------- hình học
WHEELBASE = 0.450                       # m, đường chéo trục motor-trục motor (DJI F450)
R_MOTOR = WHEELBASE / 2                 # 0.225 m từ tâm
XY = R_MOTOR / math.sqrt(2)             # 0.1591 m theo x và y
Z_MOTOR_BASE = 0.010                    # mặt đế motor trên tay [ƯỚC LƯỢNG – đo lại]
Z_PROP = 0.050                          # độ cao mặt cánh [ƯỚC LƯỢNG – đo lại]
Z_FOOT = -0.120                         # đáy chân đáp (điểm chạm đất) [ƯỚC LƯỢNG – đo lại]

# ---------------------------------------------------------------- motor + cánh (giữ như đồ án)
PROP_D = 0.254                          # cánh 1045: 10 inch
CT, CP, RHO = 0.11, 0.045, 1.225        # GIẢ THIẾT (giống bản v21) – thay khi có bảng lực đẩy
K_T = CT * RHO * PROP_D**4 / (4 * math.pi**2)    # N/(rad/s)^2
K_M = CP * PROP_D / (2 * math.pi * CT)           # m
W_MAX = 743.2                           # rad/s  (800 gf/motor theo nhà bán, chưa kiểm chứng)
W_MIN = 111.5                           # rad/s  (15 % W_MAX, như x500)
PROP_MASS = 0.012                       # kg, GIẢ THIẾT


@dataclass
class Part:
    name: str
    m: float                            # kg
    pos: tuple[float, float, float]     # m, trọng tâm của chi tiết
    size: tuple[float, float, float] = (0, 0, 0)   # hộp đặc a×b×c (0 = khối điểm)
    src: str = ""


def arms() -> list[Part]:
    """4 tay F450 (gồm chân đáp liền). Mỗi tay coi như thanh dọc theo đường chéo."""
    out = []
    for k, (sx, sy) in enumerate([(1, -1), (-1, 1), (1, 1), (-1, -1)]):
        r = 0.130                       # trọng tâm tay+chân, cách tâm [ƯỚC LƯỢNG]
        out.append(Part(f"tay_F450_{k}", 0.050, (sx * r / math.sqrt(2), sy * r / math.sqrt(2), -0.010),
                        src="F450 282 g tổng – chia: 4×50 g tay+chân, 82 g 2 tấm [ƯỚC LƯỢNG]"))
    return out


def motors() -> list[Part]:
    out = []
    for k, (sx, sy) in enumerate([(1, -1), (-1, 1), (1, 1), (-1, -1)]):
        out.append(Part(f"motor_2212_{k}", 0.066, (sx * XY, sy * XY, Z_MOTOR_BASE + 0.016),
                        src="60 g motor + 4 g adapter + 2 g đai ốc (bảng v21)"))
        out.append(Part(f"ESC_30A_{k}", 0.009, (sx * 0.075, sy * 0.075, -0.012),
                        src="9 g (bảng v21), dán dưới tay"))
    return out


PARTS: list[Part] = [
    *arms(),
    Part("2 tấm giữa F450 (PDB liền)", 0.082, (0, 0, 0.0), (0.140, 0.140, 0.030),
         "phần còn lại của 282 g [ƯỚC LƯỢNG]"),
    *motors(),
    Part("Pin 3S 4000 mAh + dây + XT60", 0.318, (0, 0, -0.045), (0.138, 0.043, 0.032),
         "bảng v21; treo DƯỚI tấm dưới [vị trí ƯỚC LƯỢNG]"),
    Part("Dây đai pin", 0.012, (0, 0, -0.045), src="bảng v21"),
    Part("Kakute H7 Mini + receiver RC", 0.010, (0, 0, 0.045), src="5.7 g FC (v21) + ~4 g receiver [ƯỚC LƯỢNG]"),
    Part("ESP32 + tấm mang", 0.013, (-0.045, 0, 0.040), src="bảng v21"),
    Part("GPS NEO-M8N + la bàn + cột", 0.025, (-0.040, 0, 0.110), src="10+1.5 g (v21) + cột ~13 g [ƯỚC LƯỢNG]"),
    Part("Camera Ant Nano + giá", 0.006, (0.080, 0, -0.020), src="[ƯỚC LƯỢNG]"),
    Part("VTX Unify Pro32 Nano + anten", 0.008, (-0.060, 0, 0.035), src="[ƯỚC LƯỢNG]"),
    Part("Dây, vít, cọc, băng dính", 0.045, (0, 0, 0.0), src="[ƯỚC LƯỢNG]"),
]


def inertia_of(p: Part) -> np.ndarray:
    a, b, c = p.size
    return p.m / 12 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])


def mass_properties(parts):
    m = sum(p.m for p in parts)
    cg = sum(p.m * np.array(p.pos) for p in parts) / m
    J = np.zeros((3, 3))
    for p in parts:
        d = np.array(p.pos) - cg
        J += inertia_of(p) + p.m * (d @ d * np.eye(3) - np.outer(d, d))
    return m, cg, J


ROTORS = [  # (k, x, y, spin)  thứ tự PX4 quad X
    (0, XY, -XY, "ccw"), (1, -XY, XY, "ccw"), (2, XY, XY, "cw"), (3, -XY, -XY, "cw")]


def rotor_link(k, x, y):
    L = PROP_D
    izz = PROP_MASS * L * L / 12
    return f"""
    <link name="rotor_{k}">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>{x:.4f} {y:.4f} {Z_PROP:.4f} 0 0 0</pose>
      <inertial>
        <mass>{PROP_MASS}</mass>
        <inertia><ixx>{PROP_MASS*(0.02**2)/12:.3e}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{izz:.3e}</iyy><iyz>0</iyz><izz>{izz:.3e}</izz></inertia>
      </inertial>
      <visual name="rotor_{k}_visual">
        <geometry><cylinder><radius>{L/2:.3f}</radius><length>0.003</length></cylinder></geometry>
        <material><diffuse>0.1 0.1 0.1 0.35</diffuse><ambient>0.1 0.1 0.1 0.35</ambient></material>
        <transparency>0.6</transparency>
      </visual>
      <collision name="rotor_{k}_collision">
        <geometry><cylinder><radius>{L/2:.3f}</radius><length>0.003</length></cylinder></geometry>
        <surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact></surface>
      </collision>
    </link>
    <joint name="rotor_{k}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{k}</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis>
    </joint>"""


def motor_plugin(k, spin):
    return f"""
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{k}_joint</jointName>
      <linkName>rotor_{k}</linkName>
      <turningDirection>{spin}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>{W_MAX}</maxRotVelocity>
      <motorConstant>{K_T:.5e}</motorConstant>
      <momentConstant>{K_M:.5f}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{k}</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>"""


def box_visual(name, pose, size, rgb):
    return f"""
      <visual name="{name}">
        <pose>{' '.join(f'{v:.4f}' for v in pose)}</pose>
        <geometry><box><size>{' '.join(f'{v:.4f}' for v in size)}</size></box></geometry>
        <material><diffuse>{rgb} 1</diffuse><ambient>{rgb} 1</ambient></material>
      </visual>"""


def frame_visuals():
    v = box_visual("tam_tren", (0, 0, 0.032, 0, 0, 0), (0.114, 0.114, 0.0016), "0.9 0.9 0.9")
    v += box_visual("tam_duoi", (0, 0, 0.0, 0, 0, 0), (0.140, 0.140, 0.0016), "0.75 0.6 0.2")
    v += box_visual("pin", (0, 0, -0.045, 0, 0, 0), (0.138, 0.043, 0.032), "0.2 0.3 0.6")
    v += box_visual("gps", (-0.040, 0, 0.120, 0, 0, 0), (0.035, 0.035, 0.008), "0.1 0.3 0.7")
    v += box_visual("cot_gps", (-0.040, 0, 0.075, 0, 0, 0), (0.008, 0.008, 0.085), "0.1 0.1 0.1")
    for k, x, y, _ in ROTORS:
        yaw = math.atan2(y, x)
        rgb = "0.85 0.1 0.1" if x > 0 else "0.95 0.95 0.95"   # tay trước đỏ, sau trắng (như F450)
        v += box_visual(f"tay_{k}", (x / 2, y / 2, 0.016, 0, 0, yaw), (R_MOTOR, 0.022, 0.030), rgb)
        v += box_visual(f"chan_{k}", (x * 0.95, y * 0.95, (Z_FOOT + 0.0) / 2, 0, 0, yaw),
                        (0.010, 0.010, -Z_FOOT), rgb)
        v += f"""
      <visual name="motor_{k}">
        <pose>{x:.4f} {y:.4f} {Z_MOTOR_BASE + 0.016:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.014</radius><length>0.030</length></cylinder></geometry>
        <material><diffuse>0.15 0.15 0.15 1</diffuse><ambient>0.15 0.15 0.15 1</ambient></material>
      </visual>"""
    return v


def foot_collisions():
    c = """
      <collision name="than_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.14 0.14 0.06</size></box></geometry>
      </collision>"""
    for k, x, y, _ in ROTORS:
        c += f"""
      <collision name="chan_{k}_collision">
        <pose>{x*0.95:.4f} {y*0.95:.4f} {Z_FOOT + 0.01:.4f} 0 0 0</pose>
        <geometry><box><size>0.02 0.02 0.02</size></box></geometry>
        <surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact>
          <friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>
      </collision>"""
    return c


def model_sdf(m, cg, J):
    sensors = (REPO / "tools/f450/sensors_x500.sdf.part").read_text()
    rotors = "".join(rotor_link(k, x, y) for k, x, y, _ in ROTORS)
    plugins = "".join(motor_plugin(k, s) for k, _, _, s in ROTORS)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- SINH TỰ ĐỘNG bởi tools/f450/gen_f450.py – đừng sửa tay, sửa script rồi chạy lại. -->
<sdf version="1.9">
  <model name="f450">
    <pose>0 0 {-Z_FOOT + 0.01:.3f} 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="base_link">
      <inertial>
        <pose>{cg[0]:.5f} {cg[1]:.5f} {cg[2]:.5f} 0 0 0</pose>
        <mass>{m:.4f}</mass>
        <inertia><ixx>{J[0,0]:.6e}</ixx><ixy>{J[0,1]:.3e}</ixy><ixz>{J[0,2]:.3e}</ixz><iyy>{J[1,1]:.6e}</iyy><iyz>{J[1,2]:.3e}</iyz><izz>{J[2,2]:.6e}</izz></inertia>
      </inertial>{frame_visuals()}{foot_collisions()}
{sensors}    </link>{rotors}{plugins}
    <plugin filename="MotorFailurePlugin" name="gz::sim::systems::MotorFailureSystem"/>
  </model>
</sdf>
"""


CAM_DOWN_SDF = """<?xml version="1.0" encoding="UTF-8"?>
<!-- SINH TỰ ĐỘNG bởi tools/f450/gen_f450.py -->
<sdf version="1.9">
  <model name="f450_cam_down">
    <self_collide>false</self_collide>
    <include merge="true"><uri>model://f450</uri></include>
    <include merge="true">
      <uri>model://caddx_cam</uri>
      <pose>0.080 0 -0.030 0 1.5707 0</pose>
      <name>caddx_cam</name>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
  </model>
</sdf>
"""


def model_config(name, desc):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
"""


def airframe(hover):
    rot = ""
    for k, x, y, s in ROTORS:
        km = 0.05 if s == "ccw" else -0.05
        rot += (f"param set-default CA_ROTOR{k}_PX {x:.4f}\n"
                f"param set-default CA_ROTOR{k}_PY {-y:.4f}\n"
                f"param set-default CA_ROTOR{k}_KM {km}\n\n")
    ec = "".join(f"param set-default SIM_GZ_EC_FUNC{i} 10{i}\n" for i in range(1, 5))
    ec += "".join(f"param set-default SIM_GZ_EC_MIN{i} {round(W_MIN)}\n" for i in range(1, 5))
    ec += "".join(f"param set-default SIM_GZ_EC_MAX{i} {round(W_MAX)}\n" for i in range(1, 5))
    return f"""#!/bin/sh
#
# @name Gazebo F450 test (dien do an)
#
# @type Quadrotor
#
# SINH TU DONG boi tools/f450/gen_f450.py

. ${{R}}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=default}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:=f450}}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

{rot}{ec}
param set-default MPC_THR_HOVER {hover:.2f}
param set-default NAV_DLL_ACT 2
param set-default BAT1_N_CELLS 3

# RTL o do cao khao sat (6 m). Bay that: kiem tra cay/day dien quanh ruong truoc.
param set-default RTL_RETURN_ALT 6
param set-default RTL_DESCEND_ALT 6

# Drag Specific Force (giu nhu x500)
param set-default EKF2_BCOEF_X 0.0
param set-default EKF2_BCOEF_Y 0.0
param set-default EKF2_MCOEF 0.12
"""


CAM_AIRFRAME = """#!/bin/sh
#
# @name Gazebo F450 test camera down
#
# @type Quadrotor
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=f450_cam_down}

. ${R}etc/init.d-posix/airframes/4052_gz_f450
"""


def main():
    m_body, cg, J = mass_properties(PARTS)
    m_tot = m_body + 4 * PROP_MASS
    T_hover = m_tot * G / 4
    w_hover = math.sqrt(T_hover / K_T)
    thr = (w_hover - W_MIN) / (W_MAX - W_MIN)
    T_max = K_T * W_MAX**2
    tw = 4 * T_max / (m_tot * G)
    ev = np.linalg.eigvalsh(J)
    assert ev.min() > 0 and ev[0] + ev[1] >= ev[2], "Ten-xo quan tinh khong hop le"

    def w(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    w(REPO / "simulation/models/f450/model.sdf", model_sdf(m_body, cg, J))
    w(REPO / "simulation/models/f450/model.config",
      model_config("f450", "Drone test: khung F450 clone + dien do an. Sinh boi tools/f450/gen_f450.py"))
    w(REPO / "simulation/models/f450_cam_down/model.sdf", CAM_DOWN_SDF)
    w(REPO / "simulation/models/f450_cam_down/model.config",
      model_config("f450_cam_down", "f450 + camera Caddx nhin xuong"))
    w(REPO / "firmware/airframes/4052_gz_f450", airframe(thr))
    w(REPO / "firmware/airframes/4053_gz_f450_cam_down", CAM_AIRFRAME)

    rows = "\n".join(f"| {p.name} | {p.m*1000:.1f} | ({p.pos[0]*1000:.0f}; {p.pos[1]*1000:.0f}; "
                     f"{p.pos[2]*1000:.0f}) | {p.src} |" for p in PARTS)
    md = f"""# Thông số drone test F450 (sinh bởi `tools/f450/gen_f450.py`)

> Không sửa tay file này. Sửa bảng `PARTS` trong script khi có số đo thật rồi chạy lại.

## Kết quả

| Đại lượng | Giá trị |
|---|---|
| Khối lượng thân (không cánh) | {m_body*1000:.1f} g |
| Khối lượng cất cánh | **{m_tot*1000:.1f} g** |
| Trọng tâm (x; y; z) | ({cg[0]*1000:.1f}; {cg[1]*1000:.1f}; {cg[2]*1000:.1f}) mm |
| Ixx / Iyy / Izz | **{J[0,0]:.5f} / {J[1,1]:.5f} / {J[2,2]:.5f} kg·m²** |
| k_T / k_M | {K_T:.4e} N/(rad/s)² / {K_M:.5f} m |
| Lực đẩy treo mỗi motor | {T_hover:.2f} N ({T_hover/G*1000:.0f} gf) |
| Tốc độ treo | {w_hover:.1f} rad/s ({w_hover*60/2/math.pi:.0f} vòng/phút) |
| `MPC_THR_HOVER` | **{thr:.2f}** |
| Lực đẩy max / trọng lượng | {tw:.2f} |

## Bảng linh kiện (vị trí trọng tâm, mm; x trước, y trái, z lên)

| Linh kiện | g | Vị trí | Nguồn |
|---|---|---|---|
{rows}

## Còn là giả thiết

- Khung: 282 g là của F450 **chính hãng** (DJI); bản clone có chân liền có thể khác. Cân lại khi có cân.
- C_T = {CT}, C_P = {CP}, 800 gf/motor, cánh 12 g, hằng thời gian motor (theo x500).
- Mọi vị trí z (pin, motor, chân đáp) là ước lượng – đo bằng thước khi lắp xong.
"""
    w(REPO / "docs/f450_params.md", md)
    print(md)


if __name__ == "__main__":
    main()
