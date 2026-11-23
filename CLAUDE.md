# CLAUDE.md – rice-survey-uav

Đồ án tốt nghiệp của Minh: quadrotor mang camera khảo sát mật độ lúa. Trả lời bằng **tiếng Việt**.

## Bối cảnh
- Có **2 khung**, chung toàn bộ phần điện:
  - **X450**: khung tự thiết kế cho đồ án (bản vẽ, CAD nằm ngoài repo, KHÔNG sửa ở đây).
  - **F450 clone**: khung thương mại (450 mm, chân đáp liền tay, PDB liền tấm dưới), dùng để **bay test thuật toán**. Mô phỏng = model `f450`.
- Điện: Kakute H7 Mini v1.5 (PX4), 4× ESC BLHeli_S 30A, motor 2212 1000KV, cánh 1045, pin 3S 4000 mAh,
  GPS NEO-M8N, la bàn GY-271 (QMC5883L), ESP32 (cầu MAVLink WiFi), Caddx Ant Nano + VTX TBS Unify Pro32 Nano.
- Có tay RC (kill switch, chế độ bay, RTL). Nhiệm vụ thật bay bằng mission QGC.
- Phạm vi đồ án KHÔNG có xử lý ảnh.

## Phiên bản (ghim)
PX4 v1.17.0 · ROS 2 Jazzy · Gazebo Harmonic · Micro XRCE-DDS Agent v2.4.3 · px4_msgs `release/1.17` · Ubuntu 24.04.

## Cấu trúc
- `tools/f450/gen_f450.py` – NGUỒN DUY NHẤT của thông số F450 (bảng `PARTS`). Sinh ra
  `simulation/models/f450*`, `firmware/airframes/4052_gz_f450`, `4053_gz_f450_cam_down`, `docs/f450_params.md`.
  **Không sửa tay các file sinh ra**, sửa script rồi chạy lại.
- `tools/mission/` – `gen_rice_field.py` (ruộng lúa → `simulation/models/rice_plot`, git-ignore),
  `gen_lawnmower.py` (mission zig-zag → `simulation/missions/*.plan`), `field_site.py` (toạ độ ruộng, home).
- `scripts/setup_px4.sh` – symlink model/world/airframe vào `~/PX4-Autopilot`. `scripts/run_sitl.sh` – chạy tất cả.
- `docs/` – hướng dẫn theo giai đoạn (00-setup, 01-...), `architecture.md` (lộ trình Phase 0–8), `wiring.md`.

## Quy tắc làm việc
- Mọi con số phải có nguồn hoặc phép tính; số chưa kiểm chứng ghi rõ `[ƯỚC LƯỢNG]` / giả thiết. Không bịa lý do kỹ thuật.
- Giả thiết hiện còn: khung 282 g (của F450 chính hãng), C_T=0.11, C_P=0.045, 800 gf/motor, cánh 12 g,
  hằng thời gian motor + cản rotor theo x500, mọi vị trí z trong `PARTS`.
- Mã airframe: 4052/4053 (không dùng 4050/4051 – trùng repo cũ x450_sim).
- Trước khi commit: `ruff check .` sạch; `python3 tools/f450/gen_f450.py` chạy được.
- Giải thích ngắn gọn, từng bước, cho người mới (Minh mạnh embedded C, mới với ROS 2/Gazebo).

## Đang ở đâu
Xem mục Progress trong `README.md` và `docs/architecture.md` §6. Việc kế tiếp: hoàn thành Phase 0 (`docs/00-setup.md`),
rồi kiểm chứng Phase 1 (F450 treo ổn định 6 m, bay hết mission ruộng lúa).
