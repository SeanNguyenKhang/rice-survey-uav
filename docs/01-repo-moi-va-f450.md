# Chuyển sang repo mới và chạy mô phỏng F450

Repo này thay cho `x450_sim`. Nó gồm dàn ý theo giai đoạn (Phase 0–8), phần ruộng lúa, camera, mission
và script gió lấy từ `x450_sim`, cùng model **F450** cho con drone test.

## Bước 1 – Đẩy nốt việc cũ lên `x450_sim` trước

Repo `x450_sim` trên máy có commit chưa push và có các file chưa có trên GitHub (`analyze_flight.py`, `auto_analyze.py`,
`gen_field_env.py`, `field_env/`, `run_sitl.sh`, `snap_env_fix.sh`, `CLAUDE.md`...). Giữ lại cho an toàn:

```bash
cd ~/x450_sim
git add -A && git commit -m "Snapshot before moving to rice-survey-uav"
git push
```

Repo cũ để nguyên làm lưu trữ, không xoá.

## Bước 2 – Giải nén repo mới và mở bằng VS Code

```bash
cd ~ && unzip ~/Downloads/rice-survey-uav.zip
code ~/rice-survey-uav
```

VS Code hỏi "Install recommended extensions?" → **Install**. Hỏi "Trust the authors?" → **Yes**.

## Bước 3 – Đưa lên GitHub (bằng VS Code)

1. Mở `LICENSE`, thay `<YOUR NAME>` bằng tên bạn.
2. Thanh bên trái → **Source Control** → **Initialize Repository**.
3. Gõ message `Initial commit: rice-survey-uav (F450 test model, rice field, survey mission)` → **Commit**.
4. **Publish Branch** → chọn **Public**.
5. Trên GitHub, tab **Actions**: workflow `CI` phải xanh.

## Bước 4 – Chuyển các file còn hữu ích từ `x450_sim`

Những file này chưa có trên GitHub nên chưa nằm trong repo mới. Chép tay vào đúng chỗ:

| File trong `x450_sim` | Chép vào |
| --- | --- |
| `tools/analyze_flight.py`, `tools/auto_analyze.py` | `analysis/` |
| `tools/gen_field_env.py`, `models/field_env/` | `tools/mission/`, `simulation/models/` |

Sau đó hỏi Claude trong VS Code: *"Sửa đường dẫn trong các file vừa chép cho khớp cấu trúc repo mới (xem CLAUDE.md)"*.

## Bước 5 – Phase 0: môi trường

Làm theo `docs/00-setup.md` (PX4 v1.17.0, QGC, Agent, ROS 2). Kiểm tra: `Ctrl+Shift+P` → **Run Task** →
`0. Kiểm tra môi trường`, sửa đến khi không còn `[FAIL]`.

## Bước 6 – Chạy F450 trên ruộng lúa

Run Task theo thứ tự:

1. `A. Sinh lại thông số + model F450` – in bảng khối lượng, quán tính, ga treo.
2. `B. Liên kết model/world/airframe vào PX4` – lần đầu nó thêm 4052/4053 vào CMake.
   Sau lần đầu: `rm -rf ~/PX4-Autopilot/build/px4_sitl_default`.
3. `1. Mô phỏng SITL` → chọn `gz_f450_cam_down`. Gazebo mở ruộng lúa, QGC tự kết nối.
4. QGC → **Plan** → **Open** → `simulation/missions/lawnmower_a6_v1.5_s11.2.plan` → **Upload** → **Fly** → **Start Mission**.
5. Thử gió: `D. Đặt gió` (ví dụ 3 m/s từ hướng 90°).

Đạt khi: drone treo ổn định ở 6 m, bay hết mission rồi RTL. Khi đó tick một phần Phase 1 trong `README.md` và commit.

## Bước 7 – Khi đã lắp F450 thật

Đo bằng thước: độ cao mặt motor, vị trí pin (trên/dưới), chiều cao chân đáp, vị trí GPS/camera. Nếu cân được thì cân
khung và cả drone. Sửa bảng `PARTS` trong `tools/f450/gen_f450.py` → chạy lại task `A` → mô phỏng sát máy thật hơn.

## Bay thật – an toàn

Trong QGC gán 3 công tắc trên tay RC: **kill switch**, **chế độ bay** (Position / Altitude / Mission), **RTL**.
Thứ tự: tháo cánh test motor → treo thấp chế độ Position → mission ngắn → mission ruộng lúa.
