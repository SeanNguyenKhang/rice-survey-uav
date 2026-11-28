# Hướng dẫn cho bạn cùng làm (trên Windows)

Chào bạn! Đây là dự án đồ án **drone 4 cánh (quadrotor) khảo sát ruộng lúa**, chạy firmware PX4.
Phần cứng đang ở chỗ Minh, nên việc của bạn là phần **tính toán và lập trình Python**, làm hoàn toàn trên Windows.

Đọc từ trên xuống, làm đúng thứ tự. Gặp lỗi thì chụp màn hình gửi Minh.

---

## Phần A – Cài phần mềm (làm 1 lần, khoảng 30 phút)

### A1. Python 3.12
1. Tải tại <https://www.python.org/downloads/windows/> → chọn bản **Python 3.12.x** (Windows installer 64-bit).
2. Chạy file cài. **QUAN TRỌNG:** ở màn hình đầu tiên, tick ô **"Add python.exe to PATH"**, rồi bấm **Install Now**.
3. Kiểm tra: bấm phím Windows, gõ `cmd`, mở **Command Prompt**, gõ:
   ```
   python --version
   ```
   Phải in ra `Python 3.12.x`. Nếu báo "không tìm thấy" → cài lại và nhớ tick ô PATH.

### A2. VS Code (trình soạn code)
1. Tải tại <https://code.visualstudio.com/> → cài, cứ bấm Next.
2. Mở VS Code → thanh bên trái bấm biểu tượng **Extensions** (4 ô vuông) → tìm và cài:
   - **Python** (của Microsoft)
   - **Jupyter** (của Microsoft) – để chạy thử từng đoạn code và xem đồ thị.

### A3. Git (để sau này gửi code qua GitHub)
1. Tải tại <https://git-scm.com/download/win> → cài, để mặc định hết.
2. Tạo tài khoản <https://github.com> (nếu chưa có) và gửi **tên tài khoản** cho Minh để Minh mời vào repo.

> Nếu chưa muốn dùng GitHub thì vẫn làm được: nén thư mục gửi lại cho Minh (xem Phần E).

---

## Phần B – Mở dự án

1. Giải nén file zip Minh gửi vào chỗ **đường dẫn không có dấu và không có khoảng trắng**, ví dụ `C:\code\rice-survey-uav`.
   (Đường dẫn có dấu tiếng Việt hay khoảng trắng hay gây lỗi khó hiểu.)
2. Mở VS Code → **File → Open Folder** → chọn thư mục `rice-survey-uav`. Nếu hỏi "Trust the authors?" → **Yes**.
3. Mở terminal trong VS Code: menu **Terminal → New Terminal**. Nếu terminal là PowerShell thì vẫn dùng được.

### B1. Tạo môi trường Python riêng cho dự án (virtual environment)
Gõ lần lượt trong terminal VS Code:

```
python -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
pip install numpy scipy matplotlib control ruff jupyter
```

- Nếu PowerShell báo lỗi kiểu *"running scripts is disabled"* ở lệnh thứ 2, chạy lệnh này 1 lần rồi thử lại:
  ```
  Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
  ```
- Thành công khi đầu dòng terminal có chữ `(.venv)`.
- Mỗi lần mở lại VS Code, nếu không thấy `(.venv)` thì gõ lại `.venv\Scripts\activate`.
- Góc dưới phải VS Code, chọn Python interpreter là `.venv` (VS Code thường tự hỏi).

### B2. Thử chạy
```
python -c "import numpy, scipy, matplotlib, control; print('OK')"
```
In ra `OK` là xong phần cài đặt.

---

## Phần C – Hiểu dự án trong 10 phút

Đọc theo thứ tự (mở trong VS Code, bấm chuột phải → **Open Preview** cho dễ đọc):

1. `README.md` – tổng quan, chỉ tiêu thiết kế (bảng *Design targets*).
2. `docs/architecture.md` – lộ trình Phase 0–8.
3. `docs/f450_params.md` – **thông số con drone** (khối lượng, quán tính, lực đẩy). Đây là số liệu đầu vào cho việc của bạn.

Những thư mục bạn **không cần** quan tâm (chạy trên Ubuntu/phần cứng): `simulation/`, `firmware/`, `ros2_ws/`, `scripts/`.

---

## Phần D – Việc của bạn

### Việc 1 (chính): Phase 2 – Mô hình động lực học + bộ điều khiển PID tầng
Thư mục: `control_design/` (hiện mới có README, bạn viết từ đầu).

**Mục tiêu:** dựng mô hình toán của drone bằng Python, thiết kế bộ PID tầng (cascade) theo cấu trúc của PX4, chọn hệ số (gain) để đạt chỉ tiêu, rồi Minh nạp các gain đó vào mô phỏng PX4/drone thật.

**Số liệu đầu vào** (từ `docs/f450_params.md`):

| Đại lượng | Giá trị |
| --- | --- |
| Khối lượng cất cánh | 1.067 kg |
| Ixx / Iyy / Izz | 0.00998 / 0.01056 / 0.01810 kg·m² |
| k_T (hệ số lực đẩy) | 1.4207e-05 N/(rad/s)² |
| k_M (tỉ số mô-men/lực đẩy) | 0.01654 m |
| Khoảng cách tâm–motor | 0.225 m (motor ở x, y = ±0.159 m) |
| Lực đẩy max / trọng lượng | 3.0 |

Một số số này còn là **ước lượng** (ghi `[ƯỚC LƯỢNG]` trong file). Viết code sao cho đổi số dễ dàng (gom vào 1 chỗ).

**Chỉ tiêu cần đạt** (từ `README.md`):
- Sai số tư thế < 5°
- Đáp ứng bậc tư thế: vọt lố < 15 %, thời gian xác lập < 2 s
- Bám quỹ đạo: RMSE < 0.5 m ở 1–2 m/s

**Gợi ý chia nhỏ (làm lần lượt, mỗi bước xong gửi Minh xem):**
1. `control_design/params.py` – gom thông số ở bảng trên.
2. `control_design/quad_model.py` – mô hình 6 bậc tự do: vị trí, vận tốc, góc (roll/pitch/yaw), vận tốc góc. Đầu vào là tốc độ 4 motor (hoặc lực đẩy tổng + 3 mô-men).
   Kiểm tra đơn giản: cho 4 motor quay ở tốc độ treo (≈ 429 rad/s) → drone phải đứng yên.
3. `control_design/pid_cascade.py` – bộ điều khiển tầng giống PX4:
   - Vòng trong: **vận tốc góc** (rate) – PID
   - Vòng giữa: **góc** (attitude) – P
   - Vòng ngoài: **vận tốc** (PID) và **vị trí** (P)
   Tham khảo tên tham số của PX4 để Minh nạp được ngay: `MC_ROLLRATE_P/I/D`, `MC_ROLL_P`, `MPC_XY_P`, `MPC_XY_VEL_P_ACC`… (tài liệu: <https://docs.px4.io/v1.17/en/config_mc/pid_tuning_guide_multicopter.html>).
4. `control_design/step_test.py` – cho bước 10° roll/pitch, vẽ đồ thị, in vọt lố và thời gian xác lập.
5. Chỉnh gain đến khi đạt chỉ tiêu → xuất một bảng gain cuối cùng.

**Một vấn đề thật đang cần giải:** trong mô phỏng, sai số tư thế đạt 7.4° (> 5°) chỉ ở chỗ drone **rẽ cuối mỗi đường bay** (lệnh vị trí nhảy bậc). Nếu mô hình của bạn tái hiện được và đề xuất được cách giảm (giảm gain, giới hạn gia tốc/jerk, làm trơn quỹ đạo) thì rất có giá trị cho báo cáo.

### Việc 2 (phụ, nếu còn thời gian): mô hình thời gian bay
File: `analysis/endurance.py`. Ước lượng thời gian bay từ pin 3S 4000 mAh, khối lượng 1.067 kg, lực đẩy treo ~267 gf/motor (motor 2212 1000KV, cánh 1045). Chỉ tiêu 12–15 phút. Ghi rõ nguồn của mọi số (datasheet motor, phép tính).

---

## Phần E – Quy tắc chung

1. **Mọi con số phải có nguồn hoặc phép tính.** Số tự đoán thì ghi chú `[ƯỚC LƯỢNG]`.
2. **Không sửa** các file sinh tự động: `docs/f450_params.md`, `simulation/models/f450*`, `firmware/airframes/*`. Nếu thấy thông số sai, báo Minh.
3. Trước khi gửi code, chạy kiểm tra định dạng (phải không báo lỗi):
   ```
   ruff check .
   ```
4. Viết chú thích trong code bằng tiếng Việt hoặc tiếng Anh đều được, nhưng rõ ràng.

### Gửi bài lại cho Minh
- **Cách 1 (đơn giản):** nén thư mục `control_design/` (và `analysis/` nếu có sửa) thành zip, gửi Minh. **Không** nén thư mục `.venv` (rất nặng, không cần).
- **Cách 2 (sau khi được mời vào GitHub):** Minh sẽ hướng dẫn thêm cách `git clone` và tạo pull request.

Chúc bạn làm vui!
