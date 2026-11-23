# Thông số drone test F450 (sinh bởi `tools/f450/gen_f450.py`)

> Không sửa tay file này. Sửa bảng `PARTS` trong script khi có số đo thật rồi chạy lại.

## Kết quả

| Đại lượng | Giá trị |
|---|---|
| Khối lượng thân (không cánh) | 1019.0 g |
| Khối lượng cất cánh | **1067.0 g** |
| Trọng tâm (x; y; z) | (-1.6; 0.0; -6.4) mm |
| Ixx / Iyy / Izz | **0.00998 / 0.01056 / 0.01810 kg·m²** |
| k_T / k_M | 1.4207e-05 N/(rad/s)² / 0.01654 m |
| Lực đẩy treo mỗi motor | 2.62 N (267 gf) |
| Tốc độ treo | 429.1 rad/s (4098 vòng/phút) |
| `MPC_THR_HOVER` | **0.50** |
| Lực đẩy max / trọng lượng | 3.00 |

## Bảng linh kiện (vị trí trọng tâm, mm; x trước, y trái, z lên)

| Linh kiện | g | Vị trí | Nguồn |
|---|---|---|---|
| tay_F450_0 | 50.0 | (92; -92; -10) | F450 282 g tổng – chia: 4×50 g tay+chân, 82 g 2 tấm [ƯỚC LƯỢNG] |
| tay_F450_1 | 50.0 | (-92; 92; -10) | F450 282 g tổng – chia: 4×50 g tay+chân, 82 g 2 tấm [ƯỚC LƯỢNG] |
| tay_F450_2 | 50.0 | (92; 92; -10) | F450 282 g tổng – chia: 4×50 g tay+chân, 82 g 2 tấm [ƯỚC LƯỢNG] |
| tay_F450_3 | 50.0 | (-92; -92; -10) | F450 282 g tổng – chia: 4×50 g tay+chân, 82 g 2 tấm [ƯỚC LƯỢNG] |
| 2 tấm giữa F450 (PDB liền) | 82.0 | (0; 0; 0) | phần còn lại của 282 g [ƯỚC LƯỢNG] |
| motor_2212_0 | 66.0 | (159; -159; 26) | 60 g motor + 4 g adapter + 2 g đai ốc (bảng v21) |
| ESC_30A_0 | 9.0 | (75; -75; -12) | 9 g (bảng v21), dán dưới tay |
| motor_2212_1 | 66.0 | (-159; 159; 26) | 60 g motor + 4 g adapter + 2 g đai ốc (bảng v21) |
| ESC_30A_1 | 9.0 | (-75; 75; -12) | 9 g (bảng v21), dán dưới tay |
| motor_2212_2 | 66.0 | (159; 159; 26) | 60 g motor + 4 g adapter + 2 g đai ốc (bảng v21) |
| ESC_30A_2 | 9.0 | (75; 75; -12) | 9 g (bảng v21), dán dưới tay |
| motor_2212_3 | 66.0 | (-159; -159; 26) | 60 g motor + 4 g adapter + 2 g đai ốc (bảng v21) |
| ESC_30A_3 | 9.0 | (-75; -75; -12) | 9 g (bảng v21), dán dưới tay |
| Pin 3S 4000 mAh + dây + XT60 | 318.0 | (0; 0; -45) | bảng v21; treo DƯỚI tấm dưới [vị trí ƯỚC LƯỢNG] |
| Dây đai pin | 12.0 | (0; 0; -45) | bảng v21 |
| Kakute H7 Mini + receiver RC | 10.0 | (0; 0; 45) | 5.7 g FC (v21) + ~4 g receiver [ƯỚC LƯỢNG] |
| ESP32 + tấm mang | 13.0 | (-45; 0; 40) | bảng v21 |
| GPS NEO-M8N + la bàn + cột | 25.0 | (-40; 0; 110) | 10+1.5 g (v21) + cột ~13 g [ƯỚC LƯỢNG] |
| Camera Ant Nano + giá | 6.0 | (80; 0; -20) | [ƯỚC LƯỢNG] |
| VTX Unify Pro32 Nano + anten | 8.0 | (-60; 0; 35) | [ƯỚC LƯỢNG] |
| Dây, vít, cọc, băng dính | 45.0 | (0; 0; 0) | [ƯỚC LƯỢNG] |

## Còn là giả thiết

- Khung: 282 g là của F450 **chính hãng** (DJI); bản clone có chân liền có thể khác. Cân lại khi có cân.
- C_T = 0.11, C_P = 0.045, 800 gf/motor, cánh 12 g, hằng thời gian motor (theo x500).
- Mọi vị trí z (pin, motor, chân đáp) là ước lượng – đo bằng thước khi lắp xong.
