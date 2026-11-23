#!/usr/bin/env bash
# Liên kết model, world, airframe của repo này vào cây PX4-Autopilot (symlink, không copy).
# Chạy lại mỗi khi thêm model/airframe mới.  REGEN=1 ./scripts/setup_px4.sh  -> sinh lại ruộng lúa.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
GZ="$PX4/Tools/simulation/gz"
AF="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
[[ -d "$GZ/models" ]] || { echo "Không thấy PX4 ở $PX4 (đặt biến PX4_DIR)"; exit 1; }

link() { ln -sfn "$1" "$2"; echo "  $(basename "$2") -> $1"; }

echo "Sinh model F450 (tools/f450/gen_f450.py):"
python3 "$REPO/tools/f450/gen_f450.py" > /dev/null && echo "  ok"

if [[ ! -f "$REPO/simulation/models/rice_plot/model.sdf" || "${REGEN:-0}" == "1" ]]; then
  echo "Sinh ruộng lúa simulation/models/rice_plot:"
  python3 "$REPO/tools/mission/gen_rice_field.py"
fi

echo "Models:";   for m in "$REPO"/simulation/models/*/; do n=$(basename "$m"); link "$REPO/simulation/models/$n" "$GZ/models/$n"; done
echo "Worlds:";   for w in "$REPO"/simulation/worlds/*.sdf; do link "$w" "$GZ/worlds/$(basename "$w")"; done
echo "Airframes:"
for a in "$REPO"/firmware/airframes/*; do
  n=$(basename "$a"); [[ "$n" == .gitkeep ]] && continue
  link "$a" "$AF/$n"
  if ! grep -qE "^[[:space:]]*$n$" "$AF/CMakeLists.txt"; then
    sed -i "/^[[:space:]]*4001_gz_x500$/a\\\t$n" "$AF/CMakeLists.txt"
    echo "  + thêm $n vào CMakeLists.txt (lần đầu: xoá build/px4_sitl_default rồi build lại)"
  fi
done
echo "Xong."
