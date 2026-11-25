#!/usr/bin/env bash
# Ghi tình trạng máy mỗi 2 giây vào ~/monitor.log (kèm sync) để nếu máy đứng thì
# dòng cuối của file cho biết lúc đó CPU / RAM / nhiệt độ / GPU ra sao.
# Cách dùng: mở 1 terminal riêng, chạy ./scripts/monitor.sh, rồi chạy SITL ở terminal khác.
#            Dừng bằng Ctrl+C.
OUT="${1:-$HOME/monitor.log}"
echo "Ghi vào $OUT (Ctrl+C để dừng)"
while true; do
  {
    echo "=== $(date +%T) load: $(cut -d' ' -f1-3 /proc/loadavg)"
    free -m | awk 'NR==2{printf "RAM used %d MB, avail %d MB | ", $3, $7} NR==3{printf "swap used %d MB\n", $3}'
    for h in /sys/class/hwmon/hwmon*; do
      n=$(cat "$h/name" 2>/dev/null)
      case "$n" in k10temp|amdgpu|nvme|asus*) for t in "$h"/temp*_input; do [ -r "$t" ] && echo "  $n $(basename "$t" _input): $(($(cat "$t") / 1000)) C"; done ;; esac
    done
    nvidia-smi --query-gpu=temperature.gpu,utilization.gpu,power.draw,memory.used,pstate --format=csv,noheader 2>/dev/null | sed 's/^/  GPU: /'
    ps -eo pcpu,pmem,comm --sort=-pcpu | sed -n '2,4p' | sed 's/^/  top: /'
  } >> "$OUT"
  sync
  sleep 2
done
