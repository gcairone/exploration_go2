#!/bin/bash
# Run a list of exploration jobs with N parallel simulations (real-time factor stays ~1 with N <= cores - 1).
#
#   sim/scripts/run_batch.sh <jobs.csv> <out_dir> [n_workers=3]
#
# jobs.csv lines (no header): name,world,x,y,yaw,metric,config,timeout   (see make_jobs.py)
# Each job writes <out_dir>/<name>/; jobs whose directory already contains done.txt are skipped,
# so an interrupted batch can simply be restarted. FRAMES=1 records frames.npz for make_gif.py.
JOBS=$1; OUT=$2; N=${3:-3}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$OUT/.claims"

worker() {
  local slot=$1
  while IFS=, read -r name world x y yaw metric config timeout; do
    [ -z "$name" ] && continue
    [ -f "$OUT/$name/done.txt" ] && continue
    mkdir "$OUT/.claims/$name" 2>/dev/null || continue
    bash "$HERE/run_exploration.sh" --world "$world" --pose "$x,$y,$yaw" --metric "$metric" --config "$config" \
      --timeout "$timeout" --slot "$slot" --out "$OUT/$name" ${FRAMES:+--frames} > "$OUT/$name.out" 2>&1 < /dev/null
    echo "[slot $slot] $name: $(tail -n 1 "$OUT/$name.out")"
  done < "$JOBS"
}

rm -rf "$OUT/.claims"/*
for k in $(seq 1 "$N"); do worker "$k" & sleep 15; done
wait
