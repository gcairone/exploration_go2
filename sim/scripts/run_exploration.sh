#!/bin/bash
# One headless exploration run: Gazebo (gzserver) + slam_toolbox + Nav2 + explorer_pkg + monitor.
#
#   sim/scripts/run_exploration.sh [options]
#     --world NAME      tb3_world | square | corridor | corridor_taper | four_rooms | three_rooms  (tb3_world)
#     --pose X,Y,YAW    initial robot pose in the world frame                                      (0,0,0)
#     --metric M        euclidean | pseudo_info_gain | dim_cluster                                 (euclidean)
#     --config C        longrange: 12 m lidar + share/mapper_params_long_range.yaml                (longrange)
#                       original:  stock 3.5 m lidar + share/mapper_params_online_async.yaml
#     --timeout S       maximum simulated exploration time [s]                                     (600)
#     --out DIR         output directory (logs, metrics.csv, done.txt, final_map.npz)              (/tmp/exploration_run)
#     --slot N          isolates parallel runs (ROS_DOMAIN_ID 40+N, Gazebo port 11345+N)           (1)
#     --frames          also record frames.npz for make_gif.py
#     --gui             start gzclient too (needs a display)
# Environment overrides: SLAM_PARAMS, NAV2_PARAMS, MODEL_SDF.
# The run ends when the monitor reports >= 99% coverage of the world interior, after 150 s
# without map growth, or at --timeout.
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
SIM=$REPO/sim
WORLD=tb3_world; POSE=0,0,0; METRIC=euclidean; CONFIG=longrange; TIMEOUT=600; OUT=/tmp/exploration_run
SLOT=1; FRAMES=""; GUI=false
while [ $# -gt 0 ]; do
  case $1 in
    --world) WORLD=$2; shift ;; --pose) POSE=$2; shift ;; --metric) METRIC=$2; shift ;;
    --config) CONFIG=$2; shift ;; --timeout) TIMEOUT=$2; shift ;; --out) OUT=$2; shift ;;
    --slot) SLOT=$2; shift ;; --frames) FRAMES=--frames ;; --gui) GUI=true ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac; shift
done
IFS=, read -r X0 Y0 YAW0 <<< "$POSE"

case $CONFIG in
  longrange) SLAM_PARAMS=${SLAM_PARAMS:-$REPO/share/mapper_params_long_range.yaml}
             MODEL_SDF=${MODEL_SDF:-$SIM/models/turtlebot3_burger_lr/model.sdf} ;;
  original)  SLAM_PARAMS=${SLAM_PARAMS:-$REPO/share/mapper_params_online_async.yaml}
             MODEL_SDF=${MODEL_SDF:-/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf} ;;
  *) echo "unknown config $CONFIG"; exit 2 ;;
esac
NAV2_PARAMS=${NAV2_PARAMS:-$REPO/share/nav2_params_humble.yaml}
EXPLORER_PARAMS=$REPO/ros2_ws/src/explorer_pkg/config/explorator_params.yaml
if [ "$WORLD" = tb3_world ]; then WORLD_FILE=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
else WORLD_FILE=$SIM/worlds/$WORLD.world; fi

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source $REPO/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_MASTER_URI=http://127.0.0.1:$((11345 + SLOT))
export ROS_DOMAIN_ID=$((40 + SLOT))
export ROS_LOCALHOST_ONLY=1
export RCUTILS_COLORIZED_OUTPUT=0

PIDS=()
start() { local name=$1; shift; setsid "$@" > "$OUT/$name.log" 2>&1 & PIDS+=($!); }
stop_all() {
  for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 5
  for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done
  PIDS=()
}
trap stop_all EXIT
wait_topic() { local t=$2; while [ $t -gt 0 ]; do ros2 topic list 2>/dev/null | grep -qx "$1" && return 0; sleep 2; t=$((t - 2)); done; return 1; }

bringup() {
  start gazebo ros2 launch $SIM/launch/tb3_sim.launch.py world:=$WORLD_FILE model:=$MODEL_SDF \
    x_pose:=$X0 y_pose:=$Y0 yaw:=$YAW0 gui:=$GUI
  wait_topic /scan 180 || { echo "no /scan"; return 1; }
  start slam ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=$SLAM_PARAMS
  wait_topic /map 90 || { echo "no /map"; return 1; }
  start nav2 ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=$NAV2_PARAMS
  local t=150
  until grep -q "Managed nodes are active" "$OUT/nav2.log" || [ $t -le 0 ]; do sleep 3; t=$((t - 3)); done
  grep -q "Managed nodes are active" "$OUT/nav2.log" || { echo "Nav2 lifecycle activation timed out"; return 1; }
}

case $OUT in ""|/) echo "bad --out"; exit 2 ;; esac
mkdir -p "$OUT"; rm -f "$OUT"/*
echo "world=$WORLD pose=$POSE metric=$METRIC config=$CONFIG timeout=$TIMEOUT" > "$OUT/run.txt"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
for attempt in 1 2; do   # Nav2 occasionally fails to activate at startup: retry once from scratch
  bringup && break
  echo "bringup failed (attempt $attempt)" >> "$OUT/run.txt"; stop_all
  [ $attempt = 2 ] && exit 1
done

start monitor python3 $SIM/scripts/monitor.py --out "$OUT" --world $WORLD $FRAMES
MON=${PIDS[-1]}
start frontier_scanner ros2 run explorer_pkg frontier_scanner --ros-args -r __node:=frontier_scanner \
  --params-file $EXPLORER_PARAMS -p reduction:=dbscan -p metric:=$METRIC
start goal_manager ros2 run explorer_pkg goal_manager --ros-args -r __node:=goal_manager --params-file $EXPLORER_PARAMS

t0=$(date +%s)
while [ ! -f "$OUT/done.txt" ]; do
  sleep 2
  sim=$(tail -n 1 "$OUT/metrics.csv" 2>/dev/null | cut -d, -f1)
  if [ -n "$sim" ] && [ "${sim%.*}" -ge "$TIMEOUT" ] 2>/dev/null; then echo "timeout $sim" > "$OUT/done.txt"; fi
  [ $(( $(date +%s) - t0 )) -gt $((TIMEOUT * 3)) ] && echo "walltime $sim" > "$OUT/done.txt"
done
kill -INT -- -$MON 2>/dev/null; sleep 3
echo "done: $(cat "$OUT/done.txt")"
