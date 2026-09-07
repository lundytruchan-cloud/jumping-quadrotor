#!/usr/bin/env bash
# sim_drop.sh - 自由落体掉落仿真（被动弹簧-阻尼腿）+ 阶段记录
#
# 用法:
#   ./sim_drop.sh                          # drop_height=0.50m, 记录 5s
#   ./sim_drop.sh 0.5 5 ../docs/data/task5_drop_leg_state.csv
#
# 说明:
#   - 无头运行 Gazebo（gui:=false rviz:=false），按 drop_height 生成模型
#   - 等 /quadcopter/leg_state 出现后运行 leg_phase_node 记录 CSV
#   - 结束后自动清理仿真进程
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws
source install/setup.bash

# ROS2 参数要求浮点型，统一补 .0
to_float() {
  case "$1" in
    *.*) echo "$1" ;;
    *)   echo "$1.0" ;;
  esac
}

DROP_HEIGHT=$(to_float "${1:-0.5}")
DURATION=$(to_float "${2:-5.0}")
LOG=${3:-../docs/data/task5_drop_leg_state.csv}
SIM_LOG=/tmp/task5_drop_sim.log

mkdir -p ../docs/data
rm -f "$LOG"

STALE=$(ps -eo cmd | grep -E "quadcopter_sim\.launch|gz sim -s -r.*quadcopter_world|leg_phase_node" \
  | grep -v grep | wc -l)
if [ "$STALE" -gt 0 ]; then
  echo "[sim_drop] 检测到已有仿真进程（$STALE 个），请先清理：./tools/sim_stop.sh"
  exit 1
fi

echo "[sim_drop] drop_height=$DROP_HEIGHT m, duration=$DURATION s"
echo "[sim_drop] 日志: $LOG"
ros2 launch quadcopter_description quadcopter_sim.launch.py \
  gui:=false rviz:=false drop_height:="$DROP_HEIGHT" > "$SIM_LOG" 2>&1 &
LAUNCH_PID=$!

echo "[sim_drop] 等待模型生成与 leg_state 桥接..."
READY=0
for i in $(seq 1 90); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -q '/quadcopter/leg_state'; then
    READY=1
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[sim_drop] launch 提前退出，见 $SIM_LOG"
    exit 1
  fi
  sleep 1
done
if [ "$READY" -ne 1 ]; then
  echo "[sim_drop] 超时：未发现 /quadcopter/leg_state"
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi

echo "[sim_drop] 开始记录（${DURATION}s 仿真时间）..."
ros2 run quadcopter_description leg_phase_node --ros-args \
  -p log_file:="$LOG" \
  -p duration:="$DURATION" \
  -p use_sim_time:=True || true

echo "[sim_drop] 停止仿真"
kill "$LAUNCH_PID" 2>/dev/null || true
sleep 2
bash ./tools/sim_stop.sh >/dev/null 2>&1 || true

echo "[sim_drop] 完成，CSV 行数: $(wc -l < "$LOG")"
