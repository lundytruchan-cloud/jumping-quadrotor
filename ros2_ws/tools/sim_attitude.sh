#!/usr/bin/env bash
# sim_attitude.sh - 无头姿态控制验证（悬停 / 姿态阶跃 / 零推力弹道）
#
# 用法:
#   ./sim_attitude.sh                     # 默认 hover=5s step=5s zero=5s
#   ./sim_attitude.sh 8 6 5               # 自定义各阶段时长
#   ./sim_attitude.sh 8 6 5 /tmp/a.csv    # 自定义日志路径
#   ./sim_attitude.sh 5 5 5 out.csv 1.0   # 自定义初始足高（空中生成，默认 0.8 m）
#   ./sim_attitude.sh 5 5 5 out.csv 1.0 true  # 同时打开 rviz2 可视化
#
# 说明:
#   - 后台启动 quadcopter_control.launch（无 GUI）
#   - 运行 attitude_setpoint_node 驱动悬停/阶跃/零推力
#   - 结束后清理仿真并运行 task6_analyze_attitude.py 输出指标与图
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws
source install/setup.bash

to_float() {
  case "$1" in
    *.*) echo "$1" ;;
    *)   echo "$1.0" ;;
  esac
}

HOVER=$(to_float "${1:-5.0}")
STEP=$(to_float "${2:-5.0}")
ZERO=$(to_float "${3:-5.0}")
LOG=${4:-../docs/data/task6_attitude_control.csv}
DROP=$(to_float "${5:-0.8}")
RVZ=${6:-false}
SIM_LOG=/tmp/task6_attitude_sim.log

mkdir -p ../docs/data
rm -f "$LOG"

STALE=$(ps -eo cmd | grep -E "quadcopter_(sim|control)\.launch|gz sim -s -r.*quadcopter_world|attitude_control_node|attitude_setpoint_node" \
  | grep -v grep | wc -l)
if [ "$STALE" -gt 0 ]; then
  echo "[sim_attitude] 检测到已有仿真进程（$STALE 个），请先清理：./tools/sim_stop.sh"
  exit 1
fi

echo "[sim_attitude] hover=${HOVER}s step=${STEP}s zero=${ZERO}s drop_height=${DROP}m"
echo "[sim_attitude] 日志: $LOG"
ros2 launch quadcopter_description quadcopter_control.launch.py \
  gui:=false rviz:="$RVZ" drop_height:="$DROP" log_file:="$LOG" > "$SIM_LOG" 2>&1 &
LAUNCH_PID=$!

echo "[sim_attitude] 等待 IMU 桥接..."
READY=0
for i in $(seq 1 120); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -q '/quadcopter/imu'; then
    READY=1
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[sim_attitude] launch 提前退出，见 $SIM_LOG"
    tail -30 "$SIM_LOG" || true
    exit 1
  fi
  sleep 1
done
if [ "$READY" -ne 1 ]; then
  echo "[sim_attitude] 超时：未发现 /quadcopter/imu"
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi

echo "[sim_attitude] 运行姿态阶跃驱动..."
DRIVER_TIMEOUT=$(awk "BEGIN{print int($HOVER + 4 * $STEP + $ZERO + 20)}")
timeout "$DRIVER_TIMEOUT" ros2 run quadcopter_description attitude_setpoint_node --ros-args \
  -p hover_duration:="$HOVER" \
  -p step_duration:="$STEP" \
  -p zero_thrust_duration:="$ZERO" \
  -p use_sim_time:=True || true

echo "[sim_attitude] 停止仿真"
kill "$LAUNCH_PID" 2>/dev/null || true
sleep 2
bash ./tools/sim_stop.sh >/dev/null 2>&1 || true

echo "[sim_attitude] 分析数据"
python3 ../scripts/task6_analyze_attitude.py \
  --csv "$LOG" \
  --hover-duration "$HOVER" \
  --step-duration "$STEP" \
  --out-prefix ../docs/data/task6_attitude
