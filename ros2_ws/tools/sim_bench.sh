#!/usr/bin/env bash
# sim_bench.sh - 任务九性能基准仿真（图4A 周期-高度曲线 + 续航占空比）
#
# 用法:
#   ./sim_bench.sh "0.6 0.8 1.0 1.2 1.4 1.63" 8
#   ./sim_bench.sh "0.6" 6
#
# 说明:
#   - 每个高度跑 spot 原地连续跳 HOPS 次（position_feedback=true,
#     control_strategy=combined，与任务七/八同一套控制器）
#   - 最后跑一次 8 s 悬停作为功率基准（占空比分母）
#   - 产物: docs/data/task9_h{height}_{jump,attitude}.csv + task9_hover.csv
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws
source install/setup.bash

HEIGHTS=${1:-"0.6 0.8 1.0 1.2 1.4 1.63"}
HOPS=${2:-8}
OUT=${3:-../docs/data}

mkdir -p "$OUT"

STALE=$(ps -eo cmd | grep -E "quadcopter_(sim|control|jump)\.launch|gz sim -s -r.*quadcopter_world|jump_control_node|attitude_control_node|leg_phase_node" \
  | grep -v grep | wc -l)
if [ "$STALE" -gt 0 ]; then
  echo "[sim_bench] 检测到已有仿真进程（${STALE} 个），请先清理：./tools/sim_stop.sh"
  exit 1
fi

for H in $HEIGHTS; do
  PREFIX="${OUT}/task9_h${H}"
  CSV="${PREFIX}_jump.csv"
  ATT="${PREFIX}_attitude.csv"
  LEG="${PREFIX}_leg.csv"
  SIM_LOG=/tmp/task9_h${H}_sim.log
  rm -f "$CSV" "$ATT" "$LEG"

  echo "[sim_bench] 高度 ${H} m：${HOPS} 跳（仿真时间约 ${HOPS} x 周期）"
  ros2 launch quadcopter_description quadcopter_jump.launch.py \
    gui:=false rviz:=false drop_height:=0.8 desired_height:="$H" \
    trajectory:=spot num_hops:="$HOPS" \
    log_file:="$CSV" attitude_log:="$ATT" leg_log:="$LEG" \
    > "$SIM_LOG" 2>&1 &
  LAUNCH_PID=$!

  READY=0
  for i in $(seq 1 90); do
    if timeout 2 ros2 topic list 2>/dev/null | grep -q '/quadcopter/leg_state'; then
      READY=1
      break
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "[sim_bench] launch 提前退出，查看 $SIM_LOG"
      tail -40 "$SIM_LOG" || true
      exit 1
    fi
    sleep 1
  done
  if [ "$READY" -ne 1 ]; then
    echo "[sim_bench] 超时：未发现 /quadcopter/leg_state"
    kill "$LAUNCH_PID" 2>/dev/null || true
    exit 1
  fi

  echo "[sim_bench] 等待 ${HOPS} 次 takeoff（最多 300 s 墙钟）..."
  DONE=0
  for i in $(seq 1 300); do
    if [ -f "$CSV" ]; then
      CNT=$(grep -c '^cycle,' "$CSV" || true)
      if [ "$CNT" -ge "$HOPS" ]; then
        DONE=1
        break
      fi
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "[sim_bench] launch 提前退出，查看 $SIM_LOG"
      tail -40 "$SIM_LOG" || true
      exit 1
    fi
    sleep 1
  done
  if [ "$DONE" -ne 1 ]; then
    echo "[sim_bench] 超时：高度 ${H} 未完成 ${HOPS} 跳"
    kill "$LAUNCH_PID" 2>/dev/null || true
    bash ./tools/sim_stop.sh >/dev/null 2>&1 || true
    exit 1
  fi

  echo "[sim_bench] 停止仿真"
  kill "$LAUNCH_PID" 2>/dev/null || true
  sleep 2
  bash ./tools/sim_stop.sh >/dev/null 2>&1 || true
  echo "[sim_bench] 高度 ${H} 完成：$(grep -c '^cycle,' "$CSV" || true) 跳"
done

# --- 悬停功率基准 ---
HOVER="${OUT}/task9_hover.csv"
rm -f "$HOVER"
echo "[sim_bench] 悬停基准（8 s sim 时间）..."
ros2 launch quadcopter_description quadcopter_control.launch.py \
  gui:=false rviz:=false log_file:="$HOVER" > /tmp/task9_hover_sim.log 2>&1 &
LAUNCH_PID=$!

READY=0
for i in $(seq 1 90); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -q '/quadcopter/imu'; then
    READY=1
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[sim_bench] hover launch 提前退出，查看 /tmp/task9_hover_sim.log"
    tail -30 /tmp/task9_hover_sim.log || true
    exit 1
  fi
  sleep 1
done
if [ "$READY" -ne 1 ]; then
  echo "[sim_bench] 超时：未发现 /quadcopter/imu"
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi

timeout 45 ros2 run quadcopter_description attitude_setpoint_node --ros-args \
  -p hover_duration:=8.0 \
  -p step_duration:=0.0 \
  -p zero_thrust_duration:=0.0 \
  -p use_sim_time:=True || true

echo "[sim_bench] 停止悬停仿真"
kill "$LAUNCH_PID" 2>/dev/null || true
sleep 2
bash ./tools/sim_stop.sh >/dev/null 2>&1 || true

echo "[sim_bench] 全部完成，产物在 $OUT"
