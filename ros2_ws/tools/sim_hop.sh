#!/usr/bin/env bash
# sim_hop.sh - 高层跳跃控制器仿真验证（原地 / 圆 / 阶跃轨迹）
#
# 用法:
#   ./sim_hop.sh spot 12                    # 原地连续 12 跳
#   ./sim_hop.sh circle 14 0.6 0.8          # 圆轨迹 14 跳（半径 0.5 m）
#   ./sim_hop.sh step 14 0.6 0.8            # 阶跃轨迹 14 跳（0 <-> 0.6 m）
#   ./sim_hop.sh spot 4 0.6 0.8 "" false "fixed_tilt_deg:=8.0"  # 附加参数
#
# 说明:
#   - 无头运行 quadcopter_jump.launch（gui:=false rviz:=false）
#   - 初始 drop_height 预载弹簧腿后由控制器接管连续跳跃
#   - 等待 CSV 中 takeoff 行数达到 num_hops 后自动清理并分析
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

TRAJ=${1:-spot}
HOPS=${2:-12}
Z_D=$(to_float "${3:-0.6}")
DROP=$(to_float "${4:-0.8}")
PREFIX=${5:-../docs/data/task7_${TRAJ}}
RVZ=${6:-false}
EXTRA_ARGS=${7:-}
SIM_LOG=/tmp/task7_${TRAJ}_sim.log
CSV=${PREFIX}_jump.csv
ATT_LOG=${PREFIX}_attitude.csv
LEG_LOG=${PREFIX}_leg.csv

mkdir -p ../docs/data
rm -f "$CSV" "$ATT_LOG" "$LEG_LOG"

STALE=$(ps -eo cmd | grep -E "quadcopter_(sim|control|jump)\.launch|gz sim -s -r.*quadcopter_world|jump_control_node|attitude_control_node|leg_phase_node" \
  | grep -v grep | wc -l)
if [ "$STALE" -gt 0 ]; then
  echo "[sim_hop] 检测到已有仿真进程（${STALE} 个），请先清理：./tools/sim_stop.sh"
  exit 1
fi

EXTRA=""
if [ "$TRAJ" = "circle" ]; then
  EXTRA="circle_radius:=0.4 circle_omega:=0.3 circle_phase0:=0.0"
elif [ "$TRAJ" = "step" ]; then
  EXTRA="step_targets:=0,0;0.6,0 step_hold:=2.0"
fi

echo "[sim_hop] trajectory=$TRAJ hops=$HOPS z_d=${Z_D}m drop=${DROP}m"
echo "[sim_hop] 日志: $CSV"
ros2 launch quadcopter_description quadcopter_jump.launch.py \
  gui:=false rviz:="$RVZ" drop_height:="$DROP" desired_height:="$Z_D" \
  trajectory:="$TRAJ" num_hops:="$HOPS" \
  log_file:="$CSV" attitude_log:="$ATT_LOG" leg_log:="$LEG_LOG" \
  $EXTRA $EXTRA_ARGS > "$SIM_LOG" 2>&1 &
LAUNCH_PID=$!

echo "[sim_hop] 等待模型生成与 leg_state 桥接..."
READY=0
for i in $(seq 1 90); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -q '/quadcopter/leg_state'; then
    READY=1
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[sim_hop] launch 提前退出，查看 $SIM_LOG"
    tail -40 "$SIM_LOG" || true
    exit 1
  fi
  sleep 1
done
if [ "$READY" -ne 1 ]; then
  echo "[sim_hop] 超时：未发现 /quadcopter/leg_state"
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi

echo "[sim_hop] 等待 ${HOPS} 次 takeoff（仿真时间，最多 240 s 墙钟）..."
DONE=0
for i in $(seq 1 240); do
  if [ -f "$CSV" ]; then
    CNT=$(grep -c '^cycle,' "$CSV" || true)
    if [ "$CNT" -ge "$HOPS" ]; then
      DONE=1
      echo "[sim_hop] 已达到 ${CNT} 次 takeoff"
      break
    fi
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[sim_hop] launch 提前退出，查看 $SIM_LOG"
    tail -40 "$SIM_LOG" || true
    exit 1
  fi
  sleep 1
done
if [ "$DONE" -ne 1 ]; then
  echo "[sim_hop] 超时：${HOPS} 跳未完成"
  kill "$LAUNCH_PID" 2>/dev/null || true
  bash ./tools/sim_stop.sh >/dev/null 2>&1 || true
  tail -30 "$SIM_LOG" || true
  exit 1
fi

echo "[sim_hop] 停止仿真"
kill "$LAUNCH_PID" 2>/dev/null || true
sleep 2
bash ./tools/sim_stop.sh >/dev/null 2>&1 || true

echo "[sim_hop] 分析数据"
ANALYZE_EXTRA=""
if [ "$TRAJ" = "circle" ]; then
  ANALYZE_EXTRA="--radius 0.4 --omega 0.3"
elif [ "$TRAJ" = "step" ]; then
  ANALYZE_EXTRA="--targets 0,0;0.6,0 --hold 2.0"
fi
python3 ../scripts/task7_analyze_hop.py \
  --csv "$CSV" \
  --attitude-csv "$ATT_LOG" \
  --trajectory "$TRAJ" \
  --desired-height "$Z_D" \
  --num-hops "$HOPS" \
  --out-prefix "$PREFIX" \
  $ANALYZE_EXTRA
