#!/usr/bin/env bash
# sim_task8.sh - 任务八稳定器三策略仿真（无位置反馈）
#
# 用法:
#   ./sim_task8.sh attitude_only 8      # 仅姿态控制器（预期 3-5 跳发散）
#   ./sim_task8.sh stabilizer_only 12   # 仅气动稳定器（预期 6-10 跳发散）
#   ./sim_task8.sh combined 32          # 组合策略（预期 >=30 跳稳定）
#
# 说明:
#   - 固定 position_feedback:=false（控制器不使用位姿/速度反馈）
#   - desired_height=0.5 m（对应论文图 6C），drop_height=0.75 m（论文 75 cm）
#   - no_fb_dt_pa 为开环动力爬升时长；0（默认）表示按上一周期实测飞行时间
#     自适应（无位置反馈高度控制，对应论文 Δt_PA）
#   - 第 8 个参数为初始俯仰扰动（默认 3 度，用于复现论文实验的初始偏差）
#   - 第 9 个参数为下落段姿态力矩缩放（默认 1.0；组合策略调小可让稳定器
#     在着陆前更多塑造姿态，对应论文“调节控制增益与稳定器配置”）
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws
source install/setup.bash

STRATEGY=${1:-combined}
HOPS=${2:-32}
Z_D=${3:-0.5}
DROP=${4:-0.75}
NO_FB_DT_PA=${5:-0.0}
PREFIX=${6:-../docs/data/task8_${STRATEGY}}
EXTRA_ARGS=${7:-}
DISTURB_DEG=${8:-3.0}
ATT_SCALE=${9:-1.0}
case "$PREFIX" in
  /*|../*) ;;
  *) PREFIX="../$PREFIX" ;;
esac
SIM_LOG=/tmp/task8_${STRATEGY}_sim.log
CSV=${PREFIX}_jump.csv
ATT_LOG=${PREFIX}_attitude.csv
LEG_LOG=${PREFIX}_leg.csv
STAB_LOG=${PREFIX}_stabilizer.csv

mkdir -p "$(dirname "$CSV")"
rm -f "$CSV" "$ATT_LOG" "$LEG_LOG" "$STAB_LOG"

STALE=$(ps -eo cmd | grep -E "quadcopter_(sim|control|jump)\.launch|gz sim -s -r.*quadcopter_world|jump_control_node|attitude_control_node|leg_phase_node" \
  | grep -v grep | wc -l)
if [ "$STALE" -gt 0 ]; then
  echo "[sim_task8] 检测到已有仿真进程（${STALE} 个），请先清理：./tools/sim_stop.sh"
  exit 1
fi

echo "[sim_task8] strategy=$STRATEGY hops=$HOPS z_d=${Z_D}m drop=${DROP}m no_fb_dt_pa=${NO_FB_DT_PA}s"
ros2 launch quadcopter_description quadcopter_jump.launch.py \
  gui:=false rviz:=false drop_height:="$DROP" desired_height:="$Z_D" \
  trajectory:=spot num_hops:="$HOPS" control_strategy:="$STRATEGY" \
  position_feedback:=false no_fb_dt_pa:="$NO_FB_DT_PA" \
  spawn_pitch_deg:="$DISTURB_DEG" \
  att_only_torque_scale:="$ATT_SCALE" \
  log_file:="$CSV" attitude_log:="$ATT_LOG" leg_log:="$LEG_LOG" \
  $EXTRA_ARGS > "$SIM_LOG" 2>&1 &
LAUNCH_PID=$!

echo "[sim_task8] 等待模型生成与 leg_state 桥接..."
READY=0
for i in $(seq 1 90); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -q '/quadcopter/leg_state'; then
    READY=1
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[sim_task8] launch 提前退出，查看 $SIM_LOG"
    tail -40 "$SIM_LOG" || true
    exit 1
  fi
  sleep 1
done
if [ "$READY" -ne 1 ]; then
  echo "[sim_task8] 超时：未发现 /quadcopter/leg_state"
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi

timeout 600 ros2 topic echo --field data /quadcopter/stabilizer_state \
  > "$STAB_LOG" 2>/dev/null &
STAB_PID=$!

echo "[sim_task8] 等待 ${HOPS} 次 takeoff（含崩溃停滞检测）..."
DONE=0
PREV_CYCLES=0
PREV_LANDINGS=0
STALL=0
for i in $(seq 1 180); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[sim_task8] launch 已退出（正常完成或提前结束）"
    break
  fi
  CYCLES=0
  LANDINGS=0
  if [ -f "$CSV" ]; then
    CYCLES=$(grep -c '^cycle,' "$CSV" || true)
    LANDINGS=$(grep -c '^landing,' "$CSV" || true)
  fi
  if [ "$CYCLES" -ge "$HOPS" ]; then
    # 目标跳数已到：给节点 30 s 完成收尾退出
    for j in $(seq 1 15); do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        break
      fi
      sleep 2
    done
    break
  fi
  if [ "$CYCLES" -eq "$PREV_CYCLES" ] && [ "$LANDINGS" -eq "$PREV_LANDINGS" ]; then
    STALL=$((STALL + 1))
    if [ "$STALL" -ge 20 ] && [ "$CYCLES" -gt 0 ]; then
      echo "[sim_task8] 停滞 ${STALL} 个轮询周期：判定崩溃，提前停止"
      break
    fi
  else
    STALL=0
  fi
  PREV_CYCLES=$CYCLES
  PREV_LANDINGS=$LANDINGS
  sleep 2
done

echo "[sim_task8] 停止仿真"
kill "$LAUNCH_PID" 2>/dev/null || true
kill "$STAB_PID" 2>/dev/null || true
sleep 2
bash ./tools/sim_stop.sh >/dev/null 2>&1 || true

echo "[sim_task8] 分析数据"
python3 ../scripts/task8_analyze_sim.py \
  --jump-csv "$CSV" \
  --attitude-csv "$ATT_LOG" \
  --strategy "$STRATEGY" \
  --num-hops "$HOPS" \
  --out-prefix "$PREFIX"
