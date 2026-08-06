#!/usr/bin/env bash
# sim_drop_photos.sh - 无头渲染掉落仿真截图（空中 + 最大压缩两套五视图）
#
# 用法: ./sim_drop_photos.sh [compression] [mode]
#   compression: 最大压缩时的 leg_joint 位移（m），默认 -0.027
#   mode: 输出前缀，默认 compressed；空中姿态用：./sim_drop_photos.sh 0 aerial
#
# 说明:
#   - 从当前 xacro 生成 SDF，去掉弹簧插件、固定模型并设置 leg_joint
#     初始位移，用 ModelPhotoShoot 无头渲染五视图到 docs/data/task5_photos/
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws

COMPRESSION=${1:--0.027}
MODE=${2:-compressed}
PHOTO_DIR="../docs/data/task5_photos"
mkdir -p "$PHOTO_DIR"

X="install/quadcopter_description/share/quadcopter_description/urdf/quadcopter.urdf.xacro"
xacro "$X" -o /tmp/qc_drop_photo.urdf
gz sdf -p /tmp/qc_drop_photo.urdf > /tmp/qc_drop_photo.sdf

python3 - "$COMPRESSION" <<'PYEOF'
import re
import sys

compression = float(sys.argv[1])

with open('/tmp/qc_drop_photo.sdf', 'r') as f:
    model_sdf = f.read()

m = re.search(r'<model.*?</model>', model_sdf, re.DOTALL)
if not m:
    raise RuntimeError('model block not found')
model_block = m.group(0)

# Remove the spring plugin so the photo pose is held statically.
model_block = re.sub(
    r'<plugin[^>]*SpringDamperLegSystem.*?</plugin>',
    '',
    model_block,
    flags=re.DOTALL,
)

# Freeze the model and render the leg at the requested compression by moving
# the leg_joint pose (SDF 1.9 has no joint initial_position element).
model_block = re.sub(
    r'<model name=["\']quadcopter["\']>',
    lambda match: match.group(0) +
    '<pose>0 0 {:.6f} 0 0 0</pose><static>true</static>'
    .format(0.22 + compression),
    model_block,
    count=1,
)

def _compressed_joint_pose(match):
    z = -0.22 - compression
    return '{}{} {} {:.6f}'.format(
        match.group(1), match.group(2), match.group(3), z)


model_block = re.sub(
    r"(<joint name=['\"]leg_joint['\"][^>]*>\s*"
    r"<pose[^>]*>)\s*([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)",
    _compressed_joint_pose,
    model_block,
    count=1,
)

photo_plugin = (
    '<plugin filename="gz-sim-model-photo-shoot-system"'
    ' name="gz::sim::systems::ModelPhotoShoot">'
    '<random_joints_pose>false</random_joints_pose>'
    '</plugin>'
)
model_block = re.sub(
    r'<static>true</static>',
    lambda match: match.group(0) + photo_plugin,
    model_block,
    count=1,
)

world = '''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="photo_world">
    <gravity>0 0 0</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
      <background_color>0.85 0.9 1.0</background_color>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="photo_shoot">
      <link name="link">
        <pose>0 0 0 0 0 0</pose>
        <sensor name="camera" type="camera">
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.05</near><far>100</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>true</visualize>
          <topic>camera</topic>
        </sensor>
      </link>
      <static>true</static>
    </model>
    {model_block}
  </world>
</sdf>
'''.format(model_block=model_block)

with open('/tmp/qc_drop_photo_world.sdf', 'w') as f:
    f.write(world)
print('photo world written, compression =', compression)
PYEOF

cd "$PHOTO_DIR"
# 已知现象：gz sim 退出阶段可能 segfault，但图片在此之前已写入磁盘。
gz sim -s -r -v 3 --iterations 60 /tmp/qc_drop_photo_world.sdf > /tmp/photo_drop.log 2>&1 || true

declare -A VIEWS=([1]=perspective [2]=top [3]=front [4]=side [5]=back)
for index in 1 2 3 4 5; do
  if [ -f "$index.png" ]; then
    mv -f "$index.png" "task5_drop_${MODE}_${VIEWS[$index]}.png"
  else
    echo "[sim_drop_photos] 警告: 缺少 $index.png"
  fi
done
ls -la
