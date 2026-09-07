#!/usr/bin/env bash
# sim_photos.sh - 无头渲染四旋翼模型五视图到 docs/data/task3_photos/
#
# 用法: ./sim_photos.sh
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws

PHOTO_DIR="../docs/data/task3_photos"
mkdir -p "$PHOTO_DIR"

X="install/quadcopter_description/share/quadcopter_description/urdf/quadcopter.urdf.xacro"
xacro "$X" -o /tmp/qc_photo.urdf
gz sdf -p /tmp/qc_photo.urdf > /tmp/qc_photo.sdf

python3 - <<'PYEOF'
import re

with open('/tmp/qc_photo.sdf', 'r') as f:
    model_sdf = f.read()

m = re.search(r'<model.*?</model>', model_sdf, re.DOTALL)
if not m:
    raise RuntimeError('model block not found')
model_block = m.group(0)

photo_plugin = (
    '<plugin filename="gz-sim-model-photo-shoot-system"'
    ' name="gz::sim::systems::ModelPhotoShoot">'
    '<random_joints_pose>false</random_joints_pose>'
    '</plugin>'
)
model_block = re.sub(
    r'<model name=["\']quadcopter["\']>',
    lambda match: match.group(0) + photo_plugin,
    model_block,
    count=1,
)

world = f'''<?xml version="1.0"?>
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
    {model_block}
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
  </world>
</sdf>
'''

with open('/tmp/qc_photo_world.sdf', 'w') as f:
    f.write(world)
print('photo world written')
PYEOF

cd "$PHOTO_DIR"
# 说明：gz sim 在退出阶段可能报 segfault，但五张图在此之前已写入磁盘，
# 属渲染线程收尾的已知现象，不影响产物。
gz sim -s -r -v 3 --iterations 60 /tmp/qc_photo_world.sdf > /tmp/photo_shoot.log 2>&1 || true

# 插件默认按 1.png..5.png 输出，顺序与视图的对应关系固定：
# 1=perspective 2=top 3=front 4=side 5=back
declare -A VIEWS=([1]=perspective [2]=top [3]=front [4]=side [5]=back)
for index in 1 2 3 4 5; do
  if [ -f "$index.png" ]; then
    mv -f "$index.png" "task3_gazebo_${VIEWS[$index]}.png"
  else
    echo "[sim_photos] 警告: 缺少 $index.png，渲染可能不完整"
  fi
done
ls -la
