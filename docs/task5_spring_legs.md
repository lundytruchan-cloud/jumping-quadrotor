# 任务五：被动弹簧-阻尼腿 + 混合阶段切换 + 掉落仿真验证

日期：2026-08-07

## 结论

验收通过：自由落体掉落仿真中，**支撑时间 31.0 ms**（论文约 32 ms，±20% 带为
25.6–38.4 ms），**腿收缩（弹性元件总拉伸）4.81 cm**（论文 4.7–5.5 cm，±20% 带为
3.76–6.60 cm）。两项指标均落在论文区间内，且与任务四的解析模型预测
（32.2 ms / 4.70 cm，落地速度 3.13 m/s）偏差仅约 3%。

| 指标 | 论文目标 | 本仿真 | 模型预测（任务四模块） | 结论 |
| --- | --- | --- | --- | --- |
| 支撑时间 | 约 32 ms（±20%） | 31.0 ms | 32.2 ms | PASS |
| 腿收缩（l₀+l_p−l_min） | 4.7–5.5 cm（±20%） | 4.81 cm | 4.70 cm | PASS |
| 几何压缩（l₀−l_min） | — | 2.84 cm | 2.73 cm | — |
| 落地速度 | — | 3.13 m/s | 理论 3.13 m/s | 一致 |

## 实现

### 1. C++ gz-sim 系统插件（`ros2_ws/src/quadcopter_gz_plugins/`）

Gazebo Sim 无原生弹簧关节，因此自写 gz-sim 系统插件
`SpringDamperLegSystem`，在每个物理步（1 ms）对 `leg_joint` 施加论文式 7 的力：

```
F = k·(l_p − q) − f_c·tanh(q̇/ε)，q < 0（受压）
F = 0，q ≥ 0（机械止点，腿处于静息长度）
```

- 参数：k = 174 N/m（k/m = 5.00×10³ s⁻²）、f_c = 0.442 N（f_c/m = 12.7 m/s²）、
  l_p = 1.97 cm、l₀ = 22 cm、ε = 0.02 m/s。
- 同时以 1 kHz 在 `/model/quadcopter/leg_state` 发布
  `[sim_time, q, q̇, F, 足端高度]`（`gz.msgs.Float_V`），由 ros_gz_bridge 桥接为
  `/quadcopter/leg_state`。

### 2. URDF 伸缩腿（`quadcopter.urdf.xacro`）

- 新增中央单腿 `foot_link` + `leg_joint`（prismatic，轴 0 0 −1，行程 −0.16~0 m）。
- 整机质量改为论文值 34.8 g（机身 22.8 g + 4 桨 11 g + 足 1 g）。
- 保留四个角腿作为视觉/防擦地结构；只有中央足参与落地。

### 3. 阶段状态机与状态输出（`phase_machine.py` / `leg_phase_node.py`）

实现「空中弹道 → 着陆 → 支撑 → 离地」循环，规则与论文掉落/跳跃检测口径一致：

1. `AERIAL`（弹道）：足端高度 > 3 cm。
2. `LANDING`（着陆）：足端高度下穿 3 cm 阈值。
3. `SUPPORT`（支撑）：腿开始受压（q < −2 mm）；持续到腿回到静息长度。
4. `TAKEOFF`（离地）：腿回位；足端高度上穿 3 cm 后回到 `AERIAL`。

节点发布 `/quadcopter/phase`（`std_msgs/String`）并把全部样本写入 CSV。

## 掉落仿真

- 初始条件：模型竖直、无初速，足端离地 0.50 m（落地速度理论值 3.13 m/s），
  世界 1 ms 步长（dartsim），无推力。
- 首个支撑段（腿受压 q < −1 mm 起止）：t_LD = 2.440 s、t_TO = 2.471 s，
  支撑时间 31.0 ms。
- 4 s 记录内共 4 次弹跳，后三次高度递减（能量被库仑阻尼耗散），阶段序列完整循环。

## 产物

- 数据：`docs/data/task5_drop_leg_state.csv`（1979 行，1 kHz）、
  `docs/data/task5_drop_summary.csv`
- 图：`docs/data/task5_drop_verification.png`（CoM/足高、腿拉伸、阶段与弹簧力时序）
- 截图：`docs/data/task5_photos/`（`task5_drop_aerial_*.png` 空中姿态、
  `task5_drop_compressed_*.png` 最大压缩姿态，各五视图）
- 脚本：`scripts/task5_analyze_drop.py`、`ros2_ws/tools/sim_drop.sh`、
  `ros2_ws/tools/sim_drop_photos.sh`

## 复现步骤

```bash
# 1) 构建 + 测试（C++ 插件 + ROS 包）
wsl -e bash -c "./ros2_ws/tools/sim_build.sh"

# 2) 掉落仿真 + 阶段记录（高度 0.5 m，记录 5 s）
wsl -e bash -c "./ros2_ws/tools/sim_drop.sh 0.5 5"

# 3) 分析 + 出图
python scripts/task5_analyze_drop.py --csv docs/data/task5_drop_leg_state.csv

# 4) 截图（空中 + 最大压缩）
wsl -e bash -c "./ros2_ws/tools/sim_drop_photos.sh 0 aerial"
wsl -e bash -c "./ros2_ws/tools/sim_drop_photos.sh -0.027 compressed"
```

## 限制与说明

- 库仑阻尼用 `tanh(q̇/ε)` 平滑（ε = 0.02 m/s）避免物理引擎颤振；在支撑段速度
  量级（~3 m/s）下与理想符号阻尼几乎一致。
- 支撑时间口径为「腿受压段」（与任务四模型 t_TO 一致）；若按足高 3 cm 阈值计，
  会额外包含落地前/后各约 10 ms 自由落体，这与论文掉落数据处理口径
  （`track_all.m` 用 CoM < 0.225 m）不同。
- 截图由 ModelPhotoShoot 无头渲染（SDF 1.9 无 joint `initial_position`，
  压缩姿态通过调整 leg_joint pose 渲染）；模型无法直接查看图片，已用像素统计
  确认图像非空且两套姿态不同，构图细节请以人工查看为准。
