# 任务七：高层跳跃控制器（论文式 22–33）复现记录

## 结论

> 本轮已取得论文主文（scirobotics.adi8912.pdf）、补充材料（_sm.pdf）与
> Zenodo 轨迹跟踪数据包（TrajectoryTracking.zip），将式 22–33 从“笔记重建”
> 升级为“原文核对”，并把轨迹跟踪 RMSE 分析口径改为论文作者脚本的
> 口径（连续轨迹 vs 延迟 1.5 周期参考）。

| 验证项 | 结果 | 论文对照（作者脚本同口径复算） |
| --- | --- | --- |
| 原地连续跳跃 | **12/12 跳稳定**，落点 RMSE 0.000 m，论文口径横向 RMSE 0.000 m | 任务要求 ≥10 跳 |
| 原地高度控制 | 高度 RMSE **0.059 m** | 论文 4.1–4.8 cm（同量级） |
| 圆轨迹跟踪（10 跳） | 论文口径横向 RMSE **0.227 m**，落点 RMSE 0.310 m，高度 RMSE **0.057 m** | 论文圆 0.16 m / 高度 4.1 cm（同量级，1.4×） |
| 阶跃轨迹跟踪（10 跳） | 论文口径横向 RMSE **0.386 m**，落点 RMSE 0.289 m，高度 RMSE **0.066 m** | 论文阶跃 0.40 m / 高度 4.8 cm（基本一致） |

论文基准使用作者随附 MATLAB 脚本（`show_all.m` / `show_step.m`）对
`TrajectoryTracking.zip` 原始数据复算：圆 0.159–0.161 m、阶跃 0.361–0.469 m、
高度 0.037–0.051 m，与论文 16/40 cm、4.1/4.8 cm 一致。

## 实现内容（与论文式的对应）

### 1. 纯 Python 规划模块 `src/hopcopter_model/jump_planner.py`（式 22–33 原文核对）

- 式 23–25（着陆状态预测）：`Δt_LD` 由弹道方程
  `½gΔt² − żΔt − (z−l0) = 0` 求正根（PDF 中“ż/2”为排版/OCR 伪影，
  按物理正确的二次方程实现，ż=0 时与原文一致）；
  `p(t_LD)|k = p(t) + ṗ(t)Δt_LD − e3·½gΔt_LD²`；
  `ṗ(t_LD)|k = ṗ(t) − e3·gΔt_LD`。
- 式 26：`θ(t_LD) = arccos(−z_bᵀ(t_LD)ṗ(t_LD)/‖ṗ(t_LD)‖)`——与本仓库
  `stance.py` 的 θ 定义一致。
- 式 27–29（高度控制，与原文一致）：
  `Δt_PA = z_d/ż(t_TO) − ż(t_TO)/(2g)`（条件 `ż² < 2gz_d`，否则 0）；
  `ż(t_TO) = e3ᵀṗ(t_TO)`；`Δt_PJ = ż(t_TO)/g + √(2z_d/g)`。
- 式 30（下一落点）：`p(t_LD)|k+1 = p(t_LD)|k + [e1 e2]ᵀṗ(t_TO)|k(Δt_PA+Δt_PJ)`
  ——本实现用完整弹道积分（含足端高度条件）替代，精度更高。
- 式 31：`z_b(t_LD)|k = argmin ‖pd − p(t_LD)|k+1‖`——数值优化求期望着陆姿态，
  SLSQP + 抛光轮收敛代数环（落点时间 ↔ 支撑映射 ↔ 下一落点）。
- 式 32（无滑移约束）：`arccos(e3·z_b(t_LD))`、`arccos(e3·z_b(t_TO)) < tan⁻¹μ`，
  与补充材料式 S22 一致。
- 式 33（参考时刻）：`t_LD|k+1 − t = Δt_LD + 2√(2z_d/g)`——控制器已按此式
  取下一周期期望落点位置作为参考。

### 2. 控制器状态机 `jump_controller.py`（无 ROS 依赖，纯逻辑）

每跳周期状态：`STANDBY → ASCENT_POWERED → ASCENT_BALLISTIC → DESCENT →
PRE_LAND → STANCE`。

- 离地：按上一周期实测顶点高度反推 `vz = √(2g·h_meas)`，计算 `Δt_PA`
  （含电机起转补偿 `spin_up_comp`），`hover` 模式施加 mg 推力。
- 顶点：以当前顶点状态规划本期着陆姿态；参考点取**下一周期期望落点时刻**
  （论文“落点参考取下一周期期望位置以消除延迟”）。
- 预着陆：对姿态设定点做 0.25–0.35 s 斜坡（避免无推力姿态环被阶跃激励），
  `attitude_only` 模式（零推力 + 姿态力矩）保持 `z_b(t_LD)` 触地。
- 支撑：零推力保持姿态，被动弹簧腿完成“着陆→离地”映射。
- 鲁棒项：单跳最大修正距离 `max_step`、速度限制器 `max_speed`、
  参考软起步 `ramp_hops`、固定倾角增益 `tilt_scale`（高速时在线增益校准会
  把携带位移误判为倾角增益而发散，最终验证采用固定增益 1.0）。

### 3. 低层控制扩展（任务六基础上）

- `attitude_only` 模式：**下落段保持 ≈mg/10 的小推力**（论文 SM：姿态控制仅需
  ≈mg/10 推力），配合姿态 PID 力矩；限制净推力上限，防止分配器“火箭化”。
- 弹道段关闭偏航位置/速率控制（跳跃周期不需要偏航，避免挤占滚转/俯仰力矩预算）。
- 位姿发布频率 20 → 100 Hz（顶点/速度估计需要）。

## 仿真验证

环境：ROS2 Jazzy + Gazebo Harmonic（DART，1 ms 步长），无头运行。
入口脚本：`ros2_ws/tools/sim_hop.sh`；分析：`scripts/task7_analyze_hop.py`。

```bash
./ros2_ws/tools/sim_hop.sh spot 12 0.6 0.8        # 原地 12 跳
./ros2_ws/tools/sim_hop.sh circle 10 0.6 0.8      # 圆轨迹 10 跳
./ros2_ws/tools/sim_hop.sh step 10 0.6 0.8        # 阶跃轨迹 10 跳
```

最终存档使用的参数：

- spot：`desired_height=0.6, drop=0.8`，12 跳
- circle：`R=0.4 m, ω=0.2 rad/s, drop=0.5, desired_height=0.5, max_tilt=10°,
  max_step=0.25 m, ramp_hops=2, tilt_scale=1.0, adaptive_scale=false`，10 跳
- step：目标 `0 ↔ 0.4 m`，保持 4 跳，`drop=0.8, desired_height=0.6,
  max_tilt=20°, max_step=0.5 m, tilt_scale=1.0, adaptive_scale=false`，10 跳

产物：`docs/data/task7_{spot,circle,step}_{jump,attitude,leg}.csv`、
`task7_{...}_summary.csv`、`task7_{...}_verification.png`。

### RMSE 口径（与论文作者脚本一致）

- 横向：**连续轨迹误差**，参考取 **1.5 个跳跃周期延迟**后的期望位置
  （论文 `show_all.m` / `show_step.m` 的 `traj_delay = jumping_period*1.5`），
  在轨迹跟踪窗口内逐样本取 `√((x−x_d)²+(y−y_d)²)` 再求 RMSE。
- 纵向：每跳顶点（apex）高度相对期望高度的 RMSE（本实现取
  `z_apex − z_takeoff` vs `z_d`，与论文阶跃脚本
  `pks − (desired_jumping_altitude + 0.2)` 物理等价；论文圆脚本省略 +0.2 的
  基准差异使其数值略小，量级一致）。
- 落点 RMSE（每跳落点 vs 落点时刻参考）作为辅助指标一并输出。

## 限制与差距（如实记录）

1. **圆轨迹在更高速度/更大转角配置下仍会偶发散**：Gazebo 无气动稳定器
   （任务八范围），远侧急转处一旦出现一次不良着陆，姿态误差会逐跳累积并
   导致停跳；10 跳存档配置（低速小圆）全程稳定、RMSE 与论文同量级。
2. **支撑增益随速度增长**：仿真触地动力学使有效转向增益约为模型的
   1.3–2 倍且随水平速度增长；固定倾角增益 1.0 在该配置下表现最佳。
3. **姿态权限受限**：无推力姿态控制受电机升速（20 ms）与净推力上限约束，
   大倾角（>25°）在 0.35 s 下落窗口内难以收敛，故限制单跳修正距离与倾角。
4. 论文圆实验半径为约 1.0 m、跳高 0.6–0.75 m，本存档圆为 R=0.4 m、跳高 0.5 m；
   RMSE 对比基于相同评估口径，几何尺寸不同已在文中说明。

## 文件清单

- `src/hopcopter_model/jump_planner.py` + `tests/test_jump_planner.py`
- `quadcopter_description/jump_controller.py`、`jump_control_node.py`、
  `launch/quadcopter_jump.launch.py`
- `quadcopter_description/attitude_control_node.py`（`attitude_only` 模式）
- `quadcopter_description/attitude_controller.py`（偏航增益开关）
- `tools/sim_hop.sh`、`scripts/task7_analyze_hop.py`
- 本记录与 `docs/data/task7_*`
