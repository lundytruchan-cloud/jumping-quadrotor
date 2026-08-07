# 任务八：气动稳定器、Poincaré 映射与三策略仿真验证

## 结论

1. **Poincaré 映射与论文图 6C 完全一致**：按作者 `run_this_0.m` 移植的
   0.5 m 跳高映射与 `variables_results_05.mat` 全网格最大差 **0.0000°**；
   `θ_z|k+1/θ_z|k < 1` 定义的稳定区占全网格 13.6%，而在
   `φ=0`（仅姿态）与 `φ=θ_z`（仅稳定器）两条虚线之间占 **75.5%**，
   与论文「单独策略均贴稳定边界、组合策略落入重叠稳定区」一致。
2. **StabilityTests 数据复算**：组合策略 45 s 内 **56 次落地**（论文 56+），
   落地 θ_z 中位数 **3.5°**（近零横向速度）；仅姿态控制器 3–5 跳发散；
   仅稳定器 7–12 跳发散（论文 6–10），量级一致。
3. **仿真三组实验（无位置反馈，3° 初始俯仰扰动）**：

| 策略 | 目标跳数 | 实际跳数 | 落地 θ_z 中位数 | 结果 |
| --- | --- | --- | --- | --- |
| 仅姿态控制器 | 8 | **4** | 20.9°（最大 76.1°） | 发散（漂移 10.6 m） |
| 仅气动稳定器 | 12 | **3** | 2.7°（最大 59.4°） | 发散（漂移 2.1 m） |
| 组合策略 | 32 | **32** | 2.6°（最大 26.8°） | **连续 32 跳稳定** ✅ |

   组合策略落地 φ 中位数 3.5°、最大 12.1°，32 跳内没有出现 90° 级姿态
   失控；最终横向漂移 2.6 m（略超论文 3×3 m 场地，见限制说明）。

## 气动稳定器模型（论文 SM 式 S33）

- 结构：4.9 g 可拆卸模块，3 个 39 cm²（5.2×7.5 cm）水平铰接面，
  H-King 282AS 舵机经拉线拉紧/放松（论文图 6A）。
- 气动力（平板理论）：
  `F_A = ρ·S·sinθ_LD·U²`，ρ=1.2 kg/m³，S_eff=2×39 cm²=78 cm²；
  力矩臂取 0.06 m（论文：CoM 到压心约为电机臂的 1–2 倍）。
- 只施加**对齐力矩**：力矩轴 `z_b × (−v̂)`，使 `z_b → −v̂`；约 10 mN 级
  气动力本身相对自重（≈340 mN）可忽略，故不施加力。
- 激活调度：上升段/悬停段松弛（气动力可忽略，不影响姿态控制），
  下落段拉紧；`attitude_only` 永不激活，`stabilizer_only` 始终激活，
  `combined` 仅下落段激活。

实现文件：
- `src/hopcopter_model/stabilizer.py`：模型 + 激活/策略逻辑（纯 Python，可单测）。
- `src/hopcopter_model/poincare.py`：映射移植 + 稳定区/不动点轨迹。
- `quadcopter_gz_plugins`：`aero_stabilizer.hh` / `aerodynamic_stabilizer_system.cc`
  在物理步长内按自身位姿差分速度施加力矩（**不用位姿反馈**），
  订阅 `/model/quadcopter/stabilizer_active`（由控制器发布、launch 桥接）。
- `quadcopter_description/jump_controller.py` / `jump_control_node.py`：
  三策略切换、无位置反馈开环周期、`q_dot` 离地速度估计（高度控制）。
- `tools/sim_task8.sh`、`scripts/task8_*.py`：仿真与分析。

## Poincaré 映射与图 6C 对照

映射定义（论文方法与作者脚本）：

```
θ_z|k  = 落地速度方向与竖直的夹角
φ|k    = 落地姿态（机身轴与竖直的有符号夹角）= θ_z|k − α|k
α|k    = 机身轴与 −v̂(t_LD) 的夹角
θ_z|k+1 = π/2 − atan2(v_vert, |v_x_TO|)，v_vert = √(2g·0.5)
```

产物：
- 图：`docs/data/task8_poincare_fig6c.png`
- 网格数据：`docs/data/task8_poincare_map.csv`、`task8_poincare_fixed_points.csv`
- 对照摘要：`docs/data/task8_poincare_summary.txt`
- 作者数据叠加图：`docs/data/task8_stability_fig6c_overlay.png`

## StabilityTests 数据复算

`data/task8/StabilityTests`（作者 Zenodo 原始 .mat）按 `show_all_data.m`
口径处理：

| 组 | 文件数 | 落地数 | 结论 |
| --- | --- | --- | --- |
| with_damper（组合/被动稳定） | 1 | **56**（45 s） | 稳定，θ_z 中位 3.5° |
| no_damper（仅姿态） | 4 | 3–5 | 发散 |
| no_controller（仅稳定器） | 4 | 7–12 | 发散 |

逐跳数据：`docs/data/task8_stability_hops.csv`、
`docs/data/task8_stability_summary.csv`。

## 仿真实现要点与调试记录

- **无位置反馈**：控制器不订阅位姿；跳周期由腿接触事件（leg_state）+ 仿真
  时钟驱动，离地速度用腿伸展速度 `q_dot`（编码器级信号）估计并做高度
  自适应（`dt_PA` 按 `powered_ascent_duration` 计算），全程不读取
  `/quadcopter/pose`。
- **力矩符号修正**：初版按 `(−v̂) × z_b` 施加力矩，经仿真发现会把机身
  越推越偏（φ 变负、k_eff 越大越糟）；按转动动力学
  `ż_b ∝ τ × z_b` 修正为 `z_b × (−v̂)`，组合策略从 1–2 跳失控变为 32 跳
  稳定。C++/Python 均有对应单测锁定该方向。
- **速度估计**：插件用 20 ms 位姿窗口差分（1 kHz 差分噪声会污染力矩方向），
  并拒绝 spawn 首帧的位姿跳变尖峰。
- 最终配置：`k_eff=8`、力矩臂 0.06 m、`att_only_torque_scale=1.0`、
  初始俯仰扰动 3°、跳高 0.5 m、初始 0.75 m 下落（论文 75 cm）。

## 复现命令

```bash
# 映射 + StabilityTests 复算（Windows 直接跑）
python scripts/task8_poincare_map.py
python scripts/task8_analyze_stability.py

# 三组仿真（WSL；无位置反馈）
./ros2_ws/tools/sim_task8.sh attitude_only 8 0.5 0.75 0.0 '' '' 3.0
./ros2_ws/tools/sim_task8.sh stabilizer_only 12 0.5 0.75 0.15 '' '' 3.0
./ros2_ws/tools/sim_task8.sh combined 32 0.5 0.75 0.0 '' '' 3.0
```

## 限制与差距（如实记录）

1. 仅稳定器在本文仿真中 3 跳即发散，比论文 6–10 跳更快：仿真中的腿/足
   接触动力学更“脆”，且无姿态阻尼时首跳离地姿态偏差较大；结论（单独
   策略不稳健、单跳数级内发散）与论文一致，具体跳数有差异。
2. 组合策略 32 跳横向漂移累计 2.6 m，略超论文 3×3 m 场地；θ_z 中位数
   2.6° 属「近零横向速度」量级，但极限环仍有 ~0.08 m/跳的系统性漂移，
   主要来自初始扰动在姿态/稳定器平衡点留下的残余 φ。仿真世界无边界，
   不影响连续跳跃本身。
3. `k_eff=8` 是对本仿真植物（姿态 PID、电机模型）的标定值；论文只给
   出量级（气动力 10 mN、力矩与 mg/10 姿态推力可比），具体等效面积/
   压心需要实物风洞或飞行标定。

## 文件清单

- `src/hopcopter_model/stabilizer.py`、`poincare.py` + 根目录测试
- `ros2_ws/src/quadcopter_description/...`：URDF 插件、launch、控制器
- `ros2_ws/src/quadcopter_gz_plugins/...`：C++ 插件与 gtest
- `ros2_ws/tools/sim_task8.sh`、`scripts/task8_*.py`
- `docs/data/task8_*`：图、CSV、摘要
- 作者原始数据在 `data/task8/`（gitignore，不入库）
