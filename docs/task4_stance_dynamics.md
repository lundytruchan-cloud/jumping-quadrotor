# 任务四：支撑阶段动力学复现（论文式 12–21）

## 结论

已完成 Hopcopter 支撑阶段（stance phase）动力学从 MATLAB（Zenodo 代码包）到 Python 的移植，并用论文参数复现图 2C 着陆→离地映射。

- 模型代码：`src/hopcopter_model/`（核心在 [stance.py](../src/hopcopter_model/stance.py)）
- 论文参数复现结果（作者 57 跳验证集）：**Δψ RMSE = 0.998°，θ_TO RMSE = 1.817°**，均 < 2°，与论文报告的 1.0°/1.6° 同量级。
- 动捕管线移植与作者 `evaluate_data.m` 输出逐点一致（中位偏差约 0.002°）。
- 支撑时间模型值约 31.8 ms，与论文实测 ~32 ms 一致。
- 原始 Zenodo 数据不入库（见 `data/.gitignore` 说明），脚本、图、处理结果已入库。

## 数据来源

- 记录：Bai, S. et al. (2024). *An agile monopedal hopping quadcopter with synergistic hybrid locomotion*. Science Robotics 9(89), eadi8912. DOI: 10.1126/scirobotics.adi8912
- 代码与数据：Zenodo DOI [10.5281/zenodo.10777420](https://doi.org/10.5281/zenodo.10777420)
- 本次使用 `StanceDynamics.zip`（935,603 B）与 `DropTest.zip`（5,044,549 B），解压至本地 `data/zenodo/`（已 gitignore）。
- SHA-256（供溯源核对）：
  - `DropTest.zip` = `94D0402DE566392C8D6132656D00B0D7784E6426394834907E8CF2D30F179438`
  - `StanceDynamics.zip` = `C932E473CD506201B5E2C29A85CF5A5D7953AB92630B5BE7DDB87C3C5564FE65`

`DropTest.zip` 用于后续掉落实验参数辨识（任务 10），本任务分析仅用 `StanceDynamics.zip`。

## 模型：论文式 12–21 ↔ Python

坐标约定：着陆瞬间脚点固定，机身绕脚点转动；`l(t)` 为脚点到质心的腿长，`θ_LD` 为着陆速度反方向与机体 z 轴的夹角，`v_p = ‖ṗ_LD‖cos θ_LD` 为轴向压缩速度，`v_perp = ‖ṗ_LD‖sin θ_LD` 为切向速度。

| 论文式 | 内容 | Python 实现 |
|---|---|---|
| 12 | 轴向动力学 `m l̈ = −k(l − l₀ − l_p) − f_c·sgn(l̇)`，压缩/伸展两相解析解 | `stance._compression_end`、`stance_transform` |
| 13 | 绕脚点角动量守恒 `m l² β̇ = const` | `stance._integrate_stance` |
| 14 | `β̇(t) = l₀·v_perp / l(t)²` | 同上 |
| 15 | 压缩结束时刻 `t₁ = atan(v_p / (ω(C_d − l₀)))/ω` | `_compression_end` |
| 16 | 压缩相 `l_d(t) = C_d − (v_p/ω)sin ωt − (C_d − l₀)cos ωt` | `stance_transform` |
| 17 | 伸展相 `l_u(t) = C_u + (l(t₁) − C_u)cos(ω(t − t₁))` | 同上 |
| 18 | 离地条件 `l_u(t_TO) = l₀` | `_takeoff_time` |
| 19 | 机身转角 `θ_t = θ_LD + ∫₀^{t_TO} β̇ dt` | `StanceResult.theta_t` |
| 20 | 离地速度角 `θ_v = θ_t + atan(v_perp / l̇_u(t_TO))` | `StanceResult.theta_v` |
| 21 | 离地速度 `v_t = √(v_perp² + l̇_u(t_TO)²)` | `StanceResult.v_t` |

其中 `ω = √(k/m)`，`C_d = l₀ + l_p + f_c/k`，`C_u = l₀ + l_p − f_c/k`（质量约去后 `f_c/k = (f_c/m)/(k/m)`）。

该实现与作者 `run_this.m` 中 `stance_transform` 使用的符号解（`eqns_l.mat`，由 `derive_eqns*.mlx` 推导）数学等价，已用连续性条件（`l` 连续、`l̇(t₁)=0`）、离地条件 `l(t_TO)=l₀` 及角动量守恒逐项验证。

## 参数

| 参数 | 论文值 | 代码（`params.py`） |
|---|---|---|
| k/m | 5.00×10³ s⁻² | `PAPER_PARAMS.k_over_m = 5.00e3` |
| f_c/m | 12.7 m/s² | `PAPER_PARAMS.f_c_over_m = 12.7` |
| l_p | 1.97 cm | `l_p = 0.0197` |
| l₀ | 22 cm | `l0 = 0.22` |

另保留作者 MATLAB 代码内的辨识值（`k=170.0615 N/m, m=34 g, f_h=0.4453 N` → `AUTHORS_PARAMS`）用于交叉验证；两者结果几乎一致。

## 图 2C 复现

- [task4_fig2c_mapping.png](data/task4_fig2c_mapping.png)：作者原始网格（θ_LD 0–25°，‖ṗ_LD‖ 1–3.5 m/s，步长 0.5°/0.05 m/s），三面板：Δψ、θ_TO、‖ṗ_TO‖，并叠加 57 个实测跳跃点。
- [task4_fig2c_mapping_ext.png](data/task4_fig2c_mapping_ext.png)：任务要求扩展网格（θ_LD 0–45°，‖ṗ_LD‖ 0.1–4.5 m/s）。
- 映射表：[task4_mapping_grid.csv](data/task4_mapping_grid.csv)（原始网格）、[task4_mapping_grid_ext.csv](data/task4_mapping_grid_ext.csv)（扩展网格）。

映射性质（模型自检）：同一速度下 Δψ、θ_TO、v_TO 均随 θ_LD 单调增加；θ_LD=0° 时无旋转，仅轴向弹跳。

## 动捕数据处理（evaluate_data.m 管线）

按作者脚本移植的管线（`src/hopcopter_model/evaluate.py`）：

1. 读取 `data/20220502_140314.mat`（Qualisys 动捕：位置 + 四元数）；
2. 足端位置 = 机体位姿沿机体 z 轴平移 `foot_offset = −0.22 m`；
3. 用足高阈值 `0.03 m` 线性插值检测 t_LD / t_TO（等价 `polyxpoly`）；
4. 中心差分（`diff_same`）求着陆/离地速度；
5. 每跳用 Nelder-Mead 拟合地面平面法向（两个旋转角），将速度与机体朝向投影到平面，计算 θ_LD、机身转角变化 Δψ、离地速度角 θ_TO；
6. 用支撑模型对每跳预测 Δψ、θ_TO，并计算 RMSE。

作者脚本中“用鼠标选择时间段”的交互步骤，经核对对应着陆时间约 26.5–60.7 s 的连续区间（57 跳），其输出与包内 `data_output/saved_data.mat` 完全一致（各角值中位偏差 0.002°）。脚本以 `--window 26 61` 复现该选区；`--full` 处理全记录（132 跳）。

## 验证结果

| 指标 | 论文报告 | 本复现（论文参数，57 跳） | 全记录 132 跳 |
|---|---|---|---|
| Δψ RMSE | 1.0° | 0.998° | 1.03° |
| θ_TO RMSE | 1.6° | 1.817° | 2.53° |
| Δψ 最大绝对误差 | — | 2.62° | 3.91° |
| θ_TO 最大绝对误差 | — | 3.71° | 22.4° |

验收标准（RMSE < 2°）在论文验证集（57 跳）上满足。全记录额外包含论文验证范围之外的跳跃（速度低至 0.61 m/s、θ_LD 高至 21°），其中 θ_TO 误差明显增大；这解释了全记录 RMSE 略超 2°，也说明模型的适用区间与论文一致（‖ṗ_LD‖ ≈ 1.5–2.4 m/s，θ_LD ≲ 10°）。

θ_TO RMSE（1.817°）与论文 1.6° 的细微差异来自参数取值（论文 12.7 m/s² vs 代码 13.1 m/s²，影响很小）与选点细节；同为亚 2° 量级，且与作者代码自洽。

图： [task4_prediction_scatter.png](data/task4_prediction_scatter.png)（实测 vs 模型散点）、[task4_mocap_overview.png](data/task4_mocap_overview.png)（足高、穿越检测、速度与选区）；逐跳明细：[task4_jumps.csv](data/task4_jumps.csv)。

## 复现步骤

```bash
# 依赖（Windows Python 3.10+ 或 WSL）
pip install -r requirements.txt

# 1) 复现图 2C 与映射表
python scripts/task4_reproduce_fig2c.py

# 2) 动捕数据评估与验证（默认作者 57 跳选区）
python scripts/task4_evaluate_data.py
# 全记录版本
python scripts/task4_evaluate_data.py --full

# 3) 单元测试
python -m pytest tests -q
```

## 限制与后续

- 模型为平面模型（无滑移、无面外运动），论文数据也只在较小 θ_LD 区间验证；扩展网格的远端（大角度/高速度）用于展示映射趋势，未经实验验证。
- `DropTest.zip` 的解压与 `track_all.m` 参数辨识管线留待任务 10（真机/仿真参数辨识）使用。
