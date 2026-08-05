# Jumping Quadrotor

四旋翼可跳跃飞行器复现项目：以学习为目的，从动力学建模、仿真到控制算法，逐步复现一台可跳跃的四旋翼平台。

## 项目定位

- 学习者：自动化专业，通过本项目学习机器人学、控制理论与 ROS2 开发。
- 阶段目标：四旋翼可跳跃飞行器复现（当前为阶段性目标，非全局目标）。
- 技术路线：ROS2 + Gazebo 仿真优先，控制算法验证后再考虑真机。

## 学习路线

1. **ROS2 基础**：节点、话题、服务、参数；colcon 工作空间与 launch 文件。
2. **四旋翼动力学**：刚体运动学/动力学、螺旋桨推力与力矩模型、状态表示（四元数/欧拉角）。
3. **控制理论**：PID 串级控制 → LQR / 模型预测控制；仿真中调参与验证。
4. **Gazebo 仿真**：URDF/Xacro 建模、传感器仿真、控制器插件。
5. **跳跃机构**：跳跃运动学、触地/腾空阶段控制、能量分析（复现核心）。

## 参考资源

- ROS2 官方文档：https://docs.ros.org
- Gazebo 文档：https://gazebosim.org/docs
- 四旋翼控制经典教材：*Quadcopters: Dynamics and Control*（或同主题学术论文）
- PX4 开发者文档（真机阶段参考）：https://docs.px4.io

## 仓库约定

- 分支：`codex/` 前缀；提交信息：`feat/fix/docs/chore/refactor/test` 类型前缀。
- 验证通过（colcon build + 测试）才可提交；详见 [AGENTS.md](AGENTS.md)。
