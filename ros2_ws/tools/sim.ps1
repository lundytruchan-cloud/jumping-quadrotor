<#
  sim.ps1 - 四旋翼仿真常用命令的 Windows 快捷入口
  （在 PowerShell 里调用，免去 PowerShell -> wsl 的转义问题）

  用法:
    .\tools\sim.ps1 start                      # 启动仿真（GUI + rviz2）
    .\tools\sim.ps1 start gui:=false rviz:=false
    .\tools\sim.ps1 stop                       # 清理所有仿真进程
    .\tools\sim.ps1 status                     # 查看状态 / 多实例检查
    .\tools\sim.ps1 demo 0.25 5 25             # 电机起转演示 + 记录
    .\tools\sim.ps1 build                      # 构建 + 测试
    .\tools\sim.ps1 photos                     # 模型五视图渲染

  复杂参数（含引号/$ 等）请直接在 WSL 里运行 ros2_ws/tools/sim_*.sh。
#>
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet('start', 'stop', 'status', 'demo', 'build', 'photos')]
  [string]$Action,
  [string]$Extra = ''
)

$script = "ros2_ws/tools/sim_$Action.sh"
$command = "./$script"
if ($Extra) {
  $command += " $Extra"
}
Write-Host "==> wsl: $command"
wsl -e bash -c $command
exit $LASTEXITCODE
