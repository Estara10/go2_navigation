# AGENTS.md — A1 项目工作说明

## 范围

本仓库是 A1：基于 D435i 定位的 Go2 平地曲线路径到点控制。当前是提取后的独立工程起步资产，**尚未实现** A1 自动曲线到点和 5 cm 实测验收。

旧 `goto x y` 不是 A1：它按首次 `origin` 的固定坐标运动，并可能通过 `vy` 侧移。不要把它描述成已满足 A1 的“下达命令时按当前机头方向确定固定目标、前进+转向跟踪”语义。

## 离线与实机边界

- 默认只做离线构建、自检、文档和配置整理。
- 构建和自检不得初始化实机 DDS、连接机器人/相机或发送运动命令。
- 不运行实机入口；需要现场动作时由项目负责人另行安排。
- 不在未经授权时创建/推送 GitHub 仓库、下载或安装系统包、修改系统规则。
- 实机只允许一个运动命令发送入口；保留停止失败锁存。程序退出或 `StopMove` 返回 0 都不代表实际停稳。
- 网卡、IP、DDS domain、SDK 路径、输出目录必须由使用者显式确认/配置；不要静默使用个人电脑或历史设备默认值。

## 构建与自检

在项目根目录：

```bash
PROJECT_ROOT=.   # 或绝对路径
# 无 SDK：只构建核心与自检
cmake -S "$PROJECT_ROOT/scripts/go2_native_motion" \
  -B "$PROJECT_ROOT/build/native_motion-offline" \
  -DGO2_REAL_ROBOT=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$PROJECT_ROOT/build/native_motion-offline" --parallel 2
"$PROJECT_ROOT/build/native_motion-offline/go2_native_motion_selftest"
```

实机入口必须显式要求并给出外部 SDK 前缀：

```bash
cmake -S "$PROJECT_ROOT/scripts/go2_native_motion" \
  -B "$PROJECT_ROOT/build/native_motion-sdk" \
  -DGO2_REAL_ROBOT=ON \
  -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$PROJECT_ROOT/build/native_motion-sdk" --parallel 2
```

`GO2_REAL_ROBOT=ON` 而 SDK 缺失时，配置必须失败。`AUTO` 缺少 SDK 时只构建核心/自检并明确说明实机入口未构建。

## VINS 与相机

- 固定上游提交、补丁顺序、已验证环境和独立构建目录见 `scripts/go2_vins/README.md`。
- 不自动下载/安装/升级系统包；不改变现有估计参数、算法或生产候选配置语义。
- 相机—IMU 外参与相机—Go2 机身安装外参是两组不同变换；本机 D435i 标定不是通用标定。
- 历史记录保留在 `docs/history/`，其中的原路径和命令只作追溯，不能直接照抄执行。

## 数据与收尾

- 原始 bag、视频、模型、第三方源码、构建/安装产物不提交普通 Git 历史。
- 保留原始资料和异常结果；不删除失败记录，不修改本项目外的文件。
- 每次运行后关闭本次启动的相机、VINS、记录器和运动工具，核对残留进程。
- 只按任务需要做验证；不建设通用框架、复杂审批体系或无关的重复测试。

## 接口与协作

统一参考点、坐标系、单位、时间戳和速度含义见 `docs/INTERFACE.md`；协作流程、分支/PR 和实机安全边界见 `docs/COLLABORATION.md`。先确认接口，再按定位、路径跟踪、Go2 接口与实验三人分工实现。
