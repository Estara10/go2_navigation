# go2_navigation

基于 D435i/VINS 定位的 Go2 平地曲线路径到点控制。

当前状态：**起步阶段**。已有原生运动工具、VINS 候选配置和离线自检；自动曲线到点与机身参考点 5 cm 实测精度**尚未实现**。

## 目标

- 只在平地、开阔、短距离低速场景开展。
- 用 D435i 双目和 IMU，经 VINS 计算位置与速度。
- 对外统一到固定机身参考点 `base_link`，不使用全身重心。
- 默认目标按下达命令时的机体坐标解释：前 +X、左 +Y；下令后目标固定，不随航向变化。
- 通过前进和转向跟踪平滑路径，正常跟踪不主动横移。
- 停稳后机身参考点的实际水平误差 ≤ 5 cm；用卷尺、地面标记或视频独立测量，不以估计进入容差代替验收。

## 目录

| 路径 | 内容 |
|---|---|
| `STATUS.md` | 当前状态与下一步任务 |
| `A1_PROJECT_REQUIREMENTS.md` | 需求、坐标约定和验收定义 |
| `AGENTS.md` | 项目边界与安全约定 |
| `docs/COLLABORATION.md` | 工作流、接口和安全边界 |
| `docs/INTERFACE.md` | 冻结的机身状态/速度命令接口与 `T_base_cam` 方向 |
| `docs/MEASUREMENT_CHECKLIST.md` | `base_link` 原点和 `T_base_cam` 的测量清单 |
| `scripts/go2_nav_core/` | 仅标准库的机身状态变换与离线自检 |
| `scripts/go2_nav_plan/` | VINS→`base_link` 回放、Dubins + Pure Pursuit 离线原型 |
| `scripts/go2_native_motion/` | Go2 原生 SDK2 终端工具、只读查询、离线自检 |
| `scripts/go2_vins/` | VINS 固定版本、补丁、D435i 候选配置 |
| `docs/history/` | 历史记录，仅作追溯 |
| `THIRD_PARTY.md` | 第三方依赖与许可证 |

## 当前能力

- 原生 `SportClient.Move/StopMove` 终端工具：10 Hz 重发、状态超时停止、`StopMove` 失败锁存、CSV 记录。
- `goto x y` / `yaw deg` 是旧工具：目标在首次 `origin` 坐标系中，`goto` 可能使用侧移，不是 A1 的“按当前机头方向下达目标、前进加转向跟踪”。
- VINS 可构建、可输出位姿和速度，但精度、尺度、停后稳定性、相机到机身外参都未验证。
- `scripts/go2_nav_core/` 已有 `BodyState`/`VelocityCommand` 接口、相机 IMU→`base_link` 变换、杠杆臂补偿、nav 系转换、相对目标一次性转换和 valid 判据；`T_base_cam` 仍为未实测状态，测试只用合成值。
- `scripts/go2_nav_plan/` 已有 VINS CSV→`BodyState` 回放、直线/Dubins 路径和 Pure Pursuit 离线模拟；模拟 A→B 位置误差约 2 cm，不是实机 5 cm。
- 路径生成、Pure Pursuit 跟踪、VINS 与运动工具联动、5 cm 实机验收：未实现。

## 离线构建与自检

无 SDK 构建和自检不会初始化 DDS，也不会连接机器人：

```bash
cd /path/to/go2_navigation
cmake -S scripts/go2_native_motion -B build/native_motion-offline \
  -DGO2_REAL_ROBOT=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/native_motion-offline --parallel 2
./build/native_motion-offline/go2_native_motion_selftest
```

预期最后一行：`all selftests passed; no DDS initialized`。

实机入口需要外部 Unitree SDK2：

```bash
cmake -S scripts/go2_native_motion -B build/native_motion-sdk \
  -DGO2_REAL_ROBOT=ON -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/native_motion-sdk --parallel 2
```

`GO2_REAL_ROBOT=ON` 而 SDK 缺失时配置直接失败；`AUTO` 缺 SDK 时只构建核心和自检，并明确说明实机入口未构建。

机身状态接口模块只依赖 C++17 标准库：

```bash
cmake -S scripts/go2_nav_core -B build/go2_nav_core \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/go2_nav_core --parallel 2
./build/go2_nav_core/go2_nav_core_selftest
```

## VINS

- 上游：<https://github.com/zinuok/VINS-Fusion-ROS2>
- 固定提交：`72023bc5d2abb14faa4c02a5c040d84f996b587a`
- 基础补丁：`scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch`
- 候选配置：`scripts/go2_vins/config/d435i_480x270/`（本机候选，不是通用标定）
- 获取、补丁顺序、构建目录和限制见 `scripts/go2_vins/README.md`。

VINS 上游为 GPLv3。分发补丁或派生代码前先核对上游许可证。

## 安全边界

- 首期只在平地、开阔、无障碍区域验证，不声称自主避障。
- 实机只允许一个运动命令发送入口；保留停止失败锁存。
- 程序退出或 `StopMove` 返回 0 都不代表机器人实际停稳，现场必须人工确认。
- 运动前确认网卡、DDS domain、SDK 路径和停止方式；不要照抄历史个人路径或设备参数。
- 定位失效、超时或明显跳变时停止任务，不静默切回原生位置反馈继续走。

## 数据

原始 bag、视频、模型、第三方源码和构建产物不进入普通 Git 历史；失败和异常记录保留，不删除或改写。

## 许可证

本仓库暂不声明统一许可证。第三方依赖和许可证要求见 `THIRD_PARTY.md`。
