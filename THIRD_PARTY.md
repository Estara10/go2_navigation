# 第三方依赖、来源与许可证

本项目**没有**为全部代码声明新的统一许可证。各文件和依赖的授权以原作者/上游为准；使用、修改或分发前必须核对并保留相应声明。本目录没有复制 VINS 上游源码、Unitree SDK 二进制/头文件、RealSense 源码或系统包。

## 1. 外部源码与二进制

| 依赖 | 来源/固定版本 | 在本项目中的使用 | 许可证要求 |
|---|---|---|---|
| VINS-Fusion-ROS2 | <https://github.com/zinuok/VINS-Fusion-ROS2>，提交 `72023bc5d2abb14faa4c02a5c040d84f996b587a` | 外部 clone；本目录只保存配置和补丁 | 上游为 GPLv3（移植自 HKUST VINS-Fusion）。补丁是上游代码的修改，分发时需按 GPL 核对源码提供、声明保留等义务。**以 clone 后仓库的 `LICENSE` 为准。** |
| Unitree SDK2 | Unitree 官方 SDK2；本机外部安装 | 实机入口静态链接 `libunitree_sdk2.a`，运行时加载 `libddsc*.so` | 本目录不复制 SDK。许可证以 SDK 随附文件和 Unitree 发布条款为准；不把 SDK 二进制提交到普通 Git 历史。 |
| RealSense ROS 2 / librealsense | 系统安装的 `ros-humble-realsense2-camera`、`ros-humble-librealsense2`（历史验证版本 4.58.4 / 2.58.4） | 相机驱动，本目录不链接、不复制 | 以系统包随附版权文件和上游仓库 `LICENSE` 为准；本项目不重新授权。 |
| ROS 2 Humble、Ceres、OpenCV、Eigen、Boost、glog、gflags、SuiteSparse | 系统包 | VINS 构建/运行依赖 | 各自系统包/上游许可证；本项目不复制这些源码，也不代为声明。 |

## 2. 本项目文件

`README.md`、本目录配置、补丁、文档和原生运动工具代码的来源见 [COPY_ORIGIN.md](../COPY_ORIGIN.md)。复制自原项目的文件仍受原作者/原项目条款约束；本轮没有把它们改授为新的统一许可证。

配置文件中的标定值、外参和噪声参数来自本机记录或上游示例，**不是**许可证授权，也不代表通用标定。

## 3. 分发检查清单

- 保留上游 `LICENSE`、版权头和来源提交记录；
- 不把第三方源码、SDK、系统包、模型或构建产物当作本项目自有资产提交；
- 不复制密码、SSH 私钥、令牌或设备凭据；
- 若把 VINS 补丁或派生代码随二进制分发，先确认 GPLv3 的相应义务；
- 若要给本项目新增许可证，由项目负责人确认后单独说明，不要默认覆盖第三方文件。
