# VINS-Fusion-ROS2：独立获取与构建

本目录不含上游 VINS 源码，只保存本项目的候选配置和窄适配补丁。目标是让新成员在**独立源码、构建、安装目录**中复现同一固定版本，不依赖任何个人 `/tmp` 构建树或毕业设计总仓库。

## 1. 固定版本与已验证环境

- 上游：<https://github.com/zinuok/VINS-Fusion-ROS2>
- 固定提交：`72023bc5d2abb14faa4c02a5c040d84f996b587a`
- 上游许可证：GPLv3；分发前请核对上游 `LICENSE`，本仓库不为上游代码重新授权。
- 构建目标：只构建 `camera_models` 和 `vins` 估计器；不构建 `loop_fusion`、`global_fusion`，不安装完整 RTAB-Map。

2026-11-05 / 2026-11-08 在**一台**机器上验证过的环境：

| 项目 | 已验证版本 |
|---|---|
| OS | Ubuntu 22.04.5 amd64 |
| ROS | ROS 2 Humble |
| CMake | 系统 3.22 |
| 编译器 | 系统 GCC 11 |
| Ceres | 2.0.0（系统 APT 包） |
| OpenCV | 4.5.4（系统 APT 包） |
| Eigen | 3.4.0（系统 APT 包） |
| 其他 | Boost 1.74、glog 0.5/0.4、gflags 2.2.2、SuiteSparse 5.10.1 |

上游移植 README 写的是 Ubuntu 20.04 / ROS 2 Foxy / OpenCV 3.4.1 / Ceres 2.1.0 / Eigen 3.3.9；**不能把本项目的 Humble 构建视为上游承诺支持的组合**。本项目只做了最小兼容补丁，没有改变估计器状态、残差、协方差或求解器配置。

> **验证边界：** 2026-11-09 的独立化整理没有重新 clone、打补丁或构建 VINS；上述构建结果来自历史记录。全新机器、不同 APT 版本和网络下载均未在本轮验证。不要把本文当作“任意电脑一键构建”的证明。

## 2. 目录约定

建议使用项目内独立目录，均已被 `.gitignore` 排除：

```bash
export PROJECT_ROOT=/path/to/go2_navigation
export VINS_SRC_DIR="$PROJECT_ROOT/third_party/VINS-Fusion-ROS2"
export VINS_BUILD_DIR="$PROJECT_ROOT/build/vins"
export VINS_INSTALL_DIR="$PROJECT_ROOT/install/vins"
```

不要复用其他项目的 `/tmp` 源码树或安装前缀，也不要覆盖现场运行中的构建。

## 3. 系统依赖

VINS 需要系统 ROS 2 Humble、colcon、Ceres、OpenCV、Eigen、Boost、glog、gflags、SuiteSparse 等开发包。**本文不自动下载、安装或升级系统包**；请按已确认的环境由使用者自行准备和核对。

已知历史问题：本机曾有 `/usr/local/include/eigen3` 的 Eigen 3.3.9 与 APT Eigen 3.4.0 混用。补丁把上游的 `<eigen3/Eigen/...>` 规范为 `<Eigen/...>`，以便命中 APT 头文件；不要为此修改系统安装或覆盖 `/usr/local`。

## 4. 获取与补丁（需要网络，手动执行）

```bash
mkdir -p "$(dirname "$VINS_SRC_DIR")"
git clone https://github.com/zinuok/VINS-Fusion-ROS2.git "$VINS_SRC_DIR"
git -C "$VINS_SRC_DIR" checkout 72023bc5d2abb14faa4c02a5c040d84f996b587a
```

基础补丁必须从固定提交开始，按以下顺序执行：

```bash
cd "$VINS_SRC_DIR"
git apply --check "$PROJECT_ROOT/scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch"
git apply "$PROJECT_ROOT/scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch"
git diff --check
```

基础补丁 `0001-vins-humble-cpu-and-odometry.patch` 包含：

- 首次清理前初始化预积分/边缘化指针；
- CPU `DENSE_SCHUR`，移除 CPU 构建仍会解析的 `ceres::CUDA` 引用；
- Ceres 2.1 `Manifold` 到本机 Ceres 2.0 `LocalParameterization` 的接口适配；
- Eigen include 路径规范化；
- Humble Marker 寿命设置；
- Odometry 世界系速度旋到 child/body 系；
- SIGINT 时有序停止并 join 图像同步/估计线程。

可选诊断 diff `diagnostic_visualization.diff` **不是第二份基础补丁，也不是 `0001` 之后的增量补丁**：

- 它是历史 A/B 回放临时源码的 `visualization.cpp` 整文件差异，已包含 `0001` 中的 `clearPub`、`child_frame_id`、body 系 twist 和 Marker 寿命改动，因此不能直接应用在已打 `0001` 的树上；
- 它额外包含默认关闭的 `VINS_DIAG_EXTRINSIC` 输出，不新建线程、不改求解路径；
- 本目录保留它用于审计。若需要诊断构建，应从该 diff 中只提取 `VINS_DIAG_EXTRINSIC` 相关改动，基于已应用 `0001` 的源码生成新的增量 diff，并先用 `git apply --check` 验证；
- 该提取和验证本轮未执行，不能把本文件当作可直接运行的现场步骤。

## 5. 构建

使用干净环境，避免 Anaconda 的 CMake/Python 污染；构建与安装都写入项目内独立目录：

```bash
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  VINS_SRC_DIR="$VINS_SRC_DIR" \
  VINS_BUILD_DIR="$VINS_BUILD_DIR" \
  VINS_INSTALL_DIR="$VINS_INSTALL_DIR" \
  CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  bash --noprofile --norc -c '
set -e
source /opt/ros/humble/setup.bash
cd "$VINS_SRC_DIR"
colcon --log-base "$VINS_BUILD_DIR/log" build --packages-up-to vins \
  --parallel-workers 1 \
  --event-handlers console_direct+ \
  --build-base "$VINS_BUILD_DIR/build" \
  --install-base "$VINS_INSTALL_DIR" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-DEIGEN_DONT_PARALLELIZE
'
```

构建后检查：

```bash
source "$VINS_INSTALL_DIR/setup.bash"
ros2 pkg prefix vins
ros2 pkg executables vins
ls -l "$VINS_INSTALL_DIR/lib/vins/vins_node"
ldd "$VINS_INSTALL_DIR/lib/vins/vins_node" | grep "not found" || true
```

`ldd` 不应出现 `not found`。历史记录中该检查共有 183 项依赖，包含系统 `libceres.so.2`；不同机器的条目数会不同，不能把 183 当作固定指标。

## 6. 配置与运行边界

候选配置在 `config/d435i_480x270/`：

- `realsense_stereo_imu.yaml`：VINS 主配置；
- `left.yaml`、`right.yaml`：本机 480×270 红外内参。

这些是**本机 D435i 候选值，不是通用标定**：

- `left.yaml` / `right.yaml` 的内参只对这台相机和该分辨率成立；
- `body_T_cam0/1` 是**相机到相机 IMU** 的外参，不是相机到 Go2 机身的安装外参；
- 相机到 Go2 `base_link` 的安装变换 `T_base_cam` 必须另行测量/标定，本目录没有该值；转弯时还要处理相机 IMU 原点到机身参考点的杠杆臂速度差；
- 配置保留 `estimate_extrinsic: 1`、`estimate_td: 1`；历史 A/B 回放只冻结外参做诊断，**没有**把冻结外参设为生产配置，也没有改 td、噪声、频率或标定；
- `output_path` / `pose_graph_save_path` 现在是相对于启动 `vins_node` 时工作目录的路径。每次现场应复制配置到独立运行目录并显式改写这两个路径；`cam0_calib`/`cam1_calib` 保持相对文件名，不要改成绝对路径（当前移植实现会再拼接配置目录）。

运行前确认话题名和 QoS，按实际发布端选择 `VINS_IMAGE_QOS`。记录用的 QoS 覆盖示例见 [`field_record_qos_best_effort.yaml`](field_record_qos_best_effort.yaml)；其中的话题名必须与现场 `ros2 topic list` 一致，不能当作所有设备的通用名称。不要在未确认相机传输、时间对齐和初始化状态前接入运动控制。

`udev/99-realsense-d435i-local.rules` 是历史本机 USB 权限资产，本轮**未安装**；不要在未确认设备 ID、权限和系统策略前覆盖系统规则。完整现场链路的步骤见 [CYCLONE_FIELD_CHECK.md](CYCLONE_FIELD_CHECK.md)；历史构建与 IMU 告警结论见 [2026-11-08 记录](../../docs/history/20261108_VINS_BUILD_AND_IMU_REVIEW.md)。

## 7. 当前状态

- 已做：固定提交、补丁顺序、独立目录构建命令、配置路径去个人化、标定/外参边界说明。
- 未做/未验证：本轮未重新下载或构建 VINS；未验证全新机器依赖安装；未进行新的相机或实机运行；未证明定位精度、动态速度或 5 cm 到点能力。
- 本目录不包含 VINS 上游源码、系统包、模型、原始 bag 或构建产物。
