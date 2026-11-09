# go2_vins_bridge：VINS → base_link → UDP

独立 ROS 2 节点，把 VINS 相机 IMU 位姿经 `T_base_cam` 转成 `base_link` 的 `BodyState`，通过本机 UDP 发给 `go2_native_motion`。桥接**只发状态和 ORIGIN 响应，不发运动命令**。

本任务只做离线构建和自检；不运行 ROS 节点、不连接机器人/相机、不初始化 DDS。

## 1. 内容

| 文件 | 作用 |
|---|---|
| `bridge_core.*` | 无 ROS 的核心：VINS 样本→`BodyStateEstimator`、nav 建立、valid 原因、STATE 生成 |
| `udp_bridge_server.*` | 127.0.0.1 UDP：接收 SET_ORIGIN/PING，发送 ORIGIN 响应/STATE |
| `main.cpp` | rclcpp 节点：订阅 VINS 里程计，调用 `BridgeCore`，运行 UDP 线程和状态定时器 |
| `selftest.cpp` | 无 ROS 的单元/UDP 回环测试 |
| `CMakeLists.txt` | 无 ROS 时只构建 core/selftest；找到 rclcpp/nav_msgs 才构建节点 |

核心只依赖 C++17 标准库、`go2_nav_core` 和 `go2_nav_plan`；ROS 节点单独构建，**不链接进原生 SDK 工具**。

## 2. 构建

有 ROS 2 Humble 时：

```bash
source /opt/ros/humble/setup.bash
cmake -S scripts/go2_vins_bridge -B build/go2_vins_bridge \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/go2_vins_bridge --parallel 2
./build/go2_vins_bridge/go2_vins_bridge_selftest
./build/go2_vins_bridge/go2_vins_bridge_integration_test
```

**运行环境：** 不要在当前设置了 Anaconda `LD_LIBRARY_PATH` 的 shell 直接运行 ROS 节点；Anaconda 的 `libstdc++` 可能缺少 Humble `librclcpp.so` 需要的 `GLIBCXX_3.4.30`。先 `conda deactivate` 或使用干净的 Humble shell，再 `source /opt/ros/humble/setup.bash`。构建不受此影响，但运行前要核对。

无 ROS 时仍可构建核心和自检：

```bash
cmake -S scripts/go2_vins_bridge -B build/go2_vins_bridge_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_nav_msgs=ON
cmake --build build/go2_vins_bridge_core --parallel 2
./build/go2_vins_bridge_core/go2_vins_bridge_selftest
./build/go2_vins_bridge_core/go2_vins_bridge_integration_test
```

自检不调用 `rclcpp::init`，不初始化 DDS。

测试覆盖：

- `go2_vins_bridge_selftest`：外参配置/光学旋转、无 nav/无 VINS、origin、位置/航向、STALE、JUMP、RESET、NONFINITE、UDP 服务端回环。
- `go2_vins_bridge_integration_test`：合成 VINS → `BridgeCore` → UDP 服务端 → 原生 `VinsUdpClient` → `runInteractiveSession`（假 Go2 Backend）；检查 origin 成功、plan 发出 `vy=0` Move、VINS 停流后 `VINS_INVALID` 停车、停止后无 `cmd`。
- 两者都不链接 ROS/SDK、不初始化 DDS。

## 3. 启动顺序

1. 启动 VINS（相机/VINS 链路按 `scripts/go2_vins/` 与 `CYCLONE_FIELD_CHECK.md` 的步骤；本任务不执行）。
2. 启动桥接：

```bash
source /opt/ros/humble/setup.bash
./build/go2_vins_bridge/go2_vins_bridge \
  --config scripts/go2_nav_plan/config/d435i_provisional.conf \
  --vins-topic /vins/odometry \
  --control-port 46001 --state-port 46002
```

3. 启动原生工具：

```bash
./build/native_motion-sdk/go2_native_motion \
  --control-source vins \
  --vins-control-port 46001 --vins-state-port 46002 \
  --vins-timeout-s 0.5 \
  --domain "$DDS_DOMAIN" --interface "$GO2_NIC" \
  --csv ...
```

4. 在原生工具中确认状态新鲜后输入 `origin`；原生工具向桥接发送 `SET_ORIGIN`，桥接用当前 VINS `base_link` 位姿建立 nav 并回复 `ORIGIN_OK`。失败时 `ORIGIN_FAIL <reason>`，原生工具不进入运动。

桥接可以先启动，也可以后启动；`SET_ORIGIN` 必须有有效 VINS 样本才能成功。

## 4. UDP 协议（第一版文本）

只使用 `127.0.0.1`。一个消息一个 UDP datagram；行尾 `\n` 可省略。

原生→桥接（控制端口，默认 46001）：

```text
SET_ORIGIN
PING
```

桥接→原生（状态端口，默认 46002）：

```text
ORIGIN_OK
ORIGIN_FAIL <reason>
STATE <seq> <stamp> <x> <y> <z> <yaw> <valid> <reason>
```

- `seq`：状态序号，uint64。
- `stamp`：桥接电脑的 `CLOCK_MONOTONIC` 秒。
- `x/y/z`：`base_link` 原点在桥接 nav 系中的位置，m。
- `yaw`：`base_link` +X 相对 nav +X，左转为正，rad。
- `valid`：`1` 或 `0`。
- `reason`：`OK`、`NO_NAV`、`NO_VINS`、`STALE`、`NONFINITE`、`JUMP`、`RESET`。
- `valid=1` 时 reason 必须为 `OK`；`valid=0` 时 reason 不能为 `OK`。

`PING` 的响应是一条当前 `STATE`（不额外定义 `PONG`）。

## 5. 时间语义

- 桥接的 `STATE.stamp` 和 VINS 样本的接收时间都取同一台电脑的 `CLOCK_MONOTONIC`。
- VINS `header.stamp` 来自相机/Orin 时钟，单独记录和打印，**不与 PC 单调时钟比较**。
- 过期判定使用 PC 接收时间：超过 `--vins-timeout-s` 没有有效样本即 `STALE`。
- 桥接检测 VINS header 回退：视为 `RESET`，nav 失效，原生工具必须停止并重新确认；不会沿用旧目标。
- 原生工具再用自己的 PC 单调时钟检查 STATE 包龄；两侧必须运行在同一台电脑。

## 6. valid 原因与行为

| reason | 含义 |
|---|---|
| `NO_NAV` | 尚未 SET_ORIGIN 或 nav 已因 reset 失效 |
| `NO_VINS` | 尚未收到 VINS 样本 |
| `STALE` | 超过 `--vins-timeout-s` 未收到有效样本 |
| `NONFINITE` | VINS 位姿/twist/时间戳含非有限值 |
| `JUMP` | 相对上一次已接受状态的位置/航向跳变超阈值 |
| `RESET` | VINS header 时间回退，nav 已失效 |

`RESET`/`JUMP`/`STALE` 后原生工具立即停止，不回退到原生 `SportModeState` 继续控制。恢复需要重新确认 VINS 并重新 `origin`（通常重启会话）。

## 7. 外参与测量边界

- 默认读取 `scripts/go2_nav_plan/config/d435i_provisional.conf`；当前 `measured=0`，只是临时外参。
- 启动日志会打印 `measured=0/1`；`measured=0` 时**不得用于实机控制或 5 cm 验收**。
- `body_T_cam0/1` 是相机到相机 IMU 的 VINS 外参；桥接的 `T_base_cam` 是相机 IMU 到 Go2 `base_link` 的安装外参，两者不同。
- 正式控制前必须按 `docs/MEASUREMENT_CHECKLIST.md` 实测 `T_base_cam` 和物理 `base_link`。

## 8. 未验证

- 未在本任务运行桥接节点或 VINS；未接实时相机/机器人。
- 未验证 VINS header 与 PC 时间的现场偏差、实时 UDP 丢包和 stop 延迟。
- 临时外参、`base_link` 物理原点、D435i/VINS 精度和 5 cm 均未验收。
- `--imu-topic` 可选；若同时订阅 `/vins/odometry` 和 `/vins/imu_propagate`，当前实现是“最新样本覆盖”，不是融合。默认不订阅 propagate。
- 节点使用 `SensorDataQoS`（best-effort 订阅），可兼容 reliable/best-effort 发布端；若收不到数据，先用 `ros2 topic info -v` 核对实际 QoS 和话题名。
