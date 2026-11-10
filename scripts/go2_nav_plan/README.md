# go2_nav_plan：VINS→base_link 与 A→B 路径/跟踪离线原型

状态：离线原型，2026-11-09。只依赖 C++17 标准库和 `go2_nav_core`；不接 VINS 实时数据、DDS、机器人或相机。原生工具 `scripts/go2_native_motion/` 已通过 CMake 链接本模块的路径和跟踪库，但当前只用原生 SportModeState 位姿做机制验证。

## 内容

| 文件 | 作用 |
|---|---|
| `vins_replay.*` | 读取 VINS odometry CSV（相机 IMU 位姿 + body twist），经 `T_base_cam` 转成 `BodyState` |
| `path.*` | 直线和 Dubins（六词，选最短）路径，带弧长参数化、投影和曲率 |
| `pure_pursuit.*` | 标准 Pure Pursuit + 曲率/终点速度调节；输出 `vx/vyaw`，`vy=0` |
| `demo_main.cpp` | A→B 直线和 Dubins 曲线的离线运动学模拟 |
| `selftest.cpp` | 路径端点、投影、跟踪、目标语义、VINS 四元数/变换/CSV 回放 |

## 算法来源

- Dubins 六词公式和采样方式参考 **PythonRobotics**（MIT，Atsushi Sakai）：<https://github.com/AtsushiSakai/PythonRobotics>。本仓库用 C++ 重写，没有复制完整文件；推导另参考 Shkel & Lumelsky 的 Dubins set 分类。
- Pure Pursuit 采用标准纯跟踪公式；**Nav2 Regulated Pure Pursuit**（Apache-2.0）作为后续复用候选，本模块未复制其代码。
- 复用第三方代码前按 `THIRD_PARTY.md` 记录来源和许可证。

## 构建与测试

```bash
cmake -S scripts/go2_nav_plan -B build/go2_nav_plan \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/go2_nav_plan --parallel 2
./build/go2_nav_plan/go2_nav_plan_selftest
./build/go2_nav_plan/go2_nav_plan_demo
```

自检和演示不初始化 DDS，不连接机器人或相机。

## 用历史 VINS CSV 回放

```bash
./build/go2_nav_plan/go2_vins_replay \
  --csv /path/to/odometry.csv \
  --out /tmp/go2_vins_body.csv
```

当前现场临时外参（**未标定**，只用于离线）：

```bash
./build/go2_nav_plan/go2_vins_replay \
  --csv /path/to/odometry.csv \
  --config scripts/go2_nav_plan/config/d435i_provisional.conf
```

`config/d435i_provisional.conf` 的值来自 `docs/MEASUREMENT_CHECKLIST.md`：相机中心前向 0.3475 m、左右 0、向上 0.0525 m；仰角未测，`measured=0`。`--camera-optical` 使用相机光学系到 `base_link` 的标称旋转（相机 X 右、Y 下、Z 前 → 机身 X 前、Y 左、Z 上）；`--yaw-deg/--pitch-deg/--roll-deg` 可再加安装角。

## 从现场 bag 提取 VINS CSV

现场用 `ros2 bag record` 记录 `/odometry`，建议同时记录 `/imu_propagate` 和 `/camera/camera/imu`。回来后：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/go2_nav_plan/tools/extract_vins_bag.py \
  --bag /path/to/bag_dir \
  --topic /odometry \
  --out /tmp/odometry.csv
```

- 必须用 `/usr/bin/python3`；Anaconda 的 Python 3.13 加载不了 Humble 的 `rclpy` 扩展。
- 脚本只读 SQLite，不创建 ROS 节点，不初始化 DDS。
- 已在历史 bag 上验证：1341 行，输出与原审阅 CSV 逐字节一致。
- `/imu_propagate` 也可以用 `--topic` 提取，但控制先用优化后的 `/odometry`。

CSV 列约定：

```text
bag_ns,header_s,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id
```

- 默认 `T_base_cam` 为单位变换且 `measured=false`；此时输出是**相机 IMU 参考点**，不是实测机身点。
- 可在命令行给 `--tx --ty --tz --yaw-deg` 做简单平移/航向检查，但仍不是完整安装外参标定。
- 回放使用宽松的 validity 阈值，避免历史大跳变全部被判无效；生产阈值另行确定，不沿用这里的设置。
- 历史 CSV 只作只读测试输入，不作为本仓库依赖。

## 当前离线结果

- 直线 A→B 1.0 m：模拟停稳，位置误差约 2 cm，`vy=0`。
- Dubins A→B（1.0 m / 1.0 m，终点航向 π/2）：模拟停稳，位置误差约 2 cm，`vy=0`；终点航向尚未单独控制，航向误差约 0.25 rad。
- 历史 VINS CSV（单位外参）：1341 行、1341 行有效；末帧约 `(0.164, 1.167)` m；最大相邻跳变约 9.6 cm（已知优化重锚定）；最大机身速度约 0.26 m/s。
- 同一历史 CSV（临时外参 `0.3475/0/0.0525` + 光学旋转）：1341 行有效；末帧约 `(1.164, -0.204)` m；最大相邻跳变约 10.4 cm。这验证了相机→机身的转换链，不是精度证明。
- 闭环离线测试（相机→机身→Pure Pursuit→理想独轮车）：1.0 m 直走最终位置误差约 1.99 cm，停稳，`vy=0`。

## 离线验收清单

已通过：

- C++17 构建，不依赖 ROS、DDS 或 Unitree SDK。
- 相机 IMU→`base_link` 的平移、旋转和杠杆臂单元测试。
- 端到端合成测试：已知 `base_link` 轨迹 → 生成相机位姿/twist → 转回 `base_link`，位置、航向、速度误差 <1e-6。
- 闭环相机→机身→跟踪测试：1.0 m 直走最终位置误差约 1.99 cm，停稳，`vy=0`。
- VINS CSV 解析和历史数据回放。
- 现场 bag→CSV 提取脚本已在历史 bag 上验证，输出与历史审阅 CSV 逐字节一致。
- 直线/Dubins 路径端点、投影和曲率。
- Pure Pursuit A→B 模拟：直线和曲线均停稳，位置误差约 2 cm，`vy=0`。
- 相对目标一次性转换和 `valid` 规则。

待现场/实时验证：

- 真实 `T_base_cam` 的 yaw/pitch/roll 和相机 IMU 偏移；当前按水平正前方、相机外壳中心。
- D435i/VINS 相对卷尺的重复性和停后稳定性。
- 原生步态最小步长、停车距离和指令延迟。
- 实时 VINS 传输、时间同步和 `valid` 阈值。
- 实机 A→B 和 5 cm 验收。

进入实机前的最低条件：

1. `T_base_cam` 的平移和朝向有独立记录，不能只用标称 0°。
2. 原生步态最小稳定步长和停车距离已实测；若 >5 cm，先报告，不进入 5 cm 验收。
3. D435i/VINS 静态和短直线误差已实测；若明显 >5 cm，先解决定位，不用控制器补。

## 限制

- `T_base_cam` 未实测，`base_link` 物理原点未现场确认。
- 模拟使用理想独轮车模型，没有建模原生步态最小速度、步长、指令延迟和停车距离。
- 当前只控制位置，不控制终点航向。
- 不是实机 5 cm 验收；验收仍需卷尺/标记独立测量。
