# 机身状态与速度命令接口（v1，冻结）

状态：T1 冻结版本，2026-11-09。实现位于 `scripts/go2_nav_core/`，只依赖 C++17 标准库，不依赖 ROS、DDS、Eigen 或 Unitree SDK。

本文冻结**进程内数据结构与数学语义**；ROS 话题、共享内存或其他传输方式尚未确定。后续消费者必须先按本文实现，改变字段或语义需要单独评审。

## 1. 坐标系与参考点

### 1.1 base_link

`base_link` 是固定在 Go2 机身上的坐标系：

- X 向前，Y 向左，Z 向上；
- 绕 Z 轴左转（逆时针）为正；
- 原点固定在机身上，不随腿部动作重新定义，不是全身重心；
- 位置、速度、航向和现场测量使用同一个参考点。

本仓库把 `base_link` 定义为宇树官方 `unitree_ros` 的 Go2 模型根链接 `base` 的坐标系：

- 仓库：<https://github.com/unitreerobotics/unitree_ros>
- 固定核对提交：`41dec58c7a3ccb3050243ef16ac2ca74b2b0f9ca`
- 文件：`robots/go2_description/urdf/go2_description.urdf`
- 该模型只有根链接 `base`，**没有** `trunk` 或 `base_link`；
- `base` 的 visual/collision origin 为 `(0, 0, 0)`，visual mesh 为 `package://go2_description/dae/base.dae`；
- `base` 的 inertial origin/COM 为 `(0.021112, 0, -0.005366)`，**不是**导航参考点。

有些 ROS 2 描述使用 `base_link → trunk` 的固定连接。那些模型不是上述官方模型的同一命名；如使用，必须记录具体仓库、提交和 joint origin，不能假设 `base_link`、`base`、`trunk` 是同一点。`base_link` 原点的物理位置仍需按 [测量清单](MEASUREMENT_CHECKLIST.md) 现场确认；本接口不把模型原点当作已测量的机身物理点。

### 1.2 nav

任务开始时建立固定导航参考系 `nav`：

- 原点：初始 `base_link` 原点的水平投影；
- +X：初始机身前向的水平投影；
- +Z：竖直向上；
- 建立后不随机器人移动或转向改变。

本模块的垂直基准说明：当前没有地面高度估计，`nav` 的 z=0 取初始 `base_link` 高度所在的水平面。因此 `position_m.z` 是相对初始机身高度的高度，不是实测地面高度；水平 x/y 和 5 cm 水平指标不受影响。若现场需要地面高度，另行测量并记录，不在此模块内猜测。

建立顺序：先由初始相机 IMU 位姿和 `T_base_cam` 求出初始 `base_link` 位姿，再用它建立 `NavFrame`；之后所有 `BodyState` 都由同一个 `NavFrame` 和同一个 `T_base_cam` 产生。初始位姿的 roll/pitch 只用于求航向投影，`nav` 的 Z 始终取竖直向上。

定位重新初始化、重置或参考系不连续变化后，不得沿用旧 `nav` 目标和旧 `BodyState`；停止任务并重新建立参考系。

## 2. BodyState

```cpp
struct BodyState {
  double stamp_s;          // 状态时间戳，s
  Vec3 position_m;         // base_link 原点在 nav 中的位置，m
  double yaw_rad;          // base_link +X 相对 nav +X，左转为正，rad
  Vec2 velocity_mps;       // base_link 原点的速度，表达在 base_link：vx 前、vy 左，m/s
  double yaw_rate_radps;   // base_link 绕 Z 角速度，左转为正，rad/s
  bool valid;              // 是否可用于闭环
};
```

固定语义：

- `position_m` 的 x/y 是水平位置；z 的基准见 1.2。
- `velocity_mps` 是**同一 `base_link` 原点**的速度，表达在 `base_link` 中，不是相机速度，也不是 nav 系速度。需要 nav 系速度时由消费者按 `yaw_rad` 旋转。
- `yaw_rad` 只表示航向；横滚/俯仰不进入此接口。`yaw_rate_radps` 是机体系角速度的 Z 分量。
- `stamp_s` 与调用者的 `now_s` 必须使用同一时钟域。模块不转换 VINS header 时间、电脑接收时间、bag 时间或 Orin 时间；混用时钟会直接导致超时判定错误。
- `valid=false` 时，数值可能仍是最后一次变换结果或零值，消费者不得用于控制。

## 3. VelocityCommand

```cpp
struct VelocityCommand {
  double stamp_s;        // 请求时间，s
  double vx_mps;         // 机身前向速度，前进为正
  double vy_mps;         // 机身左向速度，左为正
  double vyaw_radps;     // 机身转向角速度，左转为正
  bool stop_request;     // true 表示请求停止
};
```

A1 正常跟踪应尽量使用 `vx` 和 `vyaw`，不以持续侧移 `vy` 作为到点手段。`stop_request=true` 后不得因旧任务自动恢复运动。运动限幅、10 Hz 重发、状态超时和 `StopMove` 仍由 Go2 接口模块负责，本模块不发送命令。

## 4. T_base_cam 方向

`T_base_cam` 把相机 IMU 坐标系中的点变换到 `base_link`：

```text
p_base = R_base_cam * p_cam + t_base_cam
```

- `R_base_cam`：3×3 旋转矩阵，把相机系向量映射到 `base_link`；其列是相机三轴在 `base_link` 中的方向。
- `t_base_cam`：相机 IMU 原点在 `base_link` 中的位置，m。
- 反变换：`R_cam_base = R_base_cam^T`，`t_cam_base = -R_base_cam^T * t_base_cam`。
- `measured=false`（默认）表示该变换尚未实测/标定，**不得用于机器人控制**。
- 本仓库不保存生产 `T_base_cam` 数值；测试只用合成值。

不要把两组变换混为一谈：

1. **相机—相机 IMU** 外参：VINS 配置中的 `body_T_cam0/1`，描述双目相机到相机 IMU 的关系；
2. **相机—Go2 机身** 安装外参：本文的 `T_base_cam`，描述相机 IMU 到 `base_link` 的关系。

第二组必须另行测量，见 [测量清单](MEASUREMENT_CHECKLIST.md)。

## 5. 变换公式

输入相机 IMU 状态：

- `R_world_camera`、`p_world_camera`：相机 IMU 位姿，父系为 VINS world；
- `v_camera`、`w_camera`：相机 IMU 的 body twist，均表达在相机 IMU 系。

输出 `base_link` 位姿和速度：

```text
R_world_base = R_world_camera * R_base_cam^T
p_world_base = p_world_camera - R_world_base * t_base_cam

w_base = R_base_cam * w_camera
v_base = R_base_cam * v_camera - cross(w_base, t_base_cam)
```

其中 `v_base` 是 `base_link` 原点的速度，表达在 `base_link`；`w_base` 是机身角速度，表达在 `base_link`。第二项是相机原点到机身参考点的杠杆臂补偿。只旋转速度而不补偿杠杆臂会在转弯时把相机速度误当机身速度。

导航系：

```text
R_nav_world = R_world_nav^T
p_nav = R_nav_world * (p_world_base - origin_world)
R_nav_base = R_nav_world * R_world_base
yaw_nav = atan2(R_nav_base(1,0), R_nav_base(0,0))
```

## 6. 相对目标一次性转换

目标 `(forward_m, left_m)` 在下令时按当时的 `BodyState` 转换：

```text
target_nav.x = position_m.x + cos(yaw_rad) * forward_m - sin(yaw_rad) * left_m
target_nav.y = position_m.y + sin(yaw_rad) * forward_m + cos(yaw_rad) * left_m
target_nav.z = position_m.z
```

该函数只在接收命令时调用一次。之后机器人转向或移动，不得用新的当前航向重新计算同一个目标；否则目标会随机器人移动。实现为纯函数 `relativeTargetToNav()`。

## 7. valid 规则

`BodyState.valid=true` 需要同时满足：

1. 上游 `valid_input` 为真；
2. 时间戳、位姿、旋转矩阵和 twist 全部有限；
3. `now_s - stamp_s <= max_age_s`，且未来偏差不超过 `max_future_skew_s`；
4. 时间戳不小于上一次已接受状态（回退视为重置）；
5. 相对上一次已接受状态，位置跳变不超过 `max_position_jump_m`，航向跳变不超过 `max_yaw_jump_rad`；
6. 变换后的位置、航向、速度、角速度全部有限。

默认值（初始离线占位，不是现场验证过的安全阈值）：

| 参数 | 默认值 |
|---|---:|
| `max_age_s` | 0.5 s |
| `max_position_jump_m` | 0.25 m |
| `max_yaw_jump_rad` | 30° |
| `max_future_skew_s` | 0.05 s |

任一条件失败时返回 `valid=false`，且**不更新**上一次已接受状态；消费者应停止任务。源端确认重置后，调用 `BodyStateEstimator::reset()` 再恢复。`valid` 只表示定位状态有效性，不表示 `T_base_cam` 已测量；控制层必须同时检查 `Extrinsic.measured`。

## 8. 本任务不包含

- 不接 VINS 实时数据、ROS 话题或 DDS；
- 不做路径规划、Pure Pursuit、终点接近；
- 不修改旧 `goto`/`yaw` 行为；
- 不做实机测试、5 cm 验收或 VINS 参数调整；
- 不定义传输层、时间同步策略或线程模型。
