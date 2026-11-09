# base_link 与 T_base_cam 测量清单

状态：测量方法清单，2026-11-09。**当前 `base_link` 物理原点未现场确认，`T_base_cam` 未实测。**

未完成本清单前：

- 不得把猜测的 `T_base_cam` 写入生产配置；
- 不得用未测量的变换做机器人控制；
- 不得把 VINS 相机 IMU 位姿直接当成 `base_link` 位姿；
- 不得用全身重心、机头或脚尖代替 `base_link` 参考点做 5 cm 评价。

## 1. base_link：模型事实与命名歧义

已核对的官方模型：

- 仓库：<https://github.com/unitreerobotics/unitree_ros>
- 提交：`41dec58c7a3ccb3050243ef16ac2ca74b2b0f9ca`
- 文件：`robots/go2_description/urdf/go2_description.urdf`
- 根链接：`base`；**没有** `trunk` 或 `base_link`。
- `base` 的 visual/collision origin：`(0, 0, 0)`。
- `base` 的 visual mesh：`package://go2_description/dae/base.dae`。
- `base` 的 inertial origin/COM：`(0.021112, 0, -0.005366)`；这是质心参数，不是导航参考点。
- 头部、IMU、雷达、前相机和四条腿均挂在 `base` 下；例如 `imu_joint` origin 为 `(-0.02557, 0, 0.04232)`。

歧义必须记录，不能自动合并：

- 某些 ROS 2 描述使用 `base_link → trunk` 的固定连接；另一些使用 `base_footprint → base`。它们不是同一个命名模型。
- 如果使用其他仓库/模型，必须记录仓库、提交、链接名和 joint origin；不能假设 `base_link`、`base`、`trunk` 是同一个物理点。
- 本仓库把 `base_link` 定义为上述官方 `base` 坐标系；**该坐标系的物理位置仍需现场确认**，不能只凭 URDF 名称下结论。

## 2. base_link 物理原点确认方法

目标：确认官方 `base` 坐标系原点在实物上的位置，并建立可重复的测量基准。

建议步骤：

1. 打印并保存官方 URDF 的 `<link name="base">` 和与机身连接的 joint；记录仓库提交。
2. 选择可重复识别的机身特征作为辅助基准，例如前/后髋关节中心、机身壳体对称面、IMU 安装座；不要用机头外壳尖端或脚尖作为唯一点。
3. 让机器人站在平坦地面、已知姿态；用卷尺/直尺/激光测距从上述特征测到候选原点在水平面内的投影。
4. 与 URDF 中 `base` 到各 joint 的偏移交叉核对（例如 hip joint、`imu_joint`）；记录偏差。
5. 在机身或工装上做可重复标记；拍照并记录相机/测量方向。
6. 若无法从官方模型和实物特征确定同一点，停止，记录歧义；不得用“大概在机身中心”继续。

记录字段：

| 字段 | 内容 |
|---|---|
| 日期 / 操作者 | |
| 机器人型号/编号 | Go2 EDU，序列号 |
| 模型仓库/提交 | `unitree_ros` / `41dec58c...` 或实际使用的模型 |
| 链接名 | `base` / `base_link` / `trunk`，以及父子链 |
| 所选物理基准 | 特征名称和位置 |
| 测量方法 | 卷尺、卡尺、激光、工装等 |
| 候选原点相对基准的 x/y/z | m，注明方向和符号 |
| 测量不确定度 | m |
| 与 URDF 交叉核对结果 | 偏差和解释 |
| 结论 | 已确认 / 有歧义 / 停止 |

## 3. T_base_cam 测量/标定

先区分两组变换：

1. **相机—相机 IMU**：VINS 配置的 `body_T_cam0` / `body_T_cam1`，描述双目相机到相机 IMU；不是本清单的 `T_base_cam`。
2. **相机 IMU—Go2 base_link**：本清单要求测量的 `T_base_cam`，满足：

```text
p_base = R_base_cam * p_cam + t_base_cam
```

- `R_base_cam`：相机系到 `base_link` 的旋转，列是相机轴在 `base_link` 中的方向；
- `t_base_cam`：相机 IMU 原点在 `base_link` 中的位置，m。

可选测量方法：

- **机械/CAD 测量（优先用于固定安装）**：测量相机安装面、IMU 芯片/相机原点相对 `base_link` 的平移和三轴方向；记录工装和基准。
- **标定板/手眼标定**：相机看固定标定板，机器人/相机做已知运动，求相机到机身的刚体变换；记录残差和运动范围。
- **已知旋转台/工装**：用可重复的纯旋转和纯平移检查方向和杠杆臂符号。

无论哪种方法，必须记录：

| 字段 | 内容 |
|---|---|
| 日期 / 操作者 | |
| 相机型号/序列号 | D435i，序列号 |
| 安装位置/工装 | 相机朝向、支架、减震件 |
| base_link 定义 | 使用的模型/提交/链接名；引用第 2 节记录 |
| `R_base_cam` | 3×3，行主序；注明约定 |
| `t_base_cam` | x/y/z，m |
| 单位与符号 | m、rad；左转/上为正等 |
| 测量/标定方法 | 机械、标定板、手眼等 |
| 残差/重复性 | 平移 m、旋转 deg |
| 不确定度 | 平移 m、旋转 deg |
| `measured` | true/false；只有完成并复核后为 true |
| 备注 | 温度、拆装、线缆、镜头等 |

## 4. 离线 sanity check

在写入 `measured=true` 前，至少检查：

1. **单位变换**：相机与机身重合时，位姿和速度应一致。
2. **纯平移**：只改变 `t_base_cam` 时，位置偏移方向和大小符合手算。
3. **纯旋转**：只改变 `R_base_cam` 时，方向符号符合右手系；含小 roll/pitch 时仍一致。
4. **杠杆臂**：机身原点静止、相机绕机身转时，算出的机身线速度接近 0；角速度保留。
5. **组合变换**：平移+旋转与二维手算一致。
6. **相对目标**：非零初始航向下只转换一次；机器人转向后目标不移动。
7. **valid**：非有限值、超时、时间戳回退、位置/航向跳变均返回 `valid=false`。

合成值只用于测试，不是生产标定。测试命令：

```bash
cmake -S scripts/go2_nav_core -B build/go2_nav_core \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/go2_nav_core --parallel 2
./build/go2_nav_core/go2_nav_core_selftest
```

## 5. 完成与未完成边界

- 完成第 2 节记录且复核：`base_link` 物理原点可标记为“已确认”。
- 完成第 3 节记录且离线 sanity check 通过：可把对应 `Extrinsic.measured` 标为 true；仍不等于实机定位精度通过。
- 未完成：`T_base_cam` 未实测、`base_link` 原点未现场确认、传输方式未定。不得用猜测值做控制。
- 下一步建议：用已有 bag 做 VINS 位姿离线回放，验证本接口的位姿、速度和 valid 行为；不在此任务中接实时 VINS 或改参数。
