# 项目状态与下一步

> 每次任务完成后更新本文档。最后更新：2026-11-09

## 当前阶段

起步阶段。工程已入 GitHub；已有原生运动工具、VINS 候选配置和离线自检。`scripts/go2_nav_core/` 提供机身状态接口，`scripts/go2_nav_plan/` 提供 VINS CSV→`BodyState` 回放、Dubins 路径和 Pure Pursuit 离线原型；原生工具已接入 `pulse`、`plan`、路径+Pure Pursuit（`vy=0`）和 `--control-source vins`；`scripts/go2_vins_bridge/` 提供离线 ROS 桥/UDP 状态链。当前里程碑是 D435i/VINS 定位下的 A→B 5 cm，实机尚未实现。

## 协作方式

按定位与坐标、路径与跟踪、接口与实验三条工作流共享责任，不固定人员。日常只使用 `Lidio` 分支直接提交/推送，不新开功能分支、不逐任务建 PR；先冻结接口，再各自实现；实机只保留一个运动命令发送入口。

## 已完成

- 2026-11-09：初始化 GitHub 仓库并推送 `main`，提交 `63f3294`，32 个文件。
- 2026-11-09：T1 机身状态接口与相机 IMU→`base_link` 离线变换；模块 `scripts/go2_nav_core/`；接口冻结在 `docs/INTERFACE.md`，测量清单在 `docs/MEASUREMENT_CHECKLIST.md`；离线构建和自检通过；`T_base_cam` 仍为未实测，测试只用合成值。
- 2026-11-09：用临时外参 `(0.3475, 0, 0.0525)` + 相机光学旋转回放历史 VINS CSV；1341 行全部转换；`base_link` 末帧约 `(1.164, -0.204)` m；最大相邻跳变约 10.5 cm；仍是临时外参，不是 5 cm 证明。
- 2026-11-09：原生工具接入短脉冲 `pulse` 和 `plan`；`goto` 改为一次生成的路径 + Pure Pursuit，删除侧移控制，所有正常运动 `vy=0`；离线自检、无 SDK/SDK 构建通过。仍是原生日姿机制验证，不是 VINS/5 cm。
- 2026-11-09：VINS 实时桥接离线实现；`scripts/go2_vins_bridge/` 提供 VINS 样本→`base_link`→UDP、origin 握手和 valid 原因；原生工具新增 `--control-source vins`，plan/goto/yaw 可用 VINS 状态，valid=0/包超时停止且不回退原生位置；桥接/原生/UDP 自检通过。未运行 ROS 节点或真实 VINS。
- 2026-11-09：STATE 增加 `vx/vy/wz` 和机器可读 `measured`；原生 vins 模式默认拒绝 `measured=0`，只有 `--allow-unmeasured-extrinsic` 可覆盖；CSV 增加 `vins_vx/vy/wz`、`vins_measured`、`measured_override`；离线自检通过。
- 工程独立化：目录、文档、去个人路径、构建选项已整理；无 SDK 构建和自检、SDK 构建、`ON` 缺 SDK 失败均有记录。本轮未重跑构建。
- 原生工具：`Move/StopMove`、10 Hz 重发、状态超时停止、停止失败锁存、CSV、离线自检。
- VINS：固定提交、补丁、480×270 候选配置、独立构建说明。
- 历史验证：三轮慢走、A/B 外参回放、IMU 和 StopMove 审阅；只作追溯。

## 离线已确定

- 相机 IMU→`base_link` 的平移、旋转和杠杆臂公式有单元测试；端到端合成测试用已知机身轨迹生成相机位姿/twist，再转回机身，位置/速度误差 0、航向误差约 7e-17 rad。
- 闭环离线测试：相机→机身→Pure Pursuit→理想独轮车，1.0 m 直走最终位置误差约 1.99 cm，停稳，`vy=0`。
- VINS CSV 回放可用；现场 bag→CSV 提取脚本已在历史 bag 上验证，输出与历史审阅 CSV 逐字节一致。
- 临时外参 `(0.3475, 0, 0.0525)` + 相机光学旋转能跑通历史数据（1341 行）。
- 原生工具 `pulse`/`plan`/`goto` 离线自检通过；`straight`、`pulse`、`goto`、`plan` 的 Move 和 CSV `cmd_vy_mps` 均为 0；模拟到达/超时/状态超期/停止失败锁存均通过。
- VINS 桥接核心/UDP 自检通过：`SET_ORIGIN`/`PING`、`ORIGIN_OK/FAIL`、`STATE` 编解码，`NO_NAV`/`NO_VINS`/`STALE`/`NONFINITE`/`JUMP`/`RESET`，UDP 回环。
- 原生 vins 模式自检通过：origin 成功/失败、VINS 驱动 plan、valid=0/包超时立即 `VINS_INVALID` 停止、不回退、`straight/pulse` 拒绝、`vy=0`。
- STATE 携带 `vx/vy/wz` 和 `measured`；VINS `vx/vy/wz` 进入 Pure Pursuit；`measured=0` 默认拒绝 origin/plan/goto，`--allow-unmeasured-extrinsic` 覆盖时 CSV 标记 `vins_measured=0`/`measured_override=1`。
- 直线/Dubins 路径和 Pure Pursuit 离线模拟可用；A→B 直线和曲线均停稳，位置误差约 2 cm，`vy=0`。
- 相对目标一次性转换和 `valid` 规则有离线测试。
- 临时外参放在 `scripts/go2_nav_plan/config/d435i_provisional.conf`，`measured=0`。

## 离线仍未确定

- 真实 `T_base_cam` 的 yaw/pitch/roll 和相机 IMU 偏移；当前按水平正前方、相机外壳中心。
- D435i/VINS 相对卷尺的重复性、尺度和停后稳定性。
- 原生步态最小步长、停车距离和指令延迟。
- 实时 VINS 传输、时间同步和现场 `valid` 阈值。
- 实机 A→B 和 5 cm 验收。

## 未完成 / 未验证

- `base_link` 物理原点未现场确认；`T_base_cam` 未测量。
- 机身状态接口已接离线 CSV 回放和离线 VINS/UDP 桥；未运行真实 ROS/VINS，现场时间同步、丢包和 valid 阈值未验证。
- 原生工具 `plan`/`goto`/`yaw` 在 `--control-source vins` 下使用 VINS；native 模式仍用原生 SportModeState；实机未验证。
- 离线已有直线/Dubins + Pure Pursuit；终点航向未控制，实机未验证。
- 单一发送入口仍是原生工具；VINS/UDP 已离线接入，但真实 VINS 与运动工具未实机联调。
- 实机 `goto 0.5 0`、`pulse`、`plan` 未执行；0.30 m/s 连续行走只是用户报告；实际停稳未验证。
- 5 cm 未验收。

## 已知问题与边界

- VINS 第 1 轮出现 2.875 m/约 1 m 跳变；第 2/3 轮约 1.2 m；A/B 回放冻结外参后稳定，但未复现现场异常。冻结外参只是诊断，不是生产配置。
- VINS 停后仍有约 1.9–9.5 cm 变化；IMU 使用默认标定。
- `Move` 返回 0 只表示发送成功；`StopMove` 返回 0 不代表实际停稳。
- 首期只在平地、开阔、低速验证，不声称避障。
- 历史文档保留 `/home/lidio`、`192.168.123.x` 等原路径，仅作追溯。

## 当前里程碑

**M1：机器人从 A 到 B，`base_link` 原点停稳后的实际水平误差 ≤5 cm。**

- A：初始 `base_link` 原点的地面投影。
- B：固定地面点。首期先做正前方 0.5–1.0 m 直线，再扩展到侧向、斜向。
- 定位：**D435i 双目 + 相机 IMU → VINS → `base_link`**，使用已冻结的 `BodyState` 接口。控制和 5 cm 验收都使用这条链路，不用原生 `SportModeState` 位置反馈替代；原生反馈只作对照。
- 路径与跟踪：直接复用现有算法。首期直线；需要转向时用 Dubins 路径 + Pure Pursuit/Regulated Pure Pursuit。
- 验收：卷尺、地面标记、视频独立测量；不以控制器估计容差代替。

### 5 cm 的前置条件与风险

1. **D435i 相机 + VINS 的机身定位误差必须明显小于 5 cm。** 历史数据只到“量级接近”，还不能证明 5 cm。控制与验收必须使用 D435i/VINS 链路；原生 `SportModeState` 只作对照，不能替代。
2. **原生步态的最小稳定步长和停车距离必须小于 5 cm。** 0.30 m/s 能走只是用户报告，尚未实测。如果最小步长或停车距离本身大于 5 cm，当前原生 SportClient 控制下达不到 5 cm，必须停止并报告，不能靠缩小容差或删失败记录制造通过。
3. **`base_link` 原点和 `T_base_cam` 必须现场确认/测量。** 模型定义已明确，但物理点和安装变换仍未实测。
4. **测量重复性要好于 5 cm。** 用同一 `base_link` 物理基准和同一套地面标记，测量不确定度目标 ≤1 cm。

## 任务目标（M1）

### M1-0 5 cm 可行性标定（现场，最先）

- 目标：实测原生低速最小稳定步长、停车距离、指令延迟；实测 D435i/VINS 静态和短直线相对卷尺的误差。
- 交付：0.20/0.25/0.30 m/s 等候选速度的重复记录；最小步长、停车距离、延迟、定位误差和重复性。
- 完成：给出明确数值，判断当前原生控制下 5 cm 是否可达。
- 停止：若最小步长或停车距离明显大于 5 cm，停止并报告，不修改验收标准。

### M1-1 定位链（现场 + 离线）

- 目标：确认 `base_link` 物理原点，测量 `T_base_cam`，把 D435i 相机 + VINS 输出接成 `BodyState`；用已有 bag 做离线回放验证。
- 交付：测量记录、回放对照、VINS 机身位置/速度误差报告。
- 完成：`base_link` 原点可重复标记；`T_base_cam` 实测；D435i/VINS 位姿和速度语义明确。
- 状态：离线 CSV 回放已完成；离线 VINS/UDP 桥和原生 `--control-source vins` 已完成并通过合成/回环自检；未运行真实 ROS/VINS。现场部分测量：`base_link` 离地 0.33 m，相机顶面离地 0.395 m，水平向前 0.36 m 到相机最前表面（折到中心 0.3475 m）、左右居中，误差 1–2 cm；临时 `t=(0.3475, 0, 0.0525)` m，光学旋转暂按水平正前方，仰角和相机 IMU 偏移未确认，`measured=false`。
- 依赖：T1。

### M1-2 路径与跟踪离线（现有算法）

- 目标：完成 A→B 的路径生成和跟踪；正前方先走直线，需要转向时用现有 Dubins 实现；跟踪用 Pure Pursuit 或 Nav2 Regulated Pure Pursuit。
- 交付：路径生成、跟踪器、`VelocityCommand` 输出、离线模拟、A→B 到达测试。
- 完成：模拟中 A→B 稳定到达；正常跟踪 `vy≈0`；能减速停车；停止后旧任务不恢复。
- 状态：离线原型已完成；自检通过；模拟 A→B 约 2 cm；原生工具已接入路径 + Pure Pursuit；实机未验证。
- 依赖：T1；可与 M1-0、M1-1 并行。
- 算法边界：直接复用现有算法和实现，不新写通用规划器；首期不用 TEB/MPC/DWA，它们需要代价地图和较多调参，开阔 A→B 不需要。

### M1-3 单一运动发送入口整合

- 目标：把跟踪器的 `VelocityCommand` 接到现有 Go2 工具，保留 10 Hz、状态超时和停止失败锁存。
- 交付：适配层、离线自检；仍不运行实机。
- 完成：模拟命令只经过一个发送入口；停止、超时和锁存行为不变。
- 状态：原生工具已接入 `pulse`/`plan`/`goto` 的路径 + Pure Pursuit，且仍是唯一运动发送入口；`--control-source vins` 已离线接入 VINS/UDP 桥，valid=0/包超时停止且不回退原生位置；实机未运行。
- 依赖：M1-2。

### M1-4 现场 A→B 闭环

- 目标：0.5 m 直线 → 1 m 直线 → 侧向/斜向；单次运行、人工停止、独立测量。
- 交付：现场记录、CSV、视频和卷尺测量。
- 完成：实际到达并停稳；异常可终止；不靠人工补走。
- 依赖：M1-0、M1-1、M1-2、M1-3。

### M1-5 5 cm 验收

- 目标：固定条件、重复测试、独立测量，逐次报告误差和失败。
- 交付：验收报告。
- 完成：在明确条件下评价是否 ≤5 cm，不泛化到其他场景。
- 依赖：M1-4。

## 任务记录

| 日期 | 任务 | 结果 | 提交/PR | 备注 |
|---|---|---|---|---|
| 2026-11-09 | 工程独立化 | 基本完成，待轻量验收 | — | 构建/自检为记录，本轮未重跑 |
| 2026-11-09 | 初始化 GitHub 仓库和 README | 完成 | `63f3294` | 32 个文件，未提交 build/logs |
| 2026-11-09 | 建立状态文档 | 完成 | 待提交 | 本文档 |
| 2026-11-09 | T1 机身状态接口与离线变换 | 完成（离线） | Lidio | 自检通过；`T_base_cam` 未实测 |
| 2026-11-09 | M1-1 离线回放 + M1-2 路径/跟踪原型 | 完成（离线） | Lidio | VINS CSV→`BodyState`；Dubins + Pure Pursuit；模拟 A→B 约 2 cm；临时外参配置；实机未验证 |
| 2026-11-09 | M1-3 原生工具脉冲 + 路径跟踪接入 | 完成（离线） | Lidio | `pulse`/`plan`；`goto` 改用路径 + PP；`vy=0`；无 SDK/SDK 构建和自检通过；实机未运行 |
| 2026-11-09 | M1-3 VINS/UDP 桥接 + native vins 模式 | 完成（离线） | Lidio | VINS→base_link→UDP；origin 握手；valid=0/超时停止；桥接/原生/UDP 自检通过；未运行 ROS/VINS/实机 |
| 2026-11-09 | STATE 速度 + measured 拦截 | 完成（离线） | Lidio | `vx/vy/wz`、`measured`；默认拒绝 `measured=0`；`--allow-unmeasured-extrinsic` 覆盖并标记 CSV；离线自检通过 |

## 更新规则

1. 每完成一项任务，更新“最后更新”“当前阶段”“已完成”“未完成”“下一步任务”。
2. 在“任务记录”追加一行；失败或异常也记录，不删除。
3. 不把单次实验结果写成生产结论；不因结论不好改写历史。
4. 任务未完成时，保留原任务和阻塞原因，不擅自换题。
