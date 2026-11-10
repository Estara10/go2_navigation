# Go2 原生终端控制：现场命令

> **历史续接（2026-11-08）：**0.20 m/s、6秒的直走已经执行，用户仍观察为只前倾；之后用户报告0.30 m/s可正常行走。下文0.20/6直走步骤保留为历史操作，不再作为当前首项重复。历史 goto 首试为 vmax=approach=0.30 m/s 的 `goto 0.5 0`；该段控制律已被 2026-11-09 的路径 + Pure Pursuit 取代，旧侧移描述不再适用。当前操作看下方 2026-11-09 更新、6A 短脉冲和 7 短距离 plan/goto。

> **容差更新：**用户要求将goto位置容差改为5厘米（0.05 m）。重新启动本次构建的程序，终端应显示 `plan/goto 位置容差=0.050 m`。速度仍沿用0.30 m/s候选，历史10厘米容差试验不改写。移动到新场地后，在静止状态重新origin；停稳后先读取status和测量，再趴伏或搬动。若越过目标后反向追赶或反复调整，立即stop，本次不放任持续纠偏。5厘米是原生估计判据，不是实际精度保证。

> **2026-11-09 更新：**新增短脉冲 `pulse vx duration` 和相对目标 `plan forward left`；`goto` 保留初始坐标固定点语义，但改用一次生成的路径 + Pure Pursuit，**所有正常运动 `vy=0`，不再侧移**。另新增 `--control-source vins`：plan/goto/yaw 可由 VINS 桥通过 UDP 提供 `base_link` 状态；详见 0A。下方历史直走和旧 goto 步骤保留作追溯；当前现场操作优先看“短脉冲”和“plan/goto 新控制律”两节。

本文件供用户在电脑本地终端逐步操作。首次只做 `straight` 短时直走与停止，不运行论文控制器或低层控制。没有外接传感器要求。

命令不需要 RTK 或 sudo。不要在 `unitree@ubuntu` 的 SSH 终端中运行；本工具和 SDK 位于电脑本地。

## 0. 现场参数（执行前确认并显式设置）

本文件中的网卡、IP、DDS domain、SDK 路径、构建目录和日志目录都是外部参数。下面的值只是占位符或历史记录，**不是默认值**；未确认前不要照抄执行。

```bash
export PROJECT_ROOT=/path/to/go2_navigation   # 本仓库根目录
export UNITREE_SDK2_ROOT=/path/to/unitree_sdk2 # 含 lib/cmake/unitree_sdk2
export GO2_NIC=...                            # 电脑连接 Go2 的网卡名
export PC_IP=...                              # 电脑在该网段的地址
export ORIN_IP=...                            # Go2 机载 Orin 地址
export DDS_DOMAIN=...                         # Go2 原生 DDS domain
export ROS_DOMAIN_ID=...                      # 相机/VINS 使用的独立 ROS domain
```

2026-11-04 的历史配置是 `enp7s0`、电脑 `192.168.123.99/24`、Orin `192.168.123.18`、DDS domain `0`；这些只说明当时现场，不表示当前仍适用。

## 0A. VINS 控制来源（A1 机制验证，默认不启用）

本任务只离线实现；以下步骤供现场安排后使用，执行器未运行。

启动顺序：

1. 按 `scripts/go2_vins/` 起 VINS，确认 `/odometry` 有数据；这不在本工具内。
2. 另开终端启动桥接（需要 ROS 2 Humble；不要沿用 Anaconda 的 `LD_LIBRARY_PATH`，先 `conda deactivate` 或使用干净 shell）：

```bash
source /opt/ros/humble/setup.bash
"$PROJECT_ROOT/build/go2_vins_bridge/go2_vins_bridge" \
  --config "$PROJECT_ROOT/scripts/go2_nav_plan/config/d435i_provisional.conf" \
  --vins-topic /odometry \
  --control-port 46001 --state-port 46002
```

3. 启动原生工具（`--control-source vins`）：

```bash
vins_log_dir="$PROJECT_ROOT/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)_vins"
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:"$UNITREE_SDK2_ROOT/lib" \
  "$PROJECT_ROOT/build/native_motion-sdk/go2_native_motion" \
  --control-source vins \
  --vins-control-port 46001 --vins-state-port 46002 \
  --vins-timeout-s 0.5 \
  --domain "$DDS_DOMAIN" --interface "$GO2_NIC" \
  --csv "$vins_log_dir/native_motion.csv"
```

4. 等待原生状态就绪；确认桥接日志显示 `measured=0/1`。当前临时外参为 `measured=0`，**不得用于实机控制或 5 cm 验收**。
   - 原生工具默认拒绝 `measured=0` 的 origin/plan/goto。
   - 只有离线/机制验证可显式加 `--allow-unmeasured-extrinsic`；CSV 会记录 `vins_measured=0`、`measured_override=1`。
5. 机器人静止且 VINS 状态有效后输入：

```text
命令> origin
```

原生工具向桥接发送 `SET_ORIGIN`，等待 `ORIGIN_OK <measured>`。`measured=0` 且未加 `--allow-unmeasured-extrinsic` 时，原生工具拒绝 origin，不进入运动；`ORIGIN_FAIL <reason>` 时先检查 VINS/桥接。只有成功才记录起点。

6. 用 `status` 查看 `control_source=vins`、`vins_valid`、`vins_reason`、`vins seq/stamp/age`、`vx/vy/wz`、`measured`、`measured_override`。然后按已审阅范围使用：

```text
命令> plan 0.5 0
命令> goto 1 0
命令> yaw 30
```

vins 模式下 `plan`/`goto`/`yaw` 使用 VINS `BodyState`（含 `vx/vy/wz`）；`straight`/`pulse` 只允许 `--control-source native`，会被拒绝。所有正常运动仍 `vy=0`。

失效处理：

- `vins_valid=0`（`NO_NAV`、`NO_VINS`、`STALE`、`NONFINITE`、`JUMP`、`RESET`）或 STATE 包龄超过 `--vins-timeout-s` 时，原生工具立即停止并记录 `VINS_INVALID`。
- **不切回原生位置继续运动**。确认 VINS/桥接恢复后，通常需要 `quit` 重启并重新 `origin`。
- `measured=0` 默认拒绝 origin/plan/goto；显式覆盖时仍不得用于 5 cm 验收。
- 停止失败锁存仍有效：`StopMove` 失败后本会话拒绝运动命令，只允许 `status`、`stop`、`quit`。
- 桥接只发状态，不发运动；原生工具仍是唯一运动发送入口。

## 1. 现场准备

- Go2 原生运动服务保持运行，电脑通过网线连接机器人。
- 场地空旷，线缆有余量，机器人与周围人员保持距离。
- 用户使用原装遥控器站稳机器人，遥控器拿在手中，先确认已核对的原生停止操作能够响应。
- 站立后还须由用户按熟悉的遥控操作确认已进入原生可行走状态，摇杆归中；不能把“已站立”自动等同于“已解除遥控锁定”。2026-11-08先前StopMove返回-1，用户确认此前锁定；进入可行走状态后的同工具静止stop返回0，记录见 `logs/go2_native_motion/20261108_044903/native_motion.csv`。该结果不代表运动后的实际停止已验证。
- 不同时运行其他向机器人发送运动指令的程序。
- 以下命令逐步输入；不要一次粘贴整套交互命令，以免动作连续开始。

## 2. 电脑本地检查网卡

```bash
ip -br addr show "$GO2_NIC"
ip route get "$ORIN_IP"
```

确认路由查询显示 `dev "$GO2_NIC"`。这一步只确认网络配置，不证明 SDK 通信或停止可用。若配置不符，先核对，不照抄启动参数。

## 3. 构建（已有最新产物则跳过）

执行器已报告现有离线自检通过；若产物仍在且源码未变，不必重复构建、自检。构建产物丢失或源码更新时，再执行：

```bash
cmake -S "$PROJECT_ROOT/scripts/go2_native_motion" \
  -B "$PROJECT_ROOT/build/native_motion-sdk" \
  -DGO2_REAL_ROBOT=ON \
  -DUNITREE_SDK2_ROOT="$UNITREE_SDK2_ROOT" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$PROJECT_ROOT/build/native_motion-sdk" --parallel 2
```

构建成功后做离线自检，不初始化 DDS、不连接机器人：

```bash
"$PROJECT_ROOT/build/native_motion-sdk/go2_native_motion_selftest"
```

若只想验证无 SDK 的核心与自检，把上述配置改为 `-DGO2_REAL_ROBOT=OFF` 并使用独立构建目录。任一步失败就停止，不继续现场启动。

## 4. 历史直走启动：0.20 m/s、6秒（已执行，不重复）

此前 0.20 m/s、3 秒已经能前进，但实际约 10 cm，尚未确认连续行走。本次保持 0.20 m/s，把一次观察延长到 6 秒，用于区分起步时间不足与持续行走异常。只做一次，不自动执行更高速度、更长时长或连续任务。

允许范围是 `vx 0.05–0.50 m/s`、`duration 1–30 s`；本次只用 `0.20 m/s / 6.0 s`。

在电脑本地终端执行：

```bash
motion_log_dir="$PROJECT_ROOT/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)"
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:"$UNITREE_SDK2_ROOT/lib" \
  "$PROJECT_ROOT/build/native_motion-sdk/go2_native_motion" \
  --domain "$DDS_DOMAIN" --interface "$GO2_NIC" \
  --vx 0.20 --duration-s 6.0 \
  --goto-max-speed 0.30 --goto-approach-speed 0.30 \
  --csv "$motion_log_dir/native_motion.csv"
```

启动会建立原生 SDK 通信、订阅状态，但不会自动运动。程序自动创建日志目录并打印 CSV 路径；状态就绪后会打印本次 `straight` 的 vx=0.200 m/s、duration=6.000 s，以及 `plan/goto` 的 `vmax=0.300 m/s`、`approach=0.300 m/s`。等待显示状态就绪与 `命令>`；无状态或启动报错时，不执行后续运动操作。

**`--vx` 和 `--duration-s` 只配置 `straight`，不会限制 `pulse`、`plan`、`goto` 和 `yaw`。**

## 5. 在程序的“命令>”中逐步操作

以下不是 Bash 命令，必须在工具运行后的交互提示处输入，每条按 Enter。不要整段粘贴。

### 查看状态

```text
status
```

检查状态在更新，且没有停止失败提示。原生反馈仍是估计值。

### 用户确认机器人静止后记录起点

```text
origin
```

程序记录一次初始位置和朝向，不会运动。本会话不能再次记录原点。

### 静止时检查电脑停止调用

```text
stop
```

应看到 `StopMove返回=0`；这只能确认接口返回，用户仍需确认机器人状态。若停止失败，先现场停止机器人，不继续运动；本会话即使再次 stop 成功，也不再接受运动命令。

### 首次直走观察

确认现场准备完成后输入：

```text
straight
```

工具沿当前机体前方，以 `0.20 m/s` 持续发送 6 秒指令，然后停止重发并调用 `StopMove`。终端应显示本次 `vx=0.200 m/s`、`duration=6.000 s`。理论指令距离为 `0.20 × 6 = 1.2 m`，**不代表实际距离**。

观察：

- 起步后是否连续迈步，而不是只有一次迈步；
- 到时是否实际停住，以及之后是否保持停止；
- 用地面标记和卷尺测量实际位移，与理论 1.2 m 分开记录。

只做这一次。若仍只有一次迈步、持续前倾或姿态异常，立即停止试验，不自动继续增加速度、时长或连续任务。没有明显位移也不等于程序故障，不自动加大参数。

### 停止与退出

运动中需要停止，在同一终端输入并按 Enter：

```text
stop
```

也可以按键盘 **Ctrl+C**，尝试停止并退出；不要输入字符 `Ctrl+C`。

正常任务结束、用户确认实际停住后，输入：

```text
quit
```

普通停止先终止任务、停止发送 Move，再调用 StopMove。SDK 调用可能阻塞：当前调用超时设置为1秒，停止并非即时完成，若 SDK 不按超时返回则可能更久。

发生危险时，用户优先使用已核对的原生遥控停止操作，同时尽快 stop 或 Ctrl+C，终止电脑重发。不要把松开遥控摇杆当成取消电脑命令。遥控与电脑指令优先级尚未在本机验证；本工具不提供独立急停，不用拔线或强杀进程代替停止。

## 6. 退出后查看结果

程序退出、重新出现 Bash 提示符后，先执行以下命令，保留刚才的退出码：

```bash
motion_exit_code=$?
printf '程序退出码：%s\nCSV：%s\n' "$motion_exit_code" "$motion_log_dir/native_motion.csv"
```

- `0`：正常退出，且本会话没有停止请求失败；不证明机器人实际停住。
- `1`：启动阶段未等到状态或收到中断，未进入运动。
- `2`：参数、初始化或运行异常。
- `3`：本会话出现过停止请求失败，或最终停止请求失败。

检查本工具进程：

```bash
pgrep -af '[g]o2_native_motion'
```

没有输出表示未发现名称匹配的进程；有输出时先确认来源，不盲目结束其他会话。用户确认机器人实际停住后，再处理日志或残留进程。

首次完成条件：起步后连续迈步、方向正确、姿态平稳、到时实际停住且未自行再次运动；实际位移已用地面标记/卷尺测量并记录，CSV保留。接口返回、CSV中的完成事件和进程退出都不能代替现场观察。

## 6A. 短脉冲 pulse（当前首选测量）

用途：测原生步态的最小步长、起步延迟和停车距离。一次 `Move` 可能让机器人保持约 1 秒，**实际位移包含延迟和停车距离，不能用 `vx × duration` 估算**。

前提：现场停止方式已确认，遥控器在手；机器人静止且状态新鲜；已 `origin`；只运行本工具一个运动发送入口。

在现有会话中逐条输入（不要整段粘贴）：

```text
命令> pulse 0.20 0.10
任务开始: pulse vx=0.200 duration=0.100 vy=0 vyaw=0
...
任务结束: pulse COMPLETED
停止流程: reason=COMPLETED StopMove返回=0
```

观察并记录：

- 是否连续迈步、是否只有一次前倾；
- 指令结束后实际是否停住、停后是否继续移动；
- 用地面标记/卷尺测实际位移和停车距离；
- CSV 的 `task=pulse`、`pulse_vx_mps`、`pulse_duration_s`、每条 `cmd` 和 `stop_result`。

按已审阅的顺序，每次只做一次：`pulse 0.20 0.20`、`pulse 0.20 0.30`、`pulse 0.20 0.50`。`pulse` 允许 `vx 0.05–0.50 m/s`、`duration 0.1–0.5 s`；超出直接拒绝，不发送运动。若出现姿态异常、未停稳、明显过冲或停止失败，立即停止，不自动增加速度/时长。停止失败后本会话只允许 `status`、`stop`、`quit`，`pulse` 也被拒绝。

## 7. 后续验证入口（首次不执行）

先审阅首次结果和短脉冲结果，之后再安排运动中 stop、plan/goto、yaw 的验证；本节只说明命令，不是要求现在连续执行。

### 验证运动中 stop

本次 6 秒窗口足够在运动中输入 `stop`；但本次首要观察是完整 6 秒的连续行走与到时停止，不应为了再测一次而自动追加任务。若本次因异常使用 `stop` 提前终止，记录原因；若要单独验证运动中 stop，由主助手另行安排会话，仍使用已审阅参数，不自动提高速度或时长。

本段为此前直走操作记录。0.20 m/s / 6秒已执行，当前不重复；后续按本页顶部更新和下方独立 plan/goto 步骤进行。

### 短距离 plan / goto

另一次已安排的验证中，使用独立会话（不输入 `straight`，不会自动运动）：

```bash
goto_log_dir="$PROJECT_ROOT/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)_plan"
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:"$UNITREE_SDK2_ROOT/lib" \
  "$PROJECT_ROOT/build/native_motion-sdk/go2_native_motion" \
  --domain "$DDS_DOMAIN" --interface "$GO2_NIC" \
  --goto-max-speed 0.30 --goto-approach-speed 0.30 \
  --csv "$goto_log_dir/native_motion.csv"
```

`--vx/--duration-s` 使用默认值，只影响 `straight`，不会被 `plan`/`goto` 使用。确认状态并重新 `origin` 后，先做一次短距离相对目标：

```text
命令> plan 0.5 0
任务开始: plan relative=(0.500,0.000) local=(...) native=(...) path_len=... radius=0.600 vy=0
...
任务结束: plan ARRIVED
```

`plan f l` 的 `(f,l)` 按**下令时**机头方向解释，目标随后固定；机器人转向后目标不移动。`goto x y` 仍到本会话初始原点坐标系中的固定点，但同样使用一次路径 + Pure Pursuit。两者都只发 `vx/vyaw`，`vy=0`；侧方目标通过转向和前进处理，不用侧移。

`approach=0.30` 是接近目标时的防失速下限候选，不是已验证的硬件最低速度；0.30 也只是当前已实测能走的候选最大速度。观察是否连续迈步、到时是否实际停住、是否明显过冲，并用地面标记/卷尺测量实际位移。只做这一次，不自动提高速度、降低下限或连续追加任务。

### 到点与转向

在短距离 plan/goto 结果审阅后，按已安排的范围逐项测试：

| 命令 | 含义 |
| --- | --- |
| `plan 0.5 0` | 下令时前方 0.5 m 的相对目标 |
| `plan 0 1` | 下令时左侧 1 m 的相对目标；应转向后前进，不侧移 |
| `goto 0.5 0` | 到初始原点前方 0.5 m 的固定点 |
| `goto 1 0` | 到初始原点前方 1 m 的固定点 |
| `goto 0 1` | 到初始原点左侧 1 m 的固定点；应转向后前进，不侧移 |
| `yaw 30` | 原地转到相对初始朝向左转 30° 的方向 |
| `status` | 查看当前本地位置、朝向和任务 |
| `stop` | 取消任务并尝试停止 |
| `quit` | 停止后退出 |

`goto` 目标都相对本会话的初始原点；`plan` 目标相对下令时机头。两者都不相对上一任务终点。需要独立方向验证时，退出重启并在静止状态重新 `origin`。

当前独立限制：

- `plan`/`goto` 最大平移速度默认 `0.30 m/s`，可用 `--goto-max-speed` 配置（范围 `0.05–0.50 m/s`）；接近目标速度不低于 `--goto-approach-speed`（默认 `0.30 m/s`，范围 `0.05–max`）。`approach` 是首轮防失速候选，不是已验证的硬件最低速度。目标距离不超过 3 米，最长 30 秒，位置容差 0.05 米；3 米不是累计路程或每段运动距离上限。`straight` 的 0.50 m/s 上限只适用于 `straight`。
- 路径最小转弯半径默认 `0.60 m`（`goto_max_speed / 0.50`）；纯跟踪前视距离为初始离线值，未按实机步态验证。
- `yaw` 最大角速度 `0.50 rad/s`（约 28.6°/秒），最长 15 秒，朝向容差 5°，采用最短角度转向；原地转向不产生平移。
- `plan`/`goto` 不单独控制终点朝向；`yaw` 不主动控制位置；没有自动避障。
- `ARRIVED` 只表示原生估计进入容差。使用地面标记、卷尺核对实际距离，不将原生估计当作独立真值。

方向、速度、姿态、状态或线缆有任何异常，立即停止，不扩大目标。

## 8. 本次 StopMove 返回 -1 后的只读查询

### 8.1 已有事实

2026-11-08 三次会话日志 `logs/go2_native_motion/20261108_032016/native_motion.csv`、`20261108_033359/native_motion.csv` 和 `20261108_044308/native_motion.csv` 中都没有 `cmd` 行；静止 stop 和退出时的 StopMove 均返回 -1，结束事件为 QUIT_STOP_FAILED。第三次会话没有 `origin`、没有任务，直接 `quit` 仍返回 -1，说明不是先前运动/任务残留。运动工具已确认退出，当前不继续运动任务。

两次会话之间的5秒只读观察结果：LowState约500 Hz，SportModeState约498–499 Hz，后者最大接收间隔约13.6 ms；mode=6、error_code=0，原生线速度接近零。观察器正常退出。后右髋部温度原始字段由72升至73、后左髋部为81，其他电机约27–33。尚无本机温度保护阈值依据，不能把温度认定为StopMove=-1的原因；mode=6也尚未取得适用于本机固件的官方语义依据，不用SDK示例中的测试选项编号解释它。

**下一步不是再运行只订阅状态的观察器，而是运行独立的只读查询入口。**该入口会订阅状态，并发送 `CheckMode`、`ServiceList`、`GetServerApiVersion` 查询请求；它不是“只订阅、完全不发包”。

### 8.2 构建查询入口

查询目标与运动工具同一构建目录。若 `"$PROJECT_ROOT/build/native_motion-sdk/go2_native_readonly_query"` 不存在，先执行第 3 节的 cmake 配置与构建；源码更新后也重新构建。不要运行运动工具。

```bash
ls -l "$PROJECT_ROOT/build/native_motion-sdk/go2_native_readonly_query"
```

### 8.3 现场最小只读命令

机器人保持静止，用户手持遥控器；不要在运动工具中执行 `straight/pulse/plan/goto/yaw`。只在电脑本地终端执行：

```bash
query_log_dir="$PROJECT_ROOT/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)_readonly_query"
mkdir -p "$query_log_dir"
timeout --signal=INT --kill-after=2s 20s \
  env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:"$UNITREE_SDK2_ROOT/lib" \
  "$PROJECT_ROOT/build/native_motion-sdk/go2_native_readonly_query" \
  --domain "$DDS_DOMAIN" --interface "$GO2_NIC" \
  2>&1 | tee "$query_log_dir/readonly_query.txt"
query_exit=${PIPESTATUS[0]}
printf 'query exit: %s\nlog: %s\n' "$query_exit" "$query_log_dir/readonly_query.txt"
```

该命令只做以下事情：

- 初始化 DDS；
- 订阅 `rt/sportmodestate`，最多等待3秒取得状态；
- 发送 `CheckMode`（`motion_switcher`，API 1001）、`ServiceList`（`robot_state`，API 1003）、`GetServerApiVersion`（Sport 通用 API 1）三个查询请求；
- 关闭订阅并释放 DDS，退出。

不调用 `Move`、`StopMove`、站立、趴伏、阻尼、步态、模式切换、`ServiceSwitch`、`SwitchJoystick`、`SetSilent` 或 `SetReportFreq`；退出、异常和析构中不发送停止或恢复指令。查询请求本身会发包，不能称为“完全只订阅”。

构造并 `Init` 各客户端只做本地 API 版本设置和 API 注册，不发送运动、停止或模式切换请求；随后只调用上面的查询方法。

等待时间：`CheckMode` 和 `ServiceList` 各设置 1 秒；`GetServerApiVersion` 是 SDK 现有方法，内部使用 1 秒并对发送/超时错误重试一次；本入口没有新增重试或超时。外部 `timeout 20s` 只是总时限。

### 8.4 预期输出与解释

保留终端和 `readonly_query.txt`。输出大致为：

```text
state before queries: seq=... mode=... error_code=... age_s=... pos=(...) vel=(...) yaw_raw=...
query check_mode: rc=... form="..." name="..."
query service_list: rc=... count=...
  service[0] name="..." status=... protect=...
  ...
query sport_api: local="1.0.0.1" server="..."
state after queries: seq=... mode=... error_code=... age_s=...
queries complete; no motion or mode-changing request sent; no stop/restore on exit
```

解释边界：

- `check_mode` 的 `form`/`name` 是机器人返回的原生模式；`rc` 非零时只记录返回码，不补默认模式，不据此猜测。
- `service_list` 的 `name/status/protect` 是机器人返回的服务状态；`rc` 非零或空列表时按实际记录，不补默认服务。
- `sport_api` 的 `local` 是本机 SDK 声明版本，`server` 是机器人返回版本；为空时显示 `NOT OBTAINED`，不补默认版本。
- `state` 的 `mode`、`error_code`、`age_s` 是原始状态和本机接收时龄；`mode=6` 仍按原始值记录，不用测试编号或社区说法当作官方定义。
- `NOT OBTAINED` / 空值表示没取得，不是正常值，也不是失败原因。

下一次查询要区分的候选：

- `CheckMode` 显示原生模式不允许运动/停止命令，或 `ServiceList` 显示运动服务未运行/受保护；
- Sport 服务端 API 版本与本机声明的 `1.0.0.1` 不一致；
- 服务列表中没有预期的原生运动服务，或服务状态异常；
- 以上都正常但 StopMove 仍返回 -1：现有本地证据无法解释，下一步查官方资料/支持渠道，不用 Move 或模式切换试探。

### 8.5 退出与清理

- 程序正常完成且未被中断时返回0；各个查询自身的 `rc` 仍可能是非零，必须以输出为准。
- DDS 初始化失败或运行中被中断返回1；参数错误返回2；外部 `timeout` 返回124属于限时终止，需记录，不代表机器人状态。
- 程序关闭订阅并释放 DDS；不发送停止/恢复指令，也不承担停止机器人的作用。
- 查询结束后核对进程：
  ```bash
  pgrep -af '[g]o2_native_readonly_query'
  ```
  没有输出表示未发现名称匹配的残留进程。
