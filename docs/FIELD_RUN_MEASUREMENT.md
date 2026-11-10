# 现场 A→B 测量与记录流程（自测版）

> 目的：让现场人员在没有协助者的情况下，独立完成一轮 A→B 走行、当场记录、并把可分析的数据带回来。适用于 M1-4 / M1-5 的 0.5 m / 1 m / 侧向 / 斜向和重复测试。
>
> 只用卷尺 + 地面标记（胶带/粉笔/贴纸）。不要求视频。测量基准是 `base_link` 原点的地面投影，不是机身壳体、机头或脚尖。

## 0. 一次只做一轮，做完立刻记

丢数据的原因只有一个：**量了但没当场写下来**。所以每轮走完，先花 30 秒把四个数填进记录卡（见 §6），再开始下一轮。机器人端的 CSV 也要用 `--csv` 保存（见 §4）。

## 1. 每轮开始前：标记 A、参考线、B

1. 把机器人放到起点，水平站立，朝向你选好的前进方向。
2. 找到 `base_link` 原点（四个髋关节旋转中心的中点）的地面投影，贴一个标记，记为 **A**。找法见 `docs/MEASUREMENT_CHECKLIST.md` §2；四个髋中点前后距 0.3868 m、左右距 0.0930 m，作为交叉核对。
3. 从 A 沿机器人当前机头方向在地面拉一条**参考线**（一条直胶带或粉笔线），越长越好（≥2 m）。这是后面所有“前向/横向/朝向”的基准。
4. 按本轮命令 `plan X Y` 标出目标点 **B**：从 A 沿参考线前进 `X`，再向机身左侧 `Y`（左 +Y）。`plan 0.5 0` 时 B 就在参考线上 A 前方 0.5 m 处。

## 2. 每轮执行顺序（终端 + 测量）

先建本轮目录并启动原生工具（照 `终端命令.txt` 的终端 B，带 `--csv` 和 `--allow-unmeasured-extrinsic`）：

```bash
RUN=/home/lidio/go2_navigation/logs/field_runs/$(date -u +%Y%m%d_%H%M%S)_plan_$(X)_$(Y)
mkdir -p "$RUN"; echo "RUN=$RUN"
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  ./build/final_sdk/go2_native_motion --domain 0 --interface enp7s0 \
  --control-source vins --allow-unmeasured-extrinsic \
  --vins-control-port 46001 --vins-state-port 46002 --vins-timeout-s 0.5 \
  --csv "$RUN/native_motion.csv"
```

依次输入：

1. `origin` —— 确认不是 `unmeasured_extrinsic` 拒绝（已加 `--allow-unmeasured-extrinsic`，应通过）。
2. `status` —— 确认 `valid=1 reason=OK`、`vins_age` 小（<0.1 s）。
3. `plan X Y` —— 若第一次回 `STALE`，先再发一次 `status`，再发 `plan X Y`（已知 UDP 旧包问题，重发即可）。
4. 等它报 `ARRIVED` 并停稳后，再输入 `stop`，然后 `quit`。
5. 人工确认机器人确实停稳（`StopMove` 返回 0 不代表真停）。

## 3. 到达后测量（只用卷尺 + 标记）

1. 找到**终点** `base_link` 原点的地面投影，贴标记记为 **P**。
2. 测 **前向 L_fwd**：从 P 向参考线作垂线，垂足为 H；卷尺沿参考线量 A→H 的距离。
3. 测 **横向 L_lat**：卷尺量 H→P 的垂直距离；P 在参考线**左侧为正**、右侧为负。
4. 测 **朝向**（两个数，不用算角度）：
   - 记**前髋中点**（FL/FR 中点）的地面投影到参考线的垂直距离 `d_front`（左正）；
   - 记**后髋中点**（RL/RR 中点）的同样距离 `d_rear`。
   - 朝向我回来算：`heading ≈ (d_front − d_rear) / 0.3868`（rad，左正）。

做垂线可用直角尺/方尺，或一条横放的卷尺配合手机水平仪；没有就用粉笔在地面画十字。

## 4. 保存（机器人端，别丢）

- 原生工具启动时加了 `--csv`，全程命令/VINS 状态/到达事件会写到 `$RUN/native_motion.csv`。
- 在本轮目录写一个 `notes.md`（或手机记同款字段），把 §6 的四列填进去。
- 若现场录了 bag，把它和 `metadata.yaml` 一起留在 `$RUN/bag/`。

## 5. 带回分析

- 把每轮的 `$RUN` 目录整包留在这台电脑 `logs/field_runs/` 下（已经在 `.gitignore`，不提交）。
- 或者更简单：**把 §6 的记录卡数字直接发我**，我结合 CSV 做误差和重复性分析。
- 至少给我：命令、`L_fwd`、`L_lat`、`d_front`、`d_rear`，以及有没有 CSV/bag。

## 6. 记录卡（每轮一行，当场填）

| 运行编号/时间 | 命令 plan X Y | 目标前向 X | 目标左 Y | 实测前向 L_fwd | 实测横向 L_lat(左正) | 前髋横向 d_front | 后髋横向 d_rear | 备注/现象 |
|---|---|---|---|---|---|---|---|---|
| 例：09:10 | 0.5 0 | 0.5 | 0 | 0.51 | ? | ? | ? | 非常准 |

只记这 8 列；误差（前向 = L_fwd − X，横向 = L_lat − Y，朝向 = (d_front − d_rear)/0.3868）由我回来算。单位一律 m（朝向列 m，不是度）。

## 7. 常见坑

- **别用机头外壳或脚尖当 P**：要量四个髋关节中点，否则和 `base_link` 定义对不上。
- **参考线要直、要长**：短或弯的参考线会让横向/朝向读数漂。
- **每轮重新放回 A、重新对齐**：`plan` 的目标是相对下令时机头的，连续 `plan` 会累积；要独立重复测试就每轮回到 A。
- **`status` 先于 `plan`**：避免第一次 `plan` 被 `STALE` 拦。
- **量完再 `quit`**：先停稳、量完、填完记录卡，再结束进程，避免 CSV 没刷完。
