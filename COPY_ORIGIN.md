# 初始复制记录

日期：2026-11-09

> **路径说明**：本文中的 `/home/lidio/quadruped_robots`、`/home/lidio/go2_navigation` 等路径是复制时的来源/目标记录，仅用于追溯，不是当前构建或运行依赖。当前有效命令请以项目根 `README.md` 和 `scripts/` 下的模块说明为准。

来源：`/home/lidio/quadruped_robots`，Git HEAD 为 `dc7c4a46a2b908211865e049dbce90f60d5a7f60`。

本次复制的是工作区文件，其中包括尚未提交的 2026-11-09 A/B 审阅与 A1 需求，以及来自日志目录的诊断 diff；不能声称它们全部包含在该 HEAD 中。

原目录未删除、未迁移维护权。新目录已有的 A1_PROJECT_REQUIREMENTS.md 与源文件一致，保持不动。此次复制不是 Git 子模块，也未初始化 Git。

## 复制文件

- `scripts/go2_native_motion/CMakeLists.txt` ← `scripts/go2_native_motion/CMakeLists.txt`
- `scripts/go2_native_motion/native_motion_core.cpp` ← `scripts/go2_native_motion/native_motion_core.cpp`
- `scripts/go2_native_motion/selftest.cpp` ← `scripts/go2_native_motion/selftest.cpp`
- `scripts/go2_native_motion/native_motion_core.hpp` ← `scripts/go2_native_motion/native_motion_core.hpp`
- `scripts/go2_native_motion/README.md` ← `scripts/go2_native_motion/README.md`
- `scripts/go2_native_motion/main.cpp` ← `scripts/go2_native_motion/main.cpp`
- `scripts/go2_native_motion/FIELD_COMMANDS.md` ← `scripts/go2_native_motion/FIELD_COMMANDS.md`
- `scripts/go2_native_motion/readonly_query.cpp` ← `scripts/go2_native_motion/readonly_query.cpp`
- `scripts/go2_vins/field_record_qos_best_effort.yaml` ← `scripts/go2_vins/field_record_qos_best_effort.yaml`
- `scripts/go2_vins/CYCLONE_FIELD_CHECK.md` ← `scripts/go2_vins/CYCLONE_FIELD_CHECK.md`
- `scripts/go2_vins/udev/99-realsense-d435i-local.rules` ← `scripts/go2_vins/udev/99-realsense-d435i-local.rules`
- `scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch` ← `scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch`
- `scripts/go2_vins/config/d435i_480x270/left.yaml` ← `scripts/go2_vins/config/d435i_480x270/left.yaml`
- `scripts/go2_vins/config/d435i_480x270/right.yaml` ← `scripts/go2_vins/config/d435i_480x270/right.yaml`
- `scripts/go2_vins/config/d435i_480x270/realsense_stereo_imu.yaml` ← `scripts/go2_vins/config/d435i_480x270/realsense_stereo_imu.yaml`
- `docs/history/20261105_D435I_VINS_FUSION_PREPARATION.md` ← `docs/thesis_project/real_go2/20261105_D435I_VINS_FUSION_PREPARATION.md`
- `docs/history/20261108_VINS_BUILD_AND_IMU_REVIEW.md` ← `docs/thesis_project/real_go2/20261108_VINS_BUILD_AND_IMU_REVIEW.md`
- `docs/history/20261108_VINS_NATIVE_WALK_REVIEW.md` ← `docs/thesis_project/real_go2/20261108_VINS_NATIVE_WALK_REVIEW.md`
- `docs/history/20261108_REPEAT_WALK_REVIEW.md` ← `docs/thesis_project/real_go2/20261108_REPEAT_WALK_REVIEW.md`
- `docs/history/20261109_EXTRINSIC_AB_REPLAY.md` ← `docs/thesis_project/real_go2/20261109_EXTRINSIC_AB_REPLAY.md`
- `docs/history/20261108_STOPMOVE_RETURN_MINUS1_INVESTIGATION.md` ← `docs/thesis_project/real_go2/20261108_STOPMOVE_RETURN_MINUS1_INVESTIGATION.md`
- `scripts/go2_vins/patches/diagnostic_visualization.diff` ← `logs/real_go2/20261109_extrinsic_ab_replay/diagnostic_visualization.diff`

## 本次必要适配

仅将以下操作文档、候选配置中的 `/home/lidio/quadruped_robots` 改为 `/home/lidio/go2_navigation`，其余复制文件内容保持一致：

- `scripts/go2_native_motion/README.md`
- `scripts/go2_native_motion/FIELD_COMMANDS.md`
- `scripts/go2_vins/config/d435i_480x270/realsense_stereo_imu.yaml`

没有改变 C++ 算法、运动参数、停止路径、标定值和在线外参设置。未复制原始数据、第三方源码、SDK、构建树、模型、密码或 SSH 配置。

以上是复制当时的最小适配。2026-11-09 的独立化整理又移除了当前有效文档、配置和构建命令中的个人绝对路径，并新增了构建选项、协作/接口说明和 VINS 独立获取步骤；当前用法以项目根 `README.md` 和各模块说明为准。

## 后续协作与许可

当前仅完成文件提取，不代表项目可以直接自动到点。后续确定 A1 为提取模块的维护位置后，总项目通过接口及固定版本集成，避免长期两处各自修改。

复制时尚未为本项目声明新的统一开源许可证。分发前核对源项目及 VINS、Unitree SDK2 等上游许可证，保留原作者声明；不要未经确认将所有资产改授相同许可证。外部源码按固定提交拉取并保留自身 LICENSE。
