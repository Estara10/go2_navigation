#!/usr/bin/env bash
# 现场一键编排（笔记本侧）：拉起 桥接 + VINS，等初始化，前台进入原生工具交互。
# 相机在 Orin（另一台机器）：本脚本只检查话题，不自动启停它。
#
# 用法：scripts/field/run_field.sh [--gait default|static] [--allow-unmeasured]
#   --gait default|static : 默认不变；static 请求 SportClient::StaticWalk()。
#   --allow-unmeasured     : 显式允许 measured=0 外参做机制验证（默认关闭，
#                            不加时 origin 会被工具拒绝——这是安全的默认）。
#
# 现场参数不静默：网卡/DDS IP/ROS domain/SDK 用环境变量覆盖，脚本会回显并要求 Enter 确认。
#   GO2_IFACE(默认 enp7s0) GO2_DDS_IP(192.168.123.99) ROS_DOMAIN_ID(42)
#   UNITREE_SDK2_ROOT(/home/lidio/Libraries/unitree_sdk2)
#
# A→B→C：进入原生工具后 origin 一次，之后可连续 plan（每条相对下令时机头方向）。
set -o pipefail

PROJECT="/home/lidio/go2_navigation"
GAIT="default"
ALLOW_UNMEASURED=0
IFACE="${GO2_IFACE:-enp7s0}"
DDS_IP="${GO2_DDS_IP:-192.168.123.99}"
ROS_DOMAIN="${ROS_DOMAIN_ID:-42}"
SDK="${UNITREE_SDK2_ROOT:-/home/lidio/Libraries/unitree_sdk2}"
ORIN_HINT="ssh unitree@192.168.123.18"

usage() { echo "用法: $0 [--gait default|static] [--allow-unmeasured]"; }
while [ $# -gt 0 ]; do
  case "$1" in
    --gait) GAIT="${2:-}"; shift 2 ;;
    --allow-unmeasured) ALLOW_UNMEASURED=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1"; usage; exit 2 ;;
  esac
done
[ "$GAIT" = "default" ] || [ "$GAIT" = "static" ] || { echo "--gait 只能是 default 或 static"; exit 2; }

echo "========== 现场参数确认（不对请 Ctrl-C，用环境变量覆盖）=========="
echo "  网卡 interface    : $IFACE"
echo "  DDS 本机 IP        : $DDS_IP"
echo "  ROS_DOMAIN_ID      : $ROS_DOMAIN"
echo "  Unitree SDK        : $SDK"
echo "  步态               : $GAIT"
echo "  未测外参覆盖       : $([ "$ALLOW_UNMEASURED" = 1 ] && echo '是（--allow-unmeasured，仅机制验证）' || echo '否')"
echo "=================================================================="
read -r -p "确认以上参数按 Enter 继续，Ctrl-C 退出: " _ || exit 2
[ "$ALLOW_UNMEASURED" = 1 ] || echo "注意：未加 --allow-unmeasured；T_base_cam 仍 measured=0，origin 会被工具拒绝（安全默认）。"

conda deactivate 2>/dev/null || true
set +u   # ROS setup.bash 不容忍 nounset
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="$ROS_DOMAIN" ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${DDS_IP}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
export VINS_IMAGE_QOS=best_effort
source "$PROJECT/install/vins/setup.bash"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
set -u

if ! timeout 6 ros2 topic list 2>/dev/null | grep -q "/camera/camera/infra1/image_rect_raw"; then
  echo "!! 看不到相机话题 /camera/camera/infra1/image_rect_raw。" >&2
  echo "   请先在 Orin（另一台机）启动相机：" >&2
  echo "   $ORIN_HINT" >&2
  echo "   然后照 终端命令.txt 的“终端 A”启动 realsense2_camera_node。" >&2
  exit 3
fi
echo "相机话题 OK。"

RUN="$PROJECT/logs/field_runs/$(date -u +%Y%m%d_%H%M%S)_script"
mkdir -p "$RUN/config" "$RUN/vins_out" || { echo "无法创建 $RUN"; exit 4; }
echo "RUN=$RUN"

CLEANED=0
cleanup() {
  [ "$CLEANED" = 1 ] && return
  CLEANED=1
  echo; echo "收尾：停 VINS / 桥接…（只停本脚本启的，按本次唯一 config 路径精确匹配）"
  [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null
  [ -n "${RUN:-}" ] && pkill -TERM -f "$RUN/config/realsense_stereo_imu.yaml" 2>/dev/null
  [ -n "${VINS_PID:-}" ] && kill "$VINS_PID" 2>/dev/null
  sleep 1
  [ -n "${RUN:-}" ] && pkill -KILL -f "$RUN/config/realsense_stereo_imu.yaml" 2>/dev/null
  [ -n "${BRIDGE_PID:-}" ] && kill -KILL "$BRIDGE_PID" 2>/dev/null
  echo "已停本脚本进程。请现场确认机器人停稳、核对残留进程；相机在 Orin 另停。"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

cd "$PROJECT" || exit 4
"$PROJECT/build/go2_vins_bridge/go2_vins_bridge" \
  --config scripts/go2_nav_plan/config/d435i_provisional.conf \
  --vins-topic /odometry --control-port 46001 --state-port 46002 >"$RUN/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 1
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "!! 桥接启动失败，见 $RUN/bridge.log"; exit 4
fi

cp scripts/go2_vins/config/d435i_480x270/*.yaml "$RUN/config/" || { echo "复制 VINS 配置失败"; exit 4; }
sed -i "s|^output_path:.*|output_path: \"$RUN/vins_out\"|" "$RUN/config/realsense_stereo_imu.yaml" || exit 4
sed -i "s|^pose_graph_save_path:.*|pose_graph_save_path: \"$RUN/pose_graph\"|" "$RUN/config/realsense_stereo_imu.yaml" || exit 4
ros2 run vins vins_node "$RUN/config/realsense_stereo_imu.yaml" >"$RUN/vins.log" 2>&1 &
VINS_PID=$!

echo "等待 VINS 初始化（Initialization finish!）… 期间机器人请保持静止、视野有纹理"
ok=0
for _ in $(seq 1 120); do
  if grep -q "Initialization finish!" "$RUN/vins.log" 2>/dev/null; then ok=1; break; fi
  kill -0 "$VINS_PID" 2>/dev/null || { echo "!! VINS 意外退出，见 $RUN/vins.log"; exit 5; }
  sleep 1
done
if [ "$ok" != 1 ]; then
  echo "!! 120s 内未初始化；检查纹理/静止/相机，见 $RUN/vins.log"; exit 5
fi
# 放行 native 前确认 /odometry 真在出数据（初始化日志不等于持续输出）
if ! timeout 6 ros2 topic echo --once /odometry >/dev/null 2>&1; then
  echo "!! /odometry 无数据，VINS 未真正输出；见 $RUN/vins.log"; exit 6
fi
echo "VINS 初始化完成，/odometry 有输出。"

NATIVE_ARGS=( --domain 0 --interface "$IFACE" --control-source vins
  --vins-control-port 46001 --vins-state-port 46002 --vins-timeout-s 0.5
  --csv "$RUN/native_motion.csv" --gait "$GAIT" )
[ "$ALLOW_UNMEASURED" = 1 ] && NATIVE_ARGS+=( --allow-unmeasured-extrinsic )

echo "================ 进入交互：先 origin，之后可连续 plan（A→B→C 多次）================"
echo "提示：origin / plan 前确认机器人站立、周围清空；结束用 quit（脚本会自动停 VINS/桥接）。"
env LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$SDK/lib" \
  "$PROJECT/build/final_sdk/go2_native_motion" "${NATIVE_ARGS[@]}"
native_rc=$?
cleanup
exit "$native_rc"
