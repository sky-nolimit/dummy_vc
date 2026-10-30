#!/usr/bin/env bash
# 起 Orbbec 相机，顺手把那两行"没用的"报错过滤掉。
#
# 过滤的是这两行（都来自 orbbec_camera 驱动自己）：
#
#   [component_container-2] [ERROR] [camera.camera]: Failed to convert frame to video frame
#   [component_container-2] [ERROR] [camera.camera]: color frame is not decoded
#
# 它们是同一件事的两行：一帧彩色图在虚拟机里被弄丢了，驱动解码失败后跳过该帧。
# 原因是 VirtualBox 虚拟 xHCI 的等时传输事件失步（见 docs/04-已知缺陷与未决问题.md
# 的 G5），不是相机、线、CPU 或参数的问题，换参数也修不掉。丢一帧对抓取没有影响
# （vision_node 限流 10 Hz、只取最新帧），所以这里只是让它别再刷屏。
#
# 只过滤这两条，别的日志（包括真正的错误）照常打印——别把整个 stderr 关掉，
# 那样下次相机真坏了会看不见。
#
# 用法（参数原样转给 orbbec_camera 的 launch）：
#
#   bash tools/quiet_camera.sh
#   bash tools/quiet_camera.sh enable_sync:=false depth_registration:=false \
#       color_width:=640 color_height:=480 color_fps:=15
#
# 注意：相机已经在别的 launch 里起着的时候不要再用它，会抢同一个设备。

set -u

if ! command -v ros2 >/dev/null 2>&1; then
    echo "找不到 ros2，先 source /opt/ros/humble/setup.bash" >&2
    exit 1
fi

ros2 launch orbbec_camera gemini_330_series.launch.py "$@" 2>&1 |
    grep --line-buffered -v \
        -e 'Failed to convert frame to video frame' \
        -e 'color frame is not decoded'
