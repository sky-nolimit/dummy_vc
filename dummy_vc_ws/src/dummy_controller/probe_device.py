#!/usr/bin/env python3
"""临时脚本：连接设备并打印完整接口树"""
import sys
sys.path.insert(0, '/opt/ros_ws/dummy_ws/src/dummy_ros2_driver')

import dummy_cli_tool.ref_tool as ref_tool

print("Connecting to device on usb ...")
dev = ref_tool.find_any(path="usb", timeout=15.0)

if dev is None:
    print("Device not found!")
    sys.exit(1)

print("Connected!\n")
print("=== Full Device Interface Tree ===\n")
# depth=5 确保能看到 robot 下的函数和参数
print(dev._dump("", depth=5))

# 额外列出 robot 下所有函数名
print("\n=== Functions under 'robot' ===\n")
if hasattr(dev, 'robot') and 'robot' in dev._remote_attributes:
    robot = dev._remote_attributes['robot']
    for key, val in robot._remote_attributes.items():
        print(f"  {val._dump()}")
else:
    print("'robot' not found on device. Top-level attributes:")
    for key in dev._remote_attributes:
        print(f"  {key}")
