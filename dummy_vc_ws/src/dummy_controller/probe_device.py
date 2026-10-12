#!/usr/bin/env python3
"""Connect to the arm and dump its complete fibre interface tree.

Useful on the bench: it is the authoritative answer to "what can this device
actually do", independent of any code that consumes it. Run it from anywhere:

    python3 probe_device.py

It must NOT be imported by the node: it is a standalone diagnostic and is not
installed by setup.py.
"""
import os
import sys

# Resolve dummy_cli_tool relative to this file rather than a hardcoded path.
# The previous version pointed at the original author's machine
# (/opt/ros_ws/dummy_ws/src/dummy_ros2_driver), which silently does not exist
# here: the import still worked only because the directory happened to be
# sys.path[0] when the script was run from its own folder, so running it from
# anywhere else failed for a confusing reason.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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
