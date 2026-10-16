"""dummy_vc_perception：把相机图像变成 base_link 下的抓取目标。

分两层，界线是"能不能离线自测"：

    geometry.py / gripper.py   纯 numpy，无 ROS、无 OpenCV。
                               在没有 ROS 的机器上就能跑
                               tools/selftest_vision_geometry.py。
    vision_node.py（下一步）   只做 ROS 接口与 TF 查询，几何全部调上面两个文件。

相机到货前先用 driver 里现成的 USB 相机（dummy_controller/fingerprint_camera.py），
接口按深度相机的标准设计（vision_msgs/Detection3DArray），相机到位后只换数据源。
"""
