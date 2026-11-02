"""dummy_vc_perception：把相机图像变成 base_link 下的抓取目标。

分两层，界线是"能不能离线自测"：geometry.py / gripper.py / detection.py 只依赖 numpy，
无 ROS、无 OpenCV；vision_node.py 只做 ROS 接口与 TF 查询，几何全部交给这三个文件。

接口按深度相机的标准设计（vision_msgs/Detection3DArray），换数据源不用动下游。
"""
