"""dummy_vc_grasp：把"/detected_objects 里有个方块"变成"手臂真的去把它抓起来"。

分层和 `dummy_vc_perception` 一样，界线是**能不能离线自测**：

    planner.py / task.py   纯逻辑（不 import ROS），
                           用 tools/selftest_grasp_task.py 在没装 ROS 的机器上跑
    pick_place_node.py     只做 ROS 接口：订阅检测、调 MoveIt 的 MoveGroup 动作、
                           读夹爪模型、把结果翻译成状态机事件

夹爪那一步刻意留了闸门：`dummy_vc_perception/gripper.py` 的 `FingerOpeningModel`
在"角度↔开口"没实测之前，`grasp_width_for()` 会抛 `UnverifiedGripperModel`，
所以整条链会在**规划阶段**就拒绝，而不是发一条可能卡死整机的 `hand.*` 命令。
"""
