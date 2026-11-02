"""dummy_vc_grasp：把"/detected_objects 里有个方块"变成"手臂真的去把它抓起来"。

分层界线是**能不能离线自测**：planner.py / task.py 是纯逻辑，不 import ROS；
pick_place_node.py 只做 ROS 接口（订阅检测、调 MoveGroup 动作、读夹爪模型、
把结果翻译成状态机事件）。

夹爪那一步留了闸门：`FingerOpeningModel` 在"角度↔开口"没实测之前，
`grasp_width_for()` 抛 `UnverifiedGripperModel`，整条链在**规划阶段**就拒绝，
而不是发一条会把整机锁死的 `hand.*` 命令。
"""
