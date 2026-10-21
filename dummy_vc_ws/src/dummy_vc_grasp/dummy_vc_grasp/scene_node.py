"""把工作面（地面 + 地台）发进 MoveIt 的 planning scene。

不这么做的后果很具体：规划器不知道脚下有地面，于是"往下抓"的最后一段可以
规划出一条从地面里穿过去的路径 —— 而且它认为自己是正确的。碰撞检查只看
planning scene，URDF 里没有桌子。

为什么是**反复发**而不是只发一次：`/planning_scene` 的订阅端（move_group 的
scene monitor）用的是什么 QoS 由 MoveIt 决定，只发一次的话，"场景节点先起、
move_group 后起"这种情况就永远收不到。每 2 秒重发一次是最省事的稳妥做法
（一条消息，几十字节）。

关掉它：`ros2 run dummy_vc_grasp scene_node --ros-args -p publish:=false`，
或者干脆不起这个节点。想要干净地移除时，用 `clear:=true` 发一条 REMOVE。

    ros2 run dummy_vc_grasp scene_node
"""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from shape_msgs.msg import SolidPrimitive

from dummy_vc_grasp import scene_objects


class SceneNode(Node):

    def __init__(self):
        super().__init__('scene_objects')

        self.declare_parameter('scene_topic', '/planning_scene')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish', True)
        self.declare_parameter('clear', False)
        self.declare_parameter('republish_s', 2.0)

        # -- 工作面 ---------------------------------------------------------
        self.declare_parameter('table_z', scene_objects.DEFAULT_TABLE_Z)
        self.declare_parameter('table_size_x', 0.8)
        self.declare_parameter('table_size_y', 0.8)
        self.declare_parameter('table_center_x', 0.0)
        self.declare_parameter('table_center_y', 0.30)
        self.declare_parameter('table_thickness', scene_objects.DEFAULT_TABLE_THICKNESS_M)
        # -- 底座下面的地台 --------------------------------------------------
        self.declare_parameter('include_platform', True)
        self.declare_parameter('base_footprint', scene_objects.DEFAULT_BASE_FOOTPRINT_M)
        self.declare_parameter('base_bottom_z', scene_objects.DEFAULT_BASE_BOTTOM_Z_M)

        self.frame_id = self.get_parameter('frame_id').value
        self.boxes, notes = scene_objects.workspace_boxes(
            table_z=float(self.get_parameter('table_z').value),
            table_size_x=float(self.get_parameter('table_size_x').value),
            table_size_y=float(self.get_parameter('table_size_y').value),
            table_center_x=float(self.get_parameter('table_center_x').value),
            table_center_y=float(self.get_parameter('table_center_y').value),
            table_thickness_m=float(self.get_parameter('table_thickness').value),
            include_platform=bool(self.get_parameter('include_platform').value),
            footprint_m=float(self.get_parameter('base_footprint').value),
            base_bottom_z=float(self.get_parameter('base_bottom_z').value))

        qos = QoSProfile(depth=1,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            PlanningScene, self.get_parameter('scene_topic').value, qos)

        self.clearing = bool(self.get_parameter('clear').value)
        for box in self.boxes:
            self.get_logger().info('碰撞体 {}'.format(box.describe()))
        for msg in notes:
            self.get_logger().info(msg)

        if bool(self.get_parameter('publish').value):
            self._publish()
            every = float(self.get_parameter('republish_s').value)
            if every > 0.0:
                self.create_timer(every, self._publish)

    def _publish(self):
        scene = PlanningScene()
        scene.is_diff = True
        stamp = self.get_clock().now().to_msg()
        if self.clearing:
            self.get_logger().warn('clear:=true —— 只发 REMOVE，不加任何碰撞体')
        for box in self.boxes:
            scene.world.collision_objects.append(
                self._collision_object(box, stamp,
                                       remove=self.clearing))
        self.publisher.publish(scene)

    def _collision_object(self, box, stamp, remove=False):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(v) for v in box.size]

        pose = Pose()
        pose.position.x = float(box.center[0])
        pose.position.y = float(box.center[1])
        pose.position.z = float(box.center[2])
        pose.orientation.w = 1.0

        obj = CollisionObject()
        obj.header.frame_id = self.frame_id
        obj.header.stamp = stamp
        obj.id = box.name
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = (CollisionObject.REMOVE if remove
                         else CollisionObject.ADD)
        return obj


def main(args=None):
    rclpy.init(args=args)
    node = SceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
