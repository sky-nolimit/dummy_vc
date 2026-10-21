"""工作面（地面 + 地台）的碰撞体，纯几何，不 import ROS。

为什么需要这一层：规划器只认识 URDF 里的东西。桌子、地台都不在 URDF 里，
于是"规划成功"只保证机械臂不会撞到它自己。抓取的最后一段是往下走，规划器
不知道脚底下有地面，就会给出从桌面里穿过去的路径 —— 而且它是有理由的：
按它知道的信息，那里什么都没有。

这个模块只算"盒子该多大、该放在哪"，与 ROS 无关，所以能在没有 ROS 的机器上
用 `tools/selftest_grasp_task.py` 直接验：

    * 地面的**顶面**必须正好在 `table_z`（差 1 mm 就等于多一层台阶）
    * 地台不能伸进 `base_link` 的实体里（顶面留 1 mm 余量），否则起始状态
      一上来就被判成自碰撞，规划直接失败
"""
from dataclasses import dataclass

from dummy_vc_perception import geometry as _geometry

#: 工作面的高度（base_link 下的 z）。与 `geometry.project_scene_measurements()`
#: 共用同一个数，避免这里一份、视觉那边一份。
DEFAULT_TABLE_Z = _geometry.project_scene_measurements().table_z.value

#: 地面盒子的厚度。它只是为了"有体积"（薄的盒子和关节空间里的碰撞检查一起
#: 用，太薄会让末端在两帧之间穿过去），顶面才是要紧的那个数。
DEFAULT_TABLE_THICKNESS_M = 0.02

#: 底座在 URDF 里的包围盒：`base_link.stl` 是 100 x 100 mm，底面在 z = -15.5 mm。
DEFAULT_BASE_FOOTPRINT_M = 0.10
DEFAULT_BASE_BOTTOM_Z_M = -0.0155

#: 地台顶面与 `base_link` 底面之间留的缝。0 会让两个面重合，有的 FCL 版本
#: 把"接触"也算碰撞，于是起始状态永远在碰撞里。
DEFAULT_BASE_CLEARANCE_M = 0.001

#: 地台是两层（2026-10-21 实测）：上层和底座一样大、高 1 cm，机械臂固定在它
#: 上面；下层 20x20 cm、高 1.5 cm，中心在 base_link 的 +y 4 cm 处（前沿
#: +14 cm、后沿 -6 cm，左右各留 5 cm）。
DEFAULT_PLATFORM_UPPER_HEIGHT_M = 0.010
DEFAULT_PLATFORM_LOWER_SIZE_M = 0.20
DEFAULT_PLATFORM_LOWER_HEIGHT_M = 0.015
DEFAULT_PLATFORM_LOWER_CENTER_Y_M = 0.04


@dataclass(frozen=True)
class Box(object):
    """一个方块碰撞体：`size` 是全长（不是半长），`center` 是几何中心。"""

    name: str
    size: tuple            # (x, y, z) 米
    center: tuple          # (x, y, z) 米
    note: str = ''

    @property
    def top_z(self):
        return self.center[2] + 0.5 * self.size[2]

    @property
    def bottom_z(self):
        return self.center[2] - 0.5 * self.size[2]

    def describe(self):
        return '{}: {:.0f}x{:.0f}x{:.0f} mm，中心 ({:.3f}, {:.3f}, {:.3f})，顶面 z={:.4f}{}'.format(
            self.name, self.size[0] * 1e3, self.size[1] * 1e3, self.size[2] * 1e3,
            self.center[0], self.center[1], self.center[2], self.top_z,
            '（{}）'.format(self.note) if self.note else '')


def table_box(table_z=None, size_x=0.8, size_y=0.8, center_x=0.0, center_y=0.30,
              thickness_m=DEFAULT_TABLE_THICKNESS_M):
    """地面（工作面）。顶面精确落在 `table_z` 上，厚度往下长。"""
    if table_z is None:
        table_z = DEFAULT_TABLE_Z
    if thickness_m <= 0.0:
        raise ValueError('地面厚度必须大于 0，收到 {}'.format(thickness_m))
    if size_x <= 0.0 or size_y <= 0.0:
        raise ValueError('地面尺寸必须大于 0，收到 {} x {}'.format(size_x, size_y))
    return Box(name='ground',
               size=(float(size_x), float(size_y), float(thickness_m)),
               center=(float(center_x), float(center_y),
                       float(table_z) - 0.5 * float(thickness_m)),
               note='工作面顶面在 table_z')


def base_platform_boxes(table_z=None, footprint_m=DEFAULT_BASE_FOOTPRINT_M,
                        base_bottom_z=DEFAULT_BASE_BOTTOM_Z_M,
                        clearance_m=DEFAULT_BASE_CLEARANCE_M,
                        upper_height_m=DEFAULT_PLATFORM_UPPER_HEIGHT_M,
                        lower_size_m=DEFAULT_PLATFORM_LOWER_SIZE_M,
                        lower_height_m=DEFAULT_PLATFORM_LOWER_HEIGHT_M,
                        lower_center_y_m=DEFAULT_PLATFORM_LOWER_CENTER_Y_M):
    """两层地台，2026-10-21 在整机上量出来的。

    上层：和底座（那个 10x10 cm 的黑盒子）一样大，高 1 cm，机械臂整个固定在
          它上面，所以它就在 base_link 正下方居中。
    下层：20x20 cm、高 1.5 cm。上层**不居中** —— 下层前沿在 base_link +14 cm、
          后沿 -6 cm，也就是下层的中心在 +y 4 cm 处；左右各留 5 cm。

    上层顶面比 `base_link` 底面低 `clearance_m`，否则固定底座和它会面贴面，
    MoveIt 会把"机器人一直在碰撞"当成常态，规划永远失败。

    厚度不足（底座本来就坐在地上）时返回空列表 —— 硬塞一个反而制造接触。
    """
    if table_z is None:
        table_z = DEFAULT_TABLE_Z
    if footprint_m <= 0.0:
        raise ValueError('地台尺寸必须大于 0，收到 {}'.format(footprint_m))
    if lower_size_m <= 0.0:
        raise ValueError('下层地台尺寸必须大于 0，收到 {}'.format(lower_size_m))

    table_z = float(table_z)
    # 那 1 mm 的缝从**上层**扣：上层是贴在底座下面、看不见的那面，下层保持
    # 实测的 1.5 cm，它的底面正好落在工作面上。
    upper_top = float(base_bottom_z) - float(clearance_m)
    upper_thick = float(upper_height_m) - float(clearance_m)
    upper_bottom = upper_top - upper_thick
    lower_top = upper_bottom
    lower_bottom = lower_top - float(lower_height_m)

    boxes = []
    if upper_thick > 0.0 and upper_bottom > table_z:
        boxes.append(Box(
            name='platform_upper',
            size=(float(footprint_m), float(footprint_m), upper_thick),
            center=(0.0, 0.0, 0.5 * (upper_top + upper_bottom)),
            note='和底座同大的第二层，顶面比 base_link 底面低 {:.1f} mm'.format(
                clearance_m * 1e3)))
    if lower_height_m > 0.0 and lower_bottom >= table_z - 1e-9:
        boxes.append(Box(
            name='platform_lower',
            size=(float(lower_size_m), float(lower_size_m), float(lower_height_m)),
            center=(0.0, float(lower_center_y_m), 0.5 * (lower_top + lower_bottom)),
            note='第一层 20x20 cm，中心在 +y {:.0f} mm（前沿 +14、后沿 -6 cm）'.format(
                lower_center_y_m * 1e3)))
    return boxes


def workspace_boxes(table_z=None, table_size_x=0.8, table_size_y=0.8,
                    table_center_x=0.0, table_center_y=0.30,
                    table_thickness_m=DEFAULT_TABLE_THICKNESS_M,
                    include_platform=True,
                    footprint_m=DEFAULT_BASE_FOOTPRINT_M,
                    base_bottom_z=DEFAULT_BASE_BOTTOM_Z_M,
                    clearance_m=DEFAULT_BASE_CLEARANCE_M):
    """返回 `(boxes, notes)`：要发给 planning scene 的盒子，以及给人看的说明。"""
    if table_z is None:
        table_z = DEFAULT_TABLE_Z
    boxes = [table_box(table_z=table_z, size_x=table_size_x, size_y=table_size_y,
                       center_x=table_center_x, center_y=table_center_y,
                       thickness_m=table_thickness_m)]
    notes = []
    if include_platform:
        platform = base_platform_boxes(table_z=table_z, footprint_m=footprint_m,
                                       base_bottom_z=base_bottom_z,
                                       clearance_m=clearance_m)
        if not platform:
            notes.append(
                '底座底面 z={:.4f} 不高于工作面 z={:.4f}，没有地台可加'.format(
                    base_bottom_z, table_z))
        else:
            boxes.extend(platform)
    return boxes, notes
