#!/usr/bin/env python3
"""夹爪开合模型：开口宽度 <-> hand.set_angle 指令值。

为什么单独一个文件、还要一个"未实测就拒绝下发"的开关：

    docs/04 的 A1 写着 URDF 里的夹爪几何（0~0.023 m 行程、两指 origin）来自
    原作者的通用模型，**与实物不符**；A2 又写着 hand.* 三个方法会卡死整机。
    所以"开口 40 mm 对应 hand.angle 多少"这件事现在**没有任何可信来源**。

    既然没有可信来源，代码能做的只有一件事：大声拒绝。按猜出来的线性关系
    去夹，猜错的方向是"夹碎"或"崩飞"（docs/01 的风险 R9）。

怎么变成可信的：按 docs/02 的阶段 0.5，夹几个已知厚度的物体，量出两个端点
（开/合各一个），填成 Measurement(source='measured')，
command_for_width() 才会返回指令值。
"""
from dataclasses import dataclass
from typing import List, Tuple

from .geometry import GeometryError, Measurement

#: 开口 = 边长 + 这个间隙（docs/01 的阶段 5 写的是"+2mm"）。
DEFAULT_GRASP_CLEARANCE_M = 0.002


class UnverifiedGripperModel(RuntimeError):
    """端点还没实测，拒绝给出指令值。"""


@dataclass(frozen=True)
class FingerOpeningModel:
    """开口宽度 <-> hand.angle 的映射，按两段之间的线性插值。

    角度端点的正负不定（设备角度 0 点是哪一端还没实测），所以这里只要求
    两个角度不相等，不去比大小。
    """
    angle_open_deg: Measurement
    width_open_m: Measurement
    angle_closed_deg: Measurement
    width_closed_m: Measurement

    FIELDS = ('angle_open_deg', 'width_open_m', 'angle_closed_deg', 'width_closed_m')

    def __post_init__(self):
        for name in self.FIELDS:
            measurement = getattr(self, name)
            if not isinstance(measurement, Measurement):
                raise GeometryError(
                    '{} 要是 geometry.Measurement，收到 {}'.format(
                        name, type(measurement).__name__))
        if self.width_open_m.value <= self.width_closed_m.value:
            raise GeometryError(
                '开口端点反了：width_open 必须大于 width_closed（收到 {} / {}）'.format(
                    self.width_open_m.value, self.width_closed_m.value))
        if self.angle_open_deg.value == self.angle_closed_deg.value:
            raise GeometryError('两个角度端点相同，映射无从插值')

    @property
    def verified(self) -> bool:
        """四个端点全是实测值，才敢用它生成指令。"""
        return not self.unmeasured()

    def unmeasured(self) -> List[str]:
        return sorted(n for n in self.FIELDS
                      if not getattr(self, n).is_measured)

    def angle_for_width(self, width_m):
        """开口宽度 -> hand.angle 指令值。越界直接抛：范围外的开口不夹。"""
        width = float(width_m)
        low = self.width_closed_m.value
        high = self.width_open_m.value
        if width < low - 1e-9 or width > high + 1e-9:
            raise GeometryError(
                '开口 {:.1f} mm 超出实测范围 [{:.1f}, {:.1f}] mm'.format(
                    width * 1000.0, low * 1000.0, high * 1000.0))
        span = high - low
        ratio = 0.0 if span <= 0.0 else (width - low) / span
        return (self.angle_closed_deg.value
                + ratio * (self.angle_open_deg.value - self.angle_closed_deg.value))

    def width_for_angle(self, angle_deg):
        """反向换算，用于日志和回读比对。"""
        low = self.angle_closed_deg.value
        high = self.angle_open_deg.value
        angle = float(angle_deg)
        if angle < min(low, high) - 1e-9 or angle > max(low, high) + 1e-9:
            raise GeometryError(
                '角度 {:.2f} deg 超出实测范围 [{:.2f}, {:.2f}] deg'.format(
                    angle, min(low, high), max(low, high)))
        ratio = 0.0 if high == low else (angle - low) / (high - low)
        return (self.width_closed_m.value
                + ratio * (self.width_open_m.value - self.width_closed_m.value))

    def command_for_width(self, width_m):
        """唯一允许用来生成 hand.set_angle 指令值的入口。未实测就抛。"""
        if not self.verified:
            raise UnverifiedGripperModel(
                '夹爪开口↔角度映射还没实测（未量：{}），拒绝给出指令值。'
                '按 docs/02 的阶段 0.5 量两个端点后填成 measured。'.format(
                    ', '.join(self.unmeasured())))
        return self.angle_for_width(width_m)

    def grasp_width_for(self, object_edge_m, clearance_m=DEFAULT_GRASP_CLEARANCE_M):
        """夹一个边长 object_edge 的方块需要的开口宽度。"""
        if object_edge_m <= 0.0:
            raise GeometryError('方块边长必须为正：{}'.format(object_edge_m))
        return float(object_edge_m) + float(clearance_m)

    def describe(self) -> str:
        return ('开口 {:.1f}~{:.1f} mm <-> 角度 {:.2f}~{:.2f} deg（{}）'.format(
            self.width_closed_m.value * 1000.0, self.width_open_m.value * 1000.0,
            self.angle_closed_deg.value, self.angle_open_deg.value,
            '已实测' if self.verified else '含占位值'))


def unverified_placeholder_model() -> FingerOpeningModel:
    """一个纯粹用来把数据通路跑通的占位映射 —— 两个方向都可能反。

    0.046 m 开口来自 URDF 的行程 2×0.023 m；0 / 30 deg 来自设备角度范围
    0~30（docs/04）。A1 说这个几何与实物不符，A2 说 hand.* 会卡死整机，
    所以这个模型的 command_for_width() 永远抛 UnverifiedGripperModel。
    """
    return FingerOpeningModel(
        angle_open_deg=Measurement(0.0, 'deg', 'placeholder',
                                   'hand 角度范围 0~30，哪端是开未实测'),
        width_open_m=Measurement(0.046, 'm', 'placeholder',
                                 '2 x URDF 行程 0.023 m，实物几何不同（docs/04 A1）'),
        angle_closed_deg=Measurement(30.0, 'deg', 'placeholder',
                                     '同 angle_open_deg，方向未实测'),
        width_closed_m=Measurement(0.0, 'm', 'placeholder',
                                   '同上'),
    )


def opening_endpoints_from_measurements(closed_width_m, closed_angle_deg,
                                        open_width_m, open_angle_deg,
                                        where_closed, where_open) -> FingerOpeningModel:
    """现场量完之后，用这个函数把四个数装成已实测的模型。

    参数名按"闭合端点 / 打开端点"分组，就是阶段 0.5 的操作顺序。
    """
    return FingerOpeningModel(
        angle_open_deg=Measurement(float(open_angle_deg), 'deg', 'measured', where_open),
        width_open_m=Measurement(float(open_width_m), 'm', 'measured', where_open),
        angle_closed_deg=Measurement(float(closed_angle_deg), 'deg', 'measured', where_closed),
        width_closed_m=Measurement(float(closed_width_m), 'm', 'measured', where_closed),
    )
