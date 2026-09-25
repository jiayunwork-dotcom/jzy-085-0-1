"""方向余弦与局部/整体坐标变换。

约定：
- 整体坐标系：x 向右，y 向上，转角以逆时针为正。
- 杆件局部坐标系：局部 x 轴由端点 1 指向端点 2，局部 y 轴为局部 x 轴逆时针转 90°。
- 每个节点 3 个自由度：[u_x, u_y, θ]，单元自由度向量为 6 阶。
"""
from __future__ import annotations

import math

import numpy as np


def direction_cosines(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float, float]:
    """由两端坐标求方向余弦 (c, s) 与杆长。c=cosα, s=sinα，α 为局部 x 轴相对整体 x 轴的倾角。"""
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    return dx / length, dy / length, length


def transformation_matrix(c: float, s: float) -> np.ndarray:
    """6 阶坐标变换矩阵 T。

    局部位移 d_local = T @ d_global；
    整体力   F_global = T.T @ F_local（T 为正交矩阵，T.T 即 T 的逆）。
    """
    T = np.zeros((6, 6))
    # 局部 x 方向 = (c, s)，局部 y 方向 = (-s, c)（整体坐标下的分量）
    T[0, 0] = c
    T[0, 1] = s
    T[1, 0] = -s
    T[1, 1] = c
    T[2, 2] = 1.0
    T[3, 3] = c
    T[3, 4] = s
    T[4, 3] = -s
    T[4, 4] = c
    T[5, 5] = 1.0
    return T
