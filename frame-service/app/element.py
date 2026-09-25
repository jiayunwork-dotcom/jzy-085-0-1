"""平面刚架单元：局部坐标下同时带轴向和弯曲的 6 阶刚度矩阵，以及均布荷载的等效节点荷载。

单元局部自由度顺序：[u1, v1, θ1, u2, v2, θ2]
- u：沿局部 x 轴（杆轴）的线位移
- v：沿局部 y 轴（垂直杆轴）的线位移
- θ：转角，逆时针为正
"""
from __future__ import annotations

import numpy as np


def local_stiffness_matrix(E: float, A: float, I: float, L: float) -> np.ndarray:
    """6x6 局部刚度矩阵：轴向部分 EA/L + 弯曲部分 EI/L 的 Hermite 梁刚度。"""
    a = E * A / L
    b = 12.0 * E * I / L**3
    c = 6.0 * E * I / L**2
    d = 4.0 * E * I / L
    e = 2.0 * E * I / L
    return np.array(
        [
            [ a,  0.0, 0.0, -a,  0.0, 0.0],
            [0.0,  b,   c,  0.0, -b,   c ],
            [0.0,  c,   d,  0.0, -c,   e ],
            [-a,  0.0, 0.0,  a,  0.0, 0.0],
            [0.0, -b,  -c,  0.0,  b,  -c ],
            [0.0,  c,   e,  0.0, -c,   d ],
        ]
    )


def udl_consistent_load_vector(q: float, L: float) -> np.ndarray:
    """垂直于杆轴的均布荷载 q（以局部 +y 方向为正）的等效节点荷载向量。

    由 Hermite 形函数积分得到的功等效节点荷载：
        f = [0, qL/2, qL^2/12, 0, qL/2, -qL^2/12]
    固端力（位移为零时节点作用于杆端的力）为其相反数 -f。
    杆端力回代公式：r = k_local @ d_local - f。
    """
    return np.array([0.0, q * L / 2.0, q * L * L / 12.0, 0.0, q * L / 2.0, -q * L * L / 12.0])
