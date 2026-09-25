"""位移回代：由节点位移求各杆杆端内力与各支座反力。"""
from __future__ import annotations

import numpy as np

from .assembly import Assembly


def element_end_forces(asm: Assembly, u: np.ndarray) -> dict[int, np.ndarray]:
    """每根杆的杆端力，严格由该杆自身的局部刚度乘其局部位移、再减去杆上荷载的等效节点力得到：

        r = k_local @ d_local - f_local

    返回 {杆件编号: [N1, V1, M1, N2, V2, M2]}。
    各分量为节点作用于杆端的力，方向与局部自由度正方向一致：
    - N1/N2：沿局部 x 轴的轴力（N2 为正表示杆件受拉，受拉时恒有 N1 = -N2）
    - V1/V2：沿局部 y 轴的剪力
    - M1/M2：杆端弯矩，逆时针为正
    """
    forces: dict[int, np.ndarray] = {}
    for ed in asm.elements:
        d_local = ed.T @ u[ed.dofs]
        forces[ed.eid] = ed.k_local @ d_local - ed.f_local
    return forces


def support_reactions(asm: Assembly, u: np.ndarray) -> dict[int, np.ndarray]:
    """支座反力：R = K @ u - F 在受约束自由度上的取值（整体坐标，力矩逆时针为正）。

    返回 {支座节点编号: [fx, fy, mz]}；未被约束的自由度分量恒为零。
    """
    residual = asm.K @ u - asm.F
    reactions: dict[int, np.ndarray] = {}
    for nid, dofs in asm.dof_of_node.items():
        mask = asm.restrained[dofs]
        if mask.any():
            r = np.zeros(3)
            r[mask] = residual[np.array(dofs)[mask]]
            reactions[nid] = r
    return reactions
