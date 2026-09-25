"""总刚组装、荷载向量组装与约束施加。

约束施加方法（全程仅此一种）：**行列缩减法（划去受约束自由度）**。
受约束自由度的位移恒为零，因此只把自由自由度对应的方程拿出来求解；
求解完成后再用完整的总刚回代受约束自由度上的支座反力。
不采用大数罚系数法。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .element import local_stiffness_matrix, udl_consistent_load_vector
from .schemas import FrameModel
from .transform import direction_cosines, transformation_matrix

DOFS_PER_NODE = 3  # 每个节点 3 个自由度：u_x, u_y, θ


@dataclass
class ElementData:
    """组装阶段为每根杆缓存的数据，供内力回代使用。"""

    eid: int
    dofs: list[int]      # 该杆 6 个整体自由度编号
    T: np.ndarray        # 坐标变换矩阵
    k_local: np.ndarray  # 局部刚度矩阵
    f_local: np.ndarray  # 杆上荷载的局部等效节点荷载向量
    length: float


@dataclass
class Assembly:
    K: np.ndarray                # 整体总刚度矩阵（未缩减）
    F: np.ndarray                # 整体荷载向量（节点荷载 + 杆上荷载的等效节点力）
    restrained: np.ndarray       # bool 数组，True 表示该自由度被约束
    dof_of_node: dict[int, list[int]]  # 节点编号 -> 3 个整体自由度编号
    elements: list[ElementData]


def assemble(model: FrameModel) -> Assembly:
    """把全部单元刚度转到整体坐标并叠加进总刚，同时组装荷载向量与约束标记。"""
    node_ids = [n.id for n in model.nodes]
    dof_of_node = {nid: [DOFS_PER_NODE * i + j for j in range(DOFS_PER_NODE)] for i, nid in enumerate(node_ids)}
    ndof = DOFS_PER_NODE * len(node_ids)
    K = np.zeros((ndof, ndof))
    F = np.zeros(ndof)
    xy = {n.id: (n.x, n.y) for n in model.nodes}

    # 同一根杆上的多个均布荷载按代数和叠加
    q_by_element: dict[int, float] = {}
    for udl in model.loads.element_udl:
        q_by_element[udl.element] = q_by_element.get(udl.element, 0.0) + udl.q

    elements: list[ElementData] = []
    for e in model.elements:
        x1, y1 = xy[e.n1]
        x2, y2 = xy[e.n2]
        c, s, length = direction_cosines(x1, y1, x2, y2)
        T = transformation_matrix(c, s)
        k_local = local_stiffness_matrix(e.E, e.A, e.I, length)
        f_local = udl_consistent_load_vector(q_by_element.get(e.id, 0.0), length)
        dofs = dof_of_node[e.n1] + dof_of_node[e.n2]

        # 局部刚度借方向余弦转到整体坐标后叠加进总刚
        k_global = T.T @ k_local @ T
        K[np.ix_(dofs, dofs)] += k_global
        # 杆上荷载的等效节点力同样转到整体坐标
        F[dofs] += T.T @ f_local

        elements.append(ElementData(eid=e.id, dofs=dofs, T=T, k_local=k_local, f_local=f_local, length=length))

    # 节点集中力与集中力矩直接进入荷载向量
    for nl in model.loads.nodal:
        d = dof_of_node[nl.node]
        F[d[0]] += nl.fx
        F[d[1]] += nl.fy
        F[d[2]] += nl.m

    # 约束标记：支座按自由度逐个锁死
    restrained = np.zeros(ndof, dtype=bool)
    for sup in model.supports:
        d = dof_of_node[sup.node]
        restrained[d[0]] = restrained[d[0]] or sup.ux
        restrained[d[1]] = restrained[d[1]] or sup.uy
        restrained[d[2]] = restrained[d[2]] or sup.rz

    return Assembly(K=K, F=F, restrained=restrained, dof_of_node=dof_of_node, elements=elements)
