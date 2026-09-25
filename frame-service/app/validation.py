"""输入模型的语义校验。

所有非法模型都在这里被明确拒绝并抛出带错误码的 ModelError，
绝不静默丢弃杆件、也绝不让非法数据进入组装阶段。
"""
from __future__ import annotations

import math

import numpy as np

from .errors import (
    DISCONNECTED_STRUCTURE,
    DUPLICATE_ELEMENT_ID,
    DUPLICATE_NODE_ID,
    DUPLICATE_SUPPORT,
    EMPTY_MODEL,
    INSUFFICIENT_CONSTRAINTS,
    INVALID_SCHEMA,
    INVALID_SECTION,
    UNKNOWN_ELEMENT,
    UNKNOWN_NODE,
    ZERO_LENGTH_ELEMENT,
    ModelError,
)
from .schemas import FrameModel


def validate_model(model: FrameModel) -> None:
    if not model.nodes:
        raise ModelError(EMPTY_MODEL, "模型不包含任何节点。")
    if not model.elements:
        raise ModelError(EMPTY_MODEL, "模型不包含任何杆件。")

    # ---- 节点：编号唯一、坐标为有限数 ----
    node_ids: set[int] = set()
    for n in model.nodes:
        if n.id in node_ids:
            raise ModelError(DUPLICATE_NODE_ID, f"节点编号 {n.id} 重复。")
        node_ids.add(n.id)
        if not (math.isfinite(n.x) and math.isfinite(n.y)):
            raise ModelError(INVALID_SCHEMA, f"节点 {n.id} 的坐标不是有限数值。")
    xy = {n.id: (n.x, n.y) for n in model.nodes}

    # ---- 杆件：编号唯一、端点存在、截面量为正、杆长非零 ----
    element_ids: set[int] = set()
    for e in model.elements:
        if e.id in element_ids:
            raise ModelError(DUPLICATE_ELEMENT_ID, f"杆件编号 {e.id} 重复。")
        element_ids.add(e.id)
        for nid in (e.n1, e.n2):
            if nid not in node_ids:
                raise ModelError(UNKNOWN_NODE, f"杆件 {e.id} 的端点引用了不存在的节点 {nid}。")
        for name, value in (("E", e.E), ("A", e.A), ("I", e.I)):
            if not math.isfinite(value) or value <= 0.0:
                raise ModelError(INVALID_SECTION, f"杆件 {e.id} 的 {name} 必须为正数，当前值为 {value}。")
        x1, y1 = xy[e.n1]
        x2, y2 = xy[e.n2]
        if math.hypot(x2 - x1, y2 - y1) == 0.0:
            raise ModelError(ZERO_LENGTH_ELEMENT, f"杆件 {e.id} 的长度为零。")

    # ---- 支座：指向存在的节点、同一节点不重复定义 ----
    supported: set[int] = set()
    for sup in model.supports:
        if sup.node not in node_ids:
            raise ModelError(UNKNOWN_NODE, f"支座定义引用了不存在的节点 {sup.node}。")
        if sup.node in supported:
            raise ModelError(DUPLICATE_SUPPORT, f"节点 {sup.node} 上重复定义了支座。")
        supported.add(sup.node)

    # ---- 荷载：指向存在的节点/杆件、数值有限 ----
    for nl in model.loads.nodal:
        if nl.node not in node_ids:
            raise ModelError(UNKNOWN_NODE, f"节点荷载引用了不存在的节点 {nl.node}。")
        if not all(math.isfinite(v) for v in (nl.fx, nl.fy, nl.m)):
            raise ModelError(INVALID_SCHEMA, f"节点 {nl.node} 上的荷载不是有限数值。")
    for udl in model.loads.element_udl:
        if udl.element not in element_ids:
            raise ModelError(UNKNOWN_ELEMENT, f"均布荷载引用了不存在的杆件 {udl.element}。")
        if not math.isfinite(udl.q):
            raise ModelError(INVALID_SCHEMA, f"杆件 {udl.element} 上的均布荷载不是有限数值。")

    _check_connectivity(model, node_ids)
    _check_rigid_body_restraint(model, xy)


def _check_connectivity(model: FrameModel, node_ids: set[int]) -> None:
    """结构必须是一个整体：用并查集检查所有节点落在同一个连通分量里。"""
    parent = {nid: nid for nid in node_ids}

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for e in model.elements:
        ra, rb = find(e.n1), find(e.n2)
        if ra != rb:
            parent[ra] = rb

    roots = {find(nid) for nid in node_ids}
    if len(roots) > 1:
        raise ModelError(DISCONNECTED_STRUCTURE, f"结构被分成 {len(roots)} 个互不相连的部分。")


def _check_rigid_body_restraint(model: FrameModel, xy: dict[int, tuple[float, float]]) -> None:
    """受约束自由度必须能约束住平面刚体的三个运动：x 平动、y 平动、绕 z 转动。

    把每个约束写成刚体运动基 (T_x, T_y, R_z) 下的系数列：
    - 约束 u_x（作用于节点 (x, y)）：[1, 0, -y]
    - 约束 u_y：                   [0, 1,  x]
    - 约束 θ：                     [0, 0,  1]
    这些列张成的空间秩必须为 3，否则存在未被约束的刚体运动。
    """
    columns: list[list[float]] = []
    for sup in model.supports:
        x, y = xy[sup.node]
        if sup.ux:
            columns.append([1.0, 0.0, -y])
        if sup.uy:
            columns.append([0.0, 1.0, x])
        if sup.rz:
            columns.append([0.0, 0.0, 1.0])
    rank = int(np.linalg.matrix_rank(np.array(columns).T)) if columns else 0
    if rank < 3:
        raise ModelError(
            INSUFFICIENT_CONSTRAINTS,
            "约束不足以消除整体刚体运动（需要能独立约束 x 平动、y 平动和转动的约束组合）。",
        )
