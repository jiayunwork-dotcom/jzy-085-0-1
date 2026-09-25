"""力学恒等式守门类测试：平衡方程必须在给定容差内成立。

容差一律写成具体数值：本文件统一取 TOL = 1e-6（模型荷载量级为 10^1 kN，
双精度求解的数值残差在 1e-10 以下，1e-6 既严格又稳定）。
"""
import math

import pytest

from app.schemas import FrameModel
from app.service import solve_frame

TOL = 1e-6  # 力 / 力矩平衡的绝对容差（kN / kN·m）

# 一副受组合荷载的门式刚架：横梁满跨均布荷载 + 两个节点集中荷载
MODEL = {
    "nodes": [
        {"id": 1, "x": 0.0, "y": 0.0},
        {"id": 2, "x": 0.0, "y": 4.0},
        {"id": 3, "x": 4.0, "y": 4.0},
        {"id": 4, "x": 4.0, "y": 0.0},
    ],
    "supports": [
        {"node": 1, "ux": True, "uy": True, "rz": True},
        {"node": 4, "ux": True, "uy": True, "rz": True},
    ],
    "elements": [
        {"id": 1, "n1": 1, "n2": 2, "E": 2.0e8, "A": 0.02, "I": 1.0e-4},
        {"id": 2, "n1": 2, "n2": 3, "E": 2.0e8, "A": 0.02, "I": 1.0e-4},
        {"id": 3, "n1": 3, "n2": 4, "E": 2.0e8, "A": 0.02, "I": 2.0e-4},
    ],
    "loads": {
        "nodal": [
            {"node": 2, "fx": 10.0, "fy": -5.0, "m": 3.0},
            {"node": 3, "fx": 0.0, "fy": -8.0, "m": -2.0},
        ],
        "element_udl": [{"element": 2, "q": -6.0}],
    },
}

XY = {n["id"]: (n["x"], n["y"]) for n in MODEL["nodes"]}
ELEM_ENDS = {e["id"]: (e["n1"], e["n2"]) for e in MODEL["elements"]}


@pytest.fixture(scope="module")
def result():
    return solve_frame(FrameModel(**MODEL))


def _applied_loads_global():
    """全部外荷载在整体坐标下的 (作用点, fx, fy, m) 列表：节点荷载 + 均布荷载合力。"""
    loads = []
    for nl in MODEL["loads"]["nodal"]:
        loads.append((XY[nl["node"]], nl["fx"], nl["fy"], nl["m"]))
    for udl in MODEL["loads"]["element_udl"]:
        n1, n2 = ELEM_ENDS[udl["element"]]
        x1, y1 = XY[n1]
        x2, y2 = XY[n2]
        length = math.hypot(x2 - x1, y2 - y1)
        c, s = (x2 - x1) / length, (y2 - y1) / length
        # 均布荷载合力 qL 作用于杆件中点，方向为局部 +y = 整体 (-s, c)
        fx = udl["q"] * length * (-s)
        fy = udl["q"] * length * c
        loads.append((((x1 + x2) / 2, (y1 + y2) / 2), fx, fy, 0.0))
    return loads


def test_global_force_equilibrium(result):
    """全部支座反力 + 全部外荷载：水平与竖向合力代数和为零。"""
    sum_fx = sum(r.fx for r in result.reactions) + sum(f[1] for f in _applied_loads_global())
    sum_fy = sum(r.fy for r in result.reactions) + sum(f[2] for f in _applied_loads_global())
    assert abs(sum_fx) <= TOL
    assert abs(sum_fy) <= TOL


@pytest.mark.parametrize("point", [(0.0, 0.0), (2.5, -1.0), (-3.0, 7.0)])
def test_global_moment_equilibrium_about_any_point(result, point):
    """对任一固定点的合力矩代数和为零。"""
    x0, y0 = point
    moment = 0.0
    for r in result.reactions:
        x, y = XY[r.node]
        moment += (x - x0) * r.fy - (y - y0) * r.fx + r.mz
    for (x, y), fx, fy, m in _applied_loads_global():
        moment += (x - x0) * fy - (y - y0) * fx + m
    assert abs(moment) <= TOL


def test_element_axial_forces_equal_and_opposite(result):
    """同一根杆两端的轴力等值反向：N1 + N2 = 0。"""
    for ef in result.element_forces:
        assert abs(ef.n1.N + ef.n2.N) <= TOL * max(1.0, abs(ef.n1.N), abs(ef.n2.N))


def test_joint_moment_balance(result):
    """每个节点处：交汇各杆的杆端弯矩之和 = 外加力矩 + 支座反力矩。"""
    applied_m = {nid: 0.0 for nid in XY}
    for nl in MODEL["loads"]["nodal"]:
        applied_m[nl["node"]] += nl["m"]
    reaction_m = {r.node: r.mz for r in result.reactions}

    end_moments = {ef.element: (ef.n1.M, ef.n2.M) for ef in result.element_forces}
    for nid in XY:
        total = 0.0
        for eid, (n1, n2) in ELEM_ENDS.items():
            if n1 == nid:
                total += end_moments[eid][0]
            elif n2 == nid:
                total += end_moments[eid][1]
        rhs = applied_m[nid] + reaction_m.get(nid, 0.0)
        assert abs(total - rhs) <= TOL, f"节点 {nid} 弯矩不平衡：{total} vs {rhs}"
