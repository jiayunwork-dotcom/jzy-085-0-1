"""杆上均布荷载（垂直杆轴、局部坐标）的等效节点荷载与固端力校核。"""
import pytest

from app.schemas import FrameModel
from app.service import solve_frame

E = 2.0e8
A = 1.0
I = 1.0e-4
L = 4.0
W = 10.0  # 向下均布荷载集度（局部 -y 方向）


def _beam_model(supports, extra_node=False):
    """水平单跨梁，局部 +y 即整体 +y，q=-W 表示竖直向下。"""
    nodes = [{"id": 1, "x": 0.0, "y": 0.0}, {"id": 2, "x": L, "y": 0.0}]
    elements = [{"id": 1, "n1": 1, "n2": 2, "E": E, "A": A, "I": I}]
    udl = [{"element": 1, "q": -W}]
    if extra_node:
        # 跨中加一个节点，便于直接读跨中位移
        nodes.insert(1, {"id": 3, "x": L / 2, "y": 0.0})
        elements = [
            {"id": 1, "n1": 1, "n2": 3, "E": E, "A": A, "I": I},
            {"id": 2, "n1": 3, "n2": 2, "E": E, "A": A, "I": I},
        ]
        udl = [{"element": 1, "q": -W}, {"element": 2, "q": -W}]
    return {
        "nodes": nodes,
        "supports": supports,
        "elements": elements,
        "loads": {"nodal": [], "element_udl": udl},
    }


def test_fixed_fixed_beam_end_forces():
    """两端固结梁满跨均布荷载：固端剪力 wL/2，固端弯矩 wL²/12（左逆右顺）。"""
    supports = [
        {"node": 1, "ux": True, "uy": True, "rz": True},
        {"node": 2, "ux": True, "uy": True, "rz": True},
    ]
    result = solve_frame(FrameModel(**_beam_model(supports)))
    elem = result.element_forces[0]
    assert elem.n1.V == pytest.approx(W * L / 2, rel=1e-9)
    assert elem.n2.V == pytest.approx(W * L / 2, rel=1e-9)
    assert elem.n1.M == pytest.approx(W * L**2 / 12, rel=1e-9)
    assert elem.n2.M == pytest.approx(-W * L**2 / 12, rel=1e-9)
    # 位移恒为零（无自由自由度），反力即固端力
    r1 = next(r for r in result.reactions if r.node == 1)
    assert r1.fy == pytest.approx(W * L / 2, rel=1e-9)
    assert r1.mz == pytest.approx(W * L**2 / 12, rel=1e-9)


def test_simply_supported_beam():
    """简支梁满跨均布荷载：支座反力 wL/2，跨中挠度 5wL⁴/(384EI)，跨中弯矩 wL²/8。"""
    supports = [
        {"node": 1, "ux": True, "uy": True, "rz": False},
        {"node": 2, "ux": False, "uy": True, "rz": False},
    ]
    result = solve_frame(FrameModel(**_beam_model(supports, extra_node=True)))

    for nid in (1, 2):
        r = next(r for r in result.reactions if r.node == nid)
        assert r.fy == pytest.approx(W * L / 2, rel=1e-9)

    mid = next(d for d in result.displacements if d.node == 3)
    assert mid.uy == pytest.approx(-5 * W * L**4 / (384 * E * I), rel=1e-9)

    # 跨中弯矩：左半杆 n2 端弯矩即跨中截面弯矩（sagging 为正）
    left_half = next(e for e in result.element_forces if e.element == 1)
    assert left_half.n2.M == pytest.approx(W * L**2 / 8, rel=1e-9)

    # 端部转角 wL³/(24EI)，左端顺时针、右端逆时针
    d1 = next(d for d in result.displacements if d.node == 1)
    d2 = next(d for d in result.displacements if d.node == 2)
    assert d1.rz == pytest.approx(-W * L**3 / 24 / (E * I), rel=1e-9)
    assert d2.rz == pytest.approx(W * L**3 / 24 / (E * I), rel=1e-9)


def test_udl_on_inclined_element_follows_local_axes():
    """斜杆上的均布荷载按局部坐标给定：等效合力应沿局部 +y（即整体 (-s, c)）方向。"""
    # 45° 斜杆，两端固结
    import math

    payload = {
        "nodes": [{"id": 1, "x": 0.0, "y": 0.0}, {"id": 2, "x": 3.0, "y": 3.0}],
        "supports": [
            {"node": 1, "ux": True, "uy": True, "rz": True},
            {"node": 2, "ux": True, "uy": True, "rz": True},
        ],
        "elements": [{"id": 1, "n1": 1, "n2": 2, "E": E, "A": A, "I": I}],
        "loads": {"nodal": [], "element_udl": [{"element": 1, "q": W}]},
    }
    result = solve_frame(FrameModel(**payload))
    r1 = next(r for r in result.reactions if r.node == 1)
    length = 3.0 * math.sqrt(2.0)
    # 局部 +y 在整体下为 (-√2/2, +√2/2)，反力合力应与荷载合力等大反向
    assert r1.fx == pytest.approx(-W * length / 2 * (-math.sqrt(2) / 2), rel=1e-9)
    assert r1.fy == pytest.approx(-W * length / 2 * (math.sqrt(2) / 2), rel=1e-9)
