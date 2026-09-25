"""门式刚架手算校核算例（examples/portal_frame.json）。

结构：左柱脚 A(0,0)、右柱脚 D(4,0) 均固结，柱高 h=4 m，横梁跨度 L=4 m；
左柱、横梁 EI = 2.0e8 × 1.0e-4 = 2.0e4 kN·m²，右柱 EI 加倍；
左柱顶 B 处施加水平集中力 P = 10 kN（+x 方向）。截面面积取得很大（A=100），
使轴向变形可忽略，从而可与结构力学教材中"忽略轴向变形"的手算结果对照。

手算（直接刚度/位移法，侧移 λ=Δ/h，k1=EI_左柱/h，k2=2k1，kb=EI_梁/L=k1）：
    节点 B 弯矩平衡：  6λ + 8θ_B + 2θ_C = 0
    节点 C 弯矩平衡： 12λ + 2θ_B + 12θ_C = 0
    层间剪力平衡：    36λ + 6θ_B + 12θ_C = P·h/k1
解得 θ_B = -12λ/23，θ_C = -21λ/23，λ = 23·P·h/(504·k1)。

回代得到（P=10, h=L=4）：
    H_A = -17P/42 ≈ -4.0476 kN      H_D = -25P/42 ≈ -5.9524 kN
    V_A = -11P/28 ≈ -3.9286 kN      V_D = +11P/28 ≈ +3.9286 kN
    M_A = 19Ph/84 ≈  9.0476 kN·m    M_D = 32Ph/84 ≈ 15.2381 kN·m
    横梁左端弯矩（杆端，逆时针正）= -5Ph/28 ≈ -7.1429 kN·m
    横梁右端弯矩                    = -3Ph/14 ≈ -8.5714 kN·m
    横梁跨中弯矩 = (M_右 - M_左)/2  = -Ph/56 ≈ -0.7143 kN·m
    柱顶侧移 Δ = 23Ph³/(504·EI_左柱) ≈ 1.4603e-3 m
"""
import pytest

from app.service import solve_frame

P = 10.0
H = 4.0
L = 4.0
EI_LEFT = 2.0e8 * 1.0e-4

# 手算结果与数值解的对比容差：
# 数值模型含微小轴向变形（A=100 而非无穷大），相对偏差约 1e-7，容差取 1e-4 足够严格又不吃亏
RTOL = 1e-4


@pytest.fixture()
def portal_result(portal_model):
    return solve_frame(portal_model)


def _reaction(result, node_id):
    return next(r for r in result.reactions if r.node == node_id)


def _element(result, element_id):
    return next(e for e in result.element_forces if e.element == element_id)


def test_base_horizontal_reactions(portal_result):
    ra = _reaction(portal_result, 1)
    rd = _reaction(portal_result, 4)
    assert ra.fx == pytest.approx(-17 * P / 42, rel=RTOL)
    assert rd.fx == pytest.approx(-25 * P / 42, rel=RTOL)


def test_base_vertical_reactions(portal_result):
    ra = _reaction(portal_result, 1)
    rd = _reaction(portal_result, 4)
    assert ra.fy == pytest.approx(-11 * P / 28, rel=RTOL)
    assert rd.fy == pytest.approx(11 * P / 28, rel=RTOL)


def test_base_moments(portal_result):
    ra = _reaction(portal_result, 1)
    rd = _reaction(portal_result, 4)
    assert ra.mz == pytest.approx(19 * P * H / 84, rel=RTOL)
    assert rd.mz == pytest.approx(32 * P * H / 84, rel=RTOL)


def test_beam_end_moments(portal_result):
    beam = _element(portal_result, 2)
    assert beam.n1.M == pytest.approx(-5 * P * H / 28, rel=RTOL)
    assert beam.n2.M == pytest.approx(-3 * P * H / 14, rel=RTOL)


def test_beam_midspan_moment(portal_result):
    """横梁上无横向荷载，弯矩沿跨长线性变化，跨中弯矩 = (M_右 - M_左)/2。"""
    beam = _element(portal_result, 2)
    midspan = (beam.n2.M - beam.n1.M) / 2.0
    assert midspan == pytest.approx(-P * H / 56, rel=RTOL)


def test_sway_displacement(portal_result):
    """柱顶水平位移与手算侧移一致（轴向变形影响 < 1e-6，容差放宽到 1e-3）。"""
    d2 = next(d for d in portal_result.displacements if d.node == 2)
    expected = 23 * P * H**3 / (504 * EI_LEFT)
    assert d2.ux == pytest.approx(expected, rel=1e-3)
    d3 = next(d for d in portal_result.displacements if d.node == 3)
    assert d3.ux == pytest.approx(expected, rel=1e-3)  # 横梁轴向刚性 → 两柱顶侧移相同
