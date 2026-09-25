"""求解器层面的奇异保护：总刚奇异必须报错，绝不返回 NaN。"""
import numpy as np
import pytest

from app.errors import SINGULAR_STIFFNESS, ModelError
from app.solver import solve_reduced


def test_exactly_singular_matrix_raises():
    # 一个无约束单杆的缩减总刚：轴向刚度块奇异（刚体模态未被约束）
    K = np.array([[1.0, -1.0], [-1.0, 1.0]])
    with pytest.raises(ModelError) as exc_info:
        solve_reduced(K, np.array([1.0, 2.0]))
    assert exc_info.value.code == SINGULAR_STIFFNESS


def test_indefinite_matrix_raises():
    K = np.array([[1.0, 0.0], [0.0, -1.0]])
    with pytest.raises(ModelError) as exc_info:
        solve_reduced(K, np.array([1.0, 1.0]))
    assert exc_info.value.code == SINGULAR_STIFFNESS


def test_zero_free_dofs_is_legal():
    # 全部自由度被约束（如两端固结单杆）：位移恒为零，不是错误
    u = solve_reduced(np.zeros((0, 0)), np.zeros(0))
    assert u.size == 0


def test_well_conditioned_system_solves():
    K = np.array([[4.0, 1.0], [1.0, 3.0]])
    f = np.array([1.0, 2.0])
    u = solve_reduced(K, f)
    assert np.allclose(K @ u, f)
