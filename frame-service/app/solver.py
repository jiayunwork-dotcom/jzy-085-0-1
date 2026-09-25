"""线性方程组求解。

只对行列缩减后的自由自由度求解。缩减后的总刚对稳定结构应为对称正定阵，
因此用 Cholesky 分解求解：一旦分解失败（矩阵奇异或不正定），说明结构存在
机构或约束不足，立即报错，绝不返回 NaN 蒙混过去。
"""
from __future__ import annotations

import numpy as np

from .errors import SINGULAR_STIFFNESS, ModelError

# 解的相对残差上限：||K u - f|| 超过该值即认为矩阵病态
RESIDUAL_RTOL = 1e-8


def solve_reduced(K_ff: np.ndarray, f_f: np.ndarray) -> np.ndarray:
    """求解 K_ff @ u = f_f，返回自由自由度的位移。"""
    n = K_ff.shape[0]
    if n == 0:
        # 全部自由度均被约束（例如两端固定的单杆）：位移恒为零，是合法情形
        return np.zeros(0)

    try:
        chol = np.linalg.cholesky(K_ff)
        y = np.linalg.solve(chol, f_f)
        u = np.linalg.solve(chol.T, y)
    except np.linalg.LinAlgError as exc:
        raise ModelError(
            SINGULAR_STIFFNESS,
            "缩减后的总刚度矩阵奇异或不正定：结构可能存在机构、约束不足，无法求解。",
        ) from exc

    if not np.all(np.isfinite(u)):
        raise ModelError(SINGULAR_STIFFNESS, "位移解出现非有限值，总刚度矩阵可能病态。")

    residual = float(np.linalg.norm(K_ff @ u - f_f))
    scale = max(1.0, float(np.linalg.norm(f_f)))
    if residual > RESIDUAL_RTOL * scale * n:
        raise ModelError(SINGULAR_STIFFNESS, "位移解残差过大，总刚度矩阵可能病态。")

    return u
