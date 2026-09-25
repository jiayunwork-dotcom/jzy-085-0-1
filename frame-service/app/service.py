"""求解流程编排：校验 → 组装 → 求解 → 回代 → 组装响应。"""
from __future__ import annotations

import numpy as np

from .assembly import assemble
from .postprocess import element_end_forces, support_reactions
from .schemas import (
    Displacement,
    ElementForces,
    EndForce,
    FrameModel,
    Reaction,
    SolveResponse,
)
from .solver import solve_reduced
from .validation import validate_model


def solve_frame(model: FrameModel) -> SolveResponse:
    """喂进一副刚架模型，吐出节点位移、杆端内力与支座反力。"""
    validate_model(model)
    asm = assemble(model)

    # 行列缩减：只解自由自由度，受约束自由度位移恒为零
    free = ~asm.restrained
    u_free = solve_reduced(asm.K[np.ix_(free, free)], asm.F[free])
    u = np.zeros(asm.F.shape[0])
    u[free] = u_free

    end_forces = element_end_forces(asm, u)
    reactions = support_reactions(asm, u)

    displacements = []
    for n in model.nodes:
        d = asm.dof_of_node[n.id]
        displacements.append(
            Displacement(node=n.id, ux=float(u[d[0]]), uy=float(u[d[1]]), rz=float(u[d[2]]))
        )

    forces = []
    for e in model.elements:
        f = end_forces[e.id]
        forces.append(
            ElementForces(
                element=e.id,
                n1=EndForce(N=float(f[0]), V=float(f[1]), M=float(f[2])),
                n2=EndForce(N=float(f[3]), V=float(f[4]), M=float(f[5])),
            )
        )

    reaction_list = [
        Reaction(node=s.node, fx=float(reactions[s.node][0]), fy=float(reactions[s.node][1]), mz=float(reactions[s.node][2]))
        for s in model.supports
    ]

    return SolveResponse(displacements=displacements, element_forces=forces, reactions=reaction_list)
