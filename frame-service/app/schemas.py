"""HTTP 层的数据模型（请求/响应）。

符号约定：
- 整体坐标：x 向右，y 向上，转角/力矩以逆时针为正。
- 支座按自由度逐个给定：ux/uy/rz 为 true 表示该自由度被锁死。
  铰支 = {ux: true, uy: true}；固定端 = 三者全 true；滑动支座 = 只锁被限制的方向。
- 杆上均布荷载 q 垂直于杆轴，以杆件局部 +y 方向为正
  （局部 x 轴由 n1 指向 n2，局部 y 轴为局部 x 轴逆时针转 90°）。
"""
from __future__ import annotations

from pydantic import BaseModel, Field


class Node(BaseModel):
    id: int
    x: float
    y: float


class Support(BaseModel):
    node: int
    ux: bool = False
    uy: bool = False
    rz: bool = False


class Element(BaseModel):
    id: int
    n1: int
    n2: int
    E: float  # 弹性模量
    A: float  # 截面面积
    I: float  # 截面惯性矩


class NodalLoad(BaseModel):
    node: int
    fx: float = 0.0
    fy: float = 0.0
    m: float = 0.0


class ElementUDL(BaseModel):
    element: int
    q: float  # 垂直于杆轴的均布荷载集度，局部 +y 为正


class Loads(BaseModel):
    nodal: list[NodalLoad] = Field(default_factory=list)
    element_udl: list[ElementUDL] = Field(default_factory=list)


class FrameModel(BaseModel):
    nodes: list[Node]
    supports: list[Support] = Field(default_factory=list)
    elements: list[Element]
    loads: Loads = Field(default_factory=Loads)


class Displacement(BaseModel):
    node: int
    ux: float
    uy: float
    rz: float


class EndForce(BaseModel):
    N: float  # 轴力，沿局部 x 方向（n2 端为正表示受拉）
    V: float  # 剪力，沿局部 y 方向
    M: float  # 弯矩，逆时针为正


class ElementForces(BaseModel):
    element: int
    n1: EndForce
    n2: EndForce


class Reaction(BaseModel):
    node: int
    fx: float
    fy: float
    mz: float


class SolveResponse(BaseModel):
    success: bool = True
    displacements: list[Displacement]
    element_forces: list[ElementForces]
    reactions: list[Reaction]


class ErrorInfo(BaseModel):
    code: str
    message: str


class ErrorResponse(BaseModel):
    success: bool = False
    error: ErrorInfo
