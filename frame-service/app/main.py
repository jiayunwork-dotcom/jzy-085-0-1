"""FastAPI 接入层：对外只有一个求解入口 POST /solve。"""
from __future__ import annotations

from fastapi import FastAPI
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

from .errors import INVALID_SCHEMA, ModelError
from .schemas import ErrorResponse, FrameModel, SolveResponse
from .service import solve_frame

app = FastAPI(
    title="平面刚架静力核算服务",
    version="1.0.0",
    description="直接刚度法求解平面刚架：输入几何/截面/荷载，输出节点位移、杆端内力与支座反力。",
)


@app.exception_handler(ModelError)
async def model_error_handler(_request, exc: ModelError) -> JSONResponse:
    """非法模型与奇异矩阵：HTTP 422 + 机器可读错误码与可读说明。"""
    body = ErrorResponse(error={"code": exc.code, "message": exc.message})
    return JSONResponse(status_code=422, content=body.model_dump())


@app.exception_handler(RequestValidationError)
async def schema_error_handler(_request, exc: RequestValidationError) -> JSONResponse:
    """请求体连数据格式都不符合（缺字段、类型错误等）。"""
    first = exc.errors()[0] if exc.errors() else {}
    location = ".".join(str(part) for part in first.get("loc", []))
    message = f"请求体不符合模型格式：{location} {first.get('msg', '')}".strip()
    body = ErrorResponse(error={"code": INVALID_SCHEMA, "message": message})
    return JSONResponse(status_code=422, content=body.model_dump())


@app.post(
    "/solve",
    response_model=SolveResponse,
    responses={422: {"model": ErrorResponse, "description": "模型非法或总刚奇异"}},
)
def solve(model: FrameModel) -> SolveResponse:
    return solve_frame(model)


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}
