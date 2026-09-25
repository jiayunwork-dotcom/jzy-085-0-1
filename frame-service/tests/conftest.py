"""测试共用的模型构造工具与夹具。"""
from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.schemas import FrameModel

EXAMPLES_DIR = Path(__file__).resolve().parent.parent / "examples"


def base_frame_dict() -> dict:
    """一副合法的门式刚架（即 examples/portal_frame.json），各非法用例在它上面做一点破坏。"""
    with open(EXAMPLES_DIR / "portal_frame.json", encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture()
def client() -> TestClient:
    return TestClient(app)


@pytest.fixture()
def portal_model() -> FrameModel:
    return FrameModel(**base_frame_dict())


@pytest.fixture()
def portal_payload() -> dict:
    return base_frame_dict()


def mutated(**changes) -> dict:
    """在合法门式刚架字典上按关键字覆盖，得到被破坏的模型。"""
    model = base_frame_dict()
    for key, value in changes.items():
        if value is None:
            model.pop(key, None)
        else:
            model[key] = value
    return copy.deepcopy(model)
