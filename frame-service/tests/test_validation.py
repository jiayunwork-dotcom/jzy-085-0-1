"""非法输入必须被明确拒绝：返回对应的机器可读错误码，程序不崩、杆件不被静默丢弃。"""
import pytest

from app import errors
from app.errors import ModelError
from app.schemas import FrameModel
from app.service import solve_frame
from tests.conftest import base_frame_dict, mutated


def _code_of(payload: dict) -> str:
    with pytest.raises(ModelError) as exc_info:
        solve_frame(FrameModel(**payload))
    return exc_info.value.code


def test_duplicate_node_id():
    payload = base_frame_dict()
    payload["nodes"].append({"id": 2, "x": 9.0, "y": 9.0})
    assert _code_of(payload) == errors.DUPLICATE_NODE_ID


def test_duplicate_element_id():
    payload = base_frame_dict()
    payload["elements"].append({"id": 1, "n1": 2, "n2": 3, "E": 2.0e8, "A": 1.0, "I": 1.0e-4})
    assert _code_of(payload) == errors.DUPLICATE_ELEMENT_ID


def test_element_endpoint_unknown_node():
    payload = base_frame_dict()
    payload["elements"][0]["n2"] = 99
    assert _code_of(payload) == errors.UNKNOWN_NODE


def test_support_on_unknown_node():
    payload = base_frame_dict()
    payload["supports"].append({"node": 42, "ux": True, "uy": True, "rz": True})
    assert _code_of(payload) == errors.UNKNOWN_NODE


def test_nodal_load_on_unknown_node():
    payload = base_frame_dict()
    payload["loads"]["nodal"].append({"node": 42, "fx": 1.0})
    assert _code_of(payload) == errors.UNKNOWN_NODE


def test_udl_on_unknown_element():
    payload = base_frame_dict()
    payload["loads"]["element_udl"].append({"element": 42, "q": -5.0})
    assert _code_of(payload) == errors.UNKNOWN_ELEMENT


def test_zero_length_element_same_node():
    payload = base_frame_dict()
    payload["elements"][0]["n2"] = payload["elements"][0]["n1"]
    assert _code_of(payload) == errors.ZERO_LENGTH_ELEMENT


def test_zero_length_element_coincident_nodes():
    payload = base_frame_dict()
    payload["nodes"].append({"id": 5, "x": 0.0, "y": 0.0})  # 与节点 1 坐标重合
    payload["elements"].append({"id": 4, "n1": 1, "n2": 5, "E": 2.0e8, "A": 1.0, "I": 1.0e-4})
    assert _code_of(payload) == errors.ZERO_LENGTH_ELEMENT


@pytest.mark.parametrize("field", ["E", "A", "I"])
@pytest.mark.parametrize("value", [0.0, -1.0, -1.0e-6])
def test_non_positive_section_values(field, value):
    payload = base_frame_dict()
    payload["elements"][1][field] = value
    assert _code_of(payload) == errors.INVALID_SECTION


def test_insufficient_constraints_no_support_at_all():
    payload = mutated(supports=[])
    assert _code_of(payload) == errors.INSUFFICIENT_CONSTRAINTS


def test_insufficient_constraints_only_translations_at_one_node():
    # 单铰支座：锁不死转动刚体运动
    payload = mutated(supports=[{"node": 1, "ux": True, "uy": True, "rz": False}])
    assert _code_of(payload) == errors.INSUFFICIENT_CONSTRAINTS


def test_insufficient_constraints_parallel_sliding():
    # 两个只锁水平位移的滑动支座：竖向平动与转动都锁不死
    payload = mutated(
        supports=[
            {"node": 1, "ux": True, "uy": False, "rz": False},
            {"node": 4, "ux": True, "uy": False, "rz": False},
        ]
    )
    assert _code_of(payload) == errors.INSUFFICIENT_CONSTRAINTS


def test_disconnected_structure():
    payload = base_frame_dict()
    payload["nodes"] += [{"id": 5, "x": 10.0, "y": 0.0}, {"id": 6, "x": 14.0, "y": 0.0}]
    payload["elements"].append({"id": 4, "n1": 5, "n2": 6, "E": 2.0e8, "A": 1.0, "I": 1.0e-4})
    payload["supports"].append({"node": 5, "ux": True, "uy": True, "rz": True})
    assert _code_of(payload) == errors.DISCONNECTED_STRUCTURE


def test_isolated_node_counts_as_disconnected():
    payload = base_frame_dict()
    payload["nodes"].append({"id": 5, "x": 10.0, "y": 0.0})
    assert _code_of(payload) == errors.DISCONNECTED_STRUCTURE


def test_empty_model():
    assert _code_of(mutated(nodes=[])) == errors.EMPTY_MODEL
    assert _code_of(mutated(elements=[])) == errors.EMPTY_MODEL


def test_duplicate_support():
    payload = base_frame_dict()
    payload["supports"].append({"node": 1, "ux": True, "uy": True, "rz": True})
    assert _code_of(payload) == errors.DUPLICATE_SUPPORT
