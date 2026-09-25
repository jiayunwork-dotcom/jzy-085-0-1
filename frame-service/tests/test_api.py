"""HTTP 接入层测试：唯一入口 POST /solve 的契约。"""
from tests.conftest import mutated


def test_solve_success(client, portal_payload):
    resp = client.post("/solve", json=portal_payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body["success"] is True
    assert len(body["displacements"]) == 4
    assert len(body["element_forces"]) == 3
    assert len(body["reactions"]) == 2
    # 每个节点三个自由度、每根杆两端 N/V/M、每个支座三个反力分量
    assert set(body["displacements"][0]) == {"node", "ux", "uy", "rz"}
    assert set(body["element_forces"][0]["n1"]) == {"N", "V", "M"}
    assert set(body["reactions"][0]) == {"node", "fx", "fy", "mz"}


def test_invalid_model_returns_machine_readable_error(client):
    resp = client.post("/solve", json=mutated(supports=[]))
    assert resp.status_code == 422
    body = resp.json()
    assert body["success"] is False
    assert body["error"]["code"] == "INSUFFICIENT_CONSTRAINTS"
    assert body["error"]["message"]


def test_unknown_node_error(client):
    payload = mutated()
    payload["elements"][0]["n2"] = 99
    resp = client.post("/solve", json=payload)
    assert resp.status_code == 422
    assert resp.json()["error"]["code"] == "UNKNOWN_NODE"


def test_schema_error(client):
    resp = client.post("/solve", json={"nodes": "not-a-list"})
    assert resp.status_code == 422
    body = resp.json()
    assert body["success"] is False
    assert body["error"]["code"] == "INVALID_SCHEMA"


def test_malformed_json(client):
    resp = client.post("/solve", content="{not json", headers={"Content-Type": "application/json"})
    assert resp.status_code == 422
    assert resp.json()["success"] is False


def test_health(client):
    assert client.get("/health").json() == {"status": "ok"}
