"""Public review writes evidence findings without submitting motion commands."""

from tests.gateway.test_gateway_inspection_routes import _client, _persist_evidence


def test_review_api_survives_store_reload_and_leaves_navigation_untouched(monkeypatch, tmp_path):
    from runtime.contracts.inspection_evidence import InspectionEvidenceStore

    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(tmp_path))
    _persist_evidence(tmp_path)
    client, native = _client(monkeypatch)
    before = native.calls.copy()
    response = client.post("/api/v1/inspection/evidence/evidence-001/review",
                           json=dict(verdict="anomaly", note="Red indicator"))
    assert response.status_code == 200
    assert response.json()["evidence"]["review"]["verdict"] == "anomaly"
    assert native.calls == before
    assert InspectionEvidenceStore(tmp_path).get("evidence-001").manifest["review"]["note"] == "Red indicator"
    assert client.get("/api/v1/inspection/evidence/evidence-001").json()["evidence"]["review"]["verdict"] == "anomaly"


def test_review_api_rejects_invalid_findings_and_unknown_evidence(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(tmp_path))
    client, _ = _client(monkeypatch)
    assert client.post("/api/v1/inspection/evidence/missing/review", json=dict(verdict="success")).status_code == 422
    assert client.post("/api/v1/inspection/evidence/missing/review", json=dict(verdict="normal")).status_code == 404
