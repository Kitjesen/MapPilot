import json
from pathlib import Path

import pytest
from sim.tools.worlds.factory_park_hf.inspection_route import create_inspection_route

from gateway.schemas import InspectionRouteRequest


def test_factory_route_uses_existing_checkpoints_and_public_native_task_contract():
    root = Path(__file__).resolve().parents[2]
    layout = json.loads((root / "sim/packages/worlds/factory_park_hf/generated/expanded-layout.json").read_text(encoding="utf-8"))
    route = create_inspection_route(layout, map_id="saved-factory", map_content_epoch=9, revision=2)
    request = InspectionRouteRequest.model_validate(route)
    by_id = {point["id"]: point for point in layout["checkpoints"]}
    assert [point.id for point in request.points] == ["checkpoint_gate", "checkpoint_warehouse", "checkpoint_tank_farm"]
    for point in request.points:
        assert [point.x, point.y, point.z] == by_id[point.id]["position_m"]
        assert point.action == "capture:overview"
        assert point.dwell > 0
    assert request.map_id == "saved-factory"
    assert request.map_content_epoch == 9
    assert request.revision == 2


@pytest.mark.parametrize("epoch", [0, -1, True])
def test_factory_route_requires_real_saved_map_identity(epoch):
    with pytest.raises(ValueError, match="map"):
        create_inspection_route({"checkpoints": []}, map_id="factory", map_content_epoch=epoch, revision=1)
