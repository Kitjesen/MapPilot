from gateway.schemas import InspectionRoutePointRequest


def test_new_inspection_route_point_defaults_to_photo_only_overview():
    point = InspectionRoutePointRequest(id="p1", x=1.0, y=2.0)
    assert point.action == "capture:overview"
    assert point.dwell == 0.0
