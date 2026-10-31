"""An operating area selects original components, never clips collision geometry."""

from copy import deepcopy
from xml.etree import ElementTree as ET

import mujoco
from sim.tools.worlds.factory_environment_collection.collision_candidate import convert_manifest

from tests.sim.test_factory_collision_candidate import available, box, source_fixture


def test_crossing_floor_is_kept_whole_and_curve_helper_is_not_missing_collision():
    from sim.tools.worlds.factory_environment_collection.bay_layout import review_operating_area

    source = source_fixture(shapes=[box(size=(2000, 2000, 40))])
    curve = dict(source_id="/actor/curve", kind="other_scene_component", class_="SplineComponent")
    curve["class"] = curve.pop("class_")
    curve.update(visual_bounds_min_cm=[-2000, 0, 0], visual_bounds_max_cm=[2000, 0, 0])
    source["actors"][0]["components"].append(curve)
    _, candidate = convert_manifest(source)
    before = deepcopy((source, candidate))
    result = review_operating_area(source, candidate, minimum_m=[-1, -1, -1], maximum_m=[1, 1, 2])
    assert result["component_ids"] == ["/actor/component"]
    assert result["geometry_ids"] == candidate["components"][0]["geometry_ids"]
    assert result["retained_geometry_bounds_m"] == dict(min_m=[-10, -10, -0.2], max_m=[10, 10, 0.2])
    assert result["geometry_clipped"] is False
    assert result["dispositions"]["/actor/curve"]["decision"] == "authoring_curve_not_render_geometry"
    assert result["blockers"] == []
    assert (source, candidate) == before


def test_skeletal_in_operating_area_is_a_concrete_blocker_not_dropped():
    from sim.tools.worlds.factory_environment_collection.bay_layout import review_operating_area

    source = source_fixture()
    source["actors"][0]["components"].append(
        dict(
            source_id="/actor/lift",
            kind="skeletal_mesh",
            **{"class": "SkeletalMeshComponent"},
            visual_bounds_min_cm=[0, 0, 0],
            visual_bounds_max_cm=[100, 100, 300],
        )
    )
    _, candidate = convert_manifest(source)
    report = review_operating_area(source, candidate, minimum_m=[-1, -1, -1], maximum_m=[2, 2, 4])
    assert report["blockers"] == [
        dict(source_id="/actor/lift", reason="blocked_missing_source_collision", kind="skeletal_mesh")
    ]
    assert report["qualified_world"] is report["spawn_approved"] is False


def test_materialized_area_keeps_whole_original_geoms_and_their_source_identity():
    from sim.tools.worlds.factory_environment_collection.bay_layout import materialize_selection, review_operating_area

    source = source_fixture(shapes=[box(size=(2000, 2000, 40))])
    other = deepcopy(source["actors"][0]["components"][0])
    other["source_id"] = "/actor/far"
    other["world_transform"]["translation_cm"] = [10000, 0, 0]
    source["actors"][0]["components"].append(other)
    xml, candidate = convert_manifest(source)
    selection = review_operating_area(source, candidate, minimum_m=[-1, -1, -1], maximum_m=[1, 1, 2])
    output, subset = materialize_selection(xml, candidate, selection)
    model = mujoco.MjModel.from_xml_string(output)
    assert model.ngeom == len(subset["geometries"]) == 1
    assert subset["geometries"] == [candidate["geometries"][0]]
    assert subset["components"] == [candidate["components"][0]]
    assert subset["summary"]["accepted_geometries"] == 1
    assert subset["summary"]["components_with_geometry"] == 1
    assert subset["collision_bounds"] == selection["retained_geometry_bounds_m"]
    original_geom = ET.fromstring(xml).find(".//geom[@name='source_geom_00000']")
    final_geom = ET.fromstring(output).find(".//geom[@name='source_geom_00000']")
    assert final_geom.attrib == original_geom.attrib
    assert subset["qualified_collision"] is subset["published_world"] is False
