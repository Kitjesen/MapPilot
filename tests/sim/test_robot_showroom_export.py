"""Preserve face assignments when Blender clears and rebuilds material slots."""
from types import SimpleNamespace

from sim.tools.assets.export_robot_showroom import (
    _budget_display_meshes,
    _consolidate_material_slots,
    _display_plan,
)


def test_consolidation_restores_faces_after_blender_clear():
    silver, rubber = object(), object()
    polygons = [SimpleNamespace(material_index=index) for index in (0, 1, 2, 1)]

    class Materials(list):
        def clear(self):
            super().clear()
            # This is the relevant bpy side effect that caused all-silver exports.
            for polygon in polygons:
                polygon.material_index = 0

    materials = Materials([silver, rubber, silver])
    mesh = SimpleNamespace(
        material_slots=[SimpleNamespace(material=value) for value in materials],
        data=SimpleNamespace(polygons=polygons, materials=materials),
    )
    _consolidate_material_slots(mesh)
    assert materials == [silver, rubber]
    assert [face.material_index for face in polygons] == [0, 1, 0, 1]


def test_budget_keeps_small_mechanical_shells_intact(monkeypatch):
    import sys

    applied = []
    active = SimpleNamespace(active=None)

    class Modifiers:
        def new(self, name, kind):
            assert kind == "DECIMATE"
            self.modifier = SimpleNamespace(name=name, ratio=1)
            return self.modifier

    def part(name, polygons):
        return SimpleNamespace(name=name, data=SimpleNamespace(polygons=range(polygons)), modifiers=Modifiers())

    shell, wheel, motor = part("shell", 3614), part("wheel", 500_000), part("motor", 300_000)
    bpy = SimpleNamespace(context=SimpleNamespace(view_layer=SimpleNamespace(objects=active)),
        ops=SimpleNamespace(object=SimpleNamespace(modifier_apply=lambda **kwargs:
            applied.append((active.active.name, active.active.modifiers.modifier.ratio)))))
    monkeypatch.setitem(sys.modules, "bpy", bpy)
    _budget_display_meshes([shell, wheel, motor])
    assert [name for name, _ in applied] == ["wheel", "motor"]
    assert applied[0][1] == applied[1][1] == (600_000 - 3614) / 800_000
    assert len(shell.data.polygons) == 3614
    assert not hasattr(shell.modifiers, "modifier")


def test_budget_does_not_reduce_already_small_assets(monkeypatch):
    import sys

    # No bpy operator is provided: using one in the below-budget path fails.
    monkeypatch.setitem(sys.modules, "bpy", SimpleNamespace())
    _budget_display_meshes([SimpleNamespace(data=SimpleNamespace(polygons=range(4500)))])
    _budget_display_meshes([])


def test_budget_preserves_dense_cad_chassis(monkeypatch):
    import sys

    # A dense CAD shell must survive even when it alone exceeds the display budget.
    # No bpy operators are supplied, so attempting to decimate it fails this check.
    monkeypatch.setitem(sys.modules, "bpy", SimpleNamespace())
    chassis = SimpleNamespace(data=SimpleNamespace(polygons=range(700_000)))
    _budget_display_meshes([chassis], protected=[chassis])


def test_full_cad_display_replaces_links_and_keeps_source_plan(monkeypatch, tmp_path):
    import json

    import sim.tools.assets.export_robot_showroom as exporter

    paths = {"base_link": "base.stl", "fr_hip_link": "hip.stl"}
    for filename in paths.values():
        (tmp_path / filename).write_bytes(b"mesh")
    manifest = tmp_path / "display_meshes.json"
    paint = {"base_color_rgba": [0.015, 0.02, 0.025, 1.0]}
    (tmp_path / "r43_pads.stl").write_bytes(b"mesh")
    (tmp_path / "battery_led.stl").write_bytes(b"mesh")
    led = {"base_color_rgba": [0.03, 1.0, 0.08, 1.0],
           "emissive": {"rgb": [0.03, 1.0, 0.08], "strength": 6.0}}
    manifest.write_text(json.dumps({"meshes": paths, "paint_mesh": "DOSO.obj", "paint_material": paint,
        "materials": {"battery_led_green": led},
        "triangle_budget": 750_000, "additional_base_meshes": [
            {"name": "R43_pads", "mesh": "r43_pads.stl", "material": "rubber"},
            {"name": "battery_led", "mesh": "battery_led.stl", "material": "battery_led_green"}]}))
    monkeypatch.setattr(exporter, "THUNDER_DISPLAY_MANIFEST", manifest)
    source = {"id": "thunderv4", "geometries": [
        {"name": "base_link_unnamed_1", "type": "mesh", "mesh": "original/base_link.STL", "material": "silver", "position": [0, 0, 0.6]},
        {"name": "FR_hip", "type": "mesh", "mesh": "original/fr_hip_link.STL", "material": "silver"},
    ]}
    display, protected = _display_plan(source)
    assert [g["mesh"] for g in display["geometries"][:2]] == [str(tmp_path / path) for path in paths.values()]
    assert len(protected) == 4
    assert display["display_triangle_budget"] == 750_000
    assert display["geometries"][2]["material"] == "rubber"
    assert display["geometries"][2]["position"] == [0, 0, 0.6]
    assert display["geometries"][3]["material"] == "battery_led_green"
    assert display["geometries"][3]["position"] == [0, 0, 0.6]
    assert display["geometries"][4]["material"] == "doso_paint"
    assert display["materials"]["doso_paint"] == paint
    assert display["materials"]["battery_led_green"] == led
    assert source["geometries"][0]["mesh"] == "original/base_link.STL"
    assert len(source["geometries"]) == 2
    assert "materials" not in source
