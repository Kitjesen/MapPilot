from sim.tools.worlds.factory_park_hf.lookdev import surface_for


def test_surface_palette_keeps_marks_and_equipment_intact():
    for source, semantic in (
        ("road_paint", "lane_marking"),
        ("glass", "window"),
        ("rubber", "dock_bumper"),
        ("painted_steel", "safety_bollard"),
        ("oil_stain", "road_stain"),
    ):
        assert surface_for(source, semantic) is None


def test_world_scale_surface_palette():
    assert surface_for("asphalt", "road")[0:2] == ("asphalt_02", 400.0)
    assert surface_for("concrete", "curb")[0:2] == ("concrete_floor_worn_001", 300.0)
    assert surface_for("factory_cladding", "wall")[0] == "box_profile_metal_sheet"
    assert surface_for("warehouse_cladding", "warehouse_building")[0] == "box_profile_metal_sheet"
    assert surface_for("warehouse_cladding", "equipment_cabinet") is None
    assert surface_for("gravel", "terrain")[0] == "aerial_grass_rock"
    assert surface_for("unrelated", "equipment") is None
