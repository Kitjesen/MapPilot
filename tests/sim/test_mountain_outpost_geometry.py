"""Shared mountain terrain and trail geometry stays grounded in meter units."""

import pytest
from sim.tools.assets.mountain_outpost_geometry import TerrainGrid


def test_height_query_matches_the_actual_terrain_triangles():
    # A non-planar quad exposes the difference from bilinear interpolation.
    terrain = TerrainGrid(span_m=2.0, heights=((0.0, 0.0), (0.0, 2.0)))

    assert terrain.height_at(0.5, -0.5) == pytest.approx(0.5)
    assert terrain.height_at(-0.5, 0.5) == pytest.approx(0.5)
    vertices, faces = terrain.mesh_data()
    assert vertices == [(-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (-1.0, 1.0, 0.0), (1.0, 1.0, 2.0)]
    assert faces == [(0, 1, 3), (0, 3, 2)]


def test_trail_is_a_feathered_ground_mask_instead_of_a_floating_ribbon():
    from sim.tools.assets.mountain_outpost_geometry import trail_weights

    vertices = [(0.0, 0.0, 9.0), (0.0, 1.0, -4.0), (0.0, 1.5, 2.0), (0.0, 2.0, 1.0)]
    weights = trail_weights(vertices, [(-4.0, 0.0), (4.0, 0.0)], width_m=2.0, feather_m=1.0)

    assert weights == pytest.approx([1.0, 1.0, 0.5, 0.0])
    assert vertices[0][2] == 9.0  # The material mask never moves terrain geometry.


def test_generated_terrain_uses_meters_and_is_repeatable():
    from sim.tools.assets.mountain_outpost_geometry import mountain_grid

    terrain = mountain_grid(samples=17)
    vertices, faces = terrain.mesh_data()
    assert terrain == mountain_grid(samples=17)
    assert len(vertices) == 17 * 17
    assert len(faces) == 16 * 16 * 2
    assert vertices[0][:2] == (-42.0, -42.0)
    assert vertices[-1][:2] == (42.0, 42.0)
    assert terrain.height_at(*vertices[-1][:2]) == vertices[-1][2]


def test_ground_queries_match_triangle_centers_across_the_generated_mesh():
    from sim.tools.assets.mountain_outpost_geometry import mountain_grid

    terrain = mountain_grid(samples=9)
    vertices, faces = terrain.mesh_data()
    for face in faces:
        center = [sum(vertices[index][axis] for index in face) / 3 for axis in range(3)]
        assert terrain.height_at(*center[:2]) == pytest.approx(center[2])


@pytest.mark.parametrize("x,y", [(-1, -1), (1, -1), (-1, 1), (1, 1)])
def test_ground_queries_include_all_four_terrain_boundaries(x, y):
    terrain = TerrainGrid(span_m=2.0, heights=((0.0, 1.0), (2.0, 3.0)))
    assert terrain.height_at(x, y) == pytest.approx((x + 1) / 2 + y + 1)


def test_trail_mask_follows_bends_and_has_no_extension_beyond_endpoints():
    from sim.tools.assets.mountain_outpost_geometry import trail_weights

    weights = trail_weights(
        [(0, 0, 0), (4, 2, 0), (4, 4, 0), (0, 4, 0), (-4, 0, 0)],
        [(0, 0), (4, 0), (4, 4)],
        width_m=1.0,
        feather_m=0.5,
    )
    assert weights == [1.0, 1.0, 1.0, 0.0, 0.0]


def test_background_is_continuous_terrain_not_vertical_backdrop_panels():
    from sim.tools.assets.mountain_outpost_geometry import background_mesh, mountain_grid

    terrain = mountain_grid(samples=9)
    vertices, faces = background_mesh(terrain)
    seam = [v for v in vertices if max(abs(v[0]), abs(v[1])) == 42.0]
    assert seam
    for x, y, z in seam:
        assert z == pytest.approx(terrain.height_at(x, y))
    for a, b, c in faces:
        ax, ay, _ = vertices[a]
        bx, by, _ = vertices[b]
        cx, cy, _ = vertices[c]
        assert (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) > 0
        center = [(vertices[a][i] + vertices[b][i] + vertices[c][i]) / 3 for i in (0, 1)]
        assert max(abs(center[0]), abs(center[1])) >= 42.0
