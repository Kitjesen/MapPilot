from .scene import ScenePCD, SceneMap, SceneTrav


class SceneStairs():
    pcd = ScenePCD()
    pcd.file_name = 'stairs_best.pcd'

    map = SceneMap()
    map.resolution = 0.10
    map.ground_h = -0.108
    map.slice_dh = 1.0

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.60
    trav.step_max = 0.2
    trav.standable_ratio = 0.20
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.4/1.0
    trav.inflation = 0.2/1.0
