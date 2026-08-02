# External Simulation Scenes

External scenes are optional assets used for server-side validation. Keep large
or license-restricted assets out of git unless their redistribution terms are
confirmed.

Expected server layout for CMU Unity assets:

```text
sim/external_scenes/cmu_unity/
  environment/
    Model.x86_64
    Model_Data/
    Dimensions.csv
    Categories.csv
    AssetList.csv
  map.ply
  traversable_area.ply
  object_list.txt
```

These assets support `launch_cmu_unity_baseline.sh` and manual use of the pure
`cmu_unity_lingtu_adapter.py` topic relay. They are external experiments:
ProductControl does not resolve them and they do not produce a RunPlan or a
LingTu Product acceptance claim.

Expected server layout for imported Gazebo worlds:

```text
sim/external_scenes/gazebo/
  <scene_name>/
    worlds/
    models/
    maps/
```

Use a Product-backed scene for an acceptance claim only after its current gate
proves sensor topics, frame alignment, map growth, non-zero simulated motion,
obstacle clearance, and no hardware command output.
