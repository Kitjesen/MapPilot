# Factory corridor background

`factory-corridor.png` is a clean 1920×1080 UE 5.8.1 render of the user's
licensed Factory Environment Collection by Denys Rutkovskyi:
https://www.fab.com/listings/2ee66462-8c2b-4303-892c-83f7fc0d9b3e

The acquired original stays in `D:/epic/FabLibrary`; the working copy stays in
`D:/epic/Projects/FactoryPreview`. The capture uses `/Game/Maps/Demonstration`,
not a generated concept image. No original mesh, texture or map package is
embedded in this front-end resource. Fab license terms still govern use of
the source collection; this file does not relicense it as public-domain art.

Reproduction: `sim/tools/assets/capture_factory_frontend.py`, with the existing
external project's DDC. Successful capture diagnostics:
`build/unreal-tests/factory-menu-20261020/factory-capture-r4/`.

The source map was not saved. Editor overlays, chromatic aberration and motion
blur are excluded. The legacy collection's unbuilt-lighting/old Blueprint
warnings remain source-scene limitations, not qualified gameplay behavior.

The welcome screen uses this static render to avoid loading the entire factory
for menu navigation. It is not an interactive or MuJoCo-collision-ready map.
Robot selection retains its separate live 3D inspection viewport.
