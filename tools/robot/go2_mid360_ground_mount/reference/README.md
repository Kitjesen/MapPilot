# Official sensor geometry provenance

Downloaded 2026-09-21 for local mechanical fit evaluation.

- MID-360 original STEP: https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/mid-360-asm.stp
- D435i original SLDPRT: D435i_Solid.SLDPRT from https://dev.realsenseai.com/download/41950 (official D400 CAD bundle).

Original CAD copyright remains with the vendors. No redistribution license has been established. Original STEP/SLDPRT files are retained locally and excluded from this GitHub bundle; download them from the official links above. Registered display meshes and extracted interface data are provided for assembly evaluation, with vendor attribution; they are not original project designs.

Converted using the installed SolidWorks 2025, STL output in metres. MID-360 assembly components retain assembly coordinates. Converted display NPZ coordinates are in millimetres and registered by the rotations/translations in mount_interfaces.json. Vertices rounded to a 0.25 mm grid, collapsed/zero-area/duplicate triangles removed; no dimensional scaling was applied. Display meshes can contain internal surfaces and are not printable parts; collision and mass properties use separate envelopes.

MID-360 source bottom Y = -25.9171 mm; optical height is 47 mm above it. Four exact STEP Cartesian positions are retained in mount_interfaces.json. D435i mounting cylinders were extracted from the original SolidWorks analytic faces: centres (+/-22.5,0,-25.05) mm, axis +Z, minor radius 1.25 mm. Nominal M3 threads are specified by the manufacturer drawing; minor radius is not a clearance-hole diameter.

D435i rear origin is X=0, front X=25.05 mm. Depth optical nominal origin is [20.75,17.5,0] mm, based on vendor ROS description (glass setback 0.1 mm, zero-depth setback 4.2 mm). This project's camera_link is explicitly the rear mechanical datum, unlike the vendor's camera_link optical-origin convention.
