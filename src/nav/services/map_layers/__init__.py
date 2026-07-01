"""Real-time L2 map-layer Modules.

These modules build runtime spatial layers such as occupancy, voxel, ESDF,
elevation, and traversability from incoming map clouds. The lifecycle facade
remains `nav.services.maps.MapService`.

Hot-path map math belongs in `lingtu_nav_kernel` via `cpp_backend.py`; these files own
Module ports, throttling, frame metadata, and payload packing.
"""
