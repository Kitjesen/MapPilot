"""Cross-frame tracking: instance tracking, 2D re-id/association, projection.

``instance_tracker.py`` is the production scene-graph tracker consumed by
``PerceptionModule``. ``projection.py`` turns 2D detections + depth into the
``Detection3D`` records that feed the tracker. ``bpu_tracker.py`` and
``bpu_qp_bridge.py`` are BPU-hardware-specific 2D tracking/re-id adapters.
"""

__all__: list[str] = []
