"""Object detection backends (2D detectors + detector registry).

Backends: yoloe, yolo_world, grounding_dino, bpu, sim_scene. Each backend
subclasses ``DetectorBase`` (see ``detector_base.py``) and is registered
under the ``detector`` category via ``runtime.registry``.
"""

__all__: list[str] = []
