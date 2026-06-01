# Semantic Perception Tests

Tests for VLN semantic perception: detector backends (YOLO-E, YOLO-World, BPU, Grounding DINO), encoder backends (CLIP, MobileCLIP), instance tracking, and scene graph construction.

```bash
python -m pytest src/semantic/perception/tests/ -q
```

No special markers required.
