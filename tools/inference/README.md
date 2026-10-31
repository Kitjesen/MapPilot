# Vision inference tools

This directory contains the optional development inference service and the
fixed-person model exporter used by person-follow work. Camera acquisition,
depth projection, Host modules, and motion decisions belong to their runtime
components under `src/`.

| Tool | Purpose |
| --- | --- |
| [`follow_service.py`](follow_service.py) | FastAPI service with `/health`, `/observe`, `/embed`, and `/chat` endpoints. |
| [`export_follow.py`](export_follow.py) | Export a fixed-person YOLO-World detector to ONNX and write its sibling `.contract.json`. |

The service reads explicit model paths from the environment:

| Variable | Model |
| --- | --- |
| `LINGTU_FOLLOW_YOLO_MODEL` | Detector |
| `LINGTU_FOLLOW_CLIP_MODEL` | CLIP encoder |
| `LINGTU_FOLLOW_DINO_MODEL` | DINO encoder |
| `LINGTU_FOLLOW_QWEN_MODEL` | Vision-language model |
| `LINGTU_FOLLOW_DEVICE` | Inference device; defaults to `cuda`. |

Use the existing model/runtime environment. The application is
`tools.inference.follow_service:app`; run it from the repository root with
`src/` on `PYTHONPATH`. Use one Uvicorn worker because the models and tracker
are process-local state. Model loading occurs during application startup.

Export a detector into an explicit output directory:

```bash
python -m tools.inference.export_follow --model PATH_TO_MODEL --output build/follow-model/person.onnx
```

Model deployment and hardware evidence are recorded in the
[person-follow research record](../../docs/research/person-follow.md). Local
HTTP contract tests do not demonstrate model accuracy or robot following.
