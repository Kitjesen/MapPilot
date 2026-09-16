# V2 YOLO11s Rect 640x320 RDK Compile Input

This package fixes the camera geometry mismatch.

- Camera single-eye image after rotation is portrait-like.
- Static square 640x640 introduces large side padding.
- This candidate uses static rectangular input `1x3x640x320`.
- Runtime input is RGB NHWC, INT8/S8, with `scale_value=1/255`.
- Expected output is `1x10x4200`.
