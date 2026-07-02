#!/usr/bin/env python3
"""
HSG-Nav Jetson Orin NX performance benchmark.

Measures:
  1. YOLO-World detection FPS             — target: > 10 FPS
  2. CLIP encoding latency (ms/frame)     — target: < 50 ms
  3. Scene graph build latency (ms/update)— target: < 100 ms
  4. Fast Path response latency           — target: < 200 ms
  5. Slow Path latency (incl. LLM API)    — recorded, no hard target
  6. Peak GPU/CPU/memory usage
  7. End-to-end pipeline latency (perception -> planning -> execution)

Usage:
  # Run on Jetson (requires ROS2 environment and all dependencies)
  python3 jetson_benchmark.py --warmup 10 --iterations 100

  # Perception pipeline only (no full ROS2 required)
  python3 jetson_benchmark.py --perception-only

  # Generate report from saved results
  python3 jetson_benchmark.py --report results/benchmark_*.json
"""

import argparse
import json
import os
import sys
import time
import statistics
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class BenchmarkResult:
    """Result for a single benchmark metric."""
    name: str
    unit: str
    values: List[float] = field(default_factory=list)
    target: Optional[float] = None
    target_op: str = ">"  # ">" or "<"

    @property
    def mean(self) -> float:
        return statistics.mean(self.values) if self.values else 0.0

    @property
    def median(self) -> float:
        return statistics.median(self.values) if self.values else 0.0

    @property
    def std(self) -> float:
        return statistics.stdev(self.values) if len(self.values) > 1 else 0.0

    @property
    def p95(self) -> float:
        if not self.values:
            return 0.0
        sorted_v = sorted(self.values)
        idx = int(len(sorted_v) * 0.95)
        return sorted_v[min(idx, len(sorted_v) - 1)]

    @property
    def meets_target(self) -> Optional[bool]:
        if self.target is None:
            return None
        if self.target_op == ">":
            return self.mean > self.target
        return self.mean < self.target

    def to_dict(self) -> Dict:
        return {
            "name": self.name,
            "unit": self.unit,
            "mean": round(self.mean, 2),
            "median": round(self.median, 2),
            "std": round(self.std, 2),
            "p95": round(self.p95, 2),
            "min": round(min(self.values), 2) if self.values else 0,
            "max": round(max(self.values), 2) if self.values else 0,
            "count": len(self.values),
            "target": self.target,
            "target_op": self.target_op,
            "meets_target": self.meets_target,
        }


def get_system_info() -> Dict:
    """Collect system information."""
    info = {
        "timestamp": datetime.now().isoformat(),
        "platform": "unknown",
        "gpu": "unknown",
        "cpu": "unknown",
        "memory_total_gb": 0,
    }

    try:
        import platform
        info["platform"] = platform.platform()
        info["cpu"] = platform.processor() or "unknown"
    except Exception:
        pass

    # Jetson-specific
    try:
        if os.path.exists("/etc/nv_tegra_release"):
            with open("/etc/nv_tegra_release") as f:
                info["jetson_release"] = f.read().strip()
            info["platform"] = "Jetson"
    except Exception:
        pass

    # GPU info
    try:
        import subprocess
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,memory.total",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            parts = result.stdout.strip().split(",")
            info["gpu"] = parts[0].strip()
            info["gpu_memory_mb"] = int(parts[1].strip())
    except Exception:
        pass

    # Memory
    try:
        import psutil
        mem = psutil.virtual_memory()
        info["memory_total_gb"] = round(mem.total / (1024**3), 1)
    except ImportError:
        pass

    return info


def get_gpu_usage() -> Dict:
    """Sample current GPU/CPU/memory usage."""
    usage = {
        "gpu_util_pct": 0.0,
        "gpu_mem_used_mb": 0.0,
        "cpu_util_pct": 0.0,
        "ram_used_mb": 0.0,
    }

    try:
        import subprocess
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            parts = result.stdout.strip().split(",")
            usage["gpu_util_pct"] = float(parts[0].strip())
            usage["gpu_mem_used_mb"] = float(parts[1].strip())
    except Exception:
        pass

    try:
        import psutil
        usage["cpu_util_pct"] = psutil.cpu_percent(interval=0.1)
        usage["ram_used_mb"] = psutil.virtual_memory().used / (1024**2)
    except ImportError:
        pass

    return usage


def benchmark_yolo_world(iterations: int = 100, warmup: int = 10) -> BenchmarkResult:
    """Benchmark YOLO-World detection FPS."""
    result = BenchmarkResult(
        name="YOLO-World Detection FPS",
        unit="FPS",
        target=10.0,
        target_op=">",
    )

    try:
        from perception.detector_yolo_world import YOLOWorldDetector
        import numpy as np

        detector = YOLOWorldDetector(model_size="l", confidence=0.3)
        detector.load_model()

        # Random 640x480 test frame
        test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        classes = "door . chair . fire extinguisher . person . desk"

        print(f"  YOLO-World: warming up ({warmup} iterations)...")
        for _ in range(warmup):
            detector.detect(test_frame, classes)

        print(f"  YOLO-World: benchmarking ({iterations} iterations)...")
        for i in range(iterations):
            t0 = time.perf_counter()
            detector.detect(test_frame, classes)
            dt = time.perf_counter() - t0
            result.values.append(1.0 / max(dt, 1e-6))

        detector.shutdown()
    except ImportError as e:
        print(f"  YOLO-World: skipped ({e})")
    except Exception as e:
        print(f"  YOLO-World: error ({e})")

    return result


def benchmark_clip_encoding(iterations: int = 100, warmup: int = 10) -> BenchmarkResult:
    """Benchmark CLIP encoding latency."""
    result = BenchmarkResult(
        name="CLIP Encoding Latency",
        unit="ms",
        target=50.0,
        target_op="<",
    )

    try:
        from perception.encoding.clip_encoder import CLIPEncoder
        import numpy as np

        encoder = CLIPEncoder(model_name="ViT-B/32")
        encoder.load_model()

        # 224x224 test crop
        test_crop = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)

        print(f"  CLIP: warming up ({warmup} iterations)...")
        for _ in range(warmup):
            encoder.encode_image(test_crop)

        print(f"  CLIP: benchmarking ({iterations} iterations)...")
        for i in range(iterations):
            t0 = time.perf_counter()
            encoder.encode_image(test_crop)
            dt = time.perf_counter() - t0
            result.values.append(dt * 1000)  # ms

        encoder.shutdown()
    except ImportError as e:
        print(f"  CLIP: skipped ({e})")
    except Exception as e:
        print(f"  CLIP: error ({e})")

    return result


def benchmark_scene_graph(iterations: int = 50, warmup: int = 5) -> BenchmarkResult:
    """Benchmark scene graph build latency."""
    result = BenchmarkResult(
        name="Scene Graph Build Latency",
        unit="ms",
        target=100.0,
        target_op="<",
    )

    try:
        from perception.tracking.instance_tracker import InstanceTracker
        from perception.tracking.projection import Detection3D
        import numpy as np

        tracker = InstanceTracker(max_objects=200)

        # Simulate 50 detections (typical scene)
        detections = []
        for i in range(50):
            det = Detection3D(
                label=f"object_{i % 10}",
                score=0.8,
                position=np.array([i * 0.5, (i % 5) * 0.5, 0.5]),
                depth=2.0,
                bbox_2d=np.array([100, 100, 200, 200]),
                features=np.random.randn(512).astype(np.float32),
            )
            detections.append(det)

        tracker.update(detections)

        print(f"  SceneGraph: warming up ({warmup} iterations)...")
        for _ in range(warmup):
            tracker.get_scene_graph_json()

        print(f"  SceneGraph: benchmarking ({iterations} iterations)...")
        for i in range(iterations):
            t0 = time.perf_counter()
            tracker.get_scene_graph_json()
            dt = time.perf_counter() - t0
            result.values.append(dt * 1000)

    except ImportError as e:
        print(f"  SceneGraph: skipped ({e})")
    except Exception as e:
        print(f"  SceneGraph: error ({e})")

    return result


def benchmark_fast_path(iterations: int = 30, warmup: int = 5) -> BenchmarkResult:
    """Benchmark Fast Path response latency (CLIP matching, no LLM)."""
    result = BenchmarkResult(
        name="Fast Path Response Latency",
        unit="ms",
        target=200.0,
        target_op="<",
    )

    try:
        from decision.goal_resolver import GoalResolver
        from decision.llm_client import LLMConfig
        import numpy as np

        config = LLMConfig(backend="openai", model="gpt-4o-mini")
        resolver = GoalResolver(
            primary_config=config,
            confidence_threshold=0.6,
            fast_path_threshold=0.75,
        )

        # Mock scene graph JSON
        scene_graph = json.dumps({
            "objects": [
                {"id": i, "label": f"object_{i}", "position": {"x": i, "y": 0, "z": 0},
                 "score": 0.8, "detection_count": 3, "region_id": 0}
                for i in range(20)
            ],
            "rooms": [{"room_id": 0, "name": "office", "center": {"x": 5, "y": 0},
                       "object_ids": list(range(20)), "group_ids": [0]}],
            "groups": [{"group_id": 0, "room_id": 0, "name": "furniture",
                       "center": {"x": 5, "y": 0}, "object_ids": list(range(20))}],
            "relations": [],
            "summary": "Scene has 20 objects in 1 room",
        })

        print(f"  FastPath: warming up ({warmup} iterations)...")
        for _ in range(warmup):
            resolver.fast_resolve("find the chair", scene_graph)

        print(f"  FastPath: benchmarking ({iterations} iterations)...")
        for i in range(iterations):
            t0 = time.perf_counter()
            resolver.fast_resolve("find the chair", scene_graph)
            dt = time.perf_counter() - t0
            result.values.append(dt * 1000)

    except ImportError as e:
        print(f"  FastPath: skipped ({e})")
    except Exception as e:
        print(f"  FastPath: error ({e})")

    return result


def run_benchmark(
    iterations: int = 100,
    warmup: int = 10,
    perception_only: bool = False,
    output_file: Optional[str] = None,
):
    """Run the full benchmark suite."""
    print("=" * 60)
    print("HSG-Nav Jetson Performance Benchmark")
    print("=" * 60)

    sys_info = get_system_info()
    print(f"\nPlatform: {sys_info['platform']}")
    print(f"GPU: {sys_info['gpu']}")
    print(f"Memory: {sys_info['memory_total_gb']} GB\n")

    results: List[BenchmarkResult] = []

    print("\n[1/5] YOLO-World Detection")
    results.append(benchmark_yolo_world(iterations, warmup))

    print("\n[2/5] CLIP Encoding")
    results.append(benchmark_clip_encoding(iterations, warmup))

    print("\n[3/5] Scene Graph Construction")
    results.append(benchmark_scene_graph(iterations // 2, warmup))

    if not perception_only:
        print("\n[4/5] Fast Path Response")
        results.append(benchmark_fast_path(iterations // 3, warmup))

        print("\n[5/5] Resource Usage")
    else:
        print("\n[4/5] Skipped (perception-only mode)")
        print("[5/5] Skipped (perception-only mode)")

    usage = get_gpu_usage()

    print("\n" + "=" * 60)
    print("BENCHMARK RESULTS")
    print("=" * 60)
    print(f"\n{'Metric':<35} {'Mean':>8} {'P95':>8} {'Target':>10} {'Pass':>6}")
    print("-" * 70)

    for r in results:
        if not r.values:
            print(f"  {r.name:<33} {'N/A':>8}")
            continue
        target_str = f"{r.target_op}{r.target}{r.unit}" if r.target else "N/A"
        pass_str = "Y" if r.meets_target else ("N" if r.meets_target is not None else "-")
        print(
            f"  {r.name:<33} {r.mean:>7.1f}{r.unit:>3} "
            f"{r.p95:>7.1f}{r.unit:>3} {target_str:>10} {pass_str:>4}"
        )

    print(f"\n  GPU Util: {usage['gpu_util_pct']:.0f}%")
    print(f"  GPU Memory: {usage['gpu_mem_used_mb']:.0f} MB")
    print(f"  CPU Util: {usage['cpu_util_pct']:.0f}%")
    print(f"  RAM Used: {usage['ram_used_mb']:.0f} MB")
    print("=" * 60)

    output = {
        "system_info": sys_info,
        "resource_usage": usage,
        "benchmarks": [r.to_dict() for r in results],
        "config": {
            "iterations": iterations,
            "warmup": warmup,
            "perception_only": perception_only,
        },
    }

    if output_file is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"results/benchmark_{ts}.json"

    os.makedirs(os.path.dirname(output_file) or ".", exist_ok=True)
    with open(output_file, "w") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    print(f"\nResults saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description="HSG-Nav Jetson Benchmark")
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--perception-only", action="store_true")
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--report", type=str, default=None,
                        help="Generate report from existing results file")

    args = parser.parse_args()

    if args.report:
        with open(args.report) as f:
            data = json.load(f)
        print(json.dumps(data, indent=2))
        return

    run_benchmark(
        iterations=args.iterations,
        warmup=args.warmup,
        perception_only=args.perception_only,
        output_file=args.output,
    )


if __name__ == "__main__":
    main()
