# HSG-Nav Offline Evaluation Report

Generated: 2026-02-18 11:54

## Note

This report tests the **algorithmic core** of HSG-Nav using synthetic scene graphs.
It validates goal resolution, task decomposition, belief systems, and scheduling
**without** requiring real RGB-D data, ROS2, or a physical robot.

---

## Results

| Module | Metric | Value |
|--------|--------|-------|
| Fast Path (L1, English) | Resolution Rate | 100% (20/20) |
| Fast Path (L2, English) | Resolution Rate | 100% (15/15) |
| Fast Path (Chinese) | Falls to Slow Path | 100% (expected) |
| Fast Path | Avg Latency | <5ms |
| Task Decomposition (L1) | Success Rate | 100% (20/20) |
| Task Decomposition (L2) | Success Rate | 100% (15/15) |
| Task Decomposition (L3) | Rule Success Rate | 50% (5/10) |
| Task Decomposition (L3) | Note | Remaining 50% need LLM decomposition |
| BA-HSG Belief | P_exist convergence | >0.87 after 30 frames |
| BA-HSG Belief | Credibility range | 0.72-0.73 |
| VoI Scheduler | Continue rate | 77% |
| VoI Scheduler | Reperceive rate | 23% |
| VoI Scheduler | Slow reason rate | 0% |
| Multi-Hypothesis | Success Rate | 100% (20/20) |
| Multi-Hypothesis | Avg Attempts to Find | 1.6 |
