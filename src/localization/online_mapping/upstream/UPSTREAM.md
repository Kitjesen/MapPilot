# LIO-SAM backend provenance

- Upstream: https://github.com/TixiaoShan/LIO-SAM
- Revision: `0be1fbe6275fb8366d5b800af4fc8c76a885c869`
- Retrieved: 2026-09-24
- License: BSD-3-Clause; see LICENSE. Retain this notice in binary releases.
- `research/localization/lio-sam-upstream/mapOptmization.cpp` is the unmodified
  reference, outside product sources.
  It is never compiled by the native target.

`../backend.cpp` ports addOdomFactor, saveKeyFramesAndFactor's iSAM2 updates,
detectLoopClosureDistance, loopFindNearKeyframes, performLoopClosure, and
correctPoses. Native GTSAM Pose3 factors and PCL ICP implement the mathematics.
Initial prior, odometry variances and iSAM2 settings start at upstream values.
These fixed variances are NOT calibrated Fast-LIO covariances. The 2026-09-25
candidate adds the explicitly documented verification changes below; it is no
longer a numerical-parity claim for upstream loop processing.

Explicit adaptations:

- Continuous Fast-LIO body odometry replaces LIO-SAM's feature-mapping odometry.
  Between measurements use original odometry deltas; corrected estimates only
  initialize new states. Raw poses must never be replaced with optimized poses.
- Deskewed full body-local clouds replace the two feature-cloud containers.
- Native input structs replace ROS messages, publishers and timer/thread wiring.
  The caller owns keyframe selection, timestamp continuity and sensor quality.
- Single worker performs one loop check per appended keyframe and applies a
  successful loop immediately, including when no later keyframe arrives.
- Linear nearest-within-radius selection replaces the spatial index for the
  bounded keyframe set; same distance/time criteria, deterministic tie order.
- Every target frame, not just its center, must be older than min_time_s. The
  current source can never appear in its historical target.
- Cache body-local voxel-filtered clouds, transform them at the current optimized
  poses, then voxel-filter the merged submap. This changes voxel grouping versus
  upstream and must be included in replay comparisons. Full saved scans remain.
- ICP correspondence range is independent of candidate radius. Require source
  overlap inside an inlier-distance bound in addition to convergence and fitness.
- Loop angular sigma and translation sigma have separate units. Translation
  variance is inflated by inlier mean-square residual; angular sigma is a fixed
  tuning parameter, not an ICP covariance estimate. GTSAM block Huber noise is
  used on loop factors. Odometry remains Gaussian with configurable six variances.
- Persist resolved parameters and one decision record per appended keyframe,
  including target membership, accepted measurement/noise and rejection reason.
- GPS factors and the unused external-loop ROS interface are not included.

This is an extracted backend candidate, not a claim that the entire LIO-SAM
sensor pipeline has been ported or that the production process already uses it.
The ROS Velodyne/Ouster feature frontend is not substituted for MID-360 Fast-LIO.
Online/save wiring now calls this backend in the candidate source. ARM integration
tests, recorded-input parity and field acceptance remain distinct gates.
The previously inspected engcang FAST-LIO-SAM source is not vendored because its
license restricts commercial use; no source from that repository is used here.
