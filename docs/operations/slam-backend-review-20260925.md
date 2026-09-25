# SLAM backend review, 2026-09-25

Scope: source inspection and primary-source research. No deployment, map rewrite,
or motion commands. Field baseline is the previously verified .71 release and
903room_v3 save evidence, not a fresh check of the running robot.

Implementation follow-up: [native SAM candidate changes](sam-backend-hardening-20260925.md).
The list below records the original findings; the follow-up distinguishes the
implemented changes, ARM validation, and remaining field-scale acceptance.

## Legacy PGO status

OnlineMapping calls sam::Backend. SaveMap recognizes lio_sam_isam2 snapshots and
skips the legacy PGO stage. However, opt/CMakeLists.txt still requires the Rust
kernel (or a supplied prebuilt library), links it through map_opt, and installs
lt_pgo. SaveMap still invokes the older path for explicit offline constraints or
complete bundles without a SAM report. The legacy implementation is bypassed for
new SAM snapshots, not deleted or unreachable across the entire product.

## Ordered improvements

1. Separate current/recent source frames from historical loop targets.
   sam::Backend::submap includes key +/- 25, clipped only to array bounds. The
   candidate center must be older than 30 seconds, but its neighboring frames
   need not be. Sparse keyframes across a long pause can therefore put the source
   itself in its target. This is inherited from the upstream example, not proof
   of divergence from upstream. Exclude the current frame and a documented recent
   temporal neighborhood when constructing a historical target. Test a long
   pause with fewer than 25 intervening keyframes and a genuine revisit. Actual
   incidence among v3's 115 loop factors has not been established.

2. Persist loop evidence before tuning. LoopResult has a candidate index, fitness,
   and point counts, but OnlineMapping retains only acceptance counts. Record
   frame identities, target membership, relative measurement, noise, acceptance
   reason and correction size. Include the resolved SAM parameters with the saved
   capture. Replay v3 using actual raw poses, clouds and timestamps. Counts and
   a small resulting correction do not establish correct loop associations.

3. Evaluate acceptance and noise together. The current gate checks point counts,
   ICP convergence and fitness <= 0.3. ICP correspondence distance is 30 m under
   default radius 15 m. The same fitness scalar becomes all six Pose3 variances;
   it is not a calibrated angular/linear uncertainty. Odometry variances are also
   fixed upstream defaults. Measure overlap, residual distribution and geometry
   on correct/incorrect room revisits before choosing thresholds. Do not replace
   relative uncertainty with a single-frame Fast-LIO marginal covariance.

4. Add tested outlier resistance for accepted loops. Currently they are ordinary
   Gaussian BetweenFactors. Compare an existing GTSAM robust noise model with
   plain factors using genuine and deliberately false loops. Consider consistency
   methods such as Kimera-RPGO only if the bounded comparison justifies the extra
   dependency. Robust kernels do not replace geometric verification and do not
   guarantee rejection of a mutually consistent false association.

5. Remove legacy build coupling after identifying offline consumers. Split the
   native SAM online target from legacy offline optimization so building slamd
   does not require Cargo solely for unused online code. Preserve explicitly
   supported offline behavior until its replacement is validated. Test native
   online configuration without Cargo and the remaining offline commands.

6. Measure scale costs before expanding collection. Each new frame retrieves all
   estimates; loop search is linear, submaps are assembled/downsampled repeatedly,
   and changed poses can rebuild the preview. Full clouds exist in the online
   adapter and converted backend storage. Measure peak memory, loop/preview time
   and queue pressure on ARM near representative map sizes; share immutable data
   or cache submaps only where measurements identify a bottleneck. Existing bounded
   queues do not by themselves establish throughput at scale.

Keep full SE(3) geometry: real stairs and lifting prohibit blanket flattening.
Do not replace the optimizer again before addressing evidence and constraints.
These are proposed changes; this review does not claim they are implemented.

## Primary sources

- LIO-SAM paper: https://arxiv.org/abs/2007.00258
- Pinned mapping implementation:
  https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp
- Official README explicitly describes its loop closure as a proof of concept:
  https://github.com/TixiaoShan/LIO-SAM#other-notes
- iSAM2 paper: https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf
- Existing GTSAM robust noise models:
  https://gtsam.org/2019/09/20/robust-noise-model.html
- Switchable Constraints paper:
  https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf
- Robust consistency reference implementation:
  https://github.com/MIT-SPARK/Kimera-RPGO

LIO-SAM mapping PGO uses Pose3 prior/odometry/loop factors (and optional GPS),
with iSAM2 incremental updates. Its separate IMU-preintegration graph is not the
same graph. LingTu uses Fast-LIO in place of that original front-end arrangement.
