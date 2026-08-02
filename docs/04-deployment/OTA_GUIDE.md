# Native Release and OTA Distribution

Status: current repository-supported release path.

LingTu has one deployable field artifact: the native release produced by
`scripts/deploy/package_native_release.sh`. There is no second release
implementation.

## Supported boundary

The repository owns:

- deterministic native release packaging;
- an outer SHA-256 file and release manifest;
- optional Ed25519 signing of the release manifest;
- inner per-file SHA-256 verification;
- versioned installation under `/opt/lingtu/releases/<version>`;
- atomic activation through `/opt/lingtu/current`;
- ProductControl quiesce, persistent-process restart, RunPlan reapply, and
  rollback.

The repository does not contain a robot-side polling agent or fleet rollout
controller. The release workflow may upload an
artifact to an externally operated OTA server, but distribution does not own
robot lifecycle. The packaged `install_nav.sh` is the only installer.

## Build and package

Build the required native artifacts first, then package one semantic version:

```bash
bash scripts/deploy/package_native_release.sh v2.1.1 dist
```

The output includes:

```text
dist/lingtu-2.1.1-aarch64-native-release.tar.gz
dist/lingtu-2.1.1-aarch64-native-release.sha256
dist/lingtu-2.1.1-aarch64-native-release-manifest.json
dist/lingtu-2.1.1-aarch64-native-release-system-manifest.json
```

Set `LINGTU_NATIVE_RELEASE_SIGNING_KEY` to an Ed25519 PEM private key when the
external distributor requires a signed manifest. Packaging fails if a supplied
key is not Ed25519. Run the complete isolated package/install/rollback contract
with:

```bash
bash scripts/deploy/package_native_release.sh --self-test
```

## Verify and install

Verify the downloaded tarball before extraction:

```bash
cd dist
sha256sum -c lingtu-2.1.1-aarch64-native-release.sha256
mkdir -p /tmp/lingtu-release
tar -xzf lingtu-2.1.1-aarch64-native-release.tar.gz \
  -C /tmp/lingtu-release
cd /tmp/lingtu-release/lingtu-2.1.1-aarch64-native-release
sudo ./install_nav.sh --dry-run
sudo ./install_nav.sh
```

The installer:

1. validates `metadata.json` and every file in
   `config/native-release-sha256.txt`;
2. refuses to replace an active release when its current fingerprinted RunPlan
   is unavailable;
3. stages a new immutable version directory;
4. asks ProductControl to quiesce the current Product;
5. atomically moves `/opt/lingtu/current` to the new release;
6. restarts persistent processes declared by the current RunPlan;
7. asks ProductControl to reapply that exact RunPlan and wait for readiness;
8. restores the previous link, persistent processes, and Product if activation
   fails.

No install step may select systemd units, infer a Product, or reproduce
readiness policy outside ProductControl.

## Cutting a release on the robot

For an explicitly reviewed development checkout already on the robot:

```bash
cd ~/data/SLAM/navigation
bash scripts/deploy/cut_release.sh v2.1.1
```

This command first loads ProductControl's canonical `current.json` and strictly
validates its referenced RunPlan. It refuses to continue when that state is
missing or when its fingerprint, Product, or Env disagrees with the plan. It then
builds and validates the declared native artifacts, snapshots them under
`/opt/lingtu/releases/v2.1.1`, installs the release-owned unit files, atomically
changes `/opt/lingtu/current`, and reapplies the same RunPlan through
ProductControl. Product/control identity and selected process targets are never
reconstructed from ambient variables or a shell-owned unit catalog. ROS2
compatibility builds and services are not part of this canonical release path.

Use `package_native_release.sh` plus the packaged installer for normal
distribution. `cut_release.sh` is the direct, on-robot engineering path.

## Rollback

Both supported activation paths attempt automatic rollback before returning a
failure. A symlink-only manual rollback is incomplete because a release can
also update systemd unit and runtime environment files.

If automatic rollback reports that operator intervention is required, restore
the previous release's activation files, run `systemctl daemon-reload`, restore
the `/opt/lingtu/current` link, and then reapply the committed Product:

```bash
bash /opt/lingtu/current/scripts/lingtu --env real svc reapply
```

Do not recover by restarting `lingtu.service` or a hand-written list of native
units. That can leave the Host and native processes on different RunPlans.

## Post-release evidence

After a successful ProductControl reapply:

```bash
bash /opt/lingtu/current/scripts/lingtu status
bash /opt/lingtu/current/scripts/lingtu doctor --non-motion --strict
```

Hardware motion acceptance remains a separate operator-controlled gate. A
package self-test or successful service readiness result is not S100P motion
validation.
