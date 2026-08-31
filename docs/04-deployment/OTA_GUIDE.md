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
- ProductControl Product switch and rollback.

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
2. refuses to replace an active release when its current resolved RunPlan
   is unavailable;
3. stages a new immutable version directory;
4. atomically moves `/opt/lingtu/current` to the new release;
5. switches the same Product through ProductControl and waits for readiness;
6. restores the previous link and switches the previous Product if activation
   fails.

No install step may select systemd units, infer a Product, or reproduce
readiness policy outside ProductControl.

## Cutting a release on the robot

For an explicitly reviewed development checkout already on the robot:

```bash
cd ~/data/SLAM/navigation
bash scripts/deploy/cut_release.sh v2.1.1 nav
```

This convenience command runs `deploy_robot.sh nav` and then
`package_native_release.sh v2.1.1 dist`. Product resolution, building, activation,
packaging, and rollback remain owned by those existing entrypoints; the cutter
does not duplicate them.

## Rollback

Both supported activation paths attempt automatic rollback before returning a
failure. A symlink-only manual rollback is incomplete because a release can
also update systemd unit and runtime environment files.

If automatic rollback reports that operator intervention is required, restore
the previous release's activation files, run `systemctl daemon-reload`, restore
the `/opt/lingtu/current` link, and then switch the previous Product:

```bash
bash /opt/lingtu/current/scripts/lingtu --robot <vendor/model> --env real switch <product> [--map <name>]
```

Do not recover by restarting `lt-host.service` or a hand-written list of native
units. That can leave the Host and native processes on different RunPlans.

## Post-release evidence

After a successful ProductControl switch:

```bash
bash /opt/lingtu/current/scripts/lingtu status
PYTHONPATH=/opt/lingtu/current/src python -m diagnostics.field.doctor --non-motion --strict
```

Hardware motion acceptance remains a separate operator-controlled gate. A
package self-test or successful service readiness result is not S100P motion
validation.
