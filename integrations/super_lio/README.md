# Super-LIO experimental integration

Super-LIO is preserved as an **experimental external integration**. It is not a Product, not a Profile, not an env value, and not installed by default. The normal field runtime remains the native DDS SLAM implementation selected by the `real` env.

This directory exists so the algorithm can still be evaluated in the lab without adding another operator-visible runtime identity or another default control plane. The upstream source is not vendored here; the integration was checked against the ROS 2 branch of `Liansheng-Wang/Super-LIO` at commit `a9220861e59194e3f10019192c6e5d61427a5b58`.

## Interface

The reference units under `systemd/` adapt the upstream ROS topics to LingTu's legacy bridge topics:

| Direction | Topic |
| --- | --- |
| input | `/lidar/raw_frame` |
| input | `/imu/raw` |
| output | `/slam/odometry` |
| output | `/slam/map_cloud` |
| optional output | `/nav/imu_odom` |
| optional output | `/nav/robot_odom` |

They require an external Super-LIO workspace, ROS 2 compatibility setup, and a ROS topic provider. They are reference units for lab evaluation only. The canonical S100P/Thunder installers do not copy, enable, start, or monitor them, and the native release package does not include them.

## Lab-only runbook

Build the external workspace on the robot:

```bash
mkdir -p ~/data/inovxio/super-lio/src
cd ~/data/inovxio/super-lio/src
git clone https://github.com/Liansheng-Wang/Super-LIO.git Super-LIO
cd ~/data/inovxio/super-lio
source /opt/lingtu/config/ros2-env.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Manually stage the reference units for a lab session only:

```bash
sudo cp ~/data/inovxio/lingtu/integrations/super_lio/systemd/super_lio*.service /etc/systemd/system/
sudo systemctl daemon-reload
```

For relocation, provide the LingTu map directory and keep the upstream parameter relative to `SUPER_LIO_ROOT`:

```bash
sudo install -d /run/lingtu
cat >/tmp/super_lio_relocation.env <<'EOF'
SUPER_LIO_RELOCATION_SOURCE_MAP_DIR=/home/sunrise/data/nova/maps/active
SUPER_LIO_RELOCATION_EFFECTIVE_MAP_DIR=map/lingtu_active
SUPER_LIO_RELOCATION_MAP_NAME=map.pcd
EOF
sudo mv /tmp/super_lio_relocation.env /run/lingtu/super_lio_relocation.env
```

Run only in a stationary lab window after stopping any ProductControl-managed Product. Do not run these units in parallel with `teleop`, `map`, `nav`, `explore`, `inspection`, or any other ProductControl session.

```bash
bash ~/data/inovxio/lingtu/scripts/lingtu --env real svc stop all
sudo systemctl start super_lio.service
systemctl status --no-pager super_lio.service
bash ~/data/inovxio/lingtu/scripts/lingtu doctor --non-motion --json --strict
```

Relocation smoke is also no-motion until a separate operator explicitly authorizes a ProductControl navigation session:

```bash
sudo systemctl stop super_lio.service
sudo systemctl start super_lio_relocation.service
systemctl status --no-pager super_lio_relocation.service
bash ~/data/inovxio/lingtu/scripts/lingtu doctor --non-motion --json --strict
```

Rollback removes only the manually staged lab units and runtime overrides:

```bash
sudo systemctl stop super_lio_relocation.service super_lio.service
sudo rm -f /etc/systemd/system/super_lio.service /etc/systemd/system/super_lio_relocation.service
sudo rm -f /run/lingtu/super_lio_relocation.env
sudo systemctl daemon-reload
bash ~/data/inovxio/lingtu/scripts/lingtu --env real svc reapply
```

## Why it is isolated

The old integration exposed `super_lio` and `super_lio_relocation` as if they were operating modes. They are SLAM implementations, not Products. That model duplicated navigation/map Products and spread backend-specific branches across Gateway, map save, diagnostics, ServiceManager, and the robot shell.

The active runtime aliases and branches remain removed. Do not restore `slamcheck`, Gateway backend switching, Product/Profile/ServiceManager branches, or default installer hooks for this integration. The algorithm assets are retained here until a typed adapter can make Super-LIO an internal `real` env SLAM implementation. That future adapter must be applied through the same ProductControl transaction as every other field process; these reference units must not be wired into the default installer in the meantime.

## Evaluation status

- Mapping/LIO: external upstream process; historical topic adapter retained.
- Saved-map relocation: experimental and not accepted on the S100P field gate.
- Native DDS integration: not implemented.
- ProductControl ownership: not implemented.

Do not use this integration as evidence that saved-map navigation or recovery is production-ready. Promote it only after S100P build, long-run localization, map-save, relocation, and failure-recovery acceptance all pass.

