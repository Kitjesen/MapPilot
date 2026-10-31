# Getting Started

**Status:** Current setup and first-run guide
**Audience:** New contributors and first-time simulation or robot installers
**Runs on:** Local development, `env=sim`, and `env=real`

Use this page once to prepare the software, select a robot and operating mode,
and reach the first startup. For daily mapping, navigation, and recovery, use
[Operations](operations.md). [Product scope](product.md) describes what the
system is intended to deliver and what still needs qualification.

## Prerequisites

- Git, Python 3.10 or newer, and [uv](https://docs.astral.sh/uv/).
- CMake and the platform's C++ toolchain when building native components.
- Node.js 24 and npm when building the Web application.
- MuJoCo and the simulation extras when running simulation.
- CycloneDDS and the selected robot/device SDKs on the hosts that need them.

The current native SLAM backend uses GTSAM and PCL. Preparing the current video
runtime also builds its WebRTC plugin with Rust/Cargo; the preparation scripts
select the pinned tools. Native prerequisites are maintained in
[Build](../scripts/build/README.md).

From the repository root, install the locked development environment and
inspect the lifecycle CLI:

```bash
uv sync --locked --extra dev
uv run --locked python -m lingtu.control --help
```

These commands do not connect to hardware or request motion.

## Windows native development

For Windows x64 development, use PowerShell 7,
Visual Studio 2022 C++ Build Tools, Windows SDK, and CMake 3.27 or newer.
A new checkout needs:

```powershell
git lfs install
git lfs pull
git submodule update --init --recursive
npm --prefix web ci
npm --prefix web run build
```

Then follow the [native build instructions](../scripts/build/README.md#windows-native-development-and-simulation).
Python and Web setup alone does not build `slamd`, `mapd`, or `navd`.
Robot releases use the declared Linux target platform and architecture.
Windows is a development and simulation target, not an implicit cross-compile host.

## Select the robot and mode

The shared delivery workflow uses two Products:

| Product | Role | Saved map required |
| --- | --- | --- |
| `map` | Build the live map and expose durable save | No |
| `nav` | Reload a selected map, localize, plan, and navigate | Yes |

Other declared Products are `teleop`, `teleop_avoid`, `explore`, `tracking`, and
`inspection`. A declared mode is not a field acceptance claim. Their
capabilities and process roles live in
[`config/system/`](../config/system/README.md); delivery boundaries live in
[Product scope](product.md).

Public environment values are exactly `real` and `sim`. A Product is
independent of the environment; ProductControl is the single entry that
resolves the selected mode, robot, and environment before starting anything.
Select the actual profile under `config/robots/<vendor>/<model>/robot.yaml`.
That profile supplies geometry, devices, network interfaces, and calibration.
The examples use `doso/thunder_v4` in simulation and `unitree/go2` on a robot;
they do not establish qualification for every profile or narrow support to Go2.
Internal assembly and the execution snapshot need no separate operator action.

## Preview a Product

Resolve both workflow Products before starting processes:

```bash
uv run --locked python -m lingtu.control switch map \
  --robot doso/thunder_v4 --env sim --dry-run --json
uv run --locked python -m lingtu.control switch nav \
  --robot doso/thunder_v4 --env sim --map MAP_NAME --dry-run --json
```

A dry run validates the selected declarations and request parameters without
starting processes. In simulation, map loading is deferred to the actual
switch. It does not prove that the saved map can be loaded, that a process can
start, that localization is valid, or that motion is safe.

## Run in simulation

Follow [Simulation](simulation.md) to install its extras and build the required
native roles before starting. Its first interactive mapping command is:

```bash
uv run --locked python -m lingtu.control switch map \
  --robot doso/thunder_v4 --env sim --backend mujoco --viewer
```

Use the simulation guide for save/reload, goal submission, stop, and the
headless workflow. Simulation evidence and physical robot evidence stay
separate.

## Build and output layout

Native prerequisites, commands, and the `build/` to `install/` layout are
maintained in [Build](../scripts/build/README.md). Linux release assembly into
`dist/`, target installation, the layout transition, and rollback are maintained
in [Deployment](../scripts/deploy/README.md).

## First robot installation

Build or obtain a release for the selected Linux target, then follow the
[deployment instructions](../scripts/deploy/README.md) to install its versioned
files, Python runtime, machine configuration, and service catalog. Production
services run from `/opt/lingtu/current`, not from a developer `build/` tree.
Installing or replacing a release can restart an existing Product; save any
needed mapping session and keep the robot stationary before installation.

Do not reproduce release activation as an independent `systemctl start`
sequence. The installer and ProductControl handle the declared lifecycle.

### Enable Web mode switching

Gateway serves the operator Web application; its default port is `5050`.
Open the configured target address from the operator computer. The optional
`lt-control` loopback service lets Web requests use the same ProductControl
entry as the CLI. On a deployment that includes it, set the actual robot in
`/etc/lingtu/control.env`:

```ini
LINGTU_ROBOT=unitree/go2
# Use only when this deployment includes the camera Product variant:
# LINGTU_CONTROL_VARIANT=camera
```

After the release and Python runtime are installed:

```bash
sudo bash /opt/lingtu/current/scripts/deploy/thunder/install_catalog_service.sh control
sudo systemctl enable --now lt-control.service
curl http://127.0.0.1:5051/status
```

This starts the loopback control transport, not a mapping or navigation Product.
The deployed machine configuration owns addresses, ports, and credentials.
For operational Web controls, continue with
[Web mapping and navigation switches](operations.md#web-mapping-and-navigation-switches).

## Continue with the first session

On the installed target, follow [the session preflight](operations.md#before-each-session)
for read-only release, device, and readiness checks, then the mapping workflow
in [Operations](operations.md#build-save-and-reload-a-map). Use the configured
absolute `LINGTU_PYTHON` path for that release. Resolve reported issues before
switching modes; installation success is not motion readiness.

For the evidence required before physical motion, use [Testing](testing.md).
For source ownership and repository layout, use [Architecture](architecture.md);
for code changes, use [Development](development.md).
