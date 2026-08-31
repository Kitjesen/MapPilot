# RobotSimUE Windows distribution

This module owns the local, fail-closed Cook/Stage/Package lane for the
`RobotSimUE` Win64 product. It does not own simulation sessions, replay,
RobotPackage content, or HTTP orchestration.

## Trusted interface

Use the repository wrapper from an operator-controlled PowerShell session:

```powershell
# Read-only toolchain and project validation.
.\scripts\sim\package_windows.ps1 -Preflight

# Validate and write a deterministic plan manifest; never starts UAT.
.\scripts\sim\package_windows.ps1 -DryRun

# Execute the pinned BuildCookRun command.
.\scripts\sim\package_windows.ps1
```

The wrapper exposes no Unreal executable, UAT path, command argument, project,
map, configuration, or output-directory parameter. The Python CLI accepts only
the three operation names. This module is not registered with SimStudio or any
other HTTP router. A future HTTP adapter may select an operation but must not
accept paths, executables, environment variables, or UAT arguments.

Installation roots may be suggested by the process environment or discovered
from Epic Launcher and standard Windows installation roots. A suggested root is
only a candidate. Executables and toolchain files are addressed by fixed relative
paths from `policy.v1.json`; the detected Unreal and MSVC versions must match the
policy before use.

## Pipeline and claims

The immutable policy fixes:

- Unreal Engine 5.8.1 and changelist 56057345;
- RobotSimUE, Win64, Shipping;
- the four checked-in RobotSimUE maps;
- Build, Cook, Stage, Pak, IoStore, Package, Archive, and prerequisites;
- MSVC 14.44.35207, Windows SDK 10.0.26100.0, and the Unreal AutoSDK
  .NET Framework header used by this workstation.

Dry-run output lives at:

```text
build/distribution/windows/plans/robotsimue-win64-shipping/distribution.manifest.json
```

The manifest is canonical JSON with no timestamp or absolute host path.
Repeating dry-run against unchanged inputs produces identical bytes.

Actual package candidates are first written below a private work directory.
They are published atomically to:

```text
build/distribution/windows/releases/robotsimue-win64-shipping/
```

only when all of these conditions hold:

1. version and path preflight passes;
2. UAT exits successfully;
3. `package/Windows/RobotSimUE.exe` and the Shipping runtime exist;
4. both executables have a Windows PE header.

Before that point, all Cook/Stage/Package/Shipping claims remain `false`.
`packaged_smoke_passed` remains `false` even after packaging; a separate launch
and runtime acceptance must promote that claim. Failed work directories are
retained for diagnosis and are never published as releases.

AutomationTool's writable `LOCALAPPDATA`, `APPDATA`, temporary, .NET CLI,
and NuGet roots are redirected below the deliberately short `build/ue/`
runtime root so UE's remaining legacy Win32 path checks stay below `MAX_PATH`.
This keeps Cook independent of the operator profile's ACLs and prevents UAT
from writing outside the repository-owned distribution area.
The UE-native `uebp_EngineSavedFolder`, `uebp_LogFolder`, and
`uebp_FinalLogFolder` variables are pinned to the same run-local area because
.NET Known Folder resolution does not honor `LOCALAPPDATA` overrides on every
Windows host.

## Deliberate limits

This lane packages the UE visual/sensor host. It does not yet assemble an
installer containing the external MuJoCo physics host, native DDS publishers,
SessionBundles, VC runtime policy, Pixel Streaming infrastructure, or signed
release metadata. Those are separate distribution gates and must not be
inferred from a successful RobotSimUE Cook.
