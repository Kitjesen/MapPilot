# Windows toolchain lock

This directory defines the Windows inputs for RobotSimUE and the C++ MuJoCo runtime: Unreal Engine, Visual Studio/MSVC, Windows SDK, MuJoCo SDK, and Blender.

## What “locked” means

`detected` means a candidate was found. `validated` means an exact version, usable installation path, and SHA-256 evidence file were found. `locked` is stronger: a reviewed lock file must match exact version, source location, architecture, SHA-256, and installation evidence. The detector only reads the machine and an optional lock file; it never installs, downloads, changes the registry, or deletes files.

The example deliberately contains no fabricated versions. Unknown or uninstalled tools remain `null` and `not_found`/`detected`.

```powershell
pwsh -NoProfile -File .\sim\toolchains\detect_windows_toolchain.ps1 | Out-File .\sim\toolchains\windows.detected.json -Encoding utf8
pwsh -NoProfile -File .\sim\toolchains\detect_windows_toolchain.ps1 -LockFile .\sim\toolchains\windows.lock.json
```

The detector checks Epic/UE and `LauncherInstalled.dat`, `vswhere` and x64 MSVC, Windows Kits, `MUJOCO_HOME`/`MUJOCO_DIR` plus repository SDK candidates, and Blender. It emits JSON on stdout and performs no setup action.

## Owned Unreal builds

The three repository UE authoring launchers delegate `Build.bat` to
`sim/toolchains/ue_build.py`. The same runner also has an explicit direct UBT
mode for a reviewed `dotnet.exe` plus `UnrealBuildTool.dll`; it is not a PATH
lookup or an automatic fallback. The runner releases a gated Python child only
after `ProcessTreeOwner` owns it, so both the `cmd.exe -> Build.bat -> dotnet ->
UnrealBuildTool` tree and the direct `dotnet -> UnrealBuildTool` tree are
covered from their first descendant. Normal completion, build failure,
timeout, Ctrl+C, and parent-process termination all close the ownership
boundary. The launchers default to a finite 90-minute build deadline and
expose `-UnrealBuildTimeoutMinutes` for explicit overrides.

Builds are additionally serialized per canonical `.uproject` by advisory lock
files under `build/locks/ue-build/`. The runner always supplies `-WaitMutex`
and `-NoHotReloadFromIDE`, rejects every case-insensitive `-NoMutex` spelling,
and prevents extra arguments from overriding its mandatory values. Do not
invoke `Build.bat`, `dotnet`, or UBT directly from repository automation.

Direct mode requires normalized absolute, existing, regular, no-link paths for
both tools, an explicit no-link audit-log destination, a UE root containing
`Engine/Build/Build.version`, and a no-link UBT user-settings directory. The
worker removes every inherited `LINGTU_UBT_*` value, then injects only the two
validated root/settings inputs. It launches the argv with `shell=False` and
shares the same project lock and total deadline as `Build.bat` mode:

```powershell
& .\.venv\Scripts\python.exe .\sim\toolchains\ue_build.py `
  --ubt-dll 'D:\absolute\reviewed\UnrealBuildTool.dll' `
  --dotnet 'D:\absolute\reviewed\dotnet.exe' `
  --log 'D:\absolute\build-evidence\RobotSimUEEditor.log' `
  --engine-root 'D:\absolute\Epic Games\UE_5.8' `
  --ubt-user-settings-dir 'D:\absolute\ubt-user-settings' `
  --uproject 'D:\absolute\RobotSimUE\RobotSimUE.uproject'
```

## Why this is needed

Parallel Codex/CC workers can implement contracts, loaders, UE modules, asset importers, tests, and documentation concurrently. They cannot make different compilers, SDK headers, Unreal binaries, or Blender exporters produce the same binary and Cook output. The lock is the reproducibility boundary for Windows builds, UE packaging, and asset conversion.

Parallel workers still do not remove serial wall-clock work: the first C++ compile must finish before its ABI can be consumed; Unreal must shader-compile and Cook before a packaged run can be tested; large asset imports can be limited by one machine's CPU, disk, GPU, or license. Acceleration comes from keeping those critical-path jobs running while other workers build contracts, tests, import validation, and adapters.

The detector is evidence collection, not permission to claim Windows support. A shipping claim additionally needs a clean build, Cook/package test, runtime smoke test, and recorded build manifest.
