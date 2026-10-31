# GTSAM vcpkg overlay

This port is copied without modifications from Microsoft vcpkg commit
[`8948160a6f307241436b11c2a305c3760651222b`](https://github.com/microsoft/vcpkg/tree/8948160a6f307241436b11c2a305c3760651222b/ports/gtsam),
the official GTSAM 4.3.0 port update. Its MIT license is retained in
`LICENSE.txt`; the port installs the GTSAM source licenses with the package.

The five upstream files are `vcpkg.json`, `portfile.cmake`,
`build-fixes.patch`, `path-fixes.patch`, and `usage`.

LingTu keeps its existing vcpkg baseline for every other dependency. This
overlay replaces only GTSAM so the Windows preparation command can use the
same 4.3.0 release as the Linux build. The root manifest's GTSAM constraint
and this port version are checked against `scripts/build/locks/gtsam.json`.
The adjacent standard `vcpkg-configuration.json` selects this overlay for
both the preparation command and CMake manifest installs.
