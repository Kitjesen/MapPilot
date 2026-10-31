# Elevator-LIO retained source and implementation provenance

Source: https://github.com/xiaofan4122/Elevator-LIO

Revision: `1d79af77f3d9747ea57ef52a9b01d326a8ec561a`

Author notice retained from the referenced elevator implementation:

> Copyright (c) 2026 xiaofan

The upstream `src/elevator/ElevatorProcess.cpp`,
`include/elevator/ElevatorProcess.h`, and `src/elevator/ElevatorSelfExit.cpp`
declare `SPDX-License-Identifier: MIT`. The upstream
`include/elevator/ElevatorSelfExit.h` declares `GPL-2.0-or-later`. The repository
`LICENSE` contains GPL version 2. These are file-specific declarations; do not
describe the entire upstream repository as MIT. The original repository license
is retained as `../licenses/Elevator-LIO-LICENSE` and copied without changes to
`src/localization/ltlio/vendor/elevator_lio/LICENSE-GPL-2.0`.

The production numerical implementation is maintained once in
`src/localization/ltlio/frontend.{hpp,cpp}` and `src/localization/ltlio/detail/`,
under namespace `lingtu::ltlio` and library `ltlio_core`.
`vendor/elevator_lio/` retains the license texts and the exact upstream-to-local
source correspondence. It does not contain a second runtime implementation.

The complete default mapping chain includes original `IMUProcess`, `ESEKF`,
`LidarPipeline`, `Types`, `IkdMap` and `IkdNearestQuery` numerical bodies, with
their GPL-2.0-or-later/repository GPL provenance and original author attribution.
`ikd_Tree.{h,cpp}` retain GPL-2.0-or-later and the Yixi Cai/hku-mars and KennyWGH
notices. `AdaptiveVoxelPController.hpp` is MIT. `ElevatorProcess` is MIT;
`ElevatorSelfExit` retains the header/implementation license split above.
Native adapter files retain GPL provenance and do not relabel upstream code MIT.

The adaptation replaces mutable globals with per-instance Context and removes
ROS-shaped carriers, publishers, visualization and disk logging. Typed IMU input
declares acceleration scale explicitly; raw point input retains nanosecond offsets,
line/tag fields and original frame end time. The original 21-state propagation,
history interpolation/reintegration, point-to-plane update and default transport
update order form the numerical baseline. The retired 24-state partial integration
and its extracted `ZeroVelocityUpdate` helper are not a second production path.

LingTu adaptations in the sole implementation include calibrated input/resource
settings; external pose correction through that same history; measured stationary
initialization and constraints; per-scan diagnostics; empty/sparse scan cleanup;
expired-history handling; and map-save exclusion of active cabin observations at
the backend boundary. Earlier verified tree lifetime, rebuild and concurrent-reader
fixes are retained in the single spatial tree. Tree snapshots protect the full copy
against background replacement, and destruction joins the worker before releasing
nodes. These are marked local changes, not claims that the source is unmodified.

The upstream ROS node and wall-clock scheduling were not executed. Wheel/image
fusion, every sensor-specific decoder and all optional experimental branches are
not implied by reuse of the default numerical chain. The separate offline original
baseline under `build/elevator-official-20261016/` and historical dual-core results
remain evidence for their named binaries. Single-core integration evidence belongs
under `build/ltlio-unified-20261017/`. Source availability, compilation and numerical
agreement do not by themselves establish field behavior or ground-truth accuracy.

MIT permission notice for the referenced MIT-marked implementations:

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
