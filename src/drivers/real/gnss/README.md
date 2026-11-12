# GNSS

`gnss` is the short runtime role for positioning receivers.

Product path:

- `native/module.hpp` and `native/module.cpp` expose the C++ service boundary.
- `native/dds_module.hpp` and `native/dds_module.cpp` publish GNSS DDS topics.
- `native/gnss_dds.cpp` is only the process entrypoint.
- `impl/wtrtk980/` contains the WTRTK-980 serial/NMEA backend.

The native path publishes DDS directly:

- `/gnss/fix` -> `rt/gnss/fix`
- `/gnss/status` -> `rt/gnss/status`
- `/gnss/odom` -> `rt/gnss/odom` when a map origin is configured

ROS2 GNSS reader code is legacy compatibility only and is not present in this
real driver tree. Do not add ROS2 imports to the product GNSS role
implementation.

The same serial owner writes `/dev/shm/lingtu/gnss_status.json` (override with
`LINGTU_GNSS_STATUS_FILE` or `--status-file`). Each snapshot is written completely
to a temporary file in the same directory, then atomically renamed into place.
Its optional position is represented by `"location":null` until a valid fix, or:

```json
{"lat":48.1173,"lon":11.516666666666667,"fix_type":4,"ts":1794110400.125,"accuracy_m":null}
```

`lat`/`lon` are WGS84 degrees; `fix_type` uses the existing NMEA codes 1..8.
`ts` is the Unix time at reception of that fix, independent of diagnostic
`last_ts`. No fix, serial timeout, or process error clears location; losing a fix
is written immediately even between periodic status writes. Startup clears any
previous process's position. Consumers must also reject stale snapshots if the
process stops unexpectedly. `accuracy_m` is null because the current NMEA backend
provides no measured horizontal uncertainty in metres; HDOP and the existing DDS
covariance estimates are not used to invent one. DDS remains the positioning
source; no additional serial reader is introduced.

Run the focused local behavior test without CycloneDDS or hardware:
The concurrent replacement check runs on POSIX, matching the field runtime;
Windows runs the remaining snapshot and write-failure checks.

```sh
g++ -std=c++17 -Wall -Wextra -Werror -pthread -Isrc -Isrc/drivers/real/gnss \
  -Isrc/drivers/real/gnss/impl tests/drivers/real/gnss/test_snapshot.cpp \
  src/drivers/real/gnss/native/module.cpp src/drivers/real/gnss/impl/wtrtk980/nmea.cpp \
  -o /tmp/lingtu_gnss_snapshot_test
/tmp/lingtu_gnss_snapshot_test
```
