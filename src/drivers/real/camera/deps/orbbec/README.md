# Orbbec deps

The native camera uses the pure Orbbec SDK v2 package fetched into the ignored
build tree:

```text
build/deps/orbbec-sdk/
```

Run `scripts/build/fetch_orbbec_sdk.sh` to fetch the field-tested v2.8.7
package. Override the location with `LINGTU_ORBBEC_SDK_ROOT` when needed.
ROS2 wrapper sources are not part of the native camera dependency path.
