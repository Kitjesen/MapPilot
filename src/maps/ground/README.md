Patchwork++ pure C++ core, pinned to upstream v1.4.1 commit
3e6903a1d5537a4cc2ace897b0bbb98a92d6014c, is vendored in `vendor/` under
its BSD-2-Clause license. Upstream: https://github.com/url-kaist/patchwork-plusplus
Only modification to the vendored core: constructor output honors `verbose`.

The LingTu adapter takes one gravity-aligned scan relative to the calibrated
LiDAR origin. It estimates initial sensor height from multiple nearby sectors,
rejects insufficient data, disables reflectivity-based RNR for XYZ input, and
reduces the plane residual threshold to 4 cm. Ground masks alone do not make
historical points immune to deletion: repeated observations and stable local
support are checked by the cleaning caller. This is not a navigation surface query.
