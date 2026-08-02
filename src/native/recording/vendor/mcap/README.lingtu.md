# Vendored MCAP C++

This directory contains the MCAP C++ headers from upstream tag
`releases/cpp/v2.1.3` (commit `1420296ffcfdcde4b6894c0c1aba0ad083f93dde`).

Upstream: <https://github.com/foxglove/mcap>

Only the C++ headers and upstream MIT license are vendored. LingTu compiles
with `MCAP_COMPRESSION_NO_LZ4` and `MCAP_COMPRESSION_NO_ZSTD`, so this batch
does not add compression-library dependencies.
