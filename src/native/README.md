# Shared native services

`src/native/` contains C++ support shared by multiple functional domains when
it is neither a portable compute kernel nor owned by one domain package.

| Path | Owns |
| --- | --- |
| `recording/` | Native recording and replay support. |
| `snapshot_file.hpp` | Shared snapshot-file utility for native services. |

Domain algorithms stay under their owning `src/<domain>/` tree. Portable
Rust/C ABI compute stays under `src/kernels/`. Product selection and lifecycle
stay under `config/runtime_graph/products/` and `src/lingtu/`.
