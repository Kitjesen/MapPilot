# ERASOR2 Backend Notice

Files in this folder are GPL-3.0-only because this backend is designed to run
or link the upstream ERASOR2 implementation.

The upstream ERASOR2 checkout is kept at:

```text
third_party/research_nav/ERASOR2
```

The ERASOR2 core files needed for local migration reading are also copied under:

```text
src/maps/prune/cpp/refs/erasor2/upstream
```

That snapshot is GPL-3.0-only and is not compiled into the default LingTu
`prune` binary.

Do not move files from this folder into the LingTu-owned `core/` cleaner unless
the code is rewritten independently and no upstream ERASOR2 source or headers
are copied.
