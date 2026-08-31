# SimStudio

SimStudio is the local UI and API for authoring and inspecting simulation
sessions. It is a developer tool, not a robot Product and not part of
`ProductControl`.

Run it from the repository root:

```powershell
python -m tools.simstudio
```

The application uses shared simulation code from `sim/`:

- `sim/catalog/` and `sim/importers/` resolve and import simulation assets.
- `sim/runtime/coordinator/` runs coordinated MuJoCo and RobotSimUE sessions.
- `sim/worlds/` and `sim/packages/` remain the shared simulation assets.

The field-style simulation lifecycle remains in `src/lingtu/sim/` and is
started through ProductControl. SimStudio must not own or replace that path.
