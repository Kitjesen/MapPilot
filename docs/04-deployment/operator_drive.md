# Bounded Operator Drive

`lingtu-drive` is the short operator command for a supervised motion check. It
keeps `lingtu_nav_control` as an internal native tool and sends every request
through the active LingTu `teleop` or `teleop_avoid` Product:

```text
lingtu-drive
  -> typed operator-motion request
  -> navd arbitration and final safety
  -> rt/nav/cmd_vel
  -> lingtu_driver
  -> selected robot adapter
```

Start the map-free Product first:

```bash
lingtu switch teleop --robot unitree/go2 --env real
```

Confirm that the intended Product is the committed active Product:

```bash
lingtu status --robot unitree/go2 --env real
```

Then run the generic stationary preflight. It reads live runtime evidence but
does not publish a motion command:

```bash
PYTHONPATH=/opt/lingtu/current/src python3 -m diagnostics.field.doctor \
  --non-motion --json --strict
```

Any nonzero result or reported blocker stops the physical test.

The normal first command is intentionally short:

```bash
lingtu-drive forward
```

It commands `0.20 m/s` for `2 s`, so the nominal open-loop distance is `0.40 m`.
The value is an estimate, not closed-loop distance control. Increase it explicitly
when the open area and stop path have been checked:

```bash
lingtu-drive forward --speed 0.30 --seconds 3
```

That requests a nominal `0.90 m`. One bounded command may run for at most `5 s`,
and speed may not exceed the active RunPlan's physical limit.

Supported directions are:

```text
forward  backward  left  right  turn-left  turn-right
```

The Product lifecycle lock covers the exact RunPlan read and native operator
claim. It is released as soon as the caller observes claim confirmation and
does not span the bounded motion or cleanup, so a lifecycle `stop` remains
available. The native client streams at 20 Hz, then sends hold and release
before returning success. A non-teleop Product, rejected typed ACK, missing
native client, excessive speed, admission timeout, or cleanup failure returns
nonzero.
