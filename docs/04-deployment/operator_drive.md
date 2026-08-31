# Operator Teleop

The Web Teleop panel is the normal interactive keyboard controller. Start either
`teleop` for direct operator control or `teleop_avoid` for operator control with
live local avoidance, then open the Web scene and press **连接**.

- Hold `W/S` to move forward/backward, `A/D` to move laterally, and `Q/E` to turn.
- Hold `Shift` for 40% precision speed.
- Release the last movement key or press `Space` / **保持** to publish hold.
- Press **断开** when this browser should stop owning the Web control session.
  A new direction input after **保持** automatically reclaims native authority;
  there is no Web-facing Lease, heartbeat, CLAIM, epoch, or “恢复控制” operation.
- Page blur, page hide, disconnect, and closing the panel also publish hold.

The native one-second authority timeout remains a fail-safe inside `navd`: if
velocity samples stop, `navd` publishes zero. Gateway renews or recreates that
native authority only when the same connected browser sends a fresh movement
input, so an idle browser sends no control traffic. The Web client refreshes a
missing teleop bootstrap while a teleop Product is active and reconnects after
a Host/WebSocket restart; a connection owned by another browser requires an
explicit retry instead of an automatic retry loop.

The Web panel publishes active operator samples at 50 Hz. The native navigation
and safety loop consumes them at 20 Hz; the robot driver keeps its own 50 Hz
control loop. A Gateway ingress acknowledgement means the intent was queued,
not that the final velocity or motor command executed.

`lingtu-drive` is not a second teleop UI. It is a bounded command for one
supervised motion check and sends every request through the active `teleop` or
`teleop_avoid` Product:

```text
lingtu-drive
  -> typed operator-motion request
  -> navd arbitration and final safety
  -> rt/nav/cmd_vel
  -> lingtu-driver
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

The command reads the active Product limits, claims native operator control,
streams at 50 Hz, then sends hold and release before returning success. The
claim does not block a Product `stop`. A non-teleop Product, rejected typed ACK,
missing native client, excessive speed, admission timeout, or cleanup failure
returns nonzero.
