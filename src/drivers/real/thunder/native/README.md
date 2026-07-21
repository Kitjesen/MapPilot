# Thunder native driver

Status: current field driver contract as of 2026-07-18.

`lingtu_driver` is the product `driver` backend for Thunder. The systemd unit
and Brainstem lease identify it as the single hardware owner `lingtu-driver`.
It consumes the
canonical `/nav/cmd_vel` command from the derived DDS wire topic
`rt/nav/cmd_vel` and sends sequence-numbered
`RobotControl.WalkChecked(...)` calls to the Brainstem service running on the
separate robot-control computer.

The field systemd unit reads the endpoint from
`/opt/lingtu/config/brainstem.env`:

```text
LINGTU_BRAINSTEM_HOST=REMOTE_BRAINSTEM_IP
LINGTU_BRAINSTEM_PORT=13145
LINGTU_BRAINSTEM_TLS_CA_FILE=/opt/lingtu/config/tls/brainstem-ca.crt
LINGTU_BRAINSTEM_TLS_CERT_FILE=/opt/lingtu/config/tls/lingtu-driver.crt
LINGTU_BRAINSTEM_TLS_KEY_FILE=/opt/lingtu/config/tls/lingtu-driver.key
LINGTU_BRAINSTEM_TLS_SERVER_NAME=OPTIONAL_CERTIFICATE_SAN
```

The field installer accepts a literal remote IPv4 address only and rewrites a
root-owned canonical environment file on every install. Missing/unreadable TLS
files, partial TLS configuration, loopback, hostnames, and missing hosts are
rejected. The driver never starts a local Brainstem process and never falls
back from TLS to clear text.

The Brainstem computer must independently set
`HAN_DOG_LINGTU_ALLOWED_IPS=LINGTU_HOST_IP` plus the Brainstem server
certificate, key, and LingTu-client CA. Its default allowlist is empty. Remote
lease-aware motion requires both an allowlisted source IP and an authenticated
client certificate; actuator and posture RPCs remain loopback-only.

Safety behavior:

- accepts only `body` and compatibility `base_link` frames;
- rejects non-finite values and immediately zeroes active motion;
- sends one zero command after 200 ms without a fresh command;
- acquires and renews an explicit `lingtu-driver` control lease;
- requires an acknowledged `WalkChecked(0,0,0)` after acquiring a lease before
  publishing ready, and respects Brainstem's checked-command rate limit;
- requires standing/walking state, motor output enabled, no critical motor
  fault, and Brainstem ownership before becoming ready;
- uses sequence-numbered `WalkChecked` commands and requires a matching,
  accepted acknowledgement before treating a command as executed;
- drops commands received while disconnected and never replays them;
- sends a final zero command during shutdown;
- never enables motors or changes robot posture.

The driver also publishes native DDS driver-control readiness back to the
navigation endpoint. The endpoint requires that readiness to be fresh before
field motion. Loss of Brainstem connection, lease ownership, motor readiness,
or checked-command acknowledgement causes the endpoint to stop publishing
motion instead of replaying old commands.

The Brainstem SDK is proprietary and is not copied here. `brainstem.proto` is a
minimal independently maintained compatibility declaration for the required
field boundary. The remote Brainstem must implement `AcquireControl`,
`RenewControlLease`, `ReleaseControl`, `WalkChecked`, and the status fields in
the v1 contract. A legacy server that only implements `Walk(Vector3)` is
reported as `protocol_incompatible` and is deliberately rejected; the driver
does not fall back to unchecked motion.
