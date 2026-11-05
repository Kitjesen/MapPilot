# Foreground video acceptance Gateway

`python -m gateway.video_test` is a development/acceptance entry point. It
constructs the real `GatewayModule` status projection without calling its
`setup()` or `start()`. It does not load a Host graph, start a Product, acquire
the camera, or launch navigation/driver processes. Product deployment and
lifecycle still belong to ProductControl.

The endpoint serves the four versioned snapshots read by DOSO's video-only
connection page, plus the existing native WHEP status/offer/session-delete and
camera/video diagnostics routes. This is the client's **read-only subset** of
the Gateway contract, not the complete production Gateway API. Capability
flags and endpoint/link lists exclude unregistered controls and streams.
Readiness honestly reports `no_modules_loaded`, `ready=false` and
`motion_ready=false`; robot online state remains unknown. Existing camera
readiness in a full Product must not be inferred from this endpoint.

Run the camera publisher and native video pipeline separately with unique
SHM names and a bounded lifetime. Set these values to that actual test's
configuration, in an environment without `LINGTU_PRODUCT`,
`LINGTU_PRODUCT_SESSION_ID` or `LINGTU_RUN_PLAN`:

```bash
export LINGTU_ROBOT=unitree/go2
export LINGTU_ENV=real
export LINGTU_WEBRTC_URL=http://127.0.0.1:19090/whep/endpoint
export LINGTU_VIDEO_STATUS_FILE=/absolute/test/run/video-status.json
PYTHONPATH=src python -m gateway.video_test --host 127.0.0.1 --port 15051
```

The default listener is loopback. For an authorized LAN acceptance test,
select the target's actual LAN address with `--host`, check the chosen port is
unused, and bound the outer process lifetime. Native video should retain LAN
ICE candidates while its signalling listener can stay on loopback. HTTP
signalling success does not prove client H.264 decoding or frame progression.

Optional `--probe-dir /absolute/test/web` serves only `index.html`, `probe.mjs`,
`probe-core.mjs` and `whepReceiver.mjs` at `/video-probe/`; it does not expose
other files in that directory. The files are separately supplied test assets,
not included with this Python module. Without that flag no page is mounted.

The existing opt-in MediaAccess middleware is retained. Its shared Bearer
policy is not per-App identity, TLS provisioning, or immediate revocation of
established peers. Keep an unprotected test listener within the agreed local
test network and terminate the process after acceptance. No runtime credential
is embedded in this entry point.

Local checks:

```bash
pytest -q tests/gateway/test_video_test_gateway.py \
  tests/gateway/test_video_routes.py tests/gateway/test_media_access.py
```

These check unstarted state, absence of control routes, configured media
forwarding, retained access policy and the static-file allowlist. They do not
establish NX deployment, ICE reachability, actual decoding, or product readiness.
