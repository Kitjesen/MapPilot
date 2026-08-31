# Inspection

`src/nav/inspection` is the ROS-free C++ owner of inspection routes and route
execution state. It does not publish velocity commands and it does not contain a
second planner. The native navigation endpoint feeds each pending route point to
the existing global planner, local planner, path follower, and safety chain.

Routes are pinned to `map_id + content_epoch`, persisted below
`<inspection-dir>/routes/<map-id>`, and published with an atomic rename. Runtime
status and the restart checkpoint live directly below `<inspection-dir>`; they
are not map artifacts. Starting a route against a different active map content
epoch is rejected.

The runtime contract is:

```text
Gateway -> typed inspection command -> native navigation endpoint
                                         -> Inspection Executor
                                         -> existing single-goal navigation loop
native navigation endpoint -> inspection status -> Gateway/Web
```

Python may wrap the C ABI for HTTP integration. Route validation, persistence,
state transitions, retries, looping, and point advancement remain in C++.
