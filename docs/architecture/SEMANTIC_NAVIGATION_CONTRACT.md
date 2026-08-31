# Semantic Navigation Contract

Status: current

Semantic navigation turns a named place into one native navigation goal. It
does not own planning, map activation, floor transitions, or motion output.

## Flow

```text
instruction
  -> semantic intent
  -> place lookup in the active map
  -> typed goto command
  -> native navigation
```

The place catalog is read from mapd. A resolved place must provide a finite
map-frame pose and must belong to the active map. Ambiguous, stale, incomplete,
or cross-map results are rejected instead of guessed.

The Host publishes only the goal request. Native `navd` owns admission,
planning, tracking, avoidance, safety, and final motion output.

## Current limits

- Named-place navigation works only inside the active map.
- Cross-map and multi-floor navigation are not supported.
- Facility lift or stair integration is not part of a current Product.
- Voice clients may use the normal instruction or navigation APIs; LingTu does
  not maintain a separate voice compatibility endpoint.

Place data ownership and persistence are defined in
[`semantic_layer_contract.md`](./semantic_layer_contract.md).
