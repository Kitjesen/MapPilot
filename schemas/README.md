# LingTu schemas

This directory contains language-neutral contracts for stable serialized data
at repository and process boundaries. A schema belongs here when independently
implemented producers or consumers need to agree on a persisted or transported
document; a code-local evidence/error envelope does not move here merely because
it carries a `schema` discriminator.

| Boundary | Source of truth | Purpose |
| --- | --- | --- |
| Simulation packages, resolved plans, runtime JSON | [`simulation/`](./simulation/README.md) | Catalog authoring, generated bundles, allocation, runtime evidence, and local presentation transport |
| Native DDS messages | [`../src/message/`](../src/message/README.md) | Typed cross-process data plane and CDR wire types |
| Python Host messages | [`../src/runtime/msgs/`](../src/runtime/msgs/README.md) | Typed in-process Module communication |
| Gateway HTTP payloads | [`../src/gateway/schemas.py`](../src/gateway/schemas.py) | External API request and response models |
| Topic ownership and QoS | [`../config/runtime_graph/topics.yaml`](../config/runtime_graph/topics.yaml) | Producer, consumer, topic, and delivery policy |

These boundaries intentionally do not share one universal envelope. A JSON
simulation plan, a DDS sample, and an HTTP request have different owners,
lifecycle rules, and performance constraints. Reuse the same domain term where
the meaning matches, but do not duplicate a DDS or HTTP contract here merely to
claim that the repository has one global message format.
