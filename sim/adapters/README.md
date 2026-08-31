# Simulation adapters

Adapters translate at runtime seams but do not advance physics or own session
lifecycle.

`dds/` converts simulation state and commands to the LingTu typed DDS
contracts. Camera shared-memory and future presentation/input adapters should
be added as siblings only when they implement a distinct transport seam.
