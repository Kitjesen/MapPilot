# Semantic configuration

This directory contains the two shared, model-independent semantic inputs:

- `taxonomy.json` assigns stable class IDs, names, aliases, and display colors.
- `scoring.yaml` tunes semantic contributions used by decision modules.

Detector model files and per-robot detector vocabulary do not belong here.
Changing taxonomy IDs is a data-contract change; changing scoring weights is a
runtime tuning change.
