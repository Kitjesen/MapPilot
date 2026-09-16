# Acceptance configuration

This directory contains local acceptance orchestration configuration. It is not
part of the Product runtime graph and does not define field evidence.

MuJoCo manifests live in [`mujoco/`](mujoco/). Their filenames omit
`mujoco`, `native`, and `acceptance` because those meanings are already expressed
by this directory. Each manifest continues to be run by its existing consumer;
the manifest does not introduce a second command or lifecycle entry point.
