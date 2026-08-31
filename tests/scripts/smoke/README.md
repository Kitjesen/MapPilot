# Thunder Manual Smoke Scripts

Run these manually from the repository root. They are not pytest tests.

```bash
# Observe an already-running native SLAM process through its Host adapter.
python tests/scripts/smoke/mapping.py

# Verify typed native navigation command request/ACK handling.
bash tests/scripts/smoke/smoke_nav_command_ack.sh
```
