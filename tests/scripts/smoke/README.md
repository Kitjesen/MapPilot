# Thunder Manual Smoke Scripts

Run these manually from the repository root. They are compatibility smoke
checks, not pytest tests, and may require ROS 2 compatibility services, Gateway,
an active map, MCP, or robot/simulator access.

```bash
# Mapping chain; requires Fast-LIO2/LiDAR compatibility services.
python tests/scripts/smoke/mapping.py

# Planning against an active tomogram under NAV_MAP_DIR.
python tests/scripts/smoke/nav_planning.py

# Compatibility full-system startup check.
python tests/scripts/smoke/s100p_start.py

# MCP JSON-RPC smoke; requires Gateway and MCP.
python tests/scripts/smoke/mcp_full.py
```

The script names are kept stable for compatibility. New product-facing docs
should use Thunder naming and the CLI/product profile entry points.
