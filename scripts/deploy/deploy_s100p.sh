#!/usr/bin/env bash
# Legacy compatibility entrypoint. Use deploy_thunder.sh for new deployments.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "deploy_s100p.sh is a compatibility alias; forwarding to deploy_thunder.sh" >&2
exec "${SCRIPT_DIR}/deploy_thunder.sh" "$@"
