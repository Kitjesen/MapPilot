# T3 Agent integration

LingTu uses T3 Code as an integration reference, not as a second robot runtime.
T3's useful boundary is the provider control plane: a provider reports its model
and authentication state, then receives a text turn through its own protocol.

## What is connected

- `grok-4.7` uses xAI's OpenAI-compatible endpoint and `XAI_API_KEY`. It runs in
  the existing LingTu conversation loop, history, approvals, and tool cards.
- `acp:grok` is a local ACP command slot. Set `LINGTU_ACP_COMMAND` and optional
  `LINGTU_ACP_ARGS` in the Host environment to use Grok Build or another ACP
  agent that speaks JSON-RPC over stdio. LingTu starts one process for one text
  turn, collects `session/update` text chunks, and terminates that process.
- `acp:kilo` uses the installed Kilo Code CLI's `kilo acp` entry point by
  default. Set `LINGTU_KILO_COMMAND` or `LINGTU_KILO_ARGS` only when the Kilo
  binary or its launch arguments differ.
- `acp:kiro` uses `kiro-cli acp` when Kiro CLI is installed, and `acp:opencode`
  uses `opencode acp`. Their authentication and model selection remain inside
  the installed agent; LingTu only owns the ACP conversation boundary.
- `acp:registry` is a generic slot for an ACP Registry agent or local wrapper;
  configure `LINGTU_ACP_REGISTRY_COMMAND` and optional
  `LINGTU_ACP_REGISTRY_ARGS`. It is listed as unavailable until that command is
  present, so the UI never presents an unlaunchable agent as ready.

The ACP slot forwards text and permission requests. The external agent's
filesystem, terminal and MCP tools remain its own responsibility; default mode
routes permission requests through the current LingTu approval, full access
accepts the agent's selected permission, and read-only cancels them. No external
ACP tool is injected into LingTu's motion tools.

The model settings panel reads `GET /api/v1/agent/providers` to show each ACP
command's availability, first version line, setup environment variables, and
copyable installation/login commands. The endpoint is read-only and never runs
an installer. `available` means the executable can be found; `authenticated`
is intentionally `null` because login belongs to the external provider. A
provider must still pass a real ACP turn before it is considered usable.

## T3 Code boundary

T3 Code is an Agent control plane and ACP client, not an ACP agent executable
that LingTu can start as a model provider. LingTu therefore shares its ACP
provider boundary and can run the same Kilo or Registry commands, while T3
continues to own its own provider installation, account flows, native sessions,
and delegation. There is no supported `t3 acp` command to add here; presenting
one would make the connection look configured while no agent could answer.

## Grok naming

T3 Code's built-in `Grok` provider means Grok Build CLI, not the animated Grok
Bot desktop application. The desktop application has no discovered ACP server,
CLI contract, or supported API for T3 to launch. It can only be connected after
an official agent endpoint or a separately reviewed ACP wrapper exists.

## Runtime examples

```text
XAI_API_KEY=...                         # direct Grok API
LINGTU_ACP_COMMAND=grok                # installed Grok Build executable
LINGTU_ACP_ARGS=agent stdio             # T3's Grok ACP transport
LINGTU_KILO_COMMAND=kilo               # optional binary override
LINGTU_KILO_ARGS=acp
LINGTU_KIRO_COMMAND=kiro-cli            # optional binary override
LINGTU_KIRO_ARGS=acp
LINGTU_OPENCODE_COMMAND=opencode        # optional binary override
LINGTU_OPENCODE_ARGS=acp
```

The command is launched without a shell. `ProductControl`, native navigation,
driver, and Brainstem ownership are unchanged; an external agent cannot issue a
motor or velocity command through this adapter.

## LingTu-owned workspace slices

LingTu exposes only the development pieces that belong beside the conversation
UI:

- `GET/POST/DELETE /api/v1/agent/environments` records the local workspace and
  explicitly configured remote endpoints. A remote entry is `configured`, not
  `ready`; Gateway does not open SSH, run a remote command, or create a second
  execution path for an Agent.
- `GET/POST/DELETE /api/v1/agent/workspaces` creates and removes Git worktrees
  below `.lingtu/agent-workspaces`; the main worktree is protected.
- `GET/POST /api/v1/agent/workspaces/{id}/checkpoints` creates a named Git
  checkpoint. Creating one is an explicit commit; restoring one refuses a dirty
  worktree by default and requires `?force=true` to discard tracked and
  untracked changes for an exact restore.

Provider setup is a guided installer: the UI reports command availability and
version, links to the provider's guide, and offers copyable install/login
commands. It never downloads or executes an unreviewed provider from the robot
service.
