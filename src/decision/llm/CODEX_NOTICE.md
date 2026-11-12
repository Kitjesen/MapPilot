# Codex authorization provenance

`codex.py` adapts the device authorization request/poll/exchange flow and refresh
grant from NousResearch/hermes-agent v0.23.1, commit
`3b421578588243f920886e6fe79a6faa87ea026e`:

- `hermes_cli/web_routers/oauth.py`: `_codex_request_user_code`,
  `_codex_poll_authorization`, `_codex_exchange_tokens`.
- `hermes_cli/auth_codex.py`: `refresh_codex_oauth_pure`.
- `hermes_cli/auth_constants.py`: public client ID and endpoint constants.

Original copyright (c) 2025 Nous Research. The original MIT license is retained
in `LICENSE.hermes.txt` and must accompany distributed copies.

Local changes:

- Python 3.10-compatible implementation using the already installed httpx.
- LingTu-owned account storage, no borrowed Hermes/Codex CLI credentials.
- Polling is advanced by explicit Web requests, bounded by the provider interval;
  no gateway, background scheduler or persistent worker is started.
- A single Host lock serializes login, cancellation, refresh and disconnect.
- Only device display fields and connection status reach Web; model access
  tokens stay within the decision layer.
- Requests execute through LingTu's existing Responses tool loop. Robot tool
  admission, approvals, cancellation and completion evidence keep their owners.

This is the Hermes Codex device-code route, not the newer SIWC dynamic client
registration flow. It does not establish approval for commercial distribution,
account eligibility, model entitlement, or successful inference. The configured
model is a selectable candidate; upstream can reject it. A real authorized
inference remains the acceptance gate.
