import json
from urllib.parse import parse_qs

import httpx
import pytest

from decision.llm.codex import CodexAccount


@pytest.fixture
def auth(tmp_path):
    requests = []
    replies = []

    def handle(request):
        requests.append(request)
        status, payload = replies.pop(0)
        return httpx.Response(status, json=payload)

    return CodexAccount(tmp_path / "codex.json", transport=httpx.MockTransport(handle)), requests, replies


def start(auth):
    account, _, replies = auth
    replies.append((200, {"user_code": "TEST-CODE", "device_auth_id": "private-id", "interval": "3"}))
    login = account.start()
    account._login["next_poll"] = 0
    return login


def test_login_pending_then_authorized_persists_without_disclosing_tokens(auth):
    account, requests, replies = auth
    login = start(auth)
    assert "device_auth_id" not in login
    replies.append((403, {}))
    assert account.poll(login["id"])["state"] == "pending"
    account._login["next_poll"] = 0
    replies.extend([(200, {"authorization_code": "private-code", "code_verifier": "private-verifier"}),
                    (200, {"access_token": "private-access", "refresh_token": "private-refresh", "expires_in": 3600})])
    assert account.poll(login["id"])["state"] == "approved"
    assert account.access_token() == "private-access"
    assert account.status()["connected"] is True
    assert account.status()["inference_verified"] is False
    assert "private" not in json.dumps(account.status())
    exchanged = parse_qs(requests[-1].content.decode())
    assert exchanged["redirect_uri"] == ["https://auth.openai.com/deviceauth/callback"]
    assert exchanged["code_verifier"] == ["private-verifier"]
    restored = CodexAccount(account.path)
    assert restored.access_token() == "private-access"
    account.disconnect()
    assert not account.path.exists()
    assert not account.status()["connected"]


def test_cancel_and_expiry_do_not_exchange_or_save(auth):
    account, requests, _ = auth
    login = start(auth)
    assert account.cancel(login["id"])["state"] == "cancelled"
    assert account.poll(login["id"])["state"] == "cancelled"
    assert len(requests) == 1 and not account.path.exists()
    login = start(auth)
    account._login["expires_at"] = 0
    assert account.poll(login["id"])["state"] == "expired"
    assert len(requests) == 2 and not account.path.exists()
    with pytest.raises(ValueError, match="login_not_found"):
        account.poll("another-login")


def test_polling_respects_interval_and_backoff(auth):
    account, requests, replies = auth
    login = start(auth)
    replies.append((429, {}))
    assert account.poll(login["id"])["interval"] == 8
    assert account.poll(login["id"])["state"] == "pending"
    assert len(requests) == 2


def test_refresh_rotates_and_invalid_grant_requires_relogin(auth):
    account, requests, replies = auth
    account._save({"access_token": "old-access", "refresh_token": "old-refresh", "expires_in": 1})
    replies.append((200, {"access_token": "new-access", "refresh_token": "new-refresh", "expires_in": 3600}))
    assert account.access_token() == "new-access"
    assert parse_qs(requests[-1].content.decode())["refresh_token"] == ["old-refresh"]
    account._save({"access_token": "new-access", "refresh_token": "new-refresh", "expires_in": 1})
    replies.append((401, {"error": "invalid_grant"}))
    with pytest.raises(ValueError, match="重新登录"):
        account.access_token()
    assert not account.status()["connected"]


def test_transient_refresh_preserves_account_and_does_not_leak_error_body(auth):
    account, _, replies = auth
    account._save({"access_token": "old-access", "refresh_token": "old-refresh", "expires_in": 1})
    replies.append((503, {"error": "private-refresh"}))
    with pytest.raises(ValueError, match="HTTP 503") as error:
        account.access_token()
    assert "private-refresh" not in str(error.value)
    assert account.status()["connected"]


def test_failed_exchange_is_terminal_and_cannot_replay_code(auth):
    account, requests, replies = auth
    login = start(auth)
    replies.extend([(200, {"authorization_code": "code", "code_verifier": "verifier"}), (503, {})])
    with pytest.raises(ValueError, match="HTTP 503"):
        account.poll(login["id"])
    assert account.poll(login["id"])["state"] == "failed"
    assert len(requests) == 3 and not account.path.exists()


@pytest.mark.parametrize("tokens", [{}, {"access_token": "a", "refresh_token": "r"},
    {"access_token": "a", "refresh_token": "r", "expires_in": float("nan")}])
def test_malformed_tokens_never_replace_saved_connection(auth, tokens):
    account, _, _ = auth
    account._save({"access_token": "original", "refresh_token": "refresh", "expires_in": 3600})
    with pytest.raises(ValueError):
        account._save(tokens)
    assert account.access_token() == "original"
