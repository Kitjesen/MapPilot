"""Codex device authorization adapted from Hermes v0.23.1; see CODEX_NOTICE.md.

One Host owns the account and pending login. No Hermes gateway, CLI credential
import, model daemon, or background scheduler is started.
"""

from __future__ import annotations

import base64
import json
import math
import os
import secrets
import tempfile
import threading
import time
from pathlib import Path

import httpx

ISSUER = "https://auth.openai.com"
CLIENT_ID = "app_EMoamEEZ73f0CkXaXp7hrann"
BASE_URL = "https://chatgpt.com/backend-api/codex"
MODELS = {"codex:gpt-6.1-sol": "GPT-6.1 Sol · Codex"}


class CodexAccount:
    def __init__(self, path: Path | None = None, *, transport=None):
        state = Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state")
        self.path = path or state / "lingtu" / "model_accounts" / "codex.json"
        self._transport = transport
        self._lock = threading.RLock()
        self._login = None

    def _post(self, path: str, **kwargs) -> httpx.Response:
        try:
            with httpx.Client(timeout=15, transport=self._transport) as client:
                return client.post(ISSUER + path, **kwargs)
        except httpx.HTTPError:
            raise ValueError("Codex 授权服务暂不可达，请重试。") from None

    @staticmethod
    def _require_ok(response):
        if response.status_code != 200:
            raise ValueError(f"Codex 授权请求未成功（HTTP {response.status_code}）。请检查账号权限后重试。")
        try:
            result = response.json()
        except ValueError:
            raise ValueError("Codex 授权返回了无效数据。") from None
        if not isinstance(result, dict):
            raise ValueError("Codex 授权返回了无效数据。")
        return result

    def _read(self):
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}
        except (OSError, ValueError):
            raise ValueError("无法读取 LingTu 的 Codex 连接记录，请检查文件权限或重新登录。") from None

    def _save(self, tokens, previous=None):
        access = tokens.get("access_token")
        refresh = tokens.get("refresh_token") or (previous or {}).get("refresh_token")
        if not isinstance(access, str) or not access or not isinstance(refresh, str) or not refresh:
            raise ValueError("Codex 未返回完整授权，连接没有保存。")
        expires = tokens.get("expires_in")
        if expires is not None:
            try:
                expires_at = time.time() + float(expires)
            except (ValueError, TypeError):
                raise ValueError("Codex 返回的有效期无效，连接没有保存。") from None
        else:
            try:
                # Expiry is only a refresh hint; never use unsigned claims as identity or permissions.
                payload = access.split(".")[1]
                expires_at = float(json.loads(base64.urlsafe_b64decode(payload + "=" * (-len(payload) % 4)))["exp"])
            except (ValueError, KeyError, IndexError, TypeError):
                raise ValueError("Codex 授权缺少有效期，连接没有保存。") from None
        if not math.isfinite(expires_at) or expires_at <= time.time():
            raise ValueError("Codex 返回的授权已过期，连接没有保存。")
        record = {"access_token": access, "refresh_token": refresh, "expires_at": expires_at}
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd, name = tempfile.mkstemp(dir=self.path.parent, prefix=".codex-")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as file:
                json.dump(record, file)
            os.chmod(name, 0o600)
            os.replace(name, self.path)
        finally:
            Path(name).unlink(missing_ok=True)
        return record

    def status(self):
        with self._lock:
            record = self._read()
            return {"provider": "codex", "connected": bool(record.get("refresh_token")),
                    "expires_at": record.get("expires_at"), "inference_verified": False}

    def start(self):
        with self._lock:
            if self._login and self._login["state"] == "pending" and self._login["expires_at"] > time.time():
                return self._public_login()
            result = self._require_ok(self._post("/api/accounts/deviceauth/usercode", json={"client_id": CLIENT_ID}))
            if not all(isinstance(result.get(key), str) and result[key] for key in ("user_code", "device_auth_id")):
                raise ValueError("Codex 没有返回有效设备码。")
            try:
                interval = max(3, int(result.get("interval", 5)))
            except (ValueError, TypeError):
                raise ValueError("Codex 返回的轮询间隔无效。") from None
            self._login = {"id": secrets.token_urlsafe(24), "state": "pending",
                           "user_code": result["user_code"], "device_auth_id": result["device_auth_id"],
                           "expires_at": time.time() + 900, "interval": interval, "next_poll": time.monotonic() + interval}
            return self._public_login()

    def _public_login(self):
        return {key: self._login[key] for key in ("id", "state", "user_code", "expires_at", "interval")} | {
            "verification_url": ISSUER + "/codex/device"}

    def poll(self, login_id):
        with self._lock:
            if not self._login or self._login["id"] != login_id:
                raise ValueError("login_not_found")
            login = self._login
            if login["state"] != "pending":
                return self._public_login()
            if time.time() >= login["expires_at"]:
                login["state"] = "expired"
                return self._public_login()
            if time.monotonic() < login["next_poll"]:
                return self._public_login()
            login["next_poll"] = time.monotonic() + login["interval"]
            response = self._post("/api/accounts/deviceauth/token", json={
                "device_auth_id": login["device_auth_id"], "user_code": login["user_code"]})
            if response.status_code in {403, 404}:
                return self._public_login()
            if response.status_code == 429:
                login["interval"] += 5
                login["next_poll"] = time.monotonic() + login["interval"]
                return self._public_login()
            try:
                result = self._require_ok(response)
                if not all(isinstance(result.get(key), str) and result[key] for key in ("authorization_code", "code_verifier")):
                    raise ValueError("Codex 未返回完整授权码。")
                tokens = self._require_ok(self._post("/oauth/token", data={
                    "grant_type": "authorization_code", "code": result["authorization_code"],
                    "code_verifier": result["code_verifier"], "client_id": CLIENT_ID,
                    "redirect_uri": ISSUER + "/deviceauth/callback"}))
                self._save(tokens)
                login["state"] = "approved"
            except (ValueError, OSError):
                # Do not replay an authorization code after an uncertain exchange/save.
                login["state"] = "failed"
                raise
            return self._public_login()

    def cancel(self, login_id):
        with self._lock:
            if not self._login or self._login["id"] != login_id:
                raise ValueError("login_not_found")
            if self._login["state"] == "pending":
                self._login["state"] = "cancelled"
            return self._public_login()

    def disconnect(self):
        with self._lock:
            self._login = None
            self.path.unlink(missing_ok=True)
            return self.status()

    def access_token(self):
        with self._lock:
            record = self._read()
            if not record.get("refresh_token"):
                raise ValueError("请先在模型连接中登录 ChatGPT / Codex。")
            if record.get("expires_at", 0) > time.time() + 60:
                return record["access_token"]
            response = self._post("/oauth/token", data={"grant_type": "refresh_token",
                "refresh_token": record["refresh_token"], "client_id": CLIENT_ID})
            if response.status_code in {400, 401, 403}:
                self.path.unlink(missing_ok=True)
                raise ValueError("Codex 授权已失效，请重新登录。")
            return self._save(self._require_ok(response), record)["access_token"]


account = CodexAccount()
