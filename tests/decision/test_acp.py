import asyncio
import json
import sys

from decision.llm.acp import AcpClient, acp_provider_statuses
from decision.llm.client import LLMConfig


def test_acp_stdio_turn_returns_agent_message():
    script = (
        "import sys,json\n"
        "for line in sys.stdin:\n"
        " m=json.loads(line); i=m.get('id'); method=m.get('method')\n"
        " if method=='initialize': print(json.dumps({'jsonrpc':'2.0','id':i,'result':{'protocolVersion':1}}),flush=True)\n"
        " elif method=='session/new': print(json.dumps({'jsonrpc':'2.0','id':i,'result':{'sessionId':'s'}}),flush=True)\n"
        " elif method=='session/prompt':\n"
        "  print(json.dumps({'jsonrpc':'2.0','method':'session/update','params':{'sessionId':'s','update':{'sessionUpdate':'agent_message_chunk','content':{'type':'text','text':'ok'}}}}),flush=True)\n"
        "  print(json.dumps({'jsonrpc':'2.0','id':i,'result':{'stopReason':'end_turn'}}),flush=True)\n"
    )
    config = LLMConfig(backend="acp", model="grok", command=sys.executable,
                       command_args=("-c", script), timeout_sec=5, max_retries=0)
    result = asyncio.run(AcpClient(config).chat([{"role": "user", "content": "hello"}]))
    assert result == "ok"


def test_acp_full_access_answers_permission_request():
    script = (
        "import sys,json\n"
        "for line in sys.stdin:\n"
        " m=json.loads(line); i=m.get('id'); method=m.get('method')\n"
        " if method=='initialize': print(json.dumps({'jsonrpc':'2.0','id':i,'result':{'protocolVersion':1}}),flush=True)\n"
        " elif method=='session/new': print(json.dumps({'jsonrpc':'2.0','id':i,'result':{'sessionId':'s'}}),flush=True)\n"
        " elif method=='session/prompt':\n"
        "  print(json.dumps({'jsonrpc':'2.0','id':9,'method':'session/request_permission','params':{'options':[{'optionId':'allow_once'}]}}),flush=True)\n"
        "  for reply in sys.stdin:\n"
        "   if json.loads(reply).get('id')==9:\n"
        "    print(json.dumps({'jsonrpc':'2.0','method':'session/update','params':{'sessionId':'s','update':{'sessionUpdate':'agent_message_chunk','content':{'type':'text','text':'edited'}}}}),flush=True)\n"
        "    print(json.dumps({'jsonrpc':'2.0','id':i,'result':{'stopReason':'end_turn'}}),flush=True); break\n"
    )
    config = LLMConfig(backend="acp", model="kilo", command=sys.executable,
                       command_args=("-c", script), timeout_sec=5, max_retries=0)
    result = asyncio.run(AcpClient(config).chat([{"role": "user", "content": "edit"}], permission_mode="full_access"))
    assert result == "edited"


def test_acp_provider_status_reports_command_and_instructions(monkeypatch):
    monkeypatch.setenv("LINGTU_KILO_COMMAND", sys.executable)
    statuses = {item["id"]: item for item in acp_provider_statuses(LLMConfig())}
    kilo = statuses["acp:kilo"]
    assert kilo["available"] is True
    assert kilo["version"]
    assert kilo["install_command"]
    assert kilo["login_command"]
    assert kilo["authenticated"] is None
