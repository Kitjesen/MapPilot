from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

VIEWER = (
    Path(__file__).resolve().parents[2]
    / "src"
    / "gateway"
    / "templates"
    / "map_viewer.html"
)


def _function_body(html: str, name: str, next_name: str) -> str:
    return html.split(f"function {name}(", 1)[1].split(f"function {next_name}(", 1)[0]


def _function_source(html: str, name: str) -> str:
    start = html.index(f"function {name}(")
    opening = html.index("{", start)
    depth = 0
    for index in range(opening, len(html)):
        if html[index] == "{":
            depth += 1
        elif html[index] == "}":
            depth -= 1
            if depth == 0:
                return html[start : index + 1]
    raise AssertionError(f"unterminated JavaScript function: {name}")


def test_explore_click_reports_http_rejection_without_claiming_state() -> None:
    html = VIEWER.read_text(encoding="utf-8")
    body = _function_body(html, "toggleExplore", "_setExploring")

    assert "r.ok" in body
    assert "detail.operator_command" in html
    assert "detail.reason" in html
    assert "等待状态确认" in body
    assert "_setExploring(" not in body


def test_explore_display_state_is_refreshed_only_from_status_or_sse() -> None:
    html = VIEWER.read_text(encoding="utf-8")

    assert 'id="expBtn" onclick="toggleExplore()" disabled>…&nbsp;探索状态</button>' in html
    assert "let _exploring = null;" in html
    assert "fetch('/api/v1/explore/status'" in html
    assert "_setExploring(data.exploring" in html
    assert "else if(ev.type==='exploring')   _setExploring(ev.active);" in html


@pytest.mark.skipif(shutil.which("node") is None, reason="node is required for viewer behavior test")
def test_explore_http_failure_keeps_status_state_and_surfaces_operator_guidance() -> None:
    html = VIEWER.read_text(encoding="utf-8")
    functions = "\n".join(
        _function_source(html, name)
        for name in (
            "_exploreFailureMessage",
            "_refreshExploreStatus",
            "toggleExplore",
            "_setExploring",
        )
    )
    script = f"""
const assert=require('assert');
let _exploring=true;
let _exploreRequestPending=false;
const $expBtn={{disabled:false,textContent:'',className:''}};
const $mission={{textContent:''}};
let toast='';
function showToast(message){{toast=message;}}
function response(ok,status,payload){{
  return {{ok,status,text:()=>Promise.resolve(JSON.stringify(payload))}};
}}
{functions}
(async()=>{{
  const calls=[];
  global.fetch=async url=>{{
    calls.push(url);
    if(calls.length===1) return response(false,409,{{
      message:'ProductControl required',
      detail:{{
        reason:'parking_evidence_required',
        operator_command:'python -m lingtu.control stop --expected-product explore'
      }}
    }});
    return response(true,200,{{exploring:true}});
  }};
  toggleExplore();
  await new Promise(resolve=>setTimeout(resolve,20));
  assert.deepStrictEqual(calls,['/api/v1/explore/stop','/api/v1/explore/status']);
  assert.match(toast,/parking_evidence_required/);
  assert.match(toast,/python -m lingtu\\.control stop --expected-product explore/);
  assert.doesNotMatch(toast,/已发送/);
  assert.strictEqual(_exploring,true);
  assert.strictEqual($expBtn.textContent,'■ 停止探索');
}})().catch(error=>{{console.error(error);process.exit(1);}});
"""

    result = subprocess.run(
        ["node", "-e", script],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )

    assert result.returncode == 0, json.dumps(
        {"stdout": result.stdout, "stderr": result.stderr}, ensure_ascii=False
    )
