#!/usr/bin/env python3
"""
upload_mid360_test.py — 下载 MID-360 扫描模式并上传到机器人，运行导航测试

步骤:
  1. 从 OmniPerception GitHub 下载 mid360.npy
  2. 上传 nova_nav_bridge.py (已集成 LivoxMid360) + mid360.npy 到 S100P
  3. 运行 smoke test 验证 mid360.npy 可加载
  4. 运行完整导航测试 (test_factory_nova.sh)
"""
import sys
import io
import time
import urllib.request
from pathlib import Path

import paramiko

HOST = "192.168.66.190"
USER = "sunrise"
PASS = "sunrise"

REPO = Path(__file__).resolve().parent.parent.parent

MID360_URL = (
    "https://raw.githubusercontent.com/aCodeDog/OmniPerception/main/"
    "LidarSensor/LidarSensor/sensor_pattern/sensor_lidar/scan_mode/mid360.npy"
)

LOCAL_BRIDGE  = REPO / "sim/bridge/nova_nav_bridge.py"
LOCAL_SH      = REPO / "sim/scripts/test_factory_nova.sh"
REMOTE_BRIDGE = "/tmp/nova_sim/bridge/nova_nav_bridge.py"
REMOTE_MID360 = "/tmp/nova_sim/sensors/mid360.npy"
REMOTE_SH     = "/tmp/test_factory_nova.sh"


# ── 辅助 ──────────────────────────────────────────────────────────

def ssh_run(ssh, cmd, check=True):
    _, stdout, stderr = ssh.exec_command(cmd)
    out = stdout.read().decode(errors="replace").strip()
    err = stderr.read().decode(errors="replace").strip()
    if err:
        print(f"  [stderr] {err[:300]}")
    return out


def stream_run(ssh, cmd, timeout=300):
    """执行命令并实时打印输出."""
    transport = ssh.get_transport()
    chan = transport.open_session()
    chan.set_combine_stderr(True)
    chan.exec_command(cmd)
    deadline = time.time() + timeout
    while not chan.exit_status_ready():
        if chan.recv_ready():
            data = chan.recv(4096)
            sys.stdout.buffer.write(data)
            sys.stdout.flush()
        if time.time() > deadline:
            print("\n[TIMEOUT]")
            break
        time.sleep(0.05)
    while chan.recv_ready():
        sys.stdout.buffer.write(chan.recv(4096))
    sys.stdout.flush()
    return chan.recv_exit_status()


# ── 主流程 ────────────────────────────────────────────────────────

def main():
    # 1. 下载 mid360.npy
    print(f"[1] Downloading mid360.npy from GitHub ...")
    print(f"    {MID360_URL}")
    try:
        with urllib.request.urlopen(MID360_URL, timeout=30) as resp:
            mid360_data = resp.read()
        print(f"    Downloaded {len(mid360_data):,} bytes")
    except Exception as e:
        print(f"    ERROR: {e}")
        print("    Falling back: upload without mid360.npy (will use golden angle)")
        mid360_data = None

    # 2. 连接机器人
    print(f"\n[2] Connecting to {USER}@{HOST} ...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PASS, timeout=10)
    sftp = ssh.open_sftp()

    # 创建目录
    for d in ["/tmp/nova_sim/bridge", "/tmp/nova_sim/sensors"]:
        ssh_run(ssh, f"mkdir -p {d}")

    # 3. 上传文件
    print(f"\n[3] Uploading files ...")
    print(f"    nova_nav_bridge.py → {REMOTE_BRIDGE}")
    sftp.put(str(LOCAL_BRIDGE), REMOTE_BRIDGE)

    print(f"    test_factory_nova.sh → {REMOTE_SH} (unix line endings)")
    sh_bytes = LOCAL_SH.read_bytes().replace(b'\r\n', b'\n')
    with sftp.open(REMOTE_SH, "wb") as f:
        f.write(sh_bytes)
    ssh_run(ssh, f"chmod +x {REMOTE_SH}")

    if mid360_data is not None:
        print(f"    mid360.npy ({len(mid360_data):,} bytes) → {REMOTE_MID360}")
        with sftp.open(REMOTE_MID360, "wb") as f:
            f.write(mid360_data)
    else:
        print("    mid360.npy: SKIPPED (download failed)")

    sftp.close()

    # 4. Smoke test
    print(f"\n[4] Smoke test — verify mid360.npy loads ...")
    smoke = """python3 -c "
import numpy as np, sys
try:
    a = np.load('/tmp/nova_sim/sensors/mid360.npy')
    print(f'mid360.npy shape={a.shape} dtype={a.dtype}')
    print(f'theta range: [{a[:,0].min():.2f}, {a[:,0].max():.2f}] rad')
    print(f'phi   range: [{a[:,1].min():.2f}, {a[:,1].max():.2f}] rad')
    print('OK')
except Exception as e:
    print(f'FAIL: {e}')
    sys.exit(1)
"
"""
    out = ssh_run(ssh, smoke)
    print(f"    {out}")

    # 5. 运行导航测试
    import sys as _sys
    viz_flag = "viz" if "--viz" in _sys.argv else ""
    mode_str = "viz (MuJoCo viewer)" if viz_flag else "headless"
    print(f"\n[5] Running navigation test (goal=14,3,0.35, mode={mode_str}) ...")
    print("    (monitoring up to 240s, 'Goal Reached!' = PASS)")
    print("─" * 60)

    rc = stream_run(
        ssh,
        "source /opt/ros/humble/setup.bash 2>/dev/null; "
        f"bash /tmp/test_factory_nova.sh 14 3 0.35 240 {viz_flag}",
        timeout=280
    )

    print("─" * 60)
    print(f"\n[6] Test finished (exit={rc})")

    # 6. 最终日志摘要
    print("\n=== Bridge log (last 5 lines) ===")
    print(ssh_run(ssh, "tail -5 /tmp/nova_bridge.log 2>/dev/null || echo '(no log)'"))
    print("\n=== Adapter log (last 5 lines) ===")
    print(ssh_run(ssh, "tail -5 /tmp/pct_adapter.log 2>/dev/null || echo '(no log)'"))

    ssh.close()


if __name__ == "__main__":
    main()
