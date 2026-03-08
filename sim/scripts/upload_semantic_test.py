#!/usr/bin/env python3
"""
upload_semantic_test.py — 上传语义导航测试文件到 S100P 并执行

步骤:
  1. 上传 factory_stub_test.py 到 /tmp/nova_sim/semantic/
  2. 上传 test_semantic_nav.sh 到 /tmp/test_semantic_nav.sh
  3. 运行 test_semantic_nav.sh
"""
import sys
import time
from pathlib import Path

import paramiko

HOST = "192.168.66.190"
USER = "sunrise"
PASS = "sunrise"

REPO = Path(__file__).resolve().parent.parent.parent

LOCAL_STUB = REPO / "sim/semantic/factory_stub_test.py"
LOCAL_SH   = REPO / "sim/scripts/test_semantic_nav.sh"

REMOTE_STUB = "/tmp/nova_sim/semantic/factory_stub_test.py"
REMOTE_SH   = "/tmp/test_semantic_nav.sh"


def ssh_run(ssh, cmd, check=True):
    _, stdout, stderr = ssh.exec_command(cmd)
    out = stdout.read().decode(errors="replace").strip()
    err = stderr.read().decode(errors="replace").strip()
    if err:
        print(f"  [stderr] {err[:300]}")
    return out


def stream_run(ssh, cmd, timeout=300):
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


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default="导航到目标区域",
                        help="自然语言导航指令")
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("--viz", action="store_true", help="开启 MuJoCo viewer")
    args = parser.parse_args()

    print(f"[1] Connecting to {USER}@{HOST} ...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PASS, timeout=10)
    sftp = ssh.open_sftp()

    # 创建目录
    ssh_run(ssh, "mkdir -p /tmp/nova_sim/semantic")

    print(f"[2] Uploading files ...")
    print(f"    factory_stub_test.py → {REMOTE_STUB}")
    sftp.put(str(LOCAL_STUB), REMOTE_STUB)

    print(f"    test_semantic_nav.sh → {REMOTE_SH}")
    sh_bytes = LOCAL_SH.read_bytes().replace(b'\r\n', b'\n')
    with sftp.open(REMOTE_SH, "wb") as f:
        f.write(sh_bytes)
    ssh_run(ssh, f"chmod +x {REMOTE_SH}")

    sftp.close()

    print(f"\n[3] Running semantic navigation test ...")
    print(f"    Instruction: '{args.instruction}'")
    print(f"    Timeout: {args.timeout}s")
    print(f"    Mode: {'viz' if args.viz else 'headless'}")
    print("─" * 60)

    viz_flag = "viz" if args.viz else ""
    rc = stream_run(
        ssh,
        f"source /opt/ros/humble/setup.bash 2>/dev/null; "
        f"bash {REMOTE_SH} \"{args.instruction}\" {args.timeout} {viz_flag}",
        timeout=args.timeout + 30,
    )

    print("─" * 60)
    print(f"\n[4] Test finished (exit={rc})")

    print("\n=== Semantic planner log (last 10) ===")
    print(ssh_run(ssh, "tail -10 /tmp/semantic_planner.log 2>/dev/null || echo '(no log)'"))
    print("\n=== Stub log (last 10) ===")
    print(ssh_run(ssh, "tail -10 /tmp/semantic_stub.log 2>/dev/null || echo '(no log)'"))

    ssh.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
