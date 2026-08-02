import sys
import io
import paramiko

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

HOST = "192.168.66.190"
USER = "bsrl1"
PWD = "123456"
ROOT = "/home/bsrl1/brainstem"


def run(client, cmd):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=60)
    return (stdout.read().decode("utf-8", "replace"),
            stderr.read().decode("utf-8", "replace"))


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=15,
              look_for_keys=False, allow_agent=False)

    cmds = [
        ("dart real check", "ls ~/dart-sdk/bin/dart ~/flutter/bin/dart 2>/dev/null; echo '--- try run ---'; ~/dart-sdk/bin/dart --version 2>&1 | head -2; ~/flutter/bin/dart --version 2>&1 | head -2"),
        ("bashrc path", "grep -nE 'dart-sdk|flutter' ~/.bashrc 2>/dev/null"),
        ("deps resolved?", f"ls {ROOT}/.dart_tool/package_config.json 2>/dev/null && echo 'package_config EXISTS'; stat -c '%y' {ROOT}/.dart_tool/package_config.json 2>/dev/null"),
        ("git remote/branch", f"cd {ROOT} && git remote -v 2>/dev/null; echo '---'; git branch --show-current; echo '---'; git log --oneline -1"),
        ("git dirty count", f"cd {ROOT} && git status -s 2>/dev/null | wc -l"),
        ("sshd running", "systemctl is-active ssh 2>/dev/null || service ssh status 2>/dev/null | head -1"),
    ]
    for title, cmd in cmds:
        out, err = run(c, cmd)
        print(f"\n===== {title} =====")
        if out.strip():
            print(out.rstrip())
        if err.strip():
            print("[stderr]", err.rstrip())
    c.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("ERROR:", type(e).__name__, e)
        sys.exit(1)
