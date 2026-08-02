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
        ("sim dir", f"ls -la {ROOT}/sim"),
        ("sim py scripts", f"find {ROOT}/sim -maxdepth 2 -name '*.py' -o -name '*.xml' -o -name '*.urdf' 2>/dev/null | head -40"),
        ("walk_test head", f"sed -n '1,60p' {ROOT}/sim/walk_test.py 2>/dev/null"),
        ("python + mujoco", "python3 --version; echo '---'; python3 -c 'import mujoco; print(\"mujoco\", mujoco.__version__)' 2>&1; echo '---'; python3 -c 'import numpy; print(\"numpy\", numpy.__version__)' 2>&1"),
        ("server.dart sim mode", f"sed -n '1,40p' {ROOT}/han_dog/bin/server.dart"),
        ("Dockerfile.sim", f"cat {ROOT}/Dockerfile.sim"),
        ("sim readme/requirements", f"ls {ROOT}/sim/*.md {ROOT}/sim/*.txt {ROOT}/sim/requirements* 2>/dev/null; echo '==='; head -40 {ROOT}/sim/README.md 2>/dev/null"),
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
