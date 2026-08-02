import sys
import io
import paramiko

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

HOST = "192.168.66.190"
USER = "bsrl1"
PWD = "123456"


def run(client, cmd):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=30)
    return (stdout.read().decode("utf-8", "replace"),
            stderr.read().decode("utf-8", "replace"))


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=15,
              look_for_keys=False, allow_agent=False)

    cmds = [
        ("arch/os", "uname -a; echo '---'; cat /etc/os-release 2>/dev/null | head -3; echo '---'; cat /etc/nv_tegra_release 2>/dev/null | head -2"),
        ("cpu/mem", "nproc; echo '---'; free -h | head -2"),
        ("dart/flutter", "which dart; dart --version 2>&1; echo '---'; which flutter; flutter --version 2>&1 | head -3"),
        ("remote dev servers", "ls -d ~/.cursor-server ~/.vscode-server 2>/dev/null; echo '---'; ls ~/.cursor-server/bin 2>/dev/null | head"),
        ("hardware ports", "ls -l /dev/imu /dev/ttyUSB* 2>/dev/null; echo '--- can ---'; ip link show 2>/dev/null | grep -i can; echo '--- js ---'; ls /dev/input/js* 2>/dev/null"),
        ("native libs", "ls -l /usr/local/lib/*onnx* /usr/local/lib/*pcan* /usr/local/lib/*serial* 2>/dev/null; echo '---'; ldconfig -p 2>/dev/null | grep -iE 'onnx|pcan|serialport' | head"),
        ("disk", "df -h ~ | tail -1"),
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
