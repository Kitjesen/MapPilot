import sys
import io
import paramiko

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

HOST = "192.168.66.190"
USER = "bsrl1"
PWD = "123456"
ROOT = "/home/bsrl1/brainstem"


def run(client, cmd):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=40)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    return out, err


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PWD, timeout=15,
                   look_for_keys=False, allow_agent=False)

    cmds = [
        ("han_dog bin", f"ls -la {ROOT}/han_dog/bin"),
        ("han_dog lib/src", f"ls -la {ROOT}/han_dog/lib/src"),
        ("han_dog main entry head", f"sed -n '1,120p' {ROOT}/han_dog/bin/han_dog.dart"),
        ("control arbiter search", f"cd {ROOT} && grep -rIl --include='*.dart' -i 'ControlArbiter\\|arbiter' han_dog han_dog_brain | head"),
        ("model onnx refs", f"cd {ROOT} && grep -rIn --include='*.dart' -i 'onnx\\|policy\\|\\.onnx\\|InferenceSession\\|loadModel' han_dog han_dog_brain | head -40"),
        ("profiles", f"ls -la {ROOT}/han_dog/profiles && echo '=== mini.json ===' && cat {ROOT}/han_dog/profiles/mini.json"),
    ]
    for title, cmd in cmds:
        out, err = run(client, cmd)
        print(f"\n===== {title} =====")
        if out.strip():
            print(out.rstrip())
        if err.strip():
            print("[stderr]", err.rstrip())

    client.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("ERROR:", type(e).__name__, e)
        sys.exit(1)
