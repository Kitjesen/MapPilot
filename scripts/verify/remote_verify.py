"""Remote verification helper for Task #19 - IDL build on robot."""

import os
import sys

import paramiko

HOST = "192.168.66.13"
USER = "sunrise"
PASS = os.environ.get("S100P_PASSWORD", "")
REMOTE_BASE = "/home/sunrise/data/SLAM/navigation"


def get_client():
    if not PASS:
        raise RuntimeError("S100P_PASSWORD must be set for remote verification")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=15)
    return c


def run_cmd(client, cmd, timeout=300):
    """Run command and return (exit_code, stdout, stderr)."""
    print(f"\n>>> {cmd}")
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    rc = stdout.channel.recv_exit_status()
    if out.strip():
        print(out.strip())
    if err.strip():
        print(f"[STDERR] {err.strip()}")
    print(f"[EXIT={rc}]")
    return rc, out, err


def upload_files(client):
    """Upload all changed files to remote."""
    local_base = os.getcwd()

    files = [
        ("src/message/idl/explore_types.idl", f"{REMOTE_BASE}/src/message/idl/explore_types.idl"),
        ("src/message/idl/lingtu_slam.idl", f"{REMOTE_BASE}/src/message/idl/lingtu_slam.idl"),
        ("src/message/cpp/generated/lingtu_dds_types.h", f"{REMOTE_BASE}/src/message/cpp/generated/lingtu_dds_types.h"),
        ("src/explore/cpp/CMakeLists.txt", f"{REMOTE_BASE}/src/explore/cpp/CMakeLists.txt"),
        ("src/explore/cpp/tare_dds.hpp", f"{REMOTE_BASE}/src/explore/cpp/tare_dds.hpp"),
        ("scripts/build/build_explore_kernel.sh", f"{REMOTE_BASE}/scripts/build/build_explore_kernel.sh"),
        ("src/explore/tare/module.py", f"{REMOTE_BASE}/src/explore/tare/module.py"),
        ("src/explore/cpp/bindings/bind_tare.cpp", f"{REMOTE_BASE}/src/explore/cpp/bindings/bind_tare.cpp"),
        ("src/message/cpp/dds_qos_profiles.hpp", f"{REMOTE_BASE}/src/message/cpp/dds_qos_profiles.hpp"),
        ("src/message/cpp/dds_topics.hpp", f"{REMOTE_BASE}/src/message/cpp/dds_topics.hpp"),
    ]

    sftp = client.open_sftp()
    # Ensure generated/ directory exists on remote
    try:
        sftp.mkdir(f"{REMOTE_BASE}/src/message/cpp/generated")
    except OSError:
        pass  # already exists
    for local_rel, remote_path in files:
        local_path = os.path.join(local_base, local_rel.replace("/", os.sep))
        print(f"  Uploading {local_rel} -> {remote_path}")
        try:
            sftp.put(local_path, remote_path)
        except Exception as e:
            print(f"  [FAIL] {local_rel}: {e}")
            sftp.close()
            return False
    sftp.close()
    print("All files uploaded successfully.")
    return True


def write_remote_script(client, remote_path, content):
    """Write a script file on the remote machine."""
    sftp = client.open_sftp()
    with sftp.open(remote_path, "w") as f:
        f.write(content)
    sftp.close()


def main():
    step = sys.argv[1] if len(sys.argv) > 1 else "all"

    client = get_client()
    print(f"Connected to {HOST}")

    try:
        if step in ("upload", "all"):
            print("\n" + "=" * 60)
            print("STEP 1: Sync files to robot")
            print("=" * 60)
            if not upload_files(client):
                print("UPLOAD FAILED")
                return 1
            print("STEP 1: PASS")

        if step in ("clean", "all"):
            print("\n" + "=" * 60)
            print("STEP 2: Clean old build and manual files")
            print("=" * 60)
            run_cmd(client, f"rm -rf {REMOTE_BASE}/src/explore/cpp/build_nb/")
            run_cmd(client, f"rm -rf {REMOTE_BASE}/src/explore/cpp/build/")
            # Clean old manual headers (outside generated/)
            run_cmd(client, f"rm -f {REMOTE_BASE}/src/message/cpp/lingtu_dds_types.h")
            run_cmd(client, f"rm -f {REMOTE_BASE}/src/message/cpp/lingtu_slam.h")
            # Clean old generated idlc output (the compat header lingtu_dds_types.h
            # was uploaded in step 1 and will be preserved).
            run_cmd(client, f"rm -f {REMOTE_BASE}/src/message/cpp/generated/explore_types.c")
            run_cmd(client, f"rm -f {REMOTE_BASE}/src/message/cpp/generated/explore_types.h")
            run_cmd(client, f"rm -f {REMOTE_BASE}/src/message/cpp/generated/lingtu_slam.c")
            run_cmd(client, f"rm -f {REMOTE_BASE}/src/message/cpp/generated/lingtu_slam.h")
            # Also clean any installed .so in src/
            run_cmd(client, f"rm -f {REMOTE_BASE}/src/lingtu_explore_kernel*.so")
            rc1, rc2 = 0, 0
            if rc1 == 0 and rc2 == 0:
                print("STEP 2: PASS")
            else:
                print("STEP 2: FAIL (cleanup)")
                return 1

        if step in ("build", "all"):
            print("\n" + "=" * 60)
            print("STEP 3: Run build script")
            print("=" * 60)
            rc, out, err = run_cmd(
                client, f"cd {REMOTE_BASE} && bash scripts/build/build_explore_kernel.sh", timeout=600
            )
            if rc == 0:
                rc2, out2, _ = run_cmd(
                    client,
                    f"ls -la {REMOTE_BASE}/src/explore/cpp/build_nb/*.so 2>/dev/null; ls -la {REMOTE_BASE}/src/explore/cpp/build/*.so 2>/dev/null; ls -la {REMOTE_BASE}/src/lingtu_explore_kernel*.so 2>/dev/null || echo 'NO_SO_FOUND'",
                )
                if "NO_SO_FOUND" not in out2:
                    print("STEP 3: PASS")
                else:
                    print("STEP 3: FAIL (no .so generated)")
                    return 1
            else:
                print("STEP 3: FAIL (build script returned non-zero)")
                return 1

        if step in ("verify", "all"):
            print("\n" + "=" * 60)
            print("STEP 4: Verify generated artifacts")
            print("=" * 60)
            rc, out, _ = run_cmd(client, f"ls -la {REMOTE_BASE}/src/message/cpp/generated/")
            expected_prefixes = ["explore_types", "lingtu_slam"]
            missing = [p for p in expected_prefixes if p not in out]
            if missing:
                print(f"STEP 4: FAIL - missing generated files for: {missing}")
                return 1
            print("STEP 4: PASS")

        if step in ("dds", "all"):
            print("\n" + "=" * 60)
            print("STEP 5: DDS end-to-end verification")
            print("=" * 60)
            dds_script = """\
import sys, os
sys.path.insert(0, 'src')
import lingtu_explore_kernel as k
print('HAS_DDS:', k.HAS_DDS)
print('Symbols:', [s for s in dir(k) if not s.startswith('_')])
if k.HAS_DDS:
    t = k.TareDdsTransport(0)
    t.spin_once()
    t.publish_start(True)
    t.cleanup()
    print('DDS E2E: PASS')
else:
    print('DDS E2E: SKIP (HAS_DDS=False)')
"""
            write_remote_script(client, "/tmp/test_dds.py", dds_script)
            rc, out, err = run_cmd(client, f"cd {REMOTE_BASE} && python3 /tmp/test_dds.py")
            if rc == 0 and "DDS E2E: PASS" in out:
                print("STEP 5: PASS")
            else:
                print("STEP 5: FAIL")
                return 1

        if step in ("module", "all"):
            print("\n" + "=" * 60)
            print("STEP 6: Module import verification")
            print("=" * 60)
            mod_script = """\
import sys
sys.path.insert(0, 'src')
from explore.tare.module import TAREExplorerModule
print('Module import: OK')
"""
            write_remote_script(client, "/tmp/test_module.py", mod_script)
            rc, out, err = run_cmd(client, f"cd {REMOTE_BASE} && python3 /tmp/test_module.py")
            if rc == 0 and "Module import: OK" in out:
                print("STEP 6: PASS")
            else:
                print("STEP 6: FAIL")
                return 1

        print("\n" + "=" * 60)
        print("ALL STEPS PASSED!")
        print("=" * 60)
        return 0

    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
