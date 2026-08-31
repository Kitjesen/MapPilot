#!/usr/bin/env bash

set -euo pipefail

readonly test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repo_root="$(cd -- "${test_dir}/../../.." && pwd)"
readonly provisioner="${repo_root}/scripts/deploy/configure_go2_network.sh"
readonly work_dir="$(mktemp -d)"
readonly bin_dir="${work_dir}/bin"
readonly nmcli_log="${work_dir}/nmcli.log"

cleanup() {
    rm -rf -- "${work_dir}"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $*" >&2
    exit 1
}

mkdir -p -- "${bin_dir}"
cat >"${bin_dir}/sudo" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == "-n" ]] && shift
exec "$@"
EOF
cat >"${bin_dir}/nmcli" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >>"${LINGTU_TEST_NMCLI_LOG}"
exit 0
EOF
cat >"${bin_dir}/ip" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
case "$*" in
    "link show dev eth-test")
        printf '2: eth-test: <BROADCAST,MULTICAST,UP,LOWER_UP>\n'
        ;;
    "-o -4 addr show dev eth-test")
        printf '2: eth-test    inet 192.168.123.18/24 scope global eth-test\n'
        ;;
    "-4 route get 192.168.123.161 oif eth-test")
        printf '192.168.123.161 dev eth-test src 192.168.123.18\n'
        ;;
    *)
        exit 2
        ;;
esac
EOF
cat >"${bin_dir}/ping" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
chmod +x "${bin_dir}/sudo" "${bin_dir}/nmcli" "${bin_dir}/ip" "${bin_dir}/ping"

PATH="${bin_dir}:/usr/bin:/bin" \
    LINGTU_TEST_NMCLI_LOG="${nmcli_log}" \
    bash "${provisioner}" eth-test 192.168.123.18/24 192.168.123.161 >/dev/null

grep -Fqx \
    "connection add type ethernet ifname eth-test con-name lingtu-go2-eth-test" \
    "${nmcli_log}" || fail "persistent profile was not created"
grep -Fq "ipv4.method manual" "${nmcli_log}" || fail "manual IPv4 mode is missing"
grep -Fq "ipv4.addresses 192.168.123.18/24" "${nmcli_log}" || \
    fail "static Go2 address is missing"
grep -Fq "ipv4.never-default yes" "${nmcli_log}" || fail "default-route isolation is missing"
grep -Fq "ipv6.method disabled" "${nmcli_log}" || fail "IPv6 disablement is missing"
grep -Fqx "connection up lingtu-go2-eth-test" "${nmcli_log}" || \
    fail "persistent profile was not activated"

echo "configure_go2_network tests passed"
