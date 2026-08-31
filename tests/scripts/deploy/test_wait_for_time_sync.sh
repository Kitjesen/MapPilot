#!/usr/bin/env bash

set -euo pipefail

readonly test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly repo_root="$(cd -- "${test_dir}/../../.." && pwd)"
readonly waiter="${repo_root}/scripts/deploy/thunder/wait_for_time_sync.sh"
readonly work_dir="$(mktemp -d)"

cleanup() {
    rm -rf -- "${work_dir}"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $*" >&2
    exit 1
}

assert_eq() {
    local expected="$1"
    local actual="$2"
    local context="$3"
    [[ "${actual}" == "${expected}" ]] || fail "${context}: expected '${expected}', got '${actual}'"
}

make_unsynchronized_timedatectl() {
    local bin_dir="$1"
    mkdir -p -- "${bin_dir}"
    cat >"${bin_dir}/timedatectl" <<'EOF'
#!/usr/bin/env bash
printf 'no\n'
EOF
    chmod +x "${bin_dir}/timedatectl"
}

make_recording_timeout() {
    local bin_dir="$1"
    cat >"${bin_dir}/timeout" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "${3-}" >>"${LINGTU_TEST_TIMEOUT_LOG}"
exit 124
EOF
    chmod +x "${bin_dir}/timeout"
}

run_waiter() {
    local case_dir="$1"
    local duration="$2"
    local marker="$3"
    local timeout_log="$4"
    PATH="${case_dir}/bin:/usr/bin:/bin" \
        LINGTU_TIME_SYNC_WAIT_SECONDS="${duration}" \
        LINGTU_TIME_SYNC_WAIT_MARKER="${marker}" \
        LINGTU_TEST_TIMEOUT_LOG="${timeout_log}" \
        bash "${waiter}"
}

test_offline_timeout_is_bounded() {
    local case_dir="${work_dir}/offline"
    local marker="${case_dir}/run/time-sync-wait.done"
    local timeout_log="${case_dir}/unused.log"
    local started_ms
    local elapsed_ms
    local output
    make_unsynchronized_timedatectl "${case_dir}/bin"

    started_ms="$(date +%s%3N)"
    output="$(run_waiter "${case_dir}" 1 "${marker}" "${timeout_log}" 2>&1)"
    elapsed_ms=$(( $(date +%s%3N) - started_ms ))

    (( elapsed_ms >= 700 )) || fail "offline wait returned too early (${elapsed_ms}ms)"
    (( elapsed_ms < 3000 )) || fail "offline wait exceeded hard test bound (${elapsed_ms}ms)"
    [[ "${output}" == *"1s bound reached"* ]] || fail "offline wait did not report its bound"
    [[ -f "${marker}" ]] || fail "offline wait did not create the once-per-boot marker"
}

test_marker_short_circuits_second_attempt() {
    local case_dir="${work_dir}/marker"
    local marker="${case_dir}/run/time-sync-wait.done"
    local timeout_log="${case_dir}/timeout.log"
    local second_output
    make_unsynchronized_timedatectl "${case_dir}/bin"
    make_recording_timeout "${case_dir}/bin"

    run_waiter "${case_dir}" 1 "${marker}" "${timeout_log}" >/dev/null 2>&1
    second_output="$(run_waiter "${case_dir}" 1 "${marker}" "${timeout_log}" 2>&1)"

    assert_eq "1" "$(wc -l <"${timeout_log}" | tr -d '[:space:]')" "timeout invocation count"
    [[ "${second_output}" == *"already attempted this boot"* ]] || fail "second attempt did not short-circuit"
}

test_untrusted_durations_are_normalized_before_arithmetic() {
    local long_duration
    local index=0
    long_duration="$(printf '9%.0s' {1..256})"

    while IFS='|' read -r duration expected; do
        local case_dir="${work_dir}/duration-${index}"
        local marker="${case_dir}/run/time-sync-wait.done"
        local timeout_log="${case_dir}/timeout.log"
        make_unsynchronized_timedatectl "${case_dir}/bin"
        make_recording_timeout "${case_dir}/bin"

        run_waiter "${case_dir}" "${duration}" "${marker}" "${timeout_log}" >/dev/null 2>&1
        assert_eq "${expected}" "$(<"${timeout_log}")" "normalized duration '${duration:0:16}'"
        index=$((index + 1))
    done <<EOF
not-a-number|8s
08|8s
${long_duration}|8s
16|15s
EOF
}

test_offline_timeout_is_bounded
test_marker_short_circuits_second_attempt
test_untrusted_durations_are_normalized_before_arithmetic
echo "wait_for_time_sync tests passed"
