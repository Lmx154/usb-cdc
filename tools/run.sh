#!/usr/bin/env bash
# Builds one on-target test from hil/, flashes it through the debug probe, runs it, and prints
# its report lines. The exit status is the firmware's verdict: 0 pass, 1 fail; a timeout or a
# crash gives another status. The full log is kept under hil/logs/.
#
# Usage: tools/run.sh <bin> [timeout_seconds, default 120]
set -u
bin="${1:?usage: tools/run.sh <bin> [timeout_seconds]}"
tmo="${2:-120}"
cd "$(dirname "$0")/../hil" || exit 1

cargo build --release --workspace --bin "$bin" || exit $?

mkdir -p logs
log="logs/${bin}-$(date +%Y%m%d-%H%M%S).log"
# The build folder is wherever cargo puts it: CARGO_TARGET_DIR or a cargo config can move it.
target_dir="$(cargo metadata --format-version 1 --no-deps | sed -n 's/.*"target_directory":"\([^"]*\)".*/\1/p')"
elf="${target_dir:?cargo metadata gave no target directory}/thumbv8m.main-none-eabihf/release/${bin}"

# One probe, one user at a time, across every checkout on this machine.
lock="${XDG_RUNTIME_DIR:-/tmp}/marv-v2-probe.lock"
flock "$lock" timeout "$tmo" probe-rs run --chip RP235x "$elf" >"$log" 2>&1
status=$?

grep -E '\[(PASS|FAIL|SKIP|INFO|STAT)\]|RESULT|panicked|ERROR|Error' "$log" | tail -n 80
echo "log: hil/$log"
exit "$status"
