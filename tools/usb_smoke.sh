#!/usr/bin/env bash
# Runs one on-board USB program through tools/run.sh in the background, runs the host program
# against it, waits for both, prints both reports. Exit status 0 only if both exit 0.
#
# Usage: tools/usb_smoke.sh [smoke|fullpacket]
set -u
mode="${1:-smoke}"
case "$mode" in
  smoke) bin=usb_cdc_smoke ;;
  fullpacket) bin=usb_cdc_fullpacket ;;
  *) echo "usage: tools/usb_smoke.sh [smoke|fullpacket]" >&2; exit 2 ;;
esac
cd "$(dirname "$0")/.." || exit 1
root="$PWD"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

# The build folder is shared by every driver repo: make the crate rebuild from its sources.
touch src/lib.rs hil/src/lib.rs "hil/src/bin/$bin.rs"
cargo build --release --manifest-path host/Cargo.toml || exit $?
host_bin="$(cargo metadata --format-version 1 --no-deps --manifest-path host/Cargo.toml | sed -n 's/.*"target_directory":"\([^"]*\)".*/\1/p')/release/usb-cdc-host"
[ -x "$host_bin" ] || { echo "host program not found at $host_bin" >&2; exit 1; }

mkdir -p hil/logs
stamp="$work/stamp"
touch "$stamp"
sleep 1
tools/run.sh "$bin" 150 >"$work/device.out" 2>&1 &
dev_pid=$!

# run.sh builds, flashes and starts the program; its log appears in hil/logs and shows the start line.
log=""
for _ in $(seq 1 1500); do
  log="$(find hil/logs -name "${bin}-*.log" -newer "$stamp" 2>/dev/null | sort | tail -n 1)"
  if [ -n "$log" ] && grep -q "device start" "$log" 2>/dev/null; then break; fi
  if ! kill -0 "$dev_pid" 2>/dev/null; then break; fi
  log=""
  sleep 0.1
done
if [ -z "$log" ] || ! grep -q "device start" "$log" 2>/dev/null; then
  wait "$dev_pid"; dev_status=$?
  echo "== device program: did not start (exit $dev_status) =="
  cat "$work/device.out"
  exit 1
fi

"$host_bin" "$mode" >"$work/host.out" 2>&1
host_status=$?
wait "$dev_pid"
dev_status=$?

echo "== host report ($mode) =="
cat "$work/host.out"
echo "== device report ($bin) =="
cat "$work/device.out"
echo "== host exit $host_status, device exit $dev_status =="
if [ "$host_status" -eq 0 ] && [ "$dev_status" -eq 0 ]; then exit 0; fi
exit 1
