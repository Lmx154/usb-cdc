#!/usr/bin/env bash
# Runs one on-board USB program through tools/run.sh in the background, runs the host program
# against it, waits for both, prints both reports. Exit status 0 only if both exit 0.
#
# Usage: tools/usb_smoke.sh [smoke|fullpacket|link|unplug]   (unplug needs a person at the board)
set -u
mode="${1:-smoke}"
tmo=150
case "$mode" in
  smoke) bin=usb_cdc_smoke ;;
  fullpacket) bin=usb_cdc_fullpacket ;;
  link) bin=usb_cdc_link; tmo=280 ;;
  unplug) bin=usb_cdc_unplug; tmo=960 ;;
  *) echo "usage: tools/usb_smoke.sh [smoke|fullpacket|link|unplug]" >&2; exit 2 ;;
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
# The device run gets its own process group, so that stopping this script (Ctrl-C included) also
# stops it: a run left behind would keep the debug probe locked for its whole timeout.
setsid tools/run.sh "$bin" "$tmo" >"$work/device.out" 2>&1 &
dev_pid=$!
trap 'kill -- "-$dev_pid" 2>/dev/null; rm -rf "$work"' EXIT
trap 'exit 130' INT TERM

echo "Building and flashing $bin (this waits if another run is using the debug probe) ..."
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

if [ "$mode" = unplug ]; then
  echo "The board program is running. Prompts follow: act only when one tells you to."
  # A person has to act on what the host program says: show it as it comes.
  "$host_bin" "$mode" 2>&1 | tee "$work/host.out"
  host_status=${PIPESTATUS[0]}
else
  "$host_bin" "$mode" >"$work/host.out" 2>&1
  host_status=$?
fi
wait "$dev_pid"
dev_status=$?

if [ "$mode" != unplug ]; then
  echo "== host report ($mode) =="
  cat "$work/host.out"
fi
echo "== device report ($bin) =="
cat "$work/device.out"
echo "== host exit $host_status, device exit $dev_status =="
if [ "$host_status" -eq 0 ] && [ "$dev_status" -eq 0 ]; then exit 0; fi
if [ "$mode" = unplug ] && [ "$host_status" -eq 0 ] && grep -q '^\[SKIP\] F: the board lost power' "$work/host.out"; then
  # The board is powered through the cable that was pulled: the debug session ends with the power
  # ("An ARM specific error occurred"), so the device program cannot give a verdict. Not a failure.
  echo "The board lost power with its cable, so the device run ended early: expected on a board powered through that cable."
  exit 0
fi
exit 1
