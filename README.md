# usb-cdc

USB CDC-ACM byte pipe and link state on `embassy-usb` 0.6, written for the MARV V2 Rev 1 flight controller. The crate is `no_std`. It adds one CDC-ACM function and one handler to the firmware's `embassy_usb::Builder` and gives back `Tx` (`embedded_io_async::Write`), `Rx` (`embedded_io_async::Read`) and `Link` (up or down, DTR, counters). It moves bytes and knows nothing about what they mean: a protocol layer such as MAVLink sits on top of `Tx` and `Rx`, never inside. Dependencies: `embassy-usb` 0.6 (no default features), `embassy-sync` 0.8 (already in embassy-usb's tree), `embedded-io-async` 0.7 and, optionally, `defmt`.

**Status: version 0.1.0 (`publish = false`). The API may still change.** Host tests pass; nothing has been run on the board yet; see "Not covered and not verified". It works with any HAL that implements `embassy-usb-driver` and with any executor. It does not work on the `usb-device` stack and has no blocking front-end (a USB device has to be serviced all the time).

## Quick start

No hardware needed to read it; the firmware side looks like this (`ignore`d here because it needs a HAL driver):

```rust,ignore
use embedded_io_async::{Read, Write};

let mut builder = embassy_usb::Builder::new(driver, usb_config, /* descriptor and control buffers */);
// The firmware's identity is in `usb_config`; this crate has none.
static STATE: StaticCell<usb_cdc::State<'static>> = StaticCell::new();
let cdc = usb_cdc::UsbCdc::new(&mut builder, STATE.init(usb_cdc::State::new()), usb_cdc::Config::new());
let (mut tx, mut rx, link) = cdc.split();
let mut usb = builder.build();
spawner.spawn(usb_task(usb)).unwrap(); // `usb.run().await`, never cancelled

loop {
    link.wait_up().await;
    let mut buf = [0u8; 64];
    // `Err(NotConnected)` comes at once when the cable is gone, also from a call that was waiting.
    while let Ok(n) = rx.read(&mut buf).await {
        if tx.write_all(&buf[..n]).await.is_err() { break; }
    }
}
```

## What the application provides

- The `embassy_usb::Builder`, with its own vendor and product id, strings, power and `device_class` (CDC: `0x02`, or the composite values if other functions share the device). Call `UsbCdc::new` before `builder.build()`, and keep one handler slot free (`max_handler_count`, default 4; the CDC class uses one, this crate one).
- A `State` that lives as long as the device, and a task that runs `UsbDevice::run()` and is never cancelled (a dropped `run()` can leave the bus in an invalid state, per its rustdoc).
- The task that awaits `Tx`, `Rx` and `Link` futures. Only that task should wait on them: a `write` waits while the computer is not reading, and only the caller waits.
- Timeouts, if wanted. The crate has no timer.

## Rules for correct use

- **No panics.** `src/lib.rs` carries a `deny` block of clippy lints (`unwrap_used`, `expect_used`, `panic`, `unreachable`, `todo`, `unimplemented`, `indexing_slicing`) outside tests, enforced when `cargo clippy` runs; `forbid(unsafe_code)` is enforced by the compiler. The one panic path left is inside `embassy-usb`: `Builder::handler` asserts that a handler slot is free.
- **`write` takes at most `max_packet_size - 1` bytes** (63 by default). Use `write_all` for more. Do not expect one `write` per message.
- **`read` returns at least one byte or an error**, never `Ok(0)` for a non-empty buffer. A zero-length buffer returns `Ok(0)`, or `Err` while the link is down.
- **`NotConnected`** is the only error. While the link is down every `read`, `write` and `flush` returns it at once. A call that is waiting when the link goes down (reset, deconfigure, suspend, disable) returns it too, also if the link is back before the call is polled.
- **Dropped futures.** Dropping a `write` has sent nothing or one whole packet (never a part that the next call repeats or tears). Dropping a `read` loses no byte the crate already holds. Both are tested at every await point.
- **A partly read packet is thrown away when the link goes down**, so bytes of the last session never mix with the next one.
- **DTR** is reported by `Link::dtr` and never gates data. It reads false after a bus reset.
- **Not `Send`.** `Tx`, `Rx` and `Link` hold embassy-usb's class types, which are not `Send` (they borrow a `RefCell`). Run them in a task on one executor.
- **Max packet size** is 8, 16, 32 or 64 (`Config::with_max_packet_size`); another value is rounded down. The rounding is silent.

## What the crate does

- Keeps the link state: up = configured and not suspended, from its own `embassy_usb::Handler`. Down on reset, `configured(false)`, `suspended(true)` and `enabled(false)`. `Link::wait_up` and `wait_down` complete on the transition, and at once if already there. `Link::stats` counts ups, downs, resets, suspends and skipped zero-length packets (saturating `u32`).
- `Tx::write` sends one packet per call, never longer than `max_packet_size - 1`, so no zero-length packet is ever needed; `flush` does nothing.
- `Rx::read` serves a received packet over as many calls as needed and skips and counts zero-length packets.
- Ends pending calls when the link goes down. The HAL's endpoint futures do not end by themselves on a bus reset on this chip (embassy-rp 0.10.0); every `read` and `write` also waits on the link state, and the handler wakes it. See `docs/device.md` for the three traps of the bare class with file and line.
- Has no `static`, no queue, no task, no timer, no spin loop and no `critical_section::with` of its own. The link state sits behind embassy-sync's `CriticalSectionMutex` (a few instructions per poll).

## Not covered and not verified

- **Nothing has been run on the board.** The host tests use a fake `Driver` whose endpoint futures never end by themselves, as embassy-rp's do; whether the real chip, the real host stack and the three operating systems behave as `docs/device.md` says is for the board packet.
- High speed, isochronous and interrupt endpoints of the class (the notification endpoint is allocated and never used), line coding and RTS (read them from the class if needed; this crate does not expose them), remote wakeup.
- Several CDC functions on one builder (`State` and `UsbCdc` are per function; not tried).
- `Link` is not `Send` and not `Sync`.
- `addressed` is ignored.
- The test builds the `UsbDevice` and, in one test, polls its `run()` against a scripted bus. No other test runs the device.

## Measured on the MARV V2 Rev 1 board

Nothing yet. The next step measures, with a host tool on `serialport` and the firmware's own identity: enumeration within 5 s; a 1 MiB echo in random write sizes (forcing 63, 64, 65, 128, 4096) with 0 mismatches; device-to-host throughput (at least 250 kB/s wanted); bursts of exactly 64, 128 and 4096 bytes arriving complete within 50 ms, with a positive control (a build that sends full 64-byte packets must fail it); port open but not read for 10 s, then closed for 10 s, with a 1 kHz tick task missing 0 ticks; 20 close and reopen cycles with 0 errors; 10 soft detaches with a write pending giving `NotConnected` within 50 ms and the link coming back; one physical unplug.

## Tests

- Host: `cargo test --features _test-hooks` runs 34 tests (observed on this tree; plain `cargo test` also works, a dev-dependency turns the feature on). Three are property tests: IN packets of 1 to 63 bytes with the bytes unchanged for writes of 0 to 4096 bytes, the same for packet sizes 8 to 64, and `read` never giving `Ok(0)` or losing a byte over random packets (with empty ones) and random buffer sizes. The tests build a real `embassy_usb::Builder` on a fake driver (`tests/fake/mod.rs`: endpoints the test feeds, watches and makes pending). `the_real_device_reaches_the_crates_handler_and_sets_dtr` polls a real `UsbDevice::run()` against a scripted bus and control pipe, so the handler registration and DTR are tested through the real device; the other tests call the same `Handler` methods through the `_test-hooks` feature (`Link::inject`).
- Negative controls, run by hand on this tree and not kept in the repository: ten one-line mutations (sending a full 64-byte packet, returning `Ok(0)` on a zero-length packet, no wake-up on a link change, no `builder.handler` call, no session check, no down check on entry, a lost packet after a cancel, no discard of a held packet, an extra zero-length packet, no suspend clear on reset) each made at least one test fail.
- On the board: `tools/run.sh alive` only proves the crate links for the board (`hil/`); it has not been run.
- Build checks: `cargo build --target thumbv8m.main-none-eabihf` (also `--features defmt`), `cargo clippy --all-targets --features _test-hooks -- -D warnings`, `cargo fmt -- --check`, `cargo tree -e normal` (no `mavlink`, `embassy-rp` or `embassy-executor`).

## Cargo features

| Feature | Default | Effect |
|---|---|---|
| `defmt` | off | `defmt::Format` derives on `Error`, `Stats` and `Config`; enables `defmt` in `embassy-usb`, `embassy-sync` and `embedded-io-async` |
| `_test-hooks` | off | `Link::inject` and `Event`, to drive the link state from host tests. Not for production |

## Layout and licence

```text
src/            lib.rs (State, Config, UsbCdc, Link), pipe.rs (Tx, Rx, Error), shared.rs (link state, handler, Stats)
tests/          host tests and the fake driver (tests/fake/mod.rs)
docs/device.md  the three traps of the bare class with file and line, and the rules applied
hil/            on-board programs (own workspace; the only place that uses embassy-rp)
tools/run.sh    builds, flashes and runs one on-board program
```

MIT, see `LICENSE`.
