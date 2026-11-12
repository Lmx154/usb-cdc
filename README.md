# usb-cdc

USB CDC-ACM byte pipe and link state on `embassy-usb` 0.6, written for the MARV V2 Rev 1 flight controller. The crate is `no_std`. It adds one CDC-ACM function and one handler to the firmware's `embassy_usb::Builder` and gives back `Tx` (`embedded_io_async::Write`), `Rx` (`embedded_io_async::Read`) and `Link` (up or down, DTR, counters). It moves bytes and knows nothing about what they mean: a protocol layer such as MAVLink sits on top of `Tx` and `Rx`, never inside. Dependencies: `embassy-usb` 0.6 (no default features), `embassy-sync` 0.8 (already in embassy-usb's tree), `embedded-io-async` 0.7 and, optionally, `defmt`.

**Status: version 0.1.0 (`publish = false`). The API may still change.** Host tests pass and the crate has been run on the MARV V2 Rev 1 board (results in "Measured on the MARV V2 Rev 1 board"; what is still open is in "Not covered and not verified"). It works with any HAL that implements `embassy-usb-driver` and with any executor. It does not work on the `usb-device` stack and has no blocking front-end (a USB device has to be serviced all the time).

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

- The `embassy_usb::Builder`, with its own vendor and product id, strings and power. Leave `device_class`, `device_sub_class` and `device_protocol` at the defaults of `embassy_usb::Config::new` (0xEF, 0x02, 0x01): the CDC function is added with an interface association descriptor, which is what those values announce, and `Builder::new` panics for any other class while `composite_with_iads` is true, its default (`embassy-usb` 0.6.0 `builder.rs:176-182`; setting 0x02 was not tried on the board). Call `UsbCdc::new` before `builder.build()`, and keep one handler slot free (`max_handler_count`, default 4; the CDC class uses one, this crate one).
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
- **Not `Send`.** `Tx`, `Rx` and `Link` hold embassy-usb's class types, which are not `Send` (they borrow a `RefCell`): never use them from two cores. Tasks on different executors of the same core are fine (the shared link state sits behind a critical-section mutex); the on-board link program runs the writer, the reader and a task that reads `Link` as three tasks on one executor.
- **Max packet size** is 8, 16, 32 or 64 (`Config::with_max_packet_size`); another value is rounded down. The rounding is silent.

## What the crate does

- Keeps the link state: up = configured and not suspended, from its own `embassy_usb::Handler`. Down on reset, `configured(false)`, `suspended(true)` and `enabled(false)`. `Link::wait_up` and `wait_down` complete on the transition, and at once if already there. `Link::stats` counts ups, downs, resets, suspends and skipped zero-length packets (saturating `u32`).
- `Tx::write` sends one packet per call, never longer than `max_packet_size - 1`, so no zero-length packet is ever needed; `flush` does nothing.
- `Rx::read` serves a received packet over as many calls as needed and skips and counts zero-length packets.
- Ends pending calls when the link goes down. The HAL's endpoint futures do not end by themselves on a bus reset on this chip (embassy-rp 0.10.0); every `read` and `write` also waits on the link state, and the handler wakes it. See `docs/device.md` for the three traps of the bare class with file and line.
- Has no `static`, no queue, no task, no timer, no spin loop and no `critical_section::with` of its own. The link state sits behind embassy-sync's `CriticalSectionMutex` (a few instructions per poll).

## Not covered and not verified

- **A cable pulled while the board keeps running.** The owner ran the cable-pull program once (`tools/usb_smoke.sh unplug`, `usb_cdc_unplug` with `usb-cdc-host unplug`, 2026-11-12). This board is powered through its own USB cable, so pulling it switched the board off: the debug session ended ("An ARM specific error occurred"), and after the cable was back the program had started again from the beginning (records 0, downs 0, the normal power-up sequence with `configured(true)` 349 ms after start). What that run does show: 119162 records of 63 bytes reached the computer before the pull with 0 bad and 0 missing; after the cable was back the port returned (8.1 s after the pull was seen, most of it the person) and an echo of 4 KiB in the new session had 0 mismatched, missing or extra bytes. What it cannot show: what a pending write does when the cable goes while the board stays powered. That needs the board powered from another source and has not been run. The only on-board evidence that a pending write ends when the link goes down is the soft detach (check C below), where the controller reported a bus reset; a pulled cable on a powered board would probably show as a suspend after about 3 ms of idle bus, which the crate treats as down (host tests) but which has not been seen with the link up on the board. After that run the host program was changed to report this case as `[SKIP]` with the reason instead of a failed check, and the script to show the prompts as they come and to stop its board run when it is stopped; those changes are built but have not been run with a person at the board.
- **Check D, the computer making the link go away without a person.** No way was found that works without root on this computer: the device node `/dev/bus/usb/BBB/DDD` is `root:root 0664` (no `USBDEVFS_RESET`), and `power/control`, `authorized` and `remove` in sysfs are not writable by the ordinary user. The device side (`CYCLE 1`) and the host side (an ioctl on the node, if it can be opened) exist in the programs but have never run past that check.
- **Windows and macOS.** Everything above ran on one Linux computer (kernel 7.0.0-42, `cdc_acm`, `serialport` 4.x). The held-back full packet, DTR on open and the enumeration sequence are host-stack behaviour that may differ.
- Only one physical board, one cable, no hub; one full-speed controller (RP2350 USB). A cable pulled in the middle of an enumeration, a bus reset while a `read` is half served, and a suspend that the computer causes (selective suspend) were not run on the board; the host tests cover the crate's reaction to them against a fake driver.
- The longest interrupts-off time was measured as a whole (see check E). This crate's own critical section could not be separated from embassy-usb's and the HAL's.
- High speed, isochronous and interrupt endpoints of the class (the notification endpoint is allocated and never used), line coding and RTS (read them from the class if needed; this crate does not expose them), remote wakeup.
- Several CDC functions on one builder (`State` and `UsbCdc` are per function; not tried).
- `Link` is not `Send` and not `Sync`.
- `addressed` is ignored.
- Throughput with the computer under load, and long runs (the longest run was 50 s of the link program).
- The host tests build the `UsbDevice` and, in one test, poll its `run()` against a scripted bus. No other test runs the device.

## Measured on the MARV V2 Rev 1 board

Board: MARV V2 Rev 1 (RP2354B, 150 MHz), its own USB port cabled to a Linux computer (kernel 7.0.0-42-generic, `cdc_acm`, port `/dev/ttyACM1`; `ttyACM0` is the debug probe). USB identity of the test programs: VID 0xc0de, PID 0xcafe, "MARV", "usb-cdc test", "usb-cdc-hil". Each figure names the program that measured it. `tools/usb_smoke.sh smoke|fullpacket|link` builds, flashes (through `tools/run.sh` only), runs the device program and the host program (`host/`, on `serialport`) together.

From the first on-board packet (`usb_cdc_smoke`, `usb_cdc_fullpacket`; recorded then, and `usb_cdc_smoke` passed again, 17 checks, when the second packet was done):

| What | Measured |
|---|---|
| Link up after the pull-up | about 345 ms; the host sees the port by VID and PID; reading the strings through `serialport` right after the port appears gives none, sysfs does |
| Bus before the first configuration | Linux resets the bus twice and the bus idles once (`resets` 2, `suspends` 1 while still down) |
| DTR | true 17 to 56 ms after the link came up in those runs, when the computer opened the port; no modem service touched the port |
| Echo 1 MiB in awkward sizes (63, 64, 65, 128, 4096 and random) | 0 errors, 319 kB/s each way |
| Device to computer, 4 MiB | 757 kB/s |
| Computer to device, 1 MiB | 639 kB/s |
| Bursts of 64 / 128 / 4096 bytes | complete within 0.1 / 0.2 / 5.4 ms |
| Positive control (`usb_cdc_fullpacket`, the bare class with full 64-byte packets) | this computer held a lone full packet back until more data followed: 10 of 10; 128 and 4096 bytes arrived promptly |

From the second packet, `usb_cdc_link` with `usb-cdc-host link` (`tools/usb_smoke.sh link`; 7 complete runs, every host check passed and the device program passed each time (13 host checks in the first run, 14 in the other six, which have the control of check C); figures are the range over the runs; the last three runs, with the final programs, passed in a row):

| Check | Measured |
|---|---|
| A. Computer keeps the port open and does not read for 10 s, then closes it for 10 s (1 kHz tick task, link task and a writer of numbered 63-byte records, each `write_all` in a 100 ms timeout, on one executor) | Tick task: 0 periods over 1.5 ms in both phases (about 10000 ticks each), largest period 1005 to 1270 us, smallest 758 to 995 us. Writer: phase 1 wrote 81 records and timed out 99 times, phase 2 wrote 0 and timed out 100 times, 0 errors other than the timeout, longest single `write_all` 100.005 to 100.012 ms. Computer received 4 to 6 records over the whole run, all whole with a right checksum, sequence strictly increasing (0 bad frames); the first one received is number 80, the last record written before the buffers filled, which the endpoint held until the port was opened again (a completed `Tx::write` means the endpoint took the packet, not that the computer has it). `Link::is_up()` read true in the middle of both phases; `dtr()` read true in phase 1 and false in phase 2. Buffered by the computer's side with nothing reading: the device's writes were accepted until 81 records (5103 bytes, of which 63 sit in the endpoint) were out; `FIONREAD` read 4095 (the tty read buffer limit) in every run |
| B. 20 cycles of close, 200 ms, open, START, echo 64 KiB in awkward sizes | 0 mismatched, 0 missing, 0 extra bytes in every cycle, every run. `Stats` did not change across the cycles (the link stays up when the port closes: ups, downs, resets and suspends unchanged); `dtr()` was true after every open; a 1 kHz sampler on the device saw 40 DTR edges for the 20 close/open pairs |
| C. Soft detach (release of the D+ pull-up, as `usb_cdc_smoke` does at its end) with a `Tx::write` pending, 10 times per run | The pending write returned `Err(NotConnected)` 161 to 234 us after the pull-up was released, every time (70 of 70, limit 50 ms). After 500 ms the pull-up was enabled again, the link came back, and an echo of 4 KiB passed every time. No byte of the old session in either direction: 0 stray bytes from the device, and 40 bytes the computer had sent before the detach (sitting unread in the endpoint) never reached the device (a control with no detach showed that they do reach it when the link stays up). `Stats`: +10 ups, +10 downs, +10 suspends and +30 resets per run (+31 in three of the seven: a cycle can report a second reset within 0.2 ms of the first). Events: see below |
| D. Computer makes the link go away without a person | Not possible here without root (see "Not covered and not verified"); no figure |
| E. Longest time interrupts are masked (longest gap between SysTick interrupts, SysTick every 300 cycles = 2 us, highest priority; the gap is at least the masked time and at most 2 us more) | Link up and idle: 7.5 to 15.6 us. Echo of 256 KiB running: 8.7 to 15.8 us. Link flapping (the C cycles): 17.5 to 23.3 us. One `Link::is_up()` call (one critical section of the crate's shared state, including entering and leaving it) costs 91 to 93 cycles (0.6 us), `Link::stats()` 113 to 115 cycles. The method cannot separate this crate's critical section from embassy-usb's, the HAL's and the timer driver's: the idle figure, with no traffic and the crate's lock only taken by a 1 kHz task, is already 7.5 us or more, so the crate's own sections (a few compares and at most six wakes) are below what the measurement can resolve |

What the controller reports on a soft detach (the second `Handler` in `hil_common::spy` stores each call with its time; times are after the pull-up was released, all ten cycles of every run alike within 10 us at the start and 3 ms later): `reset` 98 to 116 us after the release (in 3 of 70 cycles a second one within 0.2 ms; why the controller reports a reset was not investigated, presumably it reads the released line as SE0), so the crate goes down and the pending write returns at once; then nothing for 500 ms; the pull-up is enabled; `suspended(true)` 3 ms after that (the bus is idle until the computer notices); `reset` twice, at about 604 to 610 ms and 716 to 720 ms; `addressed` at about 830 ms; `configured(true)` at about 843 to 854 ms, about 345 ms after the pull-up came back. The computer notices the disconnect 296 to 318 ms after the release (the port is gone from the port list).

Check F (`usb_cdc_unplug`) was run once by the owner; on this board the cable is also the power supply, so it could not show what it was written for: see "Not covered and not verified".

Run F: `tools/usb_smoke.sh unplug`. It builds, flashes and starts the device program, then prints banners: "PULL THE BOARD'S USB CABLE NOW" (pull only the board's own cable, not the debug probe's), then "PUT THE BOARD'S USB CABLE BACK NOW" once it saw the port go away. It reports the time from the last good write to `NotConnected`, the `Handler` calls with their times, and whether an echo of 4 KiB passes in the new session.

## Tests

- Host: `cargo test --features _test-hooks` runs 34 tests (observed on this tree; plain `cargo test` also works, a dev-dependency turns the feature on). Three are property tests: IN packets of 1 to 63 bytes with the bytes unchanged for writes of 0 to 4096 bytes, the same for packet sizes 8 to 64, and `read` never giving `Ok(0)` or losing a byte over random packets (with empty ones) and random buffer sizes. The tests build a real `embassy_usb::Builder` on a fake driver (`tests/fake/mod.rs`: endpoints the test feeds, watches and makes pending). `the_real_device_reaches_the_crates_handler_and_sets_dtr` polls a real `UsbDevice::run()` against a scripted bus and control pipe, so the handler registration and DTR are tested through the real device; the other tests call the same `Handler` methods through the `_test-hooks` feature (`Link::inject`).
- Negative controls, run by hand on this tree and not kept in the repository: ten one-line mutations (sending a full 64-byte packet, returning `Ok(0)` on a zero-length packet, no wake-up on a link change, no `builder.handler` call, no session check, no down check on entry, a lost packet after a cancel, no discard of a held packet, an extra zero-length packet, no suspend clear on reset) each made at least one test fail.
- On the board: `hil/` has `alive` (the crate links for the board), `usb_cdc_smoke`, `usb_cdc_fullpacket`, `usb_cdc_link` and `usb_cdc_unplug`; `tools/usb_smoke.sh smoke|fullpacket|link|unplug` runs one of them with the matching `host/` subcommand. `unplug` has not been run (needs a person).
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
tools/usb_smoke.sh   runs an on-board program and the host program (host/) together
```

MIT, see `LICENSE`.
