# USB CDC-ACM on the MARV V2 Rev 1 board: the bare class and the rules this crate applies

Source read: `embassy-usb` 0.6.0 (`EU`), `embassy-usb-driver` 0.2.2, `embassy-rp` 0.10.0 (`ERP`), all under
`~/.cargo/registry/src/*/`. Line numbers are in those files. The sections above this line are from reading the sources; what the board does is in the last
section and in `README.md` ("Measured on the MARV V2 Rev 1 board").

## The three traps of the bare class

1. **A full-size IN packet is held back, and `flush` does nothing.**
   `EU/src/class/cdc_acm.rs:90-93` (doc of `CdcAcmClass`): a packet of exactly `max_packet_size` bytes "won't be
   processed by the host operating system until a subsequent shorter packet is sent". `Sender`'s `Write::write`
   (`cdc_acm.rs:451-463`) sends `min(buf.len(), max_packet_size)` bytes, so a 64-byte write is a full packet and
   nothing sends the zero-length packet that would end it; `flush` is `Ok(())`. A host that reads the port as a
   stream can sit on the data until the next short packet.
2. **A zero-length packet from the computer reads as end of file.**
   `BufferedReceiver::read` (`cdc_acm.rs:576-604`) stores the length of the received packet (`cdc_acm.rs:597-603`)
   and returns `read_from_buffer` (`cdc_acm.rs:542-548`), which is 0 for an empty packet. The branch for a caller buffer
   larger than a packet (`cdc_acm.rs:585-591`) returns the endpoint's length directly, also 0. `embedded-io-async`
   defines `Ok(0)` for a non-empty buffer as end of file, and `read_exact` and most framing code stop there.
3. **A pending endpoint call does not end when the cable goes.**
   `EndpointIn::write` and `EndpointOut::read` of embassy-rp 0.10.0 (`ERP/src/usb.rs:571-590` and `614-640`) wait in a
   `poll_fn` that only looks at the buffer's `available` bit (`usb.rs:580`, `usb.rs:624`); nothing in the file returns
   `EndpointError::Disabled`. A bus reset (`usb.rs:393-412`) disables the endpoints in the registers and clears
   `buff_status`, but it leaves the `available` bit alone. A write in flight when the cable is pulled therefore stays
   pending until the next enumeration. Nuance found while checking: the reset does wake every endpoint waker
   (`usb.rs:406-411`), so the futures are polled again, but they find the same state and go back to sleep.
   The class does not turn this into an error either: `Handler::configured(false)` is not called on a reset
   (`EU/src/lib.rs:447-457` calls only `reset()` and `set_alternate_setting`), and the device clears its own
   `suspended` flag on reset without calling `suspended(false)`.

## What the handler hears, and what "up" means here

The crate registers one `embassy_usb::Handler` with `Builder::handler` (`EU/src/builder.rs:275`; at most
`max_handler_count` of them, default 4, the CDC class uses one). The device calls it from `UsbDevice::run()`:

| Call | Where | Link |
|---|---|---|
| `enabled(true / false)` | `EU/src/lib.rs:486`, `:307`, `:495` | down (not configured, not suspended) |
| `reset()` | `lib.rs:455` | down, suspend cleared, `Stats::resets` + 1 |
| `addressed(addr)` | `lib.rs:526` | ignored (a device is addressed after a reset, which already made the link down) |
| `configured(true / false)` | `lib.rs:544`, `:562` | up / down (unless suspended) |
| `suspended(true / false)` | `lib.rs:477`, `:470`, `:336` | down / up again if configured |

Up = configured and not suspended. Every change of up counts in `Stats` and wakes every waiting future. A counter
(the session) changes each time the link comes up, so a call that began in one session ends with `NotConnected` even
if the link went down and up again before the call was polled.

## The rules this crate applies

- `Tx::write` sends one packet of at most `max_packet_size - 1` bytes (63 by default) and returns its length. No IN
  packet reaches `max_packet_size`, so none needs a zero-length packet. A zero-length write sends nothing.
  `flush` returns at once (`Ok` while up).
- `Rx::read` keeps one packet in a 64-byte buffer and serves it over as many calls as the caller's buffer needs.
  An empty packet is skipped and counted in `Stats::zlp_skipped`. A partly served packet is dropped when the link goes
  down. Data is taken from the endpoint only when nothing is held, and the packet becomes the crate's in the poll in
  which the endpoint completes: a dropped `read` loses nothing that was held.
- Every `read` and `write` first checks the link (a down link is `Err(NotConnected)` without waiting), then waits on
  the endpoint future and on the link at once; the link handler's wake-up is what ends the call when the HAL's
  future does not.
- The endpoint's `Disabled` and `BufferOverflow` errors both map to `NotConnected`. `BufferOverflow` cannot happen:
  the packet is at most `max_packet_size - 1` bytes and the receive buffer is a full packet.
- DTR comes from the class (`ControlChanged::dtr`) and is only reported.

## Shared state and its cost

The link state sits in the crate's `State` behind `embassy_sync::blocking_mutex::CriticalSectionMutex` (as the CDC class's
own control state does, `cdc_acm.rs:108-115`): the handler runs in the task of `UsbDevice::run()`, the futures in
the link task, possibly on another executor or core. The crate has no `critical_section::with` call of its own
and no atomic; the lock is held for a few instructions per poll.

## What the board did (measured with `usb_cdc_link`; figures in `README.md`)

- **Soft detach.** Releasing the D+ pull-up while the device is configured gives `Handler::reset()` about 100 us
  later (98 to 116 us in 70 cycles) and nothing else until the pull-up is enabled again. The crate therefore sees
  the link go down and a pending `Tx::write` returns `NotConnected` 161 to 234 us after the release. No suspend
  comes first. After the pull-up is enabled again: `suspended(true)` 3 ms later (idle
  bus before the computer reacts), `reset()` twice (about 0.6 s and 0.7 s after the release), `addressed`, then
  `configured(true)` about 345 ms after the pull-up came back. So `Stats` counts one down, one up, three resets (sometimes
  four) and one suspend per soft detach.
- **Closing the port on the computer does not touch the link.** `Stats` did not change over 20 close/open cycles; DTR
  goes false on close and true on open (two edges per cycle) and nothing else is reported.
- **A computer that stops reading.** `Tx::write` completes when the endpoint takes the packet: with the port open and
  not read, the computer's side took about 5 KB (81 records of 63 bytes with the one in the endpoint) and `FIONREAD`
  stayed at 4095; then every `write_all` waited for its full timeout (100.005 to 100.012 ms measured), and the
  executor's 1 kHz task never ran late (largest period 1.27 ms over all runs, limit 1.5 ms). With the port closed the endpoint stays NAKing
  and writes wait the same way. The packet that sat in the endpoint was delivered when the port was opened again.
- **Bytes from the old session.** 40 bytes the computer sent while the device was not reading were still in the OUT
  endpoint when the device detached; none of them reached the device after the bus reset in 70 cycles (in a control without a
  detach they do reach it). `Rx` also drops a packet it holds when the session changes.
- **Interrupts-off time.** Longest gap between SysTick interrupts: idle 7.5 to 15.6 us, echo running 8.7 to 15.8 us,
  link flapping 17.5 to 23.3 us. `Link::is_up()` costs 91 to 93 cycles including the critical section. The figures
  cover the whole program (HAL, embassy-usb, timer driver, this crate).

- **A physical unplug with the board on battery power** (`usb_cdc_unplug`, one pull, 2026-11-12). The controller has
  no VBUS detection in embassy-rp, so the pull shows as a suspend and nothing else: `suspended(true)` 3208 us after
  the last good write, the pending `Tx::write` back with `NotConnected` 38 us later. With the cable back: two resets,
  `configured(true)` 240 ms after the first. Details in the README, check F.

## Still open for the board

- Whether the computer can end the link without a person (`USBDEVFS_RESET` and sysfs need root here).
- Windows and macOS.
- Throughput with 63-byte packets was measured by the first packet: 757 kB/s device to computer (at least 250 kB/s
  was required).
