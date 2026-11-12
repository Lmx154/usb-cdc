# USB CDC-ACM on the MARV V2 Rev 1 board: the bare class and the rules this crate applies

Source read: `embassy-usb` 0.6.0 (`EU`), `embassy-usb-driver` 0.2.2, `embassy-rp` 0.10.0 (`ERP`), all under
`~/.cargo/registry/src/*/`. Line numbers are in those files. Nothing here was measured on the board yet;
the board facts are in `README.md` ("Measured on the MARV V2 Rev 1 board", still empty).

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

## Open questions for the board

- Whether the host really holds back a 64-byte IN packet on Linux, Windows and macOS (the crate avoids it; the
  positive control of the board test shows it).
- Whether a bus reset or a soft detach ends a pending `write` within 50 ms through this crate (host tests prove the
  wake-up path against a fake that never ends its futures, as embassy-rp does).
- Throughput with 63-byte packets (the contract asks for at least 250 kB/s device to host).
