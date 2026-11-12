//! A USB CDC-ACM byte pipe and a link state, on `embassy-usb` 0.6.
//!
//! The firmware builds the `embassy_usb::Builder` with its own identity (vendor and product id,
//! strings, power) and runs `UsbDevice::run()` in a task it never cancels. This crate adds the CDC
//! function and one handler to that builder and gives back [`Tx`] (`embedded_io_async::Write`),
//! [`Rx`] (`embedded_io_async::Read`) and [`Link`] (is the cable usable, DTR, counters). It moves
//! bytes and knows nothing about what they mean. It owns no queue, task, timer or `static`.
//!
//! What it fixes in the bare `CdcAcmClass` (details in `docs/device.md`):
//!
//! * No IN packet is ever full size: [`Tx`] sends at most `max_packet_size - 1` bytes per `write`,
//!   so the computer never holds data back waiting for a short packet, and no zero-length packet
//!   is ever needed. `flush` has nothing to do.
//! * A zero-length packet from the computer is skipped and counted; `read` never returns `Ok(0)`
//!   for a non-empty buffer (which `embedded-io-async` defines as end of file).
//! * A call that is waiting when the link goes down (bus reset, deconfigure, suspend, disable) ends
//!   with [`Error::NotConnected`], and while the link is down every call returns it at once. The
//!   HAL's endpoint futures do not end by themselves on this; the link handler wakes them.
//!
//! "Up" means configured and not suspended, as reported to the crate's own handler. A partly read
//! packet is thrown away when the link goes down. DTR is reported by [`Link::dtr`] and never gates
//! data. Dropping a `write` future has sent nothing or one whole packet; dropping a `read` future
//! loses no byte the crate already holds.
//!
//! ```ignore
//! static STATE: StaticCell<usb_cdc::State<'static>> = StaticCell::new();
//! // builder: the firmware's embassy_usb::Builder, with its own identity
//! let cdc = usb_cdc::UsbCdc::new(&mut builder, STATE.init(usb_cdc::State::new()), usb_cdc::Config::new());
//! let (mut tx, mut rx, link) = cdc.split();
//! // spawn `usb.run()` for the device built from `builder`, then in the link task:
//! loop {
//!     link.wait_up().await;
//!     let mut buf = [0u8; 64];
//!     while let Ok(n) = rx.read(&mut buf).await {
//!         if tx.write_all(&buf[..n]).await.is_err() { break; }
//!     }
//! }
//! ```
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

mod pipe;
mod shared;

use core::future::poll_fn;
use core::mem::MaybeUninit;

use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, ControlChanged};
use embassy_usb::driver::Driver;

pub use pipe::{Error, Rx, Tx};
pub use shared::Stats;
use shared::{LinkHandler, Shared};

/// The largest max packet size of a full-speed bulk endpoint.
const LARGEST_PACKET: u16 = 64;

/// Storage the USB class and the link handler need for as long as the device lives.
/// Create it with [`State::new`] in a `StaticCell` or another place that outlives the device.
pub struct State<'d> {
    class: embassy_usb::class::cdc_acm::State<'d>,
    handler: MaybeUninit<LinkHandler<'d>>,
    shared: Shared,
}

impl State<'_> {
    /// An empty state.
    pub const fn new() -> Self {
        Self {
            class: embassy_usb::class::cdc_acm::State::new(),
            handler: MaybeUninit::uninit(),
            shared: Shared::new(),
        }
    }
}

impl Default for State<'_> {
    fn default() -> Self {
        Self::new()
    }
}

/// Settings of the CDC function.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Max packet size of the two bulk endpoints: 8, 16, 32 or 64 (full speed). Another value is
    /// rounded down to the next of those (below 8 up to 8) when the class is built. Each `write`
    /// then sends at most this minus one byte.
    pub max_packet_size: u16,
}

impl Config {
    /// 64 bytes, so `write` sends up to 63.
    pub const fn new() -> Self {
        Self {
            max_packet_size: LARGEST_PACKET,
        }
    }

    /// Sets the bulk endpoints' max packet size (8, 16, 32 or 64; anything else is rounded down).
    #[must_use]
    pub const fn with_max_packet_size(mut self, max_packet_size: u16) -> Self {
        self.max_packet_size = max_packet_size;
        self
    }

    const fn packet_size(&self) -> u16 {
        match self.max_packet_size {
            64.. => 64,
            32..=63 => 32,
            16..=31 => 16,
            _ => 8,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// The CDC function added to a builder, not yet split into its halves.
pub struct UsbCdc<'d, D: Driver<'d>> {
    class: CdcAcmClass<'d, D>,
    shared: &'d Shared,
    packet: usize,
}

impl<'d, D: Driver<'d>> UsbCdc<'d, D> {
    /// Adds the CDC-ACM function (two interfaces, three endpoints) and one handler to `builder`.
    /// The builder needs one free handler slot (`max_handler_count` of `embassy-usb`, default 4;
    /// the CDC class itself takes one). Call it before `builder.build()`.
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State<'d>, config: Config) -> Self {
        let State {
            class,
            handler,
            shared,
        } = state;
        let shared: &'d Shared = shared;
        let packet = config.packet_size();
        let class = CdcAcmClass::new(builder, class, packet);
        builder.handler(handler.write(LinkHandler { shared }));
        Self {
            class,
            shared,
            packet: usize::from(packet),
        }
    }

    /// Splits into the write half, the read half and the link state. Give `Tx` and `Rx` to the
    /// task that talks over the link; `Link` can be used from any task.
    pub fn split(self) -> (Tx<'d, D>, Rx<'d, D>, Link<'d>) {
        let (sender, receiver, control) = self.class.split_with_control();
        let tx = Tx {
            ep: sender,
            shared: self.shared,
            chunk: self.packet.saturating_sub(1),
        };
        let rx = Rx {
            ep: receiver,
            shared: self.shared,
            packet: self.packet,
            buf: [0; 64],
            pos: 0,
            len: 0,
            held_in: 0,
        };
        let link = Link {
            shared: self.shared,
            control,
        };
        (tx, rx, link)
    }
}

/// The state of the USB link: up (configured and not suspended) or down, DTR and counters.
pub struct Link<'d> {
    shared: &'d Shared,
    control: ControlChanged<'d>,
}

impl Link<'_> {
    /// True while the device is configured and not suspended.
    pub fn is_up(&self) -> bool {
        self.shared.is_up()
    }

    /// The DTR line as the computer last set it (false after a bus reset). Reported only: it never
    /// gates data.
    pub fn dtr(&self) -> bool {
        self.control.dtr()
    }

    /// Completes when the link is up; at once if it already is.
    pub async fn wait_up(&self) {
        poll_fn(|cx| self.shared.poll_state(true, cx)).await;
    }

    /// Completes when the link is down; at once if it already is.
    pub async fn wait_down(&self) {
        poll_fn(|cx| self.shared.poll_state(false, cx)).await;
    }

    /// The counters since power-up.
    pub fn stats(&self) -> Stats {
        self.shared.stats()
    }
}

/// What the device tells its handlers. Only for the host tests of this crate.
#[cfg(feature = "_test-hooks")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// `Handler::enabled`
    Enabled(bool),
    /// `Handler::reset`
    Reset,
    /// `Handler::addressed`
    Addressed(u8),
    /// `Handler::configured`
    Configured(bool),
    /// `Handler::suspended`
    Suspended(bool),
}

#[cfg(feature = "_test-hooks")]
impl Link<'_> {
    /// Calls the same `Handler` method the USB device calls, on this link's own handler.
    pub fn inject(&self, event: Event) {
        use embassy_usb::Handler;
        let mut handler = LinkHandler {
            shared: self.shared,
        };
        match event {
            Event::Enabled(e) => handler.enabled(e),
            Event::Reset => handler.reset(),
            Event::Addressed(a) => handler.addressed(a),
            Event::Configured(c) => handler.configured(c),
            Event::Suspended(s) => handler.suspended(s),
        }
    }
}
