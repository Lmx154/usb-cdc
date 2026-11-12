//! The link state shared by the USB device's handler (written from the task that runs
//! `UsbDevice::run()`) and the `Tx`, `Rx` and `Link` handles (read from any task).

use core::cell::RefCell;
use core::task::{Context, Poll};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::waitqueue::{MultiWakerRegistration, WakerRegistration};
use embassy_usb::Handler;

/// Counters kept by the link, readable through [`Link::stats`](crate::Link::stats). They saturate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Stats {
    /// Times the link went from down to up.
    pub ups: u32,
    /// Times the link went from up to down, for any reason.
    pub downs: u32,
    /// Bus resets reported by the device (counted whether or not the link was up).
    pub resets: u32,
    /// Times the bus entered suspend (a second suspend report without a resume is not counted).
    pub suspends: u32,
    /// Zero-length packets received from the computer and skipped by `Rx::read`.
    pub zlp_skipped: u32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Side {
    Tx,
    Rx,
}

struct Inner {
    configured: bool,
    suspended: bool,
    /// Changes every time the link comes up. A call that started in one session never goes on in another.
    session: u32,
    stats: Stats,
    tx: WakerRegistration,
    rx: WakerRegistration,
    link: MultiWakerRegistration<4>,
}

impl Inner {
    const fn up(&self) -> bool {
        self.configured && !self.suspended
    }
}

pub(crate) struct Shared {
    inner: CriticalSectionMutex<RefCell<Inner>>,
}

impl Shared {
    pub(crate) const fn new() -> Self {
        Self {
            inner: CriticalSectionMutex::new(RefCell::new(Inner {
                configured: false,
                suspended: false,
                session: 0,
                stats: Stats {
                    ups: 0,
                    downs: 0,
                    resets: 0,
                    suspends: 0,
                    zlp_skipped: 0,
                },
                tx: WakerRegistration::new(),
                rx: WakerRegistration::new(),
                link: MultiWakerRegistration::new(),
            })),
        }
    }

    fn with<R>(&self, f: impl FnOnce(&mut Inner) -> R) -> R {
        self.inner.lock(|cell| f(&mut cell.borrow_mut()))
    }

    /// Applies one change and, if the link went up or down because of it, counts it and wakes
    /// every waiting future so that it sees the new state.
    fn change(&self, f: impl FnOnce(&mut Inner)) {
        self.with(|s| {
            let was_up = s.up();
            f(s);
            let now_up = s.up();
            if now_up == was_up {
                return;
            }
            if now_up {
                s.session = s.session.wrapping_add(1);
                s.stats.ups = s.stats.ups.saturating_add(1);
            } else {
                s.stats.downs = s.stats.downs.saturating_add(1);
            }
            s.tx.wake();
            s.rx.wake();
            s.link.wake();
        });
    }

    pub(crate) fn is_up(&self) -> bool {
        self.with(|s| s.up())
    }

    /// The current session, or `None` while the link is down.
    pub(crate) fn session(&self) -> Option<u32> {
        self.with(|s| s.up().then_some(s.session))
    }

    /// True while the link is still up in `session`; then also registers the waker of `side`,
    /// so that the next change of the link wakes the caller.
    pub(crate) fn still_in(&self, session: u32, side: Side, cx: &mut Context<'_>) -> bool {
        self.with(|s| {
            let alive = s.up() && s.session == session;
            if alive {
                match side {
                    Side::Tx => s.tx.register(cx.waker()),
                    Side::Rx => s.rx.register(cx.waker()),
                }
            }
            alive
        })
    }

    pub(crate) fn poll_state(&self, want_up: bool, cx: &mut Context<'_>) -> Poll<()> {
        self.with(|s| {
            if s.up() == want_up {
                Poll::Ready(())
            } else {
                s.link.register(cx.waker());
                Poll::Pending
            }
        })
    }

    pub(crate) fn count_zlp(&self) {
        self.with(|s| s.stats.zlp_skipped = s.stats.zlp_skipped.saturating_add(1));
    }

    pub(crate) fn stats(&self) -> Stats {
        self.with(|s| s.stats)
    }
}

/// The handler registered with the `embassy_usb::Builder`. It is the only writer of the link state.
pub(crate) struct LinkHandler<'d> {
    pub(crate) shared: &'d Shared,
}

impl Handler for LinkHandler<'_> {
    fn enabled(&mut self, _enabled: bool) {
        self.shared.change(|s| {
            s.configured = false;
            s.suspended = false;
        });
    }

    fn reset(&mut self) {
        self.shared.change(|s| {
            s.stats.resets = s.stats.resets.saturating_add(1);
            s.configured = false;
            // The device clears its own suspended flag on reset without calling `suspended(false)`.
            s.suspended = false;
        });
    }

    fn configured(&mut self, configured: bool) {
        self.shared.change(|s| s.configured = configured);
    }

    fn suspended(&mut self, suspended: bool) {
        self.shared.change(|s| {
            if suspended && !s.suspended {
                s.stats.suspends = s.stats.suspends.saturating_add(1);
            }
            s.suspended = suspended;
        });
    }
}
