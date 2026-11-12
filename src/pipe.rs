//! The byte pipe: `Tx` and `Rx` on the two bulk endpoints of the CDC data interface.

use core::future::poll_fn;
use core::pin::pin;
use core::task::Poll;

use embassy_usb::class::cdc_acm::{Receiver, Sender};
use embassy_usb::driver::Driver;

use crate::shared::{Shared, Side};

/// The only way a call on the pipe fails.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The link is down (not configured, or suspended, or reset, or the device is disabled), or it
    /// went down while the call was waiting.
    NotConnected,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotConnected => f.write_str("USB link is not connected"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io_async::Error for Error {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self {
            Self::NotConnected => embedded_io_async::ErrorKind::NotConnected,
        }
    }
}

/// The write half. `embedded_io_async::Write`: every `write` sends one packet of at most
/// `max_packet_size - 1` bytes and returns how many bytes it took.
pub struct Tx<'d, D: Driver<'d>> {
    pub(crate) ep: Sender<'d, D>,
    pub(crate) shared: &'d Shared,
    /// `max_packet_size - 1`.
    pub(crate) chunk: usize,
}

/// The read half. `embedded_io_async::Read`: `read` returns at least one byte or an error.
pub struct Rx<'d, D: Driver<'d>> {
    pub(crate) ep: Receiver<'d, D>,
    pub(crate) shared: &'d Shared,
    pub(crate) packet: usize,
    pub(crate) buf: [u8; 64],
    /// The part of `buf` not yet handed to the caller is `pos..len`.
    pub(crate) pos: usize,
    pub(crate) len: usize,
    /// The link session the held packet arrived in.
    pub(crate) held_in: u32,
}

impl<'d, D: Driver<'d>> embedded_io_async::ErrorType for Tx<'d, D> {
    type Error = Error;
}

impl<'d, D: Driver<'d>> embedded_io_async::Write for Tx<'d, D> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        let Some(session) = self.shared.session() else {
            return Err(Error::NotConnected);
        };
        if buf.is_empty() {
            return Ok(0);
        }
        let n = buf.len().min(self.chunk);
        let (data, _) = buf.split_at(n);
        let shared = self.shared;
        let mut send = pin!(self.ep.write_packet(data));
        poll_fn(|cx| {
            if !shared.still_in(session, Side::Tx, cx) {
                return Poll::Ready(Err(Error::NotConnected));
            }
            match send.as_mut().poll(cx) {
                Poll::Pending => Poll::Pending,
                Poll::Ready(Ok(())) => Poll::Ready(Ok(n)),
                // bound: `Disabled` is the endpoint saying it is down; `BufferOverflow` cannot
                // happen because `data` is at most max packet size - 1 bytes.
                Poll::Ready(Err(_)) => Poll::Ready(Err(Error::NotConnected)),
            }
        })
        .await
    }

    async fn flush(&mut self) -> Result<(), Error> {
        // Nothing is buffered: every `write` handed its packet to the endpoint.
        if self.shared.is_up() {
            Ok(())
        } else {
            Err(Error::NotConnected)
        }
    }
}

impl<'d, D: Driver<'d>> Rx<'d, D> {
    fn discard(&mut self) {
        self.pos = 0;
        self.len = 0;
    }

    /// Moves as much of the held packet as fits into `out`.
    fn serve(&mut self, out: &mut [u8]) -> usize {
        let held = self.buf.get(self.pos..self.len).unwrap_or_default();
        let n = held.len().min(out.len());
        let (src, _) = held.split_at(n);
        let (dst, _) = out.split_at_mut(n);
        dst.copy_from_slice(src);
        self.pos += n;
        n
    }
}

impl<'d, D: Driver<'d>> embedded_io_async::ErrorType for Rx<'d, D> {
    type Error = Error;
}

impl<'d, D: Driver<'d>> embedded_io_async::Read for Rx<'d, D> {
    async fn read(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        let Some(session) = self.shared.session() else {
            self.discard();
            return Err(Error::NotConnected);
        };
        if self.held_in != session {
            self.discard();
        }
        if out.is_empty() {
            return Ok(0);
        }
        let shared = self.shared;
        loop {
            if self.pos < self.len {
                return Ok(self.serve(out));
            }
            // Nothing is held: wait for the next packet. The packet becomes ours in the poll in
            // which the endpoint completes, with no await after it, so a dropped future either
            // has the packet in `buf` with `len` set, or never took it from the endpoint.
            let (dst, _) = self.buf.split_at_mut(self.packet);
            let mut recv = pin!(self.ep.read_packet(dst));
            let got = poll_fn(|cx| {
                if !shared.still_in(session, Side::Rx, cx) {
                    return Poll::Ready(Err(Error::NotConnected));
                }
                match recv.as_mut().poll(cx) {
                    Poll::Pending => Poll::Pending,
                    Poll::Ready(Ok(n)) => Poll::Ready(Ok(n)),
                    // bound: `Disabled` is the endpoint saying it is down; `BufferOverflow` cannot
                    // happen because `dst` is the full max packet size.
                    Poll::Ready(Err(_)) => Poll::Ready(Err(Error::NotConnected)),
                }
            })
            .await?;
            self.held_in = session;
            self.pos = 0;
            self.len = got.min(self.packet);
            if got == 0 {
                shared.count_zlp();
            }
        }
    }
}
