//! The one thing every on-target test shares: reporting the verdict to the host.
#![no_std]

/// Logs the verdict and ends the `probe-rs run` session through semihosting SYS_EXIT.
/// The host exit status is 0 when `failures == 0`, otherwise 1.
pub fn exit(failures: u32) -> ! {
    if failures == 0 {
        defmt::info!("RESULT: PASS");
    } else {
        defmt::error!("RESULT: FAIL ({} failed checks)", failures);
    }
    semihosting::process::exit(if failures == 0 { 0 } else { 1 })
}

/// The tiny line protocol the computer uses to drive the USB test programs. Bytes only: no
/// device program knows what the bytes mean beyond these commands.
pub mod proto {
    use core::fmt;

    /// Everything the device receives before this is discarded and counted.
    pub const MARKER: &[u8; 6] = b"START\n";

    /// The known pattern: byte `i` of a stream. 251 is prime, so a lost or repeated byte shows.
    pub const fn pattern(i: u32) -> u8 {
        (i % 251) as u8
    }

    /// One command line from the computer.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum Cmd {
        /// Reply `PONG`.
        Ping,
        /// Reply with the link state.
        State,
        /// Echo the next N bytes, then reply `DONE <n>`.
        Echo(u32),
        /// Send N bytes of the pattern as fast as possible, then reply `SENT <n> <us>`.
        Send(u32),
        /// Send exactly N bytes of the pattern and then nothing.
        Burst(u32),
        /// Count and check the next N bytes of the pattern, then reply `RECVD <n> <errors> <us>`.
        Recv(u32),
        /// Send one byte (0xA5).
        Nudge,
        /// Reply `BYE ...` and finish.
        Finish,
        /// Anything else.
        Unknown,
    }

    /// Parses a line without its newline.
    pub fn parse(line: &[u8]) -> Cmd {
        let Ok(text) = core::str::from_utf8(line) else {
            return Cmd::Unknown;
        };
        let mut words = text.split(' ');
        let name = words.next().unwrap_or("");
        let arg = words.next().and_then(|w| w.parse::<u32>().ok());
        match (name, arg) {
            ("PING", None) => Cmd::Ping,
            ("STATE", None) => Cmd::State,
            ("NUDGE", None) => Cmd::Nudge,
            ("FINISH", None) => Cmd::Finish,
            ("ECHO", Some(n)) => Cmd::Echo(n),
            ("SEND", Some(n)) => Cmd::Send(n),
            ("BURST", Some(n)) => Cmd::Burst(n),
            ("RECV", Some(n)) => Cmd::Recv(n),
            _ => Cmd::Unknown,
        }
    }

    /// Finds [`MARKER`] in a byte stream, one byte at a time.
    pub struct Marker {
        window: [u8; 6],
    }

    impl Marker {
        /// Nothing seen yet.
        pub const fn new() -> Self {
            Self { window: [0; 6] }
        }

        /// True when `b` completes the marker.
        pub fn push(&mut self, b: u8) -> bool {
            self.window.rotate_left(1);
            self.window[5] = b;
            self.window == *MARKER
        }
    }

    impl Default for Marker {
        fn default() -> Self {
            Self::new()
        }
    }

    /// A reply line under construction (at most 96 bytes, the rest is cut off).
    pub struct Out {
        buf: [u8; 96],
        len: usize,
    }

    impl Out {
        /// Empty.
        pub const fn new() -> Self {
            Self {
                buf: [0; 96],
                len: 0,
            }
        }

        /// The bytes so far.
        pub fn bytes(&self) -> &[u8] {
            &self.buf[..self.len]
        }
    }

    impl Default for Out {
        fn default() -> Self {
            Self::new()
        }
    }

    impl fmt::Write for Out {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let n = s.len().min(self.buf.len() - self.len);
            self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
            self.len += n;
            Ok(())
        }
    }
}

/// Numbered 63-byte records: a sequence number, a body that depends on it and a checksum, so that
/// a torn, repeated or reordered record shows on the computer. The host program has the same code.
pub mod rec {
    /// One record is exactly one IN packet of the crate.
    pub const LEN: usize = 63;
    /// Report frames of the link program start with these four bytes, which no sequence number
    /// below 1.1 billion has.
    pub const REPORT_TAG: &[u8; 4] = b"REPO";

    fn sum(data: &[u8]) -> u32 {
        let mut h: u32 = 0x811c_9dc5;
        for b in data {
            h ^= u32::from(*b);
            h = h.wrapping_mul(0x0100_0193);
        }
        h
    }

    /// Builds record `seq`.
    pub fn fill(seq: u32, out: &mut [u8; LEN]) {
        out[..4].copy_from_slice(&seq.to_le_bytes());
        for (i, b) in out[4..59].iter_mut().enumerate() {
            *b = (seq as u8)
                .wrapping_mul(31)
                .wrapping_add(i as u8 ^ (seq >> 8) as u8);
        }
        let c = sum(&out[..59]);
        out[59..].copy_from_slice(&c.to_le_bytes());
    }
}

/// What the device's `embassy_usb::Handler` calls are, with the time of each: a second handler
/// next to the crate's own, to see what the USB controller reports. It only stores; nothing is
/// logged from the callbacks.
pub mod spy {
    use core::cell::RefCell;
    use core::fmt::Write as _;
    use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

    use critical_section::Mutex;
    use embassy_usb::Handler;

    use crate::proto::Out;

    const CAP: usize = 24;

    struct Log {
        n: usize,
        t: [u32; CAP],
        c: [u8; CAP],
    }

    static LOG: Mutex<RefCell<Log>> = Mutex::new(RefCell::new(Log {
        n: 0,
        t: [0; CAP],
        c: [0; CAP],
    }));
    /// `configured(true)` calls since power-up.
    pub static CONFIGURED: AtomicU32 = AtomicU32::new(0);
    /// Calls that did not fit the log since the last `clear`.
    pub static DROPPED: AtomicU32 = AtomicU32::new(0);

    /// Microseconds of the embassy timer, wrapping after 71 minutes.
    pub fn now_us() -> u32 {
        embassy_time::Instant::now().as_micros() as u32
    }

    fn log(code: u8) {
        let t = now_us();
        critical_section::with(|cs| {
            let mut l = LOG.borrow_ref_mut(cs);
            let i = l.n;
            if i < CAP {
                l.t[i] = t;
                l.c[i] = code;
                l.n += 1;
            } else {
                DROPPED.fetch_add(1, Relaxed);
            }
        });
    }

    /// Forgets the log.
    pub fn clear() {
        critical_section::with(|cs| LOG.borrow_ref_mut(cs).n = 0);
        DROPPED.store(0, Relaxed);
    }

    /// Entries logged since the last `clear`.
    pub fn len() -> usize {
        critical_section::with(|cs| LOG.borrow_ref(cs).n)
    }

    /// Appends up to `max` entries from `from` on to `out` as `<code>@<microseconds after base>`
    /// separated by commas; returns the index after the last one written.
    /// Codes: E enabled(true), e enabled(false), R reset, A addressed, C configured(true),
    /// c configured(false), S suspended(true), U suspended(false).
    pub fn write(out: &mut Out, base: u32, from: usize, max: usize) -> usize {
        let n = len();
        let mut i = from;
        while i < n && i < from + max {
            let (c, t) = critical_section::with(|cs| {
                let l = LOG.borrow_ref(cs);
                (l.c[i], l.t[i])
            });
            let sep = if i == from { "" } else { "," };
            let _ = write!(out, "{}{}@{}", sep, c as char, t.wrapping_sub(base) as i32);
            i += 1;
        }
        i
    }

    /// The second handler. Register it with `Builder::handler` before the crate's.
    pub struct Spy;

    impl Handler for Spy {
        fn enabled(&mut self, enabled: bool) {
            log(if enabled { b'E' } else { b'e' });
        }
        fn reset(&mut self) {
            log(b'R');
        }
        fn addressed(&mut self, _addr: u8) {
            log(b'A');
        }
        fn configured(&mut self, configured: bool) {
            if configured {
                CONFIGURED.fetch_add(1, Relaxed);
            }
            log(if configured { b'C' } else { b'c' });
        }
        fn suspended(&mut self, suspended: bool) {
            log(if suspended { b'S' } else { b'U' });
        }
    }
}
