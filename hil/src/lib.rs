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
