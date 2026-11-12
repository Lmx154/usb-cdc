//! On-board proof of the usb-cdc crate: the board enumerates as a CDC-ACM serial port (VID 0xc0de,
//! PID 0xcafe) and serves the line protocol of `hil_common::proto` on the crate's `Tx` and `Rx`.
//! The host program in `host/` drives it (`tools/usb_smoke.sh`). Nothing is logged while a command
//! runs: counters are kept and printed after the computer says `FINISH`.
#![no_std]
#![no_main]

use core::fmt::{self, Write as _};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::{Builder, Config, UsbDevice};
use embedded_io_async::{Read, Write};
use hil_common::proto::{self, Cmd, Marker, Out, pattern};
use panic_probe as _;
use static_cell::StaticCell;
use usb_cdc::{Link, Rx, Stats, Tx, UsbCdc};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;

/// How long the link may take to come up, and how long the computer may take to send the marker.
const LINK_UP_LIMIT: Duration = Duration::from_secs(10);
const MARKER_LIMIT: Duration = Duration::from_secs(30);
/// Longest silence between two commands, and between two packets inside a data command.
const IDLE: Duration = Duration::from_secs(15);
const DATA: Duration = Duration::from_secs(3);

macro_rules! check {
    ($failures:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
    };
}

struct Dev {
    tx: Tx<'static, UsbDriver>,
    rx: Rx<'static, UsbDriver>,
    link: Link<'static>,
    /// The packet being served; `pos..len` is not yet used.
    buf: [u8; 64],
    pos: usize,
    len: usize,
    line: [u8; 64],
    /// Bytes the pipe gave us (all of them) and bytes we gave it.
    rx_total: u32,
    tx_total: u32,
    /// Bytes before the start marker (the marker itself not counted), and the first of them.
    pre_total: u32,
    pre_first: [u8; 32],
    pre_first_len: usize,
    marker_seen: bool,
    pattern_errors: u32,
    /// Commands served, unknown commands, failures inside a command.
    commands: u32,
    unknown: u32,
    errors: u32,
    last_error: &'static str,
    progress: u32,
    up_ms: u32,
    up_at: Instant,
    dtr_at_up: bool,
    /// Milliseconds after the link came up when DTR was first seen true, or -1.
    dtr_first_ms: i32,
    dtr_at_marker: bool,
    dtr_at_state: bool,
    stats_at_up: Stats,
    stats_at_bye: Option<Stats>,
    up_at_bye: bool,
    up_at_end: bool,
}

impl Dev {
    async fn send(&mut self, data: &[u8]) -> Result<(), &'static str> {
        self.tx.write_all(data).await.map_err(|_| "tx: link down")?;
        self.tx_total += data.len() as u32;
        Ok(())
    }

    async fn say(&mut self, args: fmt::Arguments<'_>) -> Result<(), &'static str> {
        let mut out = Out::new();
        let _ = out.write_fmt(args);
        let _ = out.write_str("\n");
        self.send(out.bytes()).await
    }

    /// Makes sure `pos < len`: reads one packet if nothing is left.
    async fn fill(&mut self, timeout: Duration) -> Result<(), &'static str> {
        if self.pos < self.len {
            return Ok(());
        }
        match with_timeout(timeout, self.rx.read(&mut self.buf)).await {
            Ok(Ok(n)) => {
                self.pos = 0;
                self.len = n;
                self.rx_total += n as u32;
                Ok(())
            }
            Ok(Err(_)) => Err("rx: link down"),
            Err(_) => Err("rx: timeout"),
        }
    }

    /// Discards everything up to and including the start marker, counting it.
    async fn await_marker(&mut self) -> Result<(), &'static str> {
        let deadline = Instant::now() + MARKER_LIMIT;
        let mut marker = Marker::new();
        let mut seen: u32 = 0;
        loop {
            if Instant::now() > deadline {
                return Err("no start marker");
            }
            if self.dtr_first_ms < 0 && self.link.dtr() {
                self.dtr_first_ms = (Instant::now() - self.up_at).as_millis() as i32;
            }
            if self.pos >= self.len {
                match with_timeout(Duration::from_millis(2), self.rx.read(&mut self.buf)).await {
                    Err(_) => continue,
                    Ok(Err(_)) => {
                        if with_timeout(DATA, self.link.wait_up()).await.is_err() {
                            return Err("link down before the marker");
                        }
                        continue;
                    }
                    Ok(Ok(n)) => {
                        self.pos = 0;
                        self.len = n;
                        self.rx_total += n as u32;
                    }
                }
            }
            while self.pos < self.len {
                let b = self.buf[self.pos];
                self.pos += 1;
                seen += 1;
                if self.pre_first_len < self.pre_first.len() {
                    self.pre_first[self.pre_first_len] = b;
                    self.pre_first_len += 1;
                }
                if marker.push(b) {
                    self.pre_total = seen - proto::MARKER.len() as u32;
                    self.pre_first_len = self.pre_first_len.min(self.pre_total as usize);
                    self.marker_seen = true;
                    self.dtr_at_marker = self.link.dtr();
                    if self.dtr_first_ms < 0 && self.dtr_at_marker {
                        self.dtr_first_ms = (Instant::now() - self.up_at).as_millis() as i32;
                    }
                    return Ok(());
                }
            }
        }
    }

    /// Reads one command line (without the newline) into `self.line`.
    async fn read_line(&mut self, timeout: Duration) -> Result<usize, &'static str> {
        let mut n = 0;
        loop {
            self.fill(timeout).await?;
            let b = self.buf[self.pos];
            self.pos += 1;
            if b == b'\n' {
                return Ok(n);
            }
            if b != b'\r' && n < self.line.len() {
                self.line[n] = b;
                n += 1;
            }
        }
    }

    /// Takes up to `max` bytes of the current packet as `buf[a..b]`.
    async fn chunk(&mut self, max: u32) -> Result<(usize, usize), &'static str> {
        self.fill(DATA).await?;
        let take = (self.len - self.pos).min(max as usize);
        let a = self.pos;
        self.pos += take;
        Ok((a, a + take))
    }

    async fn echo(&mut self, n: u32) -> Result<(), &'static str> {
        self.progress = 0;
        while self.progress < n {
            let (a, b) = self.chunk(n - self.progress).await?;
            self.tx
                .write_all(&self.buf[a..b])
                .await
                .map_err(|_| "tx: link down")?;
            self.tx_total += (b - a) as u32;
            self.progress += (b - a) as u32;
        }
        Ok(())
    }

    /// Sends `n` bytes of the pattern as fast as the pipe takes them; returns the microseconds.
    async fn blast(&mut self, n: u32) -> Result<u32, &'static str> {
        let mut block = [0u8; 504];
        let t0 = Instant::now();
        let mut i = 0u32;
        while i < n {
            let k = (n - i).min(block.len() as u32) as usize;
            for (j, slot) in block[..k].iter_mut().enumerate() {
                *slot = pattern(i + j as u32);
            }
            self.send(&block[..k]).await?;
            i += k as u32;
        }
        Ok((Instant::now() - t0).as_micros() as u32)
    }

    /// Counts and checks `n` bytes of the pattern; returns the microseconds from the first byte.
    async fn sink(&mut self, n: u32) -> Result<u32, &'static str> {
        self.progress = 0;
        let mut first = None;
        while self.progress < n {
            let (a, b) = self.chunk(n - self.progress).await?;
            if first.is_none() {
                first = Some(Instant::now());
            }
            for k in a..b {
                if self.buf[k] != pattern(self.progress) {
                    self.pattern_errors += 1;
                }
                self.progress += 1;
            }
        }
        Ok(first.map_or(0, |t| (Instant::now() - t).as_micros() as u32))
    }

    fn fail(&mut self, why: &'static str) {
        self.errors += 1;
        self.last_error = why;
    }

    /// Serves commands until `FINISH`.
    async fn serve(&mut self) -> Result<(), &'static str> {
        self.await_marker().await?;
        let (pre, dtr_up, dtr_first, up_ms) = (
            self.pre_total,
            u8::from(self.dtr_at_up),
            self.dtr_first_ms,
            self.up_ms,
        );
        self.say(format_args!(
            "READY pre={} dtr_up={} dtr_first_ms={} link_up_ms={}",
            pre, dtr_up, dtr_first, up_ms
        ))
        .await?;
        loop {
            let n = self.read_line(IDLE).await?;
            let cmd = proto::parse(&self.line[..n]);
            self.commands += 1;
            match cmd {
                Cmd::Ping => self.say(format_args!("PONG")).await?,
                Cmd::State => {
                    self.dtr_at_state = self.link.dtr();
                    self.say(format_args!(
                        "STATE up={} dtr={}",
                        u8::from(self.link.is_up()),
                        u8::from(self.dtr_at_state)
                    ))
                    .await?
                }
                Cmd::Echo(count) => {
                    if let Err(e) = self.echo(count).await {
                        self.fail(e);
                    }
                    let done = self.progress;
                    self.say(format_args!("DONE {}", done)).await?;
                }
                Cmd::Send(count) => match self.blast(count).await {
                    Ok(us) => self.say(format_args!("SENT {} {}", count, us)).await?,
                    Err(e) => {
                        self.fail(e);
                        return Err(e);
                    }
                },
                Cmd::Burst(count) => {
                    if let Err(e) = self.blast(count).await {
                        self.fail(e);
                        return Err(e);
                    }
                }
                Cmd::Recv(count) => {
                    let errors_before = self.pattern_errors;
                    match self.sink(count).await {
                        Ok(us) => {
                            let errors = self.pattern_errors - errors_before;
                            self.say(format_args!("RECVD {} {} {}", count, errors, us))
                                .await?
                        }
                        Err(e) => {
                            self.fail(e);
                            let got = self.progress;
                            self.say(format_args!("RECVD {} -1 0", got)).await?;
                        }
                    }
                }
                Cmd::Nudge => self.send(&[0xA5]).await?,
                Cmd::Finish => {
                    let stats = self.link.stats();
                    self.stats_at_bye = Some(stats);
                    self.up_at_bye = self.link.is_up();
                    let (rx_total, tx_total) = (self.rx_total, self.tx_total);
                    self.say(format_args!(
                        "BYE ups={} downs={} resets={} suspends={} zlp={} in={} out={}",
                        stats.ups,
                        stats.downs,
                        stats.resets,
                        stats.suspends,
                        stats.zlp_skipped,
                        rx_total,
                        tx_total
                    ))
                    .await?;
                    Timer::after_millis(100).await;
                    self.up_at_end = self.link.is_up();
                    return Ok(());
                }
                Cmd::Unknown => {
                    self.unknown += 1;
                    self.say(format_args!("ERR")).await?;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("[INFO] usb_cdc_smoke: device start");
    let p = embassy_rp::init(Default::default());

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("MARV");
    config.product = Some("usb-cdc test");
    config.serial_number = Some("usb-cdc-hil");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<usb_cdc::State<'static>> = StaticCell::new();

    let driver = Driver::new(p.USB, Irqs);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let cdc = UsbCdc::new(
        &mut builder,
        STATE.init(usb_cdc::State::new()),
        usb_cdc::Config::new(),
    );
    let (tx, rx, link) = cdc.split();
    let device = builder.build();

    let t0 = Instant::now();
    spawner.spawn(usb_task(device).unwrap());

    let up = with_timeout(LINK_UP_LIMIT, link.wait_up()).await.is_ok();
    let up_ms = (Instant::now() - t0).as_millis() as u32;
    let dtr_at_up = link.dtr();
    let stats_at_up = link.stats();

    let mut dev = Dev {
        tx,
        rx,
        link,
        buf: [0; 64],
        pos: 0,
        len: 0,
        line: [0; 64],
        rx_total: 0,
        tx_total: 0,
        pre_total: 0,
        pre_first: [0; 32],
        pre_first_len: 0,
        marker_seen: false,
        pattern_errors: 0,
        commands: 0,
        unknown: 0,
        errors: 0,
        last_error: "",
        progress: 0,
        up_ms,
        up_at: Instant::now(),
        dtr_at_up,
        dtr_first_ms: -1,
        dtr_at_marker: false,
        dtr_at_state: false,
        stats_at_up,
        stats_at_bye: None,
        up_at_bye: false,
        up_at_end: false,
    };
    let served = if up {
        dev.serve().await
    } else {
        Err("link did not come up")
    };

    let mut failures = 0u32;
    check!(
        failures,
        up && up_ms <= 5000,
        "{=str} enumeration: link up {=u32} ms after the pull-up (limit 5000)",
        up_ms
    );
    defmt::info!(
        "[INFO] dtr: at link up {=bool}, first true {=i32} ms after (-1 never), at marker {=bool}, at STATE {=bool}",
        dev.dtr_at_up,
        dev.dtr_first_ms,
        dev.dtr_at_marker,
        dev.dtr_at_state
    );
    defmt::info!(
        "[INFO] at link up: ups {=u32} downs {=u32} resets {=u32} suspends {=u32}",
        dev.stats_at_up.ups,
        dev.stats_at_up.downs,
        dev.stats_at_up.resets,
        dev.stats_at_up.suspends
    );
    check!(
        failures,
        dev.marker_seen,
        "{=str} start marker received after {=u32} bytes that were discarded",
        dev.pre_total
    );
    defmt::info!(
        "[INFO] before the marker: {=u32} byte(s), first {=usize}: {=[u8]:x}",
        dev.pre_total,
        dev.pre_first_len,
        &dev.pre_first[..dev.pre_first_len]
    );
    match served {
        Ok(()) => check!(
            failures,
            dev.errors == 0 && dev.unknown == 0,
            "{=str} {=u32} commands served, {=u32} failed inside ({=str}), {=u32} unknown",
            dev.commands,
            dev.errors,
            dev.last_error,
            dev.unknown
        ),
        Err(why) => check!(
            failures,
            false,
            "{=str} serving stopped after {=u32} commands: {=str}",
            dev.commands,
            why
        ),
    }
    check!(
        failures,
        dev.pattern_errors == 0,
        "{=str} bytes from the computer that broke the pattern: {=u32}",
        dev.pattern_errors
    );
    if let Some(stats) = dev.stats_at_bye {
        check!(
            failures,
            stats.ups == 1 && stats.downs == 0 && dev.up_at_bye && dev.up_at_end,
            "{=str} link before the detach: ups {=u32} (want 1), downs {=u32} (want 0), up {=bool} / {=bool}",
            stats.ups,
            stats.downs,
            dev.up_at_bye,
            dev.up_at_end
        );
        defmt::info!(
            "[STAT] link stats at FINISH: ups {=u32} downs {=u32} resets {=u32} suspends {=u32} zlp_skipped {=u32}",
            stats.ups,
            stats.downs,
            stats.resets,
            stats.suspends,
            stats.zlp_skipped
        );
    } else {
        check!(
            failures,
            false,
            "{=str} no FINISH seen, so no link stats before the detach: {}",
            dev.link.stats()
        );
    }
    defmt::info!(
        "[STAT] bytes from the pipe {=u32}, into the pipe {=u32}",
        dev.rx_total,
        dev.tx_total
    );

    // The driver has no Drop: release the D+ pull-up so the computer sees a clean disconnect.
    embassy_rp::pac::USB
        .sie_ctrl()
        .modify(|w| w.set_pullup_en(false));
    hil_common::exit(failures)
}
