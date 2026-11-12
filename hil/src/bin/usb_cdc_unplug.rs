//! Check F, for a person: the physical unplug. Not run by the agent that wrote it. The board's own USB
//! cable is pulled while this program writes numbered 63-byte records (`GO`), and put back. The
//! program measures the time from the last successful write to the `NotConnected` of the pending
//! one, notes every `Handler` call the controller makes with its time, and serves an echo in the
//! new session. `usb-cdc-host unplug` (`tools/usb_smoke.sh unplug`) prints the banners. The status
//! LED is not used (it needs the ws2812c crate): the banner on the computer and the log lines tell
//! when. Commands: `START`, `PING`, `GO`, `UNP`, `ECHO n`, `FINISH`.
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
use hil_common::proto::{self, Cmd, Marker, Out};
use hil_common::{rec, spy};
use panic_probe as _;
use static_cell::StaticCell;
use usb_cdc::{Link, Rx, Tx, UsbCdc};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;

struct Dev {
    tx: Tx<'static, UsbDriver>,
    rx: Rx<'static, UsbDriver>,
    link: &'static Link<'static>,
    buf: [u8; 64],
    pos: usize,
    len: usize,
    line: [u8; 64],
    pre: u32,
    /// The records run: records written, microseconds from the last one to the error (-1 none),
    /// the time base of the event log (the last successful write), events seen.
    records: u32,
    gap_us: i32,
    went_down: bool,
    base: u32,
}

impl Dev {
    async fn say(&mut self, args: fmt::Arguments<'_>) -> Result<(), ()> {
        let mut out = Out::new();
        let _ = out.write_fmt(args);
        let _ = out.write_str("\n");
        self.tx.write_all(out.bytes()).await.map_err(|_| ())
    }

    async fn fill(&mut self) -> Result<(), ()> {
        if self.pos < self.len {
            return Ok(());
        }
        let n = self.rx.read(&mut self.buf).await.map_err(|_| ())?;
        self.pos = 0;
        self.len = n;
        Ok(())
    }

    async fn await_marker(&mut self) -> Result<(), ()> {
        let mut marker = Marker::new();
        let mut seen = 0u32;
        loop {
            self.fill().await?;
            while self.pos < self.len {
                let b = self.buf[self.pos];
                self.pos += 1;
                seen += 1;
                if marker.push(b) {
                    self.pre = seen - proto::MARKER.len() as u32;
                    return Ok(());
                }
            }
        }
    }

    async fn read_line(&mut self) -> Result<usize, ()> {
        let mut n = 0;
        loop {
            self.fill().await?;
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

    async fn echo(&mut self, n: u32) -> Result<(), ()> {
        let mut done = 0u32;
        while done < n {
            self.fill().await?;
            let k = (self.len - self.pos).min((n - done) as usize);
            let (a, b) = (self.pos, self.pos + k);
            self.pos = b;
            self.tx.write_all(&self.buf[a..b]).await.map_err(|_| ())?;
            done += k as u32;
        }
        self.say(format_args!("DONE {}", done)).await
    }

    /// Writes numbered records until a write fails; the failure is the measurement.
    async fn go(&mut self) -> Result<(), ()> {
        spy::clear();
        self.records = 0;
        let mut last_ok = Instant::now();
        let mut seq = 0u32;
        defmt::info!("[INFO] unplug: records running, pull the cable now");
        loop {
            let mut r = [0u8; rec::LEN];
            rec::fill(seq, &mut r);
            match self.tx.write_all(&r).await {
                Ok(()) => {
                    last_ok = Instant::now();
                    self.base = spy::now_us();
                    seq += 1;
                    self.records += 1;
                }
                Err(_) => {
                    self.gap_us = (Instant::now() - last_ok).as_micros() as i32;
                    self.went_down = true;
                    return Err(());
                }
            }
        }
    }

    async fn unp(&mut self) -> Result<(), ()> {
        let st = self.link.stats();
        let (down, records, gap) = (u8::from(self.went_down), self.records, self.gap_us);
        let (up, dtr) = (u8::from(self.link.is_up()), u8::from(self.link.dtr()));
        self.say(format_args!(
            "UNP went_down={} records={} last_ok_to_err_us={} ups={} downs={} rst={} susp={} up={} dtr={}",
            down, records, gap, st.ups, st.downs, st.resets, st.suspends, up, dtr
        ))
        .await?;
        let mut i = 0;
        while i < spy::len() {
            let mut out = Out::new();
            let _ = out.write_str("UEV ");
            i = spy::write(&mut out, self.base, i, 7);
            let _ = out.write_str("\n");
            self.tx.write_all(out.bytes()).await.map_err(|_| ())?;
        }
        Ok(())
    }

    async fn session(&mut self) -> Result<bool, ()> {
        self.pos = 0;
        self.len = 0;
        self.await_marker().await?;
        let pre = self.pre;
        self.say(format_args!("READY pre={}", pre)).await?;
        loop {
            let n = self.read_line().await?;
            let line = self.line;
            match core::str::from_utf8(&line[..n]).unwrap_or("") {
                "START" => self.say(format_args!("READY pre=0")).await?,
                "GO" => {
                    if self.go().await.is_err() {
                        return Err(());
                    }
                }
                "UNP" => self.unp().await?,
                "FINISH" => {
                    self.say(format_args!("BYE")).await?;
                    Timer::after_millis(100).await;
                    return Ok(true);
                }
                _ => match proto::parse(&line[..n]) {
                    Cmd::Ping => self.say(format_args!("PONG")).await?,
                    Cmd::Echo(k) => self.echo(k).await?,
                    _ => self.say(format_args!("ERR")).await?,
                },
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
    defmt::info!("[INFO] usb_cdc_unplug: device start");
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
    static SPY: StaticCell<spy::Spy> = StaticCell::new();
    static LINK: StaticCell<Link<'static>> = StaticCell::new();

    let driver = Driver::new(p.USB, Irqs);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    builder.handler(SPY.init(spy::Spy));
    let cdc = UsbCdc::new(
        &mut builder,
        STATE.init(usb_cdc::State::new()),
        usb_cdc::Config::new(),
    );
    let (tx, rx, link) = cdc.split();
    let link: &'static Link<'static> = LINK.init(link);
    let device = builder.build();
    spawner.spawn(usb_task(device).unwrap());

    let mut dev = Dev {
        tx,
        rx,
        link,
        buf: [0; 64],
        pos: 0,
        len: 0,
        line: [0; 64],
        pre: 0,
        records: 0,
        gap_us: -1,
        went_down: false,
        base: 0,
    };
    let mut finished = false;
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(600) {
        if with_timeout(Duration::from_secs(600), dev.link.wait_up())
            .await
            .is_err()
        {
            break;
        }
        if let Ok(true) = dev.session().await {
            finished = true;
            break;
        }
        Timer::after_millis(1).await;
    }
    let st = link.stats();
    defmt::info!(
        "[INFO] unplug: went down {=bool}, {=u32} records, last write to error {=i32} us, ups {=u32} downs {=u32} resets {=u32} suspends {=u32}",
        dev.went_down,
        dev.records,
        dev.gap_us,
        st.ups,
        st.downs,
        st.resets,
        st.suspends
    );
    let mut failures = 0u32;
    if finished && dev.went_down && st.downs >= 1 && st.ups >= 2 {
        defmt::info!("[PASS] the pending write ended with NotConnected and the link came back");
    } else {
        failures += 1;
        defmt::error!(
            "[FAIL] finished {=bool}, went down {=bool}",
            finished,
            dev.went_down
        );
    }
    embassy_rp::pac::USB
        .sie_ctrl()
        .modify(|w| w.set_pullup_en(false));
    hil_common::exit(failures)
}
