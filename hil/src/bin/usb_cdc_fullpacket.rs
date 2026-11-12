//! Positive control for the 63-byte rule, and nothing else: the bare `embassy_usb` `CdcAcmClass`
//! with full 64-byte packets and no zero-length packet. `BURST n` sends exactly `n` bytes of the
//! pattern in packets of 64 (the last one shorter only when `n` is not a multiple of 64), so the
//! host program can show whether the computer holds back a transfer that ends on a full packet.
//! `NUDGE` sends one byte (a short packet) to release anything held. Same line protocol as
//! `usb_cdc_smoke` (START, PING, BURST, NUDGE, FINISH); replies are short lines.
#![no_std]
#![no_main]

use core::fmt::Write as _;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use hil_common::proto::{self, Cmd, Marker, Out, pattern};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;

const LINK_UP_LIMIT: Duration = Duration::from_secs(10);
const IDLE: Duration = Duration::from_secs(15);

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

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

struct Counters {
    rx_total: u32,
    tx_total: u32,
    pre_total: u32,
    marker_seen: bool,
    commands: u32,
    unknown: u32,
    bursts: u32,
    nudges: u32,
}

/// Serves commands until `FINISH`; `Err` says why it stopped early.
async fn serve(
    class: &mut CdcAcmClass<'static, UsbDriver>,
    c: &mut Counters,
) -> Result<(), &'static str> {
    let mut marker = Marker::new();
    let mut line = [0u8; 64];
    let mut line_len = 0usize;
    let mut packet = [0u8; 64];
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut seen = 0u32;
    loop {
        let limit = if c.marker_seen {
            IDLE
        } else {
            Duration::from_secs(2)
        };
        let n = match with_timeout(limit, class.read_packet(&mut packet)).await {
            Ok(Ok(n)) => n,
            Ok(Err(_)) => return Err("rx: link down"),
            Err(_) => {
                if c.marker_seen || Instant::now() > deadline {
                    return Err("rx: timeout");
                }
                continue;
            }
        };
        c.rx_total += n as u32;
        for &b in &packet[..n] {
            if !c.marker_seen {
                seen += 1;
                if marker.push(b) {
                    c.marker_seen = true;
                    c.pre_total = seen - proto::MARKER.len() as u32;
                    reply(class, c, b"READY\n").await?;
                }
                continue;
            }
            if b != b'\n' {
                if line_len < line.len() {
                    line[line_len] = b;
                    line_len += 1;
                }
                continue;
            }
            let cmd = proto::parse(&line[..line_len]);
            line_len = 0;
            c.commands += 1;
            match cmd {
                Cmd::Ping => reply(class, c, b"PONG\n").await?,
                Cmd::Burst(count) => {
                    let mut sent = 0u32;
                    while sent < count {
                        let k = (count - sent).min(64) as usize;
                        let mut pkt = [0u8; 64];
                        for (j, slot) in pkt[..k].iter_mut().enumerate() {
                            *slot = pattern(sent + j as u32);
                        }
                        class
                            .write_packet(&pkt[..k])
                            .await
                            .map_err(|_| "tx: link down")?;
                        c.tx_total += k as u32;
                        sent += k as u32;
                    }
                    c.bursts += 1;
                }
                Cmd::Nudge => {
                    class
                        .write_packet(&[0xA5])
                        .await
                        .map_err(|_| "tx: link down")?;
                    c.tx_total += 1;
                    c.nudges += 1;
                }
                Cmd::Finish => {
                    let mut out = Out::new();
                    let _ = write!(out, "BYE in={} out={}\n", c.rx_total, c.tx_total);
                    reply(class, c, out.bytes()).await?;
                    Timer::after_millis(100).await;
                    return Ok(());
                }
                _ => {
                    c.unknown += 1;
                    reply(class, c, b"ERR\n").await?;
                }
            }
        }
    }
}

/// One short reply line (under 64 bytes, so a short packet).
async fn reply(
    class: &mut CdcAcmClass<'static, UsbDriver>,
    c: &mut Counters,
    line: &[u8],
) -> Result<(), &'static str> {
    class
        .write_packet(line)
        .await
        .map_err(|_| "tx: link down")?;
    c.tx_total += line.len() as u32;
    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("[INFO] usb_cdc_fullpacket: device start");
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
    static STATE: StaticCell<State<'static>> = StaticCell::new();

    let driver = Driver::new(p.USB, Irqs);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let mut class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    let device = builder.build();

    let t0 = Instant::now();
    spawner.spawn(usb_task(device).unwrap());

    let up = with_timeout(LINK_UP_LIMIT, class.wait_connection())
        .await
        .is_ok();
    let up_ms = (Instant::now() - t0).as_millis() as u32;

    let mut c = Counters {
        rx_total: 0,
        tx_total: 0,
        pre_total: 0,
        marker_seen: false,
        commands: 0,
        unknown: 0,
        bursts: 0,
        nudges: 0,
    };
    let served = if up {
        serve(&mut class, &mut c).await
    } else {
        Err("the port was not enabled")
    };

    let mut failures = 0u32;
    check!(
        failures,
        up && up_ms <= 5000,
        "{=str} enumeration: endpoints enabled {=u32} ms after the pull-up (limit 5000)",
        up_ms
    );
    check!(
        failures,
        c.marker_seen,
        "{=str} start marker received after {=u32} bytes that were discarded",
        c.pre_total
    );
    match served {
        Ok(()) => check!(
            failures,
            c.unknown == 0,
            "{=str} {=u32} commands served ({=u32} bursts, {=u32} nudges), {=u32} unknown",
            c.commands,
            c.bursts,
            c.nudges,
            c.unknown
        ),
        Err(why) => check!(
            failures,
            false,
            "{=str} serving stopped after {=u32} commands: {=str}",
            c.commands,
            why
        ),
    }
    defmt::info!(
        "[STAT] bytes from the pipe {=u32}, into the pipe {=u32}",
        c.rx_total,
        c.tx_total
    );

    embassy_rp::pac::USB
        .sie_ctrl()
        .modify(|w| w.set_pullup_en(false));
    hil_common::exit(failures)
}
