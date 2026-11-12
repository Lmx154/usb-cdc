//! On-board proof of how the usb-cdc crate behaves when the computer misbehaves or goes away.
//! The host program (`usb-cdc-host link`, run by `tools/usb_smoke.sh link`) drives it.
//!
//! One executor runs: the USB task, a 1 kHz tick task (largest and smallest period, periods over
//! 1.5 ms, DTR edges), the writer task (owns `Tx`: numbered 63-byte records with a 100 ms timeout
//! each, echo data and reply lines) and the link task (this `main`: owns `Rx`, serves commands).
//! A second `embassy_usb::Handler` (`hil_common::spy`) stores every call the controller causes,
//! with its time. Nothing is logged while a command runs: figures are printed after `FINISH`.
//!
//! Commands (lines): `START` (also in the middle of a session), `PING`, `STATE`, `ECHO n`,
//! `RUNA` (phases of check A: 10 s with the port open and not read, 10 s with it closed),
//! `REPORT` (stops the writer, sends the figures of A as 63-byte frames), `CYCLE 0` (fill the
//! computer's buffers, then release the D+ pull-up with a write pending, 500 ms later enable it),
//! `CYCLE 1` (same without touching the pull-up: the computer makes the link go away),
//! `CYCLE 2` (control: same fill, the link stays up, nothing is detached),
//! `CRES` (figures of the last cycle), `GAP 1` / `GAP 0` (SysTick gap measurement), `LOCK` (cycles
//! per `Link::is_up` and `Link::stats` call), `FINISH`.
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::{self, Write as _};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering::Relaxed};

use critical_section::Mutex;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
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

const M_IDLE: u8 = 0;
const M_RECORDS: u8 = 1;
const M_FILL: u8 = 2;
static MODE: AtomicU8 = AtomicU8::new(M_IDLE);

/// Tick task: periods in microseconds since the last `tick_take`.
static TICK_MAX: AtomicU32 = AtomicU32::new(0);
static TICK_MIN: AtomicU32 = AtomicU32::new(u32::MAX);
static TICK_LATE: AtomicU32 = AtomicU32::new(0);
static TICK_N: AtomicU32 = AtomicU32::new(0);
static DTR_EDGES: AtomicU32 = AtomicU32::new(0);

/// Writer task.
static W_WRITTEN: AtomicU32 = AtomicU32::new(0);
static W_TIMEOUTS: AtomicU32 = AtomicU32::new(0);
static W_OTHER: AtomicU32 = AtomicU32::new(0);
static W_MAXBLOCK: AtomicU32 = AtomicU32::new(0);
static W_SEQ: AtomicU32 = AtomicU32::new(0);
static W_ECHO_ERR: AtomicU32 = AtomicU32::new(0);
static W_START: AtomicU32 = AtomicU32::new(0);
static W_PENDING: AtomicBool = AtomicBool::new(false);

/// The last cycle: time base, time the pending write returned, 1 Ok / 2 NotConnected / 9 never
/// pending, time the pull-up was enabled again, longest SysTick gap in cycles.
static C_BASE: AtomicU32 = AtomicU32::new(0);
static C_RET: AtomicU32 = AtomicU32::new(0);
static C_RES: AtomicU8 = AtomicU8::new(0);
static C_REEN: AtomicU32 = AtomicU32::new(0);
static C_GAP: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
struct Snap {
    tmax: u32,
    tmin: u32,
    late: u32,
    n: u32,
    up: bool,
    dtr: bool,
    wr: u32,
    to: u32,
    oth: u32,
    blk: u32,
    seq: u32,
}

const SNAP0: Snap = Snap {
    tmax: 0,
    tmin: 0,
    late: 0,
    n: 0,
    up: false,
    dtr: false,
    wr: 0,
    to: 0,
    oth: 0,
    blk: 0,
    seq: 0,
};
static SNAPS: Mutex<RefCell<[Snap; 2]>> = Mutex::new(RefCell::new([SNAP0; 2]));

enum Msg {
    Wake,
    Data(usize, [u8; 64]),
    Line(usize, [u8; 96]),
    Report,
}
static CH: Channel<CriticalSectionRawMutex, Msg, 6> = Channel::new();

const SYSTICK_PERIOD: u32 = 300;
static SYSTICK_LAST: AtomicU32 = AtomicU32::new(0);
static SYSTICK_GAP: AtomicU32 = AtomicU32::new(0);

fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

/// SysTick (the highest priority) fires every `SYSTICK_PERIOD` cycles while armed and records the
/// longest gap between two of its interrupts: interrupts masked for D cycles give a gap of at
/// least D and at most D + one period.
#[cortex_m_rt::exception]
fn SysTick() {
    let now = cycles();
    let last = SYSTICK_LAST.swap(now, Relaxed);
    if last != 0 {
        SYSTICK_GAP.fetch_max(now.wrapping_sub(last), Relaxed);
    }
}

fn gap_arm() {
    SYSTICK_LAST.store(0, Relaxed);
    SYSTICK_GAP.store(0, Relaxed);
    // SAFETY: SysTick is used by nothing else in this program.
    let mut syst = unsafe { cortex_m::Peripherals::steal() }.SYST;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload(SYSTICK_PERIOD - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

fn gap_disarm() -> u32 {
    // SAFETY: as in `gap_arm`.
    let mut syst = unsafe { cortex_m::Peripherals::steal() }.SYST;
    syst.disable_interrupt();
    syst.disable_counter();
    SYSTICK_GAP.load(Relaxed)
}

fn pullup(on: bool) {
    embassy_rp::pac::USB
        .sie_ctrl()
        .modify(|w| w.set_pullup_en(on));
}

fn tick_take() -> (u32, u32, u32, u32) {
    (
        TICK_MAX.swap(0, Relaxed),
        TICK_MIN.swap(u32::MAX, Relaxed),
        TICK_LATE.swap(0, Relaxed),
        TICK_N.swap(0, Relaxed),
    )
}

async fn set_mode(m: u8) {
    MODE.store(m, Relaxed);
    CH.send(Msg::Wake).await;
}

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::task]
async fn tick_task(link: &'static Link<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut last = Instant::now();
    let mut dtr = link.dtr();
    loop {
        ticker.next().await;
        let now = Instant::now();
        let p = (now - last).as_micros() as u32;
        last = now;
        TICK_N.fetch_add(1, Relaxed);
        TICK_MAX.fetch_max(p, Relaxed);
        TICK_MIN.fetch_min(p, Relaxed);
        if p > 1500 {
            TICK_LATE.fetch_add(1, Relaxed);
        }
        let d = link.dtr();
        if d != dtr {
            dtr = d;
            DTR_EDGES.fetch_add(1, Relaxed);
        }
    }
}

async fn frame(tx: &mut Tx<'static, UsbDriver>, out: &Out) {
    let mut f = [b' '; rec::LEN];
    let b = out.bytes();
    let n = b.len().min(rec::LEN - 1);
    f[..n].copy_from_slice(&b[..n]);
    f[rec::LEN - 1] = b'\n';
    let _ = tx.write_all(&f).await;
}

async fn report(tx: &mut Tx<'static, UsbDriver>, link: &'static Link<'static>) {
    let snaps = critical_section::with(|cs| *SNAPS.borrow_ref(cs));
    for (i, s) in snaps.iter().enumerate() {
        let mut o = Out::new();
        let _ = write!(
            o,
            "REPORT P{} tmax={} tmin={} late={} n={} up={} dtr={}",
            i + 1,
            s.tmax,
            s.tmin,
            s.late,
            s.n,
            u8::from(s.up),
            u8::from(s.dtr)
        );
        frame(tx, &o).await;
        let mut o = Out::new();
        let _ = write!(
            o,
            "REPORT W{} wr={} to={} oth={} blk={} seq={}",
            i + 1,
            s.wr,
            s.to,
            s.oth,
            s.blk,
            s.seq
        );
        frame(tx, &o).await;
    }
    let mut o = Out::new();
    let _ = write!(
        o,
        "REPORT F wr={} to={} oth={} seq={} blk={}",
        W_WRITTEN.load(Relaxed),
        W_TIMEOUTS.load(Relaxed),
        W_OTHER.load(Relaxed),
        W_SEQ.load(Relaxed),
        W_MAXBLOCK.load(Relaxed)
    );
    frame(tx, &o).await;
    let st = link.stats();
    let mut o = Out::new();
    let _ = write!(
        o,
        "REPORT END ups={} downs={} rst={} susp={} edges={}",
        st.ups,
        st.downs,
        st.resets,
        st.suspends,
        DTR_EDGES.load(Relaxed)
    );
    frame(tx, &o).await;
}

async fn handle(tx: &mut Tx<'static, UsbDriver>, link: &'static Link<'static>, m: Msg) {
    match m {
        Msg::Wake => {}
        Msg::Data(n, b) => {
            if tx.write_all(&b[..n]).await.is_err() {
                W_ECHO_ERR.fetch_add(1, Relaxed);
            }
        }
        Msg::Line(n, b) => {
            if tx.write_all(&b[..n]).await.is_err() {
                W_ECHO_ERR.fetch_add(1, Relaxed);
            }
        }
        Msg::Report => report(tx, link).await,
    }
}

#[embassy_executor::task]
async fn writer_task(mut tx: Tx<'static, UsbDriver>, link: &'static Link<'static>) {
    loop {
        match MODE.load(Relaxed) {
            M_RECORDS => {
                while let Ok(m) = CH.try_receive() {
                    handle(&mut tx, link, m).await;
                }
                if MODE.load(Relaxed) != M_RECORDS {
                    continue;
                }
                let seq = W_SEQ.fetch_add(1, Relaxed);
                let mut r = [0u8; rec::LEN];
                rec::fill(seq, &mut r);
                let t0 = Instant::now();
                let res = with_timeout(Duration::from_millis(100), tx.write_all(&r)).await;
                let dt = (Instant::now() - t0).as_micros() as u32;
                W_MAXBLOCK.fetch_max(dt, Relaxed);
                match res {
                    Ok(Ok(())) => {
                        W_WRITTEN.fetch_add(1, Relaxed);
                    }
                    Err(_) => {
                        W_TIMEOUTS.fetch_add(1, Relaxed);
                    }
                    Ok(Err(_)) => {
                        W_OTHER.fetch_add(1, Relaxed);
                        Timer::after_millis(1).await;
                    }
                }
            }
            M_FILL => {
                let seq = W_SEQ.fetch_add(1, Relaxed);
                let mut r = [0u8; rec::LEN];
                rec::fill(seq, &mut r);
                W_START.store(spy::now_us(), Relaxed);
                W_PENDING.store(true, Relaxed);
                let res = tx.write_all(&r).await;
                let t = spy::now_us();
                W_PENDING.store(false, Relaxed);
                if res.is_err() {
                    C_RET.store(t, Relaxed);
                    C_RES.store(2, Relaxed);
                    MODE.store(M_IDLE, Relaxed);
                }
            }
            _ => {
                let m = CH.receive().await;
                handle(&mut tx, link, m).await;
            }
        }
    }
}

struct Dev {
    rx: Rx<'static, UsbDriver>,
    link: &'static Link<'static>,
    buf: [u8; 64],
    pos: usize,
    len: usize,
    line: [u8; 64],
    /// Bytes that were not a command: before the start marker, or unknown lines.
    junk: u32,
    pre: u32,
    cycles: u32,
}

impl Dev {
    async fn send_out(&self, out: &mut Out) {
        let _ = out.write_str("\n");
        let b = out.bytes();
        let mut buf = [0u8; 96];
        buf[..b.len()].copy_from_slice(b);
        CH.send(Msg::Line(b.len(), buf)).await;
    }

    async fn say(&self, args: fmt::Arguments<'_>) {
        let mut out = Out::new();
        let _ = out.write_fmt(args);
        self.send_out(&mut out).await;
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
                    self.junk += self.pre;
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

    async fn ready(&self) {
        let st = self.link.stats();
        self.say(format_args!(
            "READY pre={} junk={} dtr={} ups={} downs={}",
            self.pre,
            self.junk,
            u8::from(self.link.dtr()),
            st.ups,
            st.downs
        ))
        .await;
    }

    async fn echo(&mut self, n: u32) -> Result<(), ()> {
        let mut done = 0u32;
        while done < n {
            self.fill().await?;
            let k = (self.len - self.pos).min((n - done) as usize);
            let mut b = [0u8; 64];
            b[..k].copy_from_slice(&self.buf[self.pos..self.pos + k]);
            self.pos += k;
            done += k as u32;
            CH.send(Msg::Data(k, b)).await;
        }
        self.say(format_args!("DONE {}", done)).await;
        Ok(())
    }

    /// Check A: two phases of 10 s, state sampled in the middle of each.
    async fn run_a(&self) {
        tick_take();
        for a in [&W_WRITTEN, &W_TIMEOUTS, &W_OTHER, &W_MAXBLOCK, &W_SEQ] {
            a.store(0, Relaxed);
        }
        set_mode(M_RECORDS).await;
        let t0 = Instant::now();
        for ph in 0..2u64 {
            let base = t0 + Duration::from_secs(10 * ph);
            Timer::at(base + Duration::from_secs(5)).await;
            let (up, dtr) = (self.link.is_up(), self.link.dtr());
            Timer::at(base + Duration::from_secs(10)).await;
            let (tmax, tmin, late, n) = tick_take();
            let snap = Snap {
                tmax,
                tmin,
                late,
                n,
                up,
                dtr,
                wr: W_WRITTEN.load(Relaxed),
                to: W_TIMEOUTS.load(Relaxed),
                oth: W_OTHER.load(Relaxed),
                blk: W_MAXBLOCK.swap(0, Relaxed),
                seq: W_SEQ.load(Relaxed),
            };
            critical_section::with(|cs| SNAPS.borrow_ref_mut(cs)[ph as usize] = snap);
        }
    }

    /// Checks C and D. The writer is made to have a `Tx::write` pending (the computer is not
    /// reading). `external` false: release the D+ pull-up, note when the write returns, enable it
    /// again 500 ms after. `kind` 1: only wait for the computer to make the link go away.
    /// `kind` 2: the control, nothing is detached; the computer reads again after 900 ms.
    async fn cycle(&mut self, kind: u8) {
        let external = kind == 1;
        spy::clear();
        C_BASE.store(0, Relaxed);
        C_RET.store(0, Relaxed);
        C_RES.store(0, Relaxed);
        C_REEN.store(0, Relaxed);
        self.cycles += 1;
        let c0 = spy::CONFIGURED.load(Relaxed);
        set_mode(M_FILL).await;
        let t_wait = Instant::now();
        let mut pending = false;
        while t_wait.elapsed() < Duration::from_secs(8) {
            if W_PENDING.load(Relaxed)
                && spy::now_us().wrapping_sub(W_START.load(Relaxed)) >= 300_000
            {
                pending = true;
                break;
            }
            Timer::after_millis(5).await;
        }
        if !pending {
            C_RES.store(9, Relaxed);
            MODE.store(M_IDLE, Relaxed);
            return;
        }
        if kind == 2 {
            Timer::after_millis(600).await;
            MODE.store(M_IDLE, Relaxed);
            return;
        }
        gap_arm();
        if external {
            C_BASE.store(spy::now_us(), Relaxed);
            let t = Instant::now();
            while C_RES.load(Relaxed) == 0 && t.elapsed() < Duration::from_secs(20) {
                Timer::after_millis(2).await;
            }
        } else {
            let t = Instant::now();
            C_BASE.store(spy::now_us(), Relaxed);
            pullup(false);
            while C_RES.load(Relaxed) == 0 && t.elapsed() < Duration::from_secs(2) {
                Timer::after_millis(1).await;
            }
            Timer::at(t + Duration::from_millis(500)).await;
            C_REEN.store(spy::now_us(), Relaxed);
            pullup(true);
        }
        let t = Instant::now();
        while t.elapsed() < Duration::from_secs(15) {
            if spy::CONFIGURED.load(Relaxed) > c0 && self.link.is_up() {
                break;
            }
            Timer::after_millis(5).await;
        }
        C_GAP.store(gap_disarm(), Relaxed);
        MODE.store(M_IDLE, Relaxed);
    }

    async fn cres(&self) {
        let base = C_BASE.load(Relaxed);
        let ret = C_RET.load(Relaxed);
        let reen = C_REEN.load(Relaxed);
        let st = self.link.stats();
        self.say(format_args!(
            "CRES n={} res={} ret_us={} reen_us={} gap={} ups={} downs={} junk={}",
            self.cycles,
            C_RES.load(Relaxed),
            if ret == 0 {
                -1
            } else {
                ret.wrapping_sub(base) as i32
            },
            if reen == 0 {
                -1
            } else {
                reen.wrapping_sub(base) as i32
            },
            C_GAP.load(Relaxed),
            st.ups,
            st.downs,
            self.junk
        ))
        .await;
        let mut i = 0;
        while i < spy::len() {
            let mut out = Out::new();
            let _ = out.write_str("CEV ");
            i = spy::write(&mut out, base, i, 7);
            self.send_out(&mut out).await;
        }
    }

    /// Serves commands until `FINISH` (`Ok`) or until the link goes down (`Err`).
    async fn session(&mut self) -> Result<(), ()> {
        self.pos = 0;
        self.len = 0;
        self.await_marker().await?;
        self.ready().await;
        loop {
            let n = self.read_line().await?;
            let line = self.line;
            let text = core::str::from_utf8(&line[..n]).unwrap_or("");
            match text {
                "START" => self.ready().await,
                "STATE" => {
                    let st = self.link.stats();
                    self.say(format_args!(
                        "STATE up={} dtr={} ups={} downs={} rst={} susp={} zlp={} edges={} echoerr={} junk={}",
                        u8::from(self.link.is_up()),
                        u8::from(self.link.dtr()),
                        st.ups,
                        st.downs,
                        st.resets,
                        st.suspends,
                        st.zlp_skipped,
                        DTR_EDGES.load(Relaxed),
                        W_ECHO_ERR.load(Relaxed),
                        self.junk
                    ))
                    .await;
                }
                "RUNA" => self.run_a().await,
                "REPORT" => {
                    MODE.store(M_IDLE, Relaxed);
                    CH.send(Msg::Report).await;
                }
                "CYCLE 0" => self.cycle(0).await,
                "CYCLE 1" => self.cycle(1).await,
                "CYCLE 2" => self.cycle(2).await,
                "LOCK" => {
                    let t0 = cycles();
                    let mut n = 0u32;
                    for _ in 0..1000 {
                        n += u32::from(core::hint::black_box(self.link.is_up()));
                    }
                    let t1 = cycles();
                    for _ in 0..1000 {
                        n += core::hint::black_box(self.link.stats()).ups;
                    }
                    let t2 = cycles();
                    self.say(format_args!(
                        "LOCK is_up {} stats {} cycles per call ({})",
                        t1.wrapping_sub(t0) / 1000,
                        t2.wrapping_sub(t1) / 1000,
                        n
                    ))
                    .await;
                }
                "CRES" => self.cres().await,
                "GAP 1" => gap_arm(),
                "GAP 0" => {
                    let g = gap_disarm();
                    let clk = embassy_rp::clocks::clk_sys_freq();
                    let ns = (u64::from(g) * 1_000_000_000 / u64::from(clk)) as u32;
                    self.say(format_args!("GAP {} {}", g, ns)).await;
                }
                "FINISH" => {
                    let st = self.link.stats();
                    self.say(format_args!(
                        "BYE ups={} downs={} rst={} susp={} zlp={} edges={}",
                        st.ups,
                        st.downs,
                        st.resets,
                        st.suspends,
                        st.zlp_skipped,
                        DTR_EDGES.load(Relaxed)
                    ))
                    .await;
                    Timer::after_millis(100).await;
                    return Ok(());
                }
                _ => match proto::parse(&line[..n]) {
                    Cmd::Ping => self.say(format_args!("PONG")).await,
                    Cmd::Echo(k) => self.echo(k).await?,
                    _ => {
                        self.junk += n as u32 + 1;
                        self.say(format_args!("ERR")).await;
                    }
                },
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("[INFO] usb_cdc_link: device start");
    let p = embassy_rp::init(Default::default());
    // SAFETY: the debug block; the cycle counter is used by this program only.
    unsafe {
        let mut core = cortex_m::Peripherals::steal();
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
    }

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
    spawner.spawn(tick_task(link).unwrap());
    spawner.spawn(writer_task(tx, link).unwrap());

    let mut dev = Dev {
        rx,
        link,
        buf: [0; 64],
        pos: 0,
        len: 0,
        line: [0; 64],
        junk: 0,
        pre: 0,
        cycles: 0,
    };
    loop {
        dev.link.wait_up().await;
        if dev.session().await.is_ok() {
            break;
        }
        Timer::after_millis(1).await;
    }

    let mut failures = 0u32;
    let st = link.stats();
    defmt::info!(
        "[PASS] FINISH served; link stats ups {=u32} downs {=u32} resets {=u32} suspends {=u32}",
        st.ups,
        st.downs,
        st.resets,
        st.suspends
    );
    let other = W_OTHER.load(Relaxed);
    if other != 0 {
        failures += 1;
        defmt::error!(
            "[FAIL] records writer saw {=u32} errors other than a timeout",
            other
        );
    }
    defmt::info!(
        "[STAT] cycles {=u32}, echo write errors {=u32}, junk bytes {=u32}, spy events dropped {=u32}",
        dev.cycles,
        W_ECHO_ERR.load(Relaxed),
        dev.junk,
        spy::DROPPED.load(Relaxed)
    );
    pullup(false);
    hil_common::exit(failures)
}
