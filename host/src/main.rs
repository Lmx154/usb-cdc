//! Computer side of the usb-cdc on-board proof. Finds the test board by VID 0xc0de and PID 0xcafe,
//! drives the line protocol of `hil/src/proto` and prints [PASS] / [FAIL] / [INFO] lines.
//! `usb-cdc-host smoke` drives `usb_cdc_smoke`; `usb-cdc-host fullpacket` drives `usb_cdc_fullpacket`.
//! Exit status 0 only if every check passed.

use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::process::exit;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, channel};
use std::thread;
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, SerialPort, SerialPortType};

const VID: u16 = 0xc0de;
const PID: u16 = 0xcafe;
const OPEN_BUDGET: Duration = Duration::from_secs(10);
const ENUM_LIMIT: Duration = Duration::from_secs(5);
const BURST_LIMIT: Duration = Duration::from_millis(50);
const ECHO_BYTES: usize = 1 << 20;
const SEND_BYTES: u32 = 4 << 20;
const RECV_BYTES: u32 = 1 << 20;
const THROUGHPUT_FLOOR_KBPS: f64 = 250.0;

fn pat(i: u64) -> u8 {
    (i % 251) as u8
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

#[derive(Default)]
struct Rep {
    fails: u32,
    passes: u32,
}

impl Rep {
    fn check(&mut self, ok: bool, text: String) {
        if ok {
            self.passes += 1;
            println!("[PASS] {text}");
        } else {
            self.fails += 1;
            println!("[FAIL] {text}");
        }
    }
}

fn info(text: String) {
    println!("[INFO] {text}");
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

enum Msg {
    Data(Instant, Vec<u8>),
    Failed(String),
}

struct Chunk {
    at: Instant,
    data: Vec<u8>,
    off: usize,
}

struct Taken {
    data: Vec<u8>,
    first: Option<Instant>,
    last: Option<Instant>,
}

struct Conn {
    port: Box<dyn SerialPort>,
    rx: Receiver<Msg>,
    queue: VecDeque<Chunk>,
    stop: Arc<AtomicBool>,
    read_error: Option<String>,
}

impl Conn {
    fn new(port: Box<dyn SerialPort>) -> Result<Conn, serialport::Error> {
        let mut reader = port.try_clone()?;
        reader.set_timeout(Duration::from_millis(5))?;
        let (tx, rx) = channel();
        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        thread::spawn(move || {
            let mut buf = vec![0u8; 65536];
            while !flag.load(Ordering::Relaxed) {
                match reader.read(&mut buf) {
                    Ok(0) => {}
                    Ok(n) => {
                        if tx
                            .send(Msg::Data(Instant::now(), buf[..n].to_vec()))
                            .is_err()
                        {
                            return;
                        }
                    }
                    Err(e)
                        if e.kind() == ErrorKind::TimedOut
                            || e.kind() == ErrorKind::Interrupted => {}
                    Err(e) => {
                        let _ = tx.send(Msg::Failed(e.to_string()));
                        return;
                    }
                }
            }
        });
        Ok(Conn {
            port,
            rx,
            queue: VecDeque::new(),
            stop,
            read_error: None,
        })
    }

    fn send(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.port.write_all(bytes)
    }

    fn cmd(&mut self, text: &str) -> std::io::Result<()> {
        self.send(format!("{text}\n").as_bytes())
    }

    /// Waits for one more message until `deadline`; false when nothing came.
    fn pump(&mut self, deadline: Instant) -> bool {
        let wait = deadline.saturating_duration_since(Instant::now());
        match self.rx.recv_timeout(wait) {
            Ok(Msg::Data(at, data)) => {
                self.queue.push_back(Chunk { at, data, off: 0 });
                true
            }
            Ok(Msg::Failed(e)) => {
                self.read_error = Some(e);
                false
            }
            Err(RecvTimeoutError::Timeout) | Err(RecvTimeoutError::Disconnected) => false,
        }
    }

    /// Takes up to `n` bytes, waiting until `deadline` for them.
    fn take(&mut self, n: usize, deadline: Instant) -> Taken {
        let mut t = Taken {
            data: Vec::with_capacity(n),
            first: None,
            last: None,
        };
        while t.data.len() < n {
            if let Some(front) = self.queue.front_mut() {
                let want = n - t.data.len();
                let avail = front.data.len() - front.off;
                let k = want.min(avail);
                t.data
                    .extend_from_slice(&front.data[front.off..front.off + k]);
                t.first.get_or_insert(front.at);
                t.last = Some(front.at);
                front.off += k;
                if front.off == front.data.len() {
                    self.queue.pop_front();
                }
            } else if !self.pump(deadline) {
                break;
            }
        }
        t
    }

    /// Reads one line (without the newline), or None on timeout. Bytes read so far stay lost.
    fn line(&mut self, deadline: Instant) -> Option<String> {
        let mut out = Vec::new();
        loop {
            let t = self.take(1, deadline);
            match t.data.first() {
                Some(b'\n') => return Some(String::from_utf8_lossy(&out).into_owned()),
                Some(&b) => out.push(b),
                None => return None,
            }
        }
    }

    /// Everything that arrives until `quiet` passes with nothing.
    fn drain(&mut self, quiet: Duration) -> Vec<u8> {
        let mut out = Vec::new();
        loop {
            while let Some(c) = self.queue.pop_front() {
                out.extend_from_slice(&c.data[c.off..]);
            }
            if !self.pump(Instant::now() + quiet) {
                return out;
            }
        }
    }

    /// Sends PING and wants exactly `PONG`; returns the number of stray bytes before it.
    fn ping(&mut self) -> Result<usize, String> {
        self.cmd("PING").map_err(|e| format!("write: {e}"))?;
        let mut stray = 0;
        for _ in 0..8 {
            match self.line(Instant::now() + Duration::from_secs(2)) {
                Some(l) if l == "PONG" => return Ok(stray),
                Some(l) => stray += l.len() + 1,
                None => return Err("no PONG within 2 s".into()),
            }
        }
        Err("no PONG in 8 lines".into())
    }
}

impl Drop for Conn {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

struct Opened {
    conn: Conn,
    name: String,
}

fn open_port(rep: &mut Rep) -> Option<Opened> {
    let t0 = Instant::now();
    let mut notes: Vec<(String, f64, u32)> = Vec::new();
    let mut found_at: Option<Duration> = None;
    let mut ident = String::new();
    let mut note = |text: String, t: Duration| {
        if let Some(n) = notes.iter_mut().find(|n| n.0 == text) {
            n.2 += 1;
        } else {
            notes.push((text, ms(t), 1));
        }
    };
    let opened = loop {
        match serialport::available_ports() {
            Ok(ports) => {
                let hit = ports.into_iter().find(|p| {
                    matches!(&p.port_type, SerialPortType::UsbPort(u) if u.vid == VID && u.pid == PID)
                });
                match hit {
                    Some(p) => {
                        found_at.get_or_insert(t0.elapsed());
                        if let SerialPortType::UsbPort(u) = &p.port_type {
                            ident = format!(
                                "manufacturer {:?}, product {:?}, serial {:?}",
                                u.manufacturer, u.product, u.serial_number
                            );
                        }
                        match serialport::new(&p.port_name, 115_200)
                            .timeout(Duration::from_secs(5))
                            .open()
                        {
                            Ok(port) => break Some((p.port_name, port, t0.elapsed())),
                            Err(e) => note(format!("open {}: {e}", p.port_name), t0.elapsed()),
                        }
                    }
                    None => note("port not listed".to_string(), t0.elapsed()),
                }
            }
            Err(e) => note(format!("listing ports: {e}"), t0.elapsed()),
        }
        if t0.elapsed() > OPEN_BUDGET {
            break None;
        }
        thread::sleep(Duration::from_millis(50));
    };
    for (text, first, count) in &notes {
        info(format!(
            "while waiting: {text} (first at {first:.0} ms, {count} times)"
        ));
    }
    match opened {
        Some((name, port, took)) => {
            info(format!(
                "port {name} found after {:.0} ms, opened after {:.0} ms ({ident})",
                ms(found_at.unwrap_or(took)),
                ms(took)
            ));
            rep.check(
                took <= ENUM_LIMIT,
                format!(
                    "port appears and opens within 5 s of the device program starting: {:.0} ms",
                    ms(took)
                ),
            );
            let sys = sysfs_identity(&name);
            rep.check(
                sys == ["MARV", "usb-cdc test", "usb-cdc-hil"],
                format!("identity strings from sysfs: {sys:?}"),
            );
            match Conn::new(port) {
                Ok(conn) => {
                    let _ = conn.port.clear(ClearBuffer::Input);
                    Some(Opened { conn, name })
                }
                Err(e) => {
                    rep.check(false, format!("cloning the port for the reader: {e}"));
                    None
                }
            }
        }
        None => {
            rep.check(
                false,
                format!("port with VID {VID:#06x} PID {PID:#06x} opened within 10 s"),
            );
            None
        }
    }
}

/// Manufacturer, product and serial of the USB device behind a tty, read from sysfs (udev may not
/// have filled its own database yet when the port first appears).
fn sysfs_identity(port: &str) -> Vec<String> {
    let tty = port.rsplit('/').next().unwrap_or(port);
    ["manufacturer", "product", "serial"]
        .iter()
        .map(|f| {
            std::fs::read_to_string(format!("/sys/class/tty/{tty}/device/../{f}"))
                .map(|s| s.trim().to_string())
                .unwrap_or_else(|e| format!("<{e}>"))
        })
        .collect()
}

/// START / READY. Returns the READY line.
fn handshake(c: &mut Conn, rep: &mut Rep) -> Option<String> {
    // PREMARKER=1 sends 40 junk bytes first, to exercise the device's discard-and-count path.
    let junk: usize = if std::env::var_os("PREMARKER").is_some() {
        40
    } else {
        0
    };
    if junk > 0 {
        let bytes: Vec<u8> = (0..junk as u8).collect();
        if let Err(e) = c.send(&bytes) {
            rep.check(false, format!("sending junk before the marker: {e}"));
            return None;
        }
        thread::sleep(Duration::from_millis(50));
    }
    if let Err(e) = c.send(b"START\n") {
        rep.check(false, format!("sending the start marker: {e}"));
        return None;
    }
    match c.line(Instant::now() + Duration::from_secs(5)) {
        Some(l) if l.starts_with("READY") => {
            info(format!("device: {l}"));
            rep.check(true, "start marker accepted, device says READY".to_string());
            if junk > 0 {
                rep.check(
                    field(&l, "pre") == Some(junk as i64),
                    format!(
                        "device counted the {junk} junk bytes sent before the marker: {:?}",
                        field(&l, "pre")
                    ),
                );
            }
            Some(l)
        }
        other => {
            rep.check(false, format!("start marker: wanted READY, got {other:?}"));
            None
        }
    }
}

fn field(line: &str, key: &str) -> Option<i64> {
    line.split(' ')
        .find_map(|w| w.strip_prefix(key)?.strip_prefix('=')?.parse().ok())
}

fn make_sizes(rng: &mut Rng, total: usize) -> Vec<usize> {
    let mut sizes = Vec::new();
    for s in [63usize, 64, 65, 128, 4096] {
        sizes.extend(std::iter::repeat_n(s, 20));
    }
    let mut sum: usize = sizes.iter().sum();
    loop {
        let s = 1 + (rng.next() % 4096) as usize;
        if sum + s > total {
            break;
        }
        sizes.push(s);
        sum += s;
    }
    if total > sum {
        sizes.push(total - sum);
    }
    for i in (1..sizes.len()).rev() {
        let j = (rng.next() % (i as u64 + 1)) as usize;
        sizes.swap(i, j);
    }
    sizes
}

fn echo_test(c: &mut Conn, rep: &mut Rep) {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let sizes = make_sizes(&mut rng, ECHO_BYTES);
    let mut data = vec![0u8; ECHO_BYTES];
    for chunk in data.chunks_mut(8) {
        let r = rng.next().to_le_bytes();
        chunk.copy_from_slice(&r[..chunk.len()]);
    }
    let counts: Vec<String> = [63usize, 64, 65, 128, 4096]
        .iter()
        .map(|s| format!("{s}: {}", sizes.iter().filter(|x| *x == s).count()))
        .collect();
    info(format!(
        "echo plan: {} writes, sizes 1..=4096, forced counts {{{}}}",
        sizes.len(),
        counts.join(", ")
    ));
    rep.check(
        [63usize, 64, 65, 128, 4096]
            .iter()
            .all(|s| sizes.iter().filter(|x| *x == s).count() >= 20)
            && sizes.iter().sum::<usize>() == ECHO_BYTES
            && sizes.iter().all(|s| (1..=4096).contains(s)),
        "echo plan covers 1 MiB, every size in 1..=4096, each forced size at least 20 times"
            .to_string(),
    );
    if c.cmd(&format!("ECHO {ECHO_BYTES}")).is_err() {
        rep.check(false, "sending ECHO".to_string());
        return;
    }
    let t0 = Instant::now();
    let mut off = 0;
    for s in &sizes {
        if let Err(e) = c.send(&data[off..off + s]) {
            rep.check(false, format!("echo write at offset {off} failed: {e}"));
            return;
        }
        off += s;
    }
    let t_written = t0.elapsed();
    let got = c.take(ECHO_BYTES, Instant::now() + Duration::from_secs(30));
    let t_echo = got.last.map_or(Duration::ZERO, |t| t - t0);
    let missing = ECHO_BYTES - got.data.len();
    let mismatches = got.data.iter().zip(&data).filter(|(a, b)| a != b).count();
    let mut extra = 0usize;
    let mut done = None;
    for _ in 0..4 {
        match c.line(Instant::now() + Duration::from_secs(5)) {
            Some(l) if l.starts_with("DONE") => {
                done = Some(l);
                break;
            }
            Some(l) => extra += l.len() + 1,
            None => break,
        }
    }
    extra += c.drain(Duration::from_millis(100)).len();
    rep.check(
        mismatches == 0 && missing == 0 && extra == 0,
        format!("echo 1 MiB: {mismatches} mismatches, {missing} missing, {extra} extra bytes"),
    );
    rep.check(
        done.as_deref() == Some(&format!("DONE {ECHO_BYTES}")),
        format!("device counted the echo: {done:?}"),
    );
    let rate = ECHO_BYTES as f64 / t_echo.as_secs_f64().max(1e-9) / 1000.0;
    info(format!(
        "echo: written in {:.0} ms, last echoed byte back after {:.0} ms: {:.0} kB/s each way",
        ms(t_written),
        ms(t_echo),
        rate
    ));
}

fn send_test(c: &mut Conn, rep: &mut Rep) {
    if c.cmd(&format!("SEND {SEND_BYTES}")).is_err() {
        rep.check(false, "sending SEND".to_string());
        return;
    }
    let got = c.take(
        SEND_BYTES as usize,
        Instant::now() + Duration::from_secs(60),
    );
    let bad = got
        .data
        .iter()
        .enumerate()
        .filter(|(i, b)| **b != pat(*i as u64))
        .count();
    let first_bad = got
        .data
        .iter()
        .enumerate()
        .position(|(i, b)| *b != pat(i as u64));
    let missing = SEND_BYTES as usize - got.data.len();
    let secs = match (got.first, got.last) {
        (Some(a), Some(b)) => (b - a).as_secs_f64(),
        _ => 0.0,
    };
    let kbps = if secs > 0.0 {
        (got.data.len() as f64 - 1.0) / secs / 1000.0
    } else {
        0.0
    };
    let line = c.line(Instant::now() + Duration::from_secs(5));
    let extra = c.drain(Duration::from_millis(100)).len();
    rep.check(
        bad == 0 && missing == 0 && extra == 0,
        format!("device to computer 4 MiB: {bad} wrong bytes (first at {first_bad:?}), {missing} missing, {extra} extra"),
    );
    rep.check(
        kbps >= THROUGHPUT_FLOOR_KBPS,
        format!("device to computer throughput {kbps:.1} kB/s over {secs:.2} s (floor {THROUGHPUT_FLOOR_KBPS} kB/s)"),
    );
    info(format!("device says: {line:?}"));
}

fn recv_test(c: &mut Conn, rep: &mut Rep) {
    if c.cmd(&format!("RECV {RECV_BYTES}")).is_err() {
        rep.check(false, "sending RECV".to_string());
        return;
    }
    let t0 = Instant::now();
    let mut buf = vec![0u8; 16384];
    let mut off: u64 = 0;
    while off < RECV_BYTES as u64 {
        let k = buf.len().min((RECV_BYTES as u64 - off) as usize);
        for (j, b) in buf[..k].iter_mut().enumerate() {
            *b = pat(off + j as u64);
        }
        if let Err(e) = c.send(&buf[..k]) {
            rep.check(
                false,
                format!("computer to device write failed at {off}: {e}"),
            );
            return;
        }
        off += k as u64;
    }
    let t_written = t0.elapsed();
    let line = c.line(Instant::now() + Duration::from_secs(20));
    let t_total = t0.elapsed();
    let extra = c.drain(Duration::from_millis(100)).len();
    let parts: Vec<i64> = line
        .as_deref()
        .and_then(|l| l.strip_prefix("RECVD "))
        .map(|l| l.split(' ').filter_map(|w| w.parse().ok()).collect())
        .unwrap_or_default();
    let ok = parts.len() == 3 && parts[0] == RECV_BYTES as i64 && parts[1] == 0 && extra == 0;
    rep.check(
        ok,
        format!(
            "computer to device 1 MiB counted and checked by the device: {line:?}, {extra} extra"
        ),
    );
    let host_kbps = RECV_BYTES as f64 / t_total.as_secs_f64() / 1000.0;
    let dev_kbps = if parts.len() == 3 && parts[2] > 0 {
        RECV_BYTES as f64 / (parts[2] as f64 / 1e6) / 1000.0
    } else {
        0.0
    };
    info(format!(
        "computer to device: written in {:.0} ms, reply after {:.0} ms: {host_kbps:.1} kB/s (host clock), {dev_kbps:.1} kB/s (device clock, first to last byte)",
        ms(t_written),
        ms(t_total)
    ));
}

fn burst_once(c: &mut Conn, n: usize, wait: Duration) -> (Taken, Instant) {
    let _ = c.cmd(&format!("BURST {n}"));
    let sent = Instant::now();
    (c.take(n, sent + wait), sent)
}

fn burst_test(c: &mut Conn, rep: &mut Rep) {
    for n in [64usize, 128, 4096, 63, 1, 65] {
        let mut worst = Duration::ZERO;
        let mut worst_cmd = Duration::ZERO;
        let mut bad = 0;
        let mut short = 0;
        let mut stray = 0;
        let mut times = Vec::new();
        for _ in 0..10 {
            let (got, sent) = burst_once(c, n, Duration::from_secs(1));
            if got.data.len() < n {
                short += 1;
            }
            bad += got
                .data
                .iter()
                .enumerate()
                .filter(|(i, b)| **b != pat(*i as u64))
                .count();
            if let (Some(a), Some(b)) = (got.first, got.last) {
                let d = b - a;
                worst = worst.max(d);
                worst_cmd = worst_cmd.max(b - sent);
                times.push(ms(d));
            }
            match c.ping() {
                Ok(s) => stray += s,
                Err(e) => {
                    rep.check(false, format!("burst {n}: {e}"));
                    return;
                }
            }
            stray += c.drain(Duration::from_millis(20)).len();
        }
        rep.check(
            short == 0 && bad == 0 && stray == 0 && worst <= BURST_LIMIT,
            format!(
                "burst of {n} bytes x10: {short} incomplete, {bad} wrong bytes, {stray} stray bytes, worst first-to-last {:.1} ms (limit 50), worst command-to-last {:.1} ms",
                ms(worst),
                ms(worst_cmd)
            ),
        );
        info(format!(
            "burst {n}: first-to-last ms per run {:?}",
            times
                .iter()
                .map(|t| (t * 10.0).round() / 10.0)
                .collect::<Vec<_>>()
        ));
    }
}

fn fullpacket_test(c: &mut Conn, rep: &mut Rep) {
    let mut held_total = 0;
    for n in [64usize, 128, 4096] {
        let mut prompt = 0;
        let mut held = 0;
        let mut partial_before_nudge = Vec::new();
        let mut worst = Duration::ZERO;
        let mut nudge_lat = Duration::ZERO;
        let mut bad = 0;
        for _ in 0..10 {
            let (got, _) = burst_once(c, n, Duration::from_millis(300));
            let mut data = got.data;
            let was_held = data.len() < n;
            if !was_held {
                prompt += 1;
                if let (Some(a), Some(b)) = (got.first, got.last) {
                    worst = worst.max(b - a);
                }
            } else {
                held += 1;
                partial_before_nudge.push(data.len());
                let _ = c.cmd("NUDGE");
                let nudged = Instant::now();
                let rest = c.take(n + 1 - data.len(), nudged + Duration::from_secs(2));
                if let Some(l) = rest.last {
                    nudge_lat = nudge_lat.max(l - nudged);
                }
                data.extend_from_slice(&rest.data);
            }
            let body = &data[..data.len().min(n)];
            bad += body
                .iter()
                .enumerate()
                .filter(|(i, b)| **b != pat(*i as u64))
                .count();
            if data.len() != if was_held { n + 1 } else { n } {
                bad += 1;
            }
            if c.ping().is_err() {
                rep.check(false, format!("fullpacket burst {n}: no PONG"));
                return;
            }
            c.drain(Duration::from_millis(20));
        }
        held_total += held;
        rep.check(bad == 0, format!("full-packet burst {n}: all bytes arrived with the right values after the nudge ({bad} wrong)"));
        info(format!(
            "CONTROL burst {n} (full 64-byte packets, no zero-length packet): {prompt}/10 delivered within 300 ms (worst first-to-last {:.1} ms), {held}/10 held back (bytes seen before the nudge: {partial_before_nudge:?}; worst nudge-to-last {:.1} ms)",
            ms(worst),
            ms(nudge_lat)
        ));
    }
    info(format!(
        "CONTROL RESULT: {held_total}/30 bursts were held back until a short packet arrived ({})",
        if held_total > 0 {
            "the control fails as it should: the 63-byte rule matters on this computer"
        } else {
            "all arrived promptly: this computer does not hold back a full-packet transfer"
        }
    ));
}

fn finish(c: &mut Conn, rep: &mut Rep, smoke: bool) {
    let _ = c.cmd("FINISH");
    match c.line(Instant::now() + Duration::from_secs(5)) {
        Some(l) if l.starts_with("BYE") => {
            info(format!("device: {l}"));
            if smoke {
                rep.check(
                    field(&l, "ups") == Some(1) && field(&l, "downs") == Some(0),
                    "device link stats before the detach: ups 1, downs 0".to_string(),
                );
            }
        }
        other => rep.check(false, format!("FINISH: wanted BYE, got {other:?}")),
    }
}

fn smoke(rep: &mut Rep) {
    let Some(mut o) = open_port(rep) else { return };
    let name = o.name.clone();
    let c = &mut o.conn;
    info(format!("port name {name}"));
    let Some(ready) = handshake(c, rep) else {
        return;
    };
    info(format!(
        "DTR as the device read it: at link up {:?}, first true {:?} ms after link up (-1 never)",
        field(&ready, "dtr_up"),
        field(&ready, "dtr_first_ms")
    ));
    let _ = c.cmd("STATE");
    match c.line(Instant::now() + Duration::from_secs(2)) {
        Some(l) => {
            info(format!("device: {l}"));
            rep.check(
                field(&l, "up") == Some(1),
                "device reports the link up".to_string(),
            );
            info(format!(
                "DTR after the computer opened the port: {:?}",
                field(&l, "dtr")
            ));
        }
        None => rep.check(false, "STATE: no reply".to_string()),
    }
    echo_test(c, rep);
    send_test(c, rep);
    recv_test(c, rep);
    burst_test(c, rep);
    finish(c, rep, true);
}

fn fullpacket(rep: &mut Rep) {
    let Some(mut o) = open_port(rep) else { return };
    let name = o.name.clone();
    let c = &mut o.conn;
    info(format!("port name {name}"));
    if handshake(c, rep).is_none() {
        return;
    }
    fullpacket_test(c, rep);
    finish(c, rep, false);
}

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_default();
    if mode != "smoke" && mode != "fullpacket" {
        eprintln!("usage: usb-cdc-host smoke | fullpacket");
        exit(2);
    }
    thread::spawn(|| {
        thread::sleep(Duration::from_secs(150));
        println!("[FAIL] host program ran longer than 150 s");
        exit(1);
    });
    let mut rep = Rep::default();
    let t0 = Instant::now();
    if mode == "smoke" {
        smoke(&mut rep);
    } else {
        fullpacket(&mut rep);
    }
    println!(
        "host {mode}: {} passed, {} failed, {:.1} s",
        rep.passes,
        rep.fails,
        t0.elapsed().as_secs_f64()
    );
    exit(if rep.fails == 0 { 0 } else { 1 });
}
