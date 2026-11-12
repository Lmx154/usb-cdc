//! `usb-cdc-host link` (checks A to E of the on-board proof, against `usb_cdc_link`) and
//! `usb-cdc-host unplug` (check F, against `usb_cdc_unplug`, needs a person).

use std::fs::OpenOptions;
use std::os::fd::AsRawFd;
use std::thread;
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, SerialPortType};

use super::{Conn, PID, Rep, Rng, VID, field, handshake, info, make_sizes, ms, open_port};

const REC: usize = 63;

fn sleep_until(t: Instant) {
    thread::sleep(t.saturating_duration_since(Instant::now()));
}

/// Must match `hil_common::rec::fill`.
fn rec_build(seq: u32) -> [u8; REC] {
    let mut out = [0u8; REC];
    out[..4].copy_from_slice(&seq.to_le_bytes());
    for (i, b) in out[4..59].iter_mut().enumerate() {
        *b = (seq as u8)
            .wrapping_mul(31)
            .wrapping_add((i as u8) ^ ((seq >> 8) as u8));
    }
    let mut h: u32 = 0x811c_9dc5;
    for b in &out[..59] {
        h ^= u32::from(*b);
        h = h.wrapping_mul(0x0100_0193);
    }
    out[59..].copy_from_slice(&h.to_le_bytes());
    out
}

fn rec_check(r: &[u8]) -> Option<u32> {
    if r.len() != REC {
        return None;
    }
    let seq = u32::from_le_bytes([r[0], r[1], r[2], r[3]]);
    (rec_build(seq)[..] == *r).then_some(seq)
}

fn port_name() -> Option<String> {
    serialport::available_ports()
        .ok()?
        .into_iter()
        .find_map(|p| {
            matches!(&p.port_type, SerialPortType::UsbPort(u) if u.vid == VID && u.pid == PID)
                .then_some(p.port_name)
        })
}

/// Opens the test board's port again (it may take the enumeration time to appear).
fn open_again(budget: Duration) -> Option<(Conn, Duration)> {
    let t0 = Instant::now();
    loop {
        if let Some(name) = port_name() {
            if let Ok(port) = serialport::new(&name, 115_200)
                .timeout(Duration::from_secs(5))
                .open()
            {
                if let Ok(conn) = Conn::new(port) {
                    return Some((conn, t0.elapsed()));
                }
            }
        }
        if t0.elapsed() > budget {
            return None;
        }
        thread::sleep(Duration::from_millis(10));
    }
}

struct EchoRes {
    mismatches: usize,
    missing: usize,
    extra: usize,
    done: bool,
}

/// Echoes `n` random bytes written in awkward sizes; checks them and the `DONE` line.
fn echo_quiet(c: &mut Conn, n: usize, seed: u64, forced: usize) -> EchoRes {
    let mut rng = Rng(seed | 1);
    let sizes = make_sizes(&mut rng, n, forced);
    let mut data = vec![0u8; n];
    for chunk in data.chunks_mut(8) {
        let r = rng.next().to_le_bytes();
        chunk.copy_from_slice(&r[..chunk.len()]);
    }
    let bad = EchoRes {
        mismatches: 0,
        missing: n,
        extra: 0,
        done: false,
    };
    if c.cmd(&format!("ECHO {n}")).is_err() {
        return bad;
    }
    let mut off = 0;
    for s in &sizes {
        if c.send(&data[off..off + s]).is_err() {
            return EchoRes {
                missing: n - off,
                ..bad
            };
        }
        off += s;
    }
    let got = c.take(n, Instant::now() + Duration::from_secs(20));
    let mut extra = 0usize;
    let mut done = false;
    for _ in 0..4 {
        match c.line(Instant::now() + Duration::from_secs(3)) {
            Some(l) if l == format!("DONE {n}") => {
                done = true;
                break;
            }
            Some(l) => extra += l.len() + 1,
            None => break,
        }
    }
    extra += c.drain(Duration::from_millis(50)).len();
    EchoRes {
        mismatches: got.data.iter().zip(&data).filter(|(a, b)| a != b).count(),
        missing: n - got.data.len(),
        extra,
        done,
    }
}

fn ask(c: &mut Conn, cmd: &str, want: &str) -> Option<String> {
    c.cmd(cmd).ok()?;
    for _ in 0..4 {
        let l = c.line(Instant::now() + Duration::from_secs(3))?;
        if l.starts_with(want) {
            return Some(l);
        }
    }
    None
}

/// Lines that follow a command until `quiet` passes with nothing.
fn lines_until_quiet(c: &mut Conn, prefix: &str, quiet: Duration) -> Vec<String> {
    let mut out = Vec::new();
    while let Some(l) = c.line(Instant::now() + quiet) {
        if l.starts_with(prefix) {
            out.push(l);
        }
    }
    out
}

fn f(line: &str, key: &str) -> i64 {
    field(line, key).unwrap_or(-999)
}

fn event_name(tok: &str) -> String {
    let (code, dt) = tok.split_once('@').unwrap_or((tok, "?"));
    let name = match code {
        "E" => "enabled(true)",
        "e" => "enabled(false)",
        "R" => "reset",
        "A" => "addressed",
        "C" => "configured(true)",
        "c" => "configured(false)",
        "S" => "suspended(true)",
        "U" => "suspended(false)",
        _ => code,
    };
    format!("{name} at {dt} us")
}

fn events_text(lines: &[String]) -> String {
    let toks: Vec<String> = lines
        .iter()
        .flat_map(|l| {
            l.trim_start_matches("CEV ")
                .trim()
                .split(',')
                .map(str::to_string)
        })
        .filter(|t| !t.is_empty())
        .collect();
    if toks.is_empty() {
        "none".to_string()
    } else {
        toks.iter()
            .map(|t| event_name(t))
            .collect::<Vec<_>>()
            .join(", ")
    }
}

struct Frames {
    recs: Vec<u32>,
    bad: u32,
    report: Vec<String>,
}

/// Splits the stream into 63-byte frames: numbered records, and report frames up to `END`.
fn read_frames(c: &mut Conn, deadline: Instant) -> Frames {
    let mut fr = Frames {
        recs: Vec::new(),
        bad: 0,
        report: Vec::new(),
    };
    loop {
        let t = c.take(REC, deadline);
        if t.data.len() < REC {
            fr.bad += 1;
            break;
        }
        if &t.data[..4] == b"REPO" {
            let l = String::from_utf8_lossy(&t.data).trim().to_string();
            let end = l.contains("END");
            fr.report.push(l);
            if end {
                break;
            }
        } else if let Some(seq) = rec_check(&t.data) {
            fr.recs.push(seq);
        } else {
            fr.bad += 1;
        }
    }
    fr
}

fn line_with<'a>(lines: &'a [String], tag: &str) -> &'a str {
    lines
        .iter()
        .find(|l| l.contains(tag))
        .map(String::as_str)
        .unwrap_or("")
}

/// A: the computer keeps the port open and does not read for 10 s, then closes it for 10 s.
fn check_a(mut c: Conn, rep: &mut Rep) -> Option<Conn> {
    let _ = c.cmd("RUNA");
    c.hold(true);
    let t0 = Instant::now();
    sleep_until(t0 + Duration::from_millis(9500));
    let buffered = c.port.bytes_to_read().ok();
    sleep_until(t0 + Duration::from_secs(10));
    c.close();
    sleep_until(t0 + Duration::from_secs(20));
    let Some((mut c2, took)) = open_again(Duration::from_secs(10)) else {
        rep.check(false, "A: port opened again after 10 s closed".to_string());
        return None;
    };
    info(format!("A: port opened again after {:.0} ms", ms(took)));
    let _ = c2.cmd("REPORT");
    let fr = read_frames(&mut c2, Instant::now() + Duration::from_secs(30));
    for l in &fr.report {
        info(format!("A device: {l}"));
    }
    let (p1, p2) = (line_with(&fr.report, "P1"), line_with(&fr.report, "P2"));
    let (w1, w2) = (line_with(&fr.report, "W1"), line_with(&fr.report, "W2"));
    let fin = line_with(&fr.report, "REPORT F");
    let end = line_with(&fr.report, "END");
    rep.check(
        !p1.is_empty() && !p2.is_empty() && !fin.is_empty() && !end.is_empty(),
        format!(
            "A: the device's report arrived ({} frames)",
            fr.report.len()
        ),
    );
    let (late1, late2) = (f(p1, "late"), f(p2, "late"));
    rep.check(
        late1 == 0 && late2 == 0 && f(p1, "n") > 9000 && f(p2, "n") > 9000,
        format!(
            "A: 1 kHz tick task missed no tick: periods over 1.5 ms {late1} / {late2}; ticks {} / {}; largest period {} / {} us, smallest {} / {} us",
            f(p1, "n"), f(p2, "n"), f(p1, "tmax"), f(p2, "tmax"), f(p1, "tmin"), f(p2, "tmin")
        ),
    );
    let (wr1, to1) = (f(w1, "wr"), f(w1, "to"));
    let (wr2, to2) = (f(w2, "wr") - wr1, f(w2, "to") - to1);
    let blk = f(w1, "blk").max(f(w2, "blk"));
    rep.check(
        to1 > 0 && to2 > 0 && f(fin, "oth") == 0 && blk > 0 && blk <= 105_000,
        format!(
            "A: writer saw timeouts and no other error: phase 1 written {wr1} timed out {to1}; phase 2 written {wr2} timed out {to2}; other errors {}; longest single write {blk} us (timeout 100000 us)",
            f(fin, "oth")
        ),
    );
    let increasing = fr.recs.windows(2).all(|w| w[0] < w[1]);
    let total_seq = f(fin, "seq");
    let (written, timed_out) = (f(fin, "wr"), f(fin, "to"));
    let received = fr.recs.len() as i64;
    rep.check(
        fr.bad == 0
            && increasing
            && received > 0
            && fr.recs.iter().all(|s| i64::from(*s) < total_seq)
            && received <= written + timed_out,
        format!(
            "A: computer received {received} records, all whole with a right checksum ({} bad frames), sequence strictly increasing: {increasing}; device wrote {written} (since phase 1 start, whole run), {timed_out} timed out, last sequence number {total_seq}",
            fr.bad
        ),
    );
    if let (Some(first), Some(last)) = (fr.recs.first(), fr.recs.last()) {
        info(format!(
            "A: first received sequence number {first}, last {last}; {} numbers in that range were not received (given up on or discarded when the port was closed)",
            i64::from(*last) - i64::from(*first) + 1 - received
        ));
    }
    rep.check(
        f(p1, "up") == 1 && f(p2, "up") == 1,
        format!(
            "A: Link::is_up() in the middle of phase 1 / phase 2: {} / {}; dtr() {} / {}",
            f(p1, "up"),
            f(p2, "up"),
            f(p1, "dtr"),
            f(p2, "dtr")
        ),
    );
    info(format!(
        "A: computer side buffered while the port was open and not read: FIONREAD {:?} bytes at 9.5 s; the device's writes accepted in phase 1: {} records = {} bytes",
        buffered,
        wr1,
        wr1 * 63
    ));
    info(format!("A: {end}"));
    Some(c2)
}

/// B: 20 cycles of close, wait 200 ms, open, START, echo 64 KiB.
fn check_b(mut c: Conn, rep: &mut Rep) -> Option<Conn> {
    let s0 = ask(&mut c, "STATE", "STATE");
    let mut bad_cycles = 0;
    let mut dtr_bad = 0;
    let mut worst = (0usize, 0usize, 0usize);
    let mut last_state = s0.clone();
    let t0 = Instant::now();
    for i in 0..20u64 {
        c.close();
        thread::sleep(Duration::from_millis(200));
        let Some((c2, _)) = open_again(Duration::from_secs(10)) else {
            rep.check(false, format!("B cycle {i}: port opened again"));
            return None;
        };
        c = c2;
        let stray = c.drain(Duration::from_millis(30)).len();
        let ready = ask(&mut c, "START", "READY");
        let r = echo_quiet(&mut c, 65536, 0x1234_5678 + i, 3);
        let st = ask(&mut c, "STATE", "STATE");
        if let Some(s) = &st {
            if f(s, "dtr") != 1 {
                dtr_bad += 1;
            }
        }
        let ok = ready.is_some()
            && st.is_some()
            && stray == 0
            && r.mismatches == 0
            && r.missing == 0
            && r.extra == 0
            && r.done;
        if !ok {
            bad_cycles += 1;
            info(format!(
                "B cycle {i}: ready {ready:?}, stray {stray}, mismatched {}, missing {}, extra {}, done {}",
                r.mismatches, r.missing, r.extra, r.done
            ));
        }
        worst = (
            worst.0.max(r.mismatches),
            worst.1.max(r.missing),
            worst.2.max(r.extra + stray),
        );
        last_state = st;
    }
    rep.check(
        bad_cycles == 0,
        format!(
            "B: 20 cycles of close, 200 ms, open, START, echo 64 KiB: {bad_cycles} bad cycles; worst per cycle: {} mismatched, {} missing, {} extra bytes; {:.1} s",
            worst.0, worst.1, worst.2, t0.elapsed().as_secs_f64()
        ),
    );
    info(format!("B: device STATE before the cycles: {s0:?}"));
    info(format!("B: device STATE after the cycles:  {last_state:?}"));
    if let (Some(a), Some(b)) = (&s0, &last_state) {
        rep.check(
            f(a, "ups") == f(b, "ups") && f(a, "downs") == f(b, "downs") && dtr_bad == 0,
            format!(
                "B: closing and opening the port changed no Stats counter (ups {}, downs {}, resets {}, suspends {}) and dtr() read true after every open ({dtr_bad} not); dtr() edges seen by the 1 kHz sampler: {} for 20 close/open pairs",
                f(b, "ups"), f(b, "downs"), f(b, "rst") - f(a, "rst"), f(b, "susp") - f(a, "susp"),
                f(b, "edges") - f(a, "edges")
            ),
        );
    }
    Some(c)
}

fn gap_line(c: &mut Conn) -> Option<(i64, i64)> {
    let l = ask(c, "GAP 0", "GAP")?;
    let v: Vec<i64> = l
        .split(' ')
        .skip(1)
        .filter_map(|w| w.parse().ok())
        .collect();
    (v.len() == 2).then(|| (v[0], v[1]))
}

/// E: longest SysTick gap (interrupts masked) idle and with the echo running.
fn check_e(c: &mut Conn, rep: &mut Rep) -> Option<f64> {
    let lock = ask(c, "LOCK", "LOCK");
    info(format!(
        "E: {lock:?} (150 MHz core clock; one call = one critical section of the crate's shared state)"
    ));
    let _ = c.cmd("GAP 1");
    thread::sleep(Duration::from_secs(2));
    let idle = gap_line(c);
    let _ = c.cmd("GAP 1");
    thread::sleep(Duration::from_millis(50));
    let r = echo_quiet(c, 1 << 18, 0xE0E0_0001, 3);
    let hard = gap_line(c);
    rep.check(
        r.mismatches == 0 && r.missing == 0 && r.extra == 0 && idle.is_some() && hard.is_some(),
        format!(
            "E: echo of 256 KiB while measuring: {} mismatched, {} missing, {} extra",
            r.mismatches, r.missing, r.extra
        ),
    );
    info(format!(
        "E: longest gap between SysTick interrupts (every 300 cycles = 2 us): idle {idle:?} (cycles, ns), echo running {hard:?}"
    ));
    idle.map(|(cy, ns)| ns as f64 / cy.max(1) as f64)
}

/// C: 10 soft detaches with a write pending.
fn check_c(mut c: Conn, rep: &mut Rep, ns_per_cycle: f64) -> Option<Conn> {
    // Control: the same fill and junk, but no detach. The junk must reach the device then.
    let _ = c.cmd("CYCLE 2");
    c.hold(true);
    thread::sleep(Duration::from_millis(20));
    let _ = c.send(b"JUNKJUNKJUNKJUNKJUNKJUNKJUNKJUNKJUNKJUN\n");
    thread::sleep(Duration::from_millis(900));
    c.hold(false);
    c.drain(Duration::from_millis(400));
    let ctl = ask(&mut c, "STATE", "STATE");
    let junk0 = ctl.as_deref().map_or(-1, |l| f(l, "junk"));
    rep.check(
        junk0 >= 40,
        format!("C control: 40 bytes sent while the device was not reading (no detach) reached the device afterwards: device junk count {junk0}"),
    );
    let s0 = ctl;
    let (mut worst_ret, mut bad, mut stale) = (0i64, 0, 0usize);
    let mut gap_max = 0i64;
    let mut gone_ms = Vec::new();
    let mut back_ms = Vec::new();
    let mut last_events = String::new();
    let mut ret_all = Vec::new();
    for i in 0..10u32 {
        let _ = c.cmd("CYCLE 0");
        c.hold(true);
        thread::sleep(Duration::from_millis(20));
        let _ = c.send(b"JUNKJUNKJUNKJUNKJUNKJUNKJUNKJUNKJUNKJUN\n");
        let t = Instant::now();
        let mut gone = None;
        while t.elapsed() < Duration::from_secs(6) {
            if port_name().is_none() {
                gone = Some(t.elapsed());
                break;
            }
            thread::sleep(Duration::from_millis(5));
        }
        c.close();
        let Some((c2, took)) = open_again(Duration::from_secs(20)) else {
            rep.check(false, format!("C cycle {i}: port found again within 20 s"));
            return None;
        };
        c = c2;
        gone_ms.push(gone.map_or(-1.0, ms));
        back_ms.push(ms(took).round());
        let stray = c.drain(Duration::from_millis(150)).len();
        let ready = ask(&mut c, "START", "READY");
        let cres = ask(&mut c, "CRES", "CRES");
        let cev = lines_until_quiet(&mut c, "CEV", Duration::from_millis(300));
        let r = echo_quiet(&mut c, 4096, 0xC0C0_0000 + u64::from(i), 0);
        let (res, ret) = cres
            .as_deref()
            .map_or((-1, -1), |l| (f(l, "res"), f(l, "ret_us")));
        let junk = ready.as_deref().map_or(-1, |l| f(l, "junk") - junk0);
        let pre = ready.as_deref().map_or(-1, |l| f(l, "pre"));
        gap_max = gap_max.max(cres.as_deref().map_or(0, |l| f(l, "gap")));
        last_events = events_text(&cev);
        ret_all.push(ret);
        let this_ok = res == 2
            && (0..=50_000).contains(&ret)
            && junk == 0
            && pre == 0
            && stray == 0
            && r.mismatches == 0
            && r.missing == 0
            && r.extra == 0
            && r.done;
        worst_ret = worst_ret.max(ret);
        stale += stray + junk.max(0) as usize;
        if !this_ok {
            bad += 1;
        }
        info(format!(
            "C cycle {i}: {} | write returned {} us after the pull-up was released (result {res}: 1 Ok, 2 NotConnected, 9 never pending); port gone after {:.0} ms, found and opened {:.0} ms after that; stray bytes {stray}, device junk {junk}, pre {pre}; echo 4 KiB: {} mismatched, {} missing, {} extra; events: {}",
            if this_ok { "ok" } else { "BAD" },
            ret,
            gone_ms[i as usize],
            back_ms[i as usize],
            r.mismatches,
            r.missing,
            r.extra,
            events_text(&cev)
        ));
    }
    rep.check(
        bad == 0,
        format!(
            "C: 10 soft detaches with a write pending: {bad} bad cycles; the pending write returned NotConnected within 50 ms every time? worst {worst_ret} us; returns {ret_all:?}; stale bytes seen in either direction {stale}"
        ),
    );
    let s1 = ask(&mut c, "STATE", "STATE");
    if let (Some(a), Some(b)) = (&s0, &s1) {
        rep.check(
            f(b, "ups") - f(a, "ups") == 10 && f(b, "downs") - f(a, "downs") == 10,
            format!(
                "C: Stats after 10 detaches: ups +{}, downs +{}, resets +{}, suspends +{} (wanted +10 ups, +10 downs)",
                f(b, "ups") - f(a, "ups"), f(b, "downs") - f(a, "downs"), f(b, "rst") - f(a, "rst"), f(b, "susp") - f(a, "susp")
            ),
        );
    }
    info(format!(
        "C: events of the last cycle (after the pull-up was released): {last_events}"
    ));
    info(format!(
        "E: longest SysTick gap while the link flapped (C cycles, 10 x about 1.5 s): {gap_max} cycles = {:.0} ns",
        gap_max as f64 * ns_per_cycle
    ));
    Some(c)
}

/// D: a way for the computer to make the device's link go away without root.
fn check_d(mut c: Conn, rep: &mut Rep) -> Option<Conn> {
    let tty = port_name()
        .and_then(|n| n.rsplit('/').next().map(str::to_string))
        .unwrap_or_default();
    let dev = |f: &str| {
        std::fs::read_to_string(format!("/sys/class/tty/{tty}/device/../{f}"))
            .map(|s| s.trim().to_string())
            .unwrap_or_default()
    };
    let (bus, num) = (
        dev("busnum").parse::<u32>().unwrap_or(0),
        dev("devnum").parse::<u32>().unwrap_or(0),
    );
    let node = format!("/dev/bus/usb/{bus:03}/{num:03}");
    let mode = std::fs::metadata(&node).map(|m| {
        use std::os::unix::fs::PermissionsExt;
        format!("{:o}", m.permissions().mode() & 0o7777)
    });
    let usb = format!("/sys/class/tty/{tty}/device/..");
    let node_rw = OpenOptions::new().read(true).write(true).open(&node);
    let ctl = OpenOptions::new()
        .write(true)
        .open(format!("{usb}/power/control"));
    let auth = OpenOptions::new()
        .write(true)
        .open(format!("{usb}/authorized"));
    let rm = OpenOptions::new().write(true).open(format!("{usb}/remove"));
    info(format!(
        "D: tried as the ordinary user: open {node} (mode {mode:?}) for USBDEVFS_RESET: {}; write sysfs power/control: {}; authorized: {}; remove: {}",
        node_rw
            .as_ref()
            .map(|_| "ok".to_string())
            .unwrap_or_else(|e| e.to_string()),
        ctl.as_ref()
            .map(|_| "ok".to_string())
            .unwrap_or_else(|e| e.to_string()),
        auth.as_ref()
            .map(|_| "ok".to_string())
            .unwrap_or_else(|e| e.to_string()),
        rm.as_ref()
            .map(|_| "ok".to_string())
            .unwrap_or_else(|e| e.to_string()),
    ));
    let Ok(file) = node_rw else {
        info(
            "D: no way for the computer to make the link go away without root was found; D not run"
                .to_string(),
        );
        return Some(c);
    };
    unsafe extern "C" {
        fn ioctl(fd: i32, request: u64, ...) -> i32;
    }
    let _ = c.cmd("CYCLE 1");
    c.hold(true);
    thread::sleep(Duration::from_millis(1500));
    let t = Instant::now();
    // SAFETY: USBDEVFS_RESET (_IO('U', 20)) takes no argument; the descriptor is open.
    let rc = unsafe { ioctl(file.as_raw_fd(), 0x5514) };
    info(format!(
        "D: USBDEVFS_RESET returned {rc} after {:.0} ms",
        ms(t.elapsed())
    ));
    drop(file);
    c.close();
    let Some((mut c2, took)) = open_again(Duration::from_secs(20)) else {
        rep.check(false, "D: port found again after the reset".to_string());
        return None;
    };
    info(format!(
        "D: port found again {:.0} ms after the reset (open took {:.0} ms)",
        ms(t.elapsed()),
        ms(took)
    ));
    thread::sleep(Duration::from_millis(100));
    let ready = ask(&mut c2, "START", "READY");
    let cres = ask(&mut c2, "CRES", "CRES");
    let cev = lines_until_quiet(&mut c2, "CEV", Duration::from_millis(300));
    let r = echo_quiet(&mut c2, 4096, 0xD0D0_0001, 0);
    rep.check(
        cres.as_deref().is_some_and(|l| f(l, "res") == 2)
            && ready.is_some()
            && r.mismatches == 0
            && r.missing == 0
            && r.extra == 0
            && r.done,
        format!("D: reset by the computer with a write pending: {cres:?}; events {}; echo 4 KiB afterwards: {} mismatched, {} missing, {} extra", events_text(&cev), r.mismatches, r.missing, r.extra),
    );
    Some(c2)
}

pub fn run(rep: &mut Rep) {
    let Some(mut o) = open_port(rep) else { return };
    info(format!("port name {}", o.name));
    if handshake(&mut o.conn, rep).is_none() {
        return;
    }
    let c = o.conn;
    let Some(c) = check_a(c, rep) else { return };
    let Some(mut c) = check_b(c, rep) else { return };
    let ns_per_cycle = check_e(&mut c, rep).unwrap_or(20.0 / 3.0);
    let Some(c) = check_c(c, rep, ns_per_cycle) else {
        return;
    };
    let Some(mut c) = check_d(c, rep) else { return };
    let _ = c.cmd("FINISH");
    match c.line(Instant::now() + Duration::from_secs(5)) {
        Some(l) if l.starts_with("BYE") => info(format!("device: {l}")),
        other => rep.check(false, format!("FINISH: wanted BYE, got {other:?}")),
    }
}

/// F: the physical unplug. Prints banners for the person.
pub fn unplug(rep: &mut Rep) {
    let Some(mut o) = open_port(rep) else { return };
    if handshake(&mut o.conn, rep).is_none() {
        return;
    }
    let mut c = o.conn;
    println!(
        "\n==== UNPLUG TEST: the device writes numbered records; in a moment this program says PULL. ===="
    );
    println!(
        "==== Pull ONLY the board's own USB cable (not the debug probe's), wait for PUT BACK, then plug it back. ====\n"
    );
    let _ = c.cmd("GO");
    let t0 = Instant::now();
    let mut next: u32 = 0;
    let (mut bad, mut gaps) = (0u32, 0u32);
    let mut last_banner = Instant::now() - Duration::from_secs(10);
    let mut pulled_at = None;
    loop {
        if t0.elapsed() > Duration::from_secs(5) && last_banner.elapsed() > Duration::from_secs(4) {
            println!("**** PULL THE BOARD'S USB CABLE NOW ****");
            last_banner = Instant::now();
        }
        let t = c.take(REC, Instant::now() + Duration::from_millis(1000));
        if t.data.len() == REC {
            match rec_check(&t.data) {
                Some(seq) if seq == next => next += 1,
                Some(seq) => {
                    gaps += 1;
                    next = seq + 1;
                }
                None => bad += 1,
            }
            continue;
        }
        if c.read_error.is_some() || port_name().is_none() {
            pulled_at = Some(Instant::now());
            break;
        }
        if t0.elapsed() > Duration::from_secs(600) {
            break;
        }
    }
    rep.check(
        pulled_at.is_some(),
        format!("F: the cable was pulled and the port went away ({next} records received before, {bad} bad, {gaps} gaps)"),
    );
    if pulled_at.is_none() {
        return;
    }
    println!("\n**** CABLE GONE DETECTED. PUT THE BOARD'S USB CABLE BACK NOW ****\n");
    c.close();
    let Some((mut c, took)) = open_again(Duration::from_secs(300)) else {
        rep.check(
            false,
            "F: port found again within 300 s of the cable pull".to_string(),
        );
        return;
    };
    info(format!(
        "F: port back {:.1} s after the pull was seen",
        ms(took) / 1000.0
    ));
    thread::sleep(Duration::from_millis(100));
    let _ = c.port.clear(ClearBuffer::Input);
    let ready = ask(&mut c, "START", "READY");
    let unp = ask(&mut c, "UNP", "UNP");
    let uev = lines_until_quiet(&mut c, "UEV", Duration::from_millis(300));
    let r = echo_quiet(&mut c, 4096, 0xF0F0_0001, 0);
    info(format!("F: device {ready:?}"));
    info(format!(
        "F: device {unp:?}; events after the last good write: {}",
        events_text(&uev)
    ));
    rep.check(
        unp.as_deref().is_some_and(|l| f(l, "went_down") == 1)
            && r.mismatches == 0
            && r.missing == 0
            && r.extra == 0
            && r.done,
        format!("F: the pending write ended with NotConnected, and an echo of 4 KiB in the new session passes ({} mismatched, {} missing, {} extra)", r.mismatches, r.missing, r.extra),
    );
    let _ = c.cmd("FINISH");
    let _ = c.line(Instant::now() + Duration::from_secs(5));
}
