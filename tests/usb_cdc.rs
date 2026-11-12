//! Host tests of the crate against a fake driver. The link state is driven through the
//! `_test-hooks` feature, which calls the same `Handler` methods the USB device calls.

mod fake;

use std::future::Future;
use std::pin::{Pin, pin};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::task::{Context, Poll, Wake, Waker};

use embedded_io_async::{Error as _, ErrorKind, Read, Write};
use proptest::prelude::*;
use usb_cdc::{Config, Error, Event, Link};

use fake::{Rig, rig, rig_up};

// ---------------------------------------------------------------- helpers

struct Flag(AtomicBool);

impl Wake for Flag {
    fn wake(self: Arc<Self>) {
        self.0.store(true, Ordering::SeqCst);
    }
}

/// A waker that remembers whether it was woken.
struct Probe {
    flag: Arc<Flag>,
    waker: Waker,
}

impl Probe {
    fn new() -> Self {
        let flag = Arc::new(Flag(AtomicBool::new(false)));
        Self {
            waker: Waker::from(flag.clone()),
            flag,
        }
    }

    fn woken(&self) -> bool {
        self.flag.0.load(Ordering::SeqCst)
    }

    fn poll<F: Future + ?Sized>(&self, fut: Pin<&mut F>) -> Poll<F::Output> {
        self.flag.0.store(false, Ordering::SeqCst);
        fut.poll(&mut Context::from_waker(&self.waker))
    }
}

/// Polls to completion; a future that stays pending for 10 000 polls is a test failure.
fn block_on<F: Future>(fut: F) -> F::Output {
    let probe = Probe::new();
    let mut fut = pin!(fut);
    for _ in 0..10_000 {
        if let Poll::Ready(v) = probe.poll(fut.as_mut()) {
            return v;
        }
    }
    panic!("future still pending after 10000 polls");
}

/// Polls `fut` at most `polls` times and then drops it. `Some` if it finished first.
fn drop_after<F: Future>(fut: F, polls: usize) -> Option<F::Output> {
    let probe = Probe::new();
    let mut fut = pin!(fut);
    for _ in 0..polls {
        if let Poll::Ready(v) = probe.poll(fut.as_mut()) {
            return Some(v);
        }
    }
    None
}

fn ready<F: Future>(fut: F) -> F::Output {
    let probe = Probe::new();
    match probe.poll(pin!(fut).as_mut()) {
        Poll::Ready(v) => v,
        Poll::Pending => panic!("returned Pending where an answer at once is required"),
    }
}

fn counters(link: &Link<'_>) -> [u32; 5] {
    let s = link.stats();
    [s.ups, s.downs, s.resets, s.suspends, s.zlp_skipped]
}

fn pattern(len: usize) -> Vec<u8> {
    (0..len).map(|i| (i % 251) as u8 ^ 0x5a).collect()
}

/// Every cause that must bring a link that is up down.
const DOWN_CAUSES: [Event; 4] = [
    Event::Reset,
    Event::Configured(false),
    Event::Suspended(true),
    Event::Enabled(false),
];

// ---------------------------------------------------------------- 1. link state

#[test]
fn link_is_up_only_when_configured_and_not_suspended() {
    let r = rig(Config::new());
    assert!(!r.link.is_up(), "a new link is down");
    r.link.inject(Event::Suspended(false));
    r.link.inject(Event::Addressed(5));
    assert!(!r.link.is_up(), "addressed is not configured");
    r.link.inject(Event::Configured(true));
    assert!(r.link.is_up());
    r.link.inject(Event::Suspended(true));
    assert!(!r.link.is_up(), "suspended is down");
    r.link.inject(Event::Configured(false));
    r.link.inject(Event::Suspended(false));
    assert!(!r.link.is_up(), "resumed but not configured is down");
    r.link.inject(Event::Configured(true));
    assert!(r.link.is_up());
}

#[test]
fn each_cause_brings_an_up_link_down() {
    for cause in DOWN_CAUSES {
        let r = rig_up(Config::new());
        r.link.inject(cause);
        assert!(!r.link.is_up(), "{cause:?} must bring the link down");
    }
}

#[test]
fn reset_clears_a_suspend_because_the_device_does() {
    let r = rig_up(Config::new());
    r.link.inject(Event::Suspended(true));
    r.link.inject(Event::Reset);
    r.link.inject(Event::Configured(true));
    assert!(
        r.link.is_up(),
        "embassy-usb clears its suspended flag on reset without calling suspended(false)"
    );
}

#[test]
fn stats_count_ups_downs_resets_and_suspends() {
    let r = rig(Config::new());
    assert_eq!(counters(&r.link), [0, 0, 0, 0, 0]);
    r.link.inject(Event::Configured(true));
    assert_eq!(counters(&r.link), [1, 0, 0, 0, 0]);
    r.link.inject(Event::Configured(true));
    assert_eq!(
        counters(&r.link),
        [1, 0, 0, 0, 0],
        "no transition, no count"
    );
    r.link.inject(Event::Suspended(true));
    assert_eq!(counters(&r.link), [1, 1, 0, 1, 0]);
    r.link.inject(Event::Suspended(true));
    assert_eq!(
        counters(&r.link),
        [1, 1, 0, 1, 0],
        "a second suspend report is not a second suspend"
    );
    r.link.inject(Event::Suspended(false));
    assert_eq!(counters(&r.link), [2, 1, 0, 1, 0]);
    r.link.inject(Event::Reset);
    assert_eq!(counters(&r.link), [2, 2, 1, 1, 0]);
    r.link.inject(Event::Reset);
    assert_eq!(
        counters(&r.link),
        [2, 2, 2, 1, 0],
        "a reset while down is a reset, not a down"
    );
    r.link.inject(Event::Configured(true));
    r.link.inject(Event::Configured(false));
    assert_eq!(counters(&r.link), [3, 3, 2, 1, 0]);
    r.link.inject(Event::Configured(true));
    r.link.inject(Event::Enabled(false));
    assert_eq!(counters(&r.link), [4, 4, 2, 1, 0]);
}

#[test]
fn wait_up_and_wait_down_complete_at_once_when_already_there() {
    let r = rig(Config::new());
    ready(r.link.wait_down());
    r.link.inject(Event::Configured(true));
    ready(r.link.wait_up());
}

#[test]
fn wait_up_completes_on_the_transition() {
    let r = rig(Config::new());
    let probe = Probe::new();
    let mut wait = pin!(r.link.wait_up());
    assert!(probe.poll(wait.as_mut()).is_pending());
    r.link.inject(Event::Configured(true));
    assert!(probe.woken(), "the transition wakes the waiter");
    assert!(probe.poll(wait.as_mut()).is_ready());
}

#[test]
fn wait_down_completes_on_each_cause() {
    for cause in DOWN_CAUSES {
        let r = rig_up(Config::new());
        let probe = Probe::new();
        let mut wait = pin!(r.link.wait_down());
        assert!(probe.poll(wait.as_mut()).is_pending());
        r.link.inject(cause);
        assert!(probe.woken(), "{cause:?} wakes the waiter");
        assert!(probe.poll(wait.as_mut()).is_ready());
    }
}

#[test]
fn several_waiters_are_all_woken() {
    let r = rig(Config::new());
    let (p1, p2) = (Probe::new(), Probe::new());
    let (mut w1, mut w2) = (pin!(r.link.wait_up()), pin!(r.link.wait_up()));
    assert!(p1.poll(w1.as_mut()).is_pending());
    assert!(p2.poll(w2.as_mut()).is_pending());
    r.link.inject(Event::Configured(true));
    assert!(p1.woken() && p2.woken());
}

// ---------------------------------------------------------------- 2. down means NotConnected, at once

#[test]
fn while_down_every_call_fails_at_once_and_never_pends() {
    let mut r = rig(Config::new());
    r.bulk_in.set_host_reads(false);
    assert_eq!(ready(r.tx.write(b"abc")), Err(Error::NotConnected));
    assert_eq!(ready(r.tx.write(b"")), Err(Error::NotConnected));
    assert_eq!(ready(r.tx.flush()), Err(Error::NotConnected));
    let mut buf = [0u8; 8];
    assert_eq!(ready(r.rx.read(&mut buf)), Err(Error::NotConnected));
    assert_eq!(ready(r.rx.read(&mut [])), Err(Error::NotConnected));
    r.bulk_out.feed(b"queued"); // data waiting does not make a down link readable
    assert_eq!(ready(r.rx.read(&mut buf)), Err(Error::NotConnected));
    assert!(r.bulk_in.sent().is_empty());
}

#[test]
fn every_down_cause_makes_the_next_calls_fail_at_once() {
    for cause in DOWN_CAUSES {
        let mut r = rig_up(Config::new());
        r.link.inject(cause);
        assert_eq!(
            ready(r.tx.write(b"abc")),
            Err(Error::NotConnected),
            "{cause:?}"
        );
        assert_eq!(
            ready(r.rx.read(&mut [0u8; 4])),
            Err(Error::NotConnected),
            "{cause:?}"
        );
    }
}

#[test]
fn a_pending_write_ends_with_not_connected_when_the_link_goes_down() {
    for cause in DOWN_CAUSES {
        let mut r = rig_up(Config::new());
        r.bulk_in.set_host_reads(false);
        let probe = Probe::new();
        let mut write = pin!(r.tx.write(b"hello"));
        assert!(probe.poll(write.as_mut()).is_pending());
        r.link.inject(cause);
        assert!(
            probe.woken(),
            "{cause:?}: the link change must wake the pending write"
        );
        assert_eq!(
            probe.poll(write.as_mut()),
            Poll::Ready(Err(Error::NotConnected)),
            "{cause:?}"
        );
        assert!(r.bulk_in.sent().is_empty());
    }
}

#[test]
fn a_pending_read_ends_with_not_connected_when_the_link_goes_down() {
    for cause in DOWN_CAUSES {
        let mut r = rig_up(Config::new());
        let probe = Probe::new();
        let mut buf = [0u8; 16];
        let mut read = pin!(r.rx.read(&mut buf));
        assert!(probe.poll(read.as_mut()).is_pending());
        r.link.inject(cause);
        assert!(
            probe.woken(),
            "{cause:?}: the link change must wake the pending read"
        );
        assert_eq!(
            probe.poll(read.as_mut()),
            Poll::Ready(Err(Error::NotConnected)),
            "{cause:?}"
        );
    }
}

#[test]
fn a_pending_call_still_fails_when_the_link_is_back_before_it_is_polled() {
    let mut r = rig_up(Config::new());
    r.bulk_in.set_host_reads(false);
    let probe = Probe::new();
    {
        let mut write = pin!(r.tx.write(b"hello"));
        assert!(probe.poll(write.as_mut()).is_pending());
        r.link.inject(Event::Reset);
        r.link.inject(Event::Configured(true));
        assert!(r.link.is_up());
        assert_eq!(
            probe.poll(write.as_mut()),
            Poll::Ready(Err(Error::NotConnected))
        );
    }
    r.bulk_in.set_host_reads(true);
    assert_eq!(
        block_on(r.tx.write(b"hello")),
        Ok(5),
        "the next call works in the new session"
    );
    assert_eq!(r.bulk_in.sent_bytes(), b"hello");
    let mut buf = [0u8; 16];
    {
        let mut read = pin!(r.rx.read(&mut buf));
        assert!(probe.poll(read.as_mut()).is_pending());
        r.link.inject(Event::Suspended(true));
        r.link.inject(Event::Suspended(false));
        assert_eq!(
            probe.poll(read.as_mut()),
            Poll::Ready(Err(Error::NotConnected))
        );
    }
}

#[test]
fn the_error_is_embedded_io_not_connected() {
    assert_eq!(Error::NotConnected.kind(), ErrorKind::NotConnected);
    assert_eq!(Error::NotConnected.to_string(), "USB link is not connected");
}

// ---------------------------------------------------------------- 3. write: one short packet per call

#[test]
fn write_sends_one_packet_of_at_most_63_bytes() {
    let mut r = rig_up(Config::new());
    assert_eq!(r.bulk_in.max_packet_size(), 64);
    let data = pattern(200);
    assert_eq!(block_on(r.tx.write(&data)), Ok(63));
    assert_eq!(r.bulk_in.sent(), vec![data[..63].to_vec()]);
    assert_eq!(block_on(r.tx.write(&data[..10])), Ok(10));
    assert_eq!(block_on(r.tx.write(&data[..63])), Ok(63));
    assert_eq!(
        block_on(r.tx.write(&data[..64])),
        Ok(63),
        "64 bytes would be a full packet"
    );
    assert!(r.bulk_in.sent().iter().all(|p| (1..=63).contains(&p.len())));
}

#[test]
fn the_packet_is_one_less_than_the_configured_max_packet_size() {
    for (asked, packet, chunk) in [
        (8, 8, 7),
        (16, 16, 15),
        (32, 32, 31),
        (64, 64, 63),
        (100, 64, 63),
        (40, 32, 31),
        (0, 8, 7),
    ] {
        let mut r = rig_up(Config::new().with_max_packet_size(asked));
        assert_eq!(r.bulk_in.max_packet_size(), packet, "asked {asked}");
        assert_eq!(r.bulk_out.max_packet_size(), packet);
        assert_eq!(
            block_on(r.tx.write(&pattern(200))),
            Ok(chunk),
            "asked {asked}"
        );
        assert_eq!(r.bulk_in.sent()[0].len(), chunk);
    }
}

#[test]
fn a_zero_length_write_returns_zero_and_sends_nothing() {
    let mut r = rig_up(Config::new());
    assert_eq!(ready(r.tx.write(b"")), Ok(0));
    assert!(r.bulk_in.sent().is_empty());
    ready(r.tx.write_all(b"")).unwrap();
    assert!(r.bulk_in.sent().is_empty());
}

#[test]
fn flush_does_nothing_and_succeeds_while_up() {
    let mut r = rig_up(Config::new());
    r.bulk_in.set_host_reads(false);
    assert_eq!(ready(r.tx.flush()), Ok(()));
    assert!(r.bulk_in.sent().is_empty());
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn every_in_packet_is_1_to_63_bytes_and_the_bytes_are_unchanged(
        data in proptest::collection::vec(any::<u8>(), 0..=4096),
        yields in 0usize..3,
    ) {
        let mut r = rig_up(Config::new());
        r.bulk_in.set_yields(yields);
        block_on(r.tx.write_all(&data)).unwrap();
        let sent = r.bulk_in.sent();
        prop_assert!(sent.iter().all(|p| (1..=63).contains(&p.len())), "packet lengths {:?}", sent.iter().map(Vec::len).collect::<Vec<_>>());
        prop_assert_eq!(sent.concat(), data.clone());
        prop_assert_eq!(sent.len(), data.len().div_ceil(63));
    }

    #[test]
    fn small_packet_sizes_keep_the_same_promise(
        data in proptest::collection::vec(any::<u8>(), 0..=600),
        size in prop::sample::select(vec![8u16, 16, 32, 64]),
    ) {
        let mut r = rig_up(Config::new().with_max_packet_size(size));
        block_on(r.tx.write_all(&data)).unwrap();
        let sent = r.bulk_in.sent();
        prop_assert!(sent.iter().all(|p| (1..usize::from(size)).contains(&p.len())));
        prop_assert_eq!(sent.concat(), data);
    }
}

// ---------------------------------------------------------------- 4. read

#[test]
fn read_returns_bytes_from_a_received_packet() {
    let mut r = rig_up(Config::new());
    r.bulk_out.feed(b"hello");
    let mut buf = [0u8; 64];
    assert_eq!(block_on(r.rx.read(&mut buf)), Ok(5));
    assert_eq!(&buf[..5], b"hello");
}

#[test]
fn a_packet_larger_than_the_buffer_is_served_over_several_calls_without_loss() {
    for step in [1usize, 2, 7, 10, 63] {
        let mut r = rig_up(Config::new());
        let a = pattern(64);
        let b = pattern(37);
        r.bulk_out.feed(&a);
        r.bulk_out.feed(&b);
        let mut got = Vec::new();
        let mut buf = vec![0u8; step];
        while got.len() < a.len() + b.len() {
            let n = block_on(r.rx.read(&mut buf)).unwrap();
            assert!((1..=step).contains(&n));
            got.extend_from_slice(&buf[..n]);
        }
        assert_eq!(got, [a, b].concat(), "step {step}");
        assert_eq!(r.bulk_out.unread(), 0);
    }
}

#[test]
fn zero_length_packets_are_skipped_and_counted() {
    let mut r = rig_up(Config::new());
    for p in [&b""[..], b"", b"abc", b"", b"de"] {
        r.bulk_out.feed(p);
    }
    let mut buf = [0u8; 16];
    assert_eq!(block_on(r.rx.read(&mut buf)), Ok(3));
    assert_eq!(&buf[..3], b"abc");
    assert_eq!(r.link.stats().zlp_skipped, 2);
    assert_eq!(block_on(r.rx.read(&mut buf)), Ok(2));
    assert_eq!(&buf[..2], b"de");
    assert_eq!(r.link.stats().zlp_skipped, 3);
}

#[test]
fn a_zero_length_packet_alone_leaves_read_pending_not_ok_zero() {
    let mut r = rig_up(Config::new());
    r.bulk_out.feed(b"");
    let probe = Probe::new();
    let mut buf = [0u8; 16];
    let mut read = pin!(r.rx.read(&mut buf));
    assert!(probe.poll(read.as_mut()).is_pending());
    assert_eq!(r.link.stats().zlp_skipped, 1);
    r.bulk_out.feed(b"x");
    assert_eq!(probe.poll(read.as_mut()), Poll::Ready(Ok(1)));
}

#[test]
fn an_empty_buffer_reads_zero_without_touching_the_endpoint() {
    let mut r = rig_up(Config::new());
    r.bulk_out.feed(b"abc");
    assert_eq!(ready(r.rx.read(&mut [])), Ok(0));
    assert_eq!(r.bulk_out.unread(), 1);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn read_never_returns_zero_and_loses_no_byte(
        packets in proptest::collection::vec(proptest::collection::vec(any::<u8>(), 0..=64), 1..24),
        steps in proptest::collection::vec(1usize..100, 1..8),
        yields in 0usize..3,
    ) {
        let mut r = rig_up(Config::new());
        r.bulk_out.set_yields(yields);
        for p in &packets {
            r.bulk_out.feed(p);
        }
        let want = packets.concat();
        let mut got = Vec::new();
        let mut i = 0;
        while got.len() < want.len() {
            let mut buf = vec![0u8; steps[i % steps.len()]];
            i += 1;
            let n = block_on(r.rx.read(&mut buf)).unwrap();
            prop_assert!(n >= 1 && n <= buf.len());
            got.extend_from_slice(&buf[..n]);
        }
        prop_assert_eq!(got, want);
        // Packets after the last data packet may still be queued; the ones before it were skipped.
        let skipped = packets
            .iter()
            .rposition(|p| !p.is_empty())
            .map_or(0, |last| packets[..=last].iter().filter(|p| p.is_empty()).count() as u32);
        prop_assert_eq!(r.link.stats().zlp_skipped, skipped);
    }
}

#[test]
fn a_partly_served_packet_is_thrown_away_when_the_link_goes_down() {
    for cause in DOWN_CAUSES {
        let mut r = rig_up(Config::new());
        r.bulk_out.feed(&pattern(64));
        let mut buf = [0u8; 10];
        assert_eq!(block_on(r.rx.read(&mut buf)), Ok(10));
        r.link.inject(cause);
        assert_eq!(
            ready(r.rx.read(&mut buf)),
            Err(Error::NotConnected),
            "{cause:?}"
        );
        r.link.inject(Event::Reset);
        r.link.inject(Event::Configured(true));
        r.bulk_out.feed(b"new");
        let mut big = [0u8; 100];
        assert_eq!(
            block_on(r.rx.read(&mut big)),
            Ok(3),
            "{cause:?}: the 54 old bytes must be gone"
        );
        assert_eq!(&big[..3], b"new");
    }
}

#[test]
fn a_partly_served_packet_is_thrown_away_even_if_the_link_came_back_before_the_next_read() {
    let mut r = rig_up(Config::new());
    r.bulk_out.feed(&pattern(64));
    let mut buf = [0u8; 10];
    assert_eq!(block_on(r.rx.read(&mut buf)), Ok(10));
    r.link.inject(Event::Suspended(true));
    r.link.inject(Event::Suspended(false));
    assert!(r.link.is_up());
    r.bulk_out.feed(b"new");
    let mut big = [0u8; 100];
    assert_eq!(block_on(r.rx.read(&mut big)), Ok(3));
    assert_eq!(&big[..3], b"new");
}

// ---------------------------------------------------------------- 5. cancel safety

const AWAIT_POINTS: usize = 8;

#[test]
fn dropping_a_write_at_every_await_point_sends_nothing_or_a_whole_packet() {
    let data = pattern(150);
    for k in 0..=AWAIT_POINTS {
        for host_reads in [true, false] {
            let mut r = rig_up(Config::new());
            r.bulk_in.set_yields(3);
            r.bulk_in.set_host_reads(host_reads);
            let first = drop_after(r.tx.write(&data), k);
            if first.is_none() {
                assert!(
                    r.bulk_in.sent().is_empty(),
                    "k={k}: a dropped write sent something"
                );
            }
            let skip = match first {
                Some(Ok(n)) => n,
                Some(Err(e)) => panic!("k={k}: {e}"),
                None => 0,
            };
            r.bulk_in.set_host_reads(true);
            block_on(r.tx.write_all(&data[skip..])).unwrap();
            let sent = r.bulk_in.sent();
            assert!(sent.iter().all(|p| (1..=63).contains(&p.len())), "k={k}");
            assert_eq!(sent.concat(), data, "k={k} host_reads={host_reads}");
        }
    }
}

#[test]
fn dropping_a_read_at_every_await_point_loses_no_byte() {
    let a = pattern(64);
    let b = pattern(30);
    let want = [a.clone(), b.clone()].concat();
    for k1 in 0..=AWAIT_POINTS {
        for k2 in 0..=AWAIT_POINTS {
            let mut r = rig_up(Config::new());
            r.bulk_out.set_yields(3);
            for p in [&b""[..], &a, b"", &b] {
                r.bulk_out.feed(p);
            }
            let mut got = Vec::new();
            for k in [k1, k2] {
                let mut buf = [0u8; 20];
                if let Some(res) = drop_after(r.rx.read(&mut buf), k) {
                    let n = res.unwrap();
                    assert!(n >= 1);
                    got.extend_from_slice(&buf[..n]);
                }
            }
            while got.len() < want.len() {
                let mut buf = [0u8; 20];
                let n = block_on(r.rx.read(&mut buf)).unwrap();
                got.extend_from_slice(&buf[..n]);
            }
            assert_eq!(got, want, "k1={k1} k2={k2}");
            assert_eq!(r.link.stats().zlp_skipped, 2, "k1={k1} k2={k2}");
        }
    }
}

// ---------------------------------------------------------------- 6. a computer that is not reading

#[test]
fn write_waits_while_the_computer_is_not_reading_and_completes_when_it_does() {
    let mut r = rig_up(Config::new());
    r.bulk_in.set_host_reads(false);
    let probe = Probe::new();
    let data = pattern(100);
    {
        let mut write = pin!(r.tx.write(&data));
        assert!(probe.poll(write.as_mut()).is_pending());
        assert!(probe.poll(write.as_mut()).is_pending());
        assert!(r.bulk_in.sent().is_empty());
        assert!(
            r.link.is_up(),
            "a computer that is not reading is not a link fault"
        );
        r.bulk_in.set_host_reads(true);
        assert!(
            probe.woken(),
            "the endpoint accepting the packet wakes the writer"
        );
        assert_eq!(probe.poll(write.as_mut()), Poll::Ready(Ok(63)));
    }
    assert_eq!(r.bulk_in.sent(), vec![data[..63].to_vec()]);
    assert_eq!(counters(&r.link), [1, 0, 0, 0, 0]);
}

#[test]
fn a_waiting_write_does_not_stop_reading() {
    let mut r = rig_up(Config::new());
    r.bulk_in.set_host_reads(false);
    r.bulk_out.feed(b"ping");
    let probe = Probe::new();
    let mut write = pin!(r.tx.write(b"pong"));
    assert!(probe.poll(write.as_mut()).is_pending());
    let mut buf = [0u8; 8];
    assert_eq!(block_on(r.rx.read(&mut buf)), Ok(4));
}

// ---------------------------------------------------------------- 7. DTR

#[test]
fn dtr_is_reported_and_never_gates_data() {
    let mut r = rig_up(Config::new());
    assert!(!r.link.dtr(), "no DTR yet, and the data still flows");
    assert_eq!(block_on(r.tx.write(b"abc")), Ok(3));
    r.bulk_out.feed(b"xyz");
    let mut buf = [0u8; 8];
    assert_eq!(block_on(r.rx.read(&mut buf)), Ok(3));
    assert_eq!(r.bulk_in.sent_bytes(), b"abc");
}

// ---------------------------------------------------------------- the handler is on the builder

/// Everything above drives the handler by hand. This one lets the real `UsbDevice` call it, by
/// polling `run()` against a scripted bus and control pipe (no task, no timer).
#[test]
fn the_real_device_reaches_the_crates_handler_and_sets_dtr() {
    fn settle(r: &mut Rig, probe: &Probe) {
        let mut run = pin!(r.device.run());
        for _ in 0..4 {
            assert!(probe.poll(run.as_mut()).is_pending());
        }
    }
    const SET_ADDRESS: [u8; 8] = [0x00, 0x05, 0x07, 0, 0, 0, 0, 0];
    const SET_CONFIGURATION_1: [u8; 8] = [0x00, 0x09, 0x01, 0, 0, 0, 0, 0];
    const SET_CONFIGURATION_0: [u8; 8] = [0x00, 0x09, 0x00, 0, 0, 0, 0, 0];
    const DTR_ON: [u8; 8] = [0x21, 0x22, 0x01, 0, 0, 0, 0, 0];
    const DTR_OFF: [u8; 8] = [0x21, 0x22, 0x00, 0, 0, 0, 0, 0];

    let mut r = rig(Config::new());
    let probe = Probe::new();
    r.script.bus_event(embassy_usb_driver::Event::PowerDetected);
    settle(&mut r, &probe);
    r.script.bus_event(embassy_usb_driver::Event::Reset);
    settle(&mut r, &probe);
    r.script.setup(SET_ADDRESS);
    settle(&mut r, &probe);
    assert!(!r.link.is_up());
    r.script.setup(SET_CONFIGURATION_1);
    settle(&mut r, &probe);
    assert!(
        r.link.is_up(),
        "SET_CONFIGURATION(1) through the real device"
    );
    assert_eq!(r.script.0.borrow().rejected, 0);
    assert!(!r.link.dtr());
    r.script.setup(DTR_ON);
    settle(&mut r, &probe);
    assert!(r.link.dtr());
    assert_eq!(block_on(r.tx.write(b"abc")), Ok(3));
    r.script.setup(DTR_OFF);
    settle(&mut r, &probe);
    assert!(!r.link.dtr());
    assert!(r.link.is_up(), "DTR low does not take the link down");
    assert_eq!(block_on(r.tx.write(b"def")), Ok(3));

    // Suspend ends run_until_suspend; run() goes on waiting for the resume.
    r.script.bus_event(embassy_usb_driver::Event::Suspend);
    settle(&mut r, &probe);
    assert!(!r.link.is_up());
    r.script.bus_event(embassy_usb_driver::Event::Resume);
    settle(&mut r, &probe);
    assert!(r.link.is_up());
    r.script.setup(SET_CONFIGURATION_0);
    settle(&mut r, &probe);
    assert!(!r.link.is_up());
    r.script.setup(SET_CONFIGURATION_1);
    settle(&mut r, &probe);
    assert!(r.link.is_up());
    r.script.bus_event(embassy_usb_driver::Event::Reset);
    settle(&mut r, &probe);
    assert!(!r.link.is_up());
    assert_eq!(counters(&r.link), [3, 3, 2, 1, 0]);
}
