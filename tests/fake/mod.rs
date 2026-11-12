//! A fake `embassy_usb_driver::Driver`: endpoints the test can feed, watch and make pending, and a
//! scripted bus and control pipe. Like embassy-rp 0.10 on a bus reset, its endpoint futures never
//! end by themselves.
#![allow(dead_code)]

use std::cell::RefCell;
use std::collections::VecDeque;
use std::future::{Future, poll_fn};
use std::pin::Pin;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

use embassy_usb::{Builder, UsbDevice};
use embassy_usb_driver::{
    Bus, ControlPipe, Direction, Driver, Endpoint, EndpointAddress, EndpointAllocError,
    EndpointError, EndpointIn, EndpointInfo, EndpointOut, EndpointType, Event, Unsupported,
};
use usb_cdc::{Config, Link, Rx, Tx, UsbCdc};

/// One bulk endpoint as the test sees it.
pub struct Pipe {
    /// IN: the packets the device has put on the wire, in order.
    pub sent: Vec<Vec<u8>>,
    /// OUT: the packets the computer has sent and the device has not read yet.
    pub incoming: VecDeque<Vec<u8>>,
    /// IN: false means the computer is not reading, so `write` stays pending.
    pub host_reads: bool,
    /// Every endpoint operation returns `Pending` this many times before it does anything.
    pub yields: usize,
    pub waker: Option<Waker>,
    pub max_packet_size: u16,
}

#[derive(Clone)]
pub struct Handle(pub Rc<RefCell<Pipe>>);

impl Handle {
    fn new() -> Self {
        Self(Rc::new(RefCell::new(Pipe {
            sent: Vec::new(),
            incoming: VecDeque::new(),
            host_reads: true,
            yields: 0,
            waker: None,
            max_packet_size: 0,
        })))
    }

    fn wake(&self) {
        let waker = self.0.borrow_mut().waker.take();
        if let Some(w) = waker {
            w.wake();
        }
    }

    /// The computer sends a packet (may be empty).
    pub fn feed(&self, packet: &[u8]) {
        self.0.borrow_mut().incoming.push_back(packet.to_vec());
        self.wake();
    }

    pub fn set_host_reads(&self, reads: bool) {
        self.0.borrow_mut().host_reads = reads;
        self.wake();
    }

    pub fn set_yields(&self, yields: usize) {
        self.0.borrow_mut().yields = yields;
    }

    pub fn sent(&self) -> Vec<Vec<u8>> {
        self.0.borrow().sent.clone()
    }

    pub fn sent_bytes(&self) -> Vec<u8> {
        self.0.borrow().sent.concat()
    }

    pub fn unread(&self) -> usize {
        self.0.borrow().incoming.len()
    }

    pub fn max_packet_size(&self) -> u16 {
        self.0.borrow().max_packet_size
    }
}

async fn yield_n(n: usize) {
    for _ in 0..n {
        let mut first = true;
        poll_fn(|cx| {
            if first {
                first = false;
                cx.waker().wake_by_ref();
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
}

pub struct FakeOut {
    pipe: Handle,
    info: EndpointInfo,
}

pub struct FakeIn {
    pipe: Handle,
    info: EndpointInfo,
}

impl Endpoint for FakeOut {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }
    async fn wait_enabled(&mut self) {}
}

impl Endpoint for FakeIn {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }
    async fn wait_enabled(&mut self) {}
}

impl EndpointOut for FakeOut {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let yields = self.pipe.0.borrow().yields;
        yield_n(yields).await;
        let pipe = self.pipe.clone();
        poll_fn(|cx| {
            let mut p = pipe.0.borrow_mut();
            match p.incoming.front() {
                None => {
                    p.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
                Some(packet) if packet.len() > buf.len() => {
                    Poll::Ready(Err(EndpointError::BufferOverflow))
                }
                Some(_) => {
                    let packet = p.incoming.pop_front().unwrap();
                    buf[..packet.len()].copy_from_slice(&packet);
                    Poll::Ready(Ok(packet.len()))
                }
            }
        })
        .await
    }
}

impl EndpointIn for FakeIn {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > usize::from(self.info.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        let yields = self.pipe.0.borrow().yields;
        yield_n(yields).await;
        let pipe = self.pipe.clone();
        poll_fn(|cx| {
            let mut p = pipe.0.borrow_mut();
            if p.host_reads {
                p.sent.push(buf.to_vec());
                Poll::Ready(Ok(()))
            } else {
                p.waker = Some(cx.waker().clone());
                Poll::Pending
            }
        })
        .await
    }
}

/// What the scripted bus and control pipe will deliver to a running `UsbDevice`.
#[derive(Default)]
pub struct Script {
    pub events: VecDeque<Event>,
    pub setups: VecDeque<[u8; 8]>,
    pub bus_waker: Option<Waker>,
    pub setup_waker: Option<Waker>,
    pub accepted: usize,
    pub rejected: usize,
}

#[derive(Clone, Default)]
pub struct ScriptHandle(pub Rc<RefCell<Script>>);

impl ScriptHandle {
    pub fn bus_event(&self, event: Event) {
        let waker = {
            let mut s = self.0.borrow_mut();
            s.events.push_back(event);
            s.bus_waker.take()
        };
        if let Some(w) = waker {
            w.wake();
        }
    }

    pub fn setup(&self, packet: [u8; 8]) {
        let waker = {
            let mut s = self.0.borrow_mut();
            s.setups.push_back(packet);
            s.setup_waker.take()
        };
        if let Some(w) = waker {
            w.wake();
        }
    }
}

pub struct FakeBus(ScriptHandle);
pub struct FakeControl(ScriptHandle);

impl Bus for FakeBus {
    async fn enable(&mut self) {}
    async fn disable(&mut self) {}
    async fn poll(&mut self) -> Event {
        let script = self.0.clone();
        poll_fn(|cx| {
            let mut s = script.0.borrow_mut();
            match s.events.pop_front() {
                Some(e) => Poll::Ready(e),
                None => {
                    s.bus_waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            }
        })
        .await
    }
    fn endpoint_set_enabled(&mut self, _ep_addr: EndpointAddress, _enabled: bool) {}
    fn endpoint_set_stalled(&mut self, _ep_addr: EndpointAddress, _stalled: bool) {}
    fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
        false
    }
    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

impl ControlPipe for FakeControl {
    fn max_packet_size(&self) -> usize {
        64
    }
    async fn setup(&mut self) -> [u8; 8] {
        let script = self.0.clone();
        poll_fn(|cx| {
            let mut s = script.0.borrow_mut();
            match s.setups.pop_front() {
                Some(p) => Poll::Ready(p),
                None => {
                    s.setup_waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            }
        })
        .await
    }
    async fn data_out(
        &mut self,
        _buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        Ok(0)
    }
    async fn data_in(
        &mut self,
        _data: &[u8],
        _first: bool,
        _last: bool,
    ) -> Result<(), EndpointError> {
        Ok(())
    }
    async fn accept(&mut self) {
        self.0.0.borrow_mut().accepted += 1;
    }
    async fn reject(&mut self) {
        self.0.0.borrow_mut().rejected += 1;
    }
    async fn accept_set_address(&mut self, _addr: u8) {
        self.0.0.borrow_mut().accepted += 1;
    }
}

pub struct FakeDriver {
    bulk_in: Handle,
    bulk_out: Handle,
    script: ScriptHandle,
}

impl<'a> Driver<'a> for FakeDriver {
    type EndpointOut = FakeOut;
    type EndpointIn = FakeIn;
    type ControlPipe = FakeControl;
    type Bus = FakeBus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        _ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<FakeOut, EndpointAllocError> {
        assert_eq!(ep_type, EndpointType::Bulk);
        self.bulk_out.0.borrow_mut().max_packet_size = max_packet_size;
        Ok(FakeOut {
            pipe: self.bulk_out.clone(),
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(1, Direction::Out),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        _ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<FakeIn, EndpointAllocError> {
        let pipe = if ep_type == EndpointType::Bulk {
            self.bulk_in.0.borrow_mut().max_packet_size = max_packet_size;
            self.bulk_in.clone()
        } else {
            Handle::new()
        };
        let index = if ep_type == EndpointType::Bulk { 2 } else { 3 };
        Ok(FakeIn {
            pipe,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, Direction::In),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }

    fn start(self, _control_max_packet_size: u16) -> (FakeBus, FakeControl) {
        (FakeBus(self.script.clone()), FakeControl(self.script))
    }
}

/// A CDC function built through a real `embassy_usb::Builder` on the fake driver. The device is
/// built but its `run()` is not polled unless a test does so on purpose.
pub struct Rig {
    pub tx: Tx<'static, FakeDriver>,
    pub rx: Rx<'static, FakeDriver>,
    pub link: Link<'static>,
    pub bulk_in: Handle,
    pub bulk_out: Handle,
    pub script: ScriptHandle,
    pub device: UsbDevice<'static, FakeDriver>,
}

pub fn rig(config: Config) -> Rig {
    let bulk_in = Handle::new();
    let bulk_out = Handle::new();
    let script = ScriptHandle::default();
    let driver = FakeDriver {
        bulk_in: bulk_in.clone(),
        bulk_out: bulk_out.clone(),
        script: script.clone(),
    };
    fn buf(n: usize) -> &'static mut [u8] {
        Box::leak(vec![0u8; n].into_boxed_slice())
    }
    let mut builder = Builder::new(
        driver,
        embassy_usb::Config::new(0x1209, 0x0001),
        buf(256),
        buf(256),
        buf(256),
        buf(64),
    );
    let state = Box::leak(Box::new(usb_cdc::State::new()));
    let (tx, rx, link) = UsbCdc::new(&mut builder, state, config).split();
    let device = builder.build();
    Rig {
        tx,
        rx,
        link,
        bulk_in,
        bulk_out,
        script,
        device,
    }
}

/// A rig whose link has been brought up.
pub fn rig_up(config: Config) -> Rig {
    let r = rig(config);
    r.link.inject(usb_cdc::Event::Configured(true));
    assert!(r.link.is_up());
    r
}

pub fn pin_poll<F: Future>(fut: Pin<&mut F>, cx: &mut Context<'_>) -> Poll<F::Output> {
    fut.poll(cx)
}
