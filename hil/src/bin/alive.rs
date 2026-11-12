//! Proves the crate links for the board: names one public type, prints `alive`, exits 0.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    let config = usb_cdc::Config::new();
    defmt::info!(
        "[INFO] alive: usb-cdc linked, max_packet_size = {}",
        config.max_packet_size
    );
    hil_common::exit(0)
}
