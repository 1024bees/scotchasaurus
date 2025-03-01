#![no_std]
#![no_main]

mod l2cap;
use core::f32::consts::PI;
use core::ops::BitAnd;
use l2cap::run;

use esp_backtrace as _;
use esp_hal::riscv::register::time;
use esp_hal::rng::Rng;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config, I2c},
    main,
    time::{Duration, RateExtU32, now},
    timer::timg::TimerGroup,
};
use esp_println::logger;
use esp_wifi::ble::controller::BleConnector;
use {esp_alloc as _, esp_backtrace as _};

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;

use {esp_alloc as _, esp_backtrace as _};

/// I2C address for the MCP4725 (as configured on your board).
const MCP4725_I2C_ADDR: u8 = 0x60;

/// We want 100 samples per cycle, so the sample rate = 420 * 100 = 42,000 samples/sec.
const SAMPLE_RATE: u32 = 10025;

const DELAY: u32 = 1_000_000 / SAMPLE_RATE;

/// Replace with your actual audio data from scream.wav
/// This example assumes 8-bit unsigned PCM format
const SCREAM_DATA: &[u8] = include_bytes!("scream.raw"); // See conversion notes below

//#[main]
//fn main() -> ! {
//    logger::init_logger_from_env();
//
//    // -------------------------
//    // 1. Get Peripherals & Clocks
//    // -------------------------
//
//    let config = esp_hal::Config::default();
//    let peripherals = esp_hal::init(config);
//
//    let mut bluetooth = peripherals.BT;
//    let mut timer_group0 = TimerGroup::new(peripherals.TIMG0);
//    let mut wdt0 = timer_group0.wdt;
//    let mut timer_group1 = TimerGroup::new(peripherals.TIMG1);
//    let mut wdt1 = timer_group1.wdt;
//
//    // Provide delay
//    let mut delay = Delay::new();
//
//    // -------------------------
//    // 2. Set up I2C
//    // -------------------------
//    // Adjust GPIO numbers to match your board.
//    // For example on ESP32, GPIO21 is SDA, GPIO22 is SCL, etc.
//
//    let sda_pin = peripherals.GPIO2;
//    let scl_pin = peripherals.GPIO3;
//
//    // Create the I2C driver at ~100kHz or 400kHz.
//    // The faster the bus, the easier to achieve the higher sample rate,
//    // but watch out for overhead.
//    let mut i2c = I2c::new(
//        peripherals.I2C0,
//        Config::default().with_frequency(400.kHz()),
//    )
//    .unwrap()
//    .with_sda(sda_pin)
//    .with_scl(scl_pin);
//
//    // Initialize the MCP4725 driver
//
//    log::info!("Done scanning i2c");
//    let mut dac = Mcp4725::new(i2c);
//    log::info!(
//        "Starting MCP4725 scream output on i2c address {:x}",
//        MCP4725_I2C_ADDR
//    );
//
//    let mut idx = 0;
//    let mut time1 = now();
//    loop {
//        // For 8-bit samples: scale up to 12-bit (<< 4)
//        // For 16-bit samples: use appropriate conversion (>> 4 for 12-bit DAC)
//        // Suppose we read the sample from the raw file in little-endian 16-bit:
//        let raw_low = SCREAM_DATA[idx];
//        let raw_high = SCREAM_DATA[idx + 1];
//        let sample_u16 = u16::from_le_bytes([raw_low, raw_high]);
//
//        // sample_i16 is signed from -32768..32767
//
//        // Now scale down from 16 bits to 12 bits
//        let sample_12 = sample_u16 >> 4; // 0..4095
//
//        // Write to DAC
//
//        let rv = dac.write(sample_12);
//        delay.delay_micros(DELAY / 7);
//
//        idx = (idx + 2) % SCREAM_DATA.len();
//        // Add pause after completing full playback
//        if idx == 0 {
//            let time2 = now();
//            let duration = time2 - time1;
//            log::info!("took {:?} ms to play 10025 samples", duration.to_millis());
//            time1 = time2;
//            delay.delay_millis(500);
//        }
//    }
//}

/// A small wrapper around the I2C peripheral for MCP4725 (12-bit DAC).
pub struct Mcp4725<I2C> {
    i2c: I2C,
}

impl<I2C, E> Mcp4725<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Write a 12-bit value (0..4095) in "fast mode" to the MCP4725.
    /// The MCP4725 fast-mode two-byte write format:
    ///   Byte 1 = [0b0110 D11..D8]
    ///   Byte 2 = [D7..D0]
    /// PD bits = 00 (normal operation).
    pub fn write(&mut self, value12: u16) -> Result<(), E> {
        let value12 = value12 & 0x0FFF;
        // Correct fast mode format:
        // First byte = upper 4 bits (D11-D8) in lower nibble
        // Second byte = lower 8 bits (D7-D0)
        let bytes = [
            (value12 >> 8) as u8,   // Contains D11-D8 in lower 4 bits
            (value12 & 0xFF) as u8, // Contains D7-D0
        ];
        self.i2c.write(MCP4725_I2C_ADDR, &bytes)
    }

    /// If you need to free the I2C peripheral
    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[esp_hal_embassy::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });
    esp_alloc::heap_allocator!(72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);

    let init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    esp_hal_embassy::init(timg1.timer0);

    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    run::<_, 256>(controller).await;
}
