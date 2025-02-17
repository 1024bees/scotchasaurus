#![no_std]
#![no_main]

use core::f32::consts::PI;
use core::ops::BitAnd;

use esp_backtrace as _;
use esp_hal::rng::Rng;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config, I2c},
    main,
    time::{Duration, RateExtU32},
    timer::timg::TimerGroup,
};
use esp_println::logger;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    att::Uuid,
    attribute_server::{AttributeServer, NotificationData, WorkResult},
    gatt, Ble, HciConnector,
};
use esp_wifi::ble::controller::BleConnector;

/// I2C address for the MCP4725 (as configured on your board).
const MCP4725_I2C_ADDR: u8 = 0x62;

/// We build a 100-sample sine wave table (16-bit),
/// but MCP4725 is 12-bit, so we will shift down by 4 bits when writing.
const SINE_TABLE_SIZE: usize = 100;

/// Desired output frequency for the tone.
const OUTPUT_FREQ: u32 = 420;
/// We want 100 samples per cycle, so the sample rate = 420 * 100 = 42,000 samples/sec.
const SAMPLE_RATE: u32 = OUTPUT_FREQ * SINE_TABLE_SIZE as u32; // 42,000

/// Pre-computed 16-bit sine wave lookup table with 100 samples.
static SINE_LUT: [u16; SINE_TABLE_SIZE] = {
    let mut table = [0u16; SINE_TABLE_SIZE];
    let mut i = 0;
    while i < SINE_TABLE_SIZE {
        // Sine from -1.0..1.0
        //let sine_val = (2.0 * PI * i as f32 / SINE_TABLE_SIZE as f32).sin();
        let sine_val = 2.0 * PI * i as f32 / SINE_TABLE_SIZE as f32;

        // Shift from -1..1 to 0..65535
        let scaled = ((sine_val * 0.5) + 0.5) * (u16::MAX as f32);
        // Guard range in case of rounding

        let clamped = if scaled < 0.0 {
            0
        } else if scaled > u16::MAX as f32 {
            u16::MAX
        } else {
            scaled as u16
        };
        table[i] = clamped;
        i += 1;
    }
    table
};

#[main]
fn main() -> ! {
    logger::init_logger_from_env();

    // -------------------------
    // 1. Get Peripherals & Clocks
    // -------------------------

    let config = esp_hal::Config::default();
    let peripherals = esp_hal::init(config);

    let mut bluetooth = peripherals.BT;
    let mut timer_group0 = TimerGroup::new(peripherals.TIMG0);
    let mut wdt0 = timer_group0.wdt;
    let mut timer_group1 = TimerGroup::new(peripherals.TIMG1);
    let mut wdt1 = timer_group1.wdt;

    // Provide delay
    let mut delay = Delay::new();

    // -------------------------
    // 2. Set up I2C
    // -------------------------
    // Adjust GPIO numbers to match your board.
    // For example on ESP32, GPIO21 is SDA, GPIO22 is SCL, etc.

    let sda_pin = peripherals.GPIO6;
    let scl_pin = peripherals.GPIO7;

    // Create the I2C driver at ~100kHz or 400kHz.
    // The faster the bus, the easier to achieve the higher sample rate,
    // but watch out for overhead.
    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(100.kHz()),
    )
    .unwrap()
    .with_sda(sda_pin)
    .with_scl(scl_pin);

    // Initialize the MCP4725 driver
    let mut dac = Mcp4725::new(i2c);

    log::info!("Starting MCP4725 tone output (420 Hz)...");

    // Each sample is 1 / 42,000 seconds ≈ 23.8µs.
    let sample_interval_us = 1_000_000u32 / SAMPLE_RATE;

    // Optionally enable BLE (commented out if not needed):
    /*
    let init = esp_wifi::init(
        timer_group0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    let connector = BleConnector::new(&init, &mut bluetooth);
    let now = || esp_hal::time::now().duration_since_epoch().to_millis();
    let hci = HciConnector::new(connector, now);
    let mut ble = Ble::new(&hci);

    log::info!("{:?}", ble.init());
    log::info!("{:?}", ble.cmd_set_le_advertising_parameters());
    log::info!(
        "{:?}",
        ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                AdStructure::CompleteLocalName(esp_hal::chip!()),
            ])
            .unwrap()
        )
    );
    log::info!("{:?}", ble.cmd_set_le_advertise_enable(true));
    log::info!("Started advertising");
    */

    let mut idx = 0;
    loop {
        // Write next sample
        let value_16 = SINE_LUT[idx];
        // MCP4725 is 12-bit, so shift down by 4.
        // If you prefer, you can also create a 12-bit LUT directly.
        dac.write_fast_mode(value_16 >> 4).ok();

        idx = (idx + 1) % SINE_TABLE_SIZE;

        log::info!("wrote!");
        // Delay to maintain ~42 kHz sample rate
        delay.delay_micros(sample_interval_us);
    }
}

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
    pub fn write_fast_mode(&mut self, value12: u16) -> Result<(), E> {
        // Make sure value12 is at most 12 bits:
        let value12 = value12 & 0x0FFF;

        // Upper byte = 0b0110xxxx (fast mode + normal power-down)
        // 0x60 = 0b01100000
        let high_byte = 0x00 | ((value12 >> 8) & 0x0F) as u8;
        let low_byte = (value12 & 0xFF) as u8;

        self.i2c.write(MCP4725_I2C_ADDR, &[high_byte, low_byte])
    }

    /// If you need to free the I2C peripheral
    pub fn release(self) -> I2C {
        self.i2c
    }
}
