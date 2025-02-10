#![no_std]
#![no_main]

use core::ops::BitAnd;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config, I2c},
    main,
    time::RateExtU32,
    timer::timg::TimerGroup,
};
use esp_println::logger;

// Replace with the actual I2C address of the AD5693
// (0x0C, 0x0D, 0x0E, 0x0F, etc., depending on your A0/A1 pins).
const AD5693_I2C_ADDR: u8 = 0x4C;

// This is "Write to and Update DAC" command for AD5693 if we
// want to send a 16-bit value. According to the AD5693 datasheet,
// the command bits for "Write to and Update (Normal Operation)" is 0b0011 (3 << 4 == 0x30).
const CMD_WRITE_UPDATE: u8 = 0x30;

// How many samples in one cycle of the sine wave table.
const SINE_TABLE_SIZE: usize = 100;

// Desired output frequency = 420 Hz
// If we have SINE_TABLE_SIZE = 100, we need a sample rate of 420 * 100 = 42_000 samples/sec
// That is quite high for I2C at standard mode (100 kHz). At 400 kHz Fast Mode,
// you might get close, but watch out for I2C overhead.
// For demonstration we'll try a smaller rate or see if we can push I2C to 400 kHz.

const OUTPUT_FREQ: u32 = 420;
const SAMPLE_RATE: u32 = OUTPUT_FREQ * SINE_TABLE_SIZE as u32; // 42_000

// Build a 100-sample sine wave table scaled to 16-bit output.
// The AD5693 has 16-bit resolution, so valid values are 0 ..= 65535.
static SINE_LUT: [u16; SINE_TABLE_SIZE] = {
    const PI: f32 = core::f32::consts::PI;
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

    // Timers can be used for WDT or alarms; we disable WDT here for simplicity.
    let mut timer_group0 = TimerGroup::new(peripherals.TIMG0);
    let mut wdt0 = timer_group0.wdt;

    let mut timer_group1 = TimerGroup::new(peripherals.TIMG1);
    let mut wdt1 = timer_group1.wdt;

    // Delay provider
    let mut delay = Delay::new();

    // -------------------------
    // 2. Set up I2C
    // -------------------------
    // Adjust GPIO numbers to match your board.
    // For example on ESP32, GPIO21 is SDA, GPIO22 is SCL

    let sda_pin = peripherals.GPIO6;
    let scl_pin = peripherals.GPIO7;

    // Create the I2C driver at 400 kHz (Fast Mode).
    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(100.kHz()),
    )
    .unwrap()
    .with_sda(sda_pin)
    .with_scl(scl_pin);

    // Wrap I2C in a RefCell or Mutex if you plan to share across tasks/interrupts.
    let mut dac = AD5693::new(i2c);

    log::info!("Starting 420 Hz tone output...");

    // We want to output a sample every 1 / SAMPLE_RATE seconds
    let sample_interval_us = (1_000_000u32 / SAMPLE_RATE) as u32;

    let mut idx = 0;
    loop {
        // 3. Write next sample from the LUT
        let value = SINE_LUT[idx];

        // Send data to DAC: AD5693 expects two data bytes plus
        // the command nibble in the high 4 bits of the first byte.
        // Full 16-bit input data is: D[15:0].
        // For "Write & update" command, upper nibble is 0x3 (0011).
        // Byte 1: [CMD(4 bits) | D15..D12 (4 bits)]
        // Byte 2: [D11..D4 (8 bits)]
        // Byte 3: [D3..D0 (4 bits) << 4, typically] (But AD5693 is 16-bit: we use 2 data bytes total.)
        //
        // The AD5693 datasheet shows a 24-bit frame for some devices with multiple channels,
        // but typically for a single-channel device you have 2 data bytes plus the control nibble.
        // Double-check with your specific device.
        log::info!("writing");

        dac.write_update(value).ok();

        // Increment LUT index
        idx = (idx + 1) % SINE_TABLE_SIZE;

        let val = dac.read();

        if let Ok(inner) = val {
            log::info!("{}", inner);
        } else {
            log::info!("failed to read i2c");
        }

        // Delay to achieve ~42k samples/sec
        delay.delay_micros(sample_interval_us);
    }
}

/// A small wrapper around the I2C peripheral for AD5693
pub struct AD5693<I2C> {
    i2c: I2C,
}

impl<I2C, E> AD5693<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        AD5693 { i2c }
    }

    /// Writes a 16-bit raw value to the DAC and updates its output.
    /// For single-channel AD5693, the command nibble for 'Write to DAC and Update' is 0x3 (0b0011).
    /// The final command byte = 0x3 << 4 = 0x30 for the upper nibble, plus the top bits of data.
    pub fn write_update(&mut self, value: u16) -> Result<(), E> {
        // The AD5693's 16-bit data is split across two bytes:

        let byte1 = CMD_WRITE_UPDATE;
        let byte2 = (value >> 8).bitand(0x0F) as u8;
        let byte3 = (value).bitand(0x0F) as u8;

        let data = [byte1, byte2, byte3];

        self.i2c.write(AD5693_I2C_ADDR, &data)
    }

    pub fn read(&mut self) -> Result<u16, E> {
        let mut bytes: [u8; 2] = [0; 2];
        self.i2c.read(AD5693_I2C_ADDR, bytes.as_mut_slice())?;
        Ok(u16::from_be_bytes(bytes))
    }

    /// If you need to free the I2C peripheral
    pub fn release(self) -> I2C {
        self.i2c
    }
}
