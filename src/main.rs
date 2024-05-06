//! Test the DHT22 sensor + PT100 over RTD sensor
#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use core::fmt::Write;
use embedded_hal::digital::OutputPin;
use hal::fugit::RateExtU32;
use hal::Clock;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

// DHT22 sensor related types
use dht_sensor::{dht22, DhtReading};
// PT100 RTD chip
use max31865::{FilterMode, Max31865, SensorType};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // UART Periph & pins Configuration
    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function(),
    );
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.write_full_blocking(b"Coucou Hibou\n");

    // These are implicitly used by the spi driver
    let drdy = pins.gpio8.into_pull_up_input();
    let mut spi_csn = pins.gpio5.into_push_pull_output();
    let spi_mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));
    spi_csn.set_high().unwrap();

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1.MHz(),
        embedded_hal::spi::MODE_1,
    );

    // Init MAX31865
    let mut rtd_sensor = Max31865::new(spi, spi_csn, drdy);
    let ref_sensor = &mut rtd_sensor;
    let _ = ref_sensor.as_mut().expect("REASON").configure(
        true,
        true,
        false,
        SensorType::TwoOrFourWire,
        FilterMode::Filter50Hz,
    );
    ref_sensor
        .as_mut()
        .expect("REASON")
        .set_calibration(43200_u32);

    // Use GPIO 2 as an InOutPin
    // NOTE: set_high() impl is not working well on my hardware and do not tie up the GP2 !
    // I had to append an external  10k pull up resistor to make it work
    let mut pin = hal::gpio::InOutPin::new(pins.gpio2);

    // The DHT datasheet suggests 2 seconds
    delay.delay_ms(2000_u32);

    loop {
        // Read PT100 temperature
        let data_pt100 = rtd_sensor
            .as_mut()
            .expect("REASON")
            .read_default_conversion();
        let _ = match data_pt100 {
            Ok(x) => writeln!(uart, "Temp is :{:.2}", x as f32 / 100.0),
            Err(e) => writeln!(uart, "Err is:{:?}", e),
        };

        // Read both temperature and humidity
        let _ = match dht22::Reading::read(&mut delay, &mut pin) {
            Ok(dht22::Reading {
                temperature: t,
                relative_humidity: h,
            }) => writeln!(uart, "Temp is: {}, Humidity is:{}", t, h),
            Err(e) => writeln!(uart, "Raise DHTError {:?}", e),
        };

        delay.delay_ms(1000_u32);
    }
}

// End of file
