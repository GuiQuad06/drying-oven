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
use embedded_hal_0_2::serial::{Read, Write};
use hal::fugit::RateExtU32;
use hal::Clock;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};
// ESP8266 related lib
extern crate ESP8266;
// Credentials for WiFi connection
mod credentials;

// We need to implement Read trait for InfallibleReader
struct InfallibleReader<R: Read<u8>>(R);

impl<R: Read<u8>> Read<u8> for InfallibleReader<R> {
    type Error = core::convert::Infallible;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        match self.0.read() {
            Ok(byte) => Ok(byte),
            Err(nb::Error::Other(_)) => Ok(0),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
        }
    }
}

// We need to impplement Write trait for InfallibleWriter
struct InfallibleWriter<W: Write<u8>>(W);

impl<W: Write<u8>> Write<u8> for InfallibleWriter<W> {
    type Error = core::convert::Infallible;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        match self.0.write(word) {
            Ok(()) => Ok(()),
            Err(nb::Error::Other(_)) => Ok(()),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        match self.0.flush() {
            Ok(()) => Ok(()),
            Err(nb::Error::Other(_)) => Ok(()),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
        }
    }
}

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

    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // UART Periph & pins Configuration
    // UART0 (console)
    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function(),
    );
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    // UART1 (ESP8266)
    let esp8266_pins = (pins.gpio8.into_function(), pins.gpio9.into_function());
    let esp8266 = hal::uart::UartPeripheral::new(pac.UART1, esp8266_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // esp8266 UART interface needs to be splitted as ESP8266 external crate
    // is expecting InfallibleReader for the RX channel and InfallibleWriter for
    // TX channel.
    let (rx, tx) = esp8266.split();
    let rx = InfallibleReader(rx);
    let tx = InfallibleWriter(tx);

    uart.write_full_blocking(b"Coucou Hibou\n");

    let mut esp = ESP8266::esp8266::new(tx, rx, delay).unwrap();
    let _ = esp.init();
    let _ = esp.join_AP(credentials::SSID, credentials::PASSWORD);
    let _ = esp.tcp_server(80);

    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
