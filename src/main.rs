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
use hal::fugit::RateExtU32;
use hal::Clock;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

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

// Credentials for WiFi connection
mod credentials;

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
    // UART0
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
    let esp8266_pins = (
        pins.gpio8.into_function(),
        pins.gpio9.into_function(),
    );
    let esp8266 = hal::uart::UartPeripheral::new(pac.UART1, esp8266_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.write_full_blocking(b"Coucou Hibou\n");

    // Initialize WIFI connection
    // Restart: "AT+RST\r\n"
    esp8266.write_full_blocking(b"AT+RST\r\n");

    // SSID config
    let ssid = credentials::SSID;
    let password = credentials::PASSWORD;
    let mut cmd = [0u8; 50]; // Adjust the size as needed

    let prefix = b"AT+CWJAP=\"";
    let infix = b"\",\"";
    let postfix = b"\"\r\n";

    let mut index = 0;

    let mut copy_into_cmd = |src: &[u8]| {
        cmd[index..index + src.len()].copy_from_slice(src);
        index += src.len();
    };

    copy_into_cmd(prefix);
    copy_into_cmd(ssid);
    copy_into_cmd(infix);
    copy_into_cmd(password);
    copy_into_cmd(postfix);

    esp8266.write_full_blocking(&cmd[..index]);

    delay.delay_ms(3000_u32);
    // STA config: "AT+CWMODE=1\r\n"
    esp8266.write_full_blocking(b"AT+CWMODE=1\r\n");
    delay.delay_ms(1500_u32);
    // IP address: "AT+CIFSR\r\n"
    esp8266.write_full_blocking(b"AT+CIFSR\r\n");
    delay.delay_ms(1500_u32);
    // Several connections: "AT+CIPMUX=1\r\n"
    esp8266.write_full_blocking(b"AT+CIPMUX=1\r\n");
    delay.delay_ms(1500_u32);
    // Server config: "AT+CIPSERVER=1,80\r\n"
    esp8266.write_full_blocking(b"AT+CIPSERVER=1,80\r\n");

    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
