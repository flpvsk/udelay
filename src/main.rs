//! This example shows how to read from and write to PIO using DMA.
//!
//! If a LED is connected to that pin, like on a Pico board, it will continously output "HELLO
//! WORLD" in morse code. The example also tries to read the data back. If reading the data fails,
//! the message will only be shown once, and then the LED remains dark.
//!
//! See the `Cargo.toml` file for Copyright and licence details.
#![no_std]
#![no_main]

use cortex_m::singleton;
use cortex_m_rt::entry;
use hal::dma::{DMAExt, DoubleBufferingConfig};
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::sio::Sio;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure LED pin for Pio0.
    let _led: Pin<_, FunctionPio0> = pins.gpio22.into_mode();
    // PIN id for use inside of PIO
    let out_pin_id = 22;

    // PDM sine wave
    #[allow(clippy::unusual_byte_groupings)]
    let message = [
        0b_01101011_01110111_11011111_11111111,
        0b_11111111_11111011_11110110_11101010,
        0b_10101001_00100001_00000000_00000000,
        0b_00000001_00000000_00100010_01001010
    ];

    // Define a PIO program which reads data from the TX FIFO bit by bit, configures the LED
    // according to the data, and then writes the data back to the RX FIFO.
    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "    out pins, 1 [9]",
        ".wrap"
    );

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (mut sm, _, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .out_pins(out_pin_id, 1)
        .clock_divisor_fixed_point(10_000, 0)
        .autopull(true)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([(out_pin_id, hal::pio::PinDir::Output)]);
    sm.start();

    let dma = pac.DMA.split(&mut pac.RESETS);

    // Transfer a single message via DMA.
    let tx_buf = singleton!(: [u32; 4] = message).unwrap();

    // Chain some buffers together for continuous transfers
    let tx_buf2 = singleton!(: [u32; 4] = message).unwrap();

    let tx_transfer = DoubleBufferingConfig::new((dma.ch0, dma.ch1), tx_buf, tx).start();
    let mut tx_transfer = tx_transfer.read_next(tx_buf2);

    loop {
        // When a transfer is done we immediately enqueue the buffers again.
        if tx_transfer.is_done() {
            let (tx_buf, next_tx_transfer) = tx_transfer.wait();
            tx_transfer = next_tx_transfer.read_next(tx_buf);
        }
    }
}
