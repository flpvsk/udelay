#![no_std]
#![no_main]

use cortex_m::singleton;
use cortex_m_rt::entry;
use hal::dma::{DMAExt, double_buffer};
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::sio::Sio;
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::pio::PIOBuilder;

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

    // configure out pin for Pio0.
    let _: Pin<_, FunctionPio0> = pins.gpio22.into_mode();
    let out_pin_id = 22;

    let _: Pin<_, FunctionPio0> = pins.gpio21.into_mode();
    let modulator_in_pin_id = 21;
    let _: Pin<_, FunctionPio0> = pins.gpio20.into_mode();
    let modulator_out_pin_id = 20;

    let modulate = pio_proc::pio_asm!(
        ".wrap_target",
        "    mov pins, x",
        "    in pins, 1",
        "    mov x, isr",
        ".wrap"
    );

    // PDM sine wave
    #[allow(clippy::unusual_byte_groupings)]
    let message = [
        0b_01101011_01110111_11011111_11111111,
        0b_11111111_11111011_11110110_11101010,
        0b_10101001_00100001_00000000_00000000,
        0b_00000001_00000000_00100010_01001010
    ];

    let read_buf = pio_proc::pio_asm!(
        ".wrap_target",
        "    out pins, 1 [1]",
        ".wrap"
    );

    // Initialize and start PIO
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let read_buf_installed = pio.install(&read_buf.program).unwrap();
    let (mut read_buf_sm, _, tx) = PIOBuilder::from_program(read_buf_installed)
        .out_pins(out_pin_id, 1)
        .clock_divisor_fixed_point(10_000, 0)
        .autopull(true)
        .build(sm0);

    // The GPIO pin needs to be configured as an output.
    read_buf_sm.set_pindirs([(out_pin_id, hal::pio::PinDir::Output)]);
    read_buf_sm.start();

    let modulate_installed = pio.install(&modulate.program).unwrap();
    let (mut modulate_sm, rx, _) = PIOBuilder::from_program(modulate_installed)
        .out_pins(modulator_out_pin_id, 1)
        .in_pin_base(modulator_in_pin_id)
        .clock_divisor_fixed_point(10_000, 0)
        .autopush(true)
        .build(sm1);
    modulate_sm.set_pindirs([
        (modulator_out_pin_id, hal::pio::PinDir::Output),
        (modulator_in_pin_id, hal::pio::PinDir::Input)
    ]);

    let dma = pac.DMA.split(&mut pac.RESETS);

    let buf1 = singleton!(: [u32; 4] = message).unwrap();
    let buf2 = singleton!(: [u32; 4] = [0; 4]).unwrap();

    let tx_transfer = double_buffer::Config::new((dma.ch0, dma.ch1), buf1, tx).start();
    let mut tx_transfer = tx_transfer.read_next(buf2);

    let rx_transfer = double_buffer::Config::new((dma.ch2, dma.ch3), rx, buf2).start();
    let mut rx_transfer = rx_transfer.write_next(buf1);

    loop {
        // When a transfer is done we immediately enqueue the buffers again.
        if tx_transfer.is_done() {
            let (tx_buf, next_tx_transfer) = tx_transfer.wait();
            tx_transfer = next_tx_transfer.read_next(tx_buf);
        }

        if rx_transfer.is_done() {
            let (rx_buf, next_rx_transfer) = rx_transfer.wait();
            rx_transfer = next_rx_transfer.write_next(rx_buf);
        }
    }
}
