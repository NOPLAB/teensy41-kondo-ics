//! Demonstrates a loopback UART peripheral.
//!
//! It uses the alpha board, with the following pinout:
//!
//! - Pin 14 is TX.
//! - Pin 15 is RX.
//!
//! Baud rate is 115200bps.
//!
//! Every time you send the Teensy a character, it replies with
//! that same character, and it toggles the LED.

#![no_std]
#![no_main]

use core::{
    convert::Infallible,
    future::{self, IntoFuture},
};

use embedded_hal::{
    blocking::{self, serial::Write},
    serial::Read,
};
use rtic_monotonics::imxrt;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::{
    board,
    hal::{lpuart::ReadFlags, timer::Blocking},
    ral,
};

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pit,
        pins,
        mut gpio2,
        mut gpio4,
        lpuart6,
        usb,
        ..
    } = board::t41(board::instances());
    bsp::LoggingFrontend::default_log().register_usb(usb);
    let mut delay = Blocking::<_, { board::PERCLK_FREQUENCY }>::from_pit(pit.0);
    let led = board::led(&mut gpio2, pins.p13);
    let mut enable_pin = gpio4.output(pins.p2);
    let mut lpuart6: board::Lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);
    lpuart6.disable(|lpuart| {
        // lpuart.enable_fifo(Watermark::tx(3));
        // lpuart.enable_fifo(Watermark::rx(NonZeroU32::new(3).unwrap()));
        lpuart.set_parity(Some(bsp::hal::lpuart::Parity::Even));
    });

    // loop {
    //     led.toggle();
    //     let byte = nb::block!(lpuart6.read()).unwrap();
    // nb::block!(lpuart6.write(byte)).unwrap();
    // }

    enable_pin.clear();

    led.set();
    delay.block_ms(1000);
    led.clear();

    loop {
        servo_pos(&mut delay, &mut lpuart6, &mut enable_pin, 7500);
        delay.block_ms(3000);
        servo_pos(&mut delay, &mut lpuart6, &mut enable_pin, 5500);
        delay.block_ms(3000);
        servo_pos(&mut delay, &mut lpuart6, &mut enable_pin, 9500);
        delay.block_ms(3000);
    }
}

fn h_byte(byte: u16) -> u8 {
    (byte >> 7 & 0x7f) as u8
}

fn l_byte(byte: u16) -> u8 {
    (byte & 0x7f) as u8
}

fn servo_pos<T, D, P, E>(delay: &mut D, uart: &mut T, enable_pin: &mut P, pos: u16)
where
    T: Write<u8, Error = Infallible> + Read<u8, Error = ReadFlags>,
    D: embedded_hal::blocking::delay::DelayUs<u32>,
    P: embedded_hal::digital::v2::OutputPin<Error = E>,
    E: core::fmt::Debug,
{
    enable_pin.set_high().unwrap();

    delay.delay_us(1000);

    let array = [0x80, h_byte(pos), l_byte(pos)];

    uart.bwrite_all(&array).unwrap();

    delay.delay_us(1000);

    enable_pin.set_low().unwrap();

    for _ in 0..3 {
        match uart.read() {
            Ok(byte) => {
                log::info!("{byte}");
            }
            Err(_) => {}
        };
    }
}

fn servo_pos_dma<D, U, const N: u8, P, E>(
    delay: &mut D,
    uart: &mut bsp::hal::lpuart::Lpuart<U, N>,
    enable_pin: &mut P,
    pos: u16,
) -> nb::Result<(), Infallible>
where
    D: embedded_hal::blocking::delay::DelayUs<u32>,
    P: embedded_hal::digital::v2::OutputPin<Error = E>,
    E: core::fmt::Debug,
{
    enable_pin.set_high().unwrap();

    delay.delay_us(1000);

    let array = [0x80, h_byte(pos), l_byte(pos)];

    let mut channels = bsp::hal::dma::channels(unsafe { ral::dma::DMA::instance() }, unsafe {
        ral::dmamux::DMAMUX::instance()
    });

    uart.dma_write(&mut channels[13].take().unwrap(), &array);

    delay.delay_us(1000);

    enable_pin.set_low().unwrap();

    for _ in 0..3 {
        match uart.read() {
            Ok(byte) => {
                log::info!("{byte}");
            }
            Err(_) => {}
        };
    }

    Ok(())
}
