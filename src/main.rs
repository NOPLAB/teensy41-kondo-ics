#![no_std]
#![no_main]

use core::convert::Infallible;

use embedded_hal::{blocking::serial::Write, serial::Read};

use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::{
    board,
    hal::{
        lpuart::{ReadFlags, Status},
        timer::Blocking,
    },
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
        mut dma,
        ..
    } = board::t41(board::instances());

    bsp::LoggingFrontend::default_log().register_usb(usb);

    let mut delay = Blocking::<_, { board::PERCLK_FREQUENCY }>::from_pit(pit.0);

    let led = board::led(&mut gpio2, pins.p13);

    let mut enable_pin = gpio4.output(pins.p2);

    let mut lpuart6: board::Lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);

    lpuart6.disable(|lpuart| {
        lpuart.set_parity(Some(bsp::hal::lpuart::Parity::Even));
    });

    enable_pin.clear();

    led.set();
    delay.block_ms(1000);
    led.clear();

    log::info!("Hello from the USB logger!");

    let mut dma_channel = dma[0].take().unwrap();

    loop {
        // servo_pos(&mut delay, &mut lpuart6, &mut enable_pin, 7500);
        // delay.block_ms(3000);
        // servo_pos(&mut delay, &mut lpuart6, &mut enable_pin, 5500);
        // delay.block_ms(3000);
        // servo_pos(&mut delay, &mut lpuart6, &mut enable_pin, 9500);
        // delay.block_ms(3000);

        servo_pos_dma(
            &mut delay,
            &mut dma_channel,
            &mut lpuart6,
            &mut enable_pin,
            7500,
        )
        .unwrap();
        delay.block_ms(3000);
        servo_pos_dma(
            &mut delay,
            &mut dma_channel,
            &mut lpuart6,
            &mut enable_pin,
            5500,
        )
        .unwrap();
        delay.block_ms(3000);
        servo_pos_dma(
            &mut delay,
            &mut dma_channel,
            &mut lpuart6,
            &mut enable_pin,
            9500,
        )
        .unwrap();
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
    dma_channel: &mut bsp::hal::dma::channel::Channel,
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

    // 13usで動作 余裕を持って20us
    delay.delay_us(20);

    let array = [0x80, h_byte(pos), l_byte(pos)];

    dma_channel.set_disable_on_completion(true);

    uart.clear_status(Status::IDLE);

    log::info!("{:?}", uart.status());

    let future = uart.dma_write(dma_channel, &array);
    spin_on::spin_on(future).unwrap();

    while !uart
        .status()
        .contains(Status::TRANSMIT_EMPTY | Status::TRANSMIT_COMPLETE)
    {
        log::info!("TX Waiting: {:?}", uart.status());
        delay.delay_us(100);
    }

    log::info!("TX Done: {:?}", uart.status());

    enable_pin.set_low().unwrap();

    delay.delay_us(20);

    log::info!("read start");

    let mut buffer = [0u8; 3];
    match spin_on::spin_on(uart.dma_read(dma_channel, &mut buffer)) {
        Ok(_) => {
            for byte in buffer {
                log::info!("{byte}");
            }
        }
        Err(e) => {
            log::info!("{:?}", e);
        }
    }

    log::info!("read end");

    Ok(())
}
