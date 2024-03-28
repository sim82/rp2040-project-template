//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use rp2040_hal::{
    entry,
    fugit::RateExtU32,
    gpio::{PullNone, PullUp},
    Timer, I2C,
};
use smart_leds::{SmartLedsWrite, RGB8};

use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::v2::OutputPin, PwmPin};
use panic_probe as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionI2C, Pin},
    pac,
    prelude::*,
    sio::Sio,
    watchdog::Watchdog,
};
use ssd1306::{mode::TerminalMode, prelude::*, I2CDisplayInterface, Ssd1306};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm4 = &mut pwm_slices.pwm4;
    pwm4.set_ph_correct();
    pwm4.enable();
    let pwm4_ch_b = &mut pwm4.channel_b;
    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    pwm4_ch_b.output_to(pins.gpio25);
    let pwm0 = &mut pwm_slices.pwm0;
    pwm0.set_ph_correct();
    pwm0.enable();
    let pwm0_ch_a = &mut pwm0.channel_a;
    let pwm0_ch_b = &mut pwm0.channel_b;

    pwm0_ch_a.output_to(pins.gpio16);
    pwm0_ch_b.output_to(pins.gpio17);
    let mut gpio_test = pins.gpio1.into_push_pull_output();

    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio18.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio19.reconfigure();
    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400_u32.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear().unwrap();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio4.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    writeln!(display, "Hello, world!");
    info!("Hello, world!");

    const BLACK: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
    let mut leds = [BLACK; 8];
    leds[0] = (64, 0, 0).into();
    leds[1] = (0, 64, 0).into();
    leds[2] = (0, 0, 64).into();
    leds[3] = (64, 64, 0).into();
    leds[4] = (64, 0, 64).into();
    leds[5] = (0, 64, 64).into();
    leds[6] = (64, 0, 0).into();
    leds[7] = (0, 64, 0).into();
    for loop_count in 0u32.. {
        writeln!(display, "on!");
        let LOW = 0;
        let HIGH = 25000;
        for i in LOW..=HIGH {
            delay.delay_us(8);
            pwm4_ch_b.set_duty(i);
            pwm0_ch_a.set_duty(i);
            pwm0_ch_b.set_duty(HIGH - i);
        }
        // led_pin.set_high().unwrap();
        gpio_test.set_high();
        // delay.delay_ms(500);
        writeln!(display, "off!");
        for i in (LOW..=HIGH).rev() {
            delay.delay_us(8);
            pwm4_ch_b.set_duty(i);
            pwm0_ch_a.set_duty(i);
            pwm0_ch_b.set_duty(HIGH - i);
        }

        gpio_test.set_low();
        ws.write(
            leds.iter()
                // .map(|c| (c.r / 4, c.g / 4, c.b / 4).into())
                .copied(),
        )
        .unwrap();
        leds.rotate_right(1);
        // leds[0].r = (loop_count % 255) as u8;
        // leds[0].g = ((loop_count + 100) % 255) as u8;
        // leds[0].b = ((loop_count + 200) % 255) as u8;
        // delay.delay_us(16);
        // delay.delay_ms(500);
    }
    loop {}
}

// End of file
