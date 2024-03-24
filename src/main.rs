//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use bsp::{
    entry,
    hal::{fugit::RateExtU32, gpio::PullNone, I2C},
};
use core::fmt::Write;
// use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::v2::OutputPin, PwmPin};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionI2C, Pin},
    pac,
    prelude::*,
    sio::Sio,
    watchdog::Watchdog,
};
use ssd1306::{mode::TerminalMode, prelude::*, I2CDisplayInterface, Ssd1306};
use ws2812_pio::Ws2812;

#[entry]
fn main() -> ! {
    // info!("Program start");
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm4 = &mut pwm_slices.pwm4;
    pwm4.set_ph_correct();
    pwm4.enable();
    let pwm4_ch_b = &mut pwm4.channel_b;
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    pwm4_ch_b.output_to(pins.led);
    let pwm0 = &mut pwm_slices.pwm0;
    pwm0.set_ph_correct();
    pwm0.enable();
    let pwm0_ch_a = &mut pwm0.channel_a;
    let pwm0_ch_b = &mut pwm0.channel_b;

    pwm0_ch_a.output_to(pins.gpio16);
    pwm0_ch_b.output_to(pins.gpio17);
    let mut gpio_test = pins.gpio1.into_push_pull_output();
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    // let mut led_pin = pins.led.into_push_pull_output();

    let sda_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio18.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio19.reconfigure();
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
    display.write_str("Hello, world!");
    loop {
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
        // led_pin.set_low().unwrap();
        // delay.delay_ms(500);
    }
}

// End of file
