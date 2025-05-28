// #![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use embedded_hal::spi;
use inputs::{DebouncedDInput, DebouncedOutput, PotRead};
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use defmt;
use defmt_rtt as _;

use crate::hal::{
    prelude::*,
    pac,
    timer,
    rcc,
    gpio::NoPin,
    i2c::{I2c, Mode},
};

mod error;
mod inputs;
mod lcd;
mod pwm_fan;
mod stoptimer;

fn setup_clocks(rcc: rcc::Rcc) -> rcc::Clocks {
    rcc.cfgr
        // .hclk(48.MHz())
        .sysclk(48.MHz())
        // .pclk1(24.MHz())
        // .pclk2(24.MHz())
        .freeze()
}

#[entry]
fn main() -> ! {
    defmt::info!("Running!\n");

    let Some(dp) = pac::Peripherals::take() else {
        panic!("Oh no!");
    };

    let Some(cp) = cortex_m::peripheral::Peripherals::take() else {
        panic!("Oh no!");
    };
    
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // Sysclk
    let rcc = dp.RCC.constrain();
    let clocks = setup_clocks(rcc);
    let mut delay = cp.SYST.delay(&clocks);
    let mut lcd_delay = dp.TIM1.delay_us(&clocks);

    // Millis timer
    stoptimer::init_timer(dp.TIM3, &clocks);

    // Pot
    let p_d11 = gpioa.pa7.into_analog();
    let mut pot_obj = PotRead::with_adc01(p_d11, dp.ADC1);
    
    // User button
    let mut pc13 = DebouncedDInput::with_pullup(gpioc.pc13.into_pull_up_input().erase());

    // RGB fan
    let spi02 = dp.SPI2.spi(
        (gpiob.pb13.into_alternate(), NoPin::new(), gpiob.pb15.into_alternate()),
        spi::MODE_1,
        3.MHz(),
        &clocks,
    );
    let p_d6 = gpiob.pb10.into_alternate();
    let ch3 = timer::Channel3::new(p_d6);
    let mut pwm_obj = pwm_fan::AdjustablePwmFan::with_rgb(dp.TIM2, ch3, timer::Channel::C3, spi02, &clocks);
    pwm_obj.init();
    pwm_obj.set_duty(50);

    // LCD
    let i2c_01 = I2c::new(
        dp.I2C1,
        (gpiob.pb8, gpiob.pb9),
        Mode::standard(100.kHz()),
        &clocks,
    );
    let mut lcd_obj = lcd::I2CLcd::new(i2c_01, &mut lcd_delay).unwrap();
    lcd_obj.init().unwrap();

    loop {
        // Fan speed update
        let new_duty_percent = pot_obj.read_percent();
        // defmt::println!("Pot read: {}", new_duty_percent);
        
        if new_duty_percent != pwm_obj.get_duty() {
            pwm_obj.set_duty(new_duty_percent);
            lcd_obj.write_duty_cycle(new_duty_percent as u8).unwrap();
        }

        // Fan RGB update
        if let Some(rgb_obj) = &mut pwm_obj.rgb {    
            if let DebouncedOutput::Changed(d) = pc13.is_low() {
                if !d {
                    rgb_obj.increment_mode().unwrap();
                    defmt::println!("RGB mode change!");
                }
            }

            lcd_obj.write_message(rgb_obj.get_mode_text(), (1, 0)).unwrap();
            rgb_obj.update().unwrap();
        }

        delay.delay_ms(10);
    }
}
