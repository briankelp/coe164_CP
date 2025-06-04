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

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::pac::interrupt;
use stm32f4xx_hal::gpio::{Edge, ExtiPin};


static RGB_MODE_CHANGE_REQUESTED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static LAST_INTERRUPT_TIME: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static FAN_TOGGLE_REQUESTED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
const DEBOUNCE_TIME_MS: u32 = 200; // 200ms debounce time

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
    stoptimer::init_timer(dp.TIM3, &clocks); // Ensure stoptimer is initialized for debouncing

    // Pot
    let p_d11 = gpioa.pa7.into_analog();
    let mut pot_obj = PotRead::with_adc01(p_d11, dp.ADC1);
    
    // User button PC13 setup
    let mut syscfg = dp.SYSCFG.constrain();
    let mut exti_hal = dp.EXTI; // Use a distinct name from pac::EXTI if used directly in ISR
    
    let mut pc13 = gpioc.pc13.into_pull_up_input();
    pc13.make_interrupt_source(&mut syscfg);
    pc13.enable_interrupt(&mut exti_hal);
    pc13.trigger_on_edge(&mut exti_hal, Edge::Falling);
    
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI15_10);
    }

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

    // Initial LCD display for RGB mode
    if let Some(rgb_obj) = &pwm_obj.rgb { // Use non-mutable borrow for initial text
        lcd_obj.write_message(rgb_obj.get_mode_text(), (1, 0)).unwrap();
    }

    let mut last_lcd_update = 0u32;
    let mut pot_readings = [0u16; 4];
    let mut pot_index = 0;
    
    let mut fan_enabled = true; // Fan starts enabled
    let mut pc14 = gpioc.pc14.into_pull_up_input();
    pc14.make_interrupt_source(&mut syscfg);
    pc14.enable_interrupt(&mut exti_hal);
    pc14.trigger_on_edge(&mut exti_hal, Edge::Falling);

    loop {
        // Check for RGB mode change request from interrupt
        let mut mode_change_triggered = false;
        cortex_m::interrupt::free(|cs| {
            let mut requested_flag = RGB_MODE_CHANGE_REQUESTED.borrow(cs).borrow_mut();
            if *requested_flag {
                mode_change_triggered = true;
                *requested_flag = false; // Reset the flag
            }
        });

        if mode_change_triggered {
            if let Some(rgb_obj) = &mut pwm_obj.rgb {
                rgb_obj.increment_mode().unwrap(); // Ensure PwmFanRgb has this method
                let new_mode_text = rgb_obj.get_mode_text();
                lcd_obj.write_message(new_mode_text, (1, 0)).unwrap();
                defmt::println!("RGB Mode updated by interrupt to: {}", new_mode_text);
            }
        }

        // Check the fan on/off button
        let raw_button_state = pc14.is_low();
        if raw_button_state {
            // Button is pressed (assuming active low)
            defmt::println!("PC14 raw state: pressed");
        } else {
            defmt::println!("PC14 raw state: released");
        }

        // Fan speed update with averaging
        let new_duty_percent = pot_obj.read_percent();
        pot_readings[pot_index] = new_duty_percent;
        pot_index = (pot_index + 1) % pot_readings.len();
        
        let avg_duty = pot_readings.iter().sum::<u16>() / pot_readings.len() as u16;
        
        if (avg_duty as i16 - pwm_obj.get_duty() as i16).abs() > 2 || !fan_enabled {
            // Set duty to either the potentiometer value or 0 based on fan_enabled
            pwm_obj.set_duty(if fan_enabled { avg_duty } else { 0 });
            
            let now = stoptimer::get_millis();
            if now - last_lcd_update > 100 {
                lcd_obj.write_duty_cycle(if fan_enabled { avg_duty as u8 } else { 0 }).unwrap();
                last_lcd_update = now;
            }
        }

        // Fan RGB animation update (handles the visual change of the current mode)
        if let Some(rgb_obj) = &mut pwm_obj.rgb {
            // The mode text is updated above only when it actually changes.
            // This update() handles the animation/color cycling of the current mode.
            rgb_obj.update().unwrap();
        }

        // In the main loop, replace the raw button state check
        let mut fan_toggle_triggered = false;
        cortex_m::interrupt::free(|cs| {
            let mut fan_toggle_flag = FAN_TOGGLE_REQUESTED.borrow(cs).borrow_mut();
            if *fan_toggle_flag {
                fan_toggle_triggered = true;
                *fan_toggle_flag = false; // Reset the flag
            }
        });

        if fan_toggle_triggered {
            fan_enabled = !fan_enabled;
            defmt::println!("Fan toggled: {}", if fan_enabled { "ON" } else { "OFF" });
            lcd_obj.write_message(if fan_enabled { "Fan: ON " } else { "Fan: OFF" }, (0, 9)).unwrap();
        }

        delay.delay_ms(5);
    }
}

#[interrupt]
fn EXTI15_10() {
    let exti_regs = unsafe { &*pac::EXTI::ptr() };

    // Handle PC13 (RGB mode change)
    if exti_regs.pr.read().pr13().bit_is_set() {
        exti_regs.pr.write(|w| w.pr13().set_bit());
        
        cortex_m::interrupt::free(|cs| {
            let current_time_ms = stoptimer::get_millis();
            let mut last_press_time_ms = LAST_INTERRUPT_TIME.borrow(cs).borrow_mut();

            if current_time_ms.saturating_sub(*last_press_time_ms) > DEBOUNCE_TIME_MS {
                *last_press_time_ms = current_time_ms;
                
                let mut requested_flag = RGB_MODE_CHANGE_REQUESTED.borrow(cs).borrow_mut();
                *requested_flag = true;
                defmt::println!("PC13 Interrupt: RGB mode change requested.");
            }
        });
    }

    // Handle PC14 (fan toggle)
    if exti_regs.pr.read().pr14().bit_is_set() {
        exti_regs.pr.write(|w| w.pr14().set_bit());
        
        cortex_m::interrupt::free(|cs| {
            let current_time_ms = stoptimer::get_millis();
            let mut last_press_time_ms = LAST_INTERRUPT_TIME.borrow(cs).borrow_mut();

            if current_time_ms.saturating_sub(*last_press_time_ms) > DEBOUNCE_TIME_MS {
                *last_press_time_ms = current_time_ms;
                
                let mut fan_toggle_flag = FAN_TOGGLE_REQUESTED.borrow(cs).borrow_mut();
                *fan_toggle_flag = true;
                defmt::println!("PC14 Interrupt: Fan toggle requested.");
            }
        });
    }
}