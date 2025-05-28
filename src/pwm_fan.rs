use core::u16;

use smart_leds::{
    colors,
    gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
    RGB,
    RGB8,
};

use stm32f4xx_hal::{
    hal::spi,
    prelude::*,
    rcc,
    timer,
};

use ws2812_spi as ws2812;
use defmt;

use crate::stoptimer;

pub struct AdjustablePwmFan <SPI, TIM, PINS>
    where SPI: spi::SpiBus<u8>,
    TIM: timer::PwmExt,
    PINS: timer::Pins <TIM> {
    device: timer::PwmHz <TIM, PINS>,
    channel: timer::Channel,
    current_duty: u16,
    pub rgb: Option <PwmFanRgb <SPI>>,
}

pub struct PwmFanRgb <SPI>
    where SPI: spi::SpiBus<u8> {
    pub device: ws2812::Ws2812 <SPI>,
    color_mode: u8,
    brightness: u8,
}

impl <SPI, TIM, PINS> AdjustablePwmFan <SPI, TIM, PINS>
    where SPI: spi::SpiBus<u8>,
    TIM: timer::PwmExt,
    PINS: timer::Pins <TIM> {
    pub fn new(timer: TIM, pwm_pin: PINS, pwm_channel: timer::Channel, clock: &rcc::Clocks) -> Self {
        let pwm_obj = timer.pwm_hz(pwm_pin, 1000.kHz(), &clock);

        Self {
            device: pwm_obj,
            channel: pwm_channel,
            current_duty: 25,
            rgb: None,
        }
    }

    pub fn with_rgb(timer: TIM, pwm_pin: PINS, pwm_channel: timer::Channel, spi_bus: SPI, clock: &rcc::Clocks) -> Self {
        let mut new_obj = Self::new(timer, pwm_pin, pwm_channel, clock);
        new_obj.rgb = Some(PwmFanRgb::new(spi_bus));

        new_obj
    }

    pub fn init(&mut self) {
        self.device.enable(self.channel);
    }

    /// Set duty cycle
    /// 
    /// The duty cycle should be between 0% and 100% inclusive.
    pub fn set_duty(&mut self, mut duty: u16) {
        let max_duty = match self.device.get_max_duty() {
            0 => u16::MAX,
            other => other,
        };
        
        duty = duty.clamp(0, 100);
        let scaled_duty = u32::from(duty)
            .saturating_mul(u32::from(max_duty))
            .saturating_mul(655)
            .wrapping_shr(16)
            .clamp(0, u32::from(u16::MAX))
            as u16;

        self.device.set_duty(self.channel, scaled_duty);
        self.current_duty = duty;
    }

    /// Get duty cycle
    pub fn get_duty(&mut self) -> u16 {
        // let now_duty = u32::from(self.device.get_duty(self.channel))
        //     .saturating_mul(100)
        //     .saturating_mul(0)
        //     .wrapping_shr(16)
        //     .clamp(0, 100)
        //     as u16;

        // duty (100 / max_duty) (max_duty / 2^16) (2^16 / max_duty)

        self.current_duty
    }
}

impl <SPI> PwmFanRgb <SPI>
    where SPI: spi::SpiBus<u8> {
    pub const MAX_MODES: usize = 13;
    pub const FAN_LED_QTY: usize = 8;
    pub const LED_COLOR_PALETTES: [[RGB8; 16]; 4] = [
        // Forest
        [
            colors::DARK_GREEN,
            colors::DARK_GREEN,
            colors::DARK_OLIVE_GREEN,
            colors::DARK_GREEN,
            colors::GREEN,
            colors::FOREST_GREEN,
            colors::OLIVE_DRAB,
            colors::GREEN,
            colors::SEA_GREEN,
            colors::MEDIUM_AQUAMARINE,
            colors::LIME_GREEN,
            colors::YELLOW_GREEN,
            colors::LIGHT_GREEN,
            colors::LAWN_GREEN,
            colors::MEDIUM_AQUAMARINE,
            colors::FOREST_GREEN,
        ],
        // Cloud
        [
            colors::BLUE,
            colors::DARK_BLUE,
            colors::DARK_BLUE,
            colors::DARK_BLUE,
            colors::DARK_BLUE,
            colors::DARK_BLUE,
            colors::DARK_BLUE,
            colors::DARK_BLUE,
            colors::BLUE,
            colors::DARK_BLUE,
            colors::SKY_BLUE,
            colors::SKY_BLUE,
            colors::LIGHT_BLUE,
            colors::WHITE,
            colors::LIGHT_BLUE,
            colors::SKY_BLUE,
        ],
        // Heat
        [
            RGB::new(0, 0, 0),
            RGB::new(0x33, 0, 0),
            RGB::new(0x66, 0, 0),
            RGB::new(0x99, 0, 0),
            RGB::new(0xCC, 0, 0),
            RGB::new(0xFF, 0, 0),
            RGB::new(0xFF, 0x33, 0),
            RGB::new(0xFF, 0x66, 0),
            RGB::new(0xFF, 0x99, 0),
            RGB::new(0xFF, 0xCC, 0),
            RGB::new(0xFF, 0xFF, 0),
            RGB::new(0xFF, 0xFF, 0x33),
            RGB::new(0xFF, 0xFF, 0x66),
            RGB::new(0xFF, 0xFF, 0x99),
            RGB::new(0xFF, 0xFF, 0xCC),
            RGB::new(0xFF, 0xFF, 0xFF),
        ],
        // Rainbow
        [
            RGB::new(0xFF, 0, 0),
            RGB::new(0xD5, 0x2A, 0),
            RGB::new(0xAB, 0x55, 0),
            RGB::new(0xAB, 0x7F, 0),
            RGB::new(0xAB, 0xAB, 0),
            RGB::new(0x56, 0xD5, 0),
            RGB::new(0, 0xFF, 0),
            RGB::new(0, 0xD5, 0x2A),
            RGB::new(0, 0xAB, 0x55),
            RGB::new(0, 0x56, 0xAA),
            RGB::new(0, 0, 0xFF),
            RGB::new(0x2A, 0, 0xD5),
            RGB::new(0x55, 0, 0xAB),
            RGB::new(0x7F, 0, 0x81),
            RGB::new(0xAB, 0, 0x55),
            RGB::new(0xD5, 0, 0x2B),
        ],
    ];
    
    fn new(spi_bus: SPI) -> Self {
        let device = ws2812::Ws2812::new(spi_bus);

        PwmFanRgb {
            color_mode: 0,
            brightness: 128u8,
            device,
        }
    }

    pub fn increment_mode(&mut self) -> Result <(), crate::error::Error> {
        self.color_mode += 1;
        self.color_mode %= u8::try_from(Self::MAX_MODES).unwrap();
        self.update()?;

        Ok(())
    }

    pub fn get_mode_text(&self) -> &str {
        match self.color_mode {
            0 => "Solid (Black)",
            c @ 1..=8 => {
                match c {
                    1 => "Solid (Red)     ",
                    2 => "Solid (OYellow) ",
                    3 => "Solid (Green)   ",
                    4 => "Solid (SBlue)   ",
                    5 => "Solid (Blue)    ",
                    6 => "Solid (Indigo)  ",
                    7 => "Solid (Violet)  ",
                    8 => "Solid (FPink)   ",
                    _ => "Solid (??)      ",
                }
            },
            c @ 9..=12 => {
                match c {
                    9 =>  "Palette (Forest)",
                    10 => "Palette (Cloud) ",
                    11 => "Palette (Heat)  ",
                    12 => "Palette (Rbow)  ",
                    _ =>  "Palette (???)   ",
                }
            },
            _ => "???             ",
        }
    }

    pub fn set_brightness(&mut self, brightness: u8) -> Result <(), crate::error::Error> {
        self.brightness = brightness;
        self.update()?;

        Ok(())
    }

    pub fn update(&mut self) -> Result <(), crate::error::Error> {
        match self.color_mode {
            c @ 1..=8 => {
                // Single color
                let fan_color_buffer = (0..Self::FAN_LED_QTY).map(|v| {
                    hsv2rgb(Hsv {hue: 36 * (c - 1), sat: 255, val: self.brightness})
                });

                self.device.write(fan_color_buffer).map_err(|_| crate::error::Error::SPI)?;
            }
            c @ 9..=12 => {
                // Color palette
                let now_color = Self::LED_COLOR_PALETTES[usize::try_from(c - 9).unwrap()];
                let val = stoptimer::beat_u8(128);
                // defmt::println!("beat {}", val);

                let fan_color_buffer = (0..Self::FAN_LED_QTY).map(|i| {
                    let idx = (i.wrapping_add(usize::try_from(val).unwrap())) % now_color.len();
                    
                    now_color[idx]
                });

                self.device.write(gamma(fan_color_buffer)).map_err(|_| crate::error::Error::SPI)?;
            }
            other => {
                // 0 - "black"
                if !(0..Self::MAX_MODES).contains(&(usize::from(other))) {
                    self.color_mode = 0;
                }

                let fan_color_buffer = (0..Self::FAN_LED_QTY).map(|v| {
                    hsv2rgb(Hsv {hue: 0, sat: 0, val: 0})
                });

                self.device.write(fan_color_buffer).map_err(|_| crate::error::Error::SPI)?;
            }
        }

        Ok(())
    }
}