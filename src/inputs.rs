/* */
use stm32f4xx_hal::{
    adc, gpio::{EPin, Input}, pac
};

use embedded_hal;
use crate::stoptimer;

pub struct DebouncedDInput {
    pin: EPin <Input>,
    delay_millis: u32,
    now_state: bool,
    last_state: bool,
    last_millis: u32,
}

pub enum DebouncedOutput {
    Constant(bool),
    Changed(bool),
}

pub struct PotRead {
    device: adc::Adc <pac::ADC1>,
}

impl DebouncedDInput {
    pub fn with_pullup(pin: EPin <Input>) -> Self {
        Self {
            pin: pin,
            delay_millis: 10,
            now_state: true,
            last_state: true,
            last_millis: 0,
        }
    }

    pub fn is_low(&mut self) -> DebouncedOutput {
        match self.read() {
            DebouncedOutput::Constant(v) => DebouncedOutput::Constant(!v),
            DebouncedOutput::Changed(v) => DebouncedOutput::Changed(!v),
        }
    }

    pub fn is_high(&mut self) -> DebouncedOutput {
        self.read()
    }

    fn read(&mut self) -> DebouncedOutput {
        let is_pin_high = self.pin.is_high();
        let mut ans = DebouncedOutput::Constant(self.now_state);
        
        // defmt::println!("millis {}", stoptimer::get_millis());
        if is_pin_high != self.last_state {
            self.last_millis = stoptimer::get_millis();
        } 

        if stoptimer::get_millis().saturating_sub(self.last_millis) > self.delay_millis && is_pin_high != self.now_state {
            self.now_state = is_pin_high;
            ans = DebouncedOutput::Changed(self.now_state);
        }

        self.last_state = is_pin_high;
        ans
    }
}

impl PotRead {
    pub fn with_adc01(pin: impl embedded_hal::adc::Channel <pac::ADC1, ID = u8>, adc: pac::ADC1) -> Self {
        // Input analog pot read
        let mut adc01 = adc::Adc::adc1(adc, true, adc::config::AdcConfig::default());
        adc01.configure_channel(&pin, adc::config::Sequence::One, adc::config::SampleTime::Cycles_480);

        Self {
            device: adc01,
        }
    }

    pub fn read_percent(&mut self) -> u16 {
        if !self.device.is_enabled() {        
            self.device.enable();
        }

        self.device.start_conversion();
        let sample = self.device.current_sample();
        let percent = u32::from(sample).saturating_mul(100).saturating_div(1u32 << 12).clamp(0, 100);

        u16::try_from(percent).unwrap()
    }
}
