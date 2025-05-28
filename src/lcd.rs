use embedded_hal::blocking::delay;
use stm32f4xx_hal::{
    i2c::{self, I2c},
};

use hd44780_driver::{bus::I2CBus, Cursor, CursorBlink, Display, DisplayMode, HD44780};

pub struct I2CLcd <'a, I2C, Delay>
    where I2C: i2c::Instance {
    device: HD44780 <I2CBus <I2c<I2C>>>,
    delay: &'a mut Delay,
}

impl <'a, I2C, Delay> I2CLcd <'a, I2C, Delay>
    where I2C: i2c::Instance,
    Delay: delay::DelayUs <u16> + delay::DelayMs <u8> {

    pub fn new(i2c_bus: I2c <I2C>, delay: &'a mut Delay) -> Result<Self, crate::error::Error> {
        let lcd_1602: HD44780<I2CBus<I2c<I2C>>> = HD44780::new_i2c(i2c_bus, 0x27, delay)?;

        Ok(Self {
            device: lcd_1602,
            delay,
        })
    }

    pub fn init(&mut self) -> Result <(), crate::error::Error> {
        self.device.reset(self.delay)?;
        self.device.clear(self.delay)?;

        self.device.set_display_mode(DisplayMode {
            display: Display::On,
            cursor_visibility: Cursor::Invisible,
            cursor_blink: CursorBlink::Off,
        }, self.delay)?;

        Ok(())
    }

    pub fn write_duty_cycle(&mut self, mut duty_cycle: u8) -> Result<(), crate::error::Error> {
        duty_cycle = duty_cycle.clamp(0, 100);
        let (duty_cycle_num_start, duty_cycle_bytes) = Self::from_number(u32::from(duty_cycle));

        self.device.set_cursor_pos(0, self.delay)?;
        self.device.write_str("Fan:     ", self.delay)?;
        self.device.set_cursor_pos(5, self.delay)?;
        self.device.write_bytes(&duty_cycle_bytes[duty_cycle_num_start..], self.delay)?;
        self.device.write_str("%", self.delay)?;

        Ok(())
    }

    pub fn write_number(&mut self, number: u32) -> Result<(), crate::error::Error> {
        self.device.set_cursor_pos(0, self.delay)?;
        self.device.write_str("DNum: ", self.delay)?;

        let (num_bytes_start, num_bytes) = Self::from_number(number);
        self.device.write_bytes(&num_bytes[num_bytes_start..], self.delay)?;

        Ok(())
    }

    pub fn write_message(&mut self, message: &str, position: (u8, u8)) -> Result<(), crate::error::Error> {
        self.device.set_cursor_pos(position.0 * 40 + position.1, self.delay)?;
        self.device.write_str(message, self.delay)?;

        Ok(())
    }

    fn from_number(mut number: u32) -> (usize, [u8; 10]) {
        let mut ans = [0u8; 10];
        let mut ans_start = 10usize;

        ans[9] = b'0';

        for v in ans.iter_mut().rev() {
            if number <= 0 {
                break;
            }

            *v = char::from_digit(u32::from(number % 10), 10)
                .map(|v| u8::try_from(v).unwrap())
                .unwrap();
            number /= 10;
            ans_start -= 1;
        }

        (ans_start, ans)
    }
}