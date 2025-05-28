use hd44780_driver;
use stm32f4xx_hal;

#[derive(Debug)]
pub enum Error {
    I2C,
    SPI,
    Generic,
}

impl From <hd44780_driver::error::Error> for Error {
    fn from(_: hd44780_driver::error::Error) -> Self {
        Error::I2C
    }
}

impl From <stm32f4xx_hal::spi::Error> for Error {
    fn from(_: stm32f4xx_hal::spi::Error) -> Self {
        Error::SPI
    }
}