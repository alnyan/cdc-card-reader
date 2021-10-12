use core::ops::Deref;
use cortex_m::asm::delay;
use cortex_m::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use nb::block;
use stm32f1xx_hal::{
    pac,
    gpio::{gpiob, Output, PushPull},
    spi::{Error as SpiError, Spi},
};

#[derive(Copy, Clone)]
pub enum SdCardStatus {
    Init,
    Ready(u64),
    Failed
}

#[derive(Debug)]
pub enum SdCardError {
    SpiError,
    Timeout,
    InvalidResponse
}

pub struct SpiSdCard<SPI: Deref<Target = pac::spi1::RegisterBlock>, REMAP, PINS> {
    spi: Spi<SPI, REMAP, PINS, u8>,
    cs: gpiob::PB12<Output<PushPull>>,

    status: SdCardStatus
}

impl From<SpiError> for SdCardError {
    fn from(_e: SpiError) -> Self {
        Self::SpiError
    }
}

impl<SPI: Deref<Target = pac::spi1::RegisterBlock>, REMAP, PINS> SpiSdCard<SPI, REMAP, PINS> {
    pub const fn new(spi: Spi<SPI, REMAP, PINS, u8>, cs: gpiob::PB12<Output<PushPull>>) -> Self {
        Self { spi, cs, status: SdCardStatus::Init }
    }

    pub fn txrx(&mut self, w: u8) -> Result<u8, SdCardError> {
        block!(self.spi.send(w))?;
        let res = block!(self.spi.read())?;
        Ok(res)
    }

    pub fn send_cmd(&mut self, cmd: u8, crc: u8, arg: u32) -> Result<u8, SdCardError> {
        self.txrx(cmd)?;

        self.txrx(((arg >> 24) & 0xFF) as u8)?;
        self.txrx(((arg >> 16) & 0xFF) as u8)?;
        self.txrx(((arg >> 8) & 0xFF) as u8)?;
        self.txrx((arg & 0xFF) as u8)?;

        self.txrx(crc)?;

        for _ in 0..10 {
            let tmp = self.txrx(0xFF)?;
            if tmp & 0x80 == 0 {
                return Ok(tmp);
            }
        }

        Err(SdCardError::InvalidResponse)
    }

    pub fn read_sector(&mut self, addr: u32, buf: &mut [u8; 512]) -> Result<(), SdCardError> {
        let mut tmp = 0u8;
        self.cs.set_low().ok();
        delay(10000);

        self.send_cmd(0x40 + 17, 0x00, addr)?;

        for _ in 0..10 {
            tmp = self.txrx(0xFF)?;
            if tmp != 0xFF {
                break;
            }
        }

        if tmp == 0xFE {
            for byte in buf.iter_mut() {
                *byte = self.txrx(0xFF)?;
            }
            self.txrx(0xFF)?;
            self.txrx(0xFF)?;
            self.cs.set_high().ok();
            delay(10000);
            Ok(())
        } else {
            self.cs.set_high().ok();
            delay(10000);
            Err(SdCardError::InvalidResponse)
        }
    }

    pub fn write_sector(&mut self, addr: u32, buf: &[u8; 512]) -> Result<(), SdCardError> {
        let mut tmp = 0u8;
        let res;
        self.cs.set_low().ok();
        delay(10000);

        self.send_cmd(0x40 + 24, 0x00, addr)?;

        // Wait for SD to become ready
        for _ in 0..10 {
            tmp = self.txrx(0xFF)?;
            if tmp != 0xFF {
                break;
            }
        }

        if tmp == 0xFF {
            delay(10000);
            self.txrx(0xFE)?;

            for &byte in buf {
                self.txrx(byte)?;
            }

            // Discard CRC
            self.txrx(0xFF)?;
            self.txrx(0xFF)?;

            for _ in 0..64 {
                tmp = self.txrx(0xFF)?;
                if (tmp & 0x1F) == 0x05 {
                    break;
                }
            }
            res = if (tmp & 0x1F) == 0x05 {
                Ok(())
            } else {
                Err(SdCardError::InvalidResponse)
            };

            while self.txrx(0xFF)? == 0 {
                delay(100);
            }

            res
        } else {
            Err(SdCardError::InvalidResponse)
        }
    }

    pub fn rx_data_block(&mut self, buf: &mut [u8]) -> Result<(), SdCardError> {
        let mut tmp = 0u8;

        delay(10000);
        for _ in 0..10 {
            tmp = self.txrx(0xFF)?;
            if tmp != 0xFF {
                break;
            }
            delay(10000);
        }
        if tmp == 0xFF {
            return Err(SdCardError::Timeout);
        }

        if tmp != 0xFE {
            return Err(SdCardError::InvalidResponse);
        }

        for byte in buf.iter_mut() {
            *byte = self.txrx(0xFF)?;
        }

        // Discard CRC
        self.txrx(0xFF)?;
        self.txrx(0xFF)?;

        Ok(())
    }

    pub fn get_capacity(&mut self) -> Result<u64, SdCardError> {
        let size: u64;
        let res = self.send_cmd(0x40 + 9, 0x00, 0x01)?;

        if res == 0 {
            let mut csd = [0u8; 16];
            self.rx_data_block(&mut csd)?;
            if csd[0] >> 6 == 1 {
                // SDCv2
                let csize = (csd[9] as u32) + ((csd[8] as u32) << 8) + 1;
                size = (csize as u64) << 10;
            } else {
                todo!()
            }
        } else {
            todo!()
        }

        Ok(size)
    }

    pub fn status(&self) -> SdCardStatus {
        self.status
    }

    fn init_inner(&mut self) -> Result<u64, SdCardError> {
        delay(100000);
        for _ in 0..10 {
            self.txrx(0xFF)?;
        }

        // CMD0: software reset
        let res = self.send_cmd(0x40, 0x95, 0)?;
        self.txrx(0xFF)?;
        if res & 0x7F != 1 {
            return Err(SdCardError::InvalidResponse);
        }
        delay(100000);

        // CMD8: set voltage to 2.7-3.3V
        let res = self.send_cmd(0x40 + 8, 0x86, 0x1AA)?;
        if res == 0x01 {
            let mut buf = [0u8; 4];
            for byte in buf.iter_mut() {
                *byte = self.txrx(0xFF)?;
            }

            // TODO validate that result is 0x1AA
        }
        self.txrx(0xFF)?;
        if res != 0x01 {
            return Err(SdCardError::InvalidResponse);
        }
        delay(100000);

        // ACMD41: init sd card
        self.send_cmd(0x40 + 55, 0x00, 0x00)?;
        self.send_cmd(0x40 + 41, 0x00, 0x40000000)?;
        delay(1000000);

        // Try ACMD1 until R1 == 0
        for _ in 0..10 {
            self.send_cmd(0x40 + 55, 0x00, 0x0)?;
            let res = self.send_cmd(0x40 + 41, 0x00, 0x40000000)?;
            delay(100000);

            if res == 0x00 {
                // Identify card
                return self.get_capacity();
            }

            delay(100000);
        }

        Err(SdCardError::Timeout)
    }

    pub fn init(&mut self) -> Result<u64, SdCardError> {
        self.status = SdCardStatus::Init;
        self.cs.set_low().ok();
        let res = self.init_inner();
        self.cs.set_high().ok();
        match res {
            Err(_) => {
                self.status = SdCardStatus::Failed;
            }
            Ok(cap) => {
                self.status = SdCardStatus::Ready(cap);
            }
        }
        res
    }
}
