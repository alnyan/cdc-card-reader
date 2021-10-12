use crate::{SdCardStatus, SpiSdCard, sd::SdCardError};
use core::ops::Deref;
use stm32f1xx_hal::pac::spi1::RegisterBlock as SpiRegs;
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

const CMD_STATUS: u8 = 0x02;
const CMD_READ: u8 = 0x03;
const CMD_WRITE: u8 = 0x04;
const CMD_END: u8 = 0xF3;

const CMD_STATUS_GENERAL: u8 = 0x00;
const CMD_STATUS_INIT: u8 = 0x01;

static mut WRITEBUF: [u8; 512] = [0; 512];

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
enum Status {
    Ok = 0x01,
    SerialFault = 0x80,
    SdFault = 0x81,
    SdTimeout = 0x82,
    SdNotReady = 0x83,
    SdTransportFault = 0x84
}

enum Command {
    Status(u8),
    Read(u64),
    Write(u64),
}

struct PendingWrite {
    lba: u64,
    off: usize,
}

pub struct Driver<'a, SPI: Deref<Target = SpiRegs>, REMAP, PINS, B: bus::UsbBus> {
    sd: SpiSdCard<SPI, REMAP, PINS>,

    cdc: SerialPort<'a, B>,
    dev: UsbDevice<'a, B>,
    cmd_buffer: [u8; 32],
    cmd_len: usize,

    write: Option<PendingWrite>,
}

trait Writer {
    fn send_byte(&mut self, b: u8);

    fn send_half_le(&mut self, w: u16) {
        self.send_byte((w & 0xFF) as u8);
        self.send_byte((w >> 8) as u8);
    }

    fn send_word_le(&mut self, w: u32) {
        self.send_byte((w & 0xFF) as u8);
        self.send_byte(((w >> 8) & 0xFF) as u8);
        self.send_byte(((w >> 16) & 0xFF) as u8);
        self.send_byte((w >> 24) as u8);
    }

    fn send_dword_le(&mut self, w: u64) {
        self.send_word_le((w & 0xFFFFFFFF) as u32);
        self.send_word_le((w >> 32) as u32);
    }
}

fn read_dword_le(b: &[u8]) -> u64 {
    (b[0] as u64)
        | ((b[1] as u64) << 8)
        | ((b[2] as u64) << 16)
        | ((b[3] as u64) << 24)
        | ((b[4] as u64) << 32)
        | ((b[5] as u64) << 40)
        | ((b[6] as u64) << 48)
        | ((b[7] as u64) << 56)
}

impl From<SdCardError> for Status {
    fn from(t: SdCardError) -> Self {
        match t {
            SdCardError::SpiError => Status::SdTransportFault,
            SdCardError::Timeout => Status::SdTimeout,
            SdCardError::InvalidResponse => Status::SdFault
        }
    }
}

impl TryFrom<&[u8]> for Command {
    type Error = ();

    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        if bytes.is_empty() {
            return Err(());
        }

        match bytes[0] {
            CMD_STATUS => {
                if bytes.len() != 2 {
                    return Err(());
                }
                Ok(Command::Status(bytes[1]))
            }
            CMD_READ => {
                if bytes.len() != 10 {
                    return Err(());
                }
                Ok(Command::Read(read_dword_le(&bytes[1..])))
            }
            CMD_WRITE => {
                if bytes.len() != 10 {
                    return Err(());
                }
                Ok(Command::Write(read_dword_le(&bytes[1..])))
            }
            _ => {
                todo!()
            }
        }
    }
}

impl<'a, SPI: Deref<Target = SpiRegs>, R, P, B: bus::UsbBus> Writer for Driver<'a, SPI, R, P, B> {
    fn send_byte(&mut self, b: u8) {
        self.cdc.write(&[b]).ok();
    }
}

impl<'a, SPI: Deref<Target = SpiRegs>, R, P, B: bus::UsbBus> Driver<'a, SPI, R, P, B> {
    pub const fn new(
        sd: SpiSdCard<SPI, R, P>,
        dev: UsbDevice<'a, B>,
        cdc: SerialPort<'a, B>,
    ) -> Self {
        Self {
            sd,

            dev,
            cdc,
            cmd_buffer: [0; 32],
            cmd_len: 0,

            write: None,
        }
    }

    fn handle_status(&mut self, mode: u8) {
        match mode {
            CMD_STATUS_GENERAL => {
                // status: u8
                // _0: u8
                // max_lba: u64
                let (status, cap) = match self.sd.status() {
                    SdCardStatus::Ready(cap) => (Status::Ok, cap),
                    SdCardStatus::Init => (Status::SdNotReady, u64::MAX),
                    _ => (Status::SdFault, u64::MAX),
                };

                self.send_byte(10);
                self.send_byte(status as u8);
                self.send_byte(0x00);
                self.send_dword_le(cap);
            }
            CMD_STATUS_INIT => {
                let status = match self.sd.init() {
                    Ok(_) => Status::Ok,
                    Err(e) => e.into()
                };

                self.send_byte(1);
                self.send_byte(status as u8);
            }
            _ => {}
        }

        self.cdc.flush().ok();
    }

    fn handle_read(&mut self, lba: u64) {
        // Send sector data

        let mut buf = [0u8; 512];
        match self.sd.read_sector(lba as u32, &mut buf) {
            Ok(_) => {
                self.send_byte(1);
                self.send_byte(Status::Ok as u8);

                for &byte in &buf {
                    self.send_byte(byte);
                }
            }
            Err(e) => {
                self.send_byte(1);
                self.send_byte(Status::from(e) as u8);
            }
        }

        self.cdc.flush().ok();
    }

    fn handle_write_begin(&mut self, lba: u64) {
        match self.sd.status() {
            SdCardStatus::Ready(_) => {
                self.write = Some(PendingWrite { lba, off: 0 });
                self.send_byte(1);
                self.send_byte(Status::Ok as u8);
                self.cdc.flush().ok();
            }
            _ => {
                self.send_byte(1);
                self.send_byte(Status::SdNotReady as u8);
                self.cdc.flush().ok();
            }
        }
    }

    fn handle_write_byte(&mut self, b: u8) {
        if let Some(ref mut write) = self.write {
            assert!(write.off != 512);

            unsafe {
                WRITEBUF[write.off] = b;
            }
            write.off += 1;

            if write.off == 512 {
                // Acknowledge write
                self.handle_write_done();
            }
        }
    }

    fn handle_write_done(&mut self) {
        if let Some(ref mut write) = self.write {
            let status = match self.sd.write_sector(write.lba as u32, unsafe { &WRITEBUF }) {
                Ok(_) => Status::Ok,
                Err(e) => e.into(),
            };
            self.send_byte(1);
            self.send_byte(status as u8);
            self.cdc.flush().ok();
        } else {
            self.send_byte(1);
            self.send_byte(Status::SerialFault as u8);
            self.cdc.flush().ok();
        }
        self.write = None;
    }

    fn handle_command(&mut self, cmd: Command) {
        match cmd {
            Command::Status(mode) => self.handle_status(mode),
            Command::Read(lba) => self.handle_read(lba),
            Command::Write(lba) => self.handle_write_begin(lba),
        }
    }

    pub fn poll(&mut self) {
        let mut buf = [0u8; 16];

        if !self.dev.poll(&mut [&mut self.cdc]) {
            return;
        }

        match self.cdc.read(&mut buf) {
            Ok(count) if count > 0 => {
                for &byte in &buf[0..count] {
                    if self.write.is_some() {
                        self.handle_write_byte(byte);
                        continue;
                    }

                    if byte == CMD_END {
                        if self.cmd_len == 0 {
                            continue;
                        }
                        if let Ok(cmd) = Command::try_from(&self.cmd_buffer[0..self.cmd_len]) {
                            self.handle_command(cmd);
                        }
                        self.cmd_len = 0;
                        continue;
                    }

                    if self.cmd_len == self.cmd_buffer.len() {
                        self.cmd_len = 0;
                    }
                    self.cmd_buffer[self.cmd_len] = byte;
                    self.cmd_len += 1;
                }
            }
            _ => {}
        }
    }
}
