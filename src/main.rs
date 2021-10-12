#![no_std]
#![no_main]
#![feature(const_fn_trait_bound)]

extern crate cortex_m_rt;
extern crate panic_halt;

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::usb::{Peripheral, UsbBus};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

mod sd;
pub use sd::{SdCardStatus, SpiSdCard};
mod driver;
use driver::Driver;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let (_a, _b, _c) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // SPI
    let spi_pins = (
        gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
        gpiob.pb14.into_floating_input(&mut gpiob.crh),
        gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
    );
    let spi_cs = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi2(
        dp.SPI2,
        spi_pins,
        spi_mode,
        100.khz(),
        clocks,
        &mut rcc.apb1,
    );
    let mut sd = SpiSdCard::new(spi, spi_cs);

    // LED
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // USB
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low().ok();
    delay(clocks.sysclk().0 / 100);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
    };
    let usb_bus = UsbBus::new(usb);

    let usb_serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1234, 0x1234))
        .manufacturer("alnyan")
        .product("card-o-matic")
        .serial_number("616c6e79616e01")
        .device_class(USB_CLASS_CDC)
        .build();

    usb_dev.force_reset().ok();

    delay(clocks.sysclk().0 / 100);

    // Don't care about the result: will have a chance to reinit
    sd.init().ok();

    let mut driver = Driver::new(sd, usb_dev, usb_serial);

    loop {
        driver.poll();
        led.toggle().ok(); // Turn off
    }
}
