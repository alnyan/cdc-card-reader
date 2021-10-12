TARGET=thumbv7m-none-eabi
OBJCOPY?=$(shell llvm-config --bindir)/llvm-objcopy
O=target/$(TARGET)/release

all:
	cargo build --release
	$(OBJCOPY) -O ihex $(O)/stm32f103 $(O)/stm32f103.hex

flash: all
	st-flash --format ihex write $(O)/stm32f103.hex
