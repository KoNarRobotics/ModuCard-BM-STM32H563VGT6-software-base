# ModuCard-BM-STM32H563VGT6 Software Base

Minimum software configuration for ModuCard base module board with STM32H563VGT6 MCU.

## Features

- **MCU**: STM32H563VGT6 (Cortex-M33, 250 MHz)
- **Memory**: 2MB Flash, 640KB RAM
- **Peripherals**: Basic GPIO initialization
- **Example**: LED blink on PA5

## Project Structure

```
.
├── Core/
│   ├── Inc/              # Header files
│   │   ├── stm32h5xx.h
│   │   ├── stm32h563xx.h
│   │   └── core_cm33.h
│   └── Src/              # Source files
│       ├── main.c
│       ├── system_stm32h5xx.c
│       └── startup_stm32h563xx.s
├── STM32H563VGTx_FLASH.ld  # Linker script
├── Makefile
└── README.md
```

## Prerequisites

- **ARM GCC Toolchain**: `arm-none-eabi-gcc`
- **GNU Make**: Build system
- **OpenOCD** or **STM32CubeProgrammer**: For flashing (optional)

### Installing ARM GCC Toolchain

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi
```

**macOS:**
```bash
brew install arm-none-eabi-gcc
```

**Windows:**
Download from [ARM Developer website](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)

## Building

To build the project:

```bash
make
```

This will generate:
- `build/STM32H563VGT6_ModuCard.elf` - ELF binary
- `build/STM32H563VGT6_ModuCard.hex` - Intel HEX format
- `build/STM32H563VGT6_ModuCard.bin` - Raw binary format

To clean build artifacts:

```bash
make clean
```

## Flashing

### Using OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32h5x.cfg -c "program build/STM32H563VGT6_ModuCard.elf verify reset exit"
```

### Using STM32CubeProgrammer

```bash
STM32_Programmer_CLI -c port=SWD -w build/STM32H563VGT6_ModuCard.hex -v -rst
```

## Hardware Configuration

The example code toggles LED on **PA5**. Connect an LED with appropriate current-limiting resistor:

- **PA5** -> LED Anode
- LED Cathode -> GND (via 220-330Ω resistor)

## System Configuration

- **Clock Source**: HSI (High-Speed Internal) @ 64 MHz
- **System Clock**: 64 MHz
- **GPIO**: PA5 configured as push-pull output

## Customization

To modify GPIO pins or add functionality:

1. Edit `Core/Src/main.c`
2. Update GPIO initialization in `GPIO_Init()` function
3. Add new peripheral initializations as needed

## License

This is a minimal starter template for STM32H563VGT6 development.

## Support

For issues or questions, please open an issue in the repository.
