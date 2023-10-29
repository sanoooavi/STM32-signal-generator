# STM32-signal-generator using SPI Communication: Proteus and Keil

This repository contains a project that demonstrates SPI communication between two STM32 microcontroller boards. The project utilizes Proteus for simulation and Keil as the development environment, along with the STM32 HAL library for firmware development.

## Features

- Establishes SPI communication between two STM32 microcontroller boards.
- Allows the user to select a waveform and its frequency.
- Sets the duration of the waveform display via a keypad input.
- Transmits waveform information from the master to the slave using SPI protocol.
- The slave board generates the desired waveform based on the received data.

## Getting Started

### Prerequisites

To set up and run the project, make sure you have the following software and hardware components:

#### Software

- Proteus for simulation (Proteus Professional and ISIS software).
- Keil MDK (Microcontroller Development Kit) for firmware development.
- STM32Cube firmware package for the respective STM32 boards.

#### Hardware

- Two STM32 microcontroller boards (e.g., STM32F4 Discovery boards).
- Jumper wires for connecting the boards.
- USB cables for power and programming.

### Installation and Configuration

1. Clone this repository to your local machine.

2. Install Proteus, Keil MDK, and the required STM32Cube firmware package.

3. Connect the STM32 boards as per the hardware requirements.

4. Open the project in Keil MDK and configure one board as the SPI master and the other as the SPI slave using the STM32 HAL library.
   - Set the SPI master's NSS pin as an output and keep it high when the bus is not in use.
   - Configure the SPI slave's NSS pin as an input.
   - Use the HAL library functions to configure the data frame format, clock polarity, and clock phase based on desired settings.

5. Implement the necessary SPI initialization and communication routines on both boards using the HAL library.
   - Initialize the SPI peripheral with appropriate settings using the HAL library functions.
   - Create functions to send and receive data over SPI using the HAL library functions.

## Usage

1. Power on the STM32 boards and ensure they are correctly programmed.

2. Use the master board to initiate SPI communication by sending commands or data.
   - Utilize the HAL library functions on the master board to send commands or data to the slave board.

3. Implement the appropriate handling on the slave board to receive and process the commands or data.
   - Utilize the HAL library functions on the slave board to receive and interpret the SPI messages.

4. Customize the firmware logic to perform the desired actions based on the received commands or data.
   - Implement the necessary code on both boards to interpret and respond to the received SPI messages.
   - Handle any error conditions or exceptional cases that may arise during SPI communication.

## Acknowledgements

- [Proteus](https://www.labcenter.com/)
- [Keil MDK](https://www.keil.com/mdk)
- [STM32Cube](https://www.st.com/en/embedded-software/stm32cube.html)
