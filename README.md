# Smart THC

![Smart THC Logo](https://github.com/guiguijke/SmartTHC/blob/main/SmartTHC_LOGO_PNG.png)

Welcome to **Smart THC**, an open-source Arduino-based Torch Height Controller (THC) designed for CNC plasma cutting machines. Smart THC ensures precise control of the torch height by adjusting it dynamically based on the plasma arc voltage, delivering clean and accurate cuts. With a user-friendly LCD interface, real-time parameter adjustments via an encoder, and robust PID control, this project is ideal for hobbyists and professionals looking to enhance their plasma cutting setups.

## Project Overview

Smart THC (Torch Height Controller) automates the vertical positioning of a plasma cutting torch to maintain optimal cutting height. It uses an Arduino microcontroller to monitor the plasma arc voltage, control a stepper motor for Z-axis movement, and provide real-time feedback on an LCD display. Key features include:

- **Dynamic Height Adjustment**: Uses a PID controller to maintain a target arc voltage, ensuring consistent cut quality.
- **Stepper Motor Control**: Precisely drives the Z-axis stepper motor for smooth torch movement.
- **Interactive Interface**: A 16x2 LCD with custom characters and an encoder for navigating menus and adjusting settings like target voltage, PID coefficients, and cutting speed.
- **Anti-Dive Protection**: Prevents the torch from diving too low during rapid voltage changes, improving cut reliability.
- **EEPROM Storage**: Saves user settings (e.g., PID parameters, target voltage) for persistence across power cycles.
- **Future Imperial Units Support**: Upcoming version will include measurements in inches and other imperial units.

Smart THC is licensed under the [GNU General Public License v3 (GPL v3)](#license), allowing you to modify and share it freely, provided derivative works are also licensed under GPL v3. A premium documentation package with detailed guides and advanced tuning tips is planned for release soon.

## Features

- **Precise Torch Height Control**: Maintains optimal cutting height using PID-based arc voltage regulation.
- **Real-time Monitoring**: Displays arc voltage, torch speed, and system status on a 16x2 LCD.
- **User-Friendly Configuration**: Adjust settings like target voltage, PID gains (Kp, Ki, Kd), cutting speed, and voltage correction factor via an encoder.
- **Anti-Dive Mechanism**: Detects rapid voltage drops to prevent torch diving, with adaptive timing based on cutting speed.
- **Speed Calculation**: Measures torch travel speed in mm/min using X/Y-axis step counts, with filtering for accuracy.
- **Persistent Settings**: Stores user configurations in EEPROM for seamless operation.
- **Extensible Design**: Easily adaptable for different plasma cutters, sensors, or CNC setups.

## Hardware Requirements

To build and run Smart THC, you'll need:

- **Arduino Board**: Arduino Uno R4 Minima (or wifi).
- **LCD Display**: 16x2 LCD with I2C interface.
- **Stepper Motor and Driver**: 4-wire stepper motor with a driver (e.g., ULN2003 or A4988) for Z-axis movement.
- **Rotary Encoder**: For menu navigation and parameter adjustment (with CLK, DT, and SW pins).
- **Plasma Arc Sensor**: Voltage divider or ADC input to measure plasma arc voltage (connected to A0).
- **CNC Stepper Signals**: X/Y-axis step signals (connected to interrupt pins for speed calculation).
- **1 Switch**: For enabling/disabling THC and detecting plasma arc status.
- **Jumper Wires and Breadboard**: For prototyping and connections.

## Software Requirements

Smart THC is developed using **Visual Studio Code** with the **PlatformIO** extension, which we recommend for an efficient workflow. The project depends on the following Arduino libraries:

- **LiquidCrystal_I2C.h**: Controls the I2C LCD display for real-time feedback.
- **AccelStepper.h**: Manages the Z-axis stepper motor with acceleration support.
- **ArduPID.h**: Implements the PID controller for arc voltage regulation.

You'll also need:
- **Visual Studio Code**: Latest version.
- **PlatformIO Extension**: For project management, compilation, and uploading.
- **Arduino Framework**: Configured via PlatformIO.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/your-username/Smart-THC.git
   ```
   Replace `your-username` with your GitHub username.

2. **Set Up PlatformIO**:
   - Install [Visual Studio Code](https://code.visualstudio.com/).
   - Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode).
   - Open the `Smart-THC` folder in VS Code.
   - PlatformIO will detect the `platformio.ini` file and install dependencies.

3. **Install Library Dependencies**:
   - The required libraries (`LiquidCrystal_I2C`, `AccelStepper`, `ArduPID`) are specified in `platformio.ini`.
   - PlatformIO will download them automatically during the build process.
   - To manually install, use:
     ```bash
     pio lib install "LiquidCrystal_I2C"
     pio lib install "AccelStepper"
     pio lib install "ArduPID"
     ```

4. **Configure Hardware Settings**:
   - Verify the I2C address of your LCD (default: 0x27) in `src/main.cpp`.
   - Check pin assignments for the stepper motor, encoder, and plasma arc input (defined in the code).
   - Ensure your Arduino board is set in `platformio.ini` (e.g., `board = uno_r4_minima`).

5. **Build and Upload**:
   - Connect your Arduino board via USB.
   - In VS Code, open the PlatformIO sidebar (alien icon).
   - Click `Build` to compile the code.
   - Click `Upload` to flash the code to your Arduino.

6. **Connect the Hardware**:
   - Wire the LCD, stepper motor, encoder, and sensors according to the pin definitions in `src/main.cpp`.
   - Ensure the plasma arc voltage input is properly scaled (e.g., via a voltage divider) to avoid damaging the Arduino.

## Usage

Usage will be shared in a paid documentation, this will encourage me to continue developpment!

## Upcoming Features

- **Imperial Units Support**: Future versions will include measurements in inches and other imperial units for broader compatibility.
- **Enhanced EEPROM Management**: Improved storage and retrieval of user settings.
- **Advanced Anti-Dive Algorithms**: Further optimization for varying cutting conditions.

## License

Smart THC is licensed under the [GNU General Public License v3 (GPL v3)](LICENSE). You are free to use, modify, and distribute the code, provided all derivative works are also licensed under GPL v3. The included libraries have their own licenses, which are respected:

- **LiquidCrystal_I2C**: Often under CC BY-SA 3.0 or similar (check the library version).
- **AccelStepper**: GNU General Public License v3 (GPL v3).
- **ArduPID**: MIT License.

See the [LICENSE](LICENSE) file and library documentation for details.

## Premium Documentation

While the code is open source, a premium documentation package is in development, including:
- Step-by-step hardware setup guides.
- Customizing anti-dive and speed thresholds.
- Integration with various CNC plasma systems.

Stay tuned for availability and pricing details!

## Contact

Questions or feedback? Open an issue on [GitHub Issues](https://github.com/your-username/Smart-THC/issues) or join my [Discord](https://discord.gg/Z9JJdjPDb4) server to talk to me directly.

Happy cutting with Smart THC!

---

**Acknowledgments**  
- Thanks to the authors of `LiquidCrystal_I2C`, `AccelStepper`, and `ArduPID` for their excellent libraries.
- Gratitude to the Arduino, PlatformIO, and CNC plasma cutting communities for their inspiration and support.
