# Propeller Thrust Stand

ESP32-based thrust stand for measuring real-world propeller thrust with an HX711 load cell. Readings are served live over WiFi at `http://thruststand.local`, and the ESC/motor is controlled from the same web interface. Collected data will be correlated against ANSYS Fluent CFD models of each propeller.

## Hardware

- ESP32 dev board
- HX711 load cell amplifier (DT → GPIO 14, SCK → GPIO 13)
- Load cell, ESC + brushless motor

## Setup

1. Install [PlatformIO](https://platformio.org/) (VS Code extension)
2. Copy `include/secrets.h.example` to `include/secrets.h` and fill in your WiFi credentials
3. Run the calibration sketch, then set `CALIBRATION_FACTOR` in `src/main.cpp`
4. Build and upload: `pio run -t upload`

## Author

Braden Gallagher — [portfolio](https://bmoney2029.github.io/portfolio/) · [LinkedIn](https://www.linkedin.com/in/braden-gallagher)
