# Pico PID with LCD user interface
Make a Raspberry Pi Pico into a graphical PID controller. Runs using MicroPython.

![Image of completed device in action](https://github.com/grunkyb/pico-pid-lcd/blob/main/images/IMG_9144.png "PID GUI in action")

## Requirements

### For the controller
- Raspberry Pi Pico with male headers
- [Pico 0.96" LCD with buttons](https://www.waveshare.com/wiki/Pico-LCD-0.96)
- BME280 sensor with I2C connection
- Jumper wires

### For the heater
- N-channel power MOSFET with logic-level gating (e.g., [IRLB8721](https://thepihut.com/products/n-channel-power-mosfet))
- DC power supply
- Resistive heating elements (e.g., an [electric heating pad](https://thepihut.com/products/electric-heating-pad-10cm-x-5cm))
- Female DC 2.1mm jack for power adapter (depending on power supply wiring)

## Hardware setup

### Pins taken by LCD display and buttons
| GP# | Pin | Function |
| :---: | :---: | :--- |
|  2  |  4 | Joystick up
|  3  |  5 | Joystick centre press
|  8  | 11 | SPI DI1 data (high) command (low)
|  9  | 12 | SPI CS1 chip select (low active)
| 10  | 14 | SPI SCK1 clock input
| 11  | 15 | SPI DO1 data input
| 12  | 16 | Reset (low active)
| 13  | 17 | Backlight
| 15  | 20 | User key A
| 16  | 21 | Joystick left
| 17  | 22 | User key B
| 18  | 24 | Joystick down
| 20  | 25 | Joystick right
| VSYS | 39 | 1.8 to 5.5V power supply

## Software setup

### Loading on to the Raspberry Pi Pico

### Settings file

Settings are stored in JSON format in `settings.json`.

| Key | Meaning | Typical value |
| --- | --- | --- |
| setpoint_C | Setpoint temperature in °C | 37.0
| Kp | xxx | 49248 |
| smoothingalpha | xxx | 0.7 |
| sampletime_s | xxx | 1.0 |
| heaterpin | GPIO number controlling the heater output by PWM | 0 |
| displaysleep_s | xxx | 30 |
| outmin | xxx | 0 |
| outmax | xxx | 65535 |
| tempbar_frac | xxx | 0.6 |
| i2cbus | xxx | 0 |
| Kd | xxx | 963413 |
| Ki | xxx | 629.4 |
| scl_pin | xxx | 5 |
| sda_pin | xxx | 4 |
| heaterfreq_Hz| xxx | 16384 |
| displayupdate_Hz | xxx | 5 |
| lookback_points | xxx | 60 |
| tempoffset_C | xxx | 0 |
| pressureoffset_hPa | xxx | 0 |
| rhoffset_pct | xxx | 0 |
| tuningrule | xxx | "ziegler-nichols" |

## Using the device
