# Pico PID with LCD user interface

How to make a Raspberry Pi Pico into a graphical PID controller. Runs using MicroPython.

![Image of completed device in action](https://github.com/grunkyb/pico-pid-lcd/blob/main/images/LCD_display_heater_on.png "PID GUI in action")

## Requirements

### For the controller
- Raspberry Pi Pico with male headers
- [Pico 0.96" LCD with buttons](https://www.waveshare.com/wiki/Pico-LCD-0.96)
- BME280 sensor breakout with I2C connection
- Jumper wires

### For the heater
- N-channel power MOSFET with logic-level gating (e.g., [IRLB8721](https://thepihut.com/products/n-channel-power-mosfet))
- DC power supply
- Resistive heating elements (e.g., an [electric heating pad](https://thepihut.com/products/electric-heating-pad-10cm-x-5cm))
- Female DC 2.1mm jack for power adapter (depending on power supply wiring)

## Hardware setup

### Wiring the heater circuit

From left to right in the image below (heat sink at the back), the pins are gate, drain, and source.

[![TO-220 front](https://upload.wikimedia.org/wikipedia/commons/thumb/a/a6/TO-220_Front_Coloured.svg/550px-TO-220_Front_Coloured.svg.png)](https://en.wikipedia.org/wiki/TO-220)

- Connect the gate to the PWM output (default: GP0, pin 1).
- Connect the source to one of the Pico's ground pins.
- Connect the drain to one of the resistive heater's connectors.
- Connect the positive (red) line from the power supply to the other connector on the resistive heater
- Connect the negative (black) line from the power supply to one of the Pico's ground pins

### Connections to BME280

If not already in place, headers need to be soldered to the BME280 breakout, then connecgted with jumper wires. [Adafruit](https://www.adafruit.com/product/2652) and others sell a BME280 breakout with [STEMMA QT](https://learn.adafruit.com/introducing-adafruit-stemma-qt/what-is-stemma-qt) connections.

![BME280 with jumpers](https://github.com/grunkyb/pico-pid-lcd/blob/main/images/BME280_with_jumpers.png)

The Pico pinout is given in the image below.

[![Pico pinout](https://projects-static.raspberrypi.org/projects/getting-started-with-the-pico/991cb74a9ee566023ff2811e49fe0447d80966db/en/images/Pico-R3-Pinout.png)](https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/1)

The default pins for the Pico's I2C bus 0 are taken by the LCD display's SPI bus. The connections in the settings file use I2C bus 0 with GP4 (header pin 6) for data (SDA) and GP5 (header pin 7) for clock (SCL). If the default pin for controlling the heater level is not used the following combinations are available.

| SDA GP# (Pin #) | SCL GP# (Pin #) | I2C bus |
| :---: | :---: | :---: |
| 0 (1) | 1 (2) | 0 |
| 6 (9) | 7 (10) | 1 |
| 26 (31) | 27 (32) | 1 |

VIN on the BME280 should be connected to 3V3 out (header pin 36). GND should be connected to any of the free ground connections on the Pico.

### Pins taken by LCD display and buttons
| GP# | Pin | Function |
| :---: | :---: | :--- |
|  2  |  4 | Joystick up |
|  3  |  5 | Joystick centre press |
|  8  | 11 | SPI DI1 data (high) command (low) |
|  9  | 12 | SPI CS1 chip select (low active) |
| 10  | 14 | SPI SCK1 clock input |
| 11  | 15 | SPI DO1 data input |
| 12  | 16 | Reset (low active) |
| 13  | 17 | Backlight |
| 15  | 20 | User key A |
| 16  | 21 | Joystick left |
| 17  | 22 | User key B |
| 18  | 24 | Joystick down |
| 20  | 25 | Joystick right |
| VSYS | 39 | 1.8 to 5.5V power supply |

## Software setup

### Installing MicroPython on the Raspberry Pi Pico

The Raspberry Pi Foundation offer a [tutorial on installing MicroPython on a Pico](https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/1).

### Files to control the PID and LCD

The files in the code folder should be copied to the Pico. This can be done using Thonny.

* If there are no menus at the top to the Thonny window, switch to regular mode using the hyperlink in the top-right of the window and restart Thonny.
* Select View | Files
* Right click on the files to transfer and select Upload to /.

Rename `pico-pid.py` to `main.py` for the program to launch at startup.

### Settings file

Settings are stored in JSON format in `settings.json`.

| Key | Meaning | Typical value |
| --- | --- | --- |
| `setpoint_C` | setpoint temperature in °C | `37.0` |
| `smoothingalpha` | [Exponential smoothing factor](https://en.wikipedia.org/wiki/Exponential_smoothing) 0 < &alpha; ≤ 1 | `0.7` |
| `sampletime_s` | seconds between BME280 measurements | `1.0` |
| `heaterpin` | GPIO number controlling the heater output by PWM | `0` |
| `displaysleep_s` | seconds without action before LCD backlight dims | `30` |
| `outmin` | minimum PWM output for heater control | `0` |
| `outmax` | maximum PWM output for heater control | `65535` |
| `tempbar_frac` | fraction of screen width where vertical setpoint bar appears on temperature line | `0.6` |
| `i2cbus` | I2C bus used to connect to BME280 | `0` |
| `sda_pin` | I2C data GPIO | `4` |
| `scl_pin` | I2C clock GPIO | `5` |
| `Kp` | proportional gain parameter in [parallel-form PID](https://en.wikipedia.org/wiki/PID_controller#Alternative_nomenclature_and_forms) equation | `49248` |
| `Ki` | integral gain parameter in [parallel-form PID](https://en.wikipedia.org/wiki/PID_controller#Alternative_nomenclature_and_forms) equation (=*K*<sub>p</sub>/*T*<sub>i</sub>) | `629.4` |
| `Kd` | derivative gain parameter in [parallel-form PID](https://en.wikipedia.org/wiki/PID_controller#Alternative_nomenclature_and_forms) equation (=*K*<sub>p</sub> *T*<sub>d</sub>) | `963413` |
| `heaterfreq_Hz` | PWM frequency in Hz | `16384` |
| `displayupdate_Hz` | number of times per second the controls are checked | `5` |
| `lookback_points` | number of measurements points used in autotune to find peaks or troughs | `60` |
| `tempoffset_C` | number of degrees to add on to reading from BME280 | `0.0` |
| `pressureoffset_hPa` | number of hPa to add to reading from BME280 | `0.0` |
| `rhoffset_pct` | number of percentage points to add to reading from BME280 | `0.0` |
| `tuningrule` | set of empirical parameters to convert amplitude and period of tuning into PID gain parameters | `"ziegler-nichols"` |

## Using the device

### Starting and stopping

Button A

### Change the temperature setpoint

Joystick

### Autotune

Button B


