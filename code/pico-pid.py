#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
MicroPython PID controller with display for Raspberry Pi Pico
https://github.com/grunkyb/pico-pid-lcd

Requires BME280 sensor connected by I2C

Requires Pico 0.96 LCD with buttons and joystick
https://www.waveshare.com/wiki/Pico-LCD-0.96

These pins are taken by the LCD hat
 GP  Pin  Function
  2    4  Joystick up
  3    5  Joystick centre press
  8   11  SPI DI1 data (high) command (low)
  9   12  SPI CS1 chip select (low active)
 10   14  SPI SCK1 clock input
 11   15  SPI DO1 data input
 12   16  Reset (low active)
 13   17  Backlight
 15   20  User key A
 16   21  Joystick left
 17   22  User key B
 18   24  Joystick down
 20   25  Joystick right
 VSYS 39  1.8 to 5.5V power supply
"""

from machine import Pin,SPI,PWM,I2C
import framebuf
import json
import sys
from time import sleep_ms, ticks_ms, ticks_add, ticks_diff
from picolcd import LCD_0inch96
from bme280_float import BME280
from pid import PID
from autotune import PIDAutotune

# 16-bit colour, 5 bit R, 6 bit G, 5 bit B, least significant byte first
RED = 0x00F8
GREEN = 0xE007
BLUE = 0x1F00
WHITE = 0xFFFF
BLACK = 0x0000
PURPLE = 0x1368
YELLOW = 0x66FE

# Pin mapping
KEY_UP = Pin(2,Pin.IN,Pin.PULL_UP)
KEY_DOWN = Pin(18,Pin.IN,Pin.PULL_UP)
KEY_LEFT= Pin(16,Pin.IN,Pin.PULL_UP)
KEY_RIGHT= Pin(20,Pin.IN,Pin.PULL_UP)
KEY_CTRL=Pin(3,Pin.IN,Pin.PULL_UP)
KEY_A=Pin(15,Pin.IN,Pin.PULL_UP)
KEY_B=Pin(17,Pin.IN,Pin.PULL_UP) 

def startup():
    # import settings
    with open("settings.json") as settingsfile:
        settings = json.load(settingsfile)

    # Set pin to drive MOSFET for heater
    heater = PWM(Pin(int(settings["heaterpin"])))
    heater.freq(int(settings["heaterfreq_Hz"]))

    lcd = LCD_0inch96()
    # check if sensor is connected
    try:
        bme = BME280(i2c=I2C(int(settings["i2cbus"]),
                             scl=Pin(int(settings["scl_pin"])),
                             sda=Pin(int(settings["sda_pin"]))))
    except:
        lcd.fill(BLACK)
        lcd.fill_rect(0,0,lcd.width,10,RED)
        lcd.text("BME280 not detected!",0,1,BLACK)
        lcd.text("Connections:",0,11,GREEN)
        lcd.text("SCL to GP{:d}".format(int(settings["scl_pin"]))+
                 " (pin {:d})".format(int(1+(5*settings["scl_pin"]
                                             +2)/4)),
                 0,21,YELLOW)
        lcd.text("SDA to GP{:d}".format(int(settings["sda_pin"]))+
                 " (pin {:d})".format(int(1+(5*settings["sda_pin"]
                                             +2)/4)),
                 0,31,BLUE)
        lcd.text("VIN to 3V3 (pin 36)",0,41,RED)
        lcd.text("GND to pin 3, 13, 18",0,51,WHITE)
        lcd.text("23, 28, 33 or 38",0,61,WHITE)
        lcd.display()
        sys.exit(1)

    return(settings, lcd, heater, bme)

def mainloop(settings, lcd, heater, bme):
    runtime = 0
    heaterduty = 0
    running = False
    autotune = False
    backlight = True
    keyflag = 0
    waitsave = 0
    if 'oldtemp' not in locals():
        oldtemp = 20.0
    action = settings["displaysleep_s"] * settings["displayupdate_Hz"]
    pid = PID(settings["sampletime_s"],
              settings["Kp"],settings["Ki"],settings["Kd"],
              settings["outmin"],settings["outmax"],lambda:runtime)
    while True:
        nextloop = ticks_add(ticks_ms(),
                             round(1000*settings["sampletime_s"]/
                                   settings["displayupdate_Hz"]))
        if (action % settings["displayupdate_Hz"]) == 0:
            lcd.fill(BLACK)
            # Make sure BME280 is connected
            try:
                newtemp, pressure, rh = bme.read_compensated_data()
                readerror = False
            except:
                readerror = True
            if readerror:
                lcd.fill_rect(0,0,lcd.width,10,RED)
                lcd.text("BME280 not detected!",0,1,BLACK)
                lcd.text("Connections:",0,11,GREEN)
                lcd.text("SCL to GP{:d}".format(int(settings["scl_pin"]))+
                         " (pin {:d})".format(int(1+(5*settings["scl_pin"]
                                                     +2)/4)),
                         0,21,YELLOW)
                lcd.text("SDA to GP{:d}".format(int(settings["sda_pin"]))+
                         " (pin {:d})".format(int(1+(5*settings["sda_pin"]
                                                     +2)/4)),
                         0,31,BLUE)
                lcd.text("VIN to 3V3 (pin 36)",0,41,RED)
            else:
                newtemp += settings["tempoffset_C"]
                pressure += settings["pressureoffset_hPa"]*100
                rh += settings["rhoffset_pct"]
                rectwidth, rectcolor = colorbar(newtemp,
                                                settings["setpoint_C"],
                                                int(lcd.width),
                                                center=settings["tempbar_frac"])
                lcd.fill_rect(0,0,rectwidth,10,rectcolor)
                lcd.vline(int(settings["tempbar_frac"]*lcd.width),0,10,WHITE)
                lcd.text("Temperature    {:.1f}C".format(newtemp),0,1,WHITE)
                lcd.text("Setpoint       {:.1f}C".format(settings["setpoint_C"]),0,11,WHITE)
                lcd.fill_rect(0,20,round(rh*1.6),10,BLUE)
                lcd.text("Rel. humidity  {:.1f}%".format(rh),0,21,WHITE)
                lcd.text("Pressure  {:.1f} hPa".format(pressure/100),0,31,WHITE)
                runtime += settings["sampletime_s"]
                temp = settings["smoothingalpha"]*newtemp + (1-settings["smoothingalpha"])*oldtemp
                oldtemp = temp
            
            # Count down before saving file if setpoint has changed
            if waitsave > 1:
                waitsave -= 1
            elif waitsave == 1:
                with open("settings.json","w") as settingsfile:
                    json.dump(settings, settingsfile)
                waitsave = 0
            if autotune:
                if not tune.run(temp):
                    heaterduty = tune.output
                else:
                    autotune = False
                    lcd.backlight(1000)
                    if tune.state == PIDAutotune.STATE_SUCCEEDED:
                        params = tune.get_pid_parameters(settings["tuningrule"])
                        settings["Kp"] = params.Kp
                        settings["Ki"] = params.Ki
                        settings["Kd"] = params.Kd
                        # Save new PID settings
                        with open("settings.json","w") as settingsfile:
                            json.dump(settings, settingsfile)
                        # Apply new PID settings
                        pid = PID(settings["sampletime_s"],
                                  settings["Kp"],settings["Ki"],
                                  settings["Kd"],settings["outmin"],
                                  settings["outmax"],lambda:runtime)
                        running = True
                    else:
                        print('Autotune failed!')
            else:
                if running and not readerror:
                    heaterduty = int(pid.calc(temp,settings["setpoint_C"]))
                else:
                    heaterduty = 0
            if not readerror:
                lcd.fill_rect(0,40,round(heaterduty/65535*160),10,RED)
                lcd.text("Heater        {:5.1f}%".format(heaterduty/655.35),0,41,WHITE)
                run_m, run_s = divmod(runtime,60)
                run_h, run_m = divmod(run_m,60)
            lcd.text("Runtime {:5d}h{:02d}m{:02}s".format(int(run_h),
                                                          int(run_m),
                                                          int(run_s)),
                     0,51,WHITE)
            lcd.fill_rect(0,60,160,40,0xef7b)
            if autotune:
                lcd.text("Autotune in progress",0,61,RED)
                lcd.text("Hold B to cancel",0,71,RED)
            else:
                if running:
                    lcd.text("A:Stop",0,61,RED)
                else:
                    lcd.text("A:Start",0,61,GREEN)
                lcd.text("B:autotune",80,61,PURPLE)
                lcd.text("Joystick:setpoint",0,71,YELLOW)
            lcd.display()
            heater.duty_u16(heaterduty)
        # Display sleep
        if action > 0:
            action -= 1
        else:
            lcd.backlight(5)
            action = settings["displaysleep_s"] * settings["displayupdate_Hz"] - 1
            backlight = False
        if (keyflag > 0):
            keyflag -= 1
        elif (KEY_UP.value()==0 or KEY_DOWN.value()==0
              or KEY_LEFT.value()==0 or KEY_RIGHT.value()==0
              or KEY_CTRL.value()==0 or KEY_A.value()==0
              or KEY_B.value()==0 ): # key down
            keyflag=settings["displayupdate_Hz"]
            if not backlight:
                lcd.backlight(1000)
                backlight = True
                action = settings["displaysleep_s"] * settings["displayupdate_Hz"] - 1
            elif KEY_A.value()==0 and not autotune:
                running = not running
            elif ((KEY_UP.value()==0 or KEY_RIGHT.value()==0) and
                  not autotune):
                settings["setpoint_C"] += 0.1
                shownewtemp(lcd,settings["setpoint_C"])
                keyflag = 0
                waitsave = 10 # 10s delay before saving new value
            elif ((KEY_DOWN.value()==0 or KEY_LEFT.value()==0) and
                  not autotune):
                settings["setpoint_C"] -= 0.1
                shownewtemp(lcd,settings["setpoint_C"])
                keyflag = 0
                waitsave = 10 # 10s delay before saving new value
            elif KEY_B.value()==0:
                if autotune:
                    autotune = False # Cancel autotune
                else:
                    autotune = True
                    tune = PIDAutotune(settings["setpoint_C"],
                                       out_step=int(settings["outmax"]),
                                       sampletime=settings["sampletime_s"],
                                       lookback=settings["lookback_points"],
                                       out_min=int(settings["outmin"]),
                                       out_max=int(settings["outmax"]),
                                       time=lambda:runtime)
        while ticks_diff(nextloop,ticks_ms()) > 0:
            sleep_ms(5)

# Startup splash screen
def splash(lcd,bkgdcol,textcol):
    lcd.fill(bkgdcol)   
    lcd.text("PID controller",24,15,textcol)
    lcd.text("Chris Blanford",24,35,textcol)    
    lcd.text("2022-02-25",40,55,textcol)
    lcd.display()
    sleep_ms(3000)
    lcd.fill(BLACK)
    lcd.display()

# set red-green temperature colourbar depending on temperature deviation
def colorbar(measured,setpoint,lcdwidth,scale=8.0,center=0.6):
    diff = measured - setpoint
    # full red when difference >= 1 C
    # full green when difference <= 1 C
    red = min(1,max(0,abs(diff)))
    green = min(1,max(0,2-abs(diff)))
    rectwidth = int(min(lcdwidth,max(0,round(diff*scale+center*lcdwidth))))
    # scale to 10 bits little endian
    tempcolor = ((int(min(31,max(0,red*31))) << 11) +
                  (int(min(63,max(0,green*63))) << 5))
    rectcolor = (((tempcolor & 0xff00) >> 8) +
                 ((tempcolor & 0x00ff) << 8))
    return(rectwidth,rectcolor)

# replace existing temp with new temp on LCD display
def shownewtemp(lcd,temp,x=120,y=11):
    lcd.fill_rect(int(x),int(y-1),int(lcd.width-x),10,0x0000)
    lcd.text("{:.1f}C".format(temp),int(x),int(y),0xFFFF)
    lcd.display()
    return

if __name__=='__main__':
    try:
        settings, lcd, heater, bme = startup()
        splash(lcd,PURPLE,YELLOW)
        mainloop(settings, lcd, heater, bme)
    except KeyboardInterrupt:
        heater.duty_u16(0) # Turn off heater on Ctrl-C
        lcd.fill(BLACK)
        lcd.display()