# SPDX-FileCopyrightText: 2024 esp32beans@gmail.com
# SPDX-License-Identifier: MIT
""" Convert Wii Nunchuk joystick and 2 buttons to analog voltages for use
    with Playstation Sony Access Controller expansion ports. """

import time
import board
import busio
import adafruit_mcp4728
import adafruit_nunchuk

MCP4728_DEFAULT_ADDRESS = 0x60
MCP4728A4_DEFAULT_ADDRESS = 0x64

# i2c = board.I2C()  # uses board.SCL and board.SDA
#i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
# SCL1/SDA1 are the pins on STEMMA/QWIIC connector on QT Py RP2040
i2c = busio.I2C(board.SCL1, board.SDA1, frequency=400000)
# SCL/SDA are the pins on STEMMA/QWIIC connector on Feather RP2040
# i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# Nunchuck
nunc = adafruit_nunchuk.Nunchuk(i2c)

#  use for MCP4728 variant
mcp4728 = adafruit_mcp4728.MCP4728(i2c, adafruit_mcp4728.MCP4728_DEFAULT_ADDRESS)
#  use for MCP4728A4 variant
#  mcp4728 = adafruit_mcp4728.MCP4728(i2c, adafruit_mcp4728.MCP4728A4_DEFAULT_ADDRESS)

# 1 joystick and 2 buttons
Axes = [mcp4728.channel_a, mcp4728.channel_b]
Buttons = [mcp4728.channel_c, mcp4728.channel_d]

# https://www.playstation.com/content/dam/global_pdc/en/corporate/support/manuals/accessories/ps5-accessories/access-controller/access-docs/Access%20Controller%20for%20PlayStation%205%20Expansion%20Port%20Specifications.pdf
# if the link is broken, search for the doc title
# "Access Controller for PlayStation 5 Expansion Port Specifications"
FULL_VREF_RAW_VALUE = 4095
DAC_MAX_V = 1.80    # Vdd
DAC_ABS_MAX = int((DAC_MAX_V / 2.048) * FULL_VREF_RAW_VALUE)
DAC_MID = int(DAC_ABS_MAX / 2)
DAC_MIN = int((0.6 / 2.048) * FULL_VREF_RAW_VALUE)
DAC_MAX = int((1.2 / 2.048) * FULL_VREF_RAW_VALUE)
DAC_BUTTON = DAC_MID

# Set default neutral values. Stick centered and buttons released.
for axis in Axes:
    # DAC output: 0V to 2.048V with 4096 steps
    # pylint: disable=no-member
    axis.vref = ( adafruit_mcp4728.Vref.INTERNAL )
    axis.gain = 1
    axis.raw_value = DAC_MID

for button in Buttons:
    # DAC output: 0V to 2.048V with 4096 steps
    # pylint: disable=no-member
    button.vref = ( adafruit_mcp4728.Vref.INTERNAL )
    button.gain = 1
    button.raw_value = 0

# Save neutral values to EEPROM
mcp4728.save_settings()

def amap(val, val_min, val_max, out_min, out_max):
    """ Similar to Arduino map function """
    return int((((out_max - out_min) / (val_max - val_min)) * (val - val_min)) + out_min)

def button_press(button_index, value):
    """ Press and release button """
    if button_index not in range(len(Buttons)):
        print('Invalid button:', button_index)
        return

    if value == 0:
        Buttons[button_index].raw_value = 0
    else:
        Buttons[button_index].raw_value = DAC_BUTTON

def stick_move(my_x, my_y):
    """ Update stick DAC values """
    Axes[0].raw_value = amap(my_x, 0, 255, DAC_MIN, DAC_MAX)
    Axes[1].raw_value = amap(my_y, 0, 255, DAC_MIN, DAC_MAX)

Last_C = -1
Last_Z = -1
Last_x = -1
Last_y = -1

while True:
    x, y = nunc.joystick
    if Last_x != x or Last_y != y:
        Last_x = x
        Last_y = y
        stick_move(x, y)
    C, Z = nunc.buttons
    if C != Last_C:
        Last_C = C
        if C:
            button_press(0, 1)
        else:
            button_press(0, 0)
    if Z != Last_Z:
        Last_Z = Z
    if Z:
        button_press(1, 1)
    else:
        button_press(1, 0)
    time.sleep(0.008)
