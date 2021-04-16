# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Jeff Epler for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

"""Capture an image from the camera and display it as ASCII art.

The camera is placed in YUV mode, so the top 8 bits of each color
value can be treated as "greyscale"."""

import time

import digitalio
import busio
import board

from adafruit_ov7670 import OV7670, OV7670_SIZE_DIV16, OV7670_COLOR_YUV

# Ensure the camera is shut down, so that it releases the SDA/SCL lines,
# then create the configuration I2C bus

with digitalio.DigitalInOut(board.D39) as shutdown:
    shutdown.switch_to_output(True)
    time.sleep(0.001)
    bus = busio.I2C(board.D24, board.D25)

cam = OV7670(
    bus,
    data0=board.PCC_D0,
    clock=board.PCC_CLK,
    vsync=board.PCC_DEN1,
    href=board.PCC_DEN2,
    mclk=board.D29,
    shutdown=board.D39,
    reset=board.D38,
)
cam.size = OV7670_SIZE_DIV16
cam.colorspace = OV7670_COLOR_YUV

buf = bytearray(2 * cam.width * cam.height)
chars = " .:-=+*#%@"

cam.capture(buf)
width = cam.width
for j in range(cam.height):
    for i in range(cam.width):
        b = buf[2 * (width * j + i)] * (len(chars) - 1) // 255
        print(end=chars[b] * 2)
    print()
