# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Jeff Epler for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ov7670`
================================================================================

CircuitPython driver for OV7670 cameras


* Author(s): Jeff Epler

Implementation Notes
--------------------

**Hardware:**

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* The CircuitPython build for your board must support the ``imagecapture`` module.
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

# imports

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_OV7670.git"

import time

import digitalio
import imagecapture
import pwmio
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

try:
    from typing import List, Optional, Union
except ImportError:
    pass

# Supported color formats
OV7670_COLOR_RGB = 0
"""RGB565 big-endian"""
OV7670_COLOR_YUV = 1
"""YUV/YCbCr 422 big-endian"""

# Supported sizes (VGA division factor) for OV7670_set_size()
OV7670_SIZE_DIV1 = 0
"""640 x 480"""
OV7670_SIZE_DIV2 = 1
"""320 x 240"""
OV7670_SIZE_DIV4 = 2
"""160 x 120"""
OV7670_SIZE_DIV8 = 3
"""80 x 60"""
OV7670_SIZE_DIV16 = 4
"""40 x 30"""

# Test patterns
OV7670_TEST_PATTERN_NONE = 0
"""Normal operation mode (no test pattern)"""
OV7670_TEST_PATTERN_SHIFTING_1 = 1
""""Shifting 1" pattern"""
OV7670_TEST_PATTERN_COLOR_BAR = 2
"""8 color bars"""
OV7670_TEST_PATTERN_COLOR_BAR_FADE = 3
"""Color bars w/fade to white"""

# Table of bit patterns for the different supported night modes.
# There's a "same frame rate" option for OV7670 night mode but it
# doesn't seem to do anything useful and can be skipped over.
OV7670_NIGHT_MODE_OFF = 0
"""Disable night mode"""
OV7670_NIGHT_MODE_2 = 0b10100000
"""Night mode 1/2 frame rate"""
OV7670_NIGHT_MODE_4 = 0b11000000
"""Night mode 1/4 frame rate"""
OV7670_NIGHT_MODE_8 = 0b11100000
"""Night mode 1/8 frame rate"""

OV7670_ADDR = 0x21
"""Default I2C address if unspecified"""

_OV7670_REG_GAIN = const(0x00)  # AGC gain bits 7:0 (9:8 in VREF)
_OV7670_REG_BLUE = const(0x01)  # AWB blue channel gain
_OV7670_REG_RED = const(0x02)  # AWB red channel gain
_OV7670_REG_VREF = const(0x03)  # Vert frame control bits
_OV7670_REG_COM1 = const(0x04)  # Common control 1
_OV7670_COM1_R656 = const(0x40)  # COM1 enable R656 format
_OV7670_REG_BAVE = const(0x05)  # U/B average level
_OV7670_REG_GbAVE = const(0x06)  # Y/Gb average level
_OV7670_REG_AECHH = const(0x07)  # Exposure value - AEC 15:10 bits
_OV7670_REG_RAVE = const(0x08)  # V/R average level
_OV7670_REG_COM2 = const(0x09)  # Common control 2
_OV7670_COM2_SSLEEP = const(0x10)  # COM2 soft sleep mode
_OV7670_REG_PID = const(0x0A)  # Product ID MSB (read-only)
_OV7670_REG_VER = const(0x0B)  # Product ID LSB (read-only)
_OV7670_REG_COM3 = const(0x0C)  # Common control 3
_OV7670_COM3_SWAP = const(0x40)  # COM3 output data MSB/LSB swap
_OV7670_COM3_SCALEEN = const(0x08)  # COM3 scale enable
_OV7670_COM3_DCWEN = const(0x04)  # COM3 DCW enable
_OV7670_REG_COM4 = const(0x0D)  # Common control 4
_OV7670_REG_COM5 = const(0x0E)  # Common control 5
_OV7670_REG_COM6 = const(0x0F)  # Common control 6
_OV7670_REG_AECH = const(0x10)  # Exposure value 9:2
_OV7670_REG_CLKRC = const(0x11)  # Internal clock
_OV7670_CLK_EXT = const(0x40)  # CLKRC Use ext clock directly
_OV7670_CLK_SCALE = const(0x3F)  # CLKRC Int clock prescale mask
_OV7670_REG_COM7 = const(0x12)  # Common control 7
_OV7670_COM7_RESET = const(0x80)  # COM7 SCCB register reset
_OV7670_COM7_SIZE_MASK = const(0x38)  # COM7 output size mask
_OV7670_COM7_PIXEL_MASK = const(0x05)  # COM7 output pixel format mask
_OV7670_COM7_SIZE_VGA = const(0x00)  # COM7 output size VGA
_OV7670_COM7_SIZE_CIF = const(0x20)  # COM7 output size CIF
_OV7670_COM7_SIZE_QVGA = const(0x10)  # COM7 output size QVGA
_OV7670_COM7_SIZE_QCIF = const(0x08)  # COM7 output size QCIF
_OV7670_COM7_RGB = const(0x04)  # COM7 pixel format RGB
_OV7670_COM7_YUV = const(0x00)  # COM7 pixel format YUV
_OV7670_COM7_BAYER = const(0x01)  # COM7 pixel format Bayer RAW
_OV7670_COM7_PBAYER = const(0x05)  # COM7 pixel fmt proc Bayer RAW
_OV7670_COM7_COLORBAR = const(0x02)  # COM7 color bar enable
_OV7670_REG_COM8 = const(0x13)  # Common control 8
_OV7670_COM8_FASTAEC = const(0x80)  # COM8 Enable fast AGC/AEC algo,
_OV7670_COM8_AECSTEP = const(0x40)  # COM8 AEC step size unlimited
_OV7670_COM8_BANDING = const(0x20)  # COM8 Banding filter enable
_OV7670_COM8_AGC = const(0x04)  # COM8 AGC (auto gain) enable
_OV7670_COM8_AWB = const(0x02)  # COM8 AWB (auto white balance)
_OV7670_COM8_AEC = const(0x01)  # COM8 AEC (auto exposure) enable
_OV7670_REG_COM9 = const(0x14)  # Common control 9 - max AGC value
_OV7670_REG_COM10 = const(0x15)  # Common control 10
_OV7670_COM10_HSYNC = const(0x40)  # COM10 HREF changes to HSYNC
_OV7670_COM10_PCLK_HB = const(0x20)  # COM10 Suppress PCLK on hblank
_OV7670_COM10_HREF_REV = const(0x08)  # COM10 HREF reverse
_OV7670_COM10_VS_EDGE = const(0x04)  # COM10 VSYNC chg on PCLK rising
_OV7670_COM10_VS_NEG = const(0x02)  # COM10 VSYNC negative
_OV7670_COM10_HS_NEG = const(0x01)  # COM10 HSYNC negative
_OV7670_REG_HSTART = const(0x17)  # Horiz frame start high bits
_OV7670_REG_HSTOP = const(0x18)  # Horiz frame end high bits
_OV7670_REG_VSTART = const(0x19)  # Vert frame start high bits
_OV7670_REG_VSTOP = const(0x1A)  # Vert frame end high bits
_OV7670_REG_PSHFT = const(0x1B)  # Pixel delay select
_OV7670_REG_MIDH = const(0x1C)  # Manufacturer ID high byte
_OV7670_REG_MIDL = const(0x1D)  # Manufacturer ID low byte
_OV7670_REG_MVFP = const(0x1E)  # Mirror / vert-flip enable
_OV7670_MVFP_MIRROR = const(0x20)  # MVFP Mirror image
_OV7670_MVFP_VFLIP = const(0x10)  # MVFP Vertical flip
_OV7670_REG_LAEC = const(0x1F)  # Reserved
_OV7670_REG_ADCCTR0 = const(0x20)  # ADC control
_OV7670_REG_ADCCTR1 = const(0x21)  # Reserved
_OV7670_REG_ADCCTR2 = const(0x22)  # Reserved
_OV7670_REG_ADCCTR3 = const(0x23)  # Reserved
_OV7670_REG_AEW = const(0x24)  # AGC/AEC upper limit
_OV7670_REG_AEB = const(0x25)  # AGC/AEC lower limit
_OV7670_REG_VPT = const(0x26)  # AGC/AEC fast mode op region
_OV7670_REG_BBIAS = const(0x27)  # B channel signal output bias
_OV7670_REG_GbBIAS = const(0x28)  # Gb channel signal output bias
_OV7670_REG_EXHCH = const(0x2A)  # Dummy pixel insert MSB
_OV7670_REG_EXHCL = const(0x2B)  # Dummy pixel insert LSB
_OV7670_REG_RBIAS = const(0x2C)  # R channel signal output bias
_OV7670_REG_ADVFL = const(0x2D)  # Insert dummy lines MSB
_OV7670_REG_ADVFH = const(0x2E)  # Insert dummy lines LSB
_OV7670_REG_YAVE = const(0x2F)  # Y/G channel average value
_OV7670_REG_HSYST = const(0x30)  # HSYNC rising edge delay
_OV7670_REG_HSYEN = const(0x31)  # HSYNC falling edge delay
_OV7670_REG_HREF = const(0x32)  # HREF control
_OV7670_REG_CHLF = const(0x33)  # Array current control
_OV7670_REG_ARBLM = const(0x34)  # Array ref control - reserved
_OV7670_REG_ADC = const(0x37)  # ADC control - reserved
_OV7670_REG_ACOM = const(0x38)  # ADC & analog common - reserved
_OV7670_REG_OFON = const(0x39)  # ADC offset control - reserved
_OV7670_REG_TSLB = const(0x3A)  # Line buffer test option
_OV7670_TSLB_NEG = const(0x20)  # TSLB Negative image enable
_OV7670_TSLB_YLAST = const(0x04)  # TSLB UYVY or VYUY, see COM13
_OV7670_TSLB_AOW = const(0x01)  # TSLB Auto output window
_OV7670_REG_COM11 = const(0x3B)  # Common control 11
_OV7670_COM11_NIGHT = const(0x80)  # COM11 Night mode
_OV7670_COM11_NMFR = const(0x60)  # COM11 Night mode frame rate mask
_OV7670_COM11_HZAUTO = const(0x10)  # COM11 Auto detect 50/60 Hz
_OV7670_COM11_BAND = const(0x08)  # COM11 Banding filter val select
_OV7670_COM11_EXP = const(0x02)  # COM11 Exposure timing control
_OV7670_REG_COM12 = const(0x3C)  # Common control 12
_OV7670_COM12_HREF = const(0x80)  # COM12 Always has HREF
_OV7670_REG_COM13 = const(0x3D)  # Common control 13
_OV7670_COM13_GAMMA = const(0x80)  # COM13 Gamma enable
_OV7670_COM13_UVSAT = const(0x40)  # COM13 UV saturation auto adj
_OV7670_COM13_UVSWAP = const(0x01)  # COM13 UV swap, use w TSLB[3]
_OV7670_REG_COM14 = const(0x3E)  # Common control 14
_OV7670_COM14_DCWEN = const(0x10)  # COM14 DCW & scaling PCLK enable
_OV7670_REG_EDGE = const(0x3F)  # Edge enhancement adjustment
_OV7670_REG_COM15 = const(0x40)  # Common control 15
_OV7670_COM15_RMASK = const(0xC0)  # COM15 Output range mask
_OV7670_COM15_R10F0 = const(0x00)  # COM15 Output range 10 to F0
_OV7670_COM15_R01FE = const(0x80)  # COM15              01 to FE
_OV7670_COM15_R00FF = const(0xC0)  # COM15              00 to FF
_OV7670_COM15_RGBMASK = const(0x30)  # COM15 RGB 555/565 option mask
_OV7670_COM15_RGB = const(0x00)  # COM15 Normal RGB out
_OV7670_COM15_RGB565 = const(0x10)  # COM15 RGB 565 output
_OV7670_COM15_RGB555 = const(0x30)  # COM15 RGB 555 output
_OV7670_REG_COM16 = const(0x41)  # Common control 16
_OV7670_COM16_AWBGAIN = const(0x08)  # COM16 AWB gain enable
_OV7670_REG_COM17 = const(0x42)  # Common control 17
_OV7670_COM17_AECWIN = const(0xC0)  # COM17 AEC window must match COM4
_OV7670_COM17_CBAR = const(0x08)  # COM17 DSP Color bar enable
_OV7670_REG_AWBC1 = const(0x43)  # Reserved
_OV7670_REG_AWBC2 = const(0x44)  # Reserved
_OV7670_REG_AWBC3 = const(0x45)  # Reserved
_OV7670_REG_AWBC4 = const(0x46)  # Reserved
_OV7670_REG_AWBC5 = const(0x47)  # Reserved
_OV7670_REG_AWBC6 = const(0x48)  # Reserved
_OV7670_REG_REG4B = const(0x4B)  # UV average enable
_OV7670_REG_DNSTH = const(0x4C)  # De-noise strength
_OV7670_REG_MTX1 = const(0x4F)  # Matrix coefficient 1
_OV7670_REG_MTX2 = const(0x50)  # Matrix coefficient 2
_OV7670_REG_MTX3 = const(0x51)  # Matrix coefficient 3
_OV7670_REG_MTX4 = const(0x52)  # Matrix coefficient 4
_OV7670_REG_MTX5 = const(0x53)  # Matrix coefficient 5
_OV7670_REG_MTX6 = const(0x54)  # Matrix coefficient 6
_OV7670_REG_BRIGHT = const(0x55)  # Brightness control
_OV7670_REG_CONTRAS = const(0x56)  # Contrast control
_OV7670_REG_CONTRAS_CENTER = const(0x57)  # Contrast center
_OV7670_REG_MTXS = const(0x58)  # Matrix coefficient sign
_OV7670_REG_LCC1 = const(0x62)  # Lens correction option 1
_OV7670_REG_LCC2 = const(0x63)  # Lens correction option 2
_OV7670_REG_LCC3 = const(0x64)  # Lens correction option 3
_OV7670_REG_LCC4 = const(0x65)  # Lens correction option 4
_OV7670_REG_LCC5 = const(0x66)  # Lens correction option 5
_OV7670_REG_MANU = const(0x67)  # Manual U value
_OV7670_REG_MANV = const(0x68)  # Manual V value
_OV7670_REG_GFIX = const(0x69)  # Fix gain control
_OV7670_REG_GGAIN = const(0x6A)  # G channel AWB gain
_OV7670_REG_DBLV = const(0x6B)  # PLL & regulator control
_OV7670_REG_AWBCTR3 = const(0x6C)  # AWB control 3
_OV7670_REG_AWBCTR2 = const(0x6D)  # AWB control 2
_OV7670_REG_AWBCTR1 = const(0x6E)  # AWB control 1
_OV7670_REG_AWBCTR0 = const(0x6F)  # AWB control 0
_OV7670_REG_SCALING_XSC = const(0x70)  # Test pattern X scaling
_OV7670_REG_SCALING_YSC = const(0x71)  # Test pattern Y scaling
_OV7670_REG_SCALING_DCWCTR = const(0x72)  # DCW control
_OV7670_REG_SCALING_PCLK_DIV = const(0x73)  # DSP scale control clock divide
_OV7670_REG_REG74 = const(0x74)  # Digital gain control
_OV7670_REG_REG76 = const(0x76)  # Pixel correction
_OV7670_REG_SLOP = const(0x7A)  # Gamma curve highest seg slope
_OV7670_REG_GAM_BASE = const(0x7B)  # Gamma register base (1 of 15)
_OV7670_GAM_LEN = const(15)  # Number of gamma registers
_OV7670_R76_BLKPCOR = const(0x80)  # REG76 black pixel corr enable
_OV7670_R76_WHTPCOR = const(0x40)  # REG76 white pixel corr enable
_OV7670_REG_RGB444 = const(0x8C)  # RGB 444 control
_OV7670_R444_ENABLE = const(0x02)  # RGB444 enable
_OV7670_R444_RGBX = const(0x01)  # RGB444 word format
_OV7670_REG_DM_LNL = const(0x92)  # Dummy line LSB
_OV7670_REG_LCC6 = const(0x94)  # Lens correction option 6
_OV7670_REG_LCC7 = const(0x95)  # Lens correction option 7
_OV7670_REG_HAECC1 = const(0x9F)  # Histogram-based AEC/AGC ctrl 1
_OV7670_REG_HAECC2 = const(0xA0)  # Histogram-based AEC/AGC ctrl 2
_OV7670_REG_SCALING_PCLK_DELAY = const(0xA2)  # Scaling pixel clock delay
_OV7670_REG_BD50MAX = const(0xA5)  # 50 Hz banding step limit
_OV7670_REG_HAECC3 = const(0xA6)  # Histogram-based AEC/AGC ctrl 3
_OV7670_REG_HAECC4 = const(0xA7)  # Histogram-based AEC/AGC ctrl 4
_OV7670_REG_HAECC5 = const(0xA8)  # Histogram-based AEC/AGC ctrl 5
_OV7670_REG_HAECC6 = const(0xA9)  # Histogram-based AEC/AGC ctrl 6
_OV7670_REG_HAECC7 = const(0xAA)  # Histogram-based AEC/AGC ctrl 7
_OV7670_REG_BD60MAX = const(0xAB)  # 60 Hz banding step limit
_OV7670_REG_ABLC1 = const(0xB1)  # ABLC enable
_OV7670_REG_THL_ST = const(0xB3)  # ABLC target
_OV7670_REG_SATCTR = const(0xC9)  # Saturation control

_OV7670_REG_LAST = const(_OV7670_REG_SATCTR)  # Maximum register address

# Manual output format, RGB, use RGB565 and full 0-255 output range
_OV7670_rgb = bytes(
    [
        _OV7670_REG_COM7,
        _OV7670_COM7_RGB,
        _OV7670_REG_RGB444,
        0,
        _OV7670_REG_COM15,
        _OV7670_COM15_RGB565 | _OV7670_COM15_R00FF,
    ]
)

# Manual output format, YUV, use full output range
_OV7670_yuv = bytes(
    [
        _OV7670_REG_COM7,
        _OV7670_COM7_YUV,
        _OV7670_REG_COM15,
        _OV7670_COM15_R00FF,
    ]
)

_OV7670_init = bytes(
    [
        _OV7670_REG_TSLB,
        _OV7670_TSLB_YLAST,  # No auto window
        _OV7670_REG_COM10,
        _OV7670_COM10_VS_NEG,  # -VSYNC (req by SAMD PCC)
        _OV7670_REG_SLOP,
        0x20,
        _OV7670_REG_GAM_BASE,
        0x1C,
        _OV7670_REG_GAM_BASE + 1,
        0x28,
        _OV7670_REG_GAM_BASE + 2,
        0x3C,
        _OV7670_REG_GAM_BASE + 3,
        0x55,
        _OV7670_REG_GAM_BASE + 4,
        0x68,
        _OV7670_REG_GAM_BASE + 5,
        0x76,
        _OV7670_REG_GAM_BASE + 6,
        0x80,
        _OV7670_REG_GAM_BASE + 7,
        0x88,
        _OV7670_REG_GAM_BASE + 8,
        0x8F,
        _OV7670_REG_GAM_BASE + 9,
        0x96,
        _OV7670_REG_GAM_BASE + 10,
        0xA3,
        _OV7670_REG_GAM_BASE + 11,
        0xAF,
        _OV7670_REG_GAM_BASE + 12,
        0xC4,
        _OV7670_REG_GAM_BASE + 13,
        0xD7,
        _OV7670_REG_GAM_BASE + 14,
        0xE8,
        _OV7670_REG_COM8,
        _OV7670_COM8_FASTAEC | _OV7670_COM8_AECSTEP | _OV7670_COM8_BANDING,
        _OV7670_REG_GAIN,
        0x00,
        _OV7670_COM2_SSLEEP,
        0x00,
        _OV7670_REG_COM4,
        0x00,
        _OV7670_REG_COM9,
        0x20,  # Max AGC value
        _OV7670_REG_BD50MAX,
        0x05,
        _OV7670_REG_BD60MAX,
        0x07,
        _OV7670_REG_AEW,
        0x75,
        _OV7670_REG_AEB,
        0x63,
        _OV7670_REG_VPT,
        0xA5,
        _OV7670_REG_HAECC1,
        0x78,
        _OV7670_REG_HAECC2,
        0x68,
        0xA1,
        0x03,  # Reserved register?
        _OV7670_REG_HAECC3,
        0xDF,  # Histogram-based AEC/AGC setup
        _OV7670_REG_HAECC4,
        0xDF,
        _OV7670_REG_HAECC5,
        0xF0,
        _OV7670_REG_HAECC6,
        0x90,
        _OV7670_REG_HAECC7,
        0x94,
        _OV7670_REG_COM8,
        _OV7670_COM8_FASTAEC
        | _OV7670_COM8_AECSTEP
        | _OV7670_COM8_BANDING
        | _OV7670_COM8_AGC
        | _OV7670_COM8_AEC,
        _OV7670_REG_COM5,
        0x61,
        _OV7670_REG_COM6,
        0x4B,
        0x16,
        0x02,  # Reserved register?
        _OV7670_REG_MVFP,
        0x07,  # 0x07,
        _OV7670_REG_ADCCTR1,
        0x02,
        _OV7670_REG_ADCCTR2,
        0x91,
        0x29,
        0x07,  # Reserved register?
        _OV7670_REG_CHLF,
        0x0B,
        0x35,
        0x0B,  # Reserved register?
        _OV7670_REG_ADC,
        0x1D,
        _OV7670_REG_ACOM,
        0x71,
        _OV7670_REG_OFON,
        0x2A,
        _OV7670_REG_COM12,
        0x78,
        0x4D,
        0x40,  # Reserved register?
        0x4E,
        0x20,  # Reserved register?
        _OV7670_REG_GFIX,
        0x5D,
        _OV7670_REG_REG74,
        0x19,
        0x8D,
        0x4F,  # Reserved register?
        0x8E,
        0x00,  # Reserved register?
        0x8F,
        0x00,  # Reserved register?
        0x90,
        0x00,  # Reserved register?
        0x91,
        0x00,  # Reserved register?
        _OV7670_REG_DM_LNL,
        0x00,
        0x96,
        0x00,  # Reserved register?
        0x9A,
        0x80,  # Reserved register?
        0xB0,
        0x84,  # Reserved register?
        _OV7670_REG_ABLC1,
        0x0C,
        0xB2,
        0x0E,  # Reserved register?
        _OV7670_REG_THL_ST,
        0x82,
        0xB8,
        0x0A,  # Reserved register?
        _OV7670_REG_AWBC1,
        0x14,
        _OV7670_REG_AWBC2,
        0xF0,
        _OV7670_REG_AWBC3,
        0x34,
        _OV7670_REG_AWBC4,
        0x58,
        _OV7670_REG_AWBC5,
        0x28,
        _OV7670_REG_AWBC6,
        0x3A,
        0x59,
        0x88,  # Reserved register?
        0x5A,
        0x88,  # Reserved register?
        0x5B,
        0x44,  # Reserved register?
        0x5C,
        0x67,  # Reserved register?
        0x5D,
        0x49,  # Reserved register?
        0x5E,
        0x0E,  # Reserved register?
        _OV7670_REG_LCC3,
        0x04,
        _OV7670_REG_LCC4,
        0x20,
        _OV7670_REG_LCC5,
        0x05,
        _OV7670_REG_LCC6,
        0x04,
        _OV7670_REG_LCC7,
        0x08,
        _OV7670_REG_AWBCTR3,
        0x0A,
        _OV7670_REG_AWBCTR2,
        0x55,
        _OV7670_REG_MTX1,
        0x80,
        _OV7670_REG_MTX2,
        0x80,
        _OV7670_REG_MTX3,
        0x00,
        _OV7670_REG_MTX4,
        0x22,
        _OV7670_REG_MTX5,
        0x5E,
        _OV7670_REG_MTX6,
        0x80,  # 0x40?
        _OV7670_REG_AWBCTR1,
        0x11,
        _OV7670_REG_AWBCTR0,
        0x9F,  # Or use 0x9E for advance AWB
        _OV7670_REG_BRIGHT,
        0x00,
        _OV7670_REG_CONTRAS,
        0x40,
        _OV7670_REG_CONTRAS_CENTER,
        0x80,  # 0x40?
    ]
)

_window = [
    [9, 162, 2, 2],  # SIZE_DIV1  640x480 VGA
    [10, 174, 0, 2],  # SIZE_DIV2  320x240 QVGA
    [11, 186, 2, 2],  # SIZE_DIV4  160x120 QQVGA
    [12, 210, 0, 2],  # SIZE_DIV8  80x60   ...
    [15, 252, 3, 2],  # SIZE_DIV16 40x30
]


class OV7670:  # pylint: disable=too-many-instance-attributes
    """Library for the OV7670 digital camera"""

    def __init__(
        self,
        i2c_bus: busio.I2C,
        data_pins: List[microcontroller.Pin],
        clock: microcontroller.Pin,
        vsync: microcontroller.Pin,
        href: microcontroller.Pin,
        shutdown: Optional[microcontroller.Pin] = None,
        reset: Optional[microcontroller.Pin] = None,
        mclk: Optional[microcontroller.Pin] = None,
        mclk_frequency: int = 16_000_000,
        i2c_address: int = 0x21,
    ) -> None:  # pylint: disable=too-many-arguments
        """
        Args:
            i2c_bus (busio.I2C): The I2C bus used to configure the OV7670
            data_pins (List[microcontroller.Pin]): A list of 8 data pins, in order.
            clock (microcontroller.Pin): The pixel clock from the OV7670.
            vsync (microcontroller.Pin): The vsync signal from the OV7670.
            href (microcontroller.Pin): The href signal from the OV7670, \
                sometimes inaccurately called hsync.
            shutdown (Optional[microcontroller.Pin]): If not None, the shutdown
                signal to the camera, also called the powerdown or enable pin.
            reset (Optional[microcontroller.Pin]): If not None, the reset signal
                to the camera.
            mclk (Optional[microcontroller.Pin]): The pin on which to create a
                master clock signal, or None if the master clock signal is
                already being generated.
            mclk_frequency (int): The frequency of the master clock to generate, \
                ignored if mclk is None, requred if it is specified
            i2c_address (int): The I2C address of the camera.
        """
        # Initialize the master clock
        if mclk:
            self._mclk_pwm = pwmio.PWMOut(mclk, frequency=mclk_frequency)
            self._mclk_pwm.duty_cycle = 32768
        else:
            self._mclk_pwm = None

        if shutdown:
            self._shutdown = digitalio.DigitalInOut(shutdown)
            self._shutdown.switch_to_output(True)
            time.sleep(0.001)
            self._shutdown.switch_to_output(False)
            time.sleep(0.3)
        else:
            self._shutdown = None

        if reset:
            self._reset = digitalio.DigitalInOut(reset)
            self._reset.switch_to_output(False)
            time.sleep(0.001)
            self._reset.switch_to_output(True)

        self._i2c_device = I2CDevice(i2c_bus, i2c_address)

        if not reset:
            self._write_register(_OV7670_REG_COM7, _OV7670_COM7_RESET)

        time.sleep(0.001)

        self._colorspace = None
        self.colorspace = OV7670_COLOR_RGB

        self._write_list(_OV7670_init)

        self._size = None
        self.size = OV7670_SIZE_DIV8

        self._test_pattern = None
        self.test_pattern = OV7670_TEST_PATTERN_NONE

        self._flip_x = False
        self._flip_y = False

        self._night = OV7670_NIGHT_MODE_OFF

        self._imagecapture = imagecapture.ParallelImageCapture(
            data_pins=data_pins, clock=clock, vsync=vsync, href=href
        )

    def capture(self, buf: Union[bytearray, memoryview]) -> None:
        """Capture an image into the buffer.

        Args:
            buf (Union[bytearray, memoryview]): A WritableBuffer to contain the \
                captured image.  Note that this can be a ulab array or a displayio Bitmap.
        """
        self._imagecapture.capture(buf)

    @property
    def mclk_frequency(self) -> Optional[int]:
        """Get the actual frequency the generated mclk, or None"""
        return self._mclk_pwm.frequency if self._mclk_pwm else None

    @property
    def width(self):
        """Get the image width in pixels.  A buffer of 2*width*height bytes \
        stores a whole image."""
        return 640 >> self._size

    @property
    def height(self):
        """Get the image height in pixels.  A buffer of 2*width*height bytes \
        stores a whole image."""
        return 480 >> self._size

    @property
    def colorspace(self) -> Optional[int]:
        """Get or set the colorspace, one of the ``OV7670_COLOR_`` constants."""
        return self._colorspace

    @colorspace.setter
    def colorspace(self, colorspace: int) -> None:
        self._colorspace = colorspace
        self._write_list(_OV7670_rgb if colorspace == OV7670_COLOR_RGB else _OV7670_yuv)

    def deinit(self) -> None:
        """Deinitialize the camera"""
        self._imagecapture.deinit()
        if self._mclk_pwm:
            self._mclk_pwm.deinit()
        if self._shutdown:
            self._shutdown.deinit()
        if self._reset:
            self._reset.deinit()

    @property
    def size(self) -> Optional[int]:
        """Get or set the captured image size, one of the ``OV7670_SIZE_`` constants."""
        return self._size

    @size.setter
    def size(self, size):
        self._frame_control(size, *_window[size])
        self._size = size

    @property
    def test_pattern(self):
        """Get or set the test pattern, one of the ``OV7670_TEST_PATTERN_`` constants."""
        return self._test_pattern

    @test_pattern.setter
    def test_pattern(self, pattern: int) -> None:
        # Modify only test pattern bits (not scaling bits)
        xsc = self._read_register(_OV7670_REG_SCALING_XSC) & ~0x80
        ysc = self._read_register(_OV7670_REG_SCALING_YSC) & ~0x80
        if pattern & 1:
            xsc |= 0x80
        if pattern & 2:
            ysc |= 0x80
        # Write modified result back to SCALING_XSC and SCALING_YSC
        self._write_register(_OV7670_REG_SCALING_XSC, xsc)
        self._write_register(_OV7670_REG_SCALING_YSC, ysc)

    def _set_flip(self) -> None:
        mvfp = self._read_register(_OV7670_REG_MVFP)
        if self._flip_x:
            mvfp |= _OV7670_MVFP_MIRROR
        else:
            mvfp &= ~_OV7670_MVFP_MIRROR
        if self._flip_y:
            mvfp |= _OV7670_MVFP_VFLIP
        else:
            mvfp &= ~_OV7670_MVFP_VFLIP
        self._write_register(_OV7670_REG_MVFP, mvfp)

    @property
    def flip_x(self) -> bool:
        """Get or set the X-flip flag"""
        return self._flip_x

    @flip_x.setter
    def flip_x(self, value: int) -> None:
        self._flip_x = bool(value)
        self._set_flip()

    @property
    def flip_y(self) -> bool:
        """Get or set the Y-flip flag"""
        return self._flip_y

    @flip_y.setter
    def flip_y(self, value: int) -> None:
        self._flip_y = bool(value)
        self._set_flip()

    @property
    def night(self) -> int:
        """Get or set the night-vision mode, one of the ``OV7670_NIGHT_MODE_`` constants."""
        return self._night

    @night.setter
    def night(self, value: int) -> None:
        com11 = self._read_register(_OV7670_REG_COM11)
        com11 = (com11 & 0b00011111) | value
        self._write_register(_OV7670_REG_COM11, com11)
        self._night = value

    @property
    def product_id(self) -> int:
        """Get the product id (PID) register.  The expected value is 0x76."""
        return self._read_register(_OV7670_REG_PID)

    @property
    def product_version(self) -> int:
        """Get the version (VER) register.  The expected value is 0x73."""
        return self._read_register(_OV7670_REG_VER)

    def _write_list(self, reg_list: bytes) -> None:
        for i in range(0, len(reg_list), 2):
            self._write_register(reg_list[i], reg_list[i + 1])
            time.sleep(0.001)

    def _write_register(self, reg: int, value: int) -> None:
        b = bytearray(2)
        b[0] = reg
        b[1] = value
        with self._i2c_device as i2c:
            i2c.write(b)

    def _read_register(self, reg: int) -> int:
        b = bytearray(1)
        b[0] = reg
        with self._i2c_device as i2c:
            i2c.write(b)
            i2c.readinto(b)
        return b[0]

    def _frame_control(
        self, size: int, vstart: int, hstart: int, edge_offset: int, pclk_delay: int
    ) -> None:  # pylint: disable=too-many-arguments

        # Enable downsampling if sub-VGA, and zoom if 1:16 scale
        value = _OV7670_COM3_DCWEN if (size > OV7670_SIZE_DIV1) else 0
        if size == OV7670_SIZE_DIV16:
            value |= _OV7670_COM3_SCALEEN
        self._write_register(_OV7670_REG_COM3, value)

        # Enable PCLK division if sub-VGA 2,4,8,16 = 0x19,1A,1B,1C
        value = (0x18 + size) if (size > OV7670_SIZE_DIV1) else 0
        self._write_register(_OV7670_REG_COM14, value)

        # Horiz/vert downsample ratio, 1:8 max (H,V are always equal for now)
        value = size if (size <= OV7670_SIZE_DIV8) else OV7670_SIZE_DIV8
        self._write_register(_OV7670_REG_SCALING_DCWCTR, value * 0x11)

        # Pixel clock divider if sub-VGA
        value = (0xF0 + size) if (size > OV7670_SIZE_DIV1) else 0x08
        self._write_register(_OV7670_REG_SCALING_PCLK_DIV, value)

        # Apply 0.5 digital zoom at 1:16 size (others are downsample only)
        value = 0x40 if (size == OV7670_SIZE_DIV16) else 0x20  # 0.5, 1.0

        # Read current SCALING_XSC and SCALING_YSC register values because
        # test pattern settings are also stored in those registers and we
        # don't want to corrupt anything there.
        xsc = self._read_register(_OV7670_REG_SCALING_XSC)
        ysc = self._read_register(_OV7670_REG_SCALING_YSC)
        xsc = (xsc & 0x80) | value  # Modify only scaling bits (not test pattern)
        ysc = (ysc & 0x80) | value
        # Write modified result back to SCALING_XSC and SCALING_YSC
        self._write_register(_OV7670_REG_SCALING_XSC, xsc)
        self._write_register(_OV7670_REG_SCALING_YSC, ysc)

        # Window size is scattered across multiple registers.
        # Horiz/vert stops can be automatically calc'd from starts.
        vstop = vstart + 480
        hstop = (hstart + 640) % 784
        self._write_register(_OV7670_REG_HSTART, hstart >> 3)
        self._write_register(_OV7670_REG_HSTOP, hstop >> 3)
        self._write_register(
            _OV7670_REG_HREF,
            (edge_offset << 6) | ((hstop & 0b111) << 3) | (hstart & 0b111),
        )
        self._write_register(_OV7670_REG_VSTART, vstart >> 2)
        self._write_register(_OV7670_REG_VSTOP, vstop >> 2)
        self._write_register(_OV7670_REG_VREF, ((vstop & 0b11) << 2) | (vstart & 0b11))

        self._write_register(_OV7670_REG_SCALING_PCLK_DELAY, pclk_delay)
