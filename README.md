# MFRC522_RFID
Standard tools as well as multi-device implementation

MFRC522 is a library developed for communication with 13.56MHz RFID tags/devices using tag readers equipped with the MFRC522 chipset.  My contribution/alteration to this existing library is as follows:
- Allow multi-device SPI communication, effectively expanding maximum devices from 2 to 11
  - This is accomplished by disabling the chip enable/chip select and replacing it with a GPIO that will mimic its behavior

To mimic its behavior, a number of things have to be altered within the MFRC522 library.  First, the max speed is manually set to 10 MHz, then the chip select pin is passed via the SimpleMFRC522 object in the script invoking the MFRC522 library by way of a standard set/get pair.  That CS pin is then initialized and asserted high to begin acting like the active low CS pin originally in its place.  During communication, that GPIO's output will be asserted low then back high again to signal its end before having the SPI channel closed and the GPIO cleaned up to complete its tour of duty.


How to use this addition on your Raspberry Pi:
- Navigate to your Python packages
- Replace the original MFRC522.py and SimpleMFRC522.py with MFRC522_Multi-Reader.py and SimpleMFRC522_Multi-Reader.py and rename them to the originals
- Follow the pinout scheme below for your device:

For Standard MFRC522-based 13.56MHz SPI Devices:
RFID Reader/Writer Wiring Configuration:

PIN     -------  GPIO

3.3V    -------  17

MOSI    -------  19

MISO    -------  21

RST     -------  22

SCK     -------  23

GND     -------  25

11 Available CS GPIO Pins: [11, 13, 15, 16, 18, 29, 31, 32, 33, 36, 37]

##############################################################################

To use this in your program, include the following information in your header:

##############################################################################

#!/usr/bin/env python3

import RPi.GPIO as GPIO

import time

import signal

import sys

from mfrc522 import SimpleMFRC522

##############################################################################

Then, to use the library (multi-reader or standard):

##############################################################################

reader = SimpleMFRC522() ### instantiates the reader object

reader.next_chip() ### adds 1 to the CHIP value which references the index of the CHIPSELECT list. CHIP is initialized to 0 (GPIO 13)

reader.reset() ### recommended to reset reader when changing CS/GPIO assigned

time.sleep(.25) ### minimum delay between read/write/cs-assign operations


