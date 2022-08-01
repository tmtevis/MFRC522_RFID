#!/usr/bin/env python3
# choose which GPIO/CS to use your device with and scan/write the test text to the RFID tag
import os
import signal
import sys
import time
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

GPIO.setmode(GPIO.BOARD)

def signal_handler(sig, frame):
	print("\nProgram Successfully Terminated")
	sys.exit(0)

def select_device_write(CS, SerialNumber, Hours):
	data = str(SerialNumber + ':' + Hours)
	reader = SimpleMFRC522(CS)
	time.sleep(.25)
	reader.write(data)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


try:

	choice = input("Select GPIO/CS to Write To [11, 13, 15, 16, 18, 29, 31, 32, 33, 36, 37] : ")
	print("GPIO #", choice)
	device = 0
	if choice == '11':
        	device = 0
	elif choice == '13':
        	device = 1
	elif choice == '15':
        	device = 2
	elif choice == '16':
        	device = 3
	elif choice == '18':
        	device = 4
	elif choice == '29':
        	device = 5
	elif choice == '31':
        	device = 6
	elif choice == '32':
        	device = 7
	elif choice == '33':
        	device = 8
	elif choice == '36':
        	device = 9
	elif choice == '37':
        	device = 10

	print("CHIP SELECT #", device)
	select_device_write(device, "Test Test!", "Hello World!")
	print("WRITE SUCCESSFULL")
finally:
	GPIO.cleanup()
