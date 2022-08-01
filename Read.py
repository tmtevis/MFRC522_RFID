#!/usr/bin/env python3
# choose which CS/GPIO to use your device with, then scan an RFID tag to check for function
import RPi.GPIO as GPIO
import time
import signal
import sys
from mfrc522 import SimpleMFRC522

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

def signal_handler(sig, frame):
	print("\nProgram Successfully Terminated")
	GPIO.cleanup()
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def select_device_read(CS):
	reader = SimpleMFRC522(CS)
	time.sleep(.25)
	tid, data = reader.read()
	tag = [tid, data]
	return tag


try:
	choice = input("Select GPIO/CS of device to read from [11, 13, 15, 16, 18, 29, 31, 32, 33, 36, 37] : ")
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
	print(select_device_read(device))

finally:
	GPIO.cleanup()
	sys.exit(0)
