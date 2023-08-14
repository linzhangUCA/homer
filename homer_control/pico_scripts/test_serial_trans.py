"""
Connect Pico with a computer, run this code on computer to test pico_controller
"""

import serial
from time import sleep


ser = serial.Serial('/dev/ttyACM0', 115200)
x = 0.50
y = 0.
while True:
    msg = f"{x},{y}\n"
    ser.write(bytes(msg.encode('utf-8')))
    # if ser.inWaiting() > 0:
    #     pico_data = ser.readline()
    #     pico_data = pico_data.decode("utf-8","ignore")
    #     print (pico_data[:-2])
    sleep(0.1)
