"""
Rename this script to main.py, then upload to the pico board.
"""

import sys
import select
from diff_drive_controller import DiffDriveController
from machine import WDT, freq, reset

# SETUP
# Overclock
freq(200_000_000)  # Pico 2 original: 150_000_000
# Instantiate robot
homer = DiffDriveController(
    left_ids=((2, 3, 4), (20, 21)), right_ids=((6, 7, 8), (10, 11))
)
# Create a poll to receive messages from host machine
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)
event = cmd_vel_listener.poll()
# Config watchdog timer
wdt = WDT(timeout=500)  # ms

# LOOP
try:
    while True:
        # print(homer.lin_vel, homer.ang_vel)  # transmit actual robot velocity to host machine
        for msg, _ in event:
            if msg:
                target_lin_vel, target_ang_vel = 0.0, 0.0
                wdt.feed()
                buffer = msg.readline().rstrip().split(",")
                if len(buffer) == 2:
                    try:
                        target_lin_vel, target_ang_vel = (
                            float(buffer[0]),
                            float(buffer[1]),
                        )
                        homer.set_vel(target_lin_vel, target_ang_vel)
                    except ValueError:
                        # print("ValueError!")  # debug
                        reset()
            else:
                homer.set_vel(0.0, 0.0)

except Exception as e:
    # print('Pico reset')  # debug
    reset()
finally:
    # print('Pico reset')  # debug
    reset()
