"""
Rename this script to main.py, then upload to the pico board.
"""
import sys
import select
from diff_drive_controller import DiffDriveController

# SETUP
# Instantiate robot
homer = DiffDriveController(left_ids=((18, 19, 20), (17, 16)), right_ids=((11, 12, 13), (14, 15)))
# Create a poll to receive messages from host machine
cmd_vel_listener = select.poll()
cmd_vel_listener.register(sys.stdin, select.POLLIN)
event = cmd_vel_listener.poll()

# LOOP
while True:
    # print(homer.lin_vel, homer.ang_vel)  # transmit actual robot velocity to host machine
    for msg, _ in event:
        buffer = msg.readline().rstrip().split(',')
        if len(buffer) == 2:
            target_lin_vel, target_ang_vel = float(buffer[0]), float(buffer[1])
            homer.set_vel(target_lin_vel, target_ang_vel)
