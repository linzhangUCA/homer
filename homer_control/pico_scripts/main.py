"""
Rename this script to main.py, then upload to the pico board.
"""
import sys
import select
from diff_drive_controller import DiffDriveController

# SETUP
# Instantiate robot
homer = DiffDriveController(left_ids=((2, 3, 4), (20, 21)), right_ids=((6, 7, 8), (10, 11)))
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
