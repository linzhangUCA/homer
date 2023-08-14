"""
Rename this script to main.py, then upload to the pico board.
"""
import sys
import select
from pico_controller import Robot
from time import sleep_ms


homeplate_robot = Robot(left_motor_pins=(16, 17, 2, 3), right_motor_pins=(18, 19, 4, 5))
cmd_vel_poller = select.poll()
cmd_vel_poller.register(sys.stdin, select.POLLIN)
event = cmd_vel_poller.poll()
while True:
    # read data from serial
    for cmd_vel_msg, _ in event:
        cmd_vel_buffer = cmd_vel_msg.readline().rstrip().split(',')
        if len(cmd_vel_buffer) == 2:
            target_lin, target_ang = float(cmd_vel_buffer[0]), float(cmd_vel_buffer[1])
            homeplate_robot.set_velocity(target_lin, target_ang)
            homeplate_robot.led.value(1)
    else:
        homeplate_robot.led.value(0)
