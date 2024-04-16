import sys
from machine import Timer
from wheel_controller import WheelController

class DiffDriveController:
    def __init__(self, left_pins: tuple, right_pins: tuple) -> None:
        # Wheels
        self.left_wheel = WheelController(*left_pins)
        self.right_wheel = WheelController(*right_pins)        
        # Properties
        self.WHEEL_SEP = 0.207
        # Variables
        self.lin_vel = 0.
        self.ang_vel = 0.
        # Velocity monitor timer
        self.velmon_timer = Timer()
        self.velmon_timer.init(freq=100, callback=self.velmon_cb)

    def velmon_cb(self, timer):
        self.lin_vel = 0.5 * (self.left_wheel.lin_vel + self.right_wheel.lin_vel)
        self.ang_vel = (self.right_wheel.lin_vel - self.left_wheel.lin_vel) / self.WHEEL_SEP

    def set_vel(self, target_lin, target_ang):
        v_l = target_lin - 0.5 * (target_ang * self.WHEEL_SEP)
        v_r = target_lin + 0.5 * (target_ang * self.WHEEL_SEP)
        self.left_wheel.set_vel(v_l)
        self.right_wheel.set_vel(v_r)


# TEST
if __name__=='__main__':
    from time import sleep
    bot = DiffDriveController(left_pins=(4, 2, 6, 10, 11), right_pins=(5, 3, 7, 12, 13))
    bot.set_vel(0.4, -2.0)
    for _ in range(50):
        print(bot.lin_vel, bot.ang_vel)
        sleep(0.1)
    bot.set_vel(-0.45, 2.5)
    for _ in range(50):
        print(bot.lin_vel, bot.ang_vel)
        sleep(0.1)
    
    bot.velmon_timer.deinit()
    bot.left_wheel.controller_timer.deinit()
    bot.right_wheel.controller_timer.deinit()
    bot.left_wheel.stop()
    bot.right_wheel.stop()
    bot.left_wheel.halt()
    bot.right_wheel.halt()

