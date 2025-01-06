from machine import Timer
from wheel_controller import WheelController

class DiffDriveController:
    def __init__(self, left_ids: tuple, right_ids: tuple) -> None:
        # Wheels
        self.left_wheel = WheelController(*left_ids)
        self.right_wheel = WheelController(*right_ids)        
        self.velmon_timer = Timer(mode=Timer.PERIODIC, freq=100, callback=self.monitor_velocity)
        # Properties
        self.WHEEL_SEP = 0.21  # wheel separation distance
        # Variables
        self.lin_vel = 0.
        self.ang_vel = 0.

    def monitor_velocity(self, timer):
        self.lin_vel = 0.5 * (self.left_wheel.lin_vel + self.right_wheel.lin_vel)  # robot's linear velocity
        self.ang_vel = (self.right_wheel.lin_vel - self.left_wheel.lin_vel) / self.WHEEL_SEP  # robot's angular velocity

    def set_vel(self, target_lin, target_ang):
        v_l = target_lin - 0.5 * (target_ang * self.WHEEL_SEP)
        v_r = target_lin + 0.5 * (target_ang * self.WHEEL_SEP)
        self.left_wheel.set_vel(v_l)
        self.right_wheel.set_vel(v_r)


# TEST
if __name__=='__main__':
    from time import sleep
    
    bot = DiffDriveController(left_ids=((11, 12, 13), (14, 15)), right_ids=((18, 19, 20), (17, 16)))
    bot.set_vel(0.4, -2.0)
    sleep(1)
    print(f"Robot velocity: \n\tlin: {bot.lin_vel}, ang: {bot.ang_vel}")
    bot.set_vel(0., 0.)
    # for _ in range(50):
    #     print(bot.lin_vel, bot.ang_vel)
    #     sleep(0.1)
    # bot.set_vel(-0.45, 2.5)
    # for _ in range(50):
    #     print(bot.lin_vel, bot.ang_vel)
    #     sleep(0.1)
    
    # bot.velmon_timer.deinit()
    # bot.left_wheel.controller_timer.deinit()
    # bot.right_wheel.controller_timer.deinit()
    # bot.left_wheel.stop()
    # bot.right_wheel.stop()
    # bot.left_wheel.halt()
    # bot.right_wheel.halt()