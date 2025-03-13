from machine import Timer
from wheel_controller import WheelController
import sys


class DiffDriveController:
    def __init__(self, left_ids: tuple, right_ids: tuple) -> None:
        # Wheels
        self.left_wheel = WheelController(*left_ids)
        self.right_wheel = WheelController(*right_ids)
        self.velmon_timer = Timer(
            mode=Timer.PERIODIC, freq=50, callback=self.monitor_velocity
        )
        # Properties
        self.WHEEL_SEP = 0.21  # wheel separation distance
        # Variables
        self.lin_vel = 0.0
        self.ang_vel = 0.0

    def monitor_velocity(self, timer):
        """
        Compute and transmit robot velocity
        Note - if transmitting activated, Pico may stop responding.
        Nuke the Pico if further changes on code are needed.
        """
        self.lin_vel = 0.5 * (
            self.left_wheel.lin_vel + self.right_wheel.lin_vel
        )  # robot's linear velocity
        self.ang_vel = (
            self.right_wheel.lin_vel - self.left_wheel.lin_vel
        ) / self.WHEEL_SEP  # robot's angular velocity
        sys.stdout.write(
            f"{self.lin_vel},{self.ang_vel}\n"
        )  # uncomment to transmit robot velocity

    def set_vel(self, target_lin_vel, target_ang_vel):
        left_target = target_lin_vel - 0.5 * (target_ang_vel * self.WHEEL_SEP)
        right_target = target_lin_vel + 0.5 * (target_ang_vel * self.WHEEL_SEP)
        self.left_wheel.set_lin_vel(left_target)
        self.right_wheel.set_lin_vel(right_target)


# TEST
if __name__ == "__main__":
    from time import sleep
    from math import pi, sin, cos

    bot = DiffDriveController(
        left_ids=((2, 3, 4), (20, 21)), right_ids=((6, 7, 8), (10, 11))
    )
    vel_candidates = range(5)
    for i in range(10):
        for j in range(10):
            tl, ta = sin(i / 10 * 2 * pi), 2 * pi * cos(j / 10 * 2 * pi)
            bot.set_vel(tl, ta)
            sleep(0.75)
            print(
                f"target vel: {tl} m/s, {ta} rad/s\nactual vel: {bot.lin_vel} m/s, {bot.ang_vel} rad/s\n"
            )
    bot.set_vel(0.0, 0.0)
