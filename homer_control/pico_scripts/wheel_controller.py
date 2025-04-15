from wheel_driver import WheelDriver
from machine import Timer


class WheelController(WheelDriver):
    def __init__(self, driver_ids, encoder_ids):
        # Pin configuration
        super().__init__(driver_ids, encoder_ids)  # call super class's "__init__"
        self.vel_executor = Timer(
            mode=Timer.PERIODIC, freq=30, callback=self.regulate_velocity
        )
        # Variables
        self.target_vel = 0.0
        self.err = 0.0
        self.prev_err = 0.0
        self.acc_err = 0.0
        self.diff_err = 0.0
        self.duty = 0
        self.set_vel_counter = 0
        # Properties: Incremental PID
        self.K_P = 16384.0
        self.K_I = 0.0
        self.K_D = 0.0

    def regulate_velocity(self, timer):
        self.err = self.target_vel - self.lin_vel
        self.acc_err += self.err  # err_sum = err_sum + err
        self.diff_err = self.err - self.prev_err
        self.prev_err = self.err
        inc_duty = (
            self.K_P * self.err + self.K_I * self.acc_err + self.K_D * self.diff_err
        )
        self.duty += inc_duty
        if self.duty > 0:  # forward
            if self.duty > 65535:
                self.duty = 65535
            self.forward(int(self.duty))
        elif self.duty < 0:  # backward
            if self.duty < -65535:
                self.duty = -65535
            self.backward(int(-self.duty))  # self.duty is negative
        else:
            self.stop()
        # No need for PID control if target velocity is 0
        if self.target_vel == 0:
            self.stop()
        self.set_vel_counter += 1

    def set_lin_vel(self, target_vel):
        """
        Set a reference LINEAR VELOCITY for this wheel
        """
        if target_vel is not self.target_vel:
            self.target_vel = target_vel
            self.acc_err = 0.0
        else:
            if not self.set_vel_counter % 30:
                self.stop()


# TEST
if __name__ == "__main__":
    from time import sleep

    w = WheelController((2, 3, 4), (20, 21))
    # w = WheelController((6, 7, 8), (10, 11))
    print(f"target velocity: {w.target_vel}")
    for v in range(1, 11):
        w.set_lin_vel(v / 10)
        sleep(1.5)
        print(f"target velocity: {w.target_vel}, actual velocity: {w.lin_vel}")
    for v in reversed(range(10)):
        w.set_lin_vel(v / 10)
        sleep(1.5)
        print(f"target velocity: {w.target_vel}, actual velocity: {w.lin_vel}")
    w.set_lin_vel(0)
