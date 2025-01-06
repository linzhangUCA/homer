from wheel_driver import WheelDriver
from machine import Timer


class WheelController(WheelDriver):
    def __init__(self, driver_ids, encoder_ids):
        # Pin configuration
        super().__init__(driver_ids, encoder_ids)  # call super class's "__init__"
        self.vel_executor = Timer(mode=Timer.PERIODIC, freq=50, callback=self.regulate_velocity)
        # Variables
        self.target_vel = 0.
        self.err = 0.
        self.prev_err = 0.
        self.sum_err = 0.
        self.diff_err = 0.
        self.duty = 0
        # Properties
        self.K_P = 6500.
        self.K_I = 120.
        self.K_D = 0.
    
    def regulate_velocity(self, timer):
        self.err = self.target_vel - self.lin_vel
        self.sum_err += self.err  # err_sum = err_sum + err
        self.diff_err = self.err - self.prev_err
        self.prev_err = self.err
        d_duty = self.K_P * self.err + self.K_I * self.sum_err + self.K_D * self.diff_err  # compute change of duty cycle with PID
        self.duty += d_duty
        if self.duty > 0:  # forward
            if self.duty > 65025:
                self.duty = 65025
            self.forward(int(self.duty))
        elif self.duty < 0:  # backward
            if self.duty < -65025:
                self.duty = -65025
            self.backward(int(-self.duty))  # self.duty is negative
        else:
            self.stop()
        # No need for PID control if target velocity is 0
        if self.target_vel == 0:
            self.stop()

    def set_vel(self, target_vel):
        """
        Set a reference LINEAR VELOCITY for this wheel 
        """
        self.target_vel = target_vel
        self.sum_err = 0.

# TEST
if __name__ == '__main__':
    from time import sleep
    w = WheelController((11, 12, 13), (14, 15))
    # w = WheelController((18, 19, 20), (17, 16))
    print(f"target velocity: {w.target_vel}")
    for v in range(1, 11):
        w.set_vel(v / 10)
        sleep(1)
        print(f"target velocity: {w.target_vel}, actual velocity: {w.lin_vel}")
    for v in reversed(range(10)):
        w.set_vel(v / 10)
        sleep(1)
        print(f"target velocity: {w.target_vel}, actual velocity: {w.lin_vel}")
    w.set_vel(0)