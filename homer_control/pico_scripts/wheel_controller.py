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
        self.K_P = 3000.
        self.K_I = 0.
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
        self.target_vel = target_vel
        self.sum_err = 0.

# TEST
if __name__ == '__main__':
    from time import sleep
    w = WheelController((11, 12, 13), (14, 15))
    w.set_vel(0.5)  # m/s
    print(f"target velocity: {w.target_vel}")
    for _ in range(5):
        print(f"actual velocity: {w.lin_vel}")
        sleep(1)
    w.set_vel(0)

#     def __init__(
#         self, 
#         ina_id: int, 
#         inb_id: int, 
#         pwm_id: int, 
#         enca_id: int, 
#         encb_id: int,
#     ) -> None:
#         self.INA_PIN = Pin(ina_id, Pin.OUT)
#         self.INB_PIN = Pin(inb_id, Pin.OUT)
#         self.PWM_PIN = PWM(Pin(pwm_id))
#         self.PWM_PIN.freq(1000)
#         self.ENCA_PIN = Pin(enca_id, Pin.IN, Pin.PULL_DOWN)
#         self.ENCB_PIN = Pin(encb_id, Pin.IN, Pin.PULL_DOWN)
#         self.ENCA_PIN.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.inc_counts_a)
#         self.ENCB_PIN.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.inc_counts_b)
#         # Properties
#         self.WHEEL_RADIUS = 0.0375  # m
#         self.GEAR_RATIO = 46.8512
#         self.CPR = 48
#         self.K_P = 0.3
#         self.K_I = 0.
#         self.K_D = 0.
#         # Variables
#         self.wdir = 0  # wheel direction: 1: forward
#         self.enc_counts = 0
#         self.prev_enc_counts = 0
#         self.lin_vel = 0.
#         self.duty = 0.
#         self.err = 0.
#         self.prev_err = 0.
#         self.err_sum = 0.
#         self.ref_vel = 0.
#         # Linear velocity monitor timer
#         self.monitor_timer = Timer()
#         self.monitor_timer.init(freq=100, callback=self.mon_cb)
#         # PID controller timer
#         self.controller_timer = Timer()
#         self.controller_timer.init(freq=100, callback=self.con_cb)

#     def inc_counts_a(self, pin):
#         self.enc_counts += self.wdir

#     def inc_counts_b(self, pin):
#         self.enc_counts += self.wdir

#     def mon_cb(self, timer):
#         enc_diff = self.enc_counts - self.prev_enc_counts
#         omega_m = enc_diff / self.CPR * 2 * pi * 100 # angular velocity on motor shaft (rad / s)
#         omega_w = omega_m / self.GEAR_RATIO
#         self.lin_vel = omega_w * self.WHEEL_RADIUS
#         self.prev_enc_counts = self.enc_counts

#     def con_cb(self, timer):
#         self.err = self.ref_vel - self.lin_vel
#         self.err_sum += self.err  # err_sum = err_sum + err
#         self.err_diff = self.err - self.prev_err
#         self.prev_err = self.err
#         duty_inc = self.K_P * self.err + self.K_I * self.err_sum + self.K_D * self.err_diff  # Proportional, Integral, Derivative
#         self.duty += duty_inc
#         if self.duty > 0:  # forward
#             if self.duty >= 1:
#                 self.duty = 0.999
#             self._forward(self.duty)
#         elif self.duty < 0:  # backward
#             if self.duty <= -1:
#                 self.duty = -0.999
#             self._backward(-self.duty)
#         else:
#             self.stop()
#         if self.ref_vel == 0:
#             self.dc = 0.
#             self.stop()

#     def set_vel(self, target_vel):
#         self.ref_vel = target_vel
#         self.err_sum = 0.

#     def _forward(self, duty: float = 0.0):
#         assert -1.<=duty<= 1.
#         self.INA_PIN.value(0)
#         self.INB_PIN.value(1)
#         self.PWM_PIN.duty_u16(int(65535 * duty))
#         self.wdir = 1 

#     def _backward(self, duty: float = 0.0):
#         assert -1.<=duty<= 1.
#         self.INA_PIN.value(1)
#         self.INB_PIN.value(0)
#         self.PWM_PIN.duty_u16(int(65535 * duty))
#         self.wdir = -1 

#     def stop(self):
#         self.PWM_PIN.duty_u16(0)
#         self.wdir = 0 

#     def halt(self):
#         self.INA_PIN.value(0)
#         self.INB_PIN.value(0)
#         self.stop()
#         self.wdir = 0 

# # TEST
# if __name__ == '__main__':
#     from time import sleep
#     lwh.set_vel(-0.44)
#     rwh.set_vel(-0.4)
#     for _ in range(400):
#         print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.01)
#     for d in range(10):
#         lwh.set_vel(d/10)
#         rwh.set_vel(d/10)
#         print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.5)
#     print(lwh.lin_vel, rwh.lin_vel)
#     for d in reversed(range(10)):
#         lwh.set_vel(d/10)
#         rwh.set_vel(d/10)
#         print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.5)
#     print(lwh.lin_vel, rwh.lin_vel)
#     for d in range(10):
#         lwh.set_vel(-d/10)
#         rwh.set_vel(-d/10)
#         print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.5)
#     print(lwh.lin_vel, rwh.lin_vel)
#     for d in reversed(range(10)):
#         lwh.set_vel(-d/10)
#         rwh.set_vel(-d/10)
#         print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.5)
#     print(lwh.lin_vel, rwh.lin_vel)
#     lwh.controller_timer.deinit()
#     rwh.controller_timer.deinit()

#     for d in range(100):
#         lwh._forward(duty=d/100)
#         rwh._forward(duty=d/100)
#         # print(lwh.enc_counts, rwh.enc_counts)
#         # print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.04)
#     for d in reversed(range(100)):
#         lwh._forward(duty=d/100)
#         rwh._forward(duty=d/100)
#         # print(lwh.enc_counts, rwh.enc_counts)
#         # print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.04)
#     for d in range(100):
#         lwh._backward(duty=d/100)
#         rwh._backward(duty=d/100)
#         # print(lwh.enc_counts, rwh.enc_counts)
#         # print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.04)
#     for d in reversed(range(100)):
#         lwh._backward(duty=d/100)
#         rwh._backward(duty=d/100)
#         # print(lwh.enc_counts, rwh.enc_counts)
#         # print(lwh.lin_vel, rwh.lin_vel)
#         sleep(0.04)
#     lwh.stop()
#     rwh.stop()
