from machine import Pin, Timer
from diff_driver import PhaseEnableMotorDriver
from math import pi


class PhaseEnableMotorController(PhaseEnableMotorDriver):
    def __init__(self, ENCA_pin, ENCB_pin, DIR_pin, PWM_pin, SLP_pin=None, pwm_freq=1000, ab_order=1):
        super().__init__(DIR_pin, PWM_pin, SLP_pin, pwm_freq)
        assert ab_order == 1 or ab_order == -1
        # Properties
        self._enca_pin_id = ENCA_pin
        self._encb_pin_id = ENCB_pin
        self.ab_order = ab_order
        self.CPR = 48
        self.gear_ratio = 46.8512
        self.monitor_freq = 50
        self.controller_freq = 100
        self.K_P = 0.01
        self.K_I = 0.002
        self.K_D = 0.
        # Config pico
        self.ENCA = Pin(ENCA_pin, Pin.IN, Pin.PULL_DOWN)
        self.ENCB = Pin(ENCB_pin, Pin.IN, Pin.PULL_DOWN)
        self.ENCA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.handle_enca)
        self.ENCB.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.handle_encb)
        self.velocity_monitor = Timer()
        self.velocity_monitor.init(mode=Timer.PERIODIC, freq=self.monitor_freq, callback=self.vel_mon_cb)
        self.velocity_controller = Timer()
        self.velocity_controller.init(mode=Timer.PERIODIC, freq=self.controller_freq, callback=self.vel_con_cb)
        # Variables
        self.encoder_counter = 0
        self.prev_counts = 0
        self.err = 0.
        self.prev_err = 0.
        self.err_sum = 0.
        self.dutycycle = 0.
        self.target_vel = 0.
        self.velocity = 0.
        self.enca_val = self.ENCA.value()
        self.encb_val = self.ENCB.value()
    
    def handle_enca(self, pin):
        if self.ENCA.value():
            if self.ENCB.value():
                self.encoder_counter -= self.ab_order
            else:
                self.encoder_counter += self.ab_order
        else:
            if self.ENCB.value():
                self.encoder_counter += self.ab_order
            else:
                self.encoder_counter -= self.ab_order

    def handle_encb(self, pin):
        if self.ENCB.value():
            if self.ENCA.value():
                self.encoder_counter += self.ab_order
            else:
                self.encoder_counter -= self.ab_order
        else:
            if self.ENCA.value():
                self.encoder_counter -= self.ab_order
            else:
                self.encoder_counter += self.ab_order
                
    def vel_mon_cb(self, timer):
        counts_diff = self.encoder_counter - self.prev_counts
        self.velocity = counts_diff / (self.CPR * self.gear_ratio) * 2 * pi * self.monitor_freq
        self.prev_counts = self.encoder_counter

    def vel_con_cb(self, timer):
        err = self.target_vel - self.velocity
        err_diff = err - self.prev_err
        self.err_sum += err_diff
        self.prev_err = err
        self.dutycycle += self.K_P * self.err + self.K_D * err_diff + self.K_I * self.err_sum
        if self.dutycycle > 0:
            if self.dutycycle > 1:
                self.dutycycle = 1
            self.forward(duty=self.dutycycle)
        elif self.dutycycle < 0:
            if self.dutycycle < -1:
                self.dutycycle = -1
            self.backward(duty=-self.dutycycle)
        else:
            self.stop()

    def set_vel(self, target_vel):
        if not target_vel == self.target_vel:  # zero error if new target vel set
            self.err_sum = 0.
        self.target_vel = target_vel

if __name__ == '__main__':
    from time import sleep
    lm = PhaseEnableMotorController(10, 11, 4, 2, 6, ab_order=-1)
#     lm.set_vel(-12)
#     while lm.encoder_counter <= 2248.86 * 4:
    for i in range(40):
        lm.set_vel(i*0.4)
        print(lm.velocity)
        sleep(0.5)
    for i in reversed(range(40)):
        lm.set_vel(i*0.4)
        print(lm.velocity)
        sleep(0.5)
    for i in range(40):
        lm.set_vel(-i*0.4)
        print(lm.velocity)
        sleep(0.5)
    for i in reversed(range(40)):
        lm.set_vel(-i*0.4)
        print(lm.velocity)
        sleep(0.5)
    lm.sleep()

