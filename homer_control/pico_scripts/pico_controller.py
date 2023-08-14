"""
Upload this MicroPython script to the pico board.
"""
import sys
from math import pi
from machine import Pin, PWM, Timer

class Motor:

    def __init__(self, dir_pin, pwm_pin, enca_pin, encb_pin, ab_order=1, frequency=1000):
        # set pins
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._enca_pin = Pin(enca_pin, Pin.IN, Pin.PULL_UP)
        self._encb_pin = Pin(encb_pin, Pin.IN, Pin.PULL_UP)
        self._pwm_pin.freq(frequency)
        self._enca_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._enca_handler)
        self._encb_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._encb_handler)
        # constants
        self.ab_order = ab_order  # 1: a triggers first; -1: b triggers first
        self.CPR = 48
        self.GEAR_RATIO = 46.85
        self.monitor_period = 10  # millisecond
        self.controller_period = 20  # millisecond
        self.K_P = 0.01
        self.K_I = 0.002
        self.K_D = 0
        # variables
        self.encoder_counts = 0
        self.prev_counts = 0
        self.velocity = 0.  # actual motor velocity
        self.target_vel = 0.
        self.enca_val = self._enca_pin.value()
        self.encb_val = self._encb_pin.value()
        self.err = 0.
        self.prev_err = 0.
        self.diff_err = 0.
        self.inte_err = 0.
        self.dutycycle = 0.
        # set timer to monitor and control motor velocity
        self._velocity_monitor = Timer(
            mode=Timer.PERIODIC, 
            period=self.monitor_period, 
            callback=self._velmon_cb
        )
        self._velocity_controller = Timer(
            mode=Timer.PERIODIC, 
            period=self.controller_period, 
            callback=self._velcon_cb
        )

    def _enca_handler(self, pin):
        # self.encoder_counts, self.a_val
        self.enca_val = pin.value()
        if self.enca_val == 1:
            if self.encb_val == 0:
                self.encoder_counts -= 1
            else:
                self.encoder_counts += 1
        else:
            if self.encb_val == 1:
                self.encoder_counts -= 1
            else:
                self.encoder_counts += 1

    def _encb_handler(self, pin):
        self.encb_val = pin.value()
        if self.encb_val == 1:
            if self.enca_val == 0:
                self.encoder_counts += 1
            else:
                self.encoder_counts -= 1
        else:
            if self.enca_val == 1:
                self.encoder_counts += 1
            else:
                self.encoder_counts -= 1

    def _velmon_cb(self, timer):
        """
        compute motor velocity
        """
        counts_diff = self.encoder_counts - self.prev_counts
        self.velocity = self.ab_order * counts_diff / (self.CPR * self.GEAR_RATIO * self.monitor_period / 1000) * (2 * pi)
        self.prev_counts = self.encoder_counts

    def _velcon_cb(self, timer):
        """
        bring motor velocity to target
        """
        self.err = self.target_vel - self.velocity
        self.diff_err = self.err - self.prev_err
        self.inte_err += self.diff_err
        self.prev_err = self.err
        self.dutycycle += self.K_P * self.err + self.K_D * self.diff_err + self.K_I * self.inte_err
        if self.dutycycle > 0:
            if self.dutycycle > 1:
                self.dutycycle = 1
            self._forward(duty=self.dutycycle)
        elif self.dutycycle < 0:
            if self.dutycycle < -1:
                self.dutycycle = -1
            self._backward(duty=-self.dutycycle)
        else:
            self.stop()

    def stop(self):
        """
        Makes the motor stop.
        """
        self._pwm_pin.duty_u16(0)
        self.prev_err = 0.
        self.diff_err = 0.
        self.inte_err = 0.

    def set_velocity(self, target_vel):
        if not target_vel == self.target_vel:  # zero error if new target vel set
            self.prev_err, self.diff_err, self.inte_err = 0, 0, 0
        self.target_vel = target_vel

    def _forward(self, duty=1.0):
        """
        Set motor speed according to PWM signal's duty cycle.
        """
        self._dir_pin.value(0)
        self._pwm_pin.duty_u16(int(duty*65536))

    def _backward(self, duty=1.0):
        """
        Set motor speed according to PWM signal's duty cycle.
        """
        self._dir_pin.value(1)
        self._pwm_pin.duty_u16(int(duty*65536))

    def pwm_forward(self, duty=1.0):
        """
        USE WITH CAUTION!!! Will disable velocity control timer.
        """
        self._velocity_controller.deinit()
        self._forward(duty=duty)

    def pwm_backward(self, duty=1.0):
        """
        USE WITH CAUTION!!! Will disable velocity control timer.
        """
        self._velocity_controller.deinit()
        self._backward(duty=duty)


class Robot:
    
    def __init__(self, left_motor_pins, right_motor_pins, frequency=1000):
        assert len(left_motor_pins)==4
        assert len(right_motor_pins)==4
        self._left_motor = Motor(
            dir_pin=left_motor_pins[0], 
            pwm_pin=left_motor_pins[1], 
            enca_pin=left_motor_pins[2], 
            encb_pin=left_motor_pins[3], 
            ab_order=1, 
            frequency=frequency
        )
        self._right_motor = Motor(
            dir_pin=right_motor_pins[0], 
            pwm_pin=right_motor_pins[1], 
            enca_pin=right_motor_pins[2], 
            encb_pin=right_motor_pins[3], 
            ab_order=-1, 
            frequency=frequency
        )
        self.led = Pin(25, Pin.OUT)  # serial data receiving indicator
        # constants
        self.WHEEL_RADIUS = 0.0375
        self.WHEEL_SEPARATION = 0.205
        self.tx_period = 20  # millisecond
        # variables
        self.linear_velocity = 0.  # actual velocity
        self.angular_velocity = 0.
        self.target_lin = 0.  # target velocity
        self.target_ang = 0.
        # set a timer to broadcast measured velocity via usb serial port
        self._velocity_transmitter = Timer(
            mode=Timer.PERIODIC, 
            period=self.tx_period, 
            callback=self._veltrans_cb
        )

    def _veltrans_cb(self, timer):
        """
        Compute and transmit robot's measured linear and angular velocity
        """
        self.left_lin_vel = self._left_motor.velocity * self.WHEEL_RADIUS
        self.right_lin_vel = self._right_motor.velocity * self.WHEEL_RADIUS
        self.linear_velocity = (self.left_lin_vel + self.right_lin_vel) / 2
        self.angular_velocity = (self.right_lin_vel - self.left_lin_vel) / self.WHEEL_SEPARATION
        sys.stdout.write(f"{self.linear_velocity},{self.angular_velocity}\n")

    def forward(self, speed=1.0):
        """
        USE WITH CAUTION!!! Will disable velocity control timer.
        """
        self._left_motor.pwm_forward(duty=speed)
        self._right_motor.pwm_forward(duty=speed)

    def backward(self, speed=1.0):
        """
        USE WITH CAUTION!!! Will disable velocity control timer.
        """
        self._left_motor.pwm_backward(duty=speed)
        self._right_motor.pwm_backward(duty=speed)

    def stop(self):
        "Halt"
        self.set_velocity(0, 0)
        self._left_motor.stop()
        self._right_motor.stop()

    def set_velocity(self, target_lin, target_ang):
        self.target_lin = target_lin
        self.target_ang = target_ang
        left_target_vel = (target_lin - (target_ang * self.WHEEL_SEPARATION) / 2) / self.WHEEL_RADIUS  # motor angular
        right_target_vel = (target_lin + (target_ang * self.WHEEL_SEPARATION) / 2) / self.WHEEL_RADIUS
        self._left_motor.set_velocity(left_target_vel) 
        self._right_motor.set_velocity(right_target_vel)
