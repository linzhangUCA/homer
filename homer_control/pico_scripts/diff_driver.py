from machine import Pin, PWM


class PhaseEnableMotorDriver:
    def __init__(self, DIR_pin, PWM_pin, SLP_pin=None, pwm_freq=1000):
        # properties
        self._dir_pin_id = DIR_pin 
        self._pwm_pin_id = PWM_pin
        self._slp_pin_id = SLP_pin 
        self._pwm_freq = pwm_freq
        # pin config
        self.M_DIR = Pin(DIR_pin, Pin.OUT)  # motor direction
        self.M_PWM = PWM(Pin(PWM_pin))  # motor speed
        if SLP_pin:
            self.M_SLP = Pin(SLP_pin, Pin.OUT)
        self.M_PWM.freq(pwm_freq)
        # init
        if SLP_pin:
            self.M_SLP.value(1)  # enable motor
        self.M_PWM.duty_u16(0)
        
    def forward(self, duty=0):
        assert 0 <= duty <= 1
        self.M_DIR.value(1)
        self.M_PWM.duty_u16(int(65535 * duty))

    def backward(self, duty=0):
        assert 0 <= duty <= 1
        self.M_DIR.value(0)
        self.M_PWM.duty_u16(int(65535 * duty))

    def stop(self):
        self.M_PWM.duty_u16(0)

    def sleep(self):
        """
        Put motor to sleep mode. Use with caution.
        """
        self.stop()
        self.M_SLP.value(0)

class PhaseEnableDiffDriver:
    def __init__(self, left_pins, right_pins, pwm_freq=1000):
        assert len(left_pins)==2 or len(left_pins)==3
        assert len(left_pins)==2 or len(right_pins)==3
        # properties
        self._pwm_freq = pwm_freq
        # instantiate motors
        self.left_motor = PhaseEnableMotorDriver(left_pins[0], left_pins[1], left_pins[2], pwm_freq=pwm_freq)
        self.right_motor = PhaseEnableMotorDriver(right_pins[0], right_pins[1], right_pins[2], pwm_freq=pwm_freq)

    def forward(self, speed=0):
        assert 0 <= speed <= 1
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)

    def backward(self, speed=0):
        assert 0 <= speed <= 1
        self.left_motor.backward(speed)
        self.right_motor.backward(speed)

    def left(self, speed=0):
        assert 0 <= speed <= 1
        self.left_motor.backward(speed)
        self.right_motor.forward(speed)

    def right(self, speed=0):
        assert 0 <= speed <= 1
        self.left_motor.forward(speed)
        self.right_motor.backward(speed)
 
    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def sleep(self):
        self.left_motor.sleep()
        self.right_motor.sleep()

if __name__ == '__main__':
    from time import sleep
    bot = PhaseEnableDiffDriver(left_pins=(4, 2, 6), right_pins=(5, 3, 7))
    print("forward accel")
    for d in range(101):
        bot.forward(d/100)
        sleep(0.05)
    print("forward deccel")
    for d in reversed(range(101)):
        bot.forward(d/100)
        sleep(0.05)
    print("backward accel")
    for d in range(101):
        bot.backward(d/100)
        sleep(0.05)
    print("backward deccel")
    for d in reversed(range(101)):
        bot.backward(d/100)
        sleep(0.05)
    print("spin left accel")
    for d in range(101):
        bot.left(d/100)
        sleep(0.05)
    print("spin left deccel")
    for d in reversed(range(101)):
        bot.left(d/100)
        sleep(0.05)
    print("spin right accel")
    for d in range(101):
        bot.right(d/100)
        sleep(0.05)
    print("spin right deccel")
    for d in reversed(range(101)):
        bot.right(d/100)
        sleep(0.05)

    bot.stop()
    print("STOP")
    bot.sleep()
    print("SLEEP")
    

