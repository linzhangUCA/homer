from motor_driver import MotorDriver
from machine import Pin

class SensoredMotorDriver(MotorDriver):
    
    def __init__(self, driver_ids, encoder_ids):
        # Pin configuration
        super().__init__(*driver_ids)  # call super class's "__init__"
        self.enca_pin = Pin(encoder_ids[0], Pin.IN, Pin.PULL_UP)  # yellow  
        self.encb_pin = Pin(encoder_ids[1], Pin.IN, Pin.PULL_UP)  # white
        self.enca_pin.irq(trigger=Pin.IRQ_RISING, handler=self.update_pulses)
        # Variables
        self.pulses = 0
        # Properties
        self.PPR = 12  # pulses per revolution, PPR * 4 = CPR
        self.GEAR_RATIO = 46.8512  # speed reduction rate, v_wheel = v_motor / GEAR_RATIO
        # self.WHEEL_RADIUS = 0.032  # diameter in mm -> radius in m  
    
    def update_pulses(self, pin):
        if self.encb_pin.value() == self.enca_pin.value():  # A channel RISE later than B channel
            self.pulses -= 1
        else:
            self.pulses += 1
        
if __name__ == '__main__':
    from time import sleep
    from math import pi
    m = SensoredMotorDriver((11, 12, 13), (14, 15))
    # m = SensoredMotorDriver((18, 19, 20), (17, 16))
    prev_pulses = 0

    # Forward
    for d in range(200):  # ramp up
        m.forward(int(65025 / 200 * d))
        sleep(0.02)
        pulses_inc = m.pulses - prev_pulses
        prev_pulses = m.pulses
        revs_inc = pulses_inc / m.PPR
        rads_inc = revs_inc * 2 * pi
        ang_vel_mtr = rads_inc / 0.02  # motor angular velocity
        ang_vel_whl = ang_vel_mtr / m.GEAR_RATIO  # wheel angular velocity
        print(f"wheel angular velocity: {ang_vel_whl} rad/s")
        # lin_vel = ang_vel_wheel * m.WHEEL_RADIUS
        # print(f"wheel linear velocity: {lin_vel} m/s")        
    for d in reversed(range(200)):  # ramp down
        m.forward(int(65025 / 200 * d))
        sleep(0.02)
        pulses_inc = m.pulses - prev_pulses
        prev_pulses = m.pulses
        revs_inc = pulses_inc / m.PPR
        rads_inc = revs_inc * 2 * pi
        ang_vel_mtr = rads_inc / 0.02  # motor angular velocity
        ang_vel_whl = ang_vel_mtr / m.GEAR_RATIO  # wheel angular velocity
        print(f"wheel angular velocity: {ang_vel_whl} rad/s")
        # lin_vel = ang_vel_wheel * m.WHEEL_RADIUS
        # print(f"wheel linear velocity: {lin_vel} m/s")
    # Reverse
    for d in range(200):  # ramp up
        m.backward(int(65025 / 200 * d))
        sleep(0.02)
        pulses_inc = m.pulses - prev_pulses
        prev_pulses = m.pulses
        revs_inc = pulses_inc / m.PPR
        rads_inc = revs_inc * 2 * pi
        ang_vel_mtr = rads_inc / 0.02  # motor angular velocity
        ang_vel_whl = ang_vel_mtr / m.GEAR_RATIO  # wheel angular velocity
        print(f"wheel angular velocity: {ang_vel_whl} rad/s")
        # lin_vel = ang_vel_wheel * m.WHEEL_RADIUS
        # print(f"wheel linear velocity: {lin_vel} m/s")        
    for d in reversed(range(200)):  # ramp down
        m.backward(int(65025 / 200 * d))
        sleep(0.02)
        pulses_inc = m.pulses - prev_pulses
        prev_pulses = m.pulses
        revs_inc = pulses_inc / m.PPR
        rads_inc = revs_inc * 2 * pi
        ang_vel_mtr = rads_inc / 0.02  # motor angular velocity
        ang_vel_whl = ang_vel_mtr / m.GEAR_RATIO  # wheel angular velocity
        print(f"wheel angular velocity: {ang_vel_whl} rad/s")
        # lin_vel = ang_vel_wheel * m.WHEEL_RADIUS
        # print(f"wheel linear velocity: {lin_vel} m/s")

    m.stop()
    