from sensored_motor_driver import SensoredMotorDriver
from machine import Timer
from math import pi

class WheelDriver(SensoredMotorDriver):
    
    def __init__(self, driver_ids, encoder_ids):
        # Pin configuration
        super().__init__(driver_ids, encoder_ids)  # call super class's "__init__"
        self.vel_probe = Timer(mode=Timer.PERIODIC, freq=100, callback=self.exam_velocity)
        # Variables
        self.ang_vel = 0.
        self.lin_vel = 0.
        self.prev_pulses = 0
        # Properties
        self.WHEEL_RADIUS = 0.032  # diameter in mm -> radius in m  
    
    def exam_velocity(self, timer):
        delta_pulses = self.pulses - self.prev_pulses
        self.prev_pulses = self.pulses
        delta_revolutions = delta_pulses / self.PPR
        delta_radians = delta_revolutions * 2 * pi
        ang_vel_mtr = delta_radians / 0.01  # compute velocity every 0.01 seconds
        self.ang_vel = ang_vel_mtr / self.GEAR_RATIO
        self.lin_vel = self.ang_vel * self.WHEEL_RADIUS

if __name__ == '__main__':
    from time import sleep
    w = WheelDriver((11, 12, 13), (14, 15))
    # m = WheelDriver((18, 19, 20), (17, 16))
    prev_counts = 0

    # Forward
    for d in range(200):  # ramp up
        w.forward(int(65025 / 200 * d))
        sleep(0.02)
        print(f"wheel velocity: \n\tangular: {w.ang_vel} rad/s, linear velocity: {w.lin_vel} m/s")
    for d in reversed(range(200)):  # ramp down
        w.forward(int(65025 / 200 * d))
        sleep(0.02)
        print(f"wheel velocity: \n\tangular: {w.ang_vel} rad/s, linear velocity: {w.lin_vel} m/s")
    # Reverse
    for d in range(200):  # ramp up
        w.backward(int(65025 / 200 * d))
        sleep(0.02)
        print(f"wheel velocity: \n\tangular: {w.ang_vel} rad/s, linear velocity: {w.lin_vel} m/s")
    for d in reversed(range(200)):  # ramp down
        w.backward(int(65025 / 200 * d))
        sleep(0.02)
        print(f"wheel velocity: \n\tangular: {w.ang_vel} rad/s, linear velocity: {w.lin_vel} m/s")

    w.stop()