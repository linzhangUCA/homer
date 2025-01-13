from machine import Pin, PWM

class MotorDriver:
    
    def __init__(self, ina_id, inb_id, pwm_id):
        self.ina_pin = Pin(ina_id, Pin.OUT)
        self.inb_pin = Pin(inb_id, Pin.OUT)
        self.pwm_pin = PWM(Pin(pwm_id))
        self.pwm_pin.freq(1000)
        # Stop motor
        self.pwm_pin.duty_u16(0)
        self.ina_pin.off()
        self.inb_pin.off()

    def stop(self):
        self.pwm_pin.duty_u16(0)
        
    def forward(self, duty):
        self.ina_pin.on()
        self.inb_pin.off()
        self.pwm_pin.duty_u16(duty)

    def backward(self, duty):
        self.ina_pin.off()
        self.inb_pin.on()
        self.pwm_pin.duty_u16(duty)

if __name__ == '__main__':
    from time import sleep
    # m = MotorDriver(2, 3, 4)
    m = MotorDriver(6, 7, 8)
    for d in range(200):  # ramp up
        m.forward(int(65025 / 200 * d))
        sleep(0.02)
    for d in reversed(range(200)):  # ramp up
        m.forward(int(65025 / 200 * d))
        sleep(0.02)
    for d in range(200):  # ramp up
        m.backward(int(65025 / 200 * d))
        sleep(0.02)
    for d in reversed(range(200)):  # ramp up
        m.backward(int(65025 / 200 * d))
        sleep(0.02)
    m.stop()

