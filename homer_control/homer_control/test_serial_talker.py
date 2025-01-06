import serial
from time import sleep

# SETUP
messenger = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
print(messenger.name)

# LOOP
cmd_vel_msg = "0.5, 0.0\n".encode('utf-8')
for i in range(10):
    if messenger.inWaiting() > 0:
        pico_data = messenger.readline()
        pico_msg = pico_data.decode('utf-8', 'ignore')
        print(f"pico msg: {pico_msg}")
    messenger.write(cmd_vel_msg)
    sleep(1)

messenger.write("0.0, 0.0\n".encode('utf-8'))

messenger.close()
