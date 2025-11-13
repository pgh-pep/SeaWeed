import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

def write_read(angle):
    arduino.write(str(angle).encode('utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data


while True:
    num = input("Enter an angle: ")
    angle = write_read(num)
    print(angle)