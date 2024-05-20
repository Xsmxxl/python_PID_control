import serial
from time import sleep

arduino = serial.Serial(port='COM5',  baudrate=115200, timeout=.1)
sleep(1.0)

def write_read(x):
    arduino.write(bytes(str(x),  'utf-8'))
    sleep(0.05)
    data = arduino.readline()
    if(data.decode('utf-8') == ""):
        data = 0
    return data.decode('utf-8')


#while True:
#    num = input("Enter a number: ")
value  = write_read(86)
print(value)