import serial
from time import sleep

ser = serial.Serial(port=None, baudrate=57600)
num = 0
success = False
while not success:
    try:
        ser.port = f'/dev/ttyACM{num}'
        ser.open()

        break
    except:
        print(f"Failed to connect to /dev/ttyACM{num}")
        if num > 100:
            break
    try:
        ser.port = f'/dev/ttyUSB{num}'
        ser.open()
        success = True
        break
    except:
        print(f"Failed to connect to /dev/ttyUSB{num}")
        num += 1
        if num > 100:
            break

sleep(5)
print("A")
ser.write(bytes([255, ord('R')]))
print("B")
ser.flush()
print("C")
c = ser.read(1)
print("D")
s = bytes("", "ascii")
print("E")
while c not in (b'\n', b'\r'):
    s += c
    c = ser.read(1)
    print("F")
ser.flushInput()
print("read:", s)
