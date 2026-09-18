import serial
import time

arduinoData = serial.Serial('/dev/tty.usbmodem1301', 115200)
time.sleep(1)
while (True):
    while (arduinoData.inWaiting()==0):
        pass
    dataPacket = arduinoData.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(",")
    if ((len(splitPacket) != 6)):
        continue
    accelerometer = float(splitPacket[0])
    gyroscope = float(splitPacket[1])
    magnetometer = float(splitPacket[2])
    system = float(splitPacket[3])
    roll = float(splitPacket[4])
    pitch = float(splitPacket[5])

    print("Cal:", accelerometer, gyroscope, magnetometer, system, roll, pitch)