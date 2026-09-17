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
    x = float(splitPacket[0])
    y = float(splitPacket[1])
    print("X = ", x, "Y = ", y)