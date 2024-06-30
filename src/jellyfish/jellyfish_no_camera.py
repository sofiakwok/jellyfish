from get_coordinates import get_coordinates
from webcamvideostream import WebcamVideoStream
from fps import FPS
import serial
import time
import cv2
import cv2.aruco as aruco
import numpy as np

# open serial connection with Arduino
# baudrate of 115200
usb_port = 'COM8' 
arduino = serial.Serial(port=usb_port, baudrate=115200, timeout=.01) #TODO: update baudrate?
time.sleep(2)
i = 0
while not KeyboardInterrupt:
    print("in loop")
    #TODO: write controller (JJ's job?)
    theta = i
    alpha1 = 0
    alpha2 = 0
    # create the command to send to Arduino
    command = "%i,%i,%i" % (theta, alpha1, alpha2)

    # print the projected commands
    print("[COMMANDS]: theta={:.0f} alpha1={:.0f} alpha2={:.0f}".format(theta, alpha1, alpha2))

    # send to Arduino via serial port
    command = command + ",\n"
    arduino.write(command.encode())

    i += 0

# close the connection to arduino
arduino.close()
arduino = serial.Serial(usb_port, 115200, timeout=.01)
arduino.close()

print("finished")

