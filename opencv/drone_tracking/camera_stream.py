"""Flies the drone.
Uses the serial port to send PPM commands to the drone via an Arduino.
Uses a different thread to grab frames from the webcam.
Implements a PID controller."""

from jellyfish.fps import *
from jellyfish.webcamvideostream import *
import cv2
import cv2.aruco as aruco
import numpy as np
from get_coordinates import *
import serial
import time
import scipy.io
from datetime import datetime
from scipy.signal import butter, lfilter

# load the camera calibration parameters
calfile = np.load('calibration.npz')
newcameramtx = calfile['newcameramtx']
mtx = calfile['mtx']
dist = calfile['dist']
roi = calfile['roi']

# load origin marker pose
origin = np.load('origin.npz')
rvec_origin = origin['rvec']
tvec_origin = origin['tvec']

# get the 3x3 rotation matrix (R_origin) and the 3x1 translation vector (tvec_origin)
R_origin, jac = cv2.Rodrigues(rvec_origin)
tvec_origin = np.ndarray.flatten(tvec_origin)

# use a dictionary of 25 3x3 markers
aruco_dict = aruco.Dictionary_create(25, 3)

# size of the frames
width = 640
height = 480

# create the mappings to undistort the frames
map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), cv2.CV_32FC1)

# size of the drone marker
# same unit as the unit of the world frame coordinates
marker_length = 2.6

# create the (default) parameters for marker detection
parameters = aruco.DetectorParameters_create()

# id of the drone marker
marker_id = 5

# initialize drone variables
x, y, angle = 0, 0, 0

# define a clamping function
def clamp(n, minimum, maximum):
    return max(min(maximum, n), minimum)

# define a Butterworth filter to filter the measurements
# call y = lfilter(b, a, data) to filter the data signal
order = 2  # second order filter
fs = 30  # sampling frequency is around 30 Hz #TODO: change this
nyq = 0.5 * fs
lowcut = 2  # cutoff frequency at 2 Hz
low = lowcut / nyq
b, a = butter(order, low, btype='low')

ii = 0

# record everything
t = []
x_hist = []
x_filter_hist = []
y_hist = []
y_filter_hist = []
angle_hist = []
angle_filter_hist = []
xErrorRecord = []
yErrorRecord = []
angleErrorRecord = []

# count loops
loopsCount = 0

# specify the USB port for the Arduino
usb_port = '/dev/cu.usbmodem1421'
# usb_port = '/dev/cu.usbmodem1421'

# open serial connection with Arduino
# baudrate of 115200
arduino = serial.Serial(usb_port, 115200, timeout=.01)

# wait a bit for the connection to settle
time.sleep(2)

# tell the drone to calibrate the gyroscope
# left stick to bottom left
# command = "%i,%i,%i,%i" % (throttle_off, 1000, 1000, rudder_middle)
# command = command + "\n"
# print("Starting calibration.")
# arduino.write(command.encode())
# # after 1.5 second, left stick goes back to neutral position
# time.sleep(1.5)
# command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle)
# command = command + "\n"
# arduino.write(command.encode())

# start the capture (on camera channel 0) thread
cap = WebcamVideoStream(src=0).start()

# wait one second for everything to settle before reading first frame
time.sleep(1)

# create a video of the tracking
timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
out = cv2.VideoWriter('recordings/video' + timestamp + '.mov', fourcc, 30, (612, 425), True)

# start FPS counter
fps = FPS()
fps.start()

# record time
timerStart = datetime.now()

while not KeyboardInterrupt:
    # capture a frame
    frame = cap.read()
    # undistort and crop the frame
    # cv2.undistort() is slow so we use a remapping
    # undistorted = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
    x, y, w, h = roi
    cropped = undistorted[y:y + h, x:x + w]
    # blur (optional)
    # blurred = cv2.GaussianBlur(cropped, (3, 3), 0)
    # RGB to gray
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    # drone detected or not
    detected = False
    # detect the ArUco markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # if markers are detected
    if ids is not None:
        # go through the ids
        for i in range(len(ids)):
            # if the drone marker is detected, get its world coordinates and orientation
            if ids[i] == marker_id:
                detected = True
                frames_without_jellyfish = 0
                # estimate the drone marker pose wrt to the camera
                # dist = None because we already give the function an undistorted image
                rvec, tvec = aruco.estimatePoseSingleMarkers([corners[i]], marker_length, mtx, None)
                # draw the drone marker
                withMarkers = aruco.drawDetectedMarkers(cropped, [corners[i]], ids[i], (0, 255, 0))
                # draw its axis
                withMarkers = aruco.drawAxis(withMarkers, mtx, None, rvec, tvec, marker_length*2)
                # get the coordinates and orientation of the drone
                x, y, angle = get_coordinates(R_origin, tvec_origin, rvec, tvec)
        # if markers are detected, but the drone was not detected among the markers
        if not detected:
            frames_without_jellyfish += 1
            # draw the undistorted and cropped frame, without marker and axis
            # assume same coordinates and orientation as in last frame
            withMarkers = cropped
            # same position and orientation
    else:
        frames_without_jellyfish += 1
        # draw the undistorted and cropped frame, without marker and axis
        # assume same coordinates and orientation as in last frame
        withMarkers = cropped

    # print coordinates and orientation
    print("[DRONE] X={:.1f} Y={:.1f} angle={:.1f}".format(x, y, angle))

    # display the frame
    # cv2.imshow('frame', withMarkers)

    # write in the video
    out.write(withMarkers)

    # wait 1 ms for a key to be pressed
    key = cv2.waitKey(20)

    # abort if we lost the drone for about 1.5 secs
    if frames_without_jellyfish >= 45:
        print("could not find jellyfish")

    # record the position and orientation
    x_hist.append(x)
    y_hist.append(y)
    angle_hist.append(angle)

    # filter x
    xFiltered = lfilter(b, a, x_hist)
    xFiltered = xFiltered[-1]
    x_filter_hist.append(xFiltered)
    # filter y
    yFiltered = lfilter(b, a, y_hist)
    yFiltered = yFiltered[-1]
    y_filter_hist.append(yFiltered)
    # filter angle
    angleFiltered = lfilter(b, a, angle_hist)
    angleFiltered = angleFiltered[-1]
    angle_filter_hist.append(angleFiltered)
    # record everything
    timerStop = datetime.now() - timerStart
    t.append(timerStop.total_seconds())

    # if ESC is pressed, stop the program
    if key == 27:
        print("Exit!")
        break

    # update the FPS counter
    fps.update()

fps.stop()
print("[INFO] Elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] Approximate FPS: {:.2f}".format(fps.fps()))

# release capture and close all the windows
cap.stop()

# release video
out.release()
cv2.destroyAllWindows()

# close the connection and reopen it
arduino.close()
arduino = serial.Serial(usb_port, 115200, timeout=.01)
arduino.close()

# save the log in Matlab format
scipy.io.savemat('recordings/recording' + timestamp + '.mat', mdict={'time': t, 'x': x_hist, 'xFiltered': x_filter_hist, 'y': y_hist, 'yFiltered': y_filter_hist, 'z': zRecord, 'zFiltered': zFilteredRecord, 'angle': angle_hist, 'angleFiltered': angle_filter_hist, 'xError': xErrorRecord, 'yError': yErrorRecord, 'zError': zErrorRecord, 'angleError': angleErrorRecord, 'aileron': aileronRecord, 'elevator': elevatorRecord, 'throttle': throttleRecord, 'rudder': rudderRecord})