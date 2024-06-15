from get_coordinates import get_coordinates
from webcamvideostream import WebcamVideoStream
from fps import FPS
import serial
import time
import cv2
import cv2.aruco as aruco
import numpy as np

# load the camera calibration parameters
calfile = np.load('opencv/camera_calibration/calibration.npz') #TODO: get actual camera calibration parameters
newcameramtx = calfile['newcameramtx']
mtx = calfile['mtx']
dist = calfile['dist']
roi = calfile['roi']

# load origin marker pose
origin = np.load('opencv/camera_calibration/origin.npz')
rvec_origin = origin['rvec']
tvec_origin = origin['tvec']

# get the 3x3 rotation matrix (R_origin) and the 3x1 translation vector (tvec_origin)
R_origin, jac = cv2.Rodrigues(rvec_origin)
tvec_origin = np.ndarray.flatten(tvec_origin)

# use a dictionary of 50 4x4 markers
aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# aruco_dict = aruco.Dictionary_create(25, 3)

# size of the frames
width = 640
height = 480

# create the mappings to undistort the frames
map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), cv2.CV_32FC1)

# size of the marker
# same unit as the unit of the world frame coordinates
marker_length = 2.6 # TODO: update when marker is printed
#TODO: find units

# create the (default) parameters for marker detection
parameters = aruco.DetectorParameters()

# open serial connection with Arduino
# baudrate of 115200
usb_port = 'COM2' #TODO: update port
arduino = serial.Serial(port=usb_port, baudrate=115200, timeout=.01) #TODO: update baudrate?
time.sleep(2)

# start the capture (on camera channel 0) thread
print("starting capture")
cap = WebcamVideoStream(src=0).start()
# wait one second for everything to settle before reading first frame
time.sleep(1)

# start FPS counter
print("starting FPS")
fps = FPS()
fps.start()

marker_id = 2 #TODO: update when marker is printed

# save x y theta history
x_hist = []
y_hist = []
theta_hist = []
savedata = False

while not KeyboardInterrupt:
    print("reading frame:")
    frame = cap.read()
    
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
            # if the marker is detected, get its world coordinates and orientation
            if ids[i] == marker_id:
                detected = True
                frames_without_jellyfish = 0
                # estimate the marker pose wrt to the camera
                # dist = None because we already give the function an undistorted image
                rvec, tvec = aruco.estimatePoseSingleMarkers([corners[i]], marker_length, mtx, None)
                # draw the marker
                withMarkers = aruco.drawDetectedMarkers(cropped, [corners[i]], ids[i], (0, 255, 0))
                # draw its axis
                withMarkers = aruco.drawAxis(withMarkers, mtx, None, rvec, tvec, marker_length*2)
                # get the coordinates and orientations
                x, y, angle = get_coordinates(R_origin, tvec_origin, rvec, tvec)

    #TODO: write controller (JJ's job?)
    theta = 0
    alpha1 = 0
    alpha2 = 0
    # create the command to send to Arduino
    command = "%i,%i,%i" % (theta, alpha1, alpha2)

    # print the projected commands
    print("[COMMANDS]: theta={:.0f} alpha1={:.0f} alpha2={:.0f}".format(theta, alpha1, alpha2))

    # send to Arduino via serial port
    command = command + ",\n"
    arduino.write(command.encode())

    # save data
    if savedata:
        x_hist.append(x)
        y_hist.append(y)
        theta_hist.append(theta)

fps.stop()
print("[INFO] Elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] Approximate FPS: {:.2f}".format(fps.fps()))

# save data
if savedata:
    np.savetxt("data/x_hist.txt", x_hist)
    np.savetxt("data/y_hist.txt", y_hist)
    np.savetxt("data/theta_hist.txt", theta_hist)

# release capture and close all the windows
cap.stop()

# release video
cv2.destroyAllWindows()

# close the connection to arduino
arduino.close()
arduino = serial.Serial(usb_port, 115200, timeout=.01)
arduino.close()

print("finished")

