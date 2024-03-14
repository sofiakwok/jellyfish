import cv2
from pcv.vidIO import Camera, GuaranteedVideoWriter
from datetime import datetime
from pathlib import Path

video_path = Path("async_processing/video_data")

filename = input("filename >> ") + datetime.now().strftime(" %Y-%m-%d %H-%M")

with Camera(0, cv2.CAP_DSHOW) as cam: # using DSHOW eliminates strange Windows open delay
    cam.set(cv2.CAP_PROP_EXPOSURE, -6) # speed up "shutter speed" to decrease blur
    # cam.set(cv2.CAP_PROP_FPS, 2)

    cam.record_stream((video_path / filename).with_suffix(".mp4").as_posix())#, writer=GuaranteedVideoWriter)

print(f"File saved as '{filename}'")
