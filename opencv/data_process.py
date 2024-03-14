from contextlib import contextmanager
from math import atan2, pi
from dataclasses import dataclass
from typing import Any
import os
from pathlib import Path
import pickle

from prompt_toolkit import prompt
from prompt_toolkit.completion import WordCompleter

import cv2
from alive_progress import alive_bar
from pcv.vidIO import VideoReader
from pupil_apriltags import Detector

from c_stderr_redirect import c_stderr_redirected

import sys

# setup apriltags detector
## USB Webcam
# fx, fy = (820, 820) # camera focal length, px
# cx, cy = (320, 240) # camera focal center, px (usually center)
## Nikon Camera
fx, fy = (3.433e3, 3.433e3) # camera focal length, px
cx, cy = (9.789e2, 1.0929e3) # camera focal center, px (usually center)
tagsize = 2.7e-2# size of tag in [m]
detector = Detector(families="tagStandard41h12")#, quad_sigma=0.8)
angles = {42: 0, 64: 0, 18: 0}

video_path = Path("async_processing/video_data")
pose_path = Path("async_processing/pose_data")
angle_path = Path("async_processing/angle_data")

def main():
    filename = prompt("filename >> ", completer=WordCompleter(os.listdir(video_path)))
    link_poses = {}
    angle_record = {}

    if os.path.isdir(video_path/filename): # asked for processing of multiple files
        dirname = Path(filename)
        filenames = [dirname/s for s in os.listdir(video_path/dirname)]
        try:
            os.mkdir(pose_path/filename)
        except FileExistsError:
            pass
        try:
            os.mkdir(angle_path/filename)
        except FileExistsError:
            pass
    else:
        filenames = [Path(filename)]

    for filename in filenames:
        if not filename.suffix.lower() == '.mp4':
            continue
        if not os.path.exists((pose_path / filename).with_suffix(".pkl")):
            try:
                print(f"Processing {filename}")
                with VideoReader(str(video_path / filename), destroy=None) as vid, c_stderr_redirected(), alive_bar(int(vid.get(cv2.CAP_PROP_FRAME_COUNT)-2), bar='fish') as bar:
                    for _, frame_img in vid:
                        gray = cv2.cvtColor(frame_img, cv2.COLOR_BGR2GRAY)
                        
                        # with c_stderr_redirected():
                        res = detector.detect(gray, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=tagsize)

                        link_poses[vid.frame] = {tag.tag_id: tag for tag in res}

                        # for each pair of tags, compute extract absolute angle
                        for tag in res:
                            θ = atan2(tag.pose_R[1,0], tag.pose_R[0,0])*180/pi
                            angles[tag.tag_id] = θ

                        # angle differences 
                        δ = angles[42] - angles[18]
                        α = angles[64] - angles[42]

                        angle_record[vid.frame] = (δ, α)

                        bar()
            except Exception as e:
                print(e)

            # manually destroy cv window, since the libary is bad at this
            # window_still_exists = cv2.getWindowProperty('video', cv2.WND_PROP_VISIBLE) > -1
            try:
                cv2.destroyWindow('video')
            except:
                pass # if the window is already closed, thats fine, but we don;t want it to end the program

            # pickle pose data
            with open((pose_path / filename).with_suffix(".pkl"), 'wb+') as f:
                pickle.dump(link_poses, f)

            # pickle angle data
            with open((angle_path / filename).with_suffix(".pkl"), 'wb+') as f:
                pickle.dump(angle_record, f)
        
        else: # load existing poses
            with open((pose_path / filename).with_suffix(".pkl"), 'rb') as f:
                link_poses = pickle.load(f)

        draw = pose_draw_process(link_poses)
        with VideoReader(str(video_path / filename), process=draw, fps=30, skip_frames=True) as vid:
            draw.vid = vid # bind VideoReader so we can reference frame
            vid.stream()
            vid.record_stream("async_processing/last_video.mp4", show=True)

@dataclass
class pose_draw_process():
    poses: Any
    vid: VideoReader = None

    def __call__(self, frame_img: Any):
        out_img = frame_img.copy()
        for tag in self.poses.get(self.vid.frame+1, {}).values():
            out_img = cv2.polylines(img=out_img, pts=[tag.corners.reshape((-1,1,2)).astype(int)], isClosed=True, color=(0,255,0), thickness=2)
        return out_img

main()
