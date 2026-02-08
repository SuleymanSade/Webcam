# import pandas as pd
import numpy as np
import cv2
from pupil_apriltags import Detector
from networktables import NetworkTablesInstance
import json
import os
import time

# look into cv2's camera calibration for better parameters
# This changes from camera to camera
# Camera parameters (fx, fy, cx, cy)
CAM_PARAMETERS = (1048.508930, 1046.948683, 594.227767, 366.160094) # TODO: placeholder values, change these with real calibration

# global shutter
# >90 deg
# fixed focus
class WebCam():
    def __init__(self, id: str = "0"):
        with open("webcam.json", 'r', encoding="utf-8") as file:
            settings = json.load(file)
            self.cam = cv2.VideoCapture(settings[id]["stream"])
        if(not self.cam.isOpened()):
            print("Camera not found or cannot be opened.")
        
        self.detector = Detector(
            families="tag36h11", # default, FRC uses tag36h11
            nthreads=1,
            # quad_decimate=1.0,
            quad_sigma=0.3,
            # refine_edges=1,
            decode_sharpening=0.5,
            # debug=0
        )

        # look into cv2's camera calibration for better parameters
        # This changes from camera to camera
        # Camera parameters (fx, fy, cx, cy)
        with open("webcam.json", 'r', encoding="utf-8") as file:
            settings = json.load(file)
            self.cam_params = (settings[id]['fx'], settings[id]['fy'], settings[id]['cx'], settings[id]['cy']) # TODO: placeholder values, change these with real calibration
        self.tag_size_meters = 0.1651 #in meters
        self.id = id
        
        # self.cam_params = CAM_PARAMETERS
        
        # NetworkTables setup
        self.ntinst = NetworkTablesInstance.getDefault()
        self.ntinst.startClient("cam" + str(self.id))
        self.ntinst.setServerTeam(2635)
        # self.ntinst.setServer("localhost")  # For testing purposes
        self.ntinst.startDSClient()
        self.table = self.ntinst.getTable("apriltags" + str(self.id))

    def push_network_table(self, detections):
        x_list = []
        y_list = []
        z_list = []
        tag_ids = []
        
        try:        
            for det in detections:
                tag_id, pos = det
                pos = pos.flatten() # turns to 1D numpy array
                tag_ids.append(tag_id)
                # pos.flatten() has [x, y, z]
                x_list.append(pos[0])
                y_list.append(pos[1])
                z_list.append(pos[2] / 3 * 2)
        except:
            pass
            # return 
            
            
        self.table.putNumberArray("tag_id", tag_ids)
        self.table.putNumberArray("x", x_list)
        self.table.putNumberArray("y", y_list)
        self.table.putNumberArray("z", z_list)

    def detect(self) -> np.ndarray:
        ret, frame = self.cam.read()
        if not ret:
            print("Failed to grab frame")
            return np.nan

        # Needs gray for detections
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # cv2.imshow("Camera Frame", frame)

        # Detect AprilTags
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.cam_params,
            tag_size=self.tag_size_meters
        )
                
        # axis:
        # z, positive z is away from camera
        # x, positive x is to the right
        # y, positive y is down

        if detections:
            for det in detections or []:
                print(f"Detected Tag ID: {det.tag_id}")
                print(f"Pose (X,Y,Z): {det.pose_t.flatten() / 3 * 2}")
                
            return [(det.tag_id, det.pose_t.flatten()) for det in detections]
        
        return np.nan

try:
    cam = WebCam("test")
except:
    print("cannot access 1")
    
# try:
#     cam2 = WebCam("0")
# except:
#     print("cannot access 2")
start = time.time()

while True:
    try:
        cam.push_network_table(cam.detect())
    except:
        print("cannot access cam id", cam.id)
    
    # try:
    #     cam2.push_network_table(cam2.detect())
    # except:
    #     print("cannot access cam id", cam2.id)
        
    # fps = 1 / (time.time() - start)
    # start = time.time()
    # print("fps:", fps)

cam.cam.release()
cv2.destroyAllWindows()