# import pandas as pd
import numpy as np
import cv2
from networktables import NetworkTablesInstance
import json
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
        
        

        # look into cv2's camera calibration for better parameters
        # This changes from camera to camera
        # Camera parameters (fx, fy, cx, cy)
        # with open("webcam.json", 'r', encoding="utf-8") as file:
        #     settings = json.load(file)
        #     cam_params = (settings[id]['fx'], settings[id]['fy'], settings[id]['cx'], settings[id]['cy']) # TODO: placeholder values, change these with real calibration
        cam_params = CAM_PARAMETERS
        self.tag_size_meters = 0.1651 #in meters
        self.id = id
        
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11),
            cv2.aruco.DetectorParameters()
        )
        
        self.camera_matrix = np.array([
            [cam_params[0], 0, cam_params[2]],
            [0, cam_params[1], cam_params[3]],
            [0, 0, 1]
        ], dtype=np.float64)
        
        self.dist_coeffs = np.zeros((5, 1))
        
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
                z_list.append(pos[2])
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
        corners, ids, rejected = self.detector.detectMarkers(gray)
                
        # axis:
        # z, positive z is away from camera
        # x, positive x is to the right
        # y, positive y is down
        
        

        rvecs = []
        tvecs = []
        tag_size = self.tag_size_meters

        for c in corners:
            # c shape: (1, 4, 2) → reshape to (4,2)
            c = c.reshape((4,2)).astype(np.float32)

            # object points (tag corners in 3D)
            obj_points = np.array([
                [-tag_size/2,  tag_size/2, 0],
                [ tag_size/2,  tag_size/2, 0],
                [ tag_size/2, -tag_size/2, 0],
                [-tag_size/2, -tag_size/2, 0]
            ], dtype=np.float32)

            # solvePnP
            retval, rvec, tvec = cv2.solvePnP(obj_points, c, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
            rvecs.append(rvec)
            tvecs.append(tvec)
            print(tvec.flatten()[2])

# try:
cam = WebCam("test")
# except:
#     print("cannot access 1")
    
# try:
#     cam2 = WebCam("0")
# except:
#     print("cannot access 2")
start = time.time()

while True:
    # try:
        # cam.push_network_table(cam.detect())
    cam.detect()
    # except:
    #     print("cannot access cam id", cam.id)
    
    # try:
    #     cam2.push_network_table(cam2.detect())
    # except:
    #     print("cannot access cam id", cam2.id)
        
    # fps = 1 / (time.time() - start)
    # start = time.time()
    # print("fps:", fps)

cam.cam.release()
cv2.destroyAllWindows()