# import pandas as pd
import numpy as np
import cv2
from pupil_apriltags import Detector
from networktables import NetworkTablesInstance

# look into cv2's camera calibration for better parameters
# This changes from camera to camera
# Camera parameters (fx, fy, cx, cy)
CAM_PARAMETERS = (600, 600, 320, 240) # TODO: placeholder values, change these with real calibration

class Camera():
    def __init__(self, id: int = 0):
        self.cam = cv2.VideoCapture(1)
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
        self.cam_params = CAM_PARAMETERS # TODO: placeholder values, change these with real calibration
        self.tag_size_meters = 0.1651 #in meters
        self.id = id
        
        #20.7

        # NetworkTables setup
        self.ntinst = NetworkTablesInstance.getDefault()
        self.ntinst.startClient("cam" + str(self.id))
        # self.ntinst.setServerTeam(2635)
        self.ntinst.setServer("localhost")  # For testing purposes
        self.ntinst.startDSClient()
        self.table = self.ntinst.getTable("apriltags" + str(self.id))

    def push_network_table(self, detections):
        
        x_list = []
        y_list = []
        z_list = []
        tag_ids = []
        if detections != np.nan:
            for det in detections:
                tag_id, pos = det
                pos = pos.flatten() # turns to 1D numpy array
                tag_ids.append(tag_id)
                # pos.flatten() has [x, y, z]
                x_list.append(pos[0])
                y_list.append(pos[1])
                z_list.append(pos[2])
            
        self.table.putArray("tag_id", tag_ids)
        self.table.putArray("x", x_list)
        self.table.putArray("y", y_list)
        self.table.putArray("z", z_list)

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
        
        print(type(detections))
        
        # axis:
        # z, positive z is away from camera
        # x, positive x is to the right
        # y, positive y is down

        if detections:
            for det in detections or []:
                print(f"Detected Tag ID: {det.tag_id}")
                print(f"Pose (X,Y,Z): {det.pose_t.flatten()}")
                
            return np.array([(det.tag_id, det.pose_t.flatten()) for det in detections])
        
        return None
        


cam = Camera(1)
while True:
    cam.push_network_table(cam.detect())
cam.cam.release()
cv2.destroyAllWindows()