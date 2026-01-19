import pandas as pd
import numpy as np
import cv2
from pupil_apriltags import Detector


class Camera():
    def __init__(self):
        self.cam = cv2.VideoCapture(1)
        if(not self.cam.isOpened()):
            print("Camera not found or cannot be opened.")
        
        self.detector = Detector(
            families="tag36h11", # default, FRC uses tag36h11
            nthreads=4,
            # quad_decimate=1.0,
            # quad_sigma=0.0,
            # refine_edges=1,
            # decode_sharpening=0.25,
            # debug=0
        )

        # Camera parameters (fx, fy, cx, cy)
        self.cam_params = (600, 600, 320, 240)
        self.tag_size_meters = 0.1651 #in meters

    def detect(self):
        while True:
            ret, frame = self.cam.read()
            if not ret:
                print("Failed to grab frame")
                return
            # print(frame)
            

            # Needs gray for detections
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            cv2.imshow("Camera Frame", frame)

            # Detect AprilTags
            # detections = self.detector.detect(
            #     gray,
            #     estimate_tag_pose=True,
            #     camera_params=self.cam_params,
            #     tag_size=self.tag_size_meters

            # )

            # for det in detections:
            #     print(f"Detected Tag ID: {det.tag_id}")
            #     print(f"Pose (X,Y,Z): {det.pose_t.flatten()}")
        self.cam.release()
        cv2.destroyAllWindows()
        


cam = Camera()
# while True:
    
cam.detect()
cam.cam.release()
cv2.destroyAllWindows()