# import pandas as pd
import numpy as np
import cv2
import json
import os
import time
import robotpy_apriltag as apriltag

try:
    # works on the computer
    from networktables import NetworkTablesInstance
    using_ntcore = False
except ImportError:
    # works on the pi, as pi uses ntcore instead of pynetworktables
    # pynetworktables is older and not advised to use
    # but for unkown reasons, ntcore doesn't work on the computer, so we have to use pynetworktables there
    import ntcore
    using_ntcore = True
    

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
            
        self.detector = apriltag.AprilTagDetector()
        self.detector.addFamily("tag36h11")
        
        
        self.frame_count = 0
        self.start_time = time.time()

        # look into cv2's camera calibration for better parameters
        # This changes from camera to camera
        # Camera parameters (fx, fy, cx, cy)
        with open("webcam.json", 'r', encoding="utf-8") as file:
            settings = json.load(file)
        
        cam_params = (settings[id]['fx'], settings[id]['fy'], settings[id]['cx'], settings[id]['cy']) # TODO: placeholder values, change these with real calibration
        self.dist_coeffs = np.array(settings[id]['dist'], dtype=np.float64).reshape((5, 1))
        self.camera_matrix = np.array(settings[id]["cam_matrix"], dtype=np.float64)
        self.tag_size_meters = 0.1651 #in meters
        self.id = id
        
        self.pose_est = apriltag.AprilTagPoseEstimator(
            apriltag.AprilTagPoseEstimator.Config(
                tagSize=self.tag_size_meters,
                fx=cam_params[0],
                fy=cam_params[1],
                cx=cam_params[2],
                cy=cam_params[3],
            )
        )

        h = settings[id]['height']
        w = settings[id]['width']

        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, settings[id]['width'])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, settings[id]['height'])

        self.new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, self.new_camera_mtx, (w, h), 5
        )
        
        # NetworkTables setup
        if using_ntcore:
            self.ntinst = ntcore.NetworkTableInstance.getDefault()
            self.ntinst.setServerTeam(2635)
            # self.ntinst.setServer("localhost")  # For testing purposes
            self.ntinst.startClient4("cam" + str(self.id))
        else:
            self.ntinst = NetworkTablesInstance.getDefault()
            self.ntinst.setServerTeam(2635)
            # self.ntinst.setServer("localhost")  # For testing purposes
            self.ntinst.startClient("cam" + str(self.id))
        
        
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
        # height, width = frame.shape[:2] # .shape returns (rows, cols, channels)
        # print(f"Frame dimensions: {width}x{height}")
        
        # We remap the frame here to remove distortion, this ensures moer accurate detections and pos est.
        frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        # Needs gray for detections
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags
        detections = self.detector.detect(gray)
        
        for det in detections:
            pose = self.pose_est.estimate(det)
            
            t = pose.translation()
            x, y, z = t.x, t.y, t.z
            
            R = pose.rotation().toMatrix()
            
            print("x: ", x, "y:", y, "z:", z)
            
            # for i in range(4):
            #     p1_raw = det.getCorner(i)
            #     p2_raw = det.getCorner((i + 1) % 4)

            #     # Convert to integer tuples for OpenCV drawing
            #     p1 = (int(p1_raw.x), int(p1_raw.y))
            #     p2 = (int(p2_raw.x), int(p2_raw.y))

            #     cv2.line(frame, p1, p2, (0, 255, 0), 2)
                
        self.frame_count += 1

        
        # cv2.imshow("Camera Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # This allows you to press 'q' to stop the loop if needed
            return np.nan
        
        print(f"FPS: {self.frame_count / (time.time() - self.start_time):.2f}")
                
        # axis:
        # z, positive z is away from camera
        # x, positive x is to the right
        # y, positive y is down

        # if detections:
        #     for det in detections or []:
        #         print(f"Detected Tag ID: {det.tag_id}")
        #         print(f"Pose (X,Y,Z): {det.pose_t.flatten() / 3 * 2}")
                
        #     return [(det.tag_id, det.pose_t.flatten()) for det in detections]
        
        # return np.nan

# try:
cam = WebCam("comp")
# except:
#     print("cannot access 1")
    
# try:
#     cam2 = WebCam("0")
# except:
#     print("cannot access 2")
start = time.time()

while True:
    # try:
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