# import pandas as pd
import numpy as np
import cv2
import json
import os
import time
import robotpy_apriltag as apriltag
import math

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

IS_UNDISTORT = False


def get_camera_index(port_id):
    v4l_dir = "/sys/class/video4linux"

    for dev_node in os.listdir(v4l_dir):
        real_path = os.path.realpath(os.path.join(v4l_dir, dev_node))

        if "video4linux" in real_path:
            # We do divisible by 2, because generally video with odd numbers represent
            # meta data and the even numbers are actually the stream itself
            if port_id in real_path:  # and int(dev_node.replace("video", "")) % 2==0:
                return int(dev_node.replace("video", "")) - 1

    raise KeyError(
        f"cannot find the port_id {port_id}, this might mean your stream values in webcam.json is wrong."
    )


class WebCam:
    def __init__(self, id: str):
        with open(
            # The position of the settings file (webcam.json)
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "webcam.json"),
            "r",
            encoding="utf-8",
        ) as file:
            settings = json.load(file)

        if settings["dev"]["is_device_pi"]:
            # Use this on pi
            self.cam = cv2.VideoCapture(
                "/dev/video" + str(get_camera_index(settings[id]["stream"])),
                cv2.CAP_V4L2,
            )
        else:
            # When using with computer
            self.cam = cv2.VideoCapture(0)

        if not self.cam.isOpened():
            print(f"Camera not found or cannot be opened for id (id).")

        self.detector = apriltag.AprilTagDetector()
        self.detector.addFamily("tag36h11")

        self.frame_count = 0
        self.start_time = 0

        # look into cv2's camera calibration for better parameters
        # This changes from camera to camera
        # Camera parameters (fx, fy, cx, cy)

        self.dist_coeffs = np.array(settings[id]["dist"], dtype=np.float64).reshape(
            (5, 1)
        )
        self.camera_matrix = np.array(settings[id]["cam_matrix"], dtype=np.float64)
        self.tag_size_meters = 0.1651  # in meters
        self.id = id

        h = settings[id]["height"]
        w = settings[id]["width"]

        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, w)

        actual_h = self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_w = self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)

        scale_y = actual_h / h
        scale_x = actual_w / w

        print(f"[{id}] Resolution: Requested {w}x{h}, Got {actual_w}x{actual_h}")
        if scale_x != 1.0 or scale_y != 1.0:
            print(
                f"[{id}] Warning: Resolution mismatch. Scaling calibration by x:{scale_x:.2f}, y:{scale_y:.2f}"
            )

        self.STORE_DATA = settings["storage"]["store_data"]
        if self.STORE_DATA:
            # Write individual JPEG frames instead of a video container.
            # Video containers (.avi/.mp4) require a finalization step that never
            # happens on a hard power cut — individual JPEGs are complete the moment
            # they land on disk, so no frames are lost.
            self.frame_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "rec",
                f"cam_{id}_{int(time.time())}"
            )
            os.makedirs(self.frame_dir, exist_ok=True)
            print(f"[{id}] Recording frames to: {self.frame_dir}")

        self.camera_matrix[0, 0] *= scale_x  # fx
        self.camera_matrix[1, 1] *= scale_y  # fy
        self.camera_matrix[0, 2] *= scale_x  # cx
        self.camera_matrix[1, 2] *= scale_y  # cy

        print(f"Camera {id} requested {w}x{h}, got {actual_w}x{actual_h}")

        if IS_UNDISTORT:
            self.new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
            )
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                self.camera_matrix,
                self.dist_coeffs,
                None,
                self.new_camera_mtx,
                (w, h),
                5,
            )
        else:
            self.new_camera_mtx = self.camera_matrix

        # configures the estimator with the new values from undistortion
        self.pose_est = apriltag.AprilTagPoseEstimator(
            apriltag.AprilTagPoseEstimator.Config(
                self.tag_size_meters,
                self.new_camera_mtx[0, 0],
                self.new_camera_mtx[1, 1],
                self.new_camera_mtx[0, 2],
                self.new_camera_mtx[1, 2],
            )
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
                id, pose = det
                t = pose.translation()

                tag_ids.append(id)
                x_list.append(t.x)
                y_list.append(t.y)
                z_list.append(t.z)
        except:
            pass
            # return

        self.table.putNumberArray("tag_id", tag_ids)
        self.table.putNumberArray("x", x_list)
        self.table.putNumberArray("y", y_list)
        self.table.putNumberArray("z", z_list)

    def release(self):
        self.cam.release()

    def detect(self) -> np.ndarray:
        if not self.start_time:
            self.start_time = time.time()
        ret, frame = self.cam.read()

        if not ret:
            print("Failed to grab frame")
            return np.nan

        # We remap the frame here to remove distortion, this ensures more accurate detections and pos est.
        if IS_UNDISTORT:
            frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)


        # Needs gray for detections
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.STORE_DATA:
            frame_path = os.path.join(self.frame_dir, f"{self.frame_count:06d}.jpg")
            cv2.imwrite(frame_path, gray, [cv2.IMWRITE_JPEG_QUALITY, 80])

        # Detect AprilTags
        detections = self.detector.detect(gray)
        poses = []

        for det in detections:

            pose = self.pose_est.estimate(det)

            poses.append((det.getId(), pose))

            t = pose.translation()
            x, y, z = t.x, t.y, t.z

            # TODO: gives error in pi, don't know why
            # R = pose.rotation().toMatrix()

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
        if cv2.waitKey(1) & 0xFF == ord("q"):
            # This allows you to press 'q' to stop the loop if needed
            return np.nan

        print(f"FPS: {self.frame_count / (time.time() - self.start_time):.2f}")

        return poses

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


cam = WebCam("fl")
cam2 = WebCam("fr")
# cam3 = WebCam("br")
# cam =WebCam("computer")

try:
    while True:
        cam.push_network_table(cam.detect())
        cam2.push_network_table(cam2.detect())
        # cam3.push_network_table(cam3.detect())
except KeyboardInterrupt:
    pass
finally:
    cam.release()
    cam2.release()
    # cam3.release()