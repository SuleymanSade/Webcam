import sys
import numpy as np
import cv2

WIDTH = 640
HEIGHT = 480

BYTES_PER_FRAME = WIDTH * HEIGHT * 3  # bgr24

def get_frame():
    raw = sys.stdin.buffer.read(BYTES_PER_FRAME)
    if len(raw) < BYTES_PER_FRAME:
        return None

    frame = np.frombuffer(raw, dtype=np.uint8)
    frame = frame.reshape((HEIGHT, WIDTH, 3))
    return frame

while True:
    frame = get_frame()
    if frame is None:
        continue

    # Now you have a real cv2 frame
    cv2.imshow("ffmpeg frame", frame)
    if cv2.waitKey(1) == 27:
        break