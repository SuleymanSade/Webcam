import cv2
import os

# Configuration
CHECKERBOARD = (10, 7) # Internal corners
IMAGE_DIR = 'calibration_images'
if not os.path.exists(IMAGE_DIR):
    os.makedirs(IMAGE_DIR)

cap = cv2.VideoCapture(0)
count = 0

print("Press 's' to save a frame, 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Find corners to give visual feedback
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    display_frame = frame.copy()
    if ret_corners:
        cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_corners)
        cv2.putText(display_frame, "Pattern Found! Press 's'", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Calibration Capture', display_frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and ret_corners:
        img_name = os.path.join(IMAGE_DIR, f'calib_{count}.jpg')
        cv2.imwrite(img_name, frame)
        print(f"Saved: {img_name}")
        count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()