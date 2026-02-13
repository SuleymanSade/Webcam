import os

# This directory lists all video devices as symbolic links
v4l_path = "/sys/class/video4linux"

for dev in sorted(os.listdir(v4l_path)):
    # Get the real path of the symbolic link
    real_path = os.path.realpath(os.path.join(v4l_path, dev))
    if "video4linux" in real_path:
        print(f"Device: /dev/{dev}  |  Physical Path: {real_path}")
