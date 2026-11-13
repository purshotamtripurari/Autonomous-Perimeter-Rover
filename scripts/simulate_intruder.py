#!/usr/bin/env python3
import os
import sys
import time

venv_path = os.path.expanduser("~/autonomous-perimeter-rover/.venv/lib/python3.12/site-packages")
if os.path.exists(venv_path) and venv_path not in sys.path:
    sys.path.insert(0, venv_path)

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image


def main():
    mode = "breach"
    duration = 5

    # Parse arguments: e.g. "python3 simulate_intruder.py [safe|breach] [seconds]"
    args = sys.argv[1:]
    if len(args) > 0:
        if args[0] in ("safe", "outside"):
            mode = "safe"
            if len(args) > 1:
                try:
                    duration = int(args[1])
                except ValueError:
                    pass
        elif args[0] in ("breach", "inside"):
            mode = "breach"
            if len(args) > 1:
                try:
                    duration = int(args[1])
                except ValueError:
                    pass
        else:
            try:
                duration = int(args[0])
            except ValueError:
                pass

    print(f"[*] Initializing Intruder Simulator in '{mode.upper()}' mode for {duration} seconds...")
    rclpy.init()
    node = rclpy.create_node("intruder_simulator")
    pub = node.create_publisher(Image, "/camera/image_raw", 10)

    image_path = os.path.expanduser("~/autonomous-perimeter-rover/bus.jpg")
    img = cv2.imread(image_path)
    if img is None:
        print(f"[!] Error: Could not load image at {image_path}")
        return

    # Resize to standard rover camera resolution (320x240)
    img_resized = cv2.resize(img, (320, 240))

    if mode == "safe":
        # Shift the person image to the upper-left (outside the ground geofence polygon)
        # Create a background canvas and place the person in the upper authorized zone
        canvas = np.zeros((240, 320, 3), dtype=np.uint8)
        person_crop = cv2.resize(img[380:920, 40:350], (70, 70))
        # Place in upper sky/background region (y: 10..80, outside the ground geofence)
        canvas[10:80, 20:90] = person_crop
        broadcast_img = canvas
        print("[*] Simulating person OUTSIDE the restricted yard geofence (Authorized Area)...")
        print("[*] Expected behavior: Yellow tag, NO ALARM fired, rover keeps patrolling!")
    else:
        broadcast_img = img_resized
        print("[*] Simulating person INSIDE the restricted yard geofence (Breach Area)...")
        print("[*] Expected behavior: Red alert, /perimeter/alert=TRUE, rover auto-brakes!")

    msg = Image()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "camera_depth_frame"
    msg.height, msg.width = broadcast_img.shape[:2]
    msg.encoding = "bgr8"
    msg.step = broadcast_img.shape[1] * 3
    msg.data = broadcast_img.tobytes()

    frames = int(duration * 5)
    for _ in range(frames):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.2)

    print(f"[*] Simulation complete. View cleared.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
